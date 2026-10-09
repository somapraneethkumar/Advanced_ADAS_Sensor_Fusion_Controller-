/*
 * imu_node.c  -  QNX-eHACK 2026: MPU6050 IMU process of the ADAS fusion system
 *
 * Same acquisition as imu_mpu6050.c, plus: every 20 ms batch is sent to fusion_server as ONE
 * IOV message (MsgSendv: part 0 = adas_hdr_t, part 1 = the sample array, no repacking into a
 * single buffer). Health changes are sent as pulses (MsgSendPulse). If fusion_server is not
 * running the node keeps working and reconnects with name_open() once per second.
 *
 * What makes this different from the usual "read 14 registers in a loop" approach:
 *
 *  1. HARDWARE-FIFO BATCH ACQUISITION
 *     The MPU6050 samples itself at exactly 200 Hz (its own clock) and stores every
 *     sample in its 1 KB FIFO. A QNX timer -> pulse wakes our real-time thread every
 *     20 ms and it drains the FIFO in one burst. No sample is ever missed, even if the
 *     reader is late, and the sampling instant does not depend on Linux/QNX scheduling.
 *
 *  2. RECONSTRUCTED PER-SAMPLE TIMESTAMPS + SENSOR CLOCK-DRIFT ESTIMATION
 *     Each sample in a batch gets its own timestamp, rebuilt from the FIFO position and
 *     the *measured* sensor sample period. The MPU6050's internal oscillator is never
 *     exactly 200 Hz (typically off by ~1%); we measure its real rate against the QNX
 *     monotonic clock and correct for it. That is exactly what the fusion stage needs for
 *     timestamp alignment.
 *
 *  3. SENSOR HEALTH MONITOR (feeds the "sensor degraded" path of the fusion design)
 *     WHO_AM_I check, I2C error counting with automatic re-init, FIFO-overflow
 *     detection (data loss is detected, never silent), frozen-sensor detection,
 *     range-saturation detection and gravity plausibility check  ->  OK / DEGRADED / FAULT.
 *
 *  4. STARTUP SELF-CALIBRATION
 *     Gyro bias and gravity scale are measured while the board is still; calibration is
 *     rejected and retried if the board is moving.
 *
 *  5. ORIENTATION
 *     Complementary filter (roll / pitch) using the reconstructed, true per-sample dt.
 *
 * Wiring (Raspberry Pi 4 header):
 *     MPU6050 VCC -> pin 1  (3.3 V)        MPU6050 GND -> pin 6 (GND)
 *     MPU6050 SDA -> pin 3  (GPIO2/SDA1)   MPU6050 SCL -> pin 5 (GPIO3/SCL1)
 *     MPU6050 AD0 -> GND (address 0x68) or 3.3 V (address 0x69)
 *
 * Build: Momentics QNX project, aarch64le, link with -lm.
 * Usage: imu_mpu6050 [-d /dev/i2c1] [-a 0x68] [-t seconds] [-o imu_log.csv] [-c]
 *   -d  I2C device (default /dev/i2c1)
 *   -a  I2C address (default 0x68)
 *   -t  run time in seconds (0 = until Ctrl-C)
 *   -o  CSV log with every sample (timestamp + values), for plots
 *   -c  skip startup calibration
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#include <sys/dispatch.h>
#include <sys/uio.h>
#include "adas_msg.h"

/* ------------------------------------------------------------ MPU6050 */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_FIFO_EN         0x23
#define REG_INT_ENABLE      0x38
#define REG_INT_STATUS      0x3A
#define REG_USER_CTRL       0x6A
#define REG_PWR_MGMT_1      0x6B
#define REG_FIFO_COUNTH     0x72
#define REG_FIFO_R_W        0x74
#define REG_WHO_AM_I        0x75

#define FIFO_EN_ACC_TEMP_GYRO 0xF8  /* TEMP | XG | YG | ZG | ACCEL          */
#define FRAME_BYTES         14      /* ax ay az temp gx gy gz, big-endian    */
#define FIFO_SIZE           1024
#define INT_FIFO_OFLOW      0x10

#define SAMPLE_HZ           200     /* sensor's own sample rate              */
#define ACC_LSB_PER_G       8192.0  /* +-4 g                                 */
#define GYRO_LSB_PER_DPS    65.5    /* +-500 deg/s                           */
#define GRAVITY             9.80665

/* ------------------------------------------------------------ timing */
#define NS_PER_SEC          1000000000LL
#define NS_PER_MS           1000000LL
#define READ_PERIOD_NS      (20 * NS_PER_MS)   /* drain FIFO at 50 Hz */
#define PRIO_IMU            28
#define PULSE_CODE_TICK     (_PULSE_CODE_MINAVAIL + 1)

#define MAX_BATCH           64      /* frames handled per wake-up            */
#define I2C_CHUNK           (FRAME_BYTES * 16)
#define FAULT_AFTER_ERRORS  5
#define FROZEN_AFTER        50      /* identical raw frames -> frozen sensor */

/* ------------------------------------------------------------ types */
typedef struct {
    int64_t t_ns;       /* reconstructed sampling instant (CLOCK_MONOTONIC) */
    double  a[3];       /* m/s^2, calibrated  */
    double  g[3];       /* deg/s, bias-removed */
    double  temp_c;
    int16_t raw[7];
} imu_sample_t;

typedef enum { H_OK, H_DEGRADED, H_FAULT } health_t;
static const char *HEALTH_NAME[] = { "OK", "DEGRADED", "FAULT" };

typedef struct {
    bool    init;
    int64_t t_last;     /* timestamp of newest sample so far */
    double  ts_ns;      /* estimated true sample period      */
    int64_t anchor_t;
    uint64_t anchor_n, total;
} tstamp_t;

typedef struct {
    imu_sample_t last;
    double   roll, pitch, a_norm_g;
    double   rate_hz, read_us, worst_read_us;
    int      batch;
    uint64_t samples, overflows, i2c_errors, reinits, saturations;
    bool     frozen;
    health_t health;
} imu_status_t;

/* ------------------------------------------------------------ globals */
static volatile sig_atomic_t g_run = 1;
static int      g_fd = -1;
static uint8_t  g_addr = 0x68;
static double   g_gyro_bias[3];
static double   g_acc_scale = 1.0;

static imu_status_t    g_st;
static pthread_mutex_t g_st_lock = PTHREAD_MUTEX_INITIALIZER;

/* link to fusion_server */
static int      g_coid = -1;
static uint32_t g_seq;
static int64_t  g_next_connect;
static int      g_sent_health = -1;
static uint64_t g_msgs_sent, g_send_fail;

/* CSV ring (RT thread -> main thread) */
#define LOG_N 4096
static imu_sample_t    g_log[LOG_N];
static double          g_log_rp[LOG_N][2];
static uint64_t        g_log_w, g_log_r, g_log_lost;
static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;

/* ============================================================ utils */
static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void ns_to_ts(int64_t ns, struct timespec *ts)
{
    ts->tv_sec = ns / NS_PER_SEC;
    ts->tv_nsec = ns % NS_PER_SEC;
}

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * NS_PER_MS };
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR)
        ;
}

static int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

/* ============================================================ QNX I2C */
/* QNX talks to I2C with devctl() only: DCMD_I2C_SEND / DCMD_I2C_SENDRECV */
static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    struct {
        i2c_send_t hdr;
        uint8_t    buf[2];
    } msg;
    memset(&msg, 0, sizeof msg);
    msg.hdr.slave.addr = g_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.len        = 2;
    msg.hdr.stop       = 1;
    msg.buf[0] = reg;
    msg.buf[1] = val;
    return devctl(g_fd, DCMD_I2C_SEND, &msg, sizeof msg, NULL);   /* EOK or errno */
}

static int i2c_read_regs(uint8_t reg, uint8_t *out, int len)
{
    struct {
        i2c_sendrecv_t hdr;
        uint8_t        buf[I2C_CHUNK];
    } msg;
    if (len <= 0 || len > I2C_CHUNK) return EINVAL;
    memset(&msg.hdr, 0, sizeof msg.hdr);
    msg.hdr.slave.addr = g_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len   = 1;
    msg.hdr.recv_len   = (uint32_t)len;
    msg.hdr.stop       = 1;
    msg.buf[0] = reg;
    int rc = devctl(g_fd, DCMD_I2C_SENDRECV, &msg, sizeof msg.hdr + (size_t)len, NULL);
    if (rc == EOK) memcpy(out, msg.buf, (size_t)len);
    return rc;
}

/* ============================================================ MPU6050 setup */
static int mpu_fifo_reset(void)
{
    int rc = 0;
    rc |= i2c_write_reg(REG_FIFO_EN, 0x00);
    rc |= i2c_write_reg(REG_USER_CTRL, 0x04);           /* FIFO_RESET */
    sleep_ms(1);
    rc |= i2c_write_reg(REG_USER_CTRL, 0x40);           /* FIFO_EN    */
    rc |= i2c_write_reg(REG_FIFO_EN, FIFO_EN_ACC_TEMP_GYRO);
    return rc;
}

static int mpu_init(bool verbose)
{
    uint8_t who = 0;
    int rc = i2c_read_regs(REG_WHO_AM_I, &who, 1);
    if (rc != EOK) {
        if (verbose) fprintf(stderr, "I2C read of WHO_AM_I failed: %s\n", strerror(rc));
        return -1;
    }
    if (verbose) {
        if (who == 0x68) printf("MPU6050 found at 0x%02X (WHO_AM_I=0x%02X)\n", g_addr, who);
        else printf("warning: WHO_AM_I=0x%02X (expected 0x68) - clone or MPU6500/9250? continuing\n", who);
    }

    rc  = i2c_write_reg(REG_PWR_MGMT_1, 0x80);          /* device reset            */
    sleep_ms(100);
    rc |= i2c_write_reg(REG_PWR_MGMT_1, 0x01);          /* wake, PLL on X gyro     */
    sleep_ms(10);
    rc |= i2c_write_reg(REG_CONFIG, 0x03);              /* DLPF 44 Hz, 1 kHz base  */
    rc |= i2c_write_reg(REG_SMPLRT_DIV, (uint8_t)(1000 / SAMPLE_HZ - 1));
    rc |= i2c_write_reg(REG_GYRO_CONFIG, 0x08);         /* +-500 deg/s             */
    rc |= i2c_write_reg(REG_ACCEL_CONFIG, 0x08);        /* +-4 g                   */
    rc |= i2c_write_reg(REG_INT_ENABLE, INT_FIFO_OFLOW);
    rc |= mpu_fifo_reset();
    if (rc != EOK) {
        if (verbose) fprintf(stderr, "MPU6050 configuration failed\n");
        return -1;
    }
    return 0;
}

/*
 * Drain the FIFO. Returns number of frames (>=0), -1 on I2C error.
 * *overflow is set if the sensor reported FIFO overflow (data was lost).
 */
static int mpu_read_fifo(imu_sample_t *s, int max, bool *overflow)
{
    uint8_t st, cnt[2];
    *overflow = false;

    if (i2c_read_regs(REG_INT_STATUS, &st, 1) != EOK) return -1;
    if (st & INT_FIFO_OFLOW) {
        *overflow = true;
        return mpu_fifo_reset() == EOK ? 0 : -1;
    }
    if (i2c_read_regs(REG_FIFO_COUNTH, cnt, 2) != EOK) return -1;
    int count = (cnt[0] << 8) | cnt[1];
    if (count > FIFO_SIZE) {                            /* corrupted count */
        *overflow = true;
        return mpu_fifo_reset() == EOK ? 0 : -1;
    }

    int n = count / FRAME_BYTES;
    if (n > max) n = max;

    uint8_t buf[MAX_BATCH * FRAME_BYTES];
    int total = n * FRAME_BYTES, off = 0;
    while (off < total) {
        int len = total - off > I2C_CHUNK ? I2C_CHUNK : total - off;
        if (i2c_read_regs(REG_FIFO_R_W, buf + off, len) != EOK) return -1;
        off += len;
    }

    for (int i = 0; i < n; i++) {
        const uint8_t *p = buf + i * FRAME_BYTES;
        for (int k = 0; k < 7; k++) s[i].raw[k] = be16(p + 2 * k);
        for (int k = 0; k < 3; k++) {
            s[i].a[k] = s[i].raw[k] / ACC_LSB_PER_G * GRAVITY * g_acc_scale;
            s[i].g[k] = s[i].raw[4 + k] / GYRO_LSB_PER_DPS - g_gyro_bias[k];
        }
        s[i].temp_c = s[i].raw[3] / 340.0 + 36.53;
    }
    return n;
}

/* ============================================================ timestamps */
static void ts_reset(tstamp_t *t)
{
    t->init = false;
    if (t->ts_ns <= 0) t->ts_ns = (double)NS_PER_SEC / SAMPLE_HZ;
}

/*
 * Give every sample of a batch its own timestamp.
 * Model: samples are spaced by the sensor's true period Ts; the newest sample was taken
 * on average Ts/2 before we read the FIFO. A small correction keeps the timeline locked
 * to the QNX clock; the long-term sample count gives the real sensor rate (clock drift).
 */
static void ts_assign(tstamp_t *t, int64_t t_read, imu_sample_t *s, int n)
{
    if (n <= 0) return;
    double nominal = (double)NS_PER_SEC / SAMPLE_HZ;
    double Ts = t->ts_ns;
    double meas_newest = (double)t_read - Ts / 2.0;

    if (!t->init) {
        t->t_last   = (int64_t)(meas_newest - n * Ts);
        t->anchor_t = t_read;
        t->anchor_n = t->total + (uint64_t)n;
        t->init     = true;
    }

    double pred   = (double)t->t_last + n * Ts;
    double newest = pred + 0.05 * (meas_newest - pred);
    if (newest <= (double)t->t_last) newest = pred;     /* never go backwards */
    double step = (newest - (double)t->t_last) / n;
    for (int i = 0; i < n; i++)
        s[i].t_ns = t->t_last + (int64_t)((i + 1) * step);
    t->t_last = (int64_t)newest;
    t->total += (uint64_t)n;

    int64_t span = t_read - t->anchor_t;
    uint64_t cnt = t->total - t->anchor_n;
    if (span > 2 * NS_PER_SEC && cnt > 0) {
        double est = (double)span / (double)cnt;
        if (est > 0.9 * nominal && est < 1.1 * nominal) t->ts_ns = est;
    }
}

/* ============================================================ calibration */
static bool calibrate(double seconds)
{
    imu_sample_t s[MAX_BATCH];
    for (int attempt = 1; attempt <= 3 && g_run; attempt++) {
        printf("calibrating (attempt %d) - keep the sensor still for %.0f s...\n", attempt, seconds);
        fflush(stdout);
        double sg[3] = { 0 }, sg2[3] = { 0 }, sa[3] = { 0 };
        long n = 0;
        int64_t end = now_ns() + (int64_t)(seconds * NS_PER_SEC);
        bool ovf;
        mpu_fifo_reset();
        while (now_ns() < end && g_run) {
            sleep_ms(20);
            int k = mpu_read_fifo(s, MAX_BATCH, &ovf);
            if (k < 0) { fprintf(stderr, "I2C error during calibration\n"); return false; }
            for (int i = 0; i < k; i++, n++)
                for (int a = 0; a < 3; a++) {
                    double g = s[i].raw[4 + a] / GYRO_LSB_PER_DPS;
                    sg[a] += g; sg2[a] += g * g;
                    sa[a] += s[i].raw[a] / ACC_LSB_PER_G;
                }
        }
        if (n < 50) { fprintf(stderr, "too few samples (%ld)\n", n); continue; }

        double worst_std = 0, anorm = 0;
        for (int a = 0; a < 3; a++) {
            double m = sg[a] / n, var = sg2[a] / n - m * m;
            double sd = sqrt(var > 0 ? var : 0);
            if (sd > worst_std) worst_std = sd;
            g_gyro_bias[a] = m;
            anorm += (sa[a] / n) * (sa[a] / n);
        }
        anorm = sqrt(anorm);

        if (worst_std > 1.0) {
            printf("  board was moving (gyro noise %.2f deg/s) - retrying\n", worst_std);
            memset(g_gyro_bias, 0, sizeof g_gyro_bias);
            continue;
        }
        if (anorm < 0.8 || anorm > 1.2) {
            printf("  gravity reads %.2f g - suspicious, not scaling accelerometer\n", anorm);
            g_acc_scale = 1.0;
        } else {
            g_acc_scale = 1.0 / anorm;
        }
        printf("  %ld samples | gyro bias [%.2f %.2f %.2f] deg/s | gravity %.3f g | noise %.3f deg/s\n",
               n, g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2], anorm, worst_std);
        return true;
    }
    return false;
}

/* ============================================================ fusion link (IPC) */
static int to_sh(health_t h) { return h == H_OK ? SH_OK : h == H_DEGRADED ? SH_DEGRADED : SH_FAULT; }

static void fusion_link(int64_t t)
{
    if (g_coid != -1 || t < g_next_connect) return;
    g_coid = name_open(ADAS_SERVER_NAME, 0);
    g_next_connect = t + NS_PER_SEC;
    if (g_coid != -1) { g_sent_health = -1; }
}

static void fusion_drop(void)
{
    if (g_coid != -1) name_close(g_coid);
    g_coid = -1;
    g_send_fail++;
}

/* health change -> pulse (asynchronous, never blocks) */
static void fusion_health(health_t h)
{
    int sh = to_sh(h);
    if (g_coid == -1 || sh == g_sent_health) return;
    if (MsgSendPulse(g_coid, -1, PC_SENSOR_HEALTH, (SENS_IMU << 8) | sh) == -1) fusion_drop();
    else g_sent_health = sh;
}

/* one batch -> one IOV message: [adas_hdr_t][imu_sample_msg_t x n] */
static void fusion_send(const imu_sample_t *s, int n, health_t h, double rate, double temp,
                        double roll, double pitch)
{
    if (g_coid == -1 || n <= 0) return;
    static imu_sample_msg_t out[MAX_BATCH];
    for (int i = 0; i < n; i++) {
        out[i].t_ns = s[i].t_ns;
        for (int k = 0; k < 3; k++) { out[i].a[k] = (float)s[i].a[k]; out[i].g[k] = (float)s[i].g[k]; }
        out[i].roll = (float)roll; out[i].pitch = (float)pitch;   /* filter state after batch */
    }
    adas_hdr_t hdr;
    memset(&hdr, 0, sizeof hdr);
    hdr.type = MSG_SENSOR_DATA;
    hdr.sensor = SENS_IMU;
    hdr.count = (uint16_t)n;
    hdr.health = (uint16_t)to_sh(h);
    hdr.seq = g_seq++;
    hdr.sample_size = sizeof(imu_sample_msg_t);
    hdr.rate_hz = (float)rate;
    hdr.extra = (float)temp;
    iov_t iov[2];
    SETIOV(&iov[0], &hdr, sizeof hdr);
    SETIOV(&iov[1], out, (size_t)n * sizeof out[0]);
    hdr.t_send_ns = now_ns();
    if (MsgSendv(g_coid, iov, 2, NULL, 0) == -1) fusion_drop();
    else g_msgs_sent++;
}

/* ============================================================ acquisition */
static void log_push(const imu_sample_t *s, double roll, double pitch)
{
    pthread_mutex_lock(&g_log_lock);
    if (g_log_w - g_log_r >= LOG_N) g_log_lost++;
    else {
        g_log[g_log_w % LOG_N] = *s;
        g_log_rp[g_log_w % LOG_N][0] = roll;
        g_log_rp[g_log_w % LOG_N][1] = pitch;
        g_log_w++;
    }
    pthread_mutex_unlock(&g_log_lock);
}

static void *imu_thread(void *arg)
{
    (void)arg;
    /* QNX periodic timer -> pulse -> channel */
    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (chid == -1 || coid == -1) { perror("channel"); g_run = 0; return NULL; }
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, PRIO_IMU, PULSE_CODE_TICK, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("timer_create"); g_run = 0; return NULL; }
    struct itimerspec its;
    ns_to_ts(now_ns() + READ_PERIOD_NS, &its.it_value);
    ns_to_ts(READ_PERIOD_NS, &its.it_interval);
    timer_settime(tid, TIMER_ABSTIME, &its, NULL);

    tstamp_t ts = { 0 };
    ts_reset(&ts);
    imu_sample_t s[MAX_BATCH];
    int consec_err = 0, same_count = 0, ticks_since_fault = 0;
    int16_t prev_raw[7] = { 0 };
    double roll = 0, pitch = 0;
    bool att_init = false;
    int64_t t_prev_sample = 0;
    imu_status_t st = { 0 };
    st.health = H_OK;
    mpu_fifo_reset();

    while (g_run) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (p.code != PULSE_CODE_TICK) continue;

        /* FAULT: try to recover about once per second */
        if (st.health == H_FAULT) {
            if (++ticks_since_fault < 50) goto publish;
            ticks_since_fault = 0;
            if (mpu_init(false) == 0) {
                st.reinits++;
                consec_err = 0;
                ts_reset(&ts);
                st.health = H_OK;
            }
            goto publish;
        }

        int64_t t_read = now_ns();
        bool ovf;
        int n = mpu_read_fifo(s, MAX_BATCH, &ovf);
        int64_t t_done = now_ns();

        if (n < 0) {
            st.i2c_errors++;
            if (++consec_err >= FAULT_AFTER_ERRORS) st.health = H_FAULT;
            goto publish;
        }
        consec_err = 0;
        if (ovf) { st.overflows++; ts_reset(&ts); att_init = false; }

        ts_assign(&ts, t_read, s, n);
        st.read_us = (double)(t_done - t_read) / 1e3;
        if (st.read_us > st.worst_read_us) st.worst_read_us = st.read_us;
        st.batch = n;

        for (int i = 0; i < n; i++) {
            imu_sample_t *x = &s[i];

            /* health checks */
            if (memcmp(x->raw, prev_raw, sizeof prev_raw) == 0) same_count++;
            else same_count = 0;
            memcpy(prev_raw, x->raw, sizeof prev_raw);
            st.frozen = same_count >= FROZEN_AFTER;
            for (int k = 0; k < 7; k++)
                if (k != 3 && (x->raw[k] == 32767 || x->raw[k] == -32768)) { st.saturations++; break; }

            /* complementary filter with the true per-sample dt */
            double ax = x->a[0], ay = x->a[1], az = x->a[2];
            double r_acc = atan2(ay, az) * 180.0 / M_PI;
            double p_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
            if (!att_init) { roll = r_acc; pitch = p_acc; att_init = true; }
            else {
                double dt = (double)(x->t_ns - t_prev_sample) / 1e9;
                if (dt <= 0 || dt > 0.1) dt = 1.0 / SAMPLE_HZ;
                roll  = 0.98 * (roll  + x->g[0] * dt) + 0.02 * r_acc;
                pitch = 0.98 * (pitch + x->g[1] * dt) + 0.02 * p_acc;
            }
            t_prev_sample = x->t_ns;
            st.a_norm_g = sqrt(ax * ax + ay * ay + az * az) / GRAVITY;
            log_push(x, roll, pitch);
        }
        if (n > 0) st.last = s[n - 1];
        st.samples += (uint64_t)n;
        st.roll = roll;
        st.pitch = pitch;
        st.rate_hz = (double)NS_PER_SEC / ts.ts_ns;

        if (st.frozen || st.a_norm_g < 0.3 || st.a_norm_g > 3.5) st.health = H_DEGRADED;
        else st.health = H_OK;

        /* ---- send this batch to fusion_server (IOV message) */
        fusion_link(t_done);
        fusion_health(st.health);
        fusion_send(s, n, st.health, st.rate_hz, n > 0 ? s[n - 1].temp_c : 0, roll, pitch);

publish:
        fusion_link(now_ns());
        fusion_health(st.health);                    /* FAULT / recovery reach fusion as pulses */
        pthread_mutex_lock(&g_st_lock);
        g_st = st;
        pthread_mutex_unlock(&g_st_lock);
    }
    timer_delete(tid);
    ConnectDetach(coid);
    ChannelDestroy(chid);
    return NULL;
}

/* ============================================================ main */
static void on_signal(int sig) { (void)sig; g_run = 0; }

static void log_drain(FILE *f)
{
    for (;;) {
        imu_sample_t s;
        double rp[2];
        pthread_mutex_lock(&g_log_lock);
        bool have = g_log_r < g_log_w;
        if (have) {
            s = g_log[g_log_r % LOG_N];
            rp[0] = g_log_rp[g_log_r % LOG_N][0];
            rp[1] = g_log_rp[g_log_r % LOG_N][1];
            g_log_r++;
        }
        pthread_mutex_unlock(&g_log_lock);
        if (!have) break;
        if (f)
            fprintf(f, "%lld,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                    (long long)s.t_ns, s.a[0], s.a[1], s.a[2], s.g[0], s.g[1], s.g[2],
                    s.temp_c, rp[0], rp[1]);
    }
    if (f) fflush(f);
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/i2c1", *log_path = NULL;
    int run_s = 0, opt;
    bool do_cal = true;

    while ((opt = getopt(argc, argv, "d:a:t:o:c")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'a': g_addr = (uint8_t)strtol(optarg, NULL, 0); break;
        case 't': run_s = atoi(optarg); break;
        case 'o': log_path = optarg; break;
        case 'c': do_cal = false; break;
        default:
            fprintf(stderr, "usage: %s [-d /dev/i2c1] [-a 0x68] [-t sec] [-o log.csv] [-c]\n", argv[0]);
            return 1;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    g_fd = open(dev, O_RDWR);
    if (g_fd == -1) {
        fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
        fprintf(stderr, "check: ls /dev/i2c*   (driver i2c-bcm2711 must be running; root may be needed)\n");
        return 1;
    }
    uint32_t speed = 400000;                            /* fast mode if the driver allows */
    if (devctl(g_fd, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof speed, NULL) != EOK)
        printf("note: could not set 400 kHz, using driver default bus speed\n");

    if (mpu_init(true) != 0) {
        fprintf(stderr, "MPU6050 not responding - check wiring (SDA pin 3, SCL pin 5, 3.3V, GND) "
                        "and address (-a 0x69 if AD0 is high)\n");
        return 1;
    }
    if (do_cal && !calibrate(2.0)) {
        fprintf(stderr, "calibration failed - continuing without gyro bias correction\n");
        memset(g_gyro_bias, 0, sizeof g_gyro_bias);
    }

    FILE *logf = NULL;
    if (log_path) {
        logf = fopen(log_path, "w");
        if (!logf) perror("log");
        else fprintf(logf, "t_ns,ax,ay,az,gx,gy,gz,temp_c,roll_deg,pitch_deg\n");
    }

    pthread_t th;
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&a, SCHED_FIFO);
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = PRIO_IMU;
    pthread_attr_setschedparam(&a, &sp);
    if (pthread_create(&th, &a, imu_thread, NULL) != 0) {
        fprintf(stderr, "warning: could not use priority %d, using default\n", PRIO_IMU);
        pthread_create(&th, NULL, imu_thread, NULL);
    }
    pthread_setname_np(th, "imu_acq");
    printf("fusion link: name_open(\"%s\"), retried every second until fusion_server runs\n", ADAS_SERVER_NAME);
    pthread_setname_np(pthread_self(), "imu_display");

    printf("\nreading MPU6050 FIFO @ %d Hz sensor rate, drained every %lld ms (Ctrl-C to stop)\n\n",
           SAMPLE_HZ, READ_PERIOD_NS / NS_PER_MS);

    int64_t t_end = run_s > 0 ? now_ns() + (int64_t)run_s * NS_PER_SEC : 0;
    while (g_run) {
        sleep_ms(200);
        log_drain(logf);
        imu_status_t s;
        pthread_mutex_lock(&g_st_lock);
        s = g_st;
        pthread_mutex_unlock(&g_st_lock);

        printf("IMU %-8s | a[m/s2] %+6.2f %+6.2f %+6.2f |a|=%.2fg | g[dps] %+7.2f %+7.2f %+7.2f | "
               "roll %+6.1f pitch %+6.1f | %4.1fC | rate %6.2fHz batch %2d read %4.0fus | "
               "ovf %llu err %llu%s | fusion %s msgs %llu\n",
               HEALTH_NAME[s.health],
               s.last.a[0], s.last.a[1], s.last.a[2], s.a_norm_g,
               s.last.g[0], s.last.g[1], s.last.g[2],
               s.roll, s.pitch, s.last.temp_c,
               s.rate_hz, s.batch, s.read_us,
               (unsigned long long)s.overflows, (unsigned long long)s.i2c_errors,
               s.frozen ? " FROZEN" : "", g_coid != -1 ? "linked" : "not running",
               (unsigned long long)g_msgs_sent);
        fflush(stdout);
        if (t_end && now_ns() >= t_end) g_run = 0;
    }

    pthread_join(th, NULL);
    log_drain(logf);
    if (logf) fclose(logf);
    close(g_fd);
    if (g_coid != -1) name_close(g_coid);

    imu_status_t s = g_st;
    printf("\n===== IMU summary =====\n");
    printf("samples            : %llu\n", (unsigned long long)s.samples);
    printf("measured sensor rate: %.3f Hz (nominal %d Hz, drift %+.2f%%)\n",
           s.rate_hz, SAMPLE_HZ, (s.rate_hz / SAMPLE_HZ - 1.0) * 100.0);
    printf("worst FIFO read    : %.0f us\n", s.worst_read_us);
    printf("FIFO overflows     : %llu   I2C errors: %llu   re-inits: %llu   saturations: %llu\n",
           (unsigned long long)s.overflows, (unsigned long long)s.i2c_errors,
           (unsigned long long)s.reinits, (unsigned long long)s.saturations);
    if (g_log_lost) printf("log samples lost   : %llu\n", (unsigned long long)g_log_lost);
    return 0;
}
