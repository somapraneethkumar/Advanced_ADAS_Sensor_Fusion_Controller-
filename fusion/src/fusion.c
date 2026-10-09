/*
 * fusion_server.c  -  QNX-eHACK 2026: ADAS sensor-fusion server process
 *
 * QNX mechanisms used here:
 *   - name_attach("adas_fusion")         : sensors and dashboard find this server by name
 *   - MsgReceivev() with an IOV          : header lands in one buffer, the sample array in
 *                                          another (scatter/gather, no repacking)
 *   - Pulses                             : 50 Hz fusion tick (timer -> pulse), sensor
 *                                          health-change pulses, automatic DISCONNECT pulse
 *                                          when a sensor process dies (-> sensor OFFLINE)
 *   - MsgDeliverEvent()                  : wakes the dashboard with a pulse when the
 *                                          decision changes (event registered by the client)
 *   - Shared memory (shm_open + seqlock) : full fused state published for any reader
 *   - Camera fusion                      : the camera's target size is cross-calibrated
 *                                          online against the ultrasonic (K = d * size), so
 *                                          the camera alone can keep the track if the
 *                                          ultrasonic fails; camera TTC is a fallback
 *   - Priority inheritance               : the receive thread runs at the sender's priority
 *                                          while it handles a message; mutexes inherit too
 *
 * Threads:
 *   fusion_tick  prio 30  timer pulse every 20 ms: predict, update, TTC, decision, deadline
 *   fusion_rx    prio 20  (inherits sender priority) receives sensor messages and pulses
 *   main         prio 10  status line, CSV log
 *
 * Usage: fusion_server [-A x|y|z|-x|-y|-z] [-l hogs] [-p hog_prio] [-o log.csv] [-t seconds]
 *   -A  which IMU axis points forward (towards the obstacle), default x
 *   -l  CPU-hog threads for the scheduling test, -p their priority (default 5)
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
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/iomsg.h>
#include <sys/siginfo.h>
#include "adas_msg.h"

/* ------------------------------------------------------------ configuration */
#define NS_PER_SEC         1000000000LL
#define NS_PER_MS          1000000LL
#define FUSION_PERIOD_NS   (20 * NS_PER_MS)
#define FUSION_DEADLINE_NS (20 * NS_PER_MS)
#define PRIO_TICK          30
#define PRIO_RX            20

#define RANGE_MAX_AGE_NS   (200 * NS_PER_MS)
#define CAM_MAX_AGE_NS     (200 * NS_PER_MS)
#define CAM_MIN_CONF       0.4
#define CAM_REL_SIGMA      0.06              /* camera distance ~6 % sigma     */
#define CAM_CALIB_MIN      10                /* paired samples before use      */
#define IMU_MAX_AGE_NS     (100 * NS_PER_MS)
#define SENSOR_STALE_NS    (500 * NS_PER_MS)
#define COAST_MAX_NS       (1000 * NS_PER_MS)

#define R_RANGE            (0.02 * 0.02)     /* HC-SR04 ~2 cm sigma            */
#define Q_ACCEL            3.0               /* hand / vehicle accel variance  */
#define GATE_SIGMA         4.0
#define REINIT_AFTER       3
#define ACC_DEADBAND       0.15              /* m/s^2, ignore IMU noise        */
#define GRAVITY_TAU_S      2.0               /* low-pass that tracks gravity   */

#define TTC_WARNING        4.0
#define DEC_HOLD_CYCLES    10                /* 200 ms before a warning may relax */
#define TTC_CRITICAL       2.0
#define TTC_EMERGENCY      1.0
#define D_EMERGENCY_M      0.15              /* distance floors: a close, stopped */
#define D_CRITICAL_M       0.30              /* obstacle is dangerous even with   */
#define D_WARNING_M        0.50              /* no closing speed (TTC infinite)   */

_Static_assert(sizeof(struct _pulse) <= sizeof(adas_hdr_t), "pulse must fit in IOV part 0");

/* ------------------------------------------------------------ shared state */
typedef struct {
    /* per sensor */
    uint8_t  health[SENS_COUNT];
    int64_t  last_rx_ns[SENS_COUNT];
    uint64_t msgs[SENS_COUNT], samples[SENS_COUNT], lost[SENS_COUNT];
    uint32_t last_seq[SENS_COUNT];
    bool     seq_valid[SENS_COUNT];
    float    rate_hz[SENS_COUNT], latency_us[SENS_COUNT];
    int      scoid[SENS_COUNT];

    /* IMU */
    imu_sample_msg_t imu_last;
    float    imu_temp;
    double   g_lp[3];            /* gravity estimate (low-pass of accel)          */
    bool     g_init;
    int64_t  g_last_t;
    double   lin_fwd;            /* forward linear acceleration, gravity removed  */
    double   lin_norm;
    int64_t  imu_t;

    /* range */
    range_sample_msg_t rng;
    bool     rng_new;
    float    range_cpu_us;

    /* camera */
    camera_sample_msg_t cam;
    bool     cam_new;

    /* dashboard notification */
    bool     dash_ok;
    int      dash_rcvid, dash_scoid;
    struct sigevent dash_ev;

    uint64_t pulses_rx, msgs_rx, disconnects, dash_events;
} fstate_t;

static fstate_t        g;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;   /* priority inheritance */
static adas_shm_t     *g_shm;
static volatile sig_atomic_t g_run = 1;
static int  g_axis = 0, g_axis_sign = 1;
static name_attach_t *g_attach;

/* CSV log ring (tick thread -> main) */
typedef struct {
    int64_t t; float raw, d, v, ttc, lin, exec_us, resp_us; int dec, miss, rh, ih;
} logrec_t;
#define LOG_N 2048
static logrec_t        g_logr[LOG_N];
static uint64_t        g_log_w, g_log_r;
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

static void set_prio(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
}

/* ============================================================ Kalman filter */
typedef struct { double x[2], P[2][2]; bool init; int rejects; } kf_t;

static void kf_init(kf_t *k, double d, double v)
{
    k->x[0] = d; k->x[1] = v;
    k->P[0][0] = R_RANGE * 4; k->P[1][1] = 1.0;
    k->P[0][1] = k->P[1][0] = 0;
    k->init = true; k->rejects = 0;
}

static void kf_predict(kf_t *k, double dt, double u)
{
    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    k->x[0] += k->x[1] * dt + 0.5 * u * dt2;
    k->x[1] += u * dt;
    double p00 = k->P[0][0], p01 = k->P[0][1], p10 = k->P[1][0], p11 = k->P[1][1];
    k->P[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 + Q_ACCEL * dt4 / 4.0;
    k->P[0][1] = p01 + dt * p11 + Q_ACCEL * dt3 / 2.0;
    k->P[1][0] = p10 + dt * p11 + Q_ACCEL * dt3 / 2.0;
    k->P[1][1] = p11 + Q_ACCEL * dt2;
}

static bool kf_update(kf_t *k, double z, double R)
{
    double y = z - k->x[0], S = k->P[0][0] + R;
    if (y * y / S > GATE_SIGMA * GATE_SIGMA) { k->rejects++; return false; }
    double K0 = k->P[0][0] / S, K1 = k->P[1][0] / S;
    k->x[0] += K0 * y;
    k->x[1] += K1 * y;
    double P00 = k->P[0][0], P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00; k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00; k->P[1][1] -= K1 * P01;
    k->rejects = 0;
    return true;
}

/* ============================================================ receive thread */
static void handle_camera(const camera_sample_msg_t *s, int n)
{
    if (n > 0) { g.cam = s[n - 1]; g.cam_new = true; }
}

static int sensor_by_scoid(int scoid)
{
    for (int s = 0; s < SENS_COUNT; s++)
        if (g.scoid[s] == scoid) return s;
    return -1;
}

static void handle_imu(const adas_hdr_t *h, const imu_sample_msg_t *s, int n)
{
    for (int i = 0; i < n; i++) {
        double dt = g.g_init ? (double)(s[i].t_ns - g.g_last_t) / 1e9 : 0;
        if (!g.g_init || dt <= 0 || dt > 0.5) {
            for (int k = 0; k < 3; k++) g.g_lp[k] = s[i].a[k];
            g.g_init = true;
        } else {
            double alpha = dt / (GRAVITY_TAU_S + dt);
            for (int k = 0; k < 3; k++) g.g_lp[k] += alpha * (s[i].a[k] - g.g_lp[k]);
        }
        g.g_last_t = s[i].t_ns;
        double lin[3], nn = 0;
        for (int k = 0; k < 3; k++) { lin[k] = s[i].a[k] - g.g_lp[k]; nn += lin[k] * lin[k]; }
        g.lin_fwd = g_axis_sign * lin[g_axis];
        g.lin_norm = sqrt(nn);
    }
    if (n > 0) { g.imu_last = s[n - 1]; g.imu_t = s[n - 1].t_ns; }
    g.imu_temp = h->extra;
}

static void handle_range(const adas_hdr_t *h, const range_sample_msg_t *s, int n)
{
    if (n > 0) { g.rng = s[n - 1]; g.rng_new = true; }
    g.range_cpu_us = h->extra;
}

static void *rx_thread(void *arg)
{
    (void)arg;
    union {
        uint16_t       type;
        struct _pulse  pulse;
        adas_hdr_t     hdr;
    } h;
    static uint8_t samples[ADAS_MAX_SAMPLES * sizeof(imu_sample_msg_t)];
    iov_t iov[2];
    SETIOV(&iov[0], &h, sizeof(adas_hdr_t));
    SETIOV(&iov[1], samples, sizeof samples);

    while (g_run) {
        struct _msg_info info;
        int rcvid = MsgReceivev(g_attach->chid, iov, 2, &info);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("MsgReceivev");
            break;
        }

        if (rcvid == 0) {                                   /* ---------- pulse */
            pthread_mutex_lock(&g_lock);
            g.pulses_rx++;
            switch (h.pulse.code) {
            case _PULSE_CODE_DISCONNECT: {                  /* a client process went away */
                int s = sensor_by_scoid(h.pulse.scoid);
                if (s >= 0) { g.health[s] = SH_OFFLINE; g.scoid[s] = 0; g.seq_valid[s] = false; }
                if (g.dash_ok && g.dash_scoid == h.pulse.scoid) g.dash_ok = false;
                g.disconnects++;
                ConnectDetach(h.pulse.scoid);
                break;
            }
            case PC_SENSOR_HEALTH: {
                int v = h.pulse.value.sival_int;
                int s = (v >> 8) & 0xff, hl = v & 0xff;
                if (s < SENS_COUNT && hl <= SH_OFFLINE) g.health[s] = (uint8_t)hl;
                break;
            }
            default:
                break;
            }
            pthread_mutex_unlock(&g_lock);
            continue;
        }

        if (h.type == _IO_CONNECT) {                        /* name_open() handshake */
            MsgReply(rcvid, EOK, NULL, 0);
            continue;
        }

        if (h.type == MSG_SENSOR_DATA) {
            int64_t t_rx = now_ns();
            const adas_hdr_t *hd = &h.hdr;
            size_t need = sizeof(adas_hdr_t) + (size_t)hd->count * hd->sample_size;
            size_t want = hd->sensor == SENS_IMU   ? sizeof(imu_sample_msg_t)
                        : hd->sensor == SENS_RANGE ? sizeof(range_sample_msg_t)
                        :                            sizeof(camera_sample_msg_t);
            bool ok = hd->sensor < SENS_COUNT && hd->count <= ADAS_MAX_SAMPLES &&
                      info.srcmsglen >= need && hd->sample_size == want;
            if (!ok) { MsgError(rcvid, EINVAL); continue; }

            pthread_mutex_lock(&g_lock);
            int s = hd->sensor;
            g.msgs_rx++;
            g.msgs[s]++;
            g.samples[s] += hd->count;
            if (g.seq_valid[s] && hd->seq != g.last_seq[s] + 1) g.lost[s] += hd->seq - g.last_seq[s] - 1;
            g.last_seq[s] = hd->seq;
            g.seq_valid[s] = true;
            g.scoid[s] = info.scoid;
            g.health[s] = (uint8_t)hd->health;
            g.last_rx_ns[s] = t_rx;
            g.rate_hz[s] = hd->rate_hz;
            float lat = (float)(t_rx - hd->t_send_ns) / 1e3f;
            g.latency_us[s] = g.latency_us[s] == 0 ? lat : 0.9f * g.latency_us[s] + 0.1f * lat;
            if (s == SENS_IMU) handle_imu(hd, (const imu_sample_msg_t *)samples, hd->count);
            else if (s == SENS_RANGE) handle_range(hd, (const range_sample_msg_t *)samples, hd->count);
            else if (s == SENS_CAMERA) handle_camera((const camera_sample_msg_t *)samples, hd->count);
            pthread_mutex_unlock(&g_lock);

            MsgReply(rcvid, EOK, NULL, 0);                  /* unblocks the sensor */
            continue;
        }

        if (h.type == MSG_DASH_REGISTER) {
            dash_register_msg_t m;
            if (MsgRead(rcvid, &m, sizeof m, 0) < (ssize_t)sizeof m) { MsgError(rcvid, EINVAL); continue; }
            pthread_mutex_lock(&g_lock);
            g.dash_ev = m.event;
            g.dash_rcvid = rcvid;
            g.dash_scoid = info.scoid;
            g.dash_ok = true;
            pthread_mutex_unlock(&g_lock);
            MsgReply(rcvid, EOK, NULL, 0);
            continue;
        }

        MsgError(rcvid, ENOSYS);
    }
    return NULL;
}

/* ============================================================ fusion tick */
static void log_push(const logrec_t *r)
{
    pthread_mutex_lock(&g_log_lock);
    if (g_log_w - g_log_r < LOG_N) g_logr[g_log_w++ % LOG_N] = *r;
    pthread_mutex_unlock(&g_log_lock);
}

static void *tick_thread(void *arg)
{
    (void)arg;
    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, PRIO_TICK, PC_FUSION_TICK, 0);
    timer_t tid;
    if (chid == -1 || coid == -1 || timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) {
        perror("fusion timer"); g_run = 0; return NULL;
    }
    int64_t start = now_ns() + FUSION_PERIOD_NS;
    struct itimerspec its;
    ns_to_ts(start, &its.it_value);
    ns_to_ts(FUSION_PERIOD_NS, &its.it_interval);
    timer_settime(tid, TIMER_ABSTIME, &its, NULL);

    kf_t kf = { 0 };
    uint64_t k = 0, misses = 0;
    double worst_exec = 0, worst_resp = 0;
    int64_t last_t = start, last_meas_t = 0;
    double cam_K = 0;                    /* camera scale: distance = K / size_px */
    int cam_calib_n = 0;
    int64_t cam_used_t = 0;
    int last_dec = -1, since_notify = 0;
    int cur_dec = DEC_NO_TRACK, relax_count = 0;

    while (g_run) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (p.code != PC_FUSION_TICK) continue;
        int64_t release = start + (int64_t)k * FUSION_PERIOD_NS;
        k++;
        int64_t t0 = now_ns();
        double dt = (double)(t0 - last_t) / 1e9;
        last_t = t0;

        /* ---- snapshot inputs (short critical section) */
        pthread_mutex_lock(&g_lock);
        for (int s = 0; s < SENS_COUNT; s++)
            if (g.health[s] != SH_OFFLINE && g.last_rx_ns[s] && t0 - g.last_rx_ns[s] > SENSOR_STALE_NS)
                g.health[s] = SH_DEGRADED;                  /* alive but silent */
        uint8_t rh = g.health[SENS_RANGE], ih = g.health[SENS_IMU], ch = g.health[SENS_CAMERA];
        bool rnew = g.rng_new;
        range_sample_msg_t rng = g.rng;
        g.rng_new = false;
        bool cnew = g.cam_new;
        camera_sample_msg_t cam = g.cam;
        g.cam_new = false;
        double lin = g.lin_fwd, lin_norm = g.lin_norm;
        int64_t imu_t = g.imu_t;
        pthread_mutex_unlock(&g_lock);

        /* ---- IMU: ego acceleration as control input */
        double u = 0;
        bool imu_ok = ih == SH_OK && imu_t && t0 - imu_t < IMU_MAX_AGE_NS;
        if (imu_ok && fabs(lin) > ACC_DEADBAND) u = -lin;    /* ego accelerating -> gap closes */

        /* ---- predict */
        if (kf.init) kf_predict(&kf, dt, u);

        /* ---- camera: is there a usable detection? */
        bool cam_seen = ch == SH_OK && cam.scale_px > 0 && cam.conf >= CAM_MIN_CONF &&
                        t0 - cam.t_ns < CAM_MAX_AGE_NS;
        bool range_usable = !(rh == SH_NO_TARGET || rh == SH_OFFLINE || rh == SH_FAULT);
        bool range_fresh = range_usable && rnew && t0 - rng.t_ns >= 0 && t0 - rng.t_ns < RANGE_MAX_AGE_NS;

        /* ---- online cross-calibration: K = ultrasonic distance x camera size (pinhole model) */
        if (range_fresh && cam_seen && llabs(cam.t_ns - rng.t_ns) < 100 * NS_PER_MS) {
            double Kobs = rng.raw_m * cam.scale_px;
            if (cam_calib_n == 0) cam_K = Kobs;
            else if (cam_calib_n < 5 || fabs(Kobs - cam_K) < 0.5 * cam_K) cam_K = 0.9 * cam_K + 0.1 * Kobs;
            cam_calib_n++;
        }
        bool cam_calibrated = cam_calib_n >= CAM_CALIB_MIN;
        double cam_dist = (cam_calibrated && cam_seen) ? cam_K / cam.scale_px : NAN;
        bool cam_used = false;

        /* ---- measurement updates */
        double raw = NAN;
        if (!range_usable && isnan(cam_dist)) {
            kf.init = false;                                /* nothing to track */
        } else {
            if (range_fresh) {                              /* ultrasonic: precise */
                double a = (double)(t0 - rng.t_ns) / 1e9;
                raw = rng.raw_m;
                double z = rng.raw_m + (kf.init ? kf.x[1] * a : 0);   /* align to now */
                if (!kf.init) kf_init(&kf, z, rng.vel_ms);
                else if (!kf_update(&kf, z, R_RANGE) && kf.rejects >= REINIT_AFTER) kf_init(&kf, z, rng.vel_ms);
                last_meas_t = t0;
            }
            if (cnew && !isnan(cam_dist)) {                 /* camera: wider error, keeps track alive */
                double a = (double)(t0 - cam.t_ns) / 1e9;
                double z = cam_dist + (kf.init ? kf.x[1] * a : 0);
                double Rc = (CAM_REL_SIGMA * cam_dist) * (CAM_REL_SIGMA * cam_dist);
                if (!kf.init) kf_init(&kf, z, 0);
                else if (!kf_update(&kf, z, Rc) && kf.rejects >= REINIT_AFTER && !range_usable) kf_init(&kf, z, 0);
                last_meas_t = t0;
                cam_used = true;
                cam_used_t = t0;
            }
        }
        if (kf.init && t0 - last_meas_t > COAST_MAX_NS) kf.init = false;

        /* ---- TTC + decision */
        double ttc = INFINITY;
        int dec;
        if (!kf.init) {
            /* no distance track: fall back to the camera's own expansion TTC */
            if (cam_seen && isfinite(cam.ttc_s)) {
                ttc = cam.ttc_s;
                dec = ttc < TTC_EMERGENCY ? DEC_EMERGENCY : ttc < TTC_CRITICAL ? DEC_CRITICAL
                    : ttc < TTC_WARNING ? DEC_WARNING : DEC_SAFE;
            } else {
                dec = (rh == SH_NO_TARGET || cam_seen) ? DEC_SAFE : DEC_NO_TRACK;
            }
        } else {
            if (kf.x[1] < -0.05) ttc = kf.x[0] / -kf.x[1];
            double d = kf.x[0];
            /* the more severe of the time-based (TTC) and distance-based levels */
            if      (ttc < TTC_EMERGENCY || d < D_EMERGENCY_M) dec = DEC_EMERGENCY;
            else if (ttc < TTC_CRITICAL  || d < D_CRITICAL_M)  dec = DEC_CRITICAL;
            else if (ttc < TTC_WARNING   || d < D_WARNING_M)   dec = DEC_WARNING;
            else                                               dec = DEC_SAFE;
        }

        /* ---- hysteresis: escalate at once, relax only after DEC_HOLD_CYCLES agreeing cycles */
        if (dec == DEC_NO_TRACK || cur_dec == DEC_NO_TRACK || dec >= cur_dec) {
            cur_dec = dec;
            relax_count = 0;
        } else if (++relax_count >= DEC_HOLD_CYCLES) {
            cur_dec = dec;
            relax_count = 0;
        }
        dec = cur_dec;

        /* ---- deadline monitoring */
        int64_t t1 = now_ns();
        double exec_us = (double)(t1 - t0) / 1e3, resp_us = (double)(t1 - release) / 1e3;
        int miss = (t1 - release) > FUSION_DEADLINE_NS;
        misses += miss;
        if (exec_us > worst_exec) worst_exec = exec_us;
        if (resp_us > worst_resp) worst_resp = resp_us;

        /* ---- publish to shared memory (seqlock) */
        pthread_mutex_lock(&g_lock);
        shm_write_begin(g_shm);
        g_shm->t_ns = t1;
        g_shm->update_count++;
        for (int s = 0; s < SENS_COUNT; s++) {
            g_shm->sens[s].health = g.health[s];
            g_shm->sens[s].rate_hz = g.rate_hz[s];
            g_shm->sens[s].latency_us = g.latency_us[s];
            g_shm->sens[s].msgs = g.msgs[s];
            g_shm->sens[s].samples = g.samples[s];
            g_shm->sens[s].lost_msgs = g.lost[s];
            g_shm->sens[s].last_t_ns = g.last_rx_ns[s];
        }
        for (int i = 0; i < 3; i++) { g_shm->imu_a[i] = g.imu_last.a[i]; g_shm->imu_g[i] = g.imu_last.g[i]; }
        g_shm->imu_roll = g.imu_last.roll;
        g_shm->imu_pitch = g.imu_last.pitch;
        g_shm->imu_temp = g.imu_temp;
        g_shm->imu_lin_acc = (float)lin;
        g_shm->range_raw_m = g.rng.raw_m;
        g_shm->range_m = g.rng.dist_m;
        g_shm->range_vel_ms = g.rng.vel_ms;
        g_shm->range_echo_us = g.rng.echo_us;
        g_shm->range_cpu_us = g.range_cpu_us;
        g_shm->cam_scale_px = cam.scale_px;
        g_shm->cam_cx = cam.cx;
        g_shm->cam_cy = cam.cy;
        g_shm->cam_conf = cam.conf;
        g_shm->cam_ttc_s = cam.ttc_s;
        g_shm->cam_dist_m = (float)cam_dist;
        g_shm->cam_K = (float)cam_K;
        g_shm->cam_proc_us = cam.proc_us;
        g_shm->cam_age_ms = cam.t_ns ? (float)((t0 - cam.t_ns) / 1e6) : 0;
        g_shm->cam_w = cam.width;
        g_shm->cam_h = cam.height;
        g_shm->cam_calibrated = cam_calibrated;
        g_shm->cam_calib_samples = cam_calib_n;
        g_shm->cam_used = cam_used || (cam_used_t && t0 - cam_used_t < CAM_MAX_AGE_NS);
        g_shm->fused_d = kf.init ? (float)kf.x[0] : NAN;
        g_shm->fused_v = kf.init ? (float)kf.x[1] : NAN;
        g_shm->d_var = kf.init ? (float)kf.P[0][0] : NAN;
        g_shm->ttc_s = (float)ttc;
        g_shm->decision = dec;
        g_shm->ego_moving = lin_norm > 0.3;
        g_shm->exec_us = (float)exec_us;
        g_shm->resp_us = (float)resp_us;
        g_shm->worst_exec_us = (float)worst_exec;
        g_shm->worst_resp_us = (float)worst_resp;
        g_shm->cycles = k;
        g_shm->misses = misses;
        g_shm->pulses_rx = g.pulses_rx;
        g_shm->msgs_rx = g.msgs_rx;
        g_shm->disconnects = g.disconnects;
        g_shm->dash_events = g.dash_events;
        shm_write_end(g_shm);

        /* ---- wake the dashboard: immediately on a decision change, else every 100 ms */
        if (g.dash_ok && (dec != last_dec || ++since_notify >= 5)) {
            if (MsgDeliverEvent(g.dash_rcvid, &g.dash_ev) == -1) g.dash_ok = false;
            else g.dash_events++;
            since_notify = 0;
        }
        pthread_mutex_unlock(&g_lock);
        last_dec = dec;

        logrec_t r = { t1, (float)raw, (float)(kf.init ? kf.x[0] : NAN), (float)(kf.init ? kf.x[1] : NAN),
                       (float)ttc, (float)lin, (float)exec_us, (float)resp_us, dec, miss, rh, ih };
        log_push(&r);
    }
    timer_delete(tid);
    ConnectDetach(coid);
    ChannelDestroy(chid);
    return NULL;
}

/* ============================================================ CPU-load test */
static void *hog_thread(void *arg)
{
    (void)arg;
    while (g_run) {
        int64_t end = now_ns() + 30 * NS_PER_MS;
        while (g_run && now_ns() < end)
            ;
        struct timespec ts = { 0, 10 * NS_PER_MS };
        nanosleep(&ts, NULL);
    }
    return NULL;
}

static void start_thread(pthread_t *t, void *(*fn)(void *), int prio, const char *name)
{
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&a, SCHED_FIFO);
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    pthread_attr_setschedparam(&a, &sp);
    if (pthread_create(t, &a, fn, NULL) != 0) {
        fprintf(stderr, "warning: %s at prio %d failed, using default\n", name, prio);
        pthread_create(t, NULL, fn, NULL);
    }
    pthread_attr_destroy(&a);
    pthread_setname_np(*t, name);
}

static void on_signal(int sig) { (void)sig; g_run = 0; }

/* ============================================================ main */
int main(int argc, char **argv)
{
    int hogs = 0, hog_prio = 5, run_s = 0, opt;
    const char *log_path = NULL;

    while ((opt = getopt(argc, argv, "A:l:p:o:t:")) != -1) {
        switch (opt) {
        case 'A':
            g_axis_sign = optarg[0] == '-' ? -1 : 1;
            g_axis = (optarg[optarg[0] == '-'] | 0x20) - 'x';
            if (g_axis < 0 || g_axis > 2) { fprintf(stderr, "-A must be x, y, z, -x, -y or -z\n"); return 1; }
            break;
        case 'l': hogs = atoi(optarg); break;
        case 'p': hog_prio = atoi(optarg); break;
        case 'o': log_path = optarg; break;
        case 't': run_s = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-A x|y|z|-x|-y|-z] [-l hogs] [-p prio] [-o log.csv] [-t sec]\n", argv[0]);
            return 1;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    set_prio(10);

    /* shared memory for the dashboard (and any other reader) */
    int fd = shm_open(ADAS_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (fd == -1) { perror("shm_open"); return 1; }
    fchmod(fd, 0666);
    if (ftruncate(fd, sizeof(adas_shm_t)) == -1) { perror("ftruncate"); return 1; }
    g_shm = mmap(NULL, sizeof(adas_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (g_shm == MAP_FAILED) { perror("mmap shm"); return 1; }
    close(fd);
    memset(g_shm, 0, sizeof *g_shm);
    g_shm->version = ADAS_SHM_VERSION;
    g_shm->deadline_us = FUSION_DEADLINE_NS / 1e3f;
    g_shm->fusion_pid = getpid();

    for (int s = 0; s < SENS_COUNT; s++) g.health[s] = SH_OFFLINE;

    /* named server channel: sensors use name_open("adas_fusion") */
    g_attach = name_attach(NULL, ADAS_SERVER_NAME, 0);
    if (!g_attach) {
        perror("name_attach (is another fusion_server running?)");
        return 1;
    }

    printf("ADAS fusion server pid %d | name '%s' chid %d | shm %s (%zu bytes) | IMU forward axis %c%c\n",
           getpid(), ADAS_SERVER_NAME, g_attach->chid, ADAS_SHM_NAME, sizeof(adas_shm_t),
           g_axis_sign < 0 ? '-' : '+', 'x' + g_axis);
    printf("waiting for sensors... start imu_node, us_node and dashboard in any order\n\n");

    FILE *logf = NULL;
    if (log_path && (logf = fopen(log_path, "w")))
        fprintf(logf, "t_ns,raw_m,fused_d,fused_v,ttc,imu_lin_fwd,exec_us,resp_us,decision,miss,range_health,imu_health\n");

    pthread_t th_tick, th_rx;
    pthread_t *th_hog = hogs > 0 ? calloc((size_t)hogs, sizeof(pthread_t)) : NULL;
    start_thread(&th_rx, rx_thread, PRIO_RX, "fusion_rx");
    start_thread(&th_tick, tick_thread, PRIO_TICK, "fusion_tick");
    for (int i = 0; i < hogs; i++) start_thread(&th_hog[i], hog_thread, hog_prio, "cpu_hog");
    pthread_setname_np(pthread_self(), "fusion_main");

    int64_t t_end = run_s > 0 ? now_ns() + (int64_t)run_s * NS_PER_SEC : 0;
    while (g_run) {
        struct timespec ts = { 1, 0 };
        nanosleep(&ts, NULL);

        /* drain CSV */
        for (;;) {
            logrec_t r;
            pthread_mutex_lock(&g_log_lock);
            bool have = g_log_r < g_log_w;
            if (have) r = g_logr[g_log_r++ % LOG_N];
            pthread_mutex_unlock(&g_log_lock);
            if (!have) break;
            if (logf)
                fprintf(logf, "%lld,%.4f,%.4f,%.4f,%.3f,%.3f,%.1f,%.1f,%s,%d,%s,%s\n", (long long)r.t, r.raw,
                        r.d, r.v, isinf(r.ttc) ? -1.0 : r.ttc, r.lin, r.exec_us, r.resp_us,
                        DECISION_STR[r.dec], r.miss, HEALTH_STR[r.rh], HEALTH_STR[r.ih]);
        }
        if (logf) fflush(logf);

        adas_shm_t s;
        if (shm_read(g_shm, &s) == 0)
            printf("fusion | cycles %llu miss %llu worst %.0fus | IMU %s %llu msg %.0fus | RANGE %s %llu msg %.0fus | "
                   "CAM %s K %s | d %.2fm v %+.2f | %s | pulses %llu disc %llu dash %s\n",
                   (unsigned long long)s.cycles, (unsigned long long)s.misses, s.worst_resp_us,
                   HEALTH_STR[s.sens[SENS_IMU].health], (unsigned long long)s.sens[SENS_IMU].msgs,
                   s.sens[SENS_IMU].latency_us,
                   HEALTH_STR[s.sens[SENS_RANGE].health], (unsigned long long)s.sens[SENS_RANGE].msgs,
                   s.sens[SENS_RANGE].latency_us, HEALTH_STR[s.sens[SENS_CAMERA].health],
                   s.cam_calibrated ? "calibrated" : "learning", s.fused_d, s.fused_v, DECISION_STR[s.decision],
                   (unsigned long long)s.pulses_rx, (unsigned long long)s.disconnects,
                   g.dash_ok ? "on" : "off");
        fflush(stdout);
        if (t_end && now_ns() >= t_end) g_run = 0;
    }

    /* the rx thread is blocked in MsgReceivev; detaching the name ends it */
    pthread_join(th_tick, NULL);
    name_detach(g_attach, 0);
    pthread_cancel(th_rx);
    for (int i = 0; i < hogs; i++) pthread_join(th_hog[i], NULL);
    if (logf) fclose(logf);
    munmap(g_shm, sizeof *g_shm);
    shm_unlink(ADAS_SHM_NAME);
    printf("fusion server stopped\n");
    return 0;
}
