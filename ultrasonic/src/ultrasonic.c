/*
 * us_node.c  -  QNX-eHACK 2026: HC-SR04 range process of the ADAS fusion system
 *
 * Same measurement as ultrasonic_hcsr04.c, plus:
 *   - INTERRUPT-DRIVEN ECHO TIMING: the BCM2711 GPIO edge detector (rising + falling) raises
 *     GPIO bank-0 interrupt 145; InterruptAttachEvent() turns it into a pulse on the
 *     measurement thread's channel. The thread sleeps instead of busy-polling for up to
 *     25 ms per ping. If no interrupt arrives the node falls back to polling automatically.
 *     Use -I 0 to force polling, -I <n> for another interrupt number.
 *   - Every ping is sent to fusion_server as an IOV message (header + range sample);
 *     health changes are also sent as pulses.
 *
 * What makes this different from the usual "trigger, wait, divide by 58" example:
 *
 *  1. HARDWARE-COUNTER ECHO TIMING
 *     The echo pulse is timed with ClockCycles() - the ARM generic timer (54 MHz on the
 *     Pi 4, ~18.5 ns resolution) - by a SCHED_FIFO priority-29 thread with direct access
 *     to the BCM2711 GPIO registers. No sleeps inside the measurement.
 *
 *  2. TIMESTAMP AT THE REFLECTION INSTANT
 *     Each range gets a CLOCK_MONOTONIC timestamp of the moment the sound hit the object
 *     (middle of the echo pulse), not the moment the program finished reading. Same
 *     clock as the IMU, so the fusion stage can align them.
 *
 *  3. TEMPERATURE-COMPENSATED SPEED OF SOUND
 *     c = 331.3 + 0.606 * T  [m/s]. Default T = 28 C (what the MPU6050 measured on our
 *     bench); later the fusion process can feed the IMU temperature in. A 10 C error
 *     would otherwise cause ~1.8 % range error.
 *
 *  4. CLOSING VELOCITY + TIME-TO-COLLISION ON THE SENSOR ITSELF
 *     Alpha-beta tracker gives distance and closing speed; TTC -> SAFE / WARNING /
 *     CRITICAL / EMERGENCY, using the same thresholds as the fusion design.
 *
 *  5. OUTLIER GATING + HEALTH MONITOR
 *     Spikes are rejected by an innovation gate (re-init after 3 consecutive
 *     disagreements). Missing echo start (wiring / divider fault), echo stuck high and
 *     "no target in range" are detected separately -> OK / NO TARGET / DEGRADED / FAULT.
 *
 *  6. DATASHEET-SAFE PING RATE
 *     15 Hz (66.7 ms period) from a QNX timer -> pulse, above the 60 ms the HC-SR04
 *     needs so echoes from the previous ping cannot be mistaken for new ones.
 *
 * Wiring (Raspberry Pi 4 header) - ECHO IS 5 V, USE THE DIVIDER:
 *     HC-SR04 VCC  -> pin 2  (5 V)
 *     HC-SR04 GND  -> pin 14 (GND)
 *     HC-SR04 TRIG -> pin 16 (GPIO23)            (3.3 V is enough to trigger)
 *     HC-SR04 ECHO -> 1k -> pin 18 (GPIO24) -> 2k (two 1k in series) -> pin 20 (GND)
 *
 * Needs root (maps the GPIO registers at physical 0xFE200000).
 * Build: Momentics QNX project, aarch64le, link with -lm.
 * Usage: ultrasonic [-g trig_gpio] [-e echo_gpio] [-T temp_C] [-r rate_hz] [-t seconds] [-o log.csv]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#include <sys/syspage.h>
#include <sys/dispatch.h>
#include <sys/uio.h>
#include "adas_msg.h"

/* ------------------------------------------------------------ BCM2711 GPIO */
#define GPIO_PHYS_BASE   0xFE200000UL
#define GPIO_MAP_LEN     0x100
#define REG_GPSET0       (0x1C / 4)
#define REG_GPCLR0       (0x28 / 4)
#define REG_GPLEV0       (0x34 / 4)
#define REG_PUP_PDN0     (0xE4 / 4)     /* BCM2711 pull-up/down, 2 bits per pin */
#define REG_GPEDS0       (0x40 / 4)     /* event detect status (write 1 to clear) */
#define REG_GPREN0       (0x4C / 4)     /* rising-edge detect enable  */
#define REG_GPFEN0       (0x58 / 4)     /* falling-edge detect enable */
#define GPIO_BANK0_IRQ   145            /* GIC SPI 113 + 32           */

/* ------------------------------------------------------------ timing */
#define NS_PER_SEC       1000000000LL
#define NS_PER_MS        1000000LL
#define PRIO_US          29
#define PULSE_CODE_TICK  (_PULSE_CODE_MINAVAIL + 1)

#define TRIG_PULSE_US    12             /* datasheet: >= 10 us                */
#define ECHO_START_TO_US 5000           /* echo must rise within 5 ms         */
#define ECHO_MAX_US      25000          /* ~4.3 m; longer = nothing in range  */
#define STUCK_FAULT_PINGS 15            /* ECHO high at every ping for 1 s     */

#define RANGE_MIN_M      0.02
#define RANGE_MAX_M      4.0

/* ------------------------------------------------------------ tracker / risk */
#define AB_ALPHA         0.5
#define AB_BETA          0.15
#define REINIT_AFTER     3
#define FAULT_AFTER      5              /* consecutive wiring-type failures   */

#define TTC_WARNING      4.0
#define TTC_CRITICAL     2.0
#define TTC_EMERGENCY    1.0
#define D_EMERGENCY_M    0.10

typedef enum { M_OK, M_NO_START, M_NO_ECHO, M_STUCK_HIGH } mres_t;
typedef enum { H_OK, H_NO_TARGET, H_DEGRADED, H_FAULT } health_t;
static const char *HEALTH_NAME[] = { "OK", "NO TARGET", "DEGRADED", "FAULT" };
enum { UDEC_SAFE, UDEC_WARNING, UDEC_CRITICAL, UDEC_EMERGENCY, UDEC_NO_DATA };
static const char *UDEC_NAME[] = { "SAFE", "WARNING", "CRITICAL", "EMERGENCY", "NO DATA" };

typedef struct {
    bool    init;
    double  d, v;       /* distance [m], range rate [m/s] (negative = closing) */
    int64_t t;
    int     rejects;
    bool    pend;       /* track initiation: first reading waits for confirmation */
    double  pz;
    int64_t pt;
} ab_t;

typedef struct {
    int64_t  t_meas;
    double   raw_m, dist_m, vel_ms, ttc, echo_us, c_sound, rate_hz, busy_us, cpu_us;
    int      decision;
    health_t health;
    uint64_t pings, valid, no_target, no_start, stuck, rejected, reinits;
} us_status_t;

/* ------------------------------------------------------------ globals */
static volatile sig_atomic_t g_run = 1;
static volatile uint32_t *g_gpio;
static int      g_trig = 23, g_echo = 24;
static double   g_temp_c = 28.0, g_rate_hz = 15.0;
static uint64_t g_cps;                  /* ClockCycles per second */

/* interrupt mode */
static int      g_irq = GPIO_BANK0_IRQ;
static int      g_iid = -1;                      /* InterruptAttachEvent id, -1 = polling */
static int      g_chid = -1;
static uint64_t g_irq_events, g_irq_fallbacks;
static int      g_irq_broken;
static bool     g_irq_was;                       /* mode at shutdown, for the summary */

/* link to fusion_server */
static int      g_coid = -1;
static uint32_t g_seq;
static int64_t  g_next_connect;
static int      g_sent_health = -1;
static uint64_t g_msgs_sent;

static us_status_t     g_st;
static pthread_mutex_t g_st_lock = PTHREAD_MUTEX_INITIALIZER;

#define LOG_N 1024
static us_status_t     g_log[LOG_N];
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

static inline uint64_t us_to_cycles(uint64_t us) { return g_cps * us / 1000000ULL; }

/* ============================================================ GPIO (direct registers) */
static void gpio_mode(int pin, bool output)
{
    int reg = pin / 10, sh = (pin % 10) * 3;
    uint32_t v = g_gpio[reg];
    v &= ~(7u << sh);
    if (output) v |= 1u << sh;
    g_gpio[reg] = v;
}

static void gpio_pull(int pin, uint32_t mode)       /* 0 none, 1 up, 2 down */
{
    int reg = REG_PUP_PDN0 + pin / 16, sh = (pin % 16) * 2;
    uint32_t v = g_gpio[reg];
    v &= ~(3u << sh);
    v |= (mode & 3u) << sh;
    g_gpio[reg] = v;
}

static inline int  gpio_read(int pin) { return (int)((g_gpio[REG_GPLEV0] >> pin) & 1u); }
static inline void gpio_high(int pin) { g_gpio[REG_GPSET0] = 1u << pin; }
static inline void gpio_low(int pin)  { g_gpio[REG_GPCLR0] = 1u << pin; }

/* ============================================================ one ping */
/*
 * Fires a trigger pulse and times the echo with the hardware cycle counter.
 * *echo_us = echo pulse width; *t_reflect = CLOCK_MONOTONIC of the reflection instant.
 */
static mres_t ping(double *echo_us, int64_t *t_reflect)
{
    uint64_t lim;

    /* Lost echoes: cheap HC-SR04s keep ECHO high for 38 ms or more. Never busy-wait
     * for that at high priority - just skip this ping, the next one will be clean. */
    if (gpio_read(g_echo))
        return M_STUCK_HIGH;

    gpio_high(g_trig);
    lim = ClockCycles() + us_to_cycles(TRIG_PULSE_US);
    while (ClockCycles() < lim)
        ;
    gpio_low(g_trig);

    lim = ClockCycles() + us_to_cycles(ECHO_START_TO_US);
    while (!gpio_read(g_echo))
        if (ClockCycles() > lim) return M_NO_START;
    uint64_t rise = ClockCycles();

    lim = rise + us_to_cycles(ECHO_MAX_US);
    while (gpio_read(g_echo))
        if (ClockCycles() > lim) return M_NO_ECHO;
    uint64_t fall = ClockCycles();
    int64_t t_fall = now_ns();

    double width_s = (double)(fall - rise) / (double)g_cps;
    *echo_us   = width_s * 1e6;
    *t_reflect = t_fall - (int64_t)(width_s * 0.5 * 1e9);
    return M_OK;
}

/* ============================================================ interrupt-driven ping */
/* receive one pulse with a relative timeout (ns); returns -1 on timeout */
static int recv_pulse_timeout(struct _pulse *p, uint64_t timeout_ns)
{
    uint64_t to = timeout_ns;
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
    return MsgReceivePulse(g_chid, p, sizeof *p, NULL);
}

/* throw away stale interrupt pulses from the previous ping (keeps IRQ unmasked) */
static int drain_irq_pulses(void)
{
    int ticks = 0;
    struct _pulse p;
    while (recv_pulse_timeout(&p, 0) != -1) {
        if (p.code == PC_GPIO_IRQ) InterruptUnmask(g_irq, g_iid);
        else if (p.code == PC_SENSOR_TICK) ticks++;
    }
    return ticks;
}

static mres_t ping_irq(double *echo_us, int64_t *t_reflect)
{
    uint32_t bit = 1u << g_echo;
    drain_irq_pulses();
    if (gpio_read(g_echo)) return M_STUCK_HIGH;
    g_gpio[REG_GPEDS0] = bit;                       /* clear old edge events */

    gpio_high(g_trig);
    uint64_t lim = ClockCycles() + us_to_cycles(TRIG_PULSE_US);
    while (ClockCycles() < lim)
        ;
    gpio_low(g_trig);

    uint64_t rise = 0, fall = 0;
    int64_t t_fall = 0;
    int64_t deadline = now_ns() + (int64_t)ECHO_START_TO_US * 1000;
    for (;;) {
        int64_t left = deadline - now_ns();
        struct _pulse p;
        if (left <= 0 || recv_pulse_timeout(&p, (uint64_t)left) == -1) {
            if (!rise) {
                if (gpio_read(g_echo)) {                /* echo DID start: IRQ not delivered */
                    if (++g_irq_broken >= 3) {
                        InterruptDetach(g_iid);
                        g_iid = -1;
                        g_irq_fallbacks++;
                        fprintf(stderr, "no GPIO interrupts on IRQ %d - switching to polling\n", g_irq);
                    }
                    return M_STUCK_HIGH;                /* skip this ping */
                }
                return M_NO_START;
            }
            return M_NO_ECHO;
        }
        if (p.code != PC_GPIO_IRQ) continue;            /* (timer tick during a ping) */
        uint64_t c = ClockCycles();
        g_irq_events++;
        if (g_gpio[REG_GPEDS0] & bit) {
            g_gpio[REG_GPEDS0] = bit;
            int lvl = gpio_read(g_echo);
            if (!rise && lvl) {
                rise = c;
                g_irq_broken = 0;
                deadline = now_ns() + (int64_t)ECHO_MAX_US * 1000;
            } else if (rise && !lvl) {
                fall = c;
                t_fall = now_ns();
            }
        }
        InterruptUnmask(g_irq, g_iid);
        if (fall) break;
    }
    double width_s = (double)(fall - rise) / (double)g_cps;
    *echo_us   = width_s * 1e6;
    *t_reflect = t_fall - (int64_t)(width_s * 0.5 * 1e9);
    return M_OK;
}

static void irq_setup(void)
{
    if (g_irq <= 0) return;
#if defined(_NTO_TCTL_IO_LEVEL)
    if (ThreadCtl(_NTO_TCTL_IO_LEVEL, (void *)_NTO_IO_LEVEL_1) == -1)
#else
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1)
#endif
        perror("ThreadCtl (I/O privilege)");
    int coid = ConnectAttach(0, 0, g_chid, _NTO_SIDE_CHANNEL, 0);
    struct sigevent iev;
    SIGEV_PULSE_INIT(&iev, coid, PRIO_US, PC_GPIO_IRQ, 0);
    uint32_t bit = 1u << g_echo;
    g_gpio[REG_GPREN0] |= bit;                      /* rising and falling edges of ECHO */
    g_gpio[REG_GPFEN0] |= bit;
    g_gpio[REG_GPEDS0] = bit;
    g_iid = InterruptAttachEvent(g_irq, &iev, _NTO_INTR_FLAGS_TRK_MSK);
    if (g_iid == -1) {
        fprintf(stderr, "InterruptAttachEvent(%d) failed: %s - using polling\n", g_irq, strerror(errno));
        g_gpio[REG_GPREN0] &= ~bit;
        g_gpio[REG_GPFEN0] &= ~bit;
    }
}

static void irq_teardown(void)
{
    uint32_t bit = 1u << g_echo;
    g_irq_was = g_iid != -1;
    if (g_iid != -1) InterruptDetach(g_iid);
    g_iid = -1;
    g_gpio[REG_GPREN0] &= ~bit;
    g_gpio[REG_GPFEN0] &= ~bit;
    g_gpio[REG_GPEDS0] = bit;
}

/* ============================================================ fusion link (IPC) */
static int to_sh(health_t h)
{
    return h == H_OK ? SH_OK : h == H_NO_TARGET ? SH_NO_TARGET : h == H_DEGRADED ? SH_DEGRADED : SH_FAULT;
}

static void fusion_link(int64_t t)
{
    if (g_coid != -1 || t < g_next_connect) return;
    g_coid = name_open(ADAS_SERVER_NAME, 0);
    g_next_connect = t + NS_PER_SEC;
    if (g_coid != -1) g_sent_health = -1;
}

static void fusion_drop(void)
{
    if (g_coid != -1) name_close(g_coid);
    g_coid = -1;
}

static void fusion_report(const us_status_t *st, bool have_sample, double cpu_us)
{
    if (g_coid == -1) return;
    int sh = to_sh(st->health);
    if (sh != g_sent_health) {                      /* health change -> pulse */
        if (MsgSendPulse(g_coid, -1, PC_SENSOR_HEALTH, (SENS_RANGE << 8) | sh) == -1) { fusion_drop(); return; }
        g_sent_health = sh;
    }
    range_sample_msg_t smp;
    memset(&smp, 0, sizeof smp);
    smp.t_ns = st->t_meas;
    smp.dist_m = (float)st->dist_m;
    smp.raw_m = (float)st->raw_m;
    smp.vel_ms = (float)st->vel_ms;
    smp.echo_us = (float)st->echo_us;

    adas_hdr_t hdr;
    memset(&hdr, 0, sizeof hdr);
    hdr.type = MSG_SENSOR_DATA;
    hdr.sensor = SENS_RANGE;
    hdr.count = have_sample ? 1 : 0;                /* 0 = "alive, no new range" */
    hdr.health = (uint16_t)sh;
    hdr.seq = g_seq++;
    hdr.sample_size = sizeof smp;
    hdr.rate_hz = (float)st->rate_hz;
    hdr.extra = (float)cpu_us;
    iov_t iov[2];
    SETIOV(&iov[0], &hdr, sizeof hdr);
    SETIOV(&iov[1], &smp, sizeof smp);
    hdr.t_send_ns = now_ns();
    if (MsgSendv(g_coid, iov, have_sample ? 2 : 1, NULL, 0) == -1) fusion_drop();
    else g_msgs_sent++;
}

static double thread_cpu_us(void)
{
#ifdef CLOCK_THREAD_CPUTIME_ID
    struct timespec ts;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
#endif
    return 0;
}

/* ============================================================ alpha-beta tracker */
static bool ab_update(ab_t *f, double z, int64_t t, uint64_t *reinits)
{
    if (!f->init) {
        /* start a track only when two consecutive readings agree (rejects stray echoes) */
        if (f->pend && fabs(z - f->pz) < 0.10 + 0.05 * z) {
            double dt0 = (double)(t - f->pt) / 1e9;
            f->d = z;
            f->v = dt0 > 0 ? (z - f->pz) / dt0 : 0;
            f->t = t; f->init = true; f->rejects = 0; f->pend = false;
        } else {
            f->pend = true; f->pz = z; f->pt = t;
        }
        return true;
    }
    double dt = (double)(t - f->t) / 1e9;
    if (dt <= 0) dt = 1e-3;
    double pred = f->d + f->v * dt;
    double r = z - pred;
    double gate = 0.15 + 2.0 * fabs(f->v) * dt + 0.05 * pred;

    if (fabs(r) > gate) {
        if (++f->rejects >= REINIT_AFTER) {        /* consistent new value: accept it */
            f->d = z; f->v = 0; f->t = t; f->rejects = 0;
            (*reinits)++;
            return true;
        }
        return false;
    }
    f->d = pred + AB_ALPHA * r;
    f->v += AB_BETA * r / dt;
    if (f->v > 10) f->v = 10;
    if (f->v < -10) f->v = -10;
    f->t = t;
    f->rejects = 0;
    return true;
}

/* ============================================================ measurement thread */
static void log_push(const us_status_t *s)
{
    pthread_mutex_lock(&g_log_lock);
    if (g_log_w - g_log_r >= LOG_N) g_log_lost++;
    else g_log[g_log_w++ % LOG_N] = *s;
    pthread_mutex_unlock(&g_log_lock);
}

static void *us_thread(void *arg)
{
    (void)arg;
    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (chid == -1 || coid == -1) { perror("channel"); g_run = 0; return NULL; }
    g_chid = chid;
    irq_setup();                                    /* GPIO edge interrupt -> pulse */
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, PRIO_US, PC_SENSOR_TICK, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("timer_create"); g_run = 0; return NULL; }
    int64_t period = (int64_t)(NS_PER_SEC / g_rate_hz);
    struct itimerspec its;
    ns_to_ts(now_ns() + period, &its.it_value);
    ns_to_ts(period, &its.it_interval);
    timer_settime(tid, TIMER_ABSTIME, &its, NULL);

    double c = 331.3 + 0.606 * g_temp_c;
    ab_t ab = { 0 };
    us_status_t st = { 0 };
    st.c_sound = c;
    st.health = H_NO_TARGET;
    st.decision = UDEC_NO_DATA;
    st.ttc = INFINITY;
    int consec_fail = 0, consec_none = 0, consec_stuck = 0;
    int64_t t_first = 0;

    while (g_run) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (p.code == PC_GPIO_IRQ) {                /* stray edge between pings */
            if (g_iid != -1) InterruptUnmask(g_irq, g_iid);
            continue;
        }
        if (p.code != PC_SENSOR_TICK) continue;

        int64_t t_start = now_ns();
        if (!t_first) t_first = t_start;
        double echo_us = 0;
        int64_t t_ref = 0;
        double cpu0 = thread_cpu_us();
        mres_t r = g_iid != -1 ? ping_irq(&echo_us, &t_ref) : ping(&echo_us, &t_ref);
        st.busy_us = (double)(now_ns() - t_start) / 1e3;
        double cpu_us = thread_cpu_us() - cpu0;
        st.pings++;

        if (r != M_STUCK_HIGH) consec_stuck = 0;
        switch (r) {
        case M_OK: {
            consec_fail = 0;
            double d = (echo_us * 1e-6) * c / 2.0;
            st.echo_us = echo_us;
            st.raw_m = d;
            if (d < RANGE_MIN_M || d > RANGE_MAX_M) { st.no_target++; consec_none++; break; }
            consec_none = 0;
            st.valid++;
            st.t_meas = t_ref;
            if (!ab_update(&ab, d, t_ref, &st.reinits)) st.rejected++;
            break;
        }
        case M_NO_ECHO:    st.no_target++; consec_none++; consec_fail = 0; break;
        case M_NO_START:   st.no_start++;  consec_fail++; break;
        case M_STUCK_HIGH:                 /* tail of a lost echo: skipped, not a fault */
            st.stuck++; consec_none++;
            if (++consec_stuck >= STUCK_FAULT_PINGS) consec_fail = FAULT_AFTER;
            break;
        }

        /* health */
        if (consec_fail >= FAULT_AFTER) st.health = H_FAULT;
        else if (consec_fail >= 2) st.health = H_DEGRADED;
        else if (consec_fail == 1) { /* single missed ping: keep previous state */ }
        else if (consec_none >= 3) { st.health = H_NO_TARGET; ab.init = false; ab.pend = false; }
        else if (st.pings > 20 && st.rejected * 5 > st.valid) st.health = H_DEGRADED;
        else st.health = H_OK;

        /* distance, velocity, TTC, decision */
        if (st.health == H_OK && ab.init) {
            st.dist_m = ab.d;
            st.vel_ms = ab.v;
            st.ttc = ab.v < -0.05 ? ab.d / -ab.v : INFINITY;
            if (st.ttc < TTC_EMERGENCY || ab.d < D_EMERGENCY_M) st.decision = UDEC_EMERGENCY;
            else if (st.ttc < TTC_CRITICAL) st.decision = UDEC_CRITICAL;
            else if (st.ttc < TTC_WARNING)  st.decision = UDEC_WARNING;
            else                            st.decision = UDEC_SAFE;
        } else if (st.health == H_NO_TARGET) {
            st.ttc = INFINITY;
            st.decision = UDEC_SAFE;                  /* nothing in front */
        } else {
            st.ttc = INFINITY;
            st.decision = UDEC_NO_DATA;
        }
        double el = (double)(t_start - t_first) / 1e9;
        st.rate_hz = el > 0 ? (double)(st.pings - 1) / el : 0;

        /* ---- report to fusion_server: IOV message every ping, pulse on health change */
        fusion_link(t_start);
        bool fresh = r == M_OK && st.health == H_OK && ab.init && st.t_meas == t_ref;
        fusion_report(&st, fresh, cpu_us);
        st.cpu_us = cpu_us;

        pthread_mutex_lock(&g_st_lock);
        g_st = st;
        pthread_mutex_unlock(&g_st_lock);
        if (r == M_OK) log_push(&st);
    }

    irq_teardown();
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
        us_status_t s;
        pthread_mutex_lock(&g_log_lock);
        bool have = g_log_r < g_log_w;
        if (have) s = g_log[g_log_r++ % LOG_N];
        pthread_mutex_unlock(&g_log_lock);
        if (!have) break;
        if (f)
            fprintf(f, "%lld,%.1f,%.4f,%.4f,%.4f,%.3f,%s,%s\n",
                    (long long)s.t_meas, s.echo_us, s.raw_m, s.dist_m, s.vel_ms,
                    isinf(s.ttc) ? -1.0 : s.ttc, UDEC_NAME[s.decision], HEALTH_NAME[s.health]);
    }
    if (f) fflush(f);
}

static void bar(char *out, double d)
{
    const int W = 20;
    int n = (int)(d / 2.0 * W + 0.5);              /* 0..2 m */
    if (n < 0) n = 0;
    if (n > W) n = W;
    out[0] = '|';
    for (int i = 0; i < W; i++) out[1 + i] = i < n ? '#' : '.';
    out[W + 1] = '|';
    out[W + 2] = 0;
}

int main(int argc, char **argv)
{
    int run_s = 0, opt;
    const char *log_path = NULL;

    while ((opt = getopt(argc, argv, "g:e:T:r:t:o:I:")) != -1) {
        switch (opt) {
        case 'g': g_trig = atoi(optarg); break;
        case 'e': g_echo = atoi(optarg); break;
        case 'T': g_temp_c = atof(optarg); break;
        case 'r': g_rate_hz = atof(optarg); break;
        case 't': run_s = atoi(optarg); break;
        case 'o': log_path = optarg; break;
        case 'I': g_irq = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-g trig_gpio] [-e echo_gpio] [-T temp_C] [-r rate_hz] "
                            "[-t sec] [-o log.csv] [-I irq|0]\n", argv[0]);
            return 1;
        }
    }
    if (g_trig < 2 || g_trig > 27 || g_echo < 2 || g_echo > 27 || g_trig == g_echo) {
        fprintf(stderr, "invalid GPIO numbers\n");
        return 1;
    }
    if (g_rate_hz > 16.0) {
        printf("note: rate limited to 16 Hz (HC-SR04 needs >= 60 ms between pings)\n");
        g_rate_hz = 16.0;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    g_cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    void *m = mmap_device_memory(NULL, GPIO_MAP_LEN, PROT_READ | PROT_WRITE | PROT_NOCACHE, 0,
                                 GPIO_PHYS_BASE);
    if (m == MAP_FAILED) {
        fprintf(stderr, "mmap GPIO registers failed: %s (run as root: su)\n", strerror(errno));
        return 1;
    }
    g_gpio = (volatile uint32_t *)m;

    gpio_mode(g_trig, true);
    gpio_low(g_trig);
    gpio_mode(g_echo, false);
    gpio_pull(g_echo, 2);                           /* weak pull-down on ECHO */
    sleep_ms(50);

    printf("HC-SR04 node | echo timing: %s | fusion: name_open(\"%s\")\n",
           g_irq > 0 ? "GPIO interrupt (falls back to polling if none arrive)" : "polling", ADAS_SERVER_NAME);
    printf("HC-SR04 on QNX | TRIG GPIO%d  ECHO GPIO%d | %.0f Hz | T=%.1f C -> c=%.1f m/s | "
           "cycle counter %.1f MHz\n\n",
           g_trig, g_echo, g_rate_hz, g_temp_c, 331.3 + 0.606 * g_temp_c, g_cps / 1e6);

    FILE *logf = NULL;
    if (log_path) {
        logf = fopen(log_path, "w");
        if (!logf) perror("log");
        else fprintf(logf, "t_ns,echo_us,raw_m,dist_m,vel_ms,ttc_s,decision,health\n");
    }

    pthread_t th;
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&a, SCHED_FIFO);
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = PRIO_US;
    pthread_attr_setschedparam(&a, &sp);
    if (pthread_create(&th, &a, us_thread, NULL) != 0) {
        fprintf(stderr, "warning: could not use priority %d, using default\n", PRIO_US);
        pthread_create(&th, NULL, us_thread, NULL);
    }
    pthread_setname_np(th, "us_ping");
    pthread_setname_np(pthread_self(), "us_display");

    int64_t t_end = run_s > 0 ? now_ns() + (int64_t)run_s * NS_PER_SEC : 0;
    while (g_run) {
        sleep_ms(200);
        log_drain(logf);
        us_status_t s;
        pthread_mutex_lock(&g_st_lock);
        s = g_st;
        pthread_mutex_unlock(&g_st_lock);

        char b[32], ttc[16];
        bar(b, s.health == H_OK ? s.dist_m : 2.0);
        if (isinf(s.ttc)) snprintf(ttc, sizeof ttc, "  -- ");
        else snprintf(ttc, sizeof ttc, "%5.2fs", s.ttc);

        if (s.health == H_OK && s.decision == UDEC_NO_DATA)
            printf("US %-9s %s   acquiring track (raw %5.3f m, waiting for a 2nd matching echo)\n",
                   HEALTH_NAME[s.health], b, s.raw_m);
        else if (s.health == H_OK)
            printf("US %-9s %s %5.3f m (raw %5.3f) | v %+5.2f m/s | TTC %s %-9s | echo %6.0f us | "
                   "%4.1f Hz %s wait %5.0f us cpu %5.0f us | rej %llu | fusion %s\n",
                   HEALTH_NAME[s.health], b, s.dist_m, s.raw_m, s.vel_ms, ttc, UDEC_NAME[s.decision],
                   s.echo_us, s.rate_hz, g_iid != -1 ? "IRQ " : "POLL", s.busy_us, s.cpu_us,
                   (unsigned long long)s.rejected, g_coid != -1 ? "linked" : "not running");
        else
            printf("US %-9s %s   ---      (nothing in 0.02-4 m or no echo) | %-9s | %4.1f Hz | "
                   "nostart %llu skipped %llu%s\n",
                   HEALTH_NAME[s.health], b, UDEC_NAME[s.decision], s.rate_hz,
                   (unsigned long long)s.no_start, (unsigned long long)s.stuck,
                   s.health == H_FAULT ? "  <- check TRIG/ECHO wiring and divider" : "");
        fflush(stdout);
        if (t_end && now_ns() >= t_end) g_run = 0;
    }

    pthread_join(th, NULL);
    gpio_low(g_trig);
    gpio_mode(g_trig, false);
    log_drain(logf);
    if (logf) fclose(logf);

    us_status_t s = g_st;
    printf("\n===== HC-SR04 summary =====\n");
    printf("pings              : %llu  (%.2f Hz)\n", (unsigned long long)s.pings, s.rate_hz);
    printf("valid ranges       : %llu\n", (unsigned long long)s.valid);
    printf("no target / no echo: %llu\n", (unsigned long long)s.no_target);
    printf("echo never started : %llu   pings skipped (lost-echo tail): %llu\n",
           (unsigned long long)s.no_start, (unsigned long long)s.stuck);
    printf("outliers rejected  : %llu   tracker re-inits: %llu\n",
           (unsigned long long)s.rejected, (unsigned long long)s.reinits);
    printf("echo timing mode   : %s  (GPIO IRQ events %llu, fallbacks %llu)\n",
           g_irq_was ? "interrupt" : "polling", (unsigned long long)g_irq_events,
           (unsigned long long)g_irq_fallbacks);
    printf("messages to fusion : %llu\n", (unsigned long long)g_msgs_sent);
    if (g_coid != -1) name_close(g_coid);
    if (g_log_lost) printf("log records lost   : %llu\n", (unsigned long long)g_log_lost);
    return 0;
}
