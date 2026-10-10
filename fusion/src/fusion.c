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

/* config constants */
#define NS_PER_SEC         1000000000LL
#define NS_PER_MS          1000000LL
#define FUSION_PERIOD_NS   (20 * NS_PER_MS)
#define FUSION_DEADLINE_NS (20 * NS_PER_MS)
#define PRIO_TICK          30
#define PRIO_RX            20

#define RANGE_MAX_AGE_NS   (200 * NS_PER_MS)
#define CAM_MAX_AGE_NS     (200 * NS_PER_MS)
#define CAM_MIN_CONF       0.4
#define CAM_REL_SIGMA      0.06
#define CAM_CALIB_MIN      10
#define IMU_MAX_AGE_NS     (100 * NS_PER_MS)
#define SENSOR_STALE_NS    (500 * NS_PER_MS)
#define COAST_MAX_NS       (1000 * NS_PER_MS)

#define R_RANGE            (0.02 * 0.02)
#define Q_ACCEL            3.0
#define GATE_SIGMA         4.0
#define REINIT_AFTER       3
#define ACC_DEADBAND       0.15
#define GRAVITY_TAU_S      2.0

#define TTC_WARNING        4.0
#define DEC_HOLD_CYCLES    10
#define TTC_CRITICAL       2.0
#define TTC_EMERGENCY      1.0
#define D_EMERGENCY_M      0.15
#define D_CRITICAL_M       0.30
#define D_WARNING_M        0.50

_Static_assert(sizeof(struct _pulse) <= sizeof(adas_hdr_t), "pulse must fit");

/* shared state */
typedef struct {
    uint8_t  health[SENS_COUNT];
    int64_t  last_rx_ns[SENS_COUNT];
    uint64_t msgs[SENS_COUNT], samples[SENS_COUNT], lost[SENS_COUNT];
    uint32_t last_seq[SENS_COUNT];
    bool     seq_valid[SENS_COUNT];
    float    rate_hz[SENS_COUNT], latency_us[SENS_COUNT];
    int      scoid[SENS_COUNT];

    imu_sample_msg_t imu_last;
    float    imu_temp;
    double   g_lp[3];
    bool     g_init;
    int64_t  g_last_t;
    double   lin_fwd;
    double   lin_norm;
    int64_t  imu_t;

    range_sample_msg_t rng;
    bool     rng_new;
    float    range_cpu_us;

    camera_sample_msg_t cam;
    bool     cam_new;

    bool     dash_ok;
    int      dash_rcvid, dash_scoid;
    struct sigevent dash_ev;

    uint64_t pulses_rx, msgs_rx, disconnects, dash_events;
} fstate_t;

static fstate_t        g;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static adas_shm_t     *g_shm;
static volatile sig_atomic_t g_run = 1;
static int  g_axis = 0, g_axis_sign = 1;
static name_attach_t *g_attach;

/* log ring buffer */
typedef struct {
    int64_t t; float raw, d, v, ttc, lin, exec_us, resp_us; int dec, miss, rh, ih;
} logrec_t;
#define LOG_N 2048
static logrec_t        g_logr[LOG_N];
static uint64_t        g_log_w, g_log_r;
static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;

/* utils */
static int64_t now_ns(void) { ... }
static void ns_to_ts(int64_t ns, struct timespec *ts) { ... }
static void set_prio(int prio) { ... }

/* Kalman filter */
typedef struct { double x[2], P[2][2]; bool init; int rejects; } kf_t;
static void kf_init(kf_t *k, double d, double v) { ... }
static void kf_predict(kf_t *k, double dt, double u) { ... }
static bool kf_update(kf_t *k, double z, double R) { ... }

/* sensor handlers */
static void handle_camera(const camera_sample_msg_t *s, int n) { ... }
static int sensor_by_scoid(int scoid) { ... }
static void handle_imu(const adas_hdr_t *h, const imu_sample_msg_t *s, int n) { ... }
static void handle_range(const adas_hdr_t *h, const range_sample_msg_t *s, int n) { ... }

/* receive thread */
static void *rx_thread(void *arg) { ... }

/* tick thread */
static void log_push(const logrec_t *r) { ... }
static void *tick_thread(void *arg) { ... }

/* CPU hog thread */
static void *hog_thread(void *arg) { ... }
static void start_thread(pthread_t *t, void *(*fn)(void *), int prio, const char *name) { ... }

/* signal handler */
static void on_signal(int sig) { (void)sig; g_run = 0; }

/* main */
int main(int argc, char **argv)
{
    /* parse args */
    /* setup shm */
    /* name_attach server */
    /* start threads */
    /* main loop: drain log, print status */
    /* cleanup */
}
