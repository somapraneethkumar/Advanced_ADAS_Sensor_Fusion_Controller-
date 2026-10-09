/*
 * dashboard.c  -  QNX-eHACK 2026: live ADAS dashboard process
 *
 * QNX mechanisms used here:
 *   - Shared memory: maps fusion_server's /adas_state READ-ONLY and copies a consistent
 *     snapshot with the seqlock (no locks shared with the real-time writer).
 *   - Event notification: creates a pulse event, registers it with MsgRegisterEvent(), and
 *     hands it to fusion_server (MSG_DASH_REGISTER). Fusion then wakes this process with
 *     MsgDeliverEvent() whenever the decision changes (and every 100 ms), so the dashboard
 *     sleeps instead of polling. A 500 ms receive timeout (TimerTimeout) covers silence.
 *   - Shows all three sensors, including the camera's detection, its own TTC and the
 *     distance it gives once fusion has cross-calibrated it against the ultrasonic.
 *
 * Usage: dashboard [-p]     -p = plain output (no screen clearing / colours, for Momentics)
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
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/siginfo.h>
#include "adas_msg.h"

#define NS_PER_SEC 1000000000LL

static volatile sig_atomic_t g_run = 1;
static bool g_plain;

static void on_signal(int sig) { (void)sig; g_run = 0; }

static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static const adas_shm_t *map_shm(void)
{
    int fd = shm_open(ADAS_SHM_NAME, O_RDONLY, 0);
    if (fd == -1) return NULL;
    void *p = mmap(NULL, sizeof(adas_shm_t), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : (const adas_shm_t *)p;
}

/* connect to fusion and give it our pulse event */
static int register_with_fusion(int self_coid)
{
    int fcoid = name_open(ADAS_SERVER_NAME, 0);
    if (fcoid == -1) return -1;
    dash_register_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_DASH_REGISTER;
    SIGEV_PULSE_INIT(&m.event, self_coid, 10, PC_DASH_UPDATE, 0);
    if (MsgRegisterEvent(&m.event, fcoid) == -1)      /* secure event registration */
        perror("MsgRegisterEvent (continuing)");
    if (MsgSend(fcoid, &m, sizeof m, NULL, 0) == -1) {
        name_close(fcoid);
        return -1;
    }
    return fcoid;
}

static const char *col(int dec)
{
    if (g_plain) return "";
    switch (dec) {
    case DEC_SAFE:      return "\033[1;32m";
    case DEC_WARNING:   return "\033[1;33m";
    case DEC_CRITICAL:  return "\033[1;31m";
    case DEC_EMERGENCY: return "\033[1;37;41m";
    default:            return "\033[1;36m";
    }
}
static const char *rst(void) { return g_plain ? "" : "\033[0m"; }
static const char *hcol(int h)
{
    if (g_plain) return "";
    return h == SH_OK ? "\033[32m" : h == SH_NO_TARGET ? "\033[36m" : h == SH_DEGRADED ? "\033[33m" : "\033[31m";
}

static void bar(char *out, float d)
{
    const int W = 30;
    int n = isnan(d) ? W : (int)(d / 2.0f * W + 0.5f);
    if (n < 0) n = 0;
    if (n > W) n = W;
    out[0] = '[';
    for (int i = 0; i < W; i++) out[1 + i] = i < n ? '#' : '.';
    out[W + 1] = ']';
    out[W + 2] = 0;
}

static void render(const adas_shm_t *s, uint64_t wakeups, bool live)
{
    const shm_sensor_t *R = &s->sens[SENS_RANGE], *I = &s->sens[SENS_IMU], *C = &s->sens[SENS_CAMERA];
    char b[40], ttc[16], cttc[16], cdist[24], ctgt[40];
    bool csee = C->health == SH_OK && s->cam_scale_px > 0;
    if (!csee || isinf(s->cam_ttc_s) || isnan(s->cam_ttc_s)) snprintf(cttc, sizeof cttc, " -- ");
    else snprintf(cttc, sizeof cttc, "%4.1fs", s->cam_ttc_s);
    if (!s->cam_calibrated) snprintf(cdist, sizeof cdist, "learning %d/10", s->cam_calib_samples);
    else if (isnan(s->cam_dist_m)) snprintf(cdist, sizeof cdist, "  --   ");
    else snprintf(cdist, sizeof cdist, "%5.2f m", s->cam_dist_m);
    if (csee) snprintf(ctgt, sizeof ctgt, "target (%.2f,%.2f) conf %.2f", s->cam_cx, s->cam_cy, s->cam_conf);
    else snprintf(ctgt, sizeof ctgt, "no target in view");
    bar(b, s->fused_d);
    if (isinf(s->ttc_s) || isnan(s->ttc_s)) snprintf(ttc, sizeof ttc, "  --  ");
    else snprintf(ttc, sizeof ttc, "%5.2f s", s->ttc_s);

    if (!g_plain) printf("\033[H\033[2J");    /* clear + redraw travel in ONE write (full buffering) */
    printf("======================= QNX ADAS SENSOR FUSION  -  live dashboard  (fusion pid %d) ======================\n",
           s->fusion_pid);
    printf(" ULTRASONIC  HC-SR04 / GPIO       | IMU  MPU6050 / I2C FIFO          | CAMERA  USB / libcamapi\n");
    printf(" %s%-10s%s %6.1f Hz              | %s%-10s%s %7.2f Hz              | %s%-10s%s %5.1f fps %ux%u\n",
           hcol(R->health), HEALTH_STR[R->health], rst(), R->rate_hz,
           hcol(I->health), HEALTH_STR[I->health], rst(), I->rate_hz,
           hcol(C->health), HEALTH_STR[C->health], rst(), C->rate_hz, s->cam_w, s->cam_h);
    printf(" raw %6.3f m  trk %6.3f m      | a %+6.2f %+6.2f %+6.2f m/s2    | %s\n",
           s->range_raw_m, s->range_m, s->imu_a[0], s->imu_a[1], s->imu_a[2], ctgt);
    printf(" v %+5.2f m/s  echo %6.0f us     | roll %+6.1f  pitch %+6.1f deg   | size %6.1f px  cam TTC %s\n",
           s->range_vel_ms, s->range_echo_us, s->imu_roll, s->imu_pitch,
           csee ? s->cam_scale_px : 0.0f, cttc);
    printf(" CPU per ping %6.0f us           | fwd lin.acc %+5.2f m/s2  %4.1f C  | cam distance %s\n",
           s->range_cpu_us, s->imu_lin_acc, s->imu_temp, cdist);
    printf(" IPC %5.0f us  msgs %-7llu lost %-3llu| IPC %5.0f us  msgs %-7llu lost %-3llu| IPC %5.0f us  proc %5.0f us age %4.0f ms\n",
           R->latency_us, (unsigned long long)R->msgs, (unsigned long long)R->lost_msgs,
           I->latency_us, (unsigned long long)I->msgs, (unsigned long long)I->lost_msgs,
           C->latency_us, s->cam_proc_us, s->cam_age_ms);
    printf("--------------------------------------------------------------------------------------------------------------\n");
    printf(" FUSED STATE (Kalman)              | QNX REAL-TIME\n");
    if (isnan(s->fused_d))
        printf(" distance   ---                    | fusion 50 Hz, deadline %.0f ms\n", s->deadline_us / 1000);
    else
        printf(" distance %6.3f m  (+-%4.1f cm)    | fusion 50 Hz, deadline %.0f ms\n",
               s->fused_d, sqrtf(s->d_var) * 100.0f, s->deadline_us / 1000);
    printf(" closing  %+6.2f m/s               | exec %5.0f us  resp %6.0f us\n",
           isnan(s->fused_v) ? 0.0f : s->fused_v, s->exec_us, s->resp_us);
    printf(" ego motion (IMU): %-3s             | worst resp %6.0f us  ->  %s%s%s\n",
           s->ego_moving ? "YES" : "no", s->worst_resp_us,
           g_plain ? "" : (s->misses ? "\033[31m" : "\033[32m"), s->misses ? "MISSES" : "PASS", rst());
    printf(" camera in track: %-3s  K %7.1f    | cycles %-9llu misses %llu\n",
           s->cam_used ? "YES" : "no", s->cam_K, (unsigned long long)s->cycles, (unsigned long long)s->misses);
    printf(" IPC: msgs %-8llu pulses %-6llu  | disconnects %llu  dashboard wake-ups %llu\n",
           (unsigned long long)s->msgs_rx, (unsigned long long)s->pulses_rx,
           (unsigned long long)s->disconnects, (unsigned long long)wakeups);
    printf("--------------------------------------------------------------------------------------------------------------\n");
    printf(" %s  TTC %s   ADAS DECISION: %s  %-9s  %s\n", b, ttc, col(s->decision),
           DECISION_STR[s->decision], rst());
    if (!live) printf(" (fusion_server not running - waiting...)\n");
    printf("==============================================================================================================\n");
    fflush(stdout);                            /* whole frame leaves in one write */
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "p")) != -1) {
        if (opt == 'p') g_plain = true;
        else { fprintf(stderr, "usage: %s [-p]\n", argv[0]); return 1; }
    }
    /* full buffering: each frame is written to the terminal in one go (no half-drawn frames) */
    static char outbuf[16384];
    setvbuf(stdout, outbuf, _IOFBF, sizeof outbuf);
    if (!g_plain) printf("\033[2J");          /* clear once at start */
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int chid = ChannelCreate(0);
    int self = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (chid == -1 || self == -1) { perror("channel"); return 1; }

    const adas_shm_t *shm = NULL;
    int fcoid = -1;
    uint64_t wakeups = 0, last_count = 0;
    int64_t last_change = now_ns(), last_try = 0;
    adas_shm_t snap;
    memset(&snap, 0, sizeof snap);

    printf("dashboard: waiting for fusion_server ('%s', %s)...\n", ADAS_SERVER_NAME, ADAS_SHM_NAME);
    while (g_run) {
        int64_t t = now_ns();
        if ((!shm || fcoid == -1) && t - last_try > NS_PER_SEC) {   /* (re)connect */
            last_try = t;
            if (shm) { munmap((void *)shm, sizeof *shm); shm = NULL; }
            if (fcoid != -1) { name_close(fcoid); fcoid = -1; }
            shm = map_shm();
            if (shm) fcoid = register_with_fusion(self);
        }

        /* sleep until fusion delivers our pulse (or 500 ms) */
        struct _pulse p;
        uint64_t to = 500000000ULL;
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) != -1 && p.code == PC_DASH_UPDATE) wakeups++;

        if (!shm) continue;
        if (shm_read(shm, &snap) != 0) continue;
        if (snap.update_count != last_count) { last_count = snap.update_count; last_change = now_ns(); }
        bool live = now_ns() - last_change < 2 * NS_PER_SEC;
        render(&snap, wakeups, live);
        if (!live && fcoid != -1) { name_close(fcoid); fcoid = -1; }   /* fusion restarted? */
    }
    if (fcoid != -1) name_close(fcoid);
    if (!g_plain) printf("\033[0m");
    return 0;
}
