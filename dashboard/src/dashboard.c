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

/* signal handler */
static void on_signal(int sig) { (void)sig; g_run = 0; }

/* monotonic timestamp (ns) */
static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* map shared memory */
static const adas_shm_t *map_shm(void)
{
    int fd = shm_open(ADAS_SHM_NAME, O_RDONLY, 0);
    if (fd == -1) return NULL;
    void *p = mmap(NULL, sizeof(adas_shm_t), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : (const adas_shm_t *)p;
}

/* register dashboard pulse with fusion */
static int register_with_fusion(int self_coid)
{
    int fcoid = name_open(ADAS_SERVER_NAME, 0);
    if (fcoid == -1) return -1;
    dash_register_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_DASH_REGISTER;
    SIGEV_PULSE_INIT(&m.event, self_coid, 10, PC_DASH_UPDATE, 0);
    if (MsgRegisterEvent(&m.event, fcoid) == -1)
        perror("MsgRegisterEvent");
    if (MsgSend(fcoid, &m, sizeof m, NULL, 0) == -1) {
        name_close(fcoid);
        return -1;
    }
    return fcoid;
}

/* colour codes */
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

/* ASCII bar graph */
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

/* render dashboard frame */
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

    if (!g_plain) printf("\033[H\033[2J");
    /* formatted dashboard output */
    printf("======================= QNX ADAS SENSOR FUSION  -  live dashboard  (fusion pid %d) ======================\n",
           s->fusion_pid);
    ...
    fflush(stdout);
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "p")) != -1) {
        if (opt == 'p') g_plain = true;
        else { fprintf(stderr, "usage: %s [-p]\n", argv[0]); return 1; }
    }

    static char outbuf[16384];
    setvbuf(stdout, outbuf, _IOFBF, sizeof outbuf);
    if (!g_plain) printf("\033[2J");
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
        /* reconnect if needed */
        if ((!shm || fcoid == -1) && t - last_try > NS_PER_SEC) {
            last_try = t;
            if (shm) { munmap((void *)shm, sizeof *shm); shm = NULL; }
            if (fcoid != -1) { name_close(fcoid); fcoid = -1; }
            shm = map_shm();
            if (shm) fcoid = register_with_fusion(self);
        }

        /* wait for pulse or timeout */
        struct _pulse p;
        uint64_t to = 500000000ULL;
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) != -1 && p.code == PC_DASH_UPDATE) wakeups++;

        if (!shm) continue;
        if (shm_read(shm, &snap) != 0) continue;
        if (snap.update_count != last_count) { last_count = snap.update_count; last_change = now_ns(); }
        bool live = now_ns() - last_change < 2 * NS_PER_SEC;
        render(&snap, wakeups, live);
        if (!live && fcoid != -1) { name_close(fcoid); fcoid = -1; }
    }
    if (fcoid != -1) name_close(fcoid);
    if (!g_plain) printf("\033[0m");
    return 0;
}
