/*
 * camera_node.c  -  QNX-eHACK 2026: USB camera process of the ADAS fusion system
 *
 * Uses the QNX camera library (libcamapi, QNX Sensor Framework). On the Raspberry Pi 4
 * QSTI image the USB camera is camera unit 4 (see /etc/config/sensor/sensor_rpi4.conf).
 *
 * Per frame (viewfinder callback):
 *   1. COLOUR-TARGET DETECTION   the "vehicle ahead" is a coloured object (red by default).
 *                                Works directly on the camera's native format (NV12, the
 *                                4:2:2 YCbCr variants, or RGB8888) - no colour conversion
 *                                of the whole frame, every 2nd row / 2nd pixel sampled.
 *   2. IMAGE-EXPANSION TTC       an approaching object grows in the image. With s = sqrt(area),
 *                                TTC = s / (ds/dt) = 1 / (d ln s / dt). This needs no distance
 *                                and no calibration (the classic "tau" of vision science).
 *   3. TIMESTAMPS                the library's frame timestamp is CLOCK_MONOTONIC - the same
 *                                clock as the IMU and ultrasonic nodes.
 *   4. IPC                       every frame result goes to fusion_server as one IOV message
 *                                (header + camera_sample_msg_t); health changes as pulses.
 *                                Fusion cross-calibrates camera distance against the ultrasonic.
 *
 * Usage: camera_node [-u unit] [-c red|green|blue] [-w] [-t seconds] [-a min_area_percent]
 *   -u  camera unit (default 4 = USB camera on Pi 4)
 *   -c  target colour (default red)
 *   -w  also show the camera's live video window on the Pi display
 *   -a  minimum target size, percent of the image (default 0.3)
 *
 * Build: Momentics QNX project, aarch64le, LIBS += -lcamapi -lm
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
#include <camera/camera_api.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/uio.h>
#include "adas_msg.h"

#define NS_PER_SEC 1000000000LL

typedef enum { COL_RED, COL_GREEN, COL_BLUE } colour_t;
static const char *COL_NAME[] = { "red", "green", "blue" };

/* ------------------------------------------------------------ settings */
static int      g_unit = 4;
static colour_t g_col = COL_RED;
static bool     g_window;
static double   g_min_area = 0.003;             /* fraction of the image */

/* ------------------------------------------------------------ state */
static volatile sig_atomic_t g_run = 1;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    camera_sample_msg_t last;
    uint64_t frames, detections, unsupported;
    int      frametype;
    double   fps;
    int64_t  last_frame_ns;
    int      health;                             /* SH_* */
    int      miss_run;
    int      det_run;                            /* consecutive frames with target */
    double   cap_age_ms;                         /* capture -> processed */
} cam_status_t;
static cam_status_t g_st;

/* expansion tracker on ln(scale) */
typedef struct { bool init; double ls, rate; int64_t t; } exp_t;
static exp_t g_exp;

/* fusion link */
static int      g_coid = -1;
static uint32_t g_seq;
static int64_t  g_next_connect;
static int      g_sent_health = -1;
static uint64_t g_msgs_sent;

/* ============================================================ utils */
static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* ============================================================ colour test */
static inline bool is_target(int y, int cb, int cr)
{
    if (y < 25 || y > 245) return false;         /* too dark / blown out */
    switch (g_col) {
    case COL_RED:   return cr > 160 && cb < 140 && cr - cb > 40;
    case COL_GREEN: return cr < 115 && cb < 120;
    case COL_BLUE:  return cb > 155 && cr < 130;
    }
    return false;
}

typedef struct {
    uint64_t n;                                  /* matching samples */
    double   sx, sy;
    int      minx, maxx, miny, maxy;
    uint64_t samples;
} blob_t;

static inline void blob_add(blob_t *b, int x, int y)
{
    b->n++;
    b->sx += x; b->sy += y;
    if (x < b->minx) b->minx = x;
    if (x > b->maxx) b->maxx = x;
    if (y < b->miny) b->miny = y;
    if (y > b->maxy) b->maxy = y;
}

/*
 * Packed 4:2:2 formats: two pixels share one Cb and one Cr. Offsets of Y0, Cb, Cr in each
 * 4-byte macro-pixel for each layout.
 */
static void scan_422(const uint8_t *buf, int w, int h, int stride, int oy, int ocb, int ocr, blob_t *b)
{
    for (int y = 0; y < h; y += 2) {
        const uint8_t *row = buf + (size_t)y * stride;
        for (int x = 0; x + 1 < w; x += 2) {
            const uint8_t *m = row + x * 2;
            b->samples++;
            if (is_target(m[oy], m[ocb], m[ocr])) blob_add(b, x, y);
        }
    }
}

static void scan_nv12(const uint8_t *buf, const camera_frame_nv12_t *d, blob_t *b)
{
    const uint8_t *uv = buf + d->uv_offset;
    for (int y = 0; y < (int)d->height; y += 2) {
        const uint8_t *yr = buf + (size_t)y * d->stride;
        const uint8_t *cr = uv + (size_t)(y / 2) * d->uv_stride;
        for (int x = 0; x + 1 < (int)d->width; x += 2) {
            b->samples++;
            if (is_target(yr[x], cr[x], cr[x + 1])) blob_add(b, x, y);
        }
    }
}

static void scan_rgb8888(const uint8_t *buf, int w, int h, int stride, blob_t *b)
{
    for (int y = 0; y < h; y += 2) {
        const uint8_t *row = buf + (size_t)y * stride;
        for (int x = 0; x < w; x += 2) {
            const uint8_t *p = row + x * 4;      /* little-endian ARGB: B G R A */
            int B = p[0], G = p[1], R = p[2];
            int Y  = ( 77 * R + 150 * G +  29 * B) >> 8;
            int Cb = ((-43 * R -  85 * G + 128 * B) >> 8) + 128;
            int Cr = ((128 * R - 107 * G -  21 * B) >> 8) + 128;
            b->samples++;
            if (is_target(Y, Cb, Cr)) blob_add(b, x, y);
        }
    }
}

/* returns false if the frame type is not supported */
static bool scan_frame(const camera_buffer_t *buf, blob_t *b, int *w, int *h)
{
    memset(b, 0, sizeof *b);
    b->minx = b->miny = 1 << 30;
    b->maxx = b->maxy = -1;
    const uint8_t *p = buf->framebuf;
    switch (buf->frametype) {
    case CAMERA_FRAMETYPE_NV12:
        *w = (int)buf->framedesc.nv12.width; *h = (int)buf->framedesc.nv12.height;
        scan_nv12(p, &buf->framedesc.nv12, b);
        return true;
    case CAMERA_FRAMETYPE_YCBYCR:                /* Y0 Cb Y1 Cr */
        *w = (int)buf->framedesc.ycbycr.width; *h = (int)buf->framedesc.ycbycr.height;
        scan_422(p, *w, *h, (int)buf->framedesc.ycbycr.stride, 0, 1, 3, b);
        return true;
    case CAMERA_FRAMETYPE_CBYCRY:                /* Cb Y0 Cr Y1 */
        *w = (int)buf->framedesc.cbycry.width; *h = (int)buf->framedesc.cbycry.height;
        scan_422(p, *w, *h, (int)buf->framedesc.cbycry.stride, 1, 0, 2, b);
        return true;
    case CAMERA_FRAMETYPE_YCRYCB:                /* Y0 Cr Y1 Cb */
        *w = (int)buf->framedesc.ycrycb.width; *h = (int)buf->framedesc.ycrycb.height;
        scan_422(p, *w, *h, (int)buf->framedesc.ycrycb.stride, 0, 3, 1, b);
        return true;
    case CAMERA_FRAMETYPE_CRYCBY:                /* Cr Y0 Cb Y1 */
        *w = (int)buf->framedesc.crycby.width; *h = (int)buf->framedesc.crycby.height;
        scan_422(p, *w, *h, (int)buf->framedesc.crycby.stride, 1, 2, 0, b);
        return true;
    case CAMERA_FRAMETYPE_RGB8888:
        *w = (int)buf->framedesc.rgb8888.width; *h = (int)buf->framedesc.rgb8888.height;
        scan_rgb8888(p, *w, *h, (int)buf->framedesc.rgb8888.stride, b);
        return true;
    default:
        return false;
    }
}

/* ============================================================ fusion link */
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

static void fusion_health(int sh)
{
    if (g_coid == -1 || sh == g_sent_health) return;
    if (MsgSendPulse(g_coid, -1, PC_SENSOR_HEALTH, (SENS_CAMERA << 8) | sh) == -1) fusion_drop();
    else g_sent_health = sh;
}

static void fusion_send(const camera_sample_msg_t *smp, int sh, double fps)
{
    if (g_coid == -1) return;
    adas_hdr_t hdr;
    memset(&hdr, 0, sizeof hdr);
    hdr.type = MSG_SENSOR_DATA;
    hdr.sensor = SENS_CAMERA;
    hdr.count = 1;
    hdr.health = (uint16_t)sh;
    hdr.seq = g_seq++;
    hdr.sample_size = sizeof *smp;
    hdr.rate_hz = (float)fps;
    hdr.extra = smp->proc_us;
    iov_t iov[2];
    SETIOV(&iov[0], &hdr, sizeof hdr);
    SETIOV(&iov[1], (void *)smp, sizeof *smp);
    hdr.t_send_ns = now_ns();
    if (MsgSendv(g_coid, iov, 2, NULL, 0) == -1) fusion_drop();
    else g_msgs_sent++;
}

/* ============================================================ viewfinder callback */
static void vf_callback(camera_handle_t handle, camera_buffer_t *buf, void *arg)
{
    (void)handle; (void)arg;
    int64_t t_in = now_ns();
    int64_t t_cap = buf->frametimestamp > 0 ? buf->frametimestamp * 1000 : t_in;   /* us -> ns */

    blob_t b;
    int w = 0, h = 0;
    bool ok = scan_frame(buf, &b, &w, &h);
    int64_t t_out = now_ns();

    camera_sample_msg_t smp;
    memset(&smp, 0, sizeof smp);
    smp.t_ns = t_cap;
    smp.width = (uint32_t)w;
    smp.height = (uint32_t)h;
    smp.proc_us = (float)((t_out - t_in) / 1e3);
    smp.ttc_s = INFINITY;

    pthread_mutex_lock(&g_lock);
    g_st.frames++;
    g_st.frametype = (int)buf->frametype;
    if (g_st.last_frame_ns) {
        double dt = (double)(t_in - g_st.last_frame_ns) / 1e9;
        if (dt > 0) g_st.fps = g_st.fps == 0 ? 1.0 / dt : 0.9 * g_st.fps + 0.1 / dt;
    }
    g_st.last_frame_ns = t_in;
    g_st.cap_age_ms = (double)(t_out - t_cap) / 1e6;

    if (!ok) {
        g_st.unsupported++;
        g_st.health = SH_FAULT;
    } else {
        /* every sample stands for a 2x2 pixel block */
        double area_px = (double)b.n * 4.0;
        double frac = (w > 0 && h > 0) ? area_px / ((double)w * h) : 0;
        if (b.n > 0 && frac >= g_min_area) {
            double bbox = (double)(b.maxx - b.minx + 2) * (b.maxy - b.miny + 2);
            double fill = bbox > 0 ? area_px / bbox : 0;          /* compact blob -> high */
            double size_c = frac / (g_min_area * 3);
            smp.scale_px = (float)sqrt(area_px);
            smp.cx = (float)(b.sx / b.n / w);
            smp.cy = (float)(b.sy / b.n / h);
            smp.conf = (float)fmin(1.0, fmin(1.0, size_c) * fmin(1.0, fill * 1.5));
            g_st.detections++;
            g_st.miss_run = 0;
            g_st.det_run++;

            /* alpha-beta tracker on ln(scale): rate = d ln s / dt, TTC = 1 / rate */
            double ls = log(smp.scale_px);
            if (!g_exp.init || (double)(t_cap - g_exp.t) / 1e9 > 0.5) {
                g_exp.init = true; g_exp.ls = ls; g_exp.rate = 0; g_exp.t = t_cap;
            } else {
                double dt = (double)(t_cap - g_exp.t) / 1e9;
                if (dt > 0) {
                    double pred = g_exp.ls + g_exp.rate * dt;
                    double r = ls - pred;
                    g_exp.ls = pred + 0.35 * r;
                    g_exp.rate += 0.08 * r / dt;
                    g_exp.t = t_cap;
                }
            }
            smp.expand_rate = (float)g_exp.rate;
            /* publish TTC only after the tracker has settled (8 frames ~ 0.27 s) */
            if (g_st.det_run >= 8 && g_exp.rate > 0.03) smp.ttc_s = (float)(1.0 / g_exp.rate);
            g_st.health = SH_OK;
        } else {
            g_st.det_run = 0;
            g_exp.init = false;                  /* target lost: restart the expansion tracker */
            if (++g_st.miss_run >= 10) g_st.health = SH_NO_TARGET;
        }
    }
    g_st.last = smp;
    int sh = g_st.health;
    double fps = g_st.fps;
    pthread_mutex_unlock(&g_lock);

    /* IPC to fusion (only this callback thread uses the link) */
    fusion_link(t_out);
    fusion_health(sh);
    fusion_send(&smp, sh, fps);
}

static void status_callback(camera_handle_t handle, camera_devstatus_t status, uint16_t extra, void *arg)
{
    (void)handle; (void)arg;
    if (status == CAMERA_STATUS_DISCONNECTED) {
        fprintf(stderr, "camera disconnected\n");
        pthread_mutex_lock(&g_lock);
        g_st.health = SH_FAULT;
        pthread_mutex_unlock(&g_lock);
    }
    (void)extra;
}

/* ============================================================ main */
static void on_signal(int sig) { (void)sig; g_run = 0; }

static const char *ftype_name(int t)
{
    switch (t) {
    case CAMERA_FRAMETYPE_NV12:    return "NV12";
    case CAMERA_FRAMETYPE_YCBYCR:  return "YCbYCr";
    case CAMERA_FRAMETYPE_CBYCRY:  return "CbYCrY";
    case CAMERA_FRAMETYPE_YCRYCB:  return "YCrYCb";
    case CAMERA_FRAMETYPE_CRYCBY:  return "CrYCbY";
    case CAMERA_FRAMETYPE_RGB8888: return "RGB8888";
    default:                       return "unsupported";
    }
}

int main(int argc, char **argv)
{
    int run_s = 0, opt;
    while ((opt = getopt(argc, argv, "u:c:wt:a:")) != -1) {
        switch (opt) {
        case 'u': g_unit = atoi(optarg); break;
        case 'c':
            if      (!strcmp(optarg, "red"))   g_col = COL_RED;
            else if (!strcmp(optarg, "green")) g_col = COL_GREEN;
            else if (!strcmp(optarg, "blue"))  g_col = COL_BLUE;
            else { fprintf(stderr, "-c must be red, green or blue\n"); return 1; }
            break;
        case 'w': g_window = true; break;
        case 't': run_s = atoi(optarg); break;
        case 'a': g_min_area = atof(optarg) / 100.0; break;
        default:
            fprintf(stderr, "usage: %s [-u unit] [-c red|green|blue] [-w] [-t sec] [-a min_area_%%]\n", argv[0]);
            return 1;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    g_st.health = SH_NO_TARGET;

    camera_handle_t cam = CAMERA_HANDLE_INVALID;
    camera_error_t err = camera_open((camera_unit_t)g_unit, CAMERA_MODE_RW, &cam);
    if (err != CAMERA_EOK) {
        fprintf(stderr, "camera_open(unit %d) failed: error %d\n"
                        "  - is the camera plugged into a blue USB 3.0 port?\n"
                        "  - is the sensor service running?  (pidin | grep sensor)\n"
                        "  - which unit is the USB camera?   (grep -n SENSOR_UNIT /etc/config/sensor/sensor_rpi4.conf)\n",
                g_unit, (int)err);
        return 1;
    }
    camera_set_vf_mode(cam, CAMERA_VFMODE_DEFAULT);             /* ok if unsupported */
    if (!g_window) camera_set_vf_property(cam, CAMERA_IMGPROP_CREATEWINDOW, 0);

    err = camera_start_viewfinder(cam, vf_callback, status_callback, NULL);
    if (err != CAMERA_EOK) {
        fprintf(stderr, "camera_start_viewfinder failed: error %d\n", (int)err);
        camera_close(cam);
        return 1;
    }
    printf("camera node | unit %d | target colour %s | min size %.1f%% | window %s | fusion '%s'\n",
           g_unit, COL_NAME[g_col], g_min_area * 100, g_window ? "on" : "off", ADAS_SERVER_NAME);

    int64_t t_end = run_s > 0 ? now_ns() + (int64_t)run_s * NS_PER_SEC : 0;
    while (g_run) {
        struct timespec ts = { 0, 500000000L };
        nanosleep(&ts, NULL);

        pthread_mutex_lock(&g_lock);
        cam_status_t s = g_st;
        if (g_st.last_frame_ns && now_ns() - g_st.last_frame_ns > 2 * NS_PER_SEC) g_st.health = SH_FAULT;
        pthread_mutex_unlock(&g_lock);

        char ttc[16];
        if (isinf(s.last.ttc_s)) snprintf(ttc, sizeof ttc, "  -- ");
        else snprintf(ttc, sizeof ttc, "%5.2fs", s.last.ttc_s);
        if (s.health == SH_OK)
            printf("CAM OK  %4.1f fps %ux%u %s | target %s at (%.2f,%.2f) size %5.1f px conf %.2f | "
                   "expand %+5.2f/s TTC %s | proc %5.0f us age %4.1f ms | fusion %s\n",
                   s.fps, s.last.width, s.last.height, ftype_name(s.frametype), COL_NAME[g_col],
                   s.last.cx, s.last.cy, s.last.scale_px, s.last.conf, s.last.expand_rate,
                   ttc, s.last.proc_us, s.cap_age_ms,
                   g_coid != -1 ? "linked" : "not running");
        else
            printf("CAM %-9s %4.1f fps %ux%u %s | frames %llu | no %s target in view%s | fusion %s\n",
                   HEALTH_STR[s.health], s.fps, s.last.width, s.last.height, ftype_name(s.frametype),
                   (unsigned long long)s.frames, COL_NAME[g_col],
                   s.unsupported ? " | FRAME FORMAT NOT SUPPORTED" : "",
                   g_coid != -1 ? "linked" : "not running");
        fflush(stdout);
        if (t_end && now_ns() >= t_end) g_run = 0;
    }

    camera_stop_viewfinder(cam);
    camera_close(cam);
    if (g_coid != -1) name_close(g_coid);
    printf("\ncamera node stopped | frames %llu detections %llu | messages to fusion %llu\n",
           (unsigned long long)g_st.frames, (unsigned long long)g_st.detections,
           (unsigned long long)g_msgs_sent);
    return 0;
}
