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

static int      g_unit = 4;          /* default camera unit */
static colour_t g_col = COL_RED;     /* default target colour */
static bool     g_window;
static double   g_min_area = 0.003;  /* min target fraction */

static volatile sig_atomic_t g_run = 1;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/* camera status */
typedef struct {
    camera_sample_msg_t last;
    uint64_t frames, detections, unsupported;
    int      frametype;
    double   fps;
    int64_t  last_frame_ns;
    int      health;
    int      miss_run;
    int      det_run;
    double   cap_age_ms;
} cam_status_t;
static cam_status_t g_st;

/* expansion tracker */
typedef struct { bool init; double ls, rate; int64_t t; } exp_t;
static exp_t g_exp;

/* IPC link state */
static int      g_coid = -1;
static uint32_t g_seq;
static int64_t  g_next_connect;
static int      g_sent_health = -1;
static uint64_t g_msgs_sent;

/* monotonic timestamp (ns) */
static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* colour detection */
static inline bool is_target(int y, int cb, int cr)
{
    if (y < 25 || y > 245) return false;
    switch (g_col) {
    case COL_RED:   return cr > 160 && cb < 140 && cr - cb > 40;
    case COL_GREEN: return cr < 115 && cb < 120;
    case COL_BLUE:  return cb > 155 && cr < 130;
    }
    return false;
}

/* blob accumulator */
typedef struct {
    uint64_t n;
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

/* scan packed 4:2:2 formats */
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

/* scan NV12 format */
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

/* scan RGB8888 format */
static void scan_rgb8888(const uint8_t *buf, int w, int h, int stride, blob_t *b)
{
    for (int y = 0; y < h; y += 2) {
        const uint8_t *row = buf + (size_t)y * stride;
        for (int x = 0; x < w; x += 2) {
            const uint8_t *p = row + x * 4;
            int B = p[0], G = p[1], R = p[2];
            int Y  = ( 77 * R + 150 * G +  29 * B) >> 8;
            int Cb = ((-43 * R -  85 * G + 128 * B) >> 8) + 128;
            int Cr = ((128 * R - 107 * G -  21 * B) >> 8) + 128;
            b->samples++;
            if (is_target(Y, Cb, Cr)) blob_add(b, x, y);
        }
    }
}

/* frame dispatcher */
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
    case CAMERA_FRAMETYPE_YCBYCR:
        *w = (int)buf->framedesc.ycbycr.width; *h = (int)buf->framedesc.ycbycr.height;
        scan_422(p, *w, *h, (int)buf->framedesc.ycbycr.stride, 0, 1, 3, b);
        return true;
    case CAMERA_FRAMETYPE_CBYCRY:
        *w = (int)buf->framedesc.cbycry.width; *h = (int)buf->framedesc.cbycry.height;
        scan_422(p, *w, *h, (int)buf->framedesc.cbycry.stride, 1, 0, 2, b);
        return true;
    case CAMERA_FRAMETYPE_YCRYCB:
        *w = (int)buf->framedesc.ycrycb.width; *h = (int)buf->framedesc.ycrycb.height;
        scan_422(p, *w, *h, (int)buf->framedesc.ycrycb.stride, 0, 3, 1, b);
        return true;
    case CAMERA_FRAMETYPE_CRYCBY:
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
