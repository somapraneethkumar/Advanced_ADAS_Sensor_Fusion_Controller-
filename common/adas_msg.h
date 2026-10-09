/*
 * adas_msg.h  -  QNX-eHACK 2026: shared protocol for the ADAS sensor-fusion processes
 *
 * Copy this file into the src/ folder of EVERY project (fusion, imu, ultrasonic, dashboard).
 *
 *  Process            Talks to fusion with
 *  -----------------  -------------------------------------------------------------
 *  imu_node           IOV message (header + up to 64 IMU samples), health pulses
 *  us_node            IOV message (header + 1 range sample),       health pulses
 *  camera_node        IOV message (header + 1 camera detection),   health pulses
 *  fusion_server      name_attach("adas_fusion"); publishes state in shared memory
 *  dashboard          reads shared memory; woken by a pulse delivered by fusion
 */
#ifndef ADAS_MSG_H
#define ADAS_MSG_H

#include <stdint.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>

#define ADAS_SERVER_NAME   "adas_fusion"     /* name_attach / name_open            */
#define ADAS_SHM_NAME      "/adas_state"     /* shm_open                           */

/* ---------------------------------------------------------------- message types
 * Must be above _IO_MAX so they never collide with resource-manager messages. */
#define MSG_SENSOR_DATA    (_IO_MAX + 1)
#define MSG_DASH_REGISTER  (_IO_MAX + 2)

/* ---------------------------------------------------------------- pulse codes */
#define PC_FUSION_TICK     (_PULSE_CODE_MINAVAIL + 1)   /* fusion's own 50 Hz timer   */
#define PC_SENSOR_HEALTH   (_PULSE_CODE_MINAVAIL + 2)   /* value = sensor<<8 | health */
#define PC_DASH_UPDATE     (_PULSE_CODE_MINAVAIL + 3)   /* fusion -> dashboard        */
#define PC_SENSOR_TICK     (_PULSE_CODE_MINAVAIL + 4)   /* sensor's own timer         */
#define PC_GPIO_IRQ        (_PULSE_CODE_MINAVAIL + 5)   /* GPIO interrupt event       */

/* ---------------------------------------------------------------- sensors / health */
enum { SENS_IMU = 0, SENS_RANGE = 1, SENS_CAMERA = 2, SENS_COUNT = 3 };
enum { SH_OK = 0, SH_NO_TARGET = 1, SH_DEGRADED = 2, SH_FAULT = 3, SH_OFFLINE = 4 };
enum { DEC_SAFE = 0, DEC_WARNING, DEC_CRITICAL, DEC_EMERGENCY, DEC_NO_TRACK };

static const char *const SENSOR_NAME[SENS_COUNT] = { "IMU", "ULTRASONIC", "CAMERA" };
static const char *const HEALTH_STR[] = { "OK", "NO TARGET", "DEGRADED", "FAULT", "OFFLINE" };
static const char *const DECISION_STR[] = { "SAFE", "WARNING", "CRITICAL", "EMERGENCY", "NO TRACK" };

#define ADAS_MAX_SAMPLES   64

/* ---------------------------------------------------------------- messages */
typedef struct {
    uint16_t type;          /* MSG_SENSOR_DATA                                  */
    uint16_t sensor;        /* SENS_*                                           */
    uint16_t count;         /* number of samples in the second IOV part         */
    uint16_t health;        /* SH_* at time of sending                          */
    uint32_t seq;           /* message sequence number (detects lost messages)  */
    uint32_t sample_size;   /* sizeof one sample, sanity check                  */
    int64_t  t_send_ns;     /* CLOCK_MONOTONIC when MsgSendv was called         */
    float    rate_hz;       /* sensor's measured sample rate                    */
    float    extra;         /* IMU: temperature C / range: CPU us per ping      */
} adas_hdr_t;

typedef struct {            /* one IMU sample (reconstructed timestamp)         */
    int64_t t_ns;
    float   a[3];           /* m/s^2, calibrated                                */
    float   g[3];           /* deg/s, bias removed                              */
    float   roll, pitch;    /* deg                                              */
} imu_sample_msg_t;

typedef struct {            /* one range measurement                            */
    int64_t t_ns;           /* reflection instant                               */
    float   dist_m;         /* tracked distance                                 */
    float   raw_m;          /* raw distance                                     */
    float   vel_ms;         /* sensor-side closing speed (negative = closing)   */
    float   echo_us;
} range_sample_msg_t;

typedef struct {            /* one camera frame result                          */
    int64_t  t_ns;          /* frame capture time (CLOCK_MONOTONIC)             */
    float    scale_px;      /* sqrt(target area) in pixels, 0 = no target       */
    float    cx, cy;        /* target centroid, 0..1 of image width / height    */
    float    conf;          /* detection confidence 0..1                        */
    float    ttc_s;         /* camera-only TTC from image expansion (inf = none)*/
    float    expand_rate;   /* d ln(scale) / dt  [1/s], positive = approaching  */
    float    proc_us;       /* image processing time for this frame             */
    uint32_t width, height; /* frame size                                       */
} camera_sample_msg_t;

typedef struct {
    uint16_t        type;   /* MSG_DASH_REGISTER                                */
    uint16_t        zero;
    struct sigevent event;  /* pulse fusion will deliver with MsgDeliverEvent   */
} dash_register_msg_t;

/* ---------------------------------------------------------------- shared memory
 * Single writer (fusion), many readers (dashboard). Seqlock: the writer makes seq
 * odd while writing and even when done; readers retry if seq was odd or changed. */
typedef struct {
    float    rate_hz;
    float    latency_us;    /* IPC latency: send -> received by fusion          */
    uint8_t  health;
    uint8_t  pad[3];
    uint64_t msgs, samples, lost_msgs;
    int64_t  last_t_ns;
} shm_sensor_t;

typedef struct {
    volatile uint32_t seq;
    uint32_t version;
    int64_t  t_ns;
    uint64_t update_count;

    shm_sensor_t sens[SENS_COUNT];

    /* latest raw sensor values */
    float imu_a[3], imu_g[3], imu_roll, imu_pitch, imu_temp, imu_lin_acc;
    float range_raw_m, range_m, range_vel_ms, range_echo_us, range_cpu_us;

    /* camera */
    float    cam_scale_px, cam_cx, cam_cy, cam_conf, cam_ttc_s, cam_dist_m, cam_K;
    float    cam_proc_us, cam_age_ms;
    uint32_t cam_w, cam_h;
    int32_t  cam_calibrated, cam_calib_samples, cam_used;

    /* fused state + decision */
    float   fused_d, fused_v, ttc_s, d_var;
    int32_t decision;
    int32_t ego_moving;

    /* real-time stats */
    float    exec_us, resp_us, worst_exec_us, worst_resp_us, deadline_us;
    uint64_t cycles, misses;
    uint64_t pulses_rx, msgs_rx, disconnects, dash_events;
    int32_t  fusion_pid;
} adas_shm_t;

#define ADAS_SHM_VERSION 2

static inline void shm_write_begin(adas_shm_t *s)
{
    __atomic_add_fetch(&s->seq, 1, __ATOMIC_ACQ_REL);   /* -> odd */
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline void shm_write_end(adas_shm_t *s)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_add_fetch(&s->seq, 1, __ATOMIC_ACQ_REL);   /* -> even */
}

/* copies a consistent snapshot; returns 0 on success, -1 if writer kept interfering */
static inline int shm_read(const adas_shm_t *s, adas_shm_t *out)
{
    for (int tries = 0; tries < 1000; tries++) {
        uint32_t a = __atomic_load_n(&s->seq, __ATOMIC_ACQUIRE);
        if (a & 1u) continue;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        __builtin_memcpy(out, (const void *)s, sizeof *out);
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        uint32_t b = __atomic_load_n(&s->seq, __ATOMIC_ACQUIRE);
        if (a == b) return 0;
    }
    return -1;
}

#endif /* ADAS_MSG_H */
