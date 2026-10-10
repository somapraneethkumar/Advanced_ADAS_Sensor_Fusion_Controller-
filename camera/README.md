# ADAS Sensor Fusion on QNX

Deterministic collision-risk decisions on a Raspberry Pi 4 running QNX 8.0.
Three real sensors (IMU, ultrasonic, USB camera) run as separate QNX processes and feed
a fusion server that decides **SAFE / WARNING / CRITICAL / EMERGENCY** every 20 ms.

QNX India Hackathon (QNX Everywhere) · Team: **FusionDrive** · Team ID: **QNXHACK-HYD-026**

---

## Architecture

```
 MPU6050 IMU  ──I2C1──> [imu]         ──┐
 HC-SR04      ──GPIO──> [ultrasonic]  ──┼── IOV messages + pulses ──> [fusion] ──shared memory──> [dashboard]
 Logitech C920 ─USB──> (QNX sensor) ─> [camera] ──┘                     │                             ▲
                                                                        └──── pulse (MsgDeliverEvent) ┘
```

| Process | Hardware access | Job | Rate |
|---|---|---|---|
| `imu` | I2C via `/dev/i2c1` (`devctl`) | Drains the MPU6050 hardware FIFO, timestamps every sample, measures sensor clock drift, gyro calibration, roll/pitch, health monitor | 200 Hz sensor, read every 20 ms |
| `ultrasonic` | GPIO registers (`mmap_device_memory`), GPIO interrupt 145 | Times the echo with the ARM cycle counter, temperature-compensated range, alpha-beta tracker, outlier rejection | 15 Hz |
| `camera` | QNX camera library (`libcamapi`, Sensor Framework unit 4) | Colour-target detection, target size, TTC from image expansion | 30 fps |
| `fusion` | – | Kalman filter [distance, closing speed], IMU ego-motion input, camera cross-calibration, TTC and decision, deadline monitor | 50 Hz, 20 ms deadline |
| `dashboard` | – | Live view of all sensors, fused state and real-time statistics | woken by fusion |

### QNX mechanisms used

| Mechanism | Where |
|---|---|
| `name_attach` / `name_open` | fusion registers `"adas_fusion"`; clients find it in any start order |
| IOV messaging (`MsgSendv` / `MsgReceivev`) | header + sample array sent as one two-part message |
| Pulses | timer ticks, sensor health changes, kernel disconnect pulse when a sensor process dies, dashboard wake-up |
| `MsgRegisterEvent` + `MsgDeliverEvent` | fusion wakes the dashboard only when needed |
| Shared memory (`shm_open "/adas_state"`, seqlock) | lock-free publication of the fused state |
| Interrupts (`InterruptAttachEvent`, IRQ 145) | ultrasonic echo edges; automatic fallback to polling |
| Timers → pulses | every periodic loop (no `sleep`) |
| SCHED_FIFO priorities, priority inheritance | see table below |

| Thread | Process | Priority | Released by |
|---|---|---|---|
| `fusion_tick` | fusion | 30 | timer pulse, 20 ms |
| `us_ping` | ultrasonic | 29 | timer 66 ms + GPIO IRQ |
| `imu_acq` | imu | 28 | timer pulse, 20 ms |
| `fusion_rx` | fusion | 20 (inherits sender) | `MsgReceivev` |
| camera callback | camera | libcamapi thread | each frame |
| `dashboard` | dashboard | 10 | pulse from fusion |

### Fault handling

| Fault | Detection | Response |
|---|---|---|
| Sensor process killed | kernel disconnect pulse | sensor marked OFFLINE; fusion continues (camera keeps the track once calibrated) |
| Sensor process restarted | `name_open` retried every second | reconnects automatically |
| Sensor alive but silent | 500 ms stale-data timeout | DEGRADED, excluded from the filter |
| IMU I2C errors | 5 consecutive errors | FAULT, automatic re-initialisation |
| Lost / stray echoes | two-echo confirmation, innovation gate | ignored; NO TARGET after 3 misses |
| Decision chatter | hysteresis | escalate at once, relax after 200 ms |

---

## Hardware and wiring

| Device | Connection |
|---|---|
| MPU6050 | VCC pin 1 (3.3 V), GND pin 6, SDA pin 3, SCL pin 5, AD0 → GND (0x68) |
| HC-SR04 | VCC pin 2 (5 V), GND pin 14, TRIG pin 16 (GPIO23), ECHO → 1 kΩ → pin 18 (GPIO24) → 2 kΩ → GND (**voltage divider required**) |
| Logitech C920 | blue USB 3.0 port |

Board: Raspberry Pi 4 Model B with the QNX 8.0 Quick Start Target Image.

## Build

Requirements: QNX SDP 8.0, Momentics IDE, and for the camera the Software Center package
**QNX SDP 8.0 Sensor Framework Camera Library** (`com.qnx.qnx800.target.sf.camapi`).

Each folder is one Momentics project (QNX Executable, `aarch64le`). Import the five folders,
or build from a shell with the SDP environment loaded:

```sh
for p in fusion imu ultrasonic camera dashboard; do make -C $p; done
```

Binaries are produced in `<project>/build/aarch64le-debug/<project>`.

## Run (on the Pi)

Copy the five binaries and `scripts/run_all.sh` to `/home/qnxuser`, then:

```sh
sudo sh run_all.sh
```

The script starts the QNX camera service if needed, then fusion, the three sensor nodes and the dashboard.
Stop everything with `sudo slay fusion imu ultrasonic camera dashboard`.

Fault-injection demo (second terminal):

```sh
sudo slay ultrasonic                                  # ULTRASONIC -> OFFLINE, fusion keeps running
sudo sh -c './ultrasonic > ultrasonic.out 2>&1 &'     # reconnects within ~1 s
```

Useful options: `fusion -l 4 -p 5` (CPU-load test threads), `fusion -o log.csv`, `camera -w` (video window),
`camera -c red|green|blue`, `imu -t 20 -o imu.csv`, `ultrasonic -I 0` (force polling).

## Measured results (Raspberry Pi 4, QNX 8.0)

| Metric | Result |
|---|---|
| Fusion deadline misses | **0** over 14 300+ cycles |
| Fusion worst response | 501 µs in the traced run (≤ 9.7 ms in any run) vs 20 ms deadline |
| IPC latency sensor → fusion | 7–20 µs |
| IMU | 200.249 Hz, sensor clock drift +0.12 %, 0 samples lost, worst FIFO read 1.98 ms |
| Ultrasonic CPU per ping | 33–75 µs (interrupt) vs ~7 000 µs (polling) |
| Camera | 30 fps, ~2 ms processing per frame |
| System Profiler (5 s trace) | 79.2 % idle on 4 cores, 0 of 1688 buffers dropped; camera 5.3 %, ultrasonic 2.4 % of one core |

![System Profiler trace](docs/system_profiler_trace.png)

## Repository layout

```
common/adas_msg.h        shared message, pulse and shared-memory definitions
fusion/  imu/  ultrasonic/  camera/  dashboard/    one Momentics project each (src/ + Makefile)
scripts/run_all.sh       start-up script for the Pi
docs/                    presentation, IMU guide, profiler screenshot
```

## Development notes

<!-- Fill this in honestly before submitting, as the hackathon rules ask. Example:
     "Parts of this code were written with the help of an AI assistant (Claude). The team designed the system,
      wired and tested the hardware, debugged it on the Pi, and reviewed every module." -->
`<describe how the code was developed and any tools used>`
