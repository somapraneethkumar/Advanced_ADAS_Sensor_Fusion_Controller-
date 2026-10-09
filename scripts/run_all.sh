#!/bin/sh
# run_all.sh - start the whole ADAS fusion system on the Pi (QNX)
# Usage: copy this script and the five binaries to the Pi, then:   sudo sh run_all.sh
# Stop everything:                                sudo slay fusion imu ultrasonic camera dashboard
#
# Binary names = Momentics project names: fusion, imu, ultrasonic, camera, dashboard

cd "$(dirname "$0")"

# QNX camera service (same command as /etc/startup/post_startup.sh). Only one instance may run.
if [ ! -e /dev/sensor/camera4 ]; then
    echo "starting QNX camera service (sensor)..."
    mkdir -p /data/share/sensor
    sensor -U 521:521 -b external -r /data/share/sensor -c /etc/config/sensor/sensor_rpi4.conf
    waitfor /dev/sensor/camera4 10 || echo "WARNING: /dev/sensor/camera4 did not appear - is the USB camera plugged in?"
fi

echo "starting fusion server..."
./fusion -o fusion_log.csv > fusion.out 2>&1 &
sleep 1

echo "starting IMU node (keep the IMU still for 2 s - calibration)..."
./imu > imu.out 2>&1 &

echo "starting ultrasonic node (GPIO interrupt mode)..."
./ultrasonic > ultrasonic.out 2>&1 &

echo "starting camera node (USB camera, unit 4, red target)..."
./camera -u 4 -c red > camera.out 2>&1 &

sleep 3
echo "starting dashboard (Ctrl-C closes everything started by this script)"
./dashboard
