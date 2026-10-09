# Advanced_ADAS_Sensor_Fusion_Controller-
Real-time ADAS sensor fusion on QNX 8.0 / Raspberry Pi 4. IMU (I2C FIFO), HC-SR04 (GPIO interrupt) and USB camera run as separate QNX processes, feeding a 50 Hz Kalman fusion server via IOV messages, pulses and shared memory. Decides SAFE/WARNING/CRITICAL/EMERGENCY every 20 ms with 0 deadline misses. Team FusionDrive · QNXHACK-HYD-026.

