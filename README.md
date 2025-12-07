Frame: Symmetrical X (120mm motor spacing)
Weight: 58g AUW
Motors: 4x 720 coreless brushed
Props: 65mm 2-blade
Battery: 3.7V LiPo ~360mAh 25C
MCU: ESP32-C3 (ESP-IDF v5.x)
Sensors: MPU6050/IMU (I2C), optional ToF


main/
├── tasks/
│   ├── control_task (1ms PID cascade: rate → angle)
│   ├── sensor_task (IMU fusion, 500Hz)
│   └── comms_task (ESP-NOW RX/TX)
├── drivers/ (I2C/SPI/UART/PWM)
├── pid/ (tunable gains: e.g. P=0.70, I=0.044, D=0.16)
├── fusion/ (Madgwick/Kalman)
└── platformio.ini (ESP-IDF components)
