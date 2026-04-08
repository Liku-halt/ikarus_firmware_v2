A lightweight, firmware-driven quadcopter built on the ESP-IDF v5.x framework. 
This project implements a full flight control stack on a single-core RISC-V MCU,
featuring ESP-NOW communication and a cascaded PID control loop.

The project is still far from completion.
The 1ms loop is currently jittery because of the single-core architecture (ESP32-C3);
the ESP-NOW protocol is inherently blocking, which causes fluctuations in the 1ms control loop timing.

In the future, I will build the controller with much faster MCUs capable of running the flight loop in less than 0.5ms.
These will be powerful enough to handle the estimation system, a part specifically designed for swarm applications.
