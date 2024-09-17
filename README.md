# Wireless Bank Login Embedded Firmware

This project, developed as a final assignment for Embedded Systems EE474 with partner Brandon Chang, connects two ESP32s using ESPNOW wireless communication to support remote login, deposit, withdrawal, and password management. The system integrates peripherals such as a rotary encoder, ultrasonic distance sensor, button, passive buzzer, and LCD display.

Peripheral monitoring and control are handled using FreeRTOS, a widely used real-time operating system. The project leverages semaphores, mutexes, and queues to ensure data consistency and prevent race conditions while taking advantage of the ESP32's dual-core architecture.

Although Arduino was used to expedite prototyping, direct register access is employed wherever possible to enhance task efficiency by reducing computation time. A planned improvement is migrating the firmware to pure C for greater performance and reliability.
