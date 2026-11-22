# ToDo

- [ ] Ramp timer ISR: replace the FreeRTOS mutex taken on every tick (not ISR-safe) with a spinlock or a lock-free hook list, and add an option to run the timer interrupt on core 1. Costs 18-27% of core 0 while stepping on SandBot (ESP32-S3). Details: devdocs/ramp-timer-isr-cost.md
