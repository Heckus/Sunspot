meow
Raspberry Pi 40-Pin GPIO Header (Board Pin Numbers)
----------------------------------------------------
Pins marked with '*' are directly used by the configured components.
Pins marked with '#' are required for power/ground connections.

+-----------------+--+--+-----------------+
| Function        | Pin | Pin | Function        |
+-----------------+--+--+-----------------+
|#3.3V Power      |  1#| #2 |#5V Power        |
|*GPIO 2 (SDA)    |  3*| #4 |#5V Power        |
|*GPIO 3 (SCL)    |  5*| #6 |#Ground          |
| GPIO 4          |  7 |  8 | GPIO 14 (TXD)   |
|#Ground          |  9#| 10 | GPIO 15 (RXD)   |
|*GPIO 17 (Switch)| 11*| 12 | GPIO 18         |
| GPIO 27         | 13 |#14 |#Ground          |
| GPIO 22         | 15 | 16 | GPIO 23         |
|#3.3V Power      | 17#| 18 | GPIO 24         |
| GPIO 10 (MOSI)  | 19 | 20 | Ground          |
| GPIO 9 (MISO)   | 21 | 22 | GPIO 25         |
| GPIO 11 (SCLK)  | 23 | 24 | GPIO 8 (CE0)    |
| Ground          | 25 | 26 | GPIO 7 (CE1)    |
| ID_SD (GPIO 0)  | 27 | 28 | ID_SC (GPIO 1)  |
| GPIO 5          | 29 | 30 | Ground          |
| GPIO 6          | 31 |*32 |*GPIO 12 (Servo) |
| GPIO 13         | 33 | 34 | Ground          |
| GPIO 19         | 35 | 36 | GPIO 16         |
| GPIO 26         | 37 | 38 | GPIO 20         |
| Ground          | 39 | 40 | GPIO 21         |
+-----------------+--+--+-----------------+

Summary of Connections:
-----------------------
- Pin 3 (GPIO 2):  INA219 SDA
- Pin 5 (GPIO 3):  INA219 SCL
- Pin 11 (GPIO 17): Push Button Switch Input
- Pin 32 (GPIO 12): Servo Signal (via PWM0)

- Pin 1 / 17:       3.3V Power (Required for INA219)
- Pin 2 / 4:        5V Power (Required for Servo)
- Pin 6 / 9 / 14 / etc.: Ground (Required for Switch, INA219, Servo)
