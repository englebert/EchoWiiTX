EchoWiiTX is specifically for this custom made controller which can:
1. Having telemetry function
2. Able to scan the RF channels before each flight prevent interferences
3. Able to customize own settings
4. Changing the PID direct from the remote to RX module and command the FC.
5. Highly integrated to MultiWii aka rebranding as EchoWII.

Once I had completed the whole thing, I will release the schematics for everyone to build your own multirotor.


EEPROM Format Design:
+------------+-----------------------------------------+--------+
|Name        | Description                             | Addr   |
+------------+-----------------------------------------+--------+
|ThrottleMin | The minimum value of the Throttle value | 0x0000 |
|ThrottleMax | The maximum value of the Throttle value | 0x0002 |
|YawMin      | The minumum value of the Yaw value      | 0x0004 |
|YawMax      | The maximum value of the Yaw value      | 0x0006 |
|PitchMin    | The miminum value of the Pitch value    | 0x0008 |
|PitchMax    | The maximum value of the Pitch value    | 0x000A |
|RollMin     | The minimum value of the Roll value     | 0x000C |
|RollMax     | The maximum value of the Roll value     | 0x000E |
|            |                                         |        |
|checksum    | The EEPROM CHECKSUM value               | 0x03FF |
+------------+-----------------------------------------+--------+




By Englebert.

Ref:
Timers and Interrupts:
https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
