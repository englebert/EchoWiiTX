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
|RudderMin   | The minumum value of the Rudder value   | 0x0004 |
|RudderMax   | The maximum value of the Rudder value   | 0x0006 |
|ElevatorMin | The miminum value of the Elevator value | 0x0008 |
|ElevatorMax | The maximum value of the Elevator value | 0x000A |
|AileronMin: | The minimum value of the Aileron value  | 0x000C |
|AileronMax  | The maximum value of the Aileron value  | 0x000E |
|            |                                         |        |
|checksum    | The EEPROM CHECKSUM value               | 0x03FF |
+------------+-----------------------------------------+--------+




By Englebert.
