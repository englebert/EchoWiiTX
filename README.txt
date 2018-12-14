EchoWiiTX is specifically for this custom made controller which can:
1. Having telemetry function
2. Able to scan the RF channels before each flight prevent interferences
3. Able to customize own settings
4. Changing the PID direct from the remote to RX module and command the FC.
5. Highly integrated to MultiWii aka rebranding as EchoWII.

Once I had completed the whole thing, I will release the schematics for everyone to build your own multirotor.


EEPROM Format Design:
+------------+-----------------------------------------+--------+------+
|Name        | Description                             | Addr   | Size |
+------------+-----------------------------------------+--------+------+
|ThrottleMin | The minimum value of the Throttle value | 0x0000 |   2  |
|ThrottleMax | The maximum value of the Throttle value | 0x0002 |   2  |
|YawMin      | The minumum value of the Yaw value      | 0x0004 |   2  |
|YawMax      | The maximum value of the Yaw value      | 0x0006 |   2  |
|PitchMin    | The miminum value of the Pitch value    | 0x0008 |   2  |
|PitchMax    | The maximum value of the Pitch value    | 0x000A |   2  |
|RollMin     | The minimum value of the Roll value     | 0x000C |   2  |
|RollMax     | The maximum value of the Roll value     | 0x000E |   2  |
|ReverseBits | Analog input reverse settings           | 0x0010 |   1  |
|            |                                         |        |      |
|checksum    | The EEPROM CHECKSUM value               | 0x03FF |   1  |
+------------+-----------------------------------------+--------+------+


TX Format:
+--------+------+-------+----------+-----+------+-------------------------------------------------------+-----------------+
|        |      |       |          |     |      |                           CH6                         |                 |
|Channels| CH1  |  CH2  |    CH3   | CH4 |  CH5 +------+------+------+------+------+------+------+------+       CH7       |
|        |      |       |          |     |      | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |                 |
+--------+------+-------+----------+-----+------+------+------+------+------+------+------+------+------+-----------------+
| Values | Roll | Pitch | Throttle | Yaw | Aux1 | Aux2 | Aux3 | Aux4 |          Command Bits            |  Command Values |
+--------+------+-------+----------+-----+------+------+------+------+------+------+------+------+------+-----------------+

The above diagram is based on 7 channels over NRF24L01 modules. By default NRF24L01 lowest bitrate is 250kbps which is 32kB/s.
If transferring at this speed, I can actually sending a series of data over to RX module. Based on my previous test, I found
that the code able to transmit 127 packets per second which is around 1016B/s. If I am able reduce to 7 Bytes per packet, I
should be able to achieve 145 packets per second to RX module. Each process of TX requires about 7.87mS for ATMEGA328P.


On Going Development:
1. Enhancing the TX speed - DONE
2. Remote changing Channels
3. Channel Mapping
4. Switch Mapping
5. Telemetry
6. Reverse Stick Values - DONE
7. Timer
Updates:
1. Enhanced TX Speed from 127 Packets/s to 755 Packets/s. This is due to the OLED refreshing rate reduced to 2FPS. Saving some resources and give more resources to the data transmission. Also removed the CH8 from the data structure.

By Englebert.

Ref:
Timers and Interrupts:
https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/

