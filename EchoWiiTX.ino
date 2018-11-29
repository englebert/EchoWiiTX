/*
  Filename: EchoWiiTX
  Author:   Englebert
  Date: 2018-11-22
  Description:
  This program is to read all the input values and transmit to NRF24L01 module. It must also retrieve data packets from EchoWiiRX.
  For this version, it will have a few basic functions as below:
  1. Throttle Input - A0
  2. Rudder Input   - A1
  3. Elevator Input - A3
  4. Aileron Input  - A2
  5. 16 channels input configurable to any switches with default mapping if not configure (IN00 ~ IN15):
     a. IN00: throttle trim up
     b. IN01: throttle trim down
     c. IN02: rudder trim left
     d. IN03: rudder trim right
     e. IN04: elevator trim up
     f. IN05: elevator trim down
     g. IN06: aileron trim up
     h. IN07: aileron trim down
     i. IN08: switch A
     j. IN09: switch B
     k. IN10: switch D
     l. IN11: switch E
     m. IN12: switch F
     n. IN13: switch G1
     g. IN14: switch G2
     h. IN15: UNUSED
  6. Analog variable channel:
     a. Channel C
  7. Compress data sending for better rate of transmission. Only set the transmission rate to 256kbps. NRF24L01 maximum bytes per packet is 32 bytes only.
  8. Currently will design a 16-channels radio for now and the data structure will be as below:
                 2                    2                    2                     2        
     | IN00~IN04, Throttle | IN05~IN09, Rudder | IN10~IN14, Elevator | IN15,IN16,Channel C |  

     Total 8 bytes for transmission and the last two bytes will have two more extra unused bits for future use maybe for trigger remote RX command to send
     back some data.
  9. Consists of RX telemetry data:
     a. battery level
     b. value of the sensors
        1. gyro x, y, z
        2. barometer values
        3. magnet meter (Future)
        4. gps values
     c. amperage (Future)
 10. Display information to OLED
 11. For this current version, Just TX to RX will sufficient.
 12. Telemetry will wait till item 11 done then proceed.
 13. Remote change PID over menu

 REF:
 Analog Read: https://steemit.com/utopian-io/@pakganern/oled-display-gauge-meter-using-potentiometer-arduino
 Faster way to Read and Init: https://www.instructables.com/id/Fast-digitalRead-digitalWrite-for-Arduino/
 Port Manipulation: https://hekilledmywire.wordpress.com/2011/02/23/direct-port-manipulation-using-the-digital-ports-tutorial-part-3/


 Button Matrix:
 
 
 */ 
