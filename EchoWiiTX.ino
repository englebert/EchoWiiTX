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

#include <avdweb_AnalogReadFast.h>
#include "ssd1306.h"
// #include "nano_gfx.h"

// For the Radio Module - NRF24L01
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*
 * NRF24L01 Configurations
 */
/* Create a unique pipe out. The receiver has to wear the same unique code*/
const uint64_t pipeOut = 0xE8E8F0F0E1LL;  // IMPORTANT: The same as in the receiver
RF24 radio(9, 10);                        // D9 -> Enable, D10 -> CSN pin
                                          // The sizeof this struct should not exceed 32 bytes
                                          // This gives us up to 32 8 bits channels

// NRF24L01 registers we need - For 2.4G Scanning
#define _NRF24_CONFIG      0x00
#define _NRF24_EN_AA       0x01
#define _NRF24_RF_CH       0x05
#define _NRF24_RF_SETUP    0x06
#define _NRF24_RPD         0x09

#define CE  9
#define MAX_CHANNELS 125

// Use for storing the data of the channel load
int channel_loads;

struct MyData {
    unsigned int ch1;
    unsigned int ch2;
    unsigned int ch3;
    unsigned int ch4;
    unsigned int ch5;
    unsigned int ch6;
    unsigned int ch7;
    unsigned int ch8;
};
MyData data;

// Including images, icons
#include "echowiitx_logo.h"

#define NOP __asm__ __volatile__ ("nop\n\t")

//declare reset function at address 0
void(* resetFunc) (void) = 0;

/**
 * Configurations and Values
 */

// Joysticks
#define THROTTLE_PORT A0
#define RUDDER_PORT   A1
#define ELEVATOR_PORT A3
#define AILERON_PORT  A2

// Analogue Ports
#define AUX1_PORT     A6
#define BAT_IN        A7

// Voltage Scaling Value
#define VOLTAGE_SCALE 0.04878

float battery_voltage;

// For Debugging Purposes
// #define DEBUG 1

// Temporary
uint16_t sensorValue0, sensorValue1;
uint16_t throttleValue, rudderValue, elevatorValue, aileronValue;

char buf[32];
char line_buffer[32];

// Default mode = 0
// MODE:
// 0: Normal TX Mode
// 1: RF Scanning Mode
uint8_t txmode = 0;

/*
 * 2.4G Scanning Program [START]
 */
// Get the value of a nRF24L01 register
byte getRegister(byte r) {
    byte c;
    PORTB &=~_BV(2);
    c = SPI.transfer(r&0x1F);
    c = SPI.transfer(0);  
    PORTB |= _BV(2);
    return(c);
}

// enable RX 
void enableRX(void) {
     PORTB |= _BV(1);
}

// disable RX
void disableRX(void) {
    PORTB &= ~_BV(1);
}

// set the value of a nRF24L01 register
void setRegister(byte r, byte v) {
    PORTB &=~_BV(2);
    SPI.transfer((r&0x1F)|0x20);
    SPI.transfer(v);
    PORTB |= _BV(2);
}

// power up the nRF24L01 chip
void powerUp(void) {
    setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x02);
    delayMicroseconds(130);
}

// switch nRF24L01p off
void powerDown(void) {
    setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)&~0x02);
}

// setup RX-Mode of nRF24L01
void setRX(void) {
    setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x01);
    enableRX();
    // this is slightly shorter than
    // the recommended delay of 130 usec
    // delayMicroseconds(100);
    delayMicroseconds(130);
}

// Setup SPI
void setupSPI() {
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setBitOrder(MSBFIRST);
}

void rfscanMode() {
    // Setup SPI
    setupSPI();

    // Activate CE
    pinMode(CE, OUTPUT);

    // Startup Receiver
    powerUp();

    // Switch Off ShockBurst
    setRegister(_NRF24_EN_AA, 0x0);

    // make sure RF-section is set properly 
    // - just write default value... 
    setRegister(_NRF24_RF_SETUP, 0x0F);

    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(0,  0, "[ 2.4G Scanner ]", STYLE_NORMAL);

    // Preparing outlines...
    // ssd1306_drawLine(0, 55, 128, 55);
    ssd1306_printFixed(0,  60, "0        64       128", STYLE_NORMAL);

    // Forever here... till the switch position changed.
    while(txmode == 1) {
        scanChannels();
    }
}

// Scanning Channels
void scanChannels() {
    // Serial.println("Scanned Results:");
    disableRX();

    // Collects 200 samples on each channel
    for(int i = 0; i <= MAX_CHANNELS; i++) {
        channel_loads = 0;
        
        for(int j = 0; j < 200; j++) {
            // Select New Channel
            setRegister(_NRF24_RF_CH, i);
        
            // Switch on RX
            setRX();
        
            // Wait enough for RX things to settle
            delayMicroseconds(40);
        
            // This is actually the point where the RPD Flag is set, when CE goes low
            disableRX();
        
            // Read out the RPD Flag
            if(getRegister(_NRF24_RPD) > 0) channel_loads++;
        }

        // Displaying Results
        // Clear previous line...
        ssd1306_negativeMode();
        ssd1306_drawLine(i, 55, i, 16);
        if(channel_loads > 0) {
//            Serial.print("Channel [");
//            Serial.print(i);
//            Serial.print("] : ");
//            Serial.println(channel_loads);
            // Using only 38 pixels of y
            // Starting from row 17, ends at row 54
            // Converts the 200 samples into 38
            // y = channel_loads * 38 / 200;
            // Speed up and less calculation
            uint8_t y = channel_loads * 0.19;
            y = 54 - y;;
            // Draw current line
            ssd1306_positiveMode();
            ssd1306_drawLine(i, 54, i, y);    
        }

        // TODO: Temporary here and will be remove in future by using functions
        // Checking Inputs
        PORTD = B10000000;
        NOP;
        uint8_t val14 = (PIND & (1<<PD1))>>PD1;
        PORTD = B00000000;
        if(val14 == 0) {
            txmode = 0;
            ssd1306_fillScreen(0x00);
            ssd1306_setFixedFont(ssd1306xled_font6x8);
            ssd1306_printFixed(0,  0, "[ ECHOWII TX ]", STYLE_NORMAL);
            resetFunc();
            // i = MAX_CHANNELS + 1;
        }
    }
}

/*
 * 2.4G Scanning Program [END]
 */

void resetData() {
    // This are the start values of each channel
    data.ch1 = 0;
    data.ch2 = 0;
    data.ch3 = 0;
    data.ch4 = 0;
    data.ch5 = 0;
    data.ch6 = 0;
    data.ch7 = 0;
    data.ch8 = 0;
}

/*
 * Returns a corrected value for a joystick position that takes into account
 * the values of the outer extents and the middle of the joystick range.
 */
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse) {
    val = constrain(val, lower, upper);
    if ( val < middle ) {
      val = map(val, lower, middle, 0, 128);
    } else {
      val = map(val, middle, upper, 128, 255);
    }
    return ( reverse ? 255 - val : val );
}

/*
 *  Arduino Setup
 */
void setup() {
    // Debugging Start
    #ifdef DEBUG
        Serial.begin(57600);
    #endif
    // Debugging End

    // Initializing Multiplexing Input Pads
    setup_inputs();

    // Reset
    resetData();
    
    // Setup Radio Unit
    setup_radio();
    
    // Initializing OLED and display logo
    ssd1306_128x64_i2c_init();
    show_logo();
    delay(2000);

    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(0,  0, "[ ECHOWII TX ]", STYLE_NORMAL);
}

void setup_radio() {
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);

    // Added for extending the range
    radio.setPALevel(RF24_PA_MAX);

    // Above most Wifi Channels
    // TODO: Later this need to by dynamic for better control
    radio.setChannel(1);

    // 8 bits CRC
    // radio.setCRCLength( RF24_CRC_8 ) ; 

    // Disable dynamic payloads 
    // radio.write_register(DYNPD,0); 

    // increase the delay between retries & # of retries
    // radio.setRetries(15,15);
    // End of modification

    radio.openWritingPipe(pipeOut);
    resetData();
}

/*
 * Setup Input Pads
 * 0 = INPUT, 1 = OUTPUT
 * OUTPUT: Pin 7, 6, 5, 4
 * INPUT:  Pin 3, 2, 1, 0
 */
void setup_inputs() {
    DDRD = B11110000;
}

/*
 * Start Up Logo
 */
void show_logo() {
    ssd1306_drawBitmap(0, 0, 128, 64, echowiitx_logo);
}

/*
 * Arduino Main Loop
 */
void loop() {
    if(txmode == 0) {
        txMode();
    } else if(txmode == 1) {
        rfscanMode();
    }
}

void txMode() {
    throttleValue = analogReadFast(THROTTLE_PORT);
    rudderValue = analogReadFast(RUDDER_PORT);
    sprintf(buf, "T:%04i R:%04i", throttleValue, rudderValue);
    ssd1306_printFixed(0, 16, buf, STYLE_NORMAL);
  
    aileronValue = analogReadFast(AILERON_PORT);
    elevatorValue = analogReadFast(ELEVATOR_PORT);
    sprintf(buf, "A:%04i E:%04i", aileronValue, elevatorValue);
    ssd1306_printFixed(0, 24, buf, STYLE_NORMAL);
    
    // Scannning with multiplexing
    // Start from 1st Row
    PORTD = B00010000;
    NOP;
    uint8_t val1 = (PIND & (1<<PD0))>>PD0;
    uint8_t val2 = (PIND & (1<<PD1))>>PD1;
    uint8_t val3 = (PIND & (1<<PD2))>>PD2;
    uint8_t val4 = (PIND & (1<<PD3))>>PD3;
    PORTD = B00100000;
    NOP;
    uint8_t val5 = (PIND & (1<<PD0))>>PD0;
    uint8_t val6 = (PIND & (1<<PD1))>>PD1;
    uint8_t val7 = (PIND & (1<<PD2))>>PD2;
    uint8_t val8 = (PIND & (1<<PD3))>>PD3;
    PORTD = B01000000;
    NOP;
    uint8_t val9 =  (PIND & (1<<PD0))>>PD0;
    uint8_t val10 = (PIND & (1<<PD1))>>PD1;
    uint8_t val11 = (PIND & (1<<PD2))>>PD2;
    uint8_t val12 = (PIND & (1<<PD3))>>PD3;
    PORTD = B10000000;
    NOP;
    uint8_t val13 = (PIND & (1<<PD0))>>PD0;
    uint8_t val14 = (PIND & (1<<PD1))>>PD1;
    uint8_t val15 = (PIND & (1<<PD2))>>PD2;
    uint8_t val16 = (PIND & (1<<PD3))>>PD3;
    PORTD = B00000000;
  
    sprintf(buf, "%i %i %i %i", val5, val2, val3, val4);
    ssd1306_printFixed(0, 32, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", val1, val6, val7, val8);
    ssd1306_printFixed(0, 40, buf, STYLE_NORMAL);

    // BAT_IN
    sensorValue0 = analogReadFast(BAT_IN);
    battery_voltage = sensorValue0 * VOLTAGE_SCALE;
    dtostrf(battery_voltage, 3, 1, line_buffer);
    
    sprintf(buf, "%i %i %i %i BAT:%sV ", val13, val10, val11, val12, line_buffer);
    ssd1306_printFixed(0, 48, buf, STYLE_NORMAL);
    
    // AUX1_PORT
    sensorValue0 = analogReadFast(AUX1_PORT);
    sprintf(buf, "%i %i %i %i AUX:%04i", val9, val14, val15, val16, sensorValue0);
    ssd1306_printFixed(0, 56, buf, STYLE_NORMAL);

    // TODO: Testing changing mode from here
    if(val14 == 0) {
        txmode = 0;
    } else {
        txmode = 1;
    }

    // Processing data and sending
    txData();
    
    delay(1);
}

/*
 *  Sending Data to Remote. This will be changed once confirm the basic version is running
 */
void txData() {
    data.ch1 = throttleValue;
    data.ch2 = rudderValue;
    data.ch3 = aileronValue;
    data.ch4 = elevatorValue;
    data.ch5 = 50;
    data.ch6 = 60;
    data.ch7 = 70;
    data.ch8 = 80;

    // Sending Data
    radio.write(&data, sizeof(MyData));
}

