/*
 * EchoWiiTX by Englebert Lai. 2018-Nov-29.
 */
#include "Arduino.h"
#include "EchoWiiTX.h"
#include "LCD.h"
#include "ssd1306_fonts.h"
#include "ssd1306_1bit.h"

#include <avdweb_AnalogReadFast.h>

// For the Radio Module - NRF24L01
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*
 * Keypad configuration
 */
#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
    {'1','2','3','4'},
    {'5','6','7','8'},
    {'9','A','B','C'},
    {'D','E','F','#'}
};

byte rowPins[ROWS] = {3, 2, 1, 0}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {7, 6, 5, 4}; //connect to the column pinouts of the kpd
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
unsigned long loopCount;
unsigned long startTime;
String msg;


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

// Define menu items
const char *menuItems[] = {
    " RF Scanner",
    " Exit"
};

// Default mode = 0
// MODE:
// 0: Normal TX Mode
// 1: RF Scanning Mode
uint8_t txmode = 0;

SAppMenu menu;

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
uint16_t sensorValue0;
uint16_t throttleValue, rudderValue, elevatorValue, aileronValue;
char buf[32];
char line_buffer[32];

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

    showRFScanMode();

    // Forever here... till the switch position changed.
    while(txmode == 1) {
        scanChannels();
    }
}

void showRFScanMode() {
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(16,  0, "[ 2.4G Scanner ]", STYLE_NORMAL);

    // Preparing outlines...
    ssd1306_drawLine(0, 13, 128, 13);

    ssd1306_drawLine(0, 8, 0, 15);
    ssd1306_drawLine(63, 8, 63, 15);
    ssd1306_drawLine(127, 8, 127, 15);

    ssd1306_drawLine(95, 10, 95, 15);
    ssd1306_drawLine(32, 10, 32, 15);
    ssd1306_printFixed(0,  60, "0        64       128", STYLE_NORMAL);
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
        negativeMode();
        drawLine(i, 55, i, 16);
        // ssd1306_clearBlock(i, 24, 1, 16);
        if(channel_loads > 0) {
            // Using only 38 pixels of y
            // Starting from row 17, ends at row 54
            // Converts the 200 samples into 38
            // y = channel_loads * 38 / 200;
            // Speed up and less calculation
            uint8_t y = channel_loads * 0.19;
            y = 54 - y;;
            // Draw current line
            positiveMode();
            drawLine(i, 54, i, y);    
        }

        readSwitches();

        if(val3 == 1 && val8 == 1) {
            // It needs to reset so to have clean RF transmission configurations
            resetFunc();
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

    msg = "";

    // Initializing Multiplexing Input Pads
    setup_inputs();

    // Reset
    resetData();
    
    // Setup Radio Unit
    setup_radio();
    
    // Initializing OLED and display logo
    LCDInit();
    show_logo();
    delay(2000);

    showHeader();
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
 * Initial stage all input to prevent interrupts
 */
void setup_inputs() {
    DDRD = B11110000;
    // DDRD = B00000000;
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

/*
 * Reading key values
 */
void readSwitches() {
    // Start from 1st Row
    /*
    PORTD = B00010000;
    NOP;
    NOP;
    val1 = (PIND & (1<<PD0))>>PD0;
    val2 = (PIND & (1<<PD1))>>PD1;
    val3 = (PIND & (1<<PD2))>>PD2;
    val4 = (PIND & (1<<PD3))>>PD3;
    PORTD = B00100000;
    NOP;
    NOP;
    val5 = (PIND & (1<<PD0))>>PD0;
    val6 = (PIND & (1<<PD1))>>PD1;
    val7 = (PIND & (1<<PD2))>>PD2;
    val8 = (PIND & (1<<PD3))>>PD3;
    PORTD = B01000000;
    NOP;
    NOP;
    val9 =  (PIND & (1<<PD0))>>PD0;
    val10 = (PIND & (1<<PD1))>>PD1;
    val11 = (PIND & (1<<PD2))>>PD2;
    val12 = (PIND & (1<<PD3))>>PD3;
    PORTD = B10000000;
    NOP;
    NOP;
    val13 = (PIND & (1<<PD0))>>PD0;
    val14 = (PIND & (1<<PD1))>>PD1;
    val15 = (PIND & (1<<PD2))>>PD2;
    val16 = (PIND & (1<<PD3))>>PD3;
    PORTD = B00000000;
    */
    int x = 0;
    val1 = 0;
    val2 = 0;
    val3 = 0;
    val4 = 0;
    val5 = 0;
    val6 = 0;
    val7 = 0;
    val8 = 0;
    val9 = 0;
    val10 = 0;
    val11 = 0;
    val12 = 0;
    val13 = 0;
    val14 = 0;
    val15 = 0;
    val16 = 0;

    if(kpd.getKeys()) {
        for(int i = 0; i < LIST_MAX; i++) {
            if(kpd.key[i].kstate == PRESSED) {
                // msg = msg + kpd.key[i].kchar + "P ";
                sprintf(buf, "%cP", kpd.key[i].kchar);
                if(kpd.key[i].kchar == '4') val4 = 1;
                if(kpd.key[i].kchar == '7') val7 = 1;
                if(kpd.key[i].kchar == 'F') val5 = 1;
                if(kpd.key[i].kchar == 'B') val6 = 1;
                if(kpd.key[i].kchar == '3') val8 = 1;
            } else if(kpd.key[i].kstate == HOLD) {
                // msg = msg + kpd.key[i].kchar + "H ";
                sprintf(buf, "%cH", kpd.key[i].kchar);
            } else if(kpd.key[i].kstate == RELEASED) {
                // msg = msg + kpd.key[i].kchar + "R ";
                sprintf(buf, "%cR", kpd.key[i].kchar);
            } else if(kpd.key[i].kstate == IDLE) {
                // msg = msg + kpd.key[i].kchar + "I ";
                sprintf(buf, "%cI", kpd.key[i].kchar);
            }
            ssd1306_printFixed(x + (i*12), 32, buf, STYLE_NORMAL);
        }
        /*
        if(LIST_MAX == 0) {
            ssd1306_clearBlock(x, 32, 128, 8);
        }
        */
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
    
    /*
    PORTD = B00001000;
    uint8_t y = 0;
    for(int r = 0; r < 4; r++) {
        for(int c = 0; c < 4; c++) {
            PORTD = PORTD << 1;
            NOP;
            switches[c][r] = (PIND & (1<<c))>>c;
        }
        sprintf(buf, "%i %i %i %i", switches[0][r], switches[1][r], switches[2][r], switches[3][r]);
        y = 32 + (r * 8);
        ssd1306_printFixed(0, y, buf, STYLE_NORMAL);
    }
    */
    readSwitches();
    /*
    sprintf(buf, "%i %i %i %i", val1, val2, val3, val4);
    ssd1306_printFixed(0, 32, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", val5, val6, val7, val8);
    ssd1306_printFixed(0, 40, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", val9, val10, val11, val12);
    ssd1306_printFixed(0, 48, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", val13, val14, val15, val16);
    ssd1306_printFixed(0, 56, buf, STYLE_NORMAL);
    */

    // BAT_IN
    sensorValue0 = analogReadFast(BAT_IN);
    battery_voltage = sensorValue0 * VOLTAGE_SCALE;
    dtostrf(battery_voltage, 3, 1, line_buffer);
    sprintf(buf, "BAT:%sV ", line_buffer);
    ssd1306_printFixed(0, 8, buf, STYLE_NORMAL);
    
    // AUX1_PORT
    sensorValue0 = analogReadFast(AUX1_PORT);
    sprintf(buf, "AUX:%04i", sensorValue0);
    ssd1306_printFixed(64, 56, buf, STYLE_NORMAL);

    // Trigger menu if val3 and val8 is true
    if(val4 == 1 && val7 == 1) {
        showMenu();
    }
    /*
    // TODO: Testing changing mode from here
    if(val14 == 0) {
        txmode = 0;
    } else {
        txmode = 1;
    }
    */

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

void showMenu() {
    ssd1306_fillScreen(0x00);
    ssd1306_createMenu(&menu, menuItems, sizeof(menuItems) / sizeof(char *));
    ssd1306_showMenu(&menu);

    while(txmode == 0) {
        readSwitches();

        if(val6 == 1) {
            ssd1306_menuDown(&menu);
            ssd1306_updateMenu(&menu);
        }

        if(val5 == 1) {
            ssd1306_menuUp(&menu);
            ssd1306_updateMenu(&menu);
        }

        if(val8 == 1) {
            uint8_t selectedItem = ssd1306_menuSelection(&menu);
            // Only two items. So is 0 and 1....
            if(selectedItem == 0) {
                txmode = 1;
                break;
            }
            if(selectedItem == 1) {
                showHeader();
                break;
            }
        }
        delay(100);
    }
}

