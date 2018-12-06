/*
 * EchoWiiTX by Englebert Lai. 2018-Nov-29.
 */
#include "Arduino.h"
#include "EchoWiiTX.h"
#include "echowiitx_logo.h"
#include "ssd1306_fonts.h"
#include "ssd1306_1bit.h"
#include "ssd1306.h"

#include <avdweb_AnalogReadFast.h>

// For the Radio Module - NRF24L01
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// For Saving configurations
#include <EEPROM.h>

/*
 * Keypad configuration
 */
#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns

char keys[ROWS][COLS] = {
    {'Z','F','L','3'},
    {'D','A','J','1'},
    {'G','B','K','2'},
    {'H','E','I','5'}
};

byte rowPins[ROWS] = {3, 2, 1, 0}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {7, 6, 5, 4}; //connect to the column pinouts of the kpd
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

uint16_t loopCount;
unsigned long startTime;

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

#define CE           9
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

/* Menu definitions */
#define TXMODE                   0
#define MENU_SETUP               99
#define MENU_SETUP_ELIMIT        2
#define MENU_SETUP_RF_SCANNER    7
#define MENU_SETUP_KEYS_DEBUGGER 8
#define MENU_SETUP_EXIT          10

// Default mode = 0
// MODE:
// 0: Normal TX Mode
// 1: RF Scanning Mode
uint8_t txmode = 0;

uint8_t trimUpdates = 1;

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


// For Debugging Purposes
// #define DEBUG 1

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
    while(txmode == MENU_SETUP_RF_SCANNER) {
        scanChannels();
    }
}

void showRFScanMode() {
    showHeader(16, 0, "[ 2.4G Scanner ]", 0);

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
        ssd1306_negativeMode();
        ssd1306_drawLine(i, 54, i, 16);
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
            ssd1306_positiveMode();
            ssd1306_drawLine(i, 54, i, y);    
        }

        readSwitches();

        if(lgimbal_left == 1 && rgimbal_right == 1) {
            // It needs to reset so to have clean RF transmission configurations
            resetFunc();
            // showHeaderMain();
            // txmode = 0;
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
 * Writting EEPROM in 2 Bytes mode (integer - 16 bits)
 */
void EEPROMWrite16Bits(uint16_t addr, uint16_t value) {
    uint8_t lower_bits = (value & 0xFF);
    uint8_t higher_bits = ((value >> 8) & 0xFF);

    // Writing the 2 bytes into the eeprom memory
    EEPROM.write(addr, lower_bits);
    EEPROM.write(addr + 1, higher_bits);
}

/*
 * Reading EEPROM in 2 Bytes mode (integer - 16 bits)
 */
uint16_t EEPROMRead16Bits(uint16_t addr) {
    uint16_t lower_bits = EEPROM.read(addr);
    uint16_t higher_bits = EEPROM.read(addr + 1);

    // Returning the composed output by using bitshift
    return ((lower_bits << 0) & 0xFF) + ((higher_bits << 8) & 0xFFFF);
}

/*
 * EEPROM checksum functions
 */
uint8_t checksum() {
    uint8_t sum=0x68;                 // checksum init
    for(uint16_t i = 0; i < 16; i++) {
        sum += EEPROM.read(i);
    }
    return sum;
}

// Reading configurations from EEPROM to variables.
void readEEPROM() {
    uint16_t addr = 0;

    // Reading checksum from EEPROM
    eeprom_checksum = EEPROM.read(0x03FF);

    // If checksum wrong then load default else load from it.
    if(eeprom_checksum == checksum()) {
        // Reading the data
        throttle_lower_limit = EEPROMRead16Bits(0x0000);
        throttle_upper_limit = EEPROMRead16Bits(0x0002);
        rudder_lower_limit = EEPROMRead16Bits(0x0004);
        rudder_upper_limit = EEPROMRead16Bits(0x0006);
        elevator_lower_limit = EEPROMRead16Bits(0x0008);
        elevator_upper_limit = EEPROMRead16Bits(0x000A);
        aileron_lower_limit = EEPROMRead16Bits(0x000C);
        aileron_upper_limit = EEPROMRead16Bits(0x000E);
    }
}

/*
 * Save settings to EEPROM
 */
void saveSettings() {
    // Pushing the elimits to EEPROM
    EEPROMWrite16Bits(0x0000, throttle_lower_limit);
    EEPROMWrite16Bits(0x0002, throttle_upper_limit);
    EEPROMWrite16Bits(0x0004, rudder_lower_limit);
    EEPROMWrite16Bits(0x0006, rudder_upper_limit);
    EEPROMWrite16Bits(0x0008, elevator_lower_limit);
    EEPROMWrite16Bits(0x000A, elevator_upper_limit);
    EEPROMWrite16Bits(0x000C, aileron_lower_limit);
    EEPROMWrite16Bits(0x000E, aileron_upper_limit);

    // Getting the checksum and push the the last bit of the EEPROM
    EEPROM.write(0x03FF, checksum());
}

/*
 * Start Up Logo
 */
void showLogo() {
    ssd1306_drawBitmap(0, 2, 128, 45, echowiitx_logo);
    // ssd1306_printFixed(0, 32, "EchoWiiTX", STYLE_NORMAL);
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

    // Reading from EEPROM
    readEEPROM();

    // Reset
    resetData();
    
    // Setup Radio Unit
    setup_radio();
    
    // Initializing OLED and display logo
    // LCDInit();
    ssd1306_128x64_i2c_init();
    // ssd1306_vga_controller_init();
    // sd1306_clearScreen();
    ssd1306_fillScreen(0x00);
    showLogo();
    delay(2000);

    // Initialize keyboard loopCount to zero
    loopCount = 0;

    showHeaderMain();
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
 * Arduino Main Loop
 */
void loop() {
    if(txmode == TXMODE) {
        txMode();
    } else if(txmode == MENU_SETUP_RF_SCANNER) {
        rfscanMode();
    } else if(txmode == MENU_SETUP) {
        setupMode();
    } else if(txmode == MENU_SETUP_KEYS_DEBUGGER) {
        debugKeys();
    } else if(txmode == MENU_SETUP_ELIMIT) {
        elimits();
    }
}

/*
 * Reading key values
 */
void readSwitches() {
    if(kpd.getKeys()) {
        for(int i = 0; i < LIST_MAX; i++) {
            if(kpd.key[i].kstate == PRESSED) {
                // msg = msg + kpd.key[i].kchar + "P ";
                if(kpd.key[i].kchar == 'A') sw_a = 1;
                else if(kpd.key[i].kchar == 'B') sw_b = 1;
                else if(kpd.key[i].kchar == 'D') sw_d = 1;
                else if(kpd.key[i].kchar == 'E') sw_e = 1;
                else if(kpd.key[i].kchar == 'F') sw_f = 1;
                else if(kpd.key[i].kchar == 'G') sw_g = 1;
                else if(kpd.key[i].kchar == 'H') sw_h = 1;
                else if(kpd.key[i].kchar == 'I') rgimbal_up = 1;
                else if(kpd.key[i].kchar == 'J') rgimbal_left = 1;
                else if(kpd.key[i].kchar == 'K') rgimbal_down = 1;
                else if(kpd.key[i].kchar == 'L') rgimbal_right = 1;
                else if(kpd.key[i].kchar == '1') lgimbal_left = 1;
                else if(kpd.key[i].kchar == '2') lgimbal_down = 1;
                else if(kpd.key[i].kchar == '3') lgimbal_right = 1;
                else if(kpd.key[i].kchar == '5') lgimbal_up = 1;
            } else if(kpd.key[i].kstate == HOLD) {
                // msg = msg + kpd.key[i].kchar + "H ";
                if(kpd.key[i].kchar == 'A') sw_a = 9;
                else if(kpd.key[i].kchar == 'B') sw_b = 9;
                else if(kpd.key[i].kchar == 'D') sw_d = 9;
                else if(kpd.key[i].kchar == 'E') sw_e = 9;
                else if(kpd.key[i].kchar == 'F') sw_f = 9;
                else if(kpd.key[i].kchar == 'G') sw_g = 9;
                else if(kpd.key[i].kchar == 'H') sw_h = 9;
                else if(kpd.key[i].kchar == 'I') rgimbal_up = 9;
                else if(kpd.key[i].kchar == 'J') rgimbal_left = 9;
                else if(kpd.key[i].kchar == 'K') rgimbal_down = 9;
                else if(kpd.key[i].kchar == 'L') rgimbal_right = 9;
                else if(kpd.key[i].kchar == '1') lgimbal_left = 9;
                else if(kpd.key[i].kchar == '2') lgimbal_down = 9;
                else if(kpd.key[i].kchar == '3') lgimbal_right = 9;
                else if(kpd.key[i].kchar == '5') lgimbal_up = 9;
            // } else if(kpd.key[i].kstate == RELEASED) {
                // msg = msg + kpd.key[i].kchar + "R ";
                // if(kpd.key[i].kchar == 'A') sw_a = 0;
                // else if(kpd.key[i].kchar == 'B') sw_b = 0;
                // else if(kpd.key[i].kchar == 'D') sw_d = 0;
                // else if(kpd.key[i].kchar == 'E') sw_e = 0;
                // else if(kpd.key[i].kchar == 'F') sw_f = 0;
                // else if(kpd.key[i].kchar == 'G') sw_g = 0;
                // else if(kpd.key[i].kchar == 'H') sw_h = 0;
                // else if(kpd.key[i].kchar == 'I') rgimbal_up = 0;
                // else if(kpd.key[i].kchar == 'J') rgimbal_left = 0;
                // else if(kpd.key[i].kchar == 'K') rgimbal_down = 0;
                // else if(kpd.key[i].kchar == 'L') rgimbal_right = 0;
                // else if(kpd.key[i].kchar == '1') lgimbal_left = 0;
                // else if(kpd.key[i].kchar == '2') lgimbal_down = 0;
                // else if(kpd.key[i].kchar == '3') lgimbal_right = 0;
                // else if(kpd.key[i].kchar == '5') lgimbal_up = 0;
            } else if(kpd.key[i].kstate == IDLE) {
                // msg = msg + kpd.key[i].kchar + "I ";
                if(kpd.key[i].kchar == 'A') sw_a = 0;
                else if(kpd.key[i].kchar == 'B') sw_b = 0;
                else if(kpd.key[i].kchar == 'D') sw_d = 0;
                else if(kpd.key[i].kchar == 'E') sw_e = 0;
                else if(kpd.key[i].kchar == 'F') sw_f = 0;
                else if(kpd.key[i].kchar == 'G') sw_g = 0;
                else if(kpd.key[i].kchar == 'H') sw_h = 0;
                else if(kpd.key[i].kchar == 'I') rgimbal_up = 0;
                else if(kpd.key[i].kchar == 'J') rgimbal_left = 0;
                else if(kpd.key[i].kchar == 'K') rgimbal_down = 0;
                else if(kpd.key[i].kchar == 'L') rgimbal_right = 0;
                else if(kpd.key[i].kchar == '1') lgimbal_left = 0;
                else if(kpd.key[i].kchar == '2') lgimbal_down = 0;
                else if(kpd.key[i].kchar == '3') lgimbal_right = 0;
                else if(kpd.key[i].kchar == '0') lgimbal_up = 0;
            }
        }
    }
}

/*
 * Reading all the analog input data
 */
void readAnalogs() {
    // Gimbal readings
    throttleValue = analogReadFast(THROTTLE_PORT);
    rudderValue = analogReadFast(RUDDER_PORT);
    aileronValue = analogReadFast(AILERON_PORT);
    elevatorValue = analogReadFast(ELEVATOR_PORT);

    // Mapping the values to 1000 - 2000 as standard of the RC values
    throttleValue = map(throttleValue, throttle_lower_limit, throttle_upper_limit, 1000, 2000);
    rudderValue = map(rudderValue, rudder_lower_limit, rudder_upper_limit, 1000, 2000);
    aileronValue = map(aileronValue, aileron_lower_limit, aileron_upper_limit, 1000, 2000);
    elevatorValue = map(elevatorValue, elevator_lower_limit, elevator_upper_limit, 1000, 2000);

    // Battery Input
    batteryVoltageValue = analogReadFast(BAT_IN);

    // Variable Resistor
    channelCValue = analogReadFast(AUX1_PORT);
}

/*
 * Continuos transmission
 */
void txMode() {
    readSwitches();
    readAnalogs();

    // Display the animation and info about the settings
    showGimbals();

    // Trigger menu if lgimbal_right and rgimbal_left
    if(lgimbal_right == 1 && rgimbal_left == 1) {
        txmode = MENU_SETUP;
        showHeaderSetup();
        return;
    }

    // Processing data and sending
    txData();
}

void showGimbals() {
    /*
     *  [|----------][-----|-----]
     *  [-----|-----][-----|-----]
     */
    uint16_t throttleBarValue = (throttleValue - 1000) * 0.016;
    drawBars(0, 17, 17, throttleBarValue);
    uint16_t rudderBarValue = (rudderValue - 1000)* 0.016;
    drawBars(0, 25, 17, rudderBarValue);
    uint16_t elevatorBarValue = (elevatorValue - 1000) * 0.016;
    drawBars(0, 33, 17, elevatorBarValue);
    uint16_t aileronBarValue = (aileronValue - 1000)* 0.016;
    drawBars(0, 41, 17, aileronBarValue);
}

void drawBars(uint8_t x, uint8_t y, uint8_t max_bars, uint8_t bar_position) {
    sprintf(buf, "T", throttleValue, rudderValue);

    ssd1306_printFixed(x, y, "[", STYLE_NORMAL);
    x+=6;

    for(int i = 0; i < max_bars; i++) {
        if(i != bar_position)
            ssd1306_printFixed(x + (i * 6), y, "-", STYLE_NORMAL);
        else
            ssd1306_printFixed(x + (i * 6), y, "|", STYLE_NORMAL);
    }
    
    ssd1306_printFixed(x + (max_bars * 6), y, "]", STYLE_NORMAL);
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

/*
 * Just to skip some processes
 */
uint8_t shortDelay() {
    if(loopCount != 0) {
        loopCount++;
        if(loopCount >= 15000) {
            loopCount = 0;
            return 1;
        }
    } else {
        loopCount++;
    }
    return 0;
}

/*
 * Setup Menu
 */
uint8_t current_setup_page = 0;
uint8_t current_setup_selected = 0;
uint8_t setup_menu_refreshed = 0;

const char *menuItemsSetup[] = {
    "Model Setup",
    "Timers",
    "E.Limits",
    "E.Trim",
    "Reverse",
    "Mapping",
    "Expo",
    "RF Scanner",
    "Keys Debugger",
    "About",
    "Exit"
};


uint8_t max_items_setup = (sizeof(menuItemsSetup) / sizeof(char *)) - 1;
uint8_t max_page_setup = max_items_setup - 5;

void setupMode() {
    readSwitches();

    // Skip for a while
    if(shortDelay() == 0) return;

    // Refreshing screen
    if(rgimbal_up == 1 || rgimbal_down == 1) {
        setup_menu_refreshed = 0;
    }
    
    // Cursor calculation
    uint8_t y;
    if(rgimbal_down == 1) {
        current_setup_selected++;

        // Limiter
        if(current_setup_selected >= max_items_setup)
            current_setup_selected = max_items_setup;

        if(current_setup_selected > 5) {
            current_setup_page++;
        }

        if(current_setup_page > max_page_setup) current_setup_page = max_page_setup;
    } else if(rgimbal_up == 1) {
        if(current_setup_selected > 0)
            current_setup_selected--;

        if(current_setup_selected < 5 && current_setup_page != 0) {
            current_setup_page--;
        }

    // Menu selected and check which is being selected.
    } else if(rgimbal_right == 1) {
        // Prevent double enter
        rgimbal_right = 0;

        setup_menu_refreshed = 0;
        if(current_setup_selected == MENU_SETUP_EXIT) {
            // current_setup_selected = 0;
            // current_setup_page = 0;
            txmode = TXMODE;
            showHeaderMain();
            return;
        } else if(current_setup_selected == MENU_SETUP_ELIMIT) {
            txmode = MENU_SETUP_ELIMIT;
            showHeaderELimits();
            return;
        } else if(current_setup_selected == MENU_SETUP_KEYS_DEBUGGER) {
            txmode = MENU_SETUP_KEYS_DEBUGGER;
            showHeaderDebug();
            return;
        } else if(current_setup_selected == MENU_SETUP_RF_SCANNER) {
            txmode = MENU_SETUP_RF_SCANNER;
            return;
        }
    }

    if(setup_menu_refreshed == 1) return;

    // Based on pages show the menus
    uint8_t max_setup_items = current_setup_page + 5;
    uint8_t min_setup_items = current_setup_page;
    uint8_t j = 0;

    for(uint8_t i = min_setup_items; i <= max_setup_items; i++) {
        // Clear previous line
        ssd1306_printFixed(0, 17 + (j * 8), "               ", STYLE_NORMAL);
        // Print Menu
        ssd1306_printFixed(8, 17 + (j * 8), menuItemsSetup[i], STYLE_NORMAL);
        // Print cursor
        if(i == current_setup_selected) {
            ssd1306_printFixed(0, 17 + (j * 8), ">", STYLE_NORMAL);
        }
        j++;
    }

    setup_menu_refreshed = 1;
}

/*
 * Setting E-Limits on both of the gimbals
 */
void elimits() {
    readSwitches();
    readAnalogs();

    // Gimbal readings - Not using the common functions since this not requires mapping
    throttleValue = analogReadFast(THROTTLE_PORT);
    rudderValue = analogReadFast(RUDDER_PORT);
    aileronValue = analogReadFast(AILERON_PORT);
    elevatorValue = analogReadFast(ELEVATOR_PORT);

    ssd1306_printFixed(0, 17, "UP: Reset Right: Save", STYLE_NORMAL);

    // Getting the min and max values of each sticks
    if(throttle_upper_limit < throttleValue) throttle_upper_limit = throttleValue;
    if(throttle_lower_limit > throttleValue) throttle_lower_limit = throttleValue;
    if(rudder_upper_limit < rudderValue) rudder_upper_limit = rudderValue;
    if(rudder_lower_limit > rudderValue) rudder_lower_limit = rudderValue;
    if(elevator_upper_limit < elevatorValue) elevator_upper_limit = elevatorValue;
    if(elevator_lower_limit > elevatorValue) elevator_lower_limit = elevatorValue;
    if(aileron_upper_limit < aileronValue) aileron_upper_limit = aileronValue;
    if(aileron_lower_limit > aileronValue) aileron_lower_limit = aileronValue;

    uint8_t x = 8;

    sprintf(buf, "TL:%04i TU:%04i", throttle_lower_limit, throttle_upper_limit);
    ssd1306_printFixed(x, 32, buf, STYLE_NORMAL);
    sprintf(buf, "RL:%04i RU:%04i", rudder_lower_limit, rudder_upper_limit);
    ssd1306_printFixed(x, 40, buf, STYLE_NORMAL);
    sprintf(buf, "EL:%04i EU:%04i", elevator_lower_limit, elevator_upper_limit);
    ssd1306_printFixed(x, 48, buf, STYLE_NORMAL);
    sprintf(buf, "AL:%04i AU:%04i", aileron_lower_limit, aileron_upper_limit);
    ssd1306_printFixed(x, 56, buf, STYLE_NORMAL);

    // Save settings
    if(rgimbal_right == 1) {
        saveSettings();

        // Return to main menu
        txmode = MENU_SETUP;
        showHeaderSetup();
        return;
    }

    // Default elimits
    if(rgimbal_up == 1) {
         throttle_upper_limit = 600;
         throttle_lower_limit = 300;
         rudder_upper_limit = 600;
         rudder_lower_limit = 300;
         elevator_upper_limit = 600;
         elevator_lower_limit = 300;
         aileron_upper_limit = 600;
         aileron_lower_limit = 300;
    }   
    
}

/*
 *  Debug keys
 */
void debugKeys() {
    readSwitches();
    readAnalogs();

    sprintf(buf, "T:%04i R:%04i", throttleValue, rudderValue);
    ssd1306_printFixed(0, 16, buf, STYLE_NORMAL);

    sprintf(buf, "A:%04i E:%04i", aileronValue, elevatorValue);
    ssd1306_printFixed(0, 24, buf, STYLE_NORMAL);
  
    // Battery Calculation
    float batteryVoltage = batteryVoltageValue * VOLTAGE_SCALE;
    dtostrf(batteryVoltage, 3, 1, line_buffer);
    sprintf(buf, "BAT:%sV ", line_buffer);
    ssd1306_printFixed(64, 32, buf, STYLE_NORMAL);
    
    // AUX1_PORT
    sprintf(buf, "AUX:%04i", channelCValue);
    ssd1306_printFixed(64, 56, buf, STYLE_NORMAL);

    sprintf(buf, "%i %i   %i", sw_a, sw_b, sw_d);
    ssd1306_printFixed(0, 32, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", sw_e, sw_f, sw_g, sw_h);
    ssd1306_printFixed(0, 40, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", lgimbal_up, lgimbal_down, lgimbal_left, lgimbal_right);
    ssd1306_printFixed(0, 48, buf, STYLE_NORMAL);
    sprintf(buf, "%i %i %i %i", rgimbal_up, rgimbal_down, rgimbal_left, rgimbal_right);
    ssd1306_printFixed(0, 56, buf, STYLE_NORMAL);

    detectExit();
}

void detectExit() {
    if(rgimbal_right == 1 && lgimbal_left == 1) {
        txmode = MENU_SETUP;
        showHeaderSetup();
        return;
    }
}

/*
 * Headers of the Menu
 */
void showHeaderELimits() {
    showHeader(0, 0, "E.LIMITS", 1);
}

void showHeaderSetup() {
    showHeader(0, 0, "SETUP", 1);
}

void showHeaderMain() {
    showHeader(0, 0, "ECHOWII TX", 1);
}

void showHeaderDebug() {
    showHeader(0, 0, "[ DEBUG MODE ]", 0);
}

void showHeader(uint8_t x, uint8_t y, const char *title, uint8_t style) {
    ssd1306_fillScreen(0x00);
    if(style == 0) {
        ssd1306_setFixedFont(ssd1306xled_font6x8);
        ssd1306_printFixed(x, y, title, STYLE_NORMAL);
    } else if(style == 1) {
        ssd1306_setFixedFont(ssd1306xled_font6x8);
        ssd1306_printFixedN(x, y, title, STYLE_NORMAL, 1);
    }
}
