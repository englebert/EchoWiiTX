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

/* EEPROM RELATED */
#define EEPROM_MAXADDR              22
#define EEPROM_THROTTLE_LIMIT_ADDR  0x0000
#define EEPROM_YAW_LIMIT_ADDR       0x0004
#define EEPROM_PITCH_LIMIT_ADDR     0x0008
#define EEPROM_ROLL_LIMIT_ADDR      0x000C
#define EEPROM_REVERSE_ADDR         0x0010
#define EEPROM_AUX2SWITCH_ADDR      0x0011
#define EEPROM_AUX3SWITCH_ADDR      0x0012
#define EEPROM_AUX4SWITCH_ADDR      0x0013
#define EEPROM_RXAUX1SWITCH_ADDR    0x0014
#define EEPROM_IDLEUP_ADDR          0x0015

#define EEPROM_CHECKSUM_ADDR        0x03FF

/* Save Settings Definition */
#define ELIMIT          1
#define REVERSE         2
#define AUXSWITCHES     3
#define AUXRXSWITCHES   4
#define IDLEUPSWITCHES  5

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

// For the Switch A ~ Switch I
#define SWITCH_MIN_VALUE    0x40
#define SWITCH_MAX_VALUE    0x49
#define DASH                0x2D

// Use for storing the data of the channel load
int channel_loads;

struct MyData {
    uint8_t ch1;
    uint8_t ch2;
    uint8_t ch3;
    uint8_t ch4;
    uint8_t ch5;
    uint8_t ch6;
    uint8_t ch7;
};
MyData data;

// Default mode = 0
// MODE:
// 0: Normal TX Mode
// 1: RF Scanning Mode
uint8_t txmode = 0;

uint8_t trimUpdates = 1;

// Total Cycle TX
uint16_t txTotalPackets = 0;
uint16_t txPackets = 0;
uint8_t interrupt_ticks = 0;

// Prevent slows down during refresh
uint8_t oled_refresh = 0;

/**
 * Configurations and Values
 */

// Joysticks
#define THROTTLE_PORT A0
#define YAW_PORT   A1
#define PITCH_PORT A3
#define ROLL_PORT  A2

// Analogue Ports
#define AUX1_PORT     A6
#define BAT_IN        A7

// Voltage Scaling Value
#define VOLTAGE_SCALE 0.04878

// PRESCALE TIMER SETTINGS
#define PRELOAD_TIMER 0x85EE    // 34286 - 2Hz

// For Debugging Purposes
// #define DEBUG 1

// Receiver Backlight
uint8_t backlight_enable = 0;

// COMMAND BITS DEFINES
#define CMD_BACKLIGHT 0x01
#define BACKLIGHT_ENABLE 1
#define BACKLIGHT_DISABLE 0

char buf[32];
char line_buffer[32];

/*
 * Just to skip some processes
 */
uint8_t shortDelay(uint16_t max_count) {
    if(loopCount != 0) {
        loopCount++;
        if(loopCount >= max_count) {
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
    "Model Setup",          // 0
    "Timers",               // 1
    "E.Limits",             // 2
    "E.Trim",               // 3
    "Reverse",              // 4
    "FC Mapping",           // 5
    "RX Mapping",           // 6
    "IdleUp Setup",         // 7
    "RF Scanner",           // 8
    "Keys Debugger",        // 9
    "Exit"                  // 10
};

/* Menu definitions -- START */
#define TXMODE                      0
#define MENU_SETUP                  99
#define MENU_SETUP_TIMER            1
#define MENU_SETUP_ELIMIT           2
#define MENU_SETUP_REVERSE          4
#define MENU_SETUP_FC_MAPPING       5
#define MENU_SETUP_RX_MAPPING       6
#define MENU_SETUP_IDLEUP_MAPPING   7
#define MENU_SETUP_RF_SCANNER       8
#define MENU_SETUP_KEYS_DEBUGGER    9
#define MENU_SETUP_EXIT             10
/* Menu definitions -- END   */

uint8_t max_items_setup = (sizeof(menuItemsSetup) / sizeof(char *)) - 1;
uint8_t max_page_setup = max_items_setup - 5;

/*
 * Reverse Menu
 */
uint8_t current_reverse_selected = 0;
uint8_t reverse_menu_refreshed = 0;

/*
 * Mapping Menu
 */
uint8_t current_mapping_selected = 0;
uint8_t mapping_menu_refreshed = 0;

/*
 * RX Mapping Menu
 */
uint8_t current_rx_mapping_selected = 0;
uint8_t rxmapping_menu_refreshed = 0;

/*
 * IdleUp Mapping Menu
 */
uint8_t current_idleup_mapping_selected = 0;
uint8_t idleupmapping_menu_refreshed = 0;

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
    // showHeader(16, 0, "[ 2.4G Scanner ]", 0);

    // Preparing outlines...
    // ssd1306_drawLine(0, 13, 128, 13);
    // Display Header 
    showHeader(0, 0, "RF SCANNER", 1);

    // ssd1306_drawLine(0, 8, 0, 15);
    // ssd1306_drawLine(63, 8, 63, 15);
    // ssd1306_drawLine(127, 8, 127, 15);

    // ssd1306_drawLine(95, 10, 95, 15);
    // ssd1306_drawLine(32, 10, 32, 15);
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
    for(uint16_t i = 0; i < EEPROM_MAXADDR; i++) {
        sum += EEPROM.read(i);
    }
    return sum;
}

// Reading configurations from EEPROM to variables.
void readEEPROM() {
    uint16_t addr = 0;

    // Reading checksum from EEPROM
    eeprom_checksum = EEPROM.read(EEPROM_CHECKSUM_ADDR);

    // If checksum wrong then load default else load from it.
    if(eeprom_checksum == checksum()) {
        // Reading the data
        throttle_lower_limit = EEPROMRead16Bits(EEPROM_THROTTLE_LIMIT_ADDR);
        throttle_upper_limit = EEPROMRead16Bits(EEPROM_THROTTLE_LIMIT_ADDR + 2);
        yaw_lower_limit = EEPROMRead16Bits(EEPROM_YAW_LIMIT_ADDR);
        yaw_upper_limit = EEPROMRead16Bits(EEPROM_YAW_LIMIT_ADDR + 2);
        pitch_lower_limit = EEPROMRead16Bits(EEPROM_PITCH_LIMIT_ADDR);
        pitch_upper_limit = EEPROMRead16Bits(EEPROM_PITCH_LIMIT_ADDR + 2);
        roll_lower_limit = EEPROMRead16Bits(EEPROM_ROLL_LIMIT_ADDR);
        roll_upper_limit = EEPROMRead16Bits(EEPROM_ROLL_LIMIT_ADDR + 2);

        // Analog Input Reverse data
        /* EEPROM addr: 0x0010
           Bits |7|6|5|4|3|2|1|0|
                 - - - C R Y P T
           T: Throttle
           P: Pitch
           Y: Yaw
           R: Roll
           C: Channel C  */
        reverseBits = EEPROM.read(EEPROM_REVERSE_ADDR);

        // Assigning values
        reverse_throttle = reverseBits & B00000001;
        reverse_pitch = (reverseBits & B00000010) >> 1;
        reverse_yaw = (reverseBits & B00000100) >> 2;
        reverse_roll = (reverseBits & B00001000) >> 3;
        reverse_channelc = (reverseBits & B00010000) >> 4;

        // Reading Auxilary Ports For FC Channel assignments
        for(uint8_t i = 0; i < AUXMAX; i++) {
            // Aux2, Aux3 and Aux4 Port values
            portAUX[i] = EEPROM.read(EEPROM_AUX2SWITCH_ADDR + i);

            // Default minimum values
            if(portAUX[i] < SWITCH_MIN_VALUE)
                portAUX[i] = SWITCH_MIN_VALUE;
        }

        // Reading Auxiliary Ports For RX Channel assignments
        for(uint8_t i = 0; i < RXAUXMAX; i++) {
            // Aux2, Aux3 and Aux4 Port values 
            portRXAUX[i] = EEPROM.read(EEPROM_RXAUX1SWITCH_ADDR + i);

            // Default minimum values
            if(portRXAUX[i] < SWITCH_MIN_VALUE)
                portRXAUX[i] = SWITCH_MIN_VALUE;
        }

        // Reading Idle UP
        idleUpThrottle[0] = EEPROM.read(EEPROM_IDLEUP_ADDR);
        
    } else {
        // Defaults 
        throttle_lower_limit = 400;
        throttle_upper_limit = 400;
        yaw_lower_limit = 400;
        yaw_upper_limit = 400;
        pitch_lower_limit = 400;
        pitch_upper_limit = 400;
        roll_lower_limit = 400;
        roll_upper_limit = 400;
        reverse_throttle = 0;
        reverse_pitch = 0;
        reverse_yaw = 0;
        reverse_roll = 0;
        reverse_channelc = 0;
        portAUX[0] = SWITCH_MIN_VALUE;          // The - sign
        portAUX[1] = SWITCH_MIN_VALUE;          // The - sign
        portAUX[2] = SWITCH_MIN_VALUE;          // The - sign
        portRXAUX[0] = SWITCH_MIN_VALUE;        // The - sign
        idleUpThrottle[0] = SWITCH_MIN_VALUE;   // The - sign
    }
}

/*
 * Save settings to EEPROM
 */
void saveSettings(uint8_t settings_type) {
    // Pushing the elimits to EEPROM
    if(settings_type == ELIMIT) {
        EEPROMWrite16Bits(EEPROM_THROTTLE_LIMIT_ADDR, throttle_lower_limit);
        EEPROMWrite16Bits(EEPROM_THROTTLE_LIMIT_ADDR + 2, throttle_upper_limit);
        EEPROMWrite16Bits(EEPROM_YAW_LIMIT_ADDR, yaw_lower_limit);
        EEPROMWrite16Bits(EEPROM_YAW_LIMIT_ADDR + 2, yaw_upper_limit);
        EEPROMWrite16Bits(EEPROM_PITCH_LIMIT_ADDR, pitch_lower_limit);
        EEPROMWrite16Bits(EEPROM_PITCH_LIMIT_ADDR + 2, pitch_upper_limit);
        EEPROMWrite16Bits(EEPROM_ROLL_LIMIT_ADDR, roll_lower_limit);
        EEPROMWrite16Bits(EEPROM_ROLL_LIMIT_ADDR + 2, roll_upper_limit);

    // Pushing the reverse settings to EEPROM
    } else if(settings_type == REVERSE) {
        reverseBits = reverse_throttle +
                    (reverse_pitch << 1) +
                    (reverse_yaw << 2) +
                    (reverse_roll << 3) +
                    (reverse_channelc << 4);

        EEPROM.write(EEPROM_REVERSE_ADDR, reverseBits);

    // Settings AUX switches to EEPROM
    } else if(settings_type == AUXSWITCHES) {
        for(uint8_t i = 0; i < AUXMAX; i++) {
            EEPROM.write(EEPROM_AUX2SWITCH_ADDR + i, portAUX[i]);
        }

    // Settings for RX AUX to EEPROM
    } else if(settings_type == AUXRXSWITCHES) {
        for(uint8_t i = 0; i < RXAUXMAX; i++) {
            EEPROM.write(EEPROM_RXAUX1SWITCH_ADDR + i, portRXAUX[i]);
        }
    } else if(settings_type == IDLEUPSWITCHES) {
        for(uint8_t i = 0; i < IDLEUPMAX; i++) {
            EEPROM.write(EEPROM_IDLEUP_ADDR + i, idleUpThrottle[i]);
        }
    }

    // Getting the checksum and push the the last bit of the EEPROM
    EEPROM.write(EEPROM_CHECKSUM_ADDR, checksum());
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

    // Reset
    resetData();

    // Reading from EEPROM
    readEEPROM();

    // Setup Timers
    setupTimers();
    
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

/*
 * Configuring basic 1 second interrupt for timers and other related used
 */
void setupTimers() {
    // Initialize internal timer1
    // Disables all the interrupts
    cli();
    TCCR1A = 0;                 // set entire TCCR1A register to 0
    TCCR1B = 0;                 // set entire TCCR1A register to 0
    
    // use 64886 preload timer for -> 65536-16MHz/256/100Hz
    // use 64286 preload timer for -> 65536-16MHz/256/50Hz
    // use 34286 preload timer for -> 65536-16MHz/256/2Hz
    TCNT1 = PRELOAD_TIMER;             // Preload Timer

    // Set CS10 but so timer runs at clock speed. (No prescaling)
    TCCR1B |= (1 << CS12);      // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);     // enable timer overflow interrupt

    sei();                      // enable all interrupts
}

// Timer interrupt
ISR(TIMER1_OVF_vect) {
    // Do all the calculations and timing related process
    if(interrupt_ticks < 2) {
        // Increase ....
        interrupt_ticks++;

        // Allow display to refresh. Too frequent refresh causing slowness
        oled_refresh = 1;
    } else {
        // Time for calculation
        txTotalPackets = txPackets;

        // Reset it.
        txPackets = 0;

        // Reset to 0 for next cycle
        interrupt_ticks = 0;
    } 
    
    // Reload Timer
    TCNT1 = PRELOAD_TIMER;
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
    } else if(txmode == MENU_SETUP_REVERSE) {
        reverse();
    } else if(txmode == MENU_SETUP_TIMER) {
        runTimers();
    } else if(txmode == MENU_SETUP_FC_MAPPING) {
        mapping();
    } else if(txmode == MENU_SETUP_RX_MAPPING) {
        rxmapping();
    } else if(txmode == MENU_SETUP_IDLEUP_MAPPING) {
        idleup();
    }
}

/*
 * idleup Mapping
 */
void idleup() {
    readSwitches();

    // Skip for a while
    if(shortDelay(15000) == 0) return;

    // Display all the values and then let the user to select 
    // and pick. May involve Up, Down, Left and Right for selection.    
    const char *menuItemsIdleUp[] = {
        "Channel",
        "Exit"
    };

    uint8_t max_items_mapping = (sizeof(menuItemsIdleUp) / sizeof(char *)) - 1;

    // If Down switch pressed, select another menu
    if(rgimbal_down == 1) {
        if(current_idleup_mapping_selected < max_items_mapping)
            current_idleup_mapping_selected++;
        idleupmapping_menu_refreshed = 1;
    } else if(rgimbal_up == 1) {
        if(current_idleup_mapping_selected > 0)
            current_idleup_mapping_selected--;
        idleupmapping_menu_refreshed = 1;
    } else if(rgimbal_right == 1 || rgimbal_left == 1) {
        // If is the last item then exit back to the menu
        if(current_idleup_mapping_selected == max_items_mapping) {
            txmode = MENU_SETUP;
            showHeaderSetup();
            return;
        }

        // IDLE 1 Channel
        if(rgimbal_right == 1) {
            if(idleUpThrottle[current_idleup_mapping_selected] < SWITCH_MAX_VALUE)
                idleUpThrottle[current_idleup_mapping_selected]++;

            // Skip C which is not using
            if(idleUpThrottle[current_idleup_mapping_selected] == 67)
                idleUpThrottle[current_idleup_mapping_selected]++;


        } else if(rgimbal_left == 1) {
            if(idleUpThrottle[current_idleup_mapping_selected] > SWITCH_MIN_VALUE)
                idleUpThrottle[current_idleup_mapping_selected]--;

            // Skip C which is not using
            if(idleUpThrottle[current_idleup_mapping_selected] == 67)
                idleUpThrottle[current_idleup_mapping_selected]--;
        }
        
        // Refresh menu
        idleupmapping_menu_refreshed = 1;

        // Save EEPROM settings - IDELUP SWITCHES configuration only
        saveSettings(IDLEUPSWITCHES);

    }

    // Skip from here if no need to refresh screen.
    if(idleupmapping_menu_refreshed == 0) return;

    // Processing the menu also at the same time put a symbol if is toggled
    uint8_t j = 0;
    uint8_t y = 0;
    uint8_t c = 0;

    for(uint8_t i = 0; i <= max_items_mapping; i++) {
        // Y position of the menu
        y = 17 + (j * 8);

        // Clear previous line
        ssd1306_printFixed(0, y, "                 ", STYLE_NORMAL);

        // Print Menu
        ssd1306_printFixed(8, y, menuItemsIdleUp[i], STYLE_NORMAL);

        // Print Current Toggle Value
        if(i != max_items_mapping) {
            if(idleUpThrottle[i] == SWITCH_MIN_VALUE) {
                // ASCII CODE: -
                c = 45;
            } else {
                c = idleUpThrottle[i];
            }

            sprintf(buf, "[%c]", c);
            ssd1306_printFixed(64, y, buf, STYLE_NORMAL);
        }
        
        // Print cursor
        if(i == current_idleup_mapping_selected) {
            ssd1306_printFixed(0, y, ">", STYLE_NORMAL);
        }
        j++;
    }

    // Prevent menu refreshed
    idleupmapping_menu_refreshed = 0;
}

/*
 * RX Mapping keys
 */
void rxmapping() {
    readSwitches();

    // Skip for a while
    if(shortDelay(15000) == 0) return;

    // Display all the reversable sticks values and then let the user to select 
    // and pick. May involve Up, Down, Left and Right for selection.    
    const char *menuItemsMapping[] = {
        "RXAUX1",
        "Exit"
    };

    uint8_t max_items_mapping = (sizeof(menuItemsMapping) / sizeof(char *)) - 1;

    // If Down switch pressed, select another menu
    if(rgimbal_down == 1) {
        if(current_rx_mapping_selected < max_items_mapping)
            current_rx_mapping_selected++;
        rxmapping_menu_refreshed = 1;
    } else if(rgimbal_up == 1) {
        if(current_rx_mapping_selected > 0)
            current_rx_mapping_selected--;
        rxmapping_menu_refreshed = 1;
    } else if(rgimbal_right == 1 || rgimbal_left == 1) {
        // If is the last item then exit back to the menu
        if(current_rx_mapping_selected == max_items_mapping) {
            txmode = MENU_SETUP;
            showHeaderSetup();
            return;
        }

        // RX AUXILIARY 1...
        if(rgimbal_right == 1) {
            if(portRXAUX[current_rx_mapping_selected] < SWITCH_MAX_VALUE)
                portRXAUX[current_rx_mapping_selected]++;

            // Skip C which is not using
            if(portRXAUX[current_rx_mapping_selected] == 67)
                portRXAUX[current_rx_mapping_selected]++;


        } else if(rgimbal_left == 1) {
            if(portRXAUX[current_rx_mapping_selected] > SWITCH_MIN_VALUE)
                portRXAUX[current_rx_mapping_selected]--;

            // Skip C which is not using
            if(portRXAUX[current_rx_mapping_selected] == 67)
                portRXAUX[current_rx_mapping_selected]--;
        }
        
        // Refresh menu
        rxmapping_menu_refreshed = 1;

        // Save EEPROM settings - AUXRXSWITCHES configuration only
        saveSettings(AUXRXSWITCHES);

    }

    // Skip from here if no need to refresh screen.
    if(rxmapping_menu_refreshed == 0) return;

    // Processing the menu also at the same time put a symbol if is toggled
    uint8_t j = 0;
    uint8_t y = 0;
    uint8_t c = 0;

    for(uint8_t i = 0; i <= max_items_mapping; i++) {
        // Y position of the menu
        y = 17 + (j * 8);

        // Clear previous line
        ssd1306_printFixed(0, y, "               ", STYLE_NORMAL);

        // Print Menu
        ssd1306_printFixed(8, y, menuItemsMapping[i], STYLE_NORMAL);

        // Print Current Toggle Value
        if(i != max_items_mapping) {
            if(portRXAUX[i] == SWITCH_MIN_VALUE) {
                // ASCII CODE: -
                c = 45;
            } else {
                c = portRXAUX[i];
            }

            sprintf(buf, "[%c]", c);
            ssd1306_printFixed(64, y, buf, STYLE_NORMAL);
        }
        
        // Print cursor
        if(i == current_rx_mapping_selected) {
            ssd1306_printFixed(0, y, ">", STYLE_NORMAL);
        }
        j++;
    }

    // Prevent menu refreshed
    rxmapping_menu_refreshed = 0;
}

/*
 * Mapping keys
 */
void mapping() {
    readSwitches();

    // Skip for a while
    if(shortDelay(15000) == 0) return;

    // Display all the reversable sticks values and then let the user to select 
    // and pick. May involve Up, Down, Left and Right for selection.    
    const char *menuItemsMapping[] = {
        "AUX2",
        "AUX3",
        "AUX4",
        "Exit"
    };

    uint8_t max_items_mapping = (sizeof(menuItemsMapping) / sizeof(char *)) - 1;

    // If Down switch pressed, select another menu
    if(rgimbal_down == 1) {
        if(current_mapping_selected < max_items_mapping)
            current_mapping_selected++;
        mapping_menu_refreshed = 1;
    } else if(rgimbal_up == 1) {
        if(current_mapping_selected > 0)
            current_mapping_selected--;
        mapping_menu_refreshed = 1;
    } else if(rgimbal_right == 1 || rgimbal_left == 1) {
        // If is the last item then exit back to the menu
        if(current_mapping_selected == max_items_mapping) {
            txmode = MENU_SETUP;
            showHeaderSetup();
            return;
        }

        // AUXILIARY CHANNEL 2 till CHANNEL 4
        if(rgimbal_right == 1) {
            if(portAUX[current_mapping_selected] < SWITCH_MAX_VALUE)
                portAUX[current_mapping_selected]++;

            // Skip C which is not using
            if(portAUX[current_mapping_selected] == 67)
                portAUX[current_mapping_selected]++;


        } else if(rgimbal_left == 1) {
            if(portAUX[current_mapping_selected] > SWITCH_MIN_VALUE)
                portAUX[current_mapping_selected]--;

            // Skip C which is not using
            if(portAUX[current_mapping_selected] == 67)
                portAUX[current_mapping_selected]--;
        }
        
        // Refresh menu
        mapping_menu_refreshed = 1;

        // Save EEPROM settings - AUXSWITCHES configuration only
        saveSettings(AUXSWITCHES);

    }

    // Skip from here if no need to refresh screen.
    if(mapping_menu_refreshed == 0) return;

    // Processing the menu also at the same time put a symbol if is toggled
    uint8_t j = 0;
    uint8_t y = 0;
    uint8_t c = 0;

    for(uint8_t i = 0; i <= max_items_mapping; i++) {
        // Y position of the menu
        y = 17 + (j * 8);

        // Clear previous line
        ssd1306_printFixed(0, y, "               ", STYLE_NORMAL);

        // Print Menu
        ssd1306_printFixed(8, y, menuItemsMapping[i], STYLE_NORMAL);

        // Print Current Toggle Value
        if(i != max_items_mapping) {
            if(portAUX[i] == SWITCH_MIN_VALUE) {
                // ASCII CODE: -
                c = 45;
            } else {
                c = portAUX[i];
            }

            sprintf(buf, "[%c]", c);
            ssd1306_printFixed(64, y, buf, STYLE_NORMAL);
        }
        
        // Print cursor
        if(i == current_mapping_selected) {
            ssd1306_printFixed(0, y, ">", STYLE_NORMAL);
        }
        j++;
    }

    // Prevent menu refreshed
    mapping_menu_refreshed = 0;
}

/*
 * Configuring Timers
 */
void runTimers() {
// TODO: Later continue
    readSwitches();
    
    if(isPressed('A') == true) {   
        ssd1306_printFixed(0, 16, "A PRESSED ", STYLE_NORMAL);
    } else {
        ssd1306_printFixed(0, 16, "A RELEASED", STYLE_NORMAL);
    }

    detectExit();
}

/*
 * Detect Key Press
 */
uint8_t isPressed(char keyChar) {
    for(uint8_t i=0; i<LIST_MAX; i++) {
        if(kpd.key[i].kchar == keyChar) {
            if((kpd.key[i].kstate == PRESSED || kpd.key[i].kstate == HOLD))
                return true;
        }
    }
    return false;	// Not pressed.
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
                else if(kpd.key[i].kchar == '5') lgimbal_up = 0;
            }
        }
    }
}

/*
 * Reading Mapping Auxiliary switch positions
 */
void readAux() {
    // Reset value first
    ch6Value = 0;

    // Loop through the values and assign
    for(uint8_t i = 0; i < 3; i++) {
        if(isPressed(portAUX[i]) == true) {
            ch6Value = ch6Value | (1 << (7 - i));
        }
    }

    // TODO:
    // Queue command system. So it will based on queue and sending the command to remote
}

/*
 * Reading all the analog input data
 */
void readAnalogs() {
    // Gimbal readings
    throttleValue = analogReadFast(THROTTLE_PORT);
    yawValue = analogReadFast(YAW_PORT);
    rollValue = analogReadFast(ROLL_PORT);
    pitchValue = analogReadFast(PITCH_PORT);

    // If idleUpThrottle is trigger then throttleValue + 50 but the throttle value must be around 1200 before the function 
    // is applied
    if(isPressed(idleUpThrottle[0]) == true) {
        throttleValue += 50;

        // Prevent over shoot
        if(throttleValue > throttle_upper_limit) throttleValue = throttle_upper_limit;
    }

    // Mapping the values to 1000 - 2000 as standard of the RC values
    // Mapping the values to 0 - 255 from 0 - 1024. It seems transmitting 16 bits a bit lag on
    // responds. Moving back to 8 bits and see what is the result on 
    // Multiwii GUI.
    throttleValue = map(throttleValue, throttle_lower_limit, throttle_upper_limit, 0, 255);
    yawValue = map(yawValue, yaw_lower_limit, yaw_upper_limit, 0, 255);
    rollValue = map(rollValue, roll_lower_limit, roll_upper_limit, 0, 255);
    pitchValue = map(pitchValue, pitch_lower_limit, pitch_upper_limit, 0, 255);

    // Variable Resistor
    channelCValue = analogReadFast(AUX1_PORT);
    channelCValue = map(channelCValue, 0, 1024, 0, 255);

    // Auxilary 2, 3 ad 4
    // These 3 channels a bit tricky. Need to add into the bits before tx.
    // TODO:
    // Channel Mapping: Using the char value to determine the channels mapping.
    // e.g.:
    // Channel B: B = ascii code 98. So, 98 will stored to aux1 value in the eeprom for the switch status
    // mapping and etc.

    // Reverse Channels if flag is set
    if(reverse_throttle == 1) throttleValue = 255 - throttleValue;
    if(reverse_yaw == 1) yawValue = 255 - yawValue;
    if(reverse_pitch == 1) pitchValue = 255 - pitchValue;
    if(reverse_roll == 1) rollValue = 255 - rollValue;
    if(reverse_channelc == 1) channelCValue = 255 - channelCValue;

    // Battery Input
    batteryVoltageValue = analogReadFast(BAT_IN);
}

/*
 * Continuos transmission
 */
void txMode() {
    readSwitches();
    readAnalogs();
    readAux();

    // Display the animation and info about the settings
    showGimbals();

    // Trigger menu if lgimbal_right and rgimbal_left
    if(lgimbal_right == 1 && rgimbal_left == 1) {
        txmode = MENU_SETUP;
        showHeaderSetup();
        return;
    }

    // BackLight - Special Mode (Only on LONG PRESS)
    if(sw_h == 1) {
        if(backlight_state == 0) {
            ch6Value = ch6Value + CMD_BACKLIGHT;
            if(backlight_enable == 0) {
                backlight_enable = 1;
                ch7Value = BACKLIGHT_ENABLE;
            } else {
                backlight_enable = 0;
                ch7Value = BACKLIGHT_DISABLE;
            }
            backlight_state = 1;
        }
    } else {
        // Flipping the flag
        if(backlight_state == 1) backlight_state = 0;
    }

    // Processing data and sending
    txData();

    // Reset values
    ch7Value = 0;
}

/*
 * Displaying the values of the analog readings and the transmission packets/seconds
 */
void showGimbals() {
    // Analog values
    if(oled_refresh != 1) return;

    sprintf(buf, "T:%04i P:%04i", throttleValue, pitchValue);
    ssd1306_printFixed(0, 16, buf, STYLE_NORMAL);

    sprintf(buf, "Y:%04i R:%04i", yawValue, rollValue);
    ssd1306_printFixed(0, 24, buf, STYLE_NORMAL);

    // Transmission packets.
    sprintf(buf, "TX:%03i/s", txTotalPackets);
    ssd1306_printFixed(0, 32, buf, STYLE_NORMAL);

    // Reset it after done.
    oled_refresh = 0;
}

void drawBars(uint8_t x, uint8_t y, uint8_t max_bars, uint8_t bar_position) {
    sprintf(buf, "T", throttleValue, yawValue);

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
    data.ch1 = rollValue;
    data.ch2 = pitchValue;
    data.ch3 = throttleValue;
    data.ch4 = yawValue;
    data.ch5 = channelCValue;
    data.ch6 = ch6Value;
    data.ch7 = ch7Value;

    // Sending Data
    radio.write(&data, sizeof(MyData));

    // Increase counter for profiling
    txPackets++;
}

void setupMode() {
    readSwitches();

    // Skip for a while
    if(shortDelay(15000) == 0) return;

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

        // Set the value and return to main loop.
        // The main loop will determine the process to work on.
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
        } else if(current_setup_selected == MENU_SETUP_REVERSE) {
            txmode = MENU_SETUP_REVERSE;
            reverse_menu_refreshed = 1;
            current_reverse_selected = 0;
            showHeaderReverse();
            return;
        } else if(current_setup_selected == MENU_SETUP_TIMER) {
            txmode = MENU_SETUP_TIMER;
            showHeaderTimers();
            return;
        } else if(current_setup_selected == MENU_SETUP_FC_MAPPING) {
            txmode = MENU_SETUP_FC_MAPPING;
            mapping_menu_refreshed = 1;
            current_mapping_selected = 0;
            showHeaderMapping();
            return;
        } else if(current_setup_selected == MENU_SETUP_RX_MAPPING) {
            txmode = MENU_SETUP_RX_MAPPING;
            rxmapping_menu_refreshed = 1;
            current_rx_mapping_selected = 0;
            showHeaderRXMapping();
            return;
        } else if(current_setup_selected == MENU_SETUP_IDLEUP_MAPPING) {
            txmode = MENU_SETUP_IDLEUP_MAPPING;
            idleupmapping_menu_refreshed = 1;
            current_idleup_mapping_selected = 0;
            showHeaderIdleUPMapping();
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
 * Setting stick to reverse values
 */
void reverse() { 
    // Reading Inputs
    readSwitches();

    // Skip for a while
    if(shortDelay(15000) == 0) return;

    // Display all the reversable sticks values and then let the user to select 
    // and pick. May involve Up, Down, Left and Right for selection.    
    const char *menuItemsReverse[] = {
        "Throttle",
        "Pitch",
        "Yaw",
        "Roll",
        "Channel C",
        "Exit"
    };

    uint8_t max_items_reverse = (sizeof(menuItemsReverse) / sizeof(char *)) - 1;
    

    // If Down switch pressed, select another menu
    if(rgimbal_down == 1) {
        if(current_reverse_selected < max_items_reverse)
            current_reverse_selected++;
        reverse_menu_refreshed = 1;
    } else if(rgimbal_up == 1) {
        if(current_reverse_selected > 0)
            current_reverse_selected--;
        reverse_menu_refreshed = 1;
    } else if(rgimbal_right == 1 || rgimbal_left == 1) {
        // If is the last item then exit back to the menu
        if(current_reverse_selected == max_items_reverse) {
            txmode = MENU_SETUP;
            showHeaderSetup();
            return;
        }

        // Defined toggle value based on the switch selection.
        // Optimizing the coding.
        uint8_t toggle_value;
        if(rgimbal_right == 1) {
            toggle_value = 1;
        } else {
            toggle_value = 0;
        }

        // Select and unselect the option and save to the EEPROM direct
        if(current_reverse_selected == 0) {
            reverse_throttle = toggle_value;
        } else if(current_reverse_selected == 1) {
            reverse_pitch = toggle_value;
        } else if(current_reverse_selected == 2) {
            reverse_yaw = toggle_value;
        } else if(current_reverse_selected == 3) {
            reverse_roll = toggle_value;
        } else if(current_reverse_selected == 4) {
            reverse_channelc = toggle_value;
        }

        // Save EEPROM settings - REVERSE configuration only
        saveSettings(REVERSE);

        // Refresh menu
        reverse_menu_refreshed = 1;
    }

    // Skip from here if no need to refresh screen.
    if(reverse_menu_refreshed == 0) return;

    // Processing the menu also at the same time put a symbol if is toggled
    uint8_t j = 0;
    uint8_t y = 0;

    for(uint8_t i = 0; i <= max_items_reverse; i++) {
        // Y position of the menu
        y = 17 + (j * 8);

        // Clear previous line
        ssd1306_printFixed(0, y, "               ", STYLE_NORMAL);

        // Print Menu
        ssd1306_printFixed(8, y, menuItemsReverse[i], STYLE_NORMAL);

        // Print Current Toggle Value
        if(i != max_items_reverse) {
            if((reverseBits >> i) & B00000001 == 1) {
                ssd1306_printFixed(100, y, "[X]", STYLE_NORMAL);
            } else {
                ssd1306_printFixed(100, y, "[.]", STYLE_NORMAL);
            }
        }
        
        // Print cursor
        if(i == current_reverse_selected) {
            ssd1306_printFixed(0, y, ">", STYLE_NORMAL);
        }
        j++;
    }

    // Prevent menu refreshed
    reverse_menu_refreshed = 0;
}


/*
 * Setting E-Limits on both of the gimbals
 */
void elimits() {
    readSwitches();
    readAnalogs();

    // Gimbal readings - Not using the common functions since this not requires mapping
    throttleValue = analogReadFast(THROTTLE_PORT);
    yawValue = analogReadFast(YAW_PORT);
    rollValue = analogReadFast(ROLL_PORT);
    pitchValue = analogReadFast(PITCH_PORT);

    ssd1306_printFixed(0, 17, "UP: Reset Right: Save", STYLE_NORMAL);

    // Getting the min and max values of each sticks
    if(throttle_upper_limit < throttleValue) throttle_upper_limit = throttleValue;
    if(throttle_lower_limit > throttleValue) throttle_lower_limit = throttleValue;
    if(yaw_upper_limit < yawValue) yaw_upper_limit = yawValue;
    if(yaw_lower_limit > yawValue) yaw_lower_limit = yawValue;
    if(pitch_upper_limit < pitchValue) pitch_upper_limit = pitchValue;
    if(pitch_lower_limit > pitchValue) pitch_lower_limit = pitchValue;
    if(roll_upper_limit < rollValue) roll_upper_limit = rollValue;
    if(roll_lower_limit > rollValue) roll_lower_limit = rollValue;

    uint8_t x = 8;

    sprintf(buf, "TL:%04i TU:%04i", throttle_lower_limit, throttle_upper_limit);
    ssd1306_printFixed(x, 32, buf, STYLE_NORMAL);
    sprintf(buf, "RL:%04i RU:%04i", yaw_lower_limit, yaw_upper_limit);
    ssd1306_printFixed(x, 40, buf, STYLE_NORMAL);
    sprintf(buf, "EL:%04i EU:%04i", pitch_lower_limit, pitch_upper_limit);
    ssd1306_printFixed(x, 48, buf, STYLE_NORMAL);
    sprintf(buf, "AL:%04i AU:%04i", roll_lower_limit, roll_upper_limit);
    ssd1306_printFixed(x, 56, buf, STYLE_NORMAL);

    // Save settings
    if(rgimbal_right == 1) {
        saveSettings(ELIMIT);

        // Return to main menu
        txmode = MENU_SETUP;
        showHeaderSetup();
        return;
    }

    // Default elimits
    if(rgimbal_up == 1) {
         throttle_upper_limit = 600;
         throttle_lower_limit = 300;
         yaw_upper_limit = 600;
         yaw_lower_limit = 300;
         pitch_upper_limit = 600;
         pitch_lower_limit = 300;
         roll_upper_limit = 600;
         roll_lower_limit = 300;
    }   
}

/*
 *  Debug keys
 */
void debugKeys() {
    readSwitches();
    readAnalogs();

    sprintf(buf, "T:%04i P:%04i", throttleValue, pitchValue);
    ssd1306_printFixed(0, 16, buf, STYLE_NORMAL);

    sprintf(buf, "Y:%04i R:%04i", yawValue, rollValue);
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

/*
 * Shortcut to EXIT from the application
 */
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

void showHeaderMapping() {
    showHeader(0, 0, "FC MAPPING", 1);
}

void showHeaderRXMapping() {
    showHeader(0, 0, "RX MAPPING", 1);
}

void showHeaderIdleUPMapping() {
    showHeader(0, 0, "IDLE UP", 1);
}

void showHeaderDebug() {
    showHeader(0, 0, "[ DEBUG MODE ]", 0);
}

void showHeaderReverse() {
    showHeader(0, 0, "REVERSE", 1);
}

void showHeaderTimers() {
    showHeader(0, 0, "TIMERS", 1);
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
