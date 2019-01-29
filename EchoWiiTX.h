#ifndef ECHOWIITX_H_
#define ECHOWIITX_H_


#include "ssd1306_1bit.h"

#define VERSION 001
#define NOP __asm__ __volatile__ ("nop\n\t")
void readSwitches();
void readAnalogs();
void readAux();

//declare reset function at address 0
void(* resetFunc) (void) = 0;
void debugKeys();
void detectExit();
void drawBars(uint8_t x, uint8_t y, uint8_t max_bars, uint8_t bar_position);
void elimits();
void idleup();
void mapping();
void reverse();
void rfscanMode();
void rxmapping();
void saveSettings();
void scanChannels();
void setup_inputs();
void setup_radio();
void setupTimers();
void setupMode();
void showGimbals();
void showHeaderDebug();
void showHeaderELimits();
void showHeaderMain();
void showHeaderMapping();
void showHeaderReverse();
void showHeaderRXMapping();
void showHeaderIdleUPMapping();
void showHeaderSetup();
void showHeaderTimers();
void showHeader(uint8_t x, uint8_t y, const char *title, uint8_t style);
void showLogo();
void showRFScanMode();
void runTimers();
void txData();
void txMode();

/*
 * Writting EEPROM in 2 Bytes mode (integer - 16 bits)
 */
void EEPROMWrite16Bits(uint16_t addr, uint16_t value);

/*
 * Reading EEPROM in 2 Bytes mode (integer - 16 bits)
 */
uint16_t EEPROMRead16Bits(uint16_t addr);
uint8_t checksum();

/* Is the key pressed? */
uint8_t isPressed(char keyChar);

/* Skip some processes */
uint8_t shortDelay(uint16_t max_count);

/* Reading analogue values. */
uint16_t throttleValue, yawValue, pitchValue, rollValue;
uint16_t batteryVoltageValue, channelCValue;

/* For the Auxilary ports on FC module */
#define AUXMAX 3
uint8_t portAUX[AUXMAX] = {};

/* For the Auxilary ports on RX module */
#define RXAUXMAX 1
uint8_t portRXAUX[RXAUXMAX] = {};

/* For the IdleUp Throttle Switch */
#define IDLEUPMAX 1
uint8_t idleUpThrottle[IDLEUPMAX] = {};

/* For the channel transmission data */
uint8_t ch6Value = 0;
uint8_t ch7Value = 0;

/* For controlling the flight blinking light */
uint8_t backlight_state = 0;

/* Controller variables */
uint8_t sw_a, sw_b, sw_d, sw_e, sw_f, sw_g, sw_h;
uint8_t lgimbal_up, lgimbal_down, lgimbal_left, lgimbal_right;
uint8_t rgimbal_up, rgimbal_down, rgimbal_left, rgimbal_right;

/* Gimbal's upper limit and lower limit */
uint16_t throttle_upper_limit = 600;
uint16_t throttle_lower_limit = 300;
uint16_t yaw_upper_limit = 600;
uint16_t yaw_lower_limit = 300;
uint16_t pitch_upper_limit = 600;
uint16_t pitch_lower_limit = 300;
uint16_t roll_upper_limit = 600;
uint16_t roll_lower_limit = 300;

/* For each of the gimbal stick and analog input reverse state */
/* If 0: read as normal.
   If 1: reverse the read values.
   EEPROM addr: 0x0010
   Bits |7|6|5|4|3|2|1|0|
         - - - C R Y P T
   T: Throttle
   P: Pitch
   Y: Yaw
   R: Roll
   C: Channel C
*/
uint8_t reverseBits = 0;
uint8_t reverse_throttle = 0;
uint8_t reverse_yaw = 0;
uint8_t reverse_pitch = 0;
uint8_t reverse_roll = 0;
uint8_t reverse_channelc = 0;

/* For counter checking eeprom checksum */
uint8_t eeprom_checksum;

/* For Timers use */
uint32_t timer1 = 0;
uint32_t timer2 = 0;

#endif
