#ifndef ECHOWIITX_H_
#define ECHOWIITX_H_


#include "ssd1306_1bit.h"

#define VERSION 001
#define NOP __asm__ __volatile__ ("nop\n\t")
void readSwitches();

//declare reset function at address 0
void(* resetFunc) (void) = 0;
void rfscanMode();
void debugKeys();
void detectExit();
void drawBars(uint8_t x, uint8_t y, uint8_t max_bars, uint8_t bar_position);
void scanChannels();
void setup_inputs();
void setup_radio();
void setupMode();
void showRFScanMode();
void showLogo();
void showGimbals();
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

/* Skip some processes */
uint8_t shortDelay();

void showHeaderMain();
void showHeaderDebug();
void showHeaderSetup();
void showHeader(uint8_t x, uint8_t y, const char *title, uint8_t style);

/* Reading analogue values. */
uint16_t throttleValue, rudderValue, elevatorValue, aileronValue;
uint16_t batteryVoltageValue, channelCValue;

/* Controller variables */
uint8_t sw_a, sw_b, sw_d, sw_e, sw_f, sw_g, sw_h;
uint8_t lgimbal_up, lgimbal_down, lgimbal_left, lgimbal_right;
uint8_t rgimbal_up, rgimbal_down, rgimbal_left, rgimbal_right;

/* Gimbal's upper limit and lower limit */
uint16_t throttle_upper_limit = 900;
uint16_t throttle_lower_limit = 100;
uint16_t rudder_upper_limit = 900;
uint16_t rudder_lower_limit = 100;
uint16_t elevator_upper_limit = 900;
uint16_t elevator_lower_limit = 100;
uint16_t aileron_upper_limit = 900;
uint16_t aileron_lower_limit = 100;

/* For counter checking eeprom checksum */
uint8_t eeprom_checksum;

/* For preventing too sensitive on the switch */
uint8_t switches_state = 0;

#endif
