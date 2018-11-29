#ifndef ECHOWIITX_H_
#define ECHOWIITX_H_


#include "ssd1306_1bit.h"

#define VERSION 001
#define NOP __asm__ __volatile__ ("nop\n\t")
void readSwitches();

//declare reset function at address 0
void(* resetFunc) (void) = 0;
void rfscanMode();
void scanChannels();
void setup_inputs();
void setup_radio();
void showRFScanMode();
void showMenu();
void showHeader();
void txData();
void txMode();

uint8_t val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12, val13, val14, val15, val16;

#endif
