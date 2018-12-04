#ifndef LCD_H_
#define LCD_H_

#include "Arduino.h"
#include "ssd1306_fonts.h"
#include "ssd1306_1bit.h"

void drawLine(uint8_t,uint8_t,uint8_t,uint8_t);
void LCDInit();
void negativeMode();
void positiveMode();
void run_createMenu(SAppMenu, const char **, uint8_t);
void run_showMenu(SAppMenu);
void show_logo();
void showMenu();
#endif
