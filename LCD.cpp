// Display Headers
#include "LCD.h"
#include "ssd1306.h"
#include "RF.h"

// Including images, icons
#include "echowiitx_logo.h"

/*
 * Start Up Logo
 */
void show_logo() {
    ssd1306_drawBitmap(0, 0, 128, 64, echowiitx_logo);
}

void showHeader() {
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(0,  0, "[ ECHOWII TX ]", STYLE_NORMAL);
}

void negativeMode() {
    // Clear previous line...
    ssd1306_negativeMode();
}

void positiveMode() {
    // NOrmal color
    ssd1306_positiveMode();
}

void drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) { 
    ssd1306_drawLine(x1, y1, x2, y2);
}

void LCDInit() {
    ssd1306_128x64_i2c_init();
}
