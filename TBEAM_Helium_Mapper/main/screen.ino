#include <Wire.h>
#include "OLEDDisplay.h"
#include "images.h"
#include "fonts.h"

#define SCREEN_HEADER_HEIGHT    14

#ifdef SHLCD
    #include "SH1106Wire.h"
    SH1106Wire * display;
#else
    #include "SSD1306Wire.h"
    SSD1306Wire * display;
#endif

uint8_t _screen_line = SCREEN_HEADER_HEIGHT - 1;

void _screen_header() {
    if(!display) return;

    char buffer[20];

    // Message count
    snprintf(buffer, sizeof(buffer), "#%03d", ttn_get_count() % 1000);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);

    if(axp192_found){
        snprintf(buffer, sizeof(buffer), "%.1fV %.0fmA", axp.getBattVoltage()/1000, axp.getBattChargeCurrent() - axp.getBattDischargeCurrent());

    }
    
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(display->getWidth()/2, 2, buffer);

    // Satellite count
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(gps_sats(), buffer, 10));
    display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);
}

void screen_show_logo() {
    if(!display) return;

    uint8_t x = (display->getWidth() - TTN_IMAGE_WIDTH) / 2;
    uint8_t y = SCREEN_HEADER_HEIGHT + (display->getHeight() - SCREEN_HEADER_HEIGHT - TTN_IMAGE_HEIGHT) / 2 + 1;
    display->drawXbm(x, y, TTN_IMAGE_WIDTH, TTN_IMAGE_HEIGHT, TTN_IMAGE);
}

void screen_off() {
    if(!display) return;

    display->displayOff();
}

void screen_on() {
    if (!display) return;

    display->displayOn();
}

void screen_clear() {
    if(!display) return;

    display->clear();
}

void screen_print(const char * text, uint8_t x, uint8_t y, uint8_t alignment) 
{
    DEBUG_MSG(text);

    if(!display) return;

    display->setTextAlignment((OLEDDISPLAY_TEXT_ALIGNMENT) alignment);
    display->drawString(x, y, text);
}

void screen_print(const char * text, uint8_t x, uint8_t y) 
{
    screen_print(text, x, y, TEXT_ALIGN_LEFT);
}

void screen_print(const char * text) 
{
    #ifdef DEBUG
    Serial.printf("Screen: %s\n", text);
    #endif

    if(!display) return;

    display->print(text);
    if (_screen_line + 8 > display->getHeight()) {
        // scroll
    }
    _screen_line += 8;
    screen_loop();
}

void screen_update()
{
    if (display)
    {
        display->display();
    }
}
void screen_setup() 
{
    #ifdef SHLCD
        display = new SH1106Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
    #else
        display = new SSD1306Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
    #endif

    display->init();
    display->flipScreenVertically();
    display->setFont(Custom_ArialMT_Plain_10);

    // Scroll buffer
    display->setLogBuffer(5, 30);
}

void screen_loop() 
{
    if (!display) 
    {
        return;
    }

    #ifdef T_BEAM_V10
    if (axp192_found && pmu_irq) {
        pmu_irq = false;
        axp.readIRQ();
        if (axp.isChargingIRQ()) 
        {
            baChStatus = "Charging";
        } else 
        {
            baChStatus = "No Charging";
        }
        if (axp.isVbusRemoveIRQ()) 
        {
            baChStatus = "No Charging";
        }
        #ifdef DEBUG
        Serial.println(baChStatus);
        #endif

        digitalWrite(2, !digitalRead(2));
        axp.clearIRQ();
    }
    #endif

    display->clear();
    _screen_header();
    display->drawLogBuffer(0, SCREEN_HEADER_HEIGHT);
    display->display();
}

void percentDone(int donePercent)
{
    display->clear();
    display->drawString(40, 20, "Updating...");
    donePercent = map(donePercent,0, 100, 0, 128);
    display->drawLine(0, 50, donePercent, 50);
    display->display();
}