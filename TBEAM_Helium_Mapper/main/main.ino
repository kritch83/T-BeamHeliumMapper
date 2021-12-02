
#include "configuration.h"
#include "rom/rtc.h"

#include <TinyGPS++.h>
#include <Wire.h>
#include <axp20x.h>

#if defined(OTA_ENABLE)
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

const char *ssid = "helium";
const char *password = "open4meplease";
#endif

AXP20X_Class axp;
bool pmu_irq = false;
String baChStatus = "No charging";

bool ssd1306_found = false;
bool axp192_found = false;

bool packetSent, packetQueued;

#if defined(PAYLOAD_USE_FULL) // includes number of satellites and accuracy
static uint8_t txBuffer[10];
#elif defined(PAYLOAD_USE_CAYENNE)
static uint8_t txBuffer[11] = {0x03, 0x88, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00};
#endif

RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;

int buttonPressCount = 0;

// mine:
unsigned long gpsLockTimer, sendTimer = 0;
unsigned long sendInfoDelay = 30000; // default delay in checking

bool first = true;
int changeSpeed,sendInfo = 1;

int firstRunPressed = 1;
unsigned long buttonTimer1 = 0;

void setup()
{
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    #ifdef DEBUG_PORT
        DEBUG_PORT.begin(SERIAL_BAUD);
    #endif

    initDeepSleep();

    Wire.begin(I2C_SDA, I2C_SCL);
    scanI2Cdevice();

    axp192Init();

    DEBUG_MSG(APP_NAME " " APP_VERSION "\n");
    

    if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
    {
        ssd1306_found = false;
    }

    if (ssd1306_found)
    {
        screen_setup();
    }

    gps_setup();

    screen_print(APP_NAME " " APP_VERSION, 0, 0);
    screen_show_logo();
    screen_update();
    delay(3000);

    // Helium setup
    if (!ttn_setup())
    {
        screen_print("[ERR] Radio module not found!\n");
        delay(30000);
        screen_off();
        sleep_forever();
    }
    else
    {
        ttn_register(callback);
        ttn_join();
        ttn_adr(LORAWAN_ADR);
    }
    //doesn't register correctly - TODO
    // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPress, FALLING);
}

////////////////////////////////////////////////////////////

void loop()
{
    gps_loop();
    ttn_loop();
    screen_loop();

    if (packetSent)
    {
        packetSent = false;
        sleep();
    }

    if (!digitalRead(BUTTON_PIN))
    {
        buttonPressCount = 1;
        screen_print("\n\n\n\n");
        screen_print("       Select:\n");
        screen_print("1: LowPower\n");
        screen_print("2: PollingSpeed\n");
        screen_print("3: OTA\n");
        screen_print("10: ResetSession\n");
        
        delay(300);

        buttonTimer1 = millis() + 5000;
        

        while (millis() < buttonTimer1)
        {
            if (!digitalRead(BUTTON_PIN))
            {
                screen_print("ButtonPressed\n");
                buttonPressCount++;
                delay(300);
            }
        }

        switch (buttonPressCount)
        {
        case 1:
            if (sendInfo)
            {
                sendInfo = 0;
                screen_print("\n\n");
                screen_print("Radio & GPS OFF\n");
                radioContol(0);
                screen_print("\n\n");
            }
            else
            {
                sendInfo = 1;
                screen_print("\n\n");
                screen_print("resuming operation\n");
                screen_print("\n\n");
                radioContol(1);
                delay(1000);
            }
            break;
        case 2:
            switch (changeSpeed)
            {
            case 0:
                screen_print("\n\nCheck Interval now 30\n\n\n");
                sendInfoDelay = 60000;
                changeSpeed = 1;
                delay(1000);
                break;
            case 1:
                screen_print("\n\nCheck Interval now 60\n\n");
                sendInfoDelay = 150000;
                changeSpeed = 2;
                delay(1000);
                break;
            case 2:
                screen_print("\n\nCheck Interval now 15\n\n\n");
                sendInfoDelay = 30000;
                changeSpeed = 0;
                delay(1000);
                break;
            }
            Serial.println(changeSpeed);
            delay(2000);
            break;
        case 3:
            #ifdef OTA_ENABLE
            screen_print("\n\n\n");
            startWIFIAP();
            #endif
            break;
        case 10:
            #ifdef PREFS_DISCARD
            screen_print("\n\n\n\n");
            screen_print("Initialized keys!\n");
            ttn_erase_prefs();
            delay(5000);
            ESP.restart();
            #endif
            break;
        }

        buttonPressCount = 0;
    }

    if (millis() - sendTimer > sendInfoDelay && sendInfo)
        {
            if (0 < gps_hdop() && gps_hdop() < 50 && gps_latitude() != 0 &&
                gps_longitude() != 0 && gps_altitude() != 0)
            {
                sendPacket();
                sendTimer = millis();

                Serial.println("TRANSMITTED\n");
            }
            else
            {
                screen_print("Waiting GPS lock\n");
            }
            sendTimer = millis();
        }
}

////////////////////////////////////////////////////////////

void buildPacket(uint8_t txBuffer[]); // needed for platformio

bool sendPacket()
{
    packetSent = false;

    char buffer[40];
    snprintf(buffer, sizeof(buffer), "Latitude: %10.6f\n", gps_latitude());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "Longitude: %10.6f\n", gps_longitude());
    screen_print(buffer);
    snprintf(buffer, sizeof(buffer), "Accuracy/HDOP: %4.2fm\n", gps_hdop());
    screen_print(buffer);

    // Serial.print("Buffer: ");
    // Serial.println(buffer);

    buildPacket(txBuffer);

#if LORAWAN_CONFIRMED_EVERY > 0
    bool confirmed = (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);
    if (confirmed)
    {
        #ifdef DEBUG
        Serial.println("confirmation enabled");
        #endif
    }
#else
    bool confirmed = false;
#endif
    //    Serial.print("txBuffer: ");
    //    Serial.print(sizeof(txBuffer));
    //    Serial.print(" ");

    //    for (int i = 0; i < 11; i++)
    //    {
    //      Serial.println(txBuffer[i]);
    //    }

    packetQueued = true;
    ttn_send(txBuffer, sizeof(txBuffer), LORAWAN_PORT, confirmed);
}

void doDeepSleep(uint64_t msecToWake)
{
    Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

    // not using wifi yet, but once we are this is needed to shutoff the radio
    // hw esp_wifi_stop();

    screen_off();    // datasheet says this will draw only 10ua
    LMIC_shutdown(); // cleanly shutdown the radio

    if (axp192_found)
    {
        // turn on after initial testing with real hardware
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
    }

    // FIXME - use an external 10k pulldown so we can leave the RTC peripherals
    // powered off until then we need the following lines
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Only GPIOs which are have RTC functionality can be used in this bit map:
    // 0,2,4,12-15,25-27,32-39.
    uint64_t gpioMask = (1ULL << BUTTON_PIN);

    // FIXME change polarity so we can wake on ANY_HIGH instead - that would
    // allow us to use all three buttons (instead of just the first)
    gpio_pullup_en((gpio_num_t)BUTTON_PIN);

    esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);

    esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // call expects usecs
    esp_deep_sleep_start();                              // TBD mA sleep current (battery)
}

void sleep()
{
#if SLEEP_BETWEEN_MESSAGES

    // If the user has a screen, tell them we are about to sleep
    if (ssd1306_found)
    {
        // Show the going to sleep message on the screen
        char buffer[20];
        snprintf(
            buffer, sizeof(buffer), "Sleeping in %3.1fs\n",
            (MESSAGE_TO_SLEEP_DELAY / 1000.0));
        screen_print(buffer);

        // Wait for MESSAGE_TO_SLEEP_DELAY millis to sleep
        delay(MESSAGE_TO_SLEEP_DELAY);

        // Turn off screen
        screen_off();
    }

    // Set the user button to wake the board
    sleep_interrupt(BUTTON_PIN, LOW);

    // We sleep for the interval between messages minus the current millis
    // this way we distribute the messages evenly every SEND_INTERVAL millis
    uint32_t sleep_for =
        (millis() < SEND_INTERVAL) ? SEND_INTERVAL - millis() : SEND_INTERVAL;
    doDeepSleep(sleep_for);

#endif
}

void callback(uint8_t message)
{
    bool ttn_joined = false;
    if (EV_JOINED == message)
    {
        ttn_joined = true;
    }
    if (EV_JOINING == message)
    {
        if (ttn_joined)
        {
            screen_print("Helium joining...\n");
        }
        else
        {
            screen_print("Joined Helium!\n");
        }
    }
    if (EV_JOIN_FAILED == message)
        screen_print("Helium join failed\n");
    if (EV_REJOIN_FAILED == message)
        screen_print("Helium rejoin failed\n");
    if (EV_RESET == message)
        screen_print("Reset Helium connection\n");
    if (EV_LINK_DEAD == message)
        screen_print("Helium link dead\n");
    if (EV_ACK == message)
        screen_print("ACK received\n");
    if (EV_PENDING == message)
        screen_print("Message discarded\n");
    if (EV_QUEUED == message)
        screen_print("Message queued\n");

    // We only want to say 'packetSent' for our packets (not packets needed for
    // joining)
    if (EV_TXCOMPLETE == message && packetQueued)
    {
        screen_print("Message sent\n");
        packetQueued = false;
        packetSent = true;
    }

    if (EV_RESPONSE == message)
    {

        screen_print("[Helium] Response: ");

        size_t len = ttn_response_len();
        uint8_t data[len];
        ttn_response(data, len);

        char buffer[6];
        for (uint8_t i = 0; i < len; i++)
        {
            snprintf(buffer, sizeof(buffer), "%02X", data[i]);
            screen_print(buffer);
        }
        screen_print("\n");
    }
}

void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS)
            {
                ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }
            if (addr == AXP192_SLAVE_ADDRESS)
            {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        }
        else if (err == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

/**
   Init the power manager chip

   axp192 power
    DCDC1 0.7-3.5V @ 1200mA max -> OLED // If you turn this off you'll lose
   comms to the axp192 because the OLED and the axp192 share the same i2c bus,
   instead use ssd1306 sleep mode DCDC2 -> unused DCDC3 0.7-3.5V @ 700mA max ->
   ESP32 (keep this on!) LDO1 30mA -> charges GPS backup battery // charges the
   tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can
   not be turned off LDO2 200mA -> LORA LDO3 200mA -> GPS
*/

void axp192Init()
{
    if (axp192_found)
    {
        if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
        {
            #if defined (DEBUG)
            Serial.println("AXP192 Begin PASS");
            #endif
        }
        else
        {
            #if defined (DEBUG)
            Serial.println("AXP192 Begin FAIL");
            #endif
        }
        // axp.setChgLEDMode(LED_BLINK_4HZ);
        #if defined (DEBUG)
            Serial.printf(
            "DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
            Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
            Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
            Serial.println("----------------------------------------");
        #endif
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp.setDCDC1Voltage(3300); // for the OLED power

        #if defined (DEBUG)
            Serial.printf(
            "DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
            Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
            Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
            Serial.printf(
            "Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
        #endif
        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(
            PMU_IRQ, []
            { pmu_irq = true; },
            FALLING);

        axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp.enableIRQ(
            AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ |
                AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ,
            1);
        axp.clearIRQ();

        if (axp.isChargeing())
        {
            baChStatus = "Charging";
        }
    }
    else
    {
        #if defined (DEBUG)
            Serial.println("AXP192 not found");
        #endif
    }
}

// Perform power on init that we do on each wake from deep sleep
void initDeepSleep()
{
    bootCount++;
    wakeCause = esp_sleep_get_wakeup_cause();
    /*
      Not using yet because we are using wake on all buttons being low

      wakeButtons = esp_sleep_get_ext1_wakeup_status();       // If one of these
      buttons is set it was the reason we woke if (wakeCause ==
      ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons) // we must have been using the 'all
      buttons rule for waking' to support busted boards, assume button one was
      pressed wakeButtons = ((uint64_t)1) << buttons.gpios[0];
    */
    #if defined (DEBUG)
    Serial.printf(
        "booted, wake cause %d (boot count %d)\n", wakeCause, bootCount);
    #endif
}

void startWIFIAP()
{
    int m = 0;

    screen_print("turning off GPS\n");
    screen_print("turning off LORA\n");

    radioContol(0);

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
                            //   static int m;
                            // //   screen_print(".");
                            // m++;

   
                            char buffer[20];
                            int howFar = ((float)progress / total) * 100;
                            // snprintf(buffer, sizeof(buffer), "%d", howFar);
                            // screen_print(buffer);
                            // screen_print("\n\n\n\n");

                            percentDone(howFar);
                          });

    ArduinoOTA.setHostname("Helium_Mapper");

    WiFi.softAP(ssid, password);

    screen_print("wifi started\n");
    delay(1000);
    ArduinoOTA.begin();
    screen_print("OTA started\n");

    while (true)
    {
        ArduinoOTA.handle();
        screen_loop();
    }
}

void radioContol(byte Rstate)
{
    if (!Rstate)
    {
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
    }
    else
    {
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    }
}