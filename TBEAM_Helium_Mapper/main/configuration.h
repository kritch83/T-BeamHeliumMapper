
#pragma once

#include <Arduino.h>
#include <lmic.h>
void ttn_register(void (*callback)(uint8_t message));

#define APP_NAME                "Helium T-Beam"
#define APP_VERSION             "1.4.2"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// display selection
//uncomment if using SH1106 1.3", comment out for SSD1106 0.96"
#define SHLCD  

//OTA wifi update
#define OTA_ENABLE

//Uncomment to enable discarding network settings
#define PREFS_DISCARD

// ***Serial debug***
//#define DEBUG_PORT Serial    
//#define SERIAL_BAUD 9600

//auth type - select only 1
//#define USE_ABP
#define USE_OTAA

//board selection - only 1
//#define T_BEAM_V07  // Rev0
 #define T_BEAM_V10  // Rev1.x

// Select the payload format
#define PAYLOAD_USE_FULL
//#define PAYLOAD_USE_CAYENNE



#define SLEEP_BETWEEN_MESSAGES  false           // Do sleep between messages

#define MESSAGE_TO_SLEEP_DELAY  5000            // Time after message before going to sleep
#define LORAWAN_PORT            1              // Port the messages will be sent to
#define LORAWAN_CONFIRMED_EVERY 5               // Send confirmed message every these many messages (0 means never)
#define LORAWAN_SF              DR_SF7         // Spreading factor (recommended DR_SF7 for ttn network map purposes, DR_SF10 works for slow moving trackers)
#define LORAWAN_ADR             0               // Enable ADR
#define REQUIRE_RADIO           true            // If true, we will fail to start if the radio is not found

// If not defined, we will wait for lock forever
#define GPS_WAIT_FOR_LOCK       (90 * 1000)     // Wait after every boot for GPS lock (may need longer than 5s because we turned the gps off during deep sleep)

// If using a single-channel gateway, set channel
//#define SINGLE_CHANNEL_GATEWAY  0

// If you are having difficulty sending messages after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             5


// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------

#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

///////////////////////////////////////////////////////////////

#ifdef USE_ABP

    // LoRaWAN NwkSKey, network session key
    static const u1_t PROGMEM NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // LoRaWAN AppSKey, application session key
    static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // LoRaWAN end-device address (DevAddr)
    // This has to be unique for every node
    static const u4_t DEVADDR = 0x00000000;

#endif

#ifdef USE_OTAA
//    // This should also be in little endian format (lsb), see above.
//    // Note: If unset it will be generated automatically based on the device macaddr
//    static u1_t DEVEUI[8]  = { 0xB3, 0xAA, 0xDC, 0xC9, 0xFD, 0xF9, 0x81, 0x60 };

//    // This EUI must be in least-significant-byte (lsb) first.
//    static const u1_t PROGMEM APPEUI[8]  = { 0x9D, 0xD1, 0x3F, 0xF4, 0x7B, 0xF9, 0x81, 0x60 };

//    // This key should be in big endian format (msb)
//    static const u1_t PROGMEM APPKEY[16] = { 0x3D, 0x51, 0xF9, 0x18, 0xC0, 0xDA, 0xFD, 0xF6, 0xCA, 0xCA, 0x45, 0x17, 0x25, 0xFD, 0x73, 0x1D };


static u1_t DEVEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#endif

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED       100
#define EV_PENDING      101
#define EV_ACK          102
#define EV_RESPONSE     103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------

#define I2C_SDA         21
#define I2C_SCL         22

#if defined(T_BEAM_V07)
#define LED_PIN         14
#define BUTTON_PIN      39
#elif defined(T_BEAM_V10)
#define BUTTON_PIN      38
#endif

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

#define SSD1306_ADDRESS 0x3C

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM  1
#define GPS_BAUDRATE    9600
#define USE_GPS         1

#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#if defined(T_BEAM_V10)
#define RESET_GPIO      14
#else
#define RESET_GPIO      23
#endif
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

// #define AXP192_SLAVE_ADDRESS  0x34 // Now defined in axp20x.h
#define GPS_POWER_CTRL_CH     3
#define LORA_POWER_CTRL_CH    2
#define PMU_IRQ               35
