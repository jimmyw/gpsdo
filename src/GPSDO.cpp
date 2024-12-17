
#include <Arduino.h>

// programing of the u-blox 7m module from Arduino for timepulse TP-5
// hex was taken from u-center software configuration view
//
//
// plus, flashing LED connected to pin 10 indicates when enough sat's in view
// u-blox module to connect o 3 and 4 for using soft serial of Arduino
// CT2GQV 2019 mixing multiple libs and examples.

#include "esp_log.h"
#include <TinyGPS.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <stdint.h>
#include <string.h>
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

// https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf

#define ACTIVE (1 << 0)
#define LOCK_GPS_FREQS (1 << 1)
#define LOCK_OTHER_SET (1 << 2)
#define IS_REQ (1 << 3)
#define IS_LENGTH (1 << 4)
#define ALIGN_TO_TOW (1 << 5)
#define POLARITY (1 << 6)
#define GRID_UTC_GPS (1 << 7)

#define TP5_SIZE 40

static void setMode();

// Define the configuration struct
typedef struct {
  uint8_t tpIdx; // Timepulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
  uint8_t reserved0;
  uint8_t reserved1[2];
  int16_t antCableDelay; // Antenna cable delay (ns)
  int16_t rfGroupDelay;  // RF group delay (ns)
  uint32_t freqPeriod;   // Frequency or period time, depending on settingof bit
                         // 'isFreq' (hz/us)
  uint32_t freqPeriodLock; // Frequency or period time when locked to GPS time,
                           // only used if 'lockedOtherSet' is set (hs/us)
  uint32_t pulseLenRatio;  // Pulse length or duty cycle, depending on'isLength'
                           // (us/-)
  uint32_t
      pulseLenRatioLock; // Pulse length or duty cycle when locked to GPS time,
                         // only used if 'lockedOtherSet' is set (us/-)
  int32_t userConfigDelay; // User configurable timepulse delay (ns)
  uint32_t flags;
} __attribute__((packed)) UBX_CFG_TP5_t;

// Function to generate UBlox CFG-TP5 message
void generateUBloxCFGTP5(const UBX_CFG_TP5_t *config, uint8_t *message) {
  // UBX header
  message[0] = 0xB5; // Sync char 1
  message[1] = 0x62; // Sync char 2
  message[2] = 0x06; // Class
  message[3] = 0x31; // ID
  message[4] = 0x20; // Length LSB (32 bytes)
  message[5] = 0x00; // Length MSB

  assert(sizeof(*config) == 0x20);

  // Copy the configuration struct into the message payload
  memcpy(&message[6], config, sizeof(UBX_CFG_TP5_t));

  // Calculate checksum
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 38; i++) {
    ck_a += message[i];
    ck_b += ck_a;
  }

  // Append checksum to the message
  message[38] = ck_a;
  message[39] = ck_b;
}

struct mode_def {
  const char *name;
  const UBX_CFG_TP5_t value;
};

const struct mode_def timing_modes[] = {

    {.name = "1Hz",
     .value =
         {
             .tpIdx = 0x00,
             .reserved0 = 1,
             .antCableDelay = 0x00,           // ns
             .rfGroupDelay = 0x00,            // ns
             .freqPeriod = 0,                 // disable
             .freqPeriodLock = 1,             // 1 Hz (1,000,000 microseconds)
             .pulseLenRatio = 0x80000000,     // 50% (500,000 in 0.01% units)
             .pulseLenRatioLock = 0x80000000, // 50% (500,000 in 0.01% units)
             .userConfigDelay = 0,            // ns
             .flags = ACTIVE | LOCK_GPS_FREQS | LOCK_OTHER_SET | IS_REQ |
                      ALIGN_TO_TOW | POLARITY, // Active high, disable if no fix
         }},
    {.name = "1kHz",
     .value =
         {
             .tpIdx = 0x00,
             .reserved0 = 1,
             .antCableDelay = 0x00,           // ns
             .rfGroupDelay = 0x00,            // ns
             .freqPeriod = 0,                 // disable
             .freqPeriodLock = 1000,          // 10 kHz
             .pulseLenRatio = 0x80000000,     // 50% (500,000 in 0.01% units)
             .pulseLenRatioLock = 0x80000000, // 50% (500,000 in 0.01% units)
             .userConfigDelay = 0,            // ns
             .flags = ACTIVE | LOCK_GPS_FREQS | LOCK_OTHER_SET | IS_REQ |
                      ALIGN_TO_TOW | POLARITY, // Active high, disable if no fix
         }

    },
    {.name = "1kHz",
     .value =
         {
             .tpIdx = 0x00,
             .reserved0 = 1,
             .antCableDelay = 0x00,           // ns
             .rfGroupDelay = 0x00,            // ns
             .freqPeriod = 0,                 // disable
             .freqPeriodLock = 10000,         // 10 KHz
             .pulseLenRatio = 0x80000000,     // 50% (500,000 in 0.01% units)
             .pulseLenRatioLock = 0x80000000, // 50% (500,000 in 0.01% units)
             .userConfigDelay = 0,            // ns
             .flags = ACTIVE | LOCK_GPS_FREQS | LOCK_OTHER_SET | IS_REQ |
                      ALIGN_TO_TOW | POLARITY, // Active high, disable if no fix
         }

    },
    {.name = "1Mhz",
     .value =
         {
             .tpIdx = 0x00,
             .reserved0 = 1,
             .antCableDelay = 0x00,           // ns
             .rfGroupDelay = 0x00,            // ns
             .freqPeriod = 0,                 // disable
             .freqPeriodLock = 1000000,       // 1 Mhz
             .pulseLenRatio = 0x80000000,     // 50% (500,000 in 0.01% units)
             .pulseLenRatioLock = 0x80000000, // 50% (500,000 in 0.01% units)
             .userConfigDelay = 0,            // ns
             .flags = ACTIVE | LOCK_GPS_FREQS | LOCK_OTHER_SET | IS_REQ |
                      ALIGN_TO_TOW | POLARITY, // Active high, disable if no fix
         }

    },
    {.name = "10Mhz",
     .value =
         {
             .tpIdx = 0x00,
             .reserved0 = 1,
             .antCableDelay = 0x00,           // ns
             .rfGroupDelay = 0x00,            // ns
             .freqPeriod = 0,                 // disable
             .freqPeriodLock = 10000000,      // 10 Mhz
             .pulseLenRatio = 0x80000000,     // 50% (500,000 in 0.01% units)
             .pulseLenRatioLock = 0x80000000, // 50% (500,000 in 0.01% units)
             .userConfigDelay = 0,            // ns
             .flags = ACTIVE | LOCK_GPS_FREQS | LOCK_OTHER_SET | IS_REQ |
                      ALIGN_TO_TOW | POLARITY, // Active high, disable if no fix
         }},
};

int havefix = 0; // to keep track if we have fix for at least 3 seconds

TinyGPS gps;
HardwareSerial ss(1);

static float flat = 0;
unsigned long age = 0;
static float flon = 0;
static int sats = 0;
static int mode_idx = 0;

#define LCD_SCL 2
#define LCD_SDA 3
#define GPS_RX 7
#define GPS_TX 10
#define BUTTON 9

// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C(const u8g2_cb_t *rotation, uint8_t
// reset = U8X8_PIN_NONE, uint8_t clock = U8X8_PIN_NONE, uint8_t data =
// U8X8_PIN_NONE)

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(/* rotation =*/U8G2_R0,
                                         /* reset=*/U8X8_PIN_NONE,
                                         /* clock=*/LCD_SCL,
                                         /* data=*/LCD_SDA);

void setup() {
  //Serial.begin(115200);
  Serial.println("= GPSDO =");
  pinMode(BUTTON, INPUT);

  u8g2.setBusClock(100000);
  // u8g2.setI2CAddress(0x3C);
  u8g2.begin();
  u8g2.setPowerSave(0);

  u8g2.firstPage(); // Start the first page
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.setCursor(0, 10);              // Set cursor position
    u8g2.print("Hello, World!");        // Print message
  } while (u8g2.nextPage());            // Continue to the next page

  ss.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // RX, TX

  setMode();
}

static void setMode() {
  const struct mode_def *mode = &timing_modes[mode_idx];
  uint8_t message[TP5_SIZE];
  generateUBloxCFGTP5(&mode->value, message);

  // actual u-blox 7m programing
  for (int i = 0; i < sizeof(message); i++) {
    ss.write(message[i]);
    Serial.print("0x");
    Serial.println(message[i], HEX);
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are
              // not accepted by the device.
  }
  Serial.println();
  // ends here
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 100;) {
    while (ss.available()) {
      char c = ss.read();
      // Serial.write(
      //     c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData) {
    gps.f_get_position(&flat, &flon, &age);
    sats = gps.satellites();
    havefix = age < 3000 ? 1 : 0; // if we have fix for at least 3 seconds


    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(sats);
    Serial.print(" AGE=");
    Serial.print(age);
    Serial.print(" FIX=");
    Serial.println(havefix);
  } // end new data

  gps.stats(&chars, &sentences, &failed);
  if (chars == 0) {
    Serial.println("** No characters received from GPS: check wiring **");
  }

  // Use button to switch modes
  {
    static volatile bool lastState = HIGH;
    bool currentState = digitalRead(BUTTON);
    if (lastState == HIGH && currentState == LOW) {
      // Reset the setpoint to 350 degrees
      mode_idx++;
      mode_idx = mode_idx % ARRAY_SIZE(timing_modes);
      Serial.print("Button pressed, new mode: ");
      Serial.println(mode_idx);

      setMode();
    }
    lastState = currentState;
  }

  // Update LCD
  u8g2.firstPage();
  do {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenR08_tr);
    u8g2.setCursor(0, 10);
    u8g2.print("Satelites:");
    u8g2.print(sats);
    if (havefix)
      u8g2.print(" FIX");
    else
      u8g2.print(" NO FIX");
    u8g2.setCursor(0, 20);
    u8g2.print("LAT:");
    u8g2.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    u8g2.setCursor(0, 30);
    u8g2.print("LON:");
    u8g2.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

    u8g2.setCursor(0, 40);
    const struct mode_def *mode = &timing_modes[mode_idx];
    u8g2.println(mode->name);
    u8g2.sendBuffer();
  } while (u8g2.nextPage());

}
//=end code==============