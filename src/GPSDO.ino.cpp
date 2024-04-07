# 1 "/tmp/tmpgelzc5te"
#include <Arduino.h>
# 1 "/home/jimmy/Documents/PlatformIO/Projects/240407-212751-genericSTM32F103C8/src/GPSDO.ino"
# 9 "/home/jimmy/Documents/PlatformIO/Projects/240407-212751-genericSTM32F103C8/src/GPSDO.ino"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "si5351.h"
Si5351 si5351;


const char UBLOX_INIT[] PROGMEM = {
# 38 "/home/jimmy/Documents/PlatformIO/Projects/240407-212751-genericSTM32F103C8/src/GPSDO.ino"
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6E, 0x01, 0x00, 0x36,
  0x6E, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x11, 0xD8,
# 72 "/home/jimmy/Documents/PlatformIO/Projects/240407-212751-genericSTM32F103C8/src/GPSDO.ino"
};

int SATLED = PC13;
int havefix = 0;
float start_freq = 2000000000;


TinyGPS gps;

SoftwareSerial ss(A3, A4);
int sats = 0 ;
void setup();
void loop();
#line 84 "/home/jimmy/Documents/PlatformIO/Projects/240407-212751-genericSTM32F103C8/src/GPSDO.ino"
void setup()
{







  ss.begin(9600);

  pinMode(SATLED, OUTPUT);
  digitalWrite(SATLED, HIGH);
  delay(2000);
  digitalWrite(SATLED, LOW);


   for(int i = 0; i < sizeof(UBLOX_INIT); i++) {
    ss.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5);
  }


  Serial.begin(9600);
  Serial.println("= sent init string to GPS =");



  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  bool i2c_found = 0;
  while (true) {
    i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_0PF, 24000000, 0);
    if(!i2c_found)
    {
      Serial.println("Device not found on I2C bus!");
      delay(1000);
      continue;
    }



    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(100000000ULL, SI5351_CLK0);
    si5351.set_freq(1000000000ULL, SI5351_CLK1);
    si5351.set_freq(1400000000ULL, SI5351_CLK2);

    si5351.update_status();
    delay(500);
    break;
  }
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;


  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      Serial.write(c);
      if (gps.encode(c))
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    sats = gps.satellites();
    Serial.println(sats);


    if (sats >= 3){
      digitalWrite(SATLED, HIGH);
      havefix=2;
      havefix++;
      if (havefix >= 3)havefix=3;
      Serial.println(); Serial.print("SAT_HAVE_FIX: "); Serial.println(havefix);
    };
  }

  if (havefix >= 1) havefix--;
  if (havefix==0) { digitalWrite(SATLED, LOW ); };
  Serial.print("MAIN_HAVE_FIX: "); Serial.println(havefix);

  gps.stats(&chars, &sentences, &failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  si5351.update_status();
  Serial.print("  SYS_INIT: ");
  Serial.print(si5351.dev_status.SYS_INIT);
  Serial.print("  LOL_A: ");
  Serial.print(si5351.dev_status.LOL_A);
  Serial.print("  LOL_B: ");
  Serial.print(si5351.dev_status.LOL_B);
  Serial.print("  LOS: ");
  Serial.print(si5351.dev_status.LOS);
  Serial.print("  REVID: ");
  Serial.println(si5351.dev_status.REVID);

}