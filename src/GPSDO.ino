// programing of the u-blox 7m module from Arduino for timepulse TP-5
// hex was taken from u-center software configuration view
//
//
// plus, flashing LED connected to pin 10 indicates when enough sat's in view
// u-blox module to connect o 3 and 4 for using soft serial of Arduino
// CT2GQV 2019 mixing multiple libs and examples.

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "si5351.h"
Si5351 si5351; // i2c 0x60
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

const char UBLOX_INIT[] PROGMEM = {

// the actual programing string, uncoment for the one needed. 10Mhz, 2.5Mhz, 24Mhz or 2Mhz
// any frequency not integer divide of 48Mhz will have some jitter since module reference is 48. Best use is for 24 or 2 Mhz

/*
 // CFG-TP5 1Hz / 10Mhz sync
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x96,
  0x98, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x08, 0x00, 0x00, 0x7E, 0xA8,
*/

/*
  // CFG-TP5 1Hz / 10Mhz no sync 50ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x96, 0x98, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xA8, 0x08,
*/
/*
  // CFG-TP5 1Hz / 24 Mhz no sync 0ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6E, 0x01,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x6D, 0x8D,
*/

 // CFG-TP5 24MHz / 24 Mhz - allways outputing 24Mhz either in sync with GPS or not. LED on D10 will indicate GPS lock.
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6E, 0x01, 0x00, 0x36,
  0x6E, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x11, 0xD8,

/*
  // CFG-TP5 1Hz / 2.5Mhz no sync 50ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x25, 0x26, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xE5, 0x21,
*/

/*
  // CFG-TP5 1Hz / 2 Mhz no sync 50ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x1C, 0x1E,
*/

/*
  // UBX-CFG-TP5
  0xB5, 0x62, // header
  0x06, 0x31, // time pulse get/set
  0x20,  // lenght 32
  0x00, // tpIdx time pulse selection = 0 = timepulse, 1 = timepulse2  (U1 Char)
  0x00,  // reserved0 U1
  0x01, 0x00, // reserved1 U2
  0x00, 0x32, // antCableDelay ns
  0x00, 0x00, // rf group delay I2
  0x00, 0x90, 0xD0, 0x03, // freqPeriod
  0x00, 0x40, 0x42, 0x0F, // freqPeriodLoc
  0x00, 0xF0, 0x49, 0x02, // pulselenRatio
  0x00, 0x60, 0xAE, 0x0A, // pulselenRatio
  0x00, 0x00, 0x00, 0x00, // userConfigDelay ns
  0x00, 0x77, 0x00, 0x00, // flags - page 135 u-blox 7 Receiver Description Including Protocol Specification V14.pdf
  0x00, 0x48, 0x65,
*/

};

int SATLED = PC13;  // for showing we have statelites and in sync
int havefix = 0; // to keep track if we have fix for at least 3 seconds
float start_freq = 2000000000;


TinyGPS gps;
// SoftwareSerial ss(4, 3);
SoftwareSerial ss(A3, A4);
int sats = 0 ;

void setup()
{
  /*
  lcd.begin(16, 2);                           // LCD set for 16 by 2 display
  lcd.setBacklightPin(3,POSITIVE);            // (BL, BL_POL)
  lcd.setBacklight(HIGH);                     // LCD backlight turned ON

  lcd.setCursor(0, 0);                        //
  lcd.print("Booting...");    */
  ss.begin(9600);

  pinMode(SATLED, OUTPUT); // to indicate we have enough satelites
  digitalWrite(SATLED, HIGH);
  delay(2000);
  digitalWrite(SATLED, LOW);

   // actual u-blox 7m programing
   for(int i = 0; i < sizeof(UBLOX_INIT); i++) {
    ss.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
// ends here

  Serial.begin(9600);
  Serial.println("= sent init string to GPS =");
  /*lcd.setCursor(0, 0); lcd.print("NO DATA   ");*/


  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  bool i2c_found = 0;
  while (true) {
    i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, 0);
    if(!i2c_found)
    {
      Serial.println("Device not found on I2C bus!");
      delay(1000);
      continue;
    }

    // on the library Si5351/si5351.h chang it to 24Mhz
    // #define SI5351_XTAL_FREQ                                                25000000
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(100000000ULL, SI5351_CLK0); //  1.000 Mhz for the marker
    si5351.set_freq(1000000000ULL, SI5351_CLK1); // 10.000 Mhz for the ext 10Mhz of counter
    si5351.set_freq(1400000000ULL, SI5351_CLK2); // 14Mhz

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

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
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
    // lcd.setCursor(0, 0); lcd.print("Satelites:");lcd.print(sats);

    if (sats >= 3){ // we have sats in view, we should be good
      digitalWrite(SATLED, HIGH); // delay(200);digitalWrite(SATLED, LOW );
      havefix=2;
      havefix++; // increments every time we have fix until 3, if we don't have fix/sats then after 3 cycles it will go zero and we shutt the indicator led.
      if (havefix >= 3)havefix=3; // never goes over 3
      Serial.println(); Serial.print("SAT_HAVE_FIX: "); Serial.println(havefix);
    };
  } // end new data

  if (havefix >= 1) havefix--; // we had allready 1 fix in the loop
  if (havefix==0) { digitalWrite(SATLED, LOW ); }; // we don't have fix so power off the fix LED
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
//=end code==============