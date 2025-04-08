#include <Arduino.h>

//librarys - still need to import
// #include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <L298N.h>
#include <HX711.h>

#define LC_CALIBRATION_FACTOR 10 // not actual value, just a placeholder for the time being.

// Pins used so far 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, A0, A1, A2, A3, A4, A5
// Pins not used 0, 1, 13

//pin setup


const int loadcell_dt = 6;
const int loadcell_sck = A5;



// SoftwareSerial espSerial(8, A3); //(RX, TX) A voltage divider is needed for the A3 pin



HX711 scale;

int lc_reading=0;

void setup() {

//Basic Loadcell setup
  scale.begin(loadcell_dt, loadcell_sck);// TODO look into using the same sck for the SD card.
  scale.set_scale(LC_CALIBRATION_FACTOR);
  scale.tare();

  //All readings using the load cell going forward should just call scale.get_units()
}

/*
#
#
#
#
#
*/

void loop() {
  lc_reading = round(scale.get_units());
}

void LC_calibration_test(){
  if(scale.is_ready()){
    scale.set_scale();
    Serial.println(F("Remove weight from load cell. Calibration will be complete in 5 seconds..."));
    delay(5000);
    scale.tare();
    Serial.println(F("calibration complete."));
    Serial.println(F("Place weight on scale"));
    delay(2000);
    long reading = scale.get_units(10);
    Serial.print(F("result: "));
    Serial.println(reading);
  }
}


