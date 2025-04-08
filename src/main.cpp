//librarys - still need to import
#include <SoftwareSerial.h>
#include <SD.h>
#include "HX711.h"
#include <L298N.h>

#define LC_CALIBRATION_FACTOR 9771.09643232 // not actual value, just a placeholder for the time being.
#include "RotaryEncoder.h"
#include <Wire.h>

#include<Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
#define SENSOR_ADDRESS 0X38
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define RIGHT_OFFSET 55

// 'Mines-Logo-triangle-blue', 128x32px
const unsigned char Mines_Logo[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x50, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0c, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x98, 0x18, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x5c, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x02, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x81, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01, 0x80, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x02, 0x40, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x04, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x06, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x04, 0xe0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x07, 0xe0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x81, 0x21, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 528)
const int myBitmapallArray_LEN = 1;
const unsigned char* myBitmapallArray[1] = {
	Mines_Logo
};

// #include "TestControl.h"
File testbenchfile;

// Pins used so far 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5
// Pins not used 0, 1, 3, 7

//pin setup
const int motor1pin1 = 3;
const int motor1pin2 = 2; // may get rid of and short in order to only drive in one direction
const int motor1speedpin = 9;
L298N linear_motor(motor1speedpin,motor1pin1,motor1pin2);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&Wire, OLED_RESET);
RotaryEncoder selector(4,5,7);

const int motor2pin1 = 4;
//const int motor2pin2 = 7; // ditto
const int motor2speedpin = 5;
L298N myMotor(motor1pin1, motor1pin2);

const int loadcell_dt = 6;
const int loadcell_sck = A0;

const int sd_cs = 10;
const int sd_sck = 13;
const int sd_mosi = 11; //must be 11
const int sd_miso = 12; //must be 12

const int eStopPin = 12;

SoftwareSerial espSerial(8, A3); //(RX, TX) A voltage divider is needed for the A3 pin

const int encoder1_dt = A2; // enable software interupts during loop
const int encoder1_clk = A1;
const int encoder2_dt = A0;
const int encoder2_clk = A4;
const int ammeterPin = 11;


HX711 scale;

int lc_reading=0;

void LC_calibration_test();
float Test_setup();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  // espSerial.begin(9600); //if seeing gibberish change 9600 to 115200
  while (!Serial) {}

  // //display initialization
  // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //   for(;;); // Don't proceed, loop forever
  // }
  // selector.begin();

  // display.clearDisplay();
  // display.drawBitmap(0,0,Mines_Logo,128,32,WHITE);
  // display.display();  //initialize display upon boot
  // display.setTextColor(WHITE);
  // display.setTextSize(1);

  //Estop button
  //pinMode(eStopPin, INPUT_PULLUP);

  //Driver pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  // pinMode(motor2pin2, OUTPUT);
  pinMode(motor1speedpin, OUTPUT);
  pinMode(motor2speedpin, OUTPUT);

  //Load cell pins
  // scale.begin(loadcell_dt, loadcell_sck);
  // digitalWrite(loadcell_sck, LOW); //start low

  //Encoders pins
  pinMode(encoder1_dt, INPUT);
  pinMode(encoder1_clk, INPUT);
  pinMode(encoder2_dt, INPUT);
  pinMode(encoder2_clk, INPUT);

  //Ammeter pins
  // pinMode(ammeterPin, INPUT);

  //SD Card initializtion
  // Serial.print("Initializing SD card...");
  // if (!SD.begin(sd_cs)) {
  //   Serial.println("SD card initialization failed");
  //   display.setCursor(0,0);
  //   display.println(F("SD err"));
  //   display.display();
  //   while(true);
  // }
  // Serial.println("initialization done.");

  // testfile = SD.open("test.txt", FILE_WRITE);
  // if (testfile) {
  //   Serial.print("Writing to test.txt...");
  //   // test print in file
  //   testfile.println("testing 1, 2, 3.");
  //   // close file
  //   testfile.close();
  //   Serial.println("file closed.");
  // }
  // else {
  //   Serial.println("error opening test.txt");
  // }

  //open file for reading
  // testfile = SD.open("test.txt");
  // if (testfile) {
  //   Serial.println("Reading from rest.tst:");
  //   while (testfile.available()) {
  //     Serial.write(testfile.read());
  //   }
  //   testfile.close();
  // }
  // else {
  //   Serial.println("error opening test.txt");
  // }

  delay(1500);


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
  Serial.println(scale.get_units(),1);

  // delay(1000);
  // testbenchfile = SD.open("testbench.txt", FILE_WRITE);
  // if (testbenchfile) {
  //   Serial.print("testbench.txt open...");

    float spoolRadius = 0.151; //measure and find value (meters) - not accurate rn

    // //ESP8266 stuff
    // if (espSerial.available()) {
    //   Serial.write(espSerial.read());  // Read from ESP8266 and send to Serial Monitor
    // }
    
    // if (Serial.available()) {
    //   espSerial.write(Serial.read());  // Send Serial Monitor input to ESP8266
    // }

    //Estop
    // if (digitalRead(eStopPin) == LOW) {
    //   Serial.println("EMERGENCY STOP ACTIVATED!");

    //   //Stop motors
    //   digitalWrite(motor1pin1, LOW);
    //   digitalWrite(motor2pin1, LOW);
      
    //   analogWrite(motor1speedpin, 0);
    //   analogWrite(motor2speedpin, 0);


  

    //Constants
    int w_angularVelocity = 3; // NOT THE ACTUAL NUMBER DO NOT DO 200!!!!
    const float radius = 0.073; // in meters

  //Get slip value
  // Serial.println("Please input slip value: ");
  // int slipValue = getSerialInput();
  // float slipValue=Test_setup();
  // Serial.println(slipValue);
  
  // float linearVelocity = (1-slipValue)*w_angularVelocity*radius;
  // float wheelAngularVelocity = linearVelocity*spoolRadius; // not totally sure on this calculation have someone double check

  // int pwmValue = map(wheelAngularVelocity, 0, w_angularVelocity, 0, 255);
  // pwmValue = constrain(pwmValue, 0, 255); // Ensure within limits


    // //Move forward
    // digitalWrite(motor1pin1,  HIGH);
    // digitalWrite(motor2pin1, HIGH);
    // delay(3000);

    //record force and amperage

    //Write in SD card, slip value, linear velocity, force (Load Cell), amperage (ammeter)
    // testbenchfile.println("Slip value,Linear velocity,Force,Amperage");
    // testbenchfile.print(slipValue);
    // testbenchfile.print(",");
    // testbenchfile.print(linearVelocity);
    // testbenchfile.print(",");
    // //testbenchfile.print(force);
    // testbenchfile.print(",");
    // //testbenchfile.print(amperage);

    // close file
    // testbenchfile.close();
    // Serial.println("file closed.");
  // }
  // else {
  //   Serial.println("error opening testbench.txt");
  // }

}

/**
 * @brief Writes parameter data to an opened csv file line.
 * 
 * @param myfile opened and initialized SD card file
 * @param slipValue Slip value for test
 * @param linearVelocity Linear Velocity of Test
 * @param Force Force recorded from Load cell.
 * @return true 
 * @return false 
 */
bool recordData(File &myfile, float &slipValue, float &linearVelocity, float &Force){
  //Write in SD card, slip value, linear velocity, force (Load Cell), amperage (ammeter)
    myfile.println("Slip value,Linear velocity,Force,Amperage");
    myfile.print(slipValue);
    myfile.print(",");
    myfile.print(linearVelocity);
    myfile.print(",");
    myfile.println(Force);
}



float Test_setup(){
  float slip_ratio = 0;
  while(true){
    // Serial.println(selector.count);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println(F("Slip:"));

    if(selector.RotaryPressed()){
      break;
    }
    selector.ReadRotary();
    slip_ratio = selector.count%21*0.05;
    
  //Display the values the user is selecting
    display.print(slip_ratio);
    display.display();
    display.setCursor(0,0);
  }
  display.clearDisplay();
  display.display();
  return slip_ratio;

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


