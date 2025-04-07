//librarys - still need to import
#include <SoftwareSerial.h>
#include <SD.h>
#include "HX711.h"
#include <L298N.h>
File testbenchfile;

// Pins used so far 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5
// Pins not used 0, 1, 3, 7

//pin setup
const int motor1pin1 = 2;
const int motor1pin2 = 3; // may get rid of and short in order to only drive in one direction
const int motor1speedpin = 9;
const int motor2pin1 = 4;
//const int motor2pin2 = 7; // ditto
const int motor2speedpin = 5;
L298N myMotor(motor1pin1, motor1pin2);

const int loadcell_dt = 6;
const int loadcell_sck = A5;
HX711 scale; // Create an instance of HX711

const int sd_cs = 10;
const int sd_sck = 13;
const int sd_mosi = 11; //must be 11
const int sd_miso = 12; //must be 12

//const int eStopPin = 3;

SoftwareSerial espSerial(8, A3); //(RX, TX) A voltage divider is needed for the A3 pin

const int encoder1_dt = A2; // enable software interupts during loop
const int encoder1_clk = A1;
const int encoder2_dt = A0;
const int encoder2_clk = A4;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);
  delay(2000);

  /*
  espSerial.begin(9600); //if seeing gibberish change 9600 to 115200
  while (!Serial) {
  }
  */
  
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
  scale.begin(loadcell_dt, loadcell_sck);
  digitalWrite(loadcell_sck, LOW); //start low

  //Encoders pins
  pinMode(encoder1_dt, INPUT);
  pinMode(encoder1_clk, INPUT);
  pinMode(encoder2_dt, INPUT);
  pinMode(encoder2_clk, INPUT);

  // //SD Card Test
  // Serial.print("Initializing SD card...");
  // if (!SD.begin(sd_cs)) {
  //   Serial.println("SD card initialization failed");
  //   while (1);
  // }
  // Serial.println("initialization done.");
 

  // open file for reading
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

  //tare loadcells
//   if (scale.is_ready()) {
//       scale.set_scale();    
//       Serial.println("Tare... remove any weights from the scale.");
//       delay(5000);
//       scale.tare();
//       Serial.println("Tare done...");
//       Serial.print("Place a known weight on the scale...");
//       delay(5000);
//       long reading = scale.get_units(10);
//       Serial.print("Result: ");
//       Serial.println(reading);
//     } 
//   else {
//     Serial.println("HX711 not found.");
//   }
//   delay(1000);
//   //calibration factor will be the (reading)/(known weight)
//   scale.set_scale(calibrationfactor); // Reset scale to default
//   scale.tare(); // Reset to 0
 }

void loop() {

  delay(1000);
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

    //   while (true) {
    //   if (digitalRead(eStopPin) == HIGH) {
    //     Serial.println("E-Stop Released. Restart required.");
    //     delay(1000);
    //     }
    //   }


    //Constants
    int w_angularVelocity = 3; // NOT THE ACTUAL NUMBER DO NOT DO 200!!!!
    const float radius = 0.073; // in meters

    //Get slip value
    // Serial.println("Please input slip value: ");
    int slipValue = 2; //Serial.parseInt();

    float linearVelocity = (1-slipValue)*w_angularVelocity*radius;
    float wheelAngularVelocity = linearVelocity*spoolRadius; // not totally sure on this calculation have someone double check

    int pwmValue = map(wheelAngularVelocity, 0, w_angularVelocity, 0, 255);
    pwmValue = constrain(pwmValue, 0, 255); // Ensure within limits
    // Serial.println(pwmValue);


    digitalWrite(motor1speedpin, 255); //pwmValue instead // digital write??? or analogue funky funky
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    Serial.println("motor Running?");
  
    // //Set motor speed
    // analogWrite(motor1speedpin, pwmValue);
    // analogWrite(motor2speedpin, pwmValue);

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
