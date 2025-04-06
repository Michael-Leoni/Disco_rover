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

const int motor1pin1 = 2;
const int motor1pin2 = 3; // may get rid of and short in order to only drive in one direction
const int motor1speedpin = 9;
L298N linear_motor(motor1speedpin,motor1pin1,motor1pin2);


const int motor2pin1 = 4;
const int motor2pin2 = 7; // ditto
const int motor2speedpin = 5;

const int loadcell_dt = 6;
const int loadcell_sck = A5;

const int sd_cs = 10;

const int eStopPin = 12;

// SoftwareSerial espSerial(8, A3); //(RX, TX) A voltage divider is needed for the A3 pin

const int encoder1_dt = A2; // enable software interupts during loop
const int encoder1_clk = A1;
const int encoder2_dt = A0;
const int encoder2_clk = A4;

const int ammeterPin = 11;

File testfile;
HX711 scale;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  // espSerial.begin(9600); //if seeing gibberish change 9600 to 115200
  while (!Serial) {
  }

  //Estop button
  pinMode(eStopPin, INPUT_PULLUP);


  //Driver pins
  // pinMode(motor1pin1, OUTPUT);
  // pinMode(motor1pin2, OUTPUT);
  // pinMode(motor2pin1, OUTPUT);
  // pinMode(motor2pin2, OUTPUT);
  // pinMode(motor1speedpin, OUTPUT);
  // pinMode(motor2speedpin, OUTPUT);

  //Load cell pins

  //Encoders pins
  pinMode(encoder1_dt, INPUT);
  pinMode(encoder1_clk, INPUT);
  pinMode(encoder2_dt, INPUT);
  pinMode(encoder2_clk, INPUT);

  //Ammeter pins
  // pinMode(ammeterPin, INPUT);

  //SD Card Test
  Serial.print("Initializing SD card...");
  if (!SD.begin(sd_cs)) {
    Serial.println("SD card initialization failed");
    while (1);
  }
  Serial.println("initialization done.");

  testfile = SD.open("test.txt", FILE_WRITE);
  if (testfile) {
    Serial.print("Writing to test.txt...");
    // test print in file
    testfile.println("testing 1, 2, 3.");
    // close file
    testfile.close();
    Serial.println("file closed.");
  }
  else {
    Serial.println("error opening test.txt");
  }

  //open file for reading
  testfile = SD.open("test.txt");
  if (testfile) {
    Serial.println("Reading from rest.tst:");
    while (testfile.available()) {
      Serial.write(testfile.read());
    }
    testfile.close();
  }
  else {
    Serial.println("error opening test.txt");
  }

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

  float spoolRadius = 0.02; //measure and find value (meters) TODO

  //ESP8266 stuff
  // if (espSerial.available()) {
  //   Serial.write(espSerial.read());  // Read from ESP8266 and send to Serial Monitor
  // }
  
  // if (Serial.available()) {
  //   espSerial.write(Serial.read());  // Send Serial Monitor input to ESP8266
  // }

  //Estop
  if (digitalRead(eStopPin) == LOW) {
    Serial.println("EMERGENCY STOP ACTIVATED!");

    //Stop motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
    
    analogWrite(motor1speedpin, 0);
    analogWrite(motor2speedpin, 0);

    while (true) {
    if (digitalRead(eStopPin) == HIGH) {
      Serial.println("E-Stop Released. Restart required.");
      delay(1000);
      }
    }
  }

  //Load Cell test code / for calibration
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

  //Constants

  int w_angularVelocity=200; // NOT THE ACTUAL NUMBER DO NOT DO 200!!!!
  const float radius = 0.073; // in meters

  //Get slip value
  // Serial.println("Please input slip value: ");
  // int slipValue = getSerialInput();
  int slipValue=5;
  
  float linearVelocity = (1-slipValue)*w_angularVelocity*radius;
  float wheelAngularVelocity = linearVelocity*spoolRadius; // not totally sure on this calculation have someone double check

  int pwmValue = map(wheelAngularVelocity, 0, w_angularVelocity, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255); // Ensure within limits

  //Set motor speed
  // analogWrite(motor1speedpin, pwmValue);
  // analogWrite(motor2speedpin, pwmValue);

  Serial.println(pwmValue);
  linear_motor.setSpeed(255);
  linear_motor.run(L298N::FORWARD);
  delay(3000);
  linear_motor.setSpeed(255);



  //Move forward
  linear_motor.run(L298N::BACKWARD);
  delay(3000);
  // digitalWrite(motor1pin1,  HIGH);
  // digitalWrite(motor1pin2, LOW);
  // digitalWrite(motor2pin1, HIGH);
  // digitalWrite(motor2pin2, LOW);
  // delay(3000);

  //Write in SD card, slip value, linear velocity, force (Load Cell), amperage (ammeter)
}


