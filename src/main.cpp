//librarys - still need to import
#include <SD.h>
#include <SPI.h>
#include "HX711.h"
#include <L298N.h>

#define LC_CALIBRATION_FACTOR 9771.09643232 // not actual value, just a placeholder for the time being.
#include "RotaryEncoder.h"
#include <Wire.h>

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
// const byte myBitmapallArray_LEN = 1;
// const unsigned char* myBitmapallArray[1] = {
// 	Mines_Logo
// };

// #include "TestControl.h"

// Pins used so far 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5
// Pins not used 0, 1, 3, 7

//pin setup
const byte motor1pin1 = 3;
const byte motor1pin2 = 2; // may get rid of and short in order to only drive in one direction
const byte motor1speedpin = 9;
L298N linear_motor(motor1speedpin,motor1pin1,motor1pin2);
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&Wire, OLED_RESET);
RotaryEncoder selector(4,5,7);

const byte motor2pin1 = 4;
//const byte motor2pin2 = 7; // ditto
const byte motor2speedpin = 5;
L298N myMotor(motor1pin1, motor1pin2);

const float ZN_coeff[3] = {0.33,0.66,0.11};

SpeedControl LinearMotor;
const byte loadcell_dt = 6;
const byte loadcell_sck = A0;

const byte sd_cs = 10;
const byte sd_sck = 13;
const byte sd_mosi = 11; //must be 11
const byte sd_miso = 12; //must be 12
byte file_num;
char file_num_char[3];
char filename[13];


// const byte eStopPin = 12;

// SoftwareSerial espSerial(8, A3); //(RX, TX) A voltage divider is needed for the A3 pin




HX711 scale;

float lc_reading=0;

void setup() {

  filename[0] = '\0';
  // put your setup code here, to run once:
  Serial.begin(9600);
  // espSerial.begin(9600); //if seeing gibberish change 9600 to 115200

  while (!Serial) {}

  //display initialization
  // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //   Serial.println("!init disp");
  //   for(;;); // Don't proceed, loop forever
  // }
  selector.begin();

  // display.clearDisplay();
  // // display.drawBitmap(0,0,Mines_Logo,128,32,WHITE);
  // display.display();  //initialize display upon boot
  // display.setTextColor(WHITE);
  // display.setTextSize(1);

  //Estop button

  //Driver pins


  //Load cell pins
  // scale.begin(loadcell_dt, loadcell_sck);
  // digitalWrite(loadcell_sck, LOW); //start low


  //Ammeter pins
  // pinMode(ammeterPin, INPUT);

  //SD Card initializtion
  Serial.print("Initializing SD card...");
  if (!SD.begin(sd_cs)) {
    Serial.println("SD card initialization failed");
    // display.setCursor(0,0);
    // display.println(F("SD err"));
    // display.display();
    while(true);
  }
  Serial.println("initialization done.");

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

  //Should create a new file.
  file_num=0;
  
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
  

    //Constants
    int w_angularVelocity = 3; // NOT THE ACTUAL NUMBER DO NOT DO 200!!!!
    const float radius = 0.073; // in meters

  //Get slip value
  // Serial.println("Please input slip value: ");
  // int slipValue = getSerialInput();
  float slipValue=Test_setup();
  // float slipValue = 0.5;
  Serial.println(slipValue);


  
  // float linearVelocity = (1-slipValue)*w_angularVelocity*radius;
  // float wheelAngularVelocity = linearVelocity*spoolRadius; // not totally sure on this calculation have someone double check

  // int pwmValue = map(wheelAngularVelocity, 0, w_angularVelocity, 0, 255);
  // pwmValue = constrain(pwmValue, 0, 255); // Ensure within limits

  lc_reading = scale.get_units();
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
    myfile.print(slipValue);
    myfile.print(",");
    myfile.print(linearVelocity);
    myfile.print(",");
    myfile.println(Force); 
    Serial.println("writing to file");
}

float Test_setup(){
  float slip_ratio = 0;
  while(true){
    // Serial.println(selector.count);
    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(WHITE);
    // display.println(F("Slip:"));

    if(selector.RotaryPressed()){
      break;
    }
    selector.ReadRotary();
    slip_ratio = selector.count%21*0.05;
    // Serial.println(selector.count);
    
  //Display the values the user is selecting
    // display.display();
    // display.setCursor(0,0);
  }
  // display.clearDisplay();
  // display.display();
  // slip_ratio = 0.5;
  return slip_ratio;
}
void motorControlSetup(SpeedControl& controller, const uint8_t pin_dir, const uint8_t pwm_pin, const uint8_t encA_pin, const uint8_t encB_pin, const float KU, const float TU){
  controller.setPin(pin_dir,pwm_pin,encA_pin,encB_pin);//test pins
  controller.setReversePolarity(false);
  controller.setCPR(2821);
  controller.setMotorMaxSpeed(4.183); //Hz
  controller.setPIDValue(KU*ZN_coeff[0],KU*ZN_coeff[1]/TU,KU*TU*ZN_coeff[2]);
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

/**
 * @brief Create a And Open a file on the SD card with a new file number. Opens to file object passed as parameter.
 * 
 * @return true 
 * @return false 
 */
bool createAndOpen(File &newfile){
  do{
    filename[0] = '\0';
    sprintf(file_num_char,"%d",file_num);
    strcat(filename,"test");
    strcat(filename,file_num_char);
    strcat(filename,".txt");
    file_num++;
  //open file
  }while(SD.exists(filename));
  Serial.print("Attempting to open ");
  Serial.println(filename);

  newfile = SD.open(filename,FILE_WRITE);
  if (newfile) {
    Serial.print("Opening: ");
    Serial.print(filename);
    return true;
  }
  else {
    Serial.println("error opening testbench.txt");
    return false;
  }
  newfile.println("Slip value,Linear velocity,Force");

}


