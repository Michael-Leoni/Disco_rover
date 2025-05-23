//librarys - still need to import
#include <SD.h>
#include <SPI.h>
#include "HX711.h"
#include <Encoder.h>
#include "SpeedControl.h"

#define LC_CALIBRATION_FACTOR 9771.09643232 // not actual value, just a placeholder for the time being.
#define CPR 2821
#define GEAR_RATIO 10
// #include "RotaryEncoder.h"
#include <Wire.h>



// Pins used so far 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5
// Pins not used 0, 1, 3, 7

//pin setup

const float ZN_coeff[3] = {0.33,0.66,0.11};

//DIMENSIONS
const float wheel_diameter = 0.13; //Meters
const float belt_spool_diameter = 1.22; //CM //I DO NOT THINK THIS IS CORRECT
const float test_distance = 12*2.54; //CM //This will likely need to be shorter.

SpeedControl LinearMotor;
SpeedControl WheelMotor;

const byte loadcell_dt = 6;
const byte loadcell_sck = 8;


//SD CARD
const byte sd_cs = 10;

byte file_num=0;
char file_num_char[3];
char filename[13];

File testFile;

//TEST PARAMETERS
const float w_anglarVelocity = 3;

const byte start_button = 7;

// const byte eStopPin = 12;

//FUNCTION DECLARATIONS
float Test_setup();
void motorControlSetup(SpeedControl& controller, const uint8_t pin_dir, const uint8_t pwm_pin, const uint8_t encA_pin, const uint8_t encB_pin, const float KU, const float TU);
bool createAndOpen(File &newfile);
bool recordData(File &myfile, float &slipValue, float &linearVelocity, float &Force);
bool buttonDebounceCheck(uint8_t);

HX711 scale;

float lc_reading=0;

void setup() {
  pinMode(7,INPUT_PULLUP);
  filename[0] = '\0';
  // put your setup code here, to run once:
  Serial.begin(9600);
  // espSerial.begin(9600); //if seeing gibberish change 9600 to 115200
  Serial.println("Initializing");


  //Motor setup and pins
  motorControlSetup(LinearMotor,A0,9,2,A1,47,2);
  motorControlSetup(WheelMotor,1,5,3,A2,47,2);




//Loadcell setup.
  scale.begin(loadcell_dt, loadcell_sck);// TODO look into using the same sck for the SD card.
  scale.set_scale(LC_CALIBRATION_FACTOR);
  scale.tare();

//SD Setup
if (!SD.begin(sd_cs)) {
  Serial.println(F("initialization failed!"));
  while (1);
}
Serial.println(F("initialization done."));
  
}

/*
#
#
#                     LOOP DE LOOPS
#
#
*/

bool test_started = false;
float slipValue = 0.0;
void loop() {

unsigned long previous_time=0;
  if (buttonDebounceCheck(start_button)&&slipValue<=1.0){
    //Initialize the file
    createAndOpen(testFile);

    //Get slip value
    slipValue+=0.1;
    // Serial.println("Please input slip value: ");
    // int slipValue = getSerialInput();

    float linear_velocity = ((1-slipValue)*w_anglarVelocity*wheel_diameter); //m/s
    
    LinearMotor.setSpeed(linear_velocity*100/(belt_spool_diameter*M_PI));
    WheelMotor.setSpeed(w_anglarVelocity*GEAR_RATIO);
    // LinearMotor.setSpeed(0);
    // WheelMotor.setSpeed(0);

    //Actual test running.
    linear_velocity = 0;
    float actual_slip=0;

    delay(5000); //Time for us to clear out.
    /*
    * Maybe add some indicator light or some flashing LED to show that the test is starting.
    */
    previous_time = millis();
    do{
      LinearMotor.controlLoop();
      WheelMotor.controlLoop();
      // LinearMotor.stop();
      // WheelMotor.stop();
      //Calculate actual slip
      linear_velocity = LinearMotor.currentVelocity*(belt_spool_diameter*M_PI);
      actual_slip = 1-linear_velocity/(w_anglarVelocity*wheel_diameter*M_PI);
      //Record data
      lc_reading = scale.get_units();
      Serial.println("Prepping to record");
      if(recordData(testFile,actual_slip,linear_velocity,lc_reading)){
        Serial.println(F("Filewrite successful"));
      }
      Serial.println(LinearMotor.currentPosition);
      Serial.println((LinearMotor.currentPosition/CPR));
      // Serial.println((LinearMotor.currentPosition/CPR)*belt_spool_diameter);
    // The termination of this loop needs to be tested still.
    }while((LinearMotor.currentPosition/CPR)*belt_spool_diameter*M_PI<test_distance);//converts current position counts to revolutions, calculates distance based off that and compares
    LinearMotor.setSpeed(0);
    WheelMotor.setSpeed(0);
    LinearMotor.controlLoop();
    WheelMotor.controlLoop();
    //Close file
    testFile.close();
    LinearMotor.stop();
    WheelMotor.stop();
    LinearMotor.reset();
    WheelMotor.reset();
    test_started = false;
  }
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

    // if(selector.RotaryPressed()){
    //   break;
    // }
    // selector.ReadRotary();
    // slip_ratio = selector.count%21*0.05;
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
  controller.setCPR(CPR);
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
    newfile.println("Slip value,Linear velocity,Force");
    return true;
  }
  else {
    Serial.println("error opening testbench.txt");
    return false;
  }

}
//only works for one pin b/c of globals but I'm tired.
unsigned long last_debounce_time = 0;
unsigned long debounceDelay = 50;
bool last_reading = 0;
bool buttonDebounceCheck(const uint8_t pin){
  bool button_state;
  bool reading = !digitalRead(pin);
  if(reading != last_reading){
    last_debounce_time = millis();
  }
  if(millis() - last_debounce_time >debounceDelay){
    if(button_state!=reading){
      button_state = reading;
    }
  }
  last_reading = reading;
  return button_state;
}
