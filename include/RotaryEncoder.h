#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include <Arduino.h>

//TODO - software interrupts in constructor from encoder.h

class RotaryEncoder{
    public:
        RotaryEncoder(uint8_t _pinA,uint8_t _pinB,uint8_t _button_pin);
        uint8_t pinA,pinB, buttonPin;
        short count;
        byte ReadRotary();
        bool begin();
        bool RotaryPressed();
        bool button_prev_pressed;
        bool aLastState;
        bool aState;
  };
  
  RotaryEncoder::RotaryEncoder(uint8_t _pinA, uint8_t _pinB,uint8_t _buttonPin){
    pinA = _pinA;
    pinB = _pinB;
    buttonPin = _buttonPin;
    count = 0;
    button_prev_pressed=false;
  
  
    button_prev_pressed = digitalRead(buttonPin);
    aLastState = digitalRead(pinA);
  }

  bool RotaryEncoder::begin(){
    pinMode(pinA,INPUT_PULLUP);
    pinMode(pinB,INPUT_PULLUP);
    pinMode(buttonPin, INPUT_PULLUP);
    return true;
  }
  
  
  bool RotaryEncoder::RotaryPressed(){
    bool button_read = digitalRead(buttonPin);
    bool pressed = false;
    if(button_read==LOW && button_prev_pressed==HIGH){
      pressed = true;
    }
    button_prev_pressed = button_read;
    return pressed;
  }
  
  byte RotaryEncoder::ReadRotary(){ 
    aState = digitalRead(pinA); // Reads the "current" state of the outputA
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (aState != aLastState){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if (digitalRead(pinB) != aState) { 
        count++;
      } else {
        count--;
      }
    } 
    aLastState = aState;
  }

#endif