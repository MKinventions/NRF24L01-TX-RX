#include <EEPROM.h>
#include "controllersMK.h"


/************button controllers start****************************/
int buttonState[20]; // 20 buttons can connect
int lastButton[20];
int currentButton[20];
int debounceDelay = 10;
unsigned long previousMillis = 0;
/************button controllers end*******************************/

/************rotary encoder start*********************************/
int currentStateCLK[5]; // 5 rotary encoders can connect
int lastStateCLK[5];
int encoder_counter[5];
// int debounceDelayRot = 100;
// unsigned long previousMillisRot = 0;
// int encoder1_counter = 0;
// int encoder1_address = 50;
/************rotary encoder end**********************************/




/**************************************************************************************************/
/***********PUSH BUTTON AND TOGGLE BUTTON CONTROLLER**********************************************/
/**************************************************************************************************/
uint16_t button_controller(uint8_t buttonGpioPin, uint8_t buttonIndex, uint8_t buttonType) {
  
  
  if (buttonType == 1) {
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= debounceDelay){
    currentButton[buttonIndex] = digitalRead(buttonGpioPin);
    if (currentButton[buttonIndex] != lastButton[buttonIndex]) {
      lastButton[buttonIndex] = currentButton[buttonIndex];
               previousMillis = currentMillis;
      
      if (lastButton[buttonIndex] == 1) {
        buttonState[buttonIndex] = (buttonState[buttonIndex] == 1) ? 0 : 1;
      }
    }
    }
  } else if (buttonType == 0) {
    buttonState[buttonIndex] = digitalRead(buttonGpioPin);
  }
  return buttonState[buttonIndex];
}


/*************POTENTIOMETERS & jOYSTICKS CONTROL TYPE AND DIRECTIOMN*******************************/
uint16_t potentiometer_controller(uint8_t potValue, uint8_t potDirection, uint8_t potGpioPin){ 
  uint8_t controller = 0;
  uint16_t potReadValue = analogRead(potGpioPin); 
       if(potDirection == 0){ controller = map(potReadValue, 0, 4095, 0, potValue);}
  else if(potDirection == 1){ controller = map(potReadValue, 0, 4095, potValue, 0);}
return controller;
}

uint16_t joystick_controller(int joyGpio, int joyValue, int joyDirection, int calibrateValue) {
  int joystickFinal;
  int joyLevel;

  //  Serial.print(", ReadCalibratedValue:" + String(calibrateValue));

  if (joyDirection == 0) {
    joystickFinal = map(analogRead(joyGpio), 4095, 0, joyValue + (calibrateValue * 2), 0);
  }
  else if (joyDirection == 1) {
    joystickFinal = map(analogRead(joyGpio), 4095, 0, 0, joyValue - (calibrateValue * 2));
  }
  //  Serial.print(", joystickFinal:" + String(joystickFinal));
  //  Serial.print(", joystickFinalCopy:" + String(joystickFinalCopy));

  int joystickFinalCopy = joystickFinal;
  int midPointValue = joyValue / 2;

  if (joystickFinalCopy > midPointValue+2 || joystickFinalCopy < midPointValue-2) {
    joyLevel = joystickFinal;
  } else {
    joyLevel = midPointValue;// 90 if 180 (or) 127 if 255
  }

  if (joyValue == 255 && joystickFinal >= 240) {
    joyLevel = 255;
  }
  if (joyValue == 180 && joystickFinal >= 170) {
    joyLevel = 180;
  }
  //  else {
  //    joyLevel = joystickFinal;
  //  }

  return joyLevel;

}


void joystick_calibration(int joyGpio, int joyValue, int joyDirection, int joyCalibrationAddress) {
  int readRawValue = analogRead(joyGpio);
  Serial.print("Raw:" + String(readRawValue));
  int midPointValue = joyValue / 2;
  Serial.print(", mid:" + String(midPointValue));


  int calibrate;

  if (joyDirection == 0) {
    int mapRawValue = map(readRawValue, 4095, 0, joyValue, 0);
    Serial.print(", map:" + String(mapRawValue));

    calibrate = midPointValue - mapRawValue;
    Serial.print(", calibrate:" + String(calibrate));
  }
  else if (joyDirection == 1) {
    int mapRawValue = map(readRawValue, 4095, 0, 0, joyValue);
    Serial.print(", map:" + String(mapRawValue));

    calibrate = -(midPointValue - mapRawValue);
    Serial.println(", calibrate:" + String(calibrate));
  }

  EEPROM.write(joyCalibrationAddress, calibrate);
  EEPROM.commit();


}




/**************************************************************************************************/
/************ROTARY ENCODER***********************************************************************/
/**************************************************************************************************/
uint16_t rotary_encoder_controller(uint16_t encoderCLK, uint16_t encoderDT,uint8_t index,uint16_t rotaryValue, uint8_t rotaryDirection){



  // Read the current state of CLK
    currentStateCLK[index] = digitalRead(encoderCLK);  
      if (currentStateCLK[index] != lastStateCLK[index]  && currentStateCLK[index] == 1){
 

        if(rotaryDirection == 0){//rotate counter clock wise
          if (digitalRead(encoderDT) != currentStateCLK[index]) {        
            encoder_counter[index] --;          
          } else {
            encoder_counter[index] ++;
          }
        }
        else if(rotaryDirection == 1){ //rotate clock wise
          if (digitalRead(encoderDT) != currentStateCLK[index]) {        
            encoder_counter[index] ++;          
          } else {
            encoder_counter[index] --;
          }
        }
      }
    

  if(encoder_counter[index] < 0){
    encoder_counter[index] = 0;
  }
  if(encoder_counter[index] > rotaryValue){
    encoder_counter[index] = rotaryValue;
  }
        // Remember last CLK state
        lastStateCLK[index] = currentStateCLK[index];
        
    
 
  return encoder_counter[index];
}
