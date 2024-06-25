#ifndef CONTROLLERSMK_H
#define CONTROLLERSMK_H

#include <Arduino.h>

/***********PUSH BUTTON AND TOGGLE BUTTON CONTROLLER**********************************************/
uint16_t button_controller(uint8_t buttonGpioPin, uint8_t buttonIndex, uint8_t buttonType); 

/*************POTENTIOMETERS CONTROL TYPE AND DIRECTIOMN*******************************/
uint16_t potentiometer_controller(uint8_t potValue, uint8_t potDirection, uint8_t potGpioPin);



/*************JOYSTICKS CONTROL TYPE AND DIRECTIOMN*******************************/
uint16_t joystick_controller(int joyGpio, int joyValue, int joyDirection, int calibrateValue);


/*************JOYSTICKS CALIBRATION*******************************/
void joystick_calibration(int joyGpio, int joyValue, int joyDirection, int joyCalibrationAddress);


/************ROTARY ENCODER***********************************************************************/
uint16_t rotary_encoder_controller(uint16_t encoderCLK, uint16_t encoderDT,uint8_t index,uint16_t rotaryValue, uint8_t rotaryDirection);

#endif
