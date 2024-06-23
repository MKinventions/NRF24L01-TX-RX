#include <EEPROM.h>
#include "EEPROM.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "controllersMK.h"//custom library
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoJson.h>






/*********OLED DISPLAY START*********************************************/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/**********OLED DISPLAY END****************************************************/


int pagesCount = 0;
int MainMenuSelect = 0;
int SubMenuSelect = 0;
int totalPages = 9;
int totalMenus = 10;
int totalSubMenus = 6;
int debounceDelayMenu = 100;
unsigned long previousMillisMenu = 0;


/*********controllers gpio pins start***********/
#define CE_RST 4 
#define CSN_NSS 5
#define IRQ_DIO0 2

#define nextPageButton  15
#define MainMenuButton 16
#define SubMenuButton 17
#define sendDataButton 13

#define joystick1 34
#define joystick2 36
#define potentiometer1 35
#define potentiometer2 39
#define joyButton 14
#define encoderButton 25
#define switch1 27
#define switch2 26
#define encoder_DT 33
#define encoder_CLK 32
///*********controllers gpio pins end***********/



int button1State = 0;
int button2State = 0;
int switch1State = 0;
int switch2State = 0;
int statusButton = 0;
int pageDown = 0;
int pageUp = 0;
int menu = 0;
int subMenu = 0;


int lastButton1 = 0;
int lastButton2 = 0;

int encoder1_counter = 0;

/********EEPROM memory start***********************/
int pot1_address = 1;
int pot2_address = 2;
int joy1_address = 3;
int joy2_address = 4;
int pot1Dir_address = 5;
int pot2Dir_address = 6;
int joy1Dir_address = 7;
int joy2Dir_address = 8;
int btn1Dir_address = 9;
int btn2Dir_address = 10;
int btn3Dir_address = 11;
int btn4Dir_address = 12; 

int rot1_address = 13;
int rot1Dir_address = 14;
int rot1ControlType_address = 32;

int pageCount_address = 15;
int servo1Dir_address = 22;
int servo2Dir_address = 23;
int motor1Dir_address = 24;
int motor2Dir_address = 25;
int motor3Dir_address = 26;
int motor4Dir_address = 27;
int dataSpeed_address = 28;
int communication_address = 29;
int radio_address = 30; 
int receiver_address = 31;

byte pot1_value, pot2_value, joy1_value, joy2_value;
byte pot1Dir_value, pot2Dir_value, joy1Dir_value, joy2Dir_value;
byte rot1_value, rot1Dir_value, rot1ControlType_value;
byte btn1Dir_value,btn2Dir_value;
byte servo1Dir_value, servo2Dir_value;
byte motor1Dir_value,motor2Dir_value,motor3Dir_value,motor4Dir_value;
byte dataSpeed_value = 3;
byte communication_value = 3;
byte radio_value = 0;
byte receiver_value = 0;
/***************EEPROM memory end***********/


int calibrationButton = 0;
int calibration_address = 50;
int joy1CalibrateValue;
int joy1CalibrationAddress = 51;
int joy2CalibrateValue;
int joy2CalibrationAddress = 52;

int bindingRx1Button = 0;
int bindingRx1_address = 60;
int bindingRx2Button = 0;
int bindingRx2_address = 61;
int bindingRx3Button = 0;
int bindingRx3_address = 62;

const int rx1Addr = 80; // Address in EEPROM to store the receiver address
const int rx2Addr = 90; // Address in EEPROM to store the receiver address
const int rx3Addr = 100; // Address in EEPROM to store the receiver address
uint32_t readRx1Address;
uint32_t readRx2Address;
uint32_t readRx3Address;
uint32_t chipId = 0;


/********LORA RECEIVE*********/
//const char* fwVersionSlave1;
int connectionStatus = 0;
int rsi, snr;
float tmp;
int humi;
int btn1Status,btn2Status,sw1Status,sw2Status;
int pot1Status,pot2Status;
double latitude;
double longitude;
double voltage;
double distance;

void memorySaving();
void buttonsAndCotrollers(void *pvParameters);



int pot1cursor, pot2cursor, joy1cursor, joy2cursor;
int pot1_level = 0;
int pot2_level = 0;
int joy1_level = 0;
int joy2_level = 0;

int dataSpeed;


String firmwareVersionMaster = "v1.1.3";

String connection; 
String btn1Dir_Type;
String btn2Dir_Type;
String pot1Dir_Type;
String pot2Dir_Type;
String joy1Dir_Type;
String joy2Dir_Type;
String rot1Dir_Type;
String rot1Control_Type;
String servo1Dir_Type, servo2Dir_Type;
String motor1Dir_Type, motor2Dir_Type, motor3Dir_Type, motor4Dir_Type;
String dataSpeed_Type;
String communication_Type;
String radio_Type;
String receiver_Type;


void(* resetFunc) (void) = 0; //declare reset function @ address 0


RF24 radio(CE_RST, CSN_NSS); // CE, CSN


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);

 memorySaving();

  pinMode(nextPageButton, INPUT_PULLUP);
  pinMode(sendDataButton, INPUT_PULLUP);
  pinMode(MainMenuButton, INPUT_PULLUP);
  pinMode(SubMenuButton, INPUT_PULLUP);
//  pinMode(Buzzer, OUTPUT);



  Wire.begin(); // join i2c bus (address optional for master)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(40, 30);
  display.println("Restarted");
  display.display();

  xTaskCreatePinnedToCore(
    buttonsAndCotrollers
    ,  "buttonsAndCotrollers"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  1);
    


  radio.begin();
  radio.setPALevel(RF24_PA_MIN);

  // Read the uint32_t back from EEPROM
  readRx1Address = readUint32FromEEPROM(rx1Addr);
  readRx2Address = readUint32FromEEPROM(rx2Addr);
  readRx3Address = readUint32FromEEPROM(rx3Addr);

  getChipId(); //get the device chip id
  Serial.println("/****** Restarted Logs Start ******/");
  Serial.println("Self Address: " + String(chipId));
  Serial.println("Receiver1 Address: " + String(readRx1Address));
  Serial.println("Receiver2 Address: " + String(readRx2Address));
  Serial.println("Receiver3 Address: " + String(readRx3Address));
  Serial.println("/****** Restarted Logs End ******/");

}



void loop() {
  menuControllers();
  oled_display();

sendDataTX();
delay(10);
receivedData();

  
}

void receivedData() {
  getChipId(); //get the device chip id

  radio.openReadingPipe(0, chipId);
  radio.startListening();

  if (radio.available()) {
    char jsonBuffer[128] = {0};
    radio.read(&jsonBuffer, sizeof(jsonBuffer));

//    // Print the received JSON string to the serial monitor
    Serial.println("Received:" +String(jsonBuffer));

    // Parse the JSON data
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    // Check for errors in deserialization
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    int sensor = doc["sensor"];
//    Serial.print("Sensor: " + String(sensor));
    delay(10);
  }
}

void sendDataTX() {
  
  if (readRx1Address != 0 || readRx2Address != 0 || readRx3Address != 0) {
//    Serial.println("RX address: " + String(readRx1Address));

   if(receiver_value == 0){
    radio.openWritingPipe(readRx1Address);
   }
   if(receiver_value == 1){
    radio.openWritingPipe(readRx2Address);
   }
   if(receiver_value == 2){
    radio.openWritingPipe(readRx3Address);
   }
   
    radio.stopListening();

    // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["sw1"] = (button1State == 0)?1:0;
//    doc["reset"] = (button2State == 0)?1:0;

    // Serialize JSON to string
    char jsonBuffer[128];
    serializeJson(doc, jsonBuffer);

    // Send the JSON string
    radio.write(&jsonBuffer, sizeof(jsonBuffer));

//    // Print the JSON string to the serial monitor
    Serial.println("Sending:"+String(jsonBuffer));
    delay(10);
  }

  
}



void buttonsAndCotrollers(void *pvParameters){

  (void) pvParameters;
pinMode(potentiometer1, INPUT);
pinMode(potentiometer2, INPUT);
pinMode(joystick1, INPUT);
pinMode(joystick2, INPUT);

pinMode(joyButton, INPUT_PULLUP);
pinMode(encoderButton, INPUT_PULLUP);
pinMode(switch1, INPUT_PULLUP);
pinMode(switch2, INPUT_PULLUP);

pinMode(encoder_CLK,INPUT_PULLUP);
pinMode(encoder_DT,INPUT_PULLUP);


  for (;;)
  { 
    pot1_level = potentiometer_controller(pot1_value, pot1Dir_value, potentiometer1);
    pot2_level = potentiometer_controller(pot2_value, pot2Dir_value, potentiometer2);
    joy1_level = joystick_controller(joystick1, joy1_value, joy1Dir_value, joy1CalibrateValue);
    joy2_level = joystick_controller(joystick2, joy2_value, joy2Dir_value, joy2CalibrateValue);
  
    button1State = button_controller(joyButton, 1, btn1Dir_value);
    button2State = button_controller(encoderButton, 2, btn2Dir_value);
    switch1State = button_controller(switch1, 3, 0);
    switch2State = button_controller(switch2, 4, 0);
    statusButton = button_controller(sendDataButton, 5, 1);

    encoder1_counter = rotary_encoder_controller(encoder_CLK, encoder_DT, 1 , rot1_value, rot1Dir_value);   

vTaskDelay(1);
  }
}



void oled_display(){
 
connection = (connectionStatus == 0)?"Disconnected":"Connected"; 
btn1Dir_Type = (btn1Dir_value == 0)?"PSH":"TGL";
btn2Dir_Type = (btn2Dir_value == 0)?"PSH":"TGL";
pot1Dir_Type = (pot1Dir_value == 0)?"LFT":"RHT";
pot2Dir_Type = (pot2Dir_value == 0)?"LFT":"RHT";
joy1Dir_Type = (joy1Dir_value == 0)?"LFT":"RHT";
joy2Dir_Type = (joy2Dir_value == 0)?"LFT":"RHT";
rot1Dir_Type = (rot1Dir_value == 0)?"LFT":"RHT";
rot1Control_Type = (rot1ControlType_value == 0)?"ROT":"POT";
servo1Dir_Type = (servo1Dir_value == 0)?"KNOB":"SWEEP";
servo2Dir_Type = (servo2Dir_value == 0)?"KNOB":"SWEEP";
motor1Dir_Type = (motor1Dir_value == 0)?"CLOCK":"ANTI-CLOCK";
motor2Dir_Type = (motor2Dir_value == 0)?"CLOCK":"ANTI-CLOCK";
motor3Dir_Type = (motor3Dir_value == 0)?"CLOCK":"ANTI-CLOCK";
motor4Dir_Type = (motor4Dir_value == 0)?"CLOCK":"ANTI-CLOCK";
radio_Type = (radio_value == 0)?"LORA":"NRF24";
receiver_Type = (receiver_value == 0)?"CAR":(receiver_value == 1)?"DRONE":"TRUCK";

dataSpeed_Type = (dataSpeed_value == 1)?"10":(dataSpeed_value == 2)?"50":(dataSpeed_value == 3)?"100":"500";
communication_Type = (communication_value == 1)?"TX":(communication_value == 2)?"RX":"TX&RX";




   

 
  switch(pagesCount){
  
    case 0:
            EEPROM.write(pageCount_address, 0);
            display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.print("DASHBOARD");          
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

            display.setTextColor(WHITE);
            display.setTextSize(1);
            if(statusButton == 1){
            display.setCursor(0,12); display.print("RADIO:" + String(radio_Type)+","+String(communication_Type)+","+String(dataSpeed_Type)); 
//            display.setCursor(0,12); display.print("STATUS:" + String(connection)); 
//            display.setCursor(0,21); display.print("TEMP:" + String(tmp));display.setCursor(60,21);display.print("  HUMI:" + String(humi)); 
            display.setCursor(0,21); display.print("RSSI:" + String(rsi));//display.setCursor(60,31);display.print("  SNR:" + String(snr)); 
//            display.setCursor(0,31); display.print("B1:" + String(button1State)+"["+ String(btn1Status)+"]"); display.setCursor(60,31);display.print("| B2:" + String(button2State)+"["+ String(btn2Status)+"]"); 
//            display.setCursor(0,41); display.print("S1:" + String(switch1State)+"["+ String(sw1Status)+"]"); display.setCursor(60,41);display.print("| S2:" + String(switch2State)+"["+ String(sw2Status)+"]"); 
            display.setCursor(0,31); display.print("SWT:" +String(button1State)+","+String(button2State)+","+String(switch1State)+","+ String(switch2State)+" ["+String(btn1Status)+","+String(btn2Status)+","+String(sw1Status)+","+String(sw2Status)+"]");
            }
            else{
            display.setCursor(0,12); display.print("cordinates:"+String(latitude)+","+String(longitude));
            display.setCursor(0,21); display.print("distance  :"+String(distance));
            display.setCursor(0,31); display.print("voltage   :"+String(voltage));
             
//            "["+ String(btn1Status)+"]"); display.setCursor(60,31);display.print("| B2:" + String(button2State)+"["+ String(btn2Status)+"]"); 
//            display.setCursor(0,41); display.print("S1:" +String(switch1State)+"["+ String(sw1Status)+"]"); display.setCursor(60,41);display.print("| S2:" + String(switch2State)+"["+ String(sw2Status)+"]"); 
}
          
          break;
  
    case 1:
            EEPROM.write(pageCount_address, 1);
            display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.println("POTENTIOMETER");
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));
            
            display.setTextColor(WHITE);
            display.drawRect(0, 10, 128, 35, WHITE);

            display.setCursor(3,13); display.print("POT1:"+String(pot1_level));display.setCursor(60,13);display.print(" ["+ String(pot1Status)+"]");
            display.setCursor(3,23); display.print("POT2:"+String(pot2_level));display.setCursor(60,23);display.print(" ["+ String(pot2Status)+"]"); 
            display.setCursor(3,33); display.print("ENC1:" + String(encoder1_counter)); 

            pot1cursor = adc_controller_cursor_position(pot1_value, pot1Dir_value, pot1_level);
            pot2cursor = adc_controller_cursor_position(pot2_value, pot2Dir_value, pot2_level);

            
            display.fillRoundRect(0, 50, 128, 3, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(pot1cursor,48); display.print("*");
//            display.setCursor(0,45);display.print("---------------------");
            display.setTextColor(WHITE);

             display.fillRoundRect(0, 60, 128, 3, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(pot2cursor,58); display.print("*");
//            display.setCursor(0,55);display.print("---------------------");
            
          break;
          
    case 2:
            EEPROM.write(pageCount_address, 2);
            display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);            
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.println("JOYSTICKS");
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

            display.setTextColor(WHITE);
            display.drawRect(0, 10, 128, 35, WHITE);

            display.setCursor(3,15); display.print("JOY1:" + String(joy1_level));display.setCursor(60,15);display.print("| JOY2:" + String(joy2_level)); 

            joy1cursor = adc_controller_cursor_position(joy1_value, joy1Dir_value, joy1_level);
            joy2cursor = adc_controller_cursor_position(joy2_value, joy2Dir_value, joy2_level);
//            display.setCursor(joy1cursor,30); display.print("*");
//            display.setCursor(0,30);display.print("---------------------");
//            display.setCursor(joy2cursor,39); display.print("*");
//            display.setCursor(0,39);display.print("---------------------");

            display.fillRoundRect(0, 50, 128, 3, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(joy1cursor,48); display.print("*");
            display.setTextColor(WHITE);

            display.fillRoundRect(0, 60, 128, 3, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(joy2cursor,58); display.print("*");


            
           break;
    
    case 3:
            EEPROM.write(pageCount_address, 3);
            display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.print("FIRMWARE");          
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

            display.setTextColor(WHITE);
            display.setCursor(0,15); display.print("Version : " + String(firmwareVersionMaster)); 
//           display.setCursor(0,25); display.print("Slave1(RX) : "); display.println(fwVersionSlave1); 
//           display.setCursor(0,35); display.print("SLave2(TX) : " + String(fwVersionSlave2));          
//           display.setCursor(0,45); display.print(" POT2:" + String(pot2_value) + "  | "); display.setCursor(70,21);display.print(" BTN2:" + String(btn1Dir_value)); 
          break;
    case 4:
          EEPROM.write(pageCount_address, 4);
          potentiometer();
//          potentiometer("potentiometer", pot1_value, pot2_value, pot3_value, pot4_value, joy1_value, joy2_value, joy3_value, joy4_value,
//          pot1_address, pot2_address, pot3_address, pot4_address, joy1_address, joy2_address, joy3_address, joy4_address, 
//          180, 255);        
          break;
          
    case 5:
          EEPROM.write(pageCount_address, 5);
          controller_settings();
          break;
    case 6:
          EEPROM.write(pageCount_address, 6);
          rotary_encoder();
          break;      
    case 7:
          EEPROM.write(pageCount_address, 7);
          settings();
          break;                    
    case 8:
          EEPROM.write(pageCount_address, 8);
          radio_settings();
          break;               
    case 9:
          EEPROM.write(pageCount_address, 9);
          receiver_settings();
          break;             
  }
           display.display();

  EEPROM.commit(); 

}




void menuControllers(){
  
int changePageButton = digitalRead(nextPageButton);
int changeMenuButton = digitalRead(MainMenuButton);
int changeSubMenuButton = digitalRead(SubMenuButton);
   
   if(MainMenuSelect > 0){//If we are selecting the menu the page change button should not work because the values in the memory will mismatch.
      changePageButton = HIGH;
   }

   if(SubMenuSelect > 0){//If we are selecting the sub-menu the main menu change button should not work because the values in the memory will mismatch.
      changeMenuButton = HIGH;
      changePageButton = HIGH;
   }

   if(pagesCount < 4){//When pages is Not related To "menu select", the "main menu" and "submenu" should not work
      changeMenuButton = HIGH;
      changeSubMenuButton = HIGH;
   }

   if(MainMenuSelect == 0){//When menu value is not selected the "sub menu" value should not work
      changeSubMenuButton = HIGH;    
    }


    unsigned long currentMillisMenu = millis();
    if(currentMillisMenu - previousMillisMenu >= debounceDelayMenu){
  
    if (changePageButton == LOW) {
      pagesCount++;
      if (pagesCount > totalPages) { 
        pagesCount = 0;
      } 
    }
 

    if (changeMenuButton == LOW) {//Menu select
      MainMenuSelect++;
      if (MainMenuSelect == totalMenus+1) {
        MainMenuSelect = 0;
      }
     }


    if (changeSubMenuButton == LOW) {//Submenu select
      SubMenuSelect++;
      if (SubMenuSelect == totalSubMenus+1) {
        SubMenuSelect = 0;
      }
    }

    previousMillisMenu = currentMillisMenu;
    }
}



void memorySaving(){
  Serial.println("Memory saved");
  EEPROM.begin(255);
   
  pot1_value = EEPROM.read(pot1_address);//EEPROM.read(ADDRESS 0-255);
  pot2_value = EEPROM.read(pot2_address);//EEPROM.read(ADDRESS 0-255);
  joy1_value = EEPROM.read(joy1_address);//EEPROM.read(ADDRESS 0-255);
  joy2_value = EEPROM.read(joy2_address);//EEPROM.read(ADDRESS 0-255);

  pot1Dir_value = EEPROM.read(pot1Dir_address);//EEPROM.read(ADDRESS 0-255);
  pot2Dir_value = EEPROM.read(pot2Dir_address);//EEPROM.read(ADDRESS 0-255);
  joy1Dir_value = EEPROM.read(joy1Dir_address);//EEPROM.read(ADDRESS 0-255);
  joy2Dir_value = EEPROM.read(joy2Dir_address);//EEPROM.read(ADDRESS 0-255);
  
  btn1Dir_value = EEPROM.read(btn1Dir_address);//EEPROM.read(ADDRESS 0-255);
  btn2Dir_value = EEPROM.read(btn2Dir_address);//EEPROM.read(ADDRESS 0-255);


  rot1_value = EEPROM.read(rot1_address);//EEPROM.read(ADDRESS 0-255);
  rot1Dir_value = EEPROM.read(rot1Dir_address);//EEPROM.read(ADDRESS 0-255);
  rot1ControlType_value = EEPROM.read(rot1ControlType_address);//EEPROM.read(ADDRESS 0-255);
  
  pagesCount = EEPROM.read(pageCount_address);//EEPROM.read(ADDRESS 0-255); 


  servo1Dir_value = EEPROM.read(servo1Dir_address);//EEPROM.read(ADDRESS 0-255);
  servo2Dir_value = EEPROM.read(servo2Dir_address);//EEPROM.read(ADDRESS 0-255);

  motor1Dir_value = EEPROM.read(motor1Dir_address);//EEPROM.read(ADDRESS 0-255);
  motor2Dir_value = EEPROM.read(motor2Dir_address);//EEPROM.read(ADDRESS 0-255);
  motor3Dir_value = EEPROM.read(motor3Dir_address);//EEPROM.read(ADDRESS 0-255);
  motor4Dir_value = EEPROM.read(motor4Dir_address);//EEPROM.read(ADDRESS 0-255);

  dataSpeed_value = EEPROM.read(dataSpeed_address);//EEPROM.read(ADDRESS 0-255);
  communication_value = EEPROM.read(communication_address);//EEPROM.read(ADDRESS 0-255);

  radio_value = EEPROM.read(radio_address);//EEPROM.read(ADDRESS 0-255);
  receiver_value = EEPROM.read(receiver_address);//EEPROM.read(ADDRESS 0-255);


  joy1CalibrateValue = EEPROM.read(joy1CalibrationAddress);
  joy2CalibrateValue = EEPROM.read(joy2CalibrationAddress);
  calibrationButton = EEPROM.read(calibration_address);
  bindingRx1Button = EEPROM.read(bindingRx1_address);
  bindingRx2Button = EEPROM.read(bindingRx2_address);
  bindingRx3Button = EEPROM.read(bindingRx3_address);
  Serial.println("Read =  calibrationButton:" +String(calibrationButton)+",joy1CalibrateValue:" + String(joy1CalibrateValue)+",joy2CalibrateValue:" + String(joy2CalibrateValue));


//  Serial.println("Read =  btn1:" + String(btn1Dir_value)+", btn2:" + String(btn2Dir_value)+", btn3:" + String(btn3Dir_value)+", btn4:" + String(btn4Dir_value));
//  Serial.println("Read =  joy1:" + String(joy1_value)+"["+String(joy1Dir_value)+"], joy2:" + String(joy2_value)+"["+String(joy2Dir_value)+"]");
//  Serial.println("Read =  pot1:" + String(pot1_value)+"["+String(pot1Dir_value)+"], pot2:" + String(pot2_value)+"["+String(pot2Dir_value)+"]");
//  Serial.println("Read =  servo1:" + String(servo1Dir_value)+",servo2:" + String(servo2Dir_value));
//  Serial.println("Read =  motor1:" + String(motor1Dir_value)+", motor2:" + String(motor2Dir_value)+", motor3:" + String(motor3Dir_value)+", motor4:" + String(motor4Dir_value));
//  Serial.println("Read =  rot1:" + String(rot1_value)+"["+String(rot1Dir_value)+"], rot2:" + String(rot2_value)+"["+String(rot2Dir_value)+"]");
//  Serial.println("");
//  delay(2000);
}



/************POTENTIOMETERS & JOYSTICKS CURSOR POSITION CONTROLLER*********************************/
uint16_t adc_controller_cursor_position(uint8_t value, uint8_t directionValue, uint8_t gpioLevel){
    uint8_t cursorPosition; 
       if(value == 180 & directionValue == 0){ cursorPosition = map(gpioLevel, 180, 0, 0, 120);}
  else if(value == 180 & directionValue == 1){ cursorPosition = map(gpioLevel, 180, 0, 120, 0);}
  else if(value == 255 & directionValue == 0){ cursorPosition = map(gpioLevel, 255, 0, 0, 120);}
  else if(value == 255 & directionValue == 1){ cursorPosition = map(gpioLevel, 255, 0, 120, 0);}
return cursorPosition;
}





void potentiometer(){
          display.clearDisplay();
          display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
          display.setTextColor(BLACK);
          display.setTextSize(1);
          display.setCursor(3, 1);//COL,ROW
          display.print("POTENTIOMETER");          
          display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          display.drawRect(0, 10, 65, 45, WHITE);
          display.drawRect(64, 10, 64, 45, WHITE);

          display.setCursor(8,15); display.print("POT1:" + String(pot1_value)); display.setCursor(73,15);display.print("JOY1:" + String(joy1_value));   
          display.setCursor(8,25); display.print("POT2:" + String(pot2_value)); display.setCursor(73,25);display.print("JOY2:" + String(joy2_value)); 
          
            switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,56);display.print("Press To Select Menu");
                  break;       
            case 1:
                  display.fillRoundRect(4, 14, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(8,15); display.print("POT1:" + String(pot1_value));
                  display.setTextColor(WHITE);
                                                 
                  switch(SubMenuSelect){

                         case 1:
//                                display.setCursor(30,55);display.print("  180  255");
//                                display.fillRoundRect(10, 20, 108, 30, 5, WHITE);
//                                display.setTextColor(BLACK);
//                                display.setTextSize(1);
//                                display.setCursor(22, 25);//COL,ROW
//                                display.println("POTENTIOMETER 1");
//                                  popup_submenu(col,row,width,height,radius,textCol,textRow,menuName){
                                  popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");
                                display.setCursor(25,35);display.print("  180  255");
                                break;
                         case 2:
                                EEPROM.write(pot1_address, 180);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");
                                display.setCursor(25,35);display.print(" >180  255");
                                break;
                         case 3:
                                EEPROM.write(pot1_address, 255);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");
                                display.setCursor(25,35);display.print("  180 >255");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;                   
            case 2:
                  display.fillRoundRect(4, 24, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(8,25); display.print("POT2:" + String(pot2_value));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(25,35);display.print("  180  255");
                                break;
                         case 2:
                                EEPROM.write(pot2_address, 180);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(25,35);display.print(" >180  255");
                                break;
                         case 3:
                                EEPROM.write(pot2_address, 255);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(25,35);display.print("  180 >255");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;  
             case 3:
                  display.fillRoundRect(69, 14, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(73,15); display.print("JOY1:" + String(joy1_value));
                  display.setTextColor(WHITE);
                  

                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(25,35);display.print("  180  255");
                                
                                break;
                         case 2:
                                EEPROM.write(joy1_address, 180);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(25,35);display.print(" >180  255");
                                break;
                         case 3:
                                EEPROM.write(joy1_address, 255);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(25,35);display.print("  180 >255");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;                   

             case 4:
                  display.fillRoundRect(69, 24, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(73,25); display.print("JOY2:" + String(joy2_value));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(25,35);display.print("  180  255");
                                break;
                         case 2:
                                EEPROM.write(joy2_address, 180);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(25,35);display.print(" >180  255");
                                break;
                         case 3:
                                EEPROM.write(joy2_address, 255);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(25,35);display.print("  180 >255");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;      
             case 5:
                  MainMenuSelect = 0;
                  break;                         
                  }
        display.display(); 
     
}


void controller_settings(){
          display.clearDisplay();
          display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
          display.setTextColor(BLACK);
          display.setTextSize(1);
          display.setCursor(3, 1);//COL,ROW
          display.print("CONTROL SETTINGS");          
          display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          display.drawRect(0, 10, 65, 45, WHITE);
          display.drawRect(64, 10, 64, 45, WHITE);
          
          display.setCursor(8,15); display.print("POT1:" + String(pot1Dir_Type)); display.setCursor(73,15);display.print("JOY1:" + String(joy1Dir_Type));   
          display.setCursor(8,25); display.print("POT2:" + String(pot2Dir_Type)); display.setCursor(73,25);display.print("JOY2:" + String(joy2Dir_Type)); 
          display.setCursor(8,35); display.print("BTN1:" + String(btn1Dir_Type)); display.setCursor(73,35);display.print("BTN2:" + String(btn2Dir_Type)); 
  
            switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,56);display.print("Press To Select Menu");
                  break;       
            case 1:
                  display.fillRoundRect(4, 14, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(8,15); display.print("POT1:" + String(pot1Dir_Type));
                  display.setTextColor(WHITE);
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");
                                display.setCursor(20,35);display.print("  LEFT  RIGHT");
                                break;
                         case 2:
                                EEPROM.write(pot1Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");
                                display.setCursor(20,35);display.print(" >LEFT  RIGHT");
                                break;
                         case 3:
                                EEPROM.write(pot1Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 1");                            
                                display.setCursor(20,35);display.print("  LEFT >RIGHT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;                   
            case 2:
                  display.fillRoundRect(4, 24, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(8,25); display.print("POT2:" + String(pot2Dir_Type));
                  display.setTextColor(WHITE);
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(20,35);display.print("  LEFT  RIGHT");
                                break;
                         case 2:
                                EEPROM.write(pot2Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(20,35);display.print(" >LEFT  RIGHT");
                                break;
                         case 3:
                                EEPROM.write(pot2Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 22, 25, "POTENTIOMETER 2");
                                display.setCursor(20,35);display.print("  LEFT >RIGHT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;    
           case 3:
                  display.fillRoundRect(4, 34, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(8,35); display.print("BTN1:" + String(btn1Dir_Type));
                  display.setTextColor(WHITE);          
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 1");
                                display.setCursor(20,35);display.print("  PUSH  TOGGLE");
                                break;
                         case 2:
                                EEPROM.write(btn1Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 1");
                                display.setCursor(20,35);display.print(" >PUSH  TOGGLE");
                                break;
                         case 3:
                                EEPROM.write(btn1Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 1");
                                display.setCursor(20,35);display.print("  PUSH >TOGGLE");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;   
             case 4:
                  display.fillRoundRect(69, 14, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(73,15); display.print("JOY1:" + String(joy1Dir_Type));
                  display.setTextColor(WHITE);
             
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(20,35);display.print("  LEFT  RIGHT");
                                break;
                         case 2:
                                EEPROM.write(joy1Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(20,35);display.print(" >LEFT  RIGHT");
                                break;
                         case 3:
                                EEPROM.write(joy1Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 1");
                                display.setCursor(20,35);display.print("  LEFT >RIGHT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;   
            case 5:
                  display.fillRoundRect(69, 24, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(73,25); display.print("JOY2:" + String(joy2Dir_Type));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(20,35);display.print("  LEFT  RIGHT");
                                break;
                         case 2:
                                EEPROM.write(joy2Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(20,35);display.print(" >LEFT  RIGHT");
                                break;
                         case 3:
                                EEPROM.write(joy2Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 33, 25, "JOYSTICK 2");
                                display.setCursor(20,35);display.print("  LEFT >RIGHT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;  
                
            case 6:
                  display.fillRoundRect(69, 34, 55, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(73,35); display.print("BTN2:" + String(btn2Dir_Type));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 2");
                                display.setCursor(20,35);display.print("  PUSH  TOGGLE");
                                break;
                         case 2:
                                EEPROM.write(btn2Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 2");
                                display.setCursor(20,35);display.print(" >PUSH  TOGGLE");
                                break;
                         case 3:
                                EEPROM.write(btn2Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 40, 25, "BUTTON 2");
                                display.setCursor(20,35);display.print("  PUSH >TOGGLE");
                                 break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;
             
                 case 7:
                  MainMenuSelect = 0;
                  break;                         
                  }
        display.display(); 
     
}


void rotary_encoder(){
            display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.print("ROTARY ENCODER");          
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          display.drawRect(0, 10, 128, 45, WHITE);
          display.setCursor(15,15); display.print("ROT_VALUE    :" + String(rot1_value));   
          display.setCursor(15,25); display.print("ROT_DIRECTION:" + String(rot1Dir_Type)); 
          display.setCursor(15,35); display.print("ROT_TYPE:" + String(rot1Control_Type)); 

            switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,56);display.print("Press To Select Menu");
                  break;       
            case 1:
                  display.fillRoundRect(10, 14, 110, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(15,15); display.print("ROT_VALUE    :" + String(rot1_value));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 30, 25, "ROTARY VALUE");
                                display.setCursor(30,35);display.print(" 180  255");
                                break;
                         case 2:
                                EEPROM.write(rot1_address, 180);
                                popup_submenu(10, 20, 108, 25, 5, 30, 25, "ROTARY VALUE");
                                display.setCursor(30,35);display.print(">180  255");
                                break;
                         case 3:
                                EEPROM.write(rot1_address, 255);
                                popup_submenu(10, 20, 108, 25, 5, 30, 25, "ROTARY VALUE");
                                display.setCursor(30,35);display.print(" 180 >255");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;                   


            case 2:
                  display.fillRoundRect(10, 24, 110, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(15,25); display.print("ROT_DIRECTION:" + String(rot1Dir_Type));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY DIRECTION");
                                display.setCursor(25,35);display.print(" LEFT  RIGHT");
                                break;
                         case 2:
                                EEPROM.write(rot1Dir_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY DIRECTION");
                                display.setCursor(25,35);display.print(">LEFT  RIGHT");
                                break;
                         case 3:
                                EEPROM.write(rot1Dir_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY DIRECTION");
                                display.setCursor(25,35);display.print(" LEFT >RIGHT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;   
   
            case 3:
                  display.fillRoundRect(10, 34, 110, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(15,35); display.print("ROT_TYPE     :" + String(rot1Control_Type));
                  display.setTextColor(WHITE);
                  
                  switch(SubMenuSelect){
                         case 1:
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY TYPE");
                                display.setCursor(20,35);display.print(" ROTARY  POT");
                                break;
                         case 2:
                                EEPROM.write(rot1ControlType_address, 0);
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY TYPE");
                                display.setCursor(20,35);display.print(">ROTARY  POT");
                                break;
                         case 3:
                                EEPROM.write(rot1ControlType_address, 1);
                                popup_submenu(10, 20, 108, 25, 5, 16, 25, "ROTARY TYPE");
                                display.setCursor(20,35);display.print(" ROTARY >POT");
                                break;     
                         case 4:
                              SubMenuSelect = 0;  
                                break;
                  }  
                  break;   
                  
            case 4:
                  MainMenuSelect = 0;
                  break;                         
                  }
        display.display(); 
     
}

void page_settings(String pageName, String menu1, String menu2, String menu3, String menu4, String btn1Dir, String btn2Dir, String btn3Dir, 
String btn4Dir, String btnType1, String btnType2, int addr1, int addr2, int addr3, int addr4, int addrValue1, int addrValue2){
          display.clearDisplay();
                      display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.print(pageName);          
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);        
          display.setCursor(0,12);display.print(" "+String(menu1) + ":" + String(btn1Dir));
          display.setCursor(0,21);display.print(" "+String(menu2) + ":" + String(btn2Dir));   
          display.setCursor(0,31);display.print(" "+String(menu3) + ":" + String(btn3Dir));
          display.setCursor(0,41);display.print(" "+String(menu4) + ":" + String(btn4Dir)); 

           switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,55);display.print("Please Select Menu");
                  break;          
              
          case 1:
                 display.setCursor(0,12); display.print(">"+String(menu1)+ ":" + String(btn1Dir));
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 1:
                            EEPROM.write(addr1, addrValue1);
                            display.setCursor(10,55);display.print(" >"+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 2:
                            EEPROM.write(addr1, addrValue2);
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print(" >"+String(btnType2));
                            break;
                     case 3:
                           SubMenuSelect = 0;
                           break;           
              }  
              break;                   
        case 2:
              display.setCursor(0,21); display.print(">"+String(menu2)+ ":" + String(btn2Dir));
              switch(SubMenuSelect){
                     case 0:
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 1:
                            EEPROM.write(addr2, addrValue1);
                            display.setCursor(10,55);display.print(" >"+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 2:
                            EEPROM.write(addr2, addrValue2);
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print(" >"+String(btnType2));
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
              }   
              break;  
        case 3:
              display.setCursor(0,31); display.print(">"+String(menu3)+ ":" + String(btn3Dir));
              switch(SubMenuSelect){
                     case 0:
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 1:
                            EEPROM.write(addr3, addrValue1);
                            display.setCursor(10,55);display.print(" >"+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 2:
                            EEPROM.write(addr3, addrValue2);
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print(" >"+String(btnType2));
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
              }  
              break;                    
        case 4:
              display.setCursor(0,41); display.print(">"+String(menu4)+ ":" + String(btn4Dir));
              switch(SubMenuSelect){
                     case 0:
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 1:
                            EEPROM.write(addr4, addrValue1);
                            display.setCursor(10,55);display.print(" >"+String(btnType1));display.setCursor(55,55);display.print("  "+String(btnType2));
                            break;
                     case 2:
                            EEPROM.write(addr4, addrValue2);
                            display.setCursor(10,55);display.print("  "+String(btnType1));display.setCursor(55,55);display.print(" >"+String(btnType2));
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
              }   
              break;                    
                       
         case 5:
                  MainMenuSelect = 0;
                  break;   
              } 
        display.display();        
}




void settings(){
          display.clearDisplay();
            display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
            display.setTextColor(BLACK);
            display.setTextSize(1);
            display.setCursor(3, 1);//COL,ROW
            display.print("SETTINGS");          
            display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          
          display.setCursor(3,12); display.print("Transmit Speed:" + String(dataSpeed_Type));
          display.setCursor(3,22); display.print("Communication :" + String(communication_Type));    
          display.setCursor(3,32); display.print("Radio Type    :" + String(radio_Type));
          display.setCursor(3,42); display.print("Factory Reset");     

          
           switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,55);display.print("Select Transmit Speed");
                  break;          
              
          case 1:
                  display.fillRoundRect(0, 11, 128, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(3,12); display.print("Transmit Speed:" + String(dataSpeed_Type));
                  display.setTextColor(WHITE);
                  
                 
                 switch(SubMenuSelect){
                     case 1:
                            popup_submenu(10, 20, 108, 25, 5, 22, 25, "Transmit Speed");
                            display.setCursor(20,35);display.print(" 10 50 100 500");
                            break;
                     case 2:
                            EEPROM.write(dataSpeed_address, 1);
                            popup_submenu(10, 20, 108, 25, 5, 22, 25, "Transmit Speed");
                            display.setCursor(20,35);display.print(">10 50 100 500");
                            break;
                     case 3:
                            EEPROM.write(dataSpeed_address, 2);
                            popup_submenu(10, 20, 108, 25, 5, 22, 25, "Transmit Speed");
                            display.setCursor(20,35);display.print(" 10>50 100 500");
                            break;     
                     case 4:
                            EEPROM.write(dataSpeed_address, 3);
                            popup_submenu(10, 20, 108, 25, 5, 22, 25, "Transmit Speed");
                            display.setCursor(20,35);display.print(" 10 50>100 500");
                            break;
                     case 5:
                            EEPROM.write(dataSpeed_address, 4);
                            popup_submenu(10, 20, 108, 25, 5, 22, 25, "Transmit Speed");
                            display.setCursor(20,35);display.print(" 10 50 100>500");
                            break;     
                     case 6:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
          case 2:
                  display.fillRoundRect(0, 21, 128, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(3,22); display.print("Communication :" + String(communication_Type)); 
                  display.setTextColor(WHITE);
                 switch(SubMenuSelect){
                     case 1:
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Communication");
                            display.setCursor(20,35);display.print(" TX  RX  TX_RX");
                            break;
                     case 2:
                            EEPROM.write(communication_address, 1);
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Communication");
                            display.setCursor(20,35);display.print(">TX  RX  TX_RX");
                            break;
                     case 3:
                            EEPROM.write(communication_address, 2);
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Communication");
                            display.setCursor(20,35);display.print(" TX >RX  TX_RX");
                            break;     
                     case 4:
                            EEPROM.write(communication_address, 3);
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Communication");
                            display.setCursor(20,35);display.print(" TX  RX >TX_RX");
                            break;
                     case 5:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break;                                 
          case 3:
                  display.fillRoundRect(0, 31, 128, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(3,32); display.print("Radio Type    :" + String(radio_Type)); 
                  display.setTextColor(WHITE);
                 switch(SubMenuSelect){
                     case 1:
                            popup_submenu(10, 20, 108, 25, 5, 30, 25, "Radio Type");
                            display.setCursor(20,35);display.print(" LORA  NRF24L01");
                            break;
                     case 2:
                            EEPROM.write(radio_address, 0);
                            popup_submenu(10, 20, 108, 25, 5, 30, 25, "Radio Type");
                            display.setCursor(20,35);display.print(">LORA  NRF24L01");
                            break;
                     case 3:
                            EEPROM.write(radio_address, 1);
                            popup_submenu(10, 20, 108, 25, 5, 30, 25, "Radio Type");
                            display.setCursor(20,35);display.print(" LORA >NRF24L01");
                            break;     
                     case 4:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break;

          case 4:
                  display.fillRoundRect(0, 41, 128, 9, 1, WHITE);
                  display.setTextColor(BLACK);
                  display.setCursor(3,42); display.print("Factory Reset"); 
                  display.setTextColor(WHITE);
                  
                 
                 switch(SubMenuSelect){
                     case 1:
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Factory Reset");
                            display.setCursor(40,35);display.print(" YES  NO");
                            break;
                     case 2:
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Factory Reset");
                            display.setCursor(40,35);display.print(">YES  NO");
                            EEPROM.write(pot1_address, 180);
                            EEPROM.write(pot2_address, 180);
                            EEPROM.write(joy1_address, 255);
                            EEPROM.write(joy2_address, 255);
                            EEPROM.write(rot1_address, 180);
                            
                            EEPROM.write(rot1Dir_address, 0);
                            EEPROM.write(pot1Dir_address, 0);
                            EEPROM.write(pot2Dir_address, 0);
                            EEPROM.write(joy1Dir_address, 0);
                            EEPROM.write(joy2Dir_address, 0);
                         
                            EEPROM.write(btn1Dir_address, 1);
                            EEPROM.write(btn2Dir_address, 1);
                            EEPROM.write(btn3Dir_address, 0);
                            EEPROM.write(btn4Dir_address, 0);

                            writeUint32ToEEPROM(rx1Addr, 0);//erase RX address
                            writeUint32ToEEPROM(rx2Addr, 0);//erase RX address
                            writeUint32ToEEPROM(rx3Addr, 0);//erase RX address
                            
                            EEPROM.commit();
                            break;
                     case 3:
                            popup_submenu(10, 20, 108, 25, 5, 25, 25, "Factory Reset");
                            display.setCursor(40,35);display.print(" YES >NO");
                            break;     
                     case 4:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break;
                                               
         case 5:
                  MainMenuSelect = 0;
                  break;   
              }  
       display.display();        
}



void radio_settings(){
          display.clearDisplay();
          display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
          display.setTextColor(BLACK);
          display.setTextSize(1);
          display.setCursor(3, 1);//COL,ROW
          display.print("RADIO SETTINGS");          
          display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          display.setCursor(0,12); display.print(" RECEIVER: " + String(receiver_Type));   
          display.setCursor(0,21); display.print(" JOY CALIBRATION:" + String(calibrationButton)); 

  /**********************JOYSTICKS CALIBRATION*************************************************/
   if(calibrationButton == 1){
      int i = 0;
      while (i <= 100) { //2seconds
        Serial.println("Calibrating Joysticks for 2 sec: " + String(i));
        display.clearDisplay(); 
        display.setTextColor(WHITE);
        display.setCursor(0,30); display.print("Calibrating...");display.print(i); 
        display.display();
          
        i++;
          
        if (i == 100) {
         
     joystick_calibration(joystick1, joy1_value, joy1Dir_value, joy1CalibrationAddress);
     joystick_calibration(joystick2, joy2_value, joy2Dir_value, joy2CalibrationAddress);
      
          delay(1000);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(35,30); display.print("DONE");
          display.display(); 
          EEPROM.write(calibration_address, 0);
          EEPROM.commit();
          resetFunc();  //call reset
          

        }
    }
}
/**********************JOYSTICKS CALIBRATION*************************************************/
  
           switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,55);display.print("Select Receiver Type");
                  break;          
              
          case 1:
                 display.setCursor(0,12); display.print(">RECEIVER: " + String(receiver_Type));
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  CAR   DRONE   TRUCK");
                            break;
                     case 1:
                            EEPROM.write(receiver_address, 0);
                            display.setCursor(0,55);display.print(" >CAR   DRONE   TRUCK");
                            break;
                     case 2:
                            EEPROM.write(receiver_address, 1);
                            display.setCursor(0,55);display.print("  CAR  >DRONE   TRUCK");
                            break;     
                     case 3:
                            EEPROM.write(receiver_address, 2);
                            display.setCursor(0,55);display.print("  CAR   DRONE  >TRUCK");
                            break;   
                     case 4:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
                                
          case 2:
                 display.setCursor(0,21); display.print(">JOY CALIBRATION");
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  YES   NO");
                            break;
                     case 1:
                            EEPROM.write(calibration_address, 1);
                            display.setCursor(0,55);display.print(" >YES   NO");
                            break;
                     case 2:
                            EEPROM.write(calibration_address, 0);
                            display.setCursor(0,55);display.print("  YES  >NO");
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
                                        
         case 3:
                  MainMenuSelect = 0;
                  break;   
              }  
       display.display();        
}


void receiver_settings(){
          display.clearDisplay();
          display.fillRoundRect(0, 0, 128, 9, 2, WHITE);
          display.setTextColor(BLACK);
          display.setTextSize(1);
          display.setCursor(3, 1);//COL,ROW
          display.print("RX SETTINGS");          
          display.setCursor(103,1);display.print(String(pagesCount) + "/" + String(totalPages));

          display.setTextColor(WHITE);
          display.setCursor(0,12); display.print(" RX1 : " + String(readRx1Address));   
          display.setCursor(0,21); display.print(" RX2 : " + String(readRx2Address)); 
          display.setCursor(0,31); display.print(" RX3 : " + String(readRx3Address));
          display.setCursor(0,41); display.print(" DELETE-ALL-RX");



 /**********************receiver 1*****************************************/
   if(bindingRx1Button == 1 && readRx1Address == 0){
      int i = 0;
      while (i <= 50) { //2seconds
        Serial.println("Binding RX for 2 sec: " + String(i));
        display.clearDisplay(); 
        display.setTextColor(WHITE);
        display.setCursor(10,30); display.print("Binding RX1...");display.print(i); 
        display.display();
          binding_TX_RX(rx1Addr);
        i++;
          
        if (i == 50) {    
          delay(1000);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(35,30); display.print("DONE");
          display.display(); 
          EEPROM.write(bindingRx1_address, 0);
          EEPROM.commit();
          resetFunc();  //call reset

        }
    }
}

/**********************receiver 2*****************************************/
  if(bindingRx2Button == 1){
      int i = 0;
      while (i <= 50) { //2seconds
        Serial.println("Binding RX2 for 2 sec: " + String(i));
        display.clearDisplay(); 
        display.setTextColor(WHITE);
        display.setCursor(10,30); display.print("Binding RX2...");display.print(i); 
        display.display();
          binding_TX_RX(rx2Addr);
        i++;
          
        if (i == 50) {    
          delay(1000);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(35,30); display.print("DONE");
          display.display(); 
          EEPROM.write(bindingRx2_address, 0);
          EEPROM.commit();
          resetFunc();  //call reset

        }
    }
}

/**********************receiver 3*****************************************/
  if(bindingRx3Button == 1){
      int i = 0;
      while (i <= 50) { //2seconds
        Serial.println("Binding RX3 for 2 sec: " + String(i));
        display.clearDisplay(); 
        display.setTextColor(WHITE);
        display.setCursor(10,30); display.print("Binding RX3...");display.print(i); 
        display.display();
          binding_TX_RX(rx3Addr);
        i++;
          
        if (i == 50) {    
          delay(1000);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(35,30); display.print("DONE");
          display.display(); 
          EEPROM.write(bindingRx3_address, 0);
          EEPROM.commit();
          resetFunc();  //call reset

        }
    }
}

          
           switch(MainMenuSelect){
            case 0:
                  display.setCursor(0,55);display.print("Select Rx for Binding");
                  break;          
              
          case 1:
                 display.setCursor(0,12); display.print(">RX1 : " + String(readRx1Address)); 
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  YES   NO");
                            break;
                     case 1:
                            EEPROM.write(bindingRx1_address, 1);
                            display.setCursor(0,55);display.print(" >YES   NO");
                            break;
                     case 2:
                            EEPROM.write(bindingRx1_address, 0);
                            display.setCursor(0,55);display.print("  YES  >NO");
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
              
          case 2:
                 display.setCursor(0,21); display.print(">RX2 : " + String(readRx2Address)); 
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  YES   NO");
                            break;
                     case 1:
                            EEPROM.write(bindingRx2_address, 1);
                            display.setCursor(0,55);display.print(" >YES   NO");
                            break;
                     case 2:
                            EEPROM.write(bindingRx2_address, 0);
                            display.setCursor(0,55);display.print("  YES  >NO");
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
              
          case 3:
                 display.setCursor(0,31); display.print(">RX3 : " + String(readRx3Address)); 
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  YES   NO");
                            break;
                     case 1:
                            EEPROM.write(bindingRx3_address, 1);
                            display.setCursor(0,55);display.print(" >YES   NO");
                            break;
                     case 2:
                            EEPROM.write(bindingRx3_address, 0);
                            display.setCursor(0,55);display.print("  YES  >NO");
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break; 
                            
          case 4:
                 display.setCursor(0,41); display.print(">DELETE-ALL-RX");
                 switch(SubMenuSelect){
                     case 0:
                            display.setCursor(0,55);display.print("  YES   NO");
                            break;
                     case 1:
                            writeUint32ToEEPROM(rx1Addr, 0); // Clear receiver address
                            writeUint32ToEEPROM(rx2Addr, 0);
                            writeUint32ToEEPROM(rx3Addr, 0);
                            display.setCursor(0,55);display.print(" >YES   NO");
                            break;
                     case 2:
                            display.setCursor(0,55);display.print("  YES  >NO");
                            break;     
                     case 3:
                           SubMenuSelect = 0;
                           break; 
                   }  
              break;                                                
         case 5:
                  MainMenuSelect = 0;
                  break;   
              }  
       display.display();        
}


//*****POPUP SUBMENU**************
void popup_submenu(int col, int row, int width, int height, int radius, int textCol, int textRow, String menuName){
  //fillRect(x, y, width, height, color)
  display.fillRoundRect(col, row, width, height, radius, WHITE);
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(textCol, textRow);//COL,ROW
  display.println(menuName);
}


void binding_TX_RX(int receiverAddress) {
  // Define addresses as byte arrays
  const byte address1[8] = "1001001"; //common RX
  const byte address2[8] = "1001002"; //common TX

  radio.openWritingPipe(address2); // Address to write to
  radio.openReadingPipe(1, address1); // Address to read from
  radio.startListening(); // Start listening for incoming data

  getChipId(); //get the device chip id
  delay(5);
  // Send data
  radio.stopListening(); // Stop listening so we can talk
  bool result = radio.write(&chipId, sizeof(chipId));
  Serial.print("Sending to RX: ");
  Serial.print(chipId);
  Serial.print(" Result: ");
  Serial.println(result ? "Success" : "Failure");

  delay(5);
  radio.startListening(); // Start listening for response

  // Wait for a response for a certain time
  unsigned long startTime = millis();
  bool timeout = false;
  while (!radio.available()) {
    if (millis() - startTime > 200) { // 200ms timeout
      timeout = true;
      break;
    }
  }

  if (!timeout) {
    uint32_t rx_address = 0;
    radio.read(&rx_address, sizeof(rx_address));
    Serial.print("Receiving from RX: ");
    Serial.println(rx_address);
    writeUint32ToEEPROM(receiverAddress, rx_address);
    EEPROM.commit();

    //    if (rx_address != 0) {
    //      delay(2000);
    //      // Write the uint32_t to EEPROM
    //      writeUint32ToEEPROM(rxAddr, rx_address);
    //      EEPROM.commit();
    //      resetFunc();  //call reset
    //    }
  }
}

void getChipId() {
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  // Serial.print("Chip ID: "); Serial.println(chipId);
}

// Function to write a uint32_t to EEPROM
void writeUint32ToEEPROM(int addrOffset, uint32_t value) {
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addrOffset + i, (value >> (i * 8)) & 0xFF);
  }
}

// Function to read a uint32_t from EEPROM
uint32_t readUint32FromEEPROM(int addrOffset) {
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) {
    value |= ((uint32_t)EEPROM.read(addrOffset + i)) << (i * 8);
  }
  return value;
}
