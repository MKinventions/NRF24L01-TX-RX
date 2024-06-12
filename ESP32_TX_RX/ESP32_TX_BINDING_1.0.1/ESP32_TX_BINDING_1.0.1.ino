#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define led 22
#define bindBtn 27
#define rstBtn 14

#define sw1 12
//#define sw1 25

RF24 radio(4, 5); // CE, CSN

const int rxAddr = 0; // Address in EEPROM to store the receiver address
uint32_t readRxAddress;

uint32_t chipId = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

String firmwareVersion = "1.0.1";

int bindingButton = HIGH;


int lastButton = 0;
int buttonState = 0;
unsigned int btnCounter = 0;
unsigned long timer;
int interval = 2000;
bool timerExpired = false;
int ledState = 0;
unsigned long previousMillis = 0;


void setup() {
  Serial.begin(115200);
  EEPROM.begin(255);
  pinMode(led, OUTPUT);
  pinMode(bindBtn, INPUT_PULLUP);
  pinMode(rstBtn, INPUT_PULLUP);
  pinMode(sw1, INPUT_PULLUP);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);

  // Read the uint32_t back from EEPROM
  readRxAddress = readUint32FromEEPROM(rxAddr);

  getChipId(); //get the device chip id
  Serial.println("/****** Restarted Logs Start ******/");
  Serial.println("Self Address: " + String(chipId));
  Serial.println("Receiver Address: " + String(readRxAddress));
  Serial.println("/****** Restarted Logs End ******/");
}

void loop() {

  sendDataTX();
  delay(10);
  receivedData();
Serial.println("");
    
  digitalWrite(led, (readRxAddress != 0) ? 1 : 0);




  int button = digitalRead(bindBtn);
  if (button == 0) {
    btnCounter++;
    Serial.println("Counter: " + String(btnCounter));
    timer = millis();
    timerExpired = false;
  }
  delay(200);





  if (millis() - timer >= interval && !timerExpired) {
    //    lastTimer = timer;
    timerExpired = true;

    Serial.println("button count :" + String(btnCounter));


    //****************************************************************//
    if (btnCounter == 2 && readRxAddress == 0) {
      int i = 0;
      while (i <= 50) { //2seconds,  500ms = 1sec
        Serial.println("Provision for 2 sec: " + String(i));
        i++;
        //        digitalWrite(LED_PIN, HIGH);
        ledBlink(led, 100);
        binding_TX_RX();

        if (i == 50) {
          digitalWrite(led, LOW);
          delay(1000);
          resetFunc();  //call reset
        }

      }
    }

    //****************************************************************//



    //****************************************************************//
    if (btnCounter == 5) {
      for (int i = 0; i <= 2000; i++) {
        Serial.println("Deleting the Receiver Address: " + String(i));
        ledBlink(led, 100);
        writeUint32ToEEPROM(rxAddr, 0); // Clear receiver address
        EEPROM.commit();

        if (i == 2000) {
          digitalWrite(led, LOW);
          delay(1000);
          resetFunc();  //call reset
        }
      }
      //****************************************************************//
    }

    btnCounter = 0;




  }

}



void ledBlink(int ledPin, int delayTime) {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= delayTime) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    //    if (ledState == LOW) {
    //      ledState = HIGH;
    //    } else {
    //      ledState = LOW;
    //    }

    ledState = (ledState == LOW) ? HIGH : LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}



void receivedData() {
  getChipId(); //get the device chip id

  radio.openReadingPipe(0, chipId);
  radio.startListening();

  if (radio.available()) {
    char jsonBuffer[128] = {0};
    radio.read(&jsonBuffer, sizeof(jsonBuffer));

//    // Print the received JSON string to the serial monitor
    Serial.print("Received:" +String(jsonBuffer));

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
  if (readRxAddress != 0) {
//    Serial.println("RX address: " + String(readRxAddress));

    radio.openWritingPipe(readRxAddress);
    radio.stopListening();

    // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["sw1"] = digitalRead(sw1) ? 0 : 1;
    doc["reset"] = (btnCounter == 5)?1:0;

    // Serialize JSON to string
    char jsonBuffer[128];
    serializeJson(doc, jsonBuffer);

    // Send the JSON string
    radio.write(&jsonBuffer, sizeof(jsonBuffer));

//    // Print the JSON string to the serial monitor
    Serial.print("Sending:"+String(jsonBuffer));
    delay(10);
  }
}

void binding_TX_RX() {
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
    writeUint32ToEEPROM(rxAddr, rx_address);
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
