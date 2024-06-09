#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define led 22
#define bindBtn 27
#define rstBtn 14

#define sw1 12

RF24 radio(4, 5); // CE, CSN

const int rxAddr = 0; // Address in EEPROM to store the address1
uint32_t readRxAddress;


uint32_t chipId = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

String firmwareVersion = "1.0.0";

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

  getChipId();//get the device chip id
  Serial.println("/******Restarted Logs Start******/");
  Serial.println("SelfSelf Address: " + String(chipId));
  Serial.println("Receiver Address: " + String(readRxAddress));
  Serial.println("/******Restarted logs End******/");

}

void loop() {
  int bindingButton = digitalRead(bindBtn);
  int resetButton = digitalRead(rstBtn);

  //if there is no receiver address in memory then the button should work. (or)
  //if there is receiver address in memory then the button should not work.
  if (bindingButton == 0 && readRxAddress == 0) {
    binding_TX_RX();
   }else if(bindingButton == 1  && readRxAddress != 0){
  sendDataTX();
  delay(10);
  receivedData();
  }

  if (resetButton == 0) {
    delay(1000);
    // Write the uint32_t to EEPROM
    writeUint32ToEEPROM(rxAddr, 0);
    EEPROM.commit();
    resetFunc();  //call reset
  }

  if (readRxAddress != 0) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }



}

void receivedData() {
  getChipId();//get the device chip id

  radio.openReadingPipe(0, chipId);
  radio.startListening();

  if (radio.available()) {

    char jsonBuffer[128] = {0};
    radio.read(&jsonBuffer, sizeof(jsonBuffer));

    // Print the received JSON string to the serial monitor
    Serial.print("Received: ");
    Serial.println(jsonBuffer);

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

Serial.println("Sensor:"+String(sensor));
delay(10);
  }

}



void sendDataTX(){

     if(readRxAddress != 0){
     Serial.print("RX address:" + String(readRxAddress));

    radio.openWritingPipe(readRxAddress);
    radio.stopListening();

    // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["sw1"] = digitalRead(sw1);

    // Serialize JSON to string
    char jsonBuffer[128];
    serializeJson(doc, jsonBuffer);

    // Send the JSON string
    radio.write(&jsonBuffer, sizeof(jsonBuffer));

    // Print the JSON string to the serial monitor
    Serial.println(jsonBuffer);
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

  getChipId();//get the device chip id
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

    if (rx_address != 0) {
      delay(2000);
      // Write the uint32_t to EEPROM
      writeUint32ToEEPROM(rxAddr, rx_address);
      EEPROM.commit();
      resetFunc();  //call reset
    }


  }
}




void getChipId() {
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  //  Serial.print("Chip ID: "); Serial.println(chipId);

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
