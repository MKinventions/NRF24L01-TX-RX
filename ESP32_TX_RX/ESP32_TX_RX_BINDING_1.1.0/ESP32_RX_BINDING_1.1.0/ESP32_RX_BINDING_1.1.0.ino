#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <ArduinoJson.h>


//#define led 14
#define status_led 17
#define bindBtn 25
//#define bindBtn 13
#define rstBtn 27

#define load_led1 12
#define load_led2 13
//#define load_led 14

RF24 radio(4, 5); // CE, CSN

const int txAddr = 0; // Address in EEPROM to store the address1
uint32_t readTxAddress;

uint32_t chipId = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

String firmwareVersion = "1.1.0";

const char* ReceiverType = "car";
//const char* ReceiverType = "drone";
//const char* ReceiverType = "truck";

void setup() {
  Serial.begin(115200);
  EEPROM.begin(255);
  pinMode(status_led, OUTPUT);
  pinMode(load_led1, OUTPUT);
  pinMode(load_led2, OUTPUT);
  pinMode(bindBtn, INPUT_PULLUP);
  pinMode(rstBtn, INPUT_PULLUP);

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);

  // Read the uint32_t back from EEPROM
  readTxAddress = readUint32FromEEPROM(txAddr);

  getChipId();//get the device chip id
  Serial.println("/******Restarted Logs Start******/");
  Serial.println("Receiver Type......: " + String(ReceiverType));
  Serial.println("Self Address.......: " + String(chipId));
  Serial.println("Transmitter Address: " + String(readTxAddress));
  Serial.println("/******Restarted logs End******/");

}

void loop() {
  int bindingButton = (digitalRead(bindBtn) == 0) ? 1 : 0;
  int resetButton = digitalRead(rstBtn);

  //Serial.println("binding:"+String(bindingButton));

  if (bindingButton == 1  && readTxAddress == 0) {
    binding_TX_RX();
  }
  else if (bindingButton == 0  && readTxAddress != 0) {
    receivedData();
    delay(10);
    sendDataTX();
  }


  if (resetButton == 0) {
    delay(1000);
    // Write the uint32_t to EEPROM
    writeUint32ToEEPROM(txAddr, 0);
    EEPROM.commit();
    resetFunc();  //call reset
  }

  if (readTxAddress != 0) {
    digitalWrite(status_led, HIGH);
  } else {
    digitalWrite(status_led, LOW);
  }


}





void receivedData() {
  getChipId();//get the device chip id

  radio.openReadingPipe(0, chipId);
  radio.startListening();

  if (radio.available()) {

    char jsonBuffer[128] = {0};
    radio.read(jsonBuffer, sizeof(jsonBuffer));
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

    int led1State = doc["sw1"];
    int led2State = doc["sw2"];
    int reset = doc["reset"];

    digitalWrite(load_led1, led1State);
    digitalWrite(load_led2, led2State);

    if (reset == 1) {
      delay(2000);
      writeUint32ToEEPROM(txAddr, 0);
      EEPROM.commit();
      resetFunc();  //call reset
    }

  }

}

void sendDataTX() {

  if (readTxAddress != 0) {
    //     Serial.print("TX address:" + String(readTxAddress));

    radio.openWritingPipe(readTxAddress);
    radio.stopListening();

    // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["sensor"] = random(10, 100);

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
  const byte address1[8] = "1001002"; //common TX
  const byte address2[8] = "1001001"; //common RX

  radio.openWritingPipe(address2); // Address to write to
  radio.openReadingPipe(1, address1); // Address to read from
  radio.startListening(); // Start listening for incoming data

  getChipId();//get the device chip id
  delay(5);
  // Send data
  radio.stopListening(); // Stop listening so we can talk

  struct BindData1 {
    char rx_type[10];     // Adjust the size based on your needs
    uint32_t rx_address;
  };

  BindData1 bindData1;

  // Store the data
  strncpy(bindData1.rx_type, ReceiverType, sizeof(bindData1.rx_type) - 1); // Ensure null-termination
  bindData1.rx_type[sizeof(bindData1.rx_type) - 1] = '\0'; // Explicit null-termination
  bindData1.rx_address = chipId;


  bool result = radio.write(&bindData1, sizeof(bindData1));
  Serial.print("Sending to TX: [");
  Serial.print(String(bindData1.rx_type));
  Serial.print(",");
  Serial.print(String(bindData1.rx_address));
  Serial.print("] Result: ");
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
//    uint32_t tx_address = 0;
//    radio.read(&tx_address, sizeof(tx_address));
//    Serial.print("Receiving from RX: ");
//    Serial.println(tx_address);
//
//    if (tx_address != 0) {
//      delay(2000);
//      // Write the uint32_t to EEPROM
//      writeUint32ToEEPROM(txAddr, tx_address);
//      EEPROM.commit();
//      //      resetFunc();  //call reset
//    }


    struct BindData2 {
      char tx_type[10];     // Adjust the size based on your needs
      uint32_t tx_address;
    };

    BindData2 bindData2;

    radio.read(&bindData2, sizeof(bindData2));

    // Print the values
    Serial.println("Received => type: " + String(bindData2.tx_type) + ", Address: " + String(bindData2.tx_address));


    //Comparing the receiver type
    if (strcmp(bindData2.tx_type, ReceiverType) == 0) {
      writeUint32ToEEPROM(txAddr, bindData2.tx_address);
      EEPROM.commit();
    }


  }
}

void getChipId() {
  for (int i = 0; i < 17; i = i + 7) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  //    Serial.print("Chip ID: "); Serial.println(chipId);

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
