#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

//#define led 22
//#define bindBtn 27
//#define rstBtn 14

#define led 2
#define bindBtn 14
#define rstBtn 25


#define sw1 27
#define sw2 26

RF24 radio(4, 5); // CE, CSN

const int rxAddr = 0; // Address in EEPROM to store the receiver address
uint32_t readRxAddress;

uint32_t chipId = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0


int bindingButton = HIGH;


int lastButton = 0;
int buttonState = 0;
unsigned int btnCounter = 0;
unsigned long timer;
int interval = 2000;
bool timerExpired = false;
int ledState = 0;
unsigned long previousMillis = 0;


String firmwareVersion = "1.1.0";

const char* ReceiverType = "car";
//const char* ReceiverType = "drone";
//const char* ReceiverType = "truck";

void setup() {
  Serial.begin(115200);
  EEPROM.begin(255);
  pinMode(led, OUTPUT);
  pinMode(bindBtn, INPUT_PULLUP);
  pinMode(rstBtn, INPUT_PULLUP);
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);

  // Read the uint32_t back from EEPROM
  readRxAddress = readUint32FromEEPROM(rxAddr);

  getChipId(); //get the device chip id
  Serial.println("/****** Restarted Logs Start ******/");
  Serial.println("Self Address....: " + String(chipId));
  Serial.println("Receiver Address: " + String(readRxAddress));
  Serial.println("Receiver Type...: " + String(ReceiverType));
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
    Serial.print("Received:" + String(jsonBuffer));

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
    radio.openWritingPipe(readRxAddress);
    radio.stopListening();

    // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["sw1"] = digitalRead(sw1) ? 1 : 0;
    doc["sw2"] = digitalRead(sw2) ? 1 : 0;
    doc["reset"] = (btnCounter == 5) ? 1 : 0;

    // Serialize JSON to string
    char jsonBuffer[128];

    serializeJson(doc, jsonBuffer);
    radio.write(&jsonBuffer, sizeof(jsonBuffer));



    //    // Print the JSON string to the serial monitor
    Serial.print("Sending:" + String(jsonBuffer));
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

struct BindData1 {
  char tx_type[10];     // Adjust the size based on your needs
  uint32_t tx_address;
}; 

BindData1 bindData1;

  // Store the data
  strncpy(bindData1.tx_type, ReceiverType, sizeof(bindData1.tx_type) - 1); // Ensure null-termination
  bindData1.tx_type[sizeof(bindData1.tx_type) - 1] = '\0'; // Explicit null-termination
  bindData1.tx_address = chipId;

  
  bool result = radio.write(&bindData1, sizeof(bindData1));
  Serial.print("Sending to RX: [");
  Serial.print(String(bindData1.tx_type));
  Serial.print(",");
  Serial.print(String(bindData1.tx_address));
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

    struct BindData2 {
      char rx_type[10];     // Adjust the size based on your needs
      uint32_t rx_address;
    };

    BindData2 bindData2;

   radio.read(&bindData2, sizeof(bindData2));

    // Print the values
    Serial.println("Received => type: " + String(bindData2.rx_type) + ", Address: " + String(bindData2.rx_address));

    
//Comparing the receiver type
    if (strcmp(bindData2.rx_type, ReceiverType) == 0) {
      writeUint32ToEEPROM(rxAddr, bindData2.rx_address);
      EEPROM.commit();
    }

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
