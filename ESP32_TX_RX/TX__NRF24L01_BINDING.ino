#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>


#define led 33
#define btn 27
#define rstBtn 14


RF24 radio(4, 5); // CE, CSN

const int rxAddr = 0; // Address in EEPROM to store the address1
uint32_t readRxAddress;


uint32_t chipId = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {
  Serial.begin(115200);
  EEPROM.begin(255);
  pinMode(led, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
  pinMode(rstBtn, INPUT_PULLUP);

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
  int bindingButton = digitalRead(btn);
  int resetButton = digitalRead(rstBtn);

//if there is no receiver address in memory then the button should work. (or)
//if there is receiver address in memory then the button should not work.
  if (bindingButton == 0 && readRxAddress == 0) {
    binding_TX_RX();
  }

  if(resetButton == 0){
     delay(1000);
     // Write the uint32_t to EEPROM
     writeUint32ToEEPROM(rxAddr, 0);
     EEPROM.commit();
     resetFunc();  //call reset 
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

     // Write the uint32_t to EEPROM
    writeUint32ToEEPROM(rxAddr, rx_address);
    EEPROM.commit();

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
