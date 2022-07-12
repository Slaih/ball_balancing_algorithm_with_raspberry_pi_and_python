#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>  
#include <Wire.h> 
#include <string.h>
RF24 radio(9, 10); // CE, CSN
LiquidCrystal_I2C lcd(0x27, 16, 2);  
const uint64_t pipe = 0xE8E8F0F0E1LL;
const char s[2] = "*";
String printedMessage;
void setup() {
  Serial.begin(9600);
  radio.begin();        // start radio at ce csn pin 9 and 10

  radio.setPayloadSize(32);
  radio.setChannel(0x77) ;            // set chanel at 77
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);

  radio.setAutoAck(false);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  //radio.openWritingPipe(pipes[0])
  radio.openReadingPipe(1, pipe);
  //radio.printDetails();
  radio.startListening();   
  lcd.begin();  
  
}
void loop() {


  char receivedMessage[32] = {0} ;   // set incmng message for 32 bytes

  if (radio.available()) {       // check if message is coming

    radio.read(receivedMessage, sizeof(receivedMessage));    // read the message and save
    printedMessage = strtok(receivedMessage,s);
    Serial.println(printedMessage);
 
   lcd.print(receivedMessage); 
  
   delay(200);
   lcd.clear();  

  }
  else{
  lcd.print("Top Bulunamadi"); 
  delay(200);
  lcd.clear();
  }
}
