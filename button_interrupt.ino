/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include "util.h"
#include <ModbusMaster.h>

//#include "mcu_json_parser.h"
//#include "jsonparsers.h"
ModbusMaster node;
uint8_t button1_read[] = {0xFF, 0xFE, 1, 57};
uint8_t button2_read[] = {0xFF, 0xFE, 1, 58};
uint8_t resp_buf[0xFF];
uint8_t button1_write[] = {0xFF, 0xFE, 1, 59};
uint8_t button2_write[] = {0xFF, 0xFE, 1, 60};
uint16_t counter=0;
uint8_t button_count=0;
typedef struct{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
}Button_t ;

Button_t button1 = {22, 0, false};

void  IRAM_ATTR ISR() {
  button1.numberKeyPresses += 1;
  button1.pressed = true;
}


void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(button1.PIN,INPUT);
  //Serial2.setTimeout(1);
   // communicate with Modbus slave ID 1 over Serial (port 0)
  node.begin(1, Serial2);
  attachInterrupt(digitalPinToInterrupt(button1.PIN), ISR, CHANGE);
}

uint8_t digitalReadST(uint8_t ch){
  if(ch == 1){
    mcu_comm_raw(button1_read);    
  }

  else {
    mcu_comm_raw(button2_read);    
  }
    
  byte len = Serial2.readBytes(resp_buf,0xFF);
  //comPrintBuffer(resp_buf, len);

  return resp_buf[1];  
}

void digitalWriteST(uint8_t ch){
  if(ch==1){
    static uint32_t i;
    i=0;
    counter = counter +1 ;
    Serial.println(counter);
    node.setTransmitBuffer(0, lowWord(i));
    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    node.setTransmitBuffer(1, highWord(i));
    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    node.writeMultipleRegisters(1, 2);
    if(counter == 10){
        i=1;
          // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
        node.setTransmitBuffer(0, lowWord(i));
      
        // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
        node.setTransmitBuffer(1, highWord(i));
        
        // slave: write TX buffer to (2) 16-bit registers starting at register 0
        node.writeMultipleRegisters(1, 2);
        //delay(100);
        counter = 0;
      }
  }
  else{
    static uint32_t i;
    i=2;
    counter = counter +1 ;
    Serial.println(counter);
    node.setTransmitBuffer(0, lowWord(i));
    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    node.setTransmitBuffer(1, highWord(i));
    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    node.writeMultipleRegisters(1, 2);
    if(counter == 10){
        i=1;
          // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
        node.setTransmitBuffer(0, lowWord(i));
      
        // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
        node.setTransmitBuffer(1, highWord(i));
        
        // slave: write TX buffer to (2) 16-bit registers starting at register 0
        node.writeMultipleRegisters(1, 2);
        //delay(100);
        counter = 0;
      }
  }
}
void loop() {  
  //if(Serial2.available() > 0){
    static uint32_t i;
    uint8_t j, result;
    
    uint16_t data[6];
    
  
    i++;
    Serial.println("Asking Btn");

    byte btn1 = digitalReadST(1);
    delay(100);
    byte btn2 = digitalReadST(2);

    if(btn1 == 0){
      Serial.println("Btn1 is OFF");
    }
    else if(btn1==1){
      Serial.println("Btn1 is ON");
      digitalWriteST(1);
    }
  
    if(btn2 == 0){
      Serial.println("Btn2 is OFF");
    }
    else if(btn2==1){
      Serial.println("Btn2 is ON");
      digitalWriteST(2);
    }
  
  // slave: read (6) 16-bit registers starting at register 0 to RX buffer
  result = node.readHoldingRegisters(0, 6);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j <6; j++)
    {
      data[j] = node.getResponseBuffer(j);
      Serial.print(data[j]);
      Serial.print(",");
    }
    Serial.println("");
  }
  if(button_count ==1){
    i=1;
    node.setTransmitBuffer(0, lowWord(i));
    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    node.setTransmitBuffer(1, highWord(i));
    // slave: write TX buffer to (1) 16-bit registers starting at register 3
    node.writeMultipleRegisters(3, 4);
  }
  if(button_count == 2){
        Serial.println(button_count);
        i=0;   
        node.setTransmitBuffer(0, lowWord(i));
        // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
        node.setTransmitBuffer(1, highWord(i));
        // slave: write TX buffer to (1) 16-bit registers starting at register 3
        node.writeMultipleRegisters(3, 4);     
        button_count=0;
      }
  //delay(1000);
  if (button1.pressed) {
    button_count++;
    i=1;
      if(data[3] == 1){
        Serial.printf("Button 1 has been pressed %u times with interrupt\n",button1.numberKeyPresses);
        button1.pressed = false;
      }
  }
   //Detach Interrupt after 1 Minute
  static uint32_t lastMillis = 0;
  if (millis() - lastMillis > 60000) {
    lastMillis = millis();
    detachInterrupt(button1.PIN);
    Serial.println("Interrupt Detached!");
  }
}


    
