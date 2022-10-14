/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include "stdbool.h"
#include <yobotik_ino1.h>
int pin = 22;
Yobotik * Yobotik::instances[2] = { NULL, NULL };
Yobotik yobotik(pin,1);
uint8_t new_button_count=0;
void setup(){
  yobotik.InterruptInit();
}

void loop() {  
  uint8_t button_count = new_button_count;
  new_button_count=yobotik.InterruptFunc(button_count);
  
  //yobotik.digitalWriteST(1);
}


    
