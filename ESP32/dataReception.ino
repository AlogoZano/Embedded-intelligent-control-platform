// CAN_BUS_Shield - Version: Latest 
#include <can-serial.h>
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp2518fd_can.h>
#include <mcp2518fd_can_dfs.h>
#include <mcp_can.h>

#include "thingProperties.h"
#include <SPI.h>



const int spiCSPin = 5;
mcp2515_can CAN(spiCSPin);


void setup() {
  pinMode(15, OUTPUT);  

  Serial.begin(9600);
  delay(1500); 

  initProperties();

  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
    while (CAN_OK != CAN.begin(CAN_500KBPS)){
      Serial.println("CAN BUS init Failed");
      delay(100);
    }
  Serial.println("CAN BUS Shield Init OK!");
  }


unsigned char stmp[8] = {15, 1, 25, 3, 8, 5, 99, 22};
unsigned char stmp2[8] = {255, 128, 64, 32, 16, 8, 4, 2};


void loop() {
  unsigned char len = 0;
  unsigned char buf[8];
  
    if(CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    Serial.println("-----------------------------");
    Serial.print("Data from ID: 0x");
    Serial.println(canId, HEX);
    Serial.println(canId);

    if(canId == 0xAA){
      pos = buf[0];
    }
    
    else if(canId == 0xFF){
      error = false;
    }
    
    else if(canId == 0x00){
      error = true;
    }
    
    for(int i = 0; i<len; i++){
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.println();
  }
  
  ArduinoCloud.update();

}


/*
  Since Elevation is READ_WRITE variable, onElevationChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onElevationChange()  {
  // Add your code here to act upon Elevation change
  if(elevation >= 0 && elevation < 20){
    CAN.sendMsgBuf(0x43, 0, 8, stmp);
    pos = 90;

  }else if(elevation >= 20 && elevation < 40){
    CAN.sendMsgBuf(0x11, 0, 8, stmp2);
    pos = 60;
  
  }else if(elevation >= 40 && elevation <= 60){
    CAN.sendMsgBuf(0x65, 0, 8, stmp);
    pos = 30;
  }
}
/*
  Since Request is READ_WRITE variable, onRequestChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onRequestChange()  {
  CAN.sendMsgBuf(0x50, 0, 8, stmp);
  error = true;
}
/*
  Since Pos is READ_WRITE variable, onPosChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onPosChange()  {
  // Add your code here to act upon Pos change
}
/*
  Since Error is READ_WRITE variable, onErrorChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onErrorChange()  {
  // Add your code here to act upon Error change
}
/*
  Since Light is READ_WRITE variable, onLightChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLightChange()  {
  // Add your code here to act upon Light change
}