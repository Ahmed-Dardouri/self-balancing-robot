//#include "MPU9250.h"
#include <Arduino.h>
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
//MPU9250 IMU(Wire,0x68);
//int status;
int i = 0;
void setup() {
  // serial to display data
  Serial.begin(115200);
  /*while(!Serial) {}
  Serial.println("serial mrigl");
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }else{
    Serial.println("mriiigl");
  }*/
}

void loop() {
  if (i == 0){
    
    Serial.println("hello from setup");
    Serial.println("--------------------------");
    i++;
  }else{
    Serial.println("hello from loop");
    /*IMU.readSensor();

    Serial.print(IMU.getGyroX_rads(),2);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),2);
    Serial.print("\t");
    Serial.println(IMU.getGyroZ_rads(),2);
    Serial.println("______________________________");*/
    delay(1000);
  }
  
}