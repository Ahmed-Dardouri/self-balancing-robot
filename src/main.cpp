#include "MPU9250.h"

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void getIMUData();
void forward();
void backward();
void motorControl(int speed);
void PID();
void stp(int a);
// Motor A
int kp = 1.5;
int kd = 0;

int last_error = 0;

int motor1Pin1 = 4; 
int motor1Pin2 = 5; 
int enable1Pin = 13; 
// Motor B  
int motor2Pin1 = 19; 
int motor2Pin2 = 18; 
int enable2Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
double roll , pitch, yaw;
void setup() {

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);
  // serial to display data
  Serial.begin(115200);

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  while(!Serial) {}
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }
}

void loop() {
  if (SerialBT.available()) {
    char a = SerialBT.read();
    Serial.println(a);
    if (a == 'F'){
      motorControl(200);
    }else if (a == 'B'){
      motorControl(-200);
    }else{
      stp(1);
    }
  }
  delay(20);
}

void getIMUData(){  
  IMU.readSensor();
  float accelX = IMU.getAccelX_mss();
  float accelY = IMU.getAccelY_mss();
  float accelZ = IMU.getAccelZ_mss();
  float gyroX = IMU.getGyroX_rads()/57.3;
  float gyroY = IMU.getGyroY_rads()/57.3;
  float gyroZ = IMU.getGyroZ_rads()/57.3;
  float magX = IMU.getMagX_uT();
  float magY = IMU.getMagY_uT();
  float magZ = IMU.getMagZ_uT();


//Euler angle from accel

 
  pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

   // yaw from mag

  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));

  yaw =  atan2(Yh, Xh);


  roll = roll*57.3;
  pitch = pitch*57.3;
  yaw = yaw*57.3;



  /*Serial.print("pitch"); 
  Serial.println(pitch);
  Serial.print("yaw"); 
  Serial.println(yaw);
  Serial.print("roll"); 
  Serial.println(roll);*/

}

void PID(){
  
  getIMUData();
  int error = roll;
  /*Serial.print("roll  ");
  Serial.println(roll);*/
  int ms = 140 * (roll/abs(error)) + kp*error + kd * (error - last_error);
  if(abs(roll) > 60){
    if(abs(last_error) > 40){
      ms = 0;
    }
    
  }
  if(abs(ms) > 255){
    if (ms < 0){
      ms = -255;
    }else{
      ms = 255;
    }
  }
  motorControl(ms);
  last_error = error;
}

void forward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
}
void backward(){
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
}
void stp(int a){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(a); 
}
void motorControl(int speed){
  if(speed > 0){
    backward();
  }else{
    forward();
  }
  ledcWrite(pwmChannel, abs(speed));
}

/*#include <Arduino.h>
void countEncoderRA();
void countEncoderRB();

int encoderRA = 15;
int encoderRB = 2;
float R = 40.25;
float encoder_ticks = 400;
float precision = 4;
float distR = 0;
int encoderRPos = 0;
int lastEncoderRPos = 0;
boolean encoderRA_set = HIGH;
boolean encoderRB_set = HIGH;

float distance_tick = (2*PI*R)/(encoder_ticks*precision);

void countEncoderRA(){
  encoderRA_set = digitalRead(encoderRA);
  encoderRPos += (encoderRA_set == encoderRB_set) ? -1 : +1;
  distR += (encoderRA_set == encoderRB_set) ? -distance_tick : +distance_tick;
}

void countEncoderRB(){
  encoderRB_set = digitalRead(encoderRB);
  encoderRPos += (encoderRA_set != encoderRB_set) ? -1 : +1;
  distR += (encoderRA_set != encoderRB_set) ? -distance_tick : +distance_tick; 
}

void setup() {
  pinMode(encoderRA, INPUT);
  pinMode(encoderRB, INPUT);
  digitalWrite(encoderRA, HIGH);
  digitalWrite(encoderRB, HIGH);
  attachInterrupt(digitalPinToInterrupt(15), countEncoderRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), countEncoderRB, CHANGE);
  Serial.begin(115200);
}

void loop() {
      Serial.print("  position    ");
      Serial.println(encoderRPos);
      Serial.print("  distance   ");
      Serial.println(distR); 
      delay(300); 
}*/
