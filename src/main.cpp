
#include "MPU9250.h"
#include "Arduino.h"
#include "BluetoothSerial.h"
#include "string.h"

#define ENCODER_DO_NOT_USE_INTERRUPTS
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Functions
void print_calibration();
void getdata();
void minspeed();
void getIMUData();
void forward();
void backward();
void motorControl(int speed, int speed1);
void PID();
void stp(int a);

float stand_angle = 4;
float wanted_angle = 0;

float kp = 6;
float kd = 12;
float ki = 2.1;

int N = 90;

int lms;
int rms;

int base = 128;
/*int baseRB = -131;
int baseLF = 131;
int baseLB = -131;*/

int integral;
int derivative;
int proportional;

int last_error = 0;

// Motor A 
int motor1Pin1 = 4; 
int motor1Pin2 = 5; 
int enable1Pin = 13; 
// Motor B  
int motor2Pin1 = 18; 
int motor2Pin2 = 19; 
int enable2Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle = 200;

int i = 0;
int o = 0;

// mpu object
MPU9250 mpu;

// angles of the robot
float pitch, yaw, roll;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  

  //builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);

  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel1);


  SerialBT.begin("PCD"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");


  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  while (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    delay(500);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      if (SerialBT.available()) {
        char a = SerialBT.read();
        if (a == 'S'){
          wanted_angle = 0;
          digitalWrite(LED_BUILTIN, HIGH);
        }
        if (a == 'F'){
          digitalWrite(LED_BUILTIN, LOW);
          if(wanted_angle < 1.8){
            wanted_angle += 0.05;
          }
          
        }
        if (a == 'B'){
          digitalWrite(LED_BUILTIN, LOW);
          if(wanted_angle > -1.8){
            wanted_angle -= 0.05;
          }
        }
      }
      PID();
      prev_ms = millis();
    }
  }

}

void getdata() {
  pitch = mpu.getPitch(); 
  yaw = mpu.getYaw(); 
  roll = mpu.getRoll();
  Serial.print("pitch roll yaw  ");
  Serial.print(pitch);
  Serial.print(" | ");
  Serial.print(roll);
  Serial.print(" | ");
  Serial.println(yaw);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void PID(){
  getdata();
  int error = roll - stand_angle - wanted_angle;
  int s = error - last_error;
  integral += ki * error;
  derivative = kd * ((N*s)/(N+s));
  proportional = kp*error;

  float ms = proportional + derivative + integral;
  
  if((abs(ms) + base > 235) && (ms * error > 0)){
    ms -= integral;
    integral = 0;
  }
  if(error > 0){
    lms = ms + base;
    rms = ms + base;
  }else{
    lms = ms - base;
    rms = ms - base;
  }

  if(abs(roll) > 50){
    if(abs(last_error) > 50){
      lms = 0;
      rms = 0;
    }
  }
  if (abs(error) < 0.8){
    lms = 0;
    rms = 0;
  }
  if(abs(lms) > 255){
    if (lms < 0){
      lms = -255;
    }else{
      lms = 255;
    }
  }
  if(abs(rms) > 255){
    if (rms < 0){
      rms = -255;
    }else{
      rms = 255;
    }
  }
  
  motorControl(rms,lms);
  last_error = error;
}
void backward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
}
void forward(){
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
void motorControl(int speed, int speed1){
  if(speed > 0) {
    forward();
  }else{
    backward();
  }
  ledcWrite(pwmChannel, abs(speed));
  ledcWrite(pwmChannel1, abs(speed1));

}
void minspeed(){
  for(int i = 120; i < 160; i++){
    motorControl(i,i);
    delay(200);
    Serial.println(i);
  }
  for(int i = -120; i > -160; i--){
    motorControl(i,i);
    delay(200);
    Serial.println(i);
  }
}