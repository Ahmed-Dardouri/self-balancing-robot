
#include "MPU9250.h"
#include "Arduino.h"
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

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
void motorControl(int speed);
void PID();
void stp(int a);


int stand_angle = 10;
int wanted_angle = 0;

int kp = 5;
int kd = 0.5;
int ki = 0;

int N = 30;

int base = 138;
int integral;
int derivative;
int proportional;

float gyroll = 0;

ESP32Encoder encoder;
ESP32Encoder encoder2;

int loop_timer;
int initial_time;

int last_error = 0;

// Motor A 
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

int i = 0;

float pitch, yaw, roll;
// mpu object
MPU9250 mpu;

void print_roll_pitch_yaw();

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
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);


  //-----------encoder------------
  // Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors=DOWN;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;


	encoder.attachFullQuad(33, 26);

	encoder2.attachFullQuad(25, 27);
		
	// set starting count value after attaching
	encoder.clearCount();
  encoder.setCount(0);
	// clear the encoder's raw count and set the tracked count to zero
	encoder2.clearCount();
  encoder2.setCount(0);
	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()) + "   " + String((int32_t)encoder2.getCount()));
  //-----------encoder------------


  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");


  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }else{
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // calibrate anytime you want to
  /*Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();

  print_calibration();
  mpu.verbose(false);*/
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
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

/////////////////////////////////////////////////////////////////


/*void setup() {

  
  while(!Serial) {}
  // start communication with IMU 

 
      

}*/



void PID(){
  
  getdata();
  int error = roll - stand_angle - wanted_angle;
  int s = error - last_error;
  //integral += ki * (last_error + error)/2;
  integral += ki * error;
  //derivative = kd * s;
  derivative = kd * ((N*s)/(N+s));
  proportional = kp*error;

  float ms = proportional + derivative + integral;
  if(roll > 0){
    ms += base;
  }else{
    ms -= base;
  }
  if(abs(roll) > 50){
    if(abs(last_error) > 50){
      ms = 0;
    }
  }else if(abs(roll) > 25){
    ms *= 1.15;
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
void motorControl(int speed){
  if(speed > 0){
    forward();
  }else{
    backward();
  }
  ledcWrite(pwmChannel, abs(speed));
}
void minspeed(){
  for(int i = 120; i < 160; i++){
    motorControl(i);
    delay(200);
    Serial.println(i);
  }
  for(int i = 160; i > 120; i--){
    motorControl(i);
    delay(200);
    Serial.println(i);
  }
}



//-----------bluetooth-------------
  /*if (SerialBT.available()) {
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
  delay(20);*/
  //-----------bluetooth-------------


  //Serial.println("Encoder count = " + String((int32_t)encoder.getCount()) + " " + String((int32_t)encoder2.getCount()));