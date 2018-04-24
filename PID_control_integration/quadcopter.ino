/* LIBRARIES */

#include <Servo.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* CONSTANTS */
#define NUM_Z_MEASUREMENTS 5

#define BASE_THROTTLE 75

#define NUM_MOTORS 4

#define ESC1_PIN 10
#define ESC2_PIN 9
#define ESC3_PIN 3
#define ESC4_PIN 11

#define X_KP 10
#define X_KD 0.0
#define X_KI 0.0

#define Y_KP 10
#define Y_KD 0.0
#define Y_KI 0.0

#define Z_KP 10
#define Z_KD 0.0
#define Z_KI 0.0

/* GLOBALS */

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

Servo esc[NUM_MOTORS];
double throttles[NUM_MOTORS];

double x_angle;
double x_control;
double x_setPoint;
PID x_pid(&x_angle, &x_control, &x_setPoint, X_KP, X_KD, X_KI, DIRECT);

double y_angle;
double y_control;
double y_setPoint;
PID y_pid(&y_angle, &y_control, &y_setPoint, Y_KP, Y_KD, Y_KI, DIRECT);

double z_acceleration;
double z_control;
double z_setPoint;
PID z_pid(&z_acceleration, &z_control, &z_setPoint, Z_KP, Z_KD, Z_KI, DIRECT);

int baseAcceleration;
int zSum = 0;
int zMeasurements = NUM_Z_MEASUREMENTS;

bool calculateZAcceleration = false;
bool takeoff = false;

/* SETUP */

void setup()
{
  initializeIMU();
  initializePID();
  
  esc[0].attach(ESC1_PIN);
  esc[1].attach(ESC2_PIN);
  esc[2].attach(ESC3_PIN);
  esc[3].attach(ESC4_PIN);
  initializeMotors();
  
  delay(8000);

  for (int i = 0; i < NUM_MOTORS; i++){
    throttles[i] = BASE_THROTTLE;
  }
  takeoff = true;
}

/* LOOP */

void loop()
{
  if (!dmpReady) return;
  
  while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    x_angle = ypr[1];
    y_angle = ypr[2];
    z_acceleration = aaWorld.z;

    if (calculateZAcceleration){
      zSum += z_acceleration;
      zMeasurements -= 1;
      if (zMeasurements == 0){
        baseAcceleration = zSum / NUM_Z_MEASUREMENTS;
        calculateZAcceleration = false;
        takeoff = true;
      }
    }
    else if (takeoff){
      x_pid.Compute();
      y_pid.Compute();
      z_pid.Compute();

      changeSpeeds();
    }
    else{
      
    }
  }
}

/* FUNCTIONS */

void changeSpeeds(){
  esc[0].write(BASE_THROTTLE - x_control - y_control);
  esc[1].write(BASE_THROTTLE - x_control + y_control);
  esc[2].write(BASE_THROTTLE + x_control + y_control);
  esc[3].write(BASE_THROTTLE + x_control - y_control);
  

  
}

void initializeIMU(){
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);
  while (!Serial);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);      // 220
  mpu.setYGyroOffset(0);      // 76
  mpu.setZGyroOffset(0);      // -85
  mpu.setZAccelOffset(0);     // 1788

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
  
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
  
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
  
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void initializePID(){
  x_pid.SetOutputLimits(-100, 100);
  y_pid.SetOutputLimits(-100, 100);
  z_pid.SetOutputLimits(-100, 100);
  
  x_pid.SetMode(AUTOMATIC);
  y_pid.SetMode(AUTOMATIC);
  z_pid.SetMode(AUTOMATIC);
}

void initializeMotors(){
  for (int i = 0; i < NUM_MOTORS; i++){
    esc[i].write(0);
  }
  delay(3000);
  for (int i = 0; i < NUM_MOTORS; i++){
    esc[i].write(50);
  }
  delay(3000);
  for (int i = 0; i < NUM_MOTORS; i++){
    esc[i].write(0);
  }
  delay(3000);
  for (int i = 0; i < NUM_MOTORS; i++){
    esc[i].write(50);
  }
  delay(3000);
}
