#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>

// MPU vars
MPU6050 mpu;
#define INTERRUPT_PIN 2
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
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

float pitch;
float roll;
//end

const int numjoints = 9;
const int numsetpoints = 1;
const int numsetpointlists = 1;
unsigned long timeToMove;

// joint globals
int c = 0;
int delta = 5;
bool there;
const double P_VAL = 0.02;
int whichsetpoint = 0;

struct joint{
  Servo servo;
  int pin;
  double pos = 0;
  int flo = 0;
  int ciel = 180; 
};

joint j11;
joint j12; 
joint j13; 
joint j21; 
joint j22;
joint j23; 
joint j31; 
joint j32; 
joint j33; 
joint joints[numjoints] = {}; // this line breaks code if put before individual joint var declarations, not quite sure why

//{20,150,50,20,150,50,20,150,50}

int setpoints[numsetpointlists][numsetpoints][numjoints] = {{{45,170,0,45,170,0,45,170,0}, // neutral standing position
                                                            /* {40,160,0,40,160,0,40,160,0}, // slides feet out, keeps distance from ground == 0
                                                             {35,160,60,40,160,0,40,160,0}, // engage foot, keep robot level
                                                             {40,170,60,45,170,0,45,170,0}, // step back to neutral (foot still engaged until next step)
                                                
                                                             {45,170,0,45,170,0,45,170,0}, // neutral standing position
                                                             {40,160,0,40,160,0,40,160,0}, // slides feet out, keeps distance from ground == 0
                                                             {40,160,0,35,160,60,40,160,0}, // engage foot, keep robot level
                                                             {45,170,0,40,170,60,45,170,0}, // step back to neutral (foot still engaged until next step)
                                              
                                                             {45,170,0,45,170,0,45,170,0}, // neutral standing position
                                                             {40,160,0,40,160,0,40,160,0}, // slides feet out, keeps distance from ground == 0
                                                             {40,160,0,40,160,0,35,160,60}, // engage foot, keep robot level
                                                             {45,170,0,45,170,0,40,170,60}, // step back to neutral (foot still engaged until next step)
                                                             */
                                                             }};

void setup() {

  // mpu setup

  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  // IMU Setup
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Wire.setWireTimeout(3000,true);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(177);
  mpu.setYGyroOffset(-30);
  mpu.setZGyroOffset(-89);
  mpu.setXAccelOffset(-4105);
  mpu.setYAccelOffset(-2437);
  mpu.setZAccelOffset(507);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    //Serial.println("bad");
  }

  //end mpu setup

  j11.pin = 8;  
  j12.pin = 3; 
  j13.pin = 4; 
  j21.pin = 5;
  j22.pin = 6;
  j23.pin = 7; 
  j31.pin = 11;
  j32.pin = 12;
  j33.pin = 13;
  
  joints[0] = j11; 
  joints[1] = j12; 
  joints[2] = j13; 
  joints[3] = j21;
  joints[4] = j22; 
  joints[5] = j23; 
  joints[6] = j31; 
  joints[7] = j32;
  joints[8] = j33;

  for (joint j: joints){
    initServos();
    j.servo.attach(j.pin);
  }

  delay(1000);
  timeToMove = millis();
}


void loop() {
  if (Serial.available() != 0) {
     whichsetpoint = Serial.parseInt();
  }
  
  if (millis() - timeToMove >= delta){
    setpointcontroller(c);
    timeToMove = millis();
  }
  
  if (there){
    Serial.print("Reached setpoint ");Serial.println(c);
    there=false;
    c = (c+1)%numsetpoints;
    //Serial.print("going to setpoint ");Serial.println(c);
  }
  else{
    //Serial.println(c);
    //Serial.print("Moving to setpoint ");Serial.println((c+1)%numsetpoints);
  }

  if (!dmpReady) return;
  //calc roll
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = (ypr[2] * 180/M_PI);
    pitch = (ypr[1] * 180/M_PI);
   }
   
}

void checkThere(){
  for (int i = 0; i < numjoints; i++){
    //Serial.print("setpoint goal: ");Serial.println(setpoints[c][i]);
    //Serial.print("current pos: ");Serial.println(joints[i].pos);
    if (abs(setpoints[whichsetpoint][c][i] - joints[i].pos) > 1){
      there = false;
      return;
    }
  }
  there = true;
}

void setpointcontroller(int c){
  for (int i = 0; i < numjoints; i++){
    joint& j = joints[i];
    double error = setpoints[whichsetpoint][c][i] - j.pos;
    j.pos = constrain(j.pos + P_VAL * error, j.flo, j.ciel);
    j.servo.write(j.pos);
    //Serial.print("pin: ");Serial.print(j.pin);Serial.print(" at ");Serial.print(j.pos);Serial.print(", moving towards "); Serial.println(setpoints[whichsetpoint][c][i]);
    checkThere();  
  }
}

void initServos(){
  joints[0].servo.write(40);
  joints[1].servo.write(170);
  joints[2].servo.write(50);
  joints[3].servo.write(40);
  joints[4].servo.write(170);
  joints[5].servo.write(50);
  joints[6].servo.write(40);
  joints[7].servo.write(170);
  joints[8].servo.write(50);
}
