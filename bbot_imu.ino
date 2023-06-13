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

const int delta = 100;
unsigned long timeToMove;
int c = 0;
int eps = 1;
int balance_eps = 5;
int walkSPL[2] = {200,100};

struct joint{
  Servo servo;
  int pin;
  float pos;
  int flo = 0;
  int ciel = 180; 
};

struct leg{
  int index;
  joint * joint1;
  joint * joint2;
  joint * joint3;
  float x;
  float y;
  int offset1;
  int offset2;
};

struct q_struct{
  int q1;
  int q2;
};

joint j11;joint j12; joint j13; joint j21; joint j22;joint j23; joint j31; joint j32; joint j33;
leg leg1 = {0,&j11,&j12,&j13,0,0,-6,-14};
leg leg2 = {1,&j21,&j22,&j23,0,0,5,0};
leg leg3 = {2,&j31,&j32,&j33,0,0,6,3};
leg * legs[3] = {&leg1,&leg2,&leg3};

const int universaloffset1 = 50;
const int universaloffset2 = 16;
const int foot_offset = 8;
#define PI 3.14159265
int L1 = 136;
int L2 = 260;

void setup() {
  
  j11.pin = 4;  
  j12.pin = 5; 
  j13.pin = 6; 
  j21.pin = 7;
  j22.pin = 8;
  j23.pin = 9; 
  j31.pin = 10;
  j32.pin = 11;
  j33.pin = 12;

  for (leg * l: legs){
    l->joint1->servo.attach(l->joint1->pin);
    l->joint2->servo.attach(l->joint2->pin);
    l->joint3->servo.attach(l->joint3->pin);
  }

  // ***** start mpu setup *****
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
  // ***** end mpu setup *****

  delay(1000);
  timeToMove = millis();
  Serial.println("BEGIN LOOP");
}

void loop() {
  if (millis() - timeToMove >= delta){
    walk_leg1();
    timeToMove = millis();
  }

  if (!dmpReady) return;
  //calc roll+pitch
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = (ypr[2] * 180/M_PI);
    pitch = (ypr[1] * 180/M_PI);
    //Serial.print("pitch: ");Serial.println(pitch);
    //Serial.print("roll: ");Serial.println(roll);
   }
}

void walk_leg1(){
  switch(c){
    case 0:
      leg1.x = 100;
      leg1.y = -226;
      leg2.x = 100;
      leg2.y = -226;
      leg3.x = 100;
      leg3.y = -226;
      c = 1;
      break;
    case 1:
      extendLeg(0,1,1,3);
      if (leg1.x >= walkSPL[0]){
        c = 2;
      }
      break;
    case 2:
      extendLeg(0,-1,0,3);
      if (leg1.x <= walkSPL[1]){
        c = 0;
      }
      break;
  }
  if (roll > balance_eps){
    extendLeg(1,-1,1,1);
    extendLeg(2,-1,1,1);
  }
  else if (roll < balance_eps){
    extendLeg(1,1,1,1);
    extendLeg(2,1,1,1);
  }
  if (pitch > balance_eps){
    extendLeg(1,-1,1,1);
    extendLeg(2,1,1,1);
  }
  else if (pitch < balance_eps){
    extendLeg(1,1,1,1);
    extendLeg(2,-1,1,1);
  }
  writeAll();
}

void extendLeg(int index, int dir, int foot_mode, int mag){
  q_struct qs = IK(index, dir, 1, 0, mag);
  float q1 = qs.q1;
  float q2 = qs.q2;
  legs[index]->joint1->pos = q1+universaloffset1+legs[index]->offset1;
  legs[index]->joint2->pos = q2+universaloffset2+legs[index]->offset2;
  if (foot_mode == 0)
    legs[index]->joint3->pos = 20;
  else
    legs[index]->joint3->pos = 0;
  /*if (index == 0){
    Serial.print("joint1: ");Serial.println(l.joint1.pos);
    Serial.print("joint2: ");Serial.println(l.joint2.pos);
  }*/
}

struct q_struct IK(int index, int dir, int ornt, int foot_mode, int mag){
  leg * l = *(legs+index);
  l->x = l->x+dir*mag;
  float x = l->x;
  float y = l->y;
  float link2 = L2 + foot_offset*foot_mode;
  float q2 = ornt*acos((pow(x,2) + pow(y,2) - pow(L1,2) - pow(link2,2))/(2*L1*link2))*180.0/PI;
  float q1 = atan(y/x)*180.0/PI + atan((L2*sin(q2*PI/180.0))/(L1+link2*cos(q2*PI/180.0)))*180.0/PI;
  q_struct qs = {q1,q2};
  return qs;
}

void writeAll(){
  for (leg * l: legs){
    l->joint1->servo.write(l->joint1->pos);
    l->joint2->servo.write(l->joint2->pos); 
    l->joint3->servo.write(l->joint3->pos);
    /*if (l->index == 1){
      Serial.print("j1: ");Serial.println(l->joint1->pos);
      Serial.print("j2: ");Serial.println(l->joint2->pos);
      Serial.print("j3: ");Serial.println(l->joint3->pos);
    }*/
  }
}


