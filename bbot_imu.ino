#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>

#define PI 3.14159265
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

struct joint{
  Servo servo;
  int pin;
  float pos;
  int flo = 0;
  int ciel = 130; 
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

unsigned long delta = 10;
unsigned long timeToMove;
int c = 0;
float eps = 1;
float balance_eps = 0.3;
int walkSPL[2] = {180,40};
int stand_x = 70;

char temp;
char which_func = '0';
bool newData = false;
const float y_ground = -226;
const float active_mag = 1;
const float reactive_mag = 2;
const int universaloffset1 = 50;
const int universaloffset2 = 16;
const int compression_offset = 6; // affects traction mode only
const int foot_offset = 5+compression_offset;
int L1 = 136;
int L2 = 256 - compression_offset;
const int min_x = 30;
const int max_x = 220;

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
  // join I2C bus 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. 
    Wire.setWireTimeout(3000,true);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply gyro offsets here
  mpu.setXGyroOffset(177);
  mpu.setYGyroOffset(-30);
  mpu.setZGyroOffset(-89);
  mpu.setXAccelOffset(-4105);
  mpu.setYAccelOffset(-2437);
  mpu.setZAccelOffset(507);
  // make sure it worked
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP
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
  if (Serial.available() > 0) {
     temp = Serial.read();
     newData = true;
  }
  if (newData == true) {
        Serial.print("Received char input: ");Serial.println(temp);
        which_func = temp;
        newData = false;
  }
  if (millis() - timeToMove >= delta){
    switch (which_func){
      case '1':
        walk_leg1();
        break;
      case '2':
        walk_leg2();
        break;
      case '3':
        walk_leg3();
        break;
      default:
        stand();
        break;
    }
    timeToMove = millis();
    Serial.print("roll: ");Serial.print(roll);Serial.print(" pitch: ");Serial.println(pitch);
  }

  if (!dmpReady) return;
  //calc roll+pitch
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = (ypr[2] * 180/M_PI);
    pitch = (ypr[1] * 180/M_PI);
    //Serial.print("roll: ");Serial.println(roll);
    //Serial.print("pitch: ");Serial.println(pitch);
   }
}

void walk_leg1(){
  switch(c){
    case 0:
      leg1.x = walkSPL[1];
      leg1.y = y_ground;
      /*leg2.x = walkSPL[1];
      leg2.y = y_ground;
      leg3.x = walkSPL[1];
      leg3.y = y_ground;*/
      c = 1;
      break;
    case 1:
      extendLeg(0,1,1,active_mag);
      if (leg1.x >= walkSPL[0]){
        c = 2;
      }
      break;
    case 2:
      extendLeg(0,-1,0,active_mag);
      if (leg1.x <= walkSPL[1]){
        c = 0;
      }
      break;
  }
  if (roll > balance_eps){
    extendLeg(1,-1,1,reactive_mag);
    extendLeg(2,-1,1,reactive_mag);
  }
  else if (roll < -1.0*balance_eps){
    extendLeg(1,1,1,reactive_mag);
    extendLeg(2,1,1,reactive_mag);
  }
  if (pitch > balance_eps){
    extendLeg(1,-1,1,reactive_mag);
    extendLeg(2,1,1,reactive_mag);
  }
  else if (pitch < -1.0*balance_eps){
    extendLeg(1,1,1,reactive_mag);
    extendLeg(2,-1,1,reactive_mag);
  }
  if (abs(roll) < balance_eps && abs(pitch) < balance_eps){
    extendLeg(1,1,1,0);
    extendLeg(2,1,1,0);
  }
  writeAll();
}

void walk_leg2(){
  switch(c){
    case 0:
     // leg1.x = walkSPL[1];
     // leg1.y = y_ground;
      leg2.x = walkSPL[1];
      leg2.y = y_ground;
     // leg3.x = walkSPL[1];
     // leg3.y = y_ground;
      c = 1;
      break;
    case 1:
      extendLeg(1,1,1,active_mag);
      if (leg2.x >= walkSPL[0]){
        c = 2;
      }
      break;
    case 2:
      extendLeg(1,-1,0,active_mag);
      if (leg2.x <= walkSPL[1]){
        c = 0;
      }
      break;
  }
  if (roll > balance_eps){
    extendLeg(0,1,1,reactive_mag);
  }
  else if (roll < -1*balance_eps){
    extendLeg(0,-1,1,reactive_mag);
  }
  else{
    extendLeg(0,1,1,0);
  }
  if (pitch > balance_eps){
    extendLeg(2,1,1,reactive_mag);
  }
  else if (pitch < -1*balance_eps){
    extendLeg(2,-1,1,reactive_mag);
  }
  else{
    extendLeg(2,1,1,0);
  }
  writeAll();
}

void walk_leg3(){
  switch(c){
    case 0:
/*      leg1.x = walkSPL[1];
      leg1.y = y_ground;
      leg2.x = walkSPL[1];
      leg2.y = y_ground;*/
      leg3.x = walkSPL[1];
      leg3.y = y_ground;
      c = 1;
      break;
    case 1:
      extendLeg(2,1,1,active_mag);
      if (leg3.x >= walkSPL[0]){
        c = 2;
      }
      break;
    case 2:
      extendLeg(2,-1,0,active_mag);
      if (leg3.x <= walkSPL[1]){
        c = 0;
      }
      break;
  }
  if (roll > balance_eps){
    extendLeg(0,1,1,reactive_mag);
  }
  else if (roll < -1*balance_eps){
    extendLeg(0,-1,1,reactive_mag);
  }
  else{
    extendLeg(0,1,1,0);
  }
  if (pitch > balance_eps){
    extendLeg(1,-1,1,reactive_mag);
  }
  else if (pitch < -1*balance_eps){
    extendLeg(1,1,1,reactive_mag);
  }
  else{
    extendLeg(1,1,1,0);
  }
  writeAll();
}

void stand(){
  leg1.x = stand_x;
  leg1.y = y_ground;
  leg2.x = stand_x;
  leg2.y = y_ground;
  leg3.x = stand_x;
  leg3.y = y_ground;
  extendLeg(0,1,1,0);
  extendLeg(1,1,1,0);
  extendLeg(2,1,1,0);
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
  l->x = constrain(l->x+dir*mag, min_x, max_x);
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
    l->joint1->servo.write(constrain(l->joint1->pos, l->joint1->flo, l->joint1->ciel));
    l->joint2->servo.write(constrain(l->joint2->pos, l->joint2->flo, l->joint2->ciel)); 
    l->joint3->servo.write(constrain(l->joint3->pos, l->joint3->flo, l->joint3->ciel));
    if (l->index == 1){
      //Serial.print("joint 1: ");Serial.println(l->joint1->pos);
      //Serial.print("joint 2: ");Serial.println(l->joint2->pos);
      //Serial.print("j3: ");Serial.println(l->joint3->pos);
    }
  }
}


