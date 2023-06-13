 
#include <Servo.h>

Servo s1;
Servo s2;
Servo s3;
Servo s4;
Servo s5;
Servo s6;
Servo s7;
Servo s8;
Servo s9;

const int universaloffset1 = 50;
const int universaloffset2 = 16;
const int leg1offset1 = -6;
const int leg1offset2 = -14;
const int leg2offset1 = 5;
const int leg2offset2 = 0;
const int leg3offset1 = 6;
const int leg3offset2 = 3;
int q1 = 14;
int q2 = 114;


void setup() {
  //leg 1
  
  s1.attach(4);
  s2.attach(5);
  s3.attach(6);
  //leg 2
  s4.attach(7);
  s5.attach(8);
  s6.attach(9);
  //leg 3
  s7.attach(10);
  s8.attach(11);
  s9.attach(12);
}

void loop() {
  //leg 1
  
  s1.write(q1+universaloffset1+leg1offset1);
  s2.write(q2+universaloffset2+leg1offset2);
  s3.write(0);
  //leg 2
  s4.write(q1+universaloffset1+leg2offset1);  
  s5.write(q2+universaloffset2+leg2offset2);
  s6.write(0);
  //leg 3
  s7.write(q1+universaloffset1+leg3offset1);  
  s8.write(q2+universaloffset2+leg3offset2);
  s9.write(0);
}
