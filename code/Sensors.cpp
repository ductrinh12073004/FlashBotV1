#include "Sensors.h"

VL53L0X sensorL, sensorF, sensorR, sensorL1, sensorR1;
int senL, senF, senR, senR1, senL1;

// Buffers cho lọc nhiễu
const byte N = 2;    
int buf1[N], buf2[N], buf3[N], buf0[N], buf4[N];
byte idx = 0;

int thresholdF = 80;
int thresholdSide = 15;

void Setupsensor(){
  Wire.begin();
  pinMode(XshutL, OUTPUT); pinMode(XshutL1, OUTPUT);
  pinMode(XshutF, OUTPUT); pinMode(XshutR1, OUTPUT); pinMode(XshutR, OUTPUT);

  digitalWrite(XshutL,LOW); digitalWrite(XshutL1,LOW);
  digitalWrite(XshutF,LOW); digitalWrite(XshutR1,LOW); digitalWrite(XshutR,LOW);
  
  // Khởi tạo từng cảm biến
  digitalWrite(XshutL, HIGH); delay(100); sensorL.init(); sensorL.setAddress(0x30);
  digitalWrite(XshutL1, HIGH); delay(100); sensorL1.init(); sensorL1.setAddress(0x31);
  digitalWrite(XshutF, HIGH); delay(100); sensorF.init(); sensorF.setAddress(0x32);
  digitalWrite(XshutR1, HIGH); delay(100); sensorR1.init(); sensorR1.setAddress(0x33);
  digitalWrite(XshutR, HIGH); delay(100); sensorR.init(); sensorR.setAddress(0x34);

  sensorL.startContinuous(); sensorL1.startContinuous();
  sensorF.startContinuous(); sensorR1.startContinuous(); sensorR.startContinuous();

  for (byte i = 0; i < N; i++) {
    buf1[i] = buf2[i] = buf3[i] = buf0[i] = buf4[i] = 0;
  }
}

void readSensor() {
  buf1[idx] = (sensorL1.readRangeContinuousMillimeters())/10;
  buf2[idx] = (sensorF.readRangeContinuousMillimeters())/10;
  buf3[idx] = (sensorR1.readRangeContinuousMillimeters())/10;
  idx = (idx + 1) % N;

  long sumF = 0, sumL1 = 0, sumR1 = 0;
  for (byte i = 0; i < N; i++) {
    sumL1 += buf1[i]; sumF += buf2[i]; sumR1 += buf3[i];
  }
  senL1 = sumL1 / N; senF = sumF / N; senR1 = sumR1 / N;
}

int API_wallFront() { return senF <= thresholdF; } // Dùng giá trị đã đọc
int API_wallRight() { return (sensorR.readRangeContinuousMillimeters()/10) <= thresholdSide; }
int API_wallLeft()  { return (sensorL.readRangeContinuousMillimeters()/10) <= thresholdSide; }
