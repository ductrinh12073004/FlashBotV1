#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <VL53L0X.h>
#include "Config.h"

// Khai báo biến global để các file khác dùng được
extern int senL, senF, senR, senR1, senL1;
extern VL53L0X sensorF, sensorL, sensorR, sensorL1, sensorR1;

void Setupsensor();
void readSensor();
int API_wallFront();
int API_wallLeft();
int API_wallRight();

#endif
