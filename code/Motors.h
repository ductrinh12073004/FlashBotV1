#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include "Sensors.h" // Cần sensors để chạy PID

// Khai báo biến extern
extern int stepMove;
extern volatile long countLeft;
extern volatile long countRight;

void Setupmotor();
void resetEncoder();
void setMotor(int left, int right);
void motorStop();
void driveBreak();

// Các hàm di chuyển cấp cao
void moveOneCell();
void moveMultipleCells(int numCells, int runSpeed);
void turnL();
void turnR();
void turn180();

// API wrappers cho Solver
void API_moveForward();
void API_turnRight();
void API_turnLeft();
void API_turn180();

#endif
