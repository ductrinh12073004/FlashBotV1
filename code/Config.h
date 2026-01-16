#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


#define LED_PIN PC13
#define BUTTON1 PB8
#define BUTTON2 PB9

// Motor & Driver
#define PWMA PB13
#define AIN1 PB0
#define AIN2 PB1
#define PWMB PB14
#define BIN1 PB10
#define BIN2 PB11

// Encoders
#define ENCLA PA1
#define ENCLB PA0
#define ENCRA PA6
#define ENCRB PA7

// Sensors XSHUT
#define XshutL PB4
#define XshutL1 PB3
#define XshutF PA15
#define XshutR1 PA12
#define XshutR PA11


#define MAZE_WIDTH 10
#define MAZE_HEIGHT 11
#define STARTING_X 0
#define STARTING_Y 0
#define LOWER_X_GOAL 7
#define LOWER_Y_GOAL 7
#define UPPER_X_GOAL 8
#define UPPER_Y_GOAL 8

#define STARTING_TARGET 1
#define TURN_SCORE 0
#define TILE_SCORE 1
#define STREAK_SCORE 0
#define OUT_OF_BOUNDS -2
#define NOT_YET_SET -1


#define BASE_ENCODER_PER_CELL 2350 
#define SPEED_RUN_SPEED 230    
#define BASE_SPEED 120

// PID Constants
const float Kp = 5.5; 
const float Ki = 0.001;
const float Kd = 7.5; 

#endif
