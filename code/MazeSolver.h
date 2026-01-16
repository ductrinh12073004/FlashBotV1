#ifndef MAZESOLVER_H
#define MAZESOLVER_H

#include "Config.h"
#include "Types.h"
#include "Sensors.h"
#include "Motors.h"

// Variables shared with main
extern RobotState currentState;
extern coord currentXY;
extern Heading currentHeading;
extern unsigned char target;
extern int floodArray[MAZE_WIDTH][MAZE_HEIGHT];
extern int previousFlood_Target1[MAZE_WIDTH][MAZE_HEIGHT];
extern OptimizedMove optimizedPath[100];
extern int optimizedPathLength;

// Functions
void initSolver();
void generateInitialWalls();
Action solver();
void checkDestination();
void generateOptimizedPath();
void executeSpeedRun();
void printMaze();
bool compareAndCopyFlood_Target1();

#endif
