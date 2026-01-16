#ifndef TYPES_H
#define TYPES_H

typedef enum Heading {NORTH, WEST, SOUTH, EAST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE} Action;
typedef struct coord { int x; int y; } coord;

// Các trạng thái của Robot
typedef enum { EXPLORING, WAITING, SPEED_RUN } RobotState;

// Cấu trúc cho đường đi tối ưu (Gộp ô)
typedef enum PathAction {
  P_LEFT, P_RIGHT, P_180,
  P_FORWARD, 
  P_STOP
} PathAction;

typedef struct {
  PathAction action;
  int value; // Số ô nếu là P_FORWARD
} OptimizedMove;

#endif
