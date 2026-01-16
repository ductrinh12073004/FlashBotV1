#include "MazeSolver.h"

// === Variables ===
RobotState currentState = EXPLORING;
coord currentXY = {STARTING_X, STARTING_Y};
Heading currentHeading = (Heading)STARTING_HEADING;
unsigned char target = STARTING_TARGET;

int verticalWalls[MAZE_WIDTH+1][MAZE_HEIGHT] = {{0}};
int horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT+1] = {{0}};
int floodArray[MAZE_WIDTH][MAZE_HEIGHT];
int previousFlood_Target1[MAZE_WIDTH][MAZE_HEIGHT];

OptimizedMove optimizedPath[100];
int optimizedPathLength = 0;
extern unsigned long waitStartTime; // Defined in main if needed, or manage here

// === Internal Structs for Queue ===
typedef struct neighbor { coord pos; Heading heading; int streak; } neighbor;
typedef neighbor item_type;
struct node { item_type data; struct node* next; };
struct _queue { struct node* head; struct node* tail; int size; };
typedef struct _queue* queue;

// ... (Copy Queue Implementation Functions: halt_with_error, queue_create, push, pop, clear here) ...
void halt_with_error(const char* msg) { Serial.print("ERROR: "); Serial.println(msg); motorStop(); while(1); }
queue queue_create() { queue q = (queue)malloc(sizeof(struct _queue)); q->head = NULL; q->tail = NULL; q->size=0; return q; }
int queue_is_empty(queue q) { return q->head == NULL; }
void queue_push(queue q, item_type elem) {
    struct node* n = (struct node*)malloc(sizeof(struct node));
    n->data = elem; n->next = NULL;
    if (q->head == NULL) q->head = q->tail = n; else { q->tail->next = n; q->tail = n; }
    q->size++;
}
item_type queue_pop(queue q) {
    struct node* head = q->head;
    if (q->head == q->tail) { q->head = NULL; q->tail = NULL; } else { q->head = head->next; }
    q->size--; item_type data = head->data; free(head); return data;
}
void queue_clear(queue q) { while (!queue_is_empty(q)) queue_pop(q); }

// === Logic FloodFill ===
void updateFloodArray(coord c, int val) { floodArray[c.x][c.y] = val; }

int isAccessible(coord c1, coord c2) {
    if (c1.x == c2.x) { if (c1.y > c2.y) return !horizontalWalls[c1.x][c1.y]; else return !horizontalWalls[c1.x][c2.y]; }
    if (c1.y == c2.y) { if (c1.x > c2.x) return !verticalWalls[c1.x][c1.y]; else return !verticalWalls[c2.x][c1.y]; }
    return 0;
}

void generateNeighbor(queue q, neighbor n, Heading h, int streak) {
    coord newCoord = n.pos;
    switch(h) { case NORTH: newCoord.y++; break; case WEST: newCoord.x--; break; case SOUTH: newCoord.y--; break; case EAST: newCoord.x++; break; }
    if (newCoord.x < 0 || newCoord.x >= MAZE_WIDTH || newCoord.y < 0 || newCoord.y >= MAZE_HEIGHT) return;
    if (!isAccessible(n.pos, newCoord)) return;
    int floodVal = floodArray[n.pos.x][n.pos.y];
    int newFloodVal = floodArray[newCoord.x][newCoord.y];
    int score = TILE_SCORE + ((n.heading == h) ? 0 : TURN_SCORE) - ((n.heading == h) ? STREAK_SCORE : 0);
    if (newFloodVal == NOT_YET_SET || newFloodVal > floodVal + score) {
        updateFloodArray(newCoord, floodVal + score);
        queue_push(q, {newCoord, h, (n.heading == h) ? streak + 1 : 1});
    }
}

void floodFill() {
    for (int i=0; i < MAZE_WIDTH; i++) for (int j=0; j < MAZE_HEIGHT; j++) floodArray[i][j] = NOT_YET_SET;
    queue q = queue_create();
    if (target) {
        for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++) for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++) {
            updateFloodArray({x, y}, 0); queue_push(q, {{x, y}, NORTH, 0});
        }
    } else { updateFloodArray({STARTING_X, STARTING_Y}, 0); queue_push(q, {{STARTING_X, STARTING_Y}, NORTH, 0}); }
    
    while (!queue_is_empty(q)) {
        neighbor current = queue_pop(q);
        generateNeighbor(q, current, NORTH, current.streak); generateNeighbor(q, current, WEST, current.streak);
        generateNeighbor(q, current, SOUTH, current.streak); generateNeighbor(q, current, EAST, current.streak);
    }
    queue_clear(q); free(q);
}

void updateWalls() {
    if (API_wallFront()) {
        switch (currentHeading) {
            case NORTH: horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
            case WEST: verticalWalls[currentXY.x][currentXY.y] = 1; break;
            case SOUTH: horizontalWalls[currentXY.x][currentXY.y] = 1; break;
            case EAST: verticalWalls[currentXY.x+1][currentXY.y] = 1; break;
        }
    }
    if (API_wallLeft()) {
        switch (currentHeading) {
            case NORTH: verticalWalls[currentXY.x][currentXY.y] = 1; break;
            case WEST: horizontalWalls[currentXY.x][currentXY.y] = 1; break;
            case SOUTH: verticalWalls[currentXY.x+1][currentXY.y] = 1; break;
            case EAST: horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
        }
    }
    if (API_wallRight()) {
        switch (currentHeading) {
            case NORTH: verticalWalls[currentXY.x+1][currentXY.y] = 1; break;
            case WEST: horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
            case SOUTH: verticalWalls[currentXY.x][currentXY.y] = 1; break;
            case EAST: horizontalWalls[currentXY.x][currentXY.y] = 1; break;
        }
    }
}

Heading findBestHeading(coord pos, Heading heading) {
  int currentFlood = floodArray[pos.x][pos.y];
  int northFlood = (pos.y + 1 < MAZE_HEIGHT && isAccessible(pos, {pos.x, pos.y+1})) ? floodArray[pos.x][pos.y+1] : OUT_OF_BOUNDS;
  int westFlood  = (pos.x - 1 >= 0 && isAccessible(pos, {pos.x-1, pos.y})) ? floodArray[pos.x-1][pos.y] : OUT_OF_BOUNDS;
  int southFlood = (pos.y - 1 >= 0 && isAccessible(pos, {pos.x, pos.y-1})) ? floodArray[pos.x][pos.y-1] : OUT_OF_BOUNDS;
  int eastFlood  = (pos.x + 1 < MAZE_WIDTH && isAccessible(pos, {pos.x+1, pos.y})) ? floodArray[pos.x+1][pos.y] : OUT_OF_BOUNDS;
  int minFlood = currentFlood;
  Heading newHeading = heading;  

  if (northFlood != OUT_OF_BOUNDS && northFlood < minFlood) { minFlood = northFlood; newHeading = NORTH; }
  if (westFlood  != OUT_OF_BOUNDS && westFlood  < minFlood) { minFlood = westFlood; newHeading = WEST;  }
  if (southFlood != OUT_OF_BOUNDS && southFlood < minFlood) { minFlood = southFlood; newHeading = SOUTH; }
  if (eastFlood  != OUT_OF_BOUNDS && eastFlood  < minFlood) { minFlood = eastFlood;  newHeading = EAST; }
  return newHeading;
}

Action nextAction() {
    Heading newHeading = findBestHeading(currentXY, currentHeading);
    if (newHeading == currentHeading) {
        API_moveForward();
        switch (currentHeading) { case NORTH: currentXY.y++; break; case WEST: currentXY.x--; break; case SOUTH: currentXY.y--; break; case EAST: currentXY.x++; break; }
        return FORWARD;
    }
    if (currentHeading == (newHeading+3)%4) { API_turnLeft(); currentHeading = (Heading)((currentHeading + 1) % 4); return LEFT; }
    else if (currentHeading == (newHeading+1)%4) { API_turnRight(); currentHeading = (Heading)((currentHeading == NORTH) ? EAST : currentHeading - 1); return RIGHT; }
    else { API_turn180(); currentHeading = (Heading)((currentHeading + 2) % 4); return IDLE; }
}

bool compareAndCopyFlood_Target1() {
  bool isIdentical = true; 
  for (int i=0; i < MAZE_WIDTH; i++) {
    for (int j=0; j < MAZE_HEIGHT; j++) {
      if (previousFlood_Target1[i][j] != floodArray[i][j]) isIdentical = false; 
      previousFlood_Target1[i][j] = floodArray[i][j];
    }
  }
  return isIdentical;
}

void generateOptimizedPath() {
  Serial.println("Generating Optimized Path...");
  optimizedPathLength = 0;
  coord traceXY = {STARTING_X, STARTING_Y};
  Heading traceHeading = currentHeading;
  if (target != 1) { target = 1; floodFill(); }
  
  while (floodArray[traceXY.x][traceXY.y] != 0) {
    if (optimizedPathLength >= 100) return; 
    int forwardCount = 0;
    Heading bestHeading = findBestHeading(traceXY, traceHeading);
    while (bestHeading == traceHeading && isAccessible(traceXY, {traceXY.x + (traceHeading==EAST?1:traceHeading==WEST?-1:0), traceXY.y + (traceHeading==NORTH?1:traceHeading==SOUTH?-1:0)}) ) {
      forwardCount++;
      switch (traceHeading) { case NORTH: traceXY.y++; break; case WEST: traceXY.x--; break; case SOUTH: traceXY.y--; break; case EAST: traceXY.x++; break; }
      if (floodArray[traceXY.x][traceXY.y] == 0) break;
      bestHeading = findBestHeading(traceXY, traceHeading);
    }
    if (forwardCount > 0) { optimizedPath[optimizedPathLength++] = {P_FORWARD, forwardCount}; }
    if (floodArray[traceXY.x][traceXY.y] == 0) break;
    bestHeading = findBestHeading(traceXY, traceHeading);
    
    if (traceHeading == (bestHeading+3)%4) { optimizedPath[optimizedPathLength++] = {P_LEFT, 1}; traceHeading = (Heading)((traceHeading + 1) % 4); }
    else if (traceHeading == (bestHeading+1)%4) { optimizedPath[optimizedPathLength++] = {P_RIGHT, 1}; traceHeading = (Heading)((traceHeading == NORTH) ? EAST : traceHeading - 1); }
    else { optimizedPath[optimizedPathLength++] = {P_180, 1}; traceHeading = (Heading)((traceHeading + 2) % 4); }
  }
  optimizedPath[optimizedPathLength++] = {P_STOP, 0};
}

void checkDestination() {
  if (currentState != EXPLORING) return;
  if (target) {
    if (currentXY.x >= LOWER_X_GOAL && currentXY.x <= UPPER_X_GOAL && currentXY.y >= LOWER_Y_GOAL && currentXY.y <= UPPER_Y_GOAL) {
      Serial.println("GOAL Reached! Returning to START...");
      target = 0; 
    }
  } else {
    if (currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
      Serial.println("Back at START. Checking convergence...");
      target = 1;
      floodFill(); 
      if (compareAndCopyFlood_Target1()) {
        Serial.println("=> CONVERGED! Switching to WAITING.");
        API_turn180(); currentHeading = (Heading)((currentHeading + 2) % 4);
        generateOptimizedPath();
        currentState = WAITING; 
      } else {
        Serial.println("=> NOT CONVERGED. Exploring more...");
      }
    }
  }
}

void executeSpeedRun() {
  currentXY = {STARTING_X, STARTING_Y};
  currentHeading = (Heading)STARTING_HEADING;
  stepMove = 1;
  for (int i = 0; i < optimizedPathLength; i++) {
    OptimizedMove move = optimizedPath[i];
    switch (move.action) {
      case P_FORWARD:
        moveMultipleCells(move.value, SPEED_RUN_SPEED);
        switch (currentHeading) { case NORTH: currentXY.y += move.value; break; case WEST:  currentXY.x -= move.value; break; case SOUTH: currentXY.y -= move.value; break; case EAST:  currentXY.x += move.value; break; }
        break;
      case P_LEFT: API_turnLeft(); currentHeading = (Heading)((currentHeading + 1) % 4); break;
      case P_RIGHT: API_turnRight(); currentHeading = (Heading)((currentHeading == NORTH) ? EAST : currentHeading - 1); break;
      case P_180: API_turn180(); currentHeading = (Heading)((currentHeading + 2) % 4); break;
      case P_STOP:
        API_turn180(); currentHeading = (Heading)((currentHeading + 2) % 4);
        currentState = WAITING; return;
    }
    printMaze();
  }
}

void generateInitialWalls() {
    for (int i=0; i < MAZE_WIDTH; i++) { horizontalWalls[i][0] = 1; horizontalWalls[i][MAZE_HEIGHT] = 1; }
    for (int i=0; i < MAZE_HEIGHT; i++) { verticalWalls[0][i] = 1; verticalWalls[MAZE_WIDTH][i] = 1; }
    for (int i=0; i < MAZE_WIDTH; i++) for (int j=0; j < MAZE_HEIGHT; j++) previousFlood_Target1[i][j] = NOT_YET_SET;
}

Action solver() {
  checkDestination();
  if (currentState != EXPLORING) return IDLE;
  updateWalls();
  floodFill();
  return nextAction();
}

void printMaze() {
  
    Serial.println("--- PRINT MAZE ---");
    
}
