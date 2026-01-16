#include "Motors.h"

// Biến nội bộ PID
float error = 0, previous_error = 0, integral = 0;
int stepMove = 1;
volatile long countLeft = 0;
volatile long countRight = 0;
int cell;
int turn = 680;
int turnRound = 1525;

// --- Interrupt Handlers (Internal) ---
void updateEncoder(volatile long &count, uint8_t chanA, uint8_t chanB, bool isAchange) {
  bool a = digitalRead(chanA); bool b = digitalRead(chanB);
  if (isAchange) { if (a == b) count++; else count--; }
  else { if (a != b) count++; else count--; }
}
void encoderLeftA()  { updateEncoder(countLeft,  ENCLA, ENCLB, true); }
void encoderLeftB()  { updateEncoder(countLeft,  ENCLA, ENCLB, false); }
void encoderRightA() { updateEncoder(countRight, ENCRA, ENCRB, true); }
void encoderRightB() { updateEncoder(countRight, ENCRA, ENCRB, false); }

void Setupmotor(){
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(ENCLA, INPUT_PULLUP); pinMode(ENCLB, INPUT_PULLUP);
  pinMode(ENCRA, INPUT_PULLUP); pinMode(ENCRB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCLA), encoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCLB), encoderLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCRA), encoderRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCRB), encoderRightB, CHANGE);
}

void setMotorL(int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm > 0){ digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, pwm); }
  else if(pwm<0){ digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -pwm); }
  else { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 0); }
}

void setMotorR(int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm > 0){ digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm); }
  else if(pwm<0){ digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -pwm); }
  else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 0); }
}

void setMotor(int left, int right){ setMotorL(left); setMotorR(right); }
void resetEncoder(){ countLeft = 0; countRight = 0; }
void motorStop(){ setMotor(0,0); }
void driveBreak(){
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
}

// --- PID Logic ---
float getError() { error = senL1 - senR1; return constrain(error, -10, 10); }
float getErrorL(){ error = senL1 - 9; return constrain(error, -10, 10); }
float getErrorR(){ error = 9 - senR1; return constrain(error, -10, 10); }

float calculate_pid_generic(float e) {
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) integral += e;
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

void moveOneCell(){
  resetEncoder();
  cell = (stepMove == 1) ? 2610 : 2310;
  integral = 0; previous_error = 0;

  while ((abs(countLeft) + abs(countRight))/2 <= cell) {
    readSensor();
    int speed = BASE_SPEED;
    
    // Giảm tốc khi gặp tường trước
    if (senF <= 25) {
      int scale = map((int)senF, 25, 7, 100, 60);
      speed = (speed * constrain(scale, 60, 100)) / 100;
    } 
    if(senF <= 5) break;

    int wallThreshold = 13; int noWallThreshold = 14;
    bool leftWall = (senL1 <= wallThreshold); bool rightWall = (senR1 <= wallThreshold);
    bool noLeftWall = (senL1 >= noWallThreshold); bool noRightWall = (senR1 >= noWallThreshold);

    if (leftWall && rightWall && senF >= 18) {
       float pid = calculate_pid_generic(getError());
       setMotor(constrain(speed + pid, 0, 150), constrain(speed - pid, 0, 150));
    } else if (leftWall && noRightWall && senF >= 12) {
       float pid = calculate_pid_generic(getErrorL());
       setMotor(constrain(speed + pid, 0, 150), constrain(speed - pid, 0, 150));
    } else if (rightWall && noLeftWall && senF >= 12) {
       float pid = calculate_pid_generic(getErrorR());
       setMotor(constrain(speed + pid, 0, 150), constrain(speed - pid, 0, 150));
    } else {
       setMotor(speed, speed);
       integral = 0; previous_error = 0;
    }
  }
  stepMove++;
  driveBreak(); motorStop();
}

void moveMultipleCells(int numCells, int runSpeed) {
  long targetEncoderCount = (long)BASE_ENCODER_PER_CELL * numCells;
  if(numCells <=2) targetEncoderCount -= 210;
  if (numCells > 4) targetEncoderCount += 600;

  resetEncoder(); integral = 0; previous_error = 0;

  while ((abs(countLeft) + abs(countRight)) / 2 <= targetEncoderCount) {
    readSensor();
    int speed = runSpeed; 
    if (senF <= 20) {
       int scale = map((int)senF, 20, 7, 100, 50);
       speed = (speed * constrain(scale, 60, 100)) / 100;
    }  
    if(senF <= 6) break;
    
    // ... (Logic PID giống moveOneCell nhưng tốc độ cao hơn - Rút gọn cho ngắn)
    // Bạn copy logic PID bên trong vòng while của moveMultipleCells gốc vào đây
     int wallThreshold = 17; 
    int noWallThreshold = 15;
    bool leftWall = (senL1 <= wallThreshold);
    bool rightWall = (senR1 <= wallThreshold);
    bool noLeftWall = (senL1 >= noWallThreshold);
    bool noRightWall = (senR1 >= noWallThreshold);
    if (leftWall && rightWall && senF >= 18) {
        float pid = calculate_pid_generic(getError());
        int leftSpeed  = constrain(speed + pid, 0, 230);
        int rightSpeed = constrain(speed - pid, 0, 230);
        setMotor(leftSpeed, rightSpeed);
    } else if (leftWall && noRightWall && senF >= 12) {
        float pidL = calculate_pid_generic(getErrorL());
        int leftSpeed  = constrain(speed + pidL, 0, 230);
        int rightSpeed = constrain(speed - pidL, 0, 230);
        setMotor(leftSpeed, rightSpeed);
    } else if (rightWall && noLeftWall && senF >= 12) {
        float pidR = calculate_pid_generic(getErrorR());
        int leftSpeed  = constrain(speed + pidR, 0, 230);
        int rightSpeed = constrain(speed - pidR, 0, 230);
        setMotor(leftSpeed, rightSpeed);
    } else {
        setMotor(speed, speed);
        integral = 0;
        previous_error = 0;
    }
  }
  driveBreak(); motorStop();
}

void turnL(){
  resetEncoder();
  while((abs(countLeft) + abs(countRight))/2 <= turn) setMotor(-170,170);
  driveBreak(); motorStop(); delay(50);
}

void turnR(){
  resetEncoder();
  while((abs(countLeft) + abs(countRight))/2 <= turn) setMotor(170,-170);
  driveBreak(); motorStop(); delay(50);
}

void turn180(){
  resetEncoder(); readSensor();
  if(senF >= 16){
    while((abs(countLeft) + abs(countRight))/2 <= turnRound) setMotor(-150,150);
    driveBreak(); motorStop();
  } else{
    while((abs(countLeft) + abs(countRight))/2 <= turnRound) setMotor(150,-150);
    driveBreak(); motorStop(); setMotor(-70,-70); delay(700);
    driveBreak(); motorStop(); delay(50); stepMove = 1;
  }  
}

// Wrapper Definitions
void API_moveForward() { moveOneCell(); }
void API_turnRight() { turnR(); }
void API_turnLeft() { turnL(); }
void API_turn180() { turn180(); }
