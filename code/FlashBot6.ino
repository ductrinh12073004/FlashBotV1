#include "Config.h"
#include "Types.h"
#include "Sensors.h"
#include "Motors.h"
#include "MazeSolver.h"

void setup() {
  Serial.begin(115200); while (!Serial);
  Setupmotor();
  Setupsensor();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Robot Initialized!");
  delay(2000); 
  generateInitialWalls();
  Serial.println("Walls Created. Start Exploring...");
}

void loop() {
  switch (currentState) {
    case EXPLORING:
      solver();
      printMaze();
      break;
      
    case WAITING:
      digitalWrite(LED_PIN, HIGH); delay(150);
      digitalWrite(LED_PIN, LOW); delay(150);
      if (digitalRead(BUTTON1) == LOW) {
        Serial.println("Button Pressed! Starting SPEED_RUN...");
        delay(50); while(digitalRead(BUTTON1) == LOW);
        digitalWrite(LED_PIN, LOW);
        currentState = SPEED_RUN;
      }
      break;
      
    case SPEED_RUN:
      executeSpeedRun();
      break;
  }
}
