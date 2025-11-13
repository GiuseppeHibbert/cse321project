#include <Arduino_FreeRTOS.h>
#include <Timers.h>

// Stepper pins Driver 0
#define IN1_0 11
#define IN2_0 10
#define IN3_0 9
#define IN4_0 8

// Stepper pins Driver 1
#define IN1_1 7
#define IN2_1 6
#define IN3_1 5
#define IN4_1 4

// Ultrasonic pins
#define TRIG_0 3
#define ECHO_0 4

// Ultrasonic pins
#define TRIG_1 5
#define ECHO_1 6

// Stepper sequence (half-step)
const int stepSequence[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

typedef enum { STATE_OFF, STATE_IDLE, STATE_SEARCH, STATE_TARGET,
               STATE_TRACKING, STATE_ERROR } turret_state_t;
typedef enum { EV_POWER_ON, EV_FOUND, EV_LOST, EV_LEFT,
               EV_RIGHT, EV_ERROR} system_event_t;

// ----- RTOS objects -----
QueueHandle_t xEventQueue;
SemaphoreHandle_t xStateMutex;

inline void pushEvent(system_event_t ev){ 
    xQueueSend(xEventQueue, &ev, 0); 
}

void stateMachineTask(void *){
  system_event_t e;
  for(;;){
    if(xQueueReceive(xEventQueue,&e,portMAX_DELAY)==pdTRUE){
      //Serial.println("Received Event!");
      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      //turret_state_t old=currentState;
      // Starting
      // Note: May need to do something in setup to "kick-start" the program.
      if (currentState==STATE_SEARCH && e==EV_FOUND) {
        // Stop Sensor Stepper Motor
        // Grab Position
        // Move Position of Assembly Stepper Motor to Match Sensor Stepper Motor
        // Move to Target State
      }
      else if ((currentState==STATE_TARGET && e==EV_LEFT) || (currentState==STATE_TARGET && e==EV_RIGHT)) {
        // Move to Tracking State
      }
      else if (currentState==STATE_TRACKING && e==EV_LEFT) {
        // Start Assembly Stepper Motor moving Left
        // Start Out-of-Range Timer
      }
      else if (currentState==STATE_TRACKING && e==EV_RIGHT) {
        // Start Assembly Stepper Motor moving right
        // Start Out-of-Range Timer
      }
      else if (currentState==STATE_TRACKING && e==EV_FOUND) {
        // Stop Assembly Stepper Motor
        // May need a "firing" state
        // "Fire" Laser
        // Move Back to Target State
      }
      else if (currentState==STATE_TRACKING && e==EV_LOST) {
        // Stop Assembly Stepper Motor
        // Move Back to Search State
      }
      xSemaphoreGive(xStateMutex);
    }
  }
}

void stateSensorTask(void *) {
  for (;;)
  {
      // Note: Need to change things to make it work well is stateMachineTask
      long left_distance = getDistanceCM(0);
      Serial.print("Distance: ");
      Serial.print(left_distance);
      Serial.println(" cm");

      long right_distance = getDistanceCM(1);
      Serial.print("Distance: ");
      Serial.print(right_distance);
      Serial.println(" cm");

      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      //turret_state_t old=currentState;

      if (left_distance >= 10 && left_distance <= 20){
        if (currentState==STATE_SEARCH) {
          pushEvent(EV_FOUND);
        }
        else {
          pushEvent(EV_RIGHT);
        }
      }
      else if (right_distance >= 10 && right_distance <= 20){
        pushEvent(EV_LEFT);
      }
      /*if(distance >= 10 && distance <= 20){
        // Move forward 180°
        for(int i=0; i<2048; i++){ // 180° (half rotation)
          stepForward();
          delayMicroseconds(stepDelay);
        }
        delay(500); // short pause
        // Move back 180°
        for(int i=0; i<2048; i++){
          stepBackward();
          delayMicroseconds(stepDelay);
        }
        delay(500); // short pause
      } else {
        // Idle if out of range
        delay(100);
      }*/
  }
}

void setup() {
  // put your setup code here, to run once:
  xTaskCreate(TaskPanning, // Task function
              "Panning", // Task name
              128, // Stack size 
              NULL, 
              0, // Priority
              &taskTesting); // Task handler

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  Serial.begin(9600);
}

void TaskPanning(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {
      long distance = getDistanceCM();
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      
      if(distance >= 10 && distance <= 20){
        // Move forward 180°
        for(int i=0; i<2048; i++){ // 180° (half rotation)
          stepForward();
          delayMicroseconds(stepDelay);
        }
        delay(500); // short pause
        // Move back 180°
        for(int i=0; i<2048; i++){
          stepBackward();
          delayMicroseconds(stepDelay);
        }
        delay(500); // short pause
      } else {
        // Idle if out of range
        delay(100);
      }
  }
}

// --- Ultrasonic distance ---
long getDistanceCM(int module) {
  if (module == 0) {
    digitalWrite(TRIG_0, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_0, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_0, LOW);
    
    long duration = pulseIn(ECHO_0, HIGH);
    long distanceCM = duration * 0.034 / 2; // cm
    return distanceCM;
  }
  else if (module == 1) {
    digitalWrite(TRIG_1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_1, LOW);
    
    long duration = pulseIn(ECHO_1, HIGH);
    long distanceCM = duration * 0.034 / 2; // cm
    return distanceCM;
  }
}

void loop() {}

