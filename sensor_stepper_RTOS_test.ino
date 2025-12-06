#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>

// Stepper pins Driver 0
#define IN1_0 11
#define IN2_0 10
#define IN3_0 9
#define IN4_0 8

// Left and right from the perspective of which the sensors fire.
// Ultrasonic 0 (left) pins
#define TRIG_0 2
#define ECHO_0 3

// Ultrasonic 1 (right) pins
#define TRIG_1 4
#define ECHO_1 6

// Laser Pointer Pin
#define LASER 50

typedef enum { STATE_OFF, STATE_IDLE, STATE_SEARCH, STATE_TARGET,
               STATE_TRACKING, STATE_ERROR } turret_state_t;
typedef enum { EV_POWER_ON, EV_FOUND, EV_LOST, EV_LEFT,
               EV_RIGHT, EV_RETURN, EV_TIMEOUT, EV_ERROR} system_event_t;

// Step Sequence full step 
const uint8_t stepSeq[4][4] = {
  {1,0,0,0},
  {0,1,0,0},
  {0,0,1,0},
  {0,0,0,1}
};

int stepIndex = 0;

// Helper function that allows for simple directional input.
void stepMotor(int dir) {
  stepIndex = (stepIndex + dir + 4) % 4;
  digitalWrite(IN1_0, stepSeq[stepIndex][0]);
  digitalWrite(IN2_0, stepSeq[stepIndex][1]);
  digitalWrite(IN3_0, stepSeq[stepIndex][2]);
  digitalWrite(IN4_0, stepSeq[stepIndex][3]);
}

void print_mapEnglish(int currentState){
  switch(currentState){
    case 0:
      Serial.println("STATE_OFF"); 
      break;
    case 1:
      Serial.println("STATE_IDLE"); 
      break;
    case 2:
      Serial.println("STATE_SEARCH"); 
      break;
    case 3:
      Serial.println("STATE_TARGET"); 
      break;
    case 4:
      Serial.println("STATE_TRACKING"); 
      break;
    case 5:
      Serial.println("STATE_ERROR"); 
      break;
    default:
      break;

  }
}

turret_state_t currentState = STATE_OFF;

// ----- RTOS objects -----
QueueHandle_t xEventQueue;
SemaphoreHandle_t xStateMutex;

// ----- Sensor Stuff -----
int leftCount = 0;
int rightCount = 0;
int laser = 0;

// Streamlined means of sending signals.
inline void pushEvent(system_event_t ev){ 
    xQueueSend(xEventQueue, &ev, 0); 
}

// Handles all state related functions
void stateMachineTask(void *){
  system_event_t e;
  for(;;){
    //Serial.println("FSM RUNNING");
    // Read in event
    if(xQueueReceive(xEventQueue,&e,portMAX_DELAY)==pdTRUE){
      //Serial.println("Received Event!");
      // Enter critical section
      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      // Found a new object
      if (currentState==STATE_SEARCH && e==EV_FOUND) {
        digitalWrite(LASER, LOW);
        // Move to Target State
        currentState = STATE_TARGET;
        //Serial.println("SEARCH + FOUND");
      }
      // Lost the current object
      else if (currentState==STATE_TARGET && e==EV_LOST){
        digitalWrite(LASER, LOW);
        // Move back to Search State
        currentState = STATE_SEARCH;
      }
      // Object moved left
      else if (currentState==STATE_TARGET && e==EV_LEFT && e!=EV_RIGHT) {
        digitalWrite(LASER, HIGH);
        //Serial.println("About to move counteclockwise");
        // No longer in critical section
        xSemaphoreGive(xStateMutex);
        for (int i=0; i<120; i++) {
          stepMotor(-1);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        // Enter the next critical section
        xSemaphoreTake(xStateMutex,portMAX_DELAY);
        //Serial.println("JUST MOVED left");
        //Serial.println("TARGET + LEFT");
        currentState = STATE_TRACKING;
      }
      // Object moved right
      else if (currentState==STATE_TARGET && e==EV_RIGHT && e!=EV_LEFT) {
        digitalWrite(LASER, HIGH);
        //Serial.println("About to move clockwise");
        // No longer in critical section
        xSemaphoreGive(xStateMutex);
        for (int i=0; i<120; i++) {
          //Serial.println("moving right");
          stepMotor(+1);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        // Enter the next critical section
        xSemaphoreTake(xStateMutex,portMAX_DELAY);
        //Serial.println("JUST MOVED right");
        //Serial.println("TARGET + RIGHT");
        currentState = STATE_TRACKING;
      }  
      // Signaled Lost Case 1
      else if (currentState==STATE_TARGET && e!=EV_LEFT && e!=EV_RIGHT){
        digitalWrite(LASER, LOW);
        // Move back to Search State
        currentState = STATE_SEARCH;
      }
      // Signaled Re-found object
      else if (currentState==STATE_TRACKING && e==EV_RETURN) {
        digitalWrite(LASER, HIGH);
        //Serial.println("TRACKING + RETURN");
        // Return to Target State
        currentState = STATE_TARGET;
      }
      // Signaled Lost Case 2
      else if(currentState==STATE_TRACKING && e==EV_LOST){
        digitalWrite(LASER, LOW);
        // Move back to Search State
        currentState = STATE_SEARCH;
        //Serial.println("TRACKING + LOST");
      }
      // Signaled Lost Case 3
      else if (currentState==STATE_TRACKING && e!=EV_LEFT && e!=EV_RIGHT){
        digitalWrite(LASER, LOW);
        // Move back to Search State
        currentState = STATE_SEARCH;
      }
      // DEBUG for state monitoring
      print_mapEnglish(currentState);      
      xSemaphoreGive(xStateMutex);
    }
  }
}

void stateSensorTask(void *) {
  for (;;)
  {
      //Serial.println("SENSOR RUNNING");
      long left_distance = getDistanceCM(0);
      //Serial.print("Left Distance: ");
      //Serial.print(left_distance);
      //Serial.println(" cm");
      long right_distance = getDistanceCM(1);
      //Serial.print("Right Distance: ");
      //Serial.print(right_distance);
      //Serial.println(" cm");
      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      //Serial.println("TOOK SEMAPHORE");
      // Sensors detected an object
      if( (left_distance >= 5 && left_distance <= 20) && (right_distance >= 5 && right_distance <= 20)) {
        if (left_distance >= 5 && left_distance <= 20){
          // Found a new object
          if (currentState==STATE_SEARCH) {
            //Serial.println("LEFT FOUND ");
            pushEvent(EV_FOUND);
            //Serial.println("AFTER PUSH EVENT ");
          }
          // Re-found a current object
          else if(currentState == STATE_TRACKING){
            pushEvent(EV_RETURN);
          }
        }
        // Both left and right conditionals could be condensed :/
        // Found a new object
        else if (right_distance >= 5 && right_distance <= 20){
          // Found a new object
          if (currentState==STATE_SEARCH) {
            pushEvent(EV_FOUND);
          }
          // Re-found a current object
          else if(currentState == STATE_TRACKING){
            pushEvent(EV_RETURN);
          }
        }
      }


      // Both Lost
      if( left_distance < 400 && right_distance < 400 && (currentState != STATE_SEARCH)){
        if (((left_distance > 20) && (right_distance > 20)) ) {
          //Serial.println("EVENT LOST ");
          pushEvent(EV_LOST);
        }
        // Right Lost
        else if ((right_distance > 20) && (left_distance >= 5 && left_distance <= 20) && currentState == STATE_TARGET) {
          //Serial.println("EVENT RIGHT LOST ");
          leftCount++;
          pushEvent(EV_LEFT);
        }
        // Left Lost
        else if ((left_distance > 20) && (right_distance >= 5 && right_distance <= 20) && currentState == STATE_TARGET) {
          //Serial.println("EVENT LEFT LOST ");
          rightCount++;
          pushEvent(EV_RIGHT);
        }
      }
      xSemaphoreGive(xStateMutex);

      //Serial.print("left count:");
      //Serial.println(leftCount);
      //Serial.print("right count:");
      //Serial.println(rightCount);
  }
}

void setup() {
  currentState = STATE_SEARCH;

  Serial.begin(9600);

  pinMode(IN1_0, OUTPUT);
  pinMode(IN2_0, OUTPUT);
  pinMode(IN3_0, OUTPUT);
  pinMode(IN4_0, OUTPUT);

  pinMode(TRIG_0, OUTPUT);
  pinMode(ECHO_0, INPUT);
  pinMode(TRIG_1, OUTPUT);
  pinMode(ECHO_1, INPUT);

  pinMode(LASER, OUTPUT);

  xEventQueue=xQueueCreate(50,sizeof(system_event_t));
  xStateMutex=xSemaphoreCreateMutex();

  xTaskCreate(stateMachineTask,"FSM",256,NULL,1,NULL);
  xTaskCreate(stateSensorTask, "Sensors Task", 256, NULL, 1, NULL);

  Serial.println("=== Turret Project: Final Version 1 ===");
  
}

// Helper function used to streamline parsing the sensor data reads.
// --- Ultrasonic distance ---
long getDistanceCM(int module) {
  if (module == 0) {
    // converted from TaskDelayMicroseconds -> vTaskDelay non blocking
    digitalWrite(TRIG_0, LOW);
    vTaskDelay(pdMS_TO_TICKS(2));
    digitalWrite(TRIG_0, HIGH);
    vTaskDelay(pdMS_TO_TICKS(2));
    digitalWrite(TRIG_0, LOW);
    
    long duration = pulseIn(ECHO_0, HIGH, 10000);
    long distanceCM = duration * 0.034 / 2; // cm
    return distanceCM;
  }
  else if (module == 1) {
    digitalWrite(TRIG_1, LOW);
    vTaskDelay(pdMS_TO_TICKS(2));
    digitalWrite(TRIG_1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(TRIG_1, LOW);
    
    long duration = pulseIn(ECHO_1, HIGH, 10000);
    long distanceCM = duration * 0.034 / 2; // cm
    return distanceCM;
  }
}

void loop() {}
