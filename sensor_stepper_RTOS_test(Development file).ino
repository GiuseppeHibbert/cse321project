#include <Arduino_FreeRTOS.h>
#include <Timers.h>
#include <Stepper.h>

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

// Ultrasonic 0 pins
#define TRIG_0 3
#define ECHO_0 4

// Ultrasonic 1 pins
#define TRIG_1 5
#define ECHO_1 6

// Laser Pointer Pin
#define LASER 50  // Arbitrary Digital Pin

const int stepsPerRevolution = 2048;

int debug = 0;

Stepper sensorStepper(stepsPerRevolution, 11, 10, 9, 8);
Stepper assemblyStepper(stepsPerRevolution, 7, 6, 5, 4);

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
               EV_RIGHT, EV_TIMEOUT, EV_ERROR} system_event_t;

// ----- RTOS objects -----
QueueHandle_t xEventQueue;
SemaphoreHandle_t xStateMutex;
TimerHandle_t xSearchTimer; // create search timer for timeout after 10s of detecting an object without moving to target state ( edge case handling )

// ----- Timer callbacks -----
void searchTimerCallback(TimerHandle_t){ system_event_t e=EV_TIMEOUT; xQueueSend(xEventQueue,&e,0); }

// ----- Sensor Stuff -----
int sensor_mode = 0;
int sensorStepPos = 0;
int assemblyStepPos = 0;

int laser = 0;

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
        sensorStepper.setSpeed(0);
        // Grab Position
        // Use sensorStepPos once setup
        // Move Position of Assembly Stepper Motor to Match Sensor Stepper Motor
        // Step assemblyStepper until StepPos of both match.
        // Move to Target State
        currentState = STATE_TARGET;
      }
      else if (currentState==STATE_TARGET) {
        // May need a "firing" state
        // "Fire" Laser i.e. Blink Laser
        laser = laser ^ 1; // Toggles Laser
        digitalWrite(LASER, laser);
      }
      /*else if ((currentState==STATE_TARGET && e==EV_LEFT) || (currentState==STATE_TARGET && e==EV_RIGHT)) {
        // Move to Tracking State
        // Set sensor_mode flag
      }*/
      // NEED TO ADD - Flag to stop both of these to fire simulataneously.
      // i.e. Which ever fires first will maintain its movement
      else if (currentState==STATE_TARGET && e==EV_LEFT) {
        // Start Assembly Stepper Motor moving Left
        sensorStepper.setSpeed(0);
        assemblyStepper.setSpeed(60); // 60 is arbitrary
        assemblyStepper.step(1);
        // Start Out-of-Range Timer
        if (!xTimerIsTimerActive(xSearchTimer)) {
          xTimerStart(xSearchTimer,0);
        }
      }
      else if (currentState==STATE_TARGET && e==EV_RIGHT) {
        // Start Assembly Stepper Motor moving right
        sensorStepper.setSpeed(0);
        assemblyStepper.setSpeed(60); // 60 is arbitrary
        assemblyStepper.step(-1);
        // Start Out-of-Range Timer
        if (!xTimerIsTimerActive(xSearchTimer)) {
          xTimerStart(xSearchTimer,0);
        }
      }
      /*else if (currentState==STATE_TRACKING && e==EV_FOUND) {
        // Stop Assembly Stepper Motor
      }*/
      else if (currentState==STATE_TRACKING && e==EV_TIMEOUT) {
        // Stop Assembly Stepper Motor
        assemblyStepper.setSpeed(0);
        // Move Back to Search State
        currentState = STATE_SEARCH;
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

      if (left_distance >= 10 && left_distance <= 20){
        if (currentState==STATE_SEARCH) {
          pushEvent(EV_FOUND);
        }
        else if (currentState==STATE_TRACKING) {
          pushEvent(EV_RIGHT);
        }
        else {
          // Do Nothing
        }
      }
      if (right_distance >= 10 && right_distance <= 20){
        if (currentState==STATE_SEARCH) {
          pushEvent(EV_FOUND);
        }
        else if (currentState==STATE_TRACKING) {
          pushEvent(EV_LEFT);
        }
        else {
          // Do Nothing
        }
      }
      // Both Lost
      if ((left_distance < 20) && (right_distance < 20)) {
        pushEvent(EV_LOST);
      }
      // Right Lost
      if ((right_distance < 20) && (right_distance >= 10 && right_distance <= 20)) {
        pushEvent(EV_LEFT);
      }
      // Left Lost
      if ((left_distance < 20) && (left_distance >= 10 && left_distance <= 20)) {
        pushEvent(EV_RIGHT);
      }
  }
}

void setup() {
  // put your setup code here, to run once:
  
  sensorStepper.setSpeed(60);
  assemblyStepper.setSpeed(60);

  Serial.begin(9600);

  pinMode(IN1_0, OUTPUT);
  pinMode(IN2_0, OUTPUT);
  pinMode(IN3_0, OUTPUT);
  pinMode(IN4_0, OUTPUT);

  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);

  pinMode(TRIG_0, OUTPUT);
  pinMode(ECHO_0, INPUT);
  pinMode(TRIG_1, OUTPUT);
  pinMode(ECHO_1, INPUT);

  pinMode(LASER, OUTPUT);

  xEventQueue=xQueueCreate(10,sizeof(system_event_t));
  xStateMutex=xSemaphoreCreateMutex();

  xSearchTimer = xTimerCreate("Search Timeout", pdMS_TO_TICKS(10000), pdFALSE, NULL, searchTimerCallback);

  xTaskCreate(stateMachineTask,"FSM",4096,NULL,2,NULL,1);
  xTaskCreate(stateSensorTask, "Sensors Task", 4096, NULL, 1, NULL, 1);

  Serial.println("=== Turret Project: Prototype ===");
  
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

void loop() {
  // Rotate CW slowly at 5 RPM
  if (debug) {
  digitalWrite(5,HIGH);
  myStepper.setSpeed(3);
  myStepper.step(stepsPerRevolution/16);
    Serial.print("Module 0 Sensed at ");
  Serial.print(getDistanceCM(0));
  Serial.println(" cm away!"); 
    Serial.print("Module 1 Sensed at ");
  Serial.print(getDistanceCM(1));
  Serial.println(" cm away!"); 
    digitalWrite(5,LOW);
  myStepper.step(-stepsPerRevolution/16);
  // Rotate CCW quickly at 10 RPM  
  }
}

