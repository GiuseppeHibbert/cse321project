#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
//#include <Timers.h>
//#include <Stepper.h>

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
#define TRIG_0 2
#define ECHO_0 3

// Ultrasonic 1 pins
#define TRIG_1 4
#define ECHO_1 6

// Laser Pointer Pin
#define LASER 50  // Arbitrary Digital Pin

typedef enum { STATE_OFF, STATE_IDLE, STATE_SEARCH, STATE_TARGET,
               STATE_TRACKING, STATE_ERROR } turret_state_t;
typedef enum { EV_POWER_ON, EV_FOUND, EV_LOST, EV_LEFT,
               EV_RIGHT, EV_RETURN, EV_TIMEOUT, EV_ERROR} system_event_t;



int debug = 0;

// Stepper sensorStepper(stepsPerRevolution, 11, 10, 9, 8);
// Stepper assemblyStepper(stepsPerRevolution, 7, 6, 5, 4);

// Step Sequence full step 
const uint8_t stepSeq[4][4] = {
  {1,0,0,0},
  {0,1,0,0},
  {0,0,1,0},
  {0,0,0,1}
};

int stepIndex = 0;

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
// TimerHandle_t xSearchTimer; // create search timer for timeout after 10s of detecting an object without moving to target state ( edge case handling )

// ----- Timer callbacks -----
// void searchTimerCallback(TimerHandle_t){ system_event_t e=EV_TIMEOUT; xQueueSend(xEventQueue,&e,0); }

// ----- Sensor Stuff -----
int sensor_mode = 0;
int sensorStepPos = 0;
int assemblyStepPos = 0;
int leftCount = 0;
int rightCount = 0;

int laser = 0;

inline void pushEvent(system_event_t ev){ 
    xQueueSend(xEventQueue, &ev, 0); 
}

void stateMachineTask(void *){
  system_event_t e;
  for(;;){
    //Serial.println("FSM RUNNING");
    if(xQueueReceive(xEventQueue,&e,portMAX_DELAY)==pdTRUE){
      //Serial.println("Received Event!");
      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      //turret_state_t old=currentState;
      // Starting
      // Note: May need to do something in setup to "kick-start" the program.

      // Once the Slip Rings allow 360degree stepping, Implement infinite spinning on the state search. 
          // Flicker the light (Fire Mode)

      if (currentState==STATE_SEARCH && e==EV_FOUND) {

        digitalWrite(LASER, LOW);
        // Stop Sensor Stepper Motor
        // sensorStepper.setSpeed(0); // !!!!!!!!!!!!!!!!!!!! REPLACE WITH CORRECT
        // Grab Position
        // Use sensorStepPos once setup
        // Move Position of Assembly Stepper Motor to Match Sensor Stepper Motor
        // Step assemblyStepper until StepPos of both match.
        // Move to Target State
        currentState = STATE_TARGET;
        //Serial.println("SEARCH + FOUND");
      }
      /*else if ((currentState==STATE_TARGET && e==EV_LEFT) || (currentState==STATE_TARGET && e==EV_RIGHT)) {
        // Move to Tracking State
        // Set sensor_mode flag
      }*/
      // NEED TO ADD - Flag to stop both of these to fire simulataneously.
      // i.e. Which ever fires first will maintain its movement


      else if (currentState==STATE_TARGET && e==EV_LOST){
        digitalWrite(LASER, LOW);
        currentState = STATE_SEARCH;
      }

      else if (currentState==STATE_TARGET && e==EV_LEFT && e!=EV_RIGHT) {
        //laser = laser ^ 1; // Toggles Laser
        digitalWrite(LASER, HIGH);
        //Serial.println("About to move counteclockwise");
        xSemaphoreGive(xStateMutex);
        for (int i=0; i<120; i++) {
          
          stepMotor(-1);
          //delay(2);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        xSemaphoreTake(xStateMutex,portMAX_DELAY);
        //Serial.println("JUST MOVED left");
        //Serial.println("TARGET + LEFT");
        currentState = STATE_TRACKING;
        // Start Assembly Stepper Motor moving Left
        //sensorStepper.setSpeed(0); // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.setSpeed(60); // 60 is arbitrary // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.step(1); // NEEDS REPLACEMENT !!!!!!!!!!!!
        /* Start Out-of-Range Timer
        if (!xTimerIsTimerActive(xSearchTimer)) {
          xTimerStart(xSearchTimer,0);
        } */
      }
      else if (currentState==STATE_TARGET && e==EV_RIGHT && e!=EV_LEFT) {
        //laser = laser ^ 1; // Toggles Laser
        digitalWrite(LASER, HIGH);
        //Serial.println("About to move clockwise");
        xSemaphoreGive(xStateMutex);
        for (int i=0; i<120; i++) {
          //Serial.println("moving right");
          stepMotor(+1);
          //delay(2);
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        xSemaphoreTake(xStateMutex,portMAX_DELAY);
                //Serial.println("JUST MOVED right");
        //Serial.println("TARGET + RIGHT");
        currentState = STATE_TRACKING;
        // Start Assembly Stepper Motor moving Left
        //sensorStepper.setSpeed(0); // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.setSpeed(60); // 60 is arbitrary // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.step(1); // NEEDS REPLACEMENT !!!!!!!!!!!!
         //Start Out-of-Range Timer
        /*if (!xTimerIsTimerActive(xSearchTimer)) {
          xTimerStart(xSearchTimer,0);
        } */
      }
            
      else if (currentState==STATE_TARGET && e!=EV_LEFT && e!=EV_RIGHT){
        digitalWrite(LASER, LOW);
        currentState = STATE_SEARCH;
      }
      

              // new addition below to fix firing state
      else if (currentState==STATE_TRACKING && e==EV_RETURN) {
        //laser = laser ^ 1; // Toggles Laser
        digitalWrite(LASER, HIGH);
        //Serial.println("TRACKING + RETURN");
        currentState = STATE_TARGET;
        // Start Assembly Stepper Motor moving right
        //sensorStepper.setSpeed(0); // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.setSpeed(60); // 60 is arbitrary // NEEDS REPLACEMENT !!!!!!!!!!!!
        //assemblyStepper.step(-1); // NEEDS REPLACEMENT !!!!!!!!!!!!
        // Start Out-of-Range Timer
/*        if (!xTimerIsTimerActive(xSearchTimer)) {
          xTimerStart(xSearchTimer,0);
        } */
      }
      /*
      else if (currentState==STATE_TARGET) {
        // May need a "firing" state
        // "Fire" Laser i.e. Blink Laser
        laser = laser ^ 1; // Toggles Laser
        digitalWrite(LASER, laser);
      } */

      /*else if (currentState==STATE_TRACKING && e==EV_FOUND) {
        // Stop Assembly Stepper Motor
      }*/
      else if(currentState==STATE_TRACKING && e==EV_LOST){
        digitalWrite(LASER, LOW);
        currentState = STATE_SEARCH;
        //Serial.println("TRACKING + LOST");
      }

      else if (currentState==STATE_TRACKING && e==EV_TIMEOUT) {
        // Stop Assembly Stepper Motor
        //assemblyStepper.setSpeed(0); // NEEDS REPLACEMENT !!!!!!!!!!!!
        // Move Back to Search State
        digitalWrite(LASER, LOW);
        currentState = STATE_SEARCH;
        //Serial.println("TRACKING + TIMEOUT");
      }
      else if (currentState==STATE_TRACKING && e!=EV_LEFT && e!=EV_RIGHT){
        digitalWrite(LASER, LOW);
        currentState = STATE_SEARCH;
      }
      print_mapEnglish(currentState);      
      xSemaphoreGive(xStateMutex);

    }
  }
}

void stateSensorTask(void *) {
  for (;;)
  {



      //Serial.println("SENSOR RUNNING");
      // Note: Need to change things to make it work well is stateMachineTask
      long left_distance = getDistanceCM(0);
      //Serial.print("Left Distance: ");
      //Serial.print(left_distance);
      //Serial.println(" cm");

      //vTaskDelay(pdMS_TO_TICKS(50));

      long right_distance = getDistanceCM(1);
      //Serial.print("Right Distance: ");
      //Serial.print(right_distance);
      //Serial.println(" cm");

      //vTaskDelay(pdMS_TO_TICKS(50));


      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      //Serial.println("TOOK SEMAPHORE");
      if( (left_distance >= 5 && left_distance <= 20) && (right_distance >= 5 && right_distance <= 20)) {
        if (left_distance >= 5 && left_distance <= 20){
          if (currentState==STATE_SEARCH) {
            //Serial.println("LEFT FOUND ");
            pushEvent(EV_FOUND);
            //Serial.println("AFTER PUSH EVENT ");
          }
          else if(currentState == STATE_TRACKING){
            pushEvent(EV_RETURN);
          }
        }
        else if (right_distance >= 5 && right_distance <= 20){
          if (currentState==STATE_SEARCH) {
            pushEvent(EV_FOUND);
          }
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
  // put your setup code here, to run once:
  
  //sensorStepper.setSpeed(60);
  //assemblyStepper.setSpeed(60);
  currentState = STATE_SEARCH;

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

  xEventQueue=xQueueCreate(50,sizeof(system_event_t));
  xStateMutex=xSemaphoreCreateMutex();

  //xSearchTimer = xTimerCreate("Search Timeout", pdMS_TO_TICKS(10000), pdFALSE, NULL, searchTimerCallback);

  xTaskCreate(stateMachineTask,"FSM",256,NULL,1,NULL);
  xTaskCreate(stateSensorTask, "Sensors Task", 256, NULL, 1, NULL);

  //vTaskStartScheduler();


  Serial.println("=== Turret Project: Prototype ===");
  
}

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

void loop() {
  // Rotate CW slowly at 5 RPM
  /*
  if (debug) {
  digitalWrite(5,HIGH);
  //myStepper.setSpeed(3);
  //myStepper.step(stepsPerRevolution/16);
    Serial.print("Module 0 Sensed at ");
  Serial.print(getDistanceCM(0));
  Serial.println(" cm away!"); 
    Serial.print("Module 1 Sensed at ");
  Serial.print(getDistanceCM(1));
  Serial.println(" cm away!"); 
    digitalWrite(5,LOW);
  //myStepper.step(-stepsPerRevolution/16);
  // Rotate CCW quickly at 10 RPM  
  
  }
  */
}
