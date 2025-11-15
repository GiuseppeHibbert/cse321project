//Includes the Arduino Stepper Library
#include <Stepper.h>

// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;



// Stepper pins Driver 0
#define IN1_0 11
#define IN2_0 10
#define IN3_0 9
#define IN4_0 8

// Ultrasonic pins
#define TRIG_0 2
#define ECHO_0 3

// Ultrasonic pins
#define TRIG_1 4
#define ECHO_1 6


// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 11, 9, 10, 8);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  digitalWrite(5,HIGH);
  myStepper.setSpeed(1);


  pinMode(TRIG_0, OUTPUT);
  pinMode(ECHO_0, INPUT);
  pinMode(TRIG_1, OUTPUT);
  pinMode(ECHO_1, INPUT);


}



void loop() {
  // Rotate CW slowly at 5 RPM
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
