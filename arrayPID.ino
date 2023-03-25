#include <Servo.h>

//This program allows for the Pololu3piPlus to follow
//an object using an Ultrasonic Sensor and turn its head
//using a servo.
//By Ryan Rosenkranse


#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

// switches
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = true;
const boolean MOTOR_DEBUG = false;
const boolean ERROR_DEBUG = false;
const boolean PROPORTIONAL_DEBUG = false;
const boolean INTEGRAL_DEBUG = false;
const boolean DERIVATIVE_DEBUG = false;
const boolean PID_DEBUG = false;

const boolean SERVO_ON = true;
const boolean US_ON = true;

// Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 250;

// Head servo constants
const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {180, 180, 90};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Ultra Sonic Sensor constants
const int ECHO_PIN = 12;
const int TRIG_PIN = 18;

const float MAX_DISTANCE = 30.0;

const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5;

// Motor constants
float MOTOR_BASE_SPEED = -80.0;
const int MOTOR_MIN_SPEED = 30;
// determine the normalization factor based on motorBaseSpeed
const float MOTOR_FACTOR =  MOTOR_BASE_SPEED / 100;

// motor compensation
const float L_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR = 1.0;
const float L_MOTOR_FACTOR_THRESHOLD = 80;
const float R_MOTOR_FACTOR_THRESHOLD = 80;

//ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50;
const unsigned long WAIT_AFTER_HEAD_STARTS_MOVING = 200;
boolean usReadFlag = false;

// current US distance reading
int currentReadPosition = 0;

float distanceReadings[NUM_HEAD_POSITIONS];

// Motor Timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 200;

float distance = 0;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

float Sl = 0.0F;
float Sr = 0.0F;

//DIAMETER AND CIRCUMFERENCE ARE IN CENTIMETERS
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 74.55F; //Slightly tweaked for accuracy
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;
float dist = 0;

const double desiredDistance = (double) 10.0;
double shortestDistance = distanceReadings[0];

const double kp[] = {6.0,6.0,3.0};
const double ki[] = {0.0,0.0,0.0};
const double kd[] = {0.0,0.0,0.0};

double kiTotal[] = {0,0,0};
double priorError[] = {0,0,0};

unsigned long pidCm;
unsigned long pidPm;
const unsigned long PID_PERIOD = 250;

float error[NUM_HEAD_POSITIONS];
float proportional[NUM_HEAD_POSITIONS];
float integral[NUM_HEAD_POSITIONS];
float derivative[NUM_HEAD_POSITIONS];
float pidResult[NUM_HEAD_POSITIONS];

boolean pidFlag = false;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(57600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(145);

  for(int i=0; i<NUM_HEAD_POSITIONS; i++){
    distanceReadings[i] = MAX_DISTANCE;
  }

  // start delay
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:

  usReadCm();
    pidCm = millis();
    if(pidCm > pidPm + PID_PERIOD){
    
    error[currentReadPosition] = desiredDistance - distanceReadings[currentReadPosition];
    
    if(ERROR_DEBUG){
      Serial.print("The error is :");
      Serial.println(error[currentReadPosition]);
    }

    proportional[currentReadPosition] = kp[currentReadPosition] * error[currentReadPosition];

    if(PROPORTIONAL_DEBUG){
      Serial.println(proportional[currentReadPosition]);
    }


    kiTotal[currentReadPosition] += error[currentReadPosition];

    integral[currentReadPosition] = ki[currentReadPosition] * kiTotal[currentReadPosition];

    if(INTEGRAL_DEBUG){
      Serial.println(integral[currentReadPosition]);
    }

    derivative[currentReadPosition] = kd[currentReadPosition] * (error[currentReadPosition] - priorError[currentReadPosition]);

    priorError[currentReadPosition] = error[currentReadPosition];

    if(DERIVATIVE_DEBUG){
      Serial.println(derivative[currentReadPosition]);
    }

    pidResult[currentReadPosition] = proportional[currentReadPosition] + integral[currentReadPosition] + derivative[currentReadPosition];

    if(PID_DEBUG){
      Serial.println(pidResult[currentReadPosition]);
    }
    
    setMotors(pidResult[currentReadPosition]);
    
    //setMotors(-MOTOR_BASE_SPEED, -MOTOR_BASE_SPEED);

    pidPm = pidCm;
    }


    moveHead();

}

void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD){

    currentReadPosition = currentHeadPosition;

    // head debug output
    if(HEAD_DEBUG) {
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.println(HEAD_POSITIONS[currentHeadPosition]);
    }

    // position head to the current position in the array
    headServo.write( HEAD_POSITIONS[currentHeadPosition]);

    // Set next head position
    // Moves servo to the next head position and changes direction when needed
    if(headDirectionClockwise){
      if(currentHeadPosition >= (NUM_HEAD_POSITIONS -1)){
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else{
        currentHeadPosition++;
      }    
    }else{
      if(currentHeadPosition <=0){
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }else {
        currentHeadPosition--;
      }
    }

    // reset previous millis
    headPm = headCm;
    // reset read flag
    usReadFlag =false;
  }
}

 void usReadCm() {
  usCm = millis();
  if(usCm > headPm + WAIT_AFTER_HEAD_STARTS_MOVING && !usReadFlag) {

    //timing debug
    if(TIMING_DEBUG){
      Serial.print("US read inititated: ");
      Serial.println(usCm);
    }

    //clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    if(US_ON){
    // Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    // note the duration (38000 microseconds) that will allow for reading up max distance supported by the sensor
    float duration = pulseIn(ECHO_PIN, HIGH, 30000);

    //caluclating the distance
    distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound
    distance = floor(10 * distance + 0.5f) / 10;

    //apply limits

    if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = 1;

    //assign the value of the current position of the array
    
    distanceReadings[currentReadPosition] = distance;
    
    }

    if(TIMING_DEBUG){
      Serial.print("US read Finished: ");
      Serial.println(millis());
    }

    // Displays the distance on the Serial Monitor
    if(US_DEBUG) {
      Serial.print("Distance Readings: [ ");
      for(int i = 0; i < NUM_HEAD_POSITIONS; i++){
        Serial.print(distanceReadings[i]);
        if(i < (NUM_HEAD_POSITIONS - 1)) Serial.print(" - ");

      }
      Serial.println(" ]");
    }

    usReadFlag = true;
    pidFlag = true;
    // update the prevmillis
    //usPm = usCm;
  }
}

void setMotors(float pidResult) {

  // start out with the MOTOR_BASE_SPEED
  float leftSpeed = MOTOR_BASE_SPEED;
  float rightSpeed = MOTOR_BASE_SPEED;

    if(MOTOR_DEBUG){
    Serial.print("Left: ");
    Serial.print((pidResult+MOTOR_BASE_SPEED));
    Serial.print(" Right: ");
    Serial.println((-pidResult+MOTOR_BASE_SPEED));
    }

    if(HEAD_POSITIONS[currentReadPosition] > 90)
      motors.setSpeeds((pidResult+leftSpeed), (-pidResult+rightSpeed));
   else
    motors.setSpeeds((-pidResult+leftSpeed), (pidResult+rightSpeed));
}






