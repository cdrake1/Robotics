//This program allows for the Pololu3piPlus to travel set distances, by pressing the a, b, and c buttons
//By Ryan Rosenkranse
//Version: Lab2A

#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
Servo headServo;

// switches
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = false;

const boolean SERVO_ON = true;
const boolean US_ON = true;

// Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 120;

// Head servo constants
const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 7;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 120, 105, 90, 75, 60, 45};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Ultra Sonic Sensor constants
const int ECHO_PIN = 12;
const int TRIG_PIN = 18;

const float MAX_DISTANCE = 300.0;

const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5;

// Motor constants
const float MOTOR_BASE_SPEED = 300.0;
const int MOTOR_MIN_SPEED = 30;
// determine the normalization factor based on MOTOR_BASE_SPEED
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
const unsigned long WAIT_AFTER_HEAD_STARTS_MOVING = 80;
boolean usReadFlag = false;

// current US distance reading
int currentReadPosition = 0;

float distanceReadings[NUM_HEAD_POSITIONS];

// Motor Timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20;

float distance = 0;

/*
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 15;
const unsigned long US_PERIOD = 500;
*/

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



void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(57600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);

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

  moveHead();

  setMotors();

  
  /*
  if(buttonA.getSingleDebouncedRelease()){ //goes forward 1ft
    dist = 30.5;
  }
  if(buttonB.getSingleDebouncedRelease()) //goes back 1ft
    dist = -30.5;

  if(buttonC.getSingleDebouncedRelease()) //goes forward 1.5ft
    dist = 45.5;
  
  checkEncoders(dist);
  */
}

void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD){

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
    if (distance == 0) distance = MAX_DISTANCE;

    //assign the value of the current position of the array
    distanceReadings[currentReadPosition] = distance;
    }

    if(TIMING_DEBUG){
      Serial.print("US read Finished: ");
      Serial.println(millis());
    }

    // Displays the distance on the Serial Monitor
    if(US_DEBUG) {
      Serial.print("Distnace Readings: [ ");
      for(int i = 0; i < NUM_HEAD_POSITIONS; i++){
        Serial.print(distanceReadings[i]);
        if(i < (NUM_HEAD_POSITIONS - 1)) Serial.print(" - ");

      }
      Serial.println(" ]");
    }

    usReadFlag = true;
    // update the prevmillis
    //usPm = usCm;
  }
}

void setMotors() {
  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {
  
  // start out with the MOTOR_BASE_SPEED
  float leftSpeed = MOTOR_BASE_SPEED;
  float rightSpeed = MOTOR_BASE_SPEED;

  // check to see if most current distance measurement is less than / euqal to MAX_DISTANCE
  if(distance <= MAX_DISTANCE) {
    // determine the magnitude of the distance by taking the difference
    // divide by the DISTANCE_FACTOR to ensure uniform response as MAS_DISTANCE changes
    // This maps the distance range (1 - MAX_RANGE) to 0-100 for the magnitude
    float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
    
    leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
    rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);

    if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

    // add in motor compensation
    if(leftSpeed <= L_MOTOR_FACTOR_THRESHOLD) {
        leftSpeed *= L_MOTOR_FACTOR;
    }
    if(rightSpeed <= R_MOTOR_FACTOR_THRESHOLD) {
      rightSpeed *= R_MOTOR_FACTOR;
    }

    if(distance <= STOP_DISTANCE){
      leftSpeed = 0;    
      rightSpeed = 0;
    }
    Serial.print("Left: ");
    Serial.println(leftSpeed);
    Serial.print("Right: ");
    Serial.println(rightSpeed);

    motors.setSpeeds(-leftSpeed, -rightSpeed);

    motorPm = motorCm;
  }
  }
}

/*
void checkEncoders(float distToTravel) {

  //tests to see if distance that needs to be travelled is negative, is so speed will be set to negative to move in reverse and distance will be made positive  
  int direction = 1;

  if(distToTravel < 0){
    direction = -1;
    distToTravel *= -1;
  }
  
  //Retrieves the counts of the left and right encoders, when current time exceeds the previous time plus the period constant
  currentMillis = millis();
  if(currentMillis > prevMillis + PERIOD){
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //Sl and Sr are incremented by the distance traveled in centimeters
    Sl += ((countsLeft -prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight -prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    int wheelSpeed = 55 * direction;


    if(fabs(Sl) < distToTravel) {
      if(fabs(Sl) > (distToTravel - 10)) { //When approaching target, slow the robot down
        wheelSpeed = (100 * ((distToTravel-fabs(Sl)) / 10)) * direction;
        if(abs(wheelSpeed) < 20) wheelSpeed = 20 * direction; //but dont slow down too much
      }
      
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    }else{
      //Stops motors and resets variables so the robot can travel again without having to be reset
      motors.setSpeeds(0, 0);
      if(Sl>0) buzzer.play("c32");
      dist = 0;
      Sl = 0;
      Sr = 0;
    }

  
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print("Right: ");
    Serial.println(Sr);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }
}
*/



