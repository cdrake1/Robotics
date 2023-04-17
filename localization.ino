//This program implements a full localization controller
//for a pololu 3pi+ in 2D Cartesian space.
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
const boolean US_DEBUG = false;
const boolean MOTOR_DEBUG = false;
const boolean ERROR_DEBUG = true;
const boolean PROPORTIONAL_DEBUG = false;
const boolean INTEGRAL_DEBUG = false;
const boolean DERIVATIVE_DEBUG = false;
const boolean PID_DEBUG = false;
const boolean XY_DEBUG = true;
const boolean THETA_DEBUG = true;

const boolean SERVO_ON = true;
const boolean US_ON = true;
const boolean MOTORS_ON = true;

// Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 250;

// Head servo constants
const int HEAD_SERVO_PIN = 11;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {180, 180, 90};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Ultra Sonic Sensor constants
const int ECHO_PIN = 20;
const int TRIG_PIN = 21;

const float MAX_DISTANCE = 40.0;

const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5;

// Motor constants
float MOTOR_BASE_SPEED = -50.0;
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
const unsigned long MOTOR_PERIOD = 8;

float distance = 0;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const double desiredDistance[] =  {20.0,20.0,40.0};
double shortestDistance = distanceReadings[0];

const double kp = 20;
const double ki = 0;
const double kd = 0;

double kiTotal = 0;
double priorError= 0;

unsigned long pidCm;
unsigned long pidPm;
const unsigned long PID_PERIOD = 8;

boolean pidFlag = false;

// goals
const int NUMBER_OF_GOALS = 4;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0, 0};
float yGoals[NUMBER_OF_GOALS] = {0, 30, 30, 0};
int currentGoal = 0;

float error;
float proportional;
float integral;
float derivative;
float pidResult;

float currentX = 0.0F;
float currentY = 0.0F;
float currentTheta = 0.0;
float desiredTheta = 0.0;
float Sl = 0.0F;
float Sr = 0.0F;
float previousSl = 0.0F;
float previousSr = 0.0F;
float deltaS = 0.0;
float deltaX = 0.0;
float deltaY = 0.0;
float deltaTheta = 0.0;
float xError;
float yError;
float distanceFromGoal;
float prevDistanceFromGoal;

//DIAMETER AND CIRCUMFERENCE ARE IN CENTIMETERS
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 74.55F; //Slightly tweaked for accuracy
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;
float dist = 0;

unsigned long CECm;
unsigned long CEPm;
const unsigned long CE_PERIOD = 8;

int i = 1;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(57600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  for(int i=0; i<NUM_HEAD_POSITIONS; i++){
    distanceReadings[i] = MAX_DISTANCE;
  }

  // start delay
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  checkEncoders();

  pidCm = millis();
  if(pidCm > pidPm + PID_PERIOD){

    //calculate the location of the center of the robot
    deltaS = ((Sr-previousSr) + (Sl-previousSl))/2;

    //calculate the current location of X and Y     
    deltaTheta = ((Sr-previousSr) - (Sl-previousSl))/8.5;

    deltaX = deltaS * cos(currentTheta + deltaTheta/2);
    deltaY = deltaS * sin(currentTheta + deltaTheta/2);

    currentX += deltaX;
    currentY += deltaY;

    if(XY_DEBUG){
      Serial.print("X: ");
      Serial.print(currentX);
      Serial.print(" Y: ");
      Serial.print(currentY);
      Serial.print(" current Theta: ");
      Serial.print(currentTheta);
      Serial.print(" delta Theta: ");
      Serial.print(deltaTheta);
    }

    // calculate the correct orientation to the goal
    desiredTheta = atan2((yGoals[currentGoal]-currentY),(xGoals[currentGoal]-currentX));

    // calculate the distance from the goal
    distanceFromGoal = sqrt(pow((xGoals[currentGoal]-currentX),2)+pow((currentY-yGoals[currentGoal]),2));

    if(THETA_DEBUG){
      Serial.print(" Desired Theta: ");
      Serial.print(desiredTheta);
      Serial.print(" DistanceFromGoal: ");
      Serial.print(distanceFromGoal);
    }

    error = desiredTheta - currentTheta;

    // checks to see if the other direction is shorter
    if((2*PI)+error < abs(error))
      error = (2*PI)+error;
    
    // Used for Error Debugging
    if(ERROR_DEBUG){
      Serial.print(" The error is :");
      Serial.println(error);
    }

    // calculate the proprotional value
    proportional = kp * error;

    // Used for Proportional Debugging
    if(PROPORTIONAL_DEBUG){
      Serial.println(proportional);
    }

    // Calculate the Integral value
    kiTotal += error;

    if(kiTotal > 50)
      kiTotal/=2;

    integral = ki * kiTotal;

    // Used for Integral Debugging
    if(INTEGRAL_DEBUG){
      Serial.println(integral);
    }

    // calculate the Derivative value
    derivative = kd * (error - priorError);

    // sets the prior error
    priorError = error;

    // Used for Derivative Debugging
    if(DERIVATIVE_DEBUG){
      Serial.println(derivative);
    }

    // Calcuate the Pid Result by taking the sum of the Proportional, Integral and Derivative values
    pidResult = proportional + integral + derivative;

    if(distanceFromGoal < 2.0){
      currentGoal++;
      buzzer.play("c32");
    }
    
    if(currentGoal == NUMBER_OF_GOALS)
    {
      motors.setSpeeds(0, 0);
      if(i == 1)
      {
        i++;
        buzzer.play("ER16ER16E2R8");

      }
      
    }
    else
    {  
    //Set the motors given the result
      setMotors(pidResult);
    }

    // update the current theta
    if((currentTheta + deltaTheta) > PI){ 
      currentTheta = - PI;
      currentTheta += deltaTheta;
    }else if((currentTheta + deltaTheta) < -PI){
      currentTheta = PI;
      currentTheta += deltaTheta;
    }else{
      currentTheta += deltaTheta;
    }
    
    previousSl = Sl;
    previousSr = Sr;
    pidPm = pidCm;
  }  

}


void checkEncoders() {

  //Retrieves the counts of the left and right encoders, when current time exceeds the previous time plus the period constant
  CECm = millis();
  if(CECm > CEPm + CE_PERIOD){
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //Sl and Sr are incremented by the distance traveled in centimeters
    Sr -= ((countsLeft -prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sl -= ((countsRight -prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    if(MOTOR_DEBUG){
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print(" Right: ");
    Serial.println(Sr);
    }

    prevLeft = countsLeft;
    prevRight = countsRight;
    CEPm = CECm;
  }
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

  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {

  // start out with the MOTOR_BASE_SPEED
  float leftSpeed = MOTOR_BASE_SPEED;
  float rightSpeed = MOTOR_BASE_SPEED;


  motors.setSpeeds((leftSpeed-pidResult), (rightSpeed+pidResult));

  motorPm = motorCm;
  }
}








