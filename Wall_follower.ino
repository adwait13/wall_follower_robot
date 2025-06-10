/*
Project: Wall-following robot using PID control
Author: Adwait Thosare ME23B123, Group 10
Controller: PID
*/


// Motor Driver Pins (L298N)
const int enA = 3;      // PWM pin for right motor
const int in1 = 4;      // Right motor direction pin 1
const int in2 = 5;      // Right motor direction pin 2
const int in3 = 11;     // Left motor direction pin 1
const int in4 = 10;     // Left motor direction pin 2
const int enB = 9;      // PWM pin for left motor

// Ultrasonic Sensor Pins
const int trigPin = 12;
const int echoPin = 13;

// PID Constants (tuned experimentally)
float Kp = 6;
float Ki = 0.007;
float Kd = 13.9;
// Kp could slightly be increased and Kd decreased since we observed that the robot had to cover about 350cm to align when kept 50-60cm away from the wall. However, these values
// worked perfectly if kept at smaller distances

// PID Variables
float P, I = 0, D;               // Proportional, Integral, Derivative terms
float error, prevError = 0;      // Current and previous error
float errorCorrection;           // PID output
float baseSpeed = 100;           // Base motor speed
float rightSpeed, leftSpeed;     // Speed after correction
const float fixedDistance = 5.0; // Desired distance from wall in cm

// Distance Measurement
float duration, distance;

float correctionFactorR = 0.695; // Right motor correction factor (for imbalance)

bool stopBot = false; // Used to stop the bot from wall-following and proceeding to execute next steps which are taking right turn and going straight

void setup() {
  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);

  // Motors initially stopped
  stopMotors(300);
}

void loop() {
  if(stopBot) return;

  distance = readAverageDistance();

  Serial.print("Distance: ");
  Serial.println(distance);
  
  //to make sure the robot has reached the rear end of the car I have put a condition
  if(distance < 20) {
    wallFollow();
    delay(50);
  }  else {
    stopMotors(100);
    stopBot = true;
    turnRight(); // turn after sensing the rear of car
    goStraight(); // move to the middle of the rear end of car
  }
}

// Measure single distance using ultrasonic sensor
void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); //Time out at 30ms
  distance = (duration * 0.0343) / 2.0;
  //Serial.println(distance); //used for debugging
}

// Read average of 5 distance samplese
float readAverageDistance() {

  float sum = 0;
  
  for (int i = 0; i < 5; i++) {
  
    measureDistance();
    sum += distance;
    delay(10); // short wait between samples
  }

  return sum / 5.0;
}

// PID-controlled wall following
void wallFollow() {

  error = distance - fixedDistance; // Difference from desired wall distance
  P = error;
  I += error;
  
  I = constrain(I, -400, 400);
/*
  To handle cases where the robot starts far from the wall (e.g., >50 cm),
  we limit the integral term to prevent excessive buildup.
  Without this constraint, we observed that the robot made a sharp corrective turn,
  potentially losing ultrasonic contact with the wall.

  While this limit wasn't necessary during the project demo, it's included 
  to make the robot robust across a wider range of starting distances.
*/

  D = error - prevError;
  errorCorrection = (Kp * P) + (Ki * I) + (Kd * D);
  prevError = error;

  rightSpeed = baseSpeed - errorCorrection;
  leftSpeed = baseSpeed + errorCorrection;

// We put constraints to the speed of the wheels to prevent very large difference in speed of both of them.
  int constrain = 50;
  rightSpeed = constrain(rightSpeed, baseSpeed-constrain, baseSpeed+constrain);
  leftSpeed = constrain(leftSpeed, baseSpeed-constrain, baseSpeed + constrain);

  // Motors go forward
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);

  analogWrite(enA, (int)(rightSpeed * correctionFactorR));
  analogWrite(enB, (int)(leftSpeed));

 //Used for debugging
  //Serial.print("right speed: "); Serial.println(rightSpeed);
  //Serial.print("left speed: "); Serial.println(leftSpeed);

}

// Make a right turn
void turnRight() {
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);

  analogWrite(enA, 0);
  analogWrite(enB, 150);

  delay(400); // Tuned experimentally
  stopMotors(50);
}

// Move forward for a short time
void goStraight() {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);

  analogWrite(enA, (int)(120 * correctionFactorR));
  analogWrite(enB, (int)(120));

  delay(390); //Tuned experimentally to adjust how far the robot moves

  stopMotors(30);
}

// Stop both motors for a given amount of time(ms)
void stopMotors(int time) {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  delay(time);
}
