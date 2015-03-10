#include <Wire.h>
#include <Servo.h>
#include <Stepper.h> 

#define SLAVE_ADDRESS 0x05
int number = 0;
int state = 0;

Servo myServo3;  // create servo object to control servo 3
Servo myServo4;  // create servo object to control servo 4
 
int pos3 = 0;    // variable to store the servo 3 position 
int pos4 = 0;    // variable to store the servo 4 position 

//SERVO MOTOR PIN DECLARATIONS
const int servo3 = 11;          // Servo 3 PWM is on digital pin 11
const int servo4 = 10;          // Servo 4 PWM is on digital pin 10

//STEPPER MOTOR PIN DECLARATIONS
// dir is the positive leads, and brake is the negative
//Stepper 1
const int pwmA1 = 9;
const int pwmB1 = 6;
const int brakeA1 = 1;
const int brakeB1 = 4;
const int dirA1 = 0;
const int dirB1 = 2;
//Stepper 2
const int pwmA2 = 5;
const int pwmB2 = 3;
const int brakeA2 = 8;
const int brakeB2 = 13;
const int dirA2 = 7;
const int dirB2 = 12;

// The amount of steps for a full revolution of your motor.
// 360 / stepAngle
const int STEPS = 200;

// Initialize my stepper object with the number of steps per rotation and the positive leads
Stepper myStepper1(STEPS, dirA1, dirB1);
Stepper myStepper2(STEPS, dirA2, dirB2);

void setup() {
   // initialize i2c as slave
   Wire.begin(SLAVE_ADDRESS);
 
   // define callbacks for i2c communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   
  // Set the RPM of the motors
  myStepper1.setSpeed(90);
  myStepper2.setSpeed(90);
  
  // set pin modes for Stepper 1's PWM and brakes (negative leads)
  pinMode(pwmA1, OUTPUT);
  pinMode(pwmB1, OUTPUT);
  pinMode(brakeA1, OUTPUT);
  pinMode(brakeB1, OUTPUT);
  
  // set pin modes for Stepper 2's PWM and brakes (negative leads)
  pinMode(pwmA2, OUTPUT);
  pinMode(pwmB2, OUTPUT);
  pinMode(brakeA2, OUTPUT);
  pinMode(brakeB2, OUTPUT);
  
  // Turn on pulse width modulation for both stepper motors
  digitalWrite(pwmA1, HIGH);
  digitalWrite(pwmB1, HIGH);
  digitalWrite(pwmA2, HIGH);
  digitalWrite(pwmB2, HIGH);

  // Turn off the brakes for both stepper motors
  digitalWrite(brakeA1, LOW);
  digitalWrite(brakeB1, LOW);
  digitalWrite(brakeA2, LOW);
  digitalWrite(brakeB2, LOW);

  // Begin serial connection for logging and monitoring
  Serial.begin(9600); 
} 
  //END SETUP


void loop() {
  
  
}

///////////  STEPPER MOTOR METHODS  ////////////////

// Have stepper 1 motor turn x rotations
void step1Turn(int rotations) {
 
  // Move the motor however many full rotations, (positive number is forward, negative number is reverse)
  myStepper1.step(STEPS * rotations);
  // Serial.println(STEPS * rotations);    //for debugging
  // Pause for 0.5 seconds
  delay(500);  
}

void step2Turn(int rotations) {
 
  // Move the motor however many full rotations, (positive number is forward, negative number is reverse)
  myStepper2.step(STEPS * rotations);
  // Serial.println(STEPS * rotations);    //for debugging
  // Pause for 0.5 seconds
  delay(500); 
}

///////////  SERVO MOTOR METHODS  ////////////////

//functions to (open & close) servo 3
void s3Open() {
 
  for(pos3 = 0; pos3 < 180; pos3 += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myServo3.write(pos3);              // tell servo to go to position in variable 'pos3' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
}
void s3Close(){
  for(pos3 = 180; pos3>=1; pos3-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myServo3.write(pos3);              // tell servo to go to position in variable 'pos3' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

//functions to (open & close) servo 4
void s4Open() {
 
  for(pos4 = 0; pos4 < 180; pos4 += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myServo4.write(pos4);              // tell servo to go to position in variable 'pos4' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}
void s4Close(){
  for(pos4 = 180; pos4>=1; pos4-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myServo4.write(pos4);              // tell servo to go to position in variable 'pos4' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

///////////  I2C METHODS  ////////////////

// callback for received data on i2c
void receiveData(int byteCount){
 
 while(Wire.available()) {
  number = Wire.read();
 
  if (number == 3){
   if (state == 0){
    digitalWrite(13, HIGH); // set the LED on
    state = 1;
   } else{
    digitalWrite(13, LOW); // set the LED off
    state = 0;
   }
  }
  if(number==4) {
   number = 0;
  }
 }
}
 
// callback for sending data on i2c
void sendData(){
 Wire.write(number);
}
