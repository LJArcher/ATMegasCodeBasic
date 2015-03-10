#include <Wire.h>
#include <Stepper.h>

#define SLAVE_ADDRESS 0x06
int number = 0;
int state = 0;

//STEPPER MOTOR PIN DECLARATIONS
// dir is the positive leads, and brake is the negative
//Stepper 3
const int pwmA3 = 11;
const int pwmB3 = 10;
const int brakeA3 = 1;
const int brakeB3 = 4;
const int dirA3 = 0;
const int dirB3 = 2;

//SOLENOID PIN DECLARATIONS
const int sol1 = 7;
const int sol2 = 8;
const int sol3 = 12;
const int sol4 = 13;

// LIGHT SENSOR PIN DECLARATIONS
//Light sensors are for the Simon Says game
const int green = A0;
const int red = A1;
const int blue = A2;
const int yellow = A3;

int greenLight;
int redLight;
int blueLight;
int yellowLight;

// The amount of steps for a full revolution of your motor.
// 360 / stepAngle
const int STEPS = 200;

// Initialize my stepper object with the number of steps per rotation and the positive leads
Stepper myStepper3(STEPS, dirA3, dirB3);


void setup() {
   // initialize i2c as slave
   Wire.begin(SLAVE_ADDRESS);
 
   // define callbacks for i2c communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   
  // Set the RPM of the motors
  myStepper3.setSpeed(90);
  
  // set pin modes for Stepper 1's PWM and brakes (negative leads)
  pinMode(pwmA3, OUTPUT);
  pinMode(pwmB3, OUTPUT);
  pinMode(brakeA3, OUTPUT);
  pinMode(brakeB3, OUTPUT);
  
  // Turn on pulse width modulation for both stepper motors
  digitalWrite(pwmA3, HIGH);
  digitalWrite(pwmB3, HIGH);

  // Turn off the brakes for both stepper motors
  digitalWrite(brakeA3, LOW);
  digitalWrite(brakeB3, LOW);
  
  // initialize the digital pin for solenoids
  pinMode(sol1, OUTPUT);
  pinMode(sol2, OUTPUT);
  pinMode(sol3, OUTPUT);
  pinMode(sol4, OUTPUT);      

  // Begin serial connection for logging and monitoring
  //Serial.begin(9600);     
}
  //END SETUP


void loop() {
  
  
}

///////////  PHOTOCELL METHODS  ////////////////

int simonReads() {
  
  int value = 100;    // baseline value for dim light - if sensor reads above this value, it is valid
  
  greenLight = analogRead(green);
  redLight = analogRead(red);
  blueLight = analogRead(blue);
  yellowLight = analogRead(yellow);
  
  if (greenLight > value) {
    return 1;
  }
  if (redLight > value) {
    return 2;
  }
  if (blueLight > value) {
    return 3;
  }
  if (yellowLight > value) {
    return 4;
  } 
}

///////////  STEPPER MOTOR METHOD  ////////////////

// Have stepper 3 motor turn x rotations
void step3Turn(int rotations) {
 
  // Move the motor however many full rotations, (positive number is forward, negative number is reverse)
  myStepper3.step(STEPS * rotations);
  // Serial.println(STEPS * rotations);    //for debugging
  // Pause for 0.5 seconds
  delay(500);  
}

///////////  SOLENOID METHODS  ////////////////

void goSol1(){
  digitalWrite(sol1, HIGH);   // activate solenoid 1
  delay(80);               // wait 80ms for solenoid to extend
}

void goSol2(){
  digitalWrite(sol2, HIGH);   // activate solenoid 2
  delay(80);               // wait 80ms for solenoid to extend
}

void goSol3(){
  digitalWrite(sol3, HIGH);   // activate solenoid 3
  delay(80);               // wait 80ms for solenoid to extend
}

void goSol4(){
  digitalWrite(sol4, HIGH);   // activate solenoid 4
  delay(80);               // wait 80ms for solenoid to extend
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
