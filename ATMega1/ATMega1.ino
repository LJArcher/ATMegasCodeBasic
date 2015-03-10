#include <Wire.h>
#include <Servo.h> 

#define HIGH_SPEED 255
#define LOW_SPEED 100
#define BASE_SPEED 150

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

Servo myServo1;  // create servo object to control servo 1
Servo myServo2;  // create servo object to control servo 2 
 
int pos1 = 0;    // variable to store the servo 1 position 
int pos2 = 0;    // variable to store the servo 2 position 

//WHEEL MOTOR PIN DECLARATIONS
// Declare pins for wheels' positive and negative terminals
//front wheels
const int wheelFLfwd = 0;      //Front left wheel's positive terminal on digital pin 0
const int wheelFLrev = 1;      //Front left wheel's negative terminal on digital pin 1
const int wheelFRfwd = 2;      //Front right wheel's positive terminal on digital pin 2
const int wheelFRrev = 4;      //Front right wheel's negative terminal on digital pin 4
//back wheels
const int wheelBLfwd = 7;      //Back left wheel's positive terminal on digital pin 7
const int wheelBLrev = 8;      //Back left wheel's negative terminal on digital pin 8
const int wheelBRfwd = 12;     //Back right wheel's positive terminal on digital pin 12
const int wheelBRrev = 13;     //Back right wheel's negative terminal on digital pin 13
//Declare pins for wheels' PWM
const int wheelFLpwm = 11;     //Front left wheel PWM on digital pin 11
const int wheelFRpwm = 10;     //Front right wheel PWM on digital pin 10
const int wheelBLpwm = 9;      //Back left wheel PWM on digital pin 9
const int wheelBRpwm = 6;      //Back left wheel PWM on digital pin 6

//SERVO MOTOR PIN DECLARATIONS
const int servo1 = 5;          // Servo 1 PWM is on digital pin 5
const int servo2 = 3;          // Servo 2 PWM is on digital pin 3

int leftSpeed = BASE_SPEED;
int rightSpeed = BASE_SPEED;

void setup() {
   // initialize i2c as slave
   Wire.begin(SLAVE_ADDRESS);
 
   // define callbacks for i2c communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   
   //initializes & attaches the servo pin to the servo object
   myServo1.attach(servo1);
   
    // initialize the motor pins as an output:
    pinMode(wheelFLfwd, OUTPUT);
    pinMode(wheelFLrev, OUTPUT);
    pinMode(wheelFLpwm, OUTPUT);
    
    pinMode(wheelFRfwd, OUTPUT);
    pinMode(wheelFRrev, OUTPUT);
    pinMode(wheelFRpwm, OUTPUT);
    
    pinMode(wheelBLfwd, OUTPUT);
    pinMode(wheelBLrev, OUTPUT);
    pinMode(wheelBLpwm, OUTPUT);
    
    pinMode(wheelBRfwd, OUTPUT);
    pinMode(wheelBRrev, OUTPUT);
    pinMode(wheelBRpwm, OUTPUT);
}    //END SETUP


void loop() {
  
  
}

///////////  SERVO MOTOR METHODS  ////////////////

//functions to (open & close) servo 1
void s1Open() {
 
  for(pos1 = 0; pos1 < 180; pos1 += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myServo1.write(pos1);              // tell servo to go to position in variable 'pos1' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
}
void s1Close(){
  for(pos1 = 180; pos1>=1; pos1-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myServo1.write(pos1);              // tell servo to go to position in variable 'pos1' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

//functions to (open & close) servo 2
void s2Open() {
 
  for(pos2 = 0; pos2 < 180; pos2 += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myServo2.write(pos2);              // tell servo to go to position in variable 'pos2' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}
void s2Close(){
  for(pos2 = 180; pos2>=1; pos2-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myServo2.write(pos2);              // tell servo to go to position in variable 'pos2' 
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

///////////  MECANUM MOTOR METHODS  ////////////////

void stop(void) //Stop
{
  digitalWrite(wheelFLpwm,LOW);
  digitalWrite(wheelFRpwm,LOW);
  digitalWrite(wheelBLpwm,LOW);
  digitalWrite(wheelBRpwm,LOW);
}

void forward(int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,HIGH);
  digitalWrite(wheelFLrev,LOW);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,HIGH);
  digitalWrite(wheelFRrev,LOW);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,HIGH);
  digitalWrite(wheelBLrev,LOW);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,HIGH);
  digitalWrite(wheelBRrev,LOW);
}

void reverse (int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,LOW);
  digitalWrite(wheelFLrev,HIGH);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,LOW);
  digitalWrite(wheelFRrev,HIGH);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,LOW);
  digitalWrite(wheelBLrev,HIGH);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,LOW);
  digitalWrite(wheelBRrev,HIGH);
}

void ccw (int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,LOW);
  digitalWrite(wheelFLrev,HIGH);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,HIGH);
  digitalWrite(wheelFRrev,LOW);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,LOW);
  digitalWrite(wheelBLrev,HIGH);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,HIGH);
  digitalWrite(wheelBRrev,LOW);
}

void cw (int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,HIGH);
  digitalWrite(wheelFLrev,LOW);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,LOW);
  digitalWrite(wheelFRrev,HIGH);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,HIGH);
  digitalWrite(wheelBLrev,LOW);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,LOW);
  digitalWrite(wheelBRrev,HIGH);
}

void strafeLeft (int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,LOW);
  digitalWrite(wheelFLrev,HIGH);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,HIGH);
  digitalWrite(wheelFRrev,LOW);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,HIGH);
  digitalWrite(wheelBLrev,LOW);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,LOW);
  digitalWrite(wheelBRrev,HIGH);
}

void strafeRight (int leftSpeed, int rightSpeed)
{
  analogWrite (wheelFLpwm, leftSpeed);
  digitalWrite(wheelFLfwd,HIGH);
  digitalWrite(wheelFLrev,LOW);
  analogWrite (wheelFRpwm, rightSpeed);
  digitalWrite(wheelFRfwd,LOW);
  digitalWrite(wheelFRrev,HIGH);
  analogWrite (wheelBLpwm, leftSpeed);
  digitalWrite(wheelBLfwd,LOW);
  digitalWrite(wheelBLrev,HIGH);
  analogWrite (wheelBRpwm, rightSpeed);
  digitalWrite(wheelBRfwd,HIGH);
  digitalWrite(wheelBRrev,LOW);
}

