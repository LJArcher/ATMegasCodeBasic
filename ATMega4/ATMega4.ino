#include <Wire.h>
#include <QTRSensors.h>


#define SLAVE_ADDRESS 0x07
int number = 0;
int state = 0;

//#define Kp 0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
//#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 12

#define MIDDLE_POS (((NUM_SENSORS*1000) - 1000) / 2)

#define WHITE_THRESHOLD 300
#define BLACK_THRESHOLD 100

#define LINE_LOST -1
#define EDGE_LEFT -2
#define EDGE_RIGHT -3
#define END_TRACK -4

int TRACKING_WHITE = 1;

//DECLARE PINS FOR ALL SENSORS
// red light start sensor
const int startLight = A0;
const int pingSensor = A1;

int lightReading;    // to hold value returned by sensor
long duration, inches;      //hold value for ultrasonic ping sensor

// Reflectance array sensors 0 through 7 are connected to digital pins 2 through 9, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7, 8, 9},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

void setup() {
   // initialize i2c as slave
   Wire.begin(SLAVE_ADDRESS);
 
   // define callbacks for i2c communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   
   //for debugging, uncomment
//  Serial.begin(9600);
}


void loop() {
  
  //enter anything that loops here - like checking for i2c data
  
  //constant readings from the CdS photocell
  lightReading = analogRead(startLight);
  
  //call ping() before using the variable inches to find distance
  
  delay(100);
}

///////////  REFLECTANCE ARRAY CALIBRATION METHOD  ////////////////

  //ATMega1 should have been commanded to strafe L and strafe R during this operation
void sensorCalibrate() {
     int i;
   for (i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
     // UNCOMMENT FOR DEGBUGGING
     //print the calibration minimum values measured when emitters were on
//  Serial.begin(9600);
//  for (i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(qtrrc.calibratedMinimumOn[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
//
//  // print the calibration maximum values measured when emitters were on
//  for (i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(qtrrc.calibratedMaximumOn[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
//  Serial.println();
//  delay(1000);
}

///////////  NAVIGATION METHODS  ////////////////

//method for reporting if the line has been lost
boolean lineLost(unsigned int* vals){
  boolean lost = true;

  if (TRACKING_WHITE == 0){
    for(int i=0;i<NUM_SENSORS;i++){
      if (vals[i] > WHITE_THRESHOLD){
        lost =false;
      }
    }
  }else{
    for(int i=0;i<NUM_SENSORS;i++){
      if (vals[i] < BLACK_THRESHOLD){
        lost =false;
      }
    }
  }
  return lost;
}

//method for detecting if it is white line on black or black line on white (grey)
void detectTrackColor(unsigned int* vals){
  if ((vals[0] < WHITE_THRESHOLD) &&
      (vals[NUM_SENSORS - 1] < WHITE_THRESHOLD))
    TRACKING_WHITE = 0;

  if ((vals[0] > BLACK_THRESHOLD) &&
      (vals[NUM_SENSORS - 1]> BLACK_THRESHOLD))
    TRACKING_WHITE = 1;
}

//method for detecting a 90 degree turn on left or right
// values > BLACK_THRESHOLD means it is reading "relative white"
// values < WHITE_THRESHOLD means it is reading "relative black"
int checkEdge(unsigned int* vals){ 
  // for black line path
  if (TRACKING_WHITE == 0){
    //if sensors on left all read black, and sensors on right don't
    if ((vals[NUM_SENSORS - 1] > WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] > WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] > WHITE_THRESHOLD) &&
        (vals[0] < WHITE_THRESHOLD) &&
        (vals[1] < WHITE_THRESHOLD) &&
        (vals[2] < WHITE_THRESHOLD)){
      return EDGE_LEFT;
    }
    //if sensors on right all read black, and sensors on left don't
    if ((vals[0] > WHITE_THRESHOLD) &&
        (vals[1] > WHITE_THRESHOLD) &&
        (vals[2] > WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 1] < WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] < WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] < WHITE_THRESHOLD)){
      return EDGE_RIGHT;
    }
    //if sensors on both left and right are all reading black - T junction or end of path
    if ((vals[NUM_SENSORS - 1] < WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] < WHITE_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] < WHITE_THRESHOLD) &&
        (vals[0] < WHITE_THRESHOLD) &&
        (vals[1] < WHITE_THRESHOLD) &&
        (vals[2] < WHITE_THRESHOLD)){
      return END_TRACK;
    }
  }
  //for white line path
  else{
    //if sensors on left all read white, and sensors on right don't
    if ((vals[NUM_SENSORS - 1] < BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] < BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] < BLACK_THRESHOLD) &&
        (vals[0] > BLACK_THRESHOLD) &&
        (vals[1] > BLACK_THRESHOLD) &&
        (vals[2] > BLACK_THRESHOLD)){
      return EDGE_LEFT;
    }
    //if sensors on right all read white, and sensors on left don't
    if ((vals[0] < BLACK_THRESHOLD) &&
        (vals[1] < BLACK_THRESHOLD) &&
        (vals[2] < BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 1] > BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] > BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] > BLACK_THRESHOLD)){
      return EDGE_RIGHT;
    }
    //if sensors on both left and right are all reading white - T junction or end of path
    if ((vals[NUM_SENSORS - 1] > BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 2] > BLACK_THRESHOLD) &&
        (vals[NUM_SENSORS - 3] > BLACK_THRESHOLD) &&
        (vals[0] > BLACK_THRESHOLD) &&
        (vals[1] > BLACK_THRESHOLD) &&
        (vals[2] > BLACK_THRESHOLD)){
      return END_TRACK;
    }
  }
  return 0;
}

//main method that reads the position 
int readsLine(QTRSensorsRC* qtr){
  unsigned int val[NUM_SENSORS];
  qtr->readCalibrated(val);

  detectTrackColor(val);

  int line = qtr->readLine(val,QTR_EMITTERS_ON,TRACKING_WHITE);

//#ifdef DEBUG
//  Serial.print(TRACKING_WHITE);   Serial.print(" ");
//  Serial.print(line);   Serial.print(" R< ");
//  Serial.print(val[0]); Serial.print(" ");
//  Serial.print(val[1]); Serial.print(" ");
//  Serial.print(val[2]); Serial.print(" ");
//  Serial.print(val[3]); Serial.print(" ");
//  Serial.print(val[4]); Serial.print(" ");
//  Serial.print(val[5]); Serial.print(" ");
//  Serial.print(val[6]); Serial.print(" ");
//  Serial.print(val[7]); Serial.print(" >L ");
//
//  if (lineLost(val) == true)
//    Serial.print("Line Lost \n");
//  if (checkEdge(val) != 0)
//    Serial.print("Edge \n");
//#endif

  if (lineLost(val) == true)
    return LINE_LOST;

  else if (checkEdge(val) != 0)
    return checkEdge(val);

  else
    return line;
}

//method that will be called by pi
void navigate() {
// int pos = qtrrc.readLine(sensorValues);
 int pos = readsLine(&qtrrc);
 if (pos != LINE_LOST){

    if (pos == EDGE_RIGHT){
      //send right_turn instruction
    }else if (pos == EDGE_LEFT){
      //send left_turn instruction
    }else{

      int error = pos - MIDDLE_POS;    
      //send error value for course corrections

//      if (error < 0)
//        leftSpeed = map(error,-MIDDLE_POS,0,REVERSE_MAX,FORWARD_MAX);
//      else
//        m2Speed = map(error,0,MIDDLE_POS,FORWARD_MAX,REVERSE_MAX);

//#ifdef DEBUG
//      Serial.print(error);   Serial.print(" ");
//      Serial.print(m1Speed); Serial.print(" "); Serial.println(m2Speed);
//#endif

//      motor(MotorA,m1Speed);
//      motor(MotorB,m2Speed);
    }
  }
}
//END navigation function


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

///////////  PING SENSOR METHODS  ////////////////

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

void ping()
{
 // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingSensor, OUTPUT);
  digitalWrite(pingSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(pingSensor, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingSensor, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingSensor, INPUT);
  duration = pulseIn(pingSensor, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration); 
}
