/* Electronics Project 2013: Robot Arm
By Maria Jantz, Seth Miller, Petey Biddle
*/

#include <Wire.h>  // For the I2C communication used by LCD shield 
//#include <Adafruit_MCP23017.h> //what even is this?
#include <string> // For processing of serial data (in strings)
#include <Servo.h> // To send information to servos

// Create all servo objects
Servo grabber;
Servo wrotate; 
Servo wbend;
Servo elbow;
Servo shoulder;
Servo swivel;
// Define pins for each servo
int grabnum = 2; // Grabber
int wrotnum = 3; // Wrist Rotate
int wbenum = 4; // Wrist Bend
int elbnum = 5; // Elbow
int shoulnum = 6; // Shoulder
int swivnum = 7; // Swivel
int stp = 8; // Green wire to Pololu stepper control
int dir = 9; // Blue wire to Pololu stepper control

void setup(){
  // Attach all servos
  grabber.attach(grabnum); 
  wrotate.attach(wrotnum); 
  wbend.attach(wbenum); 
  elbow.attach(elbnum); 
  shoulder.attach(shoulnum); 
  swivel.attach(swivnum); 
  // Start serial
  Serial.begin(9600);
  // Start listening for stepper control
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
}


void loop(){

  char buffer[7]; // Create a buffer to read in serial data
  char servoarray[2]; // Create an array to store the servo number
  char anglearray[6]; // Create an array to store the angle
  
  if (Serial.available()>0){
    Serial.readBytesUntil('\0', buffer, 5);
    servoarray[0] = buffer[0]; // Get servo number from the serial string
    servoarray[1] = '\0';
    anglearray[0] = buffer[1]; // Get angle information from the serial string
    anglearray[1] = buffer[2];
    anglearray[2] = buffer[3]; 
    anglearray[3] = buffer[4];
    anglearray[4] = buffer[5];
    anglearray[5] = buffer[6];
    
    // Convert to strings
    int servonum = atoi(servoarray);
    int angle = atoi(anglearray);

  if (servonum == grabnum){ //grabber
    if (angle == 0){
      grabber.write(55); //closed
    } else if (angle == 1){
      grabber.write(125); //open
    }
  }else if (servonum == wrotnum){ //wrist rotate
    if (abs(angle<20)){
      wrotate.write(angle+90);
    }
  }else if (servonum == wbenum){ //wbend
    wbend.write(angle+90);
  }else if (servonum == elbnum){ //elbow
    elbow.write(angle);
  }else if (servonum == shoulnum){ //shoulder
    shoulder.write(angle);
  }else if (servonum == swivnum){ //swivel
    swivel.write(angle+90);
  }else if (servonum == 9){ //stepper motor
    if (angle == 1){ //high; go forward
      digitalWrite(dir, LOW); //set forward
      stepper(100, 1200); 
    } else if (angle == 0){  //low, go backward
      digitalWrite(dir, HIGH); //set backward
      stepper(100, 1200);
    }
    }
  } 
}

void stepper(int numsteps, int deltime){
  for (int i = 0; i < numsteps; i++){
      digitalWrite(stp, LOW);
      delayMicroseconds(10);
      digitalWrite(stp, HIGH); //low to high is when step happens
      delayMicroseconds(deltime); //delay controls speed of movement
    }
}

