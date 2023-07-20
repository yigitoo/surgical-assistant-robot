
// ========================================
// Dynamixel XL-320 Arduino library example
// ========================================

// Read more:
// https://github.com/hackerspace-adelaide/XL320

#include "XL320.h"

// Name your robot!
XL320 robot;

// If you want to use Software Serial, uncomment this line
#include <SoftwareSerial.h>

// Set the SoftwareSerial RX & TX pins
SoftwareSerial mySerial(10, 11); // (RX, TX)

// Set some variables for incrementing position & LED colour
char rgb[] = "rgbypcwo";
int servoPosition = 0;
int ledColour = 0;

// Set the default servoID to talk to
int servoID = 1;

void setup() {

  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial.begin(115200);

  // Setup Software Serial
  mySerial.begin(115200);

  // Initialise your robot
  robot.begin(Serial); // Hand in the serial object you're using
  
  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servoID, 1023);

}

void loop() {

  // LED test.. let's randomly set the colour (0-7)
//  robot.LED(servoID, &rgb[random(0,7)] );

  // LED test.. select a random servoID and colour
  robot.LED(random(1,4), &rgb[random(0,7)] );

  // LED colour test.. cycle between RGB, increment the colour and return 1 after 3
//  robot.LED(servoID, &rgb[ledColour]);
  ledColour = (ledColour + 1) % 3;

  // Set a delay to account for the receive delay period
  delay(100);

  // Servo test.. let's randomly set the position (0-1023)
//  robot.moveJoint(servoID, random(0, 1023));

  // Servo test.. select a random servoID and colour
  robot.moveJoint(random(1,4), random(0, 1023));

  // Servo test.. increment the servo position by 100 each loop
//  robot.moveJoint(servoID, servoPosition);
  servoPosition = (servoPosition + 100) % 1023;
  
  // Set a delay to account for the receive delay period
  delay(100);
}