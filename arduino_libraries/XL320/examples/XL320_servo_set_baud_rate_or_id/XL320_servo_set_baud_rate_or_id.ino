
// ========================================
// Dynamixel XL-320 Arduino library example
// ========================================

// Read more:
// https://github.com/hackerspace-adelaide/XL320

#include "XL320.h"

// Name your robot!
XL320 robot;

void setup() {

  // Talking standard serial, so connect servo data line to Digital TX 1
  // Set the default servo baud rate which is 1000000 (1Mbps) if it's a brand new servo
  Serial.begin(1000000);

  // Initialise your robot
  robot.begin(Serial); // Hand in the serial object you're using
  delay(100);
  
  // Current servoID
  int servoID = 254;

  // NOTE: comment out either the XL_BAUD_RATE or XL_ID, only send one at a time

  // ===================================
  // Set the serial connection baud rate
  // ===================================
  
  // writePacket(1, XL_BAUD_RATE, x) sets the baud rate:
  // 0: 9600, 1:57600, 2:115200, 3:1Mbps
  robot.sendPacket(servoID, XL_BAUD_RATE, 2);

  // ================
  // Set the servo ID
  // ================
  
  // writePacket(1, XL_ID, x) sets the baud rate:
  // ID can be between 1 and 253 (but not 200)
//  robot.sendPacket(servoID, XL_ID, 3);
}

void loop() {
  // NOTE: load this sketch to the Arduino > Servo
  // Then power cycle the Arduino + Servo

  // NOTE: When setting the servo ID, the baud rate defaults down to 9600
  // So then you'll need to power cycle, and re-write the baud rate to whatever you want via 9600
}