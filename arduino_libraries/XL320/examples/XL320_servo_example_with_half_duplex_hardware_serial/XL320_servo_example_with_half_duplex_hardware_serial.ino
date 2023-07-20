
// ========================================
// Dynamixel XL-320 Arduino library example
// ========================================

// Read more:
// https://github.com/hackerspace-adelaide/XL320

#include "XL320.h"

// NOTE: WORK IN PROGRESS!
// For Half Duplex Hardware Serial to read data back from the servos
#include <HalfDuplexHardwareSerial.h>

// Name your robot!
XL320 robot;

// Set some variables for incrementing position & LED colour
char rgb[] = "rgbypcwo";
int servoPosition = 0;
int ledColour = 0;

// Delay between sending & receiving data
int delayBetweenSendReceive = 500;

// Set the default servoID to talk to
int servoID = 3;

void setup() {
  
  // You have to manually set the TX and RX lines of your chosen serial port to INPUT_PULLUP
  // as this is the state they will be in when they are disconnected from the UART
  // you also need to physically link the TX and RX pins in your circuit
  // You can achieve this by connecting the data line out of the servo back into pin 0 RX
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  
  pinMode(13, OUTPUT);

  // Talk serial with your servo by connecting the servo input data line to Digital TX 1
  HalfDuplexSerial.begin(115200);
  HalfDuplexSerial.setTimeout(delayBetweenSendReceive);

  // Initialise your robot
  robot.begin(HalfDuplexSerial); // Hand in the serial object you're using
  
  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servoID, 1023);
  delay(delayBetweenSendReceive);

}

void loop() {

  // LED test.. let's randomly set the colour (0-7)
//  robot.LED(servoID, &rgb[random(0,7)] );

  // LED test.. select a random servoID and colour
  robot.LED(servoID, &rgb[ledColour] );

  // LED colour test.. cycle between RGB, increment the colour and return 1 after 3
  ledColour = (ledColour + 1) % 3;

  // Set a delay to account for the receive delay period
  delay(delayBetweenSendReceive);

  // Servo test.. select a random servoID and colour
  robot.moveJoint(servoID, servoPosition);

  // Servo test.. increment the servo position by 100 each loop
  servoPosition = (servoPosition + 100) % 1023;
  
  // Set a delay to account for the receive delay period
  delay(delayBetweenSendReceive);
  
  // To read back from the servo
  // TODO: Fix this.. getting invalid packets..
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer,256));
  if (p.isValid()) {
    // Set pin 13 HIGH so we know!
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}