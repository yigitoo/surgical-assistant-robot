#include <ArduinoJson.h>
#include "DynamixelMotor.h"

const uint8_t id=1;
int16_t angular_velocity=512;

const char* ssid = "hasan";
const char* password = "yagiz123";

int firstMotor = 0;
int secondMotor = 0;
int thirdMotor = 0;
int fourthMotor = 0;

// direction pin, if you use tristate buffer
#define DIR_PIN 2

SoftwareSerial robot_serial(3,4);
// Use this for hardware serial without tristate buffer
HardwareDynamixelInterface interface(robot_serial);
DynamixelMotor motor(interface, id);
const long unsigned int baudrate = 1000000;

DynamixelDevice broadcast_device(interface, BROADCAST_ID);

uint8_t led_state = true;

void setup() {
  interface.begin(baudrate);
  delay(100);
  
  // check if we can communicate with the motor
  // if not, we turn the led on and stop here
  uint8_t status = motor.init();
  if(status!=DYN_STATUS_OK)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    while(1);
  }
  motor.enableTorque();  

  // set to joint mode, with a 180° angle range
  // see robotis doc to compute angle values
  // every unit equals to 0.29°.
  motor.jointMode(204, 820);
  //motor.wheelMode(); if in wheel mode change the goalPosition to the motor.speed().
  motor.speed(angular_velocity);
}

void loop() {
  broadcast_device.write(DYN_ADDRESS_LED, led_state);
  led_state=!led_state;
  delay(1000);
}
