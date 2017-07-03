#include <Wire.h>
#include "MeMCore.h"

// Example project for Makeblock mBot
// http://www.makeblock.com/product/mbot-robot-kit

// Thanks to James Brown for information on pins
// http://blog.hmpg.net/2016/04/makeblock-mcore-information.html

boolean buttonPressed = false;
int button_pin = A7;
void readButtonInner(uint8_t button_pin, int8_t s)
{
  //int pin = A7;
  pinMode(button_pin,INPUT);
  boolean currentPressed = !(analogRead(button_pin)>10);

  if(buttonPressed == currentPressed){
    return;
  }
  buttonPressed = currentPressed;
}

MeBuzzer buzzer;  // Initialize buzzer
void buzzerOn(){
  buzzer.tone(500,1000);
}
void buzzerOff(){
  buzzer.noTone();
}

MeRGBLed rgb(0,30);

MeLineFollower lineFinder(PORT_2);
int sensorState = lineFinder.readSensors();
void readLine()
{
  switch(sensorState)
  {
    case S1_IN_S2_IN:
      Serial.println("Sensor 1 and 2 are inside of black line"); break;
    case S1_IN_S2_OUT:
      Serial.println("Sensor 2 is outside of black line"); break;
    case S1_OUT_S2_IN:
      Serial.println("Sensor 1 is outside of black line"); break;
    case S1_OUT_S2_OUT:
      Serial.println("Sensor 1 and 2 are outside of black line"); break;
    default:
      break;
  }
}

MeUltrasonicSensor ultrasonic(3); // Initialize ultrasonic sensor. Change the number inside the parentheses to the port where you plugged your sensor
//MeUltrasonicSensor ultrasonic(PORT_3);
const unsigned long DISTANCE_MEASUREMENT_INTERVAL = 1UL * 1000UL;
unsigned long previousDistanceMeasurementMillis = 0UL;

MBotDCMotor motorL(M1);
MBotDCMotor motorR(M2);
int direction = 0;
typedef enum {
  DIR_FORWARD = 1,
  DIR_BACKWARD = 2,
  DIR_LEFT = 3,
  DIR_RIGHT = 4,
} DIRECTIONS;
void move(int dir, int spd) {
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (dir==DIR_FORWARD) {
    leftSpeed = -spd;
    rightSpeed = spd;
    rgb.setColor(1, 0, 255, 0);
    rgb.setColor(2, 0, 255, 0);
    rgb.show();
  } else if (dir==DIR_BACKWARD) {
    leftSpeed = spd;
    rightSpeed = -spd;
    rgb.setColor(1, 255, 0, 0);
    rgb.setColor(2, 255, 0, 0);
    rgb.show();
  }  else if (dir==DIR_LEFT) {
    leftSpeed = spd;
    rightSpeed = spd;
    rgb.setColor(1, 0, 255, 0);
    rgb.setColor(2, 255, 0, 0);
    rgb.show();
  } else if (dir==DIR_RIGHT) {
    leftSpeed = -spd;
    rightSpeed = -spd;
    rgb.setColor(1, 255, 0, 0);
    rgb.setColor(2, 0, 255, 0);
    rgb.show();
  }
  motorL.reset(M1);
  motorL.run(leftSpeed);
  motorL.reset(M2);
  motorL.run(rightSpeed);
}
const unsigned long MOTOR_REVERSE_INTERVAL = 2UL * 1000UL;
unsigned long previousMotorReverseMillis = 0UL;
int motorSpeed = 100;
void stopMotors() {
  motorL.stop();
  motorR.stop();
}

void setup()
{
  Serial.begin(9600); //Begins communication with the computer at a baud rate of 9600
  direction = DIR_FORWARD;
  move(direction, motorSpeed);
  rgb.setpin(13);
  rgb.setColor(10, 255, 10);
  rgb.show();
}

void loop()
{
  unsigned long currentMillis = millis();
  // use arduino timer to publish reports
  if (currentMillis - previousMotorReverseMillis >= MOTOR_REVERSE_INTERVAL) {
    previousMotorReverseMillis = currentMillis;
    if (direction==DIR_FORWARD) {
      direction=DIR_BACKWARD;
    } else {
      direction=DIR_FORWARD;
    }
    move(direction, motorSpeed);
  }
  if (currentMillis - previousDistanceMeasurementMillis >= DISTANCE_MEASUREMENT_INTERVAL) {
    previousDistanceMeasurementMillis = currentMillis;
    Serial.print("Distance : "); //Prints the string "Distance : " over the serial most likely the usb. Can be seen using serial monitor in arduino tools setting
    Serial.print(ultrasonic.distanceCm()); //Prints the value received from the Ultrasonic Sensor in Centimeters. Can be changed to inches with .distanceIn()
    // mBot note: .distanceIn() does not seem to work on my mBot.
    Serial.println(" cm");//Prints the string "cm" followed by a new line
    if(ultrasonic.distanceCm()<20) { //if statement to check if data received is less than the value 20, in this case 20 centimeters
      //If value is true the following code executes if false, code will skip this section
      buzzer.tone(262,500); //turns buzzer on
      delay(500); //waits 500 milliseconds or half a second with a minimum value of 100 or 1/10 of a second
      buzzer.tone(0); //turns buzzer off
    }
  }
}
