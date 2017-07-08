#include <Wire.h>

#define ME_PORT_DEFINED

#ifdef ME_PORT_DEFINED

#include "MeMCore.h"

const byte BUZZER_PIN = 8;
const byte RGB_LED_PIN = 13;
const byte LIGHTSENSOR_PIN = A6;
const byte BUTTON_PIN = A7;

#else

#include "MeBuzzer.h"
#include "MeLightSensor.h"
#include "MeLineFollower.h"
#include "MeUltrasonicSensor.h"
#include "MeDCMotor.h"
#include "MeRGBLed.h"


// Example project for Makeblock mBot
// http://www.makeblock.com/product/mbot-robot-kit

// Thanks to James Brown for information on pins
// http://blog.hmpg.net/2016/04/makeblock-mcore-information.html
//
// Arduino mCore pin assignments
// D0/RXD	RXD on external radio connector
// D1/TXD	TXD on external radio connector
// D2	IR receiver input
// D3~	IR LED output (HIGH = ON)
// D4	M2 direction (HIGH = CCW)
// D5~	M2 PWM (speed)
// D6~	M1 PWM (speed)
// D7	M1 direction (HIGH = CW)
// D8	Buzzer output
// D9~	Pin 5 on RJ25 #2
// D10~ (SPI) SS	Pin 6 on RJ25 #2
// D11~ (SPI) MOSI	Pin 5 on RJ25 #1
// D12 (SPI) MISO	Pin 6 on RJ25 #1
// D13 (SPI) SCLK	Blue LED / Serial out to WS2812 LEDs
// A0	Pin 5 on RJ25 #4
// A1	Pin 6 on RJ25 #4
// A2	Pin 5 on RJ25 #3
// A3	Pin 6 on RJ25 #3
// A4 (I2C) SDA	Pin 2 on all 4 RJ25 connectors
// A5 (I2C) SCL	Pin 1 on all 4 RJ25 connectors
// A6	Light sensor input
// A7	Button input, low when pressed

// pin assignments
const byte MOTOR_R_DIR_PIN = 4;
const byte MOTOR_R_PWM_PIN = 5;
const byte MOTOR_L_PWM_PIN = 6;
const byte MOTOR_L_DIR_PIN = 7;
const byte BUZZER_PIN = 8;
const byte PORT_2_S1_PIN = 9;
const byte PORT_2_S2_PIN = 10;
const byte PORT_1_S1_PIN = 11;
const byte PORT_1_S2_PIN = 12;
const byte RGB_LED_PIN = 13;
const byte PORT_4_S1_PIN = A0;
const byte PORT_4_S2_PIN = A1;
const byte PORT_3_S1_PIN = A2;
const byte PORT_3_S2_PIN = A3;
const byte LIGHTSENSOR_PIN = A6;
const byte BUTTON_PIN = A7;

#endif

const unsigned long BAUD = 9600;

// button parameters and methods
boolean state = false;
boolean pressed = false;
const unsigned int BUTTON_LOW_VOLTAGE = 10;
boolean buttonStateChange() {
  pressed = !(analogRead(BUTTON_PIN)>BUTTON_LOW_VOLTAGE);
  if(state == pressed) {
    return false;
  }
  state = pressed;
  return true;
}

// buzzer parameters and methods
#ifdef ME_PORT_DEFINED
MeBuzzer buzzer(BUZZER_PIN);
#else
MeBuzzer buzzer(BUZZER_PIN);
#endif
void buzzerTone(int frequency, long duration) {
  buzzer.tone(frequency,duration);
}
void buzzerOn() {
  int frequency = 262;
  long duration = 500;
  buzzer.tone(frequency,duration);
}
void buzzerOff() {
  buzzer.noTone();
}

// onboard led parameters and methods
#ifdef ME_PORT_DEFINED
//MeRGBLed rgbLed(0,2);
MeRGBLed rgbLed(0,2);
const byte rgbLed_1_IDX = 0;
const byte rgbLed_2_IDX = 1;
#else
MeRGBLed rgbLed(0);
const byte rgbLed_1_IDX = 1;
const byte rgbLed_2_IDX = 2;
#endif
void rgbLedOff() {
  rgbLed.setColor(0,0,0);
  rgbLed.show();
}

// light sensor parameters and methods
#ifdef ME_PORT_DEFINED
MeLightSensor lightsensor(LIGHTSENSOR_PIN);
#else
MeLightSensor lightsensor(8, LIGHTSENSOR_PIN);
#endif
const unsigned long LIGHTSENSOR_MEASUREMENT_INTERVAL = 1UL * 1000UL;
unsigned long previousLightSensorMeasurementMillis = 0UL;

// line-follower parameters and methods
// MeLineFollower lineFinder(PORT_2);
//const byte LINEFOLLOWER_PIN = PORT_2_D1_PIN;

// int sensorState = lineFinder.readSensors();
// void readLine() {
//   switch(sensorState) {
//     case S1_IN_S2_IN:
//       Serial.println("Sensor 1 and 2 are inside of black line"); break;
//     case S1_IN_S2_OUT:
//       Serial.println("Sensor 2 is outside of black line"); break;
//     case S1_OUT_S2_IN:
//       Serial.println("Sensor 1 is outside of black line"); break;
//     case S1_OUT_S2_OUT:
//       Serial.println("Sensor 1 and 2 are outside of black line"); break;
//     default:
//       break;
//   }
// }

// ultrasonic sensor parameters and methods
#ifdef ME_PORT_DEFINED
MeUltrasonicSensor ultrasonic(PORT_3);
#else
const byte ULTRASONIC_SENSOR_S1_PIN = PORT_3_S1_PIN;
const byte ULTRASONIC_SENSOR_S2_PIN = PORT_3_S2_PIN;
MeUltrasonicSensor ultrasonic(ULTRASONIC_SENSOR_S1_PIN);
#endif
const unsigned long DISTANCE_MEASUREMENT_INTERVAL = 1UL * 1000UL;
unsigned long previousDistanceMeasurementMillis = 0UL;

// motor parameters and methods
#ifdef ME_PORT_DEFINED
MeDCMotor motorL(M1);
MeDCMotor motorR(M2);
#else
MeDCMotor motorL(MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
MeDCMotor motorR(MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
#endif
const unsigned long MOTOR_REVERSE_INTERVAL = 2UL * 1000UL;
unsigned long previousMotorReverseMillis = 0UL;
const byte DEFAULT_MOTOR_SPEED = 100;
boolean drive = false;
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
    rgbLed.setColor(rgbLed_1_IDX, 0, 255, 0);
    rgbLed.setColor(rgbLed_2_IDX, 0, 255, 0);
    rgbLed.show();
  } else if (dir==DIR_BACKWARD) {
    leftSpeed = spd;
    rightSpeed = -spd;
    rgbLed.setColor(rgbLed_1_IDX, 255, 0, 0);
    rgbLed.setColor(rgbLed_2_IDX, 255, 0, 0);
    rgbLed.show();
  }  else if (dir==DIR_LEFT) {
    leftSpeed = -spd;
    rightSpeed = -spd;
    rgbLed.setColor(rgbLed_1_IDX, 0, 255, 0);
    rgbLed.setColor(rgbLed_2_IDX, 255, 0, 0);
    rgbLed.show();
  } else if (dir==DIR_RIGHT) {
    leftSpeed = spd;
    rightSpeed = spd;
    rgbLed.setColor(rgbLed_1_IDX, 255, 0, 0);
    rgbLed.setColor(rgbLed_2_IDX, 0, 255, 0);
    rgbLed.show();
  }
//  motorL.setpin(MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  motorL.run(leftSpeed);
//  motorR.setpin(MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
  motorR.run(rightSpeed);
}
void stopMotors() {
//  motorL.setpin(MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  motorL.stop();
//  motorR.setpin(MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
  motorR.stop();
  rgbLedOff();
  // if (!rgbLed.setColor(255, 255, 0)) {
  //   Serial.println("ERROR: setColor");
  // }
  // rgbLed.show();
}

void setup()
{
  Serial.begin(BAUD);
#ifndef ME_PORT_DEFINED
  ultrasonic.setpin(ULTRASONIC_SENSOR_S1_PIN, ULTRASONIC_SENSOR_S2_PIN);
  pinMode(LIGHTSENSOR_PIN, INPUT);
#endif
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, LOW);
  pinMode(LIGHTSENSOR_PIN, INPUT);
  lightsensor.setpin(8, LIGHTSENSOR_PIN);
  buzzer.setpin(BUZZER_PIN);
  pinMode(RGB_LED_PIN,OUTPUT);
  rgbLed.setpin(RGB_LED_PIN);
  rgbLedOff();
  // if (!rgbLed.setColor(rgbLed_1_IDX, 255, 255, 0)) {
  //   Serial.println("ERROR: setColorAt rgbLed_1_IDX");
  // }
  // if (!rgbLed.setColor(rgbLed_2_IDX, 0, 0, 0)) {
  //     Serial.println("ERROR: setColor rgbLed_2_IDX");
  // }
  // rgbLed.show();
  Serial.println("my-mBot");
}

void loop()
{
  unsigned long currentMillis = millis();
  if (buttonStateChange() && pressed) {
    drive = !drive;
    Serial.print("Drive state is ");
    if (drive) Serial.println("ON");
    else Serial.println("OFF");
  }
  if (drive) {
    if (currentMillis - previousMotorReverseMillis >= MOTOR_REVERSE_INTERVAL) {
      previousMotorReverseMillis = currentMillis;
      if (direction==DIR_FORWARD) {
        direction=DIR_BACKWARD;
        Serial.println("Direction is BACKWARD");
      } else {
        direction=DIR_FORWARD;
        Serial.println("Direction is FORWARD");
      }
      move(direction, DEFAULT_MOTOR_SPEED);
    }
  } else {
    stopMotors();
  }
  if (currentMillis - previousDistanceMeasurementMillis >= DISTANCE_MEASUREMENT_INTERVAL) {
    previousDistanceMeasurementMillis = currentMillis;
    unsigned long distance = ultrasonic.distanceCm();
    Serial.print("Distance : ");
    Serial.print(distance);
    Serial.println(" cm");
    if (distance < 20) {
      buzzerOn();
    }
  }
  if (currentMillis - previousLightSensorMeasurementMillis >= LIGHTSENSOR_MEASUREMENT_INTERVAL) {
    previousLightSensorMeasurementMillis = currentMillis;
  //   // switch off leds before taking a measurement
  //   cRGB rgb_1 = rgbLed.getColorAt(rgbLed_1_IDX);
  //   cRGB rgb_2 = rgbLed.getColorAt(rgbLed_2_IDX);
  //   rgbLedOff();
  //   delay(100);
    int lightlevel = lightsensor.read();
    Serial.print("Light Level : ");
    Serial.println(lightlevel);
  //   // switch leds to previous state
  //   rgbLed.setColorAt(rgbLed_1_IDX, rgb_1.r, rgb_1.g, rgb_1.b);
  //   rgbLed.setColorAt(rgbLed_2_IDX, rgb_2.r, rgb_2.g, rgb_2.b);
  //   rgbLed.show();
  //   // if (lightlevel < 100) {
  //   //   lightsensor.lightOn();
  //   // } else {
  //   //   lightsensor.lightOff();
  //   // }
  }
}
