#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

#include "Motor.h"
typedef enum _state_Typedef
{
  LEG = 0,
  JOINT,
  ANGLE
}state_Typedef;

state_Typedef state = LEG;

  uint16_t leg;
  uint16_t joint;
  float angle;
  char buff[100];

// PCA9685 driver = PCA9685(0x41, PCA9685_MODE_LED_DIRECT, 50);
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
Motor motor;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  motor.init();
  
  // pwm.begin();
  // pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  for(int j = 0; j < 6; j++)
  {
    motor.setAngleDirect(j, 0, -80.0f);
    motor.setAngleDirect(j, 1, 20.0f);
  }
}

void loop() {

  motor.setAngleDirect(2, 1, -80);
  delayMicroseconds(1e6);
  motor.setAngleDirect(2, 0, -20);
  delayMicroseconds(1e6);
  for(int i = 0; i<5; i++)
  {
    motor.setAngleDirect(2, 2, 30);
    delayMicroseconds(1e5);
    motor.setAngleDirect(2, 2, -30);
    delayMicroseconds(1e5);
  }
  for(int i = 0; i<5; i++)
  {
    motor.setAngleDirect(2, 0, -25);
    delayMicroseconds(1e5);
    motor.setAngleDirect(2, 0, 0);
    delayMicroseconds(1e5);
  }
  motor.setAngleDirect(2, 1, 0);
  delayMicroseconds(1e6);

  motor.setAngleDirect(5, 2, 0);
  delayMicroseconds(1e6);
  motor.setAngleDirect(2, 1, -80);
  delayMicroseconds(1e6);
  motor.setAngleDirect(2, 0, -100);
  delayMicroseconds(1e6);
  motor.setAngleDirect(2, 2, 80);
  delayMicroseconds(1e6);
//this is for git!
//forgit about git 
//funnnnnnny


  // motor.setAngleDirect(0, 1, -20);
  // delayMicroseconds(1e6);
  // motor.setAngleDirect(0, 1, 20);
  // motor.setAngleDirect(1, 1, -20);
  // delayMicroseconds(1e6);
  // motor.setAngleDirect(1, 1, 20);
//   motor.setAngleDirect(2, 1, -20);
//   delayMicroseconds(1e6);
//   motor.setAngleDirect(2, 1, 20);
//   motor.setAngleDirect(3, 1, -20);
//   delayMicroseconds(1e5);
//   motor.setAngleDirect(3, 1, 20);
//   motor.setAngleDirect(4, 1, -20);
//   delayMicroseconds(1e5);
//   motor.setAngleDirect(4, 1, 20);
  // motor.setAngleDirect(5, 1, -20);
  // delayMicroseconds(1e5);
  // motor.setAngleDirect(5, 1, 20);


  // switch(state)
  // {
  //   case LEG:
  //   {
  //     while(!Serial.available());
  //     leg = Serial.readBytesUntil('\n', buff, 100);
  //     if (leg)
  //     {
  //       leg = atoi(buff);
  //       Serial.print("Chosen leg: ");
  //       Serial.println(leg);
  //       Serial.println("choose joint: ");
  //       state = (state_Typedef)JOINT;
  //     }
  //     break;
  //   }
  //   case JOINT:
  //   {
  //     while(!Serial.available());
  //     joint = Serial.readBytesUntil('\n', buff, 100);;
  //     if(joint)
  //     {
  //       joint = atoi(buff);
  //       Serial.print("Chosen joint " );
  //       Serial.println(joint);
  //       Serial.println("choose angle: ");
  //       state = (state_Typedef)ANGLE;
  //     }
  //     break;
  //   }
  //   case ANGLE:
  //   {

  //     uint8_t len = Serial.readBytesUntil('\n', buff, 100);
  //     if (len)
  //     {
  //       angle = atof(buff);
  //       Serial.print("dagrees: ");
  //       Serial.println(angle);
  //       motor.setAngleDirect(leg, joint, angle);
  //       state = LEG;
  //       Serial.println("chose leg");
  //     }
  //     break;
  //   }
  // }
}