#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


/*
*           leg index - note the front is where the battery opening
*         1  __||__ 4
*         2  __||__ 5
*         3  __||__ 6
*
*
*
*
*
*/
class Motor
{
    private:
    //
     Adafruit_PWMServoDriver    driver[2];           // main PCA9865 driver
     float                      angle[6][3];         //holds angle position
     float                      targetAngle[6][3];   //used for smoothing motion
     uint8_t                    offset[6][3];        //offset array

     float                      rate;                //dagrees per second
     float                      Ts;                  // interrupt time
    
      bool                      _initiated;
     static constexpr uint32_t         EEPROM_OFFSET_ADDR = 0x0;
     static constexpr uint16_t         MAX_ANGLE = 125;
     static constexpr uint16_t         MID_ANGLE = 302;
     static constexpr uint16_t         MIN_ANGLE = 480;

     bool isLegalMotor(uint8_t leg, uint8_t motor);
     bool isLegalAngle(float angle);
     float ramp(float current, float target);
    public:
    //
     Motor();
     void init();
     void setAngleDirect(uint8_t leg, uint8_t motor, float angle);
     void setAngleDirect(uint8_t motor, float angle);
     void setOffestSingle(uint8_t leg, uint8_t motor);
     void setOffestSingle(uint8_t leg, uint8_t motor, float offset);
     void setOffsetAll();
     void resetAllMotors();
     void saveOffset();


     void setTargerAngle(uint8_t leg, uint8_t motor, float angle);
     void setTargerAngle(uint8_t motor, float angle);
     void setRate(float rate);
     void motor_iter();

    // get functions
     float getCurrentAngle(uint8_t leg, uint8_t motor);
     float getCurrentAngle( uint8_t motor);
};

class AdvancedMotor : public Motor
{
    private:
     float currentAngle[6][3];
     float targetAngle[6][3];
     float rate;
    

    float ramp(float current, float target, float rate);

    public:
    AdvancedMotor();
    AdvancedMotor(float rate);
    void setAngle(uint8_t leg, uint8_t motor, float angle);
    void setAngle(uint8_t motor, float angle);
    float getCurrentAngle(uint8_t leg, uint8_t motor);
    float getCurrentAngle(uint8_t motor);

    void motor_iter();
};

