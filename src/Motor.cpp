#include "Motor.h"
#include <EEPROM.h>
#include <HardwareSerial.h>

//private functions
 bool Motor::isLegalMotor(uint8_t leg, uint8_t motor)
{
    return leg >= 0 && leg < 6 && motor >= 0 && motor < 3;
}

bool Motor::isLegalAngle(float angle)
{
    return angle >= -90.0f && angle <= 90.0f;
}

Motor::Motor(){}

void Motor::init()
{
    rate = 0.0f;

    //initiate PCA9685 driver class
    driver[0] = Adafruit_PWMServoDriver(0x40);
    driver[1] = Adafruit_PWMServoDriver(0x41);

    driver[0].begin();
    driver[1].begin();
    //set frequency to 50Hz
    driver[0].setPWMFreq(50);
    sleep(1);
    driver[1].setPWMFreq(50);


    //read offsets from flash and protect from unwanted values
    for (uint8_t leg = 0; leg < 6; leg++)        //TODO: change magic number
    {
        for(uint8_t joint = 0; joint < 3; joint++)
        {
            offset[leg][joint] = EEPROM.read(EEPROM_OFFSET_ADDR + leg*3 + joint);
            if(isLegalAngle(offset[leg][joint]))
            {
                offset[leg][joint] = 0;
            }
        }
    }

    //set all angles to zero:
    // resetAllMotors();

}

/*
*   @brief sets motor angle
*   
*   @arg leg - leg index [0, 5]
*        motor - motor index relative to leg [0, 2]
*        angle - wanted angle [-90, 90]
*
*   @retval none
*/
void Motor::setAngleDirect(uint8_t leg, uint8_t motor, float angle)
{
    if(isLegalAngle(angle) && isLegalMotor(leg, motor))
    {
        //get pca device index
        uint8_t pcaIdx = (leg > 2);
        //convert angle to duty cycle
        float dc = map(angle, -90.0f, 90.0f, MIN_ANGLE, MAX_ANGLE);
        // uint32_t dc = 300;
        uint8_t driverIdx;
        if (leg > 2) 
        {
            driverIdx = 1;
        }
        else
        {
            driverIdx = 0;
        }
        uint8_t index = motor + leg*3 - driverIdx*9;
        this->driver[driverIdx].setPWM(index, 0, dc);
        this->angle[leg][motor] = angle;
    }
}


/*
*   @brief sets motor angle
*   
*   @arg 
*        motor - motor index relative to leg [0, 17]
*        angle - wanted angle [-180, 180]
*
*   @retval none
*/
void Motor::setAngleDirect(uint8_t motor, float angle)
{
    uint8_t leg = motor % 6;
    uint8_t joint = (uint8_t)(motor * 0.333f);
    
    setAngleDirect(leg, joint, angle);

}

/*
*   @brief get angle 
*   
*   @arg leg - leg index [0, 5]
*        motor - motor index relative to leg [0, 2]
*
*   @retval angle in dagrees
*/
float Motor::getCurrentAngle(uint8_t leg, uint8_t motor)
{
    return angle[leg][motor] - offset[leg][motor];
}

/*
*   @brief get angle 
*   
*   @arg motor - motor index to leg [0, 17]
*
*   @retval angle in dagrees
*/
float Motor::getCurrentAngle(uint8_t motor)
{
    uint8_t leg = motor % 6;
    uint8_t joint = motor / 3;

    return angle[leg][joint] - offset[leg][joint];
}

/*
*   @brief set offset for a specific motor using the current location 
*   
*   @arg leg - leg index [0, 5]
*        motor - motor index relative to leg [0, 2]
*
*   @retval none
*/
void Motor::setOffestSingle(uint8_t leg, uint8_t motor)
{
    if(isLegalMotor(leg, motor))
    {
        offset[leg][motor] = angle[leg][motor];
    }
}

/*
*   @brief set offset for a specific motor manually 
*   
*   @arg leg - leg index [0, 5]
*        motor - motor index relative to leg [0, 2]
*        newOffset - new offset [-180, 180]
*
*   @retval none
*/
void Motor::setOffestSingle(uint8_t leg, uint8_t motor, float newOffset)
{
    if(isLegalMotor(leg, motor) && isLegalAngle(newOffset))
    {
        offset[leg][motor] = newOffset;
    }
}

/*
*   @brief set offset for all motors using the current location  
*   
*
*   @retval none
*/
void Motor::setOffsetAll()
{
    for(int leg = 0; leg < 6; leg++)
    {
        for(int joint = 0; joint < 3; joint ++)
        {
            offset[leg][joint] = angle[leg][joint];
        }
    }
}

/*
*   @brief resets all motor to zero position  
*   
*   @retval none
*/
void Motor::resetAllMotors()
{
    for(int leg = 0; leg < 6; leg++)
    {
        for (int joint = 0; joint < 3; joint++)
        {
            setAngleDirect(leg, joint, 0.0f);
            // sleep(1);
            Serial.print("moving leg ");
            Serial.print(leg);
            Serial.print(" motor: ");
            Serial.println(joint);
        }
    }
}

/*
*   @brief resets all motor to zero position  
*   
*   @retval none
*/
void Motor::saveOffset()
{
    for (uint8_t leg = 0; leg < 6; leg++)
    {
        for(uint8_t joint = 0; joint < 3; joint++)
        {
            EEPROM.write(EEPROM_OFFSET_ADDR + leg*3 + joint, offset[leg][joint]);
        }
    }
}



/*
*   @brief set target angle
*   
*   @retval none
*/
void Motor::setTargerAngle(uint8_t leg, uint8_t motor, float angle)
{
    if( isLegalAngle(angle) && isLegalMotor(leg, motor))
    {
        targetAngle[leg][motor] = angle;
    }
}

/*
*   @brief set target angle
*   
*   @retval none
*/
void Motor::setTargerAngle(uint8_t motor, float angle)
{
    uint8_t leg = motor % 6;
    uint8_t joint = motor / 3;
    setTargerAngle(leg, joint, angle);
}

/*
*   @brief set the rate of movement [dagrees/sec]
*   
*   @retval none
*/
void Motor::setRate(float rate)
{
    this->rate = rate * Ts;
}

/*
*   @brief gradient the angles
*   
*   @retval none
*/
float Motor::ramp(float current, float target)
{
    float retval = 0.0f;
    if(abs(target - current) < rate) retval = target;
    
    else if(target > current) retval = current + rate;

    else retval = current - rate;

    return retval;
    
}

void Motor::motor_iter()
{
    for (int leg = 0; leg < 6; leg++)
    {
        for(int joint = 0; joint < 3; joint ++)
        {
            float angle= ramp(angle, targetAngle[leg][joint]);
            setAngleDirect(leg, joint, angle);
        }
    }
}