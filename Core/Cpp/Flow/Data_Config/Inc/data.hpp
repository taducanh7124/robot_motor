#ifndef DATA_HPP
#define DATA_HPP

#include "stdint.h"
#include "config.hpp"

// dieu lieu dieu khien nhan qua UART
typedef struct
{
    uint8_t dir;
    uint16_t pwm;
} dataControl_t;

typedef struct
{
    uint8_t isDataNew;
    uint8_t isResponseNew;
    uint8_t isControlNew;
} state_t;


typedef struct
{
    state_t state;
    dataControl_t front_left;
    dataControl_t front_right;
    dataControl_t rear_left;
    dataControl_t rear_right;
} RobotDrive_t;

// phan hoi (neu can)
typedef struct
{
    uint8_t content[200];
} response_t;

// khai bao toan cuc
extern RobotDrive_t robot;

extern motorControl frontLeftMotor;
extern motorControl frontRightMotor;
extern motorControl rearLeftMotor;
extern motorControl rearRightMotor;

// buffer nhan UART
extern uint8_t bufferUART[200];
extern UartDma uartDriver;

// phan hoi
extern response_t response;

#endif