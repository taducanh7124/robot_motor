#ifndef DATA_HPP
#define DATA_HPP

#include "stdint.h"

// thu vien nguoi dung
#include "MotorControlPID.hpp"
#include "uartDMA.hpp"
#include "pid.h"
// dieu lieu dieu khien nhan qua UART
typedef struct
{
    uint8_t dir;
    uint16_t pwm;
    int16_t velocity;
    float setpoint;
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

typedef struct
{
    uint32_t encoderValue;
    bool dir;
} inputEncoder_t;


typedef struct
{
    double vxSetpoint;
    double vxInput;
    double vxOutput;
} dataDongCo_t;

typedef struct
{
    PID dongCo1;
    PID dongCo2;
    PID dongCo3;
    PID dongCo4;

    // data
    dataDongCo_t data_dongCo1;
    dataDongCo_t data_dongCo2;
    dataDongCo_t data_dongCo3;
    dataDongCo_t data_dongCo4;
} dongCoPIO_t;

typedef struct
{
    inputEncoder_t front_left;
    inputEncoder_t front_right;
    inputEncoder_t rear_left;
    inputEncoder_t rear_right;
} EncoderData_t;


// khai bao toan cuc
extern RobotDrive_t robot;
extern EncoderData_t encoderData;
extern dongCoPIO_t dongCoPID;
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