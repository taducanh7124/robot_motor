#include "data.hpp"

RobotDrive_t robot = {
    .front_left  = { .dir = 0, .pwm = 0 },
    .front_right = { .dir = 0, .pwm = 0 },
    .rear_left   = { .dir = 0, .pwm = 0 },
    .rear_right  = { .dir = 0, .pwm = 0 }
};

// khai bao
motorControl frontLeftMotor;
motorControl frontRightMotor;
motorControl rearLeftMotor;
motorControl rearRightMotor;

EncoderData_t encoderData;
dongCoPIO_t dongCoPID;

uint8_t bufferUART[200];
UartDma uartDriver;

response_t response;
