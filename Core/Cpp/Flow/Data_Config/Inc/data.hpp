#ifndef DATA_HPP
#define DATA_HPP

#include "stdint.h"

// thu vien nguoi dung
#include "MotorControl.hpp"
#include "uartDMA.hpp"

// dieu lieu dieu khien nhan qua UART
typedef struct
{
    uint8_t dir;
    uint16_t pwm;
    uint16_t velocity;
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
    inputEncoder_t front_left;
    inputEncoder_t front_right;
    inputEncoder_t rear_left;
    inputEncoder_t rear_right;
} EncoderData_t;

typedef struct {
    float Kp, Ki, Kd;       // Các hệ số
    float setpoint;         // Vận tốc mong muốn (xung/10ms)
    float error_sum;        // Khâu tích phân (I) cộng dồn
    float prev_error;       // Sai số quá khứ để tính Vi phân (D)
    float out_max;          // Giới hạn PWM lớn nhất (VD: 1000)
    float out_min;          // Giới hạn PWM nhỏ nhất (VD: -1000)
} PID_Controller_t;


// khai bao toan cuc
extern RobotDrive_t robot;
extern EncoderData_t encoderData;

extern motorControl frontLeftMotor;
extern motorControl frontRightMotor;
extern motorControl rearLeftMotor;
extern motorControl rearRightMotor;

// Khai báo 4 bộ PID cho 4 bánh
extern PID_Controller_t pid_FL, pid_FR, pid_RL, pid_RR;

// buffer nhan UART
extern uint8_t bufferUART[200];
extern UartDma uartDriver;

// phan hoi
extern response_t response;

#endif