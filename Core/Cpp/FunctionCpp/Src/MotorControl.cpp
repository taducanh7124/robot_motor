#include "MotorControl.hpp"
#include <math.h>
#include "const.hpp"

// Khoi tao dong co robot
MotorControl MotorCtr_FL;
MotorControl MotorCtr_FR;
MotorControl MotorCtr_RL;
MotorControl MotorCtr_RR;

// Ham khoi tao dong co robot
void MotorControl::init(TIM_HandleTypeDef *htimPWM, uint32_t timerChannel, GPIO_TypeDef *dirPort, uint16_t dirPin)
{
    // lay con tro htime
    this->htimPWM = htimPWM;
    this->timerChannel = timerChannel;

    // khoi tao chan dieu khien
    this->dir.init(dirPort, dirPin);

    switch (this->timerChannel)
    {
    case TIM_CHANNEL_1:
        CCR = &(this->htimPWM->Instance->CCR1);
        break;
    case TIM_CHANNEL_2:
        CCR = &(this->htimPWM->Instance->CCR2);
        break;
    case TIM_CHANNEL_3:
        CCR = &(this->htimPWM->Instance->CCR3);
        break;
    case TIM_CHANNEL_4:
        CCR = &(this->htimPWM->Instance->CCR4);
        break;
    default:
        CCR = nullptr;
        while (1)
        {
            /* khong the khoi tao */
        }
        break;
    }

    this->ARR = this->htimPWM->Instance->ARR;

    // bat bam xung PWM
    HAL_TIM_PWM_Start(this->htimPWM, this->timerChannel);
    this->stop();
}

// Ham dieu khien dong co robot
void MotorControl::control(uint16_t speed, MotorDir dir)
{
    // kiem tra toc do
    if (speed >= 0 && speed <= this->ARR)
    {
        // toc do hop le
        if (dir == MotorDir::Backward)
        {
            this->dir.low(); // Chay lui reset bit để tat
        }
        else if (dir == MotorDir::Forward)
        {
            this->dir.high(); // Chay tien set bit để bật
        }

        // toc do PWM
        *this->CCR = speed;
        // CNT la thanh ghi dem so sanh voi CCR va ARR
    }
    else
    {
        while (1)
        {
            // dieu khien sai => treo chuong trinh
        }
    }
}

// Ham dung dong co
void MotorControl::stop()
{
    *this->CCR = 0;
}

// Khoi tao doi tuong dieu khien robot
RobotDrive_t robot = {
    .state = {
        .isDebugMode = 0,   // 0 = dang chay binh thuong, 1 = dang chay debug
        .isDataNew = 0,     // 0 = chua co du lieu moi de nhan, 1 = da co du lieu moi tu pi
        .isSendDataNew = 1, // 0 = khong co du lieu moi de gui, 1 = co du lieu moi de gui cho pi
        .isControlNew = 0},

    .motor_front_left = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_front_right = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_rear_left = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_rear_right = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f}};

// Hàm tăng tốc độ từ từ motor
void controlOnDinh()
{
    // Tính muc xung ccr tu tu cho tung banh
    // Bên trái trước
    robot.motor_front_left.ccrHT += ALPHA * (robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT);
    if (fabsf(robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT) < delta_ccr)
    {
        robot.motor_front_left.ccrHT = robot.motor_front_left.ccrTL;
    }
    // Bên trái sau
    robot.motor_rear_left.ccrHT += ALPHA * (robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT);
    if (fabsf(robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT) < delta_ccr)
    {
        robot.motor_rear_left.ccrHT = robot.motor_rear_left.ccrTL;
    }

    // Bên phải trước
    robot.motor_front_right.ccrHT += ALPHA * (robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT);
    if (fabsf(robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT) < delta_ccr)
    {
        robot.motor_front_right.ccrHT = robot.motor_front_right.ccrTL;
    }
    // Bên phải sau
    robot.motor_rear_right.ccrHT += ALPHA * (robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT);
    if (fabsf(robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT) < delta_ccr)
    {
        robot.motor_rear_right.ccrHT = robot.motor_rear_right.ccrTL;
    }

    // Tinh huong cua tung banh Dựa trên dấu của ccrHT hiện tại
    // Ben trai
    robot.motor_front_left.dir = (robot.motor_front_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    robot.motor_rear_left.dir = (robot.motor_rear_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    // Ben phai
    robot.motor_front_right.dir = (robot.motor_front_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);
    robot.motor_rear_right.dir = (robot.motor_rear_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);

    // ĐIỀU KHIỂN MOTOR THẬT
    MotorCtr_FL.control((uint16_t)fabsf(robot.motor_front_left.ccrHT), static_cast<MotorDir>(robot.motor_front_left.dir));
    MotorCtr_FR.control((uint16_t)fabsf(robot.motor_front_right.ccrHT), static_cast<MotorDir>(robot.motor_front_right.dir));
    MotorCtr_RL.control((uint16_t)fabsf(robot.motor_rear_left.ccrHT), static_cast<MotorDir>(robot.motor_rear_left.dir));
    MotorCtr_RR.control((uint16_t)fabsf(robot.motor_rear_right.ccrHT), static_cast<MotorDir>(robot.motor_rear_right.dir));
}

// Ham dung dong co tu tu
void dungMotor()
{
    robot.motor_front_left.ccrTL = 0;
    robot.motor_front_right.ccrTL = 0;
    robot.motor_rear_left.ccrTL = 0;
    robot.motor_rear_right.ccrTL = 0;
}