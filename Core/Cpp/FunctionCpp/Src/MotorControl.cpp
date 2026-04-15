#include "MotorControl.hpp"

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
        .isDataNew     = 0, // 0 = chua co du lieu moi de nhan, 1 = da co du lieu moi tu pi
        .isSendDataNew = 1, // 0 = khong co du lieu moi de gui, 1 = co du lieu moi de gui cho pi
        .isControlNew  = 0 
    },

    .motor_front_left = {
        .dir   = 0,
        .ccrHT = 0,
        .ccrTL = 0,
        .vanToc = 0.0f
    },
    .motor_front_right = {
        .dir   = 0,
        .ccrHT = 0,
        .ccrTL = 0,
        .vanToc = 0.0f
    },
    .motor_rear_left = {
        .dir   = 0,
        .ccrHT = 0,
        .ccrTL = 0,
        .vanToc = 0.0f
    },
    .motor_rear_right = {
        .dir   = 0,
        .ccrHT = 0,
        .ccrTL = 0,
        .vanToc = 0.0f
    }
};