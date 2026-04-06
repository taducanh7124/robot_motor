#include "MotorControl.hpp"

void motorControl::init(TIM_HandleTypeDef *htim, uint32_t timerChanel, GPIO_TypeDef *dirPort, uint16_t dirPin)
{
    // lay con tro htime
    this->htimPWM = htim;
    this->timerChanel = timerChanel;

    // khoi tao chan dieu khien
    this->dir.init(dirPort, dirPin);

    switch (this->timerChanel)
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
    HAL_TIM_PWM_Start(this->htimPWM, this->timerChanel);
    this->stop();
}

void motorControl::control(uint16_t speed, MotorDir dir)
{
  debug = speed;
    // kiem tra toc do
    if (speed > 0 && speed <= this->ARR)
    {
        // toc do hop le
        if (dir == MotorDir::Backward)
        {
            // chay tien
            this->dir.low(); // Set bit để bật
        }
        else if (dir == MotorDir::Forward)
        {
            // chay lui
            this->dir.high(); // Set bit để tắt
        }

        // toc do PWM
        *this->CCR = speed;
        /**
         * thoi gian xung 1 = (CCR / ARR) * 100 = ?%
         */

        // CNT la thanh ghi dem so sanh voi CCR va ARR
    }
    else
    {
        // dieu khien sai => treo chuong trinh
        while (1)
        {
        }
    }
}

void motorControl::stop()
{
    *this->CCR = 0;
}

// float motorControl::PID_Compute(PID_Controller_t *pid, float current_velocity) 
// {
//     // 1. Tính sai số
//     float error = pid->setpoint - current_velocity;

//     // 2. Tính khâu P
//     float P_out = pid->Kp * error;

//     // 3. Tính khâu I (Có chống Wind-up - cực kỳ quan trọng)
//     pid->error_sum += (pid->Ki * error);
//     // Chống tràn khâu I (Anti-windup) để khi kẹt bánh motor không bị rồ lên
//     if (pid->error_sum > pid->out_max) pid->error_sum = pid->out_max;
//     else if (pid->error_sum < pid->out_min) pid->error_sum = pid->out_min;
    
//     float I_out = pid->error_sum;

//     // 4. Tính khâu D
//     float D_out = pid->Kd * (error - pid->prev_error);
//     pid->prev_error = error; // Lưu lại cho lần sau

//     // 5. Tổng hợp ngõ ra
//     float output = P_out + I_out + D_out;

//     // 6. Giới hạn ngõ ra trong khoảng cấu hình PWM của Timer (VD: -1000 đến 1000)
//     if (output > pid->out_max) output = pid->out_max;
//     else if (output < pid->out_min) output = pid->out_min;

//     return output;
// }
