#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL

#include "time.h"
#include "main.h"
#include "pin_config.hpp"

enum class MotorDir : uint8_t {
    Forward = 0,
    Backward = 1
};

class motorControl
{
private:
    // tao xung PWM va dieu khien huong di
    TIM_HandleTypeDef *htimPWM;
    uint32_t timerChanel;
    PinConfig_out dir;
    volatile uint32_t *CCR;
    volatile uint32_t ARR;

    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

public:
    void init(TIM_HandleTypeDef *htim, uint32_t timerChanel, GPIO_TypeDef* dirPort, uint16_t dirPin);
    void control(uint16_t speed, MotorDir dir);
    void stop();
};
#endif