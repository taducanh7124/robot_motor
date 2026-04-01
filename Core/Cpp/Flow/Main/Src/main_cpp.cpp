#include "main_cpp.hpp"
#include "config.hpp"
#include "input.hpp"
#include "output.hpp"
#include "ledmain.hpp"

void motorControl()
{
    if (robot.state.isControlNew)
    {
        robot.state.isControlNew = false;
        frontLeftMotor.control(robot.front_left.pwm, (MotorDir)robot.front_left.dir);
        frontRightMotor.control(robot.front_right.pwm, (MotorDir)robot.front_right.dir);
        rearLeftMotor.control(robot.rear_left.pwm, (MotorDir)robot.rear_left.dir);
        rearRightMotor.control(robot.rear_right.pwm, (MotorDir)robot.rear_right.dir);
    }
    else
    {
    }
}

void test()
{
    struct
    {
        PinConfig_out d1, d2, d3, d4;
    } dri_t;

    while (1)
    {
        static uint32_t timeCurent = 0;
        if (HAL_GetTick() - timeCurent >= 1000)
        {
            timeCurent = HAL_GetTick();
            dri_t.d1.high();
            dri_t.d2.high();
            dri_t.d3.high();
            dri_t.d4.high();
            HAL_Delay(100);
            dri_t.d1.low();
            dri_t.d2.low();
            dri_t.d3.low();
            dri_t.d4.low();

            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

void main_cpp()
{
    frontLeftMotor.init(&htim3, TIM_CHANNEL_1, DIR_1_GPIO_Port, DIR_1_Pin);
    frontRightMotor.init(&htim3, TIM_CHANNEL_2, DIR_2_GPIO_Port, DIR_2_Pin);
    rearLeftMotor.init(&htim3, TIM_CHANNEL_3, DIR_3_GPIO_Port, DIR_3_Pin);
    rearRightMotor.init(&htim3, TIM_CHANNEL_4, DIR_4_GPIO_Port, DIR_4_Pin);

    // uart
    uartDriver.init(&huart1, bufferUART, sizeof(bufferUART));

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // start watch dog
    MX_IWDG_Init();

    // test();

        while (1)
    {
        blinkLed();
        // xu ly input
        processData();

        // dieu khien dong co
        motorControl();

        // phan hoi (neu can)
        respond();

        // static uint32_t test = 0;
        // if (HAL_GetTick() - test >= 1000)
        // {
        //     test = HAL_GetTick();
        //     DIR_1_GPIO_Port->ODR ^= DIR_1_Pin;
        //     DIR_2_GPIO_Port->ODR ^= DIR_2_Pin;
        //     DIR_3_GPIO_Port->ODR ^= DIR_3_Pin;
        //     DIR_4_GPIO_Port->ODR ^= DIR_4_Pin;
        // }

        // refresh watch dog
        HAL_IWDG_Refresh(&hiwdg);
    }
}