#include "main_cpp.hpp"
#include "config.hpp"
#include "math.h"
#include "QMC5883LCompass.h"

#define MAX_PWM_ARR 255
#define WHEEL_BASE 0.03 // Khoảng cách giữa 2 bánh xe trái-phải (tính bằng mét, thay bằng thông số thực)
#define METERS_PER_PULSE 0.0006
#define DELTA_T 0.01 // 10ms

double input_pos_front_left = 0.0;  // Vị trí hiện tại (mét)
double input_pos_front_right = 0.0; // Vị trí hiện tại (mét)
double input_pos_rear_left = 0.0;   // Vị trí hiện tại (mét)
double input_pos_rear_right = 0.0;  // Vị trí hiện tại (mét)

double output_pwm_front_left = 0.0;  // Giá trị PWM xuất ra (-ARR đến +ARR)
double output_pwm_front_right = 0.0; // Giá trị PWM xuất ra (-ARR đến +ARR)
double output_pwm_rear_left = 0.0;   // Giá trị PWM xuất ra (-ARR đến +ARR)
double output_pwm_rear_right = 0.0;  // Giá trị PWM xuất ra (-ARR đến +ARR)

double setpoint_pos_front_left = 0.0;  // Vị trí mục tiêu (mét)
double setpoint_pos_front_right = 0.0; // Vị trí mục tiêu (mét)
double setpoint_pos_rear_left = 0.0;   // Vị trí mục tiêu (mét)
double setpoint_pos_rear_right = 0.0;  // Vị trí mục tiêu (mét)

// PID posPID_FrontLeft(&input_pos_front_left, &output_pwm_front_left, &setpoint_pos_front_left, 150.0, 5.0, 10.0, DIRECT);
PID posPID_FrontRight(&input_pos_front_right, &output_pwm_front_right, &setpoint_pos_front_right, 150.0, 5.0, 10.0, DIRECT);
PID posPID_RearLeft(&input_pos_rear_left, &output_pwm_rear_left, &setpoint_pos_rear_left, 150.0, 5.0, 10.0, DIRECT);
PID posPID_RearRight(&input_pos_rear_right, &output_pwm_rear_right, &setpoint_pos_rear_right, 150.0, 5.0, 10.0, DIRECT);

double kp = 15, ki = 5, kd = 10;
PID posPID_FrontLeft(&input_pos_front_left, &output_pwm_front_left, &setpoint_pos_front_left, kp, ki, kd, DIRECT);

QMC5883LCompass compass;

void setupPID()
{
    posPID_FrontLeft.SetMode(AUTOMATIC);
    posPID_FrontLeft.SetOutputLimits(-MAX_PWM_ARR, MAX_PWM_ARR);
    posPID_FrontLeft.SetSampleTime(10); // 10ms

    posPID_FrontRight.SetMode(AUTOMATIC);
    posPID_FrontRight.SetOutputLimits(-MAX_PWM_ARR, MAX_PWM_ARR);
    posPID_FrontRight.SetSampleTime(10); // 10ms

    posPID_RearLeft.SetMode(AUTOMATIC);
    posPID_RearLeft.SetOutputLimits(-MAX_PWM_ARR, MAX_PWM_ARR);
    posPID_RearLeft.SetSampleTime(10); // 10ms

    posPID_RearRight.SetMode(AUTOMATIC);
    posPID_RearRight.SetOutputLimits(-MAX_PWM_ARR, MAX_PWM_ARR);
    posPID_RearRight.SetSampleTime(10); // 10ms
}

// debug
// void encoder()
// {
//     // cap nhat moi 1ms
//     if (HAL_GetTick() % 1 == 0)
//     {
//     }
//     else
//     {
//         return;
//     }
//     encoderData.front_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim1);
//     encoderData.front_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
//     encoderData.rear_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim3);
//     encoderData.rear_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim4);

//     encoderData.front_left.dir = TIM1->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.front_right.dir = TIM2->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.rear_left.dir = TIM3->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.rear_right.dir = TIM4->CR1 & TIM_CR1_DIR ? 1 : 0;
// }

// di chuyen theo toa do
// void Robot_Control_Loop()
// {
//     static int16_t prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;
//     if (robot.state.isControlNew)
//     {
//         robot.state.isControlNew = false;
//         // ben trai dao chieu
//         setpoint_pos_front_left = robot.front_left.setpoint;
//         setpoint_pos_front_right = robot.front_right.setpoint;
//         setpoint_pos_rear_left = robot.rear_left.setpoint;
//         setpoint_pos_rear_right = robot.rear_right.setpoint;
//     }
//     else
//     {
//     }

//     uint16_t current_FR = __HAL_TIM_GET_COUNTER(&htim2);
//     uint16_t current_RR = __HAL_TIM_GET_COUNTER(&htim3);
//     uint16_t current_RL = __HAL_TIM_GET_COUNTER(&htim4);
//     uint16_t current_FL = __HAL_TIM_GET_COUNTER(&htim1);

//     // robot.front_left.velocity = (int16_t)(current_FL - prev_FL);
//     // robot.front_right.velocity = (int16_t)(current_FR - prev_FR);
//     // robot.rear_left.velocity = (int16_t)(current_RL - prev_RL);
//     // robot.rear_right.velocity = (int16_t)(current_RR - prev_RR);

//     robot.front_left.velocity = (int16_t)((int16_t)prev_FL - (int16_t)current_FL);
//     robot.front_right.velocity = (int16_t)((int16_t)prev_FR - (int16_t)current_FR);
//     robot.rear_left.velocity = (int16_t)((int16_t)prev_RL - (int16_t)current_RL);
//     robot.rear_right.velocity = (int16_t)((int16_t)prev_RR - (int32_t)current_RR);

//     // input_pos_front_left = (double)(encoderData.front_left.encoderValue) * METERS_PER_PULSE;
//     // input_pos_front_right = (double)(encoderData.front_right.encoderValue) * METERS_PER_PULSE;
//     // input_pos_rear_left = (double)(encoderData.rear_left.encoderValue) * METERS_PER_PULSE;
//     // input_pos_rear_right = (double)(encoderData.rear_right.encoderValue) * METERS_PER_PULSE;

//     int16_t delta_FL = (int16_t)(current_FL - prev_FL);
//     int16_t delta_FR = (int16_t)(current_FR - prev_FR);
//     int16_t delta_RL = (int16_t)(current_RL - prev_RL);
//     int16_t delta_RR = (int16_t)(current_RR - prev_RR);

//     prev_FL = current_FL;
//     prev_FR = current_FR;
//     prev_RL = current_RL;
//     prev_RR = current_RR;

//     // Công thức: v = (xung * quy_đổi) / thời_gian
//     input_pos_front_left = ((double)delta_FL * METERS_PER_PULSE) / DELTA_T;
//     input_pos_front_right = ((double)delta_FR * METERS_PER_PULSE) / DELTA_T;
//     input_pos_rear_left = ((double)delta_RL * METERS_PER_PULSE) / DELTA_T;
//     input_pos_rear_right = ((double)delta_RR * METERS_PER_PULSE) / DELTA_T;

//     if (posPID_FrontLeft.Compute())
//     {
//          uint16_t left_pwm = (uint16_t)fabs(output_pwm_front_left);
//         MotorDir dir = (output_pwm_front_left >= 0) ? MotorDir::Forward : MotorDir::Backward;
//         frontLeftMotor.control(left_pwm, dir);
//     }

//     if (posPID_FrontRight.Compute())
//     {
//          uint16_t right_pwm = (uint16_t)fabs(output_pwm_front_right);
//         MotorDir dir = (output_pwm_front_right >= 0) ? MotorDir::Forward : MotorDir::Backward;
//         frontRightMotor.control(right_pwm, dir);
//     }

//     if (posPID_RearLeft.Compute())
//     {
//         uint16_t left_pwm = (uint16_t)fabs(output_pwm_rear_left);
//         MotorDir dir = (output_pwm_rear_left >= 0) ? MotorDir::Forward : MotorDir::Backward;
//         rearLeftMotor.control(left_pwm, dir);
//     }

//     if (posPID_RearRight.Compute())
//     {
//         uint16_t right_pwm = (uint16_t)fabs(output_pwm_rear_right);
//         MotorDir dir = (output_pwm_rear_right >= 0) ? MotorDir::Forward : MotorDir::Backward;
//         rearRightMotor.control(right_pwm, dir);
//     }
// }

void Robot_Control_Loop()
{
    static int16_t prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

    if (robot.state.isControlNew)
    {
        robot.state.isControlNew = false;
        setpoint_pos_front_left = robot.front_left.setpoint;
        setpoint_pos_front_right = robot.front_right.setpoint;
        setpoint_pos_rear_left = robot.rear_left.setpoint;
        setpoint_pos_rear_right = robot.rear_right.setpoint;
    }

    uint16_t current_FR = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t current_RR = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t current_RL = __HAL_TIM_GET_COUNTER(&htim4);
    uint16_t current_FL = __HAL_TIM_GET_COUNTER(&htim1);

    // Tính delta dựa trên mốc prev của 10ms trước đó
    int16_t delta_FL = (int16_t)(current_FL - prev_FL);
    int16_t delta_FR = (int16_t)(current_FR - prev_FR);
    int16_t delta_RL = (int16_t)(current_RL - prev_RL);
    int16_t delta_RR = (int16_t)(current_RR - prev_RR);

    // XÓA 4 DÒNG prev = current Ở ĐÂY! Không được để ở ngoài này.

    // Công thức: v = (xung * quy_đổi) / thời_gian
    // input_pos_front_left = ((double)delta_FL * METERS_PER_PULSE) / DELTA_T;
    // input_pos_front_right = ((double)delta_FR * METERS_PER_PULSE) / DELTA_T;
    // input_pos_rear_left = ((double)delta_RL * METERS_PER_PULSE) / DELTA_T;
    // input_pos_rear_right = ((double)delta_RR * METERS_PER_PULSE) / DELTA_T;
    input_pos_front_left = __HAL_TIM_GET_COUNTER(&htim1);
    input_pos_front_right = __HAL_TIM_GET_COUNTER(&htim2);
    input_pos_rear_left = __HAL_TIM_GET_COUNTER(&htim4);
    input_pos_rear_right = __HAL_TIM_GET_COUNTER(&htim3);

    prev_FL = current_FL;
    prev_FR = current_FR;
    prev_RL = current_RL;
    prev_RR = current_RR;

    // --- CẬP NHẬT PID VÀ CHỐT ENCODER ---

    if (posPID_FrontLeft.Compute())
    {
        uint16_t left_pwm = (uint16_t)fabs(output_pwm_front_left);
        MotorDir dir = (output_pwm_front_left >= 0) ? MotorDir::Forward : MotorDir::Backward;
        frontLeftMotor.control(left_pwm, dir);
    }

    if (posPID_FrontRight.Compute())
    {
        uint16_t right_pwm = (uint16_t)fabs(output_pwm_front_right);
        MotorDir dir = (output_pwm_front_right >= 0) ? MotorDir::Forward : MotorDir::Backward;
        frontRightMotor.control(right_pwm, dir);
    }

    if (posPID_RearLeft.Compute())
    {
        uint16_t left_pwm = (uint16_t)fabs(output_pwm_rear_left);
        MotorDir dir = (output_pwm_rear_left >= 0) ? MotorDir::Forward : MotorDir::Backward;
        rearLeftMotor.control(left_pwm, dir);
    }

    if (posPID_RearRight.Compute())
    {
        uint16_t right_pwm = (uint16_t)fabs(output_pwm_rear_right);
        MotorDir dir = (output_pwm_rear_right >= 0) ? MotorDir::Forward : MotorDir::Backward;
        rearRightMotor.control(right_pwm, dir);
    }
}

// // di chuyen theo van toc
// void consumer()
// {
//     if (robot.state.isControlNew)
//     {
//         robot.state.isControlNew = false;
//         // ben trai dao chieu
//         setpoint_pos_front_left = robot.front_left.setpoint;
//         setpoint_pos_front_right = robot.front_right.setpoint;
//         setpoint_pos_rear_left = robot.rear_left.setpoint;
//         setpoint_pos_rear_right = robot.rear_right.setpoint;
//     }
//     else
//     {
//     }
// }
uint16_t left_pwm, right_pwm;
MotorDir left_dir;
MotorDir right_dir;
uint8_t khoiDong = false;

void main_cpp()
{
    compass.init(SCL_GPIO_Port, SCL_Pin, SDA_GPIO_Port, SDA_Pin);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    frontLeftMotor.init(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin);
    frontRightMotor.init(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin);
    rearRightMotor.init(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin);
    rearLeftMotor.init(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin);

    uartDriver.init(&huart6, bufferUART, sizeof(bufferUART));

    // hieu chinh la ban
    // thong bao hieu chinh (nhay led nhanh 6 lan)
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
    HAL_Delay(300);

   // compass.calibrate();
    MX_IWDG_Init();
    int16_t encLastTime = 0;

    setupPID();
    // VÒNG LẶP CHÍNH
    while (1)
    {
        // Nháy LED báo trạng thái sống
        blinkLed();

        // xu ly input
        processData();

        // PID
        Robot_Control_Loop();

        // // tieu thu data tu ROS
        // consumer();

        // // debug
        // encoder();

        // phan hoi (neu can)
        // respond();
        //        configPID();

        if (khoiDong == 1)
        {
            khoiDong = 0;
            //            frontLeftMotor.control(left_pwm, left_dir);
            //            rearLeftMotor.control(left_pwm, left_dir);
            //
            //            rearRightMotor.control(right_pwm, right_dir);
            //            frontRightMotor.control(right_pwm, right_dir);
        }

        // Làm tươi Watchdog
        HAL_IWDG_Refresh(&hiwdg);

        static uint32_t test = 0;
        if (HAL_GetTick() - test >= 1)
        {
            static int16_t a;
            test = HAL_GetTick();
            compass.read();
            a = compass.getAzimuth();
        }
    }
}