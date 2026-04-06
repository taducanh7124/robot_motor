#include "main_cpp.hpp"
#include "config.hpp"
#include "math.h"

#define METERS_PER_PULSE 0.0006f // Ví dụ: 0.0006 mét cho mỗi xung (tùy thuộc vào encoder và bánh xe)
#define DELTA_T 0.01f            // 10ms

QMC5883LCompass compass;

void Robot_Control_Loop()
{
    static int16_t prev_FL = 0, prev_FR = 0, prev_RR = 0, prev_RL = 0;

    // 1. Cập nhật Setpoint khi có lệnh điều khiển mới
    // (Giả sử robot.front_left.setpoint vẫn đang lấy từ struct trạng thái cũ)
    // Bạn cần điều hướng dữ liệu Setpoint vào đúng biến của struct mới
    if (robot.state.isControlNew) // Chú ý: bạn cần điều chỉnh biến state này cho phù hợp
    {
        robot.state.isControlNew = false;
        dongCoPID.data_dongCo1.vxSetpoint = robot.front_left.setpoint;
        dongCoPID.data_dongCo2.vxSetpoint = robot.front_right.setpoint;
        dongCoPID.data_dongCo3.vxSetpoint = robot.rear_right.setpoint;
        dongCoPID.data_dongCo4.vxSetpoint = robot.rear_left.setpoint;
    }

    // 2. Đọc giá trị Encoder hiện tại
    uint16_t current_FL = __HAL_TIM_GET_COUNTER(&htim1);
    uint16_t current_FR = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t current_RR = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t current_RL = __HAL_TIM_GET_COUNTER(&htim4);

    // 3. Tính số xung thay đổi (Delta)
    int16_t delta_FL = (int16_t)(current_FL - prev_FL);
    int16_t delta_FR = (int16_t)(current_FR - prev_FR);
    int16_t delta_RR = (int16_t)(current_RR - prev_RR);
    int16_t delta_RL = (int16_t)(current_RL - prev_RL);

    // 4. Quy đổi ra vận tốc (m/s) và nạp vào biến Input của PID
    dongCoPID.data_dongCo1.vxInput = ((double)delta_FL * METERS_PER_PULSE) / DELTA_T;
    dongCoPID.data_dongCo2.vxInput = ((double)delta_FR * METERS_PER_PULSE) / DELTA_T;
    dongCoPID.data_dongCo3.vxInput = ((double)delta_RR * METERS_PER_PULSE) / DELTA_T;
    dongCoPID.data_dongCo4.vxInput = ((double)delta_RL * METERS_PER_PULSE) / DELTA_T;

    prev_RL = current_RL;
    prev_FR = current_FR;
    prev_RR = current_RR;
    prev_FL = current_FL; // Chỉ cập nhật prev khi đủ 10ms

    // 5. Tính toán PID và xuất xung điều khiển

    // --- ĐỘNG CƠ 1: FRONT LEFT ---
    if (dongCoPID.dongCo1.Compute())
    {

        uint16_t pwm = (uint16_t)fabs(dongCoPID.data_dongCo1.vxOutput);
        MotorDir dir = (dongCoPID.data_dongCo1.vxOutput >= 0) ? MotorDir::Forward : MotorDir::Backward;
        frontLeftMotor.control(pwm, dir);
    }

    // --- ĐỘNG CƠ 2: FRONT RIGHT ---
    if (dongCoPID.dongCo2.Compute())
    {

        uint16_t pwm = (uint16_t)fabs(dongCoPID.data_dongCo2.vxOutput);
        MotorDir dir = (dongCoPID.data_dongCo2.vxOutput >= 0) ? MotorDir::Forward : MotorDir::Backward;
        frontRightMotor.control(pwm, dir);
    }

    // --- ĐỘNG CƠ 3: REAR RIGHT ---
    if (dongCoPID.dongCo3.Compute())
    {
        uint16_t pwm = (uint16_t)fabs(dongCoPID.data_dongCo3.vxOutput);
        MotorDir dir = (dongCoPID.data_dongCo3.vxOutput >= 0) ? MotorDir::Forward : MotorDir::Backward;
        rearRightMotor.control(pwm, dir);
    }

    // --- ĐỘNG CƠ 4: REAR LEFT ---
    if (dongCoPID.dongCo4.Compute())
    {
        uint16_t pwm = (uint16_t)fabs(dongCoPID.data_dongCo4.vxOutput);
        MotorDir dir = (dongCoPID.data_dongCo4.vxOutput >= 0) ? MotorDir::Forward : MotorDir::Backward;
        rearLeftMotor.control(pwm, dir);
    }
}

void setupPID()
{
    dongCoPID.data_dongCo1.vxSetpoint = 0;
    dongCoPID.data_dongCo1.vxInput = 0;
    dongCoPID.data_dongCo1.vxOutput = 0;
    dongCoPID.data_dongCo2.vxSetpoint = 0;
    dongCoPID.data_dongCo2.vxInput = 0;
    dongCoPID.data_dongCo2.vxOutput = 0;
    dongCoPID.data_dongCo3.vxSetpoint = 0;
    dongCoPID.data_dongCo3.vxInput = 0;
    dongCoPID.data_dongCo3.vxOutput = 0;
    dongCoPID.data_dongCo4.vxSetpoint = 0;
    dongCoPID.data_dongCo4.vxInput = 0;
    dongCoPID.data_dongCo4.vxOutput = 0;

    dongCoPID.dongCo1.Init(&dongCoPID.data_dongCo1.vxInput, &dongCoPID.data_dongCo1.vxOutput, &dongCoPID.data_dongCo1.vxSetpoint, 15, 0.05, 01.0, _PID_CD_DIRECT);
    dongCoPID.dongCo2.Init(&dongCoPID.data_dongCo2.vxInput, &dongCoPID.data_dongCo2.vxOutput, &dongCoPID.data_dongCo2.vxSetpoint, 15, 0.05, 01.0, _PID_CD_DIRECT);
    dongCoPID.dongCo3.Init(&dongCoPID.data_dongCo3.vxInput, &dongCoPID.data_dongCo3.vxOutput, &dongCoPID.data_dongCo3.vxSetpoint, 15, 0.05, 01.0, _PID_CD_DIRECT);
    dongCoPID.dongCo4.Init(&dongCoPID.data_dongCo4.vxInput, &dongCoPID.data_dongCo4.vxOutput, &dongCoPID.data_dongCo4.vxSetpoint, 15, 0.05, 01.0, _PID_CD_DIRECT);

    dongCoPID.dongCo1.SetMode(_PID_MODE_AUTOMATIC);
    dongCoPID.dongCo1.SetSampleTime(10);
    dongCoPID.dongCo1.SetOutputLimits(-255, 255);

    dongCoPID.dongCo2.SetMode(_PID_MODE_AUTOMATIC);
    dongCoPID.dongCo2.SetSampleTime(10);
    dongCoPID.dongCo2.SetOutputLimits(-255, 255);

    dongCoPID.dongCo3.SetMode(_PID_MODE_AUTOMATIC);
    dongCoPID.dongCo3.SetSampleTime(10);
    dongCoPID.dongCo3.SetOutputLimits(-255, 255);

    dongCoPID.dongCo4.SetMode(_PID_MODE_AUTOMATIC);
    dongCoPID.dongCo4.SetSampleTime(10);
    dongCoPID.dongCo4.SetOutputLimits(-255, 255);
}

void main_cpp()
{
    setupPID();

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
    // MX_IWDG_Init();
    int16_t encLastTime = 0;

    while (1)
    {
        // Nháy LED báo trạng thái sống
        blinkLed();

        // xu ly input
        processData();

        // // PID
        Robot_Control_Loop();

        // phan hoi (neu can)
        // respond();
        //        configPID();

        //        // Làm tươi Watchdog
        //        HAL_IWDG_Refresh(&hiwdg);

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