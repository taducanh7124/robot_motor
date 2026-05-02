#include "main_cpp.hpp"
#include <math.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "const.hpp"
#include "MotorControl.hpp"
#include "MPU6050.hpp"
#include "HCSR04.hpp"
#include "UART_DMA.hpp"
#include "input_output.hpp"
#include "data.hpp"
#include "led_main.hpp"

extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern I2C_HandleTypeDef hi2c1;

MPU6050 Obj_MPU6050;

void main_cpp()
{
    // KHOI TAO CAM BIEN QUAN TINH
    // resetMPU(); chuyen sang main.c
    // Kiem tra cam bien quan tinh san sang giao tiep i2c chua
    while (HAL_I2C_IsDeviceReady(&hi2c1, Obj_MPU6050.dia_chi_i2c, 3, 10) != HAL_OK)
    {
        nhayLed();
        HAL_Delay(50);
    }
    // Khoi dong timer dem cho cam bien quan tinh
    HAL_TIM_Base_Start(&htim10);
    // Nạp thông số và đánh thức cảm biến
    Obj_MPU6050.init(&hi2c1, &htim10);
    // Cau hinh cam bien, Nếu lỗi I2C, nháy LED nhanh báo hiệu
    if (!Obj_MPU6050.cauHinh())
    {
        while (1)
        {
            nhayLed();
            HAL_Delay(500);
        }
    }
    // Hiệu chuẩn Gyro Z (Robot phải đứng im tuyệt đối trong 2 giây này)
    nhayLed();
    HAL_Delay(2000);
    Obj_MPU6050.hieuChuan();
    Obj_MPU6050.tg_do_imu_qk = __HAL_TIM_GET_COUNTER(Obj_MPU6050.htim);

    // KHOI TAO CAM BIEN SIEU AM
    //  Khoi tao timer cho cam bien sieu am
    HAL_TIM_Base_Start(&htim9);

    // Khoi tao UART DMA gui du lieu cho pi
    UART_DMA_6.init(&huart6, rxBuffer, sizeof(rxBuffer));

    // Khoi tao cac dong co
    MotorCtr_FL.init(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin);
    MotorCtr_FR.init(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin);
    MotorCtr_RL.init(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin);
    MotorCtr_RR.init(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin);

    // Khoi tao timer doc encoder cho cac dong co
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    uint32_t tgTinhGocZCu = 0;
    uint32_t tgDieuKhienMotorCu = 0; // thoi gian dieu khien dong co cũ

    while (1)
    {
        // Nháy LED báo trạng thái sống
        nhayLedMode();

        // Tinh cac thong so goc z
        if (HAL_GetTick() - tgTinhGocZCu > 10)
        {
            tgTinhGocZCu = HAL_GetTick();
            Obj_MPU6050.tinhGocZ();
        }

        // xu ly input
        nhanDuLieuPi();

        // Đọc cảm biến siêu âm mỗi 25ms - TAM THOI KHONG DUNG
        // doKhoangCach();

        if (HAL_GetTick() - tgDieuKhienMotorCu >= 10)
        {
            tgDieuKhienMotorCu = HAL_GetTick();
            if (isBlocked)
            {
                // NẾU CÓ VẬT CẢN: Ghi đè lệnh mục tiêu (ccrTL) về 0!
                dungMotor();
            }
            // GỌI HÀM ĐIỀU KHIỂN:
            // Hàm này sẽ lấy ccrTL (vừa bị ép về 0 ở trên, hoặc do pi gửi)
            controlOnDinh();
        }

        // Tính toán odometry
        tinhOdom();

        // Gui du lieu odometry len pi
        guiDuLieuPi();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1. XỬ LÝ NÚT BẤM (Chân PC14) đổi chế độ
    if (GPIO_Pin == GPIO_PIN_14)
    {
        static uint32_t thoiGianBamCu = 0;
        uint32_t thoiGianBamMoi = HAL_GetTick();

        // Chống dội phím 200ms
        if (thoiGianBamMoi - thoiGianBamCu > 3000)
        {
            // Đảo cờ trạng thái của robot (Debug <-> Normal)
            robot.state.isDebugMode = true; // tam thoi khong dung
        }
        thoiGianBamCu = thoiGianBamMoi;
    }

    // 2. XỬ LÝ CẢM BIẾN SIÊU ÂM (Các chân Echo: PB0, PB1, PB2, PB10)
    /*
    - ngat khi xung len va xung xuong
    - xung len sau khi kich hoat trig
    - xung xuong khi echo nhan tin hieu song sieu am phan hoi
    - khoang thoi gian giua xung len va xung xuong dung de tinh khoang cach
    */
    else if (GPIO_Pin == ECHO1_Pin || GPIO_Pin == ECHO2_Pin)
    // ||
    //       GPIO_Pin == ECHO3_Pin) // || GPIO_Pin == ECHO4_Pin)
    {
        uint16_t tgXungHT = __HAL_TIM_GET_COUNTER(&htim9);               // Lấy giá trị timer9 để tính thời gian
        GPIO_PinState trangThaiChan = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin); // Đọc trạng thái chân echo đang cao hay thấp
        xuLiNgat(GPIO_Pin, tgXungHT, trangThaiChan);                     // Xử lý ngắt và tính khoảng cách
    }
}