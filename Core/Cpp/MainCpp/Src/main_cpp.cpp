#include "main_cpp.hpp"
#include <math.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "const.hpp"
#include "MotorControl.hpp"
#include "MPU6050.hpp"
#include "HCSR04HyperSonic.hpp"
#include "UART_DMA.hpp"
#include "input_output.hpp"
#include "data.hpp"
#include "led_main.hpp"

extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern I2C_HandleTypeDef hi2c1;
float delta_ccr = 10.0f; // Khi ccrHT gan bang ccrTL, cho ccrHT = ccrTL

// Hàm tăng tốc độ từ từ motor
void controlOnDinh()
{
    // Bên trái trước
    robot.motor_front_left.ccrHT += debug_alpha * (robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT);
    if (fabsf(robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT) < delta_ccr)
    {
        robot.motor_front_left.ccrHT = robot.motor_front_left.ccrTL;
    }
    // Bên trái sau
    robot.motor_rear_left.ccrHT += debug_alpha * (robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT);
    if (fabsf(robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT) < delta_ccr)
    {
        robot.motor_rear_left.ccrHT = robot.motor_rear_left.ccrTL;
    }

    // Bên phải trước
    robot.motor_front_right.ccrHT += debug_alpha * (robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT);
    if (fabsf(robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT) < delta_ccr)
    {
        robot.motor_front_right.ccrHT = robot.motor_front_right.ccrTL;
    }
    // Bên phải sau
    robot.motor_rear_right.ccrHT += debug_alpha * (robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT);
    if (fabsf(robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT) < delta_ccr)
    {
        robot.motor_rear_right.ccrHT = robot.motor_rear_right.ccrTL;
    }

    // CẬP NHẬT BIẾN DIR VÀO STRUCT (Dựa trên dấu của ccrHT hiện tại)
    // Ben trai
    robot.motor_front_left.dir = (robot.motor_front_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    robot.motor_rear_left.dir = (robot.motor_rear_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    // Ben phai
    robot.motor_front_right.dir = (robot.motor_front_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);
    robot.motor_rear_right.dir = (robot.motor_rear_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);

    // ĐIỀU KHIỂN MOTOR THẬT (Lấy giá trị từ struct ra)
    MotorCtr_FL.control((uint16_t)fabsf(robot.motor_front_left.ccrHT), static_cast<MotorDir>(robot.motor_front_left.dir));
    MotorCtr_FR.control((uint16_t)fabsf(robot.motor_front_right.ccrHT), static_cast<MotorDir>(robot.motor_front_right.dir));
    MotorCtr_RL.control((uint16_t)fabsf(robot.motor_rear_left.ccrHT), static_cast<MotorDir>(robot.motor_rear_left.dir));
    MotorCtr_RR.control((uint16_t)fabsf(robot.motor_rear_right.ccrHT), static_cast<MotorDir>(robot.motor_rear_right.dir));
}

MPU6050 MPU6050_1;
uint8_t dia_chi_tim_thay = 0;

void main_cpp()
{
    // Khoi tao cam bien quan tinh
    HAL_TIM_Base_Start(&htim10);
    // Quet dia chi i2c
    // dia_chi_tim_thay = 0;
    for (uint8_t i = 1; i < 128; i++)
    {
        // Gửi thử tín hiệu đến tất cả 127 địa chỉ
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 10) == HAL_OK)
        {
            dia_chi_tim_thay = i;
            __NOP(); // <--- BẠN ĐẶT 1 CÁI BREAKPOINT (Dấu chấm đỏ) Ở DÒNG NÀY!!!
        }
    }

    // Nạp thông số và đánh thức cảm biến
    MPU6050_1.init(&hi2c1, &htim10, (0x68 << 1));
    if (!MPU6050_1.begin())
    {
        // Nếu lỗi I2C, nháy LED đỏ/nhanh báo hiệu
        while (1)
        {
            nhayLed();
            HAL_Delay(20);
        }
    }

    // Hiệu chuẩn Gyro Z (Lưu ý: Robot phải đứng im tuyệt đối trong 2 giây này)
    // Nháy LED chậm báo hiệu đang hiệu chuẩn
    for (int i = 0; i < 6; i++)
    {
        nhayLed();
        HAL_Delay(200);
    }
    MPU6050_1.hieuChuan();

    // Bật LED sáng tĩnh báo hiệu đã sẵn sàng chạy
    nhayLed();
    MPU6050_1.lastTime = __HAL_TIM_GET_COUNTER(MPU6050_1.htim);

    // Khoi tao cam bien sieu am
    HAL_TIM_Base_Start(&htim9);

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

    // Khoi tao UART DMA gui du lieu cho pi
    UART_DMA_6.init(&huart6, rxBuffer, sizeof(rxBuffer));

    uint32_t tgNhayLedCu = 0;
    uint32_t tgDoHSCu = 0; // thoi gian do cam bien sieu am cu
    uint32_t tgTinhGocZCu = 0;
    uint32_t tgDieuKhienMotorCu = 0; // thoi gian dieu khien dong co

    while (1)
    {
        // Nháy LED báo trạng thái sống
        nhayLedMode();

        // Tinh cac thong so goc z
        if (HAL_GetTick() - tgTinhGocZCu > 10)
        {
            tgTinhGocZCu = HAL_GetTick();
            MPU6050_1.tinhGocZ();
        }

        // xu ly input
        if (robot.state.isDebugMode == true)
        {
            debugNhanDuLieuPi();
        }
        else
        {
            debug_alpha = ALPHA;
            nhanDuLieuPi();
        }

        // Đọc cảm biến siêu âm mỗi 50ms
        // if (HAL_GetTick() - tgDoHSCu >= 50)
        // {
        //     tgDoHSCu = HAL_GetTick();
        //     kiemTraHS();
        //     kichHoatTrig(); // kich hoat chan trig de do cam bien sieu am
        // }

        if (HAL_GetTick() - tgDieuKhienMotorCu >= 10)
        {
            tgDieuKhienMotorCu = HAL_GetTick();

            if (isBlocked)
            {
                // NẾU CÓ VẬT CẢN: Ghi đè lệnh mục tiêu (ccrTL) về 0!
                robot.motor_front_left.ccrTL = 0;
                robot.motor_front_right.ccrTL = 0;
                robot.motor_rear_left.ccrTL = 0;
                robot.motor_rear_right.ccrTL = 0;
            }

            // GỌI HÀM ĐIỀU KHIỂN:
            // Hàm này sẽ lấy ccrTL (vừa bị ép về 0 ở trên, hoặc do pi gửi)
            controlOnDinh();
        }

        // Tính toán odometry
        tinhOdom();

        // Gui du lieu odometry len pi
        if (robot.state.isDebugMode == true)
        {
            debugGuiDuLieuPi();
        }
        else
        {

            guiDuLieuPi();
        }
    }
}

/*
- ngat khi xung len va xung xuong
- xung len sau khi kich hoat trig
- xung xuong khi echo nhan tin hieu song sieu am phan hoi
- khoang thoi gian giua xung len va xung xuong dung de tinh khoang cach
*/
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // ==========================================
    // 1. XỬ LÝ NÚT BẤM (Chân PC14)
    // ==========================================
    if (GPIO_Pin == GPIO_PIN_14)
    {
        static uint32_t thoiGianBamCu = 0;
        uint32_t thoiGianBamMoi = HAL_GetTick();

        // Chống dội phím 200ms
        if (thoiGianBamMoi - thoiGianBamCu > 200)
        {
            // Đảo cờ trạng thái của robot (Debug <-> Normal)
            robot.state.isDebugMode = false;
        }
        thoiGianBamCu = thoiGianBamMoi;
    }

    // ==========================================
    // 2. XỬ LÝ CẢM BIẾN SIÊU ÂM (Các chân Echo: PB0, PB1, PB2, PB10)
    // ==========================================
    else if (GPIO_Pin == ECHO1_Pin || GPIO_Pin == ECHO2_Pin || 
             GPIO_Pin == ECHO3_Pin || GPIO_Pin == ECHO4_Pin)
    {
        uint32_t thoiGianHT = __HAL_TIM_GET_COUNTER(&htim9);             // Lấy giá trị timer9 để tính thời gian
        GPIO_PinState trangThaiChan = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin); // Đọc trạng thái chân echo đang cao hay thấp
        xuLiNgat(GPIO_Pin, thoiGianHT, trangThaiChan);                   // Xử lý ngắt và tính khoảng cách
    }
}