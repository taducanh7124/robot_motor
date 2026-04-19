#include "main_cpp.hpp"
#include <math.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "const.hpp"
#include "MotorControl.hpp"
#include "QMC5883LCompass.hpp"
#include "HCSR04HyperSonic.hpp"
#include "UART_DMA.hpp"
#include "input_output.hpp"
#include "data.hpp"
#include "led_main.hpp"

extern TIM_HandleTypeDef htim9;
float delta_ccr = 10.0f; // Khi ccrHT gan bang ccrTL, cho ccrHT = ccrTL

// Hàm tăng tốc độ từ từ motor
void controlOnDinh()
{
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

void main_cpp()
{
    // Khoi tao cam bien la ban
    compass.init(SCL_GPIO_Port, SCL_Pin, SDA_GPIO_Port, SDA_Pin);
    compass.setSmoothing(5, true); // Smoothing 5 bước
    HAL_Delay(100);                // Đợi cảm biến ổn định
    // Đọc nháp 10 lần để lấp đầy mảng lọc nhiễu (Vứt bỏ kết quả)
    for (int i = 0; i < 10; i++)
    {
        compass.read();
        HAL_Delay(10); // Đợi 10ms giữa mỗi lần đọc cho đúng nhịp
    }
    theta_goc = compass.getAzimuth(); // Lấy góc gốc khi khởi tạo để tính toán sau này

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

    // Khoi tao timer cho cam bien sieu am
    HAL_TIM_Base_Start(&htim9);

    // Khoi tao UART DMA gui du lieu cho pi
    UART_DMA_6.init(&huart6, rxBuffer, sizeof(rxBuffer));

    uint32_t tgDoHSCu = 0;           // thoi gian do cam bien sieu am cu
    uint32_t tgDieuKhienMotorCu = 0; // thoi gian dieu khien dong co

    while (1)
    {
        // Nháy LED báo trạng thái sống
        nhayLed();

        // xu ly input
        // debugNhanDuLieuPi();
        nhanDuLieuPi();

        // Đọc cảm biến siêu âm mỗi 50ms
        // if (HAL_GetTick() - tgDoHSCu >= 50)
        // {
        //     tgDoHSCu = HAL_GetTick();
        //     isBlocked = false;          // reset trạng thái có vật cản trước khi kiểm tra lại
        //     for (int i = 0; i < 4; i++) // kiem tra 4 cam bien sieu am xem co vat can hay khong
        //     {
        //         if (ArrayHS[i].khoangCach > 2.0f && ArrayHS[i].khoangCach < 100.0f)
        //         {
        //             isBlocked = true; // Có vật cản
        //             break;            // Ngay lap tuc thoat vong for
        //         }
        //     }
        //     for (int i = 0; i < 4; i++) // xoa du lieu truoc khi do cam bien sieu am lan sau
        //     {
        //         ArrayHS[i].khoangCach = 0.0f;
        //     }
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
        guiDuLieuPi();
    }
}