#include "input_output.hpp"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "usart.h"
#include "const.hpp"
#include "data.hpp"
#include "MotorControl.hpp"
#include "UART_DMA.hpp"

// vx = 40 -> pwm = 15.625 tương đương 

float debug_ccr_L = 0.0f; // Biến toàn cục để lưu giá trị CCR đã tính toán từ dữ liệu nhận được, dùng cho debug
float debug_ccr_R = 0.0f; // Biến toàn cục để lưu giá trị CCR đã tính toán từ dữ liệu nhận được, dùng cho debug

void debugNhanDuLieuPi(float vx, float w)
{

    debug_ccr_L = vx - (w * KHOANGCACH2BANH / 2.0f);
    debug_ccr_R = vx + (w * KHOANGCACH2BANH / 2.0f);

    // Vì điều khiển trực tiếp CCR nên ta xét dấu của kết quả tính toán
    MotorDir debug_dir_L = (debug_ccr_L >= 0) ? MotorDir::Forward : MotorDir::Backward;
    MotorDir debug_dir_R = (debug_ccr_R >= 0) ? MotorDir::Backward : MotorDir::Forward;

    // Lấy giá trị tuyệt đối và gán vào target cho hàm controlOnDinh
    robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = fabs(debug_ccr_L);
    robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = fabs(debug_ccr_R);

    robot.motor_front_left.dir = robot.motor_rear_left.dir = static_cast<uint8_t>(debug_dir_L);
    robot.motor_front_right.dir = robot.motor_rear_right.dir = static_cast<uint8_t>(debug_dir_R);
}

uint32_t tgNhanDuLieuPiCu = 0; // Biến toàn cục để lưu thời điểm cuối cùng nhận dữ liệu từ Pi, dùng cho debug

// phien ban toi uu hơn
void nhanDuLieuPi()
{
    if (HAL_GetTick() - tgNhanDuLieuPiCu > 1000) // Dung robot neu pi khong gui du lieu trong 1 giay
    {
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    }
    // Nếu chưa có cờ thì thoát (CPU đi làm việc khác)
    if (!robot.state.isDataNew) return;
    // Hạ cờ
    robot.state.isDataNew = false; 

    float vx = 0.0f;
    float w = 0.0f;

    // Bóc tách nhanh gọn lẹ bằng sscanf
    // Mẫu lệnh Pi gửi xuống: "0.5,0.2\n"
    if (sscanf((char*)rxBuffer, "%f,%f", &vx, &w) == 2)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu
        memset(rxBuffer, 0, sizeof(rxBuffer)); // Xóa buffer sau khi đã xử lý xong dữ liệu

        // Tính toán CCR và gán vào biến như code cũ của bạn
        float ccr_L = vx - (w * KHOANGCACH2BANH / 2);
        float ccr_R = vx + (w * KHOANGCACH2BANH / 2);

        MotorDir dir_L = (ccr_L >= 0) ? MotorDir::Forward : MotorDir::Backward;
        MotorDir dir_R = (ccr_R >= 0) ? MotorDir::Backward : MotorDir::Forward;

        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = fabs(ccr_L);
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = fabs(ccr_R);

        robot.motor_front_left.dir = robot.motor_rear_left.dir = static_cast<uint8_t>(dir_L);
        robot.motor_front_right.dir = robot.motor_rear_right.dir = static_cast<uint8_t>(dir_R);
    }
}


uint32_t tgGuiDuLieuPiCu = 0;  // thoi gian truoc do gui du lieu len pi

// Hàm gửi dữ liệu odometry ra UART cho pi
void guiDuLieuPi()
{
    // 1. Kiểm tra chu kỳ gửi (Ví dụ 50ms = 20Hz)
    if (HAL_GetTick() - tgGuiDuLieuPiCu < 50) 
    {
        return;
    }

    // 2. Kiểm tra xem bộ DMA đã rảnh chưa (1 = Rảnh, 0 = Đang bận gửi gói cũ)
    if (!robot.state.isSendDataNew)
    {
        return; 
    }
    
    tgGuiDuLieuPiCu = HAL_GetTick();

    // 3. Đóng gói dữ liệu (Dùng luôn mảng txBuffer toàn cục đã khai báo ở UART_DMA.hpp)
    // Định dạng: odom_x, odom_y, odom_theta, odom_vx, 0.000, odom_w \n
    int doDaiGoiTin = snprintf((char*)txBuffer, sizeof(txBuffer), "%.3f,%.3f,%.3f,%.3f,0.000,%.3f\n", 
                                odom_x, odom_y, odom_theta_rad, odom_vx, odom_w_rad); 

    // 4. Kích hoạt DMA gửi đi
    if (doDaiGoiTin > 0)
    {
        // Hạ cờ BẬN ngay lập tức để khóa hàm lại cho chu kỳ sau
        robot.state.isSendDataNew = 0; 
        
        // Đẩy dữ liệu đi (DMA sẽ tự động phất cờ rảnh ở ngắt TxCpltCallback)
        HAL_UART_Transmit_DMA(&huart6, txBuffer, doDaiGoiTin); 
    }
}