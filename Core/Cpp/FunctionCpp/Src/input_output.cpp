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

// KHOANG 6.5 VÒNG 1 GIÂY?
uint32_t tgNhanDuLieuPiCu = 0; // Biến toàn cục để lưu thời điểm cuối cùng nhận dữ liệu từ Pi
uint32_t tgGuiDuLieuPiCu = 0; // thoi gian truoc do gui du lieu len pi
float debug_ccr_L = 0.0f;      // Biến toàn cục để soi IAR
float debug_ccr_R = 0.0f;      // Biến toàn cục để soi IAR
float debug_alpha = 0.1f;      // Cho giá trị mặc định tránh bằng 0 lúc mới bật máy

// Bỏ tham số vx, w đi cho gọn
void debugNhanDuLieuPi()
{
    // BẢO VỆ MẤT KẾT NỐI: Dừng robot nếu quá 1 giây không nhận lệnh từ Web
    if (HAL_GetTick() - tgNhanDuLieuPiCu > 1000)
    {
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    }

    // Nếu chưa có cờ thì thoát
    if (!robot.state.isDataNew)
        return;

    // Hạ cờ
    robot.state.isDataNew = false;

    float ccr_base = 0.0f; // Tương đương vx cũ
    float ccr_turn = 0.0f; // Tương đương w cũ

    // Bóc tách 3 thông số: ccr_base, ccr_turn, debug_alpha
    if (sscanf((char *)rxBuffer, "%f,%f,%f", &ccr_base, &ccr_turn, &debug_alpha) == 3)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời gian nhận

        // TÍNH TOÁN TRỰC TIẾP VÀO BIẾN TOÀN CỤC
        debug_ccr_L = ccr_base - ccr_turn;
        debug_ccr_R = ccr_base + ccr_turn;

        // Gán vào Target để xe chạy
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = debug_ccr_L;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = debug_ccr_R;
    }
}

// Phiên bản tối ưu và siêu gọn
// TẮT KIÊM TRA > 1000 KHI DÙNG VỚI LIDAR ĐỂ TRÁNH XÓA DỮ LIỆU
void nhanDuLieuPi()
{
    // if (HAL_GetTick() - tgNhanDuLieuPiCu > 1000) // Dung robot neu pi khong gui du lieu trong 1 giay
    // {
    //     robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
    //     robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    // }

    // Nếu chưa có cờ thì thoát (CPU đi làm việc khác)
    if (!robot.state.isDataNew)
        return;

    // Hạ cờ
    robot.state.isDataNew = false;

    float vx = 0.0f;
    float w = 0.0f;

    // Bóc tách nhanh gọn lẹ bằng sscanf
    if (sscanf((char *)rxBuffer, "%f,%f", &vx, &w) == 2)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu

        // Tính toán CCR có dấu
        float ccr_L = vx - (w * KHOANGCACH2BANH / 2.0f);
        float ccr_R = vx + (w * KHOANGCACH2BANH / 2.0f);

        // GÁN THẲNG GIÁ TRỊ CÓ DẤU
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = ccr_L;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = ccr_R;
    }
}



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
    static int doDaiGoiTin;

    doDaiGoiTin = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f,%.3f,%.3f,%.3f,0.000,%.3f\n",
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