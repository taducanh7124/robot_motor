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
uint32_t tgGuiDuLieuPiCu = 0;  // thoi gian truoc do gui du lieu len pi
float debug_ccr_L = 0.0f;      // Biến toàn cục để soi IAR
float debug_ccr_R = 0.0f;      // Biến toàn cục để soi IAR
float debug_alpha = 0.1f;      // Cho giá trị mặc định tránh bằng 0 lúc mới bật máy
float debug_vx_L = 0.0f;
float debug_vx_R = 0.0f;

// Bỏ tham số vx, w đi cho gọn
void debugNhanDuLieuPi()
{
    // BẢO VỆ MẤT KẾT NỐI: Dừng robot nếu quá 1 giây không nhận lệnh từ Web
    // if (HAL_GetTick() - tgNhanDuLieuPiCu > 1000)
    // {
    //     robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
    //     robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    // }

    // Nếu chưa có cờ thì thoát
    if (!robot.state.isDataNew)
        return;

    // Hạ cờ
    robot.state.isDataNew = false;

    float in_vx = 0.0f; // Tương đương vx cũ
    float in_w = 0.0f;  // Tương đương w cũ

    // Bóc tách 3 thông số: ccr_base, ccr_turn, debug_alpha
    if (sscanf((char *)rxBuffer, "%f,%f,%f", &in_vx, &in_w, &debug_alpha) == 3)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời gian nhận
        // TÍNH TOÁN TRỰC TIẾP VÀO BIẾN TOÀN CỤC
        debug_vx_L = in_vx - (in_w * (KHOANGCACH2BANH / 2.0f));
        debug_vx_R = in_vx + (in_w * (KHOANGCACH2BANH / 2.0f));
        // Luu hướng của vx
        int debug_dir_L = (debug_vx_L >= 0) ? 1 : -1;
        int debug_dir_R = (debug_vx_R >= 0) ? 1 : -1;

        // Map vx từ m/s sang CCR
        debug_ccr_L = doi_van_toc(fabs(debug_vx_L), 0.20f, 1.47f, 15.0f, 100.0f);
        debug_ccr_R = doi_van_toc(fabs(debug_vx_R), 0.20f, 1.47f, 15.0f, 100.0f);
        // Gán lại dấu cho CCR
        debug_ccr_L = debug_ccr_L * debug_dir_L;
        debug_ccr_R = debug_ccr_R * debug_dir_R;

        // Gán vào Target để xe chạy
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = debug_ccr_L;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = debug_ccr_R;
    }
}
float ccr_L = 0.0f;
float ccr_R = 0.0f;
float vx = 0.0f;
float vy = 0.0f; // Dành cho sau này nếu muốn mở rộng điều khiển omnidirectional
float w = 0.0f;
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

    // Bóc tách nhanh gọn lẹ bằng sscanf
    if (sscanf((char *)rxBuffer, "%f,%f,%f", &vx, &vy, &w) == 0)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu

        // 1. TÍNH VẬN TỐC TỪNG BÁNH (Đơn vị: m/s)
        float v_L = vx - (w * KHOANGCACH2BANH / 2.0f);
        float v_R = vx + (w * KHOANGCACH2BANH / 2.0f);

        // 2. TÁCH DẤU (Hướng)
        int dir_L = (v_L >= 0) ? 1 : -1;
        int dir_R = (v_R >= 0) ? 1 : -1;

        // 3. MAP TỪ m/s SANG CCR & GÁN TRẢ LẠI DẤU
        ccr_L = doi_van_toc(fabs(v_L), 0.20f, 1.47f, 15.0f, 100.0f) * dir_L;
        ccr_R = doi_van_toc(fabs(v_R), 0.20f, 1.47f, 15.0f, 100.0f) * dir_R;

        // 4. GÁN THẲNG GIÁ TRỊ VÀO TARGET
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = ccr_L;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = ccr_R;
    }
}

void debugGuiDuLieuPi()
{
    // 1. Kiểm tra chu kỳ gửi (Ví dụ 50ms = 20Hz)
    if (HAL_GetTick() - tgGuiDuLieuPiCu < 50)
    {
        return;
    }

    // 2. Kiểm tra xem bộ DMA đã rảnh chưa (1 = Rảnh, 0 = Đang bận)
    if (!robot.state.isSendDataNew)
    {
        return;
    }

    tgGuiDuLieuPiCu = HAL_GetTick();

    static int doDaiGoiTin;

    // 3. Đóng gói dữ liệu với 9 thông số
    // Thứ tự: x, y, theta_rad, vx, vy, w_rad_loc, theta_deg, w_enc_tho, w_mpu_tho
    doDaiGoiTin = snprintf((char *)txBuffer, sizeof(txBuffer),
                           "%.3f,%.3f,%.3f,%.3f,0.000,%.3f,%.1f,%.3f,%.3f\n",
                           odom_x,
                           odom_y,
                           odom_theta_rad,
                           odom_vx,
                           odom_w_rad,
                           odom_theta_deg,
                           odom_w_enc,
                           odom_w_mpu);

    // 4. Kích hoạt DMA gửi đi
    if (doDaiGoiTin > 0)
    {
        robot.state.isSendDataNew = 0; // Hạ cờ BẬN
        HAL_UART_Transmit_DMA(&huart6, txBuffer, doDaiGoiTin);
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