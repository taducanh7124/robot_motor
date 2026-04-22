#include "data.hpp"
#include <math.h>
#include "main.h"
#include "tim.h"
#include "const.hpp"
#include "MPU6050.hpp"
#include "MotorControl.hpp"

// Biến toàn cục lưu trữ dữ liệu odometry
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta_rad = 0.0f;
float odom_theta_deg = 0.0f;

float odom_vx = 0.0f;
float odom_w_rad = 0.0f;

float vtTrungBinhTrai = 0;
float vtTrungBinhPhai = 0;

// Biến quản lý thời gian tổng
uint32_t tgTinhOdomCu = 0; // thoi gian truoc do tinh odom
uint32_t tgTinhOdom = 100; // thoi gian giua cac lan tinh odom (100ms)

void tinhVanToc(float deltaT) // Nhận deltaT từ hàm quản lý truyền vào
{
    // CÁC BIẾN NÀY CẦN NHỚ GIÁ TRỊ CŨ -> Bắt buộc dùng static
    static int32_t xungQK_FL = 0; // xung qua khu của bánh trước trái
    static int32_t xungQK_FR = 0; // xung qua khu của bánh trước phải
    static int32_t xungQK_RL = 0; // xung qua khu của bánh sau trái
    static int32_t xungQK_RR = 0; // xung qua khu của bánh sau phải

    int32_t xungHT_FL = (int32_t)__HAL_TIM_GET_COUNTER(&htim1); // xung hiện tại của bánh trước trái
    int32_t xungHT_FR = (int32_t)__HAL_TIM_GET_COUNTER(&htim2); // xung hiện tại của bánh trước phải
    int32_t xungHT_RL = (int32_t)__HAL_TIM_GET_COUNTER(&htim4); // xung hiện tại của bánh sau trái
    int32_t xungHT_RR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3); // xung hiện tại của bánh sau phải

    int32_t delta_FL = xungHT_FL - xungQK_FL; // chenh lech xung của bánh trước trái
    int32_t delta_FR = xungHT_FR - xungQK_FR; // chenh lech xung của bánh trước phải
    int32_t delta_RL = xungHT_RL - xungQK_RL; // chenh lech xung của bánh sau trái
    int32_t delta_RR = xungHT_RR - xungQK_RR; // chenh lech xung của bánh sau phải

    // Xử lý tràn
    // Banh truoc trai
    if (delta_FL > 32768)
        delta_FL -= 65536;
    else if (delta_FL < -32768)
        delta_FL += 65536;
    // Banh truoc phai
    if (delta_FR > 32768)
        delta_FR -= 65536;
    else if (delta_FR < -32768)
        delta_FR += 65536;
    // Banh sau trai
    if (delta_RL > 32768)
        delta_RL -= 65536;
    else if (delta_RL < -32768)
        delta_RL += 65536;
    // Banh sau phai
    if (delta_RR > 32768)
        delta_RR -= 65536;
    else if (delta_RR < -32768)
        delta_RR += 65536;

    // Tính vận tốc của từng bánh
    robot.motor_front_left.vanToc = (delta_FL * MET1XUNG) / deltaT;
    robot.motor_front_right.vanToc = -(delta_FR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu
    robot.motor_rear_left.vanToc = (delta_RL * MET1XUNG) / deltaT;
    robot.motor_rear_right.vanToc = -(delta_RR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu

    // Tinh vận tốc trung bình của robot dựa trên vận tốc của 4 bánh
    vtTrungBinhTrai = (robot.motor_rear_left.vanToc + robot.motor_front_left.vanToc) / 2.0f;
    vtTrungBinhPhai = (robot.motor_rear_right.vanToc + robot.motor_front_right.vanToc) / 2.0f;
    odom_vx = (vtTrungBinhTrai + vtTrungBinhPhai) / 2.0f;

    // Cập nhật vị trí encoder cho lần sau
    xungQK_FL = xungHT_FL;
    xungQK_FR = xungHT_FR;
    xungQK_RL = xungHT_RL;
    xungQK_RR = xungHT_RR;
}

float odom_w_enc = 0;
float odom_w_mpu = 0;

// TÍNH CẢ VẬN TỐC GÓC VÀ GÓC
void tinhThongSoGoc(float deltaT)
{
    // LẤY VẬN TỐC GÓC TỪ MPU6050 (w_gyro)
    // Lưu ý: Biến mpu.vt_goc_z đã được cập nhật liên tục bên file main_cpp.cpp
    odom_w_rad = Obj_MPU6050.vt_goc_z * (PI / 180.0f); // Đổi từ Độ/s sang Rad/s

    // GÓC HƯỚNG CỦA XE
    odom_theta_deg = Obj_MPU6050.goc_z;
    odom_theta_rad = odom_theta_deg *(PI / 180.0f);
}

void tinhToaDo(float deltaT)
{
    odom_x += odom_vx * cosf(odom_theta_rad) * deltaT;
    odom_y += odom_vx * sinf(odom_theta_rad) * deltaT;
}

// =======================================================
// 3. HÀM QUẢN LÝ CHÍNH (GỌI TRONG MAIN)
// =======================================================
void tinhOdom()
{
    uint32_t tgTinhOdomMoi = HAL_GetTick();
    if (tgTinhOdomMoi - tgTinhOdomCu < tgTinhOdom)
    {
        return; // Chưa đủ 100ms thì nghỉ
    }

    // Tính deltaT thực tế phòng trường hợp CPU bị trễ (VD: 101ms, 105ms)
    float deltaT = (tgTinhOdomMoi - tgTinhOdomCu) / 1000.0f;
    tgTinhOdomCu = tgTinhOdomMoi;

    // Chạy các hàm con theo đúng quy trình
    tinhVanToc(deltaT);
    tinhThongSoGoc(deltaT);
    tinhToaDo(deltaT);
}

// void tinhGoc()
// {
//     compass.read();
//     odom_theta_deg = compass.getAzimuth() - theta_goc; // Lấy góc hiện tại trừ đi góc gốc

//     if (odom_theta_deg > 180.0f)
//         odom_theta_deg -= 360.0f;
//     else if (odom_theta_deg < -180.0f)
//         odom_theta_deg += 360.0f;

//     odom_theta_rad = odom_theta_deg * PI / 180.0f; // Chuyển góc sang radian để tính toán
// }

// void tinhVanTocGoc(float deltaT)
// {
//     // Dùng static để giữ giá trị giữa các lần chạy hàm (Thay thế cho biến toàn cục)
//     static float odom_theta_deg_cu = 0.0f;
//     static float odom_w_rad_loc = 0.0f; // Vận tốc góc đã lọc

//     float delta_theta_deg = odom_theta_deg - odom_theta_deg_cu; // Chenh lech do giua 2 lan do

//     // xu li tran goc
//     if (delta_theta_deg > 180.0f)
//         delta_theta_deg -= 360.0f;
//     else if (delta_theta_deg < -180.0f)
//         delta_theta_deg += 360.0f;

//     float odom_w_rad_tho = (delta_theta_deg * PI / 180.0f) / deltaT; // Van toc goc thô
//     odom_w_rad_loc = odom_w_rad_loc * 0.8f + odom_w_rad_tho * 0.2f;  // Tinh van toc goc loc
//     odom_w_rad = odom_w_rad_loc;

//     odom_theta_deg_cu = odom_theta_deg;
// }