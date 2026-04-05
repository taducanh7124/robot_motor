#include "odometry.hpp"
#include <math.h>
#include <stdio.h>

// --- CÁC HẰNG SỐ CƠ KHÍ ---
#define PI 3.14159265358979323846f
#define R 0.05f    // Bán kính bánh xe (mét)
#define PPR 360.0f // Xung/vòng của encoder (Thêm .0f để ép kiểu float an toàn)
#define L 0.15f    // Nửa khoảng cách giữa 2 bánh (mét)

// --- BIẾN LƯU TRỮ TRẠNG THÁI (Khai báo static để bảo vệ dữ liệu) ---
static float odom_x = 0.0f;
static float odom_y = 0.0f;
static float odom_theta = 0.0f;

static float odom_vx = 0.0f;
static float odom_w = 0.0f;

// --- ĐỊNH NGHĨA CÁC HÀM ---

void CalculateOdometry()
{
    // 1. QUY ĐỔI: XUNG/10ms -> MÉT/GIÂY
    // Công thức: (Số xung / 10ms) * 100 (ra xung/giây) / PPR * Chu_vi_bánh
    float k_v = (100.0f * 2.0f * PI * R) / PPR;

    float v_FL = robot.front_left.velocity * k_v;
    float v_FR = robot.front_right.velocity * k_v;
    float v_RL = robot.rear_left.velocity * k_v;
    float v_RR = robot.rear_right.velocity * k_v;

    // 2. VẬN TỐC TRUNG BÌNH MỖI BÊN BÁNH
    float v_left = (v_FL + v_RL) / 2.0f;
    float v_right = (v_FR + v_RR) / 2.0f;

    // 3. ĐỘNG HỌC THUẬN (FORWARD KINEMATICS)
    odom_vx = (v_right + v_left) / 2.0f;
    odom_w = (v_right - v_left) / (2.0f * L);

    // 4. TÍCH PHÂN TỌA ĐỘ (ODOMETRY)
    float dt = 0.01f; // Do ta đang tính mỗi 10ms (0.01 giây)

    odom_theta += odom_w * dt;

    // Chuẩn hóa góc theta về giới hạn [-PI, PI] (Quan trọng để ROS không bị lỗi)
    if (odom_theta > PI)
        odom_theta -= 2.0f * PI;
    if (odom_theta < -PI)
        odom_theta += 2.0f * PI;

    odom_x += odom_vx * cos(odom_theta) * dt;
    odom_y += odom_vx * sin(odom_theta) * dt;
}

void respond()
{
    // Định thời gửi dữ liệu mỗi 50ms (Tần số 20Hz)
    static uint32_t last_send_time = 0;
    if (HAL_GetTick() - last_send_time < 50)
        return;
    last_send_time = HAL_GetTick();

    // Chuẩn bị mảng ký tự làm bộ đệm gửi (Buffer)
    char tx_buffer[100];

    // Đóng gói dữ liệu định dạng: x,y,theta,vx,vy,w\n
    // Hàm snprintf rất an toàn vì không bao giờ ghi lố bộ nhớ
    int len = snprintf(tx_buffer, sizeof(tx_buffer), "%.3f,%.3f,%.3f,%.3f,0.000,%.3f\n",
                       odom_x, odom_y, odom_theta, odom_vx, odom_w);

    // Nếu format thành công, gửi qua UART
    if (len > 0)
    {
        HAL_UART_Transmit(&huart6, (uint8_t *)tx_buffer, len, 10);
    }
}

void configPID()
{

    static uint32_t last_send_time = 0;
    if (HAL_GetTick() - last_send_time < 50)
        return;

    // cache
    float velocity_FL = robot.front_left.velocity;
    float velocity_FR = robot.front_right.velocity;
    float velocity_RL = robot.rear_left.velocity;
    float velocity_RR = robot.rear_right.velocity;

    char debug_buffer[200];
    int debug_len = snprintf(debug_buffer, sizeof(debug_buffer), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                             velocity_FL, robot.front_left.setpoint, velocity_FR, robot.front_right.setpoint, velocity_RL, robot.rear_left.setpoint, velocity_RR, robot.rear_right.setpoint);

    HAL_UART_Transmit(&huart6, (uint8_t *)debug_buffer, debug_len, 10);
}