#ifndef DATA_HPP
#define DATA_HPP

#include "main.h"
#include <stdint.h>

// Tọa độ và góc
extern float odom_x;
extern float odom_y;
extern float odom_theta_rad;
extern float odom_theta_deg;

// Vận tốc
extern float odom_vx;
extern float odom_w_rad;

// Goc
extern float odom_w_enc;
extern float odom_w_mpu;

// Biến quản lý thời gian
extern uint32_t tgTinhOdomCu;
extern uint32_t tgTinhOdom;

// Khởi tạo cảm biến la bàn QMC5883L và biến lưu góc gốc
class QMC5883LCompass;
extern QMC5883LCompass compass;
extern float theta_goc; // goc theta goc cua robot, lay khi vua khoi tao

// Hàm tính toán odometry
void tinhOdom(void);

#endif