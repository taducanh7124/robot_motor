#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "config.hpp" // Đảm bảo đã có khai báo struct robot và huart6

// Gọi hàm này định kỳ mỗi 10ms (sau khi đã tính xong velocity)
void CalculateOdometry();

// Gọi hàm này trong vòng lặp while(1) để gửi dữ liệu lên Pi (tự định thời 50ms)
void respond();

void configPID();

#endif // ODOMETRY_HPP