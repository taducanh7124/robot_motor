#include "input.hpp"

void processData()
{
    // Xu ly du lieu nhan duoc tu UART
    // Chuyen du lieu tu bufferUART sang cau truc robot
    // Sau do dieu khien dong co theo cau truc robot
    if (!robot.state.isDataNew)
        return;

    robot.state.isDataNew = false; // Reset flag sau khi đã xử lý dữ liệu mới
    static uint8_t cache[8];

    // lay 8 byte dau tien
    for (uint8_t i = 0; i < 8; i++)
    {
        cache[i] = bufferUART[i];
    }

    robot.front_left.dir = cache[0];
    robot.front_left.pwm = cache[1];
    robot.front_right.dir = cache[2];
    robot.front_right.pwm = cache[3];
    robot.rear_left.dir = cache[4];
    robot.rear_left.pwm = cache[5];
    robot.rear_right.dir = cache[6];
    robot.rear_right.pwm = cache[7];

    robot.state.isControlNew = true; // Đánh dấu có lệnh điều khiển mới để main loop xử lý
   // robot.state.isResponseNew = true; // Đánh dấu có phản hồi mới để main loop xử lý (nếu cần)
}