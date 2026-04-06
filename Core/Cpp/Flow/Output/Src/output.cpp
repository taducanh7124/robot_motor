/*
- dữ liệu cần gửi lên pi
+ x (mét): tọa độ x, số mét đã di dọc theo trục x
+ y (mét): tọa độ y, số mét đã di dọc theo trục y
+ theta (radian): góc xoay so với gốc
+ vx (mét/giây): tốc độ đi dọc tại thời điểm đo vận tốc
+ vy (mét/giây): không dùng vì robot không đi ngang
+ w (radian/giây): tốc độ quay quanh trục hoặc rẽ hướng khác

- các tính các dữ liệu
+ x: tổng sigma [quãng đường robot đã đi × cos(theta)]
+ y: tổng sigma [quãng đường robot đã đi × sin(theta)]
+ theta: dùng cảm biến la bàn lấy góc
+ vx: quãng đường robot đã đi ÷ thời gian
+ vy: không tính
+ w: (góc mới - góc cũ) ÷ thời gian, lưu ý số âm

- tính thông số cơ bản
+ chu vi bánh xe
+ tổng số xung khi quay trọn 1 vòng
+ hệ số chu vi ÷ tổng xung ->số xung × hệ số -> quãng đường
+ tính quãng đường 2 bánh trái/phải đã đi, chia trung bình -> quãng đường robot đã đi
*/

#include "output.hpp"
#include "math.h"
#include "config.hpp"
#include "stdio.h"

#define PI 3.14159265358979323846
#define chuViBanh 0.3; // met
#define xung1Vong 500 // xung/vòng của encoder
#define heSoXung chuViBanh / xung1Vong // he so doi xung sang met

// bien luu vi tri toa do va huong robot
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// bien luu van toc robot
float odom_vx = 0.0f;
float odom_w = 0.0f;

// bien luu van toc banh
float vtBanhTrenTrai = 0.0f;
float vtBanhDuoiTrai = 0.0f;
float vtBanhTrenPhai = 0.0f;
float vtBanhDuoiPhai = 0.0f;
float vtTrungBinhTrai = 0.0f;
float vtTrungBinhPhai = 0.0f;
float vtTrungBinhTong = 0.0f;

// bien thoi gian
float dt = 10.0f; // 10 milli giay
float thoiGianCuOdom = 0.0f; // thoi gian truoc do tinh odom
float thoiGianCuGui = 0.0f; // thoi gian truoc do gui du lieu len pi

// tinh toa do, thong so odometry
void tinhOdom()
{
    // tinh odom moi 10 ms
    if(HAL_GetTick() - thoiGianCuOdom < dt)
    {
        return; // Chưa đủ 10ms, thoát hàm
    }
    thoiGianCuOdom = HAL_GetTick(); // Cập nhật mốc thời gian

    // van toc vx
    vtBanhTrenTrai = robot.rear_left.velocity * heSoXung;
    vtBanhDuoiTrai = robot.front_left.velocity * heSoXung;
    vtBanhTrenPhai = robot.rear_right.velocity * heSoXung;
    vtBanhDuoiPhai = robot.front_right.velocity * heSoXung;
    vtTrungBinhTrai = (vtBanhTrenTrai + vtBanhDuoiTrai) / 2.0f;
    vtTrungBinhPhai = (vtBanhTrenPhai + vtBanhDuoiPhai) / 2.0f;
    vtTrungBinhTong = (vtTrungBinhTrai + vtTrungBinhPhai) / 2.0f;

    // toa do x, y
    odom_x += vtTrungBinhTong * cos(odom_theta);
    odom_y += vtTrungBinhTong * sin(odom_theta);

    // huong theta

    // toc do goc w
}

// ham gui du lieu odometry ra UART cho pi
void guiDuLieuPi()
{
    // Định thời gửi dữ liệu mỗi 50ms (Tần số 20Hz)
    if (HAL_GetTick() - thoiGianCuGui < 50) {
        return;
    }
    thoiGianCuGui = HAL_GetTick();

    // Chuẩn bị mảng ký tự làm bộ đệm gửi (Buffer)
    char txBuffer[100];
    
    // Đóng gói dữ liệu định dạng: odom_x,odom_y,odom_theta, odom_vx, odom_w
    // snprint in ra moi chuoi duoc dinh dang
    int doDaiGoi = snprintf(txBuffer, sizeof(txBuffer), "%.3f,%.3f,%.3f,%.3f,0.000,%.3f\n", // \n de ket thuc goi tin
                                                        odom_x, odom_y, odom_theta, odom_vx, odom_w);

    // Gui du lieu neu dinh dang thanh cong
    if (doDaiGoi > 0)
    {        
        // Hoặc dùng trực tiếp HAL nếu thư viện trên lỗi:
        HAL_UART_Transmit(&huart6, (uint8_t*)txBuffer, doDaiGoi, 10); // sau 10 ms khong gui het du lieu thi tat
    }
}