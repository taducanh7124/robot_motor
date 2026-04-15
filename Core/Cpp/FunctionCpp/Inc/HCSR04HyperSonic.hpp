#ifndef HCSR04_HYPERSONIC_HPP
#define HCSR04_HYPERSONIC_HPP

#include <stdint.h>
#include "main.h"

class HyperSonic
{
public:
    // Thuoc tinh
    float khoangCach;

    // Phuong thuc
    HyperSonic();
    ~HyperSonic();
};

extern HyperSonic ArrayHS[4];       // Mảng chứa 4 cảm biến siêu âm
extern bool isBlocked;              // Biến toàn cục để báo trạng thái có vật cản hay không
extern bool isTrigHigh;             // Biến toàn cục để báo trạng thái chân TRIG đang ở mức cao hay thấp
extern uint32_t Arr_tgXungLen[4];   // Mảng lưu thời gian xung lên của từng cảm biến siêu âm
extern uint32_t Arr_tgXungXuong[4]; // Mảng

void kichHoatTrig();                                                                // Hàm kích hoạt chân TRIG để bắt đầu đo khoảng cách
int layViTriHS(uint16_t GPIO_Pin);                                                  // Hàm xác định vị trí của cảm biến siêu âm dựa vào chân GPIO
void xuLiNgat(uint16_t GPIO_Pin, uint32_t thoiGianHT, GPIO_PinState trangThaiChan); // Hàm xử lý ngắt từ chân ECHO của cảm biến siêu âm

#endif