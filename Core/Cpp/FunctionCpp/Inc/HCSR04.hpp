#ifndef HCSR04_HPP
#define HCSR04_HPP

#include <stdint.h>
#include "main.h"

class HCSR04 // ultrasonic sensor
{
public:
    // Thuoc tinh
    GPIO_TypeDef *TRIG_Port;  // port cua chan trig
    uint16_t TRIG_Pin;        // chan trig
    uint16_t tgXungLen = 0;   // Thời gian xung lên của từng cảm biến siêu âm
    uint16_t tgXungXuong = 0; // Thời gian xung xuống của từng cảm biến siêu âm
    float khoangCach = 0;     // khoang cach cam bien sieu am do duoc cm

    // Phuong thuc
    HCSR04(GPIO_TypeDef *_TRIG_Port, uint16_t _TRIG_Pin);
    void kichHoatTRIG(); // Hàm kích hoạt chân TRIG để bắt đầu đo khoảng cách
};

extern HCSR04 Arr_HCSR04[2]; // Mảng chứa 4 cảm biến siêu âm
extern bool isBlocked;       // Biến toàn cục để báo trạng thái có vật cản hay khôngâm

void kiemTraUS();    // Kiem ta cam bien sieu am co phat hien vat can khong
void doKhoangCach(); // Hàm lần lượt kích hoạt cảm biến siêu âm

// Hàm xử lý ngắt từ chân ECHO của cảm biến siêu âm
void xuLiNgat(uint16_t GPIO_Pin, uint16_t _tgXungHT, GPIO_PinState _trangThaiChan);

#endif