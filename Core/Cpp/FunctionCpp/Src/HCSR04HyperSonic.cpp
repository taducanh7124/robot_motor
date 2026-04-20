#include "HCSR04HyperSonic.hpp"
#include "tim.h"

HyperSonic::HyperSonic()
{
    khoangCach = 0.0;
}

HyperSonic::~HyperSonic()
{
    // Ham huy, khong can lam gi o day
}

HyperSonic ArrayHS[4]; // mang 4 doi tuong cam bien sieu am

int viTriHS = 0; // vi tri cam bien sieu am trong mang

bool isBlocked = false; // bien bao trang thai co vat can hay khong, dung de dieu khien robot tranh vat can

uint32_t Arr_tgXungLen[4] = {0};   // bien luu thoi gian xung len de tinh khoang cach sau nay
uint32_t Arr_tgXungXuong[4] = {0}; // bien luu thoi gian xung xuong de tinh khoang cach sau nay
uint32_t thoiGianHT = 0;           // bien luu thoi gian hien tai de tinh thoi gian xung len va xung xuong

void kichHoatTrig()
{
    // Lấy mốc thời gian bắt đầu
    uint16_t start_time = __HAL_TIM_GET_COUNTER(&htim9);

    // Bật cả 4 chân TRIG lên HIGH
    HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_SET);

    // Chờ đúng 10 micro-giây (Dùng ép kiểu uint16_t để chống lỗi tràn timer)
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start_time) < 10)
    {
    }

    // Kéo 4 chân TRIG xuống LOW
    HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_RESET);
}

// xac dinh vi tri cua cam bien sieu am dua vao chan gpio
int layViTriHS(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ECHO1_Pin)
        return 0;
    else if (GPIO_Pin == ECHO2_Pin)
        return 1;
    else if (GPIO_Pin == ECHO3_Pin)
        return 2;
    else if (GPIO_Pin == ECHO4_Pin)
        return 3;
    return -1;
}

void xuLiNgat(uint16_t GPIO_Pin, uint32_t thoiGianHT, GPIO_PinState trangThaiChan)
{
    viTriHS = layViTriHS(GPIO_Pin); // lay vi tri cua cam bien tu GPIO_Pin
    if (viTriHS == -1)
        return; // neu GPIO_Pin khong phu hop voi bat ky cam bien nao thi thoat khoi ham
    if (trangThaiChan == GPIO_PIN_SET)
    {
        Arr_tgXungLen[viTriHS] = thoiGianHT; // luu thoi gian xung len de tinh khoang cach sau nay
    }
    else
    {
        // neu chan echo dang thap thi tinh khoang cach tu thoi gian do duoc
        Arr_tgXungXuong[viTriHS] = thoiGianHT;                                                              // luu thoi gian xung xuong de tinh khoang cach sau nay
        ArrayHS[viTriHS].khoangCach = (uint16_t)(Arr_tgXungXuong[viTriHS] - Arr_tgXungLen[viTriHS]) / 58.0; // tinh khoang cach tu thoi gian xung len va xung xuong
    }
}

void kiemTraHS()
{
    isBlocked = false; // Mặc định là không có vật cản

    // 1. Kiểm tra 4 cảm biến siêu âm
    for (int i = 0; i < 4; i++)
    {
        // Chỉ quan tâm từ 2cm đến 100cm
        if (ArrayHS[i].khoangCach > 2.0f && ArrayHS[i].khoangCach <= 100.0f)
        {
            isBlocked = true; // Phát hiện vật cản!
            break;            // Thoát vòng lặp kiểm tra ngay lập tức
        }
    }
    // 2. Xóa dữ liệu đo của chu kỳ này
    for (int i = 0; i < 4; i++)
    {
        ArrayHS[i].khoangCach = 0.0f;
    }
}