#include "HCSR04.hpp"
#include "tim.h"

HCSR04::HCSR04(GPIO_TypeDef *_TRIG_Port, uint16_t _TRIG_Pin)
    : TRIG_Port(_TRIG_Port),
      TRIG_Pin(_TRIG_Pin) {}

void HCSR04::kichHoatTRIG()
{
    // Lấy mốc thời gian bắt đầu
    uint16_t tgTrigCao = __HAL_TIM_GET_COUNTER(&htim9);

    // Bật chân TRIG lên HIGH
    HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_SET);

    // Chờ đúng 10 micro-giây (Dùng ép kiểu uint16_t để chống lỗi tràn timer)
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - tgTrigCao) < 10)
        ;

    // Kéo chân TRIG xuống LOW
    HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);
}

// xac dinh vi tri cua cam bien sieu am dua vao chan gpio
int layViTriUS(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ECHO1_Pin)
        return 0;
    else if (GPIO_Pin == ECHO2_Pin)
        return 1;
    // else if (GPIO_Pin == ECHO3_Pin)
    //     return 2;
    // else if (GPIO_Pin == ECHO4_Pin)
    //     return 3;
    return -1;
}

// Mảng chứa 4 cảm biến siêu âm
HCSR04 Arr_HCSR04[2] = {HCSR04(TRIG1_GPIO_Port, TRIG1_Pin),
                        HCSR04(TRIG2_GPIO_Port, TRIG2_Pin)};

                        // HCSR04(TRIG3_GPIO_Port, TRIG3_Pin)};                        
                        // HCSR04(TRIG4_GPIO_Port, TRIG4_Pin)};

// Hàm xử lí ngắt
void xuLiNgat(uint16_t GPIO_Pin, uint16_t _tgXungHT, GPIO_PinState _trangThaiChan)
{
    int viTriUS = layViTriUS(GPIO_Pin); // lay vi tri cua cam bien trong mang tu GPIO_Pin
    if (viTriUS == -1)
        return; // neu GPIO_Pin khong phu hop voi bat ky cam bien nao thi thoat khoi ham
    if (_trangThaiChan == GPIO_PIN_SET)
    {
        Arr_HCSR04[viTriUS].tgXungLen = _tgXungHT; // luu thoi gian xung len de tinh khoang cach sau nay
    }
    else
    {
        // neu chan echo dang thap thi tinh khoang cach tu thoi gian do duoc
        Arr_HCSR04[viTriUS].tgXungXuong = _tgXungHT;
        // 1us xung o muc cao = 1cm
        Arr_HCSR04[viTriUS].khoangCach = (uint16_t)(Arr_HCSR04[viTriUS].tgXungXuong - Arr_HCSR04[viTriUS].tgXungLen) / 58.0;
    }
}

bool isBlocked = false; // bien bao trang thai co vat can hay khong, dung de dieu khien robot tranh vat can

// Kiem tra xem cam bien siêu âm co phat hien vat can hay khong
void kiemTraUS()
{
    isBlocked = false; // Mặc định là không có vật cản

    // 1. Kiểm tra 4 cảm biến siêu âm
    for (int i = 0; i < 4; i++)
    {
        // Chỉ quan tâm từ 2cm đến 100cm
        if (Arr_HCSR04[i].khoangCach > 2.0f && Arr_HCSR04[i].khoangCach <= 100.0f)
        {
            isBlocked = true; // Phát hiện vật cản!
            break;            // Thoát vòng lặp kiểm tra ngay lập tức
        }
    }
    // 2. Xóa dữ liệu đo của chu kỳ này - tam thời không xóa dữ liệu khoảng cách nữa
    // for (int i = 0; i < 4; i++)
    // {
    //     Arr_HCSR04[i].khoangCach = 0.0f;
    // }
}

// Hàm kích hoạt toàn bộ cảm biến siêu âm để đo khoảng cách
uint32_t tgDoUS = 0; // Biến toàn cục để lưu thời gian bắt đầu đo khoảng cách
int ttDoUS = 0;      // thu tu kich hoat cam bien sieu am (0-3)
void doKhoangCach()
{
    if (HAL_GetTick() - tgDoUS < 25) // Chỉ đo khoảng cách mỗi 25ms
        return;
    kiemTraUS();
    tgDoUS = HAL_GetTick(); // Cập nhật thời gian bắt đầu đo
    Arr_HCSR04[ttDoUS].kichHoatTRIG();
    ttDoUS++;
    if (ttDoUS > 3)
    {
        ttDoUS = 0;
    }
}