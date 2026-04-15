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

HyperSonic ArrayHS[4];   // mang 4 doi tuong cam bien sieu am

int viTriHS = 0; // vi tri cam bien sieu am trong mang

bool isBlocked = false;  // bien bao trang thai co vat can hay khong, dung de dieu khien robot tranh vat can
bool isTrigHigh = false; // co bao trang thai chan trig dang cao hay thap

uint32_t Arr_tgXungLen[4] = {0};   // bien luu thoi gian xung len de tinh khoang cach sau nay
uint32_t Arr_tgXungXuong[4] = {0}; // bien luu thoi gian xung xuong de tinh khoang cach sau nay
uint32_t thoiGianHT = 0;           // bien luu thoi gian hien tai de tinh thoi gian xung len va xung xuong

void kichHoatTrig()
{
    __HAL_TIM_SET_COUNTER(&htim9, 0);                          // dat lai gia tri timer9 ve 0
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); // dat chan trig cao trong 10 micro giay
    while (__HAL_TIM_GET_COUNTER(&htim9) < 10)
        ;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // dat chan trig thap
    __HAL_TIM_SET_COUNTER(&htim9, 0);                            // bat timer de tinh thoi gian echo nhan tin hien phan hoi
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
        Arr_tgXungXuong[viTriHS] = thoiGianHT;                                                    // luu thoi gian xung xuong de tinh khoang cach sau nay
        ArrayHS[viTriHS].khoangCach = (Arr_tgXungXuong[viTriHS] - Arr_tgXungLen[viTriHS]) / 58.0; // tinh khoang cach tu thoi gian xung len va xung xuong
    }
}

/*
- ngat khi xung len va xung xuong
- xung len sau khi kich hoat trig
- xung xuong khi echo nhan tin hieu song sieu am phan hoi
- khoang thoi gian giu xung len va xung xuong dung de tinh khoang cach
*/
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    thoiGianHT = __HAL_TIM_GET_COUNTER(&htim9);                      // lay gia tri timer9 de tinh thoi gian
    GPIO_PinState trangThaiChan = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin); // doc trang thai chan echo dang cao hay thap
    xuLiNgat(GPIO_Pin, thoiGianHT, trangThaiChan);                   // xu ly ngat va tinh khoang cach tu thoi gian do duoc
}