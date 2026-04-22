#ifndef ROOT_HPP
#define ROOT_HPP

#include "main.h"

// Khai bao hang so
#define KHOANGCACH2BANH 0.4 // MET Khoang cach giua 2 banh xe (met) - dung de tinh van toc goc xoay
#define TYSOTRUYEN 19.2     // TYSOTRUYEN Ty so truyen dong tu dong co den banh xe theo tai lieu
#define SUONDEM 4           // SUONDEM So suon dem cua dong co dem len va dem xuong cua xung 2 kenh
#define CHUVIBANH 0.3       // MET Chu vi banh xe
#define XUNG1VONG 500       // XUNG So xung 1 vong quay theo ly thuyet

#define TONGXUNG 38400         // XUNG So xung 1 vong quay theo tinh toan thuc te xung1vong * tysotruyen * suondem
#define MET1XUNG 0.0000078125f // MET So met tuong duong 1 xung
#define XUNG1MET 128000.0f     // XUNG So xung tuong duong 1 met

#define PI 3.14159265358979323846 // Hang so pi
#define ALPHA 0.01f             // He so de tang/giam toc tu tu cho dong co
#define BETA 0.8f                 // He so loc van toc goc

#define TIMER_10MS 10
#define TIMER_20MS 20
#define TIMER_50MS 50
#define TIMER_100MS 100
#define TIMER_200MS 200
#define TIMER_500MS 500
#define TIMER_1000MS 1000
#define TIMER_2000MS 2000
#define TIMER_5000MS 5000

// Hàm map đa năng, an toàn, dùng được cho cả float và int
template <typename T>
T doi_van_toc(T _vx, T in_vx_min, T in_vx_max, T out_ccr_min, T out_ccr_max) {
    // Chốt an toàn: Chống lỗi chia cho 0 gây treo chip STM32
    if (in_vx_min == in_vx_max) {
        return out_ccr_min; 
    }
    
    return (_vx - in_vx_min) * (out_ccr_max - out_ccr_min) / (in_vx_max - in_vx_min) + out_ccr_min;
}

#endif