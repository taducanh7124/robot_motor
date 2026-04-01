#ifndef INIT_GPIO_HPP
#define INIT_GPIO_HPP

#include "stdint.h"
#include "main.h"

struct PinConfig_out
{
    // GPIO_TypeDef* port; // Ví dụ: GPIOA, GPIOB
    // uint16_t pin;       // Ví dụ: GPIO_PIN_0
    volatile uint32_t *bsrrReg; // Nơi lưu địa chỉ thanh ghi để BẬT (Set)
    volatile uint32_t *ordReg;
    // phuc vừa in, vừa out cho I2C
    volatile uint32_t *idrReg;
    uint32_t pinMask;

    // Hàm cài đặt (học lệnh)
    void init(GPIO_TypeDef *port, uint16_t pin)
    {
        bsrrReg = &(port->BSRR); // Lưu địa chỉ thanh ghi BSRR của port đó
        ordReg = &(port->ODR);
        idrReg = &(port->IDR);
        pinMask = pin; // Lưu vị trí chân
    }

    // HIGH
    inline void high() const
    {
        *bsrrReg = pinMask; // Ghi thẳng vào địa chỉ đã lưu -> Tốc độ cực nhanh
    }
    // LOW
    inline void low() const
    {
        *bsrrReg = pinMask << 16u; // Ghi thẳng vào địa chỉ đã lưu -> Tốc độ cực nhanh
    }
};

struct PinConfig_in
{
    volatile uint32_t *idrReg; // Nơi lưu địa chỉ thanh ghi để ĐỌC (Input Data Register)
    uint32_t pinMask;

    // Hàm cài đặt
    void init(GPIO_TypeDef *port, uint16_t pin)
    {
        idrReg = &(port->IDR); // Lưu địa chỉ thanh ghi IDR của port đó
        pinMask = pin;         // Lưu vị trí chân
    }

    // ĐỌC TRẠNG THÁI
    inline bool read() const
    {
        return ((*idrReg) & pinMask) != 0; // Trả về true nếu chân ở mức cao, false nếu mức thấp
    }
};

// Sửa tham số State thành bool hoặc kiểm tra rõ ràng
inline uint32_t generate_lut(uint16_t GPIO_Pin, bool isHigh)
{
    if (isHigh)
    {
        return (uint32_t)GPIO_Pin; // Bit thấp: SET
    }
    else
    {
        return (uint32_t)GPIO_Pin << 16u; // Bit cao: RESET
    }
}
#endif // INIT_GPIO_HPP

/**
 * file này bắt buộc phải inlucde ở các chương trình điều khiển GPIO, nó giúp ánh xạ biến trong chương trình đến IO
 */