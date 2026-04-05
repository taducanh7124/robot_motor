#ifndef _I2C_SOFTWARE_
#define _I2C_SOFTWARE_

// ================= include =================
#ifdef __cplusplus
#include "main.h"
#include "stdint.h"
#include "pin_config.hpp" // Bắt buộc phải có file này định nghĩa PinConfig_out
#else
#include "main.h"
#include "stdint.h"
#endif

// ================= định nghĩa cho C++ =================
#ifdef __cplusplus
class I2C_software
{
private:
    struct
    {
        PinConfig_out scl;
        PinConfig_out sda;
    } i2cPin_;

public:
    I2C_software();
    ~I2C_software();
    void Init_i2c_software(GPIO_TypeDef *port_scl, uint16_t scl,
                           GPIO_TypeDef *port_sda, uint16_t sda);
    uint8_t Write(uint8_t data);
    uint8_t Read(uint8_t ack);
    void Stop();
    void Start();

private:
    // --- Inline Control (Direct Register Access) ---
    // BSRR Low 16 bits = Set (High/Float)
    inline void SDA_OD() { *i2cPin_.sda.bsrrReg = i2cPin_.sda.pinMask; }
    // BSRR High 16 bits = Reset (Low) -> Tương đương ghi vào BRR
    inline void SDA_L()  { *i2cPin_.sda.bsrrReg = (i2cPin_.sda.pinMask) << 16; }
    
    inline void SCL_OD() { *i2cPin_.scl.bsrrReg = i2cPin_.scl.pinMask; }
    inline void SCL_L()  { *i2cPin_.scl.bsrrReg = (i2cPin_.scl.pinMask) << 16; }

    // Đọc IDR và Mask bit
    inline uint8_t readSDA() { 
        return (*i2cPin_.sda.idrReg & i2cPin_.sda.pinMask) ? 1 : 0; 
    }
};

extern I2C_software myI2cInstance;
#endif // __cplusplus

// ================= định nghĩa cho C (Wrapper) =================
#ifdef __cplusplus
extern "C"
{
#endif
    void I2C_software_Init(GPIO_TypeDef *port_scl, uint16_t scl,
                           GPIO_TypeDef *port_sda, uint16_t sda);
    void I2C_Start(void);
    void I2C_Stop(void);
    uint8_t I2C_Write(uint8_t data);
    uint8_t I2C_Read(uint8_t ack);
#ifdef __cplusplus
}
#endif
#endif // _I2C_SOFTWARE_

// Lưu ý: Các chân GPIO phải cài đặt ở chế độ OUTPUT OPEN-DRAIN (GPIO_MODE_OUTPUT_OD)