#include "i2c_software.hpp"

// ================== INSTANCE ==================
I2C_software myI2cInstance;

// ================== DELAY ==================

// I2C cần một khoảng trễ nhỏ để bên đầu còn lại phản hồi, em thử không gọi hàm này thì I2C lỗi
// và khi gọi hàm Delay thì không lỗi và em thử để trống hàm thì cũng không lỗi luôn nên hiện tại
// em vẫn để hàm để các hàm gửi nhận khi gọi hàm này biết đó là quảng nghỉ

static inline void I2C_Delay(void)
{
    // xung hệ thống 84Mhz -> fast mode 407kHz
    for (volatile uint16_t i = 0; i < 3; i++)
    {
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
    }
}

// ================== CTOR / DTOR ==================
I2C_software::I2C_software() {}
I2C_software::~I2C_software() {}

// ================== INIT ==================
void I2C_software::Init_i2c_software(GPIO_TypeDef *port_scl, uint16_t scl,
                                     GPIO_TypeDef *port_sda, uint16_t sda)
{
    i2cPin_.scl.init(port_scl, scl); // Yêu cầu: OUTPUT OPEN-DRAIN
    i2cPin_.sda.init(port_sda, sda); // Yêu cầu: OUTPUT OPEN-DRAIN

    SDA_OD(); // Thả bus (High) - Đã thêm ()
    SCL_OD(); // Đã thêm ()
}

// ================== START ==================
void I2C_software::Start()
{
    SDA_OD();
    SCL_OD();
    I2C_Delay();

    SDA_L(); // SDA 1->0 khi SCL=1
    I2C_Delay();

    SCL_L(); // Giữ SCL thấp
    I2C_Delay();
}

// ================== STOP ==================
void I2C_software::Stop()
{
    SCL_L();
    SDA_L();
    I2C_Delay();

    SCL_OD();
    I2C_Delay();

    SDA_OD(); // SDA 0->1 khi SCL=1
    I2C_Delay();
}

// ================== WRITE BYTE ==================
uint8_t I2C_software::Write(uint8_t data)
{

#define I2C_WRITE_BIT()  \
    if ((data >> 7) & 1) \
        SDA_OD();        \
    else                 \
        SDA_L();         \
    I2C_Delay();         \
    SCL_OD();            \
    I2C_Delay();         \
    SCL_L();             \
    I2C_Delay();         \
    data <<= 1;

    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();
    I2C_WRITE_BIT();

#undef I2C_WRITE_BIT

    // ---- ACK ----
    SDA_OD(); // Thả SDA để Slave kéo
    I2C_Delay();
    SCL_OD();
    I2C_Delay();

    uint8_t ack = readSDA(); // 0 là ACK

    SCL_L();
    I2C_Delay();

    return (ack == 0); // Trả về 1 nếu thành công
}

// ================== READ BYTE ==================
uint8_t I2C_software::Read(uint8_t ack)
{
    uint8_t data = 0;
    SDA_OD(); // Thả SDA làm input

#define I2C_READ_BIT() \
    data <<= 1;        \
    SCL_OD();          \
    I2C_Delay();       \
    data |= readSDA(); \
    SCL_L();           \
    I2C_Delay();

    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();
    I2C_READ_BIT();

#undef I2C_READ_BIT

    // ---- ACK / NACK ----
    if (ack) // ack=1 -> NACK (High)
        SDA_OD();
    else // ack=0 -> ACK (Low)
        SDA_L();

    I2C_Delay();
    SCL_OD();
    I2C_Delay();
    SCL_L();
    I2C_Delay();

    SDA_OD(); // Thả bus kết thúc
    return data;
}

// ================== C WRAPPER ==================
extern "C"
{
    void I2C_software_Init(GPIO_TypeDef *port_scl, uint16_t scl,
                           GPIO_TypeDef *port_sda, uint16_t sda)
    {
        myI2cInstance.Init_i2c_software(port_scl, scl, port_sda, sda);
    }

    void I2C_Start(void) { myI2cInstance.Start(); }
    void I2C_Stop(void) { myI2cInstance.Stop(); }
    uint8_t I2C_Write(uint8_t data) { return myI2cInstance.Write(data); }
    uint8_t I2C_Read(uint8_t ack) { return myI2cInstance.Read(ack); }
}