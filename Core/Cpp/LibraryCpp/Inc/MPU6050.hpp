#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "main.h"

class MPU6050
{
public:
    I2C_HandleTypeDef *hi2c; // Giao tiếp qua I2C
    TIM_HandleTypeDef *htim; // Timer đếm us để tính dt
    uint8_t address;         // Địa chỉ I2C của cảm biến

    float do_lech_z;   // Sai số tĩnh ban đầu (Offset của Gyro Z)
    uint16_t lastTime; // Lưu mốc thời gian của Hardware Timer

    float goc_z;    // Góc Yaw (Độ) - Tích phân từ vận tốc
    float vt_goc_z; // Vận tốc góc Yaw (Độ/giây) - Đọc trực tiếp

    MPU6050(); 

    // Hàm nạp thông số phần cứng
    void init(I2C_HandleTypeDef *_hi2c, TIM_HandleTypeDef *_htim, uint8_t i2c_addr = (0x68 << 1));

    // Các hàm xử lý chính
    bool begin();     // Đánh thức & Cấu hình phần cứng (DLPF, Range, Sample Rate)
    void hieuChuan(); // Xả rác và tìm sai số tĩnh (chạy lúc xe đứng im)
    void tinhGocZ();  // Đọc data và tích phân góc
};

extern MPU6050 mpu;
#endif // MPU6050_HPP