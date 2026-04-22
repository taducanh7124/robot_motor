#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "main.h"

class MPU6050
{
public:
    I2C_HandleTypeDef *hi2c;          // Bộ I2C để giao tiếp với cảm biến
    const uint8_t dia_chi_i2c = 0xD0; // Địa chỉ I2C của cảm biến 0x68 dịch trái 1 bit do ham doc ghi bo nho hal

    TIM_HandleTypeDef *htim;   // Timer đếm us để đếm thời gian dt -> tính góc z
    uint16_t tg_do_imu_qk = 0; // Lưu mốc thời gian của Hardware Timer

    float do_lech_z = 0; // Sai số tĩnh ban đầu (Offset của Gyro Z)
    float goc_z = 0;     // Góc Yaw (Độ) - Tích phân từ vận tốc
    float vt_goc_z = 0;  // Vận tốc góc Yaw (Độ/giây) - Đọc trực tiếp

    // Hàm khởi tạo
    MPU6050();
    void init(I2C_HandleTypeDef *_hi2c, TIM_HandleTypeDef *_htim); 
    // Các hàm xử lý chính
    bool cauHinh();   // Đánh thức & Cấu hình phần cứng (DLPF, Range, Sample Rate)
    void hieuChuan(); // Xả rác và tìm sai số tĩnh (chạy lúc xe đứng im)
    void tinhGocZ();  // Đọc dữ liệu và tính thông số góc z
};

extern MPU6050 Obj_MPU6050;
#endif