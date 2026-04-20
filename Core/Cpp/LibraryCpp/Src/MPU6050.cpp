#include "MPU6050.hpp"

MPU6050::MPU6050() {
    hi2c = nullptr;
    htim = nullptr;
    address = 0;
    
    do_lech_z = 0.0f;
    lastTime = 0;
    goc_z = 0.0f;
    vt_goc_z = 0.0f;
}

void MPU6050::init(I2C_HandleTypeDef* _hi2c, TIM_HandleTypeDef* _htim, uint8_t i2c_addr) {
    hi2c = _hi2c;
    htim = _htim;
    address = i2c_addr;
}

bool MPU6050::begin() {
    if (hi2c == nullptr) return false; 

    uint8_t data;

    // 1. Đánh thức cảm biến (Reset sleep mode)
    data = 0x00; 
    if (HAL_I2C_Mem_Write(hi2c, address, 0x6B, 1, &data, 1, 100) != HAL_OK) return false;
    HAL_Delay(10);

    // 2. Kích hoạt DLPF (Bộ lọc thông thấp)
    // Giá trị 0x03: Lọc nhiễu Gyro trên 42Hz (Rất mượt cho Robot di chuyển)
    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, address, 0x1A, 1, &data, 1, 100);

    // 3. Cấu hình độ phân giải Gyro (Gyroscope Configuration)
    // Giá trị 0x00: Dải đo +- 250 độ/s (Độ phân giải tối đa: 131 LSB/độ/s)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, address, 0x1B, 1, &data, 1, 100);

    // 4. Cấu hình độ phân giải Gia tốc kế (Accelerometer Configuration)
    // Giá trị 0x00: Dải đo +- 2g (Độ phân giải tối đa: 16384 LSB/g)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, address, 0x1C, 1, &data, 1, 100);

    // 5. Đồng bộ tốc độ lấy mẫu (Sample Rate Divider)
    // Tần số mẫu = 1kHz / (1 + SMPLRT_DIV). 
    // Ghi 0x09 => 1000 / (1+9) = 100Hz (Cập nhật dữ liệu mỗi 10ms là đủ cho Odometry)
    data = 0x09;
    HAL_I2C_Mem_Write(hi2c, address, 0x19, 1, &data, 1, 100);

    return true;
}

void MPU6050::hieuChuan() {
    long tong_gz = 0;
    int so_mau = 1000;
    uint8_t buf[2];

    HAL_Delay(500); // Đợi robot ổn định

    for (int i = 0; i < so_mau; i++) {
        // Chỉ đọc 2 byte Gyro Z (0x47, 0x48)
        if (HAL_I2C_Mem_Read(hi2c, address, 0x47, 1, buf, 2, 10) == HAL_OK) {
            int16_t gia_tri_tho = (int16_t)(buf[0] << 8 | buf[1]);
            tong_gz += gia_tri_tho;
        }
        HAL_Delay(1); 
    }
    
    // Chia trung bình và hệ số 131.0
    do_lech_z = ((float)tong_gz / so_mau) / 131.0f;
}

void MPU6050::tinhGocZ() {
    // 1. TÍNH dt BẰNG TIMER 9
    uint16_t currentTime = __HAL_TIM_GET_COUNTER(htim);
    uint16_t elapsed_us;

    if (currentTime >= lastTime) {
        elapsed_us = currentTime - lastTime;
    } else {
        elapsed_us = (65535 - lastTime) + currentTime + 1; 
    }
    lastTime = currentTime;

    float dt = elapsed_us / 1000000.0f; // Đổi us sang Giây

    // 2. ĐỌC VÀ TÍNH TOÁN
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, address, 0x47, 1, buf, 2, 10) == HAL_OK) {
        int16_t gia_tri_tho = (int16_t)(buf[0] << 8 | buf[1]);

        // Tính tốc độ (Đã loại bỏ sai số tĩnh)
        vt_goc_z = (gia_tri_tho / 131.0f) - do_lech_z;

        // Lọc nhiễu Deadband (Dưới 0.2 độ/s coi như xe đứng im)
        if (vt_goc_z > -0.2f && vt_goc_z < 0.2f) {
            vt_goc_z = 0.0f;
        }

        // Tích phân ra góc hướng
        goc_z += vt_goc_z * dt;
    }
}