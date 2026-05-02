#include "MPU6050.hpp"

MPU6050::MPU6050() : hi2c(nullptr), htim(nullptr) {}

void MPU6050::init(I2C_HandleTypeDef *_hi2c, TIM_HandleTypeDef *_htim)
{
    hi2c = _hi2c;
    htim = _htim;
}

bool MPU6050::cauHinh()
{
    if (hi2c == nullptr)
        return false; // Thoat neu i2c chua duoc khoi tao

    uint8_t du_lieu_reg; // Bien luu du lieu ghi vao thanh ghi trong cam bien quan tinh

    // 1. Đánh thức cảm biến (Reset sleep mode)
    du_lieu_reg = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, dia_chi_i2c, 0x6B, 1, &du_lieu_reg, 1, 100) != HAL_OK)
        return false;
    HAL_Delay(10);

    // 2. Kích hoạt DLPF (Bộ lọc thông thấp)
    // Giá trị 0x03: Lọc nhiễu Gyro trên 42Hz
    du_lieu_reg = 0x03;
    HAL_I2C_Mem_Write(hi2c, dia_chi_i2c, 0x1A, 1, &du_lieu_reg, 1, 100);

    // 3. Cấu hình độ phân giải Gyro
    // Giá trị 0x00: Dải đo +- 250 độ/s (Độ phân giải tối đa: 131 LSB/độ/s)
    du_lieu_reg = 0x00;
    HAL_I2C_Mem_Write(hi2c, dia_chi_i2c, 0x1B, 1, &du_lieu_reg, 1, 100);

    // 4. Cấu hình độ phân giải Gia tốc kế (Accelerometer Configuration)
    // Giá trị 0x00: Dải đo +- 2g (Độ phân giải tối đa: 16384 LSB/g)
    du_lieu_reg = 0x00;
    HAL_I2C_Mem_Write(hi2c, dia_chi_i2c, 0x1C, 1, &du_lieu_reg, 1, 100);

    // 5. Đồng bộ tốc độ lấy mẫu (Sample Rate Divider)
    // Tần số mẫu = 1kHz / (1 + SMPLRT_DIV).
    // Ghi 0x09 => 1000 / (1+9) = 100Hz (Cập nhật dữ liệu mỗi 10ms là đủ cho Odometry)
    du_lieu_reg = 0x09;
    HAL_I2C_Mem_Write(hi2c, dia_chi_i2c, 0x19, 1, &du_lieu_reg, 1, 100);

    return true;
}

void MPU6050::hieuChuan()
{
    long tong_vt_goc_z = 0;  // Biến tạm để tính tổng vận tốc góc Z trong quá trình hiệu chuẩn
    int so_mau = 1000;       // Số mẫu để lấy trung bình (Càng nhiều càng chính xác nhưng mất thời gian)
    uint8_t vt_goc_z_reg[2]; // Để đọc 2 byte của thanh ghi dữ liệu Gyro Z 8 bit cao va 8 bit thap

    HAL_Delay(500); // Đợi robot ổn định

    for (int i = 0; i < so_mau; i++)
    {
        // Chỉ đọc 2 byte Gyro Z (0x47, 0x48)
        if (HAL_I2C_Mem_Read(hi2c, dia_chi_i2c, 0x47, 1, vt_goc_z_reg, 2, 10) == HAL_OK)
        {
            int16_t vt_goc_z_tho = (int16_t)(vt_goc_z_reg[0] << 8 | vt_goc_z_reg[1]); // Kết hợp 2 byte thành giá trị 16 bit có dấu
            tong_vt_goc_z += vt_goc_z_tho;
        }
        HAL_Delay(1);
    }
    // Chia trung bình và hệ số 131.0
    do_lech_z = ((float)tong_vt_goc_z / so_mau) / 131.0f;
}

void MPU6050::tinhGocZ()
{
    // 1. TÍNH dt BẰNG TIMER 10
    uint16_t tg_do_imu_ht = __HAL_TIM_GET_COUNTER(htim); // Thoi gian hien tai cua timer (us)
    uint16_t tg_do_imu;                                  // Thoi gian dt tinh tu lan doc truoc (us)

    // Xử lí tràn timer
    if (tg_do_imu_ht >= tg_do_imu_qk)
    {
        tg_do_imu = tg_do_imu_ht - tg_do_imu_qk; // Neu khong tran
    }
    else
    { // Neu tran, thoi gian qua khu + thoi gian hien tai + 1 chuyen doi tu 65535 sang 0
        tg_do_imu = (65535 - tg_do_imu_qk) + tg_do_imu_ht + 1;
    }
    tg_do_imu_qk = tg_do_imu_ht;

    float dt = tg_do_imu / 1000000.0f; // Đổi us sang Giây

    // 2. ĐỌC VÀ TÍNH TOÁN
    uint8_t vt_goc_z_reg[2];
    if (HAL_I2C_Mem_Read(hi2c, dia_chi_i2c, 0x47, 1, vt_goc_z_reg, 2, 10) == HAL_OK)
    {
        int16_t vt_goc_z_tho = (int16_t)(vt_goc_z_reg[0] << 8 | vt_goc_z_reg[1]);

        // Tính tốc độ (Đã loại bỏ sai số tĩnh)
        vt_goc_z = (vt_goc_z_tho / 131.0f) - do_lech_z;

        // Lọc nhiễu Deadband (Dưới 0.2 độ/s coi như xe đứng im)
        if (vt_goc_z > -0.2f && vt_goc_z < 0.2f)
        {
            vt_goc_z = 0.0f;
        }

        // Tích phân ra góc hướng
        goc_z += vt_goc_z * dt;
        // Chuẩn hóa đưa góc về dải -180 đến 180 độ
        if (goc_z > 180.0f)
        {
            goc_z -= 360.0f;
        }
        else if (goc_z < -180.0f)
        {
            goc_z += 360.0f;
        }
    }
}

void resetMPU()
{
    // 1. TẮT NGUỒN CẢM BIẾN (Reset)
    // Kéo PC14 xuống mức THẤP (0V) để KHÓA transistor PNP, ngắt hoàn toàn nguồn VCC vào MPU.
    HAL_GPIO_WritePin(RST_MPU_GPIO_Port, RST_MPU_Pin, GPIO_PIN_RESET);
    //HAL_Delay(500); // Chờ 500ms để tụ trên mạch MPU xả sạch điện về 0V.

    // 2. BẬT NGUỒN TRỞ LẠI (Hoạt động bình thường)
    // Kéo PC14 lên mức CAO (3.3V) để MỞ transistor PNP, cấp nguồn VCC trở lại cho MPU.
    HAL_GPIO_WritePin(RST_MPU_GPIO_Port, RST_MPU_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Chờ 100ms để MPU boot xong firmware nội bộ trước khi giao tiếp I2C.
}