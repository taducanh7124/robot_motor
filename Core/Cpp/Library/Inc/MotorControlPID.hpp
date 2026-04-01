// #ifndef MOTOR_CONTROL_PID
// #define MOTOR_CONTROL_PID

// #include "MotorControl.hpp"

// class MotorControlPID : public motorControl
// {

// private:
//     // Cấu trúc dữ liệu cho PID
//     typedef struct {
//         float Kp, Ki, Kd;       // Các hệ số
//         float setpoint;         // Vận tốc mong muốn (xung/10ms)
//         float error_sum;        // Khâu tích phân (I) cộng dồn
//         float prev_error;       // Sai số quá khứ để tính Vi phân (D)
//         float out_max;          // Giới hạn PWM lớn nhất (VD: 1000)
//     } PID_Controller_t;

//     PID_controller_t pid;

// public:
//     void init(TIM_HandleTypeDef *htim, uint32_t timerChanel, GPIO_TypeDef* dirPort, uint16_t dirPin);
//     void initPID(TIM_HandleTypeDef *htim, uint32_t timerChanel, GPIO_TypeDef* dirPort, uint16_t dirPin, float Kp, float Ki, float Kd, float out_max);
//     void control(uint16_t speed, MotorDir dir);
//     void stop();
// };

// #endif // MOTOR_CONTROL_PID

#ifndef MOTOR_CONTROL_PID_HPP
#define MOTOR_CONTROL_PID_HPP

#include "MotorControl.hpp"

// Lưu ý: Đảm bảo tên class cha "motorControl" khớp chính xác với thư viện của bạn
class MotorControlPID : public motorControl 
{
private:
    // Cấu trúc dữ liệu cho PID
    typedef struct {
        float Kp, Ki, Kd;       // Các hệ số
        float setpoint;         // Vận tốc mong muốn (xung/10ms)
        float error_sum;        // Khâu tích phân (I) cộng dồn
        float prev_error;       // Sai số quá khứ để tính Vi phân (D)
        float out_max;          // Giới hạn PWM lớn nhất (VD: 1000)
    } PID_Controller_t;

    PID_Controller_t pid;       // Đã sửa lại lỗi viết hoa/thường (PID_Controller_t)

public:
    // Các hàm khởi tạo ban đầu của bạn
    void init(TIM_HandleTypeDef *htim, uint32_t timerChannel, GPIO_TypeDef* dirPort, uint16_t dirPin);
    void initPID(TIM_HandleTypeDef *htim, uint32_t timerChannel, GPIO_TypeDef* dirPort, uint16_t dirPin, float Kp, float Ki, float Kd, float out_max);
    
    // --- CÁC HÀM PHÁT TRIỂN THÊM ---

    // Thay đổi vận tốc mục tiêu một cách linh hoạt
    void setSetpoint(float target_speed);

    // Thay đổi hệ số PID on-the-fly (dùng khi muốn auto-tune hoặc đổi mode)
    void setTunings(float Kp, float Ki, float Kd);

    // Hàm tính toán cốt lõi: Phải được gọi ĐỊNH KỲ trong ngắt Timer (Ví dụ: mỗi 10ms)
    // Truyền vận tốc hiện tại (current_speed) tính được từ Encoder vào đây
    void computeAndControl(float current_speed);

    // Reset các biến tích phân/vi phân (Cực kỳ quan trọng để chống Wind-up khi dừng/chạy lại)
    void resetPID();

    // Bạn có thể ghi đè (override) hàm stop để đồng thời dừng PWM và reset biến PID
    void stop();
};


// wraper C
#ifdef __cplusplus
extern "C" {
#endif

// ngat timer 10ms de goi ham computeAndControl() cua tung doi tuong MotorControlPID tu main_cpp.cpp
void computeAndControlWrapper(MotorControlPID* motor, float current_speed);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_PID_HPP