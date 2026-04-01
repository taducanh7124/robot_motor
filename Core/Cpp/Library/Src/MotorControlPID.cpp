#include "MotorControlPID.hpp"
#include <math.h> // Để dùng hàm fabs() lấy trị tuyệt đối

// Khởi tạo PID và gọi khởi tạo phần cứng từ class cha
void MotorControlPID::initPID(TIM_HandleTypeDef *htim, uint32_t timerChannel, GPIO_TypeDef* dirPort, uint16_t dirPin, float Kp, float Ki, float Kd, float out_max) {
    // 1. Khởi tạo PWM và GPIO thông qua hàm init của class cha (motorControl)
    this->init(htim, timerChannel, dirPort, dirPin);

    // 2. Gán thông số PID
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.out_max = out_max;
    
    // 3. Xóa các biến rác
    resetPID();
}

// Thay đổi mục tiêu vận tốc
void MotorControlPID::setSetpoint(float target_speed) {
    pid.setpoint = target_speed;
}

// Cập nhật lại bộ thông số Kp, Ki, Kd nếu cần
void MotorControlPID::setTunings(float Kp, float Ki, float Kd) {
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
}

// Đặt lại các biến trạng thái
void MotorControlPID::resetPID() {
    pid.setpoint = 0.0f;
    pid.error_sum = 0.0f;
    pid.prev_error = 0.0f;
}

// Tính toán toán học và điều khiển động cơ
void MotorControlPID::computeAndControl(float current_speed) {
    // 1. Tính sai số
    float error = pid.setpoint - current_speed;

    // 2. Khâu Tỉ lệ (Proportional)
    float P_out = pid.Kp * error;

    // 3. Khâu Tích phân (Integral)
    pid.error_sum += error;
    float I_out = pid.Ki * pid.error_sum;

    // 4. Khâu Vi phân (Derivative)
    float D_out = pid.Kd * (error - pid.prev_error);
    pid.prev_error = error;

    // 5. Tổng hợp tín hiệu điều khiển
    float output = P_out + I_out + D_out;

    // 6. Anti-Windup (Chống bão hòa khâu tích phân)
    // Nếu output vượt quá giới hạn, giới hạn nó lại. 
    // Đồng thời không cho error_sum tiếp tục tăng/giảm vô lý.
    if (output > pid.out_max) {
        output = pid.out_max;
        // Trừ bớt phần lỗi vừa cộng vào để I không bị quá tải
        pid.error_sum -= error; 
    } 
    else if (output < -pid.out_max) {
        output = -pid.out_max;
        pid.error_sum -= error;
    }

    // 7. Chuyển tín hiệu Output (âm/dương) thành Tốc độ (PWM) và Chiều quay (Direction)
    uint16_t pwm_val = (uint16_t)fabs(output);
    
    // Giả sử class `motorControl` có enum DIR_FORWARD và DIR_REVERSE
    // Bạn cần đổi tên enum dưới đây cho khớp với thư viện gốc của bạn.
    if (output >= 0) {
        // Gọi hàm control từ class cha: điều khiển PWM và hướng
        this->control(pwm_val, DIR_FORWARD); 
    } else {
        this->control(pwm_val, DIR_REVERSE);
    }
}

// Ghi đè hàm stop
void MotorControlPID::stop() {
    // Gọi hàm stop của class cha để ngắt PWM cứng
    motorControl::stop(); 
    
    // Reset luôn bộ toán học để khi chạy lại không bị lao lên đột ngột
    resetPID();
}




// wrapper C
void computeAndControlWrapper(MotorControlPID* motor, float current_speed){
    motor->computeAndControl(current_speed);
}