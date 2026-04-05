// #include "main_cpp.hpp"
// #include "config.hpp"

// // void motorControl()
// // {
// //     if (robot.state.isControlNew)
// //     {
// //         robot.state.isControlNew = false;
// //         frontLeftMotor.control(robot.front_left.pwm, (MotorDir)robot.front_left.dir);
// //         frontRightMotor.control(robot.front_right.pwm, (MotorDir)robot.front_right.dir);
// //         rearLeftMotor.control(robot.rear_left.pwm, (MotorDir)robot.rear_left.dir);
// //         rearRightMotor.control(robot.rear_right.pwm, (MotorDir)robot.rear_right.dir);
// //     }
// //     else
// //     {
// //     }
// // }

// // void test()
// // {
// //     struct
// //     {
// //         PinConfig_out d1, d2, d3, d4;
// //     } dri_t;

// //     while (1)
// //     {
// //         static uint32_t timeCurent = 0;
// //         if (HAL_GetTick() - timeCurent >= 1000)
// //         {
// //             timeCurent = HAL_GetTick();
// //             dri_t.d1.high();
// //             dri_t.d2.high();
// //             dri_t.d3.high();
// //             dri_t.d4.high();
// //             HAL_Delay(100);
// //             dri_t.d1.low();
// //             dri_t.d2.low();
// //             dri_t.d3.low();
// //             dri_t.d4.low();

// //             HAL_IWDG_Refresh(&hiwdg);
// //         }
// //     }
// // }

// void vanToc()
// {
//     // 1. ĐỊNH THỜI: Đảm bảo hàm chỉ tính toán mỗi 10ms
//     static uint32_t last_time = 0;
//     if (HAL_GetTick() - last_time < 10)
//     {
//         return; // Nếu chưa đủ 10ms thì thoát hàm ngay lập tức
//     }
//     last_time = HAL_GetTick(); // Cập nhật lại mốc thời gian

//     // 2. KHAI BÁO BIẾN LƯU QUÁ KHỨ CHO 4 BÁNH
//     static uint16_t prev_encoder_FL = 0;
//     static uint16_t prev_encoder_FR = 0;
//     static uint16_t prev_encoder_RL = 0;
//     static uint16_t prev_encoder_RR = 0;

//     // 3. ĐỌC GIÁ TRỊ HIỆN TẠI (Ép kiểu 16-bit)
//     uint16_t current_FL = (uint16_t)encoderData.front_left.encoderValue;
//     uint16_t current_FR = (uint16_t)encoderData.front_right.encoderValue;
//     uint16_t current_RL = (uint16_t)encoderData.rear_left.encoderValue;
//     uint16_t current_RR = (uint16_t)encoderData.rear_right.encoderValue;

//     // 4. TÍNH DELTA VÀ LƯU VÀO STRUCT TOÀN CỤC (Để PID còn gọi ra dùng)
//     // Đơn vị lúc này chuẩn xác là: Số xung / 10ms
//     robot.front_left.velocity  = (int16_t)(current_FL - prev_encoder_FL);
//     robot.front_right.velocity = (int16_t)(current_FR - prev_encoder_FR);
//     robot.rear_left.velocity   = (int16_t)(current_RL - prev_encoder_RL);
//     robot.rear_right.velocity  = (int16_t)(current_RR - prev_encoder_RR);

//     // 5. CẬP NHẬT LẠI QUÁ KHỨ CHO CHU KỲ SAU
//     prev_encoder_FL = current_FL;
//     prev_encoder_FR = current_FR;
//     prev_encoder_RL = current_RL;
//     prev_encoder_RR = current_RR;
// }

// void encoder()
// {

//     // cap nhat moi 1ms
//     if (HAL_GetTick() % 1 == 0)
//     {
//     }
//     else
//     {
//         return;
//     }

//     robot.state.isDataNew = true;

//     encoderData.front_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim1);
//     encoderData.front_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
//     encoderData.rear_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim3);
//     encoderData.rear_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim4);

//     encoderData.front_left.dir = TIM1->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.front_right.dir = TIM2->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.rear_left.dir = TIM3->CR1 & TIM_CR1_DIR ? 1 : 0;
//     encoderData.rear_right.dir = TIM4->CR1 & TIM_CR1_DIR ? 1 : 0;
// }

// #include "math.h" // Dùng cho hàm fabs() nếu cần thiết

// void Robot_Control_Loop()
// {
//     // 1. Cập nhật vận tốc thực tế của 4 bánh
//     vanToc();

//     // 2. Cài đặt vận tốc mong muốn (Setpoint)
//     // Ví dụ: Đặt cả 4 bánh chạy với tốc độ 30 xung/10ms
//     pid_FL.setpoint = 30;
//     pid_FR.setpoint = 30;
//     pid_RL.setpoint = 30;
//     pid_RR.setpoint = 30;

//     // 3. Tính toán ngõ ra PID cho từng bánh
//     // Gọi phương thức PID_Compute từ từng object motor tương ứng
//     float out_FL = frontLeftMotor.PID_Compute(&pid_FL, robot.front_left.velocity);
//     float out_FR = frontRightMotor.PID_Compute(&pid_FR, robot.front_right.velocity);
//     float out_RL = rearLeftMotor.PID_Compute(&pid_RL, robot.rear_left.velocity);
//     float out_RR = rearRightMotor.PID_Compute(&pid_RR, robot.rear_right.velocity);

//     // 4. Băm xung và điều khiển phần cứng qua hàm control()
//     // Tách dấu để quyết định chiều quay, ép kiểu độ lớn thành uint16_t

//     // --- BÁNH TRÁI TRƯỚC (Front Left) ---
//     if (out_FL >= 0) {
//         frontLeftMotor.control((uint16_t)out_FL, MotorDir::Forward);
//     } else {
//         frontLeftMotor.control((uint16_t)(-out_FL), MotorDir::Backward);
//     }

//     // --- BÁNH PHẢI TRƯỚC (Front Right) ---
//     if (out_FR >= 0) {
//         frontRightMotor.control((uint16_t)out_FR, MotorDir::Forward);
//     } else {
//         frontRightMotor.control((uint16_t)(-out_FR), MotorDir::Backward);
//     }

//     // --- BÁNH TRÁI SAU (Rear Left) ---
//     if (out_RL >= 0) {
//         rearLeftMotor.control((uint16_t)out_RL, MotorDir::Forward);
//     } else {
//         rearLeftMotor.control((uint16_t)(-out_RL), MotorDir::Backward);
//     }

//     // --- BÁNH PHẢI SAU (Rear Right) ---
//     if (out_RR >= 0) {
//         rearRightMotor.control((uint16_t)out_RR, MotorDir::Forward);
//     } else {
//         rearRightMotor.control((uint16_t)(-out_RR), MotorDir::Backward);
//     }
// }

// void main_cpp()
// {
//     HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
//     HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
//     HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
//     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

//     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2 | TIM_CHANNEL_3 | TIM_CHANNEL_4);

// // PID = 0;

//     frontLeftMotor.initPID(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin, 0, 0, 0, 1000);
//     frontRightMotor.initPID(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin, 0, 0, 0, 1000);
//     rearLeftMotor.initPID(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin, 0, 0, 0, 1000);
//     rearRightMotor.initPID(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin, 0, 0, 0, 1000);

//     // frontLeftMotor.init(&htim3, TIM_CHANNEL_1, DIR_1_GPIO_Port, DIR_1_Pin);
//     // frontRightMotor.init(&htim3, TIM_CHANNEL_2, DIR_2_GPIO_Port, DIR_2_Pin);
//     // rearLeftMotor.init(&htim3, TIM_CHANNEL_3, DIR_3_GPIO_Port, DIR_3_Pin);
//     // rearRightMotor.init(&htim3, TIM_CHANNEL_4, DIR_4_GPIO_Port, DIR_4_Pin);

//     // // uart
//     // uartDriver.init(&huart1, bufferUART, sizeof(bufferUART));

//     // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//     // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//     // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//     // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

//     // start watch dog
//     MX_IWDG_Init();

//     // test();

//     while (1)
//     {
//         blinkLed();

//         encoder();

//         // // xu ly input
//         // processData();

//         // // dieu khien dong co
//         // motorControl();

//         // // phan hoi (neu can)
//         // respond();

//         // static uint32_t test = 0;
//         // if (HAL_GetTick() - test >= 1000)
//         // {
//         //     test = HAL_GetTick();
//         //     DIR_1_GPIO_Port->ODR ^= DIR_1_Pin;
//         //     DIR_2_GPIO_Port->ODR ^= DIR_2_Pin;
//         //     DIR_3_GPIO_Port->ODR ^= DIR_3_Pin;
//         //     DIR_4_GPIO_Port->ODR ^= DIR_4_Pin;
//         // }

//         // refresh watch dog
//         HAL_IWDG_Refresh(&hiwdg);
//     }
// }

#include "main_cpp.hpp"
#include "config.hpp"
#include "math.h"

void encoder()
{

    // cap nhat moi 1ms
    if (HAL_GetTick() % 1 == 0)
    {
    }
    else
    {
        return;
    }

    // robot.state.isDataNew = true;

    encoderData.front_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim1);
    encoderData.front_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
    encoderData.rear_left.encoderValue = __HAL_TIM_GET_COUNTER(&htim3);
    encoderData.rear_right.encoderValue = __HAL_TIM_GET_COUNTER(&htim4);

    encoderData.front_left.dir = TIM1->CR1 & TIM_CR1_DIR ? 1 : 0;
    encoderData.front_right.dir = TIM2->CR1 & TIM_CR1_DIR ? 1 : 0;
    encoderData.rear_left.dir = TIM3->CR1 & TIM_CR1_DIR ? 1 : 0;
    encoderData.rear_right.dir = TIM4->CR1 & TIM_CR1_DIR ? 1 : 0;
}




// Hàm này sẽ gom toàn bộ quá trình đọc cảm biến và điều khiển PID
// Chạy đúng chu kỳ 10ms
void Robot_Control_Loop()
{
    // 1. ĐỊNH THỜI 10ms
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time < 1)
    {
        return; // Chưa đủ 10ms, thoát hàm
    }
    last_time = HAL_GetTick(); // Cập nhật mốc thời gian

    // 2. ĐỌC ENCODER TRỰC TIẾP TỪ TIMER (Không cần hàm encoder() cũ)
    uint16_t current_FL = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    uint16_t current_FR = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    // doi chieu dong co sau
    uint16_t current_RR = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t current_RL = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    // 3. TÍNH VẬN TỐC (XUNG / 10ms) BẰNG ÉP KIỂU
    static int16_t prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

    // robot.front_left.velocity = (int16_t)(current_FL - prev_FL);
    // robot.front_right.velocity = (int16_t)(current_FR - prev_FR);
    // robot.rear_left.velocity = (int16_t)(current_RL - prev_RL);
    // robot.rear_right.velocity = (int16_t)(current_RR - prev_RR);

    robot.front_left.velocity = (int16_t)((int16_t)prev_FL - (int16_t)current_FL);
    robot.front_right.velocity = (int16_t)((int16_t)prev_FR - (int16_t)current_FR);
    robot.rear_left.velocity = (int16_t)((int16_t)prev_RL - (int16_t)current_RL);
    robot.rear_right.velocity = (int16_t)((int16_t)prev_RR - (int32_t)current_RR);

    prev_FL = current_FL;
    prev_FR = current_FR;
    prev_RL = current_RL;
    prev_RR = current_RR;

    // // 4. CÀI ĐẶT SETPOINT (Nên đưa ra ngoài main_cpp hoặc nhận từ UART, ở đây làm mẫu)
    // // Lưu ý: Dùng hàm của Object, không gọi trực tiếp struct bên trong
    // frontLeftMotor.setSetpoint(30);
    // frontRightMotor.setSetpoint(30);
    // rearLeftMotor.setSetpoint(30);
    // rearRightMotor.setSetpoint(30);

    // 5. TÍNH TOÁN PID VÀ XUẤT PWM CHO TỪNG ĐỘNG CƠ
    // Hàm computeAndControl (bạn đã viết ở class MotorControlPID) sẽ tự động:
    // - Lấy Setpoint trừ đi Velocity thực tế
    // - Tính toán P, I, D
    // - Tự động gọi hàm băm xung và chỉnh chân DIR (control())


    frontLeftMotor.computeAndControl(robot.front_left.velocity);
    frontRightMotor.computeAndControl(robot.front_right.velocity);
    
    rearLeftMotor.computeAndControl(robot.rear_left.velocity);
    rearRightMotor.computeAndControl(robot.rear_right.velocity);

    CalculateOdometry();
}

void consumer()
{
    if (robot.state.isControlNew)
    {
        robot.state.isControlNew = false;

        // ben trai dao chieu
        frontLeftMotor.setSetpoint(robot.front_left.setpoint);

        frontRightMotor.setSetpoint(robot.front_right.setpoint);

        // ben trai
        rearLeftMotor.setSetpoint(robot.rear_left.setpoint);


        rearRightMotor.setSetpoint(robot.rear_right.setpoint);
    }
    else
    {
    }
}

void main_cpp()
{
    // 1. KHỞI ĐỘNG CÁC TIMER ENCODER
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // 3. KHỞI TẠO ĐỐI TƯỢNG ĐỘNG CƠ PID
    // Thay đổi thông số Kp, Ki, Kd thực tế của bạn vào đây (VD: 0.0, 0.0, 0.0)
    frontLeftMotor.initPID(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin, 0.3, 0.003, 0.0, 248, LEFT);
    frontRightMotor.initPID(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin, 0.25, 0.005, 0.0, 248, RIGHT);

    rearRightMotor.initPID(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin, 0.3, 0.003, 0.0, 248, RIGHT);
    rearLeftMotor.initPID(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin, 0.3, 0.003, 0.0, 248, LEFT);

    // 4. KHỞI ĐỘNG CÁC CHỨC NĂNG KHÁC (Watchdog, UART...)
    uartDriver.init(&huart6, bufferUART, sizeof(bufferUART));
    MX_IWDG_Init();

    // VÒNG LẶP CHÍNH
    while (1)
    {
        // Nháy LED báo trạng thái sống
        blinkLed();

        // xu ly input
        processData();

        // tieu thu data tu ROS
        consumer();

        encoder();

        // dieu khien dong co
        // motorControl();

        // CHẠY HỆ THỐNG ĐIỀU KHIỂN CHÍNH (Đã được định thời cứng 10ms bên trong)
        Robot_Control_Loop();

        // phan hoi (neu can)
        // respond();
        configPID();

        // Làm tươi Watchdog
        HAL_IWDG_Refresh(&hiwdg);
    }
}