// // #include "input.hpp"

// // // void processData()
// // // {
// // //     // Xu ly du lieu nhan duoc tu UART
// // //     // Chuyen du lieu tu bufferUART sang cau truc robot
// // //     // Sau do dieu khien dong co theo cau truc robot
// // //     if (!robot.state.isDataNew)
// // //         return;

// // //     robot.state.isDataNew = false; // Reset flag sau khi đã xử lý dữ liệu mới
// // //     static uint8_t cache[8];

// // //     // lay 8 byte dau tien
// // //     for (uint8_t i = 0; i < 8; i++)
// // //     {
// // //         cache[i] = bufferUART[i];
// // //     }

// // //     robot.front_left.dir = cache[0];
// // //     robot.front_left.pwm = cache[1];
// // //     robot.front_right.dir = cache[2];
// // //     robot.front_right.pwm = cache[3];
// // //     robot.rear_left.dir = cache[4];
// // //     robot.rear_left.pwm = cache[5];
// // //     robot.rear_right.dir = cache[6];
// // //     robot.rear_right.pwm = cache[7];

// // //     robot.state.isControlNew = true; // Đánh dấu có lệnh điều khiển mới để main loop xử lý
// // //    // robot.state.isResponseNew = true; // Đánh dấu có phản hồi mới để main loop xử lý (nếu cần)
// // // }

// // // khung du lieu vx,vy,w\n
// // // void processData()
// // // {
// // //     // Xu ly du lieu nhan duoc tu UART
// // //     // Chuyen du lieu tu bufferUART sang cau truc robot
// // //     // Sau do dieu khien dong co theo cau truc robot
// // //     if (!robot.state.isDataNew)
// // //         return;

// // //     uint32_t len = uartDriver.getRxLength();
// // //     robot.state.isDataNew = false; // Reset flag sau khi đã xử lý dữ liệu mới
// // //     uint8_t cache[len];

// // //     for (uint32_t i = 0; i < len && i >= 6; i++)
// // //     {
// // //         cache[i] = bufferUART[i];
// // //     }

// // //     // boc tach tin hieu

// // //     robot.front_left.dir = cache[0];
// // //     robot.front_left.pwm = cache[1];
// // //     robot.front_right.dir = cache[2];
// // //     robot.front_right.pwm = cache[3];
// // //     robot.rear_left.dir = cache[4];
// // //     robot.rear_left.pwm = cache[5];
// // //     robot.rear_right.dir = cache[6];
// // //     robot.rear_right.pwm = cache[7];

// // //     robot.state.isControlNew = true; // Đánh dấu có lệnh điều khiển mới để main loop xử lý
// // //                                      // robot.state.isResponseNew = true; // Đánh dấu có phản hồi mới để main loop xử lý (nếu cần)
// // // }

// // #include <stdlib.h> // Để dùng hàm atof()
// // #include <string.h> // Để dùng strchr()

// // // Giả sử L là nửa khoảng cách 2 bánh (mét) theo code Arduino cũ của bạn
// // #define KINEMATIC_L 0.15f 

// // // // Hàm quy đổi từ m/s sang số Xung/10ms để nhét vào PID
// // // // Bạn cần thay đổi hệ số này theo R (bán kính bánh) và PPR (xung/vòng) thực tế
// // // float convertSpeedToPulses(float v_meters_per_sec) {
// // //     // Công thức: (v * dt * PPR) / (2 * PI * R)
// // //     // Ví dụ giả định: nhân với hệ số K nào đó. Bạn tự thay công thức của bạn vào đây.
// // //     float K_quy_doi = 10.0f; 
// // //     return v_meters_per_sec * K_quy_doi; 
// // // }


// // float convertSpeedToPulses(float v_meters_per_sec) {
// //     float K_quy_doi = 1.0f; // Sửa thành 1.0f để giữ nguyên giá trị
// //     return v_meters_per_sec * K_quy_doi; 
// // }

// // void processData()
// // {
// //     if (!robot.state.isDataNew)
// //     {
// //        return;
// //     }
// //     else{
// //     }
// //      uint32_t len = uartDriver.getRxLength();
// //     robot.state.isDataNew = 0; // Reset cờ

// //     // BƯỚC 1: ĐẢM BẢO CHUỖI CÓ KÝ TỰ KẾT THÚC (NULL TERMINATOR)
// //     // Rất quan trọng để các hàm xử lý chuỗi của C không bị tràn RAM đếm rác
// //     if (len >= sizeof(bufferUART)) {
// //         bufferUART[sizeof(bufferUART) - 1] = '\0'; 
// //     } else {
// //         bufferUART[len] = '\0';
// //     }

// //     // BƯỚC 2: BÓC TÁCH CHUỖI "vx,vy,w\n"
// //     // Dùng strchr để tìm vị trí của các dấu phẩy ','
// //     char *ptr = (char*)bufferUART;
    
// //     char *comma1 = strchr(ptr, ',');
// //     if (comma1 == NULL) return; // Lỗi: Không thấy dấu phẩy thứ nhất
    
// //     char *comma2 = strchr(comma1 + 1, ',');
// //     if (comma2 == NULL) return; // Lỗi: Không thấy dấu phẩy thứ hai

// //     // Thay dấu phẩy bằng ký tự kết thúc chuỗi '\0' để ngắt thành 3 chuỗi nhỏ
// //     *comma1 = '\0';
// //     *comma2 = '\0';

// //     // Ép kiểu (ASCII to Float)
// //     float vx = atof(ptr);               // Lấy phần từ đầu đến dấu phẩy 1
// //     float vy = atof(comma1 + 1);        // Lấy phần giữa 2 dấu phẩy
// //     float w  = atof(comma2 + 1);        // Lấy phần từ dấu phẩy 2 đến hết (kệ dấu \n)

// //     // BƯỚC 3: TÍNH TOÁN ĐỘNG HỌC (KINEMATICS)
// //     // Giả sử xe của bạn là dạng Differential Drive (chạy giống xe tăng), bỏ qua vy
// //     float v_left_mps  = vx - (w * KINEMATIC_L);
// //     float v_right_mps = vx + (w * KINEMATIC_L);

// //     // BƯỚC 4: QUY ĐỔI RA SETPOINT CHO PID VÀ CẬP NHẬT VÀO STRUCT
// //     // Từ Vận tốc thực (m/s) -> Xung/10ms
// //     robot.front_left.setpoint  = convertSpeedToPulses(v_left_mps);
// //     robot.rear_left.setpoint   = convertSpeedToPulses(v_left_mps);
    
// //     robot.front_right.setpoint = convertSpeedToPulses(v_right_mps);
// //     robot.rear_right.setpoint  = convertSpeedToPulses(v_right_mps);

// //     // Báo hiệu cho vòng lặp chính (PID) biết đã có Setpoint mới
// //     robot.state.isControlNew = true; 
    
   
// // }

#include "input.hpp"
#include <stdlib.h> 
#include <string.h> 

#define KINEMATIC_L 0.15f
// #define KINEMATIC_L 0.5f 

// Đã sửa hệ số quy đổi thành 1 để không bị nhân 10
float convertSpeedToPulses(float v_meters_per_sec) {
    float K_quy_doi = 1.0f; 
    return v_meters_per_sec * K_quy_doi; 
}

void processData()
{
    // Nếu không có dữ liệu mới thì thoát
    if (!robot.state.isDataNew) return;

    uint32_t len = uartDriver.getRxLength();
    robot.state.isDataNew = false; // Reset cờ

    if (len == 0) return;

    // BƯỚC 1: CHỐT CHUỖI AN TOÀN
    if (len >= sizeof(bufferUART)) {
        bufferUART[sizeof(bufferUART) - 1] = '\0'; 
    } else {
        bufferUART[len] = '\0';
    }

    // Khởi tạo mặc định bằng 0. 
    // Nếu ROS gửi lỗi (thiếu số), xe sẽ ưu tiên DỪNG LẠI cho an toàn.
    float vx = 0.0f;
    float vy = 0.0f;
    float w  = 0.0f;

    // BƯỚC 2: BÓC TÁCH CHUỖI LINH HOẠT
    char *ptr = (char*)bufferUART;
    char *comma1 = strchr(ptr, ',');
    
    if (comma1 != NULL) {
        *comma1 = '\0'; // Chặt chuỗi tại phẩy 1
        vx = atof(ptr); 
        
        char *comma2 = strchr(comma1 + 1, ',');
        if (comma2 != NULL) {
            *comma2 = '\0'; // Chặt chuỗi tại phẩy 2
            vy = atof(comma1 + 1);
            w  = atof(comma2 + 1);
        } else {
            // Trường hợp ROS gửi "vx,vy" (Có 1 dấu phẩy)
            vy = atof(comma1 + 1);
        }
    } else {
        // Trường hợp ROS gửi mỗi "vx" (Không có dấu phẩy, ví dụ: "0\n")
        vx = atof(ptr); 
    }

    // BƯỚC 3: TÍNH TOÁN ĐỘNG HỌC (KINEMATICS)
    float v_left_mps  = vx - (w * KINEMATIC_L);
    float v_right_mps = vx + (w * KINEMATIC_L);

    // BƯỚC 4: QUY ĐỔI RA SETPOINT VÀ CẬP NHẬT 4 BÁNH
    robot.front_left.setpoint  = convertSpeedToPulses(v_left_mps);
    robot.rear_left.setpoint   = convertSpeedToPulses(v_left_mps);
    
    robot.front_right.setpoint = convertSpeedToPulses(v_right_mps);
    robot.rear_right.setpoint  = convertSpeedToPulses(v_right_mps);

    // Báo hiệu đã có Setpoint mới
    robot.state.isControlNew = true; 
}


// #include "input.hpp"
// #include <stdlib.h> 
// #include <string.h> 

// #define KINEMATIC_L 0.15f

// // Hàm quy đổi ra xung (giữ nguyên tỷ lệ 1.0f như bạn đã setup)
// float convertSpeedToPulses(float v_meters_per_sec) {
//     float K_quy_doi = 1.0f; 
//     return v_meters_per_sec * K_quy_doi; 
// }

// void processData()
// {
//     // Nếu không có dữ liệu mới thì thoát
//     if (!robot.state.isDataNew) return;

//     uint32_t len = uartDriver.getRxLength();
//     robot.state.isDataNew = false; // Reset cờ

//     // Cần ít nhất 4 ký tự (ví dụ tối thiểu: F0R0)
//     if (len < 4) return; 

//     // BƯỚC 1: CHỐT CHUỖI AN TOÀN
//     if (len >= sizeof(bufferUART)) {
//         bufferUART[sizeof(bufferUART) - 1] = '\0'; 
//     } else {
//         bufferUART[len] = '\0';
//     }

//     float vx = 0.0f;
//     float w  = 0.0f;

//     // BƯỚC 2: BÓC TÁCH CHUỖI "AxxCyy" (VD: F99R00)
//     char dir_vx = bufferUART[0]; // Ký tự điều khiển tiến/lùi (F hoặc B)
//     char *ptr_w = NULL;

//     // Duyệt qua chuỗi để tìm vị trí của ký tự điều khiển trái/phải (L hoặc R)
//     for (uint32_t i = 1; i < len; i++) {
//         if (bufferUART[i] == 'L' || bufferUART[i] == 'R' || 
//             bufferUART[i] == 'l' || bufferUART[i] == 'r') {
//             ptr_w = (char*)&bufferUART[i];
//             break;
//         }
//     }

//     if (ptr_w != NULL) {
//         char dir_w = ptr_w[0]; // Lưu lại ký tự hướng xoay (L hoặc R)
//         *ptr_w = '\0';         // Ngắt chuỗi thành 2 nửa tại vị trí này

//         // 2.1 - Lấy giá trị tốc độ thẳng (xx) - Nằm sau ký tự đầu (index 1)
//         vx = atof((char*)&bufferUART[1]);
        
//         // Nếu là đi lùi (B), đảo dấu thành âm
//         if (dir_vx == 'B' || dir_vx == 'b') {
//             vx = -vx;
//         }

//         // 2.2 - Lấy giá trị tốc độ góc (yy) - Nằm ngay sau ký tự L/R
//         w = atof(ptr_w + 1);
        
//         // Nếu là rẽ trái (L), đảo dấu thành âm
//         if (dir_w == 'L' || dir_w == 'l') {
//             w = -w;
//         }
//     } else {
//         // Sai định dạng khung truyền (không tìm thấy L hoặc R) -> Bỏ qua an toàn
//         return; 
//     }

//     // BƯỚC 3: TÍNH TOÁN ĐỘNG HỌC (KINEMATICS)
//     float v_left_mps  = vx - (w * KINEMATIC_L);
//     float v_right_mps = vx + (w * KINEMATIC_L);

//     // BƯỚC 4: QUY ĐỔI RA SETPOINT VÀ CẬP NHẬT 4 BÁNH
//     robot.front_left.setpoint  = 100 * convertSpeedToPulses(v_left_mps);
//     robot.rear_left.setpoint   = 100 * convertSpeedToPulses(v_left_mps);
    
//     robot.front_right.setpoint = 100 * convertSpeedToPulses(v_right_mps);
//     robot.rear_right.setpoint  = 100 * convertSpeedToPulses(v_right_mps);

//     // Báo hiệu đã có Setpoint điều khiển mới
//     robot.state.isControlNew = true; 
// }