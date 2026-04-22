template <typename T>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void tinhVanToc(float deltaT) // Nhận deltaT từ hàm quản lý truyền vào
{
    // CÁC BIẾN NÀY CẦN NHỚ GIÁ TRỊ CŨ -> Bắt buộc dùng static
    static int64_t xungQK_FL = 0; // xung qua khu của bánh trước trái
    static int64_t xungQK_FR = 0; // xung \qua khu của bánh trước phải
    static int64_t xungQK_RL = 0; // xung qua khu của bánh sau trái
    static int64_t xungQK_RR = 0; // xung qua khu của bánh sau phải

    int64_t xungHT_FL = (int64_t)__HAL_TIM_GET_COUNTER(&htim1); // xung hiện tại của bánh trước trái
    int64_t xungHT_FR = (int64_t)__HAL_TIM_GET_COUNTER(&htim2); // xung hiện tại của bánh trước phải
    int64_t xungHT_RL = (int64_t)__HAL_TIM_GET_COUNTER(&htim4); // xung hiện tại của bánh sau trái
    int64_t xungHT_RR = (int64_t)__HAL_TIM_GET_COUNTER(&htim3); // xung hiện tại của bánh sau phải

    int64_t delta_FL = xungHT_FL - xungQK_FL; // chenh lech xung của bánh trước trái
    int64_t delta_FR = xungHT_FR - xungQK_FR; // chenh lech xung của bánh trước phải
    int64_t delta_RL = xungHT_RL - xungQK_RL; // chenh lech xung của bánh sau trái
    int64_t delta_RR = xungHT_RR - xungQK_RR; // chenh lech xung của bánh sau phải

    // Xử lý tràn - tam thời khong dùng đên
    // Banh truoc trai
    // if (delta_FL > 32768)
    //     delta_FL -= 65536;
    // else if (delta_FL < -32768)
    //     delta_FL += 65536;
    // // Banh truoc phai
    // if (delta_FR > 32768)
    //     delta_FR -= 65536;
    // else if (delta_FR < -32768)
    //     delta_FR += 65536;
    // // Banh sau trai
    // if (delta_RL > 32768)
    //     delta_RL -= 65536;
    // else if (delta_RL < -32768)
    //     delta_RL += 65536;
    // // Banh sau phai
    // if (delta_RR > 32768)
    //     delta_RR -= 65536;
    // else if (delta_RR < -32768)
    //     delta_RR += 65536;

    // Tính vận tốc của từng bánh
    robot.motor_front_left.vanToc = (delta_FL * MET1XUNG) / deltaT;
    robot.motor_front_right.vanToc = -(delta_FR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu
    robot.motor_rear_left.vanToc = (delta_RL * MET1XUNG) / deltaT;
    robot.motor_rear_right.vanToc = -(delta_RR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu

    // Tinh vận tốc trung bình của robot dựa trên vận tốc của 4 bánh
    vtTrungBinhTrai = (robot.motor_rear_left.vanToc + robot.motor_front_left.vanToc) / 2.0f;
    vtTrungBinhPhai = (robot.motor_rear_right.vanToc + robot.motor_front_right.vanToc) / 2.0f;
    odom_vx = (vtTrungBinhTrai + vtTrungBinhPhai) / 2.0f;

    // Cập nhật vị trí encoder cho lần sau
    xungQK_FL = xungHT_FL;
    xungQK_FR = xungHT_FR;
    xungQK_RL = xungHT_RL;
    xungQK_RR = xungHT_RR;
}