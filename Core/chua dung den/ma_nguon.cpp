// 1 Chuong trình chính
void main_cpp()
{
    // Khoi tao cam bien la ban
    compass.init(SCL_GPIO_Port, SCL_Pin, SDA_GPIO_Port, SDA_Pin);
    compass.setSmoothing(5, true); // Smoothing 5 bước
    HAL_Delay(100);                // Đợi cảm biến ổn định
    // Đọc nháp 10 lần để lấp đầy mảng lọc nhiễu (Vứt bỏ kết quả)
    for (int i = 0; i < 10; i++)
    {
        compass.read();
        HAL_Delay(10); // Đợi 10ms giữa mỗi lần đọc cho đúng nhịp
    }
    theta_goc = compass.getAzimuth(); // Lấy góc gốc khi khởi tạo để tính toán sau này

    // Khoi tao cac dong co
    MotorCtr_FL.init(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin);
    MotorCtr_FR.init(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin);
    MotorCtr_RL.init(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin);
    MotorCtr_RR.init(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin);

    // Khoi tao timer doc encoder cho cac dong co
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Khoi tao timer cho cam bien sieu am
    HAL_TIM_Base_Start(&htim9);

    // Khoi tao UART DMA gui du lieu cho pi
    UART_DMA_6.init(&huart6, rxBuffer, sizeof(rxBuffer));

    uint32_t tgDoHSCu = 0;           // thoi gian do cam bien sieu am cu
    uint32_t tgDieuKhienMotorCu = 0; // thoi gian dieu khien dong co

    while (1)
    {
        // Nháy LED báo trạng thái sống
        nhayLed();

        // xu ly input
        nhanDuLieuPi();
        // Kiem tra cam bien sieu am moi 50ms
        kiemTraHS();

        // Dieu khien dong co moi 10ms
        dieuKhienMotor();

        // Tính toán odometry
        tinhOdom();

        // Gui du lieu odometry len pi
        guiDuLieuPi();
    }
}

// 2. Hàm nhận dữ liệu từ Pi qua UART DMA
void nhanDuLieuPi()
{
    if (HAL_GetTick() - tgNhanDuLieuPiCu > 1000) // Dung robot neu pi khong gui du lieu trong 1 giay
    {
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    }

    // Nếu chưa có cờ thì thoát (CPU đi làm việc khác)
    if (!robot.state.isDataNew)
        return;

    // Hạ cờ
    robot.state.isDataNew = false;

    float vx = 0.0f;
    float w = 0.0f;

    // Bóc tách nhanh gọn lẹ bằng sscanf
    if (sscanf((char *)rxBuffer, "%f,%f", &vx, &w) == 2)
    {
        tgNhanDuLieuPiCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu

        // Tính toán CCR có dấu
        float ccr_L = vx - (w * KHOANGCACH2BANH / 2.0f);
        float ccr_R = vx + (w * KHOANGCACH2BANH / 2.0f);

        // GÁN THẲNG GIÁ TRỊ CÓ DẤU
        robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = ccr_L;
        robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = ccr_R;
    }
}

// 3. Kiểm tra cảm biến siêu âm
void kiemTraHS()
{

    if (HAL_GetTick() - tgDoHSCu >= 50)
    {
        tgDoHSCu = HAL_GetTick();
        isBlocked = false;          // reset trạng thái có vật cản trước khi kiểm tra lại
        for (int i = 0; i < 4; i++) // kiem tra 4 cam bien sieu am xem co vat can hay khong
        {
            if (ArrayHS[i].khoangCach > 2.0f && ArrayHS[i].khoangCach < 100.0f)
            {
                isBlocked = true; // Có vật cản
                break;            // Ngay lap tuc thoat vong for
            }
        }
        for (int i = 0; i < 4; i++) // xoa du lieu truoc khi do cam bien sieu am lan sau
        {
            ArrayHS[i].khoangCach = 0.0f;
        }
        kichHoatTrig(); // kich hoat chan trig de do cam bien sieu am
    }
}

// 4. Hàm điều khiển tốc độ và hướng động cơ
void controlOnDinh()
{
    // Bên trái trước
    robot.motor_front_left.ccrHT += ALPHA * (robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT);
    if (fabsf(robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT) < delta_ccr)
    {
        robot.motor_front_left.ccrHT = robot.motor_front_left.ccrTL;
    }
    // Bên trái sau
    robot.motor_rear_left.ccrHT += ALPHA * (robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT);
    if (fabsf(robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT) < delta_ccr)
    {
        robot.motor_rear_left.ccrHT = robot.motor_rear_left.ccrTL;
    }

    // Bên phải trước
    robot.motor_front_right.ccrHT += ALPHA * (robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT);
    if (fabsf(robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT) < delta_ccr)
    {
        robot.motor_front_right.ccrHT = robot.motor_front_right.ccrTL;
    }
    // Bên phải sau
    robot.motor_rear_right.ccrHT += ALPHA * (robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT);
    if (fabsf(robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT) < delta_ccr)
    {
        robot.motor_rear_right.ccrHT = robot.motor_rear_right.ccrTL;
    }

    // CẬP NHẬT BIẾN DIR VÀO STRUCT (Dựa trên dấu của ccrHT hiện tại)
    // Ben trai
    robot.motor_front_left.dir = (robot.motor_front_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    robot.motor_rear_left.dir = (robot.motor_rear_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    // Ben phai
    robot.motor_front_right.dir = (robot.motor_front_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);
    robot.motor_rear_right.dir = (robot.motor_rear_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Backward) : static_cast<uint8_t>(MotorDir::Forward);

    // ĐIỀU KHIỂN MOTOR THẬT (Lấy giá trị từ struct ra)
    MotorCtr_FL.control((uint16_t)fabsf(robot.motor_front_left.ccrHT), static_cast<MotorDir>(robot.motor_front_left.dir));
    MotorCtr_FR.control((uint16_t)fabsf(robot.motor_front_right.ccrHT), static_cast<MotorDir>(robot.motor_front_right.dir));
    MotorCtr_RL.control((uint16_t)fabsf(robot.motor_rear_left.ccrHT), static_cast<MotorDir>(robot.motor_rear_left.dir));
    MotorCtr_RR.control((uint16_t)fabsf(robot.motor_rear_right.ccrHT), static_cast<MotorDir>(robot.motor_rear_right.dir));
}

// 5. Các hàm tính toán odometry
void tinhVanToc(float deltaT) // Nhận deltaT từ hàm quản lý truyền vào
{
    // CÁC BIẾN NÀY CẦN NHỚ GIÁ TRỊ CŨ -> Bắt buộc dùng static
    static int32_t xungQK_FL = 0; // xung qua khu của bánh trước trái
    static int32_t xungQK_FR = 0; // xung qua khu của bánh trước phải
    static int32_t xungQK_RL = 0; // xung qua khu của bánh sau trái
    static int32_t xungQK_RR = 0; // xung qua khu của bánh sau phải

    int32_t xungHT_FL = (int32_t)__HAL_TIM_GET_COUNTER(&htim1); // xung hiện tại của bánh trước trái
    int32_t xungHT_FR = (int32_t)__HAL_TIM_GET_COUNTER(&htim2); // xung hiện tại của bánh trước phải
    int32_t xungHT_RL = (int32_t)__HAL_TIM_GET_COUNTER(&htim4); // xung hiện tại của bánh sau trái
    int32_t xungHT_RR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3); // xung hiện tại của bánh sau phải

    int32_t delta_FL = xungHT_FL - xungQK_FL; // chenh lech xung của bánh trước trái
    int32_t delta_FR = xungHT_FR - xungQK_FR; // chenh lech xung của bánh trước phải
    int32_t delta_RL = xungHT_RL - xungQK_RL; // chenh lech xung của bánh sau trái
    int32_t delta_RR = xungHT_RR - xungQK_RR; // chenh lech xung của bánh sau phải

    // Xử lý tràn
    // Banh truoc trai
    if (delta_FL > 32768)
        delta_FL -= 65536;
    else if (delta_FL < -32768)
        delta_FL += 65536;
    // Banh truoc phai
    if (delta_FR > 32768)
        delta_FR -= 65536;
    else if (delta_FR < -32768)
        delta_FR += 65536;
    // Banh sau trai
    if (delta_RL > 32768)
        delta_RL -= 65536;
    else if (delta_RL < -32768)
        delta_RL += 65536;
    // Banh sau phai
    if (delta_RR > 32768)
        delta_RR -= 65536;
    else if (delta_RR < -32768)
        delta_RR += 65536;

    // Tính vận tốc của từng bánh
    robot.motor_front_left.vanToc = (delta_FL * MET1XUNG) / deltaT;
    robot.motor_front_right.vanToc = -(delta_FR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu
    robot.motor_rear_left.vanToc = (delta_RL * MET1XUNG) / deltaT;
    robot.motor_rear_right.vanToc = -(delta_RR * MET1XUNG) / deltaT; // bánh phía phải ngược chiều nên đổi dấu

    // Tinh vận tốc trung bình của robot dựa trên vận tốc của 4 bánh
    float vtTrungBinhTrai = (robot.motor_rear_left.vanToc + robot.motor_front_left.vanToc) / 2.0f;
    float vtTrungBinhPhai = (robot.motor_rear_right.vanToc + robot.motor_front_right.vanToc) / 2.0f;
    odom_vx = (vtTrungBinhTrai + vtTrungBinhPhai) / 2.0f;

    // Cập nhật vị trí encoder cho lần sau
    xungQK_FL = xungHT_FL;
    xungQK_FR = xungHT_FR;
    xungQK_RL = xungHT_RL;
    xungQK_RR = xungHT_RR;
}

void tinhGoc()
{
    compass.read();
    odom_theta_deg = compass.getAzimuth() - theta_goc; // Lấy góc hiện tại trừ đi góc gốc

    if (odom_theta_deg > 180.0f)
        odom_theta_deg -= 360.0f;
    else if (odom_theta_deg < -180.0f)
        odom_theta_deg += 360.0f;

    odom_theta_rad = odom_theta_deg * PI / 180.0f; // Chuyển góc sang radian để tính toán
}

void tinhVanTocGoc(float deltaT)
{
    // Dùng static để giữ giá trị giữa các lần chạy hàm (Thay thế cho biến toàn cục)
    static float odom_theta_deg_cu = 0.0f;
    static float odom_w_rad_loc = 0.0f; // Vận tốc góc đã lọc

    float delta_theta_deg = odom_theta_deg - odom_theta_deg_cu; // Chenh lech do giua 2 lan do

    // xu li tran goc
    if (delta_theta_deg > 180.0f)
        delta_theta_deg -= 360.0f;
    else if (delta_theta_deg < -180.0f)
        delta_theta_deg += 360.0f;

    float odom_w_rad_tho = (delta_theta_deg * PI / 180.0f) / deltaT; // Van toc goc thô
    odom_w_rad_loc = odom_w_rad_loc * 0.8f + odom_w_rad_tho * 0.2f;  // Tinh van toc goc loc
    odom_w_rad = odom_w_rad_loc;

    odom_theta_deg_cu = odom_theta_deg;
}

void tinhToaDo(float deltaT)
{
    odom_x += odom_vx * cos(odom_theta_rad) * deltaT;
    odom_y += odom_vx * sin(odom_theta_rad) * deltaT;
}

// =======================================================
// 3. HÀM QUẢN LÝ CHÍNH (GỌI TRONG MAIN)
// =======================================================
void tinhOdom()
{
    uint32_t tgTinhOdomMoi = HAL_GetTick();
    if (tgTinhOdomMoi - tgTinhOdomCu < tgTinhOdom)
    {
        return; // Chưa đủ 100ms thì nghỉ
    }

    // Tính deltaT thực tế phòng trường hợp CPU bị trễ (VD: 101ms, 105ms)
    float deltaT = (tgTinhOdomMoi - tgTinhOdomCu) / 1000.0f;
    tgTinhOdomCu = tgTinhOdomMoi;

    // Chạy các hàm con theo đúng quy trình
    tinhVanToc(deltaT);
    tinhGoc();
    tinhVanTocGoc(deltaT);
    tinhToaDo(deltaT);
}

// 6. Hàm gửi dữ liệu odometry ra UART cho pi
void guiDuLieuPi()
{
    // 1. Kiểm tra chu kỳ gửi (Ví dụ 50ms = 20Hz)
    if (HAL_GetTick() - tgGuiDuLieuPiCu < 50)
    {
        return;
    }

    // 2. Kiểm tra xem bộ DMA đã rảnh chưa (1 = Rảnh, 0 = Đang bận gửi gói cũ)
    if (!robot.state.isSendDataNew)
    {
        return;
    }

    tgGuiDuLieuPiCu = HAL_GetTick();

    // 3. Đóng gói dữ liệu
    static int doDaiGoiTin;

    doDaiGoiTin = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f,%.3f,%.3f,%.3f,0.000,%.3f\n",
                           odom_x, odom_y, odom_theta_rad, odom_vx, odom_w_rad);

    // 4. Kích hoạt DMA gửi đi
    if (doDaiGoiTin > 0)
    {
        // Hạ cờ BẬN ngay lập tức để khóa hàm lại cho chu kỳ sau
        robot.state.isSendDataNew = 0;

        // Đẩy dữ liệu đi (DMA sẽ tự động phất cờ rảnh ở ngắt TxCpltCallback)
        HAL_UART_Transmit_DMA(&huart6, txBuffer, doDaiGoiTin);
    }
}