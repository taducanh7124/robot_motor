#include "UART_DMA.hpp"
#include "MotorControl.hpp"

UART_DMA UART_DMA_6;
uint8_t rxBuffer[200];
uint8_t txBuffer[200];

void UART_DMA::init(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t size)
{
    _huart = huart;
    _rxBuffer = rxBuffer;
    _size = size;

    // Kích hoạt DMA lần đầu tiên
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rxBuffer, _size);
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}

extern "C"
{
    // Hàm này tự động gọi khi nhận xong 1 gói tin hoặc đường truyền IDLE
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart->Instance == USART6)
        {
            // Bảo vệ tràn mảng: Chốt ký tự \0 an toàn
            if (size < sizeof(rxBuffer))
            {
                rxBuffer[size] = '\0';
            }
            else
            {
                rxBuffer[sizeof(rxBuffer) - 1] = '\0';
            }

            // Phất cờ báo có dữ liệu
            robot.state.isDataNew = true;

            // THỦ THUẬT: Ép dừng trạng thái Rx cũ trước khi gọi lại để tránh kẹt HAL_BUSY
            HAL_UART_AbortReceive(huart);

            // Tái kích hoạt DMA cho lần nhận tiếp theo
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    // ==========================================================
    // BẮT BUỘC PHẢI CÓ: HÀM PHAO CỨU SINH KHI UART BỊ LỖI
    // ==========================================================
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART6)
        {
            // Nếu có lỗi (Nhiễu, Overrun, Lệch Baudrate...), kéo chip ra khỏi lỗi
            // và khởi động lại DMA ngay lập tức!
            HAL_UART_AbortReceive(huart);
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    // Hàm này tự động gọi khi gửi xong (TX)
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART6)
        {
            robot.state.isSendDataNew = 1;
        }
    }
}

// #include "UART_DMA.hpp"
// #include "MotorControl.hpp"

// UART_DMA UART_DMA_6;
// uint8_t rxBuffer[200];
// uint8_t txBuffer[200];

// void UART_DMA::init(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t size)
// {
//     // Lưu tham số vào thuộc tính của đối tượng
//     _huart = huart;
//     _rxBuffer = rxBuffer;
//     _size = size;

//     // Nhận DMA và tự động dừng khi Pi ngừng gửi (IDLE)
//     HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rxBuffer, _size);

//     // Tắt ngắt Half-Transfer (HT) để đỡ bị chip gọi ngắt lắt nhắt
//     __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
// }

// extern "C"
// {
//     // Tự động được gọi khi Pi (hoặc ESP) gửi xong 1 chuỗi lệnh
//     void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
//     {
//         if (huart->Instance == USART6)
//         {
//             // Ép ký tự kết thúc chuỗi vào ngay sau byte cuối cùng nhận được
//             rxBuffer[size] = '\0';

//             // 1. Phất cờ báo cho CPU biết có dữ liệu mới
//             robot.state.isDataNew = true;

//             // 2. Tái kích hoạt DMA
//             HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
//             __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
//         }
//     }

//     // Tự động được gọi khi STM32 đã gửi xong 1 chuỗi dữ liệu cho Pi
//     void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//     {
//         if (huart->Instance == USART6)
//         {
//             // 3. PHẤT CỜ: Đã truyền xong, rảnh rỗi chờ chu kỳ tiếp theo
//             robot.state.isSendDataNew = 1;
//         }
//     }
// }