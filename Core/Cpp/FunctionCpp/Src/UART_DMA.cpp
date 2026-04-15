#include "UART_DMA.hpp"
#include "MotorControl.hpp"

UART_DMA UART_DMA_6;
uint8_t rxBuffer[200];
uint8_t txBuffer[200];

void UART_DMA::init(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t size)
{
    // Lưu tham số vào thuộc tính của đối tượng
    _huart = huart;
    _rxBuffer = rxBuffer;
    _size = size;

    // Nhận DMA và tự động dừng khi Pi ngừng gửi (IDLE)
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rxBuffer, _size);

    // Tắt ngắt Half-Transfer (HT) để đỡ bị chip gọi ngắt lắt nhắt
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}

extern "C"
{
    // Tự động được gọi khi Pi gửi xong 1 chuỗi lệnh
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart->Instance == USART6)
        {
            // 1. Phất cờ báo cho CPU biết có dữ liệu mới
            robot.state.isDataNew = true;

            // 2. Tái kích hoạt DMA để nó đi hứng gói tin tiếp theo của Pi
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    // Tự động được gọi khi STM32 đã gửi xong 1 chuỗi dữ liệu cho Pi
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART6)
        {
            // 3. PHẤT CỜ: Đã truyền xong, rảnh rỗi chờ chu kỳ tiếp theo
            robot.state.isSendDataNew = 1;
        }
    }
}