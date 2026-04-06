#include "uartDMA.hpp"
#include "config.hpp"

void UartDma::init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size) {
    _huart = huart;
    _rxBuf = rxBuffer;
    _size = size;

    __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(_huart, _rxBuf, _size);
}

void UartDma::onIdle() {
    if (_huart == nullptr || _huart->hdmarx == nullptr) {
        return; 
    }
    
    // Đánh dấu có dữ liệu mới
    robot.state.isDataNew = true; 

    uint16_t remaining = __HAL_DMA_GET_COUNTER(_huart->hdmarx);
    _rxLen = _size - remaining;

    HAL_UART_AbortReceive(_huart); 
    HAL_UART_Receive_DMA(_huart, _rxBuf, _size);
}

void UartDma::onTxCplt(){}

// --- TRIỂN KHAI C-WRAPPERS ---
extern "C" {
    void UART_DMA_Init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size) {
        // Sử dụng đối tượng uartDriver toàn cục thay vì g_uartDriver
        uartDriver.init(huart, rxBuffer, size);
    }

    void UART_DMA_IdleHandler(UART_HandleTypeDef* huart) {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            
            if (huart != NULL && huart->hdmarx != NULL) {
                // Gọi onIdle() của đối tượng uartDriver toàn cục
                uartDriver.onIdle();
            }
        }
    }

    void UART_DMA_TxCpltHandler(UART_HandleTypeDef* huart) {
        uartDriver.onTxCplt();
    }
}