// #include "uartDMA.hpp"
// #include "data.hpp"

// // Tạo một instance nội bộ (static) để wrapper sử dụng
// static UartDma g_uartDriver;

// void UartDma::init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size) {
//     _huart = huart;
//     _rxBuf = rxBuffer;
//     _size = size;

//     __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
//     HAL_UART_Receive_DMA(_huart, _rxBuf, _size);
// }

// // void UartDma::onIdle() {
// //     // 1. Tính toán số byte đã nhận (Tổng size - Số byte còn lại trong DMA)
// //     uint16_t remaining = __HAL_DMA_GET_COUNTER(_huart->hdmarx);
// //     _rxLen = _size - remaining;

// //     // 2. Restart DMA để sẵn sàng cho Frame tiếp theo
// //     HAL_UART_DMAStop(_huart);
// //     HAL_UART_Receive_DMA(_huart, _rxBuf, _size);
// // }

// void UartDma::onIdle() {
//     // 1. KIỂM TRA AN TOÀN: Tránh HardFault do con trỏ NULL
//     if (_huart == nullptr || _huart->hdmarx == nullptr) {
//         return; 
//     }
//     robot.state.isDataNew = true; // Đánh dấu có dữ liệu mới để main loop xử lý
//     // 2. Tính toán số byte nhận được
//     // CNDTR là thanh ghi đếm ngược của DMA
//     uint16_t remaining = __HAL_DMA_GET_COUNTER(_huart->hdmarx);
//     _rxLen = _size - remaining;

//     // 3. Restart DMA một cách an toàn
//     // Thay vì DMAStop, ta dùng Abort để HAL dọn dẹp các cờ trạng thái bên trong
//     HAL_UART_AbortReceive(_huart); 
//     HAL_UART_Receive_DMA(_huart, _rxBuf, _size);
// }

// void UartDma::onTxCplt() {
//     // Xử lý sau khi gửi xong (nếu cần)
// }

// // --- TRIỂN KHAI C-WRAPPERS ---
// extern "C" {
//     void UART_DMA_Init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size) {
//         g_uartDriver.init(huart, rxBuffer, size);
//     }

//     // void UART_DMA_IdleHandler(UART_HandleTypeDef* huart) {
//     //     // Kiểm tra đúng bộ UART phát sinh ngắt IDLE
//     //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
//     //         __HAL_UART_CLEAR_IDLEFLAG(huart);
//     //         g_uartDriver.onIdle();
//     //     }
//     // }

//     void UART_DMA_IdleHandler(UART_HandleTypeDef* huart) {
//     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
//         __HAL_UART_CLEAR_IDLEFLAG(huart);
        
//         // --- THÊM DÒNG NÀY ĐỂ KIỂM TRA ---
//         if (huart == NULL) {
//             // Lỗi: Biến huart truyền từ file it.c sang bị rỗng (vô lý)
//             return;
//         }
//         if (huart->hdmarx == NULL) {
//             // LỖI CHẮC CHẮN Ở CUBEMX: Bạn chưa bật DMA cho RX của UART này
//             // Hãy quay lại CubeMX và Add DMA RX.
//             return;
//         }

//         g_uartDriver.onIdle();
//     }
// }

//     void UART_DMA_TxCpltHandler(UART_HandleTypeDef* huart) {
//         g_uartDriver.onTxCplt();
//     }
// }


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