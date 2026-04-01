// #ifndef UART_DMA_HPP
// #define UART_DMA_HPP

// // tien xu ly
// #include "main.h"
// #include "stdint.h"
// #include "string.h"
// #include "stdio.h"
// #include "stdbool.h"


// // cau hinh modbus rtu
// #define MODBUS_BUF_SIZE 1024
// // toi da
// #define QUANTITY_REGISTER 125

// #define TIMEOUT_MS 500

// #ifdef __cplusplus
// #include "pin_config.hpp"

// class uart_DMA:
// {
// protected:
//     UART_HandleTypeDef *m_huart;
//     // buffer xu ly doi dai toi da 256 byte
//     uint8_t m_rxBuffer[MODBUS_BUF_SIZE];
//     uint8_t m_txBuffer[MODBUS_BUF_SIZE];

//     // trang thai he thong
//     volatile bool m_hasNewData;     // DMA ngat bao co du lieu moi
//     volatile bool m_isTransmitting; // Dang respone

// public:
//     // ham tao
//     uart_DMA();
//     void Init(UART_HandleTypeDef *huart, uint8_t *dmaBuffer, uint16_t dmaBufferSize);
//     // ham goi tu ngat
//     void OnTxComplete();
//     // ngat idle
//     void OnIdle();
// };

// #endif // c++

// // wrapper c
// #ifdef __cplusplus
// extern "C"
// {
// #endif

//     void MB_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *Port, uint16_t OE_Pin);
//     void MB_IdleCallback(void);
//     void MB_TxCpltCallback(void);

// #ifdef __cplusplus
// }
// #endif
// #endif // _MODBUS_RTU_SLAVE_

// /**
//  *
//  * 1. Khởi tạo: MB_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *Port, uint16_t OE_Pin);
//  * - huart - truyền vào con trỏ của UART tương ứng
//  * - Thư viện này viết cho mạch sử dụng IC MAX3485 để giao tiếp UART nên cần có chân DE
//  *
//  * 2. MB_Loop(void);
//  * - Cần có xung xử lý để đếm timeout gửi nếu DMA treo và tính toán phản hồi Master khi yêu cầu, chỉ cần đặt vào while(1)
//  *
//  * 3. Ngắt cờ IDLE của DMA là một ngắt toàn cục và chỉ có một hàm duy nhất (hàm này được tạo sẵn trong file cấu hình ngắt stm32xx_it.c)
//  * - Có thể sử dụng luôn hàm ngắt được tạo sẵn để thuận tiện cho việc cấu hình bằng STM32CubeMX hoạc viết ở dâu đó thì hàm ngắt này
//  *  phải được xóa đi trong file ở trên
//  *
//  */

// //  /**
// //   * @brief This function handles USART1 global interrupt.
// //   */
// // void USART1_IRQHandler(void)
// // {
// //   /* USER CODE BEGIN USART1_IRQn 0 */
// //   // Kiem tra co IDLE (Modbus Frame Received)
// //   if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
// //   {
// //     __HAL_UART_CLEAR_IDLEFLAG(&huart1);
// //     // Goi ham xu ly IDLE (copy data tu DMA)
// //     MB_IdleCallback();
// //   }
// //   /* USER CODE END USART1_IRQn 0 */
// //   HAL_UART_IRQHandler(&huart1);
// //   /* USER CODE BEGIN USART1_IRQn 1 */

// //   /* USER CODE END USART1_IRQn 1 */
// // }


#ifndef UART_DMA_HPP
#define UART_DMA_HPP

#include "main.h"
#include "pin_config.hpp"

#ifdef __cplusplus
class UartDma {
private:
    UART_HandleTypeDef* _huart;
    uint8_t* _rxBuf;
    uint16_t  _size;
    volatile uint16_t _rxLen;

public:
    UartDma() : _huart(nullptr), _rxBuf(nullptr), _size(0), _rxLen(0) {}

    void init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size);
    void onIdle();
    void onTxCplt();
    
    uint16_t getRxLength() { return _rxLen; }
    void clearFlag() { _rxLen = 0; }
};
#endif

// --- C WRAPPERS (Để gọi từ stm32f1xx_it.c) ---
#ifdef __cplusplus
extern "C" {
#endif

    void UART_DMA_Init(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t size);
    void UART_DMA_IdleHandler(UART_HandleTypeDef* huart);
    void UART_DMA_TxCpltHandler(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif