#include "output.hpp"

void respond()
{
    if (!robot.state.isResponseNew)
        return;
    HAL_UART_Transmit_DMA(&huart6, response.content, sizeof(response.content));
}