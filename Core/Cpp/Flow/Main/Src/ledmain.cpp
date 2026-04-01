#include "ledmain.hpp"

void blinkLed()
{
  static uint32_t timeCurent = 0;

  if (HAL_GetTick() - timeCurent >= TIMER_100MS)
  {
    timeCurent = HAL_GetTick();
    // Đảo bit dùng XOR (^=)
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin;
  }
}
