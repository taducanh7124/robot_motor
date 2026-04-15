#include "led_main.hpp"
#include <stdint.h>
#include "main.h"

void nhayLed()
{
  static uint32_t thoiGianHT = 0;

  if (HAL_GetTick() - thoiGianHT >= 100)
  {
    thoiGianHT = HAL_GetTick();
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin; // Đảo bit dùng XOR (^=)
  }
}