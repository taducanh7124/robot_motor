#include "led_main.hpp"
#include <stdint.h>
#include "main.h"
#include "MotorControl.hpp"

void nhayLed()
{
  static uint32_t thoiGianHT = 0;

  if (HAL_GetTick() - thoiGianHT >= 100)
  {
    thoiGianHT = HAL_GetTick();
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin; // Đảo bit dùng XOR (^=)
  }
}

void nhayLedMode()
{
  static uint32_t tgNhayLedCu = 0;

  // Nếu DebugMode = 1 thì khoảng thời gian là 100ms, nếu = 0 thì là 1000ms
  uint32_t khoangThoiGian = (robot.state.isDebugMode == 1) ? 100 : 1000;

  if (HAL_GetTick() - tgNhayLedCu >= khoangThoiGian)
  {
    tgNhayLedCu = HAL_GetTick();
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin; // Đảo trạng thái LED
  }
}