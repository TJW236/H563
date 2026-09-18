#include "drv8320s.h"
#include "spi.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>

/* DRV8320S SPI（drv8323r.pdf 8.5.1）：16bit 帧 = [R/W(1)][addr(4)][data(11)]，MSB 先。
 * 读命令 bit15=1；写命令的响应 SDO 返回被写寄存器的旧值。
 * 时序约束：CSN 翻转时 SCLK 须为低（Mode1 空闲低天然满足）；
 * CSN 高电平字间 ≥400ns（HAL 调用开销 + 1ms 间隔远超，无需垫片）；
 * ENABLE 拉高后 1ms 内 SPI 就绪。 */

static uint16_t drv_xfer(uint16_t frame)
{
  uint16_t rx = 0;
  HAL_GPIO_WritePin(DRV_CSN_GPIO_Port, DRV_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&frame, (uint8_t *)&rx, 1, 100);
  HAL_GPIO_WritePin(DRV_CSN_GPIO_Port, DRV_CSN_Pin, GPIO_PIN_SET);
  return rx;
}

uint16_t DRV8320S_ReadReg(uint8_t addr)
{
  return drv_xfer((uint16_t)((1u << 15) | ((uint16_t)addr << 11)));
}

uint16_t DRV8320S_WriteReg(uint8_t addr, uint16_t data)
{
  return drv_xfer((uint16_t)(((uint16_t)addr << 11) | (data & 0x7FFu)));
}

/* 上电 bring-up 测试：使能 → 读默认值（验读通路）→ 写 G431 四值（验写通路）
 * → 读回对比 → nFAULT 引脚。
 * 前提：电机静止、24V 在线（DRV 的 SPI 供电来自 VM 内部 LDO）。
 * 3x 模式 + INH 悬空下拉 → 三相低侧全开：静止时无压差无电流，属正常空闲态。
 * ⚠ 拖动自校准固件不可含本调用：低侧全开 + 旋转反电动势 = 短路制动。 */
void DRV8320S_BringupTest(void)
{
  char buf[128];
  int len;

  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* 阶段1：写前默认值，对照手册复位值
   * R00/R01=0000，R02=0000，R03=03FF，R04=07FF，R05=0159
   * （R05=TRETRY0/DEAD01/OCP01/DEG01/VDS1001，即 0b0_01_01_01_1001=0x159） */
  uint16_t r00 = DRV8320S_ReadReg(0x00); HAL_Delay(1);
  uint16_t r01 = DRV8320S_ReadReg(0x01); HAL_Delay(1);
  uint16_t r02 = DRV8320S_ReadReg(0x02); HAL_Delay(1);
  uint16_t r03 = DRV8320S_ReadReg(0x03); HAL_Delay(1);
  uint16_t r04 = DRV8320S_ReadReg(0x04); HAL_Delay(1);
  uint16_t r05 = DRV8320S_ReadReg(0x05); HAL_Delay(1);
  len = snprintf(buf, sizeof(buf),
      "[DRV] def R00=%04X R01=%04X R02=%04X R03=%04X R04=%04X R05=%04X"
      " (exp 0/0/0/3FF/7FF/159)\r\n",
      r00, r01, r02, r03, r04, r05);
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20);

  /* 阶段2：写 G431 定稿四值（顺序同 foc.c：R03→R02→R04→R05） */
  DRV8320S_WriteReg(0x03, DRV8320S_CFG_GATE_HS); HAL_Delay(1);
  DRV8320S_WriteReg(0x02, DRV8320S_CFG_DRVCTRL); HAL_Delay(1);
  DRV8320S_WriteReg(0x04, DRV8320S_CFG_GATE_LS); HAL_Delay(1);
  DRV8320S_WriteReg(0x05, DRV8320S_CFG_OCP);     HAL_Delay(1);

  /* 阶段3：读回对比 */
  uint16_t c02 = DRV8320S_ReadReg(0x02); HAL_Delay(1);
  uint16_t c03 = DRV8320S_ReadReg(0x03); HAL_Delay(1);
  uint16_t c04 = DRV8320S_ReadReg(0x04); HAL_Delay(1);
  uint16_t c05 = DRV8320S_ReadReg(0x05); HAL_Delay(1);
  len = snprintf(buf, sizeof(buf),
      "[DRV] rb R02=%04X(%s) R03=%04X(%s) R04=%04X(%s) R05=%04X(%s)\r\n",
      c02, c02 == DRV8320S_CFG_DRVCTRL ? "OK"   : "FAIL",
      c03, c03 == DRV8320S_CFG_GATE_HS ? "OK"   : "FAIL",
      c04, c04 == DRV8320S_CFG_GATE_LS ? "OK"   : "FAIL",
      c05, c05 == DRV8320S_CFG_OCP     ? "OK"   : "FAIL");
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20);

  /* 阶段4：nFAULT 引脚（开漏+上拉，1=无故障） */
  GPIO_PinState fault = HAL_GPIO_ReadPin(nfault_GPIO_Port, nfault_Pin);
  len = snprintf(buf, sizeof(buf), "[DRV] nFAULT=%u\r\n", fault);
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20);
}
