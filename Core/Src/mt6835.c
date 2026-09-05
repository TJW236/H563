#include "mt6835.h"
#include "spi.h"
#include "main.h"

/* CSN 时序垫片（同 G431 SPI_DelayTicks 的做法，那边用 TIM16，这边 volatile NOP）：
 * 手册要求 TL(CSN↓→首SCK↓)≥100ns、TH(末SCK↑→CSN↑)≥0.5*TSCK。
 * 每次迭代约 15~25ns@250MHz，取 2 倍以上余量。
 * 将来高频 DMA 读再换 DWT/TIM 精确延时。 */
#define MT6835_CS_LEAD_TICKS  16u   /* ~250~400ns，垫在 CSN 拉低后 */
#define MT6835_CS_LAG_TICKS   40u   /* ~600~1000ns，垫在 CSN 拉高前 */

static void MT6835_DelayTicks(uint16_t n)
{
  for (volatile uint16_t i = 0; i < n; i++) { __NOP(); }
}

/* 单字节读寄存器：24bit 帧 = [0011][addr12][数据8]
 * 每次调用一次 CSN 拉低拉高周期。注意 CSN 下降沿会重新锁存角度寄存器，
 * 所以本函数只用于配置/状态类寄存器，读角度必须用连读保证快照一致。 */
uint8_t MT6835_ReadReg(uint16_t addr)
{
  uint8_t tx[3] = { (uint8_t)((MT6835_CMD_READ_REG << 4) | (addr >> 8)),
                    (uint8_t)(addr & 0xFF),
                    0x00 };
  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_RESET);
  MT6835_DelayTicks(MT6835_CS_LEAD_TICKS);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
  MT6835_DelayTicks(MT6835_CS_LAG_TICKS);
  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_SET);

  return rx[2];
}

/* 单字节写寄存器（手册 7.6.5）：24bit 帧 = [0110][addr12][data8]。
 * 只写影子寄存器（立即生效），断电即回 EEPROM 载入值。 */
void MT6835_WriteReg(uint16_t addr, uint8_t data)
{
  uint8_t tx[3] = { (uint8_t)((MT6835_CMD_WRITE_REG << 4) | (addr >> 8)),
                    (uint8_t)(addr & 0xFF),
                    data };
  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_RESET);
  MT6835_DelayTicks(MT6835_CS_LEAD_TICKS);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
  MT6835_DelayTicks(MT6835_CS_LAG_TICKS);
  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_SET);
}

/* 烧录 EEPROM（手册 7.6.6）：24bit 帧 = [1100][addr12 全 0][数据域 0]。
 * 正确接收则 MISO 回 0x55（其他值=接收失败）。此命令把所有 EEPROM 映射
 * 寄存器按当前寄存器表值整体写入；发出后须等 ≥6s 再给芯片断电。 */
uint8_t MT6835_BurnEEPROM(void)
{
  uint8_t tx[3] = { (uint8_t)(MT6835_CMD_BURN_EEPROM << 4), 0x00, 0x00 };
  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_RESET);
  MT6835_DelayTicks(MT6835_CS_LEAD_TICKS);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10);
  MT6835_DelayTicks(MT6835_CS_LAG_TICKS);
  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_SET);

  return rx[2];
}

/* 连续读角度（手册 7.6.9）：一次 CSN 拉低 = 一次锁存快照。
 * 发 [1010][0x003] 后连续收 4 字节：rx[2]=ANGLE[20:13] rx[3]=ANGLE[12:5]
 * rx[4]=ANGLE[4:0]+STATUS rx[5]=CRC，四字节同源。 */
void MT6835_ReadAngle(uint32_t *angle, uint8_t *status, uint8_t *crc)
{
  uint8_t tx[6] = { (uint8_t)(MT6835_CMD_CONT_READ << 4), 0x03, 0, 0, 0, 0 };
  uint8_t rx[6] = {0};

  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_RESET);
  MT6835_DelayTicks(MT6835_CS_LEAD_TICKS);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, 10);
  MT6835_DelayTicks(MT6835_CS_LAG_TICKS);
  HAL_GPIO_WritePin(MT1_CSN_GPIO_Port, MT1_CSN_Pin, GPIO_PIN_SET);

  *angle  = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
  *status = rx[4] & 0x07;
  *crc    = rx[5];
}
