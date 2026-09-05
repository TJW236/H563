#ifndef __MT6835_H
#define __MT6835_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* SPI 帧头 4bit 命令字（手册 7.6.3） */
#define MT6835_CMD_READ_REG   0x3   /* 0011 单字节读寄存器 */
#define MT6835_CMD_CONT_READ  0xA   /* 1010 连续读角度 0x003~0x006 */
#define MT6835_CMD_WRITE_REG  0x6   /* 0110 单字节写寄存器 */
#define MT6835_CMD_BURN_EEPROM 0xC  /* 1100 烧录 EEPROM（地址域全 0，ACK 0x55） */

/* 常用寄存器地址 */
#define MT6835_REG_USER_ID    0x001 /* 客户可用测试寄存器（EEPROM） */

/* STATUS 位定义（0x005 低 3 位，'1'=报警） */
#define MT6835_ST_OVERSPD     0x01 /* bit0 超速 >120krpm */
#define MT6835_ST_WEAKFIELD   0x02 /* bit1 磁场太弱 */
#define MT6835_ST_UNDERVOLT   0x04 /* bit2 芯片欠压 */

#define MT6835_ANGLE_MOD      2097152UL /* 2^21 */

uint8_t MT6835_ReadReg(uint16_t addr);
void MT6835_WriteReg(uint16_t addr, uint8_t data); /* 写影子寄存器：立即生效，断电即丢，掉电保持须另发 1100 烧录命令 */
uint8_t MT6835_BurnEEPROM(void); /* 1100 烧录：寄存器表整体快照进 EEPROM；返回 MISO 应答，0x55=正确接收 */
void MT6835_ReadAngle(uint32_t *angle, uint8_t *status, uint8_t *crc);

#ifdef __cplusplus
}
#endif

#endif /* __MT6835_H */
