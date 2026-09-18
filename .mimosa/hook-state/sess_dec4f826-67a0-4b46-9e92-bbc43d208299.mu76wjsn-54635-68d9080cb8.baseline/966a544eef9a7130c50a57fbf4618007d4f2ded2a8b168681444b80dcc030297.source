#ifndef __DRV8320S_H
#define __DRV8320S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* 寄存器地址（DRV832xS，drv8323r.pdf Table 10） */
#define DRV8320S_REG_FAULT1    0x00 /* 只读：故障状态 1 */
#define DRV8320S_REG_VGS       0x01 /* 只读：VGS 状态 2 */
#define DRV8320S_REG_DRVCTRL   0x02 /* PWM_MODE[6:5] 等 */
#define DRV8320S_REG_GATE_HS   0x03 /* LOCK / IDRIVE HS */
#define DRV8320S_REG_GATE_LS   0x04 /* CBC / TDRIVE / IDRIVE LS */
#define DRV8320S_REG_OCP       0x05 /* 死区 / OCP 配置 */

/* G431 定稿配置值（真机验证，H563 照抄；R05 死区 bits[9:8]=01b=100ns） */
#define DRV8320S_CFG_DRVCTRL   0x020 /* PWM_MODE=01b → 3x PWM 模式 */
#define DRV8320S_CFG_GATE_HS   0x3EE /* LOCK=011 解锁 / IDRIVE=820/880mA */
#define DRV8320S_CFG_GATE_LS   0x6EE /* CBC=1 / TDRIVE=2000ns / IDRIVE=820/880mA */
#define DRV8320S_CFG_OCP       0x213 /* 死区100ns / OCP锁存 / 4μs / VDS=0.26V */

uint16_t DRV8320S_ReadReg(uint8_t addr);
uint16_t DRV8320S_WriteReg(uint8_t addr, uint16_t data); /* 返回写前旧值 */
void DRV8320S_BringupTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV8320S_H */
