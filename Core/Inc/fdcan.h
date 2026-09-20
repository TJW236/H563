/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* CAN 帧协议定义（2026-09-19 自 G431 7-31 定稿照抄，CAN-FD BRS）
 *
 * 帧 ID 分配（标准 11-bit，根据 MOTOR_ID i=1~4 偏移；i=0 预留给 IMU）：
 *   0x000              状态请求（主控广播，所有节点接收；IMU 收但不响应）
 *   0x100+i (i=1~4)    速度控制（主控→#i 电机，spd_ref 切速度模式，轮电机用）
 *   0x180+i (i=1~4)    位置控制（主控→#i 电机，pos_ref+max_speed 切三环，髋电机用）
 *   0x200+i (i=0~4)    状态（#i→主控，speed_rpm；i=0 是 IMU 数据）
 *   0x300+i (i=0~4)    心跳（#i→主控，state_u8，defaultTask 500ms）
 *   0x400+i (i=0~4)    错误（#i→主控，err_code_u32，仅错误时）
 *
 * byte layout（全部 little-endian，DLC=8 走 2M 数据段）：
 *   速度控制：[spd_ref_rpm_f32 ×4][pad ×4]          ← H563 语义 = CAN 版 s 命令
 *   位置控制：[pos_ref_f32 ×4][max_speed_f32 ×4]    ← H563 语义 = CAN 版 p 命令
 *   状态    ：[speed_rpm_f32 ×4][pad ×4]
 *   心跳    ：[state_u8 ×1][pad ×7]
 *   错误    ：[err_code_u32 ×4][pad ×4]
 *
 * 单位约定（同 G431）：pos=输出轴 rad（接收时 ×GEAR_RATIO 转电机轴）；
 * speed_rpm=电机轴 RPM；spd_ref 限 ±800；max_speed 限 0~800（0=禁止输出）
 */
#define CAN_ID_STATUS_REQUEST   0x000   /* 主控广播 */
#define CAN_ID_SPEED_BASE       0x100   /* +i，速度控制帧 */
#define CAN_ID_POSITION_BASE    0x180   /* +i，位置控制帧 */
#define CAN_ID_STATUS_BASE      0x200   /* +i */
#define CAN_ID_HEARTBEAT_BASE   0x300   /* +i */
#define CAN_ID_ERROR_BASE       0x400   /* +i */

#define CAN_DLC_SPEED_CTRL      8
#define CAN_DLC_POSITION_CTRL   8
#define CAN_DLC_STATUS          8
#define CAN_DLC_HEARTBEAT       8
#define CAN_DLC_ERROR           8

/* 心跳 state 字 */
#define CAN_STATE_INIT          0
#define CAN_STATE_OPERATIONAL   1
#define CAN_STATE_FAULT         2
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
/* TX 函数（RX ISR / 任务中调用，只塞 TX FIFO 不阻塞；FIFO 3 槽，塞满返错不等待） */
void FDCAN_SendStatus(uint8_t motor_id, float speed_rpm);
void FDCAN_SendHeartbeat(uint8_t motor_id, uint8_t state);
void FDCAN_SendError(uint8_t motor_id, uint32_t err_code);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

