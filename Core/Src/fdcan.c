/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include <math.h>
#include "foc.h"

/* 【CAN-FD BRS 1Mbps / 2Mbps】（2026-09-19 应用层自 G431 7-31 定稿移植；协议见 fdcan.h）
 * 位定时差异（内核时钟 G431=170M PCLK1 → H563=100M PLL1Q）：
 *   仲裁 1Mbps：Psc=10, SJW=1, Seg1=7, Seg2=2 → 100M/10/10tq，SP=80%（与 G431 同形状）
 *   数据 2Mbps：Psc=5,  SJW=1, Seg1=7, Seg2=2 → 100M/5/10tq，SP=80%
 * 无 TDC：bit_time=500ns > TCAN1057A 环路延迟 ~255ns，发节点能采到自己的 RX（G431 同结论）；
 * 25MHz HSE 内核配不出 2M（12.5tq 非整数）→ 100M PLL1Q 方案定案见 CLAUDE.md。
 * 中断：FDCAN1_IT0 优先级 5 = GPDMA0 同级（互不抢占，电流环地位不动），高于 TIM3(6) */
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /* 过滤器（照抄 G431）：RANGE 0x000~0x184 → RXFIFO0，覆盖状态请求广播 + 4 电机速度
   * (0x101~0x104) + 位置 (0x181~0x184) 帧；其他节点 ID 的帧硬件丢弃不进 ISR。
   * ⚠ FDCAN 必须配 ≥1 过滤器才能 RX（与 F407 bxCAN 不同，G431 踩坑记录） */
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_ID_STATUS_REQUEST;
  filter.FilterID2 = CAN_ID_POSITION_BASE + 4;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 + RX FIFO0 新消息中断（NVIC 已在 MspInit 使能） */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ==================== RX 回调（FDCAN1_IT0 中断触发，优先级 5） ==================== */

/* 跨上下文变量：foc.c 定义（TIM3 ISR 读，本 ISR/UART 任务写）——foc.h 已 extern */

/* CAN 控制帧与 UART s/p 命令同一套模式变量（speed_mode/pos_mode/...），双入口同账本：
 * 速度帧=CAN 版 s（bumpless 积分预置同款），位置帧=CAN 版 p（×GEAR_RATIO 入电机轴域）。
 * ⚠ 竞态说明（G431 同架构在跑）：本 ISR（5）可抢占 TIM3 速度环（6）——pi_speed.integral1
 * 的预置与 PI 写回理论可交错，最坏丢一次预置=模式切入瞬间小电流阶跃，接受 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
        return;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
        return;

    uint32_t id = rx_header.Identifier;

    if (id == CAN_ID_STATUS_REQUEST)
    {
        /* 状态请求广播帧：塞状态帧到 TX FIFO（CPU ~3µs，硬件自己发）。
         * 只回速度（主控 PID 反馈用），与 G431 一致 */
        FDCAN_SendStatus(MOTOR_ID, shadow_speed);
    }
    else if (id == (CAN_ID_SPEED_BASE + MOTOR_ID))
    {
        if (ntc_trip)   /* OTP 跳闸期间拒收控制帧（与 UART s 同规；ntc_trip 任务侧 50ms 判温 */
            return;
        /* DLC 防护（2026-09-20 加固）：短帧时 data[] 尾部是 ISR 栈未初始化字节，memcpy
         * 出任意 float——NaN 有 isfinite 兜底，有限值垃圾最坏被限幅成 ±800 满速。
         * 协议定义 DLC=8，不足 4 字节整帧丢弃 */
        if (rx_header.DataLength < FDCAN_DLC_BYTES_4)
            return;
        /* 速度控制帧：NaN/Inf（上位机 bug/未初始化内存）强制清零绕过 PI 防积分爆炸；限幅 ±800 */
        float spd;
        memcpy(&spd, data, 4);
        if (!isfinite(spd)) spd = 0.0f;
        if (spd >  800.0f) spd =  800.0f;
        if (spd < -800.0f) spd = -800.0f;
        g_foc.pi_speed.integral1 = shadow_iq_ref;   /* bumpless：接管无电流阶跃 */
        speed_target = spd;
        speed_mode = 1;    /* 写序：目标先、模式后（模式=提交标志） */
        pos_mode = 0;
    }
    else if (id == (CAN_ID_POSITION_BASE + MOTOR_ID))
    {
        if (ntc_trip)
            return;
        /* DLC 防护（同上）：位置帧需 pos+max_spd 共 8 字节，不足整帧丢 */
        if (rx_header.DataLength < FDCAN_DLC_BYTES_8)
            return;
        /* 位置控制帧：pos 输出轴 rad ×GEAR_RATIO 入电机轴误差域（kp=54 直拷前提）；
         * max_spd 动态改位置环输出上限（0=禁止输出；上限恢复默认 800 走 p/位置帧带非零值） */
        float pos, max_spd;
        memcpy(&pos,     data,     4);
        memcpy(&max_spd, data + 4, 4);
        if (!isfinite(pos))     pos     = 0.0f;
        if (!isfinite(max_spd)) max_spd = 0.0f;
        if (max_spd > 800.0f) max_spd = 800.0f;
        if (max_spd <   0.0f) max_spd =   0.0f;
        g_foc.pi_speed.integral1 = shadow_iq_ref;   /* bumpless 同 s */
        pos_target = pos * GEAR_RATIO;
        g_foc.pi_pos.max = max_spd;
        pos_mode = 1;
        speed_mode = 1;    /* 位置环经速度环出力（与 p 命令同） */
    }
    /* 其他 ID（别的电机控制帧）已被过滤器硬件丢弃，不进本 ISR */
}

/* ==================== TX 函数（塞 TX FIFO 立即返回；FIFO 3 槽满则丢弃返错） ==================== */

void FDCAN_SendStatus(uint8_t motor_id, float speed_rpm)
{
    FDCAN_TxHeaderTypeDef header;
    header.Identifier          = CAN_ID_STATUS_BASE + motor_id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;        /* BRS：8 byte 走 2M 数据段 */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_ON;
    header.FDFormat            = FDCAN_FD_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    uint8_t data[CAN_DLC_STATUS] = {0};
    memcpy(&data[0], &speed_rpm, 4);

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

void FDCAN_SendHeartbeat(uint8_t motor_id, uint8_t state)
{
    FDCAN_TxHeaderTypeDef header;
    header.Identifier          = CAN_ID_HEARTBEAT_BASE + motor_id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_ON;
    header.FDFormat            = FDCAN_FD_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    uint8_t data[CAN_DLC_HEARTBEAT] = {0};
    data[0] = state;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

void FDCAN_SendError(uint8_t motor_id, uint32_t err_code)
{
    FDCAN_TxHeaderTypeDef header;
    header.Identifier          = CAN_ID_ERROR_BASE + motor_id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_ON;
    header.FDFormat            = FDCAN_FD_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    uint8_t data[CAN_DLC_ERROR] = {0};
    memcpy(&data[0], &err_code, 4);

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

/* ==================== Bus-Off 自愈（500ms 任务侧看护）====================
 * 陷阱（G431 在案坑）：总线上无节点应答（CANable 未接/断电）时 TX 无 ACK，
 * AutoRetransmission 无限重传 → TEC 爬满 255 → Bus_Off 静默且不自动恢复——
 * 之后接上总线也没心跳，必须断电重上电。看护：PSR.BO 置位则 Stop+Start
 * 重新走 init→normal 序列（TEC 清零）；总线恢复后 ≤500ms 回来。
 * 纯任务上下文调用，不碰 ISR；Stop 期间 RX/TX 硬件停住无竞态 */
void FDCAN_BusOffWatch(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) != HAL_OK)
        return;
    if (ps.BusOff)
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_Start(&hfdcan1);
    }
}

/* USER CODE END 1 */

