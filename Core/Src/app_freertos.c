/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "main.h"
#include "mt6835.h"
#include "usart.h"
#include "tim.h"
#include "foc.h"
#include "drv8320s.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint16_t adc_buf[5];                /* main.c：0=Iu 1=Iv 2=Iw 3=NTC 4=PWR */
extern volatile uint32_t adc_scan_cnt;     /* main.c：ConvCplt 累计（30kHz） */
extern volatile uint8_t uart_rx_byte;      /* main.c：UART 命令接收（照抄 G431） */
extern volatile uint8_t uart_cmd_ready;
extern volatile uint8_t uart_cmd_idx;
extern volatile char uart_cmd_buf[32];
extern volatile uint8_t foc_run;           /* main.c：z 重对齐流程停/复环用 */
extern volatile uint32_t foc_dt_last;      /* main.c：GPDMA0 ISR 耗时统计（DWT 周期） */
extern volatile uint32_t foc_dt_max;
extern volatile uint64_t foc_dt_sum;
extern volatile uint32_t foc_dt_n;
#if 0 /* ---- v6 拖动自校准监控状态停用（恢复时随 SpeedObserveTask 一起启用）----
static volatile struct {
  int32_t  rpm;
  uint8_t  st;
  uint8_t  cal_st;
  uint8_t  trig;
  uint8_t  gear;
  uint32_t win_exit;
  uint32_t ok_tick;
} mon;
static const char *cal_name[4] = { "none", "busy", "FAIL", "OK" };
---- v6 end ---- */
#endif
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for UartTXTask */
osThreadId_t UartTXTaskHandle;
const osThreadAttr_t UartTXTask_attributes = {
  .name = "UartTXTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1048 * 4
};
/* Definitions for SpeedObserveTask */
osThreadId_t SpeedObserveTaskHandle;
const osThreadAttr_t SpeedObserveTask_attributes = {
  .name = "SpeedObserveTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1048 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float ntc_to_celsius(uint16_t code);   /* NTC 码 → °C（β 方程，定值见 foc.h） */
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UartTXTask */
  UartTXTaskHandle = osThreadNew(StartTask02, NULL, &UartTXTask_attributes);

  /* creation of SpeedObserveTask */
  SpeedObserveTaskHandle = osThreadNew(StartTask03, NULL, &SpeedObserveTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* 心跳灯：500ms 翻转。不依赖串口/SPI——亮 = 调度器活着 */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the UartTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN UartTXTask */
  /* 唯一的 UART 发送者（09-01 改版）：VOFA+ FireWater 图表流，50ms 一帧纯 CSV（09-04 降频：
   * CSV 判的是跟踪误差/id 漂移包络/转速稳定度等慢量，20Hz 采样够用，数据量降 60%）：
   *   iq×100, 目标iq_ref×100, id×100, 电角度0~359°, 电角度周期计数(09-02加), NTC×10 °C(09-05加)
   * 电流 0.01A 定标（50=0.50A）；FireWater 把行内所有数字当采样点，故正常时
   * 只发 CSV、不发任何带数字的文本行。用 int 不用 int32_t：newlib 上 int32_t=long。
   *
   * 命令（行协议照抄 G431：字母+空格+数值+\n，中断侧行缓冲，此处轮询解析）：
   *   q 数值  设 iq_ref，0.01A 定标（q 50=0.50A，q 0 归零），限幅 ±10A
   *   z       重对齐：停环→5V 锁 d 轴→复环（CSV 停 ~1s 属预期，电机会抽一下）
   *   c       清零电角度周期计数（第 5 列，判 1:1 vs ÷8 跟踪速率用）
   *   v 数值  静态矢量探针（0~359 电角度）：2V 矢量+编码器/ADC 交叉验证，
   *           判功放/采样链方向传递（v 0/90/180/270，先 q 0）
   *   t       GPDMA0 ISR 耗时快照（DWT 周期计数，覆盖 HAL 分发+FOC+SVPWM+重挂）：
   *           t/max/avg 单位 µs，load=avg/33.34µs 周期；n 为区间样本数（÷30000=秒数）。
   *           静止与旋转读数应几乎一致（FOC 无分支）= 测量自洽性判据
   *   w       按需查询母线电压/NTC 温度（09-05 上线；正常静默防污染 CSV，
   *           超温 80°C 另有 [ntc] 报警行） */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_byte, 1);  /* NVIC 已由 usart.c 使能 */
  uint8_t div = 0;
  uint32_t rate_prev_tick = 0, rate_prev_scan = 0;   /* ADC 速率守卫基准（首轮只记录） */
  uint8_t rate_armed = 0;
  for(;;)
  {
    if (uart_cmd_ready)
    {
      uart_cmd_buf[uart_cmd_idx] = '\0';

      /* 简易浮点解析（照抄 G431）：空格后的 [+|-]数字[.数字] */
      float val = 0.0f;
      {
        char *p = (char *)uart_cmd_buf;
        while (*p && *p != ' ') p++;
        if (*p == ' ')
        {
          p++;
          int neg = 0;
          if (*p == '-') { neg = 1; p++; }
          while (*p >= '0' && *p <= '9')
            val = val * 10.0f + (float)(*p++ - '0');
          if (*p == '.')
          {
            p++;
            float frac = 0.1f;
            while (*p >= '0' && *p <= '9')
            {
              val += (float)(*p++ - '0') * frac;
              frac *= 0.1f;
            }
          }
          if (neg) val = -val;
        }
      }

      char ebuf[80];
      int elen = 0;
      if (uart_cmd_buf[0] == 'q')
      {
        float a = val * 0.01f;
        if (a > 10.0f) a = 10.0f;
        if (a < -10.0f) a = -10.0f;
        shadow_iq_ref = a;
        elen = snprintf(ebuf, sizeof(ebuf), "q=%d\r\n", (int)(a * 100.0f));
      }
      else if (uart_cmd_buf[0] == 'z')
      {
        foc_run = 0;
        FOC_ZeroAlign(&g_foc);   /* 内部 HAL_Delay×2：阻塞本任务 ~1s */
        foc_run = 1;
        elen = snprintf(ebuf, sizeof(ebuf), "z re-aligned\r\n");
      }
      else if (uart_cmd_buf[0] == 'c')
      {
        FOC_ResetElecRevs();
        elen = snprintf(ebuf, sizeof(ebuf), "c cleared\r\n");
      }
      else if (uart_cmd_buf[0] == 'w')
      {
        /* 按需查询：vbus 显示 EMA 值（控制实际用的那个），NTC 现算 */
        float t = ntc_to_celsius(adc_buf[3]);
        int v10 = (int)(g_foc.vbus * 10.0f);
        int t10 = (int)(t * 10.0f);
        int tfr = t10 % 10; if (tfr < 0) tfr = -tfr;   /* 负温时余数也为负 */
        elen = snprintf(ebuf, sizeof(ebuf), "[pwr] vbus=%d.%dV ntc=%d.%dC\r\n",
                        v10 / 10, v10 % 10, t10 / 10, tfr);
      }
      else if (uart_cmd_buf[0] == 'v')
      {
        int deg = (int)val;
        while (deg < 0) deg += 360;
        deg %= 360;
        foc_run = 0;                       /* 先停环：CCR 无人覆写，矢量才锁得住 */
        FOC_StaticVectorProbe(2.0f, (uint16_t)deg);  /* 内含 1.2s 保持+采样，CSV 停 ~1.5s 属预期 */
        foc_run = 1;
        elen = snprintf(ebuf, sizeof(ebuf), "v done\r\n");
      }
      else if (uart_cmd_buf[0] == 't')
      {
        /* 关中断快照+清零（约几 µs，30kHz 中断最多晚一拍）：sum 64 位读也无需原子性 */
        uint32_t last, max, n;
        uint64_t sum;
        __disable_irq();
        last = foc_dt_last; max = foc_dt_max; n = foc_dt_n; sum = foc_dt_sum;
        foc_dt_last = 0; foc_dt_max = 0; foc_dt_n = 0; foc_dt_sum = 0;
        __enable_irq();
        if (last == 0 && n > 1)
        {
          elen = snprintf(ebuf, sizeof(ebuf), "[foc] DWT=0 n=%lu - CYCCNT 未计数\r\n",
                          (unsigned long)n);
        }
        else
        {
          if (n == 0) n = 1;
          uint32_t avg = (uint32_t)(sum / n);
          /* µs 整数部分=周期/250，0.1µs=周期/25；8334 周期=一个 PWM 周期(33.34µs) */
          elen = snprintf(ebuf, sizeof(ebuf),
              "[foc] t=%lu.%lu max=%lu.%lu avg=%lu.%lu us n=%lu load=%lu%%\r\n",
              (unsigned long)(last / 250UL), (unsigned long)(last / 25UL % 10UL),
              (unsigned long)(max / 250UL),  (unsigned long)(max / 25UL % 10UL),
              (unsigned long)(avg / 250UL),  (unsigned long)(avg / 25UL % 10UL),
              (unsigned long)n, (unsigned long)(avg * 100UL / 8334UL));
        }
      }
      if (elen > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)ebuf, (uint16_t)elen, 10);

      uart_cmd_idx = 0;
      uart_cmd_ready = 0;
    }

    osDelay(50);   /* CSV 帧+命令轮询共用此周期：命令响应最坏 50ms，无感 */
    char buf[64];
    int iq_c = (int)(shadow_iq * 100.0f);
    int tgt_c = (int)(shadow_iq_ref * 100.0f);
    int id_c = (int)(shadow_id * 100.0f);
    int th = (int)((uint32_t)shadow_theta16 * 360UL / 65536UL);
    int ntc_c = (int)(ntc_to_celsius(adc_buf[3]) * 10.0f);   /* 0.1°C 定标；单帧原始码 ±2LSB ≈ ±0.5°C 抖动属正常 */
    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d\n",
                       iq_c, tgt_c, id_c, th, (int)shadow_elec_revs, ntc_c);
    if (len > 0)
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20);
    /* DRV 健康巡检 ~500ms（10 帧）一次：仅故障态才打印（nFAULT 低或 FSR 非零），
     * 正常静默防污染 CSV 通道。纯只读——锁存故障仅 CLR_FLT/EN 脉冲可清，读无副作用 */
    if (++div >= 10)
    {
      div = 0;
      /* ADC 速率守卫（09-02 30kHz 迁移）：H5 One-Shot 若 UPDATE 触发到来时回调
       * 未重挂完 ADSTART，该拍静默丢失 → 环频下降无任何症状。按实际耗时估期望帧数
       * （30 帧/ms），偏差 >2% 才打印，正常静默不污染 CSV。首轮只记录基准 */
      uint32_t now_tick = HAL_GetTick();
      uint32_t now_scan = adc_scan_cnt;
      if (rate_armed && now_tick > rate_prev_tick)
      {
        uint32_t exp = (now_tick - rate_prev_tick) * 30UL;   /* 30kHz × 毫秒数 */
        uint32_t got = now_scan - rate_prev_scan;
        if (got + exp / 50UL < exp || got > exp + exp / 50UL)
        {
          char rbuf[48];
          int rlen = snprintf(rbuf, sizeof(rbuf), "[adc-rate] got=%u exp=%u\r\n",
                              (unsigned int)got, (unsigned int)exp);
          if (rlen > 0)
            HAL_UART_Transmit(&huart1, (uint8_t *)rbuf, (uint16_t)rlen, 20);
        }
      }
      rate_armed = 1;
      rate_prev_tick = now_tick;
      rate_prev_scan = now_scan;
      uint8_t nf = (uint8_t)HAL_GPIO_ReadPin(nfault_GPIO_Port, nfault_Pin);
      uint16_t f0 = DRV8320S_ReadReg(0x00);
      uint16_t f1 = DRV8320S_ReadReg(0x01);
      if (nf == 0 || f0 || f1)
      {
        char dbuf[48];
        int dlen = snprintf(dbuf, sizeof(dbuf), "[drv] nF=%u F0=%04X F1=%04X\r\n", nf, f0, f1);
        if (dlen > 0)
          HAL_UART_Transmit(&huart1, (uint8_t *)dbuf, (uint16_t)dlen, 20);
      }

      /* 母线电压实测（09-05 上线）：EMA 平滑 adc_buf[4] 喂 g_foc.vbus——SVPWM
       * 归一化/PI 限幅用真实 Udc，VBUS_UNDER/OVER 保护随之激活。任务写、
       * ISR 读：单字对齐 float 写原子无撕裂；EMA α=0.25 → ~2s 收敛，
       * 单帧毛刺进不了控制；首帧直接初始化（vbus_ema=0 哨兵） */
      {
        static float vbus_ema = 0.0f;
        float v = (float)adc_buf[4] * (3.3f / 4096.0f) * PWR_DIV_RATIO;
        if (vbus_ema == 0.0f) vbus_ema = v;
        vbus_ema += 0.25f * (v - vbus_ema);
        g_foc.vbus = vbus_ema;
      }
      /* NTC 超温报警：正常静默（同 [drv] 哲学，不污染 CSV），查询走 w 命令 */
      if (ntc_to_celsius(adc_buf[3]) > NTC_WARN_C)
      {
        char nbuf[40];
        int nlen = snprintf(nbuf, sizeof(nbuf), "[ntc] over %dC\r\n", (int)NTC_WARN_C);
        if (nlen > 0)
          HAL_UART_Transmit(&huart1, (uint8_t *)nbuf, (uint16_t)nlen, 20);
      }
    }
  }
  /* USER CODE END UartTXTask */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the SpeedObserveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN SpeedObserveTask */
  /* v6 拖动自校准逻辑停用（根因=⌀6 磁铁不足；⌀10×2.5 径向到货后重烧
   * H563_v6_dragcal_20260829.elf 重跑，无需改源码）。
   * 任务本体保留：MX_FREERTOS_Init 固定创建，三环移植时改造为速度环。
   * 旧 v6 任务体（20ms 绝对周期采角 + 310-330 门控 + 0x113 轮询）见
   * 归档 git/任务进度.md 2026-08-28~29 日志 */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END SpeedObserveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* NTC 码 → 温度°C：R = 10k×(4096/code−2) → β 方程（NTC_* 定值见 foc.h）。
 * code 下限钳 1：NTC 断路（极冷）时读数趋向 −90°C，一眼可辨 */
static float ntc_to_celsius(uint16_t code)
{
  float c = (float)code;
  if (c < 1.0f) c = 1.0f;
  float r = NTC_R_FIXED * (4096.0f / c - 2.0f);
  return 1.0f / (1.0f / NTC_T25_K + logf(r / NTC_R25) / NTC_BETA) - 273.15f;
}
/* USER CODE END Application */

