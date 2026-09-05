# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> 本文件只覆盖 H563 工程的**构建/烧录命令**与**代码地图**。硬件参数、PI 定档、调试结论、bring-up 进度一律以上级 `../CLAUDE.md`（motor_driver 项目总规则）为权威，两处冲突时以上级为准；进度日志写 `../任务进度.md`。固件版本以 main.c USER CODE 2 开头的横幅字符串为准（现行 step6f）。

## 构建

STM32CubeIDE 1.18.1 工程（CubeMX 生成，MCU=STM32H563RIT6，TrustZone=Disabled，FreeRTOS CMSIS-RTOS2，HAL 时基=TIM7）。

- IDE 可执行文件：`/opt/st/stm32cubeide_1.18.1/stm32cubeide`；工作区 `~/STM32CubeIDE/workspace_1.18.1`
- 日常构建走 IDE GUI（Project → Build），产物 `Debug/H563.elf`
- **Debug 配置 = -O3**（2026-09-04 定档上板）；Release 配置 = -Os（未启用，勿拿它烧板对比性能）
- 改 `.ioc` 后在 CubeMX 重新生成 → **Refresh + Clean + Rebuild**（曾因 Debug/ 旧 makefile 缺 Middlewares 路径报 FreeRTOS.h 找不到）
- ⚠ `.ioc` 重生成必须保持 TIM1 Counter Period=**4167**（30kHz）与 **RepetitionCounter=1**——Period 与 `Core/Inc/foc.h` 的 `TIMER_ARR` 强耦合（不同步则占空比超标 1.5 倍）；RCR=1 保证 UPDATE 每 PWM 周期恰一次（RCR=0 时中心对齐上下顶点双发，重挂追得上半周期就实跑 60kHz，09-05 定案）
- ⚠ CubeMX 大改/挪引脚会抹掉引脚标签宏与上拉参数——重生成后核对 `main.h` 标签 + `gpio.c` 上拉（OSC32 脚 PC14/PC15 踩过）

## 烧录

- CubeIDE Run（SWD + ST-Link），调试配置 `H563.launch`
- 烧后零输出先查 IDE Console 有无 "Download verification failed" 红字：有红字=烧录链路问题（降 SWD 速率/重插 ST-Link），无红字=才 Clean 重建查代码
- `H563_v6_dragcal_20260829.elf`（工程根）= 拖动自校准 v6 完整备份，⌀10 磁铁到货直接重烧重跑，不改源码

## 代码地图

CubeMX 生成文件（adc/spi/tim/usart/gpdma/gpio/icache.c、stm32h5xx_msp/timebase/it.c）只做外设初始化；用户代码全在 `/* USER CODE */` 区内，区外改动重生成即丢。逻辑集中在五个文件：

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 点火链（USER CODE 2）+ ADC/UART 回调（ConvCplt 分发 FOC、RxCplt 行缓冲） |
| `Core/Src/foc.c` / `Inc/foc.h` | 电流环 PI + Clarke/Park + SVPWM + 零点对齐 + 静态矢量探针；foc.h 集中硬件宏（TIMER_ARR / ENC_DIR / CURRENT_SCALE / NTC-PWR 换算）+ 1024 点 sin 表 + fast_sin/cos |
| `Core/Src/app_freertos.c` | 三任务 + 串口命令解析 + CSV 流 + 500ms 巡检（DRV FSR / ADC 速率守卫 / vbus EMA / NTC 报警） |
| `Core/Src/mt6835.c` | 编码器 SPI（Mode 3：单读 / 连读 21-bit 角 / 写影子 / BurnEEPROM，NOP 时序垫片） |
| `Core/Src/drv8320s.c` | 栅极驱动 SPI（Mode 1 16bit，BringupTest 写 G431 定稿四值 R02/R03/R04/R05） |

### 30kHz 电流环数据流（跨文件主线）

```
TIM1 中心对齐 ARR=4167 RCR=1 ──TRGO2=UPDATE（每 PWM 周期恰一次）──▶ ADC1 3ch 扫描（电流 6.5 拍=0.9µs 定稿——INA240 直连无 RC、源阻抗 Ω 级，短采样无建立之忧；NTC/PWR 09-05 迁 ADC2）
  → GPDMA1_CH0 One-Shot ──▶ GPDMA1_Channel0_IRQHandler（stm32h5xx_it.c：DWT 记 t0/dt）
  → HAL_ADC_ConvCpltCallback（main.c）：foc_run 时 FOC_CurrentLoop → svpwm → 写 TIM1 CCR
      ↳ 回调尾部重挂 HAL_ADC_Start_DMA（H5 One-Shot 块结束硬件清 ADSTART，重挂安全）
      ↳ 重挂后关 DMA_IT_HT（H5 HAL 无条件挂半传输回调，不关则中断翻倍）
```

- **CCR 映射：CCR1→W(PA8) / CCR2→V(PA9) / CCR3→U(PA10)**（与 G431 相比 U/W 互换）；foc.c 的 CurrentLoop / ZeroAlign(Open) / align_hold / StaticVectorProbe **四处**写 CCR 必须同映射，否则零点定义在反射坐标系里起环必发散
- `adc_buf[0..2]` = Iu(PA0) / Iv(PA1) / Iw(PA2)；NTC(PA3)/PWR(PC5) 走 **ADC2 软件单发**——UartTXTask 50ms 拍里 `adc2_read()`（HAL_ADC_Start + 两次 PollForConversion，640.5 拍长采样保分压建立精度），与 30kHz 控制链零耦合
- 角度链：开机 SPI 读一次 21-bit 绝对角播种 TIM2 CNT（ang>>5）→ 运行时只读 CNT（ABZ 16384 线 ×4 = 65536 cpr 恰满 16bit 回绕）；ABZ 计数方向与 SPI 角相反，由 `ENC_DIR=-1` 在软件消化

### main.c 点火链（USER CODE 2，调度器启动前，顺序不可换）

横幅 → DWT CYCCNT 使能（LAR 解锁字直写 0xE0001FB0，CMSIS 5.6 无 LAR 成员）→ `DRV8320S_BringupTest()` → `HAL_Delay(200)` 等 MT6835 EEPROM→寄存器导入 → ABZ 影子写 0x007/0x008 → 播种 TIM2 → `FOC_Init` + ADC 校准 + `Start_DMA` + TIM1 起 → 零流校准 1000 均值（覆盖三相共模偏置）→ `FOC_ZeroAlign`（v2 闭环双侧逼近，ALIGN_CURRENT_A 见 foc.h，~2.5s；开环 5V 版保留 foc.c 备用）→ `foc_run=1`。顺序关键点：首个 UPDATE 触发到来时 DMA 必须已就位。

### FreeRTOS 三任务（app_freertos.c）

- `StartDefaultTask`：心跳 LED 500ms（亮=调度器活着）
- `StartTask02` = UartTXTask（50ms 轮询）：**唯一 UART 发送者**——CSV 6 列 @50ms（iq / tgt / id ×0.01A、θ 0~359°、电周期计数、NTC ×0.1°C）+ 命令解析 + 500ms 巡检
- `StartTask03` = SpeedObserveTask：v6 采角逻辑已停用，任务壳保留（`osDelay(1000)` 占位）——**速度环移植的改造对象**

### 串口命令（USART1 PB14/PB15 115200 8N1，`字母 空格 数值 \n`）

| 命令 | 作用 |
|---|---|
| `q 数值` | 设 iq_ref，0.01A 定标（q 50=0.50A），限幅 ±10A |
| `z` | 重对齐：停环 → v2 闭环双侧逼近 → 复环（CSV 停 ~2.5s 属预期） |
| `c` | 清零电角度周期计数（判 1:1 vs ÷8 跟踪速率） |
| `v 0~359` | 静态矢量探针：2V 开环矢量 + 打印 cnt/iu/iv/iw/αβ 角（先 `q 0`） |
| `t` | GPDMA0 ISR 耗时快照：t/max/avg µs + load% + 样本数 n |
| `d` | id 直流项长窗均值：窗=8 机械圈（恰 7 个 K24 周期，零泄漏），旋转 ~3-5s 出数（静止 8s 超时，trav=0 可辨） |
| `w` | 按需查 vbus（EMA 值）/ NTC 温度 |

## 编码约定（代码层）

- newlib-nano 无 `%f`：打印一律整数定标 + `snprintf` + 阻塞 `HAL_UART_Transmit`（不用 printf 重定向）；`int` 不用 `int32_t`（newlib 上 int32_t=long）
- ISR↔任务共享变量必须 `volatile`；64 位/多字段读用 `__disable_irq()` 快照
- CSV 通道不得混入带数字的文本行（VOFA+ FireWater 会当采样点污染曲线）——诊断信息走 `[xxx]` 前缀文本行，且正常态静默（如 `[drv]`/`[adc-rate]`/`[ntc]` 只在异常时打印）
- 中文注释
