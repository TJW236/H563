# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> 本文件只覆盖 H563 工程的**构建/烧录命令**与**代码地图**。硬件参数、PI 定档、调试结论、bring-up 进度一律以上级 `../CLAUDE.md`（motor_driver 项目总规则）为权威，两处冲突时以上级为准；进度日志写 `../任务进度.md`。固件版本以 main.c USER CODE 2 开头的横幅字符串为准（现行 step9：位置环 1071Hz pi 54/0 限幅 800 + p 命令 + CSV 10 列，09-10 实现+当晚首烧实测跟踪良好——近目标粘滑极限环定案（速度积分器×静摩擦 ≈0.28N·m，wei1.docx），对策 A 冻结/B 死区/C 接受待拍板；其下速度环 4285Hz=30k/7 + pi 0.034/0.035 + OTP 70°C + s 880；**09-14 重力前馈补丁已写入源码待烧录**（m/l/g 命令+FOC_GravityFF 无状态 sin(θ_out) 前馈，横幅未动仍 step9，grav_on=0 时三环与旧版逐位一致）；step8/step9 正式验收单（t/守卫读数+OTP 两项）未做）。

## 构建

STM32CubeIDE 1.18.1 工程（CubeMX 生成，MCU=STM32H563RIT6，TrustZone=Disabled，FreeRTOS CMSIS-RTOS2，HAL 时基=TIM7）。

- IDE 可执行文件：`/opt/st/stm32cubeide_1.18.1/stm32cubeide`；工作区 `~/STM32CubeIDE/workspace_1.18.1`
- 日常构建走 IDE GUI（Project → Build），产物 `Debug/H563.elf`
- **Debug 配置 = -O3**（2026-09-04 定档上板）；Release 配置 = -Os（未启用，勿拿它烧板对比性能）
- 改 `.ioc` 后在 CubeMX 重新生成 → **Refresh + Clean + Rebuild**（曾因 Debug/ 旧 makefile 缺 Middlewares 路径报 FreeRTOS.h 找不到）
- ⚠ `.ioc` 重生成必须保持 TIM1 Counter Period=**4167**（30kHz）与 **RepetitionCounter=1**——Period 与 `Core/Inc/foc.h` 的 `TIMER_ARR` 强耦合（不同步则占空比超标 1.5 倍）；RCR=1 保证 UPDATE 每 PWM 周期恰一次（RCR=0 时中心对齐上下顶点双发，重挂追得上半周期就实跑 60kHz，09-05 定案）
- ⚠ TIM3 Counter Period=**58337**（4285.4Hz 速度环时基，PSC=0，=8334×7−1 与 TIM1 同节拍粒度→环比恰=电流环 1/7 整数分频；09-08 由 62499/4kHz 迁移）与 `foc.h` 的 `SPEED_TIM_ARR` 强耦合——同 TIMER_ARR 纪律；ADC2 保持**软件触发**（NTC/PWR 任务侧软件单发，无 DMA 无定时器无中断）
- ⚠ CubeMX 大改/挪引脚会抹掉引脚标签宏与上拉参数——重生成后核对 `main.h` 标签 + `gpio.c` 上拉（OSC32 脚 PC14/PC15 踩过）

## 烧录

- CubeIDE Run（SWD + ST-Link），调试配置 `H563.launch`
- 烧后零输出先查 IDE Console 有无 "Download verification failed" 红字：有红字=烧录链路问题（降 SWD 速率/重插 ST-Link），无红字=才 Clean 重建查代码
- `H563_v6_dragcal_20260829.elf`（工程根）= 拖动自校准 v6 完整备份，⌀10 磁铁到货直接重烧重跑，不改源码

## 代码地图

CubeMX 生成文件（adc/spi/tim/usart/gpdma/gpio/icache.c、stm32h5xx_msp/timebase/it.c）只做外设初始化；用户代码全在 `/* USER CODE */` 区内，区外改动重生成即丢。逻辑集中在五个文件：

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 点火链（USER CODE 2）+ ADC/UART/TIM3 回调（ConvCplt 分发电流环、PeriodElapsed 分发速度环、RxCplt 行缓冲） |
| `Core/Src/foc.c` / `Inc/foc.h` | 电流环 PI + Clarke/Park + SVPWM + 零点对齐 + 静态矢量探针 + 速度环（FOC_SpeedLoop：测速+速度 PI+位置 P÷4+里程表）+ 重力前馈（FOC_GravityFF：grav_amp·sin(θ_out)，无状态相位实时取 total_cnt%2^19，09-14 补丁）；foc.h 集中硬件宏（TIMER_ARR / SPEED_TIM_ARR / ENC_DIR / CURRENT_SCALE / KT_NATIVE / GRAV_CNT_PER_OUT_REV / NTC-PWR 换算 / OTP 阈值）+ 1024 点 sin 表 + fast_sin/cos |
| `Core/Src/app_freertos.c` | 三任务 + 串口命令解析 + CSV 流（10 列）+ adc2_read（NTC/PWR 软件单发 + NTC 3 帧中值）+ NTC 跳闸/开路锁扣（50ms 拍）+ 500ms 巡检（DRV FSR / ADC 速率守卫 / vbus EMA / NTC 报警）+ 重力前馈 m/l/g 命令（09-14） |
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
- **速度环链**：TIM3 4285.4Hz=30k/7（PSC=0/ARR=58337，NVIC 6 低于 GPDMA0 的 5=电流环永远可抢占）→ `HAL_TIM_PeriodElapsedCallback`（main.c）→ `FOC_SpeedLoop`（foc.c）：CNT 233.4µs 差分 ×ENC_DIR 测速（1 count=3.92 RPM，一阶滤波 α=0.14≈95Hz）+ 速度 PI → `shadow_iq_ref`（`speed_mode && foc_run` 双门控；`s` 命令 bumpless=速度 PI 积分预置当前 iq_ref）。TIM3 在对齐后才启动——搬转子的差分阶跃不进滤波器
- **位置环链（09-10 实现，同在 FOC_SpeedLoop）**：`total_cnt` 里程表 int32 逐拍累加 int16 差分（回绕免疫；零点=上电播种位置——main.c 播种记 `boot_cnt`、TIM3 启动前 `total_cnt=(int16)(CNT−boot_cnt)` 对齐挪动量入表 + `prev_cnt` 重挂防首拍双计）；位置 P 跑 ÷4 拍（≈1071Hz，kp=54/ki=0 G431 直拷，**误差域=电机轴 rad**，PI_Calc 第三参 0 关积分防 NaN）→ `speed_target`（±800=PI max 限幅）→ 现有速度 PI 级联；`p` 命令输出轴 rad 入口 ×8 + bumpless 同 s；q/s/OTP 跳闸均 `pos_mode=0` 退场
- **重力前馈链（09-14 补丁，同在 FOC_SpeedLoop）**：`FOC_GravityFF()` = grav_amp·sin(θ_out) 叠加速度 PI 出口（局部 iq_cmd + **单次 volatile 写**，防电流环抢占读中间值）；相位**无状态**实时取 total_cnt % 2^19 ×ENC_DIR 规约（2^31=4096×2^19 跨 int32 溢出相位连续）；只在 `speed_mode && foc_run` 域内叠加（q 力矩模式不加、OTP 随 speed_mode=0 自动停）；`grav_on=0`（上电默认）分支不执行=三环与旧版逐位一致；grav_amp 由 `m`/`l` 命令换算 = m·l·9.8/(8×KT_NATIVE)=m·l×9.14；⚠ 相位零点=上电悬底位（开前馈前摆自由悬停上电）

### main.c 点火链（USER CODE 2，调度器启动前，顺序不可换）

横幅 → DWT CYCCNT 使能（LAR 解锁字直写 0xE0001FB0，CMSIS 5.6 无 LAR 成员）→ `DRV8320S_BringupTest()` → `HAL_Delay(200)` 等 MT6835 EEPROM→寄存器导入 → ABZ 影子写 0x007/0x008 → 播种 TIM2 → `FOC_Init` + ADC 校准 + `Start_DMA` + TIM1 起 → 零流校准 1000 均值（覆盖三相共模偏置）→ `FOC_ZeroAlign`（v2 闭环双侧逼近，ALIGN_CURRENT_A 见 foc.h，~2.5s；开环 5V 版保留 foc.c 备用）→ `foc_run=1` → `HAL_TIM_Base_Start_IT(&htim3)`（速度环 4285Hz 时基，放对齐后=搬转子差分阶跃不进滤波器）。顺序关键点：首个 UPDATE 触发到来时 DMA 必须已就位。

### FreeRTOS 三任务（app_freertos.c）

- `StartDefaultTask`：心跳 LED 500ms（亮=调度器活着）
- `StartTask02` = UartTXTask（50ms 轮询）：**唯一 UART 发送者**——CSV 10 列 @50ms（iq / tgt / id ×0.01A、θ 0~359°、电周期计数、NTC ×0.1°C、速度目标/实测 RPM ×1、位置目标/实测 mRad 输出轴）+ 命令解析 + adc2_read（NTC/PWR 软件单发，NTC 3 帧中值）+ NTC 跳闸/开路锁扣 + 500ms 巡检
- `StartTask03` = SpeedObserveTask：v6 采角逻辑已停用，任务壳保留（`osDelay(1000)` 占位）——速度环最终走 TIM3 ISR 实现，本任务不承载控制（位置环/监控可复用）

### 串口命令（USART1 PB14/PB15 115200 8N1，`字母 空格 数值 \n`）

| 命令 | 作用 |
|---|---|
| `q 数值` | 设 iq_ref，0.01A 定标（q 50=0.50A），限幅 ±10A；同时退速度模式回力矩模式 |
| `s 数值` | 速度目标（电机轴 RPM，限 ±880=额定点，09-05 由 300 解禁）→ 进速度模式（位置模式退场）；bumpless=速度 PI 积分预置当前 iq_ref，接管无电流阶跃 |
| `p 数值` | 位置目标（**输出轴 rad**，09-10 加）→ ×8 入电机轴误差域进位置模式：位置 P 1071Hz → speed_target ±800 → 现有速度环；bumpless 同 s；零点=上电时刻位置（里程表，对齐挪动量入表；断电搬动有 45° mod 原理限制——摆好再上电）；q/s 随时退回。09-10 实测：近目标粘滑极限环 ±5 mRad（速度积分器×静摩擦，wei1.docx 定案），对策 A 冻结/B 死区/C 接受在议 |
| `k` / `i` | 速度环 kp / ki（直接浮点值，如 `k 0.034` / `i 0.035`；即时生效，CSV 第 7/8 列看跟踪） |
| `z` | 重对齐：停环 → v2 闭环双侧逼近 → 复环（CSV 停 ~2.5s 属预期） |
| `c` | 清零电角度周期计数（判 1:1 vs ÷8 跟踪速率） |
| `v 0~359` | 静态矢量探针：2V 开环矢量 + 打印 cnt/iu/iv/iw/αβ 角（先 `q 0`） |
| `t` | GPDMA0 ISR 耗时快照：t/max/avg µs + load% + 样本数 n |
| `d` | id 直流项长窗均值：窗=8 机械圈（恰 7 个 K24 周期，零泄漏），旋转 ~3-5s 出数（静止 8s 超时，trav=0 可辨） |
| `w` | 按需查 vbus（EMA 值）/ NTC 温度 |
| `m 数值` | 重力前馈：负载质量 kg（09-14 加，实验台摆锤模式）；与 l 命令共同换算 grav_amp=m·l·9.8/(8×KT_NATIVE)，回应行回显 ff 幅度 |
| `l 数值` | 重力前馈：力臂 m（09-14 加）；m/l 任一更新即重算 ff |
| `g 数值` | 重力前馈开关：g 1 开 / g 0 关（上电默认关）。⚠ 开前确认上电时摆自由悬停底部（相位零点=上电位置），扶摆上电则相位错（纹波变大，g 0 恢复）。只挂 speed_mode 域（s/p 生效、q 不加、OTP 随停）；关闭时前馈分支不执行，三环零影响 |

NTC 跳闸（≥70°C 或开路 <−30°C）期间 q/s/p/z/v 被拒（`[ntc] trip - x blocked`）；纯过温 <60°C 迟滞自动解除（解除后不出力，须重新下发命令）；开路锁扣断电才复位。

## 编码约定（代码层）

- newlib-nano 无 `%f`：打印一律整数定标 + `snprintf` + 阻塞 `HAL_UART_Transmit`（不用 printf 重定向）；`int` 不用 `int32_t`（newlib 上 int32_t=long）
- ISR↔任务共享变量必须 `volatile`；64 位/多字段读用 `__disable_irq()` 快照
- CSV 通道不得混入带数字的文本行（VOFA+ FireWater 会当采样点污染曲线）——诊断信息走 `[xxx]` 前缀文本行，且正常态静默（如 `[drv]`/`[adc-rate]`/`[ntc]` 只在异常时打印）
- 中文注释
