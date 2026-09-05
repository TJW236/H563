/* H563 电流环（自 G431 testing/new_test 7-31 定稿版移植，2026-08-31）
 * 结构与数值逐字照抄；硬件差异改动集中在：
 * - 三相 INA240 单端直采（G431 两相差分+KCL）→ adc_buf 0=Iu 1=Iv 2=Iw
 * - 电角度：G431 走 SPI DMA 影子角 → H563 回调内直读 TIM2 CNT（ABZ 65536 cpr）
 * - DRV 配置走 drv8320s.c 驱动（BringupTest 内完成），不再放在 foc.c
 * - pi_id 试装已回退（09-02）：上板直接失控，当日移除，d 轴维持纯前馈开环（G431 同构）。
 *   id 漂移定性结论不变（新板复现→结构性：e_d=ωe·ψf·sin(Δθ) 泄漏，id=e_d/R 放大）；
 *   重启 pi_id 前需先解决对齐→复环的电流交接（积分预置对齐电压的 bumpless transfer） */
#include "foc.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

FOC_t g_foc;

volatile float shadow_iq_ref = 0.0f;
volatile float shadow_id = 0.0f;
volatile float shadow_iq = 0.0f;
volatile float shadow_uq = 0.0f;
volatile float shadow_du = 0.0f;
volatile float shadow_dv = 0.0f;
volatile float shadow_dw = 0.0f;
volatile uint16_t shadow_theta16 = 0;
volatile uint16_t shadow_cnt = 0;
static volatile int32_t elec_accum = 0;  /* elec16 增量 int16 差分累计（16bit 定点）；
                                          * volatile：ISR 累加 + c 命令任务清零，跨上下文共享 */
volatile int32_t shadow_elec_revs = 0;   /* 电角度走过周期数 = elec_accum/65536 */
extern uint16_t adc_buf[5];              /* main.c：DMA 目标（foc_run=0 仍 30kHz 刷新） */

/* ==================== PI 控制器（照抄 G431） ==================== */

void PI_Init(PI_t *p, float kp, float ki, float max, float dt)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = 0.0f;
    p->max = max;
    p->dt = dt;
    p->integral1 = 0.0f;
    p->integral_limit = max / ki;
}

float PI_CalcG(PI_t *p, float error, int enable_integrate, float kp_eff, float ki_eff)
{
    if (enable_integrate)
    {
        p->integral1 += error * p->dt;
    }

    float out = kp_eff * error + ki_eff * p->integral1;

    /* 输出限幅 */
    float out_clamped = out;
    if (out_clamped > p->max)
        out_clamped = p->max;
    else if (out_clamped < -p->max)
        out_clamped = -p->max;

    /* 反积分饱和（back-calculation） */
    if (out_clamped != out && ki_eff > 0.0f)
        p->integral1 -= (out - out_clamped) / ki_eff;

    return out_clamped;
}

float PI_Calc(PI_t *p, float error, int enable_integrate)
{
    return PI_CalcG(p, error, enable_integrate, p->kp, p->ki);
}

/* ==================== 标准扇区 SVPWM（照抄 G431，PC 数值验证归一化精确） ==================== */

static void svpwm(float d, float q, float phi, float Udc, float max_duty,
                  float *d_u, float *d_v, float *d_w)
{
    if (Udc <= 2.0f)
    {
        *d_u = *d_v = *d_w = 0;
        return;
    }

    /* 电压归一化（基准 = Udc/√3，即 SVPWM 线性区最大相电压峰值） */
    float d_norm = d * SQRT3 / Udc;
    float q_norm = q * SQRT3 / Udc;

    /* 电压矢量限幅 */
    if (d_norm > MAX_NORM_V) d_norm = MAX_NORM_V;
    else if (d_norm < -MAX_NORM_V) d_norm = -MAX_NORM_V;
    if (q_norm > MAX_NORM_V) q_norm = MAX_NORM_V;
    else if (q_norm < -MAX_NORM_V) q_norm = -MAX_NORM_V;

    /* 逆 Park：d/q → α/β */
    float sin_phi = fast_sin(phi);
    float cos_phi = fast_cos(phi);
    float alpha = d_norm * cos_phi - q_norm * sin_phi;
    float beta  = d_norm * sin_phi + q_norm * cos_phi;

    /* 扇区判断 */
    int A = (beta > 0.0f);
    int B = (fabsf(beta) > SQRT3 * fabsf(alpha));
    int C = (alpha > 0.0f);
    int K = 4 * A + 2 * B + C;
    static const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    int sector = K_to_sector[K];

    /* 矢量作用时间 */
    float sector_rad = (float)sector * PI_F / 3.0f;
    float t_m = fast_sin(sector_rad) * alpha - fast_cos(sector_rad) * beta;
    float t_n = beta * fast_cos(sector_rad - PI_F / 3.0f)
              - alpha * fast_sin(sector_rad - PI_F / 3.0f);
    float t_0 = 1.0f - t_m - t_n;

    /* 过调制处理 */
    float t_sum = t_m + t_n;
    if (t_sum > 1.0f)
    {
        t_m /= t_sum;
        t_n /= t_sum;
        t_0 = 0.0f;
    }

    /* 基本矢量表 */
    static const int v[6][3] = {
        {1,0,0}, {1,1,0}, {0,1,0}, {0,1,1}, {0,0,1}, {1,0,1}
    };

    /* 合成占空比 */
    *d_u = t_m * v[sector-1][0] + t_n * v[sector%6][0] + t_0 * 0.5f;
    *d_v = t_m * v[sector-1][1] + t_n * v[sector%6][1] + t_0 * 0.5f;
    *d_w = t_m * v[sector-1][2] + t_n * v[sector%6][2] + t_0 * 0.5f;

    /* 占空比限幅 */
    if (*d_u > max_duty) *d_u = max_duty; else if (*d_u < 0.0f) *d_u = 0.0f;
    if (*d_v > max_duty) *d_v = max_duty; else if (*d_v < 0.0f) *d_v = 0.0f;
    if (*d_w > max_duty) *d_w = max_duty; else if (*d_w < 0.0f) *d_w = 0.0f;
}

/* ==================== FOC 初始化 ==================== */

void FOC_Init(FOC_t *foc)
{
    foc->iq_filtered = 0.0f;
    foc->iq_filter_alpha = 0.5f;
    foc->vbus = VBUS_DEFAULT;
    foc->elec_offset16 = 0;
    foc->prev_elec16 = 0;
    foc->omega_e = 0.0f;
    foc->id = 0.0f;
    foc->iu_raw = foc->iv_raw = foc->iw_raw = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        foc->i_zero[i] = 2048;  /* 缺省中点码，上电零流校准覆盖 */
        foc->iu_avg_buf[i] = 0.0f;
        foc->iv_avg_buf[i] = 0.0f;
        foc->iw_avg_buf[i] = 0.0f;
    }
    foc->iu_avg_sum = 0.0f;
    foc->iv_avg_sum = 0.0f;
    foc->iw_avg_sum = 0.0f;
    foc->avg_idx = 0;

    shadow_iq_ref = 0.0f;   /* 本版写死 0：上电对齐后电机自由，无输出 */

    /* 带宽 09-04 晚矩阵定档 ×2：kp/L = 0.22/403µ → ωc≈546 rad/s≈87Hz。
     * ×0.8~×5 十档矩阵（-O3, q19, 转速 134~440RPM）判据只看 iq：平均误差 |≤0.014A|
     * 十档全过；K24 泄漏比 0.23(×0.8)→0.14→0.08→0.024(×2) 后即平台——×2 为最低
     * 全抑制档，且 440RPM 全程稳（史上最高速）；id 波 0.20~0.27A/100RPM 与带宽
     * 无关（d 轴开环，纯被控对象属性）；ki/kp 保持 ≈R/L 零极点对消 */
    PI_Init(&foc->pi_iq, 0.22f, 228.0f, VBUS_DEFAULT * 0.7f, DT_CURRENT);
}

/* ==================== FOC 电流环（ADC DMA 回调调用，实跑 30kHz=PWM 频率） ==================== */

void FOC_CurrentLoop(FOC_t *foc, uint16_t *adc_buf)
{
    /* 1. 三相 ADC → 电流：adc_buf 0=Iu(PA0) 1=Iv(PA1) 2=Iw(PA2)。
     * 09-01 勘误往返：曾按口头核对改 1=Iw/2=Iv，当天被稳态数据证伪——
     * 静止段电流恰=V/R(1.13V/0.439Ω=2.57A) 且按原序电流矢量∥电压矢量
     * (−44.7° vs −46°)、按换序则⊥90°（静止物理不可能）→ 原序正确，回退 */
    float iu = ((int16_t)adc_buf[0] - foc->i_zero[0]) * CURRENT_SCALE;
    float iv = ((int16_t)adc_buf[1] - foc->i_zero[1]) * CURRENT_SCALE;
    float iw = ((int16_t)adc_buf[2] - foc->i_zero[2]) * CURRENT_SCALE;

    /* 2. 滑动平均 N=3（照抄 G431；三相全采，不再 KCL 补 V 相） */
    uint8_t idx = foc->avg_idx % CURRENT_AVG_SIZE;
    foc->iu_avg_sum -= foc->iu_avg_buf[idx];
    foc->iv_avg_sum -= foc->iv_avg_buf[idx];
    foc->iw_avg_sum -= foc->iw_avg_buf[idx];
    foc->iu_avg_buf[idx] = iu;
    foc->iv_avg_buf[idx] = iv;
    foc->iw_avg_buf[idx] = iw;
    foc->iu_avg_sum += iu;
    foc->iv_avg_sum += iv;
    foc->iw_avg_sum += iw;
    foc->avg_idx++;
    iu = foc->iu_avg_sum / CURRENT_AVG_SIZE;
    iv = foc->iv_avg_sum / CURRENT_AVG_SIZE;
    iw = foc->iw_avg_sum / CURRENT_AVG_SIZE;
    foc->iu_raw = iu;
    foc->iv_raw = iv;
    foc->iw_raw = iw;

    /* 3. 母线电压保护（vbus 暂为常数 → 死保护，保留结构） */
    if (foc->vbus < VBUS_UNDER || foc->vbus > VBUS_OVER)
    {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        return;
    }
    foc->pi_iq.max = foc->vbus * 0.7f;

    /* 4. 电角度：TIM2 CNT（65536 cpr）×21 极对 → 16bit 定点电角度，
     * uint16 乘加回绕天然免疫跨圈（cpr 恰满 16bit 的设计红利） */
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t elec16 = (uint16_t)((int32_t)cnt * (POLE_PAIRS * ENC_DIR)
                                 + (int32_t)foc->elec_offset16);
    float theta = (float)elec16 * (TWO_PI / 65536.0f);

    /* ωe：一拍差分（int16 差分回绕安全；用真实环周期 DT_LOOP。
     * 量化 1 LSB ≈ 1.9 rad/s，对 ud=-ωe·Ls·iq 贡献可忽略 */
    foc->omega_e = (float)(int16_t)(elec16 - foc->prev_elec16)
                   * (TWO_PI / 65536.0f) / DT_LOOP;
    /* 电角度周期计数（09-02 判别实验）：同一差分累计，/65536 = 走过电周期数。
     * 判据：轮转 45°（电机 1 圈）计 21 = 1:1 跟踪；~2.6 = 速率÷8；0 = ABZ 冻结 */
    elec_accum += (int16_t)(elec16 - foc->prev_elec16);
    shadow_elec_revs = elec_accum / 65536;
    foc->prev_elec16 = elec16;

    /* 5. Clarke 变换（三相实测） */
    float ialpha = iu;
    float ibeta = (iu + 2.0f * iv) / SQRT3;

    /* 6. Park 变换 */
    float cos_t = fast_cos(theta);
    float sin_t = fast_sin(theta);
    float id = ialpha * cos_t + ibeta * sin_t;
    float iq = -ialpha * sin_t + ibeta * cos_t;
    foc->id = id;
    shadow_id = id;

    /* 7. iq 低通 α=0.5（照抄 G431） */
    foc->iq_filtered += foc->iq_filter_alpha * (iq - foc->iq_filtered);
    iq = foc->iq_filtered;
    shadow_iq = iq;

    /* 8. d 轴纯前馈解耦 + q 轴 PI（G431 7-31 定稿结构；pi_id 09-02 上板直接失控
     * 已回退，d 轴维持开环。id 漂移=开环下 e_d=ωe·ψf·sin(Δθ) 泄漏经 id=e_d/R 放大，
     * 包络≈1 机械圈，已知悉接受。Ls=403μH 取自 8108 手册；最坏 800RPM×5A → ud≈-3.5V */
    const float Ls = 403e-6f;
    float ud = -foc->omega_e * Ls * iq;
    float uq = PI_Calc(&foc->pi_iq, shadow_iq_ref - iq, 1);
    shadow_uq = uq;

    /* 9. SVPWM + 写 CCR（H563 板实际线序 2026-09-01 核对：CCR1→PA8→W,
     * CCR2→PA9→V, CCR3→PA10→U，与 G431 相比 U/W 互换；与 ZeroAlign 必须一致） */
    float d_u, d_v, d_w;
    svpwm(ud, uq, theta, foc->vbus, 0.9f, &d_u, &d_v, &d_w);
    TIM1->CCR1 = (uint16_t)(d_w * TIMER_ARR);
    TIM1->CCR2 = (uint16_t)(d_v * TIMER_ARR);
    TIM1->CCR3 = (uint16_t)(d_u * TIMER_ARR);
    shadow_du = d_u;
    shadow_dv = d_v;
    shadow_dw = d_w;
    shadow_theta16 = elec16;
    shadow_cnt = cnt;
}

/* ==================== 电角度周期计数清零（c 命令） ==================== */

void FOC_ResetElecRevs(void)
{
    elec_accum = 0;
    shadow_elec_revs = 0;
}

/* ==================== 零位对齐 ==================== */

void FOC_ZeroAlign(FOC_t *foc)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 5V 矢量把 d 轴拉到 θe=0（G431 旧版开环 5V 单次对齐，~11A/1s，真机验证过） */
    float d_u, d_v, d_w;
    svpwm(5.0f, 0.0f, 0.0f, foc->vbus, 0.9f, &d_u, &d_v, &d_w);
    /* 线序映射与电流环一致（CCR1→W, CCR2→V, CCR3→U）：对齐矢量与运行矢量
     * 若走不同映射，零点就定义在反射坐标系里，起环必发散 */
    TIM1->CCR1 = (uint16_t)(d_w * TIMER_ARR);
    TIM1->CCR2 = (uint16_t)(d_v * TIMER_ARR);
    TIM1->CCR3 = (uint16_t)(d_u * TIMER_ARR);

    HAL_Delay(500);

    /* 对齐位置读 CNT → 算电角度零点，使该位置 theta=0 */
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    foc->elec_offset16 = (uint16_t)(-(int32_t)cnt * (POLE_PAIRS * ENC_DIR));
    foc->prev_elec16 = 0;    /* 对齐点 elec16=0，差分起点同步 */
    foc->omega_e = 0.0f;

    char msg[96];
    int mlen = snprintf(msg, sizeof(msg), "[align] cnt=%u elec_off=%u\r\n",
                        cnt, foc->elec_offset16);
    if (mlen > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)mlen, 20);

    HAL_Delay(500);   /* 保持通电稳定（G431 节奏 500+500ms） */
}

/* ==================== 静态矢量探针（v 命令，09-02 判别实验） ==================== */

/* 停环下打一枪开环直流矢量：编码器读转子停位（验输出侧），ADC 均值算 αβ 角（验输入侧）。
 * v 0/90/180/270 依次执行：cnt 每 +90° 应步进 ±780（+跟随=输出正确，−跟随=输出镜像）；
 * ang 跟 +θ=ADC 链正确，跟 −θ=输入侧镜像；某相恒 0=断路。两读数均不过 elec_offset16，
 * 对零点偏置免疫。前置：调用方已置 foc_run=0（CCR 无人覆写）、已 q 0（复环无阶跃）。
 * foc_run=0 期间 ADC DMA 仍 30kHz 刷新（main.c 回调只重挂不写 CCR），采样是活数据 */
void FOC_StaticVectorProbe(float volts, uint16_t elec_deg)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    float theta = (float)elec_deg * (PI_F / 180.0f);
    float d_u, d_v, d_w;
    svpwm(volts, 0.0f, theta, g_foc.vbus, 0.9f, &d_u, &d_v, &d_w);
    /* 线序映射与电流环/对齐一致（CCR1→W, CCR2→V, CCR3→U） */
    TIM1->CCR1 = (uint16_t)(d_w * TIMER_ARR);
    TIM1->CCR2 = (uint16_t)(d_v * TIMER_ARR);
    TIM1->CCR3 = (uint16_t)(d_u * TIMER_ARR);

    HAL_Delay(1200);   /* 转子吸附+欠阻尼摆动衰减 */

    /* ADC 均值 200 帧 ×1ms≈200ms；静态直流量，任务侧快照与 DMA 刷新的撕裂对均值无影响 */
    uint32_t s0 = 0, s1 = 0, s2 = 0;
    const int N = 200;
    for (int i = 0; i < N; i++)
    {
        s0 += adc_buf[0];
        s1 += adc_buf[1];
        s2 += adc_buf[2];
        HAL_Delay(1);
    }
    float iu = ((int32_t)(s0 / N) - g_foc.i_zero[0]) * CURRENT_SCALE;
    float iv = ((int32_t)(s1 / N) - g_foc.i_zero[1]) * CURRENT_SCALE;
    float iw = ((int32_t)(s2 / N) - g_foc.i_zero[2]) * CURRENT_SCALE;
    float ialpha = iu;
    float ibeta = (iu + 2.0f * iv) / SQRT3;
    int ang10 = (int)(atan2f(ibeta, ialpha) * (180.0f / PI_F) * 10.0f);

    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    char msg[80];
    int mlen = snprintf(msg, sizeof(msg),
                        "[v] cmd=%u cnt=%u iu=%d iv=%d iw=%d ang=%d\r\n",
                        elec_deg, cnt,
                        (int)(iu * 100.0f), (int)(iv * 100.0f), (int)(iw * 100.0f),
                        ang10);
    if (mlen > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)mlen, 20);
}
