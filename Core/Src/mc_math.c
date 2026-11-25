#include "mc_math.h"
#include "math.h"
#include <stdint.h>
#include "cordic.h"

Iab_Typedef Clark(Iabc_Typedef Iabc)
{
    Iab_Typedef Iab;

    Iab.I_Alpha = Iabc.I_A * 1.5f;
    Iab.I_Beta  = 0.8660254f * (Iabc.I_B - Iabc.I_C);

    return Iab;
}

Idq_Typedef Park(Iab_Typedef Iab, float sin_Angle_q, float cos_Angle_q)
{
    Idq_Typedef Idq;

    Idq.I_Q = Iab.I_Alpha * cos_Angle_q + Iab.I_Beta * sin_Angle_q;
    Idq.I_D = Iab.I_Alpha * sin_Angle_q - Iab.I_Beta * cos_Angle_q;

    return Idq;
}

Uab_Typedef Arc_Park(Udq_Typedef Udq, float sin_Angle_q, float cos_Angle_q)
{
    Uab_Typedef Uab;

    Uab.U_Alpha = Udq.U_Q * cos_Angle_q + Udq.U_D * sin_Angle_q;
    Uab.U_Beta  = Udq.U_Q * sin_Angle_q - Udq.U_D * cos_Angle_q;

    return Uab;
}

static float Sgn(float x){

    if(x>=0){
        return 1.0f;
    }
    else{
        return -1.0f;
    }

}

    float Smo_K = 3.0f;   /* 滑模增益，需大于 |E| 最大值 */
    float Smo_Filter = 0.8f;//选的越大滞后越大，选的越小噪声越大
    //如果追求低速，可以选比较大的值，如0.95；追求高速可以选比较小的值，如0.8。
    //如果需要全速域，可以选择自适应，也就是通过判断反电势大小来调整滤波系数和滑膜增益，但是极易发散，不建议给滑膜增益加自适应

    extern const float PHASE_R;
    extern const float PHASE_L;
    extern const float IN_LOOP_FRE;

Eab_Typedef Smo(Uab_Typedef Uab, Iab_Typedef Iab)
{
    static float I_Alpha_Hat = 0.0f;   
    static float I_Beta_Hat  = 0.0f;
    static float Z_Alpha     = 0.0f;   
    static float Z_Beta      = 0.0f;
    static float E_Alpha     = 0.0f;   
    static float E_Beta      = 0.0f;

    I_Alpha_Hat += (Uab.U_Alpha - I_Alpha_Hat * PHASE_R - Z_Alpha)/ (PHASE_L * IN_LOOP_FRE);
    I_Beta_Hat  += (Uab.U_Beta - I_Beta_Hat * PHASE_R - Z_Beta)/ (PHASE_L * IN_LOOP_FRE);

    Z_Alpha = Smo_K*Sgn(I_Alpha_Hat-Iab.I_Alpha);
    Z_Beta  = Smo_K*Sgn(I_Beta_Hat-Iab.I_Beta);

    E_Alpha = Smo_Filter * E_Alpha + (1.0f - Smo_Filter) * Z_Alpha;
    E_Beta  = Smo_Filter * E_Beta  + (1.0f - Smo_Filter) * Z_Beta;

    Eab_Typedef Eab;
    Eab.E_Alpha=E_Alpha;
    Eab.E_Beta=E_Beta;

    return Eab;

}

    float Lbg_w0 = 4000.0f;              /* 观测器带宽，rad/s，选的越大噪声越大滞后越小，反之亦然；这个参数建议自适应，否则只能保证低速或高速性能*/

Eab_Typedef Lbg(Uab_Typedef Uab, Iab_Typedef Iab){

    static float I_Alpha_Hat = 0.0f;
    static float I_Beta_Hat  = 0.0f;
    static float E_Alpha     = 0.0f;   
    static float E_Beta      = 0.0f;

    const float Lbg_K1 = 2.0f*Lbg_w0 - PHASE_R/PHASE_L;
    const float Lbg_K2 = -PHASE_L * Lbg_w0 * Lbg_w0;

    /* 误差只算一次：先算误差，再同步更新电流和反电势。
       以前是"半隐式"（先更新电流、再用新电流算 E 的误差），
       会让实际极点与设计不符，高速时角度误差偏大。 */
    float err_alpha = Iab.I_Alpha - I_Alpha_Hat;
    float err_beta  = Iab.I_Beta  - I_Beta_Hat;

    float dI_alpha = (Uab.U_Alpha - PHASE_R * I_Alpha_Hat - E_Alpha)
                   / PHASE_L + Lbg_K1 * err_alpha;
    float dI_beta  = (Uab.U_Beta  - PHASE_R * I_Beta_Hat  - E_Beta)
                   / PHASE_L + Lbg_K1 * err_beta;

    float dE_alpha = Lbg_K2 * err_alpha;
    float dE_beta  = Lbg_K2 * err_beta;

    I_Alpha_Hat += dI_alpha / IN_LOOP_FRE;
    I_Beta_Hat  += dI_beta  / IN_LOOP_FRE;

    E_Alpha += dE_alpha / IN_LOOP_FRE;
    E_Beta  += dE_beta  / IN_LOOP_FRE;

    Eab_Typedef Eab;
    Eab.E_Alpha=E_Alpha;
    Eab.E_Beta=E_Beta;

    return Eab;

}

float Pll(Eab_Typedef Eab, Pll_Typedef *p, float sin_Angle_q, float cos_Angle_q)
{
    float emf_abs = sqrtf(Eab.E_Alpha*Eab.E_Alpha + Eab.E_Beta*Eab.E_Beta);//计算模长，否则PLL参数无法保证全速域观测质量
    if (emf_abs < 0.05f) emf_abs = 0.05f;   /* 50mV：低反电势时防止分母过小把噪声放大 */

    float err = -(Eab.E_Alpha*sin_Angle_q - Eab.E_Beta *cos_Angle_q) / emf_abs;//归一化误差

    p->Pll_Inte += err / IN_LOOP_FRE;

    const float Pll_p = p->Pll_Kp * err;
    const float Pll_i = p->Pll_Ki * p->Pll_Inte;

    p->Pll_Omega  = Pll_p + Pll_i;

    p->Angle_q_Hat += p->Pll_Omega / IN_LOOP_FRE;

    if (p->Angle_q_Hat > M_TWOPI_F) p->Angle_q_Hat -= M_TWOPI_F;//归一化
    if (p->Angle_q_Hat < 0.0f)    p->Angle_q_Hat += M_TWOPI_F;

    return Pll_i;//这里返回角度误差的积分项，不要用PLL输出的角速度，抖动太大

}

void Pll_Reset(Pll_Typedef *p){

    p->Pll_Inte    = 0.0f;
    p->Pll_Omega   = 0.0f;
    p->Angle_q_Hat = 0.0f;

}

    extern const uint16_t PWM_ARR;

CCR_Typedef SVPWM(Uab_Typedef Uab, float Vbus)//不带死区补偿的SVPWM，一般用这个即可，下面那个死区补偿需要自行判断要不要用
{

    const float u_alpha = Uab.U_Alpha;
    const float u_beta  = Uab.U_Beta;

    const float u_a = (0.66666667f)*u_alpha;
    const float u_b = (0.66666667f)*(-0.5f * u_alpha + 0.8660254f * u_beta);
    const float u_c = (0.66666667f)*(-0.5f * u_alpha - 0.8660254f * u_beta);

    const float m1 = (u_a > u_b) ? u_a : u_b;
    const float u_max = (m1 > u_c) ? m1 : u_c;
    const float n1 = (u_a < u_b) ? u_a : u_b;
    const float u_min = (n1 < u_c) ? n1 : u_c;
    const float u_off = -0.5f * (u_max + u_min);

    float d_a = 0.5f + (u_a + u_off) / Vbus;
    float d_b = 0.5f + (u_b + u_off) / Vbus;
    float d_c = 0.5f + (u_c + u_off) / Vbus;

    CCR_Typedef CCR;
    CCR.CCR_A=(uint16_t)(d_a * PWM_ARR);
    CCR.CCR_B=(uint16_t)(d_b * PWM_ARR);
    CCR.CCR_C=(uint16_t)(d_c * PWM_ARR);

    return CCR;

}

static float Dead_Gain(float i)//死区补偿
{
    const float I0 = 1.0f;   /* 死区最低阈值电流，低于该值不进行补偿，这两个参数需要自行调整，如果不需要死区补偿，可以忽略此函数*/
    const float I1 = 2.0f;   /* 渐入结束，之后满补偿 */

    const float a = fabsf(i);

    if (a <= I0) return 0.0f;
    if (a >= I1) return Sgn(i);

    return Sgn(i) * (a - I0) / (I1 - I0);

}

    extern const float DEAD_RATIO;

CCR_Typedef SVPWM_Cps(Uab_Typedef Uab, float Vbus, Iabc_Typedef Iabc)
//带死区补偿的SVPWM，如果相电阻比较大，不推荐补偿死区，容易振荡。
//相电阻小的电机可以考虑，但是只建议在高转速下或大负载时，小占空比时插入死区补偿同样容易振荡
{

    const float u_alpha = Uab.U_Alpha;
    const float u_beta  = Uab.U_Beta;

    const float u_a = (0.66666667f)*u_alpha+Vbus*DEAD_RATIO*Dead_Gain(Iabc.I_A);
    const float u_b = (0.66666667f)*(-0.5f * u_alpha + 0.8660254f * u_beta)+Vbus*DEAD_RATIO*Dead_Gain(Iabc.I_B);
    const float u_c = (0.66666667f)*(-0.5f * u_alpha - 0.8660254f * u_beta)+Vbus*DEAD_RATIO*Dead_Gain(Iabc.I_C);

    /* ternary instead of fmaxf/fminf: single VMAX/VMIN instruction, no lib call */
    const float m1 = (u_a > u_b) ? u_a : u_b;
    const float u_max = (m1 > u_c) ? m1 : u_c;
    const float n1 = (u_a < u_b) ? u_a : u_b;
    const float u_min = (n1 < u_c) ? n1 : u_c;
    const float u_off = -0.5f * (u_max + u_min);

    float d_a = 0.5f + (u_a + u_off) / Vbus;
    float d_b = 0.5f + (u_b + u_off) / Vbus;
    float d_c = 0.5f + (u_c + u_off) / Vbus;

    CCR_Typedef CCR;
    CCR.CCR_A=(uint16_t)(d_a * PWM_ARR);
    CCR.CCR_B=(uint16_t)(d_b * PWM_ARR);
    CCR.CCR_C=(uint16_t)(d_c * PWM_ARR);

    return CCR;

}

void Cordic_Sin_Cos(float cordic_angle, float* cordic_sin, float* cordic_cos){

    /* q1.31：角度必须先折到 [-pi, pi)，再除以 pi */
    float a = cordic_angle;
    if (a > 3.14159265f)  a -= 6.28318531f;
    if (a < -3.14159265f) a += 6.28318531f;

    int32_t in  = (int32_t)(a * (2147483648.0f / 3.14159265f));
    int32_t out[2] = {0, 0};

    HAL_CORDIC_CalculateZO(&hcordic, &in, out, 1, 1);   /* ZO = zero overhead */

    *cordic_cos = (float)out[0] * (1.0f / 2147483648.0f);
    *cordic_sin = (float)out[1] * (1.0f / 2147483648.0f);

}
