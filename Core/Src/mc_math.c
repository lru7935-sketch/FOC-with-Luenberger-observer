#include "mc_math.h"
#include "math.h"
#include <stdint.h>

extern volatile float sin_Angle_q;
extern volatile float cos_Angle_q;

Iab_Typedef Clark(Iabc_Typedef Iabc)
{
    Iab_Typedef Iab;

    Iab.I_Alpha = Iabc.I_A * 1.5f;
    Iab.I_Beta  = 0.8660254f * (Iabc.I_B - Iabc.I_C);

    return Iab;
}

Idq_Typedef Park(Iab_Typedef Iab)
{
    Idq_Typedef Idq;

    Idq.I_Q = Iab.I_Alpha * cos_Angle_q + Iab.I_Beta * sin_Angle_q;
    Idq.I_D = Iab.I_Alpha * sin_Angle_q - Iab.I_Beta * cos_Angle_q;

    return Idq;
}

Uab_Typedef Arc_Park(Udq_Typedef Udq)
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

    I_Alpha_Hat += (Uab.U_Alpha - PHASE_R*I_Alpha_Hat - E_Alpha)/PHASE_L/IN_LOOP_FRE + Lbg_K1*(Iab.I_Alpha - I_Alpha_Hat)/IN_LOOP_FRE;
    I_Beta_Hat  += (Uab.U_Beta  - PHASE_R*I_Beta_Hat  - E_Beta )/PHASE_L/IN_LOOP_FRE + Lbg_K1*(Iab.I_Beta  - I_Beta_Hat )/IN_LOOP_FRE;

    E_Alpha += Lbg_K2*(Iab.I_Alpha - I_Alpha_Hat)/IN_LOOP_FRE;
    E_Beta  += Lbg_K2*(Iab.I_Beta  - I_Beta_Hat )/IN_LOOP_FRE;

    Eab_Typedef Eab;
    Eab.E_Alpha=E_Alpha;
    Eab.E_Beta=E_Beta;

    return Eab;

}

/*************PLL锁相环*************/
    float Pll_Kp = 350.0f;
    float Pll_Ki = 35000.0f;

float Pll(Eab_Typedef Eab, Pll_Typedef *p, float *Omega)
{
    float emf_abs = sqrtf(Eab.E_Alpha*Eab.E_Alpha + Eab.E_Beta*Eab.E_Beta);//计算模长，否则PLL参数无法保证全速域观测质量
    if (emf_abs < 0.001f) emf_abs = 0.001f;

    const float err = -(Eab.E_Alpha*sin_Angle_q
                        - Eab.E_Beta *cos_Angle_q) / emf_abs;//归一化误差

    p->Pll_Inte  += err / IN_LOOP_FRE;
    p->Pll_Omega  = Pll_Kp*err + Pll_Ki*p->Pll_Inte;
    p->Angle_Q_Hat += p->Pll_Omega / IN_LOOP_FRE;

    if (p->Angle_Q_Hat > M_TWOPI_F) p->Angle_Q_Hat -= M_TWOPI_F;//归一化
    if (p->Angle_Q_Hat < 0.0f)    p->Angle_Q_Hat += M_TWOPI_F;

    *Omega = Pll_Ki * p->Pll_Inte;//这里返回角度误差的积分项，不要用PLL输出的角速度，抖动太大
    return p->Angle_Q_Hat;
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
