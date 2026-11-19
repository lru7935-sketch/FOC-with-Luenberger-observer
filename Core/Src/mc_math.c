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
    float Smo_Filter = 0.8f;   /* 反电势低通系数，0.8 -> 约 420 Hz */
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

    float Lbg_w0 = 4000.0f;              /* 观测器带宽，rad/s */

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

    float Pll_Kp = 350.0f;
    float Pll_Ki = 35000.0f;

/* mc_math.c */
float Pll(Eab_Typedef Eab, Pll_Typedef *p, float *Omega)
{
    float emf_abs = sqrtf(Eab.E_Alpha*Eab.E_Alpha + Eab.E_Beta*Eab.E_Beta);
    if (emf_abs < 0.001f) emf_abs = 0.001f;

    const float err = -(Eab.E_Alpha*sin_Angle_q
                        - Eab.E_Beta *cos_Angle_q) / emf_abs;

    p->Pll_Inte  += err / IN_LOOP_FRE;
    p->Pll_Omega  = Pll_Kp*err + Pll_Ki*p->Pll_Inte;
    p->Angle_Q_Hat += p->Pll_Omega / IN_LOOP_FRE;

    if (p->Angle_Q_Hat > M_TWOPI_F) p->Angle_Q_Hat -= M_TWOPI_F;
    if (p->Angle_Q_Hat < 0.0f)    p->Angle_Q_Hat += M_TWOPI_F;

    *Omega = Pll_Ki * p->Pll_Inte;
    return p->Angle_Q_Hat;
}

    extern const uint16_t PWM_ARR;

CCR_Typedef SVPWM(Uab_Typedef Uab, float Vbus)
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

static float Dead_Gain(float i)
{
    const float I0 = 3.0f;   /* 死区：低于这个完全不补（噪声 0.03A 被挡在外面） */

    const float I1 = 4.0f;   /* 渐入结束，之后满补偿 */

    const float a = fabsf(i);

    if (a <= I0) return 0.0f;
    if (a >= I1) return Sgn(i);

    return Sgn(i) * (a - I0) / (I1 - I0);

}

    extern const float DEAD_RATIO;

CCR_Typedef SVPWM_Cps(Uab_Typedef Uab, float Vbus, Iabc_Typedef Iabc)
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
