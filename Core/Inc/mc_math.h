#include <stdint.h>

#define M_TWOPI_F   6.28318530718f
#define M_PI_F		3.14159265359f

typedef struct
{
    float I_A;
    float I_B;
    float I_C;
} Iabc_Typedef;

typedef struct
{
    float I_Alpha;
    float I_Beta;
} Iab_Typedef;

typedef struct
{
    float I_Q;
    float I_D;
} Idq_Typedef;

typedef struct
{
    float U_A;
    float U_B;
    float U_C;
} Uabc_Typedef;

typedef struct
{
    float U_Alpha;
    float U_Beta;
} Uab_Typedef;

typedef struct
{
    float U_D;
    float U_Q;
} Udq_Typedef;

typedef struct
{
    float E_Alpha;
    float E_Beta;
} Eab_Typedef;

typedef struct
{
    uint16_t CCR_A;
    uint16_t CCR_B;
    uint16_t CCR_C;
} CCR_Typedef;

/* 角度锁相环（PLL）状态 */
typedef struct
{
    float Pll_Kp;
    float Pll_Ki;
    float Pll_Inte;
    float Pll_Omega;
    float Angle_q_Hat;
} Pll_Typedef;

Iab_Typedef Clark(Iabc_Typedef Iabc);
Eab_Typedef Smo(Uab_Typedef Uab, Iab_Typedef Iab);
Eab_Typedef Lbg(Uab_Typedef Uab, Iab_Typedef Iab);
float Pll(Eab_Typedef Eab, Pll_Typedef *p, float sin_Angle_q, float cos_Angle_q);
void Pll_Reset(Pll_Typedef *p);
Idq_Typedef Park(Iab_Typedef Iab, float sin_Angle_q, float cos_Angle_q);
Uab_Typedef Arc_Park(Udq_Typedef Udq, float sin_Angle_q, float cos_Angle_q);
CCR_Typedef SVPWM(Uab_Typedef Uab, float Vbus);
CCR_Typedef SVPWM_Cps(Uab_Typedef Uab, float Vbus, Iabc_Typedef Iabc);
void Cordic_Sin_Cos(float cordic_angle, float* cordic_sin, float* cordic_cos);
