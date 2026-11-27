#include "mc_ctrl.h"
#include "adc.h"
#include "drv8340s.h"
#include "hrtim.h"
#include "i2c.h"
#include "main.h"
#include "pid.h"
#include "rgb.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_cordic.h"
#include "stm32g4xx_hal_hrtim.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "jf.h"                                    
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/***********************手动配置****************************** */

/*电机本体*/
const float PHASE_R     = 33e-3f;//相电阻
const float PHASE_L     = 2.5e-6f;//相电感
const uint8_t POLE_PAIRS = 7;//极对数

/*ADC采样*/
const float ADC_VREF = 3.3f;//ADC参考电压
const uint32_t ADC_FULL=4095;//ADC量程12位
const float ADC_FRE = 42.5e6;//ADC工作频率，单位hz
const float ADC_CYCLE = 6.5f+12.5f;//ADC转换耗时，单位cycle，前一项是保持时间，后一项是转换时间，如果ADC是16位，改成“+16.5f”

/*电流采样*/
const float SAMPLE_RES = 0.01f;//采样电阻，单位Ω
const float AMP_GAIN = 10.0f;//CSA倍率

/*电压采样*/
const float VBUS_DIV=4.0f;//母线采样分压

/*PWM*/
const float PWM_FRE = 42500.0f;//PWM频率
const uint16_t PWM_ARR = (32000-1);//PWM自动重装值
const uint16_t PWM_CCR_MIN = 40;//PWM最小比较值

/*死区时间*/
const uint16_t DEAD_TIME = 600U;//600ns，实际死区时间（上管完全关断->下管完全打开），不是单片机这边配置的死区时间。

/*控制频率*/
const float IN_LOOP_FRE = 42500.0f;//电流环频率internal_loop_frequency
const uint32_t IN_EX_DIV = 64;//内外环分频internal_external_division

/*开环参数*/
const float OL_Uq = 1.2f;  //开环的q轴电压，按需调整，这里给默认值，必须确保你给的电压能支持下面这个加速度
const float OL_ACCE = 10.0f; //开环加速度open_loop_accelerate，单位rps/s

/*闭环参数*/
const float CL_TH = 20.0f;   //切闭环阈值close_loop_threshood，单位rps
const float CL_ACCE = 100.0f;//闭环最大加速度close_loop_accelerate，如果负载比较大，这个值不是很重要，可以往大调；负载如果比较小，或者电机转矩很大，可以设定该值对加速度做限制


/***********************自动计算*******************************/

const uint32_t PWM_PERIOD = (uint32_t) 1e9/PWM_FRE;//单位ns

const float DEAD_RATIO = (float) DEAD_TIME/PWM_PERIOD;

const float ADC_SAMPLE_RATIO = ADC_CYCLE*PWM_FRE*1.2f/ADC_FRE;//这里×1.2是给采样留一点余量

const uint16_t ADC_Sample_Point_Max = (uint16_t) PWM_ARR*(1.0f-ADC_SAMPLE_RATIO);//采样位置

const float EX_LOOP_FRE = IN_LOOP_FRE/IN_EX_DIV;//外环频率external_loop_frequency

const float Speed_Step = CL_ACCE / EX_LOOP_FRE;//限制最大加速度

/***********************初始化参数，无需配置*******************************/

/***************状态机*****************/

/*
基本过程是Init_Cplt在初始化之后置1，然后允许判断Foc_EN，在用户发不等于0的数之后置1，再判断Zero_Cplt
Zero_Cplt如果是0，不会进入旋转状态。主循环判断Init_Cplt完成Foc_EN=1且为对齐时进行对齐，再把Zero_Cplt置1
最后进入强拖，到达阈值之后切闭环。如果用户发送0，则关闭PWM并清空FOC状态。
*/

volatile uint8_t Init_Cplt = 0;

volatile uint8_t Foc_EN = 0;//foc_enable

volatile uint8_t Zero_Cplt = 0;//Zero_complete
volatile uint8_t OL_Cplt = 0;//open_loop_complete
volatile uint16_t Ex_Cnt=0;//external_count

static float OL_Speed = 0.0f;
static float OL_Angle_q = 0.0f;

static float Speed =0.0f;       static float Omega_E = 0.0f;
static float Angle_q = 0.0f;    static float sin_Angle_q = 0.0f;    static float cos_Angle_q = 0.0f;

static float Ref_Speed = 0.0f;
static volatile float Id_Ref = 0.0f;   /* Id 轴电流给定：USB 发送 "速度,Id" 时第二个数写入这里 */
static float Ref_Ramp = 0.0f;//速度备份,用于斜坡响应参考值
static float Ref_Iq = 0.0f;//q轴参考电流，由速度环给定

    Pll_Typedef Pll_Lbg = {0};

    Iabc_Typedef Foc_Iabc = {0};
    Iab_Typedef  Foc_Iab  = {0};
    Idq_Typedef  Foc_Idq  = {0};
    Udq_Typedef  Foc_Udq  = {0};
    Uab_Typedef  Foc_Uab  = {0};
    Eab_Typedef  Foc_Eab  = {0};

    Pid_Typedef    Iq_Pid;
    Pid_Typedef    Id_Pid;
    Pid_Typedef    Speed_Pid;

    Pid_Typedef    OL_Iq_Pid;
    Pid_Typedef    OL_Id_Pid;

#define ADC_BUF_LEN 1//电流采样ADC缓冲区数组长度
static volatile uint16_t Adc_Buf_A[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_B[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_C[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_Vs[2];//索引0是热敏电阻温度，索引1是母线电压；本代码不处理温度，忽略索引0即可

float Lbg_w0 = 5000.0f;//不用改，下面会自适应

void Foc_Vari_Reset(void){

    Zero_Cplt = 0;
    OL_Cplt = 0;
    Ex_Cnt=0;

    OL_Speed = 0.0f;
    OL_Angle_q = 0.0f;

    Speed =0.0f;       Omega_E = 0.0f;
    Angle_q = 0.0f;    sin_Angle_q = 0.0f;    cos_Angle_q = 0.0f;

    Ref_Speed = 0.0f;
    Ref_Ramp = 0.0f;
    Ref_Iq = 0.0f;

    Lbg_w0=5000.0f;
    Pll_Reset(&Pll_Lbg);

    Foc_Iab.I_Alpha=0.0f;
    Foc_Iab.I_Beta=0.0f;

    Foc_Iabc.I_A=0.0f;
    Foc_Iabc.I_B=0.0f;
    Foc_Iabc.I_C=0.0f;

    Foc_Idq.I_D=0.0f;
    Foc_Idq.I_Q=0.0f;

    Foc_Uab.U_Alpha=0.0f;
    Foc_Uab.U_Beta=0.0f;

    Foc_Udq.U_D=0.0f;
    Foc_Udq.U_Q=0.0f;

    Foc_Eab.E_Alpha=0.0f;
    Foc_Eab.E_Beta=0.0f;

    PID_Reset(&Id_Pid);
    PID_Reset(&Iq_Pid);
    PID_Reset(&Speed_Pid);

}

void MOTOR_Init(void)
{

        if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

        if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

        if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

        if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    PWM_Count_Start();//先打开计数，如果不开的话ADC没法触发（对于STM32G4的HRTIM是这样的）
    //打开的话中断也会同时触发，由于我们还未初始化完成，这里使用Init_Cplt这个变量先屏蔽中断内容

    ADC_Set_Sample_Point(ADC_Sample_Point_Max);
    HAL_ADC_Start_DMA(&hadc4, (uint32_t *)Adc_Buf_Vs, 2);//先把母线电压采集打开，下面初始化代码中如果发现母线欠压就不会继续运行

    HAL_Delay(20U);

    DRV_Init(DRV_MODE_PWM6);
    DRV_Set_Idrivep(DRV_IDRIVEP_60MA);//栅极驱动电流，在mos不振铃的前提下大驱动电流能够缩短死区时间

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc_Buf_B, ADC_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)Adc_Buf_A, ADC_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)Adc_Buf_C, ADC_BUF_LEN);

    Foc_PID_Init();//注意，这里面绝大多数参数会自己运算，除了速度环，因此建议自己调试速度环
    Init_Cplt = 1;
}

void Foc_PID_Init(void){

    //内环PI参数自适应，零极点对消
    const float IN_LOOP_Td     = 1.5f*1.0f/IN_LOOP_FRE;//延迟等于1.5个控制周期

    const float Current_Loop_OmegaBW = M_TWOPI_F * PWM_FRE / 25.0f;

    const float OmegaBW_mlp_Td = Current_Loop_OmegaBW*IN_LOOP_Td;

    float sin_OmegaBW_mlp_Td,cos_OmegaBW_mlp_Td;

    Cordic_Sin_Cos(OmegaBW_mlp_Td, &sin_OmegaBW_mlp_Td, &cos_OmegaBW_mlp_Td);

    const float IN_LOOP_OmegaC = sqrtf(((Current_Loop_OmegaBW - sin_OmegaBW_mlp_Td)*(Current_Loop_OmegaBW - sin_OmegaBW_mlp_Td))
    /(2.0f-cos_OmegaBW_mlp_Td*cos_OmegaBW_mlp_Td));

    //以下PI可以根据对消条件和带宽计算得到
    const float IN_LOOP_kp     = PHASE_L*IN_LOOP_OmegaC;
    const float IN_LOOP_ki     = PHASE_R*IN_LOOP_OmegaC;
    const float IN_LOOP_dt     = 1.0f/IN_LOOP_FRE;

    //SVPWM有效矢量最大为二分之根号三的母线，但为保证采样质量，实际有效比例再×0.9（按硬件调整：噪声大或采样窗口不够就改小，留出消隐时间）
    const float IN_LOOP_OutAbs = Get_Vbus()*sqrtf(3)/2.0f;
    const float Valid_Ratio    = (1.0f-DEAD_RATIO-ADC_SAMPLE_RATIO)*0.85f;
 
    const float PLL_OMEGAn = Lbg_w0/5.0f;
    Pll_Lbg.Pll_Ki = PLL_OMEGAn*PLL_OMEGAn;//Ki = ωn²
    Pll_Lbg.Pll_Kp = 2.0f*PLL_OMEGAn*0.95f;//Kp = 2ζωn，ζ=0.95

    PID_Init(&Iq_Pid, IN_LOOP_kp, IN_LOOP_ki, 0.0f, IN_LOOP_dt, Valid_Ratio*IN_LOOP_OutAbs, -Valid_Ratio*IN_LOOP_OutAbs);
    PID_Init(&Id_Pid, IN_LOOP_kp, IN_LOOP_ki, 0.0f, IN_LOOP_dt, Valid_Ratio*IN_LOOP_OutAbs, -Valid_Ratio*IN_LOOP_OutAbs);

    //注意，目前版本速度环参数需要自己调，下面给出一种调试方法
    /*
    首先调kp；ki，kd设置为0；从小往大调，直至Iq达到能用的范围。当然也可以直接算一下，比如速度相差10rps，期望拿到0.5A电流，可以直接计算出kp
    ki也是从小往大调，直至看到出现积分效果，也就是误差逐渐缩小。
    注意，以上只是粗调，目的是先拿到一个基本可用的控制器，性能暂时无所谓
    然后上位机发送参考速度进行扫频，通过分析闭环系统在跟踪不同频率参考信号下的响应即可反推出系统开环传递函数
    得到开环传递函数以后可以进行仿真或者采用简单的零极点对消即可拿到速度环精确的kp，ki参数
    */
    PID_Init(&Speed_Pid, 0.2f, 0.5f, 0.0f, 1.0f / EX_LOOP_FRE, 15.0f, -15.0f);

}

void MOTOR_Start_Task(void){

    if (Init_Cplt && Foc_EN && !Zero_Cplt) {
        PWM_Start_ALL();        /* 开 6 路输出 */
        PWM_Count_Start();      /* 启动计数器 + MREP 中断（否则 ISR 不跑、PWM 也不出波）*/
        OL_Init();              /* 200ms 对齐，此时 PWM 已在输出 */
        Zero_Cplt = 1;          /* 对齐完成后 ISR 才开始跑 Foc() */
    }

}

/********************************中断**********************************/

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_idx)
{
    if (timer_idx == HRTIM_TIMERINDEX_MASTER)
    {
        if(Init_Cplt){

            if(Foc_EN){

                if(Zero_Cplt){
                    Foc();
                }

            }
            else{
            
            PWM_Stop_ALL();
            Foc_Vari_Reset();

            }

        }
    }
}

/***********************************下面是故障和刹车处理，如果没有或者不需要，直接删掉即可*************************************/
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == DRV_FAULT_PIN)//故障中断：置故障标志并停机
    {
        DRV_Fault_Flag = 1U;
        Foc_EN=0;
    }
    if (gpio_pin == BOT_BREAK_Pin)//用户手动刹车按键，如果没有，删掉这部分回调即可
    {
        Foc_EN=0;
    }
}

/*********************************开环***********************************/
void OL_Init(void){

    Foc_Uab.U_Beta=-2.0f;//电压矢量固定在270°：对齐后转子d轴在270°，q轴落在A相（0°）
    Foc_Uab.U_Alpha=0;

    CCR_Typedef CCR=SVPWM(Foc_Uab, Get_Vbus());

    PWM_CCR_ALL(CCR);
    HAL_Delay(200);

}

void OL_Drv(void)
{
    OL_Speed+=OL_ACCE/IN_LOOP_FRE;

    OL_Angle_q+=OL_Speed*M_TWOPI_F*POLE_PAIRS/IN_LOOP_FRE;

    if(OL_Angle_q>M_TWOPI_F)OL_Angle_q-=M_TWOPI_F;//归一化
    if(OL_Angle_q<0)OL_Angle_q+=M_TWOPI_F;

        Foc_Udq.U_D = 0.0f;
        Foc_Udq.U_Q = OL_Uq;

    float sin_OL_Angle_q = 0.0f;
    float cos_OL_Angle_q = 0.0f;
    Cordic_Sin_Cos(OL_Angle_q, &sin_OL_Angle_q, &cos_OL_Angle_q);

    Foc_Uab=Arc_Park(Foc_Udq, sin_OL_Angle_q, cos_OL_Angle_q);

}

/*********************************闭环***********************************/

void Foc(void){

    Foc_In_Loop();//执行电流环


    JF_Push(Foc_Iabc.I_A, Foc_Iabc.I_B, Foc_Iabc.I_C,
            Speed, Foc_Udq.U_D, Foc_Udq.U_Q);//JustFloat 遥测：Ia/Ib/Ic/Speed/Id/Iq，经 USB CDC 上传

    if(++Ex_Cnt<IN_EX_DIV)
    {
        return;
    }
    else{
        Foc_Ex_Loop();
        Ex_Cnt=0;
    }

}

void Foc_In_Loop(void)
{

    Foc_Iabc.I_A = Get_Ia();
    Foc_Iabc.I_B = Get_Ib();
    Foc_Iabc.I_C = Get_Ic();

    Foc_Iab = Clark(Foc_Iabc);

    Foc_Eab = Lbg(Foc_Uab, Foc_Iab);

    Omega_E = Pll(Foc_Eab, &Pll_Lbg, sin_Angle_q, cos_Angle_q);    Speed = Omega_E/(M_TWOPI_F*POLE_PAIRS);

    Angle_q = Pll_Lbg.Angle_q_Hat;

    Cordic_Sin_Cos(Angle_q, &sin_Angle_q, &cos_Angle_q);

    Lbg_Para_Update();      /* 每拍更新观测器带宽与 PLL 增益：开环期间也跟随转速，避免切闭环时参数突变 */

    Foc_Idq = Park(Foc_Iab, sin_Angle_q, cos_Angle_q);   /* 开环期间也算，切换时 Iq 才是最新值 */

    if(OL_Cplt)
    {
        Foc_Udq.U_D = PID_Update(&Id_Pid, Id_Ref, Foc_Idq.I_D);
        Foc_Udq.U_Q = PID_Update(&Iq_Pid, Ref_Iq, Foc_Idq.I_Q);

        float Angle_q_Cps = Angle_q + Omega_E*1.5f/IN_LOOP_FRE;
        float sin_Angle_q_Cps =0.0f;
        float cos_Angle_q_Cps =0.0f;

        if(Angle_q_Cps>M_TWOPI_F)Angle_q_Cps-=M_TWOPI_F;//归一化
        if(Angle_q_Cps<0)Angle_q_Cps+=M_TWOPI_F;

        Cordic_Sin_Cos(Angle_q_Cps, &sin_Angle_q_Cps, &cos_Angle_q_Cps);

        Foc_Uab=Arc_Park(Foc_Udq, sin_Angle_q_Cps, cos_Angle_q_Cps);
    }
    else{
        OL_Drv();

        if(Speed>=CL_TH)//PLL 给出的速度超过阈值，切到闭环
        {
            OL_Cplt = 1;
            Ref_Ramp = Speed;
            Ref_Iq = Foc_Idq.I_Q;
            Speed_Pid.Inte = Speed * Speed_Pid.Kp /Speed_Pid.Ki;
        }

    }
    CCR_Typedef CCR=SVPWM(Foc_Uab, Get_Vbus());

    uint16_t m1 = (CCR.CCR_A > CCR.CCR_B) ? CCR.CCR_A : CCR.CCR_B;
    uint16_t CCR_Max = (m1 > CCR.CCR_C) ? m1 : CCR.CCR_C;
    uint32_t ADC_Sample_Point = (uint32_t) (CCR_Max + PWM_ARR) / 2;//取最大占空比对应下管导通窗口的中点
    
    ADC_Set_Sample_Point((uint16_t) ADC_Sample_Point);
    
    PWM_CCR_ALL(CCR);

}

void Foc_Ex_Loop(void)
{
    if(OL_Cplt){

        if (Ref_Ramp < Ref_Speed)      Ref_Ramp = fminf(Ref_Ramp + Speed_Step, Ref_Speed);
        else if (Ref_Ramp > Ref_Speed) Ref_Ramp = fmaxf(Ref_Ramp - Speed_Step, Ref_Speed);

        Ref_Iq = PID_Update(&Speed_Pid, Ref_Ramp, Speed);

    }
}

/*****************参数自适应*******************/
void Lbg_Para_Update(void){

    Lbg_w0=10.0f*Omega_E;
    if(Lbg_w0<1000.0f)Lbg_w0=1000.0f;
    if(Lbg_w0>80000.0f)Lbg_w0=80000.0f;

}

/****************************************以下为底层代码，对于不同的硬件，需要自己重写以下函数的内容****************************************/

float Get_Vbus(void)
{
    return ADC_VREF * VBUS_DIV * (float)ADC_Get_Vbus() / ADC_FULL;
}

float Get_Ia(void)
{
    const int16_t adc_a = (int16_t)ADC_Get_Ia();
    const int16_t adc_b = (int16_t)ADC_Get_Ib();
    const int16_t adc_c = (int16_t)ADC_Get_Ic();

    return -(((2.0f * adc_a - adc_b - adc_c) / 3.0f) * ADC_VREF
             / (ADC_FULL * SAMPLE_RES * AMP_GAIN) * GAIN_IA
             - OFFSET_IA);
}

float Get_Ib(void)
{
    const int16_t adc_a = (int16_t)ADC_Get_Ia();
    const int16_t adc_b = (int16_t)ADC_Get_Ib();
    const int16_t adc_c = (int16_t)ADC_Get_Ic();

    return -(((2.0f * adc_b - adc_a - adc_c) / 3.0f) * ADC_VREF
             / (ADC_FULL * SAMPLE_RES * AMP_GAIN) * GAIN_IB
             - OFFSET_IB);
}

float Get_Ic(void)
{
    const int16_t adc_a = (int16_t)ADC_Get_Ia();
    const int16_t adc_b = (int16_t)ADC_Get_Ib();
    const int16_t adc_c = (int16_t)ADC_Get_Ic();

    return -(((2.0f * adc_c - adc_a - adc_b) / 3.0f) * ADC_VREF
             / (ADC_FULL * SAMPLE_RES * AMP_GAIN) * GAIN_IC
             - OFFSET_IC);
}

uint16_t ADC_Get_Ia(void)
{
    uint16_t sum = 0U;

    for (uint8_t i = 0U; i < ADC_BUF_LEN; i++)
    {
        sum += Adc_Buf_A[i];
    }

    return sum / ADC_BUF_LEN;
}

uint16_t ADC_Get_Ib(void)
{
    uint16_t sum = 0U;

    for (uint8_t i = 0U; i < ADC_BUF_LEN; i++)
    {
        sum += Adc_Buf_B[i];
    }

    return sum / ADC_BUF_LEN;
}

uint16_t ADC_Get_Ic(void)
{
    uint16_t sum = 0U;

    for (uint8_t i = 0U; i < ADC_BUF_LEN; i++)
    {
        sum += Adc_Buf_C[i];
    }

    return sum / ADC_BUF_LEN;
}

uint16_t ADC_Get_Vbus(void)
{
    return Adc_Buf_Vs[1];
}

uint16_t ADC_Get_Tem(void)
{
    return Adc_Buf_Vs[0];
}

void ADC_Set_Sample_Point(uint16_t samp)
{
    if (samp > ADC_Sample_Point_Max)
    {
        samp = ADC_Sample_Point_Max;
    }

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, samp);

}

void PWM_Count_Start(void){

    HAL_HRTIM_WaveformCountStart_IT(
        &hhrtim1,
        HRTIM_TIMERID_MASTER
            | HRTIM_TIMERID_TIMER_A
            | HRTIM_TIMERID_TIMER_E
            | HRTIM_TIMERID_TIMER_F
    );

}

void PWM_Start_A(void)
{
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
}

void PWM_Stop_A(void)
{
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
}

void PWM_Start_B(void)
{
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

void PWM_Stop_B(void)
{
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
}

void PWM_Start_C(void)
{
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
}

void PWM_Stop_C(void)
{
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_F);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
}

void PWM_CCR_A(uint16_t comp)
{
    if (comp > PWM_ARR)
    {
        comp = PWM_ARR;
    }

    if (comp < PWM_CCR_MIN)
    {
        comp = PWM_CCR_MIN;
    }
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, comp);
}

void PWM_CCR_B(uint16_t comp)
{
    if (comp > PWM_ARR)
    {
        comp = PWM_ARR;
    }

    if (comp < PWM_CCR_MIN)
    {
        comp = PWM_CCR_MIN;
    }
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, comp);
}

void PWM_CCR_C(uint16_t comp)
{
    if (comp > PWM_ARR)
    {
        comp = PWM_ARR;
    }

    if (comp < PWM_CCR_MIN)
    {
        comp = PWM_CCR_MIN;
    }
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, comp);
}

void PWM_CCR_ALL(CCR_Typedef CCR)
{

    PWM_CCR_A(CCR.CCR_A);
    PWM_CCR_B(CCR.CCR_B);
    PWM_CCR_C(CCR.CCR_C);
    
}

void PWM_Start_ALL(void)
{
    HAL_HRTIM_WaveformOutputStart(
        &hhrtim1,
        HRTIM_OUTPUT_TA1
            | HRTIM_OUTPUT_TA2
            | HRTIM_OUTPUT_TE1
            | HRTIM_OUTPUT_TE2
            | HRTIM_OUTPUT_TF1
            | HRTIM_OUTPUT_TF2
    );
}

void PWM_Stop_ALL(void)
{

    const CCR_Typedef CCR_Stop = {0};
    PWM_CCR_ALL(CCR_Stop);

    HAL_HRTIM_WaveformOutputStop(
        &hhrtim1,
        HRTIM_OUTPUT_TA1
            | HRTIM_OUTPUT_TA2
            | HRTIM_OUTPUT_TE1
            | HRTIM_OUTPUT_TE2
            | HRTIM_OUTPUT_TF1
            | HRTIM_OUTPUT_TF2
    );

    HAL_HRTIM_WaveformCountStop(
        &hhrtim1,
        HRTIM_TIMERID_MASTER
            | HRTIM_TIMERID_TIMER_A
            | HRTIM_TIMERID_TIMER_E
            | HRTIM_TIMERID_TIMER_F
    );

}

/*****************************以下为 USB CDC 命令接收，如果不需要或改用其他信道，删除或自行重写这部分*****************************/
static char     Usb_Line[24];
static uint8_t  Usb_Len = 0U;
static uint32_t Usb_Tick = 0U;

static void FOC_Usb_Parse(void)
{
    char *end;

    Usb_Line[Usb_Len] = 0;

    /* 示波器测试：USB 发 "t<百分比>"（如 t2、t10、t0）。
       A、B 相上管按给定占空比输出，C 相占空比给 0（会被 PWM_CCR_MIN 钳到最小）。
       Init_Cplt=0 后控制中断不再改占空比，只能复位或重烧退出该模式。 */
    if (Usb_Line[0] == 't' || Usb_Line[0] == 'T')
    {
        const float d_pct = strtof(&Usb_Line[1], &end);
        CCR_Typedef cr;
        const uint32_t ccr = (uint32_t)((float)PWM_ARR * (d_pct / 100.0f));
        Init_Cplt = 0;                  /* ISR does nothing now */
        Foc_EN    = 0;
        PWM_Start_ALL();
        PWM_Count_Start();
        cr.CCR_A = (uint16_t)ccr;
        cr.CCR_B = (uint16_t)ccr;
        cr.CCR_C = 0;
        PWM_CCR_ALL(cr);
        return;
    }
    Ref_Speed = strtof(Usb_Line, &end);//第一个数是速度给定（rps）
    if (*end == ',' || *end == ';' || *end == ' ') { Id_Ref = strtof(end + 1, &end); }//第二个数是 Id 给定

    if(Ref_Speed==0){
        Foc_EN = 0;//0 表示停机，中断里会关PWM并清空FOC状态
    }
    else{
        Foc_EN = 1;
        if(Ref_Speed<CL_TH){
            Ref_Speed = CL_TH;
        }
    }

}

void FOC_Usb_Receive(uint8_t *buf, uint32_t len)
{
    uint32_t i;

    Usb_Tick = HAL_GetTick();

    for (i = 0U; i < len; i++)
    {
        const char ch = (char)buf[i];

        if ((ch == '\r') || (ch == '\n'))
        {
            if (Usb_Len != 0U)
            {
                FOC_Usb_Parse();
            }

            Usb_Len = 0U;
        }
        else if (Usb_Len < (sizeof(Usb_Line) - 1U))
        {
            Usb_Line[Usb_Len++] = ch;
        }
    }
}

void FOC_Usb_Task(void)
{
    if ((Usb_Len != 0U) && ((HAL_GetTick() - Usb_Tick) >= 400U))
    {
        FOC_Usb_Parse();
        Usb_Len = 0U;
    }
}

