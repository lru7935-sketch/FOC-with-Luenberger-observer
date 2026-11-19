#include "mc_ctrl.h"

#include "adc.h"
#include "drv8340s.h"
#include "hrtim.h"
#include "i2c.h"
#include "main.h"
#include "pid.h"
#include "rgb.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_hrtim.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "jf.h"                                    
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/***********************手动配置****************************** */
const uint16_t DEAD_TIME = 700U;//700ns，注意这里指的是实际死区时间（上管完全关断->下管完全打开），不是单片机这边配置的死区。

const float PHASE_R     = 2.55f;//相电阻
const float PHASE_L     = 0.0009f;//相电感

const float SAMPLE_RES = 0.1f;//采样电阻，单位Ω
const float AMP_GAIN = 10.0f;//CSA倍率
const float VBUS_DIV=4.0f;//母线采样分压

const float ADC_VREF = 3.3f;//ADC参考电压
#define ADC_BUF_LEN 1//电流采样ADC缓冲区数组长度
const uint32_t ADC_FULL=4095;//ADC量程12位
const float ADC_FRE = 42.5e6;//ADC工作频率，单位hz
const float ADC_CYCLE = 6.5f+12.5f;//ADC转换耗时，单位cycle，前一项是保持时间，后一项是转换时间，如果ADC是16位，改成“+16.5f”

const float PWM_FRE = 21250.0f;//PWM频率
const uint32_t PWM_ARR = (64000-1);//PWM自动重装值
const uint16_t PWM_CCR_MIN = 40;//PWM最小比较值

const float IN_LOOP_FRE = 21250.0f;//电流环频率
const uint32_t IN_EX_DIV = 32;//内外环分频

const float CL_TH = 1.0f;//切闭环阈值，单位rps
const float CL_ACCE = 60.0f;//闭环最大加速度限制，如果负载比较大，这个值不是很重要，可以往大调

/***********************自动计算****************************** */
const uint32_t PWM_PERIOD = (uint32_t) 1e9/PWM_FRE;//单位ns
const float DEAD_RATIO = (float) DEAD_TIME/PWM_PERIOD;
const float ADC_SAMPLE_RATIO = ADC_CYCLE*PWM_FRE*1.2f/ADC_FRE;//这里×1.2是给采样留一点余量
const uint32_t ADC_Sample_Point = (uint32_t) PWM_ARR*(1.0f-ADC_SAMPLE_RATIO);
const float EX_LOOP_FRE = IN_LOOP_FRE/IN_EX_DIV;

const float OL_ACCE = 0.5f*CL_TH;
const float OL_Uq = 1.0f+0.5f*OL_ACCE;

/***********************无需配置****************************** */
float Angle_q = 0.0f;
float Omega_E = 0.0f;

volatile float Ref_Speed = 0.0f;
volatile float Ref_Iq = 0.0f;

static volatile uint8_t OL_Cplt = 0;
static volatile uint8_t Foc_EN = 0;

static volatile uint16_t Adc_Buf_A[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_B[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_C[ADC_BUF_LEN];
static volatile uint16_t Adc_Buf_Vs[2];     

    Iabc_Typedef Foc_Iabc = {0};
    Iab_Typedef  Foc_Iab  = {0};
    Idq_Typedef  Foc_Idq  = {0};
    Udq_Typedef  Foc_Udq  = {0};
    Uab_Typedef  Foc_Uab  = {0};
    Eab_Typedef  Foc_Eab  = {0};

    Pid_Typedef    Iq_Pid;
    Pid_Typedef    Id_Pid;
    Pid_Typedef    Speed_Pid;

extern float Pll_Kp;
extern float Pll_Ki;
extern float Smo_K;
extern float Smo_Filter;
extern float Lbg_w0;

volatile float sin_Angle_q = 0.0f;
volatile float cos_Angle_q = 0.0f;

void MOTOR_Init(void)
{

    PWM_Count_Start();

    HAL_ADC_Start_DMA(&hadc4, (uint32_t *)Adc_Buf_Vs, 2);

    HAL_Delay(20U);

    DRV_Init(DRV_MODE_PWM6);
    DRV_Set_Idrivep(DRV_IDRIVEP_200MA);

    ADC_Set_Sample_Point(ADC_Sample_Point);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc_Buf_B, ADC_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)Adc_Buf_A, ADC_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)Adc_Buf_C, ADC_BUF_LEN);

    PWM_Start_ALL();

    Foc_PID_Init();

    OL_Init();

    Foc_EN=1;
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t timer_idx)
{
    if (timer_idx == HRTIM_TIMERINDEX_MASTER)
    {
        if(Foc_EN){
            Foc();
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == DRV_FAULT_PIN)
    {
        DRV_Fault_Flag = 1U;
        HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);
    }
    else if (gpio_pin == BOT_RECORD_Pin)
    {
        HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    }
}

void OL_Init(void){

    Foc_Uab.U_Beta=-1.0f;
    Foc_Uab.U_Alpha=0;

    CCR_Typedef CCR=SVPWM(Foc_Uab, Get_Vbus());

    PWM_CCR_ALL(CCR);
    HAL_Delay(500);

}

void OL_Drv(void)
{
    static float OL_Angle=0.0f;
    static float OL_Speed=0.0f;

    OL_Speed+=OL_ACCE/IN_LOOP_FRE;

    OL_Angle+=OL_Speed*M_TWOPI_F*POLE_PAIRS/IN_LOOP_FRE;

    if(OL_Angle>M_TWOPI_F)OL_Angle-=M_TWOPI_F;
    if(OL_Angle<0)OL_Angle+=M_TWOPI_F;

    Foc_Udq.U_D=0.0f;
    Foc_Udq.U_Q=OL_Uq;

    Foc_Uab=Arc_Park(Foc_Udq);

    if(Omega_E/(M_TWOPI_F*POLE_PAIRS)>=CL_TH){
        OL_Cplt=1;
        Ref_Iq = Foc_Idq.I_Q;
    }

}

    static Eab_Typedef Lbg_Eab={0};
    static float Lbg_Omega_E=0;
    static float Lbg_Angle_q=0;

void Lbg_Para_Update(void){

    Lbg_w0=20.0f*Omega_E;
    if(Lbg_w0<1000.0f)Lbg_w0=1000.0f;
    if(Lbg_w0>8000.0f)Lbg_w0=8000.0f;

}

void Foc_PID_Init(void){

    const float IN_LOOP_Td     = 1.5f*1.0f/IN_LOOP_FRE;//延迟等于1.5个控制周期
    const float IN_LOOP_OMEGAc = M_PI/(IN_LOOP_Td*6.0f);//60°相位裕度
    const float IN_LOOP_kp     = PHASE_L*IN_LOOP_OMEGAc;
    const float IN_LOOP_ki     = PHASE_R*IN_LOOP_OMEGAc;
    const float IN_LOOP_dt     = 1.0f/IN_LOOP_FRE;
    const float IN_LOOP_OutAbs = Get_Vbus()*sqrtf(3)/2.0f;
    const float Valid_Ratio    = (1.0f-DEAD_RATIO-ADC_SAMPLE_RATIO)*0.9f;

    const float PLL_OMEGAn = IN_LOOP_OMEGAc/15.0f;
    Pll_Ki = PLL_OMEGAn*PLL_OMEGAn;
    Pll_Kp = 2.0f*PLL_OMEGAn*0.95f;

    PID_Init(&Iq_Pid, IN_LOOP_kp, IN_LOOP_ki, 0.0f, IN_LOOP_dt, Valid_Ratio*IN_LOOP_OutAbs, -Valid_Ratio*IN_LOOP_OutAbs);
    PID_Init(&Id_Pid, IN_LOOP_kp, IN_LOOP_ki, 0.0f, IN_LOOP_dt, Valid_Ratio*IN_LOOP_OutAbs, -Valid_Ratio*IN_LOOP_OutAbs);
    PID_Init(&Speed_Pid, 0.05f, 1.0f, 0.0f, 1.0f / EX_LOOP_FRE, 1.2f, -1.2f);

}

void Foc(void){

    static uint16_t ex_cnt=0;

    Foc_In_Loop();

    JF_Push(Foc_Iabc.I_A, Foc_Iabc.I_B, Foc_Iabc.I_C,
            Omega_E/(M_TWOPI_F*POLE_PAIRS), Lbg_Angle_q,Ref_Speed);

    if(++ex_cnt<IN_EX_DIV){
        return;
    }
    else{
        Foc_Ex_Loop();
        ex_cnt=0;
    }

}

void Foc_In_Loop(void)
{

    Foc_Iabc.I_A = Get_Ia();
    Foc_Iabc.I_B = Get_Ib();
    Foc_Iabc.I_C = Get_Ic();

    Foc_Iab=Clark(Foc_Iabc);

    Lbg_Eab=Lbg(Foc_Uab, Foc_Iab);
    
    static Pll_Typedef Pll_Lbg = {0};

    Lbg_Angle_q = Pll(Lbg_Eab, &Pll_Lbg, &Lbg_Omega_E);

    Angle_q=Lbg_Angle_q;

    sin_Angle_q = sinf(Angle_q);
    cos_Angle_q = cosf(Angle_q);

    Omega_E=Lbg_Omega_E;
    Foc_Eab=Lbg_Eab;

    Lbg_Para_Update();

    Foc_Idq=Park(Foc_Iab);

    if(OL_Cplt){
        Foc_Udq.U_D=PID_Update(&Id_Pid, 0, Foc_Idq.I_D);
        Foc_Udq.U_Q=PID_Update(&Iq_Pid, Ref_Iq, Foc_Idq.I_Q);
        Foc_Uab=Arc_Park(Foc_Udq);
    }
    else{
        OL_Drv();
    }

    CCR_Typedef CCR=SVPWM(Foc_Uab, Get_Vbus());

    PWM_CCR_ALL(CCR);

}

void Foc_Ex_Loop(void)
{
    if(OL_Cplt){

        static float Ref_Ramp = 0.0f;//速度备份,用于斜坡响应参考值
        static uint8_t Sw_Init = 0U;
        const float speed_fbk = Omega_E/(M_TWOPI_F*POLE_PAIRS);

        if(Ref_Speed<CL_TH)Ref_Speed=CL_TH;

        if(Sw_Init==0U){
            Sw_Init  = 1U;
            Ref_Ramp = speed_fbk;

            PID_Reset(&Speed_Pid);
            Speed_Pid.Inte = Ref_Iq / Speed_Pid.Ki;
        }//初始化时直接承接参考速度

        const float step = CL_ACCE / EX_LOOP_FRE;//限制最大加速度

        if (Ref_Ramp < Ref_Speed)      Ref_Ramp = fminf(Ref_Ramp + step, Ref_Speed);
        else if (Ref_Ramp > Ref_Speed) Ref_Ramp = fmaxf(Ref_Ramp - step, Ref_Speed);

        Ref_Iq = PID_Update(&Speed_Pid, Ref_Ramp, speed_fbk);

    }
}

/****************************************以下为底层代码*************************************** */
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
    if (samp > PWM_ARR)
    {
        samp = PWM_ARR;
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

/* A 鐩?*/
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

/* B 鐩?*/
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

/* C 鐩?*/
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

/* 姣旇緝鍊艰缃?*/
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
    HAL_HRTIM_WaveformCountStop(
        &hhrtim1,
        HRTIM_TIMERID_MASTER
            | HRTIM_TIMERID_TIMER_A
            | HRTIM_TIMERID_TIMER_E
            | HRTIM_TIMERID_TIMER_F
    );

    HAL_HRTIM_WaveformOutputStop(
        &hhrtim1,
        HRTIM_OUTPUT_TA1
            | HRTIM_OUTPUT_TA2
            | HRTIM_OUTPUT_TE1
            | HRTIM_OUTPUT_TE2
            | HRTIM_OUTPUT_TF1
            | HRTIM_OUTPUT_TF2
    );
}

/*============================================================================*/
/*                    [AI] 鍚庡姞閮ㄥ垎锛歎SB 鏀朵竴琛屾暟瀛?-> Iq_Ref           */
/*============================================================================*/

static char     Usb_Line[24];
static uint8_t  Usb_Len = 0U;
static uint32_t Usb_Tick = 0U;

/* 鎶婄紦鍐查噷鐨勮繖涓€琛岃В鏋愭帀 */
static void FOC_Usb_Parse(void)
{
    char *end;

    Usb_Line[Usb_Len] = 0;

    Ref_Speed = strtof(Usb_Line, &end);


}

/**
  * @brief  USB CDC 鎺ユ敹鍥炶皟锛堝湪 usbd_cdc_if.c 鐨?CDC_Receive_FS 閲岃皟鐢級
  * @param  buf 鏀跺埌鐨勫瓧鑺?
  * @param  len 鏈寘闀垮害
  * @note   涓€琛屼竴涓暟瀛楋紝灏辨槸 Iq_Ref锛?
  *           "0.5\n"        -> Iq_Ref = 0.5
  *           "-0.25\r\n"    -> Iq_Ref = -0.25
  *         娌℃湁鍏抽敭瀛椼€佹病鏈夊崟浣嶃€傛病鏈夋崲琛屼篃琛岋紝瑙?FOC_Usb_Task()銆?
  */
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

/**
  * @brief  [AI] 涓诲惊鐜噷璋冪敤锛氭敹鍒板崐琛屻€?00 ms 娌″悗缁瓧鑺傦紝灏卞綋鎴愪竴鏉″畬鏁村懡浠?
  * @note   杩欐牱涓嶇涓婁綅鏈哄彂涓嶅彂鎹㈣閮借兘鐢?
  */
void FOC_Usb_Task(void)
{
    if ((Usb_Len != 0U) && ((HAL_GetTick() - Usb_Tick) >= 400U))
    {
        FOC_Usb_Parse();
        Usb_Len = 0U;
    }
}

