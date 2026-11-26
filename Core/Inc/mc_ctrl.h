#ifndef __FOC_H_
#define __FOC_H_

#include <stdint.h>
#include "pid.h"
#include "mc_math.h"

// #define GAIN_IA             0.994160f
// #define OFFSET_IA           0.000000f
// #define GAIN_IB             0.985899f
// #define OFFSET_IB           0.000483f
// #define GAIN_IC             1.000000f
// #define OFFSET_IC           0.000000f

#define GAIN_IA             1.0f            /* A 相增益校准 */
#define OFFSET_IA           0.0f            /* A 相零漂校准，A */
#define GAIN_IB             1.0f            /* B 相增益校准 */
#define OFFSET_IB           0.0f            /* B 相零漂校准，A */
#define GAIN_IC             1.0f            /* C 相增益校准 */
#define OFFSET_IC           0.0f            /* C 相零漂校准，A */

void MOTOR_Init(void);
void MOTOR_Start_Task(void);
void Foc(void);
void Foc_In_Loop(void);
void Foc_Ex_Loop(void);
float Get_Vbus(void);
float Get_Ia(void);
float Get_Ib(void);
float Get_Ic(void);
uint16_t ADC_Get_Ia(void);
uint16_t ADC_Get_Ib(void);
uint16_t ADC_Get_Ic(void);
uint16_t ADC_Get_Vbus(void);
uint16_t ADC_Get_Tem(void);
void ADC_Set_Sample_Point(uint16_t samp);
void PWM_Start_A(void);
void PWM_Start_B(void);
void PWM_Start_C(void);
void PWM_Stop_A(void);
void PWM_Stop_B(void);
void PWM_Stop_C(void);
void PWM_CCR_A(uint16_t comp);
void PWM_CCR_B(uint16_t comp);
void PWM_CCR_C(uint16_t comp);
void PWM_CCR_ALL(CCR_Typedef CCR);
void PWM_Start_ALL(void);
void PWM_Stop_ALL(void);
void FOC_Usb_Receive(uint8_t *buf, uint32_t len);   /* 收一行 "速度[,Id]"，写 Ref_Speed / Id_Ref */
void FOC_Usb_Task(void);                            /* 主循环里调用：400 ms 无换行也自动断句 */
void OL_Init(void);
void PWM_Count_Start(void);
void Foc_PID_Init(void);
void Lbg_Para_Update(void);

#endif /* __FOC_H_ */