/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用位置式 PID（带输出限幅 + 积分抗饱和）
  ******************************************************************************
  */

#ifndef __PID_H_
#define __PID_H_

/*============================================================================*/
/*                                  类型定义                                  */
/*============================================================================*/

typedef struct
{
    float Kp;                   /* 比例系数 */
    float Ki;                   /* 积分系数 */
    float Kd;                   /* 微分系数 */

    float Err;                  /* 当前误差 = Ref - Fbk */
    float Inte;                 /* 积分累加值 */
    float Fbk_Last;             /* 上次反馈值，用于微分 */

    float Dt;                   /* 调用周期，s */
    float OUT_MAX;              /* 输出上限 */
    float OUT_MIN;              /* 输出下限 */
} Pid_Typedef;

/*============================================================================*/
/*                                  对外 API                                  */
/*============================================================================*/

void  PID_Init(Pid_Typedef *pid, float kp, float ki, float kd,
               float dt, float out_max, float out_min);
float PID_Update(Pid_Typedef *pid, float ref, float fbk);
void  PID_Reset(Pid_Typedef *pid);

#endif /* __PID_H_ */
