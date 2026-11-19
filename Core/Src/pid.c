/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用位置式 PID
  * @note    输出超限时把积分退回上一拍，防止积分饱和
  ******************************************************************************
  */

#include "pid.h"

/**
  * @brief  计算一次 PID 输出
  * @param  pid 控制器结构体
  * @param  ref 给定值
  * @param  fbk 反馈值
  * @retval 限幅后的输出
  */
float PID_Update(Pid_Typedef *pid, float ref, float fbk)
{
    const float inte_buf = pid->Inte;

    pid->Err = ref - fbk;

    const float p = pid->Kp * pid->Err;
    
    pid->Inte += pid->Err * pid->Dt;
    const float i = pid->Ki * pid->Inte;

    const float d = pid->Kd * (fbk - pid->Fbk_Last) / pid->Dt;
    pid->Fbk_Last = fbk;

    float out = p + i + d;

    if (out > pid->OUT_MAX)
    {
        out = pid->OUT_MAX;
        pid->Inte = inte_buf;
    }

    if (out < pid->OUT_MIN)
    {
        out = pid->OUT_MIN;
        pid->Inte = inte_buf;
    }

    return out;
}

/**
  * @brief  初始化控制器
  * @param  pid     控制器结构体
  * @param  kp/ki/kd 三个系数
  * @param  dt      调用周期，s
  * @param  out_max 输出上限
  * @param  out_min 输出下限
  */
void PID_Init(Pid_Typedef *pid, float kp, float ki, float kd,
              float dt, float out_max, float out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->Dt = dt;

    pid->Err      = 0.0f;
    pid->Inte     = 0.0f;
    pid->Fbk_Last = 0.0f;

    pid->OUT_MAX = out_max;
    pid->OUT_MIN = out_min;
}

/**
  * @brief  清空控制器状态（系数不变）
  */
void PID_Reset(Pid_Typedef *pid)
{
    pid->Err      = 0.0f;
    pid->Inte     = 0.0f;
    pid->Fbk_Last = 0.0f;
}
