/**
  ******************************************************************************
  * @file    jf.h
  * @brief   VOFA+ JustFloat 数据流（走 USB CDC 虚拟串口）
  * @note    6 个 float32(小端) + 帧尾 00 00 80 7F，VOFA+ 选 JustFloat、6 通道。
  *
  *          用法（两行）：
  *            1) 在周期性执行的函数里调 JF_Push(...)，传 6 个 float
  *            2) 在主循环 while(1) 里调 JF_Task()，负责把数据搬上 USB
  *
  *          它和 usb.c 的 printf 共用同一个 CDC 端点，同时发会互相抢 BUSY；
  *          printf 只把字符放进 FIFO，需调用 USB_Printf_Flush() 才会真正发出。
  ******************************************************************************
  */

#ifndef __JF_H_
#define __JF_H_

#include <stdint.h>

/* 抽点：每 JF_DECIM 次 JF_Push 存一帧。
 * 电流环 42.5 kHz、JF_DECIM=1 时相当于每拍一帧（28 B/帧），
 * 已接近 USB 全速 CDC 的带宽上限，CDC 忙时缓冲满会丢新帧。 */
#define JF_DECIM        1U

/* 往帧里塞 6 个通道，不保证立即发送（由 JF_Task 搬） */
void JF_Push(float c0, float c1, float c2, float c3, float c4, float c5);

/* 主循环里反复调用，把缓冲里的帧搬到 USB */
void JF_Task(void);

#endif /* __JF_H_ */
