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
  *          它和 usb.c 的 printf 通道互斥（共用同一个 CDC 端点），
  *          同一个工程里二选一。
  ******************************************************************************
  */

#ifndef __JF_H_
#define __JF_H_

#include <stdint.h>

/* 抽点：每 JF_DECIM 次 JF_Push 存一帧。
 * 在 10.625 kHz 的电流环里调用时：8 -> 1328 Hz，4 -> 2656 Hz，16 -> 664 Hz。
 * 6 通道一帧 28 字节，1328 Hz 约 37 kB/s。 */
#define JF_DECIM        1U

/* 往帧里塞 6 个通道，不保证立即发送（由 JF_Task 搬） */
void JF_Push(float c0, float c1, float c2, float c3, float c4, float c5);

/* 主循环里反复调用，把缓冲里的帧搬到 USB */
void JF_Task(void);

#endif /* __JF_H_ */
