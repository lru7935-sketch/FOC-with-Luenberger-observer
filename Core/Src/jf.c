/**
  ******************************************************************************
  * @file    jf.c
  * @brief   VOFA+ JustFloat 数据流（走 USB CDC 虚拟串口）
  * @note    帧格式：8 x float32(小端) + 00 00 80 7F，共 36 字节。
  *          JF_Push() 在中断里存帧（几乎不耗时），JF_Task() 在主循环里搬 USB。
  ******************************************************************************
  */

#include "jf.h"
#include "usbd_cdc_if.h"

#define JF_NCH      6U                      /* 通道数 */
#define JF_FRAME    (JF_NCH * 4U + 4U)      /* 一帧 28 字节 */
#define JF_RING     256U                     /* 缓冲 64 帧，必须是 2 的幂 */
#define JF_CHUNK    1024U                    /* 每次最多搬 8 帧 */

/* 用 uint32_t 数组保证 4 字节对齐（float 指针要按 4 字节对齐访问） */
static uint32_t JF_Buf[(JF_RING * JF_FRAME) / 4U];

static volatile uint16_t JF_Head = 0U;      /* 写指针（中断里改） */
static volatile uint16_t JF_Tail = 0U;      /* 读指针（主循环里改） */
static volatile uint16_t JF_Cnt  = 0U;      /* 抽点计数 */

/**
  * @brief  存一帧数据（在周期性调用的地方调用，比如电流环中断）
  * @param  c0..c5 6 个通道的值
  * @note   内部按 JF_DECIM 抽点；缓冲满了就丢新帧，不会阻塞
  */
void JF_Push(float c0, float c1, float c2, float c3, float c4, float c5)
{
    uint16_t head;
    uint16_t next;
    uint8_t *p;
    float   *f;

    if (++JF_Cnt < JF_DECIM)
    {
        return;
    }

    JF_Cnt = 0U;

    head = JF_Head;
    next = (uint16_t)((head + 1U) & (JF_RING - 1U));

    if (next == JF_Tail)
    {
        return;                             /* 缓冲满，丢帧 */
    }

    p = (uint8_t *)JF_Buf + ((uint32_t)head * JF_FRAME);
    f = (float *)p;

    f[0] = c0;
    f[1] = c1;
    f[2] = c2;
    f[3] = c3;
    f[4] = c4;
    f[5] = c5;

    p[JF_NCH * 4U + 0U] = 0x00U;            /* JustFloat 帧尾 */
    p[JF_NCH * 4U + 1U] = 0x00U;
    p[JF_NCH * 4U + 2U] = 0x80U;
    p[JF_NCH * 4U + 3U] = 0x7FU;

    JF_Head = next;
}

/**
  * @brief  把缓冲里的帧搬到 USB（在主循环 while(1) 里反复调用）
  * @note   CDC 忙就直接返回，下一次再搬，不会卡主循环
  */
void JF_Task(void)
{
    uint32_t frames;
    uint32_t room;
    uint16_t chunk;

    if (JF_Tail == JF_Head)
    {
        return;
    }

    frames = (uint32_t)((JF_Head + JF_RING - JF_Tail) & (JF_RING - 1U));

    /* 环形缓冲会回卷：一次只能发到数组末尾，不能越过 JF_Buf 尾部 */
    room = (uint32_t)(JF_RING - JF_Tail);
    if (frames > room)
    {
        frames = room;
    }

    /* 再受单次传输上限约束 */
    if (frames > (JF_CHUNK / JF_FRAME))
    {
        frames = JF_CHUNK / JF_FRAME;
    }

    chunk = (uint16_t)(frames * JF_FRAME);

    if (CDC_Transmit_FS((uint8_t *)JF_Buf + ((uint32_t)JF_Tail * JF_FRAME),
                        chunk) == USBD_OK)
    {
        JF_Tail = (uint16_t)((JF_Tail + (uint16_t)frames) & (JF_RING - 1U));
    }
}
