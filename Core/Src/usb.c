/**
  ******************************************************************************
  * @file    usb.c
  * @brief   printf 重定向到 USB CDC（发送 FIFO + 手动 flush）
  * @note    __io_putchar() 只往环形 FIFO 里塞，不阻塞；
  *          USB_Printf_Flush() 负责往 CDC 端点搬，忙就下次再搬。
  *          注意：工程里目前没有调用 USB_Printf_Flush()，printf 内容会一直留在 FIFO。
  ******************************************************************************
  */

/*============================================================================*/
/*                                   头文件                                   */
/*============================================================================*/
#include <stdint.h>
#include "usbd_cdc_if.h"
#include "usb.h"

/*============================================================================*/
/*                                  配置参数                                  */
/*============================================================================*/
#define USB_TX_FIFO_SIZE    512U            /* 发送缓冲大小 */

/*============================================================================*/
/*                                  内部变量                                  */
/*============================================================================*/
static uint8_t           Usb_Tx_Fifo[USB_TX_FIFO_SIZE];
static volatile uint16_t Fifo_Head = 0U;    /* 写指针（printf 里改） */
static volatile uint16_t Fifo_Tail = 0U;    /* 读指针（flush 里改） */

/**
  * @brief  空转延时（主频 170 MHz）
  * @param  num 空转次数
  */
static void Delay_Nop(uint32_t num)
{
    volatile uint32_t i;

    for (i = 0U; i < num; i++)
    {
        __NOP();
    }
}

/**
  * @brief  printf 重定向：把字符塞进 FIFO
  * @param  ch 待发送字符
  * @retval 原样返回 ch
  * @note   FIFO 满了直接丢弃，不会阻塞
  */
int __io_putchar(int ch)
{
    const uint16_t next_head = (Fifo_Head + 1U) % USB_TX_FIFO_SIZE;

    if (next_head != Fifo_Tail)
    {
        Usb_Tx_Fifo[Fifo_Head] = (uint8_t)ch;
        Fifo_Head = next_head;
    }

    return ch;
}

/**
  * @brief  把 FIFO 里的数据搬上 USB
  * @param  nop 每次发送后附加的空转次数，用于控制发送节奏
  * @note   可以几个 printf 之后调用一次；CDC 忙时保留 tail 下次重试
  */
void USB_Printf_Flush(uint64_t nop)
{
    uint16_t send_len;

    if (Fifo_Head == Fifo_Tail)
    {
        return;
    }

    if (Fifo_Head > Fifo_Tail)
    {
        send_len = Fifo_Head - Fifo_Tail;
    }
    else
    {
        send_len = USB_TX_FIFO_SIZE - Fifo_Tail;
    }

    /* 全速 USB 最大包 64 字节 */
    if (send_len > 64U)
    {
        send_len = 64U;
    }

    if (CDC_Transmit_FS(&Usb_Tx_Fifo[Fifo_Tail], send_len) == USBD_OK)
    {
        Fifo_Tail = (Fifo_Tail + send_len) % USB_TX_FIFO_SIZE;
    }
    /* BUSY：不移动 tail，下一轮自动重试 */

    Delay_Nop((uint32_t)nop);
}
