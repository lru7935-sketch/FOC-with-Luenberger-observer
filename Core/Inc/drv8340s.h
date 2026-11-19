/**
  ******************************************************************************
  * @file    drv8340s.h
  * @brief   DRV8340S 三相栅极驱动器（SPI 配置 + 故障处理）
  ******************************************************************************
  */

#ifndef __DRV8340S_H_
#define __DRV8340S_H_

/*============================================================================*/
/*                                   头文件                                   */
/*============================================================================*/
#include "gpio.h"
#include "spi.h"
#include "tim.h"

/*============================================================================*/
/*                              寄存器地址（SPI）                             */
/*============================================================================*/

/* 只读 */
#define DRV_REG_FAULT       0x00            /* 故障状态 */
#define DRV_REG_DIAG_A      0x01            /* 诊断状态 A */
#define DRV_REG_DIAG_B      0x02            /* 诊断状态 B */
#define DRV_REG_DIAG_C      0x03            /* 诊断状态 C */

/* 可读可写 */
#define DRV_REG_IC1         0x04
#define DRV_REG_IC2         0x05
#define DRV_REG_IC3         0x06
#define DRV_REG_IC4         0x07
#define DRV_REG_IC5         0x08
#define DRV_REG_IC6         0x09
#define DRV_REG_IC7         0x0A
#define DRV_REG_IC8         0x0B
#define DRV_REG_IC9         0x0C
#define DRV_REG_IC10        0x0D
#define DRV_REG_IC11        0x0E
#define DRV_REG_IC12        0x0F
#define DRV_REG_IC13        0x10
#define DRV_REG_IC14        0x11

/*============================================================================*/
/*                                  引脚定义                                  */
/*============================================================================*/
#define DRV_SPI             hspi3           /* 配置所用 SPI 句柄 */
#define DRV_CS_PORT         GPIOA           /* 片选 */
#define DRV_CS_PIN          GPIO_PIN_15
#define DRV_EN_PORT         GPIOA           /* 使能（低电平休眠） */
#define DRV_EN_PIN          GPIO_PIN_10
#define DRV_FAULT_PIN       GPIO_PIN_2      /* 故障中断脚 */

/*============================================================================*/
/*                                  时间参数                                  */
/*============================================================================*/
#define DRV_UVLO            5.9f            /* 欠压阈值，V */
#define DRV_T_WAKE          1U              /* 唤醒后等待，ms */
#define DRV_T_READY         1U              /* 使能后等待，ms */

/*============================================================================*/
/*                                  类型定义                                  */
/*============================================================================*/

/* 工作模式，直接写进 IC1 寄存器 */
typedef enum
{
    DRV_MODE_PWM6 = 0x00,
    DRV_MODE_PWM3 = 0x10
} Drv_Mode_Typedef;

/* 栅极驱动电流档位，直接写进 IC3~IC5 寄存器 */
typedef enum
{
    DRV_IDRIVEP_50MA   = 0x55,
    DRV_IDRIVEP_60MA   = 0x66,
    DRV_IDRIVEP_200MA  = 0x88,
    DRV_IDRIVEP_260MA  = 0xAA,
    DRV_IDRIVEP_800MA  = 0xDD,
    DRV_IDRIVEP_1000MA = 0xFF
} Drv_Idrivep_Typedef;

/*
   6x PWM 模式真值表                     3x PWM 模式真值表
   INLx INHx GLx GHx SHx + DLx          INLx INHx GLx GHx SHx + DLx
    0    0    L   L    Hi-Z              0    X    L   L    Hi-Z
    0    1    L   H     H                1    0    H   L      L
    1    0    H   L     L                1    1    L   H      H
    1    1    L   L    Hi-Z
*/

/*============================================================================*/
/*                                  对外 API                                  */
/*============================================================================*/
void DRV_Init(Drv_Mode_Typedef mode);
void DRV_Fault_Handle(void);
void DRV_Set_Idrivep(Drv_Idrivep_Typedef idrivep);
void DRV_Restart(void);

/*============================================================================*/
/*                                  对外变量                                  */
/*============================================================================*/
extern volatile uint8_t DRV_Fault_Flag;      /* 故障中断标志，置位后需手动清除 */

#endif /* __DRV8340S_H_ */
