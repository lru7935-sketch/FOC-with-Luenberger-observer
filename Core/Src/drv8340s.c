/**
  ******************************************************************************
  * @file    drv8340s.c
  * @brief   DRV8340S 三相栅极驱动器（SPI 配置 + 故障处理）
  * @note    16 bit 帧格式：bit15 = 读/写，bit14~8 = 寄存器地址，bit7~0 = 数据
  ******************************************************************************
  */

/*============================================================================*/
/*                                   头文件                                   */
/*============================================================================*/
#include "drv8340s.h"
#include "main.h"
#include "mc_ctrl.h"
#include "rgb.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_exti.h"
#include <stdint.h>
#include <stdio.h>

/*============================================================================*/
/*                                  内部函数原型                              */
/*============================================================================*/
static void    DRV_Write_Reg(uint8_t reg, uint8_t data);
static uint8_t DRV_Read_Reg(uint8_t reg);
static void    DRV_Fault_Clear(void);

/*============================================================================*/
/*                                  内部变量                                  */
/*============================================================================*/
volatile uint8_t       DRV_Fault_Flag = 0U;  /* 故障中断标志，对外声明 */
static Drv_Mode_Typedef Work_Mode = DRV_MODE_PWM6;  /* 记住工作模式，重启后要重写 */

/*============================================================================*/
/*                                 寄存器读写                                 */
/*============================================================================*/

/**
  * @brief  写一个寄存器
  * @param  reg  寄存器地址
  * @param  data 待写数据
  */
static void DRV_Write_Reg(uint8_t reg, uint8_t data)
{
    const uint16_t tx_data = ((uint16_t)reg << 8) | (uint16_t)data;

    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&DRV_SPI, (uint8_t *)&tx_data, 1, 10);
    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  读一个寄存器
  * @param  reg 寄存器地址
  * @retval 读回的数据
  */
static uint8_t DRV_Read_Reg(uint8_t reg)
{
    const uint16_t tx_data = 0x8000U | ((uint16_t)reg << 8);
    uint16_t       rx_data;

    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&DRV_SPI, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, 10);
    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_SET);

    return (uint8_t)rx_data;
}

/*============================================================================*/
/*                                  对外 API                                  */
/*============================================================================*/

/**
  * @brief  初始化驱动芯片：等母线建立 -> 唤醒 -> 配置工作模式
  * @param  mode 工作模式（6x / 3x PWM）
  * @note   每次写完都读回校验，直到读回值与写入值一致
  */
void DRV_Init(Drv_Mode_Typedef mode)
{
    Work_Mode = mode;

    HAL_NVIC_DisableIRQ(EXTI2_IRQn);//先屏蔽故障中断，避免初始化过程中误触发导致 Foc_EN 被清零

    /* 母线电压没建起来之前不往下走 */
    while (Get_Vbus() < DRV_UVLO)
    {
        HAL_Delay(1U);
    }

    RGB_Set_Blue(100U);

    HAL_Delay(DRV_T_WAKE);
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(DRV_T_READY);

    /* 配置芯片，具体都是啥自己看drv8340寄存器表*/
    uint8_t rx_data = 0xFFU;

    while (rx_data != (uint8_t)Work_Mode)
    {
        DRV_Write_Reg(DRV_REG_IC1, (uint8_t)Work_Mode);
        rx_data = DRV_Read_Reg(DRV_REG_IC1);
        HAL_Delay(1U);
    }

    rx_data = 0x00U;

    while (rx_data != 0x07U)
    {
        DRV_Write_Reg(DRV_REG_IC9, 0x07U);
        rx_data = DRV_Read_Reg(DRV_REG_IC9);
        HAL_Delay(1U);
    }

    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/**
  * @brief  设置三相的栅极驱动电流档位
  * @param  idrivep 档位（写进 IC3 / IC4 / IC5）
  */
void DRV_Set_Idrivep(Drv_Idrivep_Typedef idrivep)
{
    uint8_t rx_data = 0x00U;

    while (rx_data != (uint8_t)idrivep)
    {
        DRV_Write_Reg(DRV_REG_IC3, (uint8_t)idrivep);
        rx_data = DRV_Read_Reg(DRV_REG_IC3);
        HAL_Delay(1U);
    }

    rx_data = 0x00U;

    while (rx_data != (uint8_t)idrivep)
    {
        DRV_Write_Reg(DRV_REG_IC4, (uint8_t)idrivep);
        rx_data = DRV_Read_Reg(DRV_REG_IC4);
        HAL_Delay(1U);
    }

    rx_data = 0x00U;

    while (rx_data != (uint8_t)idrivep)
    {
        DRV_Write_Reg(DRV_REG_IC5, (uint8_t)idrivep);
        rx_data = DRV_Read_Reg(DRV_REG_IC5);
        HAL_Delay(1U);
    }
}

/**
  * @brief  故障处理：打印故障位并清标志，直到故障消失
  * @note   故障没消除时本函数不会返回（会一直重试）
  */
void DRV_Fault_Handle(void)
{
    uint8_t fault = DRV_Read_Reg(DRV_REG_FAULT);

    while (fault != 0U)
    {
        RGB_Set_Red(10U);

        for (uint8_t i = 0U; i <= 7U; i++)
        {
            if ((fault & (0x80U >> i)) == 0U)
            {
                continue;
            }

            switch (i)
            {
                case 0:
                    printf("fault: (bit0)\n");
                    break;

                case 1:
                    printf("fault: gate drive\n");
                    break;

                case 2:
                    printf("fault: charge pump undervoltage\n");
                    break;

                case 3:
                    printf("fault: undervoltage lockout\n");
                    break;

                case 4:
                    printf("fault: overcurrent (VDS)\n");
                    break;

                case 5:
                    printf("fault: overtemperature warning\n");
                    break;

                case 6:
                    printf("fault: overtemperature shutdown\n");
                    break;

                case 7:
                    printf("fault: open load / short to supply or GND\n");
                    break;

                default:
                    break;
            }
        }

        printf("\n");

        DRV_Fault_Clear();

        /* 清完标志延时一下，避免通讯太快又触发 */
        HAL_Delay(1000U);

        fault = DRV_Read_Reg(DRV_REG_FAULT);
    }

    printf("no fault now!\n");
}

/**
  * @brief  清除已锁存的故障位
  * @note   IC1 最高位写 1 清除，硬件会自动复位该位
  */
static void DRV_Fault_Clear(void)
{
    const uint8_t ic1_value = DRV_Read_Reg(DRV_REG_IC1);

    DRV_Write_Reg(DRV_REG_IC1, ic1_value | 0x80U);
}

/**
  * @brief  重启芯片：拉低使能再重新初始化
  * @note   重启后所有寄存器恢复默认，所以要重走一遍 DRV_Init
  */
void DRV_Restart(void)
{
    /* 使能脚拉低即进入低功耗休眠，20 µs 以上的低脉冲可以复位故障 */
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);

    HAL_Delay(1U);

    DRV_Init(Work_Mode);
}
