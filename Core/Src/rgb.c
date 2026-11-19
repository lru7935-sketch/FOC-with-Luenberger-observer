/**
  ******************************************************************************
  * @file    rgb.c
  * @brief   WS2812 灯珠驱动（TIM3 CH2 + DMA）
  * @note    DMA 方向必须是内存 -> 寄存器；内存数据宽度按需选字节，寄存器按 ARR 定。
  *          这里 PWM 周期 1200 ns，写 6 表示 1 码（高电平 600 ns），
  *          写 3 表示 0 码（高电平 300 ns），末尾补低电平构成复位帧。
  *
  *          DMA 必须开启中断，否则 normal 模式下即使搬完了也一直是 busy，
  *          第二次就没法再写入颜色。
  ******************************************************************************
  */

/*============================================================================*/
/*                                   头文件                                   */
/*============================================================================*/
#include "rgb.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdint.h>

/*============================================================================*/
/*                                  内部变量                                  */
/*============================================================================*/
static volatile uint8_t RGB_Buf[RGB_BUF_LEN];

/*============================================================================*/
/*                                  内部函数                                  */
/*============================================================================*/

/**
  * @brief  把一个字节编码进 DMA 缓冲（高位先行）
  * @param  value  待发送字节
  * @param  offset 该字节在缓冲中的起始下标
  */
static void RGB_Encode_Byte(uint8_t value, uint16_t offset)
{
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((value & (0x80U >> i)) != 0U)
        {
            RGB_Buf[offset + i] = 6U;
        }
        else
        {
            RGB_Buf[offset + i] = 3U;
        }
    }
}

/**
  * @brief  把角度折算到 [-PI, PI]
  */
static float Angle_Wrap(float angle)
{
    angle = fmodf(angle, 2.0f * RGB_PI);

    if (angle > RGB_PI)
    {
        angle -= 2.0f * RGB_PI;
    }

    if (angle < -RGB_PI)
    {
        angle += 2.0f * RGB_PI;
    }

    return angle;
}

/*============================================================================*/
/*                                  对外 API                                  */
/*============================================================================*/

/**
  * @brief  清空缓冲并关灯
  */
void RGB_Init(void)
{
    for (uint16_t i = 0U; i < RGB_BUF_LEN; i++)
    {
        RGB_Buf[i] = 0U;
    }

    RGB_Set_Color(0U, 0U, 0U);
}

/**
  * @brief  设置颜色并启动一次 DMA 发送
  * @param  r/g/b 三色亮度
  * @retval HAL 状态
  * @note   WS2812 的发送顺序是 G -> R -> B，高位先行
  */
HAL_StatusTypeDef RGB_Set_Color(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_Encode_Byte(g, 0U);
    RGB_Encode_Byte(r, 8U);
    RGB_Encode_Byte(b, 16U);

    return HAL_TIM_PWM_Start_DMA(&RGB_TIM, RGB_CHANNEL,
                                 (const uint32_t *)RGB_Buf, RGB_BUF_LEN);
}

void RGB_Set_Red(uint8_t brightness)
{
    RGB_Set_Color(brightness, 0U, 0U);
}

void RGB_Set_Green(uint8_t brightness)
{
    RGB_Set_Color(0U, brightness, 0U);
}

void RGB_Set_Blue(uint8_t brightness)
{
    RGB_Set_Color(0U, 0U, brightness);
}

/**
  * @brief  按正弦色环刷新一次颜色，每次调用转 1°
  * @param  brightness 最大亮度 0~255
  * @note   内部带 10 ms 延时，适合放主循环里做呼吸灯
  */
void RGB_Show(uint8_t brightness)
{
    static float hue = 0.0f;

    const float phase_r = Angle_Wrap(hue);
    const float phase_g = Angle_Wrap(hue + 0.667f * RGB_PI);
    const float phase_b = Angle_Wrap(hue - 0.667f * RGB_PI);

    const uint8_t r = brightness / 2 + (brightness / 2 - 1) * sinf(phase_r);
    const uint8_t g = brightness / 2 + (brightness / 2 - 1) * sinf(phase_g);
    const uint8_t b = brightness / 2 + (brightness / 2 - 1) * sinf(phase_b);

    RGB_Set_Color(r, g, b);

    hue += RGB_PI / 180.0f;

    HAL_Delay(10);
}

void RGB_Close(void)
{
    RGB_Set_Color(0U, 0U, 0U);
}

/**
  * @brief  闪一下指定颜色
  * @param  color     颜色
  * @param  brightness 亮度
  * @param  interval  一次亮灭的总时长，ms
  */
void RGB_Blink(RGB_Color_Typedef color, uint8_t brightness, uint32_t interval)
{
    switch (color)
    {
        case RGB_RED:
            RGB_Set_Red(brightness);
            break;

        case RGB_GREEN:
            RGB_Set_Green(brightness);
            break;

        case RGB_BLUE:
            RGB_Set_Blue(brightness);
            break;

        default:
            RGB_Close();
            break;
    }

    HAL_Delay(interval / 2U);
    RGB_Close();
    HAL_Delay(interval / 2U);
}
