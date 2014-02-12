/**
  ******************************************************************************
  * @file    led.c
  * @author  Dizic Team
  * @version V1.0.0
  * @date    05/16/2010
  * @brief   This file provides the initialization function for the LED.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, DITIC SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 DIZIC</center></h2>
  */ 

#include "stm32f10x.h"
#include "dz_tn.h"
#include "led.h"

void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* init LED */
   RCC_APB2PeriphClockCmd(TN100_LED0_GPIO_CLK,ENABLE);
    /*!< Configure TN100_LED pin: LED0 */
  GPIO_InitStructure.GPIO_Pin = TN100_LED0_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(TN100_LED0_GPIO_PORT, &GPIO_InitStructure);
  /* LED0 off*/
  LED0_OFF();
}