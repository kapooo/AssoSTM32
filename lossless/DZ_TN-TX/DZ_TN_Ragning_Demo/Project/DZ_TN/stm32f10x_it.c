/**

  ******************************************************************************

  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 

  * @author  MCD Application Team

  * @version V3.3.0

  * @date    04/16/2010

  * @brief   Main Interrupt Service Routines.

  *          This file provides template for all exceptions handler and 

  *          peripherals interrupt service routine.

  ******************************************************************************

  * @copy

  *

  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS

  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE

  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY

  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING

  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE

  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

  *

  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>

  */ 



/* Includes ------------------------------------------------------------------*/



#include "stm32f10x_tim.h" 

#include "stm32f10x_it.h"

#include "ECG_Acquisition.h"

#include "stm32f10x_dma.h"

/** @addtogroup STM32F10x_StdPeriph_Template

  * @{

  */



/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern __IO uint32_t systick;

/* Private function prototypes -----------------------------------------------*/

extern void TimingDelay_Decrement(void);

/* Private functions ---------------------------------------------------------*/



/******************************************************************************/

/*            Cortex-M3 Processor Exceptions Handlers                         */

/******************************************************************************/



/**

  * @brief   This function handles NMI exception.

  * @param  None

  * @retval None

  */

void NMI_Handler(void)

{

}



#include "usart.h"



/**

  * @brief  This function handles Hard Fault exception.

  * @param  None

  * @retval None

  */

void HardFault_Handler_OG(void)

{

  /* Go to infinite loop when Hard Fault exception occurs */

//#warning only a workaround!

 // NVIC_SystemReset();

  while (1)

 

  {

  }

}

/**

  * @brief  This function handles Memory Manage exception.

  * @param  None

  * @retval None

  */

void MemManage_Handler(void)

{

  /* Go to infinite loop when Memory Manage exception occurs */

  while (1)

  {

  }

}



/**

  * @brief  This function handles Bus Fault exception.

  * @param  None

  * @retval None

  */

void BusFault_Handler(void)

{

  /* Go to infinite loop when Bus Fault exception occurs */

  while (1)

  {

  }

}



/**

  * @brief  This function handles Usage Fault exception.

  * @param  None

  * @retval None

  */

void UsageFault_Handler(void)

{

  /* Go to infinite loop when Usage Fault exception occurs */

  while (1)

  {

  }

}



/**

  * @brief  This function handles SVCall exception.

  * @param  None

  * @retval None

  */

void SVC_Handler(void)

{

}



/**

  * @brief  This function handles Debug Monitor exception.

  * @param  None

  * @retval None

  */

void DebugMon_Handler(void)

{

}



/**

  * @brief  This function handles PendSVC exception.

  * @param  None

  * @retval None

  */

void PendSV_Handler(void)

{

}



/**

  * @brief  This function handles SysTick Handler.

  * @param  None

  * @retval None

  */

void SysTick_Handler(void)

{

   TimingDelay_Decrement();

   systick++;

}



/******************************************************************************/

/*                 STM32F10x Peripherals Interrupt Handlers                   */

/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */

/*  available peripheral interrupt handler's name please refer to the startup */

/*  file (startup_stm32f10x_xx.s).                                            */

/******************************************************************************/



/**

  * @brief  This function handles TIM2 global interrupt request.

  * @param  None

  * @retval None

  */

void TIM2_IRQHandler(void)

{

}

//void DMA1_ISR(void);
// volatile  uint16_t cont_tim1[3];       //variabile utilizzata per il calcolo dei tempi di esecuzione         

void DMA1_Channel1_IRQHandler(void)
{ 
  uint16_t Media_ECG;
  uint16_t Media_Respiro;
  uint16_t Media_Temperatura;
  //int i;
  //if ((DMA1->ISR & DMA1_FLAG_HT1 & !HT1_served) != 0)             // half transfer
  //if(DMA_GetFlagStatus(DMA_FLAG_HT1))
  if ((DMA1->ISR & DMA1_FLAG_TC1) != 0)
   {
      memcpy(SampleBuff2, SampleBuff1, sizeof(SampleBuff2)); // SampleBuff2=SampleBuff1
      //pSampleBuff2 = &SampleBuff2[0];
      pSampleBuff2 = &SampleBuff2[DATA_BUFF_SIZE];
      Buffer_Ready=1;
      DMA_ClearFlag(DMA1_FLAG_TC1);
      //DMA_ClearITPendingBit(DMA1_IT_HT1);
   }
   else if ((DMA1->ISR & DMA1_FLAG_HT1) != 0)        // complete transfer
   //else if(DMA_GetFlagStatus(DMA_FLAG_TC1))
   {
      memcpy(SampleBuff2, SampleBuff1, sizeof(SampleBuff2)); // SampleBuff2=SampleBuff1
      //pSampleBuff2 = &SampleBuff2[DATA_BUFF_SIZE];
      pSampleBuff2 = &SampleBuff2[0];
      Buffer_Ready=1; 
      DMA_ClearFlag(DMA1_FLAG_HT1);
      //DMA_ClearITPendingBit(DMA1_IT_HT1);
   }
   else 
     DMA_ClearFlag(DMA1_FLAG_GL1);
     //DMA_ClearITPendingBit(DMA1_IT_GL1);
}


/**

  * @brief  This function handles PPP interrupt request.

  * @param  None

  * @retval None

  */

/*void PPP_IRQHandler(void)

{

}*/



/**

  * @}

  */ 





/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

