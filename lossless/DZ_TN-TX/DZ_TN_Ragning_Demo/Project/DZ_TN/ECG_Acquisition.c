/******************************************************************************
 *
 * ECG_Acquisition.c
 *
 * ECG data acquisition processing
 *
 ******************************************************************************/

#include "ECG_Acquisition.h"


#define ID_IRQ_DMA1  (0x6C)

uint16_t SampleBuff1[SampleBuffSize];
uint16_t SampleBuff2[SampleBuffSize];
//uint16_t *pDataBuff;
uint16_t *pSampleBuff1;
uint16_t *pSampleBuff2;
int Buffer_Ready;

void NVIC_Configuration(void);

//void DMA1_ISR(void);

//void ECGInit(void)
//{
//   pSampleBuff1 = 0;
//   NVIC_Configuration();
//}

void ResetTIM1(void)
{
   TIM1->CR1 = 0x00;                // TIM1_Cmd(DISABLE)
}

void ResetGPIO(void)
{
   GPIOA->CRH &= 0xFFFFFFF0;
   GPIOA->CRH |= 0x04;
   GPIOA->CRL |= 0x00000400;
}

//
//void ECG_NVIC_Configuration(void)
//{
//   NVIC_InitTypeDef NVIC_InitStructure; 
//   NVIC_Init(&NVIC_InitStructure);
//
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//
//   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
//}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

// Enable the DMA1 channel1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



void DMA1_Interrupts_Stop(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

//   NVIC_InitStructure.NVIC_IRQChannel = DMAChannel1_IRQChannel;
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
   NVIC_Init(&NVIC_InitStructure);

   //Stop TIM1
   RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);  
}

void DMA1_ISR(void)
{
   if ((DMA1->ISR & DMA1_FLAG_HT1) != 0)             // half transfer
   //if(DMA_GetFlagStatus(DMA_FLAG_HT1))
   {
      //pDataBuff = &ECG_Buff[0];
      pSampleBuff1 = &SampleBuff1[0];
   }
   else if ((DMA1->ISR & DMA1_FLAG_TC1) != 0)        // complete transfer
   //else if(DMA_GetFlagStatus(DMA_FLAG_TC1))
   {
      //pDataBuff = &ECG_Buff[DATA_BUFF_SIZE];
      pSampleBuff1 = &SampleBuff1[DATA_BUFF_SIZE];
   }

   //DMA_ClearFlag(DMA_FLAG_GL1);
   DMA1->IFCR = DMA1_FLAG_GL1;
   
}
