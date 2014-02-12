/******************************************************************************
 *
 * ECG_Acquisition.h
 * Jingxi Zhang
 * 
 * Header file for ECG data acquisition processing
 *
 ******************************************************************************/

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"

#ifndef _ECG_ACQUISITION_H_
#define _ECG_ACQUISITION_H_

//#define ADC1_DR_Address    ((u32)0x4001244C)
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

#define NUM_ADC                   3 
#define SampleBuffSize            150
#define DATA_BUFF_SIZE            (SampleBuffSize/2)

//#define AVERAGE_BIN               4
//#define DISPLAY_BUFF_SIZE         120
//#define DISPLAY_BUFF_INCREMENT    (DATA_BUFF_SIZE/AVERAGE_BIN)

//#define UTIL_SetIrqHandler(a,b)   void UTIL_SetIrqHandler((int)a, (tHandler)b);
//#define UTIL_GetIrqHandler(a)     tHandler* UTIL_GetIrqHandler ((int)a);

//typedef void (*tHandler) (void);

extern uint16_t SampleBuff1[SampleBuffSize];
extern uint16_t SampleBuff2[SampleBuffSize];
extern int Buffer_Ready;
//extern uint16_t *pDataBuff;
extern uint16_t *pSampleBuff1;
extern uint16_t *pSampleBuff2;

// Function declare
//void UTIL_SetIrqHandler(int, tHandler);
//tHandler* UTIL_GetIrqHandler(int);
void ECGInit(void);
//void ECGReset(void);
void InitTIM1(void);
void ResetTIM1(void);
void InitDMA1(void);
//void ResetDMA1(void);
void InitADC(void);
void ResetADC(void);
void InitGPIO(void);
void ResetGPIO(void);
void DMA1_Interrupts_Stop(void);
//void UTIL_SetIrqHandler( s32 Offs, tHandler pHDL );
//void UTIL_SetIrqHandler ( int , tHandler );
//tHandler UTIL_GetIrqHandler( s32 Offs );
//tHandler UTIL_GetIrqHandler ( int );


#endif //_ECG_ACQUISITION_H_
