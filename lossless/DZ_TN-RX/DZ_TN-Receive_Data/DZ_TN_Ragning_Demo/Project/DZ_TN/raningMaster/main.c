/**

  ******************************************************************************

  * @file    main.c

  * @author  Alessio Carpini

  * @version V1.5

  * @date    03/02/2014

  * @brief   This file provides the application.

  * RX Device2
  ******************************************************************************

  * @copy

  */ 



/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"

#include "usart.h"

#include "tn100_lib.h"

#include "timer.h" 

#include "led.h"

#include <stdio.h>

#include <string.h>

#include "stm32f10x_adc.h"

#include "stm32f10x_tim.h"

#include "stm32f10x_crc.h"


#define MSG_LEN 128

#define NUM_SAMP 50

#define PA_EN 1

#warning RANGING_MASTER IS SELECTED



/* Private functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/**

  * @brief  Main program.

  * @param  None

  * @retval None

  */

int main(void)

{

  /*!< At this stage the microcontroller clock setting is already configured, 

       this is done through SystemInit() function which is called from startup

       file (startup_stm32f10x_xx.s) before to branch to application main.

       To reconfigure the default setting of SystemInit() function, refer to

       system_stm32f10x.c file

  */
  

  TN100_InitTypeDef  TN100_InitStructure;

  uint8_t senderID[6];

  uint8_t receiverID[6];

  uint8_t packetType;

  uint8_t rxmsg[MSG_LEN];
  
  uint8_t *prxmsg; 

  uint16_t rxmsglen;

  int16_t val,scale;
  
  int8_t val8;
  
  uint32_t rangingtimer = 0;

  uint16_t i;

  int x=0;
  
  uint8_t diz[49], j, q, CRC_val, CRC_Tx; 
  
     
  
  /* Initialize the timer */

  Timer_Init();

  /* Initialize the usart interface */

  usart_init(115200); // Init usart with 115200 Baud
  usart2_init(115200); // Abilito USART2

  /* Unlock the Flash Program Erase controller */

  FLASH_Unlock();

  /*Initialize the LEDs*/

  LED_Init();

  /*Configure the TN100*/

  TN100_InitStructure.srcId[0] = 0x04;      // Source address

  TN100_InitStructure.srcId[1] = 0x00;

  TN100_InitStructure.srcId[2] = 0x00;

  TN100_InitStructure.srcId[3] = 0x00;

  TN100_InitStructure.srcId[4] = 0x00;

  TN100_InitStructure.srcId[5] = 0x00;

  TN100_InitStructure.destId[0] = 0x02;     // Destination address

  TN100_InitStructure.destId[1] = 0x00;

  TN100_InitStructure.destId[2] = 0x00;

  TN100_InitStructure.destId[3] = 0x00;

  TN100_InitStructure.destId[4] = 0x00;

  TN100_InitStructure.destId[5] = 0x00;



  TN100_InitStructure.syncword[0] = 0xAB;

  TN100_InitStructure.syncword[1] = 0x2C;

  TN100_InitStructure.syncword[2] = 0xD5;

  TN100_InitStructure.syncword[3] = 0x92;

  TN100_InitStructure.syncword[4] = 0x94;

  TN100_InitStructure.syncword[5] = 0xCA;

  TN100_InitStructure.syncword[6] = 0x69;

  TN100_InitStructure.syncword[7] = 0xAB;    

  

  TN100_InitStructure.txpacketType = TN100_TxData;

  TN100_InitStructure.rxpacketType = TN100_RxData;  

  /*set channel (center frequency)*/

//  TN100_InitStructure.chNo = W4_2442MHZ;
  TN100_InitStructure.chNo = E1_2412MHZ;    

 // TN100_InitStructure.mode = TN100_80MHz_1MS_1us;
  //TN100_InitStructure.mode = TN100_80MHz_500kS_2us;
 TN100_InitStructure.mode = TN100_80MHz_250kS_4us;
  //TN100_InitStructure.mode = TN100_22MHz_1MS_1us;
  //TN100_InitStructure.mode = TN100_22MHz_500kS_2us;
  //TN100_InitStructure.mode = TN100_22MHz_250kS_4us;  
//
//  TN100_InitStructure.txpwr = 0x3F;
//
//  TN100_InitStructure.txArq = 0x04;
//
//  TN100_InitStructure.txArqMode = 0x01; 
//
//  TN100_InitStructure.rxArqMode = TN100_RxArqModeCrc2;
//
//  TN100_InitStructure.addrMatching = ON;
  
  TN100_InitStructure.txpwr = 0x3F;   // max =0x3F

  TN100_InitStructure.txArq = 0x01;

  TN100_InitStructure.txArqMode = 0x03; 

  TN100_InitStructure.rxArqMode = TN100_RxArqModeCrc2;

  TN100_InitStructure.addrMatching = ON;

    

#ifdef PA_EN

#warning PA is on

  TN100_PA_Init(TN100_PA_SMD_ANT); 

#endif



  /* Initialize delay pointer (needed for the TN100_lib)*/

  Ptr_Delay_ms = Delay_ms;

  /* Initialize get time pointer (needed for the TN100_lib)*/  

  Ptr_GetTime_ms = GetSysTick;

 

  /* Initialize the TN100 module */

  if(TN100_Init(TN100_INIT_FULL, &TN100_InitStructure) != 1)

  {

    myprintf("initialization failed!\n");

    myprintf("stop application!\n");

    while(1);

  }

  /* set source address */

  TN100_SetStationAddr(&TN100_InitStructure.srcId[0],&TN100_InitStructure.srcId[0]); 
  
  rxmsg[0] = 0;  
  
  while(1){
    
    prxmsg= &rxmsg[0];
     
    TN100_Callibration();
    rxmsglen = TN100_is_Msg_Received(&rxmsg[0], senderID, receiverID, &packetType);    
    
    if (rxmsglen > 0){      
      
      TN100_LED0_ON();
      
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);   
      
      CRC->CR = 0x00000001;    
      
      CRC_val=CRC_CalcBlockCRC((uint32_t *)prxmsg, ((rxmsglen-1)/4));        
      
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,DISABLE); 
      
      CRC_Tx=rxmsg[rxmsglen-1];
    
    if(CRC_Tx==CRC_val){      
    
    //myprintf("%4d\t", rxmsglen);
    //myprintf("\n"); 
     
     j=0;
     
     for(i=(rxmsglen-8); i<rxmsglen-2; i=i++){
      
       diz[j]=(rxmsg[i]&0x80)>>7;
       diz[j+1]=(rxmsg[i]&0x40)>>6;
       diz[j+2]=(rxmsg[i]&0x20)>>5;
       diz[j+3]=(rxmsg[i]&0x10)>>4;
       diz[j+4]=(rxmsg[i]&0x8)>>3;
       diz[j+5]=(rxmsg[i]&0x4)>>2;
       diz[j+6]=(rxmsg[i]&0x2)>>1;
       diz[j+7]=(rxmsg[i]&0x1);
       
       j=j+8;
     }
     
     diz[48]=rxmsg[rxmsglen-2];
     
     val=((0xFFFF & rxmsg[2])<<8) | ((0xFFFF & rxmsg[1]));      
  
     myprintf("%4d\t", rxmsg[0]);
     myprintf2("%4d\t", rxmsg[0]);
     myprintf("%4d\t", val);
     myprintf2("%4d\t", val);
    
     q=0;
     
    for(i=3; i<rxmsglen-15; i++){  
        if(diz[q]==0){val8=rxmsg[i];
                        val=val+val8;
                        myprintf("%4d\t", val);
                        myprintf2("%4d\t", val);
                        q++;
        }
        
        else{ scale=((0xFFFF & rxmsg[i+1])<<8) | ((0xFFFF & rxmsg[i]));           
              val=val+scale; 
              i++;
              q++;
              myprintf("%4d\t", val);
              myprintf2("%4d\t", val);
        
        }               
    
      }
     
    // Stampo la temperatura
    myprintf("%4d\t", rxmsg[rxmsglen-15]);
    myprintf2("%4d\t", rxmsg[rxmsglen-15]);
    
    // Stampo i dati dell'accelerometro ricostruendoli a 16bit   
    uint16_t ax = (uint16_t) ((rxmsg[rxmsglen-14] << 8) | rxmsg[rxmsglen-13] );
    myprintf("%5d\t", ax);
    myprintf2("%5d\t", ax);
    uint16_t ay = (uint16_t) ((rxmsg[rxmsglen-12] << 8) | rxmsg[rxmsglen-11] );
    myprintf("%5d\t", ay);
    myprintf2("%5d\t", ay);
    uint16_t az = (uint16_t) ((rxmsg[rxmsglen-10] << 8) | rxmsg[rxmsglen-9] );
    myprintf("%5d\t", az);
    myprintf2("%5d\t", az);
    
  
     
 //   myprintf("%4d\t", CRC_Tx);

  //  myprintf("%4d\t", CRC_val);    
  
      TN100_LED0_OFF();      
      myprintf2("\n");
      myprintf("\n");
    }
    
    else{
      myprintf("%4d\t", 999);
      TN100_LED0_OFF();
      myprintf("\n");
    }
    
    }
  }
}
