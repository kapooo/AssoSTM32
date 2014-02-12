/* TX Device 2 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "usart.h"
#include "tn100_lib.h"
#include "timer.h"
#include "led.h"
#include <stdio.h>
#include <string.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_crc.h"
#include "ECG_Acquisition.h"
#include "dz_tn.h"
#include "stm32f10x_i2c.h"
//#include "LSM303DLHC.h"
#include <stdint.h>
#include <math.h>
//#include "stm32_dsp.h"
//#include "lsm303dlhc_driver.h"


#define MSG_LEN 128
#define M_PI 3.14159
#define PA_EN 0

#define NUM_SAMP 50

#define NUM_SAMP_ECG (NUM_SAMP/2)

#define n_section 2

#define n_coef (4*n_section + 1)  /*number of coefficients*/

// Il PIN D03 del TN100 è connesso al PIN 1 del GPIOA del microcontrollore (vedi datasheet DiZiC)
#define TN100_DIIO_3_STROBE() {GPIO_SetBits(TN100_DIIO3_GPIO_PORT, TN100_DIIO3_PIN); GPIO_ResetBits(TN100_DIIO3_GPIO_PORT, TN100_DIIO3_PIN); GPIO_SetBits(TN100_DIIO3_GPIO_PORT, TN100_DIIO3_PIN); }

#define TN100_DIIO3_PIN                   GPIO_Pin_1                  /* PA.01 */

#define TN100_DIIO3_GPIO_PORT             GPIOA                       /* GPIOA */

#define TN100_DIIO3_GPIO_CLK             RCC_APB2Periph_GPIOA

#define MAX_INPUT_LEN   NUM_SAMP_ECG

//#define NUM_SAMP_HBR      150
#define NUM_SAMP_RR       300

//#define GAIN1 9.9563e-5
//#define GAIN1    4.9712e-6  // LPF 5 Hz
//#define GAIN1    9.9930e-5    // LPF 1 Hz
//#define GAIN1    9.9880e-5      // LPF 2 Hz
#define GAIN1   1.3453e-6 



// array to hold input samples

//long xv[5];
double xv[5];
//long yv[5]
double yv[5];

float *p_coef;
uint16_t *p1_in;
uint16_t *p1_out;
float *p1_his;
float *p2_his;
uint16_t *p2_in;
uint16_t *p2_out;
uint16_t *p3_in;
uint16_t *p3_out;

int cyc[2];
char buffer[16]="          ";
volatile unsigned int *DWT_CYCCNT   =  (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  =  (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR    =  (volatile unsigned int *)0xE000EDFC; //address of the register

uint8_t mems[6];

#define STOPWATCH_START { cyc[0] = *DWT_CYCCNT; }
#define STOPWATCH_STOP { cyc[1] = *DWT_CYCCNT; cyc[1] = cyc[1] - cyc[0]; }

#warning RANGING_MASTER IS SELECTED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
TIM_OCInitTypeDef         TIM_OCInitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
//void GPIO_Configuration(void);
void InitTIM1(void);
void InitTIM2(void);
void LED_Init1(void);

void Filter3( short *pSrc, short len, uint16_t *pDst);
void InitACC(void);
float iir_filter(float input, float *coef, int n, float *history);

/* Private functions ---------------------------------------------------------*/

/**

  * @brief  Main program.

  * @param  None

  * @retval None

  */

  int main(void)

{
  *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter

  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  // Activate I2C1 clock.
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  /* GPIO configuration ------------------------------------------------------*/
  //GPIO_Configuration();
  
  /* TIM1 configuration ------------------------------------------------------*/
  InitTIM1();
  //InitTIM2();
  LED_Init1();
  
  /* Accelerometer Configuration ----------------------------------------------*/
  //InitACC();
  
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SampleBuff1[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = SampleBuffSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; //TIMER1 COMANDA ADC1!!!!!!!!
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NUM_ADC;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_28Cycles5);   // Canale ECG1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);   // Canale ECG2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);   // Canale Temperatura 
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
   /* Enable ADC1 external trigger */
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
   /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  TN100_InitTypeDef  TN100_InitStructure;

  uint8_t senderID[6];

  uint8_t receiverID[6];

  uint8_t packetType;

  uint8_t rxmsg[5];

  uint16_t rxmsglen;

  uint32_t rangingtimer = 0;  
    
  uint16_t Buffer_Tx[NUM_SAMP+8];
  
  uint16_t *pBuffer_Tx;
  
  int8_t Buff_Comp[116]; //Buffer compresso  
    
  uint8_t *pBuff_Comp, num_loc, y, q, diz[49], CRC_val;
  
  int16_t Buff_app[NUM_SAMP];     
  
  uint16_t Buffer3_in[NUM_SAMP_ECG];
  uint16_t Buffer3_out[NUM_SAMP_ECG];
  
  uint16_t sample, max1, max2, imax1, imax2, Buff[58];
  
  uint16_t m = 0; 
  
  uint16_t n = 0;
  
  uint16_t Temp;
  
  int num_pacchetto=0;
  
  int i, step; int j=1; int k;
  
  int RR_Fill = 0;
  
//  float iir_coef[] = {GAIN1, -1.994468725910304, 0.99463933704617047, -1.9983537046882773, 1, -1.9916039050017902, 0.99164560916623756, -1.9886914599162702, 1, -1.9979953458398652, 0.99827402341158644, -1.9990466012362298, 1};
//  float iir_coef[] = {GAIN1, -1.9985564208306903, 0.99862589632961685, -1.9995236407524317, 1, -1.996067915503039, 0.99609980456504865, -1.9988120476320312, 1, -0.99746047624731959, 0, 1, 0};  // LPF 5 Hz
//  float iir_coef[] = {GAIN1, -1.9988611276805377, 0.9988619329900803, -1.9996627164949214, 1, -1.9995421854793711, 0.99954495837831336, -1.9999409520041804, 1};  // LPF 1 Hz
//  float iir_coef[] = {GAIN1, -1.9977219408097229, 0.99772516021851665, -1.9986512052812304, 1, -1.9990790359606454, 0.9990901250259242, -1.9997638181447535, 1};  // LPF 2 Hz
float iir_coef[] = {GAIN1, -1.9995869592235613, 0.99958765723850362, -1.9998923257509362, 1, -0.99958496672120145, 0, 1, 0};

/* Inizializzazione del Timer */

  Timer_Init();

  /* Inizializzazione dell'interfaccia seriale */

  usart_init(230400);    // Init usart with 230400 Baud

  /* Unlock the Flash Program Erase controller */

  FLASH_Unlock();

  /*Inizializzazione del LED */

  /*Configurazione del TN100 */

  TN100_InitStructure.srcId[0] = 0x02;    // Source address

  TN100_InitStructure.srcId[1] = 0x00;

  TN100_InitStructure.srcId[2] = 0x00;

  TN100_InitStructure.srcId[3] = 0x00;

  TN100_InitStructure.srcId[4] = 0x00;

  TN100_InitStructure.srcId[5] = 0x00;

  TN100_InitStructure.destId[0] = 0x04;   // Destination address

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

  TN100_InitStructure.chNo = E1_2412MHZ;    

  // -> Originale per Ranging: TN100_InitStructure.mode = TN100_80MHz_1MS_1us;
  //TN100_InitStructure.mode = TN100_80MHz_500kS_2us;
  TN100_InitStructure.mode = TN100_80MHz_250kS_4us;
  //TN100_InitStructure.mode = TN100_22MHz_1MS_1us;
  //TN100_InitStructure.mode = TN100_22MHz_500kS_2us;
  //TN100_InitStructure.mode = TN100_22MHz_250kS_4us;
 
  // La potenza in trasmissione va da 1.79 dBm (scrivere nel registro 63 ovvero 0x3F in esadecimale)
  // a -36.20 dBm (ovvero 0 nel registro 0x00) (vedi datasheet TN100 pag. 96)
  TN100_InitStructure.txpwr = 0x28; // 40 ovvero -7.31 dBm 

  TN100_InitStructure.txArq = 0x03;  

  TN100_InitStructure.txArqMode = 0x01; 

  TN100_InitStructure.rxArqMode = TN100_RxArqModeCrc2;

  TN100_InitStructure.addrMatching = ON;

#ifdef PA_EN

#warning PA is on

  TN100_PA_Init(TN100_PA_SMD_ANT); // Abilito l'antenna SMD sulla scheda DiZic

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

  rangingtimer = GetSysTick();    
  
  //valori massimi e minimi di accelerometro e magnetometro
  int16_t ax_max=1024;
  int16_t ay_max=1024;
  int16_t az_max=1024;
  int16_t ax_min=-1024;
  int16_t ay_min=-1024;
  int16_t az_min=-1024;
  int16_t mx_max = 101;
  int16_t mx_min = -104;
  int16_t my_max = 80;
  int16_t my_min = -147;
  int16_t mz_max = 79;
  int16_t mz_min = -103;
  
  /*libreria ufficiale: deinizializza tutto ciò che riguarda la i2c1 tutti i registri*/
  I2C_DeInit(I2C1);
  LSM303DLHC_I2C_InitialConfig(I2C1);
  /*abilita pb6 e pb7 descritti nella documentazione*/
  GPIOB->CRL=0xFF444444; //per i2c1 su pb6 e pb7
  /*configurazione LSM303DLHC.c dell'accelerometro e magnetometro*/
  LSM303DLHC_I2C_Accelerometer_Config(I2C1);
  LSM303DLHC_I2C_Magnetometer_Config(I2C1);
  delay(0x0FFF);
  
  //ECGInit();
  NVIC_Configuration();
  
   // initialize the filter
   // firFixedInit();
    
    //TN100_Callibration();
  
    GPIO_SetBits(GPIOC, GPIO_Pin_15); // led2 on
    
   // initialize the filter buffer
   xv[0] = xv[1] = xv[2] = xv[3] = xv[4] = 0x3F;
   yv[0] = yv[1] = yv[2] = yv[3] = yv[4] = 0x3F;   

   m = max1 = max2 = imax1 = imax2 = 0;
   

   while(1){

    
   // TN100_Callibration();
    
  //  rxmsglen = TN100_is_Msg_Received(&rxmsg[0], senderID, receiverID, &packetType);  
     
    /*faccio le letture x y z dell'accellerometro e magnetometro*/
    buffer[0]=LSM303DLHC_I2C_Accelerometer_ReadDataAXL(I2C1);
    buffer[1]=LSM303DLHC_I2C_Accelerometer_ReadDataAXH(I2C1);
      // Prendo gli 8bit (MSB e LSB), li unisco per ricreare i 16bit del accelerometro
      // ci tolgo 2^16 cosi da ottenere un numero basso ad accelerometro fermo e 65536
      // in movimento (originariamente è all'incontro)
     // mems[0] = 65536 -( (uint16_t) ((buffer[0] << 8) | buffer[1]) );
         mems[0] = buffer[0];
         mems[1] = buffer[1];
    buffer[2]=LSM303DLHC_I2C_Accelerometer_ReadDataAYL(I2C1);
    buffer[3]=LSM303DLHC_I2C_Accelerometer_ReadDataAYH(I2C1);
     // mems[1] = 65536 -( (uint16_t) ((buffer[2] << 8) | buffer[3] ));
      mems[2] = buffer[2];
      mems[3] = buffer[3];
    buffer[4]=LSM303DLHC_I2C_Accelerometer_ReadDataAZL(I2C1);
    buffer[5]=LSM303DLHC_I2C_Accelerometer_ReadDataAZH(I2C1);
    //  mems[2] = 65536 -( (uint16_t) ((buffer[4] << 8) | buffer[5] ));
      mems[4] = buffer[4];
      mems[5] = buffer[5];
    buffer[6]=LSM303DLHC_I2C_Magnetometer_ReadDataMXL(I2C1);
    buffer[7]=LSM303DLHC_I2C_Magnetometer_ReadDataMXH(I2C1);
    //  mems[3] = 65536 -( (uint16_t) ((buffer[6] << 8) | buffer[7] ));
    buffer[8]=LSM303DLHC_I2C_Magnetometer_ReadDataMYL(I2C1);
    buffer[9]=LSM303DLHC_I2C_Magnetometer_ReadDataMYH(I2C1);
    //  mems[4] = 65536 -( (uint16_t) ((buffer[8] << 8) | buffer[9] ));
    buffer[10]=LSM303DLHC_I2C_Magnetometer_ReadDataMZL(I2C1);
    buffer[11]=LSM303DLHC_I2C_Magnetometer_ReadDataMZH(I2C1);
    //  mems[5] = 65536 -( (uint16_t) ((buffer[10] << 8) | buffer[11]) );
    
    
    // Aggiusto i dati dell'accelerometro secondo la notazione 12-bit left-justified big endian
    double ax = ((int16_t)(( (buffer[0]<<8) | buffer[1] )))/16;
    double ay = ((int16_t)(( (buffer[2]<<8) | buffer[3] )))/16;
    double az = ((int16_t)(( (buffer[4]<<8) | buffer[5] )))/16;
    
    // Aggiusto i dati del Magnetometro secondo la notazione 12-bit right-justified little endian
    double mx = ((int16_t)(( (buffer[7]<<8) | buffer[6] )));
    double my = ((int16_t)(( (buffer[9]<<8) | buffer[8] )));
    double mz = ((int16_t)(( (buffer[11]<<8) | buffer[10] )));
    
    // normalizzo i dati utilizzando i massimi ed i minimi, i risultati vanno da -1 a +1
    double ax_n = ((ax - ax_min) / (ax_max - ax_min)) * 2 - 1;
    double ay_n = ((ay - ay_min) / (ay_max - ay_min)) * 2 - 1;
    double az_n = ((az - az_min) / (az_max - az_min)) * 2 - 1;
    double mx_n = ((mx - mx_min) / (mx_max - mx_min)) * 2 - 1;
    double my_n = ((my - my_min) / (my_max - my_min)) * 2 - 1;
    double mz_n = ((mz - mz_min) / (mz_max - mz_min)) * 2 - 1;
    
    // calcolo pitch e roll del piano orizzontale
    double pitch = asin(-ax_n);
    double roll = asin(ay_n / cos(pitch));
    
    //formule per calcolare l'angolo in base all'inclinazione
    double xh = mx_n * cos(pitch) + mz_n * sin(pitch);
    double yh = mx_n * sin(roll) * sin(pitch) + my_n * cos(roll) - mz_n * sin(roll) * cos(pitch);
    
    //angolo sfruttando l'accelerometro
    double heading = (180 * atan2(yh, xh)/M_PI);
    
    //angolo considerando pitch e roll uguali a zero
    double headingZero = (180 * atan2(my_n, mx_n) / M_PI);
    
    //per avere valori da 0 a 360 e non da -180 a +180
    if (yh < 0)
      heading += 360;
    
    //per avere valori da 0 a 360 e non da -180 a +180
    if (headingZero < 0)
      headingZero += 360;
    
    //inserisce i valori aggiustati utilizzando le notazioni precedenti nel buffer da inviare
    *(int16_t*)&(buffer[0]) = (int16_t)ax;
    *(int16_t*)&(buffer[2]) = (int16_t)ay;
    *(int16_t*)&(buffer[4]) = (int16_t)az;
    
    *(int16_t*)&(buffer[6]) = (int16_t)mx;
    *(int16_t*)&(buffer[8]) = (int16_t)my;
    *(int16_t*)&(buffer[10])= (int16_t)mz;
    
    //inserisco nel buffer i valori dell'angolo e dell'angolo con piano orizzontale nullo
    *(int16_t*)&(buffer[12]) = (int16_t)heading;
    *(int16_t*)&(buffer[14]) = (int16_t)headingZero; 
    

    
    pBuffer_Tx = &Buffer_Tx[0];
    
    pBuff_Comp = &Buff_Comp[0];
    

    if (Buffer_Ready==1)
    { step=0; 
      if(pSampleBuff2==&SampleBuff2[0]) 
        k=0;
      else 
        k=DATA_BUFF_SIZE;
      
      if(j<=25)
      { 
            Buffer_Tx[j] = SampleBuff2[k+step];
            
      
        
            p3_in = &Buffer3_in[0];
            p3_out = &Buffer3_out[0];
            for(i=0; i<DATA_BUFF_SIZE; i++)
            {
              if((i%3) == 2) Buffer3_in[i/3]=SampleBuff2[i+k]; // Fill Temperature Buffer
            }
            Filter3(p3_in, NUM_SAMP_ECG, p3_out);

            Temp=Buffer3_out[0]; // Decimation
          
       
            Buffer_Tx[j+25]=SampleBuff2[k+1+step];              
            j++;
            step=step+3;       
          }
      else if(j>25)      
      {  
        
        
    num_loc=2;
    Buff_app[0]= Buffer_Tx[1];
    
    for(i=1; i<NUM_SAMP; i=i+1){
      Buff_app[i]= Buffer_Tx[i+1]-Buffer_Tx[i];
    }
    
     for(i=1; i<NUM_SAMP; i=i+1){
      if(Buff_app[i]>127){diz[i-1]=1;
                          num_loc=num_loc+2;}
      
      else if(Buff_app[i]<-128){diz[i-1]=1;
                                num_loc=num_loc+2;}
      
      else {diz[i-1]=0;
            num_loc=num_loc+1;}
    }
    
    y=3;    
    
    Buff_Comp[1]=Buff_app[0];
    Buff_Comp[2]=(0xFFFF & Buff_app[0])>>8;
    
    for(i=0; i<49; i=i+1){
      if(diz[i]==0){Buff_Comp[y]=Buff_app[i+1];       
                    y++;}
      
      else{Buff_Comp[y]=Buff_app[i+1];
           Buff_Comp[y+1]=(0xFFFF & Buff_app[i+1])>>8;
           y=y+2;}
    }    
   
    Buff_Comp[num_loc+1]=Temp;   
    // Accelerometro X a 16bit con 8bit (LSB) in mems[0] e 8bit (MSB) in mems[1]
    Buff_Comp[num_loc+2]=mems[0];
    Buff_Comp[num_loc+3]=mems[1];
    // Accelerometro Y
    Buff_Comp[num_loc+4]=mems[2];
    Buff_Comp[num_loc+5]=mems[3];
    // Accelerometro Z
    Buff_Comp[num_loc+6]=mems[4];
    Buff_Comp[num_loc+7]=mems[5];
    
    Buff_Comp[0]=0x00FF & num_pacchetto;
    
    q=0;
    
    for(i=0; i<41; i=i+8){
      Buff_Comp[num_loc+8+q]=((diz[i]&1)<<7)|((diz[i+1]&1)<<6)|((diz[i+2]&1)<<5)|((diz[i+3]&1)<<4)|((diz[i+4]&1)<<3)|((diz[i+5]&1)<<2)|((diz[i+6]&1)<<1)|(diz[i+7]&1);   
      q++;
    } 
      Buff_Comp[num_loc+14]=diz[48];
    
  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);     
       
    CRC->CR = 0x00000001;    
    
    CRC_val=CRC_CalcBlockCRC((uint32_t *)pBuff_Comp, ((num_loc+15)/4));        
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,DISABLE); 
    
    Buff_Comp[num_loc+15]=CRC_val;
     
        
        GPIO_SetBits(GPIOC, GPIO_Pin_14);    // led1 on

        // Attendo che il TN100 si svegli. Per svegliarlo cambio lo stato al pin DII0_3 (D03)
          while(TN100_getWUState() != 1)
          {
            TN100_DIIO_3_STROBE();
          }
          Delay_ms(2);
        // Lo reinizializzo ed invio il dato  
          TN100_Init(TN100_INIT_FULL, &TN100_InitStructure); 
          TN100_Send_Data(&TN100_InitStructure.destId[0], (uint8_t *)pBuff_Comp, (num_loc+16)); 
        // Ad ogni ciclo ED UNA SOLA VOLTA per ciclo devo calibrarlo
        //  TN100_Callibration();    
        //  rxmsglen = TN100_is_Msg_Received(&rxmsg[0], senderID, receiverID, &packetType);  
        // Mando il Tn100 in sleep  
          TN100_Sleep(TN100_WAKEUPDIIO, TN100_PD_FULL, TN100_DIIO_3, NULL);
  
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);  // led1 off
        
      //  STOPWATCH_STOP;    
      //  STOPWATCH_START;        
        
        j=1;
        
        num_pacchetto++;
        
        Buffer_Ready = 0;        
      }   
    }
  }
}

///**
//  * @brief  Configures the different system clocks.
//  * @param  None
//  * @retval None
//  */
void RCC_Configuration(void)
{
  /* ADCCLK = PCLK2/2 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4);
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
  /* Enable ADC1 and GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */


void InitTIM1(void)
{
  //int prescaleValue = 2;            // prescale = 3
  //int prescaleValue = 11;            // prescale = 12
  int prescaleValue = 3;            // prescale = 4

  TIM_TimeBaseStructure.TIM_Prescaler = prescaleValue;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //[0x0640 = 1600 // 72000000 / 1600 / 4 = 11250 // 11250 / 3 / 25 = 150 sample/s]
  TIM_TimeBaseStructure.TIM_Period = 0x9C40; // 0x9C40 = 40000 //  72000000 / 40000 / 4 = 450 --> 450 / 3 = 150 sample/s
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  // Channel1 Configuration in PWM mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
  TIM_OCInitStructure.TIM_Pulse = 0x4E20;  // 0x4E20 = 0x9C4 / 2  [0x0320 = 0x0640 / 2]
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  
  //TIM_OC1Init(&TIM1_OCInitStructure);
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
   
  // TIM1 counter enable
  TIM_Cmd(TIM1, ENABLE);
     
  // TIM1 main Output Enable
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void LED_Init1(void)

{

  GPIO_InitTypeDef  GPIO_InitStructure;

  /* init LED */

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

// Chebyshev LPF
// The filtered result are copied in Buffer3_out
void Filter3(short *pSrc, short len, uint16_t *pDst)
{
      //short *pDst = &Buff3[0];

      // Average data
      short *pEnd = pSrc + len;
      short value;
//      while(pSrc < pEnd)
//      {
//        // average 4 data
//        value = *pSrc++;
//        value += *pSrc++;
//        value += *pSrc++;
//        value += *pSrc++;
//        // clamp to maximum
//        if( value > 16384)
//           value = 16384;
////#ifdef CHEBYSHEV_LOWPASS_FILTER_30
////        *pDst++ = value>>9;                          // convert data from range of 4096 to range of 128
////#else
       //*pDst++ = value>>7;                          // convert data from range of 4096 to range of 128
////#endif
//      }
   
      short i;
      long val;
      //len /= 4;
      
      //pSrc = pDst= &DisplayBuff[0];
      //pSrc = pDst;
      #define GAIN   3.283894280e+09
      for(i=0; i<len; ++i)
      {
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
        xv[4] = *pSrc++ / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
        yv[4] =  (1 * (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6* xv[2]
                     + ( -0.9783461376 * yv[0]) + (  3.9348021535 * yv[1])
                     + ( -5.9345643792 * yv[2]) + (  3.9781083585 * yv[3]));
//#ifdef FLOATING_POINT_PROCESS
//                  );
//#else
//                  ) >> 4;
//#endif
        val = yv[4];
//        if(val > 109)
//           *pDst++ = 109;
//        else if(val < 0)
//           *pDst++ = 0;
//        else
//           *pDst++ = val;
        *pDst++ = ((uint16_t) val) >> 4;
      }
}

// LPF

/************************************************************

iir_filter - PerformIIR filtering sample by sample on floats

Implements cascaded direct form II second order sections.
Requires arrays for history and coefficients.
The length (n) of the filter specifies the number of sections.
The size of the history array is 2+n.
The size of the coefficient array is 4*n + 1 because the first 
coefficient is the overall scale factor for the filter.
Returns one output sample for each input sample.

float iir_filter(float input, float *coef, int n, float *history)
  
  float input
  float *coeff      pointer to filter coefficients
  int n             number of sections in filter
  float *history    history array pointer

Returns float value giving the current output

(Source: Embree p. 146)

************************************************************/

float iir_filter(float input, float *coef, int n, float *history)
{
  int i;
  float *hist1_ptr, *hist2_ptr, *coef_ptr;
  float output, new_hist, history1, history2;

  coef_ptr = coef;  /* coefficient pointer */
  
  hist1_ptr = history;    /* first history */
  hist2_ptr = hist1_ptr + 1;  /*next history */
  
  output = input * (*coef_ptr++);   /* overall input scale factor */
  
  for(i=0; i<n; i++)
  {
    history1 = *hist1_ptr;
    history2 = *hist2_ptr;    /* history values */
    
    output = output - history1 * (*coef_ptr++);
    new_hist = output - history2 * (*coef_ptr++);   /* poles */
    
    output = new_hist + history1 * (*coef_ptr++);
    output = output + history2 * (*coef_ptr++);   /* zeros */
    
    *hist2_ptr++ = *hist1_ptr;
    *hist1_ptr++ = new_hist;
    
    hist1_ptr++;
    hist2_ptr++;
  }
  return(output);
}

