#ifndef __LED_H
#define __LED_H

#include "dz_tn.h"

#ifdef __cplusplus
 extern "C" {
#endif

   
 /** 
* @brief  set LED0 off   
*/  
#define LED0_OFF()     TN100_LED0_OFF()
/** 
  * @brief  set LED0 on   
  */ 
#define LED0_ON()    TN100_LED0_ON()
void LED_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */