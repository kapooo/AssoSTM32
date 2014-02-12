

#include "stm32f10x.h"

#include "dz_tn.h"

#include "usart.h"

#include <stdio.h>

#include <string.h>

#include <stdarg.h>





/** @defgroup DZ_TN_LOW_LEVEL_Private_Variables

  * @{

  */ 

static	char	txbuf[80];



USART_InitTypeDef USART_InitStructure;

USART_InitTypeDef USART2_InitStructure;

 

/**

  * @brief  Configures COM port.

  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that

  *   contains the configuration information for the specified USART peripheral.

  * @retval None

  */

void DZ_TN_COMInit(USART_InitTypeDef* USART_InitStruct)

{

  GPIO_InitTypeDef GPIO_InitStructure;



  /* Enable GPIO clock */

  RCC_APB2PeriphClockCmd(DZ_TN_COM1_TX_GPIO_CLK | DZ_TN_COM1_RX_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);



  RCC_APB2PeriphClockCmd(DZ_TN_COM1_CLK, ENABLE); 



  /* Configure USART Tx as alternate function push-pull */

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_InitStructure.GPIO_Pin = DZ_TN_COM1_TX_PIN;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(DZ_TN_COM1_TX_GPIO_PORT, &GPIO_InitStructure);



  /* Configure USART Rx as input floating */

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

  GPIO_InitStructure.GPIO_Pin = DZ_TN_COM1_RX_PIN;

  GPIO_Init(DZ_TN_COM1_RX_GPIO_PORT, &GPIO_InitStructure);



  /* USART configuration */

  USART_Init(DZ_TN_COM1, USART_InitStruct);

    

  /* Enable USART */

  USART_Cmd(DZ_TN_COM1, ENABLE);

}





/**

  * @brief  Configures COM port.

  * @param  baud: Specifies the baudrate to be configured.

  * @retval None

  */

void usart_init(u32 baud)

{

  /* USARTx configured as follow:

        - BaudRate =  baud  

        - Word Length = 8 Bits

        - One Stop Bit

        - No parity

        - Hardware flow control disabled (RTS and CTS signals)

        - Receive and transmit enabled

  */

  USART_InitStructure.USART_BaudRate = baud;

  USART_InitStructure.USART_WordLength = USART_WordLength_8b;

  USART_InitStructure.USART_StopBits = USART_StopBits_1;

  USART_InitStructure.USART_Parity = USART_Parity_No;

  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;



  DZ_TN_COMInit(&USART_InitStructure);

}



/**

  * @brief  Configures COM port USART2

  * @param  baud: Specifies the baudrate to be configured.

  * Aggiunto da AC per abilitare USART2

  * @retval None

  */

void usart2_init(u32 baud)

{

  /* USARTx configured as follow:

        - BaudRate =  baud  

        - Word Length = 8 Bits

        - One Stop Bit

        - No parity

        - Hardware flow control disabled (RTS and CTS signals)

        - Receive and transmit enabled

  */

  USART2_InitStructure.USART_BaudRate = baud;

  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;

  USART2_InitStructure.USART_StopBits = USART_StopBits_1;

  USART2_InitStructure.USART_Parity = USART_Parity_No;

  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

  /* Configure USART Tx as alternate function push-pull */

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //USART2 TX

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);



  /* Configure USART Rx as input floating */

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // USART2 RX

  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* USART configuration */

  USART_Init(USART2, &USART2_InitStructure);

    

  /* Enable USART */

  USART_Cmd(USART2, ENABLE);

}





/**

 * SerialPutChar1:

 * @c: -input- the byte to be sent

 *

 * SerialPutChar1() transmits the @c byte via the console interface (USART1).

 *

 * Returns: none

 */

void SerialPutChar(u8 c)

{

	USART_SendData(DZ_TN_COM1, c);

	while (USART_GetFlagStatus(DZ_TN_COM1, USART_FLAG_TC) == RESET);

}



/**

  * Aggiunto da AC per stampare su USART 2
  
  *

**/

void SerialPutChar2(u8 c)

{

	USART_SendData(USART2, c);

	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

}



/**

 * SerialPutString1:

 * @s: -input- the byte stream to be sent

 *

 * SerialPutString1() transmits the byte stream @s via the console interface (USART1).

 *

 * Returns: none

 */

void SerialPutString(u8 *s)

{

	while (*s != '\0')

	{

		if (*s == '\n' ||*s == '\r')

                {

                  SerialPutChar('\r');

                  SerialPutChar('\n');

                  break;

                }

		SerialPutChar(*s);

		s ++;

	}

}



/**

  * Aggiunto da AC per stampare su USART 2
  
  *

**/

void SerialPutString2(u8 *s)

{

	while (*s != '\0')

	{

		if (*s == '\n' ||*s == '\r')

                {

                  SerialPutChar2('\r');

                  SerialPutChar2('\n');

                  break;

                }

		SerialPutChar2(*s);

		s ++;

	}

}



/**

 * myprintf:

 * @fmt: -input- the formtated byte stream to be sent

 *

 * printf() transmits the formated byte stream @fmt via the console interface

 *

 * Returns: Number of bytes send

 */

int myprintf(const char *fmt, ...)

{

	int		ret = 1;

	va_list	args;



	va_start(args, fmt);

	ret = (int) vsprintf(txbuf,fmt, args);

	SerialPutString((u8 *)txbuf);

	va_end(args);

	return	ret;

}


/**

 * myprintf:

 * @fmt: -input- the formtated byte stream to be sent

 *

 * printf() transmits the formated byte stream @fmt via the console interface

 * Aggiunta da AC. Stampa su USART2

 * Returns: Number of bytes send

 */

int myprintf2(const char *fmt, ...)

{

	int ret = 1;

	va_list	args;

	va_start(args, fmt);

	ret = (int) vsprintf(txbuf,fmt, args);

	SerialPutString2((u8 *)txbuf);

	va_end(args);

	return	ret;

}



/**

 * @brief  Test to see if a key has been pressed on the HyperTerminal

 * @param  key: The key pressed

 * @retval 1: Correct

 *         0: Error

 */

u8 SerialKeyPressed(u8 *key)

{

  if ( USART_GetFlagStatus(DZ_TN_COM1, USART_FLAG_RXNE) != RESET)

  {

    *key = (uint8_t)DZ_TN_COM1->DR;

    return 1;

  }

  else

  {

    return 0;

  }

}



/**

 * @brief  Get a char from the HyperTerminal

 * @param  None

 * @retval The Key Pressed

 */

u8 SerialGetChar(void)

{

  u8 key = 0;



  /* Waiting for user input */

  while (1)

  {

    if (SerialKeyPressed((u8*)&key)) break;

  }

  return key;

}



/**

 * @brief  Get a key from the HyperTerminal

 * @param  None

 * @retval The Key Pressed

 */

void SerialGetString(u8 *string, u8 len )

{

  /* Waiting for user input */

  while(len--)

  {

    *string = SerialGetChar();

    SerialPutChar(*string);

    if (*string == '\n' || *string == '\r')

    {

      SerialPutChar('\n');

      break;

    }

    string ++;

  }  

}

