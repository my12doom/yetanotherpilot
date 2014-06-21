/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.1.1
* Date               : 04/07/2010
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t buffer_out[512];
__IO uint32_t count_out = 0;

char buffer_response[512];
int response_size = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
}

extern int parse_command_line(const char *line, char *out);

static char* parse_command(char *cmd)
{
	int response_size;
	char response[200];
	do
	{
		char *next = (char*)strchr(cmd, '\n');
		if (next)
			next[0] = 0;

		if (NULL == cmd[0])
			break;
		
		response_size = parse_command_line(cmd, response);
		/* Write the data to the USB endpoint */
		printf("cmd:%s, response: %s(%d byte)\n", cmd, response, response_size);
		USB_SIL_Write(EP1_IN, (uint8_t*)response, response_size);
#ifndef STM32F10X_CL
		SetEPTxValid(ENDP1);
#endif /* STM32F10X_CL */
		
		if (!next)
			break;
		cmd = next+1;
		
		
	}while (1);

	return cmd;
}

/*******************************************************************************
* Function Name  : EP3_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	char *end;
	
  // Get the received data buffer and update the counter
  count_out += USB_SIL_Read(EP3_OUT, buffer_out+count_out);
	buffer_out[count_out] = NULL;
	
#ifndef STM32F10X_CL
  // Enable the receive of data on EP3
  SetEPRxValid(ENDP3);
#endif /* STM32F10X_CL */
	
	// line handling and coping left to head of buffer
	end = parse_command((char*)buffer_out);
	if (end > (char*)buffer_out)
	{
		int left = strlen(end);
		memmove(buffer_out, end, left+1);
		count_out = left;
	}
}


void EP4_IN_Callback(void)
{
}
void EP6_OUT_Callback(void)
{
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

