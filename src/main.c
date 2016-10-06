/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file provides main program functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "hw_config.h"
#include <stdio.h>
#include "stm32f10x_crc.h"
#include "spi.h"
#include "poti.h"
#include "channelled.h"
#include "buttons.h"
#include "siebensegment.h"
#include "messagequeue.h"
#include "systemevent.h"

/** @addtogroup Embedded_GUI_Example
  * @{
  */

/** @defgroup Main
  * @brief Main program body
  * @{
  */

/** @addtogroup Embedded_GUI_Example
  * @{
  */

/** @defgroup Main
  * @brief Main program body
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#if defined(USE_STM32100E_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08080000)
#elif defined(USE_STM322xG_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08100000)
#elif defined(USE_STM3210C_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08040000)
#endif
/* Private macros ------------------------------------------------------------*/

/**
  * @brief   Small printf for GCC/RAISONANCE
  */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#endif /* __GNUC__ */

/* Public variables ---------------------------------------------------------*/
volatile uint16_t systemevent;
volatile uint8_t GL_led_displayreg[14];


/* Private function prototypes -----------------------------------------------*/
void Delay_us(int);
void Timer_Setup(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


int main(void)
{
	uint8_t i, j;
	volatile uint8_t tmp;
	uint16_t potivalvector[15];





	/* Startup code for clock etc ... */
	RCC_Configuration();

	/* Device specific NVIC Configuration */
	NVIC_Configuration();

	/* Start to initialize some variables */
	systemevent=0;
	GL_led_displayreg[11]=BLINKCHANNEL;
	GL_spi_send_call=0;
	GL_spi_irq_call=0;

	for(i=0;i<14;i++)
		GL_led_displayreg[i]=1;
	/*
	 * Before initializing the SPI hardware setup each SPI hardware part
	 * thus all Chip Select lines should be in their default state up on SPI Bus
	 * initialization.
	 */
	poti_led_init();
	channel_led_init();
	siebensegment_init();


	/* Now its time to fire up the SPI bus */
	SPI1_BusInit();

	/*
	 * The seven-segment display has two modes, a SPI and a non SPI mode.
	 * In non SPI mode it listens to all messages on the SPI bus.
	 * Thus after bringing the SPI BUS up the first thing to do is
	 * configuring the seven segment controller to behave like a real spi device
	 */

	siebensegment_config();

	/* Next configure the I2C interface */
	I2C_InterruptInit();
	I2C_LowLevel_Init(I2C1);
	I2C_Slave_BufferReadWrite(I2C1);





	InitializeTimer();
	poti_encoder_init();
	pushbuttoninit();

	/*
	 * finally initialize the i2c message queue before going into the main loop
	 */
	messagequeue_init();
	/*
	 * Send welcome message to controller board to indicate the system is set up.
	 */
	//message_create(uint8_t msgtype, uint8_t value);

	/*
	 * Next is to trigger an interrupt for the host system and to wait
	 * for an i2c read that
	 */
	potivalvector[0]=0;
	potivalvector[1]=1;
	potivalvector[2]=3;
	potivalvector[3]=5;
	potivalvector[4]=8;
	potivalvector[5]=12;
	potivalvector[6]=16;
	potivalvector[7]=24;
	potivalvector[8]=34;
	potivalvector[9]=48;
	poti_value_init(10,potivalvector);

	/* Set the default values of the LEDs at the Rotary encoders and enable the LEDS*/
	while(poti_send_led_data(1,0,0)!=0);
	while(poti_send_led_data(2,0,0)!=0);
	while(poti_send_led_data(3,0,0)!=0);
	while(poti_send_led_data(4,0,0)!=0);
	GPIO_WriteBit(POTILED_BLANK_GPIO_PORT, POTILED_BLANK_PIN, Bit_RESET);







	/* Infinite main loop */
	while (1)
	{
		if(!(systemevent & SYSEV_PRIORITY_GROUP1))
		{
			if(systemevent & SYSEV_BLINKLEDON)
			{
				if(GL_spi1_block == SPI_BLOCK_FREE)
				{
					switch ( GL_led_displayreg[11] )
					{
					case BLINKRING1:
					case BLINKRING2:
						poti_send_led_data(1,GL_led_displayreg[0],GL_led_displayreg[1]);
						break;
					case BLINKRING3:
					case BLINKRING4:
						poti_send_led_data(2,GL_led_displayreg[2],GL_led_displayreg[3]);
						break;
					case BLINKRING5:
					case BLINKRING6:
						poti_send_led_data(2,GL_led_displayreg[4],GL_led_displayreg[5]);
						break;
					case BLINKRING7:
						poti_send_led_data(2,0,GL_led_displayreg[6]);
						break;
					case BLINKSEGMENT:
						siebensegment_send_data(GL_led_displayreg[8],GL_led_displayreg[9],GL_led_displayreg[10]);
						break;
					case BLINKCHANNEL:
						channel_send_led_data(GL_led_displayreg[7]);
						break;
					}
					systemevent= (systemevent & (~SYSEV_BLINKLEDON));
					continue;
				}
			}
			if(systemevent & SYSEV_BLINKLEDOFF)
			{
				if(GL_spi1_block == SPI_BLOCK_FREE)
				{
					switch ( GL_led_displayreg[11] )
					{
						case BLINKRING1:
							poti_send_led_data(1,0,GL_led_displayreg[1]);
							break;
						case BLINKRING2:
							poti_send_led_data(1,GL_led_displayreg[0],0);
							break;
						case BLINKRING3:
							poti_send_led_data(2,0,GL_led_displayreg[3]);
							break;
						case BLINKRING4:
							poti_send_led_data(2,GL_led_displayreg[2],0);
							break;
						case BLINKRING5:
							poti_send_led_data(2,0,GL_led_displayreg[5]);
							break;
						case BLINKRING6:
							poti_send_led_data(2,GL_led_displayreg[4],0);
							break;
						case BLINKRING7:
							poti_send_led_data(2,0,0);
							break;
						case BLINKSEGMENT:
							siebensegment_send_data(0x20,0x20,0x20);
							break;
						case BLINKCHANNEL:
							channel_send_led_data(0);
							break;
					}
					systemevent= (systemevent & (~SYSEV_BLINKLEDOFF));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHDISPLAY)
			{
				systemevent = (systemevent | SYSEV_REFRESHPOT1 | SYSEV_REFRESHPOT2 | SYSEV_REFRESHPOT3 | SYSEV_REFRESHPOT4 | SYSEV_REFRESHSEGMENT | SYSEV_REFRESHCHAN);
				systemevent= (systemevent & (~SYSEV_REFRESHDISPLAY));
			}
			if(systemevent & SYSEV_REFRESHPOT1)
			{
				if(poti_send_led_data(1,GL_led_displayreg[0],GL_led_displayreg[1]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHPOT1));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHPOT2)
			{
				if(poti_send_led_data(2,GL_led_displayreg[2],GL_led_displayreg[3]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHPOT2));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHPOT3)
			{
				if(poti_send_led_data(3,GL_led_displayreg[4],GL_led_displayreg[5]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHPOT3));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHPOT4)
			{
				if(poti_send_led_data(4,0,GL_led_displayreg[6]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHPOT4));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHCHAN)
			{
				if(channel_send_led_data(GL_led_displayreg[7]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHCHAN));
					continue;
				}
			}
			if(systemevent & SYSEV_REFRESHSEGMENT)
			{
				if(siebensegment_send_data(GL_led_displayreg[8],GL_led_displayreg[9],GL_led_displayreg[10]) == 0)
				{
					systemevent= (systemevent & (~SYSEV_REFRESHSEGMENT));
					continue;
				}
			}
			if( systemevent & SYSEV_READINPUTS )
			{
				read_encoder_ports();
				update_buttons();
				messagequeue_update_i2c_buffers();
				systemevent= (systemevent & (~SYSEV_READINPUTS));
				tmp=GPIO_ReadOutputDataBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN);
				tmp=GPIO_ReadOutputDataBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN);
				tmp=GPIO_ReadOutputDataBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN);
				tmp=GPIO_ReadOutputDataBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN);
				tmp=GPIO_ReadOutputDataBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN);
				tmp=GPIO_ReadOutputDataBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_OE_PIN);
				tmp=GPIO_ReadOutputDataBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN);
				continue;
			}
		}
		else // if(systemevent & SYSEV_PRIORITY_GROUP1)
		{
			if( systemevent & SYSEV_I2CRXCOMPLETE)
			{
				j=2;
				if(Buffer_Rx1[0] == I2C_MESSAGE_DISPLAYDATA)
				{
					for(i=Buffer_Rx1[1]; i<(Buffer_Rx1[1]+Rx_Idx1-2);i++)
					{
						if(i<12)
						{
							GL_led_displayreg[i]=Buffer_Rx1[j];
							j++;
						}
					}
					systemevent = (systemevent | SYSEV_REFRESHDISPLAY);
				}
				else if(Buffer_Rx1[0] == I2C_MESSAGE_ACKNO)
				{
					messagequeue_i2crx_handler();
				}
				systemevent= (systemevent & (~SYSEV_I2CRXCOMPLETE));
				systemevent = (systemevent | SYSEV_REFRESHDISPLAY);
			}
			systemevent= (systemevent & (~SYSEV_PRIORITY_GROUP1));
		}
	}
	if(DMA_GetITStatus(DMA1_IT_TC2))
	{
		tmp=10;
	}
}

void InitializeTimer()
{
	/* Setup a Timer each 500 ms */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_Prescaler = 7200;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period = 5000;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBase_InitStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* Enable the Timer Interrupt */
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    /* All is set up, enable timer */
    TIM_Cmd(TIM3, ENABLE);
}


void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if(GL_led_displayreg[11] > 0)
        {
        	if(systemevent & SYSEV_BLINKLEDSTAT)
        	{
        		systemevent=systemevent | SYSEV_BLINKLEDOFF;
        		systemevent= (systemevent & (~SYSEV_BLINKLEDSTAT));
        	}
        	else
        	{
        		systemevent=systemevent | SYSEV_BLINKLEDON;
        		systemevent=systemevent | SYSEV_BLINKLEDSTAT;
        	}
        }


    }
}


void Timer_Setup(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //ms
    TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    //TIM_Cmd(TIM2,ENABLE);
}


void Delay_us( int nTime)
{
    u16 counter=nTime&0xffff;
    TIM_Cmd(TIM2,ENABLE);
    TIM_SetCounter(TIM2,counter);
    while(counter>1)
    {
        counter=TIM_GetCounter(TIM2);
    }
    TIM_Cmd(TIM2,DISABLE);
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
