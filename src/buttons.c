#include "buttons.h"
#include "poti.h"
#include "stm32f10x_adc.h"
#include "messagequeue.h"
#include "spi.h"


volatile uint16_t ADCBuffer[3];

volatile uint16_t buttonstat;
volatile uint8_t buttonnum;

void update_buttons(void)
{
	uint8_t i;
	static uint16_t buttonbounce[9];
	static uint16_t buttontrigger;

	//1
	if((ADCBuffer[0] > 1000) && (ADCBuffer[0] < 1600))
	{
		buttonbounce[0] = (buttonbounce[0] << 1) | 0x01;
	}
	else
	{
		buttonbounce[0] = (buttonbounce[0] << 1) & 0b1111111111111110;
	}
	//2
	if((ADCBuffer[0] > 1600) && (ADCBuffer[0] < 3000))
	{
		buttonbounce[1] = (buttonbounce[1] << 1) | 0x01;
	}
	else
	{
		buttonbounce[1] = (buttonbounce[1] << 1) & 0b1111111111111110;
	}
	//3
	if((ADCBuffer[0] > 3000) && (ADCBuffer[0] < 5000))
	{
		buttonbounce[2] = (buttonbounce[2] << 1) | 0x01;
	}
	else
	{
		buttonbounce[2] = (buttonbounce[2] << 1) & 0b1111111111111110;
	}
	//4
	if((ADCBuffer[1] > 300) && (ADCBuffer[1] < 900))
	{
		buttonbounce[3] = (buttonbounce[3] << 1) | 0x01;
	}
	else
	{
		buttonbounce[3] = (buttonbounce[3] << 1) & 0b1111111111111110;
	}
	//5
	if((ADCBuffer[1] > 900) && (ADCBuffer[1] < 1490))
	{
		buttonbounce[4] = (buttonbounce[4] << 1) | 0x01;
	}
	else
	{
		buttonbounce[4] = (buttonbounce[4] << 1) & 0b1111111111111110;
	}
	//6
	if((ADCBuffer[1] > 1490) && (ADCBuffer[1] < 1990))
	{
		buttonbounce[5] = (buttonbounce[5] << 1) | 0x01;
	}
	else
	{
		buttonbounce[5] = (buttonbounce[5] << 1) & 0b1111111111111110;
	}
	//7
	if((ADCBuffer[1] > 1990) && (ADCBuffer[1] < 2760))
	{
		buttonbounce[6] = (buttonbounce[6] << 1) | 0x01;
	}
	else
	{
		buttonbounce[6] = (buttonbounce[6] << 1) & 0b1111111111111110;
	}
	//8
	if((ADCBuffer[1] > 2760) && (ADCBuffer[1] < 3600))
	{
		buttonbounce[7] = (buttonbounce[7] << 1) | 0x01;
	}
	else
	{
		buttonbounce[7] = (buttonbounce[7] << 1) & 0b1111111111111110;
	}
	//9
	if((ADCBuffer[1] > 3600) && (ADCBuffer[1] < 5000))
	{
		buttonbounce[8] = (buttonbounce[8] << 1) | 0x01;
	}
	else
	{
		buttonbounce[8] = (buttonbounce[8] << 1) & 0b1111111111111110;
	}

	for(i=0; i<10; i++)
	{
		if(buttonbounce[i]==0xFFFF)
		{
			buttontrigger=0x0001<<i;
		}
		if(buttonbounce[i] == 0x0000)
		{
			if(buttontrigger == (0x0001<<i))
			{
				buttontrigger = (buttontrigger & (~(0x0001<<i)));
				if( i != 8)
				{
					message_create(MSG_BUTTONPRESS, i);
				}
				else
				{
					spi_handleDMA1Ch2Interrupt();
				}
			}
		}
	}
}

void pushbuttoninit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	uint8_t i;

	ADCBuffer[0] = ADCBuffer[1] = ADCBuffer[2] = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, DISABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TE, DISABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);

	ADC_Cmd(ADC1, ENABLE);



	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_DMACmd(ADC1, ENABLE);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
