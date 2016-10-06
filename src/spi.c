#include "spi.h"
#include "poti.h"
#include "channelled.h"
#include "siebensegment.h"
#include "hw_config.h"

volatile uint8_t SPI1_BLOCK;
volatile uint8_t GL_spi_send_call;
volatile uint8_t GL_spi_irq_call;


uint8_t SPI1_send(uint8_t n_bytes, uint8_t periph, uint32_t txdata_address, uint32_t rxdata_address)
{
	/* Just a temporary debugging array to visualize the data given to the function */
	uint8_t* tmp;


	tmp=txdata_address;

	/* Set DMA RX and TX Buffers */
	DMA1_Channel2->CMAR = (uint32_t)rxdata_address;
	DMA1_Channel3->CMAR = (uint32_t)txdata_address;

	DMA_SetCurrDataCounter(DMA1_Channel3, n_bytes);
	DMA_SetCurrDataCounter(DMA1_Channel2, n_bytes);

	/* Set or Reset CHIP Select depending on hardware
	 */
	switch( periph )
	{
	case SPI_POTILED1_WRITE_DATA:
		/* Set Chip Select of LED Controller low */
		GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_RESET);
	break;
	case SPI_POTILED2_WRITE_DATA:
		/* Set Chip Select of LED Controller low */
		GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_RESET);
	break;
	case SPI_POTILED3_WRITE_DATA:
		/* Set Chip Select of LED Controller low */
		GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_RESET);
	break;
	case SPI_POTILED4_WRITE_DATA:
		/* Set Chip Select of LED Controller low */
		GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_RESET);
	break;
	case SPI_CHANNELLED_WRITE_DATA:
		/* Set Chip Select of LED Controller low */
		GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_RESET);
	break;
	case SPI_SIEBENSEGMENT_WRITE_DATA:
		/* Set Chip Select of 7 segment Controller low */
		GPIO_WriteBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN, Bit_RESET);
	break;

	/* In case of a failure
	 * Reset everything to its default state
	 */
	default:
		GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_RESET);
		GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_RESET);
		GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_RESET);
		GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_RESET);
		GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_RESET);
		GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_OE_PIN, Bit_SET);
		GPIO_WriteBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN, Bit_RESET);
		SPI1_BLOCK=SPI_BLOCK_FREE;
		return 1;
	break;
	}

	DMA_ClearFlag(DMA1_FLAG_TC2);

	while ((DMA1_Channel2->CCR & DMA_IT_TC)==0)
	{
		DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
		DMA_ITConfig(DMA1_Channel2, DMA_IT_TE, ENABLE);
	}

	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);


	if(GL_spi_send_call < 128)
	{
		GL_spi_send_call++;
	}
	else
	{
		GL_spi_send_call=0;
	}
	return 0;
}

void SPI1_BusInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	SPI1_BLOCK=SPI_BLOCK_FREE;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);




	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	SPI_Cmd(SPI1, ENABLE);


	// DMA Channel 2 - SPI RX
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = 0x00;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

	// DMA Channel 3 - SPI TX
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = 0x00;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void spi_handleDMA1Ch2Interrupt(void)
{

	/* Test on DMA1 Channel2 Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA1_IT_TC2))
	{
		/* Clear DMA1 Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits */
		DMA_ClearITPendingBit(DMA1_IT_GL2);

		/* The next line waits for rx to complete  */
		while (DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET) {}

		/* wait for tx to complete - page 692 */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {}
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}

		/* Clear the global flag */
		DMA_ClearFlag(DMA1_FLAG_GL3);

		/* Disable DMA Transfer Complete Interrupt */
 		DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, DISABLE);

		/* Is it important to disable the SPI DMA hardware ?? 						*/
		/* SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);	*/
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);

		/* wait until DMA is actually off */
		while (DMA1_Channel2->CCR & DMA_CCR2_EN);
		while (DMA1_Channel3->CCR & DMA_CCR3_EN);

		/* Clear the Interrupt flag */
		DMA_ClearFlag(DMA1_FLAG_TC2);

		/* Release or cycle Chip Select of currently addressed hardware */
		switch( SPI1_BLOCK )
		{
		case SPI_POTILED1_WRITE_DATA:
			/* Toggle chip select of LED Controler shortly */
			GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_SET);
		    __ASM volatile ("nop");
		    __ASM volatile ("nop");
			GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_RESET);
		break;
		case SPI_POTILED2_WRITE_DATA:
			/* Toggle chip select of LED Controler shortly */
			GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_SET);
		    __ASM volatile ("nop");
		    __ASM volatile ("nop");
			GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_RESET);
		break;
		case SPI_POTILED3_WRITE_DATA:
			/* Toggle chip select of LED Controler shortly */
			GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_SET);
		    __ASM volatile ("nop");
		    __ASM volatile ("nop");
			GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_RESET);
		break;
		case SPI_POTILED4_WRITE_DATA:
			/* Toggle chip select of LED Controler shortly */
			GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_SET);
		    __ASM volatile ("nop");
		    __ASM volatile ("nop");
			GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_RESET);
		break;
		case SPI_CHANNELLED_WRITE_DATA:
			/* Toggle chip select of LED Controler shortly */
			GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_SET);
		    __ASM volatile ("nop");
		    __ASM volatile ("nop");
			GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_RESET);

		break;
		case SPI_SIEBENSEGMENT_WRITE_DATA:
			/* Set Chip Select of LED Controller high */
			GPIO_WriteBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN, Bit_SET);
		break;

		/*
		 * In case of a failure
		 * Reset everything to its default state
		 */
		default:
			GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_RESET);
			GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_RESET);
			GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_RESET);
			GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_RESET);
			GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_RESET);
			GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_OE_PIN, Bit_RESET);
			GPIO_WriteBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN, Bit_SET);
			SPI1_BLOCK=SPI_BLOCK_FREE;
		break;
		}

		SPI1_BLOCK=SPI_BLOCK_FREE;
	}
	else
	{
		/* Should not get here */
		SPI1_BLOCK=SPI_BLOCK_FREE;
		/* Is it important to disable the SPI DMA hardware ?? 						*/
		/* SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);	*/
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);

		/* wait until DMA is actually off */
		while (DMA1_Channel2->CCR & DMA_CCR2_EN);
		while (DMA1_Channel3->CCR & DMA_CCR3_EN);

		/* Clear the Interrupt flag */
		DMA_ClearFlag(DMA1_FLAG_TC2);
	}

}
