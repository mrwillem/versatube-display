
/**
 ******************************************************************************
 * @file OptimizedI2Cexamples/src/I2CRoutines.c
 * @author  MCD Application Team
 * @version  V4.0.0
 * @date  06/18/2010
 * @brief  Contains the I2Cx slave/Master read and write routines.
 * 			Upgrade by Sylvia Heib Nov/2011
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "i2c.h"
#include "main.h"
#include "systemevent.h"
#include "messagequeue.h"



/** @addtogroup Optimized I2C examples
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


I2C_InitTypeDef I2C_InitStructure1;
I2C_InitTypeDef I2C_InitStructure2;

volatile uint32_t I2CDirection;
volatile uint32_t NumbOfBytes1;
volatile uint32_t NumbOfBytes2;
volatile uint8_t Address;
volatile uint8_t i2c_rx_event;
uint8_t Buffer_Rx1[20];
uint8_t Buffer_Tx1[20];

uint8_t Buffer_Rx2[20];
uint8_t Buffer_Tx2[20];

volatile uint8_t Tx_Idx1 = 0, Rx_Idx1 = 0;
volatile uint8_t Tx_Idx2 = 0, Rx_Idx2 = 0;

volatile uint8_t I2C2_BUSY = 3;


void I2C_InterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(I2CINTERRUPT_PERIPH , ENABLE);
	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = I2CINTERRUPT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(I2CINTERRUPTS_GPIO_PORT, &GPIO_InitStructure);
	GPIO_WriteBit(I2CINTERRUPTS_GPIO_PORT, I2CINTERRUPT_PIN, Bit_RESET);
}

void I2C2_changeId(unsigned char newID)
{
	I2C_InitStructure2.I2C_OwnAddress1 = newID;
}
/**
 * @brief  Configures NVIC and Vector Table base location.
 * @param  None
 * @retval : None
 */
void I2C_NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

DMA_InitTypeDef I2CDMA_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Reads buffer of bytes  from the slave.
 * @param pBuffer: Buffer of bytes to be read from the slave.
 * @param NumByteToRead: Number of bytes to be read by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
uint8_t I2C_Master_BufferRead(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
		uint32_t NumByteToRead,  uint8_t SlaveAddress)

{
	uint32_t temp = 0;
	uint32_t Timeout = 0;

	/* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
	I2Cx->CR2 |= I2C_IT_ERR;

	/* I2Cx Master Reception using Interrupts with highest priority in an application */
	{
		/* Enable EVT IT*/
		I2Cx->CR2 |= I2C_IT_EVT;
		/* Enable BUF IT */
		I2Cx->CR2 |= I2C_IT_BUF;
		/* Set the I2C direction to reception */
		I2CDirection = I2C_DIRECTION_RX;

		SlaveAddress |= OAR1_ADD0_Set;
		Address = SlaveAddress;
		if (I2Cx == I2C1)
			NumbOfBytes1 = NumByteToRead;
		else
			NumbOfBytes2 = NumByteToRead;
		/* Send START condition */
		I2Cx->CR1 |= CR1_START_Set;
		/* Wait until the START condition is generated on the bus: START bit is cleared by hardware */
		while ((I2Cx->CR1 & 0x100) == 0x100);
		/* Wait until BUSY flag is reset (until a STOP is generated) */
		while ((I2Cx->SR2 & 0x0002) == 0x0002);
//		{
//			I2C2_EV_IRQHandler();
//		}
		/* Enable Acknowledgement to be ready for another reception */
		I2Cx->CR1 |= CR1_ACK_Set;
	}
	return 0;
}

/**
 * @brief  Writes buffer of bytes.
 * @param pBuffer: Buffer of bytes to be sent to the slave.
 * @param NumByteToWrite: Number of bytes to be sent by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
uint8_t I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, uint8_t* pBuffer,
		uint32_t NumByteToWrite,
		uint8_t SlaveAddress)

{
	volatile uint32_t temp = 0;
	volatile uint32_t Timeout = 0;


	if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
						I2C_GenerateSTOP(I2Cx, ENABLE);

	/* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
	I2Cx->CR2 |= I2C_IT_ERR;
	/* I2Cx Master Transmission using Interrupt with highest priority in the application */
	/* Enable EVT IT*/
	I2Cx->CR2 |= I2C_IT_EVT;
	/* Enable BUF IT */
	I2Cx->CR2 |= I2C_IT_BUF;
	/* Set the I2C direction to Transmission */
	I2CDirection = I2C_DIRECTION_TX;
	SlaveAddress &= OAR1_ADD0_Reset;
	Address = SlaveAddress;
	if (I2Cx == I2C1)
		NumbOfBytes1 = NumByteToWrite;
	else
		NumbOfBytes2 = NumByteToWrite;
	/* Send START condition */
	I2Cx->CR1 |= CR1_START_Set;
	/* Wait until the START condition is generated on the bus: the START bit is cleared by hardware */
	while ((I2Cx->CR1 & 0x100) == 0x100)
	{
		temp=I2Cx->SR1;
		temp=I2Cx->SR2;
	}
	/* Wait until BUSY flag is reset: a STOP has been generated on the bus signaling the end of transmission */
	while ((I2Cx->SR2 & 0x0002) == 0x0002);
//	{
//		I2C2_EV_IRQHandler();
//		I2C2_ER_IRQHandler();0
//	}
	return 0;
}

/*
 * @brief A little delay function for I2C bit Banging
 * @param None
 * @retval None
 */
void i2c_delay(void)
{
	uint16_t i;
   /*
    * 72 MHz / <= divided by
    * 400 nops = 180KHz;
    * 1600 nops ~= 45kHz
    * 3200 nops ~= 22,25 kHz
    */
   for(i=0; i<400; i++)
   {
     asm volatile ("nop");
   }
}


/**
 * @brief Prepares the I2Cx slave for transmission.
 * @param I2Cx: I2C1 or I2C2.
 * @param Mode: DMA or Interrupt having the highest priority in the application.
 * @retval : None.
 */

void I2C_Slave_BufferReadWrite(I2C_TypeDef* I2Cx)

{
	/* Enable Event IT needed for ADDR and STOPF events ITs */
	I2Cx->CR2 |= I2C_IT_EVT;
	/* Enable Error IT */
	I2Cx->CR2 |= I2C_IT_ERR;

	/* I2Cx Slave Transmission using Interrupt with highest priority in the application */
	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2Cx->CR2 |= I2C_IT_BUF;
}




/**
 * @brief  Initializes peripherals: I2Cx, GPIO, DMA channels .
 * @param  None
 * @retval The filled up I2C_InitStructure
 */
I2C_InitTypeDef I2C_LowLevel_Init(I2C_TypeDef* I2Cx) {




	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	// Configure Interrupt priority
	I2C_NVIC_Configuration();

	/* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	if (I2Cx == I2C1)
	{
		I2C_DeInit(I2C1);
		/* I2C1 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		/* I2C1 SDA and SCL configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Enable I2C1 reset state */
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
		/* Release I2C1 from reset state */
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
		/* I2C1 configuration */
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 = OwnAddress1;
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
		I2C_Init(I2C1, &I2C_InitStructure);



		// I2C Peripheral Enable

		I2C_Cmd(I2C1, ENABLE);

		/* enable genarall call detection */
		I2C1->CR1 |= CR1_ENGC_Set;
	}

	else /* I2Cx = I2C2 */

	{

		/* I2C2 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		/* I2C1 SDA and SCL configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* De Init the Bus */
		I2C_DeInit(I2C2);

		/* Enable I2C2 reset state */
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
		/* Release I2C2 from reset state */
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

		/* I2C2 configuration */
		I2C_InitStructure2.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure2.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure2.I2C_OwnAddress1 = OwnAddress2;
		I2C_InitStructure2.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure2.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure2.I2C_ClockSpeed = ClockSpeed;
		I2C_Init(I2C2, &I2C_InitStructure2);
		// I2C Peripheral Enable

		I2C_Cmd(I2C2, ENABLE);

		/* enable genarall call detection */
		I2C2->CR1 |= CR1_ENGC_Set;


	}
	if (I2Cx == I2C1)
	{

	}
	else
	{
		I2C2_BUSY=0;
	}
	return I2C_InitStructure;
}


/**
 * @brief  This function handles I2C1 Event interrupt request.
 * @param  None
 * @retval : None
 */
void I2C1_Event_IRQHandler(void) {

	volatile unsigned long SR1Register = 0;
	volatile unsigned long SR2Register = 0;

	/* Read the I2C1 SR1 and SR2 status registers */
	SR1Register = I2C1->SR1;
	SR2Register = I2C1->SR2;

	/* If I2C1 is slave (MSL flag = 0) */
	if ((SR2Register & 0x0001) != 0x0001) {
		/* If ADDR = 1: EV1 */
		if ((SR1Register & 0x0002) == 0x0002) {
			/* Clear SR1Register and SR2Register variables to prepare for next IT */
			SR1Register = 0;
			SR2Register = 0;
			/* Initialize the transmit/receive counters for next transmission/reception
			 using Interrupt  */
			Tx_Idx1 = 0;
			Rx_Idx1 = 0;
		}
		/* If TXE = 1: EV3 */
		if ((SR1Register & 0x0080) == 0x0080) {
			/* Write data in data register */
			I2C1->DR = Buffer_Tx1[Tx_Idx1++];
			SR1Register = 0;
			SR2Register = 0;
			/*
			 * Getting here actually means that the Master got our IRQ.
			 * So we set the Interrupt line low again
			 */
			GPIO_WriteBit(I2CINTERRUPTS_GPIO_PORT, I2CINTERRUPT_PIN, Bit_RESET);
		}
		/* If RXNE = 1: EV2 */
		if ((SR1Register & 0x0040) == 0x0040) {
			/* Read data from data register */
			Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
			SR1Register = 0;
			SR2Register = 0;

		}
		/* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
		if ((SR1Register & 0x0010) == 0x0010) {
			I2C1->CR1 |= CR1_PE_Set;
			SR1Register = 0;
			SR2Register = 0;
			systemevent = (systemevent | SYSEV_I2CRXCOMPLETE | SYSEV_PRIORITY_GROUP1);

		}
	} /* End slave mode */


	/* If SB = 1, I2C1 master sent a START on the bus: EV5) */
	if ((SR1Register & 0x0001) == 0x0001) {

		/* Send the slave address for transmission or for reception (according to the configured value
		 in the write master write routine */
		I2C1->DR = Address;
		SR1Register = 0;
		SR2Register = 0;
	}
	/* If I2C1 is Master (MSL flag = 1) */

	if ((SR2Register & 0x0001) == 0x0001) {
		/* If ADDR = 1, EV6 */
		if ((SR1Register & 0x0002) == 0x0002) {
			/* Write the first data in case the Master is Transmitter */
			if (I2CDirection == I2C_DIRECTION_TX) {
				/* Initialize the Transmit counter */
				Tx_Idx1 = 0;
				/* Write the first data in the data register */
				I2C1->DR = Buffer_Tx1[Tx_Idx1++];
				/* Decrement the number of bytes to be written */
				NumbOfBytes1--;
				/* If no further data to be sent, disable the I2C BUF IT
				 in order to not have a TxE  interrupt */
				if (NumbOfBytes1 == 0)
				{
					I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
				}

			}
			/* Master Receiver */
			else

			{
				/* Initialize Receive counter */
				Rx_Idx1 = 0;
				/* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
				/* EV6_1: used for single byte reception. The ACK disable and the STOP
				 Programming should be done just after ADDR is cleared. */
				if (NumbOfBytes1 == 1) {
					/* Clear ACK */I2C1->CR1 &= CR1_ACK_Reset;
					/* Program the STOP */I2C1->CR1 |= CR1_STOP_Set;
				}
			}
			SR1Register = 0;
			SR2Register = 0;

		}
		/* If BTF and TXE are set (EV8_2), program the STOP */
		if ((SR1Register & 0x0084) == 0x0084) {

			/* Program the STOP */I2C1->CR1 |= CR1_STOP_Set;
			/* Disable EVT IT In order to not have again a BTF IT */I2C1->CR2
					&= (unsigned int) ~I2C_IT_EVT;
			SR1Register = 0;
			SR2Register = 0;
		}
		/* Master transmits the remaing data: from data2 until the last one.  */
		/* If TXE is set */
		//if ((SR1Register & 0x0084) == 0x0080) {
		if ((SR1Register & 0x0080) == 0x0080) {
			/* If there is still data to write */
			if (NumbOfBytes1 != 0) {
				/* Write the data in DR register */
				I2C1->DR = Buffer_Tx1[Tx_Idx1++];
				/* Decrment the number of data to be written */
				NumbOfBytes1--;
				/* If  no data remains to write, disable the BUF IT in order
				 to not have again a TxE interrupt. */
				if (NumbOfBytes1 == 0) {
					/* Disable the BUF IT */
					I2C1->CR2 &= (unsigned int) ~I2C_IT_BUF;
				}
			}
			SR1Register = 0;
			SR2Register = 0;
		}
		/* If RXNE is set */
		if ((SR1Register & 0x0040) == 0x0040) {
			/* Read the data register */
			Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
			/* Decrement the number of bytes to be read */
			NumbOfBytes1--;
			/* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
			if (NumbOfBytes1 == 1) {
				/* Clear ACK */
				I2C1->CR1 &= CR1_ACK_Reset;
				/* Program the STOP */
				I2C1->CR1 |= CR1_STOP_Set;
			}
			SR1Register = 0;
			SR2Register = 0;
		}

	}

}


/**
 * @brief  This function handles I2C2 Event interrupt request.
 * @param  None
 * @retval : None
 */
void I2C2_Event_IRQHandler(void) {
	volatile unsigned long SR1Register = 0;
	volatile unsigned long SR2Register = 0;

	/* Read the I2C1 SR1 and SR2 status registers */
	SR1Register = I2C2->SR1;
	SR2Register = I2C2->SR2;

	/* If I2C2 is slave (MSL flag = 0) */
	if ((SR2Register & 0x0001) != 0x0001) {
		/* If ADDR = 1: EV1 */
		if ((SR1Register & 0x0002) == 0x0002) {
			/* Clear SR1Register SR2Register variables to prepare for next IT*/
			SR1Register = 0;
			SR2Register = 0;
			/* Initialize the transmit/receive counters for next transmission/reception
			 using Interrupt  */
			Tx_Idx2 = 0;
			Rx_Idx2 = 0;
		}
		/* If TXE = 1: EV3 */
		if ((SR1Register & 0x0080) == 0x0080) {
			/* Write data in data register */
			I2C2->DR = Buffer_Tx2[Tx_Idx2++];
			SR1Register = 0;
			SR2Register = 0;

		}
		/* If RXNE = 1: EV2 */
		if ((SR1Register & 0x0040) == 0x0040) {

			/* Read data from data register */
			Buffer_Rx2[Rx_Idx2++] = I2C2->DR;
			SR1Register = 0;
			SR2Register = 0;


		}
		/* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
		if ((SR1Register & 0x0010) == 0x0010)
		{
			I2C2->CR1 |= CR1_PE_Set;
			SR1Register = 0;
			SR2Register = 0;
		}
	} /* End slave mode */


	/* If SB = 1, I2C1 master sent a START on the bus: EV5) */
	if ((SR1Register & 0x0001) == 0x0001)
	{

		/* Send the slave address for transmission or for reception (according to the configured value
		 in the write master write routine */
		I2C2->DR = Address;
		SR1Register = 0;
		SR2Register = 0;

	}
	/* If I2C2 is Master (MSL flag = 1) */
	if ((SR2Register & 0x0001) == 0x0001)
	{
		/* If ADDR = 1, EV6 */
		if ((SR1Register & 0x0002) == 0x0002)
		{
			/* Write the first data in case the Master is Transmitter */
			if (I2CDirection == I2C_DIRECTION_TX)
			{
				/* Initialize the Transmit counter */
				Tx_Idx2 = 0;
				/* Write the first data in the data register */
				I2C2->DR = Buffer_Tx2[Tx_Idx2++];
				/* Decrement the number of bytes to be written */
				NumbOfBytes2--;
				/*
				 * If no further data to be sent, disable the I2C BUF IT
				 * in order to not have a TxE  interrupt
				 */
				if (NumbOfBytes2 == 0)
				{
					I2C2->CR2 &= (unsigned int) ~I2C_IT_BUF;
				}
			}
			/* Master Receiver */
			else
			{
				/* Initialize Receive counter */
				Rx_Idx2 = 0;
				/* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
				/* EV6_1: used for single byte reception. The ACK disable and the STOP
				 Programming should be done just after ADDR is cleared. */
				if (NumbOfBytes2 == 1)
				{
					/* Clear ACK */
					I2C2->CR1 &= CR1_ACK_Reset;
					/* Program the STOP */
					I2C2->CR1 |= CR1_STOP_Set;
				}
			}
			SR1Register = 0;
			SR2Register = 0;
		}
		/* If BTF and TXE are set (EV8_2), program the STOP */
		if ((SR1Register & 0x0084) == 0x0084)
		{
			/* Program the STOP */
			I2C2->CR1 |= CR1_STOP_Set;
			/* Disable EVT IT In order to not have again a BTF IT */
			I2C2->CR2 &= (uint16_t) ~I2C_IT_EVT;
			SR1Register = 0;
			SR2Register = 0;

		}
		/* Master transmits the remaing data: from data2 until the last one.  */
		/* If TXE is set */
		//if ((SR1Register & 0x0084) == 0x0080) {
		if ((SR1Register & 0x0080) == 0x0080)
		{
			/* If there is still data to write */
			if (NumbOfBytes2 != 0)
			{
				/* Write the data in DR register */
				I2C2->DR = Buffer_Tx2[Tx_Idx2++];
				/* Decrment the number of data to be written */
				NumbOfBytes2--;
				/* If  no data remains to write, disable the BUF IT in order
				 to not have again a TxE interrupt. */
				if (NumbOfBytes2 == 0)
				{
					/* Disable the BUF IT */
					I2C2->CR2 &= (uint16_t) ~I2C_IT_BUF;
				}
			}
			SR1Register = 0;
			SR2Register = 0;

		}

		/* If RXNE is set */
		if ((SR1Register & 0x0040) == 0x0040)
		{
			/* Read the data register */
			Buffer_Rx2[Rx_Idx2++] = I2C2->DR;
			/* Decrement the number of bytes to be read */
			NumbOfBytes2--;
			/* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
			if (NumbOfBytes2 == 1)
			{
				/* Clear ACK */
				I2C2->CR1 &= CR1_ACK_Reset;
				/* Program the STOP */
				I2C2->CR1 |= CR1_STOP_Set;
			}
			SR1Register = 0;
			SR2Register = 0;

		}

	}

}

/**
 * @brief  This function handles I2C2 Error interrupt request.
 * @param  None
 * @retval : None
 */
void I2C2_Error_IRQHandler(void) {

	volatile unsigned long SR1Register = 0;
	volatile unsigned long SR2Register = 0;

	/* Read the I2C2 status register */
	SR1Register = I2C2->SR1;
	SR2Register = I2C2->SR2;
	/* If AF = 1 */
	if ((SR1Register & 0x0400) == 0x0400) {
		I2C2->SR1 &= 0xFBFF;

		/* If I2C2 is Master (MSL flag = 1) */// DATASheet S.743
		if ((SR2Register & 0x0001) == 0x0001)
		{
			/* Stop condition */
			/* Program the STOP */
			I2C2->CR1 |= CR1_STOP_Set;

		}
		SR1Register = 0;
		SR2Register = 0;
	}
	/* If ARLO = 1 */
	if ((SR1Register & 0x0200) == 0x0200) {
		I2C2->SR1 &= 0xFDFF;
		SR1Register = 0;
	}
	/* If BERR = 1 */
	if ((SR1Register & 0x0100) == 0x0100) {

		I2C2->SR1 &= 0xFEFF;

		if ((I2C2->CR1 & 0x0100) == 0x0100)
		{
			/* is START bit set? */
			/* Peripheral under reset */
			I2C2->CR1 |= CR1_SWRST_Set; //Errata S. 25
			I2C_LowLevel_Init(I2C2);
			/* Enable the selected I2C interrupts */
			I2C2->CR2 |= I2C_IT_EVT;
		}

		SR1Register = 0;
	}

	/* If OVR = 1 */

	if ((SR1Register & 0x0800) == 0x0800) {
		I2C2->SR1 &= 0xF7FF;
		SR1Register = 0;
	}

	/* If PECERR = 1 */

	if ((SR1Register & 0x1000) == 0x1000) {
		I2C2->SR1 &= 0xEFFF;
		SR1Register = 0;
	}

	/* If TIMEOUT = 1 */

	if ((SR1Register & 0x4000) == 0x4000) {
		I2C2->SR1 &= 0xBFFF;
		SR1Register = 0;
	}

	/* If SMBALERT = 1 */

	if ((SR1Register & 0x8000) == 0x8000) {
		I2C2->SR1 &= 0x7FFF;
		SR1Register = 0;
	}
}

/**
 * @brief  This function handles I2C1 Error interrupt request.
 * @param  None
 * @retval : None
 */
void I2C1_Error_IRQHandler(void) {

	volatile unsigned long SR1Register = 0;
	volatile unsigned long SR2Register = 0;

	/* Read the I2C1 status register */
	SR1Register = I2C1->SR1;
	SR2Register = I2C1->SR2;

	/* If AF = 1 */
	if ((SR1Register & 0x0400) == 0x0400) {
		I2C1->SR1 &= 0xFBFF;

		/* If I2C2 is Master (MSL flag = 1) */// DATASheet S.743
		if ((SR2Register & 0x0001) == 0x0001)
		{//Stop condition
			/* Program the STOP*/
			I2C1->CR1 |= CR1_STOP_Set;

		}
		SR1Register = 0;
		SR2Register = 0;

	}
	/* If ARLO = 1 */
	if ((SR1Register & 0x0200) == 0x0200) {
		I2C1->SR1 &= 0xFDFF;
		SR1Register = 0;
	}
	/* If BERR = 1 */
	if ((SR1Register & 0x0100) == 0x0100) {
		I2C1->SR1 &= 0xFEFF;

		if ((I2C1->CR1 & 0x0100) == 0x0100) { //is START bit set?
			/* Peripheral under reset */I2C1->CR1 |= CR1_SWRST_Set; //Errata S. 25
			I2C_LowLevel_Init(I2C1);
			/* Enable the selected I2C interrupts */
			I2C1->CR2 |= I2C_IT_EVT;
		}
		SR1Register = 0;
	}

	/* If OVR = 1 */

	if ((SR1Register & 0x0800) == 0x0800) {
		I2C1->SR1 &= 0xF7FF;
		SR1Register = 0;
	}

	/* If PECERR = 1 */

	if ((SR1Register & 0x1000) == 0x1000) {
		I2C1->SR1 &= 0xEFFF;
		SR1Register = 0;
	}

	/* If TIMEOUT = 1 */

	if ((SR1Register & 0x4000) == 0x4000) {
		I2C1->SR1 &= 0xBFFF;
		SR1Register = 0;
	}

	/* If SMBALERT = 1 */

	if ((SR1Register & 0x8000) == 0x8000) {
		I2C1->SR1 &= 0x7FFF;
		SR1Register = 0;
	}

}

/**
 * @}
 */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
