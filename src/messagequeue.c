#include "hw_config.h"
#include "messagequeue.h"
#include "i2c.h"


volatile uint8_t GL_messagequeue[32][2];
volatile uint8_t GL_currentmessage;
volatile uint8_t GL_firstmessage;
volatile uint8_t GL_messagequeue_elements;
volatile uint8_t GL_i2c_messagequeue_busy;

void messagequeue_init(void)
{
	GL_currentmessage=0;
	GL_firstmessage=0;
	GL_messagequeue_elements=0;
	GL_i2c_messagequeue_busy=0;
}



uint8_t message_create(uint8_t msgtype, uint8_t value)
{
	if(GL_messagequeue_elements < 32)
	{
		GL_messagequeue[GL_currentmessage][0]=msgtype;
		GL_messagequeue[GL_currentmessage][1]=value;
		GL_messagequeue_elements++;
		if(GL_currentmessage<31)
		{
			GL_currentmessage++;
		}
		else
		{
			GL_currentmessage=0;
		}
	}
}

void messagequeue_i2crx_handler(void)
{
	/* Check whether the host got the right message */
	if(Buffer_Rx1[1] == GL_firstmessage)
	{
		if(GL_firstmessage<31)
		{
			GL_firstmessage++;
		}
		else
		{
			GL_firstmessage=0;
		}
		GL_messagequeue_elements--;
		GL_i2c_messagequeue_busy=0;
	}
	/*
	 * else try retransmission of the message. Just set GL_i2c_messageque_busy to 0.
	 * the same message should be retransmitted
	 */
	else
	{
		GL_i2c_messagequeue_busy=0;
	}
}

void messagequeue_update_i2c_buffers(void)
{
	if(GL_i2c_messagequeue_busy == 0)
	{
		GL_i2c_messagequeue_busy=2;
		if(GL_messagequeue_elements > 0)
		{
			Buffer_Tx1[0]=GL_firstmessage;
			Buffer_Tx1[1]=GL_messagequeue[GL_firstmessage][0];
			Buffer_Tx1[2]=GL_messagequeue[GL_firstmessage][1];

			GL_i2c_messagequeue_busy=1;

			/* Enable Interrupt */
			GPIO_WriteBit(I2CINTERRUPTS_GPIO_PORT, I2CINTERRUPT_PIN, Bit_SET);
		}
		else
		{
			GL_i2c_messagequeue_busy=0;
		}
	}
}
