#include "hw_config.h"
#include "i2c.h"
#include "main.h"
#include "systemevent.h"
#include "messagequeue.h"


void i2c_send(uint8_t data) {
	uint8_t i;
  i2c_delay();
  i2c_delay();

    for(i=0; i<8; i++) {
        SCLL;
        i2c_delay();
        if (data & 0x80)
          SDAH;
        else SDAL;
        data<<=1;
        i2c_delay();
        SCLH;
        i2c_delay();
    }
    SCLL;
}

uint8_t i2c_waitack(void)
{
  SCLL;
  i2c_delay();
  SDAH;
  i2c_delay();
  SCLH;
  i2c_delay();
  if(SDAread)
  {
    SCLL;
    i2c_delay();
    i2c_delay();
    i2c_delay();
    return 0;
  }
  SCLL;
  i2c_delay();
  i2c_delay();
  i2c_delay();
  return 1;
}

void i2c_stop(void)
{
  SCLL;
  i2c_delay();
  SDAL;
  i2c_delay();
  SCLH;
  i2c_delay();
  SDAH;
  i2c_delay();
  i2c_delay();
  i2c_delay();
}


uint8_t i2c_start(void)
{
  SDAH;
  SCLH;
  i2c_delay();
  if (!(SDAread)) return 0;
  SDAL;
  i2c_delay();
  if (SDAread) return 0;
  SDAL;
  i2c_delay();
  i2c_delay();
  i2c_delay();
  i2c_delay();
  return 1;
}

uint8_t i2c_write(uint8_t i2c_target, uint8_t address, uint8_t data) {
    if(!i2c_start()) {
      return 0;
    }
    i2c_send(i2c_target);
    if(!i2c_waitack()) {
      i2c_stop();
      return 0;
    }
    i2c_send(address);
    i2c_waitack();
    if(data != 0x00)
    {
    	i2c_send(data);
    	i2c_waitack();
    }
    i2c_stop();
    return 1;
}


/*
 * @brief Figure out whether SDA is high and toggle SCL to clock out a bus lockup
 * @param None
 * @retval None
 *
 */

void I2C_free_bus(I2C_TypeDef* I2Cx)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	uint8_t i;

	if(I2Cx == I2C2)
	{
		/* Enable the GPIO Port clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		/* I2C1 SDA and SCL configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		SDAH;
		SCLH;
		i2c_delay();
		for(i=0; i<9; i++)
		{
			if(!SDAread)
			{
				SCLL;
				i2c_delay();
				SCLH;
				i2c_delay();

			}
			i2c_delay();
			i2c_stop();
		}
	}


}
