
#include "spi.h"
#include "channelled.h"

uint8_t channel_send_led_data(uint8_t data)
{
	uint8_t i, j;
	static uint16_t txdata, rxdata;
	if(GL_spi1_block != SPI_BLOCK_FREE)
		return 2;
	GL_spi1_block = SPI_BUS_TRANSFER_STARTED;
	if(data >12)
		data=1;
	txdata = __REV16(channel_led_lookup[data]);
	GL_spi1_block = SPI_CHANNELLED_WRITE_DATA;
	SPI1_send(2,SPI_CHANNELLED_WRITE_DATA, &txdata, &rxdata);
	return 0;
}



void channel_led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(CHANNELLED_CS_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(CHANNELLED_OE_PERIPH , ENABLE);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = CHANNELLED_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHANNELLED_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = CHANNELLED_OE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHANNELLED_OE_GPIO_PORT, &GPIO_InitStructure);


	/* Set Pins to it default state, low in Case of the TLC5947 */
	GPIO_WriteBit(CHANNELLED_CS_GPIO_PORT, CHANNELLED_CS_PIN, Bit_RESET);
	GPIO_WriteBit(CHANNELLED_OE_GPIO_PORT, CHANNELLED_OE_PIN, Bit_RESET);

}

const uint16_t channel_led_lookup[13] =
{
		0x0000,
		CHANNELYELLOW1,
		CHANNELYELLOW2,
		CHANNELYELLOW3,
		CHANNELYELLOW4,
		CHANNELORANGE1,
		CHANNELORANGE2,
		CHANNELORANGE3,
		CHANNELORANGE4,
		CHANNELGREEN1,
		CHANNELGREEN2,
		CHANNELGREEN3,
		CHANNELGREEN4,
};
