#include "spi.h"
#include "poti.h"
#include "messagequeue.h"
#include "systemevent.h"



volatile uint8_t poti_data[40];
uint8_t rx_data[40];



int8_t potdelta[7];
uint8_t potstate[7];

volatile uint16_t ledvalue[20];


volatile uint8_t rotaryportstateA;
volatile uint8_t rotaryportstateB;
volatile uint8_t prevrotaryportstateA;
volatile uint8_t prevrotaryportstateB;

uint8_t poti_send_led_data(uint8_t number, uint8_t leftvalue, uint8_t rightvalue)
{
	/* Just counter variables */
	uint8_t i, j;
	/* Some temporary variables to store the 12bit led power values for two leds */
	uint16_t tmpled1, tmpled2;
	/* Two variables to merge both 12 bit led power values into one */
	uint32_t ledmergeleft;
	uint32_t ledmergeright;
	/*
	 * Two pointers which make it easier to copy the 24 bits bytewise in the array
	 * transmitted to the TLC5947 Chipt.
	 */
	uint8_t * ledptrleft;
	uint8_t * ledptrright;
	ledptrleft=&ledmergeleft;
	ledptrright=&ledmergeright;

	if(SPI1_BLOCK != SPI_BLOCK_FREE)
	{
		return 2;
	}


	/*
	 * First of all, get the values for the individual leds in one 36 byte array
	 * for the SPI transfer to theTLC5947
	 * The led order for the left poti is
	 * 3 4 5 6 7 8 2 1 12 11 9 10
	 * and for the right poti it is
	 * 3 4 2 1 12 11 5 6 7 8 9 10
	 */
	tmpled1=ledvalue[ledpower[rightvalue][2]];
	tmpled2=ledvalue[ledpower[rightvalue][3]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][2]];
	tmpled2=ledvalue[ledpower[leftvalue][3]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=0;i<3;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	tmpled1=ledvalue[ledpower[rightvalue][1]];
	tmpled2=ledvalue[ledpower[rightvalue][0]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][4]];
	tmpled2=ledvalue[ledpower[leftvalue][5]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=3;i<6;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	tmpled1=ledvalue[ledpower[rightvalue][11]];
	tmpled2=ledvalue[ledpower[rightvalue][10]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][6]];
	tmpled2=ledvalue[ledpower[leftvalue][7]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=6;i<9;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	tmpled1=ledvalue[ledpower[rightvalue][4]];
	tmpled2=ledvalue[ledpower[rightvalue][5]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][1]];
	tmpled2=ledvalue[ledpower[leftvalue][0]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=9;i<12;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	tmpled1=ledvalue[ledpower[rightvalue][6]];
	tmpled2=ledvalue[ledpower[rightvalue][7]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][11]];
	tmpled2=ledvalue[ledpower[leftvalue][10]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=12;i<15;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	tmpled1=ledvalue[ledpower[rightvalue][8]];
	tmpled2=ledvalue[ledpower[rightvalue][9]];
	ledmergeright=0x0000 | ((tmpled1<<12) | tmpled2);
	tmpled1=ledvalue[ledpower[leftvalue][8]];
	tmpled2=ledvalue[ledpower[leftvalue][9]];
	ledmergeleft=0x0000 | ((tmpled1<<12) | tmpled2);
	j=2;
	for(i=15;i<18;i++)
	{
		poti_data[i]=ledptrleft[j];
		poti_data[i+18]=ledptrright[j];
		j--;
	}
	/*
	 * Check whether the bus is free immediately before the transfer is startet
	 */
	if(SPI1_BLOCK != SPI_BLOCK_FREE)
	{
		return 2;
	}
	/* Quickly claim thebus */
	SPI1_BLOCK = SPI_BUS_TRANSFER_STARTED;

	switch(number)
	{
	case 1:
		SPI1_BLOCK = SPI_POTILED1_WRITE_DATA;
		SPI1_send(36,SPI_POTILED1_WRITE_DATA, poti_data, rx_data);
	break;
	case 2:
		SPI1_BLOCK = SPI_POTILED2_WRITE_DATA;
		SPI1_send(36,SPI_POTILED2_WRITE_DATA, poti_data, rx_data);
	break;
	case 3:
		SPI1_BLOCK = SPI_POTILED3_WRITE_DATA;
		SPI1_send(36,SPI_POTILED3_WRITE_DATA, poti_data, rx_data);
	break;
	case 4:
		for(i=0;i<18;i++)
		{
			poti_data[i]=0x00;
		}
		SPI1_BLOCK = SPI_POTILED4_WRITE_DATA;
		SPI1_send(36,SPI_POTILED4_WRITE_DATA, poti_data, rx_data);
	break;

	default:
	break;
	}
	return 0;
}


/*
 * The initial values are transferred before any LED is lit from the main
 * MCU Board. The valuevector might thus be the RX Buffer from the I2C Bus.
 * n has to be < 20 otherwise it will be set to 20 and only the first 20 values
 * will be used.
 */
void poti_value_init(uint8_t n, uint16_t * valuevector)
{
	uint8_t i;

	if(n>20)
		n=20;

	for(i=0;i<n;i++)
		ledvalue[i]=valuevector[i];

}

void poti_led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(POTILED1_CS_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTILED2_CS_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTILED3_CS_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTILED4_CS_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTILED_BLANK_PERIPH , ENABLE);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = POTILED1_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(POTILED1_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = POTILED2_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(POTILED2_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = POTILED3_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(POTILED3_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = POTILED4_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(POTILED4_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Chip Select (NCS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = POTILED_BLANK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(POTILED_BLANK_GPIO_PORT, &GPIO_InitStructure);

	/* Set BLANK Pin to it default state, high in Case of the TLC5947 */
	GPIO_WriteBit(POTILED_BLANK_GPIO_PORT, POTILED_BLANK_PIN, Bit_SET);

	/* Set Pins to it default state, low in Case of the TLC5947 */
	GPIO_WriteBit(POTILED1_CS_GPIO_PORT, POTILED1_CS_PIN, Bit_RESET);
	GPIO_WriteBit(POTILED2_CS_GPIO_PORT, POTILED2_CS_PIN, Bit_RESET);
	GPIO_WriteBit(POTILED3_CS_GPIO_PORT, POTILED3_CS_PIN, Bit_RESET);
	GPIO_WriteBit(POTILED4_CS_GPIO_PORT, POTILED4_CS_PIN, Bit_RESET);

}

void poti_encoder_init(void)
{
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStructure;


	RCC_APB2PeriphClockCmd(POTIENCODER1A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER1B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER2A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER2B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER3A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER3B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER4A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER4B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER5A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER5B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER6A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER6B_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER7A_PERIPH , ENABLE);
	RCC_APB2PeriphClockCmd(POTIENCODER7B_PERIPH , ENABLE);

	/* Configure the lines in floating input mode input mode */
	GPIO_InitStructure.GPIO_Pin = POTIENCODER1A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER1A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER1B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER1B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER2A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER2A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER2B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER2B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER3A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER3A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER3B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER3B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER4A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER4A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER4B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER4B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER5A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER5A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER5B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER5B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER6A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER6A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER6B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER6B_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER7A_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER7A_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POTIENCODER7B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(POTIENCODER7B_GPIO_PORT, &GPIO_InitStructure);


	for(i=0; i<7; i++)
	{
		potdelta[i]=0;
	}



	/* Setup a Timer each 1 ms */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_Prescaler = 720;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period = 200;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBase_InitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /* Enable the Timer Interrupt */
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    /* All is set up, enable timer */
    TIM_Cmd(TIM4, ENABLE);


}

void read_encoder_ports(void)
{
	uint8_t i;
	potstate[0] = potstate[0] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER1A_GPIO_PORT, POTIENCODER1A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER1B_GPIO_PORT, POTIENCODER1B_PIN)<<1)));
	potstate[1] = potstate[1] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER2A_GPIO_PORT, POTIENCODER2A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER2B_GPIO_PORT, POTIENCODER2B_PIN)<<1)));
	potstate[2] = potstate[2] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER3A_GPIO_PORT, POTIENCODER3A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER3B_GPIO_PORT, POTIENCODER3B_PIN)<<1)));
	potstate[3] = potstate[3] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER4A_GPIO_PORT, POTIENCODER4A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER4B_GPIO_PORT, POTIENCODER4B_PIN)<<1)));
	potstate[4] = potstate[4] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER5A_GPIO_PORT, POTIENCODER5A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER5B_GPIO_PORT, POTIENCODER5B_PIN)<<1)));
	potstate[5] = potstate[5] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER6A_GPIO_PORT, POTIENCODER6A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER6B_GPIO_PORT, POTIENCODER6B_PIN)<<1)));
	potstate[6] = potstate[6] | (0x03 & (GPIO_ReadInputDataBit(POTIENCODER7A_GPIO_PORT, POTIENCODER7A_PIN) | (GPIO_ReadInputDataBit(POTIENCODER7B_GPIO_PORT, POTIENCODER7B_PIN)<<1)));
	potdelta[0] += rotarytable[(potstate[0]>>4)][(0x03 & potstate[0])];
	potdelta[1] += rotarytable[(potstate[1]>>4)][(0x03 & potstate[1])];
	potdelta[2] += rotarytable[(potstate[2]>>4)][(0x03 & potstate[2])];
	potdelta[3] += rotarytable[(potstate[3]>>4)][(0x03 & potstate[3])];
	potdelta[4] += rotarytable[(potstate[4]>>4)][(0x03 & potstate[4])];
	potdelta[5] += rotarytable[(potstate[5]>>4)][(0x03 & potstate[5])];
	potdelta[6] += rotarytable[(potstate[6]>>4)][(0x03 & potstate[6])];
	potstate[0]=(0b00110000 & (potstate[0]<<4));
	potstate[1]=(0b00110000 & (potstate[1]<<4));
	potstate[2]=(0b00110000 & (potstate[2]<<4));
	potstate[3]=(0b00110000 & (potstate[3]<<4));
	potstate[4]=(0b00110000 & (potstate[4]<<4));
	potstate[5]=(0b00110000 & (potstate[5]<<4));
	potstate[6]=(0b00110000 & (potstate[6]<<4));

	for(i=0; i<7; i++)
	{
		if(potdelta[i] < (-1))
		{
			potdelta[i]=0;
			message_create(MSG_POTITURN,(0x00|i|MSG_POTILEFT));
		}
		if(potdelta[i] > 1)
		{
			potdelta[i]=0;
			message_create(MSG_POTITURN,(0x00|i|MSG_POTIRIGHT));
		}
	}

}

void Input_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        systemevent=(systemevent | SYSEV_READINPUTS);
    }
}

const int8_t rotarytable[4][4] = {
		{0,-1,1,0},
		{1,0,0,-1},
		{-1,0,0,1},
		{0,1,-1,0}
};


/*
 * The following array defines a value each LED of the LED ring around the
 * rotary encoder has in a given state. The value can then be translated to a
 * value for the TLC5947 PWM mechanism with the ledvalue array
 * The array ledvalue is filled in poti_led_init(void)
 */
const uint8_t ledpower[64][12] = {
{1,0,0,0,0,0,0,0,0,0,0,0},
{2,0,0,0,0,0,0,0,0,0,0,0},
{3,0,0,0,0,0,0,0,0,0,0,0},
{4,1,0,0,0,0,0,0,0,0,0,0},
{5,1,0,0,0,0,0,0,0,0,0,0},
{6,1,0,0,0,0,0,0,0,0,0,0},
{7,2,0,0,0,0,0,0,0,0,0,0},
{8,3,0,0,0,0,0,0,0,0,0,0},
{9,4,1,0,0,0,0,0,0,0,0,0},
{8,5,1,0,0,0,0,0,0,0,0,0},
{7,6,1,0,0,0,0,0,0,0,0,0},
{6,7,2,0,0,0,0,0,0,0,0,0},
{5,8,3,0,0,0,0,0,0,0,0,0},
{4,9,4,1,0,0,0,0,0,0,0,0},
{3,8,5,1,0,0,0,0,0,0,0,0},
{2,7,6,1,0,0,0,0,0,0,0,0},
{1,6,7,2,0,0,0,0,0,0,0,0},
{1,5,8,3,0,0,0,0,0,0,0,0},
{1,4,9,4,1,0,0,0,0,0,0,0},
{0,3,8,5,1,0,0,0,0,0,0,0},
{0,2,7,6,1,0,0,0,0,0,0,0},
{0,1,6,7,2,0,0,0,0,0,0,0},
{0,1,5,8,3,0,0,0,0,0,0,0},
{0,1,4,9,4,1,0,0,0,0,0,0},
{0,0,3,8,5,1,0,0,0,0,0,0},
{0,0,2,7,6,1,0,0,0,0,0,0},
{0,0,1,6,7,2,0,0,0,0,0,0},
{0,0,1,5,8,3,0,0,0,0,0,0},
{0,0,1,4,9,4,1,0,0,0,0,0},
{0,0,0,3,8,5,1,0,0,0,0,0},
{0,0,0,2,7,6,1,0,0,0,0,0},
{0,0,0,1,6,7,2,0,0,0,0,0},
{0,0,0,1,5,8,3,0,0,0,0,0},
{0,0,0,1,4,9,4,1,0,0,0,0},
{0,0,0,0,3,8,5,1,0,0,0,0},
{0,0,0,0,2,7,6,1,0,0,0,0},
{0,0,0,0,1,6,7,2,0,0,0,0},
{0,0,0,0,1,5,8,3,0,0,0,0},
{0,0,0,0,1,4,9,4,1,0,0,0},
{0,0,0,0,0,3,8,5,1,0,0,0},
{0,0,0,0,0,2,7,6,1,0,0,0},
{0,0,0,0,0,1,6,7,2,0,0,0},
{0,0,0,0,0,1,5,8,3,0,0,0},
{0,0,0,0,0,1,4,9,4,1,0,0},
{0,0,0,0,0,0,3,8,5,1,0,0},
{0,0,0,0,0,0,2,7,6,1,0,0},
{0,0,0,0,0,0,1,6,7,2,0,0},
{0,0,0,0,0,0,1,5,8,3,0,0},
{0,0,0,0,0,0,1,4,9,4,1,0},
{0,0,0,0,0,0,0,3,8,5,1,0},
{0,0,0,0,0,0,0,2,7,6,1,0},
{0,0,0,0,0,0,0,1,6,7,2,0},
{0,0,0,0,0,0,0,1,5,8,3,0},
{0,0,0,0,0,0,0,1,4,9,4,1},
{0,0,0,0,0,0,0,0,3,8,5,1},
{0,0,0,0,0,0,0,0,2,7,6,1},
{0,0,0,0,0,0,0,0,1,6,7,2},
{0,0,0,0,0,0,0,0,1,5,8,3},
{0,0,0,0,0,0,0,0,1,4,9,4},
{0,0,0,0,0,0,0,0,0,3,8,5},
{0,0,0,0,0,0,0,0,0,2,7,6},
{0,0,0,0,0,0,0,0,0,1,6,7},
{0,0,0,0,0,0,0,0,0,1,5,8},
{0,0,0,0,0,0,0,0,0,1,6,9}
};


/*
const uint8_t ledpower[64][12] = {
{1,0,0,0,0,0,0,0,0,0,0,0},
{2,0,0,0,0,0,0,0,0,0,0,0},
{3,0,0,0,0,0,0,0,0,0,0,0},
{4,0,0,0,0,0,0,0,0,0,0,0},
{5,0,0,0,0,0,0,0,0,0,0,0},
{6,0,0,0,0,0,0,0,0,0,0,0},
{7,2,0,0,0,0,0,0,0,0,0,0},
{8,3,0,0,0,0,0,0,0,0,0,0},
{9,4,0,0,0,0,0,0,0,0,0,0},
{8,5,0,0,0,0,0,0,0,0,0,0},
{7,6,0,0,0,0,0,0,0,0,0,0},
{6,7,2,0,0,0,0,0,0,0,0,0},
{5,8,3,0,0,0,0,0,0,0,0,0},
{4,9,4,0,0,0,0,0,0,0,0,0},
{3,8,5,0,0,0,0,0,0,0,0,0},
{2,7,6,0,0,0,0,0,0,0,0,0},
{0,6,7,2,0,0,0,0,0,0,0,0},
{0,5,8,3,0,0,0,0,0,0,0,0},
{0,4,9,4,0,0,0,0,0,0,0,0},
{0,3,8,5,0,0,0,0,0,0,0,0},
{0,2,7,6,0,0,0,0,0,0,0,0},
{0,0,6,7,2,0,0,0,0,0,0,0},
{0,0,5,8,3,0,0,0,0,0,0,0},
{0,0,4,9,4,0,0,0,0,0,0,0},
{0,0,3,8,5,0,0,0,0,0,0,0},
{0,0,2,7,6,0,0,0,0,0,0,0},
{0,0,0,6,7,2,0,0,0,0,0,0},
{0,0,0,5,8,3,0,0,0,0,0,0},
{0,0,0,4,9,4,0,0,0,0,0,0},
{0,0,0,3,8,5,0,0,0,0,0,0},
{0,0,0,2,7,6,0,0,0,0,0,0},
{0,0,0,0,6,7,2,0,0,0,0,0},
{0,0,0,0,5,8,3,0,0,0,0,0},
{0,0,0,0,4,9,4,0,0,0,0,0},
{0,0,0,0,3,8,5,0,0,0,0,0},
{0,0,0,0,2,7,6,0,0,0,0,0},
{0,0,0,0,0,6,7,2,0,0,0,0},
{0,0,0,0,0,5,8,3,0,0,0,0},
{0,0,0,0,0,4,9,4,0,0,0,0},
{0,0,0,0,0,3,8,5,0,0,0,0},
{0,0,0,0,0,2,7,6,0,0,0,0},
{0,0,0,0,0,0,6,7,2,0,0,0},
{0,0,0,0,0,0,5,8,3,0,0,0},
{0,0,0,0,0,0,4,9,4,0,0,0},
{0,0,0,0,0,0,3,8,5,0,0,0},
{0,0,0,0,0,0,2,7,6,0,0,0},
{0,0,0,0,0,0,0,6,7,2,0,0},
{0,0,0,0,0,0,0,5,8,3,0,0},
{0,0,0,0,0,0,0,4,9,4,0,0},
{0,0,0,0,0,0,0,3,8,5,0,0},
{0,0,0,0,0,0,0,2,7,6,0,0},
{0,0,0,0,0,0,0,0,6,7,2,0},
{0,0,0,0,0,0,0,0,5,8,3,0},
{0,0,0,0,0,0,0,0,4,9,4,0},
{0,0,0,0,0,0,0,0,3,8,5,0},
{0,0,0,0,0,0,0,0,2,7,6,0},
{0,0,0,0,0,0,0,0,0,6,7,2},
{0,0,0,0,0,0,0,0,0,5,8,3},
{0,0,0,0,0,0,0,0,0,4,9,4},
{0,0,0,0,0,0,0,0,0,3,8,5},
{0,0,0,0,0,0,0,0,0,2,7,6},
{0,0,0,0,0,0,0,0,0,0,6,7},
{0,0,0,0,0,0,0,0,0,0,5,8},
{0,0,0,0,0,0,0,0,0,0,6,9}
};
*/
