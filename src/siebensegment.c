#include "siebensegment.h"
#include "spi.h"

void siebensegment_send_number(uint8_t number)
{
	uint8_t a,b,c;
	while(number > 128)
	{
		number -=100;
	}
	a=number%10;
	number/=10;
	b=number%10;
	number/=10;
	c=number;
	siebensegment_send_data(c,b,a);
}

/*
 * siebensegment send data takes three integers between 0 and 36 which are translated into the displayed leds
 * by the characters table
 */
uint8_t siebensegment_send_data(uint8_t code1, uint8_t code2, uint8_t code3)
{
	uint8_t i;
	static uint8_t regdata[4];
	if(GL_spi1_block != SPI_BLOCK_FREE)
		return 2;
	regdata[0]=SIEBENSEGMENT_CMD_DIGIT1;
	regdata[1]= characters[asciitocharacter(code1)];
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);
	regdata[2]=SIEBENSEGMENT_CMD_DIGIT2;
	regdata[3]= characters[asciitocharacter(code2)];
	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}

	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, &regdata[2], &regdata[2]);
	regdata[0]=SIEBENSEGMENT_CMD_DIGIT3;
	regdata[1]=characters[asciitocharacter(code3)];
	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}

	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);
	return 0;
}

void siebensegment_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(SIEBENSEGMENT_PERIPH , ENABLE);


	/* Configure Chip Select (CS) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = SIEBENSEGMENT_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SIEBENSEGMENT_CS_GPIO_PORT, &GPIO_InitStructure);


	/* Set Pins to it default state, high in case of the AS1108 */
	GPIO_WriteBit(SIEBENSEGMENT_CS_GPIO_PORT, SIEBENSEGMENT_CS_PIN, Bit_RESET);

}

void siebensegment_config(void)
{
	uint8_t regdata[2];
	uint16_t* pointer;
	uint8_t i;
	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}

	regdata[0]=SIEBENSEGMENT_CMD_FEATURE;
	regdata[1]= (0x00 | SIEBENSEGMENT_FEATURE_DECODE_HEX | SIEBENSEGMENT_FEATURE_IF_SPI);
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_SCANLIM;
	regdata[1]= SIEBENSEGMENT_SCANLIM123;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_DECMODE;
	regdata[1]=SIEBENSEGMENT_DECMODE_NODEC;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_SHDOWN;
	regdata[1]= SIEBENSEGMENT_SHDOWN_NORMAL_FEATURE;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_DIGIT1;
	regdata[1]= 0x03;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_DIGIT2;
	regdata[1]= 0x03;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);
	while(GL_spi1_block != SPI_BLOCK_FREE)
	{
		if(i<64)
		{
			i++;
		}
		else
		{
			i=0;
		}
	}
	regdata[0]=SIEBENSEGMENT_CMD_DIGIT3;
	regdata[1]= 0x05;
	GL_spi1_block = SPI_SIEBENSEGMENT_WRITE_DATA;
	SPI1_send(2,SPI_SIEBENSEGMENT_WRITE_DATA, regdata, regdata);

}

uint8_t asciitocharacter(uint8_t ascii)
{
	/*
	 * Keep the function universal.
	 * if the provided value is smaller than 0x25 it is probably already converted
	 */
	if(ascii < 0x20)
		return ascii;
	switch(ascii)
	{
	case 0x20:
		return 0x24;
		break;
	case 0x30:
		return 0x00;
		break;
	case 0x31:
		return 0x01;
		break;
	case 0x32:
		return 0x02;
		break;
	case 0x33:
		return 0x03;
		break;
	case 0x34:
		return 0x04;
		break;
	case 0x35:
		return 0x05;
		break;
	case 0x36:
		return 0x06;
		break;
	case 0x37:
		return 0x07;
		break;
	case 0x38:
		return 0x08;
		break;
	case 0x39:
		return 0x09;
		break;
	case 0x41:
	case 0x61:
		return 0x0A; //A
		break;
	case 0x42:
	case 0x62:
		return 0x0B; //b
		break;
	case 0x43:
		return 0x0C; //C
		break;
	case 0x63:
		return 0x0D; //c
		break;
	case 0x44:
	case 0x64:
		return 0x0E; //d
		break;
	case 0x45:
	case 0x65:
		return 0x0F; //E
		break;
	case 0x46:
	case 0x66:
		return 0x10; //F
		break;
	case 0x47:
	case 0x67:
		return 0x11; //g
		break;
	case 0x48:
	case 0x68:
		return 0x12; //H
		break;
	case 0x49:
	case 0x69:
		return 0x13; //I
		break;
	case 0x4A:
	case 0x6A:
		return 0x14; //J
		break;
	case 0x4B:
	case 0x6B:
		return 0x12; //K is looking like H.
	case 0x4C:
	case 0x6C:
		return 0x15; //L
		break;
	case 0x4D:
	case 0x6D:
		return 0x16; //M
		break;
	case 0x4E:
	case 0x6E:
		return 0x17; //n
		break;
	case 0x4F:
		return 0x18; //O
		break;
	case 0x6F:
		return 0x19; //o
		break;
	case 0x50:
	case 0x70:
		return 0x1A; //P
		break;
	case 0x51:
	case 0x71:
		return 0x1B; //Q
		break;
	case 0x52:
	case 0x72:
		return 0x1C; //r
		break;
	case 0x53:
	case 0x73:
		return 0x1D; //S
		break;
	case 0x54:
	case 0x74:
		return 0x1E; //t
		break;
	case 0x55:
	case 0x75:
		return 0x1F; //U
		break;
	case 0x56:
	case 0x76:
		return 0x20; //v
		break;
	case 0x57:
	case 0x77:
	case 0x21:
		return 0x21; //W
		break;
	case 0x58:
	case 0x78:
		return 0x12; // There is no X .... We take H for it
		break;
	case 0x59:
	case 0x79:
	case 0x22:
		return 0x22; //Y
		break;
	case 0x5A:
	case 0x7A:
	case 0x23:
		return 0x23; //Z
		break;
	case 0x2D:
	case 0x25:
		return 0x25; //-
		break;
	default:
		return 0x24; //Space
		break;
	}
	return 0x24;
}

/*
 * The characters are in this order beginning with 0
 * 0,1,2,3,4,5,6,7,8,9,A,b,C,c,d,E,F,g,H,I,J,L,M,n,O,o,P,Q,r,S,t,U,v,W,Y,Z,[SPACE],-
 */
const uint8_t characters[38]=
{
		0x7E,	//0
		0x30,	//1
		0x6D,	//2
		0x79,	//3
		0x33,	//4
		0x5B,	//5
		0x5F,	//6
		0x70,	//7
		0x7F,	//8
		0x7B,	//9
		0x77,	//A
		0x1F,	//b
		0x4E,	//C
		0x0D,	//c
		0x3D,	//d
		0x4F,	//E
		0x47,	//F
		0x7B,	//g
		0x37,	//H
		0x06,	//I
		0x3C,	//J
		0x0E,	//L
		0x54,	//M
		0x15,	//n
		0x7E,	//O
		0x1D,	//o
		0x67,	//P
		0x73,	//Q
		0x15,	//R
		0x5B,	//S
		0x0F,	//t
		0x3E,	//U
		0x1C,	//v
		0x2A,	//W
		0x3B,	//Y
		0x6D,	//Z
		0x00,	//[space]
		0x01,	//-
};


