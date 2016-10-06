#ifndef _SPI_H
#define _SPI_H
#include "hw_config.h"
#define SPI_BLOCK_FREE 0
#define SPI_BLOCK_FAILURE 1
#define SPI_BUS_TRANSFER_STARTED 2
/*
 * For flash writes we use three different .
 * After the FLASH WRITE COMMAND STATE the interrupt handler should not pull CS High
 * but instead should initiate the data transfer
 *
 */

#define SPI_POTILED1_WRITE_DATA 11
#define SPI_POTILED2_WRITE_DATA 12
#define SPI_POTILED3_WRITE_DATA 13
#define SPI_POTILED4_WRITE_DATA 14
#define SPI_CHANNELLED_WRITE_DATA 15
#define SPI_SIEBENSEGMENT_WRITE_DATA 16

extern volatile uint8_t GL_spi1_block;
extern volatile uint8_t GL_spi_send_call;
extern volatile uint8_t GL_spi_irq_call;


#ifdef _USE_SPI_EEPROM
#define SPI_BLOCK_EEPROM_WRITE 4
#define SPI_BLOCK_EEPROM_COMMAND 5
#define SPI_BLOCK_EEPROM_DATA 6
#endif // _USE_SPI_FLASH

uint8_t SPI1_send(uint8_t, uint8_t, uint32_t , uint32_t);
void SPI1_BusInit(void);
void spi_handleDMA1Ch2Interrupt(void);
#endif //_SPI_H
