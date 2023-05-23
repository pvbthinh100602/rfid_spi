#ifndef _SPI_H_
#define _SPI_H_

#include <p18f4620.h>

typedef enum 
{
 SPI_MASTER_OSC_DIV4 = 0b00100000, // master mode, Fosc/4
 SPI_MASTER_OSC_DIV16 = 0b00100001, // master mode, Fosc/16
 SPI_MASTER_OSC_DIV64 = 0b00100010, // master mode, Fosc/63
 SPI_MASTER_TMR2     = 0b00100011, // master mode, timer 2 out
 SPI_SLAVE_SS_EN     = 0b00100100, // slave mode, enable SS
 SPI_SLAVE_SS_DIS    = 0b00100101 // slave mode, disable SS
} SPI_type; // mode hoat dong

typedef enum 
{
 SPI_DATA_SAMPLE_MIDDLE = 0b00000000, 
 SPI_DATA_SAMPLE_END   = 0b10000000
} SPI_dataSample; // thoi diem lay mau

typedef enum
{
 SPI_CLOCK_IDLE_HIGH = 0b00010000,
 SPI_CLOCK_IDLE_LOW = 0b00000000
} SPI_clockIdle; // trang thai nhan roi

typedef enum
{
 SPI_IDLE_2_ACTIVE = 0b00000000,
 SPI_ACTIVE_2_IDLE = 0b01000000
} SPI_transmitEdge; // canh truyen

void SPI_init(SPI_type sType, SPI_dataSample sDataSample, SPI_clockIdle sClockIdle, SPI_transmitEdge sTransmitEdge);
void SPI_sendData(unsigned char data);
unsigned char SPI_read(void);
void delay_spi(unsigned char time);
void spi_start(void);
void spi_stop(void);


#endif
