#ifndef _SPI_H_
#define _SPI_H_

#include <p18f4620.h>

#define TRIS_SS 	TRISDbits.RD0
#define TRIS_SCK	TRISDbits.RD1
#define TRIS_MOSI 	TRISDbits.RD2
#define TRIS_MISO	TRISDbits.RD3

#define SS          PORTDbits.RD0
#define SCK         PORTDbits.RD1
#define MOSI        PORTDbits.RD2
#define MISO        PORTDbits.RD3

#define WRITE_SPI   0x00
#define READ_SPI    0x80

//unsigned char BCD2Dec(unsigned char BCD);
//unsigned char Dec2BCD(unsigned char Dec);
void init_spi(void);
//void i2c_control(unsigned char mRW_bit);
void spi_start(void);
//void i2c_restart(void);
void spi_stop(void);
void spi_write(unsigned char address, unsigned char data);
unsigned char spi_read(unsigned char address);
void spi_read_new(unsigned char address, unsigned char count, unsigned char *values, unsigned char rxAlign);

#endif
