#include "spi.h"

void SPI_init(SPI_type sType, SPI_dataSample sDataSample, SPI_clockIdle sClockIdle, SPI_transmitEdge sTransmitEdge)
{
    TRISAbits.RA5 = 0; 
    PORTAbits.RA5 = 1;
    TRISCbits.RC5 = 0;
    PIR1bits.SSPIF=0;
    if (sType & 0b00000100) //If Slave Mode
    {
        SSPSTAT = sTransmitEdge;
        TRISCbits.RC3 = 1;
    } else //If Master Mode
    {
        SSPSTAT = sDataSample | sTransmitEdge;
        TRISCbits.RC3 = 0;
    }
    SSPCON1 = sType | sClockIdle;
    ADCON0=0;			/* This is for de-multiplexed the SCL
				and SDI from analog pins*/
    ADCON1=0x0F;
}

void SPI_sendData(unsigned char data) {
    unsigned char data_flush;
    SSPBUF=data;			/* Copy data in SSBUF to transmit */

    while(!PIR1bits.SSPIF);	/* Wait for complete 1 byte transmission */
    PIR1bits.SSPIF=0;		/* Clear SSPIF flag */
    data_flush=SSPBUF;		/* Flush the data */
}

unsigned char SPI_read(void) 
{
   SSPBUF=0xff;		/* Copy flush data in SSBUF */
   while(!PIR1bits.SSPIF);	/* Wait for complete 1 byte transmission */
   PIR1bits.SSPIF=0;
   return(SSPBUF);
}

void spi_start(void)
{
	PORTAbits.RA5 = 0;
}

void spi_stop(void)
{
    PORTAbits.RA5 = 1;
}
