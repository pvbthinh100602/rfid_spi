#include "spi.h"

void init_spi(void)
{
    TRIS_SS = 0;
    TRIS_SCK = 0;
    TRIS_MOSI = 0;
    TRIS_MISO = 1;
    SS = 1;
    SCK = 1;
}

void delay_spi(unsigned char time)
{
    unsigned char i;
    for (i = 0; i < time; i++);
}

char read_MISO_pin(void)
{
	if (MISO == 0)
	{
		return 0;
	}
	
	return 1;
}

void spi_start(void)
{
	SS = 0;
}

void spi_stop(void)
{
	SS = 1;
}

unsigned char spi_transfer_byte(unsigned char data)
{
    unsigned char i, read;
    read = 0;
    SCK = 0;
    for(i = 0; i < 8; i++){
        MOSI = (data & (0x80 >> i)) ? 1 : 0;
        
        //delay
        
        SCK = 1;
        
        //read
        read = (read << 1);
        if(read_MISO_pin()) read |= 0x01;
        
        //delay
        SCK = 0;
    }
    return read;
}

void spi_write(unsigned char address, unsigned char data){
    spi_start();
    spi_transfer_byte(WRITE_SPI | (address << 1));
    spi_transfer_byte(data);
    spi_stop();
}

unsigned char spi_read(unsigned char address){
    unsigned char data;
    spi_start();
    spi_transfer_byte(READ_SPI | (address << 1));
    data = spi_transfer_byte(0);
    spi_stop();
    return data;
}

void spi_read_new(unsigned char address, unsigned char count, unsigned char *values, unsigned char rxAlign){
    unsigned char mask;
    unsigned char value;
    unsigned char index = 0;							// Index in values array.
    if (count == 0) {
		return;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	
    spi_start();
	count--;								// One read is performed outside of the loop
	spi_transfer_byte(READ_SPI | (address << 1));					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		value = spi_transfer_byte(READ_SPI | (address << 1));
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = spi_transfer_byte(READ_SPI | (address << 1));	// Read value and tell that we want to read the same address again.
		index++;
	}
	values[index] = spi_transfer_byte(0);			// Read the final byte. Send 0 to stop reading.
	spi_stop();
}

void spi_write_new(	unsigned char address,	///< The register to write to. One of the PCD_Register enums.
									unsigned char count,			///< The number of bytes to write to the register
									unsigned char *values		///< The values to write. Byte array.
								) {
	unsigned char index;
    spi_start();
	spi_transfer_byte(WRITE_SPI | (address << 1));						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for ( index = 0; index < count; index++) {
		spi_transfer_byte(values[index]);
	}
	spi_stop();
}
