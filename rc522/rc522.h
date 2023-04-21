/* 
 * File:   rc522.h
 * Author: phamv
 *
 * Created on March 30, 2023, 2:28 PM
 */

#ifndef _RC522_H_
#define	_RC522_H_

#include <p18f4620.h>

#include "spi.h"
#include "uart/uart.h"
#include "timer/timer.h"
#include "interrupt/interrupt.h"
//AddicoreRFID Primary Coupling Device (PCD according to the ISO14443) Commands
#define PCD_IDLE                0x00  // no action, cancels current command execution
#define PCD_MEM                 0x01  // stores 25 bytes into the internal buffer
#define PCD_GENRANDOMID         0x02  // generates a 10-byte random ID number
#define PCD_CALCCRC             0x03  // activates the CRC calculation or performs a self test
#define PCD_TRANSMIT            0x04  // Transmit data
#define PCD_NOCMDCHANGE         0x07  // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_RECEIVE             0x08  // activates the receiver circuits (receive data)
#define PCD_TRANSCEIVE          0x0C  // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission (transmit and receive data)
#define PCD_MFAUTHENT           0x0E  // performs the MIFARE standard authentication as a reader (authentication)
#define PCD_SOFTRESET           0x0F  // resets the MFRC522

// AddicoreRFID Proximity Integrated Circuit Card (PICC) Commands
#define PICC_CMD_REQA			0x26		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define	PICC_CMD_WUPA			0x52		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define	PICC_CMD_CT				0x88		// Cascade Tag. Not really a command, but used during anti collision.
#define	PICC_CMD_SEL_CL1		0x93		// Anti collision/Select, Cascade Level 1
#define	PICC_CMD_SEL_CL2		0x95		// Anti collision/Select, Cascade Level 2
#define	PICC_CMD_SEL_CL3		0x97		// Anti collision/Select, Cascade Level 3
#define	PICC_CMD_HLTA			0x50		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
#define	PICC_CMD_RATS           0xE0    // Request command for Answer To Reset.
		// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
		// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
		// The read/write commands can also be used for MIFARE Ultralight.
#define	PICC_CMD_MF_AUTH_KEY_A	0x60		// Perform authentication with Key A
#define	PICC_CMD_MF_AUTH_KEY_B	0x61		// Perform authentication with Key B
#define	PICC_CMD_MF_READ		0x30		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define	PICC_CMD_MF_WRITE		0xA0		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define	PICC_CMD_MF_DECREMENT	0xC0		// Decrements the contents of a block and stores the result in the internal data register.
#define	PICC_CMD_MF_INCREMENT	0xC1		// Increments the contents of a block and stores the result in the internal data register.
#define	PICC_CMD_MF_RESTORE		0xC2		// Reads the contents of a block into the internal data register.
#define	PICC_CMD_MF_TRANSFER	0xB0		// Writes the contents of the internal data register to a block.
        // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
		// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define	PICC_CMD_UL_WRITE		0xA2		// Writes one 4 byte page to the PICC.

// AddicoreRFID PICC Responses
#define MI_ACK                  0xA   // The 4-bit acknowledgement returned from a MIFARE Classic PICC
#define MI_ATQA                 0x04 //The 16-bit ATQA (Answer To Request acc. to ISO/IEC 14443-4) response from PICC
#define MI_SAK                  0x08  // The 8-bit SAK (Select Acknowledge, Type A) response from PICC

// AddicoreRFID Default Definable Pins
//#define DEFAULT_CHIP_SELECT_PIN         10
//#define DEFAULT_RESET_PIN               5

//AddicoreRFID error codes
#define MI_OK                   0
#define MI_NO_TAG_ERR           1
#define MI_ERR                  2

#define		STATUS_OK				0	// Success
#define		STATUS_ERROR			1	// Error in communication
#define		STATUS_COLLISION		2	// Collission detected
#define		STATUS_TIMEOUT			3	// Timeout in communication.
#define		STATUS_NO_ROOM			4// A buffer is not big enough.
#define		STATUS_INTERNAL_ERROR	5	// Internal error in the code. Should not happen ;-)
#define		STATUS_INVALID			6	// Invalid argument.
#define		STATUS_CRC_WRONG		7	// The CRC_A does not match
#define		STATUS_MIFARE_NACK		0xff	// A MIFARE PICC responded with NAK.

////////////////////////////////////////////////////////////////
// AddicoreRFID Registers                   
////////////////////////////////////////////////////////////////
/* The information below regarding the MFRC522 registers is from 
   the MFRC522 Manufacturer's Datasheet (Rev. 3.6 ? 14 December 2011), Table 20
*/
//Page 0: Command and Status
#define     Reserved00          0x00  // reserved for future use
#define     CommandReg          0x01  // starts and stops command execution
#define     ComIEnReg           0x02  // enable and disable the passing of interrupt requests to IRq pin
#define     DivlEnReg           0x03  // enable and disable interrupt request control bits
#define     ComIrqReg           0x04  // interrupt request bits
#define     DivIrqReg           0x05  // interrupt request bits
#define     ErrorReg            0x06  // error bits showing the error status of the last command executed
#define     Status1Reg          0x07  // communication status bits
#define     Status2Reg          0x08  // receiver and transmitter status bits
#define     FIFODataReg         0x09  // input and output of 64 byte FIFO buffer
#define     FIFOLevelReg        0x0A  // number of bytes stored in the FIFO buffer
#define     WaterLevelReg       0x0B  // level for FIFO underflow and overflow warning
#define     ControlReg          0x0C  // miscellaneous control registers
#define     BitFramingReg       0x0D  // adjustments for bit-oriented frames
#define     CollReg             0x0E  // bit position of the first bit-collision detected on the RF interface
#define     Reserved01          0x0F  // reserved for future use
//Page 1: Command     
#define     Reserved10          0x10  // reserved for future use
#define     ModeReg             0x11  // defines general modes for transmitting and receiving
#define     TxModeReg           0x12  // defines transmission data rate and framing
#define     RxModeReg           0x13  // defines reception data rate and framing
#define     TxControlReg        0x14  // controls the logical behavior of the antenna driver pins TX1 and TX2
#define     TxAutoReg           0x15  // controls the setting of the transmission modulation
#define     TxSelReg            0x16  // selects the internal sources for the antenna driver
#define     RxSelReg            0x17  // selects internal receiver settings
#define     RxThresholdReg      0x18  // selects thresholds for the bit decoder
#define     DemodReg            0x19  // defines demodulator settings
#define     Reserved11          0x1A  // reserved for future use
#define     Reserved12          0x1B  // reserved for future use
#define     MfTxReg             0x1C  // controls some MIFARE communication transmit parameters
#define     MfRxReg             0x1D  // controls some MIFARE communication receive parameters
#define     Reserved13          0x1E  // reserved for future use
#define     SerialSpeedReg      0x1F  // selects the speed of the serial UART interface
//Page 2: Configuration  
#define     Reserved20          0x20  // reserved for future use
#define     CRCResultRegH       0x21  // shows the MSB values of the CRC calculation
#define     CRCResultRegL       0x22  // shows the LSB values of the CRC calculation
#define     Reserved21          0x23  // reserved for future use
#define     ModWidthReg         0x24  // controls the ModWidth setting
#define     Reserved22          0x25  // reserved for future use
#define     RFCfgReg            0x26  // configures the receiver gain
#define     GsNReg              0x27  // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define     CWGsPReg            0x28  // defines the conductance of the p-driver output during periods of no modulation
#define     ModGsPReg           0x29  // defines the conductance of the p-driver output during periods of modulation
#define     TModeReg            0x2A  // defines settings for the internal timer
#define     TPrescalerReg       0x2B  // defines settings for the internal timer
#define     TReloadRegH         0x2C  // defines the higher 8 bits of the 16-bit timer reload value
#define     TReloadRegL         0x2D  // defines the lower 8 bits of the 16-bit timer reload value
#define     TCounterValueRegH   0x2E  // shows the higher 8 bits of the 16-bit timer value
#define     TCounterValueRegL   0x2F  // shows the lower 8 bits of the 16-bit timer value
//Page 3: Test Registers    
#define     Reserved30          0x30  // reserved for future use
#define     TestSel1Reg         0x31  // general test signal configuration
#define     TestSel2Reg         0x32  // general test signal configuration and PRBS control
#define     TestPinEnReg        0x33  // enables pin output driver on pins D1 to D7
#define     TestPinValueReg     0x34  // defines the values for D1 to D7 when it is used as an I/O bus
#define     TestBusReg          0x35  // shows the status of the internal test bus
#define     AutoTestReg         0x36  // controls the digital self test
#define     VersionReg          0x37  // shows the software version
#define     AnalogTestReg       0x38  // controls the pins AUX1 and AUX2
#define     TestDAC1Reg         0x39  // defines the test value for TestDAC1
#define     TestDAC2Reg         0x3A  // defines the test value for TestDAC2
#define     TestADCReg          0x3B  // shows the value of ADC I and Q channels
#define     Reserved31          0x3C  // reserved for production tests
#define     Reserved32          0x3D  // reserved for production tests
#define     Reserved33          0x3E  // reserved for production tests
#define     Reserved34          0x3F  // reserved for production tests

//PICC_TYPE
#define		PICC_TYPE_UNKNOWN		0
#define		PICC_TYPE_ISO_14443_4	1	// PICC compliant with ISO/IEC 14443-4 
#define		PICC_TYPE_ISO_18092		2 	// PICC compliant with ISO/IEC 18092 (NFC)
#define		PICC_TYPE_MIFARE_MINI	3	// MIFARE Classic protocol, 320 bytes
#define		PICC_TYPE_MIFARE_1K		4	// MIFARE Classic protocol, 1KB
#define		PICC_TYPE_MIFARE_4K		5	// MIFARE Classic protocol, 4KB
#define		PICC_TYPE_MIFARE_UL		6	// MIFARE Ultralight or Ultralight C
#define		PICC_TYPE_MIFARE_PLUS	7	// MIFARE Plus
#define		PICC_TYPE_MIFARE_DESFIRE 8	// MIFARE DESFire
#define		PICC_TYPE_TNP3XXX		9	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
#define		PICC_TYPE_NOT_COMPLETE	0xff	// SAK indicates UID is not complete.


typedef struct {
	unsigned char		size;			// Number of bytes in the UID. 4, 7 or 10.
	unsigned char		uidByte[10];
	unsigned char		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

extern Uid uid;

typedef struct {
    unsigned char keyByte[6];
} MIFARE_Key;

//Uid uid;

void init_RFID(void);
void antennaOn(void);
void antennaOff(void);
void setBitMask(unsigned char reg, unsigned char mask);
void clearBitMask(unsigned char reg, unsigned char mask);
void rfid_reset();
unsigned char PICC_IsNewCardPresent();
unsigned char PCD_TransceiveData(	unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
													unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
													unsigned char *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													unsigned char checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 );
unsigned char PICC_REQA_or_WUPA(unsigned char command ,unsigned char *reqMode, unsigned char *bufferSize);
unsigned char PCD_CommunicateWithPICC(	unsigned char command,		///< The command to execute. One of the PCD_Command enums.
														unsigned char waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
														unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
														unsigned char *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														unsigned char checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 );
unsigned char PCD_CalculateCRC(	unsigned char *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								unsigned char length,	///< In: The number of bytes to transfer.
								unsigned char *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 );
unsigned char PICC_Select(Uid *uid,unsigned char validBits);
unsigned char PICC_GetType(unsigned char sak);
unsigned char PICC_WakeupA(	unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											unsigned char *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										);
#endif	/* RC522_H */

