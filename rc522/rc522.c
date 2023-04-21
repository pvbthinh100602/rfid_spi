#include "rc522.h"
//unsigned char _RxBits;                                       // The number of received data bits
//unsigned char str[MAX_LEN];

Uid uid;
void init_RFID(void)
{
    init_spi();
// Reset baud rates
    rfid_reset();
	spi_write(TxModeReg, 0x00);
	spi_write(RxModeReg, 0x00);
	// Reset ModWidthReg
	spi_write(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	spi_write(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	spi_write(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25?s.
	spi_write(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	spi_write(TReloadRegL, 0xE8);
	
	spi_write(TxAutoReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	spi_write(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	antennaOn(); //Open the antenna
}

void rfid_reset() {
    unsigned char count = 0;

	spi_write(CommandReg, PCD_SOFTRESET);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74?s. Let us be generous: 50ms.
	//Mark2check
    do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		SetTimer0_ms(50);
        while(!flag_timer0);
        flag_timer0 = 0;
	} while ((spi_read(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

void antennaOn(void)
{
	unsigned char temp;

	temp = spi_read(TxControlReg);
	if ((temp & 0x03) != 0x03)
	{
		spi_write(TxControlReg, temp | 0x03);
	}
}

void antennaOff(void)
{
	clearBitMask(TxControlReg, 0x03);
}

void setBitMask(unsigned char reg, unsigned char mask)  
{
    unsigned char tmp;
    tmp = spi_read(reg);
    spi_write(reg, tmp | mask);  // set bit mask
}

void clearBitMask(unsigned char reg, unsigned char mask)  
{
    unsigned char tmp;
    tmp = spi_read(reg);
    spi_write(reg, tmp & (~mask));  // clear bit mask
} 



/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PICC_RequestA( unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
                            unsigned char *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

unsigned char PICC_REQA_or_WUPA(unsigned char command ,unsigned char *reqMode, unsigned char *bufferSize)
{
    unsigned char validBits, status;
    
	if (reqMode == 0 || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	clearBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, reqMode, bufferSize, &validBits,0,0);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
//    UartSendConstString("\nreq ok");
    
	return STATUS_OK;
}

unsigned char PCD_TransceiveData(	unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
													unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
													unsigned char *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													unsigned char checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	unsigned char waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_TRANSCEIVE, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

unsigned char PCD_CommunicateWithPICC(	unsigned char command,		///< The command to execute. One of the PCD_Command enums.
														unsigned char waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
														unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
														unsigned char *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														unsigned char checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	unsigned char txLastBits = validBits ? *validBits : 0;
	unsigned char bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
    unsigned char n	;
	unsigned char completed = 0;
    unsigned char errorRegValue;
	unsigned char _validBits;
    unsigned char controlBuffer[2];
    unsigned char status;
    spi_write(CommandReg, PCD_IDLE);			// Stop any active command.
	spi_write(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	spi_write(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	spi_write_new(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	spi_write(BitFramingReg, bitFraming);		// Bit adjustments
	spi_write(CommandReg, command);				// Execute the command
	if (command == PCD_TRANSCEIVE) {
		setBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the PCD stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out.
    SetTimer0_ms(36);
	while(1) {
		n = spi_read(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        UartSendNum(n);
        UartSendConstString("\n");
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			            UartSendConstString("\nNo TimeOut");
            completed = 1;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms

			return STATUS_TIMEOUT;
		}
//		yield();
	}
    flag_timer0 = 0;
    SetTimer0_ms(0);
	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!completed) {
		return STATUS_TIMEOUT;
	}
           
	// Stop now if any errors except collisions were detected.
    errorRegValue = spi_read(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
	_validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = spi_read(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;
		spi_read_new(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = spi_read(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
//		// In this case a MIFARE Classic NAK is not OK.
//		if (*backLen == 1 && _validBits == 4) {
//			return STATUS_MIFARE_NACK;
//		}
//		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
//		if (*backLen < 2 || _validBits != 0) {
//			return STATUS_CRC_WRONG;
//		}
//		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
//		
//		status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
//		if (status != STATUS_OK) {
//			return status;
//		}
//		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
//			return STATUS_CRC_WRONG;
//		}
	}
//    UartSendConstString("\nPass ok");
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PCD_CalculateCRC(	unsigned char *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								unsigned char length,	///< In: The number of bytes to transfer.
								unsigned char *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	unsigned char n;
    spi_write(CommandReg, PCD_IDLE);		// Stop any active command.
	spi_write(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	spi_write(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	spi_write(FIFODataReg, length, data);	// Write data to the FIFO
	spi_write(CommandReg, PCD_CALCCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Check for the register to
	// indicate that the CRC calculation is complete in a loop. If the
	// calculation is not indicated as complete in ~90ms, then time out
	// the operation.
//Matk2Check
//	const uint32_t deadline = millis() + 89;

    while(1){
        n = spi_read(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			spi_write(CommandReg, PCD_IDLE);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = spi_read(CRCResultRegL);
			result[1] = spi_read(CRCResultRegH);
			return STATUS_OK;
		}
    }
//	do {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
//		n = spi_read(DivIrqReg);
//		if (n & 0x04) {									// CRCIRq bit set - calculation done
//			spi_write(CommandReg, PCD_IDLE);	// Stop calculating CRC for new content in the FIFO.
//			// Transfer the result from the registers to the result buffer
//			result[0] = spi_read(CRCResultRegL);
//			result[1] = spi_read(CRCResultRegH);
//			return STATUS_OK;
//		}
//		yield();
//	}
//	while (static_cast<uint32_t> (millis()) < deadline);

	// 89ms passed and nothing happened. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()

unsigned char PICC_IsNewCardPresent() {
	unsigned char bufferATQA[2];
	unsigned char bufferSize = sizeof(bufferATQA);
    unsigned char result;
	// Reset baud rates
	spi_write(TxModeReg, 0x00);
	spi_write(RxModeReg, 0x00);
	// Reset ModWidthReg
	spi_write(ModWidthReg, 0x26);

	result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

unsigned char PICC_ReadCardSerial() {
	unsigned char result = PICC_Select(&uid,0);
	return (result == STATUS_OK);
}

unsigned char PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
							unsigned char validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	unsigned char uidComplete;
	unsigned char selectDone;
	unsigned char useCascadeTag;
	unsigned char cascadeLevel = 1;
	unsigned char result;
	unsigned char count;
	unsigned char checkBit;
	unsigned char index;
	unsigned char uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	unsigned char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	unsigned char buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	unsigned char bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	unsigned char rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	unsigned char txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	unsigned char *responseBuffer;
	unsigned char responseLength;
    unsigned char bytesToCopy;
    unsigned char maxBytes;
    unsigned char valueOfCollReg;
    unsigned char collisionPos;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	clearBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = 0;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = 0;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
            maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = 0;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			spi_write(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,0);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				valueOfCollReg = spi_read(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = 1; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
//		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
//		if (result != STATUS_OK) {
//			return result;
//		}
//		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
//			return STATUS_CRC_WRONG;
//		}
//		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
//			cascadeLevel++;
//		}
//		else {
//			uidComplete = 1;
//			uid->sak = responseBuffer[0];
//		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

void PCD_DumpVersionToSerial() {
	// Get the MFRC522 firmware version
	unsigned char v = spi_read(VersionReg);
	UartSendConstString("\nFirmware Version: 0x");
	UartSendNum(v);
	// Lookup which version
	switch(v) {
		case 0x88: UartSendConstString(" = (clone)");  break;
		case 0x90: UartSendConstString(" = v0.0");     break;
		case 0x91: UartSendConstString(" = v1.0");     break;
		case 0x92: UartSendConstString(" = v2.0");     break;
		case 0x12: UartSendConstString(" = counterfeit chip");     break;
		default:   UartSendConstString(" = (unknown)");
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF))
		UartSendConstString("WARNING: Communication failure, is the MFRC522 properly connected?");
}

void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
								) {
	MIFARE_Key key;
    unsigned char i;
	
	// Dump UID, SAK and Type
//	PICC_DumpDetailsToSerial(uid);
	
	// Dump contents
	unsigned char piccType = PICC_GetType(uid->sak);
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for ( i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
            UartSendConstString("PICC_TYPE_MIFARE_4K");
//			PICC_DumpMifareClassicToSerial(uid, piccType, &key);
			break;
			
		case PICC_TYPE_MIFARE_UL:
            UartSendConstString("PICC_TYPE_MIFARE_UL");
//			PICC_DumpMifareUltralightToSerial();
			break;
			
		case PICC_TYPE_ISO_14443_4:
		case PICC_TYPE_MIFARE_DESFIRE:
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			UartSendConstString("Dumping memory contents not implemented for that PICC type.");
			break;
			
		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	
//	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

//void MPICC_DumpMifareUltralightToSerial() {
//	unsigned char status;
//	unsigned char byteCount;
//	unsigned char buffer[18];
//	unsigned char i;
//	
//	UartSendConstString("Page  0  1  2  3");
//	// Try the mpages of the original Ultralight. Ultralight C has more pages.
//	for (unsigned char page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
//		// Read pages
//		byteCount = sizeof(buffer);
//		status = MIFARE_Read(page, buffer, &byteCount);
//		if (status != STATUS_OK) {
//			UartSendConstString("MIFARE_Read() failed: ");
//			Serial.println(GetStatusCodeName(status));
//			break;
//		}
//		// Dump data
//		for (byte offset = 0; offset < 4; offset++) {
//			i = page + offset;
//			if(i < 10)
//				Serial.print(F("  ")); // Pad with spaces
//			else
//				Serial.print(F(" ")); // Pad with spaces
//			Serial.print(i);
//			Serial.print(F("  "));
//			for (byte index = 0; index < 4; index++) {
//				i = 4 * offset + index;
//				if(buffer[i] < 0x10)
//					Serial.print(F(" 0"));
//				else
//					Serial.print(F(" "));
//				Serial.print(buffer[i], HEX);
//			}
//			Serial.println();
//		}
//	}
//} // End PICC_DumpMifareUltralightToSerial()

unsigned char MIFARE_Read(	unsigned char blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
											unsigned char *buffer,		///< The buffer to store the data in
											unsigned char *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	unsigned char result;
	
	// Sanity check
	if (buffer == 0 || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}
	
	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, 0, 0, 1);
} // End MIFARE_Read()

unsigned char PICC_GetType(unsigned char sak) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} 

unsigned char PICC_WakeupA(	unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											unsigned char *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
}