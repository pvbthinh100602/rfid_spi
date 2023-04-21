#include "main.h"
#define _XTAL_FREQ 13560000
unsigned char status;
unsigned char str[16];

void init_system(void);
void delay_ms(int value);

void main(void)
{
    
    unsigned int k = 0;
    init_system();
//    lcd_clear();
    LcdClearS();
    LcdPrintStringS(0,0,"HELLO");
    DisplayLcdScreen();
    delay_ms(1000);
//    spi_start();
    PCD_DumpVersionToSerial();
//    spi_transfer_byte(0x33);
//    spi_stop();
    while (1)
    {
        while (!flag_timer3);
        flag_timer3 = 0;
//        str[1] = 0x4400;
//        status = RFID_Request(PICC_REQIDL, str);
//        if (status == MI_OK) LcdPrintStringS(0,0,"RFID tag detected");
//        LcdPrintNumS(1,0,status);
//        scan_key_matrix();
//        DisplayLcdScreen();
        	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
        if (PICC_IsNewCardPresent()) {
//            if (PICC_ReadCardSerial()) {
                        PCD_DumpVersionToSerial();
//            }
        }
//
//        // Select one of the cards

//        if(1) continue;
        // Dump debug info about the card; PICC_HaltA() is automatically called
//        PICC_DumpToSerial(&(uid));
    }
}

void delay_ms(int value)
{
    int i,j;
    for(i = 0; i < value; i++)
        for(j = 0; j < 160; j++);
}

void init_system(void)
{
    TRISB = 0x00;		//setup PORTB is output
//    TRISD = 0x00;
    init_lcd();
//    LED = 0x00;
    init_interrupt();
    delay_ms(1000);
    init_timer0(4695);  //dinh thoi 1ms
//    init_timer1(9390);  //dinh thoi 2ms
    init_timer3(46950); //dinh thoi 10ms
    SetTimer0_ms(0);
//    SetTimer1_ms(10);
    SetTimer3_ms(50);   //Chu ky thuc hien viec xu ly input,proccess,output
    init_key_matrix();
    init_RFID();
    init_uart();
}
