#include "main.h"
#define _XTAL_FREQ 20000000
unsigned char status;

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
    PCD_DumpVersionToSerial();

    while (1)
    {
        while (!flag_timer3);
        flag_timer3 = 0;
//        scan_key_matrix();
//        DisplayLcdScreen();
        	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
        if (PICC_IsNewCardPresent()) {
            if (PICC_ReadCardSerial()) {
                PICC_DumpToSerial(&uid);
            }
        }
        DisplayLcdScreen();
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

//    ADCON1 = 0;
//    LED = 0x00;
    init_interrupt();
    delay_ms(1000);
    init_timer0(4695);  //dinh thoi 1ms
//    init_timer1(9390);  //dinh thoi 2ms
    init_timer3(46950); //dinh thoi 10ms
//    SetTimer1_ms(10);
    SetTimer3_ms(50);   //Chu ky thuc hien viec xu ly input,proccess,output
//    init_key_matrix();   
    init_uart();
    init_RFID();
}
