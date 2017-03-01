/*
 * File:   Menu_System.c
 * Author: d00152869
 *
 * Created on 22 February 2017, 11:41
 */

/****************************************************
		Libraries included
*****************************************************/
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../Buttons_Debounce_State_Mch-master/Buttons_Debounce.X/Buttons_Debounce.h"
#include "../../LCD_library/lcdlib_2016.h"

#include <plib/timers.h>
//#include <plib/usart.h>
//#include <plib/can2510.h>

/***********************************************
		Configuration Selection bits
************************************************/

/*************************************************
					Clock
*************************************************/
#define _XTAL_FREQ 19660800

/************************************************
			Global Variables    
*************************************************/
const unsigned char msg_ary[10][16] = {"Desired> ", "Actual> ", 
                                       "Desired> ", "ADCRead> "};

const unsigned char * problem = "Problem";

const unsigned char * startup = "Ready to go";
										  

/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);

/************************************************
 Interrupt Function 
*************************************************/
unsigned char count_hb =0;
unsigned char count = 0;
bit Tick_E = 0;
void __interrupt myIsr(void)
{
    //Timer overflows every 10mS
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        
        Find_Button_Press();       //check the buttons every 10mS
        WriteTimer0(40960); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
              
        count++;
        count_hb++;
        if(count == 10){
            count = 0;
            Tick_E = 1;
        }
        //Heartbeat signal
        if(count_hb == 100){
            PORTCbits.RC7 = ~PORTCbits.RC7;   //check the timer overflow rate
            count_hb = 0;                   //Toggle every 1 second (heartbeat))
        }
    }
}

//declare Button
Bit_Mask Button_Press;	

/************************************************
			Macros
*************************************************/
#define MENU_E Button_Press.B0
#define ENTER_E Button_Press.B1

/*****************************************
 			Main Function
******************************************/

void main ( void ) 
{
    unsigned char Desired_Value = 50;
    unsigned char Actual_Value = 50;
    unsigned char Temp_Value = 0;
    
    typedef  enum {MENU_0 = 0,MENU_1} states;
    
    states  my_mch_state = MENU_1;  
    
    typedef struct{
        unsigned char desired;
        unsigned char actual;
        unsigned char temp;
    } motor;
    motor motor1 = {50,50,0};
    Initial();
    lcd_start ();
    lcd_cursor ( 0, 0 ) ;
    lcd_print ( startup ) ;
    
    delay_s(2);
    //Initial LCD Display
    Window(0);
    lcd_cursor ( 10, 0 ) ;
    lcd_display_value(motor1.desired);
    lcd_cursor ( 10, 1 ) ;
    lcd_display_value(motor1.actual);
    
    while(1)
    {
		
		//wait for a button Event
		//while(!MENU_E && !ENTER_E && !UP_E && !DOWN_E);  
		while(!Got_Button_E && !Tick_E);
        
		switch(my_mch_state)	
		{
			case MENU_0: 
				if (MENU_E){
                    my_mch_state = MENU_1; //state transition
                    Window(1);             //OnEntry action
                }
                
				break;
			case MENU_1: 
				if (MENU_E){
                    my_mch_state = MENU_0;  //state transition
                    Window(0);              //OnEntry action
                }
				break;
			default: 
				if (MENU_E){
                    my_mch_state = MENU_0;
                    Window(0);
                }
				break;
		}
		
		
		switch(my_mch_state)	
		{
			case MENU_0: 
				lcd_cursor ( 10, 0 ) ;    //state actions
                lcd_display_value(motor1.desired);
                lcd_cursor ( 10, 1 ) ;
                lcd_display_value(motor1.actual);
                LATC = PORTCbits.RC7 << 7 | 0x1 ;
				
				break;
			case MENU_1: 
                if (ENTER_E)          //state actions with guard
                    motor1.desired = motor1.temp;
                //ADC Read and Update Temp_Value
                while(ADCON0bits.GO_nDONE); //In case Conversion is not ready
                motor1.temp = ADRESH >> 2;
                ADCON0bits.GO_nDONE = 1; //Start ADC Conversion
                
				lcd_cursor ( 10, 0 ) ;
                lcd_display_value(motor1.desired);
				lcd_cursor ( 10, 1 ) ;
                lcd_display_value(motor1.temp);
                LATC= PORTCbits.RC7 << 7 | 0x2;
				break;
			
			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem );
                LATC = PORTCbits.RC7 << 7 | 0x5;
				break;
		}
		
        Button_Press.Full = 0;  //Clear all events since only allowing one button event at a time
                                //which will be dealt with immediately
        Tick_E = 0;         //Clear tick event
    }
}   


void Initial(void){
    ADCON0 = 0x01; //Enable ADC on AN0
    ADCON1 = 0x0E; //AN0 = Analog Input
	TRISB = 0xFF; //Buttons
	TRISC = 0x00;   //LEDS
    ADCON0bits.GO_nDONE = 1; //Start ADC Conversion
	LATC = 0xff;
	delay_s(3);
	LATC = 0x00;
    
    //0n, 16bit, internal clock(Fosc/4), prescale by 2)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    WriteTimer0(40960);  //65,536 - 24,576  //overflows every 10mS
    ei();
    
}

void Window(unsigned char num)
{
    lcd_clear();
    lcd_cursor ( 0, 0 ) ;	
	lcd_print ( msg_ary[num*2]);
    lcd_cursor ( 0, 1 ) ;
    lcd_print ( msg_ary[(num*2)+1]);
}


void delay_s(unsigned char secs)
{
    unsigned char i,j;
    for(j=0;j<secs;j++)
    {
        for (i=0;i<25;i++)
                __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
        
    }
}

