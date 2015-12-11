/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2015
***********************************************************************
	 	   			 		  			 		  		
 Team ID: 8

 Project Name: DroneControl

 Team Members:

   - Team/Doc Leader: Leo Welter           Signature: ______________________
   
   - Software Leader: Namrata Madan        Signature: ______________________

   - Interface Leader: Apoorv Wariagade    Signature: ______________________

   - Peripheral Leader: Shubham Rastogi    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to create a simultaneously autonomous
 and radio-frequency controlled land drone by utilizing various peripherals of
 the 9S12 family Freescale microcontrollers (specifically, the PWM, SCI, ATD, and 
 TIM) 

***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. Remote-controlled wireless communication

 2. Autonomous obstacle detection

 3. Synchronized motor and controls 

 4. Implementation of hardware sensors

***********************************************************************

  Date code started: 11/20/2015

  Update history (add an entry every time a significant change is made):

  Date: 11/30  Name: Shubham   Update: SCI routine
***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void bco(unsigned char x);	// SCI buffered character output

/* Variable declarations */
unsigned char tin	= 0;	// SCI transmit display buffer IN pointer
unsigned char tout	= 0;	// SCI transmit display buffer OUT pointer
int send = 0; //Send the value of joystick
#define TSIZE 5	// transmit buffer size (4 characters)
unsigned char tbuf[TSIZE];	// SCI transmit display buffer   	   			 		  			 		       

/* Special ASCII characters */
#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10		// RS pin mask (PTT[4])
#define RW 0x20		// R/W pin mask (PTT[5])
#define LCDCLK 0x40	// LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 = 0x80	// LCD line 1 cursor position
#define LINE2 = 0xC0	// LCD line 2 cursor position

	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x01; //set baud rate to 9600
  SCIBDL =  0x38; //24,000,000 / 16 / 312 = 4800 (approx)  
  SCICR1 =  0x00; //$138 = 312
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */

  //ATD conversion
  ATDDIEN = 0x00;
  ATDCTL2 = 0x80;
  ATDCTL3 = 0x18;
  ATDCTL4 = 0x85;
            
/* Initialize interrupts */

  //Tim 7 interupt for 1 ms
  TSCR1 = 0x80;
  TIOS = 0x80;
  TSCR2 = 0x0C;
  TC7 = 1500;
  TIE_C7I = 1;	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  int i = 0;	
  DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;

 for(;;) {
  
/* < start of your main loop > */ 
    if(send)  //Send data if send flag is set
    {
      send = 0;  //Set send flag to zero
      ATDCTL5 = 0x10;  //Start ATD conversion
      while(ATDSTAT0 == 0x00){}
      bco('A');
      bco(ATDDR0H); //X
      bco(ATDDR1H); //Y
      if(ATDDR0H>ATDDR1H) 
        bco(ATDDR0H - ATDDR1H);    //Send difference to make sure there is no noise
      else
        bco(ATDDR1H - ATDDR0H);
    }
  

  
   } /* loop forever */
   
}   /* do not leave main */




/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flag
  	CRGFLG = CRGFLG | 0x80;
   
    
}

/*
***********************************************************************                       
  TIM interrupt service routine	  		
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  	// clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80; 
  send = 1;

}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
  int i = 0;
  while(SCISR1_TDRE != 1)  {}      
  if(tin == tout)
    SCICR2_SCTIE = 0;
  else
  {
    SCIDRL = tbuf[tout];
    tout = (tout + 1) % TSIZE; 
  }

}

/*
***********************************************************************                              
  SCI buffered character output routine - bco

  Places character x passed to it into TBUF

   - check TBUF status: if FULL, wait for space; else, continue
   - place character in TBUF[TIN]
   - increment TIN mod TSIZE
   - enable SCI transmit interrupts

  NOTE: DO NOT USE OUTCHAR (except for debugging)
***********************************************************************
*/

void bco(unsigned char x)
{
  while((tin + 1) % TSIZE == tout ) {}
    
  tbuf[tin] = x;
  tin = (tin + 1) % TSIZE;
  SCICR2_SCTIE = 1;   
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}
