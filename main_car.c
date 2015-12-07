/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2015
***********************************************************************
	 	   			 		  			 		  		
 Team ID: < 8 >

 Project Name: < ? >

 Team Members:

   - Team/Doc Leader: < ? >      Signature: ______________________
   
   - Software Leader: < ? >      Signature: ______________________

   - Interface Leader: < ? >     Signature: ______________________

   - Peripheral Leader: < ? >    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to .... < ? >


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1.

 2.

 3.

 4.

 5.

***********************************************************************

  Date code started: < ? >

  Update history (add an entry every time a significant change is made):

  Date: < ? >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >


***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
int close_object();
void move_car(int follow_ob);
unsigned char bci();

/* Variable declarations */
unsigned char X = 0; //X value of Joystick
unsigned char Y = 0; //Y value of Joystick
int stop = 0; //Flag to stop car if it encounters an obstacle  	   			 		  			 		       
int inter = 0; //Set when the interrupt is sent to sample the RF data
int M_right = 0; //Right motor speed
int M_left = 0; //Left motor speed
int auto_mode = 0;//flag to check if autonomous mode is on
                  //0 for rc and 1 for autonomous
int prev_switch = 0; //previous state of autonomous switch
int switch_change = 0; //flag for change of switch state
int follow_ob; // Contains the object that needs to be followed
unsigned char read1;
unsigned char read2;
unsigned char read3;
unsigned char rin	= 0;	// SCI transmit display buffer IN pointer
unsigned char rout	= 0;	// SCI transmit display buffer OUT pointer
int ATD0;
int ATD1;
int ATD2;
#define RSIZE 4	// transmit buffer size (4 characters)
unsigned char rbuf[RSIZE];	// SCI transmit display buffer

//Thresholds for the IR-sensors
#define THRESH0_START 35
#define THRESH1_START 115
#define THRESH2_START 77
#define THRESH_STOP 240
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
#define LINE1 0x80	// LCD line 1 cursor position
#define LINE2 0xC0	// LCD line 2 cursor position



	 	   		
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
  SCIBDL =  0x38; //24,000,000 / 16 / 156 = 9600 (approx)    //0x38
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */
  ATDDIEN = 0x00;
  ATDCTL2 = 0x80;
  ATDCTL3 = 0x18;
  ATDCTL4 = 0x85;           
/* Initialize interrupts */
  TSCR1 = 0x80; //enables timer subsystem
  TIOS = 0x80;   //power on timer channel
  TSCR2 = 0x0C;  //prescaler = 16
  TC7 =  150000; //0.1s interrupt rate
  TIE = 0x00;   //disable interrupts
/*
  Initialize the RTI for an 8.192 ms interrupt rate
*/
  CRGINT = 0x00;  //disable CRG block
  RTICTL = 0x70; //8.192 ms interrupt rate
  
   
/* PWM initializations */
  MODRR = 0x0F;    //PT3,2,1,0 used as PWM Ch 3,2,1,0 output
  PWME = 0x0F;    //enable PWM Ch 0,1,2,3
  PWMPOL = 0x00;  //negative polarity
  PWMCTL	= 0x00;  // no concatenate (8-bit)
  PWMCAE	= 0x00;  // left-aligned output mode
  PWMPER3 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY3 = 0x00;  // initially clear DUTY register
  PWMPER2 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY2 = 0x00;  // initially clear DUTY register
  PWMPER1 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY1 = 0x00;  // initially clear DUTY register
  PWMPER0 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY0 = 0x00;  // initially clear DUTY register
  PWMCLK	= 0x0F;  // select scaled clock 
  PWMSCLB = 15;     //scale B register
  PWMSCLA = 15;     //scale B register
  PWMPRCLK	= 0x55; //A = B = bus clock / 32 	      	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;

 for(;;) {
 
 ATD0 = ATDDR0H;
 ATD1 = ATDDR1H;
 ATD2 = ATDDR2H;
 
 if(PTT_PTT7) {
   //auto_mode is on
   auto_mode = 1;
   TIE = 0x80;   //enable timer interrupts
   
 } else {
   //auto_mode is off 
   auto_mode = 0;
   TIE = 0x00;   //disable interrupts
 }
 
 if(switch_change == 1) {
   switch_change = 0;
   PWMDTY0 = 0x00;
   PWMDTY1 = 0x00;
   PWMDTY2 = 0x00;
   PWMDTY3 = 0x00;
 }
  
 if(stop == 1) 
 {
   PWMDTY0 = 0;
   PWMDTY1 = 0;
   PWMDTY2 = 0;
   PWMDTY3 = 0;
 }

 SCICR2_RIE = 1; //enable receiver interrupts
 if((rin + 1) % RSIZE == rout) {  //if buffer is full
    read1 = bci();
    read2 = bci();
    read3 = bci();
    if(read1 == 'A') {
      X = read2;
      Y = read3;
      inter = 1;
    } else if(read2 == 'A') {
      X = read3;
      Y = read1;
      inter = 1;
    } else if(read3 == 'A') {
      X = read1;
      Y = read2;
      inter = 1;
    } else {
      inter = 0;
    }
 }
     
 if(inter == 1 && auto_mode == 0) //Change motor speed when SCI interrupt is received
 {
   TIE = 0x00;   //disable timer interrupts
   inter = 0;
   if(stop == 0) //if car is too close 
   {
      M_right = (Y - 128); //Speed of right motor from Y of joystick 
      M_left = (Y - 128);  //Speed of left motor from Y of joystick 
      if(Y > 128) //if Y is forward, do turning
      {   
         if(X > 128) //Turn left if X is greater than 128
           M_left -= (X - 128); //Subtract speed if Y is positive to go left
         else  //Turn left if X is right
           M_right -= (128 - X);  //Subtract speed if Y is positive to go right
      } 
      else  //if Y is backward do turning
      {  
         if(X > 128) //Turn left if X is greater than 128
           M_left += (X - 128); //Add speed if Y is negative to go left
         else //Turn left if X is right
           M_right += (128 - X); //Add speed if Y is negative to go right
      }
      if(M_right >= 0)  //If speed is greater than 0 move forward
      {  
        PWMDTY3 = M_right * 2; //Multiply by 2 to get 0 to 255 for any direction
        PWMDTY0 = 0;           //dont go backward
      }
      else //If speed is less than 0 move backward
      {
        PWMDTY3 = 0; //dont go forward
        PWMDTY0 = M_right * 2 * -1; //Multiply by -2 to get 0 to 255 for any direction and make PWM positive
      }
      if(M_left >= 0) //If speed is greater than 0 move forward 
      {  
        PWMDTY2 = M_left * 2;  //Multiply by 2 to get 0 to 255 for any direction
        PWMDTY1 = 0; //dont go backward
      }
      else  //If speed is less than 0 move backward
      {
        PWMDTY2 = 0; //dont go forward
        PWMDTY1 = M_left * 2 * -1; //Multiply by -2 to get 0 to 255 for any direction and make PWM positive
      } 
   }
   
 }
  
   } /* loop forever */
   
}   /* do not leave main */

/*
***********************************************************************                       
 Find out closest object and return integer corresponding to direction
************************************************************************
*/
int close_object() 
{
   ATDCTL5 = 0x10;
   while(ATDSTAT0 != 0x80){}
   if(ATDDR0H >= THRESH_STOP || ATDDR1H >= THRESH_STOP || ATDDR2H >= THRESH_STOP) //if object is too close stop
   {
      stop = 1;
      return 4;
   }   
   else 
  {
      stop = 0;
      if(ATDDR0H > ATDDR1H && ATDDR0H > ATDDR2H) // if forward is greatest go there
         return 0;
      else if(ATDDR1H > ATDDR0H && ATDDR1H > ATDDR2H) //if right is greatest go there
         return 1;
      else if(ATDDR2H > ATDDR0H && ATDDR2H > ATDDR1H)  //if left is greatest go there
         return 2;
      else if(ATDDR0H == ATDDR1H || ATDDR0H == ATDDR2H) //if forward is equal to any direction go forward
         return 0;
      else if(ATDDR1H == ATDDR2H) //if right and left are equal go right
         return 1;
      else //if they are all equal go forward
         return 0;
   }
   
}

/*
***********************************************************************                       
 Move cars in correct direction
************************************************************************
*/
void move_car(int follow_ob) 
{
    if(follow_ob == 0) //Go forward
    {
      PWMDTY0 = 0;
      PWMDTY1 = 0;
      PWMDTY3 = 255;
      PWMDTY2 = 255;
    } 
    else if(follow_ob == 1) 
    {
      follow_ob = close_object();
      PWMDTY0 = 128;
      PWMDTY2 = 255;
      PWMDTY1 = 0;
      PWMDTY3 = 0;
    }
    else if(follow_ob == 2) //turn left until closest object is in front
    {
      follow_ob = close_object();
      PWMDTY1 = 128;
      PWMDTY3 = 255;
      PWMDTY2 = 0;
      PWMDTY0 = 0;      
    }
}


/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80; 
    if(PTT_PTT7 == 0 && prev_switch == 1)// check switch value
    {
      switch_change = 1;
    }
    prev_switch = PORTAD0_PTAD7;
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
 	follow_ob = close_object(); //find which object to follow
  if(follow_ob != 4) // dont move car if objects are too close
    move_car(follow_ob);  //move car in direction 
}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
  if(SCISR1_RDRF == 1)
  {      
    if((rin+1) % RSIZE != rout) {     //if not full
      rbuf[rin] = SCIDRL; 
      rin = (rin+1) % RSIZE;
    } else {
      SCICR2_RIE = 0;  //disable interrupts  
    }
  }
}

//BCI
unsigned char bci() {
  unsigned char x;
  while(rin==rout) {} //while empty, wait
  x = rbuf[rout];
  rout = (rout+1) % RSIZE;
  return x;
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
