/********************************************************************************
 Written by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 13th January 2010
 
 Modified by: Hussain Manasawala
 
 Date: 9th November 2010
 
 Application example: Robot control over serial port via ZigBee wireless comunication module 
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication
 
 Serial Port used: UART0

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:  	
 						
  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expainsion board
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expainsion board

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
			Keyboard Key   ASCII value	Action
				w				0x38	Forward
				x				0x32	Backward
				a				0x34	Left
				d				0x36	Right
				s				0x35	Stop
				q				0x34	SoftLeft
				e				0x36	SoftRight
				z				0x37	Buzzer on
				c				0x39	Buzzer off

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 11059200
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same. 

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

unsigned char data; //to store received data from UDR1


/*****************************************************/
/**			Function to configure Buzzer			**/
/*****************************************************/
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/*****************************************************/
/**		Function to configure Interrupt switch		**/
/*****************************************************/
void interrupt_switch_config (void)
{
 DDRE = DDRE & 0x7F;  //PORTE 7 pin set as input  
 PORTE = PORTE | 0x80; //PORTE7 internal pullup enabled
}


/*****************************************************/
/**		Function To Initialize UART0 for XBee		**/
/**		desired baud rate:9600						**/
/**		char size: 8 bit							**/
/**		parity: Disabled							**/
/*****************************************************/
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


/*****************************************************/
/**		Function to configure ports 				**/
/**		to enable robot's motion					**/
/*****************************************************/
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*****************************************************/
/**		Function to configure INT4 (PORTE 4) pin 	**/
/**		as input for the left position encoder		**/
/*****************************************************/
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pullup for PORTE 4 pin
}

/*****************************************************/
/**		Function to configure INT5 (PORTE 5) pin	**/
/**		as input for the right position encoder		**/
/*****************************************************/
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pullup for PORTE 4 pin
}

/*****************************************************/
/**		Function to initialize all ports			**/
/*****************************************************/
void port_init()
{
 motion_pin_config(); //robot motion pins config
 buzzer_pin_config(); //robot buzzer pins config
 interrupt_switch_config();
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
}

/*****************************************************/
/**		Function to enable Interrupt 4				**/
/*****************************************************/
void left_position_encoder_interrupt_init (void)
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

/*****************************************************/
/**		Function to enable Interrupt 5				**/
/*****************************************************/
void right_position_encoder_interrupt_init (void)
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

/*****************************************************/
/**		ISR for right position encoder				**/
/*****************************************************/
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


/*****************************************************/
/**		ISR for left position encoder				**/
/*****************************************************/
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


/*****************************************************/
/**		Function used for setting motor's direction	**/
/*****************************************************/
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}


/*****************************************************/
/**		Function used for turning robot by 			**/
/**		specified degrees							**/
/*****************************************************/
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop action
}

/*****************************************************/
/**		Function used for moving robot forward		**/
/**		by specified distance						**/
/*****************************************************/
void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop action
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}


/*****************************************************/
/**		ISR for receive serial interrupt			**/
/*****************************************************/
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR2 in 'data' variable 
	sei();
	
		if(data == 0x77) //ASCII value of w
		{
		forward_mm(50); //Moves robot forward 100mm
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x78) //ASCII value of x
		{
		back_mm(50);   //Moves robot backward 100mm
		stop();			
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x61) //ASCII value of a
		{
		left_degrees(94); //Rotate robot left by 90 degrees
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x64) //ASCII value of d
		{
		right_degrees(98); //Rotate robot right by 90 degrees
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x71) //ASCII value of q
		{
		soft_left_degrees(90); //Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}
		
		if(data == 0x65) //ASCII value of e
		{
		soft_right_degrees(90);	//Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x73) //ASCII value of s
		{
		stop();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x7A) //ASCII value of z
		{
		buzzer_on();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}

		if(data == 0x63) //ASCII value of c
		{
		buzzer_off();
		_delay_ms(100);
		UDR0 = data; 				//echo data back to PC
		}
	
}

/*****************************************************/
/**  	Function To Initialize all The Devices  	**/
/*****************************************************/
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART1 for serial communiaction
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();   //Enables the global interrupts
}


/*****************************************************/
/**  				Main Function				  	**/
/*****************************************************/
int main(void)
{
	interrupt_switch_config();
	while((PINE & 0x80) == 0x80); //switch is not pressed
	init_devices();
	while(1);
	return 0;

}
