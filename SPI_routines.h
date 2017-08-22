//**************************************************************
// ****** FUNCTIONS FOR SPI COMMUNICATION *******
//**************************************************************
//Controller		: ATmega32 (Clock: 8 Mhz-internal)
//Compiler			: AVR-GCC (winAVR with AVRStudio-4)
//Project Version	: DL_1.0
//Author			: CC Dharmani, Chennai (India)
//			  		  www.dharmanitech.com
//Date				: 10 May 2011
//**************************************************************

#ifndef _SPI_ROUTINES_H_
#define _SPI_ROUTINES_H_

#define SPI_SD             SPCR = 0x52	// Enable SPI and set controller as a MAster
#define SPI_HIGH_SPEED     SPCR = 0x50; SPSR &= ~(1<<SPI2X)	//approx 4.6MHz


void spi_init(void);
unsigned char SPI_transmit(unsigned char);
unsigned char SPI_receive(void);

#endif
