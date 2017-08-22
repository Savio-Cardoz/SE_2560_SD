// This code can be used for bombay parsi panchyat  as well as for incomplete reject votes.(see configuration settings to use any one of the code) 
//here voter count is saved in ext eeprom.and in case of power failure it will restore all the previous votes and voter count....16/12/2015


/* WARNING! Follow the following fuse settings only
	CLKDIV8 = 1 (Not programmed) No internal division of clock.
	CKOUT	= 1
	WDTON	= 1	No watch-dog timer
	BOOTRST	= 1
	CKSEL3	= 0	(Programmed) Full Swing oscillator high frequency 
	CKSEL2	= 1
	CKSEL1	= 1 
	CKSEL0	= 1
	SUT1	= 1	Fast rising power
	SUT0	= 0
*/
/* Set these in ProgISP fuse settings prior the programming the controller */ 
#define F_CPU 18432000UL
//#define F_CPU 11059200UL

/*Very Important - change F_CPU to match crystal value	**NOTE** There have been instances with KDS crystals where the actual frequency was not what was stated on the package 
  Note: default AVR CLKSEL is 1MHz internal RC 
  Change USART_BAUDRATE constant to change Baud Rate
*/

#include <avr/io.h> 
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
//#include "SPI_routines.h"
//#include "SD_routines.h"
//#include "FAT32.h"

typedef struct			// Using to access individual bits/pins of a register/port
{
	unsigned int bit0:1;
	unsigned int bit1:1;
	unsigned int bit2:1;
	unsigned int bit3:1;
	unsigned int bit4:1;
	unsigned int bit5:1;
	unsigned int bit6:1;
	unsigned int bit7:1;
} _io_reg;

#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt
#define DEV_ID "A115"			// A1-version 02-number in the version
#define EEPROM_DEV_ADDR 0xA0	// Address of On-board EEPROM
#define PIC_DEV_ADDR 0x80		// I2C address of PIC LED indicator controller set by code in the 

#define USART_BAUDRATE 9600	//115200
#define USART_BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)   // If not using double speed mode

// Macros for bit operations
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))			// AVRStudio method of changing the value of a bit in a byte. (to SET)
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))		// (to CLEAR)
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))			// This returns the value of a bit in a byte. use - if(CHECKBIT(variable, 7) == 1)
#define BIT_FLIP(ADDRESS,BIT) ((ADDRESS)^=(BIT))			// Changes value of any bit from 1->0 or 0->1
#define BIT(x) (0x01 << (x))								// Shifts 1 by 'x' bits to the left

#define RS REGISTER_BIT(PORTD, 7)	 		// connected to RS pin of LCD
#define EN REGISTER_BIT(PORTD, 6)			// Connected to EN pin of LCD

#define INIT REGISTER_BIT(PINL, 1)			// Initialize button
#define RESULT REGISTER_BIT(PINL, 2)		// Result button

#define matrix_A_data REGISTER_BIT(PINC, 4)	// Used to detect button presses on 1st voting pad ( data available of IC 74c922 goes high)
#define matrix_B_data REGISTER_BIT(PINC, 5)	// Used to detect button presses on 2nd voting pad ( data available of IC 74c922 goes high)
#define matrix_C_data REGISTER_BIT(PINC, 6)	// Used to detect button presses on 3rd voting pad ( data available of IC 74c922 goes high)
#define matrix_D_data REGISTER_BIT(PINC, 7)	// Used to detect button presses on 4th voting pad ( data available of IC 74c922 goes high)


#define LED REGISTER_BIT(PORTG, 0)			// To control LED on control unit
#define Buzzer REGISTER_BIT(PORTG, 1)		// To control Buzzer on Control unit and voting pad

#define LED_INDICATOR REGISTER_BIT(PORTG,5) // To indicate voting is in progress or not

#define LCD PORTA

volatile unsigned char flag_register;
#define slave_ack REGISTER_BIT(flag_register, 7)			// Flag indicating whether ack from slave I2C device was received or not.
#define master_ack REGISTER_BIT(flag_register, 6)			// Flag to determine whether to send ACK on I2C bus or not
#define voting_on REGISTER_BIT(flag_register, 5)			// Flag set when a voter is casting vote. Cleared on completion of casting vote.

uint16_t voted_button_array[64];	// This array temporarily hold the address of the buttons that have been voted for.
uint8_t elem_count;					// Counter to store address of voted buttons
volatile unsigned char myregister1;		// Vote_indicator_1 : 8		/// Used to store bits identifying button press for a candidate.
volatile unsigned char myregister2;		// Vote_indicator_9 : 16
volatile unsigned char myregister3;		// Vote_indicator_17 : 24
volatile unsigned char myregister4;		// Vote_indicator_25 : 32
volatile unsigned char myregister5;		// Vote_indicator_33 : 40
volatile unsigned char myregister6;		// Vote_indicator_41 : 48
volatile unsigned char myregister7;		// Vote_indicator_49 : 56
volatile unsigned char myregister8;		// Vote_indicator_57 : 64

volatile unsigned char temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8,temp9,temp10,temp11,temp12;	// Global multipurpose variables needed.

/*	Add note: Added variables to hold machine location ID and type of machine */
unsigned char location_ID, location_ID_number, mach_typ, cod_typ;		//17th Sept 2015 Savio added for Parsi Panchayat

#define vote_indicator_1 REGISTER_BIT(myregister1, 0) // to indicate the button was pressed and prevent re voting for the same button
#define vote_indicator_2 REGISTER_BIT(myregister1, 1) // vote_indicator_1 is bit 0 of myregister1, vote_indicator_2 is bit 1 of myregister1 and so on
#define vote_indicator_3 REGISTER_BIT(myregister1, 2)
#define vote_indicator_4 REGISTER_BIT(myregister1, 3)
#define vote_indicator_5 REGISTER_BIT(myregister1, 4)
#define vote_indicator_6 REGISTER_BIT(myregister1, 5)
#define vote_indicator_7 REGISTER_BIT(myregister1, 6)
#define vote_indicator_8 REGISTER_BIT(myregister1, 7)
#define vote_indicator_9 REGISTER_BIT(myregister2, 0)
#define vote_indicator_10 REGISTER_BIT(myregister2, 1)
#define vote_indicator_11 REGISTER_BIT(myregister2, 2)
#define vote_indicator_12 REGISTER_BIT(myregister2, 3)
#define vote_indicator_13 REGISTER_BIT(myregister2, 4)
#define vote_indicator_14 REGISTER_BIT(myregister2, 5)
#define vote_indicator_15 REGISTER_BIT(myregister2, 6)
#define vote_indicator_16 REGISTER_BIT(myregister2, 7)
#define vote_indicator_17 REGISTER_BIT(myregister3, 0)
#define vote_indicator_18 REGISTER_BIT(myregister3, 1)
#define vote_indicator_19 REGISTER_BIT(myregister3, 2)
#define vote_indicator_20 REGISTER_BIT(myregister3, 3)
#define vote_indicator_21 REGISTER_BIT(myregister3, 4)
#define vote_indicator_22 REGISTER_BIT(myregister3, 5)
#define vote_indicator_23 REGISTER_BIT(myregister3, 6)
#define vote_indicator_24 REGISTER_BIT(myregister3, 7)
#define vote_indicator_25 REGISTER_BIT(myregister4, 0)
#define vote_indicator_26 REGISTER_BIT(myregister4, 1)
#define vote_indicator_27 REGISTER_BIT(myregister4, 2)
#define vote_indicator_28 REGISTER_BIT(myregister4, 3)
#define vote_indicator_29 REGISTER_BIT(myregister4, 4)
#define vote_indicator_30 REGISTER_BIT(myregister4, 5)
#define vote_indicator_31 REGISTER_BIT(myregister4, 6)
#define vote_indicator_32 REGISTER_BIT(myregister4, 7)
#define vote_indicator_33 REGISTER_BIT(myregister5, 0)
#define vote_indicator_34 REGISTER_BIT(myregister5, 1)
#define vote_indicator_35 REGISTER_BIT(myregister5, 2)
#define vote_indicator_36 REGISTER_BIT(myregister5, 3)
#define vote_indicator_37 REGISTER_BIT(myregister5, 4)
#define vote_indicator_38 REGISTER_BIT(myregister5, 5)
#define vote_indicator_39 REGISTER_BIT(myregister5, 6)
#define vote_indicator_40 REGISTER_BIT(myregister5, 7)
#define vote_indicator_41 REGISTER_BIT(myregister6, 0)
#define vote_indicator_42 REGISTER_BIT(myregister6, 1)
#define vote_indicator_43 REGISTER_BIT(myregister6, 2)
#define vote_indicator_44 REGISTER_BIT(myregister6, 3)
#define vote_indicator_45 REGISTER_BIT(myregister6, 4)
#define vote_indicator_46 REGISTER_BIT(myregister6, 5)
#define vote_indicator_47 REGISTER_BIT(myregister6, 6)
#define vote_indicator_48 REGISTER_BIT(myregister6, 7)
#define vote_indicator_49 REGISTER_BIT(myregister7, 0)
#define vote_indicator_50 REGISTER_BIT(myregister7, 1)
#define vote_indicator_51 REGISTER_BIT(myregister7, 2)
#define vote_indicator_52 REGISTER_BIT(myregister7, 3)
#define vote_indicator_53 REGISTER_BIT(myregister7, 4)
#define vote_indicator_54 REGISTER_BIT(myregister7, 5)
#define vote_indicator_55 REGISTER_BIT(myregister7, 6)
#define vote_indicator_56 REGISTER_BIT(myregister7, 7)
#define vote_indicator_57 REGISTER_BIT(myregister8, 0)
#define vote_indicator_58 REGISTER_BIT(myregister8, 1)
#define vote_indicator_59 REGISTER_BIT(myregister8, 2)
#define vote_indicator_60 REGISTER_BIT(myregister8, 3)
#define vote_indicator_61 REGISTER_BIT(myregister8, 4)
#define vote_indicator_62 REGISTER_BIT(myregister8, 5)
#define vote_indicator_63 REGISTER_BIT(myregister8, 6)
#define vote_indicator_64 REGISTER_BIT(myregister8, 7)



/********************************************************************************** 7/1015 ***********/
/* 126 bytes are used to store the vote count for 64 candidates. Thats almost 1 page of the EEPROM.
/* EERPOM address 127 will be ignored and skipped for any purpose
/* All setting storage addresses will start from EEPROM address 128
/*****************************************************************************************************/
#define button1_addr 0x0000		// External EEPROM address where the vote_count for a particular button is stored
#define button2_addr 0x0002
#define button3_addr 0x0004
#define button4_addr 0x0006
#define button5_addr 0x0008
#define button6_addr 0x000A
#define button7_addr 0x000C
#define button8_addr 0x000E
#define button9_addr 0x0010
#define button10_addr 0x0012
#define button11_addr 0x0014
#define button12_addr 0x0016
#define button13_addr 0x0018
#define button14_addr 0x001A
#define button15_addr 0x001C
#define button16_addr 0x001E
#define button17_addr 0x0020
#define button18_addr 0x0022
#define button19_addr 0x0024
#define button20_addr 0x0026
#define button21_addr 0x0028
#define button22_addr 0x002A
#define button23_addr 0x002C
#define button24_addr 0x002E
#define button25_addr 0x0030
#define button26_addr 0x0032
#define button27_addr 0x0034
#define button28_addr 0x0036
#define button29_addr 0x0038
#define button30_addr 0x003A
#define button31_addr 0x003C
#define button32_addr 0x003E
#define button33_addr 0x0040
#define button34_addr 0x0042
#define button35_addr 0x0044
#define button36_addr 0x0046
#define button37_addr 0x0048
#define button38_addr 0x004A
#define button39_addr 0x004C
#define button40_addr 0x004E
#define button41_addr 0x0050
#define button42_addr 0x0052
#define button43_addr 0x0054
#define button44_addr 0x0056
#define button45_addr 0x0058
#define button46_addr 0x005A
#define button47_addr 0x005C
#define button48_addr 0x005E
#define button49_addr 0x0060
#define button50_addr 0x0062
#define button51_addr 0x0064
#define button52_addr 0x0066
#define button53_addr 0x0068
#define button54_addr 0x006A
#define button55_addr 0x006C
#define button56_addr 0x006E
#define button57_addr 0x0070
#define button58_addr 0x0072
#define button59_addr 0x0074
#define button60_addr 0x0076
#define button61_addr 0x0078
#define button62_addr 0x007A
#define button63_addr 0x007C
#define button64_addr 0x007E
/*****************************************************************************************************/

/******************************* Settings storage address definitions ********************************/

#define categories1_2 128			// Each category is (right now) max 15 buttons size. Space required is 1byte.
#define categories3_4 129			// Location where no. of buttons in category 3 and 4 are to be stored in EEPROM
#define categories5_6 130			// Location where no. of buttons in category 5 and 6 are to be stored in EEPROM
#define categories7_8 131			// Do note only upto categories 7 + gen. cat (total 8) are used the additional definitions are from EVM64-AT2561 and EVM64-AT2560 codes.
#define categories9_10 132
#define categories11_12 133
#define categories13_14 134
#define categories15_16 135

#define catg1_votes 136		// Each category can have a maximum of 15 votes. Each no. is stored in 1 byte.
#define catg2_votes 137		// Location at which no. of votes to be cast in category 2 are stored in EEPROM
#define catg3_votes 138		// Location at which no. of votes to be cast in category 3 are stored in EEPROM
#define catg4_votes 139		// Location at which no. of votes to be cast in category 4 are stored in EEPROM
#define catg5_votes 140			// Location at which no. of votes to be cast in category 5 are stored in EEPROM
#define catg6_votes 141			// Location at which no. of votes to be cast in category 6 are stored in EEPROM
#define catg7_votes 142			// Do note only upto categories 7 + gen. cat (total 8) are used the additional definitions are from EVM64-AT2561 and EVM64-AT2560 codes.
#define catg8_votes 143
//#define catg9_votes 144
//#define catg10_votes 145
//#define catg11_votes 146
//#define catg12_votes 147
//#define catg13_votes 148
//#define catg14_votes 149
//#define catg15_votes 150
//#define catg16_votes 151

#define voters_voted 152			// location of the no. of voters who've cast a vote

#define LOCATION_ID_NAME 153
#define LOCATION_ID_NO 154
#define MACHINE_TYPE 155	
#define CODE_TYPE 156
                                                                    
/************************************************************************			7/1015			*/
/* Settings constantly updated to external EEPROM for backup purpose incase of loss of Power */
#define vote_indic_1_8 157		// These locations will hold the backup of the myregister variables i.e. vote_indicator bits from 1 -64.
#define vote_indic_9_16 158
#define vote_indic_17_24 159
#define vote_indic_25_32 160
#define vote_indic_33_40 161
#define vote_indic_41_48 162
#define vote_indic_49_56 163
#define vote_indic_57_64 164

#define no_votes_1_bkp 165
#define no_votes_2_bkp 166
#define no_votes_3_bkp 167
#define no_votes_4_bkp 168
#define no_votes_5_bkp 169
#define no_votes_6_bkp 170
#define no_votes_7_bkp 171
//#define no_votes_8_bkp 172

#define votes_count_bkp 173
#define gen_vote_count_bkp 174

#define flag_register_bkp 175

#define elem_count_bkp 176

#define voters_not_voted 177

#define namePtrSaveAddr 256			// 128 bytes will be used to save pointers
#define STRING_STORAGE_ADDR 384		//  Added 22.04.2017 Printer integration. Names will be stored from this address onwards
/*****************************************************************************************************/

int vote_amt[3];
volatile unsigned char rx0_data, candidate_count, voter_votes, i2c_data, votes_counter, gen_vote_count, no_of_categories, vote_cast;
volatile uint16_t no_of_buttons_in_categories, general_category, voter_count, i2c_data_16;
volatile unsigned int invalid_voters ;


/******** Added 23.04.2017	**********/
#define uartReceiveTerminator REGISTER_BIT(flag_register, 4)// Flag to indicate reception of packet data over UART. A terminator indicator
															// to tell controller data packet is available in the 'uartReceiveBuffer'.         Addedd as mofication on 9/4/2017 - integrating thermal printer to EVM	
#define MAX_NAME_LEN 40
#define MAX_RECV_BUFFERSIZE 50

#define MAX_NO_CANDIDATES 64

static uint16_t stringPtrArray[MAX_NO_CANDIDATES];		// An array of pointers to 64 EEPROM locations of names saved.
static uint16_t stringSaveAddr = STRING_STORAGE_ADDR;
int8_t uartRecieveBuffer[MAX_RECV_BUFFERSIZE];
uint8_t readString[7];

void serviceCommand(uint8_t *recvBuffer);
void i2cWriteString(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen);
uint16_t i2cSaveName(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen);
void i2cReadString(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen);
uint8_t i2c_writeReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length);
uint8_t i2c_readReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length);
uint8_t i2c_start_type2(uint8_t address);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
void printName(uint8_t candidateNo, uint16_t Voter_no);

/******** Added 23.04.2017	**********/

void send_results_pc();
void i2c_send_byte(unsigned char addr, unsigned char data);

ISR(USART0_RX_vect)		// USART interrupt service routine.
{
	
	static char bufferPointer;
	rx0_data = UDR0;
	if(rx0_data == '~')
		{
			uartReceiveTerminator = 1;		// Clear flag after sericing the uartBuffer. check definition above.
			bufferPointer = 0;
		}
	else if(rx0_data == 'R')
			send_results_pc();
	else uartRecieveBuffer[bufferPointer++] = rx0_data;	// Store value in UDR to the uartBuffer and increment buffer index pointer

	if(bufferPointer > MAX_RECV_BUFFERSIZE)
		bufferPointer = 0;
}

/********************** Added 23.04.2017 **********************************************/
void printName(uint8_t candidateNo, uint16_t Voter_no)
{
	uint8_t tempStrLen;
	i2c_readReg(EEPROM_DEV_ADDR, (stringPtrArray[candidateNo]), readString, 1);	// first read the lenght byte of the name string in EEPROM
	tempStrLen = readString[0];
	_delay_ms(100);
	i2c_readReg(EEPROM_DEV_ADDR, (stringPtrArray[candidateNo] + 1), readString, tempStrLen);
	
	USART_putstring("************************");
	USART_SendByte(0x0A);
	
	USART_putstring("Voter No.: ");
	USART_Transmit_dec(Voter_no);
	USART_SendByte(0x0A);
	
	USART_putstring("Candidate Selected");
	USART_SendByte(0x0A);
	for(uint8_t i = 0; i < tempStrLen; i++)
	{
		USART_SendByte(readString[i]);
	}
	USART_SendByte(0x0a);
	
	USART_putstring("************************");
	USART_SendByte(0x0A);
	USART_SendByte(0x0A);
	USART_SendByte(0x0A);
	USART_SendByte(0x0A);
}

void serviceCommand(uint8_t *recvBuffer)
{
	uint8_t tempStrArr[40], candidateNo;
	uint16_t tempStrLen, tempAddrHolder;

	if(*recvBuffer == '@')		// Command to save string
	{
		candidateNo = *(++recvBuffer);
		tempStrLen = (*(++recvBuffer) + 1);
		stringPtrArray[candidateNo] = stringSaveAddr;

		i2c_writeReg(EEPROM_DEV_ADDR, stringSaveAddr, recvBuffer, tempStrLen);	// Save the characters of the string
		
		stringSaveAddr += (tempStrLen + 1);
		_delay_ms(500);
		i2cWriteWordArray(EEPROM_DEV_ADDR, namePtrSaveAddr, stringPtrArray, MAX_NO_CANDIDATES);
		
		USART_putstring("OK");		// Acknowledge / Feedback
	}

	if(*recvBuffer == '?')		// Command to read string
	{
		candidateNo = *(++recvBuffer);
		i2c_readReg(EEPROM_DEV_ADDR, (stringPtrArray[candidateNo]), readString, 1);	// first read the lenght byte of the name string in EEPROM
		 _delay_ms(5);

		tempStrLen = readString[0];

		i2c_readReg(EEPROM_DEV_ADDR, (stringPtrArray[candidateNo] + 1), readString, tempStrLen);
		for(uint8_t i = 0; i < tempStrLen; i++)
			{
				USART_SendByte(readString[i]);
			}

		// i2c_readReg(EEPROM_DEV_ADDR, 0x0080, tempStrArr, 40);
		// for(uint8_t i=0; i <= 40; i++)
		// 	USART_SendByte(tempStrArr[i]);
	}
}


uint8_t i2c_writeReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start_type2(devaddr | 0x00)) return 1;

	i2c_write((uint8_t)(regaddr / 256));
	i2c_write((uint8_t)(regaddr %  256));

	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}

	i2c_stop();

	return 0;
}

uint8_t i2c_readReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start_type2(devaddr)) return 1;

	i2c_write((uint8_t)(regaddr / 256));
	i2c_write((uint8_t)(regaddr %  256));

	if (i2c_start_type2(devaddr | 0x01)) return 1;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();

	i2c_stop();

	return 0;
}

uint8_t i2c_start_type2(uint8_t address)
{
	// reset TWI control register
	TWCR = 0;
	// transmit START condition 
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }
	
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	
	return 0;
}	

uint8_t i2c_write(uint8_t data)
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); 
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

/********************** Added 23.04.2017 **********************************************/

/* Please note the Controller pin conections to the other devices can vary between AT32 AT2561 and AT2560 */
void PORT_pins_init()
{
	DDRA = 0xFF;				// Output Port to 16x2 LCD Set the DDRA register nits all pins are outputs.
	SETBIT(DDRD, 3);			// NA
	SETBIT(DDRD, 4);			// NA
	CLEARBIT(PORTD, 3);			// NA
	SETBIT(DDRD, 7);			// Output RS of LCD. 
	SETBIT(DDRD, 6);			// Output EN of LCD
	SETBIT(DDRG, 0);			// Pin connected to LED set as ouptut pin
	SETBIT(DDRG, 1);			// Buzzer	
	CLEARBIT(DDRL, 1);			// Input pin connected to Initialise button
	CLEARBIT(DDRL, 2);			// Input pin connected to Result button
	
	CLEARBIT(PORTG, 1);			// Buzzer OFF
	SETBIT(PORTG, 0);			// LED OFF
	SETBIT(PORTL, 1);			// pull-up INIT line
	SETBIT(PORTL, 2);			// Pull-up RESULT line
	
		SETBIT(DDRG,5);				//LED_INDICATOR
		SETBIT(PORTG,5);
		
	
	DDRC = 0x00;				// Inputs from Encoder IC's
	PORTC = 0xFF;				// Activste pull-ups for DATA in not for 'data availables'
	
	
	DDRB = 0xEF;			// MISO pin set as input, rest are output
	
	SETBIT(DDRD, 6);		// LCD back light control
	CLEARBIT(PORTD, 6);		// Switch ON LCD back light
	
	cli();
	//spi_init();
	
}

void USART_Init(void)
{
	UBRR0L = USART_BAUD_PRESCALE;			// Load lower 8-bits into the low byte of the UBRR register
	UBRR0H = (USART_BAUD_PRESCALE >> 8); 
	/*   Load upper 8-bits into the high byte of the UBRR register
	     Default frame format is 8 data bits, no parity, 1 stop bit
		 to change use UCSRC*/
	/*   Enable receiver and transmitter and receive complete interrupt */
	UCSR0B = ((1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0));
}

void USART_SendByte(uint8_t u8Data)
{
	while((UCSR0A & (1 << UDRE0)) == 0);		// Wait until last byte has been transmitted
	UDR0 = u8Data;
}

void USART_putstring(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		USART_SendByte(*StringPtr);
		StringPtr++;
	}
}

uint8_t USART_ReceiveByte()
{
  while((UCSR0A &(1<<RXC0)) == 0);		// wait for transmit complete flag.
  return UDR0;
}


void USART_Transmit_dec(unsigned int int_data)
{
	USART_SendByte(((int_data / 10000) | 0x30));								// Teh thousand
	USART_SendByte(((int_data % 10000) / (1000)) | (0x30)); 					// Tkousand
	USART_SendByte((((int_data % 10000) % (1000)) / (100)) | (0x30)); 			// Hundred
	USART_SendByte(((((int_data % 10000) % (1000)) % (100)) /( 10)) | (0x30)); // Tens
	USART_SendByte(((((int_data % 10000) % (1000)) % (100)) % (10)) | (0x30)); // Units
}
/******************************************************************************************************/
/*	Routine for LCD																					  */
/******************************************************************************************************/
void lcd_cmd(unsigned char cmd_addr) // Function to send command to LCD
{
	LCD = cmd_addr;		// Puts the command value on the Controller port connected to LCD data lines
	RS = 0;				// RS = 0  Indicates to LCD data is a command
	EN = 1;				// ENABLE = 1 Enables LCD controller to write data to its RAM
	_delay_ms(2);
	EN = 0;
}

void lcd_data_str(char str[50])      // Function to send string
{
	int p;
	for (p=0;str[p]!='\0';p++)
	{
		LCD = str[p];
		RS = 1;				  // RS=1,FOR DATA
		EN = 1;				  // ENABLE =1
		_delay_ms(2);
		EN = 0;				  // ENABLE =0
	}
}

void lcd_data_byte(unsigned char byte) // Function to send one byte
{
	LCD = byte;
	RS = 1;							// RS = 1;
	EN = 1;							// EN = 1;
	_delay_ms(2);
	EN = 0;
}

void lcd_data_int1(unsigned int vote)// Function to send 0-9 character values
{
	char dig_ctrl_var;
	int p, j;
	for (j=1;j>=0;j--)
	{
		vote_amt[j]=vote%10;
		vote=vote/10;
	}
	for (p=0;p<=1;p++)
	{
		dig_ctrl_var = vote_amt[p] + 48;    // Here the decimal data is converted to HEX because LCD only reads HEX.
		LCD = dig_ctrl_var;					// 48 is actually 0x30. So if wanting to display 1 on the LCD what needs to be sent is
		RS = 1;								// 0x31. That is what is done here 1 + 0x30 = 0x31
		EN = 1;
		_delay_ms(2);
		EN = 0;
	}
}

void lcd_data_int(unsigned int vote)// Function to send 0-9 character values
{
	char dig_ctrl_var;
	int p, j;
	for (j=3;j>=0;j--)
	{
		vote_amt[j]=vote%10;
		vote=vote/10;
	}
	for (p=0;p<=3;p++)
	{
		dig_ctrl_var = vote_amt[p] + 48;    // Here the decimal data is converted to HEX because LCD only reads HEX.
		LCD = dig_ctrl_var;					// 48 is actually 0x30. So if wanting to display 1 on the LCD what needs to be sent is
		RS = 1;								// 0x31. That is what is done here 1 + 0x30 = 0x31			    
		EN = 1;				    
		_delay_ms(2);
		EN = 0;					
	}
}

/******************************************************************************************************/
/*						Routine for I2C																  */
/******************************************************************************************************/
void i2c_init()
{
	TWBR = 5;				//Baud rate
	TWCR = (1 << TWEN);			// Enable I2c module
}

void i2c_start()						// Start I2C communication
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	CLEARBIT(TWCR, 5);					// Clear TWSTA I2C start bit is to be cleared
	while(!(TWCR & (1 << TWINT)));		// Wait for TWINT flag to be set by hardware	
}		

void i2c_send(uint8_t i2c_data)	// Send one byte on the I2C line
{
	TWDR = i2c_data;					// Load the I2C register TWDR
	TWCR = (1 << TWINT) | (1 << TWEN);	// Clear TWINT flag by writing 1 to it, thus initiating transmission of data
	while(!(TWCR & (1<<TWINT)));		// Wait for TWINT flag to be set by hardware, end of transmission
}

void i2c_receive(uint8_t ack_value)		// Receive a byte from  a slave device
{
	if(ack_value == 1)					// To send ACK
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);		// enable acknowledge transmission
	if(ack_value == 0)					// To send a NACK
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));							// Wait for all 8bits to enter TWDR and TWINT to be set
	i2c_data = TWDR;	
}

void i2c_stop()							// Stop I2C communication
{
	TWCR |= (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);		// Send the stop condition.
}

char i2c_read_byte(uint8_t device_id, uint16_t addr)
{
	i2c_start();
	i2c_send(device_id);
	i2c_send(addr / 256);
	i2c_send(addr % 256);
	i2c_stop();
	_delay_ms(1);
	i2c_start();
	i2c_send(device_id + 1);
	i2c_receive(0);
	i2c_stop();
	
	return i2c_data;
	_delay_ms(10);
}

void i2c_read_2_bytes(uint16_t eeprom_addr)
{
	i2c_start();
	i2c_send(0xA0);
	i2c_send(eeprom_addr / 256);	// Sending the 16 bit address in two 8 bits
	i2c_send(eeprom_addr % 256);
	i2c_stop();
	_delay_ms(1);
	i2c_start();
	i2c_send(0xA1);
	i2c_receive(1);
	i2c_data_16 = (i2c_data * 256);		// Shifting the 8bits by 8 posistions to form a 16bit value below
	i2c_receive(0);
	i2c_data_16 = i2c_data_16 + i2c_data;
	i2c_stop();
}

void i2c_write_2_bytes(unsigned char device_id, uint16_t addr, uint16_t val_data)
{	
	do
	{
		i2c_start();
		i2c_send(device_id);
		if(slave_ack == 1)				// Stop I2C if no acknowledge
		i2c_stop();
	}
	while(slave_ack == 1);				//loop until acknowledge is received
	i2c_send(addr / 256);
	i2c_send(addr % 256);
	i2c_send(val_data / 256);
	i2c_send(val_data % 256);
	i2c_stop();
	_delay_ms(10);
}

/************* Added 26.04.2017 ****************************/
void i2cWriteWordArray(unsigned char device_id, uint16_t addr, uint16_t *val_data, uint8_t lenght)
{	
	do
	{
		i2c_start();
		i2c_send(device_id);
		if(slave_ack == 1)				// Stop I2C if no acknowledge
		i2c_stop();
	}
	while(slave_ack == 1);				//loop until acknowledge is received
	i2c_send(addr / 256);
	i2c_send(addr % 256);
	for(uint8_t z = 0; z <= (lenght - 1); z++)
	{
		i2c_send(val_data[z] / 256);
		i2c_send(val_data[z] % 256);	
		_delay_ms(10);
	}
	i2c_stop();
	_delay_ms(10);
}

void i2cReadWordArray(unsigned char device_id, uint16_t addr, uint16_t *val_data, uint8_t lenght)
{
	uint8_t tempWordHolderH, tempWordHolderL, z;
	uint16_t addressHolder = addr;
	for(z = 0; z <= 31; z++)
	{
		i2c_read_2_bytes(addressHolder);
		val_data[z] = i2c_data_16;
		addressHolder++;
		addressHolder++;
		_delay_ms(10);
	}
	_delay_ms(500);
	for(z = 32; z <= 64; z++)
	{
		i2c_read_2_bytes(addressHolder);
		val_data[z] = i2c_data_16;
		addressHolder++;
		addressHolder++;
		_delay_ms(10);
	}
	// i2c_start();
	// i2c_send(device_id);
	// i2c_send(addr / 256);	// Sending the 16 bit address in two 8 bits
	// i2c_send(addr % 256);
	// i2c_stop();
	// _delay_ms(5);
	// i2c_start();
	// i2c_send(device_id + 1);
	// for(uint8_t z = 0; z <= (lenght - 1); z++)
	// {
	// 	i2c_receive(1);		// receive and send acknowledge
	// 	tempWordHolderH = (i2c_data * 256);		// Shift the higher byte right by 8bits
	// 	_delay_ms(10);
	// 	i2c_receive(1);
	// 	tempWordHolderH =+ i2c_data; 
	// 	_delay_ms(10);
		
	// 	val_data[z] = tempWordHolderH;
	// }
	// 	tempWordHolderL = i2c_read_ack();

	// 	val_data[z] = (tempWordHolderH << 8) + tempWordHolderL;
	// }
	// tempWordHolderH = i2c_read_ack();
	// tempWordHolderL = i2c_read_nack();

	// val_data[lenght] = (tempWordHolderH << 8) + tempWordHolderL;
	//i2c_stop();
}

/*********************** Added 26.04.2017 	*****************************************************/

void eeprom_write_i2c(unsigned char device_id, uint16_t addr, uint8_t val_data)
{	
	do
	{
		i2c_start();
		i2c_send(device_id);
		if((TWSR & 0xF8) == 0x20)		// Masking out the prescalar bits from the status register. Stop I2C if no acknowledge. Reading the status register
		i2c_stop();
	}
	while((TWSR & 0xF8) == 0x20);				//loop until acknowledge is received
	_delay_us(50);
	
	i2c_send(addr / 256);			
	i2c_send(addr % 256);			
	i2c_send(val_data);            		
	i2c_stop();	
	_delay_ms(10);
}

void deactivate_category_buttons_and_save(uint16_t addr, char button_no)
{
if(cod_typ == 'B')
{
	lcd_cmd(0x01);
	lcd_data_str("Saving Vote");		
	/*		When storing a vote count o EEPROM, first the initial value stored in the location of EEPROM is read
			that value is incremented by one and rewritten to that same location								*/
	i2c_read_2_bytes(addr);	
	_delay_ms(100);
	
	/***********************	17.09.2015	**************************************/
	if (mach_typ == 'D')	// Char for Donor, can be set as anything else
	{						// Requirement of Parsi panchayat
		i2c_data_16 = i2c_data_16 + 2;						// Increment the value by 2 
	}
										
	else	i2c_data_16 = i2c_data_16 + 1;						// Increment the value by 1

	/**********************	 17.09.2015	 *******************************************/

		i2c_write_2_bytes(EEPROM_DEV_ADDR, addr, i2c_data_16);		// Save the Incremented value back to the same location
	
		votes_counter = votes_counter - 1;					// Decrement the total votes a voter can cast
			if(!vote_cast)		// increment the voter_count when first vote is cast by a voter
			voter_count++;
			vote_cast = 1;
			voting_on = 1;
		i2c_send_byte(PIC_DEV_ADDR, button_no);		// Indicate to PIC LED controller to light up the respective buttons LED
		Buzzer = 1;				// Buzzer ON
		LED = 0;				// LED ON (ACTIVE LOW)*/

/**********************************************	 Added 24.04.2017 	Printing on Paper ********************************/
		printName(button_no, voter_count);

/**********************************************	 Added 24.04.2017 	Printing on Paper ********************************/
		
}

else                   // below code Compulsory all votes alloted to voter have to be cast for the votes to be saved
	{
	unsigned int i;
		if(!vote_cast)
		vote_cast = 1;
		voting_on = 1;
		voted_button_array[elem_count] = addr;
		eeprom_write_byte(elem_count*2,voted_button_array[elem_count]);// saving button addresses in internal eeprom 
		elem_count++;
		votes_counter = votes_counter - 1;					// Decrement the total votes a voter can cast
		i2c_send_byte(PIC_DEV_ADDR, button_no);		// Indicate to PIC LED controller to light up the respective buttons LED
		Buzzer = 1;				// Buzzer ON
		LED = 0;				// LED ON (ACTIVE LOW)
	}	
}

void i2c_send_byte(unsigned char device_id, unsigned char data)
{
	do
	{
		i2c_start();
		i2c_send(device_id);
		if((TWSR & 0xF8) == 0x20)		// Masking out the prescalar bits from the status register. Stop I2C if no acknowledge. Reading the status register
		i2c_stop();
	}
	while((TWSR & 0xF8) == 0x20);				//loop until acknowledge is received
	_delay_us(50);
	i2c_send(data);
	i2c_stop();
}

void light_leds()

{										// 8.10.15 Needs to be implemented better.
	if(vote_indicator_1)
	i2c_send_byte(PIC_DEV_ADDR, 1);
		_delay_us(100);
	if(vote_indicator_2)
	i2c_send_byte(PIC_DEV_ADDR, 2);
		_delay_us(100);
	if(vote_indicator_3)
	i2c_send_byte(PIC_DEV_ADDR, 3);
		_delay_us(100);
	if(vote_indicator_4)
	i2c_send_byte(PIC_DEV_ADDR, 4);
		_delay_us(100);
	if(vote_indicator_5)
	i2c_send_byte(PIC_DEV_ADDR, 5);
		_delay_us(100);
	if(vote_indicator_6)
	i2c_send_byte(PIC_DEV_ADDR, 6);
		_delay_us(100);
	if(vote_indicator_7)
	i2c_send_byte(PIC_DEV_ADDR, 7);
		_delay_us(100);
	if(vote_indicator_8)
	i2c_send_byte(PIC_DEV_ADDR, 8);
		_delay_us(100);
	if(vote_indicator_9)
	i2c_send_byte(PIC_DEV_ADDR, 9);
		_delay_us(100);
	if(vote_indicator_10)
	i2c_send_byte(PIC_DEV_ADDR, 10);
		_delay_us(100);
	if(vote_indicator_11)
	i2c_send_byte(PIC_DEV_ADDR, 11);
		_delay_us(100);
	if(vote_indicator_12)
	i2c_send_byte(PIC_DEV_ADDR, 12);
		_delay_us(100);
	if(vote_indicator_13)
	i2c_send_byte(PIC_DEV_ADDR, 13);
		_delay_us(100);
	if(vote_indicator_14)
	i2c_send_byte(PIC_DEV_ADDR, 14);
		_delay_us(100);	
	if(vote_indicator_15)
	i2c_send_byte(PIC_DEV_ADDR, 15);
		_delay_us(100);
	if(vote_indicator_16)
	i2c_send_byte(PIC_DEV_ADDR, 16);
		_delay_us(100);
	if(vote_indicator_17)
	i2c_send_byte(PIC_DEV_ADDR, 17);
		_delay_us(100);
	if(vote_indicator_18)
	i2c_send_byte(PIC_DEV_ADDR, 18);
		_delay_us(100);
	if(vote_indicator_19)
	i2c_send_byte(PIC_DEV_ADDR, 19);
		_delay_us(100);
	if(vote_indicator_20)
	i2c_send_byte(PIC_DEV_ADDR, 20);
		_delay_us(100);
	if(vote_indicator_21)
	i2c_send_byte(PIC_DEV_ADDR, 21);
		_delay_us(100);
	if(vote_indicator_22)
	i2c_send_byte(PIC_DEV_ADDR, 22);
		_delay_us(100);
	if(vote_indicator_23)
	i2c_send_byte(PIC_DEV_ADDR, 23);
		_delay_us(100);
	if(vote_indicator_24)
	i2c_send_byte(PIC_DEV_ADDR, 24);
		_delay_us(100);
	if(vote_indicator_25)
	i2c_send_byte(PIC_DEV_ADDR, 25);
		_delay_us(100);
	if(vote_indicator_26)
	i2c_send_byte(PIC_DEV_ADDR, 26);
		_delay_us(100);
	if(vote_indicator_27)
	i2c_send_byte(PIC_DEV_ADDR, 27);
		_delay_us(100);
	if(vote_indicator_28)
	i2c_send_byte(PIC_DEV_ADDR, 28);
		_delay_us(100);
	if(vote_indicator_29)
	i2c_send_byte(PIC_DEV_ADDR, 29);
		_delay_us(100);
	if(vote_indicator_30)
	i2c_send_byte(PIC_DEV_ADDR, 30);
		_delay_us(100);
	if(vote_indicator_31)
	i2c_send_byte(PIC_DEV_ADDR, 31);
		_delay_us(100);
	if(vote_indicator_32)
	i2c_send_byte(PIC_DEV_ADDR, 32);
		_delay_us(100);
	if(vote_indicator_33)
	i2c_send_byte(PIC_DEV_ADDR, 33);
	_delay_us(100);
	if(vote_indicator_34)
	i2c_send_byte(PIC_DEV_ADDR, 34);
	_delay_us(100);
	if(vote_indicator_35)
	i2c_send_byte(PIC_DEV_ADDR, 35);
	_delay_us(100);
	if(vote_indicator_36)
	i2c_send_byte(PIC_DEV_ADDR, 36);
	_delay_us(100);
	if(vote_indicator_37)
	i2c_send_byte(PIC_DEV_ADDR, 37);
	_delay_us(100);
	if(vote_indicator_38)
	i2c_send_byte(PIC_DEV_ADDR, 38);
	_delay_us(100);
	if(vote_indicator_39)
	i2c_send_byte(PIC_DEV_ADDR, 39);
	_delay_us(100);
	if(vote_indicator_40)
	i2c_send_byte(PIC_DEV_ADDR, 40);
	_delay_us(100);
	if(vote_indicator_41)
	i2c_send_byte(PIC_DEV_ADDR, 41);
	_delay_us(100);
	if(vote_indicator_42)
	i2c_send_byte(PIC_DEV_ADDR, 42);
	_delay_us(100);
	if(vote_indicator_43)
	i2c_send_byte(PIC_DEV_ADDR, 43);
	_delay_us(100);
	if(vote_indicator_44)
	i2c_send_byte(PIC_DEV_ADDR, 44);
	_delay_us(100);
	if(vote_indicator_45)
	i2c_send_byte(PIC_DEV_ADDR, 45);
	_delay_us(100);
	if(vote_indicator_46)	
	i2c_send_byte(PIC_DEV_ADDR, 46);
	_delay_us(100);
	if(vote_indicator_47)
	i2c_send_byte(PIC_DEV_ADDR, 47);
	_delay_us(100);
	if(vote_indicator_48)
	i2c_send_byte(PIC_DEV_ADDR, 48);
	_delay_us(100);
	if(vote_indicator_49)
	i2c_send_byte(PIC_DEV_ADDR, 49);
	_delay_us(100);
	if(vote_indicator_50)
	i2c_send_byte(PIC_DEV_ADDR, 50);
	_delay_us(100);
	if(vote_indicator_51)
	i2c_send_byte(PIC_DEV_ADDR, 51);
	_delay_us(100);
	if(vote_indicator_52)
	i2c_send_byte(PIC_DEV_ADDR, 52);
	_delay_us(100);
	if(vote_indicator_53)
	i2c_send_byte(PIC_DEV_ADDR, 53);
	_delay_us(100);
	if(vote_indicator_54)
	i2c_send_byte(PIC_DEV_ADDR, 54);
	_delay_us(100);
	if(vote_indicator_55)
	i2c_send_byte(PIC_DEV_ADDR, 55);
	_delay_us(100);
	if(vote_indicator_56)
	i2c_send_byte(PIC_DEV_ADDR, 56);
	_delay_us(100);
	if(vote_indicator_57)
	i2c_send_byte(PIC_DEV_ADDR, 57);
	_delay_us(100);
	if(vote_indicator_58)
	i2c_send_byte(PIC_DEV_ADDR, 58);
	_delay_us(100);
	if(vote_indicator_59)
	i2c_send_byte(PIC_DEV_ADDR, 59);
	_delay_us(100);
	if(vote_indicator_60)
	i2c_send_byte(PIC_DEV_ADDR, 60);
	_delay_us(100);
	if(vote_indicator_61)
	i2c_send_byte(PIC_DEV_ADDR, 61);
	_delay_us(100);
	if(vote_indicator_62)
	i2c_send_byte(PIC_DEV_ADDR, 62);
	_delay_us(100);
	if(vote_indicator_63)
	i2c_send_byte(PIC_DEV_ADDR, 63);
	_delay_us(100);
	if(vote_indicator_64)
	i2c_send_byte(PIC_DEV_ADDR, 64);
	_delay_us(100);
	
}

void vote_registration()  			// Function to count votes
{
	unsigned char no_of_votes_cat1, no_of_votes_cat2, no_of_votes_cat3, no_of_votes_cat4, no_of_votes_cat5, no_of_votes_cat6, no_of_votes_cat7, no_of_votes_cat8;
/*		
	This section identifies which button is being pressed and which category it is a part of. For the ATmega32A a maximum of 8 categories is possible with 15 buttons per category.
	This same code can be copied for other AVR controllers like ATmega2560. More buttons and categories can be appended if required below.
	e.g. category 1 is 2 buttons and category 2 is 4 buttons. Category 3 is 2 buttons and category 4 is 4 buttons.
				So during configuration categories1_2 holds 0x42
										categories3_4 holds 0x42
				below category size 1 is calculated by splitting categories1_2, considering only 2
						category size 2 is calculated by splitting considering only 4 and adding with 2
				So what is being calculated is till which button a category extends. This is used when checking if a button is in a category.
				A button in a particular category has to be greater than the last button of the previous category but less than or equal to the last button of its own category.	*/
	
		uint8_t category_size_1 = (i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F);		// This is done to reduce repeated code showing calculations done when checking the value of a category.
		uint8_t category_size_2 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4));
		uint8_t category_size_3 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F));
		uint8_t category_size_4 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0xF0) >> 4));
		uint8_t category_size_5 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0x0F));
		uint8_t category_size_6	= ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0xF0) >> 4));
		uint8_t category_size_7 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories7_8) & 0x0F));
		uint8_t category_size_8 = ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories1_2) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories3_4) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories5_6) & 0xF0) >> 4) + (i2c_read_byte(EEPROM_DEV_ADDR, categories7_8) & 0x0F) + ((i2c_read_byte(EEPROM_DEV_ADDR, categories7_8) & 0xF0) >> 4));
	
	if (!voting_on)
	{
		no_of_votes_cat1 = i2c_read_byte(EEPROM_DEV_ADDR, catg1_votes);		// Here no. of votes a voter can cast for a particular category are loaded into variables no_of_votes_catx
		no_of_votes_cat2 = i2c_read_byte(EEPROM_DEV_ADDR, catg2_votes);		
		no_of_votes_cat3 = i2c_read_byte(EEPROM_DEV_ADDR, catg3_votes);
		no_of_votes_cat4 = i2c_read_byte(EEPROM_DEV_ADDR, catg4_votes);
		no_of_votes_cat5 = i2c_read_byte(EEPROM_DEV_ADDR, catg5_votes);
		no_of_votes_cat6 = i2c_read_byte(EEPROM_DEV_ADDR, catg6_votes);
		no_of_votes_cat7 = i2c_read_byte(EEPROM_DEV_ADDR, catg7_votes);
		//no_of_votes_cat8 = i2c_read_byte(EEPROM_DEV_ADDR, catg8_votes);
	}
	else if (voting_on)
			{
				no_of_votes_cat1 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_1_bkp);		// Here no. of votes remaining ina particular category from the previous session is loaded
				no_of_votes_cat2 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_2_bkp);		
				no_of_votes_cat3 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_3_bkp);
				no_of_votes_cat4 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_4_bkp);
				no_of_votes_cat5 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_5_bkp);
				no_of_votes_cat6 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_6_bkp);
				no_of_votes_cat7 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_7_bkp);
				//no_of_votes_cat8 = i2c_read_byte(EEPROM_DEV_ADDR, no_votes_8_bkp);
				
				light_leds();
			}
	
	while(votes_counter >= 0)				// The program stays in this loop until the voter has cast all of his/her votes or until INIT is pressed.
	{	
		if(cod_typ == 'B')
			i2c_write_2_bytes(EEPROM_DEV_ADDR, voters_voted, voter_count);	// Store the Count of no. of voters in EEPROM.
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_1_8, myregister1);		// backup the vote_indicators
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_9_16, myregister2);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_17_24, myregister3);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_25_32, myregister4);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_33_40, myregister5);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_41_48, myregister6);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_49_56, myregister7);
		eeprom_write_i2c(EEPROM_DEV_ADDR, vote_indic_57_64, myregister8);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_1_bkp, no_of_votes_cat1);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_2_bkp, no_of_votes_cat2);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_3_bkp, no_of_votes_cat3);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_4_bkp, no_of_votes_cat4);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_5_bkp, no_of_votes_cat5);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_6_bkp, no_of_votes_cat6);
		eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_7_bkp, no_of_votes_cat7);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, no_votes_8_bkp, no_of_votes_cat8);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, gen_vote_count_bkp, gen_vote_count);
		eeprom_write_i2c(EEPROM_DEV_ADDR, votes_count_bkp, votes_counter);
		eeprom_write_i2c(EEPROM_DEV_ADDR,elem_count_bkp,elem_count);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, flag_register_bkp, flag_register);
		

/*******************************	For Testing	- Comment this section when programming the controller	  ****************************************************************/			
		//USART_putstring("1: ");
		//USART_Transmit_dec(no_of_votes_cat1);
		//USART_SendByte(0x0D);
		//USART_putstring("2: ");
		//USART_Transmit_dec(no_of_votes_cat2);
		//USART_SendByte(0x0D);
		//USART_putstring("3: ");
		//USART_Transmit_dec(no_of_votes_cat3);
		//USART_SendByte(0x0D);
		//USART_putstring("4: ");
		//USART_Transmit_dec(no_of_votes_cat4);
		//USART_SendByte(0x0D);
		//USART_putstring("5: ");
		//USART_Transmit_dec(no_of_votes_cat5);
		//USART_SendByte(0x0D);
		//USART_putstring("6: ");
		//USART_Transmit_dec(no_of_votes_cat6);
		//USART_SendByte(0x0D);
		//USART_putstring("7: ");
		//USART_Transmit_dec(no_of_votes_cat7);
		//USART_SendByte(0x0D);
		//USART_putstring("G: ");
		//USART_Transmit_dec(gen_vote_count);
		//USART_SendByte(0x0D);
		
/*******************************************************************************************************************/	
			
		lcd_cmd(0x01);						
		lcd_data_str("Votes remaining:");		//	Display the no. of votes remaining for that voter
		lcd_cmd(0xC0);
		lcd_data_int(votes_counter);
		
		LED = 1;				// LED off
		Buzzer = 0;				// Buzzer off
		
		/*******************************************************  added by neeraj on 08/12/15   ***********************************************************************/
		if(votes_counter == 0)
		{
			while ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && (INIT == 1)); // Wait while no button is pressed //
			_delay_ms(100);
			if(INIT == 0)
			{
				while(INIT == 0);
				i2c_send_byte(PIC_DEV_ADDR, 0);		// Clear the LED's
				votes_counter=0;
				vote_cast=1;
				return;
			}
		}
		/*******************************************************  added by neeraj on 08/12/15   ***********************************************************************/	
		if(votes_counter > 0)             // added by neeraj on 08/12/2015
		{	
			while ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && (INIT == 1)) // Wait while no button is pressed //
			{
				USART_Transmit_dec(PINC);
				USART_SendByte(0x0D);
			}
			_delay_ms(100);		// To press bouncing button effect, false press.
		
			// Note PINB is where button data appears.
			USART_Transmit_dec(PINC);
			USART_SendByte(0x0D);
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x00) && (1 <= candidate_count) && (vote_indicator_1 == 0))		// Checking if button falls in the number of candidates selected when configuring the machine.
			{																										// Also checking if button was previously pressed by the voter when allowed to cast multiple votes.
				if((1 <= category_size_1) && (no_of_votes_cat1 != 0x00))				//
				{
					//USART_putstring("1C1 ");			// comment this out in final product
					vote_indicator_1 = 1;				// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button1_addr, 1);
					//i2c_send_byte(PIC_DEV_ADDR, 1);
					no_of_votes_cat1--;
				}
				else if((1 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("1G ");		// comment this out in final product
					vote_indicator_1 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button1_addr, 1);
					//i2c_send_byte(PIC_DEV_ADDR, 1);
					gen_vote_count--;
				}	
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x01) && (2 <= candidate_count) && vote_indicator_2 == 0)
			{
				if((2 <= category_size_1) && (no_of_votes_cat1 != 0x00))				//
				{
					//USART_putstring("2C1 ");				// comment this out in final product
					vote_indicator_2 = 1; 					// Indicating button 1 was pressed
					no_of_votes_cat1--;
					//i2c_send_byte(PIC_DEV_ADDR, 0x02);
					deactivate_category_buttons_and_save(button2_addr, 2);  // provide starting and ending buttons as arguments
				}
				else if((2 > category_size_1) && (2 <= category_size_2) && (no_of_votes_cat2 != 0))	// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("2C2 ");
					vote_indicator_2 = 1;				// Indicating button 2 was pressed.
					deactivate_category_buttons_and_save(button2_addr, 2);
					//i2c_send_byte(PIC_DEV_ADDR, 0x02);
					no_of_votes_cat2--;
				}
				else if ((2 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("2G ");		// comment this out in final product
					vote_indicator_2 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button2_addr, 2);
					//i2c_send_byte(PIC_DEV_ADDR, 0x02);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x02) && (3 <= candidate_count) && vote_indicator_3==0)
			{
				if((3 <= category_size_1)	&& (no_of_votes_cat1 != 0x00))				//
				{
					//USART_putstring("3C1 ");
					vote_indicator_3 = 1;				// Indicating button 3 was pressed.
					deactivate_category_buttons_and_save(button3_addr, 3);			// provide starting and ending buttons as arguments
					//i2c_send_byte(PIC_DEV_ADDR, 0x03);
					no_of_votes_cat1--;
				}
				else if((3 > category_size_1) && (3 <= category_size_2) && (no_of_votes_cat2 != 0))	// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("3C2 ");
					vote_indicator_3 = 1;				// Indicating button 3 was pressed.
					deactivate_category_buttons_and_save(button3_addr, 3);
					//i2c_send_byte(PIC_DEV_ADDR, 0x03);
					no_of_votes_cat2--;
				}
				else if ((3 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("3G ");		// comment this out in final product
					vote_indicator_3 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button3_addr, 3);
					//i2c_send_byte(PIC_DEV_ADDR, 0x03);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x03) && (4 <= candidate_count) && vote_indicator_4 == 0)
			{
				if((4 <= category_size_1) && no_of_votes_cat1 != 0) 			//
				{
					//USART_putstring("4C1 ");
					vote_indicator_4 = 1;			// Indicating button 3 was pressed.
					deactivate_category_buttons_and_save(button4_addr, 4);  // provide starting and ending buttons as arguments
					//i2c_send_byte(PIC_DEV_ADDR, 0x04);
					no_of_votes_cat1--;
				}
			
				else if((4 > category_size_1) && (4 <= category_size_2) && (no_of_votes_cat2 != 0))	// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("4C2 ");
					vote_indicator_4 = 1;			// Indicating button 3 was pressed.
					deactivate_category_buttons_and_save(button4_addr, 4);
					//i2c_send_byte(PIC_DEV_ADDR, 0x04);
					no_of_votes_cat2--;
				}
			
				/// Added by Brendz on March 7th 2017. Btn 4 fell into category 3 and was not checked in any if Conditions
				else if((4 > category_size_2) && (4 <= category_size_3) && (no_of_votes_cat3 != 0))	// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("4C2 ");
					vote_indicator_4 = 1;			// Indicating button 3 was pressed.
					deactivate_category_buttons_and_save(button4_addr, 4);
					//i2c_send_byte(PIC_DEV_ADDR, 0x04);
					no_of_votes_cat3--;
				}
				else if ((4 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("4G ");		// comment this out in final product
					vote_indicator_4 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button4_addr, 4);
					//i2c_send_byte(PIC_DEV_ADDR, 0x04);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x04) && (5 <= candidate_count) && vote_indicator_5==0)
			{
				if((5 <= category_size_1) && (no_of_votes_cat1 != 0x00))	//
				{
					//USART_putstring("5C1 ");
					vote_indicator_5 = 1;							// Indicating button 5 was pressed.
					deactivate_category_buttons_and_save(button5_addr, 5);		// provide starting and ending buttons as arguments
					//i2c_send_byte(PIC_DEV_ADDR, 0x05);
					no_of_votes_cat1--;
				}
				else if((5 > category_size_1) && (5 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("5C2 ");
					vote_indicator_5 = 1;	// Indicating button 5 was pressed.
					deactivate_category_buttons_and_save(button5_addr, 5);
					//i2c_send_byte(PIC_DEV_ADDR, 0x05);
					no_of_votes_cat2--;
				}
				else if((5 > category_size_2) && (5 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("5C3 ");
					vote_indicator_5 = 1;	// Indicating button 5 was pressed.
					deactivate_category_buttons_and_save(button5_addr, 5);
					//i2c_send_byte(PIC_DEV_ADDR, 0x05);
					no_of_votes_cat3--;
				}
				else if((5 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("5G ");		// comment this out in final product
					vote_indicator_5 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button5_addr, 5);
					//i2c_send_byte(PIC_DEV_ADDR, 0x05);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x05) && (6 <= candidate_count) && (vote_indicator_6 == 0))
			{
				if((6 <= category_size_1) && (no_of_votes_cat1 != 0))		// 
				{
					//USART_putstring("6C1 ");
					vote_indicator_6 = 1;					// Indicating button 6 was presses.
					deactivate_category_buttons_and_save(button6_addr, 6);  // provide starting and ending buttons as arguments
					//i2c_send_byte(PIC_DEV_ADDR, 0x06);
					no_of_votes_cat1--;
				}
				else if((6 > category_size_1) && (6 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("6C2 ");
					vote_indicator_6 = 1;	// Indicating button 6 was pressed.
					deactivate_category_buttons_and_save(button6_addr, 6);
					//i2c_send_byte(PIC_DEV_ADDR, 0x06);
					no_of_votes_cat2--;
				}
				else if((6 > category_size_2) && (6 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("6C3 ");
					vote_indicator_6 = 1;	// Indicating button 6 was presses.
					deactivate_category_buttons_and_save(button6_addr, 6);
					//i2c_send_byte(PIC_DEV_ADDR, 0x06);
					no_of_votes_cat3--;
				}
				else if ((6 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("6G ");		// comment this out in final product
					vote_indicator_6 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button6_addr, 6);
					//i2c_send_byte(PIC_DEV_ADDR, 0x06);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x06) && (7 <= candidate_count) && vote_indicator_7==0)
			{
				if((7 <= category_size_1) && (no_of_votes_cat1 != 0))			//.
				{
					//USART_putstring("7C1 ");
					vote_indicator_7 = 1;	// Indicating button 7 was presses.
					deactivate_category_buttons_and_save(button7_addr, 7);  // provide starting and ending buttons as arguments
					//i2c_send_byte(PIC_DEV_ADDR, 0x07);
					no_of_votes_cat1--;
				}
				else if((7 > category_size_1) && (7 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("7C2 ");
					vote_indicator_7 = 1;	// Indicating button 7 was presses.
					deactivate_category_buttons_and_save(button7_addr, 7);
					//i2c_send_byte(PIC_DEV_ADDR, 0x07);
					no_of_votes_cat2--;
				}
				else if((7 > category_size_2) && (7 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("7C3 ");
					vote_indicator_7 = 1;	// Indicating button 7 was presses.
					deactivate_category_buttons_and_save(button7_addr, 7);
					//i2c_send_byte(PIC_DEV_ADDR, 0x07);
					no_of_votes_cat3--;
				}
				else if((7 > category_size_3) && (7 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("7C4 ");
					vote_indicator_7 = 1;	// Indicating button 7 was presses.
					deactivate_category_buttons_and_save(button7_addr, 7);
					//i2c_send_byte(PIC_DEV_ADDR, 0x07);
					no_of_votes_cat4--;
				}
				else if ((7 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("7G ");		// comment this out in final product
					vote_indicator_7 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button7_addr, 7);
					//i2c_send_byte(PIC_DEV_ADDR, 0x07);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x07) && (8 <= candidate_count) && vote_indicator_8==0)
			{
				if((8 <= category_size_1) && (no_of_votes_cat1 != 0))		//
				{
					//USART_putstring("8C1 ");
					vote_indicator_8 = 1;	// Indicating button 8 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					deactivate_category_buttons_and_save(button8_addr, 8);  // provide starting and ending buttons as arguments
					no_of_votes_cat1--;
				}
				else if((8 > category_size_1) && (8 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("8C2 ");
					vote_indicator_8 = 1;	// Indicating button 8 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					deactivate_category_buttons_and_save(button8_addr, 8);
					no_of_votes_cat2--;
				}
				else if((8 > category_size_2) && (8 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("8C3 ");
					vote_indicator_8 = 1;
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					deactivate_category_buttons_and_save(button8_addr, 8);
					no_of_votes_cat3--;
				}
				else if((8 > category_size_3) && (8 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("8C4 ");
					vote_indicator_8 = 1;
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					deactivate_category_buttons_and_save(button8_addr, 8);
					no_of_votes_cat4--;
				}
			
				/// Added by Brendz on March 7th 2017. Btn 8 fell into category 5 and was not checked in any if Conditions
				else if((8 > category_size_4) && (8 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("8C4 ");
					vote_indicator_8 = 1;
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					deactivate_category_buttons_and_save(button8_addr, 8);
					no_of_votes_cat5--;
				}
			
				else if ((8 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("8G ");		// comment this out in final product
					vote_indicator_8 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button8_addr, 8);
					//i2c_send_byte(PIC_DEV_ADDR, 0x08);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x08) && (9 <= candidate_count) && vote_indicator_9==0)
			{
				if((9 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("9C1 ");
					vote_indicator_9 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x09);
					deactivate_category_buttons_and_save(button9_addr, 9);
					no_of_votes_cat1--;
				}
			
				else if((9 > category_size_1) && (9 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("9C2 ");
					vote_indicator_9 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x09);
					deactivate_category_buttons_and_save(button9_addr, 9);
					no_of_votes_cat2--;
				}
				else if((9 > category_size_2) && (9 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("9C3 ");
					vote_indicator_9 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x09);
					deactivate_category_buttons_and_save(button9_addr, 9);
					no_of_votes_cat3--;
				}
				else if((9 > category_size_3) && (9 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("9C4 ");
					vote_indicator_9 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 0x09);
					deactivate_category_buttons_and_save(button9_addr, 9);
					no_of_votes_cat4--;
				}
				else if((9 > category_size_4) && (9 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("9C5 ");
					vote_indicator_9 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 9);
					deactivate_category_buttons_and_save(button9_addr, 9);
					no_of_votes_cat5--;
				}
				else if ((9 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("9G ");		// comment this out in final product
					vote_indicator_9 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button9_addr, 9);
					//i2c_send_byte(PIC_DEV_ADDR, 0x09);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x09) && (10 <= candidate_count) && vote_indicator_10==0)// && (gen_vote_count != 0))
			{
				if((10 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("10C1 ");
					vote_indicator_10 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					deactivate_category_buttons_and_save(button10_addr, 10);
					no_of_votes_cat1--;
				}
				else if((10 > category_size_1) && (10 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("10C2 ");
					vote_indicator_10 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button10_addr, 10);
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					no_of_votes_cat2--;
				}
				else if((10 > category_size_2) && (10 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("10C3 ");
					vote_indicator_10 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button10_addr, 10);
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					no_of_votes_cat3--;
				}
				else if((10 > category_size_3) && (10 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("10C4 ");
					vote_indicator_10 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button10_addr, 10);
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					no_of_votes_cat4--;
				}
				else if((10 > category_size_4) && (10 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("10 C 5");
					vote_indicator_10 = 1;	// Indicating button 10 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					deactivate_category_buttons_and_save(button10_addr, 10);
					no_of_votes_cat5--;
				}
				else if ((10 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("10G ");		// comment this out in final product
					vote_indicator_10 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button10_addr, 10);
					//i2c_send_byte(PIC_DEV_ADDR, 10);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0A) && (11 <= candidate_count) && vote_indicator_11==0)// && (gen_vote_count != 0))
			{
				if((11 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("11C1 ");
					vote_indicator_11 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					deactivate_category_buttons_and_save(button11_addr, 11);
					no_of_votes_cat1--;
				}
				else if((11 > category_size_1) && (11 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("11C2 ");
					vote_indicator_11 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button11_addr, 11);
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					no_of_votes_cat2--;
				}
				else if((11 > category_size_2) && (11 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("11C3 ");
					vote_indicator_11 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button11_addr, 11);
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					no_of_votes_cat3--;
				}
				else if((11 > category_size_3) && (11 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("11C4 ");
					vote_indicator_11 = 1;	// Indicating button 10 was presses.
					deactivate_category_buttons_and_save(button11_addr, 11);
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					no_of_votes_cat4--;
				}
				else if((11 > category_size_4) && (11 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("11C5 ");
					vote_indicator_11 = 1;	// Indicating button 10 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					deactivate_category_buttons_and_save(button11_addr, 11);
					no_of_votes_cat5--;
				}
				else if((11 > category_size_5) && (11 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("11C6 ");
					vote_indicator_11 = 1;	// Indicating button 11 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					deactivate_category_buttons_and_save(button11_addr, 11);
					no_of_votes_cat6--;
				}
				else if ((11 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("11G ");		// comment this out in final product
					vote_indicator_11 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button11_addr, 11);
					//i2c_send_byte(PIC_DEV_ADDR, 11);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0B) && (12 <= candidate_count) && vote_indicator_12==0)// && (gen_vote_count != 10))
			{
				if((12 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("12C1 ");
					vote_indicator_12 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					deactivate_category_buttons_and_save(button12_addr, 12);
					no_of_votes_cat1--;
				}
				else if((12 > category_size_1) && (12 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("12C2 ");
					vote_indicator_12 = 1;	// Indicating button 12 was presses.
					deactivate_category_buttons_and_save(button12_addr, 12);
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					no_of_votes_cat2--;
				}
				else if((12 > category_size_2) && (12 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("12C3 ");
					vote_indicator_12 = 1;	// Indicating button 12 was presses.
					deactivate_category_buttons_and_save(button12_addr, 12);
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					no_of_votes_cat3--;
				}
				else if((12 > category_size_3) && (12 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("12C4 ");
					vote_indicator_12 = 1;	// Indicating button 12 was presses.
					deactivate_category_buttons_and_save(button12_addr, 12);
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					no_of_votes_cat4--;
				}
				else if((12 > category_size_4) && (12 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("12C5 ");
					vote_indicator_12 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					deactivate_category_buttons_and_save(button12_addr, 12);
					no_of_votes_cat5--;
				}
				else if((12 > category_size_5) && (12 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("12C6 ");
					vote_indicator_12 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					deactivate_category_buttons_and_save(button12_addr, 12);
					no_of_votes_cat6--;
				}
				else if ((12 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("12G ");		// comment this out in final product
					vote_indicator_12 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button12_addr, 12);
					//i2c_send_byte(PIC_DEV_ADDR, 12);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0C) && (13 <= candidate_count) && (vote_indicator_13 == 0))// && (gen_vote_count != 10))
			{
				if((13 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("13C1 ");
					vote_indicator_13 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat1--;
				}
				else if((13 > category_size_1) && (13 <= category_size_2) && (no_of_votes_cat2 != 0))// check if this button in 2nd category combination of 1st and 2nd  category buttons.
				{
					//USART_putstring("13C2 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat2--;
				}
				else if((13 > category_size_2) && (13 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("13C3 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat3--;	
				}
				else if((13 > category_size_3) && (13 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("13C4 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat4--;
				}
				else if((13 > category_size_4) && (13 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("13C5 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat5--;
				}
				else if((13 > category_size_5) && (13 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("13C6 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat6--;
				}
				else if((13 > category_size_6) && (13 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("13C7 ");
					vote_indicator_13 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					deactivate_category_buttons_and_save(button13_addr, 13);
					no_of_votes_cat7--;
				}
				//else if((13 > category_size_7) && (13 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("13C8 ");
					//vote_indicator_13 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 13);
					//deactivate_category_buttons_and_save(button13_addr, 13);
					//no_of_votes_cat8--;
				//}
				else if ((13 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("13G ");		// comment this out in final product
					vote_indicator_13 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button13_addr, 13);
					//i2c_send_byte(PIC_DEV_ADDR, 13);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0D) && (14 <= candidate_count) && vote_indicator_14==0)
			{
				if((14 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("14C1 ");
					vote_indicator_14 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat1--;
				}
				else if((14 > category_size_1) && (14 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("14C2 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat2--;
				}
				else if((14 > category_size_2) && (14 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("14C3 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat3--;
				}
				else if((14 > category_size_3) && (14 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("14C4 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat4--;
				}
				else if((14 > category_size_4) && (14 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("14C5 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat5--;
				}
				else if((14 > category_size_5) && (14 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("14C6 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat6--;
				}
				else if((14 > category_size_6) && (14 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("14C7 ");
					vote_indicator_14 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					deactivate_category_buttons_and_save(button14_addr, 14);
					no_of_votes_cat7--;
				}
				//else if((14 > category_size_7) && (14 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("14C8 ");
					//vote_indicator_14 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 14);
					//deactivate_category_buttons_and_save(button14_addr, 14);
					//no_of_votes_cat8--;
				//}
				else if ((14 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("14G ");		// comment this out in final product
					vote_indicator_14 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button14_addr, 14);
					//i2c_send_byte(PIC_DEV_ADDR, 14);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0E) && (15 <= candidate_count) && (vote_indicator_15==0))
			{
				if((15 <= category_size_1) && (no_of_votes_cat1 != 0))
				{
					//USART_putstring("15C1 ");
					vote_indicator_15 = 1;	// Indicating button 9 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat1--;
				}
				else if((15 > category_size_1) && (15 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("15C2 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat2--;
				}
				else if((15 > category_size_2) && (15 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("15C3 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat3--;
				}
				else if((15 > category_size_3) && (15 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("15C4 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat4--;
				}
				else if((15 > category_size_4) && (15 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("15C5 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat5--;
				}
				else if((15 > category_size_5) && (15 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("15C6 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat6--;
				}
				else if((15 > category_size_6) && (15 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("15C7 ");
					vote_indicator_15 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					deactivate_category_buttons_and_save(button15_addr, 15);
					no_of_votes_cat7--;
				}
				//else if((15 > category_size_7) && (15 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("15C8 ");
					//vote_indicator_15 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 15);
					//deactivate_category_buttons_and_save(button15_addr, 15);
					//no_of_votes_cat8--;
				//}
				else if ((15 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("15G ");		// comment this out in final product
					vote_indicator_15 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button15_addr, 15);
					//i2c_send_byte(PIC_DEV_ADDR, 15);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 1) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0F) && (16 <= candidate_count) && vote_indicator_16==0)
			{
				if((16 > category_size_1) && (16 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("16C2 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat2--;
				}
				else if((16 > category_size_2) && (16 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("16C3 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat3--;
				}
				else if((16 > category_size_3) && (16 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("16C4 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat4--;
				}
				else if((16 > category_size_4) && (16 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("16C5 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat5--;
				}
				else if((16 > category_size_5) && (16 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("16C6 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat6--;
				}
				else if((16 > category_size_6) && (16 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("16C7 ");
					vote_indicator_16 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					deactivate_category_buttons_and_save(button16_addr, 16);
					no_of_votes_cat7--;
				}
				//else if((16 > category_size_7) && (16 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("16C8 ");
					//vote_indicator_16 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 16);
					//deactivate_category_buttons_and_save(button16_addr, 16);
					//no_of_votes_cat8--;
				//}
				else if ((16 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("16G ");		// comment this out in final product
					vote_indicator_16 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button16_addr, 16);
					//i2c_send_byte(PIC_DEV_ADDR, 16);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x00) && (17 <= candidate_count) && vote_indicator_17==0)
			{
				if((17 > category_size_1) && (17 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("17C2 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat2--;
				}
				else if((17 > category_size_2) && (17 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("17C3 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat3--;
				}
				else if((17 > category_size_3) && (17 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("17C4 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat4--;
				}
				else if((17 > category_size_4) && (17 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("17C5 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat5--;
				}
				else if((17 > category_size_5) && (17 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("17C6 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat6--;
				}
				else if((17 > category_size_6) && (17 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("17C7 ");
					vote_indicator_17 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					deactivate_category_buttons_and_save(button17_addr, 17);
					no_of_votes_cat7--;
				}
				//else if((17 > category_size_7) && (17 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("17C8 ");
					//vote_indicator_17 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 17);
					//deactivate_category_buttons_and_save(button17_addr, 17);
					//no_of_votes_cat8--;
				//}
				else if((17 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("17G ");		// comment this out in final product
					vote_indicator_17 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button17_addr, 17);
					//i2c_send_byte(PIC_DEV_ADDR, 17);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x01) && (18 <= candidate_count) && vote_indicator_18==0)
			{
				if((18 > category_size_1) && (18 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("18C2 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat2--;
				}
				else if((18 > category_size_2) && (18 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("18C3 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat3--;
				}
				else if((18 > category_size_3) && (18 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("18C4 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat4--;
				}
				else if((18 > category_size_4) && (18 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("18C5 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat5--;
				}
				else if((18 > category_size_5) && (18 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("18C6 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat6--;
				}
				else if((18 > category_size_6) && (18 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("18C7 ");
					vote_indicator_18 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					deactivate_category_buttons_and_save(button18_addr, 18);
					no_of_votes_cat7--;
				}
				//else if((18 > category_size_7) && (18 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("18C8 ");
					//vote_indicator_18 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 18);
					//deactivate_category_buttons_and_save(button18_addr, 18);
					//no_of_votes_cat8--;
				//}
				else if((18 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("18G ");		// comment this out in final product
					vote_indicator_18 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button18_addr, 18);
					//i2c_send_byte(PIC_DEV_ADDR, 18);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x02) && (19 <= candidate_count) && vote_indicator_19==0)
			{
				if((19 > category_size_1) && (19 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("19C2 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat2--;
				}
				else if((19 > category_size_2) && (19 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("19C3 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat3--;
				}
				else if((19 > category_size_3) && (19 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("19C4 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat4--;
				}
				else if((19 > category_size_4) && (19 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("19C5 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat5--;
				}
				else if((19 > category_size_5) && (19 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("19C6 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat6--;
				}
				else if((19 > category_size_6) && (19 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("19C7 ");
					vote_indicator_19 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					deactivate_category_buttons_and_save(button19_addr, 19);
					no_of_votes_cat7--;
				}
				//else if((19 > category_size_7) && (19 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("19C8 ");
					//vote_indicator_19 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 19);
					//deactivate_category_buttons_and_save(button19_addr, 19);
					//no_of_votes_cat8--;
				//}
				else if((19 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("19G ");		// comment this out in final product
					vote_indicator_19 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button19_addr, 19);
					//i2c_send_byte(PIC_DEV_ADDR, 19);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x03) && (20 <= candidate_count) && vote_indicator_20==0)
			{
				if((20 > category_size_1) && (20 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("20C2 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat2--;
				}
				else if((20 > category_size_2) && (20 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("20C3 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat3--;
				}
				else if((20 > category_size_3) && (20 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("20C4 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat4--;
				}
				else if((20 > category_size_4) && (20 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("20C5 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat5--;
				}
				else if((20 > category_size_5) && (20 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("20C6 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat6--;
				}
				else if((20 > category_size_6) && (20 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("20C7 ");
					vote_indicator_20 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					deactivate_category_buttons_and_save(button20_addr, 20);
					no_of_votes_cat7--;
				}
				//else if((20 > category_size_7) && (20 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("20C8 ");
					//vote_indicator_20 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 20);
					//deactivate_category_buttons_and_save(button20_addr, 20);
					//no_of_votes_cat8--;
				//}
				else if((20 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("20G ");		// comment this out in final product
					vote_indicator_20 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button20_addr, 20);
					//i2c_send_byte(PIC_DEV_ADDR, 20);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x04) && (21 <= candidate_count) && vote_indicator_21==0)
			{
				if((21 > category_size_1) && (21 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("21C2 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat2--;
				}
				else if((21 > category_size_2) && (21 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("21C3 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat3--;
				}
				else if((21 > category_size_3) && (21 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("21C4 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat4--;
				}
				else if((21 > category_size_4) && (21 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("21C5 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat5--;
				}
				else if((21 > category_size_5) && (21 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("21C6 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat6--;
				}
				else if((21 > category_size_6) && (21 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("21C7 ");
					vote_indicator_21 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					deactivate_category_buttons_and_save(button21_addr, 21);
					no_of_votes_cat7--;
				}
				//else if((21 > category_size_7) && (21 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("21C8 ");
					//vote_indicator_21 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 21);
					//deactivate_category_buttons_and_save(button21_addr, 21);
					//no_of_votes_cat8--;
				//}
				else if((21 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("21G ");		// comment this out in final product
					vote_indicator_21 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button21_addr, 21);
					//i2c_send_byte(PIC_DEV_ADDR, 21);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x05) && (22 <= candidate_count) && vote_indicator_22==0)
			{
				if((22 > category_size_1) && (22 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("22C2 ");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat2--;
				}
				else if((22 > category_size_2) && (21 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("22C3 ");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat3--;
				}
				else if((22 > category_size_3) && (21 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("22C4 ");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat4--;
				}
				else if((22 > category_size_4) && (21 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("22C5 ");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat5--;
				}
				else if((22 > category_size_5) && (21 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("22C6");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat6--;
				}
				else if((22 > category_size_6) && (21 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("22C7");
					vote_indicator_22 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					deactivate_category_buttons_and_save(button22_addr, 22);
					no_of_votes_cat7--;
				}
				//else if((22 > category_size_7) && (21 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("22C8");
					//vote_indicator_22 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 22);
					//deactivate_category_buttons_and_save(button22_addr, 22);
					//no_of_votes_cat8--;
				//}
				else if((22 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("22G ");		// comment this out in final product
					vote_indicator_22 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button22_addr, 22);
					//i2c_send_byte(PIC_DEV_ADDR, 22);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x06) && (23 <= candidate_count) && vote_indicator_23==0)
			{
				if((23 > category_size_1) && (23 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("23C2 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat2--;
				}
				else if((23 > category_size_2) && (23 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("23C3 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat3--;
				}
				else if((23 > category_size_3) && (23 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("23C4 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat4--;
				}
				else if((23 > category_size_4) && (23 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("23C5 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat5--;
				}
				else if((23 > category_size_5) && (23 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("23C6 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat6--;
				}
				else if((23 > category_size_6) && (23 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("23C7 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat7--;
				}
				else if((23 > category_size_7) && (23 <= category_size_8) && (no_of_votes_cat8 != 0))
				{
					//USART_putstring("23C8 ");
					vote_indicator_23 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					deactivate_category_buttons_and_save(button23_addr, 23);
					no_of_votes_cat8--;
				}
				else if((23 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("23G ");		// comment this out in final product
					vote_indicator_23 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button23_addr, 23);
					//i2c_send_byte(PIC_DEV_ADDR, 23);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x07) && (24 <= candidate_count) && vote_indicator_24==0)
			{
				if((24 > category_size_1) && (24 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("24C2 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat2--;
				}
				else if((24 > category_size_2) && (24 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("24C3 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat3--;
				}
				else if((24 > category_size_3) && (24 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("24C4 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat4--;
				}
				else if((24 > category_size_4) && (24 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("24C5 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat5--;
				}
				else if((24 > category_size_5) && (24 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("24C6 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat6--;
				}
				else if((24 > category_size_6) && (24 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("24C7 ");
					vote_indicator_24 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					deactivate_category_buttons_and_save(button24_addr, 24);
					no_of_votes_cat7--;
				}
				//else if((24 > category_size_7) && (24 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("24C8 ");
					//vote_indicator_24 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 24);
					//deactivate_category_buttons_and_save(button24_addr, 24);
					//no_of_votes_cat8--;
				//}
				else if((24 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("24G ");		// comment this out in final product
					vote_indicator_24 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button24_addr, 24);
					//i2c_send_byte(PIC_DEV_ADDR, 24);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x08) && (25 <= candidate_count) && vote_indicator_25==0)
			{
				if((25 > category_size_1) && (25 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("25C2 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat2--;
				}
				else if((25 > category_size_2) && (25 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("25C3 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat3--;
				}
				else if((25 > category_size_3) && (25 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("25C4 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat4--;
				}
				else if((25 > category_size_4) && (25 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("25C5 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat5--;
				}
				else if((25 > category_size_5) && (25 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("25C6 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat6--;
				}
				else if((25 > category_size_6) && (25 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("25C7 ");
					vote_indicator_25 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					deactivate_category_buttons_and_save(button25_addr, 25);
					no_of_votes_cat7--;
				}
				//else if((25 > category_size_7) && (25 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("25C8 ");
					//vote_indicator_25 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 25);
					//deactivate_category_buttons_and_save(button25_addr, 25);
					//no_of_votes_cat8--;
				//}
				else if((25 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("25G ");		// comment this out in final product
					vote_indicator_25 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button25_addr, 25);
					//i2c_send_byte(PIC_DEV_ADDR, 25);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x09) && (26 <= candidate_count) && vote_indicator_26 == 0)
			{
				if((26 > category_size_1) && (26 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("26C2 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat2--;
				}
				else if((26 > category_size_2) && (26 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("26C3 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat3--;
				}
				else if((26 > category_size_3) && (26 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("26C4 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat4--;
				}
				else if((26 > category_size_4) && (26 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("26C5 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat5--;
				}
				else if((26 > category_size_5) && (26 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("26C6 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat6--;
				}
				else if((26 > category_size_6) && (26 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("26C7 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat7--;
				}
				else if((26 > category_size_7) && (26 <= category_size_8) && (no_of_votes_cat8 != 0))
				{
					//USART_putstring("26C8 ");
					vote_indicator_26 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					deactivate_category_buttons_and_save(button26_addr, 26);
					no_of_votes_cat8--;
				}
				else if((26 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("26G ");		// comment this out in final product
					vote_indicator_26 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button26_addr, 26);
					//i2c_send_byte(PIC_DEV_ADDR, 26);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0A) && (27 <= candidate_count) && vote_indicator_27==0)
			{
				if((27 > category_size_1) && (27 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("27C2 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat2--;
				}
				else if((27 > category_size_2) && (27 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("27C3 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat3--;
				}
				else if((27 > category_size_3) && (27 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("27C4 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat4--;
				}
				else if((27 > category_size_4) && (27 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("27C5 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat5--;
				}
				else if((27 > category_size_5) && (27 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("27C6 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat6--;
				}
				else if((27 > category_size_6) && (27 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("27C7 ");
					vote_indicator_27 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					deactivate_category_buttons_and_save(button27_addr, 27);
					no_of_votes_cat7--;
				}
				//else if((27 > category_size_7) && (27 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("27C8 ");
					//vote_indicator_27 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 27);
					//deactivate_category_buttons_and_save(button27_addr, 27);
					//no_of_votes_cat8--;
				//}
				else if((27 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("27G ");		// comment this out in final product
					vote_indicator_27 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button27_addr, 27);
					//i2c_send_byte(PIC_DEV_ADDR, 27);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0B) && (28 <= candidate_count) && vote_indicator_28==0)
			{
				if((28 > category_size_1) && (28 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("28C2 ");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat2--;
				}
				else if((28 > category_size_2) && (28 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("28C3 ");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat3--;
				}
				else if((28 > category_size_3) && (28 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("28C4 ");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat4--;
				}
				else if((28 > category_size_4) && (28 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("28C5 ");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat5--;
				}
				else if((28 > category_size_5) && (28 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("28C6");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat6--;
				}
				else if((28 > category_size_6) && (28 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("28C7");
					vote_indicator_28 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					deactivate_category_buttons_and_save(button28_addr, 28);
					no_of_votes_cat7--;
				}
				//else if((28 > category_size_7) && (28 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("28C8");
					//vote_indicator_28 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 28);
					//deactivate_category_buttons_and_save(button28_addr, 28);
					//no_of_votes_cat8--;
				//}
				else if((28 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("28G ");		// comment this out in final product
					vote_indicator_28 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button28_addr, 28);
					//i2c_send_byte(PIC_DEV_ADDR, 28);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0C) && (29 <= candidate_count) && vote_indicator_29 == 0)
			{
				if((29 > category_size_1) && (29 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("29C2 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat2--;
				}
				else if((29 > category_size_2) && (29 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("29C3 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat3--;
				}
				else if((29 > category_size_3) && (29 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("29C4 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat4--;
				}
				else if((29 > category_size_4) && (29 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("29C5 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat5--;
				}
				else if((29 > category_size_5) && (29 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("29C6 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat6--;
				}
				else if((29 > category_size_6) && (29 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("29C7 ");
					vote_indicator_29 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					deactivate_category_buttons_and_save(button29_addr, 29);
					no_of_votes_cat7--;
				}
				//else if((29 > category_size_7) && (29 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("29C8 ");
					//vote_indicator_29 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 29);
					//deactivate_category_buttons_and_save(button29_addr, 29);
					//no_of_votes_cat8--;
				//}
				else if((29 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("29G ");		// comment this out in final product
					vote_indicator_29 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button29_addr, 29);
					//i2c_send_byte(PIC_DEV_ADDR, 29);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0D) && (30 <= candidate_count) && vote_indicator_30==0)
			{
				if((30 > category_size_1) && (30 <= category_size_2) && (no_of_votes_cat2 != 0))
				{
					//USART_putstring("30C2 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat2--;
				}
				else if((30 > category_size_2) && (30 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("30C3 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat3--;
				}
				else if((30 > category_size_3) && (30 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("30C4 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat4--;
				}
				else if((30 > category_size_4) && (30 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("30C5 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat5--;
				}
				else if((30 > category_size_5) && (30 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("30C6 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat6--;
				}
				else if((30 > category_size_6) && (30 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("30C7 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat7--;
				}
				else if((30 > category_size_7) && (30 <= category_size_8) && (no_of_votes_cat8 != 0))
				{
					//USART_putstring("30C8 ");
					vote_indicator_30 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					deactivate_category_buttons_and_save(button30_addr, 30);
					no_of_votes_cat8--;
				}
				else if((30 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("30G ");		// comment this out in final product
					vote_indicator_30 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button30_addr, 30);
					//i2c_send_byte(PIC_DEV_ADDR, 30);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0E) && (31 <= candidate_count) && vote_indicator_31==0)
			{
				if((31 > category_size_2) && (31 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("31C3 ");
					vote_indicator_31 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					deactivate_category_buttons_and_save(button31_addr, 31);
					no_of_votes_cat3--;
				}
				if((31 > category_size_3) && (31 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("31C4 ");
					vote_indicator_31 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					deactivate_category_buttons_and_save(button31_addr, 31);
					no_of_votes_cat4--;
				}
				else if((31 > category_size_4) && (31 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("31C5 ");
					vote_indicator_31 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					deactivate_category_buttons_and_save(button31_addr, 31);
					no_of_votes_cat5--;
				}
				else if((31 > category_size_5) && (31 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("31C6 ");
					vote_indicator_31 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					deactivate_category_buttons_and_save(button31_addr, 31);
					no_of_votes_cat6--;
				}
				else if((31 > category_size_6) && (31 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("31C7 ");
					vote_indicator_31 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					deactivate_category_buttons_and_save(button31_addr, 31);
					no_of_votes_cat7--;
				}
				//else if((31 > category_size_7) && (31 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					//USART_putstring("31C8 ");
					//vote_indicator_31 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 31);
					//deactivate_category_buttons_and_save(button31_addr, 31);
					//no_of_votes_cat8--;
				//}
				else if((31 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("31G ");		// comment this out in final product
					vote_indicator_31 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button31_addr, 31);
					//i2c_send_byte(PIC_DEV_ADDR, 31);
					gen_vote_count--;
				}
			}
		
			if ((matrix_B_data == 1) && (matrix_A_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0F) && (32 <= candidate_count) && vote_indicator_32==0)
			{
				if((32 > category_size_2) && (32 <= category_size_3) && (no_of_votes_cat3 != 0))
				{
					//USART_putstring("32C3 ");
					vote_indicator_32 = 1;		// Indicating button 32 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					deactivate_category_buttons_and_save(button32_addr, 32);
					no_of_votes_cat3--;
				}
				if((32 > category_size_3) && (32 <= category_size_4) && (no_of_votes_cat4 != 0))
				{
					//USART_putstring("32C4 ");
					vote_indicator_32 = 1;		// Indicating button 32 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					deactivate_category_buttons_and_save(button32_addr, 32);
					no_of_votes_cat4--;
				}
				else if((32 > category_size_4) && (32 <= category_size_5) && (no_of_votes_cat5 != 0))
				{
					//USART_putstring("32 C 5");
					vote_indicator_32 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					deactivate_category_buttons_and_save(button32_addr, 32);
					no_of_votes_cat5--;
				}
				else if((32 > category_size_5) && (32 <= category_size_6) && (no_of_votes_cat6 != 0))
				{
					//USART_putstring("32C6 ");
					vote_indicator_32 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					deactivate_category_buttons_and_save(button32_addr, 32);
					no_of_votes_cat6--;
				}
				else if((32 > category_size_6) && (32 <= category_size_7) && (no_of_votes_cat7 != 0))
				{
					//USART_putstring("32C7 ");
					vote_indicator_32 = 1;	// Indicating button 12 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					deactivate_category_buttons_and_save(button32_addr, 32);
					no_of_votes_cat7--;
				}
				//else if((32 > category_size_7) && (32 <= category_size_8) && (no_of_votes_cat8 != 0))
				//{
					////USART_putstring("32C8 ");
					//vote_indicator_32 = 1;	// Indicating button 12 was presses.
					////i2c_send_byte(PIC_DEV_ADDR, 32);
					//deactivate_category_buttons_and_save(button32_addr, 32);
					//no_of_votes_cat8--;
				//}
				else if((32 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("32G ");		// comment this out in final product
					vote_indicator_32 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button32_addr, 32);
					//i2c_send_byte(PIC_DEV_ADDR, 32);
					gen_vote_count--;
				}
			}
	/************************************************  Half Mark  ***********************************************************************************/		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x00) && (33 <= candidate_count) && vote_indicator_33 == 0)
			{
				if((33 > category_size_2) && (33 <= category_size_3) && (no_of_votes_cat3 != 0x00))
				{
					//USART_putstring("33C3 ");
					vote_indicator_33 = 1;					// Indicating button 33 was presses.
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					deactivate_category_buttons_and_save(button33_addr, 33);
					no_of_votes_cat3--;
				}
				else if((33 > category_size_3) && (33 <= category_size_4) && (no_of_votes_cat4 != 0x00))				
				{
					//USART_putstring("33C4 ");			// comment this out in final product
					vote_indicator_33 = 1;					// Indicating button 33 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					deactivate_category_buttons_and_save(button33_addr, 33);  // provide starting and ending buttons as arguments
				}
				else if((33 > category_size_4) && (33 <= category_size_5) && (no_of_votes_cat5 != 0x00))				
				{
					//USART_putstring("33C5 ");				// comment this out in final product
					vote_indicator_33 = 1;					// Indicating button 33 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					deactivate_category_buttons_and_save(button33_addr, 33);  // provide starting and ending buttons as arguments
				}
				else if((33 > category_size_5) && (33 <= category_size_6) && (no_of_votes_cat6 != 0x00))				
				{	
					//USART_putstring("33C6 ");				// comment this out in final product
					vote_indicator_33 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					deactivate_category_buttons_and_save(button33_addr, 33);  // provide starting and ending buttons as arguments
				}
				else if((33 > category_size_6) && (33 <= category_size_7) && (no_of_votes_cat7 != 0x00))
				{
					//USART_putstring("33C7 ");				// comment this out in final product
					vote_indicator_33 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					deactivate_category_buttons_and_save(button33_addr, 33);  // provide starting and ending buttons as arguments
				}
				//else if((33 > category_size_7) && (33 <= category_size_8) && (no_of_votes_cat8 != 0x00))
				//{
					////USART_putstring("33C8 ");				// comment this out in final product
					//vote_indicator_33 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 33);
					//deactivate_category_buttons_and_save(button33_addr, 33);  // provide starting and ending buttons as arguments
				//}
				else if ((33 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("33 G ");		// comment this out in final product
					vote_indicator_33 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button33_addr, 33);
					//i2c_send_byte(PIC_DEV_ADDR, 33);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x01) && (34 <= candidate_count) && vote_indicator_34 == 0)
			{
				if((34 > category_size_2) && (34 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("34C3 ");				// comment this out in final product
					vote_indicator_34 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				}
				else if((34 > category_size_3) && (34 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("34C4 ");				// comment this out in final product
					vote_indicator_34 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				}
				else if((34 > category_size_4) && (34 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("34C5 ");				// comment this out in final product
					vote_indicator_34 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				}
				else if((34 > category_size_5) && (34 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("34C6 ");				// comment this out in final product
					vote_indicator_34 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				}
				else if((34 > category_size_6) && (34 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("34C7 ");				// comment this out in final product
					vote_indicator_34 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				}
				//else if((34 > category_size_7) && (34 <= category_size_8) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("34C8 ");				// comment this out in final product
					//vote_indicator_34 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 34);
					//deactivate_category_buttons_and_save(button34_addr, 34);  // provide starting and ending buttons as arguments
				//}
				else if ((34 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("34G ");		// comment this out in final product
					vote_indicator_34 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button34_addr, 34);
					//i2c_send_byte(PIC_DEV_ADDR, 34);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x02)  && (35 <= candidate_count) && vote_indicator_35 == 0)
			{
				if((35 > category_size_2) && (35 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("35C3 ");				// comment this out in final product
					vote_indicator_35 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				}
				else if((35 > category_size_3) && (35 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("35C4 ");				// comment this out in final product
					vote_indicator_35 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				}
				else if((35 > category_size_4) && (35 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("35C5 ");				// comment this out in final product
					vote_indicator_35 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				}
				else if((35 > category_size_5) && (35 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("35C6 ");				// comment this out in final product
					vote_indicator_35 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				}
				else if((35 > category_size_6) && (35 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("35C7 ");				// comment this out in final product
					vote_indicator_35 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				}
				//else if((35 > category_size_7) && (35 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("35C8 ");				// comment this out in final product
					//vote_indicator_35 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 35);
					//deactivate_category_buttons_and_save(button35_addr, 35);  // provide starting and ending buttons as arguments
				//}
				else if ((35 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("35G ");		// comment this out in final product
					vote_indicator_35 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button35_addr, 35);
					//i2c_send_byte(PIC_DEV_ADDR, 35);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x03)  && (36 <= candidate_count) && vote_indicator_36 == 0)
			{
				if((36 > category_size_2) && (36 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("36C3 ");				// comment this out in final product
					vote_indicator_36 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				}
				else if((36 > category_size_3) && (36 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("36C4 ");				// comment this out in final product
					vote_indicator_36 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				}
				else if((36 > category_size_4) && (36 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("36C5 ");				// comment this out in final product
					vote_indicator_36 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				}
				else if((36 > category_size_5) && (36 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("36C6 ");				// comment this out in final product
					vote_indicator_36 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				}
				else if((36 > category_size_6) && (36 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("36C7 ");				// comment this out in final product
					vote_indicator_36 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				}
				//else if((36 > category_size_7) && (36 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("36C8 ");				// comment this out in final product
					//vote_indicator_36 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 36);
					//deactivate_category_buttons_and_save(button36_addr, 36);  // provide starting and ending buttons as arguments
				//}
				else if ((36 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("36G ");		// comment this out in final product
					vote_indicator_36 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button36_addr, 36);
					//i2c_send_byte(PIC_DEV_ADDR, 36);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x04)  && (37 <= candidate_count) && vote_indicator_37 == 0)
			{
				if((37 > category_size_2) && (37 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("37C3 ");				// comment this out in final product
					vote_indicator_37 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				}
				else if((37 > category_size_3) && (37 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("37C4 ");				// comment this out in final product
					vote_indicator_37 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				}
				else if((37 > category_size_4) && (37 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("37C5 ");				// comment this out in final product
					vote_indicator_37 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				}
				else if((37 > category_size_5) && (37 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("37C6 ");				// comment this out in final product
					vote_indicator_37 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				}
				else if((37 > category_size_6) && (37 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("37C7 ");				// comment this out in final product
					vote_indicator_37 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				}
				//else if((37 > category_size_7) && (37 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("37C8 ");				// comment this out in final product
					//vote_indicator_37 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 37);
					//deactivate_category_buttons_and_save(button37_addr, 37);  // provide starting and ending buttons as arguments
				//}
				else if ((37 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("37G ");		// comment this out in final product
					vote_indicator_37 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button37_addr, 37);
					//i2c_send_byte(PIC_DEV_ADDR, 37);
					gen_vote_count--;
				}
			
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x05)  && (38 <= candidate_count) && vote_indicator_38 == 0)
			{
				if((38 > category_size_2) && (38 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("38C3 ");				// comment this out in final product
					vote_indicator_38 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				}
				else if((38 > category_size_3) && (38 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("38C4 ");				// comment this out in final product
					vote_indicator_38 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				}
				else if((38 > category_size_4) && (38 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("38C5 ");				// comment this out in final product
					vote_indicator_38 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				}
				else if((38 > category_size_5) && (38 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("38C6 ");				// comment this out in final product
					vote_indicator_38 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				}
				else if((38 > category_size_6) && (38 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("38C7 ");				// comment this out in final product
					vote_indicator_38 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				}
				//else if((38 > category_size_7) && (38 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("38C8 ");				// comment this out in final product
					//vote_indicator_38 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 38);
					//deactivate_category_buttons_and_save(button38_addr, 38);  // provide starting and ending buttons as arguments
				//}
				else if ((38 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("38G ");		// comment this out in final product
					vote_indicator_38 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button38_addr, 38);
					//i2c_send_byte(PIC_DEV_ADDR, 38);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x06)  && (39 <= candidate_count) && vote_indicator_39 == 0)
			{
				if((39 > category_size_2) && (39 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("39C3 ");				// comment this out in final product
					vote_indicator_39 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				}
				else if((39 > category_size_3) && (39 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("39C4 ");				// comment this out in final product
					vote_indicator_39 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				}
				else if((39 > category_size_4) && (39 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("39C5 ");				// comment this out in final product
					vote_indicator_39 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				}
				else if((39 > category_size_5) && (39 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("39C6 ");				// comment this out in final product
					vote_indicator_39 = 1;                  	// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				}
				else if((39 > category_size_6) && (39 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("39C7 ");				// comment this out in final product
					vote_indicator_39 = 1;                  	// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				}
				//else if((39 > category_size_7) && (39 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("39C8 ");				// comment this out in final product
					//vote_indicator_39 = 1;                  	// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 39);
					//deactivate_category_buttons_and_save(button39_addr, 39);  // provide starting and ending buttons as arguments
				//}
				else if ((39 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("39G ");		// comment this out in final product
					vote_indicator_39 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button39_addr, 39);
					//i2c_send_byte(PIC_DEV_ADDR, 39);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x07)  && (40 <= candidate_count) && vote_indicator_40 == 0)
			{
				if((40 > category_size_2) && (40 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("40C3 ");				// comment this out in final product
					vote_indicator_40 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				}
				else if((40 > category_size_3) && (40 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("40C4 ");				// comment this out in final product
					vote_indicator_40 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				}
				else if((40 > category_size_4) && (40 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("40C5 ");				// comment this out in final product
					vote_indicator_40 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				}
				else if((40 > category_size_5) && (40 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("40C6 ");				// comment this out in final product
					vote_indicator_40 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				}
				else if((40 > category_size_6) && (40 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("40C7 ");				// comment this out in final product
					vote_indicator_40 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				}
				//else if((40 > category_size_7) && (40 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("40C8 ");				// comment this out in final product
					//vote_indicator_40 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 40);
					//deactivate_category_buttons_and_save(button40_addr, 40);  // provide starting and ending buttons as arguments
				//}
				else if ((40 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("40G ");		// comment this out in final product
					vote_indicator_40 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button40_addr, 40);
					//i2c_send_byte(PIC_DEV_ADDR, 40);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x08)  && (41 <= candidate_count) && vote_indicator_41 == 0)
			{
				if((41 > category_size_2) && (41 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("41C3 ");				// comment this out in final product
					vote_indicator_41 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				}
				else if((41 > category_size_3) && (41 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("41C4 ");				// comment this out in final product
					vote_indicator_41 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				}
				else if((41 > category_size_4) && (41 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("41C5 ");				// comment this out in final product
					vote_indicator_41 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				}
				else if((41 > category_size_5) && (41 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("41C6 ");				// comment this out in final product
					vote_indicator_41 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				}
				else if((41 > category_size_6) && (41 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("41C7 ");				// comment this out in final product
					vote_indicator_41 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				}
				//else if((41 > category_size_7) && (41 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("41C8 ");				// comment this out in final product
					//vote_indicator_41 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 41);
					//deactivate_category_buttons_and_save(button41_addr, 41);  // provide starting and ending buttons as arguments
				//}
				else if ((41 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("41G ");		// comment this out in final product
					vote_indicator_41 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button41_addr, 41);
					//i2c_send_byte(PIC_DEV_ADDR, 41);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x09)  && (42 <= candidate_count) && vote_indicator_42 == 0)
			{
				if((42 > category_size_2) && (42 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("42C3 ");				// comment this out in final product
					vote_indicator_42 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				}
				else if((42 > category_size_3) && (42 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("42C4 ");				// comment this out in final product
					vote_indicator_42 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				}
				else if((42 > category_size_4) && (42 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("42C5 ");				// comment this out in final product
					vote_indicator_42 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				}
				else if((42 > category_size_5) && (42 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("42C6 ");				// comment this out in final product
					vote_indicator_42 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				}
				else if((42 > category_size_6) && (42 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("42C7 ");				// comment this out in final product
					vote_indicator_42 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				}
				//else if((42 > category_size_7) && (42 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("42C8 ");				// comment this out in final product
					//vote_indicator_42 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 42);
					//deactivate_category_buttons_and_save(button42_addr, 42);  // provide starting and ending buttons as arguments
				//}
				else if ((42 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("42G ");		// comment this out in final product
					vote_indicator_42 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button42_addr, 42);
					//i2c_send_byte(PIC_DEV_ADDR, 42);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0A)  && (43 <= candidate_count) && vote_indicator_43 == 0)
			{
				if((43 > category_size_2) && (43 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("43C3 ");				// comment this out in final product
					vote_indicator_43 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				}
				else if((43 > category_size_3) && (43 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("43C4 ");				// comment this out in final product
					vote_indicator_43 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				}
				else if((43 > category_size_4) && (43 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("43C5 ");				// comment this out in final product
					vote_indicator_43 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				}
				else if((43 > category_size_5) && (43 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("43C6 ");				// comment this out in final product
					vote_indicator_43 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				}
				else if((43 > category_size_6) && (43 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("43C7 ");				// comment this out in final product
					vote_indicator_43 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				}
				//else if((43 > category_size_7) && (43 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("43C8 ");				// comment this out in final product
					//vote_indicator_43 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 43);
					//deactivate_category_buttons_and_save(button43_addr, 43);  // provide starting and ending buttons as arguments
				//}
				else if ((43 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("43G ");		// comment this out in final product
					vote_indicator_43 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button43_addr, 43);
					//i2c_send_byte(PIC_DEV_ADDR, 43);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0B)  && (44 <= candidate_count) && vote_indicator_44 == 0)
			{
				if((44 > category_size_2) && (44 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("44C3 ");				// comment this out in final product
					vote_indicator_44 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				}
				else if((44 > category_size_3) && (44 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("44C4 ");				// comment this out in final product
					vote_indicator_44 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				}
				else if((44 > category_size_4) && (44 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("44C5 ");				// comment this out in final product
					vote_indicator_44 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				}
				else if((44 > category_size_5) && (44 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("44C6 ");				// comment this out in final product
					vote_indicator_44 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				}
				else if((44 > category_size_6) && (44 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("44C7 ");				// comment this out in final product
					vote_indicator_44 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				}
				//else if((44 > category_size_7) && (44 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("44C8 ");				// comment this out in final product
					//vote_indicator_44 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 44);
					//deactivate_category_buttons_and_save(button44_addr, 44);  // provide starting and ending buttons as arguments
				//}
				else if ((44 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("44G ");		// comment this out in final product
					vote_indicator_44 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button44_addr, 44);
					//i2c_send_byte(PIC_DEV_ADDR, 44);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0C)  && (45 <= candidate_count) && vote_indicator_45 == 0)
			{
				if((45 > category_size_2) && (45 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("45C3 ");				// comment this out in final product
					vote_indicator_45 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat3--;
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				}
				else if((45 > category_size_3) && (45 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("45C4 ");				// comment this out in final product
					vote_indicator_45 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				}
				else if((45 > category_size_4) && (45 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("45C5 ");				// comment this out in final product
					vote_indicator_45 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				}
				else if((45 > category_size_5) && (45 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("45C6 ");				// comment this out in final pro6uct
					vote_indicator_45 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				}
				else if((45 > category_size_6) && (45 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("45C7 ");				// comment this out in final pro6uct
					vote_indicator_45 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				}
				//else if((45 > category_size_7) && (45 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("45C8 ");				// comment this out in final pro6uct
					//vote_indicator_45 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 45);
					//deactivate_category_buttons_and_save(button45_addr, 45);  // provide starting and ending buttons as arguments
				//}
				else if ((45 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("45G ");		// comment this out in final product
					vote_indicator_45 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button45_addr, 45);
					//i2c_send_byte(PIC_DEV_ADDR, 45);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0D) && (46 <= candidate_count) && vote_indicator_46 == 0)
			{
				//if((46 > category_size_2) && (46 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("46 C 3");				// comment this out in final product
					//vote_indicator_46 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 46);
					//deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				//}
				if((46 > category_size_3) && (46 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("46C4 ");				// comment this out in final product
					vote_indicator_46 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 46);
					deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				}
				else if((46 > category_size_4) && (46 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("46C5 ");				// comment this out in final product
					vote_indicator_46 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 46);
					deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				}
				else if((46 > category_size_5) && (46 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("46C6 ");				// comment this out in final product
					vote_indicator_46 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 46);
					deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				}
				else if((46 > category_size_6) && (46 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("46C7 ");				// comment this out in final product
					vote_indicator_46 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 46);
					deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				}
				//else if((46 > category_size_7) && (46 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("46C8 ");				// comment this out in final product
					//vote_indicator_46 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 46);
					//deactivate_category_buttons_and_save(button46_addr, 46);  // provide starting and ending buttons as arguments
				//}
				else if ((46 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("46G ");		// comment this out in final product
					vote_indicator_46 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button46_addr, 46);
					//i2c_send_byte(PIC_DEV_ADDR, 46);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0E) && (47 <= candidate_count) && vote_indicator_47 == 0)
			{
				//if((47 > category_size_2) && (47 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("47 C 3");				// comment this out in final product
					//vote_indicator_47 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 47);
					//deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				//}
				if((47 > category_size_3) && (47 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("47C4 ");				// comment this out in final product
					vote_indicator_47 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 47);
					deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				}
				else if((47 > category_size_4) && (47 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("47C5 ");				// comment this out in final product
					vote_indicator_47 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 47);
					deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				}
				else if((47 > category_size_5) && (47 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("47C6 ");				// comment this out in final product
					vote_indicator_47 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 47);
					deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				}
				else if((47 > category_size_6) && (47 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("47C7 ");				// comment this out in final product
					vote_indicator_47 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 47);
					deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				}
				//else if((47 > category_size_7) && (47 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("47C8 ");				// comment this out in final product
					//vote_indicator_47 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 47);
					//deactivate_category_buttons_and_save(button47_addr, 47);  // provide starting and ending buttons as arguments
				//}
				else if ((47 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("47G ");		// comment this out in final product
					vote_indicator_47 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button47_addr, 47);
					//i2c_send_byte(PIC_DEV_ADDR, 47);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 1) && (matrix_D_data == 0) && ((PINC & 0x0F) == 0x0F) && (48 <= candidate_count) && vote_indicator_48 == 0)
			{
				//if((48 > category_size_2) && (48 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("48 C 3");				// comment this out in final product
					//vote_indicator_48 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 48);
					//deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				//}
				if((48 > category_size_3) && (48 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("48C4 ");				// comment this out in final product
					vote_indicator_48 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 48);
					deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				}
				else if((48 > category_size_4) && (48 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("48C5 ");				// comment this out in final product
					vote_indicator_48 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 48);
					deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				}
				else if((48 > category_size_5) && (48 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("48C6 ");				// comment this out in final product
					vote_indicator_48 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 48);
					deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				}
				else if((48 > category_size_6) && (48 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("48C7 ");				// comment this out in final product
					vote_indicator_48 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 48);
					deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				}
				//else if((48 > category_size_7) && (48 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("48C8 ");				// comment this out in final product
					//vote_indicator_48 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 48);
					//deactivate_category_buttons_and_save(button48_addr, 48);  // provide starting and ending buttons as arguments
				//}
				else if ((48 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("48G ");		// comment this out in final product
					vote_indicator_48 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button48_addr, 48);
					//i2c_send_byte(PIC_DEV_ADDR, 48);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x00) && (49 <= candidate_count) && vote_indicator_49 == 0)
			{
				//if((49 > category_size_2) && (49 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("49 C 3");				// comment this out in final product
					//vote_indicator_49 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 49);
					//deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				//}
				if((49 > category_size_3) && (49 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("49C4 ");				// comment this out in final product
					vote_indicator_49 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 49);
					deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				}
				else if((49 > category_size_4) && (49 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("49C5 ");				// comment this out in final product
					vote_indicator_49 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 49);
					deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				}
				else if((49 > category_size_5) && (49 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("49C6 ");				// comment this out in final product
					vote_indicator_49 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 49);
					deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				}
				else if((49 > category_size_6) && (49 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("49C7 ");				// comment this out in final product
					vote_indicator_49 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 49);
					deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				}
				//else if((49 > category_size_7) && (49 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("49C8 ");				// comment this out in final product
					//vote_indicator_49 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 49);
					//deactivate_category_buttons_and_save(button49_addr, 49);  // provide starting and ending buttons as arguments
				//}
				else if ((49 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					////USART_putstring("49 G ");		// comment this out in final product
					vote_indicator_49 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button49_addr, 49);
					//i2c_send_byte(PIC_DEV_ADDR, 49);
					gen_vote_count--;
				}
			}
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x01) && (50 <= candidate_count) && vote_indicator_50 == 0)
			{
				//if((50 > category_size_2) && (50 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("50 C 3");				// comment this out in final product
					//vote_indicator_50 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 50);
					//deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				//}
				if((50 > category_size_3) && (50 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("50C4 ");				// comment this out in final product
					vote_indicator_50 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 50);
					deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				}
				else if((50 > category_size_4) && (50 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("50C5 ");				// comment this out in final product
					vote_indicator_50 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 50);
					deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				}
				else if((50 > category_size_5) && (50 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("50C6 ");				// comment this out in final product
					vote_indicator_50 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 50);
					deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				}
				else if((50 > category_size_6) && (50 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("50C7 ");				// comment this out in final product
					vote_indicator_50 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 50);
					deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				}
				//else if((50 > category_size_7) && (50 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("50C8 ");				// comment this out in final product
					//vote_indicator_50 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 50);
					//deactivate_category_buttons_and_save(button50_addr, 50);  // provide starting and ending buttons as arguments
				//}
				else if ((50 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("50G ");		// comment this out in final product
					vote_indicator_50 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button50_addr, 50);
					//i2c_send_byte(PIC_DEV_ADDR, 50);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x02) && (51 <= candidate_count) && vote_indicator_51 == 0)
			{
				//if((51 > category_size_2) && (51 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("51 C 3");				// comment this out in final product
					//vote_indicator_51 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 51);
					//deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				//}
				if((51 > category_size_3) && (51 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("51C4 ");				// comment this out in final product
					vote_indicator_51 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 51);
					deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				}
				else if((51 > category_size_4) && (51 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("51C5 ");				// comment this out in final product
					vote_indicator_51 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 51);
					deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				}
				else if((51 > category_size_5) && (51 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("51C6 ");				// comment this out in final product
					vote_indicator_51 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 51);
					deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				}
				else if((51 > category_size_6) && (51 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("51C7 ");				// comment this out in final product
					vote_indicator_51 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 51);
					deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				}
				//else if((51 > category_size_7) && (51 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("51C8 ");				// comment this out in final product
					//vote_indicator_51 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 51);
					//deactivate_category_buttons_and_save(button51_addr, 51);  // provide starting and ending buttons as arguments
				//}
				else if ((51 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("51G ");		// comment this out in final product
					vote_indicator_51 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button51_addr, 51);
					//i2c_send_byte(PIC_DEV_ADDR, 51);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x03) && (52 <= candidate_count) && vote_indicator_52 == 0)
			{
				//if((52 > category_size_2) && (52 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("52 C 3");				// comment this out in final product
					//vote_indicator_52 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 52);
					//deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				//}
				if((52 > category_size_3) && (52 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("52C4 ");				// comment this out in final product
					vote_indicator_52 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 52);
					deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				}
				else if((52 > category_size_4) && (52 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("52C5 ");				// comment this out in final product
					vote_indicator_52 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 52);
					deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				}
				else if((52 > category_size_5) && (52 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("52C6 ");				// comment this out in final product
					vote_indicator_52 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 52);
					deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				}
				else if((52 > category_size_6) && (52 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("52C7 ");				// comment this out in final product
					vote_indicator_52 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 52);
					deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				}
				//else if((52 > category_size_7) && (52 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("52C8 ");				// comment this out in final product
					//vote_indicator_52 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 52);
					//deactivate_category_buttons_and_save(button52_addr, 52);  // provide starting and ending buttons as arguments
				//}
				else if ((52 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("52G ");		// comment this out in final product
					vote_indicator_52 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button52_addr, 52);
					//i2c_send_byte(PIC_DEV_ADDR, 52);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x04) && (53 <= candidate_count) && vote_indicator_53 == 0)
			{
				//if((53 > category_size_2) && (53 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("53 C 3");				// comment this out in final product
					//vote_indicator_53 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 53);
					//deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				//}
				if((53 > category_size_3) && (53 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("53C4 ");				// comment this out in final product
					vote_indicator_53 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 53);
					deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				}
				else if((53 > category_size_4) && (53 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("53C5 ");				// comment this out in final product
					vote_indicator_53 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 53);
					deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				}
				else if((53 > category_size_5) && (53 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("53C6 ");				// comment this out in final product
					vote_indicator_53 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 53);
					deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				}
				else if((53 > category_size_6) && (53 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("53C7 ");				// comment this out in final product
					vote_indicator_53 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 53);
					deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				}
				//else if((53 > category_size_7) && (53 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("53C8 ");				// comment this out in final product
					//vote_indicator_53 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 53);
					//deactivate_category_buttons_and_save(button53_addr, 53);  // provide starting and ending buttons as arguments
				//}
				else if ((53 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("53G ");		// comment this out in final product
					vote_indicator_53 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button53_addr, 53);
					//i2c_send_byte(PIC_DEV_ADDR, 53);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x05) && (54 <= candidate_count) && vote_indicator_54 == 0)
			{
				//if((54 > category_size_2) && (54 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("54 C 3");				// comment this out in final product
					//vote_indicator_54 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 54);
					//deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				//}
				if((54 > category_size_3) && (54 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("54C4 ");				// comment this out in final product
					vote_indicator_54 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 54);
					deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				}
				else if((54 > category_size_4) && (54 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("54C5 ");				// comment this out in final product
					vote_indicator_54 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 54);
					deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				}
				else if((54 > category_size_5) && (54 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("54C6 ");				// comment this out in final product
					vote_indicator_54 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 54);
					deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				}
				else if((54 > category_size_6) && (54 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("54C7 ");				// comment this out in final product
					vote_indicator_54 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 54);
					deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				}
				//else if((54 > category_size_7) && (54 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("54C8 ");				// comment this out in final product
					//vote_indicator_54 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 54);
					//deactivate_category_buttons_and_save(button54_addr, 54);  // provide starting and ending buttons as arguments
				//}
				else if ((54 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("54G ");		// comment this out in final product
					vote_indicator_54 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button54_addr, 54);
					//i2c_send_byte(PIC_DEV_ADDR, 54);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x06) && (55 <= candidate_count) && vote_indicator_55 == 0)
			{
				//if((55 > category_size_2) && (55 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("55 C 3");				// comment this out in final product
					//vote_indicator_55 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 55);
					//deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				//}
				if((55 > category_size_3) && (55 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("55C4 ");				// comment this out in final product
					vote_indicator_55 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 55);
					deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				}
				else if((55 > category_size_4) && (55 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("55C5 ");				// comment this out in final product
					vote_indicator_55 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 55);
					deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				}
				else if((55 > category_size_5) && (55 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("55C6 ");				// comment this out in final product
					vote_indicator_55 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 55);
					deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				}
				else if((55 > category_size_6) && (55 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("55C7 ");				// comment this out in final product
					vote_indicator_55 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 55);
					deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				}
				//else if((55 > category_size_7) && (55 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					//USART_putstring("55C8 ");				// comment this out in final product
					//vote_indicator_55 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 55);
					//deactivate_category_buttons_and_save(button55_addr, 55);  // provide starting and ending buttons as arguments
				//}
				else if ((55 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("55G ");		// comment this out in final product
					vote_indicator_55 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button55_addr, 55);
					//i2c_send_byte(PIC_DEV_ADDR, 55);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x07) && (56 <= candidate_count) && vote_indicator_56 == 0)
			{
				//if((56 > category_size_2) && (56 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("56 C 3");				// comment this out in final product
					//vote_indicator_56 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 56);
					//deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				//}
				if((56 > category_size_3) && (56 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("56C4 ");				// comment this out in final product
					vote_indicator_56 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 56);
					deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				}
				else if((56 > category_size_4) && (56 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("56C5 ");				// comment this out in final product
					vote_indicator_56 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 56);
					deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				}
				else if((56 > category_size_5) && (56 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("56C6 ");				// comment this out in final product
					vote_indicator_56 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 56);
					deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				}
				else if((56 > category_size_6) && (56 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("56C7 ");				// comment this out in final product
					vote_indicator_56 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 56);
					deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				}
				//else if((56 > category_size_7) && (56 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("56C8 ");				// comment this out in final product
					//vote_indicator_56 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 56);
					//deactivate_category_buttons_and_save(button56_addr, 56);  // provide starting and ending buttons as arguments
				//}
				else if ((56 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("56G ");		// comment this out in final product
					vote_indicator_56 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button56_addr, 56);
					//i2c_send_byte(PIC_DEV_ADDR, 56);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x08) && (57 <= candidate_count) && vote_indicator_57 == 0)
			{
				//if((57 > category_size_2) && (57 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("57 C 3");				// comment this out in final product
					//vote_indicator_57 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 57);
					//deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				//}
				if((57 > category_size_3) && (57 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("57C4 ");				// comment this out in final product
					vote_indicator_57 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 57);
					deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				}
				else if((57 > category_size_4) && (57 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("57C5 ");				// comment this out in final product
					vote_indicator_57 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 57);
					deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				}
				else if((57 > category_size_5) && (57 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("57C6 ");				// comment this out in final product
					vote_indicator_57 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 57);
					deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				}
				else if((57 > category_size_6) && (57 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("57C7 ");				// comment this out in final product
					vote_indicator_57 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 57);
					deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				}
				//else if((57 > category_size_7) && (57 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("57C8 ");				// comment this out in final product
					//vote_indicator_57 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 57);
					//deactivate_category_buttons_and_save(button57_addr, 57);  // provide starting and ending buttons as arguments
				//}
				else if ((57 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("57G ");		// comment this out in final product
					vote_indicator_57 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button57_addr, 57);
					//i2c_send_byte(PIC_DEV_ADDR, 57);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x09) && (58 <= candidate_count) && vote_indicator_58 == 0)
			{
				//if((58 > category_size_2) && (58 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("58 C 3");				// comment this out in final product
					//vote_indicator_58 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 58);
					//deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				//}
				if((58 > category_size_3) && (58 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("58C4 ");				// comment this out in final product
					vote_indicator_58 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 58);
					deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				}
				else if((58 > category_size_4) && (58 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("58C5 ");				// comment this out in final product
					vote_indicator_58 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 58);
					deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				}
				else if((58 > category_size_5) && (58 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("58C6 ");				// comment this out in final product
					vote_indicator_58 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 58);
					deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				}
				else if((58 > category_size_6) && (58 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("58C7 ");				// comment this out in final product
					vote_indicator_58 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 58);
					deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				}
				//else if((58 > category_size_7) && (58 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("58C8 ");				// comment this out in final product
					//vote_indicator_58 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 58);
					//deactivate_category_buttons_and_save(button58_addr, 58);  // provide starting and ending buttons as arguments
				//}
				else if ((58 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("58G ");		// comment this out in final product
					vote_indicator_58 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button58_addr, 58);
					//i2c_send_byte(PIC_DEV_ADDR, 58);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0A) && (59 <= candidate_count) && vote_indicator_59 == 0)
			{
				//if((59 > category_size_2) && (59 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("59 C 3");				// comment this out in final product
					//vote_indicator_59 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 59);
					//deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				//}
				if((59 > category_size_3) && (59 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("59C4 ");				// comment this out in final product
					vote_indicator_59 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 59);
					deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				}
				else if((59 > category_size_4) && (59 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("59C5 ");				// comment this out in final product
					vote_indicator_59 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 59);
					deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				}
				else if((59 > category_size_5) && (59 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("59C6 ");				// comment this out in final product
					vote_indicator_59 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 59);
					deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				}
				else if((59 > category_size_6) && (59 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("59C7 ");				// comment this out in final product
					vote_indicator_59 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 59);
					deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				}
				//else if((59 > category_size_8) && (59 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("59C8 ");				// comment this out in final product
					//vote_indicator_59 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 59);
					//deactivate_category_buttons_and_save(button59_addr, 59);  // provide starting and ending buttons as arguments
				//}
				else if ((59 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("59G ");		// comment this out in final product
					vote_indicator_59 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button59_addr, 59);
					//i2c_send_byte(PIC_DEV_ADDR, 59);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0B) && (60 <= candidate_count) && vote_indicator_60 == 0)
			{
				//if((60 > category_size_2) && (60 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("60 C 3");				// comment this out in final product
					//vote_indicator_60 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 60);
					//deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				//}
				if((60 > category_size_3) && (60 <= category_size_4) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("60C4 ");				// comment this out in final product
					vote_indicator_60 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat4--;
					//i2c_send_byte(PIC_DEV_ADDR, 60);
					deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				}
				else if((60 > category_size_4) && (60 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("60C5 ");				// comment this out in final product
					vote_indicator_60 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 60);
					deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				}
				else if((60 > category_size_5) && (60 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("60C6 ");				// comment this out in final product
					vote_indicator_60 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 60);
					deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				}
				else if((60 > category_size_6) && (60 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("60C7 ");				// comment this out in final product
					vote_indicator_60 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 60);
					deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				}
				//else if((60 > category_size_7) && (60 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("60C8 ");				// comment this out in final product
					//vote_indicator_60 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 60);
					//deactivate_category_buttons_and_save(button60_addr, 60);  // provide starting and ending buttons as arguments
				//}
				else if ((60 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("60G ");		// comment this out in final product
					vote_indicator_60 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button60_addr, 60);
					//i2c_send_byte(PIC_DEV_ADDR, 60);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0C) && (61 <= candidate_count) && vote_indicator_61 == 0)
			{
				//if((61 > category_size_2) && (61 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("61 C 3");				// comment this out in final product
					//vote_indicator_61 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 61);
					//deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				//}
				//else if((61 > category_size_4) && (61 <= category_size_5) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("61 C 4");				// comment this out in final product
					//vote_indicator_61 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat4--;
					////i2c_send_byte(PIC_DEV_ADDR, 61);
					//deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				//}
				if((61 > category_size_4) && (61 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("61C5 ");				// comment this out in final product
					vote_indicator_61 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 61);
					deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				}
				else if((61 > category_size_5) && (61 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("61C6 ");				// comment this out in final product
					vote_indicator_61 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 61);
					deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				}
				else if((61 > category_size_6) && (61 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("61C7 ");				// comment this out in final product
					vote_indicator_61 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 61);
					deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				}
				//else if((61 > category_size_7) && (61 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("61C8 ");				// comment this out in final product
					//vote_indicator_61 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 61);
					//deactivate_category_buttons_and_save(button61_addr, 61);  // provide starting and ending buttons as arguments
				//}
				else if ((61 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("61G ");		// comment this out in final product
					vote_indicator_61 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button61_addr, 61);
					//i2c_send_byte(PIC_DEV_ADDR, 61);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0D) && (62 <= candidate_count) && vote_indicator_62 == 0)
			{
				//if((62 > category_size_2) && (62 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("62 C 3");				// comment this out in final product
					//vote_indicator_62 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 62);
					//deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				//}
				//if((62 > category_size_4) && (62 <= category_size_5) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("62 C 4");				// comment this out in final product
					//vote_indicator_62 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat4--;
					////i2c_send_byte(PIC_DEV_ADDR, 62);
					//deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				//}
				if((62 > category_size_4) && (62 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("62C5 ");				// comment this out in final product
					vote_indicator_62 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 62);
					deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				}
				else if((62 > category_size_5) && (62 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("62C6 ");				// comment this out in final product
					vote_indicator_62 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 62);
					deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				}
				else if((62 > category_size_6) && (62 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("62C7 ");				// comment this out in final product
					vote_indicator_62 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 62);
					deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				}
				//else if((62 > category_size_7) && (62 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("62C8 ");				// comment this out in final product
					//vote_indicator_62 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 62);
					//deactivate_category_buttons_and_save(button62_addr, 62);  // provide starting and ending buttons as arguments
				//}
				else if ((62 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("62G ");		// comment this out in final product
					vote_indicator_62 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button62_addr, 62);
					//i2c_send_byte(PIC_DEV_ADDR, 62);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0E) && (63 <= candidate_count) && vote_indicator_63 == 0)
			{
				//if((63 > category_size_2) && (63 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("63 C 3");				// comment this out in final product
					//vote_indicator_63 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 63);
					//deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				//}
				//if((63 > category_size_4) && (63 <= category_size_5) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("63 C 4");				// comment this out in final product
					//vote_indicator_63 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat4--;
					////i2c_send_byte(PIC_DEV_ADDR, 63);
					//deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				//}
				if((63 > category_size_4) && (63 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("63C5 ");				// comment this out in final product
					vote_indicator_63 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 63);
					deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				}
				else if((63 > category_size_5) && (63 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("63C6 ");				// comment this out in final product
					vote_indicator_63 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 63);
					deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				}
				else if((63 > category_size_6) && (63 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("63C7 ");				// comment this out in final product
					vote_indicator_63 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 63);
					deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				}
				//else if((63 > category_size_7) && (63 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("63C8 ");				// comment this out in final product
					//vote_indicator_63 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 63);
					//deactivate_category_buttons_and_save(button63_addr, 63);  // provide starting and ending buttons as arguments
				//}
				else if ((63 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("63G ");		// comment this out in final product
					vote_indicator_63 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button63_addr, 63);
					//i2c_send_byte(PIC_DEV_ADDR, 63);
					gen_vote_count--;
				}
			}
		
			if ((matrix_A_data == 0) && (matrix_B_data == 0) && (matrix_C_data == 0) && (matrix_D_data == 1) && ((PINC & 0x0F) == 0x0F) && (64 <= candidate_count) && vote_indicator_64 == 0)
			{
				//if((64 > category_size_2) && (64 <= category_size_3) && (no_of_votes_cat3 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("64 C 3");				// comment this out in final product
					//vote_indicator_64 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat3--;
					////i2c_send_byte(PIC_DEV_ADDR, 64);
					//deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				//}
				//if((64 > category_size_4) && (64 <= category_size_5) && (no_of_votes_cat4 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("64 C 4");				// comment this out in final product
					//vote_indicator_64 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat4--;
					////i2c_send_byte(PIC_DEV_ADDR, 64);
					//deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				//}
				if((64 > category_size_4) && (64 <= category_size_5) && (no_of_votes_cat5 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("64C5 ");				// comment this out in final product
					vote_indicator_64 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat5--;
					//i2c_send_byte(PIC_DEV_ADDR, 64);
					deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				}
				else if((64 > category_size_5) && (64 <= category_size_6) && (no_of_votes_cat6 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("64C6 ");				// comment this out in final product
					vote_indicator_64 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat6--;
					//i2c_send_byte(PIC_DEV_ADDR, 64);
					deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				}
				else if((64 > category_size_6) && (64 <= category_size_7) && (no_of_votes_cat7 != 0x00))				// Mask out the MSB nibble.
				{
					//USART_putstring("64C7 ");				// comment this out in final product
					vote_indicator_64 = 1;					// Indicating button 1 was pressed
					no_of_votes_cat7--;
					//i2c_send_byte(PIC_DEV_ADDR, 64);
					deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				}
				//else if((64 > category_size_7) && (64 <= category_size_8) && (no_of_votes_cat8 != 0x00))				// Mask out the MSB nibble.
				//{
					////USART_putstring("64C8 ");				// comment this out in final product
					//vote_indicator_64 = 1;					// Indicating button 1 was pressed
					//no_of_votes_cat8--;
					////i2c_send_byte(PIC_DEV_ADDR, 64);
					//deactivate_category_buttons_and_save(button64_addr, 64);  // provide starting and ending buttons as arguments
				//}
				else if ((64 > no_of_buttons_in_categories) && (gen_vote_count != 0))
				{
					//USART_putstring("64G ");		// comment this out in final product
					vote_indicator_64 = 1;			// Indicating button 1 was pressed
					deactivate_category_buttons_and_save(button64_addr, 64);
					//i2c_send_byte(PIC_DEV_ADDR, 64);
					gen_vote_count--;
				}
			}
		
			else if(INIT == 0)
			{
				while(INIT == 0);
				i2c_send_byte(PIC_DEV_ADDR, 0);		// Clear the LED's
				//votes_counter=0;
				return;
			}
		}	
	}
}	
	
void vote_routine()                       
{
if (cod_typ == 'B')
	{	LED_INDICATOR=0;
		vote_registration();
		LED_INDICATOR=1;
		voting_on = 0;
		eeprom_write_i2c(EEPROM_DEV_ADDR, flag_register_bkp, flag_register);
		lcd_cmd(0x01);			 //clear lcd
		_delay_ms(1);
		lcd_cmd(0x80);			  
		_delay_ms(1);
		LED = 1;
		Buzzer = 0;
	}	

else             // Compulsory all votes alloted to voter have to be cast for the votes to be saved
	{
	
		char i;
		vote_cast = 0;
		LED_INDICATOR=0;
		vote_registration();
		LED_INDICATOR=1;
		voting_on = 0;
		eeprom_write_i2c(EEPROM_DEV_ADDR, flag_register_bkp, flag_register);
		lcd_cmd(0x01);			 //clear lcd
		_delay_ms(65);
		LED = 1;
		Buzzer = 0;
	
		USART_Transmit_dec(elem_count);
		if ((votes_counter == 0) && (vote_cast))
		{
			lcd_data_str("Saving Votes");
			voter_count++;
			i2c_write_2_bytes(EEPROM_DEV_ADDR, voters_voted, voter_count);	// Store the Count of no. of voters in EEPROM.
			_delay_ms(1);
			for(i=0; i < elem_count; i++)
			{
				i2c_read_2_bytes(voted_button_array[i]);
				i2c_data_16 = i2c_data_16 + 1;						// Increment the value by 1
				i2c_write_2_bytes(EEPROM_DEV_ADDR, voted_button_array[i], i2c_data_16);		// Save the Incremented value back to the same location
				_delay_ms(1);
			}
		}
		
		if (votes_counter > 0 && votes_counter < voter_votes) 
		{
			invalid_voters++;
			i2c_write_2_bytes(EEPROM_DEV_ADDR, voters_not_voted, invalid_voters); 
		}
		
	}	
}

//void send_results_pc()
//{
	//unsigned char i;
		//if(voter_votes > 1)			// Requirement for PC Application. If a voter is to vote more than one candidate send 'M' before device ID else send 'S'
		//USART_SendByte('M');		
		//if(voter_votes == 1)
		//USART_SendByte('S');
	//
	//USART_putstring(DEV_ID);		// Transmitting the device ID
		//
	//i2c_start();
	//i2c_send(0xA0);
	//i2c_send(0x00);
	//i2c_send(0x00);
	//i2c_stop();
	//
	//_delay_ms(2);
	//
	//i2c_start();
	//i2c_send(0xA1);
	//for(i=0; i <= 127; i++)			// Transmitting all 128 bytes (64 X 2bytes) of the vote_count info from the EEPROM to PC
	//{
		//i2c_receive(1);
		//USART_SendByte(i2c_data);		
	//}	
	//i2c_receive(0);
	//i2c_stop();
	//USART_SendByte('@');			// End of transmission indication.
//}
void send_results_pc()
{
	unsigned char i;

/**********************************	17.09.2015	**********************************************/
	//if(voter_votes > 1)			// Requirement for PC Application. If a voter is to vote more than one candidate send 'M' before device ID else send 'S'
	//USART_SendByte('M');
	//if(voter_votes == 1)
	//USART_SendByte('S');			// Commented out 17.09.15 due no requiremenyt now
	//USART_SendByte(mach_typ);		// Send type of machine info Donor or general
	//USART_SendByte('?');
	//USART_SendByte(location_ID);
	//USART_Transmit_dec(location_ID_number);
	//USART_SendByte('?');
	USART_SendByte(i2c_read_byte(EEPROM_DEV_ADDR, CODE_TYPE));		// Send type of code
	USART_SendByte('?');
	USART_SendByte(i2c_read_byte(EEPROM_DEV_ADDR, MACHINE_TYPE));		// Send type of machine info Donor or general
	USART_SendByte('?');
	USART_SendByte(i2c_read_byte(EEPROM_DEV_ADDR, LOCATION_ID_NAME));
	USART_Transmit_dec(i2c_read_byte(EEPROM_DEV_ADDR, LOCATION_ID_NO));
	USART_SendByte('?');
	
/**********************************	17.09.2015	**********************************************/	
	
	USART_putstring(DEV_ID);		// Transmitting the device ID
	USART_SendByte('?');
/******************************************18.09.15*************************************************/
	
USART_Transmit_dec(voter_count);//NO OF VOTERS VOTED
/*****************************************************************************************************/	
/******************************************18.09.15*************************************************/
USART_SendByte('?');
USART_Transmit_dec(invalid_voters);//NO OF VOTERS NOT VOTED

/*****************************************************************************************************/

	for(i=0; i <= 127; i++)
	{
		USART_SendByte('?');
		i2c_read_2_bytes(i);
		
		USART_Transmit_dec(i2c_data_16);
		i++;
	}
	USART_SendByte('@');	  // End of transmission indication.
}

void disp_result_LCD()				// This function will Display the stored vote_count on the LCD on every press of the RESULT button.
{
	unsigned int disp_count,count;
	disp_count = 0;
	count=1;
	while(disp_count < candidate_count*2)	//CHANGED HERE BY NEERAJ ON 21/03/2016 TO DISPLAY RESULTS OF ONLY CONFIGURED NO. OF CANDIDATES
	{										// disp_count is actually the address. So after address is incremented by 6 for 3 candidate count display next 3 values on LCD
		while (RESULT);
		lcd_cmd(0x01);
		lcd_cmd(0x82);								// shift cursor to 2nd line
		for (unsigned char i = 0; i <= 2; i++)		// read total votes of 3 candidates
		{	if (i == 0)
			{
				lcd_data_int1(count);
				lcd_cmd(0xC1);
				i2c_read_2_bytes(disp_count);			// send address of data
				lcd_data_int(i2c_data_16);				// display value on LCD
			}				
			if (i == 1)
			{
				lcd_cmd(0x87);
				lcd_data_int1(count);
				lcd_cmd(0xC6);
				i2c_read_2_bytes(disp_count);			// send address of data
				lcd_data_int(i2c_data_16);				// display value on LCD
			}
			if (i == 2)
			{
				lcd_cmd(0x8C);
				lcd_data_int1(count);
				lcd_cmd(0xCB);
				i2c_read_2_bytes(disp_count);			// send address of data
				lcd_data_int(i2c_data_16);				// display value on LCD
			}
			
			if (disp_count == 126)				//added by neeraj on 01/08/2016 So that it will display result upto 64 only
			{
				i=3;	// to exit the for loop
			}
			disp_count = disp_count + 2;			// increment address count by two. Because one candidate count is stored in 2 bytes
			count=count+1;
			if(count > candidate_count ) //CHANGED HERE BY NEERAJ ON 21/03/2016 TO DISPLAY RESULTS OF ONLY CONFIGURED NO. OF CANDIDATES
			{
				i = 3;					// to exit the for loop
				disp_count=candidate_count*2;//to exit while loop
			}
		}
		while(!RESULT);
		_delay_ms(500);
		
	}
	while(RESULT);
	_delay_ms(800);
	lcd_cmd(0X01);
}

/******* Added 23.04.2017	******/
void i2cWriteString(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen)
{
	do
	{
		i2c_start();
		i2c_send(device_id);
		if(slave_ack == 1)				// Stop I2C if no acknowledge
		i2c_stop();
	}
	while(slave_ack == 1);				//loop until acknowledge is received
	i2c_send(locAddr / 256);
	i2c_send(locAddr % 256);

	for(uint8_t i = 1; i <= stringLen; i++)
	{
		i2c_send(*stringData);
		stringData++;
	}
	i2c_stop();
}

void i2cReadString(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen)
{
	i2c_start();
	i2c_send(device_id);
	i2c_send(locAddr / 256);	// Sending the 16 bit address in two 8 bits
	i2c_send(locAddr % 256);
	i2c_stop();
	_delay_ms(1);
	i2c_start();
	i2c_send(device_id + 1);

	USART_Transmit_dec(stringLen);

	for(uint8_t i = 1; i <= stringLen; i++)
	{
		i2c_receive(1);
		*stringData = i2c_data;
		stringData++;

	}
	// i2c_receive(0);
	i2c_stop();
}

uint16_t i2cSaveName(uint8_t device_id, uint16_t locAddr, uint8_t *stringData, uint8_t stringLen)		// function returns the last address @ which data was saved.
{
	uint16_t eepromStringAddrHolder;

	eepromStringAddrHolder = locAddr;	// initial load with storing start address
	eepromStringAddrHolder += stringLen;
	i2cWriteString(device_id, locAddr, &stringData, stringLen);
	return eepromStringAddrHolder;
}

/************* Added 23.04.2017 	****************/



int main(void)
{
		unsigned char i, error;
		unsigned char fileName[13];
		PORT_pins_init();				// Initialize PORT pins to desired direction and values
		USART_Init();					// Initialize USART to set baud rate
		i2c_init();						// Initializing I2C registers of ATmega32
		//spi_init();
		sei();							// Enable global interrupt bit.
		
		lcd_cmd(0x38);			 		// Initialize 16x2 LCD for 5*7 matrix display
		lcd_cmd(0x0C);					// LCD ON ,Cursor ON
		
		vote_cast = 0;
		voter_count = 0;
		invalid_voters = 0;
		USART_putstring("Serial OK!");	// Test serial COM Comment before flashing the controller.

i2cReadWordArray(EEPROM_DEV_ADDR, namePtrSaveAddr, stringPtrArray, 30);		// Added 27.04.2017 Printer Integration. Reload Name string pointers


/************************ SD card detection routine	*****************************/
//cardType = 0;
//
//for (i = 0; i < 10; i++)
//{
	//error = SD_init();
	//if(!error) break;
//}
//
//if(error)
//{
	////TODO handle error codes here
//}
//
//error = getBootSectorData(); //read boot sector and keep necessary data in global variables
//if(error)
//{
	//// TODO handle card format type error detection
//}
//
//SPI_HIGH_SPEED;	//SCK - 4.6 MHz
//_delay_ms(1);   	//some delay for settling new spi speed

/**********************************************************************************/
			
	while (INIT && RESULT)					// When Control unit is powered on, these two messages are continuously
	{										// displayed on the LCD until a button i.e. INIT or RESULT is pressed.
		

		if(uartReceiveTerminator)
			{
				serviceCommand(&uartRecieveBuffer);
				uartReceiveTerminator = 0;
			 }

		lcd_cmd(0x01);						// Command to clear the LCD of any previous message.
		lcd_data_str("Press INIT for");
		lcd_cmd(0xC0);						// Command to go to line 2 of the LCD
		lcd_data_str("new session");
		_delay_ms(950);						// Delay before changing message on screen.
		lcd_cmd(0x01);						// Clears the screen
		lcd_data_str("RESULT to load");
		lcd_cmd(0xC0);
		lcd_data_str("previous session");
		_delay_ms(950);
	}
	
	if(!INIT && RESULT)						// If INIT is pressed, the controller enters configuration mode and should be connected to 
	{										// a PC.
		cli();								// Clear all interrupts mostly for disabling USART interrupt
		lcd_cmd(0x01);						// Clear control units LCD screen
		lcd_data_str("Connect to PC");
		while(USART_ReceiveByte() != 'A');	// Here the controller waits for a response from the EVM PC application.
		lcd_cmd(0x01);						// When 'A' is received, all previous session settings and vote count data in the EEPROM is
											// cleared.
		i2c_start();						// Put the start condition on the I2C lines.
		i2c_send(EEPROM_DEV_ADDR);			// Send the EEPROM device address.
		for (i=0; i<130; i++)				// Clear 130 bytes of the EEPROM - 2 bytes per candidate, 32 candidates + settings locations
		{
			i2c_send(0x00);
		}
		i2c_stop();							// Put stop  condition on the I2C lines.
		
		lcd_data_str("Configuring");
		lcd_cmd(0xC0);
		
		USART_putstring(DEV_ID);					// New ID format. Fixed length 1 alphabet and 3 no.s giving 25974 ID's

		while(USART_ReceiveByte() != 'L');			// Receive Location ID
		
		location_ID = USART_ReceiveByte();	
		eeprom_write_i2c(EEPROM_DEV_ADDR, LOCATION_ID_NAME, location_ID);
USART_putstring("OK");		

		location_ID_number = USART_ReceiveByte();	// ID of the machine at that location
		eeprom_write_i2c(EEPROM_DEV_ADDR, LOCATION_ID_NO, location_ID_number);
USART_putstring("OK");

		while(USART_ReceiveByte() != 'T');			// Receive machine type info 
													// Used when requires special type of voting
		mach_typ = USART_ReceiveByte();
		eeprom_write_i2c(EEPROM_DEV_ADDR, MACHINE_TYPE, mach_typ);
USART_putstring("OK");				
		/************************************************************  16.12.2015 added by neeraj 	****************************************************************/
		while(USART_ReceiveByte() != 'P');			// Receive code type info
													
		cod_typ = USART_ReceiveByte();
		eeprom_write_i2c(EEPROM_DEV_ADDR, CODE_TYPE, cod_typ);
USART_putstring("OK");		
		/************************************************************  16.12.2015 added by neeraj  	****************************************************************/
		
		
		while(USART_ReceiveByte() != 'N');			// Waiting for PC app to send 'N' - No. of candidates. This decides how many Voting pad buttons
													// should be activated.
		lcd_data_byte(0xFF);
			
		candidate_count = USART_ReceiveByte();		// waiting for 1st byte for no. of candidates
		//candidate_count = 32;
		eeprom_write_i2c(EEPROM_DEV_ADDR, 65535, candidate_count);		// No. of candidates selected is stored at FFFF in EPROM
USART_putstring("OK");				
/*****/	lcd_data_byte(0xFF);
		
		while(USART_ReceiveByte() != 'O');			// Wait for PC app to send 'O' - No. of votes a voter can cast. This decides how many votes a 
													// voter should be allowed to vote. Once a voter has cast this number of votes the Voting pad is
		voter_votes = USART_ReceiveByte();
		//voter_votes = 10;
		eeprom_write_i2c(EEPROM_DEV_ADDR, 65534, voter_votes);	// Store @ FFFE
USART_putstring("OK");		
/*****/	lcd_data_byte(0xFF);
				
		while(USART_ReceiveByte() != 'C');		
			
		temp1 = USART_ReceiveByte();				// 1st category value
		temp1 = (USART_ReceiveByte() << 4) + temp1;	// Combining 1st and 2nd category nibbles into a byte 
		//temp1 = 0x22;
		eeprom_write_i2c(EEPROM_DEV_ADDR, categories1_2, temp1);
USART_putstring("OK");		
/*****/	lcd_data_byte(0xFF);
		
		temp1 = USART_ReceiveByte();						// 3rd category value
		temp1 = (USART_ReceiveByte() << 4) + temp1;			// Combining 3rd and 4th category nibbles into a byte 
		//temp1 = 0x22;
		eeprom_write_i2c(EEPROM_DEV_ADDR, categories3_4, temp1);			// and storing into UC EEPROM @ categories3_4 ie location 1
USART_putstring("OK");		
/*****/	lcd_data_byte(0xFF);

		temp1 = USART_ReceiveByte();
		temp1 = (USART_ReceiveByte() << 4) + temp1;		// Combining 3rd and 4th category nibbles into a byte
		//temp1 = 0x22;
		eeprom_write_i2c(EEPROM_DEV_ADDR, categories5_6, temp1);		// and storing into UC EEPROM @ categories5_6 ie location 2
USART_putstring("OK");		
/*****/	lcd_data_byte(0xFF);

		temp1 = USART_ReceiveByte();
//		temp1 = (USART_ReceiveByte() << 4) + temp1;		// Combining 7th and 8th category nibbles into a byte
		//temp1 = 0x02;
		eeprom_write_i2c(EEPROM_DEV_ADDR, categories7_8, temp1);		// and storing into UC EEPROM @ categories7_8 ie location 3	 											
USART_putstring("OK");																
														// Saving the no of votes that can be cast in a category.
														// store votes in 1st category @ location 4 as defined
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg1_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg1_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg2_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg2_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg3_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg3_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg4_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg4_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg5_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg5_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg6_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg6_votes, 1);
		
		eeprom_write_i2c(EEPROM_DEV_ADDR, catg7_votes, USART_ReceiveByte());
USART_putstring("OK");		
		_delay_ms(10);
		//eeprom_write_i2c(EEPROM_DEV_ADDR, catg7_votes, 1);
		_delay_ms(10);
//		eeprom_write_i2c(EEPROM_DEV_ADDR, catg8_votes, USART_ReceiveByte());
		voting_on = 0;//ADDED ON 04.03.2016 SO THAT AFTER CONFIGURATION IT SHOULD NOT BACKUP THE VOTES AND GLOW LED'S
		sei();
		//SETBIT(UCSRB, 7); 

		i2c_write_2_bytes(EEPROM_DEV_ADDR, voters_voted, 0x00);	//Clear previous voter count
		i2c_write_2_bytes(EEPROM_DEV_ADDR, voters_not_voted, 0X00);//CLEAR INVALID VOTER COUNT
USART_putstring("ALL OK");		;
	}
	else if(INIT && !RESULT)		// If Result key was pressed. RESULT goes LOW.
	{
		lcd_cmd(0x01);
		lcd_data_str("Loading....");

		// i2cReadWordArray(EEPROM_DEV_ADDR, namePtrSaveAddr, stringPtrArray, 30);		// Added 27.04.2017 Printer Integration. Reload Name string pointers
		// _delay_ms(200);
		
		voter_votes = i2c_read_byte(EEPROM_DEV_ADDR, 0xFFFE);
		candidate_count = i2c_read_byte(EEPROM_DEV_ADDR, 0xFFFF);
		
		//voter_count = i2c_read_byte(EEPROM_DEV_ADDR, voters_voted);	// read no .of voters in previous session
		
		_delay_ms(100);
		i2c_read_2_bytes(voters_voted);
		voter_count = i2c_data_16;
		
		_delay_ms(100);
		i2c_read_2_bytes(voters_not_voted);
		invalid_voters = i2c_data_16;
		
		cod_typ=i2c_read_byte(EEPROM_DEV_ADDR, CODE_TYPE);
		mach_typ = i2c_read_byte(EEPROM_DEV_ADDR, MACHINE_TYPE);		// Send type of machine info Donor or general
		location_ID = i2c_read_byte(EEPROM_DEV_ADDR, LOCATION_ID_NAME);
		location_ID_number = i2c_read_byte(EEPROM_DEV_ADDR, LOCATION_ID_NO);
		
		flag_register = i2c_read_byte(EEPROM_DEV_ADDR, flag_register_bkp);
		
		while(!RESULT); 
	}
	
	lcd_cmd(0x01);
	 
/**********************************************************************************************************/
/*			Determining the size of the general category												  */
/**********************************************************************************************************/
// Find out the number of categories configured.
	no_of_categories = 0;
	_delay_ms(10);
	temp1 = i2c_read_byte(EEPROM_DEV_ADDR, categories1_2);		// Load categories1_2 info from internal EEPROM
	if((temp1 & 0x0F) > 0)
	no_of_categories = no_of_categories + 1;
	if((temp1 & 0xF0) > 0)
	no_of_categories = no_of_categories + 1;
	
	_delay_ms(10);
	temp1 = i2c_read_byte(EEPROM_DEV_ADDR, categories3_4);		// Load categories3_4 info from internal EEPROM
	if((temp1 & 0x0F) > 0)
	no_of_categories = no_of_categories + 1;
	if((temp1 & 0xF0) > 0)
	no_of_categories = no_of_categories + 1;
	
	_delay_ms(10);
	temp1 = i2c_read_byte(EEPROM_DEV_ADDR, categories5_6);		// Load categories5_6 info from internal EEPROM
	if((temp1 & 0x0F) > 0)
	no_of_categories = no_of_categories + 1;
	if((temp1 & 0xF0) > 0)
	no_of_categories = no_of_categories + 1;
	
	_delay_ms(10);
	temp1 = i2c_read_byte(EEPROM_DEV_ADDR, categories7_8);		// Load categories7_8 info from internal EEPROM
	if((temp1 & 0x0F) > 0)
	no_of_categories = no_of_categories + 1;
	if((temp1 & 0xF0) > 0)
	no_of_categories = no_of_categories + 1;
		
/**********************************************************************************************************/	
	USART_SendByte(0x0D);									// For testing only.
	USART_putstring("No. of categories: ");					// View the no. of categories configured
	USART_Transmit_dec(no_of_categories);
	USART_SendByte(0x0D);
/*********************************************************************************************************/
	
// CALCULATE THE NO. OF BUTTONS USED IN CATEGORIES (Total no. of buttons)
	_delay_ms(10);
	temp1 = i2c_read_byte(EEPROM_DEV_ADDR, categories1_2);
	_delay_ms(10);
	temp2 = i2c_read_byte(EEPROM_DEV_ADDR, categories3_4);
	_delay_ms(10);
	temp3 = i2c_read_byte(EEPROM_DEV_ADDR, categories5_6);
	_delay_ms(10);
	temp4 = i2c_read_byte(EEPROM_DEV_ADDR, categories7_8);
	no_of_buttons_in_categories = ((temp1 % 16) + (temp1 / 16) + (temp2 % 16) + (temp2 / 16) + (temp3 % 16) + (temp3 / 16)) + ((temp4 % 16) + (temp4 /16));

/***********************************************************************************************************/
	USART_putstring("No. of buttons in categories: ");
	USART_Transmit_dec(no_of_buttons_in_categories);	
	USART_SendByte(0x0D);
	
	USART_putstring("Size of category 1: ");
	USART_Transmit_dec(temp1 % 16);
	USART_SendByte(0x0D);
	
	USART_putstring("Winners in category 1: ");
	temp5 = i2c_read_byte(EEPROM_DEV_ADDR,catg1_votes);
	USART_Transmit_dec(temp5);
	USART_SendByte(0x0D);
	
	
	USART_putstring("Size of category 2: ");
	USART_Transmit_dec(temp1 / 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 2: ");
		temp6 = i2c_read_byte(EEPROM_DEV_ADDR,catg2_votes);
		USART_Transmit_dec(temp6);
		USART_SendByte(0x0D);
	
	USART_putstring("Size of category 3: ");
	USART_Transmit_dec(temp2 % 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 3: ");
		temp7 = i2c_read_byte(EEPROM_DEV_ADDR,catg3_votes);
		USART_Transmit_dec(temp7);
		USART_SendByte(0x0D);
	
	USART_putstring("Size of category 4: ");
	USART_Transmit_dec(temp2 / 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 4: ");
		temp8 = i2c_read_byte(EEPROM_DEV_ADDR,catg4_votes);
		USART_Transmit_dec(temp8);
		USART_SendByte(0x0D);
	
	USART_putstring("Size of category 5: ");
	USART_Transmit_dec(temp3 % 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 5: ");
		temp9 = i2c_read_byte(EEPROM_DEV_ADDR,catg5_votes);
		USART_Transmit_dec(temp9);
		USART_SendByte(0x0D);
	
	USART_putstring("Size of category 6: ");
	USART_Transmit_dec(temp3 / 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 6: ");
		temp10 = i2c_read_byte(EEPROM_DEV_ADDR,catg6_votes);
		USART_Transmit_dec(temp10);
		USART_SendByte(0x0D);
	
	USART_putstring("Size of category 7: ");
	USART_Transmit_dec(temp4 % 16);
	USART_SendByte(0x0D);
	
		USART_putstring("Winners in category 7: ");
		temp11 = i2c_read_byte(EEPROM_DEV_ADDR,catg7_votes);
		USART_Transmit_dec(temp11);
		USART_SendByte(0x0D);
	
	USART_putstring("No. of candidates: ");
	USART_Transmit_dec(candidate_count);
	USART_SendByte(0x0D);
		//GHHJ
// Size of general category is total no of candidates - candidates in categories.
	general_category = candidate_count - no_of_buttons_in_categories;

/**********************************************************************************************************/		
	USART_putstring("Size of general category: ");
	USART_Transmit_dec(general_category);
	USART_SendByte(0x0D);
	
	USART_putstring("Winners in general category: ");	
	gen_vote_count = voter_votes - (i2c_read_byte(EEPROM_DEV_ADDR, catg1_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg2_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg3_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg4_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg5_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg6_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg7_votes));
	USART_Transmit_dec(gen_vote_count);
	USART_SendByte(0x0D);
	
	USART_putstring("Voter Votes: ");
	USART_Transmit_dec(voter_votes);
	USART_SendByte(0x0D);
/**********************************************************************************************************/	

/************************		 This section was commented due to program space constraints in the ATmega32A controller. Can be used to display the configuration information 
								  on the LCD																																		*/
/*    lcd_cmd(0x01);
    lcd_data_str("Candidates: ");		// Display the candidate count on the LCD
    lcd_data_int(candidate_count);
	_delay_ms(1700);
	
	lcd_cmd(0x01);
	lcd_data_str("One voter votes");
	lcd_cmd(0xC0);
	lcd_data_int(voter_votes);
	_delay_ms(1900);
	
	lcd_cmd(0x01);
	lcd_data_str("Cat. 1:");
	lcd_data_int((eeprom_read_byte(categories1_2) & (0x0F)));
	lcd_cmd(0xC0);
	lcd_data_str("Votes 1:");
	lcd_data_int(eeprom_read_byte(catg1_votes));
	_delay_ms(1900);
	
	lcd_cmd(0x01);
	lcd_data_str("Cat. 2:");
	lcd_data_int((eeprom_read_byte(categories1_2) & (0xF0)) >> 4);
	lcd_cmd(0xC0);
	lcd_data_str("Votes 2:");
	lcd_data_int(eeprom_read_byte(catg2_votes));
	_delay_ms(1900);
	
	lcd_cmd(0x01);
	lcd_data_str("Cat. 3:");
	lcd_data_int((eeprom_read_byte(categories3_4) & (0x0F)));
	lcd_cmd(0xC0);
	lcd_data_str("Votes 3:");
	lcd_data_int(eeprom_read_byte(catg3_votes));
	_delay_ms(1900);
	
	lcd_cmd(0x01);
	lcd_data_str("Cat. 4:");
	lcd_data_int((eeprom_read_byte(categories3_4) & (0xF0)) >> 4);
	lcd_cmd(0xC0);
	lcd_data_str("Votes 4:");
	lcd_data_int(eeprom_read_byte(catg4_votes));
	_delay_ms(1900);
	
	
	temp1 = eeprom_read_byte(catg1_votes);	// Here the no. of votes in the general category is computed
	temp2 = eeprom_read_byte(catg2_votes);	// by subtracting the sum of votes a voter can cast in all non-general categories
	temp3 = eeprom_read_byte(catg3_votes);	// with the total no of votes the voter can cast.
	temp4 = eeprom_read_byte(catg4_votes);
	gen_vote_count = voter_votes - (temp1 + temp2 + temp3 + temp4 + eeprom_read_byte(catg5_votes));
	
	lcd_cmd(0x01);
	lcd_data_str("Gen cat.:");
	lcd_data_int(general_category);
	lcd_cmd(0xC0);
	lcd_data_str("Votes:");
	lcd_data_int(gen_vote_count);
	_delay_ms(1900);
/************************************************************************/
		
	while(1)						// Main Forever Loop
	{
			while(INIT)				// This is a pre-voting step before starting a voting session
			{						// Can be used for confirmation of vote-counts.
				if(RESULT == 0)		// This allows confirming of the vote count in the unit. Confirmation whether all votes are 0 when a new session
				disp_result_LCD();	// is started or they match the previous sessions vote count when reloaded can be done here.

				lcd_cmd(0x80);
				lcd_data_str( "Press INIT" );
				lcd_cmd(0xC0);							
				lcd_data_str("to begin voting");

				if(rx0_data == 'A')						// For sending device ID
				{
					USART_putstring(DEV_ID);
					rx0_data=0x00;
				}
			}
			while(!INIT);	// waiting for the INIT button to be released
			lcd_cmd(0x01);	// CLear the LCD
			while(RESULT)	// The controller will now remain in this loop until the RESULT button is pressed.
			{
				lcd_cmd(0x80);							// LCD displays from 1st line 1st character
				lcd_data_str( "Ready press INIT" );		// 
				lcd_cmd(0xc0);
				lcd_data_str("Voters: ");
				lcd_data_int(voter_count);				// This displays how many voters have cast atleast one vote
				
				/* for debug purposes*/
				USART_Transmit_dec(PINC);
				USART_SendByte(0x0D);
				
				
				
				if(rx0_data == 'A')	  // For sending device ID
				{
						USART_putstring(DEV_ID);
						rx0_data=0x00;
				}
				if(INIT == 0)							// Pressing INIT will initiate a vote cast.
				{ 
					//i2c_send_byte(PIC_DEV_ADDR, 0);		// Sends indication to PIC LED controller to clear all LED's
					while(!INIT);						// wait for INIT release
														// here the no. of votes in the general category is computed
														// by subtracting the sum of votes a voter can cast in all non-general categories
														// with the total no of votes the voter can cast.
					if (voting_on)
					{
						votes_counter = i2c_read_byte(EEPROM_DEV_ADDR, votes_count_bkp); // load remaining votes for incomplete session
						gen_vote_count = i2c_read_byte(EEPROM_DEV_ADDR, gen_vote_count_bkp);
						myregister1 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_1_8);
						myregister2 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_9_16);
						myregister3 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_17_24);
						myregister4 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_25_32);
						myregister5 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_33_40);
						myregister6 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_41_48);
						myregister7 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_49_56);
						myregister8 = i2c_read_byte(EEPROM_DEV_ADDR, vote_indic_57_64);
						//**************************************  added by neeraj on 16/12/2015    *************************************************///
						elem_count = i2c_read_byte(EEPROM_DEV_ADDR,elem_count_bkp);
						unsigned int i;
						for(i=0;i<63;i++)
						{
							voted_button_array[i]=eeprom_read_byte(i*2);
						}
						//**************************************  added by neeraj on 16/12/2015    *************************************************///
						vote_cast = 1;
					}
					else{
							votes_counter = voter_votes;
							gen_vote_count = voter_votes - (i2c_read_byte(EEPROM_DEV_ADDR, catg1_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg2_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg3_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg4_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg5_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg6_votes) + i2c_read_byte(EEPROM_DEV_ADDR, catg7_votes));
							vote_cast = 0;
							elem_count =0;// Resetting voted button address counter.
						}
/************************************************************************************************For testing purpose**********/						
					//USART_putstring("Total votes cast able: ");
					//USART_Transmit_dec(votes_counter);
					//USART_SendByte(0x0D);
					//USART_putstring("no. of general votes: ");
					//USART_Transmit_dec(gen_vote_count);
					//USART_SendByte(0x0D);
/************************************************************************************************For testing purpose**********/	
					
					vote_routine();
					myregister1 = 0;
					myregister2 = 0;
					myregister3 = 0;
					myregister4 = 0;
					myregister5 = 0;
					myregister6 = 0;
					myregister7 = 0;
					myregister8 = 0;
					/*vote_indicator_1 = vote_indicator_2 = vote_indicator_3 = vote_indicator_4 = vote_indicator_5 = vote_indicator_6 =
					vote_indicator_7 = vote_indicator_8 = vote_indicator_9 = vote_indicator_10 = vote_indicator_11 = vote_indicator_12 =
					vote_indicator_13 = vote_indicator_14 = vote_indicator_15 = vote_indicator_16 = vote_indicator_17 = vote_indicator_18 =
					vote_indicator_19 = vote_indicator_20 = vote_indicator_21 = vote_indicator_22 = vote_indicator_23 = vote_indicator_24 =
					vote_indicator_25 = vote_indicator_26 = vote_indicator_27 = vote_indicator_28 = vote_indicator_29 = vote_indicator_30 =
					vote_indicator_31 = vote_indicator_32 = vote_indicator_33 = vote_indicator_34 = vote_indicator_35 = vote_indicator_36 =
					vote_indicator_37 = vote_indicator_38 = vote_indicator_39 = vote_indicator_40 = vote_indicator_41 = vote_indicator_42 = 
					vote_indicator_43 = vote_indicator_44 = vote_indicator_45 = vote_indicator_46 = vote_indicator_47 = vote_indicator_48 = 
					vote_indicator_49 = vote_indicator_50 = vote_indicator_51 = vote_indicator_52 = vote_indicator_53 = vote_indicator_54 =
					vote_indicator_55 = vote_indicator_56 = vote_indicator_57 = vote_indicator_58 = vote_indicator_59 = vote_indicator_60 = 
					vote_indicator_61 = vote_indicator_62 = vote_indicator_63 = vote_indicator_64 = 0; */
					while(!INIT);
				}
			}
			while(1)				// If the RESULT key is pressed in the above loop, the controller enters this loop forever displaying 
			{						// only the sessions results. Further voting is disallowed in this manner unless a previous session is 
				disp_result_LCD();	// reloaded by switching OFF and ON the control unit.
				break;
			}			
	}
}



/* Change notes:

6/10/15: Shifted all settings to be stored to external EEPROM




*/