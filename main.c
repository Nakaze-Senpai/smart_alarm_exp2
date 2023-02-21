/*
 * smart_alarm_exp2.c
 *
 * Created: 2/19/2023 3:11:19 AM
 * Author : Nazat Kabir
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

//------------------------------------------------------Definitions--------------------------------------------------------------------------------------------------------
//LCD definitions
#define FALSE 0
#define TRUE 1
#define MrLCDsCrib PORTB
#define DataDir_MrLCDsCrib DDRB
#define MrLCDsControl PORTD
#define DataDir_MrLCDsControl DDRD
#define LightSwitch 5
#define ReadWrite 7
#define BiPolarMood 2
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
//tm-1637_7-segment definitions
#define TM1637_DDR  DDRA

#define TM1637_PORT PORTA
#define TM1637_PIN  PINA
#define TM1637_DIO  PA0
#define TM1637_CLK  PA1
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     --- .
//      D
//=== Decimal digits
//                       .GFEDCBA
#define TM1637_SPAT_0  0b00111111
#define TM1637_SPAT_1  0b00000110
#define TM1637_SPAT_2  0b01011011
#define TM1637_SPAT_3  0b01001111
#define TM1637_SPAT_4  0b01100110
#define TM1637_SPAT_5  0b01101101
#define TM1637_SPAT_6  0b01111101
#define TM1637_SPAT_7  0b00000111
#define TM1637_SPAT_8  0b01111111
#define TM1637_SPAT_9  0b01101111
//=== Letters
//                       .GFEDCBA
#define TM1637_SPAT_A  0b01110111
#define TM1637_SPAT_b  0b01111100
#define TM1637_SPAT_c  0b01011000
#define TM1637_SPAT_C  0b00111001
#define TM1637_SPAT_d  0b01011110
#define TM1637_SPAT_e  0b01111011
#define TM1637_SPAT_E  0b01111001
#define TM1637_SPAT_F  0b01110001
#define TM1637_SPAT_G  0b00111101
#define TM1637_SPAT_h  0b01110100
#define TM1637_SPAT_H  0b01110110
#define TM1637_SPAT_i  0b00010000
#define TM1637_SPAT_I  0b00110000
#define TM1637_SPAT_J  0b00011110
#define TM1637_SPAT_L  0b00111000
#define TM1637_SPAT_n  0b01010100
#define TM1637_SPAT_N  0b00110111
#define TM1637_SPAT_o  0b01011100
#define TM1637_SPAT_P  0b01110011
#define TM1637_SPAT_q  0b01100111
#define TM1637_SPAT_r  0b01010000
#define TM1637_SPAT_S  0b01101101
#define TM1637_SPAT_t  0b01111000
#define TM1637_SPAT_T  0b00110001
#define TM1637_SPAT_u  0b00011100
#define TM1637_SPAT_U  0b00111110
#define TM1637_SPAT_y  0b01101110
#define TM1637_SPAT_Z  0b01011011
//=== Misc
//                                 .GFEDCBA
#define TM1637_SPAT_BLANK        0
#define TM1637_SPAT_FULL         0b11111111
#define TM1637_SPAT_DOT          0b10000000
#define TM1637_SPAT_MINUS        0b01000000
#define TM1637_SPAT_DEGREE       0b01100011
#define TM1637_SPAT_HAT          0b00100011
#define TM1637_SPAT_QUOTE        0b00100010
#define TM1637_SPAT_APOSTROPHE   0b00100000
#define TM1637_SPAT_OPEN_PAREN   0b00111001
#define TM1637_SPAT_CLOSE_PAREN  0b00001111
//=== Bars
//                              .GFEDCBA
#define TM1637_SPAT_HBAR_T    0b00000001  // Top horizontal bar
#define TM1637_SPAT_HBAR_M    0b01000000  // Middle horizontal bar
#define TM1637_SPAT_HBAR_B    0b00001000  // Bottom horizontal bar
#define TM1637_SPAT_HBAR_TM   0b01000001  // Top and Middle horizontal bars
#define TM1637_SPAT_HBAR_TB   0b00001001  // Top and Bottom horizontal bars
#define TM1637_SPAT_HBAR_MB   0b01001000  // Middle and Bottom horizontal bars
#define TM1637_SPAT_HBAR_TMB  0b01001001  // Top, Middle and Bottom horizontal bars
#define TM1637_SPAT_VBAR_L    0b00110000  // Left vertical bar
#define TM1637_SPAT_VBAR_R    0b00000110  // Right vertical bar
#define TM1637_SPAT_VBAR_LR   0b00110110  // Left and Right vertical bar
#define TM1637_DIGITS_COUNT    4
#define TM1637_MAX_BRIGHTNESS  7
#define TM1637_CMD_SETUP           0b01000000
#define TM1637_CMD_SET_DATA        0b11000000
#define TM1637_CMD_SET_BRIGHTNESS  0b10000000
#define TM1637_MASK_POS             0b11
#define TM1637_MASK_BRIGHTNESS_REG  0b1111
#define TM1637_MASK_BRIGHTNESS      0b0111
#define TM1637_MASK_ON_OFF          0b1000
#define TM1637_QUARTER_CLOCK_PERIOD  100  // In microseconds (us)
#define TM1637_QUARTER_CYCLE_DELAY  _delay_us(TM1637_QUARTER_CLOCK_PERIOD)
#define TM1637_HALF_CYCLE_DELAY     _delay_us(2 * TM1637_QUARTER_CLOCK_PERIOD)
#define TM1637_COMM_START  TM1637_SET_DIO_LOW_AND_OUTPUT; TM1637_HALF_CYCLE_DELAY  // Pull DIO LOW - send START
#define TM1637_COMM_STOP   TM1637_SET_CLK_LOW_AND_OUTPUT; TM1637_QUARTER_CYCLE_DELAY;  /* Pull CLK LOW, to let TM1637 know that it can stop sending ACK (that is, it can stop pulling DIO LOW) */\
TM1637_SET_DIO_LOW_AND_OUTPUT; TM1637_QUARTER_CYCLE_DELAY;  /* Make sure DIO is LOW, because we need rising edge transition */\
TM1637_SET_CLK_HIGH_AND_INPUT; TM1637_HALF_CYCLE_DELAY;     /* Let the CLK be pulled HIGH by pullup resistor */\
TM1637_SET_DIO_HIGH_AND_INPUT; TM1637_HALF_CYCLE_DELAY      /* Let the DIO be pulled HIGH - send STOP */
#define TM1637_SET_CLK_LOW_AND_OUTPUT  TM1637_DDR |=  (1 << TM1637_CLK)
#define TM1637_SET_CLK_HIGH_AND_INPUT  TM1637_DDR &= ~(1 << TM1637_CLK)
#define TM1637_SET_DIO_LOW_AND_OUTPUT  TM1637_DDR |=  (1 << TM1637_DIO)
#define TM1637_SET_DIO_HIGH_AND_INPUT  TM1637_DDR &= ~(1 << TM1637_DIO)
#define TM1637_READ_DIO                bit_is_set(TM1637_PIN, TM1637_DIO)

extern const uint8_t TM1637_digitToSegment[] = {
	TM1637_SPAT_0,
	TM1637_SPAT_1,
	TM1637_SPAT_2,
	TM1637_SPAT_3,
	TM1637_SPAT_4,
	TM1637_SPAT_5,
	TM1637_SPAT_6,
	TM1637_SPAT_7,
	TM1637_SPAT_8,
	TM1637_SPAT_9,
	TM1637_SPAT_A,
	TM1637_SPAT_b,
	TM1637_SPAT_C,
	TM1637_SPAT_d,
	TM1637_SPAT_E,
	TM1637_SPAT_F
};

static inline uint8_t TM1637_writeByte(uint8_t data) {
	// Write data byte
	for (uint8_t i = 0; i < 8; i++) {
		TM1637_SET_CLK_LOW_AND_OUTPUT;
		TM1637_QUARTER_CYCLE_DELAY;

		// Set data bit
		if (data & 1) TM1637_SET_DIO_HIGH_AND_INPUT;
		else TM1637_SET_DIO_LOW_AND_OUTPUT;
		TM1637_QUARTER_CYCLE_DELAY;

		TM1637_SET_CLK_HIGH_AND_INPUT;
		TM1637_HALF_CYCLE_DELAY;
		data >>= 1;
	}

	// Prepare for receiving acknowledgement
	TM1637_SET_CLK_LOW_AND_OUTPUT;
	TM1637_SET_DIO_HIGH_AND_INPUT;
	TM1637_HALF_CYCLE_DELAY;

	// Read acknowledgement
	TM1637_SET_CLK_HIGH_AND_INPUT;
	TM1637_QUARTER_CYCLE_DELAY;
	uint8_t ack = TM1637_READ_DIO;
	if (ack == 0)                     // Not strictly necessary, but it's a good idea
	TM1637_SET_DIO_LOW_AND_OUTPUT;  // to avoid unnecessary voltage changes on DIO when possible
	TM1637_QUARTER_CYCLE_DELAY;

	return ack;
}

uint8_t TM1637_brightness_reg;

/** Initialize the library and prepare TM1637 chip for receiving display data **/
void TM1637_init(void) {
	TM1637_SET_DIO_HIGH_AND_INPUT;
	TM1637_SET_CLK_HIGH_AND_INPUT;
	TM1637_PORT &= ~((1 << TM1637_DIO) | (1 << TM1637_CLK));

	// Write TM1637_CMD_SETUP
	TM1637_COMM_START;
	TM1637_writeByte(TM1637_CMD_SETUP);
	TM1637_COMM_STOP;
}
static inline void TM1637_writeBrightnessReg(void) {
	// Write TM1637_CMD_SET_BRIGHTNESS + TM1637_brightness_reg
	TM1637_COMM_START;
	TM1637_writeByte(TM1637_CMD_SET_BRIGHTNESS | (TM1637_brightness_reg & TM1637_MASK_BRIGHTNESS_REG));
	TM1637_COMM_STOP;
}

static inline void TM1637_turnOnOff_inline(uint8_t on) {
	TM1637_brightness_reg = (on ? TM1637_MASK_ON_OFF : 0) | (TM1637_brightness_reg & TM1637_MASK_BRIGHTNESS);
	TM1637_writeBrightnessReg();
}
/** Turn the display on or off **/
void TM1637_turnOnOff(uint8_t on) {
	TM1637_turnOnOff_inline(on);
}
void TM1637_turnOn(void) {
	TM1637_turnOnOff_inline(1);
}
void TM1637_turnOff(void) {
	TM1637_turnOnOff_inline(0);
}
/** Set the display brightness (brightness range: 0-7) **/
void TM1637_setBrightness(uint8_t brightness) {
	TM1637_brightness_reg = (TM1637_brightness_reg & TM1637_MASK_ON_OFF) | (brightness & TM1637_MASK_BRIGHTNESS);
	TM1637_writeBrightnessReg();
}
/** Turn the display on and set its brightness (brightness range: 0-7) in a more performant
 *  way than calling TM1637_turnOn and TM1637_setBrightness subsequently **/
void TM1637_turnOnAndSetBrightness(uint8_t brightness) {
	TM1637_brightness_reg = TM1637_MASK_ON_OFF | (brightness & TM1637_MASK_BRIGHTNESS);
	TM1637_writeBrightnessReg();
}
/** Display raw segments data, starting at given digit/position **/
void TM1637_setSegments(const uint8_t segments[], uint8_t length, uint8_t pos) {
	TM1637_COMM_START;
	// Write TM1637_CMD_SET_DATA + first digit address
	TM1637_writeByte(TM1637_CMD_SET_DATA | (pos & TM1637_MASK_POS));
	// Write data bytes
	for (uint8_t i = 0; i < length; i++)
	TM1637_writeByte(segments[i]);
	TM1637_COMM_STOP;
}
#define TM1637_setSegment(segment, pos)  TM1637_setSegments((uint8_t[]) { (segment) }, 1, (pos))
/** Clear the display - turn off all segments **/
void TM1637_clear(void) {
	TM1637_COMM_START;
	TM1637_writeByte(TM1637_CMD_SET_DATA);
	// Write blank segment data bytes
	for (uint8_t i = 0; i < TM1637_DIGITS_COUNT; i++)
	TM1637_writeByte(TM1637_SPAT_BLANK);
	TM1637_COMM_STOP;
}
/** Display a number in a given numerical system (supported bases: 2-16) **/
uint8_t TM1637_displayNumber(int32_t num, uint8_t base) {
	// Assume: 2 <= base <= 16
	uint8_t hello[4]= { TM1637_SPAT_BLANK, TM1637_SPAT_BLANK, TM1637_SPAT_BLANK, TM1637_SPAT_0 };
	if (num == 0) {
		TM1637_setSegments(hello, 4, 0);
		return 0;
	}
	int16_t pos2 = base * base;
	int16_t pos3 = pos2 * base;
	// Check if number fits in 4 digits, or 3 digits + minus sign
	if (num <= -pos3 ||
	(base <  16 && num >= pos3 * base) ||
	(base == 16 && num >  0xFFFF)
	) return 1;

	uint16_t abs_num = num < 0 ? -num : num;
	// Extract digits from num
	uint8_t data[4] = {
		(abs_num / pos3) % base,
		(abs_num / pos2) % base,
		(abs_num / base) % base,
		abs_num         % base
	};
	// Convert digits to segments
	uint8_t started = 0;
	for (uint8_t i = 0; i < 4; i++) {
		if (started || data[i]) {
			data[i] = TM1637_digitToSegment[data[i]];
			if (!started && num < 0)
			data[i - 1] = TM1637_SPAT_MINUS;
			started = 1;
		} else data[i] = TM1637_SPAT_BLANK;
	}
	TM1637_setSegments(data, 4, 0);
	return 0;
}
#define TM1637_displayBinNumber(num) TM1637_displayNumber(num,  2)
#define TM1637_displayOctNumber(num) TM1637_displayNumber(num,  8)
#define TM1637_displayDecNumber(num) TM1637_displayNumber(num, 10)
#define TM1637_displayHexNumber(num) TM1637_displayNumber(num, 16)
/** Display hexadecimal digits, starting at given digit/position **/
void TM1637_displayDigits(const uint8_t digits[], uint8_t length, uint8_t pos) {
	uint8_t segments[TM1637_DIGITS_COUNT];
	for (uint8_t i = 0; i < length; i++)
	segments[i] = TM1637_digitToSegment[digits[i]];
	TM1637_setSegments(segments, length, pos);
}
//-----------------------------------------------------Global_variables---------------------------------------------------------------------------------------------------
int second_count=0;
int min_occured=1;
int allow_uart=0;
char bluetooth_array[51];
int bluetooth_array_index=0;
char bluetooth_array_case1[51];
char bluetooth_array_case2[10];
int mobile_hour,mobile_min;
int a;

//--------------------------------------------------------LCD_Functions---------------------------------------------------------------------------------------------------
void Peek_A_Boo(void)
{
	MrLCDsControl |= 1<<LightSwitch;
	asm volatile ("nop");
	asm volatile ("nop");
	MrLCDsControl &= ~1<<LightSwitch;
}

void Check_IF_MrLCD_isBusy(void)
{
	DataDir_MrLCDsCrib = 0;
	MrLCDsControl |= 1<<ReadWrite;
	MrLCDsControl &= ~1<<BiPolarMood;
	while (MrLCDsCrib >= 0x80)
	{
		Peek_A_Boo();
	}
	DataDir_MrLCDsCrib = 0xFF;
}

void Send_A_Command(unsigned char command)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = command;
	MrLCDsControl &= ~ ((1<<ReadWrite)|(1<<BiPolarMood));
	Peek_A_Boo();
	MrLCDsCrib = 0;
	_delay_ms(2);
}

void Send_A_Character(unsigned char character)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = character;
	MrLCDsControl &= ~ (1<<ReadWrite);
	MrLCDsControl |= 1<<BiPolarMood;
	Peek_A_Boo();
	MrLCDsCrib = 0;
	_delay_us(100);
}

void Send_A_String(char *StringOfCharacters)
{
	while(*StringOfCharacters > 0)
	{
		Send_A_Character(*StringOfCharacters++);
	}
}

void GotoMrLCDsLocation(uint8_t x, uint8_t y)
{
	if(y==1)
	{
		Send_A_Command(0x80 + x-1);
	}
	else if(y==2)
	{
		Send_A_Command(0xC0 + x-1);
	}
	_delay_ms(2);
}

void Send_An_Integer(unsigned int m)
{
	char my_string[100];
	itoa(m, my_string, 10);
	Send_A_String(my_string);
}

//--------------------------------------------------------Static_Storage_Functions-----------------------------------------------------------------------------------
void EEOpen()
{
	TWBR = 5;
	TWSR &= (~((1<<TWPS1)|(1<<TWPS0)));
}

uint8_t EEWriteByte(uint16_t address,uint8_t data)
{
	do
	{
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
		if((TWSR & 0xF8) != 0x08)
		return FALSE;
		TWDR=0b10100000;
		TWCR=(1<<TWINT)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
	}while((TWSR & 0xF8) != 0x18);
	TWDR=(address>>8);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x28)
	return FALSE;
	TWDR=(address);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x28)
	return FALSE;
	TWDR=(data);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x28)
	return FALSE;
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while(TWCR & (1<<TWSTO));
	_delay_ms(12);
	return TRUE;
}

uint8_t EEReadByte(uint16_t address)
{
	uint8_t data;
	do
	{
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
		if((TWSR & 0xF8) != 0x08)
		return FALSE;
		TWDR=0b10100000;
		TWCR=(1<<TWINT)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
	}while((TWSR & 0xF8) != 0x18);
	TWDR=(address>>8);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x28)
	return FALSE;
	TWDR=(address);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x28)
	return FALSE;
	TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x10)
	return FALSE;
	TWDR=0b10100001;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x40)
	return FALSE;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((TWSR & 0xF8) != 0x58)
	return FALSE;
	data=TWDR;
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while(TWCR & (1<<TWSTO));
	return data;
}

void  clear_full_eeprom(void)
{
	for(int fir_add=0;fir_add<65536;fir_add++)
	{
		EEWriteByte(fir_add,0);
	}
}

//-----------------------------------------------------------------------Bluetooth_Module_Functions-------------------------------------------------------------
void UART_init(long USART_BAUDRATE)
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE >> 8);
}

unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);
	return(UDR);
}

void UART_TxChar(char ch)
{
	while (! (UCSRA & (1<<UDRE)));
	UDR = ch ;
}

void UART_SendString(char *str)
{
	unsigned char j=0;
	while (str[j]!=0)
	{
		UART_TxChar(str[j]);
		j++;
	}
}

//---------------------------------------------------------my_functions---------------------------------------------------------------------------------------------------

void set_time_in_EEPROM(int year,int month,int day,int hour,int min)
{
	int year_array[4];
	int month_array[2];
	int day_array[2];
	int hour_array[2];
	int min_array[2];
	year_array[3]=year/1000;
	year_array[2]=(year/100)%10;
	year_array[1]=(year/10)%10;
	year_array[0]=year%10;
	month_array[1]=month/10;
	month_array[0]=month%10;
	day_array[1]=day/10;
	day_array[0]=day%10;
	hour_array[1]=hour/10;
	hour_array[0]=hour%10;
	min_array[1]=min/10;
	min_array[0]=min%10;
	EEWriteByte(0,hour_array[1]);
	EEWriteByte(1,hour_array[0]);
	EEWriteByte(2,min_array[1]);
	EEWriteByte(3,min_array[0]);
	EEWriteByte(4,day_array[1]);
	EEWriteByte(5,day_array[0]);
	EEWriteByte(6,month_array[1]);
	EEWriteByte(7,month_array[0]);
	for(int temp1=8;temp1<=11;temp1++)
	{
		EEWriteByte(temp1,year_array[11-temp1]);
	}
}

void set_time_in_EEPROM_in_any_location(int location,int year,int month,int day,int hour,int min)
{
	int year_array[4];
	int month_array[2];
	int day_array[2];
	int hour_array[2];
	int min_array[2];
	year_array[3]=year/1000;
	year_array[2]=(year/100)%10;
	year_array[1]=(year/10)%10;
	year_array[0]=year%10;
	month_array[1]=month/10;
	month_array[0]=month%10;
	day_array[1]=day/10;
	day_array[0]=day%10;
	hour_array[1]=hour/10;
	hour_array[0]=hour%10;
	min_array[1]=min/10;
	min_array[0]=min%10;
	EEWriteByte(location+0,hour_array[1]);
	EEWriteByte(location+1,hour_array[0]);
	EEWriteByte(location+2,min_array[1]);
	EEWriteByte(location+3,min_array[0]);
	EEWriteByte(location+4,day_array[1]);
	EEWriteByte(location+5,day_array[0]);
	EEWriteByte(location+6,month_array[1]);
	EEWriteByte(location+7,month_array[0]);
	for(int temp1=(location+8),temp2=8;temp1<=(location+11);temp1++,temp2++)
	{
		EEWriteByte(temp1,year_array[11-temp2]);
	}
}


int get_time_from_EEPROM(char time_type)
{
	int digit[4];
	switch (time_type)
	{
	case 'y':
		digit[0]=EEReadByte(11);
		digit[1]=EEReadByte(10);
		digit[2]=EEReadByte(9);
		digit[3]=EEReadByte(8);
		return ((1000*digit[3])+(100*digit[2])+(10*digit[1])+digit[0]);
		break;
	case 'M':
		digit[0]=EEReadByte(7);
		digit[1]=EEReadByte(6);
		return ((10*digit[1])+digit[0]);
		break;
	case 'd':
		digit[0]=EEReadByte(5);
		digit[1]=EEReadByte(4);
		return ((10*digit[1])+digit[0]);
		break;
	case 'h':
		digit[0]=EEReadByte(1);
		digit[1]=EEReadByte(0);
		return ((10*digit[1])+digit[0]);
		break;
	case 'm':
		digit[0]=EEReadByte(3);
		digit[1]=EEReadByte(2);
		return ((10*digit[1])+digit[0]);
		break;
	}
}

int increment_time_by_one_minute(char time_type,int year,int month,int day,int hour,int min)
{
	int next_year,next_month,next_day,next_hour,next_min;
	next_year=year;
	next_month=month;
	next_min=min;
	next_hour=hour;
	next_day=day;
	next_min=min+1;
	if(next_min==60)
	{
		next_min=0;
		next_hour=hour+1;
	}
	if(next_hour==24)
	{
		next_hour=0;
		next_day=day+1;
	}
	if((next_day==31)&&((month==4)||(month==6)||(month==9)||(month==11)))
	{
		next_day=1;
		next_month=month+1;
	}
	if((next_day==32)&&((month==1)||(month==3)||(month==5)||(month==7)||(month==8)||(month==10)))
	{
		next_day=1;
		next_month=month+1;
	}
	if((next_day==32)&&(month==12))
	{
		next_day=1;
		next_month=1;
		next_year=year+1;
	}
	if(month==2)
	{
		if(year%400==0)
		{
			//leap_year
			if(next_day==30)
			{
				next_day=1;
				next_month=3;
			}
		}
		else if(year%100==0)
		{
			//not_leap_year
			if(next_day==29)
			{
				next_day=1;
				next_month=3;
			}
		}
		else if(year%4==0)
		{
			//leap_year
			if(next_day==30)
			{
				next_day=1;
				next_month=3;
			}
		}
		else
		{
			//not_leap_year
			if(next_day==29)
			{
				next_day=1;
				next_month=3;
			}
		}
	}
	switch(time_type)
	{
	case 'y':
		return next_year;
		break;
	case 'M':
		return next_month;
		break;
	case 'd':
		return next_day;
		break;
	case 'h':
		return next_hour;
		break;
	case 'm':
		return next_min;
		break;		
	}
}

void increment_time_by_one_minute_in_EEPROM(void)
{
	int y,M,d,h,m;
	int ny,nM,nd,nh,nm;
	y=get_time_from_EEPROM('y');
	M=get_time_from_EEPROM('M');
	d=get_time_from_EEPROM('d');
	h=get_time_from_EEPROM('h');
	m=get_time_from_EEPROM('m');
	ny=increment_time_by_one_minute('y',y,M,d,h,m);
	nM=increment_time_by_one_minute('M',y,M,d,h,m);
	nd=increment_time_by_one_minute('d',y,M,d,h,m);
	nh=increment_time_by_one_minute('h',y,M,d,h,m);
	nm=increment_time_by_one_minute('m',y,M,d,h,m);
	set_time_in_EEPROM(ny,nM,nd,nh,nm);
}

void initialize_lcd(void)
{
		Send_A_Command(0x01);//clear the whole screen
		Send_A_Command(0x38);//2 lines and 5�7 matrix
		Send_A_Command(0x0E);//display on, cursor blinking
		Send_A_Command(0x80);//force cursor to the beginning of first line	
}

void clear_lcd(void)
{
		Send_A_Command(0x01);//clear the whole screen
		Send_A_Command(0x0E);//display on, cursor blinking
		Send_A_Command(0x80);//force cursor to the beginning of first line
}

void show_time_on_lcd(void)
{
		clear_lcd();
		Send_A_String("Time: ");
		Send_An_Integer(get_time_from_EEPROM('h'));
		Send_A_String(":");
		Send_An_Integer(get_time_from_EEPROM('m'));
		GotoMrLCDsLocation(1,2);
		Send_A_String("Date: ");
		Send_An_Integer(get_time_from_EEPROM('d'));
		Send_A_String("/");
		Send_An_Integer(get_time_from_EEPROM('M'));
		Send_A_String("/");
		Send_An_Integer(get_time_from_EEPROM('y'));
}

void show_time_on_7_segment(void)
{
	int segment_display_number;
	TM1637_clear();
	TM1637_turnOnAndSetBrightness(TM1637_MAX_BRIGHTNESS);
	segment_display_number=(100*get_time_from_EEPROM('h'))+get_time_from_EEPROM('m');
	TM1637_displayDecNumber(segment_display_number);
	int count = 0;
	while(segment_display_number)
	{
		count++;
		segment_display_number/=10;
	}
	if(count==3)
	{
		uint8_t zero_digit[1]={ TM1637_SPAT_0 };
		TM1637_setSegments(zero_digit,1, 0);
	}
	else if(count==2)
	{
		uint8_t zero_digit[2]={ TM1637_SPAT_0,TM1637_SPAT_0 };
		TM1637_setSegments(zero_digit,2, 0);
	}
	else if(count==1)
	{
		uint8_t zero_digit[3]={ TM1637_SPAT_0,TM1637_SPAT_0,TM1637_SPAT_0 };
		TM1637_setSegments(zero_digit,3, 0);
	}
	else if(count==0)
	{
		uint8_t zero_digit[4]={ TM1637_SPAT_0,TM1637_SPAT_0,TM1637_SPAT_0,TM1637_SPAT_0 };
		TM1637_setSegments(zero_digit,4, 0);
	}
}

void buzzer_blink_high_pitch(int number_of_blinks)
{
	int cnt_buzzer;
	//number_of_blinks*=2;
	while(number_of_blinks--)
	{
		cnt_buzzer=500;
		while(cnt_buzzer--)
		{
			PORTD |= (1<<PIND6);
			_delay_us(100);
			PORTD &= ~(1<<PIND6);
			_delay_us(200);
		}
		PORTD &= ~(1<<PIND6);
		_delay_ms(150);
	}
	PORTD &= ~(1<<PIND6);
}

void buzzer_blink_low_pitch(int number_of_blinks)
{
	int cnt_buzzer;
	//number_of_blinks*=2;
	while(number_of_blinks--)
	{
		cnt_buzzer=150;
		while(cnt_buzzer--)
		{
			PORTD |= (1<<PIND6);
			_delay_us(100);
			PORTD &= ~(1<<PIND6);
			_delay_us(900);
		}
		PORTD &= ~(1<<PIND6);
		_delay_ms(150);
	}
	PORTD &= ~(1<<PIND6);
}

void buzzzer_button_indication(void)
{	int cnt_buzzer;
	cnt_buzzer=100;
	while(cnt_buzzer--)
	{
		PORTD |= (1<<PIND6);
		_delay_us(100);
		PORTD &= ~(1<<PIND6);
		_delay_us(900);
	}
	PORTD &= ~(1<<PIND6);
}

uint8_t lcd_and_buzzer_info_extract_from_bluetooth(uint8_t bit_number,uint8_t lcd_buzzer_info)
{
	if(!bit_number)
	{
		return (lcd_buzzer_info&(0b00001111));
	}
	else
	{
		return ((lcd_buzzer_info&(0b11110000))>>4);
	}
}

uint8_t lcd_buzzer_info_execution(uint8_t from_bluetooth)
{
	switch(from_bluetooth%4)
	{
	case 0:
		//no_buzzer
		break;
	case 1:
		buzzer_blink_low_pitch(1);
		break;
	case 2:
		buzzer_blink_low_pitch(2);
		break;
	case 3:
		buzzer_blink_low_pitch(3);
		break;
	}
	switch(from_bluetooth/4)
	{
	case 0:
		return 1;
		break;
	case 1:
		Send_A_Command(0x00);
		GotoMrLCDsLocation(1,1);
		Send_A_String("   DEFAULT__1   ");
		return 0;
		break;
	case 2:
		Send_A_Command(0x00);
		GotoMrLCDsLocation(1,1);
		Send_A_String("   DEFAULT__2   ");
		return 0;
		break;
	case 3:
		//do_nothing
		return 0;
		break;
	}
}

int char_array_to_decimal(char arr[],int start_index,int spaces)
{
	int sum=0;
	int power_of_10=1;
	for(int i=(start_index+spaces-1);i>=start_index;i--)
	{
		sum+=(arr[i]-48)*power_of_10;
		power_of_10*=10;
	}
	return sum;
}

int total_days_of_that_year_and_month(int year,int month)
{
	int feb;
	if(year%400==0)
	{
		feb=29;
	}
	else if(year%100==0)
	{
		feb=28;
	}
	else if(year%4==0)
	{
		feb=29;
	}
	else
	{
		feb=28;
	}
	int month_array[12]={31,feb,31,30,31,30,31,31,30,31,30,31};
	for(int i=1;i<=12;i++)
	{
		if(month==i)
		{
			return month_array[i-1];
		}
	}
}

int date_after_a_certain_minute(char return_time_type,int add_minute,int py,int pM,int pd,int ph,int pm)
{
	int add_day;
	pm+=add_minute;
	ph+=(pm/60);
	pm%=60;
	add_day=ph/24;
	ph%=24;

	if((pd+add_day)<=total_days_of_that_year_and_month(py,pM))
	{
		pd+=add_day;
	}
	else
	{
		pd=pd+add_day-total_days_of_that_year_and_month(py,pM);
		pM++;
	}

	py+=(pM/12);
	pM%=12;

	switch(return_time_type)
	{
		case 'y':
		return py;
		break;
		case 'M':
		return pM;
		break;
		case 'd':
		return pd;
		break;
		case 'h':
		return ph;
		break;
		case 'm':
		return pm;
		break;
	}
}

void clear_whole_eeprom(void)
{
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("Started clearing");
	for(int i=0;i<=30000;i++)
	{
		EEWriteByte(i,0);
		GotoMrLCDsLocation(1,2);
		Send_An_Integer(i);
	}
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("hopefully cleared");	
}

void clear_alarm_blocks_in_EEPROM(void)
{
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("Started clearing");
	for(int i=301;i<=30000;i+=46)
	{
		EEWriteByte(i,0);
		GotoMrLCDsLocation(1,2);
		Send_An_Integer(i);
	}
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("hopefully");
	GotoMrLCDsLocation(1,2);
	Send_A_String("    cleared");
}

void check_if_all_alarm_block_in_EEPROM_is_clear_using_lcd(void)
{
	int a=1;
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("Started checking");
	_delay_ms(2000);
	for(int i=301;i<=30000;i+=46)
	{
		GotoMrLCDsLocation(1,2);
		Send_An_Integer(i);
		if(EEReadByte(i)!=0)
		{
			a=0;
			Send_A_Command(0x01);
			GotoMrLCDsLocation(1,1);
			Send_A_String("failed");
			break;
		}
	}
	if(a)
	{
		Send_A_Command(0x01);
		GotoMrLCDsLocation(1,1);
		Send_A_String("success");	
	}
}

void check_if_the_whole_EEPROM_is_clear_using_lcd(void)
{
	int a=1;
	Send_A_Command(0x01);
	GotoMrLCDsLocation(1,1);
	Send_A_String("Started checking");
	_delay_ms(2000);
	for(int i=0;i<=30000;i++)
	{
		GotoMrLCDsLocation(1,2);
		Send_An_Integer(i);
		if(EEReadByte(i)!=0)
		{
			a=0;
			Send_A_Command(0x01);
			GotoMrLCDsLocation(1,1);
			Send_A_String("failed");
			break;
		}
	}
	if(a)
	{
		Send_A_Command(0x01);
		GotoMrLCDsLocation(1,1);
		Send_A_String("success");
	}
}

int time_match(void)
{
	for(int i=301;i<=30000;i+=46)
	{
		if(EEReadByte(i)=='*')
		{
			int match=1;
			for(int j=1;j<=12;j++)
			{
				if(EEReadByte(i+j)!=EEReadByte(j-1))
				{
					match=0;
					break;
				}
			}
			if(match)
			{
				return i;
			}
		}
		else
		{
			return 0;
		}
	}
}

//#######################################################################----MAIN_FUNCTION----####################################################################
int main(void)
{
	//##Setting_up_the_whole_device
	
	//interrupt start
	sei();
	
	//bluetooth module-UART communication start
	char c;
	UART_init(9600);
	
	//EEPROM communication initialization
	EEOpen();
	_delay_ms(2);
	
	//initializing LCD
	DataDir_MrLCDsControl |= 1<<LightSwitch | 1<<ReadWrite | 1<<BiPolarMood;
	DDRA |= (1<<PINA2);
	_delay_ms(15);
	PORTA ^= (1<<PINA2);
	initialize_lcd();
	
	//initializing tm1637 7-segment display
	TM1637_init();
	
	//initializing buttons
	DDRD |= (1<<PIND6);
	PORTD &= ~(1<<PIND6);
	DDRA &= ~(1 << PINA3);
	PORTA |= 1 << PINA3;
	int Pressed = 0;
	_delay_ms(100);
	
	//initializing timer and interrupt
	TCCR1B |= (1<<CS12);
	TCCR1B |= (1<<WGM12);
	TIMSK |= (1<<OCIE1A);
	OCR1A = 31249;
	
	//setting present time in EEPROM
	set_time_in_EEPROM(2023,2,28,23,45);
	
	//some additional initializations
	bluetooth_array_case1[0]='0';
	bluetooth_array_case1[1]='0';
	
	while (1)
	{
		if(min_occured)
		{
			//##code_to_execute_once_every_minute 
			increment_time_by_one_minute_in_EEPROM();
			//show_time_on_lcd();
			show_time_on_7_segment();
			if(time_match()!=0)
			{
				if(lcd_buzzer_info_execution(EEReadByte(time_match())))
				{
					for(int imi=0;imi<=16;imi++)
					{
						Send_A_Character(EEReadByte(time_match()+13+imi));
					}
				}
			}
			min_occured=0;
		}
		else
		{
			//##rest_of_the_code
			if(allow_uart==1)
			{
				bluetooth_array[bluetooth_array_index]=UART_RxChar();
				if(bluetooth_array[bluetooth_array_index]=='*')
				{
					bluetooth_array_index=0;
					bluetooth_array[0]='*';
					for(int i_bluetooth=1;i_bluetooth<51;i_bluetooth++)
					{
						bluetooth_array[i_bluetooth]='#';
					}
				}
				if(bluetooth_array[0]=='*')
				{
					switch(bluetooth_array[1])
					{
					case '1':
						//input_alarm_block_from_bluetooth
						if(bluetooth_array_index>=2)
						{
							bluetooth_array_case1[bluetooth_array_index]=bluetooth_array[bluetooth_array_index];
						}
						if(bluetooth_array_index==50)
						{
							//lcd_indication
							Send_A_Command(0x01);
							GotoMrLCDsLocation(1,1);
							Send_A_String("recieved");
							//---->	
						}
						break;
					case '2':
						//set present time
						if(bluetooth_array_index>=2)
						{
							bluetooth_array_case2[bluetooth_array_index]=bluetooth_array[bluetooth_array_index];
							if(bluetooth_array_index>=5)
							{
								mobile_hour=(10*(bluetooth_array_case2[2]-48))+bluetooth_array_case2[3]-48;
								mobile_min=(10*(bluetooth_array_case2[4]-48))+bluetooth_array_case2[5]-48;
								set_time_in_EEPROM(get_time_from_EEPROM('y'),get_time_from_EEPROM('M'),get_time_from_EEPROM('d'),mobile_hour,mobile_min);
							}
						}
						break;
					case '3':
						//delete_alarm_block
						break;
					case '4':
						//bluetooth_turn_off
						//timer_on
						/*
						for(int ioi=347;ioi<=30000;ioi++)
						{
							Send_A_Command(0x01);
							GotoMrLCDsLocation(1,1);
							_delay_ms(50);
							Send_An_Integer(ioi-300);
							Send_A_Character(' ');
							if(((ioi>=302)&&(ioi<=314))||((ioi>=348)&&(ioi<=360)))
							{
								Send_An_Integer(EEReadByte(ioi));
							}
							else
							{
								Send_A_Character(EEReadByte(ioi));
							}
							_delay_ms(1000);
						}*/
						//Send_A_String("hi");
						//_delay_ms(2);
						allow_uart=0;
						TIMSK |= (1<<OCIE1A);
						break;	
					case '5':
						//burn what is inside of the array to the memory
						a=1;
						for(int eepromscan=301;eepromscan<=30000;eepromscan+=46)
						{
							if(EEReadByte(eepromscan)=='*')
							{
								continue;
							}
							else
							{
								if(a==2)
								{
									//lcd_indication
									Send_A_Command(0x01);
									GotoMrLCDsLocation(1,1);
									Send_A_String("started_writing2");
									_delay_ms(500);
									//---->
									EEWriteByte(eepromscan,'*');
									//have to add the duration here
									int yy=date_after_a_certain_minute('y',char_array_to_decimal(bluetooth_array_case1,15,4),char_array_to_decimal(bluetooth_array_case1,10,4),char_array_to_decimal(bluetooth_array_case1,8,2),char_array_to_decimal(bluetooth_array_case1,6,2),char_array_to_decimal(bluetooth_array_case1,2,2),char_array_to_decimal(bluetooth_array_case1,4,2));
									int MM=date_after_a_certain_minute('M',char_array_to_decimal(bluetooth_array_case1,15,4),char_array_to_decimal(bluetooth_array_case1,10,4),char_array_to_decimal(bluetooth_array_case1,8,2),char_array_to_decimal(bluetooth_array_case1,6,2),char_array_to_decimal(bluetooth_array_case1,2,2),char_array_to_decimal(bluetooth_array_case1,4,2));
									int dd=date_after_a_certain_minute('d',char_array_to_decimal(bluetooth_array_case1,15,4),char_array_to_decimal(bluetooth_array_case1,10,4),char_array_to_decimal(bluetooth_array_case1,8,2),char_array_to_decimal(bluetooth_array_case1,6,2),char_array_to_decimal(bluetooth_array_case1,2,2),char_array_to_decimal(bluetooth_array_case1,4,2));
									int hh=date_after_a_certain_minute('h',char_array_to_decimal(bluetooth_array_case1,15,4),char_array_to_decimal(bluetooth_array_case1,10,4),char_array_to_decimal(bluetooth_array_case1,8,2),char_array_to_decimal(bluetooth_array_case1,6,2),char_array_to_decimal(bluetooth_array_case1,2,2),char_array_to_decimal(bluetooth_array_case1,4,2));
									int mm=date_after_a_certain_minute('m',char_array_to_decimal(bluetooth_array_case1,15,4),char_array_to_decimal(bluetooth_array_case1,10,4),char_array_to_decimal(bluetooth_array_case1,8,2),char_array_to_decimal(bluetooth_array_case1,6,2),char_array_to_decimal(bluetooth_array_case1,2,2),char_array_to_decimal(bluetooth_array_case1,4,2));
									//clear_lcd();
									//Send_A_Character(mm+48);
									set_time_in_EEPROM_in_any_location(eepromscan+1,yy,MM,dd,hh,mm);
									//EEWriteByte(eepromscan+eeprom_temp,bluetooth_array_case1[eeprom_temp+1]);
									EEWriteByte(eepromscan+13,lcd_and_buzzer_info_extract_from_bluetooth(1,bluetooth_array_case1[14]));
									for(int eeprom_temp=14;eeprom_temp<=45;eeprom_temp++)
									{
										EEWriteByte(eepromscan+eeprom_temp,'#');
									}
									//lcd_indication
									Send_A_Command(0x01);
									GotoMrLCDsLocation(1,1);
									Send_A_String("ended_writing2");
									_delay_ms(500);
									//---->
									break;
								}
								if(a==1)
								{
									//lcd_indication
									Send_A_Command(0x01);
									GotoMrLCDsLocation(1,1);
									Send_A_String("started_writing1");
									_delay_ms(500);
									//---->
									EEWriteByte(eepromscan,'*');
									for(int eeprom_temp=1;eeprom_temp<=12;eeprom_temp++)
									{
										EEWriteByte(eepromscan+eeprom_temp,bluetooth_array_case1[eeprom_temp+1]-48);
									}
									EEWriteByte(eepromscan+13,lcd_and_buzzer_info_extract_from_bluetooth(0,bluetooth_array_case1[14]));
									for(int eeprom_temp=14;eeprom_temp<=45;eeprom_temp++)
									{
										EEWriteByte(eepromscan+eeprom_temp,bluetooth_array_case1[eeprom_temp+5]);
									}
									//lcd_indication
									Send_A_Command(0x01);
									GotoMrLCDsLocation(1,1);
									Send_A_String("ended_writing1");
									_delay_ms(500);
									//---->
									a=2;
								}
							}
						}
						break;
					}
					bluetooth_array_index++;
				}
			}
			if (bit_is_clear(PINA, 3)) //Check is the button is pressed
			{
				if (Pressed == 0)
				{
					//this program executes when button is clicked
					//here is the condition for the bluetooth to start receiving data
					if(allow_uart==0)
					{
						//Send_An_Integer(nth_minute_in_eeprom());
						/*Send_A_Command(0x01);
						GotoMrLCDsLocation(1,1);
						Send_An_Integer(lcd_and_buzzer_info_extract(0,125));
						GotoMrLCDsLocation(1,2);
						Send_An_Integer(lcd_and_buzzer_info_extract(1,125));*/
						/*if(!lcd_buzzer_info_execution(11))
						{
							GotoMrLCDsLocation(1,2);
							Send_A_String("    worked");
						}*/
						clear_alarm_blocks_in_EEPROM();
						_delay_ms(2000);
						check_if_all_alarm_block_in_EEPROM_is_clear_using_lcd();
						_delay_ms(2000);
						allow_uart=1;
						TIMSK &= ~(1<<OCIE1A);
					}
					Pressed = 1;
				}
			}
			else
			{
				//This code executes when the button is not pressed.
				Pressed = 0;
			}
		}
	}
}
//#######################################################################---------END---------####################################################################

//---------------------------------------------------------------------interrupt_funtion_starts-------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
	second_count++;
	if(second_count==02)
	{
		second_count=0;
		min_occured=1;
	}
}
//----------------------------------------------------------------------interrupt_function_ends-------------------------------------------------------------------


