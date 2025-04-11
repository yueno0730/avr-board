/*!	@file	ATmegaBOOT.c
	@brief	Serial Bootloader for Atmel megaAVR CPU
			ATmega168p����ATmega328�̃T�|�[�g��ǉ�
			��������ATmega�V���[�Y�ł������Ǝv��
			
	@author	Yuji Ueno
	@date	2012.07.03  1.0.0
								- Initial Version                         Y.UENO
			2013.03.21	1.0.1
								- ATmega328�ɑΉ�                         Y.UENO
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// ���savr-libc��eeprom�֘A�֐��́CATmega168���܂��T�|�[�g���Ă��Ȃ��H
// ����Ɏ��O��eeprom write/read�֐����g�p����
#if !defined(__AVR_ATmega168__) || !defined(__AVR_ATmega168P__) || !defined(__AVR_ATmega328__) || !defined(__AVR_ATmega328P__)
#include <avr/eeprom.h>
#endif

#if !defined(EEWE) && defined(EEPE)
# define EEWE EEPE
#endif

#if !defined(EEMWE) && defined(EEMPE)
# define EEMWE EEMPE
#endif

// F_CPU�iCPU Clock�j��Makefile�Ŏw�肷��

// bootloader�����[�U�[Program�ɃW�����v����܂ł̎���
#define MAX_ERROR_COUNT 5

// UART�̃{�[���[�g�ݒ�
// �{�[���[�g=19200
#ifndef BAUD_RATE
#define BAUD_RATE   19200
#endif

// AVR Studio�p�Ƀo�[�W�����ԍ���p�ӂ���
#define HW_VER	 0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x10

// CPU�ɂ���Ē[�q�̎g�������قȂ�B
// ATmega128��2��UART�������Ă��āC�ǂ�����g����2�s���ɂ��I���ł���B
// ATmega1280��4��UART�������Ă���BArduino�Ɏg���ꍇ��RXD0�����g���Ȃ��E�E�E BL0��UART0�CBL1��UART1
#ifdef __AVR_ATmega128__
#define BL_DDR  DDRF
#define BL_PORT PORTF
#define BL_PIN  PINF
#define BL0     PINF7
#define BL1     PINF6
#elif defined __AVR_ATmega1280__
#else
// ����ATmega��1��UART�𓋍ځBbootloader�ł�1�|�[�g�̒�`�̂�
#define BL_DDR  DDRD
#define BL_PORT PORTD
#define BL_PIN  PIND
#define BL      PIND6
#endif

// �I���{�[�hLED��\���p�Ɏg�p�Bbootloader���N��������3��_�ł���
#if defined __AVR_ATmega128__ || defined __AVR_ATmega1280__
// �I���{�[�hLED�́CPB7
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB7
#elif defined __AVR_ATmega328__
// ATmega328�̃{�[�h�iUSB IO PCB�j�̃I���{�[�hLED�́CPC5
#define LED_DDR  DDRC
#define LED_PORT PORTC
#define LED_PIN  PINC
#define LED      PINC5
#else
// �I���{�[�hLED�́CPB0
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB0
#endif

// ���j�^�@�\�𓋍ڃt���O
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
#define MONITOR 1
#endif

// �f�o�C�XID�̒�`
// ���̃A�v������AVR�̃��C�^�[�Ƃ��ĔF�����Ă��炤�̂ɕK�v�ɂȂ�
#define SIG1	0x1E	// ATMEL��manufacturer ID�i����ȊO�͖������ǁE�E�E�j

#if defined __AVR_ATmega1280__
#define SIG2	0x97
#define SIG3	0x03
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega1281__
#define SIG2	0x97
#define SIG3	0x04
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega128__
#define SIG2	0x97
#define SIG3	0x02
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega64__
#define SIG2	0x96
#define SIG3	0x02
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega32__
#define SIG2	0x95
#define SIG3	0x02
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega16__
#define SIG2	0x94
#define SIG3	0x03
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega8__
#define SIG2	0x93
#define SIG3	0x07
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega88__
#define SIG2	0x93
#define SIG3	0x0a
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega168__
#define SIG2	0x94
#define SIG3	0x06
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega168P__
#define SIG2	0x94
#define SIG3	0x0B
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega328__
#define SIG2	0x95
#define SIG3	0x14
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega328P__
#define SIG2	0x95
#define SIG3	0x0F
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega162__
#define SIG2	0x94
#define SIG3	0x04
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega163__
#define SIG2	0x94
#define SIG3	0x02
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega169__
#define SIG2	0x94
#define SIG3	0x05
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega8515__
#define SIG2	0x93
#define SIG3	0x06
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega8535__
#define SIG2	0x93
#define SIG3	0x08
#define PAGE_SIZE	0x20U	//32 words
#endif


// �v���g�^�C�v�錾
void putch(char);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);
char gethex(void);
void puthex(char);
void flash_led(uint8_t);

// �ϐ��̒�`
union address_union {
	uint16_t word;
	uint8_t  byte[2];
} address;

union length_union {
	uint16_t word;
	uint8_t  byte[2];
} length;

struct flags_struct {
	unsigned eeprom : 1;
	unsigned rampz  : 1;
} flags;

uint8_t buff[256];
uint8_t address_high;

uint8_t pagesz=0x80;

uint8_t i;
uint8_t bootuart = 0;

uint8_t error_count = 0;

void (*app_start)(void) = 0x0000;


/*!
@brief	main�v���O����
@param	void
@return	
*/
int main(void)
{
	uint8_t ch,ch2;
	uint16_t w;

#ifdef WATCHDOG_MODS
	ch = MCUSR;
	MCUSR = 0;

	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;

	// WDT�ɂ�郊�Z�b�g�̏ꍇ�Cbootloader���X�L�b�v���ă��[�U���[�h�ŋN������
	if (! (ch &  _BV(EXTRF)))		// �O�����Z�b�g�ł͂Ȃ��ꍇ...
		app_start();				// bootloader���X�L�b�v����
#else
	asm volatile("nop\n\t");
#endif

	// �g�p����[�q�̕����ƃv���A�b�v��L���ɂ���
	// ATmega128�́C2�̒[�q������������K�v������
#ifdef __AVR_ATmega128__
	BL_DDR &= ~_BV(BL0);
	BL_DDR &= ~_BV(BL1);
	BL_PORT |= _BV(BL0);
	BL_PORT |= _BV(BL1);
#else
	// ���̒[�q�ɂ�����炸bootloader���N������悤�ɂ���
	/*
	BL_DDR &= ~_BV(BL);
	BL_PORT |= _BV(BL);
	*/
#endif


#ifdef __AVR_ATmega128__
	// �ǂ�UART���g�����̑I���i�s���̏�ԂŎw��ł���j
	if(bit_is_clear(BL_PIN, BL0)) {
		bootuart = 1;
	} else if(bit_is_clear(BL_PIN, BL1)) {
		bootuart = 2;
	}
#endif

#if defined __AVR_ATmega1280__
	// ATmega1280��4��UART�������Ă���
	// �����Ŏw�肷��
	bootuart = 1;
#endif

	// Flash ROM�Ƀ��[�U�v���O�������������܂�Ă��邩�m�F����B������Ă���ꍇ�̓��[�U�v���O�����ɃW�����v����
	if(pgm_read_byte_near(0x0000) != 0xFF) {
		
#ifdef __AVR_ATmega128__
		// UART���I������Ă��Ȃ��ꍇ�C���[�U�v���O�����ɃW�����v
		if(!bootuart) {
			app_start();
		}
#else
		// bootloader�[�q="Low"���`�F�b�N
		// ATmega168p�ł͂��̋@�\�͎g��Ȃ����Ƃɂ���
		//if(bit_is_set(BL_PIN, BL)) {
		//	app_start();
		//}
#endif
	}

#ifdef __AVR_ATmega128__
	// bootuart���ݒ肳��Ă��Ȃ��ꍇ�́CUART0���g��
	if(!bootuart) {
		bootuart = 1;
	}
#endif


	// UART�̏������iCPU�ɂ���đ������W�X�^���Ⴄ�j
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	if(bootuart == 1) {
		UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
		UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
		UCSR0A = 0x00;
		UCSR0C = 0x06;
		UCSR0B = _BV(TXEN0)|_BV(RXEN0);
	}
	if(bootuart == 2) {
		UBRR1L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
		UBRR1H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
		UCSR1A = 0x00;
		UCSR1C = 0x06;
		UCSR1B = _BV(TXEN1)|_BV(RXEN1);
	}
#elif defined __AVR_ATmega163__
	UBRR = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRHI = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRB = _BV(TXEN)|_BV(RXEN);	
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)

#ifdef DOUBLE_SPEED
	UCSR0A = (1<<U2X0);		// Double speed mode UART0
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
#else
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
#endif
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	// D0(RX)������v���A�b�v�@�\���C�l�[�u���ɂ���
	DDRD &= ~_BV(PIND0);
	PORTD |= _BV(PIND0);
#elif defined __AVR_ATmega8__	// ATmega8
	UBRRH = (((F_CPU/BAUD_RATE)/16)-1)>>8;		// baudrate�̐ݒ�
	UBRRL = (((F_CPU/BAUD_RATE)/16)-1);
	UCSRB = (1<<RXEN)|(1<<TXEN);				// Rx & Tx���C�l�[�u��
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);	// 8N1�ɐݒ�
#else							// ATmega16,32,169,8515,8535
	UBRRL = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRH = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRC = 0x06;
	UCSRB = _BV(TXEN)|_BV(RXEN);
#endif

#if defined __AVR_ATmega1280__
	// D0(RX)������v���A�b�v�@�\���C�l�[�u���ɂ���
	DDRE &= ~_BV(PINE0);
	PORTE |= _BV(PINE0);
#endif

	// LED�Ɏg���|�[�g���o�͂ɐݒ�
	LED_DDR |= _BV(LED);

	// bootloader���N��������C�I���{�[�hLED��_�ł�����
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	flash_led(NUM_LED_FLASHES + bootuart);			// 4��=UART0, 5��=UART1 (ATmega128/1280)
#else
	flash_led(NUM_LED_FLASHES);
#endif

	// ����
	//putch('\0');
	
	// �������[�v����
	for (;;) {
		
	ch = getch();													// UART����1�����ǂ�
	

	if(ch=='0') {													// Hello�E�E�E
		nothing_response();
	}
	
	// programmer ID�̃��N�G�X�g
	// m128��64Kb�𒴂��Ă�boot block�͎g�p���Ȃ�
	// RAMPZ�̑��삪�K�v�ɂȂ�B
	else if(ch=='1') {
		if (getch() == ' ') {
			putch(0x14);
			putch('A');
			putch('V');
			putch('R');
			putch(' ');
			putch('I');
			putch('S');
			putch('P');
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}


	// AVR ISP/STK500�̃R�}���h
	// �������Ȃ�
	else if(ch=='@') {
		ch2 = getch();
		if (ch2>0x85) getch();
		nothing_response();
	}
	
	// �o�[�W�������̃��N�G�X�g
	else if(ch=='A') {
		ch2 = getch();
		if(ch2==0x80) byte_response(HW_VER);						// Hardware version
		else if(ch2==0x81) byte_response(SW_MAJOR);					// Software major version
		else if(ch2==0x82) byte_response(SW_MINOR);					// Software minor version
		else if(ch2==0x98) byte_response(0x03);						// ���e�s���Bavr studio 3.56�͂��ꂪ�K�v
		else byte_response(0x00);									// ���̑��́C�������Ȃ�
	}
	
	// �f�o�C�X�p�����[�^
	else if(ch=='B') {
		getNch(20);
		nothing_response();											// �������Ȃ�
	}
	
	// �p�������������݊֘A
	else if(ch=='E') {
		getNch(5);
		nothing_response();											// �������Ȃ�
	}

	// P: �������݊J�n
	// R: �C���[�X�f�o�C�X
	else if(ch=='P' || ch=='R') {
		nothing_response();											// �������Ȃ�
	}
	
	// �������݃��[�h����o��
	else if(ch=='Q') {
		nothing_response();											// �������Ȃ�
#ifdef WATCHDOG_MODS
		// watchdog���g���ă��Z�b�g����
		WDTCSR = _BV(WDE);
		while (1);													// 16 ms�����邩�ȁH
#endif
	}
	
	// �A�h���X�ݒ�C���g���G���f�B�A���BEEPROM�o�C�g�CFLASH�̓��[�h
	// 128KB�ȏ��FLASH�ɂ͏����Ή�
	else if(ch=='U') {
		address.byte[0] = getch();									// �A�h���X�ݒ�
		address.byte[1] = getch();									//
		nothing_response();
	}
	
	// ���j�o�[�T��SPI�������݃R�}���h�͎g��Ȃ��Bfuses��lock�r�b�g�̂ݑΉ�����
	else if(ch=='V') {
		if (getch() == 0x30) {
			getch();
			ch = getch();
			getch();
			if (ch == 0) {
				byte_response(SIG1);
			} else if (ch == 1) {
				byte_response(SIG2); 
			} else {
				byte_response(SIG3);
			} 
		} else {
			getNch(3);
			byte_response(0x00);
		}
	}
	
	// ���������C�g�R�}���h
	else if(ch=='d') {
		length.byte[1] = getch();
		length.byte[0] = getch();
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1;
		for (w=0;w<length.word;w++) {
			buff[w] = getch();										// �f�[�^���o�b�t�@�ɓ����
		}
		if (getch() == ' ') {
			if (flags.eeprom) {										// EEPROM�������݂́C1���1�o�C�g�̂�
				address.word <<= 1;
				for(w=0;w<length.word;w++) {
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EEDR = buff[w];
					EECR |= (1<<EEMPE);
					EECR |= (1<<EEPE);
#else
					eeprom_write_byte((void *)address.word,buff[w]);
#endif
					address.word++;
				}			
			}
			else {													// FLASH�ւ̏������݂�1�y�[�W����
				if (address.byte[1]>127) address_high = 0x01;		// ATmega128�̂݉CATmega256��3rd address byte���K�v
				else address_high = 0x00;
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				RAMPZ = address_high;								// �o���N��؂�ւ���
#endif
				address.word = address.word << 1;					// address * 2 -> byte location
				if ((length.byte[0] & 0x01)) length.word++;			// ��o�C�g�̏ꍇ�C�[����؂�グ��
				cli();												// ���荞�ݎ~�߂�i���S�̂��߁j
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				while(bit_is_set(EECR,EEPE));						// EEPROM�������݊�������܂ő҂�
#else
				while(bit_is_set(EECR,EEWE));						// EEPROM�������݊�������܂ő҂�
#endif
				asm volatile(
					 "clr	r17		\n\t"							// �y�[�W
					 "lds	r30,address	\n\t"						// FLASH Address�ʒu(bytes)
					 "lds	r31,address+1	\n\t"
					 "ldi	r28,lo8(buff)	\n\t"					// �o�b�t�@(RAM)�̊J�n�ʒu
					 "ldi	r29,hi8(buff)	\n\t"
					 "lds	r24,length	\n\t"						// �������݃T�C�Y(bytes)
					 "lds	r25,length+1	\n\t"
					 "length_loop:		\n\t"						// ���[�v�C�u���b�N���ƃ��s�[�g 
					 "cpi	r17,0x00	\n\t"						// ���� �y�[�W�J�E���g=0 �Ȃ� �C���[�X
					 "brne	no_page_erase	\n\t" 
					 "wait_spm1:		\n\t"
					 "lds	r16,%0		\n\t"						// SPM����������܂ő҂�
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm1       \n\t"
					"ldi	r16,0x03	\n\t"						// �C���[�X�y�[�W(Z)
					 "sts	%0,r16		\n\t"
					 "spm			\n\t" 
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "wait_spm2:		\n\t"
					 "lds	r16,%0		\n\t"						// SPM����������܂ő҂�
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm2       \n\t" 

					 "ldi	r16,0x11	\n\t"						// RWW�Z�N�V��������enable
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "no_page_erase:		\n\t"
					 "ld	r0,Y+		\n\t"						// �y�[�W�o�b�t�@��2�o�C�g����
					 "ld	r1,Y+		\n\t"
								 
					 "wait_spm3:		\n\t"
					 "lds	r16,%0		\n\t"						// SPM����������܂ő҂�
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm3       \n\t"
					 "ldi	r16,0x01	\n\t"						// FLASH�y�[�W�o�b�t�@��r0,r1�����[�h
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
								 
					 "inc	r17		\n\t"							// page_word_count++
					 "cpi r17,%1	        \n\t"
					 "brlo	same_page	\n\t"						// �����y�[�W(FLASH)�H
					 "write_page:		\n\t"
					 "clr	r17		\n\t"							// �V�����y�[�W�C�ŏ��Ɍ��ݒl������
					 "wait_spm4:		\n\t"
					 "lds	r16,%0		\n\t"						// SPM����������܂ő҂�
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm4       \n\t"
#ifdef __AVR_ATmega163__
					 "andi	r30,0x80	\n\t"						// ATmega163�́C�y�[�W�������ݎ���Z6:Z1��0�ɂ��Ȃ��ƃ_��
#endif							 							 
					 "ldi	r16,0x05	\n\t"						// Write page pointed to by Z
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
					 "ori	r30,0x7E	\n\t"						// �y�[�W�������݌�i�����ݒ���0�j�CZ6:Z1�X�e�[�g��߂�
#endif
					 "wait_spm5:		\n\t"
					 "lds	r16,%0		\n\t"						// �O���spm���I������̂�҂�
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm5       \n\t"
					 "ldi	r16,0x11	\n\t"						// RWW�Z�N�V��������enable
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "same_page:		\n\t"
					 "adiw	r30,2		\n\t"						// FLASH�̎���word
					 "sbiw	r24,2		\n\t"						// length-2
					 "breq	final_write	\n\t"						// �I��
					 "rjmp	length_loop	\n\t"
					 "final_write:		\n\t"
					 "cpi	r17,0		\n\t"
					 "breq	block_done	\n\t"
					 "adiw	r24,2		\n\t"						// length+2, �y�[�W���C�g�㒷���̃`�F�b�N���x��
					 "rjmp	write_page	\n\t"
					 "block_done:		\n\t"
					 "clr	__zero_reg__	\n\t"					// zero���W�X�^��߂�
#if defined __AVR_ATmega168__  || __AVR_ATmega168P__ || __AVR_ATmega328__ || __AVR_ATmega328P__ || __AVR_ATmega128__ || __AVR_ATmega1280__ || __AVR_ATmega1281__ 
					 : "=m" (SPMCSR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#else
					 : "=m" (SPMCR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#endif
					 );
			}
			putch(0x14);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}		
	}
	
	// ���[�h�������u���b�N�i�����̓r�b�O�G���f�B�A���j
	else if(ch=='t') {
		length.byte[1] = getch();
		length.byte[0] = getch();
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
		if (address.word>0x7FFF) flags.rampz = 1;					// ATmega256�́C�������Ȃ��i�����ύX�\��j
		else flags.rampz = 0;
#endif
		address.word = address.word << 1;							// address * 2 -> byte location
		if (getch() == 'E') flags.eeprom = 1;
		else flags.eeprom = 0;
		if (getch() == ' ') {										// �R�}���h�I��?
			putch(0x14);
			for (w=0;w < length.word;w++) {							// odd��even���n���h��
				if (flags.eeprom) {									// read EEPROM?
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EECR |= (1<<EERE);
					putch(EEDR);
#else
					putch(eeprom_read_byte((void *)address.word));
#endif
					address.word++;
				}
				else {

					if (!flags.rampz) putch(pgm_read_byte_near(address.word));
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
					else putch(pgm_read_byte_far(address.word + 0x10000));
#endif
					address.word++;
				}
			}
			putch(0x10);
		}
	}
	
	// �f�o�C�X�V�O�l�`���̃��[�h
	else if(ch=='u') {
		if (getch() == ' ') {
			putch(0x14);
			putch(SIG1);
			putch(SIG2);
			putch(SIG3);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}
	
	// OSC�L�����u���[�V�����o�C�g�̃��[�h
	else if(ch=='v') {
		byte_response(0x00);
	}


#if defined MONITOR 
	// �g�����j�^
    
	// 3��̃r�b�N���}�[�N�̃`�F�b�N
	else if(ch=='!') {
		ch = getch();
		if(ch=='!') {
		ch = getch();
		if(ch=='!') {
			PGM_P welcome = "";
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
			uint16_t extaddr;
#endif
			uint8_t addrl, addrh;

#ifdef CRUMB128
			welcome = "ATmegaBOOT / Crumb128\n\r";
#elif defined PROBOMEGA128
			welcome = "ATmegaBOOT / PROBOmega128\n\r";
#elif defined SAVVY128
			welcome = "ATmegaBOOT / Savvy128\n\r";
#elif defined __AVR_ATmega1280__ 
			welcome = "ATmegaBOOT / Arduino Mega\n\r";
#endif
			
			// LED��ON
			LED_DDR |= _BV(LED);
			LED_PORT &= ~_BV(LED);

			// welcome���b�Z�[�W���o��
			for(i=0; welcome[i] != '\0'; ++i) {
				putch(welcome[i]);
			}
			
			// �L���ȃR�}���h�̃`�F�b�N
			for(;;) {
				putch('\n');
				putch('\r');
				putch(':');
				putch(' ');
				
				ch = getch();
				putch(ch);
				
				// LED���g�O��
				if(ch == 't') {
					if(bit_is_set(LED_PIN,LED)) {
						LED_PORT &= ~_BV(LED);
						putch('1');
					} else {
						LED_PORT |= _BV(LED);
						putch('0');
					}
				} 
				
				// �A�h���X�w�肳�ꂽ�f�[�^�����[�h
				else if(ch == 'r') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					putch('=');
					ch = *(uint8_t *)((addrh << 8) + addrl);
					puthex(ch);
				}

				// �w�肳�ꂽ�A�h���X�Ƀ��C�g
				else if(ch == 'w') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					ch = getch(); putch(ch);
					ch = gethex();
					*(uint8_t *)((addrh << 8) + addrl) = ch;
				}
				
				// UART���烊�[�h�iecho�o�b�N�j
				else if(ch == 'u') {
					for(;;) {
						putch(getch());
					}
				}
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
				// �g��bus loop
				else if(ch == 'b') {
					putch('b');
					putch('u');
					putch('s');
					MCUCR = 0x80;
					XMCRA = 0;
					XMCRB = 0;
					extaddr = 0x1100;
					for(;;) {
						ch = *(volatile uint8_t *)extaddr;
						if(++extaddr == 0) {
							extaddr = 0x1100;
						}
					}
				}
#endif
				
				else if(ch == 'j') {
					app_start();
				}
				
			}	// ���j�^�I��

		}
		}
	}
#endif
	else if (++error_count == MAX_ERROR_COUNT) {
		app_start();
	}
	}	// ���[�v�I��

}

char gethexnib(void) {
	char a;
	
	a = getch(); putch(a);
	if(a >= 'a') {
		return (a - 'a' + 0x0a);
	} else if(a >= '0') {
		return(a - '0');
	}
	return a;
}

char gethex(void) {
	return (gethexnib() << 4) + gethexnib();
}


void puthex(char ch) {
	char ah;
	
	ah = ch >> 4;
	if(ah >= 0x0a) {
		ah = ah - 0x0a + 'a';
	} else {
		ah += '0';
	}
	
	ch &= 0x0f;
	if(ch >= 0x0a) {
		ch = ch - 0x0a + 'a';
	} else {
		ch += '0';
	}
	
	putch(ah);
	putch(ch);
}

void putch(char ch)
{
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	if(bootuart == 1) {
		while (!(UCSR0A & _BV(UDRE0)));
		UDR0 = ch;
	}
	else if (bootuart == 2) {
		while (!(UCSR1A & _BV(UDRE1)));
		UDR1 = ch;
	}
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = ch;
#else
	// mega8,16,32,169,8515,8535,163
	while (!(UCSRA & _BV(UDRE)));
	UDR = ch;
#endif
}

char getch(void)
{
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	uint32_t count = 0;
	if(bootuart == 1) {
		while(!(UCSR0A & _BV(RXC0))) {
			count++;
			if (count > MAX_TIME_COUNT)								// ���͂������Ȃ���΁C�����I�Ƀ��[�U�v���O�������N������
				app_start();
		}
		
		return UDR0;
	}
	else if(bootuart == 2) {
		while(!(UCSR1A & _BV(RXC1))) {
			count++;
			if (count > MAX_TIME_COUNT)								// ���͂������Ȃ���΁C�����I�Ƀ��[�U�v���O�������N������
				app_start();
		}
		
		return UDR1;
	}
	return 0;
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
	unsigned long count = 0;
	
	while(!(UCSR0A & _BV(RXC0))){
		count++;
		if (count > MAX_TIME_COUNT)									// ���͂������Ȃ���΁C�����I�Ƀ��[�U�v���O�������N������
			app_start();
	}
	return UDR0;
#else
	// m8,16,32,169,8515,8535,163
	uint32_t count = 0;
	while(!(UCSRA & _BV(RXC))){
		count++;
		if (count > MAX_TIME_COUNT)									// ���͂������Ȃ���΁C�����I�Ƀ��[�U�v���O�������N������
			app_start();
	}
	return UDR;
#endif
}

void getNch(uint8_t count)
{
	while(count--) {
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
		if(bootuart == 1) {
			while(!(UCSR0A & _BV(RXC0)));
			UDR0;
		} 
		else if(bootuart == 2) {
			while(!(UCSR1A & _BV(RXC1)));
			UDR1;
		}
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
		getch();
#else
		// mega8,16,32,169,8515,8535,163
		//while(!(UCSRA & _BV(RXC)));
		//UDR;
		getch();
#endif		
	}
}


void byte_response(uint8_t val)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(val);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}


void nothing_response(void)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}

void flash_led(uint8_t count)
{
	while (count--) {
		LED_PORT |= _BV(LED);
		_delay_ms(100);
		LED_PORT &= ~_BV(LED);
		_delay_ms(100);
	}
}
