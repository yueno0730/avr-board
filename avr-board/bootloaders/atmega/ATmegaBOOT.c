/*!	@file	ATmegaBOOT.c
	@brief	Serial Bootloader for Atmel megaAVR CPU
			ATmega168pからATmega328のサポートを追加
			多分他のATmegaシリーズでも動くと思う
			
	@author	Yuji Ueno
	@date	2012.07.03  1.0.0
								- Initial Version                         Y.UENO
			2013.03.21	1.0.1
								- ATmega328に対応                         Y.UENO
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// 現行avr-libcのeeprom関連関数は，ATmega168をまだサポートしていない？
// 代わりに自前のeeprom write/read関数を使用する
#if !defined(__AVR_ATmega168__) || !defined(__AVR_ATmega168P__) || !defined(__AVR_ATmega328__) || !defined(__AVR_ATmega328P__)
#include <avr/eeprom.h>
#endif

#if !defined(EEWE) && defined(EEPE)
# define EEWE EEPE
#endif

#if !defined(EEMWE) && defined(EEMPE)
# define EEMWE EEMPE
#endif

// F_CPU（CPU Clock）はMakefileで指定する

// bootloaderがユーザーProgramにジャンプするまでの時間
#define MAX_ERROR_COUNT 5

// UARTのボーレート設定
// ボーレート=19200
#ifndef BAUD_RATE
#define BAUD_RATE   19200
#endif

// AVR Studio用にバージョン番号を用意する
#define HW_VER	 0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x10

// CPUによって端子の使い方が異なる。
// ATmega128は2個のUARTを持っていて，どちらを使うか2ピンにより選択できる。
// ATmega1280は4個のUARTを持っている。Arduinoに使う場合はRXD0しか使えない・・・ BL0がUART0，BL1がUART1
#ifdef __AVR_ATmega128__
#define BL_DDR  DDRF
#define BL_PORT PORTF
#define BL_PIN  PINF
#define BL0     PINF7
#define BL1     PINF6
#elif defined __AVR_ATmega1280__
#else
// 他のATmegaは1個のUARTを搭載。bootloaderでは1ポートの定義のみ
#define BL_DDR  DDRD
#define BL_PORT PORTD
#define BL_PIN  PIND
#define BL      PIND6
#endif

// オンボードLEDを表示用に使用。bootloaderが起動したら3回点滅する
#if defined __AVR_ATmega128__ || defined __AVR_ATmega1280__
// オンボードLEDは，PB7
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB7
#elif defined __AVR_ATmega328__
// ATmega328のボード（USB IO PCB）のオンボードLEDは，PC5
#define LED_DDR  DDRC
#define LED_PORT PORTC
#define LED_PIN  PINC
#define LED      PINC5
#else
// オンボードLEDは，PB0
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB0
#endif

// モニタ機能を搭載フラグ
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
#define MONITOR 1
#endif

// デバイスIDの定義
// 他のアプリからAVRのライターとして認識してもらうのに必要になる
#define SIG1	0x1E	// ATMELのmanufacturer ID（これ以外は無いけど・・・）

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


// プロトタイプ宣言
void putch(char);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);
char gethex(void);
void puthex(char);
void flash_led(uint8_t);

// 変数の定義
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
@brief	mainプログラム
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

	// WDTによるリセットの場合，bootloaderをスキップしてユーザモードで起動する
	if (! (ch &  _BV(EXTRF)))		// 外部リセットではない場合...
		app_start();				// bootloaderをスキップする
#else
	asm volatile("nop\n\t");
#endif

	// 使用する端子の方向とプルアップを有効にする
	// ATmega128は，2個の端子を初期化する必要がある
#ifdef __AVR_ATmega128__
	BL_DDR &= ~_BV(BL0);
	BL_DDR &= ~_BV(BL1);
	BL_PORT |= _BV(BL0);
	BL_PORT |= _BV(BL1);
#else
	// この端子にかかわらずbootloaderを起動するようにする
	/*
	BL_DDR &= ~_BV(BL);
	BL_PORT |= _BV(BL);
	*/
#endif


#ifdef __AVR_ATmega128__
	// どのUARTを使うかの選択（ピンの状態で指定できる）
	if(bit_is_clear(BL_PIN, BL0)) {
		bootuart = 1;
	} else if(bit_is_clear(BL_PIN, BL1)) {
		bootuart = 2;
	}
#endif

#if defined __AVR_ATmega1280__
	// ATmega1280は4個のUARTを持っている
	// ここで指定する
	bootuart = 1;
#endif

	// Flash ROMにユーザプログラムが書き込まれているか確認する。書かれている場合はユーザプログラムにジャンプする
	if(pgm_read_byte_near(0x0000) != 0xFF) {
		
#ifdef __AVR_ATmega128__
		// UARTが選択されていない場合，ユーザプログラムにジャンプ
		if(!bootuart) {
			app_start();
		}
#else
		// bootloader端子="Low"かチェック
		// ATmega168pではこの機能は使わないことにする
		//if(bit_is_set(BL_PIN, BL)) {
		//	app_start();
		//}
#endif
	}

#ifdef __AVR_ATmega128__
	// bootuartが設定されていない場合は，UART0を使う
	if(!bootuart) {
		bootuart = 1;
	}
#endif


	// UARTの初期化（CPUによって多少レジスタが違う）
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
	
	// D0(RX)を内部プルアップ機能をイネーブルにする
	DDRD &= ~_BV(PIND0);
	PORTD |= _BV(PIND0);
#elif defined __AVR_ATmega8__	// ATmega8
	UBRRH = (((F_CPU/BAUD_RATE)/16)-1)>>8;		// baudrateの設定
	UBRRL = (((F_CPU/BAUD_RATE)/16)-1);
	UCSRB = (1<<RXEN)|(1<<TXEN);				// Rx & Txをイネーブル
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);	// 8N1に設定
#else							// ATmega16,32,169,8515,8535
	UBRRL = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRH = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRC = 0x06;
	UCSRB = _BV(TXEN)|_BV(RXEN);
#endif

#if defined __AVR_ATmega1280__
	// D0(RX)を内部プルアップ機能をイネーブルにする
	DDRE &= ~_BV(PINE0);
	PORTE |= _BV(PINE0);
#endif

	// LEDに使うポートを出力に設定
	LED_DDR |= _BV(LED);

	// bootloaderが起動したら，オンボードLEDを点滅させる
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
	flash_led(NUM_LED_FLASHES + bootuart);			// 4回=UART0, 5回=UART1 (ATmega128/1280)
#else
	flash_led(NUM_LED_FLASHES);
#endif

	// 調整
	//putch('\0');
	
	// 無限ループ処理
	for (;;) {
		
	ch = getch();													// UARTから1文字読む
	

	if(ch=='0') {													// Hello・・・
		nothing_response();
	}
	
	// programmer IDのリクエスト
	// m128は64Kbを超えてのboot blockは使用しない
	// RAMPZの操作が必要になる。
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


	// AVR ISP/STK500のコマンド
	// 何もしない
	else if(ch=='@') {
		ch2 = getch();
		if (ch2>0x85) getch();
		nothing_response();
	}
	
	// バージョン情報のリクエスト
	else if(ch=='A') {
		ch2 = getch();
		if(ch2==0x80) byte_response(HW_VER);						// Hardware version
		else if(ch2==0x81) byte_response(SW_MAJOR);					// Software major version
		else if(ch2==0x82) byte_response(SW_MINOR);					// Software minor version
		else if(ch2==0x98) byte_response(0x03);						// 内容不明。avr studio 3.56はこれが必要
		else byte_response(0x00);									// その他は，何もしない
	}
	
	// デバイスパラメータ
	else if(ch=='B') {
		getNch(20);
		nothing_response();											// 何もしない
	}
	
	// パラレル書き込み関連
	else if(ch=='E') {
		getNch(5);
		nothing_response();											// 何もしない
	}

	// P: 書き込み開始
	// R: イレースデバイス
	else if(ch=='P' || ch=='R') {
		nothing_response();											// 何もしない
	}
	
	// 書き込みモードから出る
	else if(ch=='Q') {
		nothing_response();											// 何もしない
#ifdef WATCHDOG_MODS
		// watchdogを使ってリセットする
		WDTCSR = _BV(WDE);
		while (1);													// 16 msかかるかな？
#endif
	}
	
	// アドレス設定，リトルエンディアン。EEPROMバイト，FLASHはワード
	// 128KB以上のFLASHには将来対応
	else if(ch=='U') {
		address.byte[0] = getch();									// アドレス設定
		address.byte[1] = getch();									//
		nothing_response();
	}
	
	// ユニバーサルSPI書き込みコマンドは使わない。fusesとlockビットのみ対応する
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
	
	// メモリライトコマンド
	else if(ch=='d') {
		length.byte[1] = getch();
		length.byte[0] = getch();
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1;
		for (w=0;w<length.word;w++) {
			buff[w] = getch();										// データをバッファに入れる
		}
		if (getch() == ' ') {
			if (flags.eeprom) {										// EEPROM書き込みは，1回に1バイトのみ
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
			else {													// FLASHへの書き込みは1ページごと
				if (address.byte[1]>127) address_high = 0x01;		// ATmega128のみ可，ATmega256は3rd address byteが必要
				else address_high = 0x00;
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				RAMPZ = address_high;								// バンクを切り替える
#endif
				address.word = address.word << 1;					// address * 2 -> byte location
				if ((length.byte[0] & 0x01)) length.word++;			// 奇数バイトの場合，端数を切り上げる
				cli();												// 割り込み止める（安全のため）
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				while(bit_is_set(EECR,EEPE));						// EEPROM書き込み完了するまで待つ
#else
				while(bit_is_set(EECR,EEWE));						// EEPROM書き込み完了するまで待つ
#endif
				asm volatile(
					 "clr	r17		\n\t"							// ページ
					 "lds	r30,address	\n\t"						// FLASH Address位置(bytes)
					 "lds	r31,address+1	\n\t"
					 "ldi	r28,lo8(buff)	\n\t"					// バッファ(RAM)の開始位置
					 "ldi	r29,hi8(buff)	\n\t"
					 "lds	r24,length	\n\t"						// 書き込みサイズ(bytes)
					 "lds	r25,length+1	\n\t"
					 "length_loop:		\n\t"						// ループ，ブロックごとリピート 
					 "cpi	r17,0x00	\n\t"						// もし ページカウント=0 なら イレース
					 "brne	no_page_erase	\n\t" 
					 "wait_spm1:		\n\t"
					 "lds	r16,%0		\n\t"						// SPMが完了するまで待つ
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm1       \n\t"
					"ldi	r16,0x03	\n\t"						// イレースページ(Z)
					 "sts	%0,r16		\n\t"
					 "spm			\n\t" 
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "wait_spm2:		\n\t"
					 "lds	r16,%0		\n\t"						// SPMが完了するまで待つ
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm2       \n\t" 

					 "ldi	r16,0x11	\n\t"						// RWWセクションを再enable
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "no_page_erase:		\n\t"
					 "ld	r0,Y+		\n\t"						// ページバッファの2バイト書く
					 "ld	r1,Y+		\n\t"
								 
					 "wait_spm3:		\n\t"
					 "lds	r16,%0		\n\t"						// SPMが完了するまで待つ
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm3       \n\t"
					 "ldi	r16,0x01	\n\t"						// FLASHページバッファのr0,r1をロード
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
								 
					 "inc	r17		\n\t"							// page_word_count++
					 "cpi r17,%1	        \n\t"
					 "brlo	same_page	\n\t"						// 同じページ(FLASH)？
					 "write_page:		\n\t"
					 "clr	r17		\n\t"							// 新しいページ，最初に現在値を書く
					 "wait_spm4:		\n\t"
					 "lds	r16,%0		\n\t"						// SPMが完了するまで待つ
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm4       \n\t"
#ifdef __AVR_ATmega163__
					 "andi	r30,0x80	\n\t"						// ATmega163は，ページ書き込み時にZ6:Z1を0にしないとダメ
#endif							 							 
					 "ldi	r16,0x05	\n\t"						// Write page pointed to by Z
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
					 "ori	r30,0x7E	\n\t"						// ページ書き込み後（書込み中は0），Z6:Z1ステートを戻す
#endif
					 "wait_spm5:		\n\t"
					 "lds	r16,%0		\n\t"						// 前回のspmが終了するのを待つ
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm5       \n\t"
					 "ldi	r16,0x11	\n\t"						// RWWセクションを再enable
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "same_page:		\n\t"
					 "adiw	r30,2		\n\t"						// FLASHの次のword
					 "sbiw	r24,2		\n\t"						// length-2
					 "breq	final_write	\n\t"						// 終了
					 "rjmp	length_loop	\n\t"
					 "final_write:		\n\t"
					 "cpi	r17,0		\n\t"
					 "breq	block_done	\n\t"
					 "adiw	r24,2		\n\t"						// length+2, ページライト後長さのチェックを騙す
					 "rjmp	write_page	\n\t"
					 "block_done:		\n\t"
					 "clr	__zero_reg__	\n\t"					// zeroレジスタを戻す
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
	
	// リードメモリブロック（長さはビッグエンディアン）
	else if(ch=='t') {
		length.byte[1] = getch();
		length.byte[0] = getch();
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
		if (address.word>0x7FFF) flags.rampz = 1;					// ATmega256は，何もしない（将来変更予定）
		else flags.rampz = 0;
#endif
		address.word = address.word << 1;							// address * 2 -> byte location
		if (getch() == 'E') flags.eeprom = 1;
		else flags.eeprom = 0;
		if (getch() == ' ') {										// コマンド終了?
			putch(0x14);
			for (w=0;w < length.word;w++) {							// oddとevenをハンドル
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
	
	// デバイスシグネチャのリード
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
	
	// OSCキャリブレーションバイトのリード
	else if(ch=='v') {
		byte_response(0x00);
	}


#if defined MONITOR 
	// 拡張モニタ
    
	// 3回のビックリマークのチェック
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
			
			// LEDをON
			LED_DDR |= _BV(LED);
			LED_PORT &= ~_BV(LED);

			// welcomeメッセージを出力
			for(i=0; welcome[i] != '\0'; ++i) {
				putch(welcome[i]);
			}
			
			// 有効なコマンドのチェック
			for(;;) {
				putch('\n');
				putch('\r');
				putch(':');
				putch(' ');
				
				ch = getch();
				putch(ch);
				
				// LEDをトグル
				if(ch == 't') {
					if(bit_is_set(LED_PIN,LED)) {
						LED_PORT &= ~_BV(LED);
						putch('1');
					} else {
						LED_PORT |= _BV(LED);
						putch('0');
					}
				} 
				
				// アドレス指定されたデータをリード
				else if(ch == 'r') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					putch('=');
					ch = *(uint8_t *)((addrh << 8) + addrl);
					puthex(ch);
				}

				// 指定されたアドレスにライト
				else if(ch == 'w') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					ch = getch(); putch(ch);
					ch = gethex();
					*(uint8_t *)((addrh << 8) + addrl) = ch;
				}
				
				// UARTからリード（echoバック）
				else if(ch == 'u') {
					for(;;) {
						putch(getch());
					}
				}
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
				// 拡張bus loop
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
				
			}	// モニタ終了

		}
		}
	}
#endif
	else if (++error_count == MAX_ERROR_COUNT) {
		app_start();
	}
	}	// ループ終了

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
			if (count > MAX_TIME_COUNT)								// 入力が何もなければ，自動的にユーザプログラムを起動する
				app_start();
		}
		
		return UDR0;
	}
	else if(bootuart == 2) {
		while(!(UCSR1A & _BV(RXC1))) {
			count++;
			if (count > MAX_TIME_COUNT)								// 入力が何もなければ，自動的にユーザプログラムを起動する
				app_start();
		}
		
		return UDR1;
	}
	return 0;
#elif defined(__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
	unsigned long count = 0;
	
	while(!(UCSR0A & _BV(RXC0))){
		count++;
		if (count > MAX_TIME_COUNT)									// 入力が何もなければ，自動的にユーザプログラムを起動する
			app_start();
	}
	return UDR0;
#else
	// m8,16,32,169,8515,8535,163
	uint32_t count = 0;
	while(!(UCSRA & _BV(RXC))){
		count++;
		if (count > MAX_TIME_COUNT)									// 入力が何もなければ，自動的にユーザプログラムを起動する
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
