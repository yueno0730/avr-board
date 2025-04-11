# Makefile for ATmegaBOOT for ATmega168p/ATmega328
# by Yuji Ueno
#
# Instructions
#
# To make bootloader .hex file:
# make piramp8
# make piramp16
# make usbio8
# etc...
#
# To burn bootloader .hex file:
# make piramp8_isp
# make usbio8_isp
# etc...

# program name should not be changed...
PROGRAM    = ATmegaBOOT

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
#ISPTOOL	   = usbasp
ISPPORT	   = com1
#ISPPORT	   = com3
#ISPPORT	   = usb
#ISPSPEED   = -b 19200
ISPSPEED   = -b 115200

MCU_TARGET = atmega168p
LDSECTION  = --section-start=.text=0x3800

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-e -u -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x2f:m

STK500 = "C:\WinApl\develop\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2

DEFS       = 
LIBS       = 

CC         = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
#override LDFLAGS       = -Wl,$(LDSECTION)
override LDFLAGS       = -Wl,-Map=$(PROGRAM)_$(TARGET).map,$(LDSECTION)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

all:

piramp8: TARGET = piramp_8mhz
piramp8:
piramp8: MCU_TARGET = atmega168p
piramp8: LDSECTION  = --section-start=.text=0x3800
piramp8: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>2' '-DNUM_LED_FLASHES=4'
piramp8: AVR_FREQ = 8000000L 
piramp8: $(PROGRAM)_168P_piramp_8mhz.hex

piramp8_isp: piramp8
piramp8_isp: TARGET = piramp_8mhz
piramp8_isp: MCU_TARGET = atmega168p
piramp8_isp: LFUSE = C2
piramp8_isp: HFUSE = DF
piramp8_isp: EFUSE = F8
piramp8_isp: isp

piramp16: TARGET = piramp_16mhz
piramp16: MCU_TARGET = atmega168p
piramp16: LDSECTION  = --section-start=.text=0x3800
piramp16: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=4'
piramp16: AVR_FREQ = 16000000L 
piramp16: $(PROGRAM)_168P_piramp_16mhz.hex

piramp16_isp: piramp16
piramp16_isp: TARGET = piramp_16mhz
piramp16_isp: MCU_TARGET = atmega168p
piramp16_isp: LFUSE = CF
piramp16_isp: HFUSE = DF
piramp16_isp: EFUSE = F8
piramp16_isp: isp

usbio8: TARGET = usbio_8mhz
usbio8: MCU_TARGET = atmega328
usbio8: LDSECTION  = --section-start=.text=0x7000
usbio8: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>2' '-DNUM_LED_FLASHES=4'
usbio8: AVR_FREQ = 8000000L 
usbio8: $(PROGRAM)_328_usbio_8mhz.hex

usbio8_isp: usbio8
usbio8_isp: TARGET = usbio_8mhz
usbio8_isp: MCU_TARGET = atmega328
usbio8_isp: LFUSE = C2
usbio8_isp: HFUSE = D8
usbio8_isp: EFUSE = F8
usbio8_isp: isp

usbio16: TARGET = usbio_16mhz
usbio16: MCU_TARGET = atmega328
usbio16: LDSECTION  = --section-start=.text=0x7000
usbio16: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=4'
usbio16: AVR_FREQ = 16000000L 
usbio16: $(PROGRAM)_328_usbio_16mhz.hex

usbio16_isp: usbio16
usbio16_isp: TARGET = usbio_16mhz
usbio16_isp: LFUSE = CF
usbio16_isp: HFUSE = D8
usbio16_isp: EFUSE = F8
usbio16_isp: isp

isp: $(TARGET)
	$(ISPFUSES)
	$(ISPFLASH)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(SIZE) $@

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
