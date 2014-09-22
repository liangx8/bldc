.PHONY: clean
CC=avrasm2.exe
CFLAG=-w
TARGET = bldc

all:$(TARGET).hex
	@echo "*** Done ***"
$(TARGET).hex:$(TARGET).asm
	${CC}  $< -l $(TARGET).lst -D debug ${CFLAG} -fI -o $(TARGET).hex -d $(TARGET).obj -e $(TARGET).eep -I . -I "D:\p\mcu_dev\avrasm32\includes" -m $(TARGET).map
clean:

	rm -rf *.hex *.obj *.map *.lst
