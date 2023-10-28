all:
	$(MAKE) -C Cube-files/ all

clean:
	$(MAKE) -C Cube-files/ clean

flash:
	STM32_Programmer_CLI -c port=SWD -w Cube-files/build/Cube-files.bin 0x08000000 -v -hardRst

build_flash:
	$(MAKE) -C Cube-files/ all -j
	STM32_Programmer_CLI -c port=SWD -w Cube-files/build/Cube-files.bin 0x08000000 -v -hardRst