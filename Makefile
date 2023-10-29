all:
	python Scripts/fast_chip_select_hack.py --cube_dir Cube-files
	$(MAKE) -C Cube-files/ all

clean:
	$(MAKE) -C Cube-files/ clean

flash:
	STM32_Programmer_CLI -c port=SWD -w Cube-files/build/Cube-files.bin 0x08000000 -v -hardRst

build_flash:
	python Scripts/fast_chip_select_hack.py --cube_dir Cube-files
	$(MAKE) -C Cube-files/ all -j
	STM32_Programmer_CLI -c port=SWD -w Cube-files/build/Cube-files.bin 0x08000000 -v -hardRst