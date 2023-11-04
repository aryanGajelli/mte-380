from pathlib import Path
import argparse

parser = argparse.ArgumentParser(description='Fast Chip Select Hack')
parser.add_argument('--cube_dir', type=Path, help='Path to CubeMX directory')
# check if stm32f4xx_hal_spi.c exists
args = parser.parse_args()

cube_dir: Path = args.cube_dir
cube_dir = cube_dir.resolve()

if not cube_dir.exists():
    raise FileNotFoundError(cube_dir)

stm32f4xx_hal_spi_c = cube_dir / 'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c'
if not stm32f4xx_hal_spi_c.exists():
    raise FileNotFoundError(stm32f4xx_hal_spi_c)

# open stm32f4xx_hal_spi.c
line_to_find = '/* Enable Tx DMA Request */'
fnc_lineno = None
enable_tx_line = []
found = False
with stm32f4xx_hal_spi_c.open('r+') as f:
    content = f.readlines()
    lines = iter(content)
    lineno = 0
    while line:= next(lines, None):
        lineno += 1
        if line_to_find in line:
            enable_tx_line.append((lineno, line))
            found = True

    if found:
        cs_en_line = '\tIMU_CS_GPIO_Port->BSRR = IMU_CS_Pin << 16U;\n'
        include_line = '#include "main.h"\n'
        if include_line in content:
            exit(0)
        
        include_lineno = [i for i, line in enumerate(content, 1) if '#include "stm32f4xx_hal.h"' in line][0]
        for lineno, line in enable_tx_line:
            content.insert(lineno - 1, cs_en_line)
        content.insert(include_lineno, include_line)
        f.seek(0)
        f.writelines(content)
    else:
        raise ValueError('Could not find line to patch')