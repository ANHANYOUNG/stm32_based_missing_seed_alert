# Planter Miss Detection System

Detects missing seed drops during planting and shows the status on an HMI display.

## Files
- `Altium/` : schematic and PCB design files
- `MCU/` : STM32 firmware (STM32CubeIDE)
- `Nextion HMI/` : display UI file (.HMI)

## Environment
- MCU: STM32F411RET6
- IDE: STM32CubeIDE (HAL)
- HW Design: Altium Designer
- Display: Nextion HMI

## Build
1. Manufacture the PCB from files in `Altium/`.
2. Upload `prox_en_v2.HMI` to the display using Nextion Editor.
3. Open `MCU/f411re/` in STM32CubeIDE, build, and flash to the board.
