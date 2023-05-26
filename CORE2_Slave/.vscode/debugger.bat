set PATH=c:\Users\Dream Machines\.vscode\HusarionTools\bin\;%PATH%
cd c:\Users\Dream Machines\Desktop\VII SEMESTR\Mechatronic design\Projekt\CORE2_Slave || exit 1
start /wait st-flash write myproject.bin 0x08010000 || exit 1
start st-util
arm-none-eabi-gdb %*