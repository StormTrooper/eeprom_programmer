## Arduino EEPROM programmer 

Arduino EEPROM programmer based on https://github.com/bouletmarc/BMBurner

Tested on Arduino 1.8.5

The original sketch has been changed so I don't have a dependency on the Windows 
software to program the EEPROM.

This however restricts the programming to 8k which is fine as I'm using it to program 8k chunks of the 
W27C512 used on the OpenC64Cart (Commodore 64 Cartridge) 
See: https://github.com/SukkoPera/OpenC64Cart


First step is to run the python program to convert the 8k cartidge bin file to a include file which is then used 
in the Arduino sketch. I'm using the Arduino Uno which has 32k of program storage. 

`python convert.py inputfile`

This will create a bin_file.h with the correct formatting to be used in the Arduino sketch
`#include "bin_file.h"`

Next, in the Arduino sketch uncomment the line to choose the mode:
`//#define READCOMPARE
//#define BLANK
//#define WRITE
//#define ERASE
`

And select the bank address you want to read/write
`#define Start_Address 0x0000        //0x0000, 0x2000, 0x4000, 0x6000, 0x8000, 0xa000, 0xc000, 0xe000`



 
