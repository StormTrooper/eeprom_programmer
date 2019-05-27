## Arduino EEPROM programmer 

Arduino EEPROM programmer based on https://github.com/bouletmarc/BMBurner

Tested on Arduino IDE 1.8.9 with Arduino Uno

The original sketch has been changed so I don't have a dependency on the Windows 
software to program the EEPROM.



First step is to run the python program to convert the binary file to a include file which is then used 
in the Arduino sketch. I'm using the Arduino Uno which has 32k of program storage. 

`python convert.py inputfile`

This will create a bin_file.h with the correct formatting to be used in the Arduino sketch
`#include "bin_file.h"`

Compile and upload to the Arduino. 
I've added in a menu system where you select the chip, start address and block sizes using the Serial monitor. 

This is still a work in progress. The original BMBurner code only allows you to read most of the chips, however with some hardware and code modification you should be able to read/write most of the chips.

This is still a work in progress and so far I have only tested the W27C512 and 27C256

