/*
   EEPROM Programmer

   Based on https://github.com/bouletmarc/BMBurner
   and modified to not need the Windows software to program

   Arduino Uno programmer which can read/write/blank check/erase E/EPROMsreads

   27C256 - tested
   Winbond W27C512 (Electrically erasable) - tested
   27C32 - not tested
   27C64 - not tested
   27C128 - not tested
   27C512 - not tested

p
   Data write to the EPROM is contained in the bin_file.h which is generated from a convert.py python file

   For more information see https://blog.gjmccarthy.co.uk

*/


#include "bin_file.h"   //Contents of binary file to be programmed



//#define DEBUG

//Define chips predefinied model numbers (2-5 is for moates compatibility, 4 is never ever as it should be used for 27SF040 which is not compatible with this project at all)
unsigned int chipType;
#define CHIP27C128    1   //Not tested
#define CHIP27C256    2   //Not tested
#define CHIP27C32     3   //INCLUDED : 2732A
#define CHIP27SF512   5   //INCLUDED : 27C512, W27E512, W27C512
#define CHIP28C64     6   //INCLUDED : 27C64
#define CHIP28C256    7


// define the IO lines for the data - bus
#define D0 2
#define D1 3
#define D2 4
#define D3 5
#define D4 6
#define D5 7
#define D6 8
#define D7 10

// for high voltage programming supply
#define VH     11     //Erase Pin (A9)  12v
#define VPP    12     //OE Pin = VPP    12v
#define VPP_27C256  9 // Vpp Pin for 27C256 12V

// shiftOut part
#define DS     A0
#define LATCH  A1
#define CLOCK  A2

// define the IO lines for the eeprom control
#define CE     A3
#define OE     A4
#define A15VPP A5     //A15 on SST, VPP on 27C256
#define A10    13

// direct access to port
#define STROBE_PORT PORTC
#define STROBE_DS      0
#define STROBE_LATCH   1
#define STROBE_CLOCK   2
#define STROBE_CE      3
#define STROBE_OE      4
#define STROBE_WE      5

//a buffer for bytes to burn
#define BUFFERSIZE 256
byte buffer[BUFFERSIZE + 1]; // +1 for checksum

//flag set if we are on the first write pass
boolean firstWritePass = true;

//Last Address (used for sending high/low output at only 1 specific location on the 74HC595
unsigned int Last_Address = 0;

int count = 0;  //Memory size
int mem_count;
int arraySize;
long BlockSize;
String mode;

long Start_Address;         //0x0000, 0x2000, 0x4000, 0x6000, 0x8000, 0xa000, 0xc000, 0xe000

//###############################################################
// Setup
//###############################################################
void setup() {

  Serial.begin(115200);

  Serial.println("eeprom programmer v1.1");
  Serial.println("");

  arraySize = sizeof(bin_file) / sizeof(bin_file[0]);

  //define the shiftOut Pins as output
  pinMode(DS, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);

  //define the boost pins as output (take care that they are LOW)
  digitalWrite(VH, LOW);
  pinMode(VH, OUTPUT);
  digitalWrite(VPP, LOW);
  pinMode(VPP, OUTPUT);
  digitalWrite(VPP_27C256, LOW);
  pinMode(VPP_27C256, OUTPUT);


  //define the EEPROM Pins as output (take care that they are HIGH)
  digitalWrite(OE, HIGH);
  pinMode(OE, OUTPUT);
  digitalWrite(CE, HIGH);
  pinMode(CE, OUTPUT);
  digitalWrite(A15VPP, HIGH);
  pinMode(A15VPP, OUTPUT);
  digitalWrite(A10, HIGH);
  pinMode(A10, OUTPUT);

  Serial.println("Select Read, Write, Verify, Erase, BlankCheck [R/W/V/E/B]");

}

//###############################################################
// Main
//###############################################################

void loop() {


  if (Serial.available() > 0) {
    ReadSerial();
  }

  if (mode == "V") {
    Serial.println(""); Serial.println("Verifying EEPROM against bin_file");
    Verify();
    resetVariables();
  }

  if (mode == "R") {
    Serial.println(""); Serial.println("Read EEPROM");
    getStartAddress();
    getBlockSize();
    Read();
    resetVariables();
  }

  if (mode == "W") {
    Serial.println(""); Serial.println("Writing to EEPROM");
    getStartAddress();
    Write();
    resetVariables();
  }

  if (mode == "E") {
    Serial.println(""); Serial.println("Erasing W27C512 EEPROM");
    Serial.println("Starting in 5 secs");
    delay(5000);
    Erase();
    resetVariables();
  }

  if (mode == "B") {
    Serial.println(""); Serial.println("Blank check EEPROM");
    getStartAddress();
    getBlockSize();
    BlankCheck();
    resetVariables();
  }



}


//###############################################################
// Functions
//###############################################################
//Almost the same as Verify

void Read() {
  long addr = Start_Address;
  Last_Address = addr;

  Serial.println(BlockSize);

  for (int y = 0; y < BlockSize / 256; y++) {               // Loop If bin_size is 8k then 8192/256 = 32  times to read 8K of memory


    Serial.println("");

    read_start();
    for (int x = 0; x < BUFFERSIZE; ++x) {        //Loop and read first 256 bytes
      buffer[x] = read_byte(addr + 256 * y + x);
      delayMicroseconds(100);
    }
    read_end();
    int count = 0;
    printHex((addr + 256 * y), 4);
    Serial.print(": ");

    for (long x = 0; x < BUFFERSIZE; x++) {
      count++;
      Serial.print(" ");

      printHex(buffer[x], 2);
      if ((count > 15) and (x < BUFFERSIZE - 1)) {
        count = 0;
        Serial.println("");
        printHex((addr + 256 * y + x), 4);
        Serial.print(": ");
      }
    }
  }

  Serial.println("");
  Serial.println("");
  Serial.println("Read Completed");
}

//###############################################################

void Verify() {
  long addr = Start_Address;
  Last_Address = addr;

  for (int y = 0; y < arraySize / 256; y++) {               // Loop 8192/256 = 32  times to read 8K of memory
    Serial.print(".");

#ifdef DEBUG
    Serial.println("");
#endif

    read_start();
    for (int x = 0; x < BUFFERSIZE; ++x) {        //Loop and read first 256 bytes
      buffer[x] = read_byte(addr + 256 * y + x);
      delayMicroseconds(100);
    }
    read_end();

#ifdef DEBUG
    int count = 0;
    printHex((addr + 256 * y), 4);
    Serial.print(": ");
#endif

    for (long x = 0; x < BUFFERSIZE; x++) {

#ifdef DEBUG
      count++;
      Serial.print(" ");

      printHex(buffer[x], 2);
      if ((count > 15) and (x < BUFFERSIZE - 1)) {
        count = 0;
        Serial.println("");
        printHex((addr + 256 * y + x), 2);
        Serial.print(": ");
      }

#endif


      if (buffer[x] !=  pgm_read_byte(bin_file + 256 * y + x)) {
        Serial.println("");
        Serial.print("Compare error at 0x");
        Serial.print(x, HEX);
        Serial.println("");
        Serial.print("Found: 0x");
        printHex(buffer[x], 2);
        Serial.println("");
        Serial.print("Expected: 0x");
        printHex(pgm_read_byte(&bin_file[x + 256 * y]), 2);
        Serial.println("");
        while (1) {
          // Halt
        }
      }
    }
  }

  Serial.println("");
  Serial.println("");
  Serial.println("Compare OK");
}

//###############################################################

void BlankCheck() {
  //Get Parameters
  long addr = 0;                                  // Start checking from 0x0000
  Last_Address = addr;

  for (int y = 0; y < mem_count; y++) {                 // If mem_count is 32 then loop 32 * 256 times to read 8K of memory

    Serial.print(".");
    read_start();
    for (int x = 0; x < BUFFERSIZE; ++x) {
      buffer[x] = read_byte(addr + 256 * y + x);
      delayMicroseconds(100);
    }
    read_end();

    //return Array+Checksum to say it passed
    //ChecksumThis();
    for (long x = 0; x < 0xff; x++) {

      //Serial.println(buffer[x]);
      if (buffer[x] !=  0xFF) {
        Serial.println("");
        Serial.println("EEPROM  is not blank");

        Serial.println("");
        Serial.print("Found 0x");
        printHex(buffer[x], 2);
        Serial.print(" at position 0x");
        printHex((addr + 256 * y + x), 2);
        while (1) {

        }
      }
    }
  }
  Serial.println("");
  Serial.println("");
  Serial.println("E/EPROM  is blank");
}

//###############################################################

void Write() {

  long addr = Start_Address;
  Last_Address = addr;


  //Check if bin_file is larger than the EPROM
  if (chipType == "27C128") {
    if ( (Start_Address + arraySize) > 16384) {
      Error();
    }
  }

  if (chipType == "27C256") {
    if ( (Start_Address + arraySize) > 32768) {
      Error();
    }
  }

  if ((chipType == "27C512") || (chipType == "W27C512") ) {
    if ( (Start_Address + arraySize) > 65535) {
      Error();
    }
  }

  for (int y = 0; y < arraySize / 256; y++) {               // Loop 8192/256 = 32  times to read 8K of memory

#ifdef DEBUG
    int count = 0;
    Serial.println("");
    printHex((addr + 256 * y), 4);
    Serial.print(": ");
#endif

#ifndef DEBUG
    Serial.print(".");
#endif

    //Write
    write_start();


    for (int x = 0; x < BUFFERSIZE; ++x)  {
#ifdef DEBUG
      count++;
      Serial.print(" ");

      printHex(pgm_read_byte(&bin_file[x + 256 * y]), 2);

      if ((count > 15) and (x < BUFFERSIZE - 1)) {
        count = 0;
        Serial.println("");
        printHex((addr + 256 * y + x), 4);
        Serial.print(": ");
      }

#endif

      fast_write(addr + 256 * y + x, pgm_read_byte(&bin_file[x + 256 * y]));

    }

    write_end();
  }
  Serial.println("");
  Serial.println("");
  Serial.println("Write complete. Check with Read/Compare");
}

//###############################################################

void Erase() {
  set_ce(HIGH);
  set_oe(HIGH);
  set_vh(HIGH);
  set_vpp(HIGH);
  delay(1);

  //erase pulse
  set_ce(LOW);
  delay(350);
  set_ce(HIGH);
  delayMicroseconds(1);

  //Turning Off
  set_vh(LOW);
  set_vpp(LOW);
  delayMicroseconds(1);

  Serial.println("Erase complete. Run a blank check");
}


//###############################################################
// COMMANDS SUBS functions
//###############################################################

void read_start() {
  data_bus_input();
  //enable chip select
  set_ce(LOW);
  //enable output
  set_oe(LOW);

  //Set VPP to Low/High (27C2128, 27C256)
  if (chipType == CHIP27C128) {
    digitalWrite(A15VPP, LOW);
    Set_Output_At(15, HIGH);        //With 27C128 A15 becomes PGM
  }
  else if (chipType == CHIP27C256 || chipType == CHIP28C64) {
    digitalWrite(A15VPP, HIGH);   //the 28C64 doesnt care but the 27C64 is VPP
  }
  else if (chipType == CHIP27C32) {
    digitalWrite(A15VPP, LOW);  //normally used for A15/VPP, this pin is not used on 24pin chips
    Set_Output_At(15, LOW);     //normally used for A14, this pin is not used on 24pin chips
    Set_Output_At(14, HIGH); //**** normally used for A13, this pin is now VCC (5v) ****
    Set_Output_At(13, LOW);     //normally used for A12, this pin is not used on 24pin chips
    delayMicroseconds(5);
  }
}

void read_end() {
  //disable output
  set_oe(HIGH);
  //disable chip select
  set_ce(HIGH);

  //Set VPP to Low/High (27C2128, 27C256)
  if (chipType == CHIP27C128) Set_Output_At(15, LOW);         //With 27C128 A15 becomes PGM
  else if (chipType == CHIP27C256)  digitalWrite(A15VPP, LOW);
}

inline byte read_byte(unsigned int address)
{
  set_address_bus(address);
  return read_data_bus();
}

void write_start() {
  firstWritePass = true;
  //disable output
  set_oe(HIGH);

  set_vpp(HIGH);
  //Set VPP to low on 29C256 (not 27C256/27SF256 as its read only)
  if (chipType == CHIP27C256) {
    //Serial.print(".");
    digitalWrite(A15VPP, LOW);
    delayMicroseconds(5);
  }

  if (chipType == CHIP27C128) {
    Set_Output_At(15, LOW);         //With 27C128 A15 becomes PGM, LOW when programming
    delayMicroseconds(5);
  }
  
  data_bus_output();
}

void write_end() {
  set_vpp(LOW);
  data_bus_input();



}

inline void fast_write(unsigned int address, byte data)
{
  if (chipType == CHIP28C64 || chipType == CHIP28C256) {
    //this function uses /DATA polling to get the end of the page write cycle. This is much faster than waiting 10ms
    static unsigned int lastAddress = 0;
    static byte lastData = 0;

    //enable chip select
    set_ce(LOW);

    //data poll
    if (((lastAddress ^ address) & 0xFFC0 || chipType == CHIP28C64) && !firstWritePass)
    {
      unsigned long startTime = millis();

      //poll data until data matches
      data_bus_input();
      set_oe(LOW);

      //set timeout here longer than JBurn timeout
      while (lastData != read_data_bus()) {
        if (millis() - startTime > 3000) return false;
      }

      set_oe(HIGH);
      delayMicroseconds(1);
      data_bus_output();
    }

    //set address and data for write
    set_address_bus(address);
    write_data_bus(data);
    delayMicroseconds(1);

    //strobe write
    set_we(LOW);
    set_we(HIGH);
    //disable chip select
    set_ce(HIGH);

    lastAddress = address;
    lastData = data;
    firstWritePass = false;
  }
  else
  {
    set_address_bus(address);
    write_data_bus(data);
    delayMicroseconds(1);

    //programming pulse
    set_ce(LOW);
    delayMicroseconds(100); // for W27E512, works for 27SF512 also (but 27SF512 should be 20ms)
    set_ce(HIGH);
    delayMicroseconds(1);
  }
}

//###############################################################
// DATA BUS functions
//###############################################################

void data_bus_input() {
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
}

void data_bus_output() {
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
}

byte read_data_bus()
{
  byte b = 0;
  if (digitalRead(D0) == HIGH) b |= 1;
  if (digitalRead(D1) == HIGH) b |= 2;
  if (digitalRead(D2) == HIGH) b |= 4;
  if (digitalRead(D3) == HIGH) b |= 8;
  if (digitalRead(D4) == HIGH) b |= 16;
  if (digitalRead(D5) == HIGH) b |= 32;
  if (digitalRead(D6) == HIGH) b |= 64;
  if (digitalRead(D7) == HIGH) b |= 128;
  return (b);
}

inline void write_data_bus(byte data)
{
  digitalWrite(D0, data & 1);
  digitalWrite(D1, data & 2);
  digitalWrite(D2, data & 4);
  digitalWrite(D3, data & 8);
  digitalWrite(D4, data & 16);
  digitalWrite(D5, data & 32);
  digitalWrite(D6, data & 64);
  digitalWrite(D7, data & 128);
}

//###############################################################
// FAST SWIFT functions
//###############################################################
inline void set_address_bus(unsigned int address)
{
  byte hi, low;
  hi = (address >> 8);
  low = address & 0xff;
  //A14 become WE on 28C64 && 28C256, make sure dont use the output that were generally used for A14
  if (chipType == CHIP28C64 || chipType == CHIP28C256) bitSet(hi, 6); //set ouput 7 on hi byte
  else if (chipType == CHIP27C32) {
    bitClear(hi, 6);  //set ouput 7 on hi byte
    bitSet(hi, 5);    //set ouput 6 on hi byte, this is now VCC
    bitClear(hi, 4);  //set ouput 5 on hi byte
  }

  ApplyShiftAt(hi, low);

  digitalWrite(A10, (address & 1024) ? HIGH : LOW );
  if (chipType == CHIP28C64 || chipType == CHIP28C256) digitalWrite(A15VPP, (address & 16384) ? HIGH : LOW); //A15/VPP become A14 on 28C64 && 28C256
  else if (chipType == CHIP27SF512) digitalWrite(A15VPP, (address & 32768) ? HIGH : LOW);
}

inline void Set_Output_At(unsigned int Position, bool IsHigh)
{
  byte hi, low;
  hi = (Last_Address >> 8);
  low = Last_Address & 0xff;
  if (Position >= 8) {
    if (IsHigh) bitSet(hi, Position - 8);
    else  bitClear(hi, Position - 8);
  }
  else {
    if (IsHigh) bitSet(low, Position);
    else  bitClear(low, Position);
  }
  ApplyShiftAt(hi, low);
}

void ApplyShiftAt(byte hi, byte low)
{
  fastShiftOut(hi);
  fastShiftOut(low);
  //strobe latch line
  bitSet(STROBE_PORT, STROBE_LATCH);
  bitClear(STROBE_PORT, STROBE_LATCH);
  delayMicroseconds(1);
}

void fastShiftOut(byte data) {
  //clear
  bitClear(STROBE_PORT, STROBE_DS);

  //Loop for the 8x outputs
  for (int i = 7; i >= 0; i--) {
    //clear clock pin
    bitClear(STROBE_PORT, STROBE_CLOCK);

    //Enable/Disable pin Output
    if (bitRead(data, i) == 1) bitSet(STROBE_PORT, STROBE_DS);
    else  bitClear(STROBE_PORT, STROBE_DS);

    //register shifts bits on upstroke of clock pin
    bitSet(STROBE_PORT, STROBE_CLOCK);
    //clear after shift to prevent bleed through
    bitClear(STROBE_PORT, STROBE_DS);
  }

  //stop shifting
  bitClear(STROBE_PORT, STROBE_CLOCK);
}

int GetAddress(unsigned int Position)
{
  int Address = 0;
  if (Position == 1) Address = 1;
  if (Position == 2) Address = 2;
  if (Position == 3) Address = 4;
  if (Position == 4) Address = 8;
  if (Position == 5) Address = 16;
  if (Position == 6) Address = 32;
  if (Position == 7) Address = 64;
  if (Position == 8) Address = 128;
  if (Position == 9) Address = 256;
  if (Position == 10) Address = 512;
  if (Position == 11) Address = 1024;
  if (Position == 12) Address = 2048;
  if (Position == 13) Address = 4096;
  if (Position == 14) Address = 8192;
  if (Position == 15) Address = 16384;
  if (Position == 16) Address = 32768;
  return Address;
}

//###############################################################
// PINS functions
//###############################################################

//**attention, this line is LOW - active**
inline void set_oe (byte state)
{
  digitalWrite(OE, state);
}

//**attention, this line is LOW - active**
inline void set_ce (byte state)
{
  digitalWrite(CE, state);
}


//**attention, this line is LOW - active**
inline void set_we (byte state)
{
  if (chipType == CHIP28C64 || chipType == CHIP28C256 || CHIP27C128) Set_Output_At(15, state);  //output 15 become WE (since there are 8 outputs by 74HC595, its the #7 ouputs on the 2nd 74HC595)
}

//Boost VPP 12V
void set_vpp (byte state)
{
  switch (chipType) {
    case CHIP27SF512:
      digitalWrite(VPP, state);
      break;
    case CHIP27C256:                          // Vpp on pin 1
      digitalWrite(VPP_27C256, state);
      break;
    case CHIP27C128:                          // Vpp on pin 1
      digitalWrite(VPP_27C256, state);
      break;
    default:
      break;
  }
}

//Boost Erase 12V
void set_vh (byte state)
{
  switch (chipType) {
    case CHIP27SF512:
      digitalWrite(VH, state);
      break;
    default:
      break;
  }
}

//==========================================================================

//Print HEX With leading zeros
void printHex(int num, int precision) {

  char tmp[16];
  char format[128];

  sprintf(format, "%%.%dX", precision);
  sprintf(tmp, format, num);
  Serial.print(tmp);
}

void ReadSerial() {

  char rx_byte = Serial.read();       // get the character

  if ((rx_byte == 'V') || (rx_byte == 'v')) {
    Serial.println(""); Serial.print("Verify selected"); Serial.println("");
    mode = "V";
    selectChip();
  }

  if ((rx_byte == 'R') || (rx_byte == 'r')) {
    Serial.println(""); Serial.print("Read selected"); Serial.println("");
    mode = "R";
    selectChip();
  }

  if ((rx_byte == 'W') || (rx_byte == 'w')) {
    Serial.println(""); Serial.print("Write selected"); Serial.println("");
    mode = "W";
    selectChip();
  }

  if ((rx_byte == 'E') || (rx_byte == 'e')) {
    Serial.println(""); Serial.print("Erase selected"); Serial.println("");
    mode = "E";
    //Can only erase W27C512
    chipType = CHIP27SF512;    //W27C512;

  }

  if ((rx_byte == 'B') || (rx_byte == 'b')) {
    Serial.println(""); Serial.print("BlankCheck selected"); Serial.println("");
    mode = "B";
    selectChip();
  }

}

void selectChip() {
  boolean selected = false;
  String rx_string;

  Serial.println(""); Serial.println("Enter chip: 27C128, 27C256, 27C512, W27C512"); Serial.println("");

  while (selected == false) {
    if (Serial.available() > 0) {
      rx_string = Serial.readString();
      rx_string.trim();

      if (rx_string == "27C128" || rx_string == "27c128") {
        chipType = CHIP27C128;
        Serial.println("27C128 selected");
        selected = true;
        mem_count = 64;
      }

      if (rx_string == "27C256" || rx_string == "27c256") {
        chipType = CHIP27C256;
        Serial.println("27C256 selected");
        selected = true;
        mem_count = 128;
      }

      if (rx_string == "27C512" || rx_string == "27c512") {
        chipType = CHIP27SF512;
        Serial.println("27C512 selected");
        selected = true;
        mem_count = 32;
      }

      if (rx_string == "W27C512" || rx_string == "w27c512") {
        chipType = CHIP27SF512;
        Serial.println("W27C512 selected");
        selected = true;
        mem_count = 32;
      }
    }
  }

}

void getStartAddress() {

  boolean selected = false;
  int charsRead;
  char input[5];

  Serial.println(""); Serial.print("Enter start address in hex ");
  if (chipType == 1) {
    Serial.println("(0000, 2000) :");
  }

  if (chipType == 2) {
    Serial.println("(0000, 2000, 4000, 6000) :");
  }

  if ((chipType == 5)) {
    Serial.println("(0000, 2000, 4000, 6000, 8000, a000, c000, e000): ");
  }

  Serial.println("");

  while (selected == false) {

    if (Serial.available() > 0) {
      charsRead = Serial.readBytesUntil('\n', input, 5);  // fetch the two characters
      input[charsRead] = '\0';                            // Make it a string

      Start_Address = StrToHex(input);                              // Convert it

      if ( (Start_Address == 0x0000) || (Start_Address == 0x2000) || (Start_Address == 0x4000) || (Start_Address == 0x6000)
           || (Start_Address == 0x8000) || (Start_Address == 0xa000) || (Start_Address == 0xc000) || (Start_Address == 0xe000) ) {
        selected = true;
      }

    }

  }

  Serial.print("Start Address: ");
  if (Start_Address == 0) {
    Serial.println("0000");
  }
  else {
    Serial.println(Start_Address, HEX);
  }
  Serial.println("");

}

void getBlockSize() {

  boolean selected = false;
  int charsRead;
  char input[5];

  Serial.print("Enter block size in hex ");
  if (chipType == 1) {
    Serial.println("(1000, 2000, 4000)");
  } else {
    Serial.println("(1000, 2000, 4000, 6000)");
  }

  while (selected == false) {

    if (Serial.available() > 0) {
      charsRead = Serial.readBytesUntil('\n', input, 5);  // fetch the two characters
      input[charsRead] = '\0';                            // Make it a string

      BlockSize = StrToHex(input);                              // Convert it

      if ( (BlockSize == 0x1000) || (BlockSize == 0x2000) || (BlockSize == 0x4000) || (BlockSize == 0x6000) ) {
        selected = true;
      }

    }

  }
  Serial.print("Block Size: "); Serial.println(BlockSize, HEX);

}


void resetVariables() {
  mode = "";
  chipType = "";
  Serial.println("");
  Serial.println("Select Read, Write, Verify, Erase, BlankCheck [R/W/V/E/B]");
}

long StrToHex(char str[])
{
  return (long) strtol(str, 0, 16);
}


void Error() {
  Serial.println("Error. Bin file  is too big");
  Serial.print("bin_file: "); Serial.println(arraySize);


  while (1);
}
