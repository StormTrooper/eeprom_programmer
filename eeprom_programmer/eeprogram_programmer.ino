/*
   EEPROM Programmer

   It is made to Read/Write Data from EEPROM directly with the Arduino
   This program is made to read Winbond W27c256

   For more information see https://blog.gjmccarthy.co.uk

*/


#include "bin_file.h"


//#define DEBUG

//####################################################################################################################################
// Select mode
//####################################################################################################################################
//#define VERIFY
#define READ
//#define CHECKBLANK
//#define WRITE
//#define ERASE


#define Start_Address 0x0000        //0x0000, 0x2000, 0x4000, 0x6000, 0x8000, 0xa000, 0xc000, 0xe000

//####################################################################################################################################


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

//###############################################################
// Setup
//###############################################################
void setup() {
  //define the shiftOut Pins as output
  pinMode(DS, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);

  //define the boost pins as output (take care that they are LOW)
  digitalWrite(VH, LOW);
  pinMode(VH, OUTPUT);
  digitalWrite(VPP, LOW);
  pinMode(VPP, OUTPUT);

  //define the EEPROM Pins as output (take care that they are HIGH)
  digitalWrite(OE, HIGH);
  pinMode(OE, OUTPUT);
  digitalWrite(CE, HIGH);
  pinMode(CE, OUTPUT);
  digitalWrite(A15VPP, HIGH);
  pinMode(A15VPP, OUTPUT);
  digitalWrite(A10, HIGH);
  pinMode(A10, OUTPUT);

  //set speed of serial connection
  Serial.begin(115200);

}

//###############################################################
// Main
//###############################################################

void loop() {

#ifdef VERIFY
  Serial.println("Verifying EEPROM against bin_file");
  Verify();
#endif

#ifdef READ
  Serial.println("Read EEPROM");
  Read();
#endif

#ifdef WRITE
  Serial.println("Writing to EEPROM");
  Write();
#endif


#ifdef ERASE
  Serial.println("Erasing EEPROM");
  Erase();
#endif

#ifdef CHECKBLANK
  Serial.println("Blank check 64k x 8 EEPROM");
  BlankCheck();
#endif

  while (1) {
  }

}


//###############################################################
// Functions
//###############################################################
//Almost the same as Verify

void Read() {
  long addr = Start_Address;
  Last_Address = addr;

  for (int y = 0; y < 32; y++) {                 // Loop 8192/256 = 32  times to read 8K of memory


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

  for (int y = 0; y < 32; y++) {                 // Loop 8192/256 = 32  times to read 8K of memory

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
        printHex((addr + 256 * y + x), 4);
        Serial.print(": ");
      }

#endif


      if (buffer[x] !=  pgm_read_word_near(bin_file + 256 * y + x)) {
        Serial.println("");
        Serial.print("Compare error at 0x");
        Serial.print(x, HEX);
        Serial.println("");
        Serial.print("Found: 0x");
        printHex(buffer[x], 2);
        Serial.println("");
        Serial.print("Expected: 0x");
        printHex(pgm_read_dword(&bin_file[x + 256 * y]), 2);
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

  for (int y = 0; y < 32; y++) {                 // Loop 8192/256 = 32  times to read 8K of memory


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
      if (buffer[x] !=  0x09) {
        Serial.println("");
        Serial.println("EEPROM  is not blank");

        Serial.println("");
        Serial.print("Found 0x");
        printHex(buffer[x], 2);
        Serial.print(" at position 0x");
        printHex((addr + 256 * y + x), 4);
        while (1) {

        }
      }
    }
  }
  Serial.println("");
  Serial.println("");
  Serial.println("EEPROM  is blank");
}

//###############################################################

void Write() {

  long addr = Start_Address; //long addr = ((long) cmdbuf[3] * 256) + (long) cmdbuf[4];
  Last_Address = addr;

  for (int y = 0; y < 32; y++) {                 // Loop 8192/256 = 32  times to read 8K of memory

#ifdef DEBUG
    Serial.println("");
    int count = 0;
    printHex((addr + 256 * y), 4);
    Serial.print(": ");
#endif

    //Write
    write_start();


    for (int x = 0; x < BUFFERSIZE; ++x)  {
#ifdef DEBUG
      count++;
      Serial.print(" ");

      printHex(pgm_read_dword(&bin_file[x + 256 * y]), 2);;
      if ((count > 15) and (x < BUFFERSIZE - 1)) {
        count = 0;
        Serial.println("");
        printHex((addr + 256 * y + x), 4);
        Serial.print(": ");
      }

#endif

      fast_write(addr + 256 * y + x, pgm_read_dword(&bin_file[x + 256 * y]));

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

}

void read_end() {
  //disable output
  set_oe(HIGH);
  //disable chip select
  set_ce(HIGH);
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

  data_bus_output();
}

void write_end() {
  set_vpp(LOW);
  data_bus_input();
}

inline void fast_write(unsigned int address, byte data)
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


  ApplyShiftAt(hi, low);

  digitalWrite(A10, (address & 1024) ? HIGH : LOW );
  digitalWrite(A15VPP, (address & 32768) ? HIGH : LOW);
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


//Boost VPP 12V
void set_vpp (byte state)
{
  digitalWrite(VPP, state);
}

//Boost Erase 12V
void set_vh (byte state)
{

  digitalWrite(VH, state);
}

//Print HEX With leading zeros
void printHex(int num, int precision) {
  char tmp[16];
  char format[128];

  sprintf(format, "%%.%dX", precision);
  sprintf(tmp, format, num);
  Serial.print(tmp);
}

