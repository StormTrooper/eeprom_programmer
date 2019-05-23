#!/usr/bin/env python3

#Convert Commodore 64 binary input file from bin to ascii so it can
#be used as a include file in an Arduino sketch  for writing to W27C512 EEPROM

import binascii
import sys
import os

bytes=0

if len(sys.argv) != 2:
        print ("Script to convert  Commodore 64 file from  bin the ascii for  Arduino sketch")
        print ("Usage: convert.py inputfile")
        exit()


try:
    file = open("bin_file.h","wb")
except:
    print ("Unable to create bin_file.h")
    exit()

try:
    statinfo = os.stat(sys.argv[1])
except:
    print ("Unable to open inputfile", sys.argv[1])
    exit()

if statinfo.st_size > 16384:
   print ("Binary file greater than 16k")
   exit()


file.write("const byte bin_file[] PROGMEM = { \\".encode())
file.write("\n".encode())
try:
    with open(sys.argv[1], "rb") as f:
        b = True
        count=0
        while b:
            if count < 16 and bytes < statinfo.st_size-1:
                bytes = bytes + 1
                hexdata = f.read(1)
                file.write("0x".encode())
                file.write(binascii.hexlify(hexdata))
                file.write(",".encode())
                count = count + 1
            elif bytes < statinfo.st_size-1:
                count = 0
                file.write(" \\".encode())
                file.write("\n".encode());
            elif bytes > statinfo.st_size-2:
                bytes = bytes + 1
                hexdata = f.read(1)
                file.write("0x".encode())
                file.write(binascii.hexlify(hexdata))
                file.write(" \\".encode())
                file.write("\n".encode())
                file.write("};".encode())
                b = False

except Exception as ex:
    print ("Unable to open", sys.argv[1])
    print(ex)
    file.close()
    exit()

file.close()
f.close()
print ("File converted. Created bin_file.h")


