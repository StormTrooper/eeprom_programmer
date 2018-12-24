#Convert Commodore 64 binary input file from bin to ascii so it can
#be used as a include file in an Arduino sketch  for writing to W27C512 EEPROM"

import binascii
import sys
import os

if len(sys.argv) <> 2:
        print "Script to convert  Commodore 64 file from  bin the ascii for  Arduino sketch"
        print "Usage: convert.py inputfile"
        exit()


try:
    file = open("bin_file.h","w")
except:
    print "Unable to create bin_file.h"
    exit()

try:
    statinfo = os.stat(sys.argv[1])
except:
    print "Unable to open inputfile", sys.argv[1]
    exit()
if statinfo.st_size <> 8192:
   print "Binary file is not 8192 bytes"
   exit()


file.write("const byte bin_file[] PROGMEM = { \\")
file.write("\n")
bytes = 0
try:
    with open(sys.argv[1], "rb") as f:
        b = True
        count=0

        while b:
            if count < 16 and bytes < 8191:
                bytes = bytes + 1
                hexdata = f.read(1)
                file.write("0x")
                file.write(binascii.hexlify(hexdata))
                file.write(",")
                count = count + 1
            elif bytes < 8191:
               count = 0
               file.write(" \\")
               file.write("\n");
            elif bytes > 8190:
                bytes = bytes + 1
                hexdata = f.read(1)
                file.write("0x")
                file.write(binascii.hexlify(hexdata))
                file.write(" \\")
                file.write("\n")
                file.write("};")
                b = False

except:
    print "Unable to open", sys.argv[1]
    file.close()
    exit

file.close()
f.close()
print "File converted. Created bin_file.h"

