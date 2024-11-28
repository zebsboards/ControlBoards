

avrdude -F -p m32u4 -c usbasp -B 5 -U flash:w:LW_24P_DEV1.hex -U lfuse:w:0x5e:m -U hfuse:w:0x99:m -U efuse:w:0xf3:m


pause


 