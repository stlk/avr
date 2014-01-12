stlk's AVR projects
===

attiny2313_temp
---
This project serves as interface between low-cost .NET MicroFramework device Netduino and DS-18B20 temperature sensor. It also handles PIR sensor notifications.



AVRdude commands
---

*Read fuses*
avrdude -p atmega8 -P com11 -c stk500v2 -b 115200 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U lock:r:-:h

*Write fuses*
avrdude -p atmega8 -P com11 -c stk500v2 -b 115200 -U lfuse:w:0x7a:m -U hfuse:w:0xfd:m

avrdude -p t2313 -P com11 -c stk500v2 -b 115200 -U lfuse:w:0xe4:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m