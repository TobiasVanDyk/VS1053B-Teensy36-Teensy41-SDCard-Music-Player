# VS1053B-Teensy36-Teensy41-SDCard-Music-Player
VS1053B Teensy36 Teensy41 SDCard Music Player
<p align="left">
<img src="images/Teensy36Teensy41VS1053SDPlayer.jpg" width="700" /> 
<br>

VS1053B		Teensy<br> 
Adafruit	3.6 and 4.1 (*1,*2)<br>

VCC   5v<br>	 
GND   GND<br>	 
SCLK  Pin 13   SPI Clock, shared with SD card<br>
MISO  Pin 12   Input data, from VS1053/SD card<br>
MOSI  Pin 11   Output data, to VS1053/SD card<br>
CS    Pin 10   VS1053 chip select pin (output)<br>
RST   Pin 09   VS1053 reset pin active low (output)<br>
XDCS  Pin 08   VS1053 Data/command select pin (output)<br>
SDCS  Pin 04   Chip select line for SD card<br>
DREQ  Pin 03   VS1053 Data request (input) (ideally an Interrupt pin)<br>

*1 and *2: Adafruit VS1053B 2020 lib and Teensy 2020 lib with VS1053playerTeensy<br>
*3 Standalone VS1053playerTeensy<br>

For the Teensy 4.1 comment out the Status Register Save SREG else get 'SREG' was not declared in this scope compiler message<br>
See alao: https://forum.pjrc.com/threads/66728-Backup-the-Interrupt-Enable-State-and-Restore-it-on-the-T4-x<br>

A recent topic: https://forum.pjrc.com/threads/70704-VS1053-Adafruit-library-player_simple-example-doesn-t-compile-for-Teensy-4-1

