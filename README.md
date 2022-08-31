# VS1053B-Teensy3.6-Teensy4.1-SDCard-Music-Player
 
<p align="left">
<img src="images/Teensy36Teensy41VS1053SDPlayer.jpg" width="700" /> 
<br>

<p align="left">
<img src="images/connect.jpg" width="595" /> 
<br>

See the [**2022 comment and solution**](https://forum.pjrc.com/threads/70944-VS1053b-audio-codec-no-audio-output) for the problem mentioned next. For the Teensy 4.1 comment out the Status Register Save SREG section - else it cause an 'SREG was not declared in this scope' compiler error. In the newer (2021) version of the Adafruit VS1053 library this is not necessary - SREG is not referred to in Adafruit_VS1053.cpp. See https://forum.pjrc.com/threads/70704-VS1053-Adafruit-library-player_simple-example-doesn-t-compile-for-Teensy-4-1

