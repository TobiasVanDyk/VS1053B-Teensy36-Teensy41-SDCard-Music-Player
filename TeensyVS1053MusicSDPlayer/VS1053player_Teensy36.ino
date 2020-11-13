//////////////////////////////////////////////////////////////////////////
// Modified for Teensy 3.6 Tobias van Dyk October 2020
//
// Also based on PJRC.com Teensy libraries
/////////////////////////////////////////////////////////////////////////
/*************************************************** 
  This is a library for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout 
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// include SPI, MP3 and SD libraries
#include <SPI.h>
//#include <Adafruit_VS1053.h>             // Used Teensy-adapted lib from pjrc.com
#include <SD.h>

// Connect CLK, MISO and MOSI to standard hardware SPI pins. 
// See http://arduino.cc/en/Reference/SPI "Connections"
#define CLK 13       // SPI Clock, shared with SD card
#define MISO 12      // Input data, from VS1053/SD card
#define MOSI 11      // Output data, to VS1053/SD card

// These are the pins used for the Adafruit VS1053B breakout module
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
// For Teensy 3.6 better to use its built-in SDCard
//#define CARDCS 4                // Use VS1053 SDCard Card chip select pin
#define CARDCS   BUILTIN_SDCARD   // Use Teensy 3.6 built-in card
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

/////////////////////////////////////////////////////////////////////////////////
// VS1053B.h
/////////////////////////////////////////////////////////////////////////////////

#define ADAFRUIT_VS1053_H

#include <Arduino.h>
#include "pins_arduino.h"
#include "wiring_private.h"

// musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT)
// musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)  see line 91 in .cpp
#define VS1053_FILEPLAYER_TIMER0_INT 255 // allows useInterrupt to accept pins 0 to 254
#define VS1053_FILEPLAYER_PIN_INT 5      // Interrupt number from dreqinttable dreq pin # = 3

#define VS1053_SCI_READ 0x03
#define VS1053_SCI_WRITE 0x02

#define VS1053_REG_MODE  0x00
#define VS1053_REG_STATUS 0x01
#define VS1053_REG_BASS 0x02
#define VS1053_REG_CLOCKF 0x03
#define VS1053_REG_DECODETIME 0x04
#define VS1053_REG_AUDATA 0x05
#define VS1053_REG_WRAM 0x06
#define VS1053_REG_WRAMADDR 0x07
#define VS1053_REG_HDAT0 0x08
#define VS1053_REG_HDAT1 0x09
#define VS1053_REG_VOLUME 0x0B

#define VS1053_GPIO_DDR 0xC017
#define VS1053_GPIO_IDATA 0xC018
#define VS1053_GPIO_ODATA 0xC019

#define VS1053_INT_ENABLE  0xC01A

#define VS1053_MODE_SM_DIFF 0x0001
#define VS1053_MODE_SM_LAYER12 0x0002
#define VS1053_MODE_SM_RESET 0x0004
#define VS1053_MODE_SM_CANCEL 0x0008
#define VS1053_MODE_SM_EARSPKLO 0x0010
#define VS1053_MODE_SM_TESTS 0x0020
#define VS1053_MODE_SM_STREAM 0x0040
#define VS1053_MODE_SM_SDINEW 0x0800
#define VS1053_MODE_SM_ADPCM 0x1000
#define VS1053_MODE_SM_LINE1 0x4000
#define VS1053_MODE_SM_CLKRANGE 0x8000

#define VS1053_SCI_AIADDR 0x0A
#define VS1053_SCI_AICTRL0 0x0C
#define VS1053_SCI_AICTRL1 0x0D
#define VS1053_SCI_AICTRL2 0x0E
#define VS1053_SCI_AICTRL3 0x0F

#define VS1053_DATABUFFERLEN 32

File currentTrack;
boolean playingMusic;
uint8_t mp3buffer[VS1053_DATABUFFERLEN];

////////////////////////////////////////////////////////////////////////////////
// VS1053B.cpp
////////////////////////////////////////////////////////////////////////////////
#define VS1053_CONTROL_SPI_SETTING  SPISettings(250000,  MSBFIRST, SPI_MODE0) // 2.5 MHz SPI speed Control 
#define VS1053_DATA_SPI_SETTING     SPISettings(8000000, MSBFIRST, SPI_MODE0) // 8 MHz SPI speed Data

//static void feeder(void) {
//  myself->VS1053FeedBuffer();       // dereference pointer to instance (*myself).feedBuffer()
//}
////////////////////////////////////////////////////////////////
// Configure interrupt for Data DREQ from VS1053
//////////////////////////////////////////////////////////////// 
void VS1053Interrupt() 
{ SPI.usingInterrupt(DREQ);                          // Disable Interrupt during SPI transactions
  attachInterrupt(DREQ, VS1053FeedBuffer, CHANGE); // Interrupt Pin 3 change
}                                                    // feeder->feedBuffer executed (lines 26, 209)

//////////////////////////////////////////////////////////
// Set the card to be disabled while we get the VS1053 up
//////////////////////////////////////////////////////////
void VS1053DisableCard ()
{ playingMusic = false;
  pinMode(CARDCS, OUTPUT);
  digitalWrite(CARDCS, HIGH);  
}

/////////////////////////////////////////////////////////////////////////
// Play file without interrupts
/////////////////////////////////////////////////////////////////////////
boolean VS1053PlayFullFile(const char *trackname) 
{ if (! VS1053StartPlayingFile(trackname)) return false;
  while (playingMusic) { VS1053FeedBuffer(); }
  return true;
}

/////////////////////////
void VS1053StopPlaying() 
{ playingMusic = false;
  currentTrack.close();
}

///////////////////////////////////////
void VS1053PausePlaying(boolean pause) 
{ if (pause) playingMusic = false;
     else { playingMusic = true;
            VS1053FeedBuffer();
          }
}

///////////////////////
boolean VS1053Paused() 
{ return (!playingMusic && currentTrack);
}

////////////////////////
boolean VS1053Stopped() 
{ return (!playingMusic && !currentTrack);
}

//////////////////////////////////////////////////////
boolean VS1053StartPlayingFile(const char *trackname) 
{ currentTrack = SD.open(trackname);
  if (!currentTrack) { return false; }
  playingMusic = true;
  while (! VS1053ReadyForData() );                                    // wait for ready for data
  while (playingMusic && VS1053ReadyForData()) VS1053FeedBuffer();   // then send data
  return true;
}

///////////////////////////////////////////////////
void VS1053FeedBuffer() 
{ static uint8_t running = 0;
  uint8_t sregsave;

  // Do not allow 2 copies of this code to run concurrently
  sregsave = SREG;
  cli();
  if (running) { SREG = sregsave;
                 return;  
               } else { running = 1;
                        SREG = sregsave;
                      }

  if (! playingMusic)   { running = 0; return; } // paused or stopped
  if (! currentTrack)   { running = 0; return; }
  if (! VS1053ReadyForData()) { running = 0; return; }

  // Send buffer
  while (VS1053ReadyForData()) 
        { int bytesread = currentTrack.read(mp3buffer, VS1053_DATABUFFERLEN);
          if (bytesread == 0) // End of File
             { playingMusic = false;
               currentTrack.close();
               running = 0;
               return;
             }
          VS1053PlayData(mp3buffer, bytesread);
        }
  running = 0;
  return;
}

/////////////////////////////
boolean VS1053ReadyForData() 
{ return digitalRead(DREQ);
}

//////////////////////////////////////////////////////
void VS1053PlayData(uint8_t *buffer, uint8_t buffsiz) 
{ SPI.beginTransaction(VS1053_DATA_SPI_SETTING);
  digitalWrite(BREAKOUT_DCS, LOW);

  for (uint8_t i=0; i<buffsiz; i++) { VS1053SpiWrite(buffer[i]); }  // buffsiz = 32
    
  digitalWrite(BREAKOUT_DCS, HIGH);
  SPI.endTransaction();
}

//////////////////////////////////////////////////
void VS1053SetVolume(uint8_t left, uint8_t right) 
{ uint16_t v;
  v = left;
  v <<= 8;
  v |= right;

  cli();
  VS1053SciWrite(VS1053_REG_VOLUME, v);
  sei();
}

////////////////////////////
uint16_t VS1053DecodeTime() 
{ cli();
  uint16_t t = VS1053SciRead(VS1053_REG_DECODETIME);
  sei();
  return t;
}

///////////////////////
void VS1053SoftReset() 
{ VS1053SciWrite(VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
  delay(100);
}

//////////////////////////
void VS1053Reset() 
{ if (BREAKOUT_RESET >= 0) 
     { digitalWrite(BREAKOUT_RESET, LOW);
       delay(100);
       digitalWrite(BREAKOUT_RESET, HIGH);
     }
  digitalWrite(BREAKOUT_CS, HIGH);
  digitalWrite(BREAKOUT_DCS, HIGH);
  delay(100);
  VS1053SoftReset();
  delay(100);

  VS1053SciWrite(VS1053_REG_CLOCKF, 0x6000);

  VS1053SetVolume(40, 40);
}

//////////////////////
uint8_t VS1053Begin() 
{ if (BREAKOUT_RESET >= 0) { pinMode(BREAKOUT_RESET, OUTPUT);  // if reset = -1 ignore
                             digitalWrite(BREAKOUT_RESET, LOW);
                           }

  pinMode(BREAKOUT_CS, OUTPUT);
  digitalWrite(BREAKOUT_CS, HIGH);
  pinMode(BREAKOUT_DCS, OUTPUT);
  digitalWrite(BREAKOUT_DCS, HIGH);
  pinMode(DREQ, INPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  
  VS1053Reset();

  return (VS1053SciRead(VS1053_REG_STATUS) >> 4) & 0x0F;
}

/////////////////////////////////////
uint16_t VS1053SciRead(uint8_t addr) 
{ uint16_t data;
  SPI.beginTransaction(VS1053_CONTROL_SPI_SETTING);
  digitalWrite(BREAKOUT_CS, LOW);  
  VS1053SpiWrite(VS1053_SCI_READ);
  VS1053SpiWrite(addr);
  delayMicroseconds(10);
  data = VS1053SpiRead();
  data <<= 8;
  data |= VS1053SpiRead();
  digitalWrite(BREAKOUT_CS, HIGH);
  SPI.endTransaction();
  return data;
}

//////////////////////////////////////////////
void VS1053SciWrite(uint8_t addr, uint16_t data) 
{ SPI.beginTransaction(VS1053_CONTROL_SPI_SETTING);
  digitalWrite(BREAKOUT_CS, LOW);  
  VS1053SpiWrite(VS1053_SCI_WRITE);
  VS1053SpiWrite(addr);
  VS1053SpiWrite(data >> 8);
  VS1053SpiWrite(data & 0xFF);
  digitalWrite(BREAKOUT_CS, HIGH);
  SPI.endTransaction();
}

static volatile uint8_t *clkportreg;
static uint8_t clkpin;

////////////////////////
uint8_t VS1053SpiRead()
{ int8_t x;
  x = 0;
  //clkportreg = portOutputRegister(digitalPinToPort(CLK));
  //clkpin = digitalPinToBitMask(CLK);
  // MSB first, clock low when inactive (CPOL 0), data valid on leading edge (CPHA 0)
  // Make sure clock starts low
  x = SPI.transfer(0x00);
  // Make sure clock ends low
  //*clkportreg &= ~clkpin;

  return x;
}

///////////////////////////////
void VS1053SpiWrite(uint8_t c)
{ // MSB first, clock low when inactive (CPOL 0), data valid on leading edge (CPHA 0)
  // Make sure clock starts low
  //clkportreg = portOutputRegister(digitalPinToPort(CLK));
  //clkpin = digitalPinToBitMask(CLK);
  SPI.transfer(c);
  //*clkportreg &= ~clkpin;   // Make sure clock ends low
}

/////////////////////////////////////
// End .cpp
/////////////////////////////////////


void setup() {
  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor
  Serial.println("Adafruit VS1053B SDCard Music Player Test");

  if (! VS1053Begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
  
  SD.begin(CARDCS);    // initialise the SD card
  
  // Set volume for left, right channels. lower numbers is higher volume
  VS1053SetVolume(20,20);

  // If DREQ is on an interrupt pin (any Teensy pin) can do background audio playing
  VS1053Interrupt();  // DREQ int
  
  // Play one file, don't return until complete - i.e. serial "s" or "p" will not interrupt
  //Serial.println(F("Playing track 001"));
  //VS1053PlayFullFile("track001.mp3");
  Serial.println(F("Playing track 008"));
  VS1053PlayFullFile("track008.wav");
  
  // Play another file in the background, use interrupt serial "s" or "p" will interrupt
  // Can only do one buffer feed at a time
  //Serial.println(F("Playing track 002 - press p to pause, s to stop"));
  //VS1053StartPlayingFile("track002.mp3");
  Serial.println(F("Playing track 010 - press p to pause, s to stop"));
  VS1053StartPlayingFile("track010.wav");
}

void loop() {
  // File is playing in the background
  if (VS1053Stopped()) {
    Serial.println("Terminated");
    while (1);
  }
  if (Serial.available()) {
    char c = Serial.read();
    
    // if we get an 's' on the serial console, stop!
    if (c == 's') {
      VS1053StopPlaying();
    }
    
    // if we get an 'p' on the serial console, pause/unpause!
    if (c == 'p') {
      if (! VS1053Paused()) {
        Serial.println("Pause - press p to pause, s to stop");
        VS1053PausePlaying(true);
      } else { 
        Serial.println("Resumed - press p to pause, s to stop");
        VS1053PausePlaying(false);
      }
    }
  }

  delay(100);
}
