# What is this?

This is a port of [SMSPlus](http://www.bannister.org/software/sms.htm) to the Espressif ESP31 chip. It will run
most Sega Master System roms at full speed with video disabled. (However, because of the low speed
of the SPI LCD used and the overhead of pushing pixels to it, the framerate is limited to 15-20FPS for now.)

# What do I need to use this?

You will need:
* A board containg an ESP31 chip and at least 2MB (16Mbit) of SPI flash, plus the tools to program it.
* A backup of a Sega Master System ROM game cartridge
* A 320x240 ILI9341 display, controllable by a 4-wire SPI interface. You can find modules with this LCD by
looking for '2.2 inch SPI 320 240 ILI9341' on eBay or other shopping sites. We used the one with the red PCB and the
SD-card on the bottom.
* Optional, for sound: A I2S codec, plus some hardware to make the sound it outputs be heard. We used a PCM5102
board here, but other I2S codec boards are also capable as long as they only require BCLK, DATA and LRCLK signals 
to work. 
* Optional, but highly recommended: A Playstation 1 (PSX) or Playstation 2 controller to actually play the game

# How do I hoop up my board?

**LCD:**
    Reset - GPIO18
    /CS   - GPIO19
    CLK   - GPIO20
    MOSI  - GPIO21
    D/C   - GPIO22
(Make sure to also wire up the backlight and power pins.)

**I2S codec**
    LRCLK - GPIO32 (also called 'WS')
    DATA  - GPIO33
    BCK   - GPIO27
(Obviously, also hook up the power pins and connect the sound output to an amp or headphones or so.)

**PSX/PS2 controller**
    DATA  - GPIO25
    CLOCK - GPIO14
    ATT   - GPIO16
    CMD   - GPIO17
(Pinouts can be found [here](http://www.gamesx.com/controldata/psxcont/psxcont.htm). Hook VCC up to 3.3V.
The 9 volt line (pin 3) and the ACK pin can be left unconnected.)

# How do I program the chip?

Using a tool capable of flashing the SPI flash connected to the ESP31, program the following files to the following
addresses:
    bin/boot.bin        -> 0x000000
    bin/irom1.bin       -> 0x004000
    bin/irom0_flash.bin -> 0x040000
    (SMS game ROM image)-> 0x140000
Because of copyright reasons, you will have to supply the game rom image yourself.


# License/legal

SMSPlus is licensed under the GPL2. The Espressif support code is licensed under the MIT license. All 
trademarks, service marks, trade names and product names appearing in these files are the property of 
their respective owner(s).