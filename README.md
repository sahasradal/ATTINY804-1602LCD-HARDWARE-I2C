# ATTINY804-1602LCD-HARDWARE-I2C
simple demo program to use ATTINY804 hardware I2C peripheral to drive a HITACHI HD44780 Chinese clone with PCF8574T-I2C backpack

simple demo program written in ASSEMBLER to use ATTINY804 hardware I2C peripheral to drive a HITACHI HD44780 Chinese clone with PCF8574T-I2C backpack.
 MCU = ATTINY 804.
 LCD = hitachiHD44780chinese clone with PCF8574T-I2C backpack.
 IDE =  Atmel studio 7.
 This LCD module PCF8574T with backpack does not ack. I had to comment out the ACK procedure to get this work other wise garbage is printed on the LCD.
 POLARISU UPDIPROG was used with CH340FTDI module to program the ATTINY804 from window7 PC (thanks to the programmer) github has the details & commands, easy to use.
 Disconnect all conncetions from the MCU except + ,- & updi when programming other wise the programming may corrupt.
 Change main: to suit your needs.
 The TWI pins works directly without setting them as output, changing TWI BAUD between 50 -180 didnt affect the display visually.I dont have any measuring instruments to check actual values.
 Simple hardware TWI procedure is used . no buss error is taken into account, works for simple hobby projects.
 PB1 = SDA = pin8 , PB0 = SCL = pin9. use with external pullup of 4k7 on each pin.
 Programming- vcc = pin1,gnd =pin14, PA0=UPDI=pin10.
 The delay subroutines were taken from 4 bit LCD code written by Donald Weiman    (weimandn@alfredstate.edu) .     
 Grattitude to the neumerous individuals who have posted on the internet from which I have taken shamelessly. 
