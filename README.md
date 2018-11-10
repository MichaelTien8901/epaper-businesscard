# epaper-businesscard
Use Waveshare e-ink e-paper to build a business card application

## E-Paper 
   The document of the e-paper is in [wiki of WaveShare](https://www.waveshare.com/wiki/2.7inch_e-Paper_HAT).  
   ### Connectors
      It is a 2.7 inches e-paper HAT for Raspberry Pi.  It also has another 8 pin connector for other hardware.  
   ### 4 Buttons
      There are 4 buttons only connected to Raspberry Pi HAT connector.
      
## STM32F070 Nucleo
   In order to access 4 buttons, the Nucleo board connect only to the HAT connector.  There are [schematic circuit diagram]
   (https://www.waveshare.com/wiki/File:2.7inch_e-Paper_HAT_Schematic.pdf) for the e-paper.
   
## Demo code and CubeMX
   The orginal [demo code is available in the link](https://www.waveshare.com/wiki/File:2.7inch-e-paper-hat-code.7z).  I use the 
   CubeMx to create the startup code.  In order to compatible with the demo code, the setting of heap size are set to be 0x2000.  
   
## Conversion of picture 
   There is a program [image2lcd v2.9] provided in the wiki(https://www.waveshare.com/wiki/File:Image2Lcd.7z). There is a document about
   how to convert picture to data array.  However, the steps doesn't work for me.  I use the following steps
   * Create a picture with width x height = 176 x 264
   * Run the image2lcd program
   * Open the picture file
   * Output file type = c array
   * Scan mode = horizon scan
   * BitPixel = monochrome
   * Max width and height = 176 264
   * The following options are all off
   * Check Reverse Color
   Then save the result
   
## Button Function defined
  * Button 1 - Show Business Card
  * Button 2 - Show WeChat QR Code
  * Button 3 - Show Line QR Code
  * Button 4 - Show Demo Picture
