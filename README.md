# DIP-REFLOW
An all through-hole, Arduino powered, reflow oven controller.

Have you ever wondered why most toaster oven reflow controllers require a shield on a development board,
or the controller has a number of surface mounted components on the PCB?  I did too and I didn't want to
sacrifice a development board, that I frequently use and tear apart, into a long-term installation such
as a reflow controller.

This reflow controller is based on RocketScream's Arduino shield based reflow controller, however, this is
a fully hand solderable, all through-hole implementaion.  It makes use of a MAX31855 breakout board
available from Adafruit (http://www.adafruit.com/product/269) to read the K-type thermocouple.

The Arduino code has been pieced together from several sources to add the features and to get the 
functionality that I was looking for. 

**I still need to finish up a Bill-of-Materials list and cleanup the code somewhat.  Boards will be available on
OSHPark for a quick purchase or you can send off the .BRD file to your fab shop of choice.  I'll also look
at rendering a quality PDF file for those that might want to try and hand craft the PCB at home.  Note, this is
a two sided board.**

I will have this project also tracked at http://hackaday.io for more information such as assembly photos.
