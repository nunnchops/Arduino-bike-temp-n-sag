# bike-temp-n-sag

##Welcome to the Open Source Arduino based Motorcycle telemetry project... 
This was created based on a requirement for an on-board tire temperature monitor from a bike racing friend. 
Now its turned into a challenge and a lot more...  

There are even possibilities to use the temperature gauge on modern downhill mountain bikes, make wireless 
telemetry access for the features using an Arduino Yun and its onboard Linux OS as well as a whole host of applications. ENJOY and be creative! if you do create something from this project please credit ALL of the contributors and associated sites listed below as its their hard work and openness that has enabled this fun project as well. Many thanks to everyone who helped out and posted their Open Source code on the net. 

## The Challenge

Using Arduino or "Home" electronics and Open Source software (everything created for this project is free and Open to all) how much of the MotoGP style telemetry can we re-create? 

##Features

The first code to post here includes the following research sites and credits (see below) and demonstrates the following functionality: 

-Infra red Tire temperature gauge: measuring Front and back Left, Middle and Right tire temperatures and outputting them on an onboard 16x2 LCD screen.

-Ultrasonic static sag measurement tool. This uses two SRF05 Ultrasonic sensors that can be temporarily mounted under the tail unit and front fork bottom yoke to measure Static sag at the press of a button. 
The concept is, just like the professional racers: in the pit garage the bike is lifted at the rear until the rear wheel is off the ground and a button is pressed, measurement 1 is taken, the bike is lowered again and again the button is pressed and measurement 2 is taken with or without the rider on board. The difference is calculated and the static sag is printed on the screen in milli meters. Its a quick and easy method of checking suspension setup.    

I have already started to tune the accuracy, with the help of related articles we can take the ambient temperature from one of the MLX90614 Infra red tire temp sensors and use it to calculate the speed of sound for our Ultrasonic measurement. This improves the accuracy due to the sensitivity of temperature and barometric pressure on speed of sound calculations :) 

## Hardware

The budget for the project was under 100 GBP and I managed that with the prototype costing approx 40 GBP.  
You can source a lot of these shields at really reasonable prices so hunt around: 
 *  MAiN HARDWARE LIST:
 *  1 X ARDUINO NANO
 *  1 X Hitachi 16x2 LCD
 *  1 X LCD i2c Serial interface 
 *  1 X BUTTON
 *  2 X SRF05 ULTRASONiC SENSOR (if measuring both front and back sag)
 *  6 X MLX90614 (if monitoring Left, Middle and Right on both front and back tire temps)

##Pics and Blog of prototype to follow here 

##Credits
Huge thanks to all of the following and credit for code snippets and example sketches

 *  MLX90614 EVAL info:
    https://learn.sparkfun.com/tutorials/mlx90614-ir-thermometer-hookup-guide?_ga=1.251311686.1757702733.1459322548
 
 *  multiple MLX90614 devices:
    http://www.chrisramsay.co.uk/posts/2014/09/arduino-and-multiple-mlx90614-sensors/
 
 *  ARDUINO YUN info:
    https://www.arduino.cc/en/Main/ArduinoBoardYun
 
 *  Reading from multiple I2C devices on the I2C bus: 
    http://www.electroschematics.com/9798/reading-temperatures-i2c-arduino/
 
 *  Updated i2C Master lib used instead of Wire.h: 
    http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library
    http://playground.arduino.cc/Main/WireLibraryDetailedReference
   
    http://www.instructables.com/id/Ultrasonic-Range-detector-using-Arduino-and-the-SR/


