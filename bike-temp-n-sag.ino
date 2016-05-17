

/* This software is licenced under the following licence: 
 *  
 *  Creative Commons Attribution-ShareAlike 4.0 https://creativecommons.org/licenses/by-sa/4.0/
 *  
 *  
 *  Creator: rog@linux.demon.co.uk
 *
 *  Additional mention and thanks to existing Arduino library contributors and device handling code 
 *  see Credits and thanks and with special mention to: SRF05, MLX90614, Arduino Nano and Arduino YUM Docs and 
 *  resources 
 *  
 *  CREDITS useful links and THANKS to all listed here: 
 *  
 *  MLX90614 EVAL info
 *  https://learn.sparkfun.com/tutorials/mlx90614-ir-thermometer-hookup-guide?_ga=1.251311686.1757702733.1459322548
 *
 *  multiple MLX90614 devices
 *  http://www.chrisramsay.co.uk/posts/2014/09/arduino-and-multiple-mlx90614-sensors/
 *
 *  ARDUINO YUN info
 *  https://www.arduino.cc/en/Main/ArduinoBoardYun
 *
 *  Reading from multiple I2C devices on the I2C bus: 
 *  http://www.electroschematics.com/9798/reading-temperatures-i2c-arduino/
 *
 *  Updated i2C Master lib used instead of Wire.h 
 *  http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library
 *  http://playground.arduino.cc/Main/WireLibraryDetailedReference
 *   
 *  http://www.instructables.com/id/Ultrasonic-Range-detector-using-Arduino-and-the-SR/
 *
 *  Serial LCD: 
 *  LCD is Hitachi HD44780 Device 
 *  with added serial driver to enable Nano to use it. 
 *  http://playground.arduino.cc/Learning/SerialLCD
 *   
 *  Debouncing switches (TODO) 
 *  http://www.avdweb.nl/arduino/hardware-interfacing/simple-switch-debouncer.html
 *  
 *  MAiN HARDWARE LIST:
 *  1 X ARDUINO NANO
 *  1 X BUTTON
 *  2 X SRF05 ULTRASONiC SENSOR (if measuring both front and back sag)
 *  6 X MLX90614 (if monitoring both front and back tire temps)
 *  
 *  Keep it OPEN! keep it real! and above all ENJOY! May the Source be with you all.....
*/


#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <i2cmaster.h>
// #include <NewPing.h>                                                 // not needed at the moment its a nice to have there is also an MLX90614 library
// #include <SparkFunMLX90614.h>                                        // not needed 

#define ECHOPINR 6                                                      // Pin to receive echo pulse rear
#define TRIGPINR 7                                                      // Pin to send trigger pulse rear
#define ECHOPINF 4                                                      // Pin to receive echo pulse front
#define TRIGPINF 5                                                      // Pin to send trigger pulse front
#define BACKLIGHT_PIN 13
#define PORTC2 2                                                        // No pullups on 9064 MLX 90614 breakout board
#define PORTC3 3
#define PORTD8 10                                                       // Enable pullups on port 8 if possible so that button works


LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);          //use find i2c code initially for confirming your Serial LCD device address here
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);          //ALTERNATIVE LCD DEVICE NOTATION ONLY

int i = 0;
long delta0;
long delta[5];
int sensors = 1;                                                        // set to 2 for two sensors if 1 then its just rear too complex
char* where[1] = {""};                                                  

int irreff = 80;                                                        // set referrence temp for front tyre here
int irrefr = 80;                                                        // set referrence temp for rear tyre here

/* ir i2c addresses set these for your devices: code on net and in credits and thanks at the top of this code */
int irad1 = 0x5A << 1;
int irad2 = 0x5B << 1;
int irad3 = 0x5C << 1;
int irad4 = 0x5D << 1;
int irad5 = 0x5E << 1;
int irad6 = 0x5F << 1;

void setup() {
  Serial.begin(9600);
  pinMode(ECHOPINR, INPUT);
  pinMode(TRIGPINR, OUTPUT);
  pinMode(ECHOPINF, INPUT);
  pinMode(TRIGPINF, OUTPUT);
  pinMode(PORTD8, INPUT);                                              // set button as a digital input
  digitalWrite (PORTD8, HIGH);                                         // set button 8 high


  // Switch on the backlight
  pinMode ( BACKLIGHT_PIN, OUTPUT );
  digitalWrite ( BACKLIGHT_PIN, HIGH );

  lcd.begin (16, 2);                                                    // initialize the lcd

  lcd.home ();                                                          // go home
  lcd.print(" STATIC SAG TOOL VER 1.0 ");
  lcd.setCursor ( 0, 1 );                                               // go to the next line
  lcd.print ("   ++LOUBEE MOTO++     ");
  delay ( 1000 );

}

boolean SagMode = false;
boolean IrMode = false;
boolean DEBUG = true;
boolean buttonshort;                                                    // trying to get two modes out of a single button press
boolean buttonlong;

void loop() {
  int c = Serial.read();                                                // read from USB-CD would be nice to pop this in an if statement if connected to usb
  long measureSag(int, int);

  //if (c != -1) {                                                      // got anything? for use with serial monitor character input not button mode change
  int button_status = button();
  if (button_status) {                                                  // short button press is sag long is temp
    if (SagMode == false) {                                             // if we aren't in command mode...
      if ( DEBUG ) {
        if (button_status) {
          Serial.println("Hit the button and hold for sag, short press for temp mode");
        } else {
          Serial.println("Hit S to enter sag mode T for tyre temp");    // otherwise write char to UART only relevant for serial monitor mode change not button
        }
      }
      //if (c == 's') {
      if (button_status == 2) {
        SagMode = true;
        if (i < 4) {                                                    // still looping in sag mode
          if (i < 2) {
            delta[i] = measureSag(TRIGPINR, ECHOPINR);                  // read on rear sensor first
          } else if (i > 1) {
            delta[i] = measureSag(TRIGPINF, ECHOPINF);                  // read on front sensor
          }
          lcd.home();                                                   // go home on the LCD
          if ( DEBUG ) {
            Serial.print("iteration=");
            Serial.println(i);
            Serial.print("delta=");
            Serial.println(delta[i]);
          }

          if (i == 0) {
            lcd.clear();
            //lcd.setCursor (0, 0);
          } else if (i == 1) {
            lcd.setCursor (8, 0);
          } else if (i == 2) {
            lcd.setCursor (0, 0);
          } else if (i == 3) {
            lcd.setCursor (8, 0);
          }
          lcd.print("M");
          lcd.print(i);
          lcd.print(":");
          lcd.print(delta[i]);
          lcd.print("mm            ");
          if ( DEBUG ) {
            Serial.println(i);
          }
          if (i < 1) { //go round again
            i += 1;
            SagMode = false;
            return;
          } else if (i == 2) {
            i += 1;
            SagMode = false;
            return;
          }

          /* if we reach here we should have an i value of 3 and 4 values in delta array */

          if (i == 1) {
            /* if we get this far we have two values to manipulate to work out rear sag:
              find the larger value and take one from the other */
            long maxdelta = max(delta[0], delta[1]);
            if ( DEBUG ) {
              Serial.print("rear maxdelta=");
              Serial.println(maxdelta);
            }
            long mindelta = min(delta[0], delta[1]);
            long sag = 0;
            sag = maxdelta - mindelta;
            if ( DEBUG ) {
              Serial.print("Calculated rear Sag =");
              Serial.println(sag);
            }
            lcd.setCursor (0, 1);
            //lcd.print(where[1] );
            lcd.print("RSAG:");
            lcd.print(sag);
            lcd.print("mm");
            SagMode = false;
            i += 1;
            return;
          } else if (i == 3) {
            //Now work out front sag from delta array
            long maxdelta = max(delta[2], delta[3]);
            if ( DEBUG ) {
              Serial.print("front maxdelta=");
              Serial.println(maxdelta);
            }
            long mindelta = min(delta[2], delta[3]);
            long sag = 0;
            sag = maxdelta - mindelta;
            if ( DEBUG ) {
              Serial.print("Calculated Front Sag =");
              Serial.println(sag);
            }
            lcd.setCursor (8, 1);
            //lcd.print(where[1] );
            lcd.print("FSAG:");
            lcd.print(sag);
            lcd.print("mm");

            //reset and start again
            i = 0;
            SagMode = false;
            return;
          }
        }
      }
      //if (c == 't') {                                                  // replaced this serial read with long button press TODO set boolean for mode selector
      if (button_status == 1) {                                          // long button press
        bool TempMode = true;
        if ( DEBUG ) {
          Serial.println("running IR mode");                             // we actually want to run this whether they enter the sag mode or not so no logic required here.
        }
        if (sensortest()) {
          if ( DEBUG ) {
            Serial.println("getting temps");
          }
          //gettemps(irad1, irad2, irad3, irad4, irad5, irad6);
          while (TempMode) {
            if ( DEBUG ) {
              Serial.println(" running function to get temps from sensors ");
            }
            float tempf1 = ReadTemp(irad1, 0);
            float tempf2 = ReadTemp(irad2, 0);
            float tempf3 = ReadTemp(irad3, 0);
            //float tempr1 = ReadTemp(irad4, 0);                         // uncomment these lines for additional sensors
            //float tempr2 = ReadTemp(irad5, 0);
            //float tempr3 = ReadTemp(irad6, 0);


            /* then we call printtemps */
            //int  printtemps(float tempf1, float tempf2, float tempf3, float tempr1, float tempr2, float tempr3, float ambient1);
            //void  printtemps(float tempf1, float tempf2);
            //gettemps(irad1);
            delay(2000);                                                  // controls how long before sensor read iterations min length = time to read all 6 sensors if fitted
          }
        }

        /* simple structure here: break out into loop to set temp values
           temps show LOW if not greater than or equal to stored values
           sensors are probed front to back by address from main while loop
           sensor check -> readtemp -> printtemp*/
      }
    }
    // its possible that we could put the IR mode in here however we would not loop on serial.read
  }
}

int button() {                    //returns short or long 1 or 2 depending on duration of press TODO: DE BOUNCE THE BUTTON

  int buttonlong = 1;
  int buttonshort = 2;
  int button_delay = 0;
  int BUTTON = digitalRead(PORTD8);

  if (DEBUG) {
    Serial.println(BUTTON);
  }

  while (BUTTON == 0) {         // we drag the button port low on grounding through the button
    delay(500);
    button_delay++;
    if (DEBUG) {
      Serial.println("HOLDING");
    }
    BUTTON = digitalRead(PORTD8);
  }

  if (button_delay > 10) {
    return buttonlong;
  } else if (button_delay > 2) {
    if (button_delay < 10) {
    }
    //
    return buttonshort;
  }

}

long  measureSag(int TRIGPIN, int ECHOPIN) {
  /* loop n times and populate distance_start - distance_max then sag = distance_max - distance_start
     May have to take this snippet out of the loop and create its own function measure_sag()
     May be better to work out which is biggest and take the smaller value from it.
     adjust this to use two TRIGPIN and ECHOPIN */
  float tempamb = ReadTemp(irad2, 1);                                    // reads the ambient temperature from the Front middle IR sensor
  //float tempamb = 20; //debug only remove later
  //lcd.clear();
  //lcd.println(" Ambient Temperature = ");
  //lcd.print(tempamb);
  digitalWrite(TRIGPIN, LOW);                                            // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);                                           // Send a 10uS high to trigger ranging
  delayMicroseconds(10);     //this was 10 microseconds
  digitalWrite(TRIGPIN, LOW);                                            // Send pin low again
  long distance = pulseIn(ECHOPIN, HIGH);                                // Read in times pulse
  if (tempamb) {
    distance = (((float) distance * (331.3 + 0.606 * tempamb)) / 2) / 1000;      // Thanks to MagicByCalvin instructables
  } else {
    distance = (((float) distance / 58) * 10);                           // Calculate distance from time of pulse * 10 for mm
  }
  if ( DEBUG ) {
    Serial.print("distance measured =");
    Serial.println(distance);
  }
  delay(50);                                                             // Adjust delay based on minimum time to settle
  return distance;
}

int  sensortest() {
  //lcd.clear();
  //lcd.println("-Sensor checks-");                                      // Not sure how to fail nicely here :) TODO
  Serial.print("sensor-checks");
}

float ReadTemp(int Address, bool ambient) {
  int data_low = 0;
  int data_high = 0;
  int pec = 0;
  PORTC = (1 << PORTC2) | (1 << PORTC3);                                // MLX90614 enable pullups on ports for temp reads this may affect lcd i2c


  if ( DEBUG ) {
    Serial.println("> Read temperature");
    // Inform the user
    Serial.print("  MLX address: ");
    Serial.print(Address, HEX);
    Serial.print(", ");
  }
  i2c_start_wait(Address + I2C_WRITE);
  // Address of temp bytes
  if (ambient) {
    i2c_write(0x06);
  } else {
    i2c_write(0x07);                                                    // note that ambient temperature address is 0x06 computation is the same
  }
  // Read - the famous repeat start
  i2c_rep_start(Address + I2C_READ);
  // Read 1 byte and then send ack (x2)
  data_low = i2c_readAck();
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();

  /* This converts high and low bytes together and processes the temperature
     MSB is a error bit and is ignored for temperatures
     Zero out the data */
  float temp = 0x0000;
  float celcius = 0x0000;
  float fahrenheit = 0x0000;

  /* This masks off the error bit of the high byte, then moves it left
     8 bits and adds the low byte. */
  temp = (float)(((data_high & 0x007F) << 8) + data_low);
  celcius = (temp * 0.02) - 273.16;
  fahrenheit = (celcius * 1.8) + 32;
  if ( DEBUG ) {
    if (ambient) {
      Serial.print("AMBIENT TEMP");
    }
    Serial.print(" ");
    Serial.print(celcius);
    Serial.print(" C,");
    Serial.print(" ");
    Serial.print(fahrenheit);
    Serial.print(" F");
  }

  //printtemps fahrenheit and address so we can print based on location
  if (ambient) {
    return celcius;                                                    // so we pass the celcius ambient temperature back to the distance equation for accuracy
  } else {
    printtemps(fahrenheit, Address);                                   // sub fahrenheit/celcius here if you want alternative output
  }
}

void printtemps(float temp, int address) {
  if ( DEBUG ) {
    Serial.println(" running function to print obtained temps ");
  }
  if (address == irad1) {
    lcd.clear();
    lcd.setCursor (0, 0);                                              // Front on first line
    lcd.print("F ");
    if (temp > irreff) {
      lcd.print(temp, 1);                                  // Reduced the number of decimal places to 1 due to 16X2 display size if you have better display increase
      lcd.print("");
    } else {
      //lcd.clear();
      //delay(500);
      lcd.setCursor (0, 0);
      lcd.print("F LOW");
    }
  }
  if (address == irad2) {
    lcd.setCursor (7, 0);
    //lcd.print(" ");
    if (temp > irreff) {
      lcd.print(temp, 1);
      lcd.print("");
    } else {
      //lcd.clear();               
      //delay(500);
      lcd.setCursor (7, 0);
      lcd.print( "LOW" );
    }
  }
  if (address == irad3) {
    lcd.setCursor (12, 0);
    //lcd.print(" ");
    if (temp > irreff) {
      lcd.print(temp, 1);
      lcd.print("");
    } else {
      //lcd.clear();
      //delay(500);
      lcd.setCursor (12, 0);
      lcd.print( "LOW" );
    }
  }
  if (address == irad4) {
    lcd.clear();
    lcd.setCursor (0, 0);                                              // Rear on second line
    lcd.print("F ");
    if (temp > irreff) {
      lcd.print(temp, 1);
      lcd.print("");
    } else {
      //lcd.clear();
      //delay(500);
      lcd.setCursor (0, 0);
      lcd.print("R LOW");
    }
  }
  if (address == irad5) {
    lcd.setCursor (7, 0);
    //lcd.print(" ");
    if (temp > irreff) {
      lcd.print(temp, 1);
      lcd.print("");
    } else {
      //lcd.clear();               // in order to flash nicely we may need to clear more than one character but function does not take this
      //delay(500);
      lcd.setCursor (7, 0);
      lcd.print( "LOW" );
    }
  }
  if (address == irad6) {
    lcd.setCursor (12, 0);
    //lcd.print(" ");
    if (temp > irreff) {
      lcd.print(temp, 1);
      lcd.print("");
    } else {
      //lcd.clear();
      //delay(500);
      lcd.setCursor (12, 0);
      lcd.print( "LOW" );
    }
  }
}





