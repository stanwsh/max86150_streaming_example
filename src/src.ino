/*

  Hardware Connections (Breakout
  board to Arduino):
  -5V = 5V
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  See the output on the Arduino Plotter utlity by:
  1) Program the code to your Arduino
  2) Place your left hand finger and the right hand finger on the two ECG electrode pads
  3) In the Arduino IDE, Open Tools->'Serial Plotter'
  4) Make sure the drop down is set to 115200 baud
  5) See your ECG and heartbeat

*/

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "max86150.h"

MAX86150 max86150Sensor;

#define debug Serial      //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21

int16_t ecgsigned16;
uint16_t ppgunsigned16;


void setup()
{
    debug.begin(115200);
    debug.println("MAX86150 Basic Readings Example");

    // Initialize sensor
    if (max86150Sensor.begin(Wire, I2C_SPEED_FAST) == false)
    {
        debug.println("MAX86150 was not found. Please check wiring/power. ");
        while (1); // block setup
    }

    max86150Sensor.setup(); //Configure sensor
}

void loop()
{
    if(max86150Sensor.check()>0)
    {
        ecgsigned16 = (int16_t) (max86150Sensor.getFIFOECG()>>2);
        ppgunsigned16 = (uint16_t) (max86150Sensor.getFIFORed()>>2);

        debug.print(millis());
        debug.print('\t');

        debug.print(ecgsigned16);
        debug.print('\t');
        debug.println(ppgunsigned16);
        delay(100);
    }
}
