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
#include <ArduinoBLE.h> // compatible BLE library

#include "max86150.h"
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
//
// IMU(Wire1);

/*
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;

//setup
  if (!IMU.begin()) //Initialize IMU sensor 
  { Serial.println("Failed to initialize IMU!"); while (1);}

  //Accelerometer values 
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    Serial.print("Accelerometer = ");Serial.print(accel_x); Serial.print(", ");Serial.print(accel_y);Serial.print(", ");Serial.println(accel_z);
  }
delay (200);

  //Gyroscope values 
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    Serial.print("Gyroscope = ");Serial.print(gyro_x); Serial.print(", ");Serial.print(gyro_y);Serial.print(", ");Serial.println(gyro_z);
  }
delay (200);

  //Magnetometer values 
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mag_x, mag_y, mag_z);
    Serial.print("Magnetometer = ");Serial.print(mag_x); Serial.print(", ");Serial.print(mag_y);Serial.print(", ");Serial.println(mag_z);
  }

*/

MAX86150 max86150Sensor;

#define debug Serial //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21

int16_t ecgsigned16;
uint16_t ppgunsigned16;
uint16_t irunsigned16;
uint16_t counts = 0;

// BLEService batteryService("1101");
// BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

void readregister(void);

void setup()
{
	Serial.begin(115200);
	delay(5000);
	Serial.println("MAX86150 Basic Readings Example");
	while (!Serial && (millis() < 30000))
		; // avoiding unnecessary serial monitor opening.
	// Initialize sensor
	if (max86150Sensor.begin(Wire, I2C_SPEED_FAST) == false)
	{
		Serial.println("MAX86150 was not found. Please check wiring/power. ");
		while (1)
			; // block setup
	}

	max86150Sensor.setup(); //Configure sensor
	readregister();
}

void loop()
{
	if (max86150Sensor.check() > 0)
	{
		ecgsigned16 = (int16_t)(max86150Sensor.getFIFOECG() >> 2);
		ppgunsigned16 = (uint16_t)(max86150Sensor.getFIFORed() >> 2);
		irunsigned16 = (uint16_t)(max86150Sensor.getFIFOIR() >> 2);
		//        debug.print(millis());
		//        debug.print('\t');

		Serial.print(ecgsigned16);
		Serial.print('\t');
		Serial.print(ppgunsigned16);
		Serial.print('\t');
		Serial.println(irunsigned16);

		// debug.println(MAX86150_INTSTAT2);
		//        delay(100);
	}
	//    if (!(counts%10000))
	//    {
	//        readregister();
	//    }
	//    counts++;
}

void readregister(void)
{

	uint8_t reg[22] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x14, 0x15, 0x3C, 0x3E, 0xFF};
	for (int i = 0; i < 22; i++)
	{
		char _str[50];
		sprintf(_str, "%02x\t", max86150Sensor.readRegister8(0x5E, reg[i]));
		Serial.print(_str);
	}
	Serial.println("end.");
}
