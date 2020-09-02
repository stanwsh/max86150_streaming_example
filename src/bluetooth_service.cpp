#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU

#include "RTClib.h"
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

BLEService deviceStatus("180A");
BLEService batteryService("180F");
BLEService accelerometerService("FFA0");

BLECharacteristic rtcCurrentTime("2A2B", BLERead | BLENotify, 50);
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);
BLEFloatCharacteristic accelerometerXAxis("FFA1", BLERead | BLENotify);
BLEFloatCharacteristic accelerometerYAxis("FFA2", BLERead | BLENotify);
BLEFloatCharacteristic accelerometerZAxis("FFA3", BLERead | BLENotify);
BLECharacteristic accelerometerMessage("FFA4", BLENotify, 30);

// uint32_t lastMilli, thisMilli;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;

// void led_blink(int light = LED_BUILTIN);

volatile uint16_t milliseconds; // volatile important here since we're changing this variable inside an interrupt service routine:
// ISR(D3)
// {
// 	++milliseconds;
// 	if (milliseconds == 999) // roll over to zero
// 		milliseconds = 0;
// }

void dealMilliseconds()
{
	++milliseconds;
	if (milliseconds == 999) // roll over to zero
		milliseconds = 0;
}

// align RTC clock with customized millis clock
// void calibrateMilliseconds(RTC_DS3231 specificRTC)
// {
// 	uint8_t s = specificRTC.now().second();
// 	Serial.print("calibrating: initial second-");
// 	Serial.println(s);
// 	while (uint8_t i = specificRTC.now().second() == s)
// 	{
// 		Serial.print("calibrating:now second-");
// 		Serial.println(i);
// 	}
// 	milliseconds = 0;
// 	Serial.print("calibrating:now second-");
// 	Serial.println(specificRTC.now().second());
// 	Serial.println("Sucessfully calibrated.");
// }

// void calibrateMilliseconds(RTC_DS3231 specificRTC)
// {
// 	// DateTime now;
// 	// now = specificRTC.now();

// 	// specificRTC.adjust(now);
// 	// milliseconds = (uint16_t)0;
// }

void setup()
{
	void dealMilliseconds();
	// void calibrateMilliseconds(RTC_DS3231);

	Serial.begin(115200);
	while (!Serial && (millis() < 5000))
		; // avoiding unnecessary serial monitor opening.

	// lastMilli = 0;
	// thisMilli = 0;
	pinMode(LED_BUILTIN, OUTPUT);

	if (!rtc.begin())
	{
		Serial.println("Couldn't find RTC");
		Serial.flush();
		abort();
	}

	if (rtc.lostPower())
	{
		Serial.println("RTC lost power, let's set the time!");
		// When time needs to be set on a new device, or after a power loss, the
		// following line sets the RTC to the date & time this sketch was compiled
		// rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		rtc.adjust(DateTime(2020, 1, 1, 8, 0, 0));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}

	rtc.writeSqwPinMode(DS3231_SquareWave1kHz);							// set SQW frequency
	attachInterrupt(digitalPinToInterrupt(D3), dealMilliseconds, RISING); // arduino style of ISR

	// calibrateMilliseconds(rtc);

	if (!IMU.begin()) //Initialize IMU sensor
	{
		Serial.println("Failed to initialize IMU!");
		while (1)
			;
	}

	if (!BLE.begin())
	{
		Serial.println("Failed to initialize Bluetooth!");
		while (1)
			;
	}

	BLE.setLocalName("SZTU Wearables");

	BLE.setAdvertisedService(deviceStatus);
	deviceStatus.addCharacteristic(rtcCurrentTime);
	BLE.addService(deviceStatus);

	BLE.setAdvertisedService(batteryService);
	batteryService.addCharacteristic(batteryLevelChar);
	BLE.addService(batteryService);

	BLE.setAdvertisedService(accelerometerService);
	accelerometerService.addCharacteristic(accelerometerXAxis);
	accelerometerService.addCharacteristic(accelerometerYAxis);
	accelerometerService.addCharacteristic(accelerometerZAxis);
	accelerometerService.addCharacteristic(accelerometerMessage);

	BLE.addService(accelerometerService);

	BLE.advertise();
	Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
	BLEDevice central = BLE.central();

	if (central)
	{
		Serial.print("Connected to central: ");
		Serial.println(central.address());
		digitalWrite(LED_BUILTIN, HIGH);

		while (central.connected())
		{
			// DateTime now = rtc.now();
			// rtc.adjust(now);
			// while (uint8_t i = rtc.now().second() == now.second());
			// milliseconds = (uint16_t)0;
			
			char timeStampChar[50];

			uint32_t timeStamp;
			timeStamp = rtc.now().unixtime();
			Serial.print("Timestamp:");
			Serial.println(timeStamp);

			String(timeStamp).toCharArray(timeStampChar, 50);

			// lastMilli = thisMilli;
			// thisMilli = millis();
			char msg[100];
			sprintf(msg, "timestamp:%s millis:%u", timeStampChar, milliseconds);
			// sprintf(msg, "last:%u, this:%u, minus:%u", lastMilli, thisMilli, thisMilli - lastMilli);
			// Serial.println(msg); // timeStamp = timeStamp * 1000 + (thisMilli - lastMilli);
			// Serial.println(timeStamp);
			// Serial.println(timeStampChar);
			Serial.println(msg);
			rtcCurrentTime.writeValue(timeStampChar);

			int battery = analogRead(A0);
			int batteryLevel = map(battery, 0, 1023, 0, 100);
			// Serial.print("Battery Level % is now: ");
			// Serial.println(batteryLevel);
			batteryLevelChar.writeValue(batteryLevel);

			if (IMU.accelerationAvailable())
			{
				IMU.readAcceleration(accel_x, accel_y, accel_z);
				accelerometerXAxis.writeValue(accel_x);
				accelerometerYAxis.writeValue(accel_y);
				accelerometerZAxis.writeValue(accel_z);
				char msg[30];
				sprintf(msg, "x:%.2f,y:%.2f,z:%.2f", accel_x, accel_y, accel_z);
				Serial.println(msg);
				accelerometerMessage.writeValue(msg);
				Serial.print("Accelerometer = ");
				Serial.print(accel_x);
				Serial.print(", ");
				Serial.print(accel_y);
				Serial.print(", ");
				Serial.println(accel_z);
			}

			delay(200);
		}
	}
	digitalWrite(LED_BUILTIN, LOW);
	Serial.print("Disconnected from central: ");
	Serial.println(central.address());
}
