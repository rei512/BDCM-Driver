#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x47

#define I2C_WHO_AM_I_REGISTER 0x75
#define I2C_DATA1_WRITE_REGISTER 0x76
#define I2C_DATA1_READ_REGISTER 0x77
#define I2C_TEMP_SENSOR_REGISTER 0x78
#define I2C_CURRENT_SENSOR_REGISTER 0x79
#define I2C_ANGLE_SENSOR_REGISTER 0x7A

int data1;

#define SDA_PIN 26
#define SCL_PIN 25

#define KELVIN 273.15f
#define NTC_B 3650.0f
#define NTC_R0 10000.0f
#define NTC_K0 (25.0f + 273.15f)
#define NTC_R 10000.0f
#define VDD 3.3f

float convertToVoltage(uint16_t adcValue)
{
	return (float)adcValue * 3.3f / 4096.0f;
}

float convertToReistance(float Voltage)
{
	return (VDD / Voltage - 1.0f) * NTC_R;
}

float ConvertToCelsius(float resistance)
{
	float temp = NTC_B / (log(resistance / NTC_R0) + NTC_B / NTC_K0);
	return temp - KELVIN;
}

float AdcToCelcius(uint16_t adcValue)
{
	return ConvertToCelsius(convertToReistance(convertToVoltage(adcValue)));
}

void setup()
{
	// put your setup code here, to run once:
	Wire.begin(SDA_PIN, SCL_PIN);
	Serial.begin(115200);
}

void loop()
{
	uint8_t buff[10], i;

	// put your main code here, to run repeatedly:
	Wire.beginTransmission(I2C_SLAVE_ADDRESS);
	Wire.write(I2C_TEMP_SENSOR_REGISTER);
	Wire.endTransmission(false);
	Wire.requestFrom(I2C_SLAVE_ADDRESS, 2, true);
	i = 0;
	if (Wire.available())
	{
		buff[0] = Wire.read();
		buff[1] = Wire.read();
	}
	Serial.printf("Temp: %.2f, ", ((buff[0] << 8) | buff[1]) / 100.0f);


	Wire.beginTransmission(I2C_SLAVE_ADDRESS);
	Wire.write(I2C_ANGLE_SENSOR_REGISTER);
	Wire.endTransmission(false);
	Wire.requestFrom(I2C_SLAVE_ADDRESS, 2, true);
	i = 0;
	while (Wire.available())
	{
		buff[i++] = Wire.read();
	}
	Serial.printf("Angle1: %d, Angle2: %d\n", buff[0], buff[1]);
	
	// delay(200);
}