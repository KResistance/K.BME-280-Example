#include<Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // SCL=D1, SDA=D2, VIN=3.3V, GND=GND
uint8_t address = BME280_ADDRESS_ALTERNATE; // or BME280_ADDRESS
Adafruit_BME280::sensor_sampling sampling = Adafruit_BME280::SAMPLING_X1;

void setup() {
  Serial.begin(115200); 
  if (!bme.begin(address)) {
            Serial.print("Could not find a valid BME280 sensor, check wiring, address, sensor ID! Used addr:"); Serial.println(address);
            Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
            Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
            Serial.print("        ID of 0x60 represents a BME 280.\n");
            Serial.print("        ID of 0x61 represents a BME 680.\n");
            while (1);
        }
  bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      sampling,
      sampling,
      sampling,
      Adafruit_BME280::FILTER_OFF);
  delay(1000);
}

void loop() {
  bme.takeForcedMeasurement();
  Serial.print("Temperature =\t");
  Serial.println(bme.readTemperature());
  Serial.print("Humidity =\t");
  Serial.println(bme.readHumidity());
  Serial.print("Pressure = \t");
  Serial.println(bme.readPressure()/100.0F);
  Serial.println("-----------");
  delay(5000);
}