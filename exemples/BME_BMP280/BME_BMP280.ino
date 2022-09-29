#include "FLOlib_Sensor.h"

// Setup sensor -> SDA / SCL -> bus I2C
BME_P280 sensor;

void setup() {
  Serial.begin(115200);
  Serial.print("Sensor begin state: "); Serial.println( sensor.Begin() );
}

void loop() {
  sensor.Read_Temp_Hum_Pres();
  Serial.println("###- NEW READING -###");
  
  Serial.print("Temperature: ");
  Serial.print(sensor.temperature);
  Serial.print("°C");

  Serial.print("  |  ");

  Serial.print("Humidity: ");
  Serial.print(sensor.humidity);
  Serial.print("%");

  Serial.print("  |  ");

  Serial.print("Pressure: ");
  Serial.print(sensor.pressure);
  Serial.print("Pa");

  Serial.println(); Serial.println();
  delay(100);
}
