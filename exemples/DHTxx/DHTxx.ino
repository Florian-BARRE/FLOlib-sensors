#include "FLOlib_Sensor.h"

// Setup sensor -> DHT22 sensor, PIN 5
DHT sensor("DHT22", 5);

void setup() {
    Serial.begin(9600);
}

void loop() { 
  Serial.println("###- NEW READING -###");
  Serial.println("#- Double reading: temperature & humidity");
  Serial.print("#-- Reading success state: ");
  Serial.println(sensor.Read_Temp_and_Hum());
  
  Serial.print("Temperature: ");
  Serial.print(sensor.temperature);
  Serial.print("°C");

  Serial.print("  |  ");

  Serial.print("#-- Humidity: ");
  Serial.print(sensor.humidity);
  Serial.print("%");

  Serial.println();
  delay(2000);
  
  Serial.println("#- Simple reading: temperature");
  Serial.print("#-- Temperature: ");
  Serial.print(sensor.Read_Temperature());
  Serial.print("°C");

  Serial.println();
  delay(2000);

  Serial.println("#- Simple reading: humidity");
  Serial.print("#-- Humidity: ");
  Serial.print(sensor.Read_Humidity());
  Serial.print("%");

  Serial.println();Serial.println();
  delay(2000);
}
