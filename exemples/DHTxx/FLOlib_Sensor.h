#ifndef FLOlib_Sensor_h
#define FLOlib_Sensor_h
    
#include <Arduino.h>

// FLOlib Sensor 

// I) DHTxx sensor
// ###--- Summary ---###
// --- Sensors supported: DHT11 | DHT12 | DHT21 | DHT22
// --- Fonctions: float Read_Temperature()
// -------------- float Read_Humidity()
// -------------- bool Read_Temp_and_Hum()
// --- Attributs: float humidity
// -------------- float temperature
// ###---------------###

// I) DHTxx sensor
// ###--- Summary ---###
// --- Sensors supported: DHT11 | DHT12 | DHT21 | DHT22
// --- Fonctions: float Read_Temperature()
// -------------- float Read_Humidity()
// -------------- bool Read_Temp_and_Hum()
// --- Attributs: float humidity
// -------------- float temperature
// ###---------------###

// I) DHTxx sensor
class DHT {
  private:
    float _humidity    = 0;
    float _temperature = 0;
    byte _model;
    byte _pin  ;

  public:
    const float &humidity    = _humidity;
    const float &temperature = _temperature;
    DHT(char* model, byte pin);
    float Read_Temperature();
    float Read_Humidity();
    bool  Read_Temp_and_Hum();
};
    
#endif