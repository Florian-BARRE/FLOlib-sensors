#include <Arduino.h>
#include "FLOlib_Sensor.h"

// I) DHTxx sensor

// ###--- Summary ---###
// 1) Constantes
// 2) Fonctions utilitaires
// 3) Fonctions publiques
// 3)a. Initialisation sensor's model and pin
// 3)a. Name: DHT(char* model, byte pin) return void, Ex: DHT sensor("DHT22", 5);
// 3)b. Read temperature
// 3)b. Name: Read_Temperature(), return: float, Ex: sensor.Read_Temperature();
// 3)c. Read humidity
// 3)c. Name: Read_Humidity(), return: float, Ex: sensor.Read_Humidity();
// 3)d. Read Temperature & Humidity
// 3)d. Name: Read_Temp_and_Hum(), return: bool, Ex: sensor.Read_Temp_and_Hum();
// ###---------------###

// 1) Constantes
#ifndef SDHT_H
#define SDHT_H

#define DHT11 0
#define DHT12 1
#define DHT21 2
#define DHT22 3
#define NonExistantDHTxx 4

#define SDHT_CYCLES microsecondsToClockCycles(200)

#define ErrCom   -997
#define ErrCode  -998
#define ErrModel -999

#define humidity_offset 0.1
#define temperature_offset 0
// 2) Fonctions utilitaires
uint16_t pulse(uint8_t pin, int state) {
    uint16_t cycle = SDHT_CYCLES;
    while ((digitalRead(pin) == state) && cycle--);
    return cycle;
}

// 3) Fonctions publiques
// a. Initialisation sensor's model and pin
DHT::DHT(char* model, byte pin) {
    if     (model == "DHT11") _model = DHT11;
    else if(model == "DHT12") _model = DHT12;
    else if(model == "DHT21") _model = DHT21;
    else if(model == "DHT22") _model = DHT22;
    else _model = NonExistantDHTxx;

    _pin = pin;
}

// b. Read temperature
float DHT::Read_Temperature() {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Start sensor communication
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    pulseIn(_pin, HIGH, (_model < DHT21) ? 20000 : 1000);
    digitalWrite(_pin, HIGH);
    pinMode(_pin, INPUT);

    // If communication error -> return ErrCom 
    if (!pulseIn(_pin, HIGH, 300)) return ErrCom;
    // Reading sensor data
    for (uint8_t i = 0; i < 40; i++) data[i / 8] += data[i / 8] + (pulse(_pin, LOW) > pulse(_pin, HIGH));
    // If reading error -> return ErrCode 
    if (data[4] != uint8_t(data[0] + data[1] + data[2] + data[3])) return ErrCode;
    
    // Choose the correct return in term of the DHTxx model
    switch (_model) {
        // Save the temp value then return it
        case DHT11:
            _temperature = ((data[2] * 10) + data[3]);
            _temperature = double(_temperature) / 10 + temperature_offset;
            return _temperature;

        case DHT12:
            _temperature = ((data[2] * 10) + (data[3] & 0x7F)) * ((data[3] & 0x80) ? -1 : 1);
            _temperature = double(_temperature) / 10 + temperature_offset;
            return _temperature;
          
        case DHT21:
        case DHT22:
            _temperature = (uint16_t((data[2] & 0x7F) << 8) | data[3]) * ((data[2] & 0x80) ? -1 : 1);
            _temperature = double(_temperature) / 10 + temperature_offset;
            return _temperature;

        // If no model is matched -> return ErrModel
        default: return ErrModel;
      }
}

// c. Read humidity
float DHT::Read_Humidity() {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Start sensor communication
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    pulseIn(_pin, HIGH, (_model < DHT21) ? 20000 : 1000);
    digitalWrite(_pin, HIGH);
    pinMode(_pin, INPUT);

    // If communication error -> return ErrCom 
    if (!pulseIn(_pin, HIGH, 300)) return ErrCom;
    // Reading sensor data
    for (uint8_t i = 0; i < 40; i++) data[i / 8] += data[i / 8] + (pulse(_pin, LOW) > pulse(_pin, HIGH));
    // If reading error -> return ErrCode 
    if (data[4] != uint8_t(data[0] + data[1] + data[2] + data[3])) return ErrCode;
    
    // Choose the correct return in term of the DHTxx model
    switch (_model) {
        // Save the temp value then return it
        case DHT11:
            _humidity = (data[0] * 10) + data[1];
            _humidity = double(_humidity) / 10 + humidity_offset;
            return _humidity;

        case DHT12:
            _humidity = (data[0] * 10) + data[1];
            _humidity = double(_humidity) / 10 + humidity_offset;
            return _humidity;
          
        case DHT21:
        case DHT22:
            _humidity = (uint16_t(data[0] << 8) | data[1]);
            _humidity = double(_humidity) / 10 + humidity_offset;
            return _humidity;

        // If no model is matched -> return ErrModel
        default: return ErrModel;
      }
}

// d. Read temperature & humidity
bool DHT::Read_Temp_and_Hum() {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    // Start sensor communication
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    pulseIn(_pin, HIGH, (_model < DHT21) ? 20000 : 1000);
    digitalWrite(_pin, HIGH);
    pinMode(_pin, INPUT);

    // If communication error -> return false
    if (!pulseIn(_pin, HIGH, 300)) return false;
    // Reading sensor data
    for (uint8_t i = 0; i < 40; i++) data[i / 8] += data[i / 8] + (pulse(_pin, LOW) > pulse(_pin, HIGH));
    // If reading error -> return ErrCode 
    if (data[4] != uint8_t(data[0] + data[1] + data[2] + data[3])) return false;

    // Choose the correct return in term of the DHTxx model
    switch (_model) {
        case DHT11:
            _humidity = (data[0] * 10) + data[1];
            _humidity = double(_humidity) / 10 + humidity_offset;

            _temperature = (data[2] * 10) + data[3];
            _temperature = double(_temperature) / 10 + temperature_offset;
            return true;

        case DHT12:
            _humidity = (data[0] * 10) + data[1];
            _humidity = double(_humidity) / 10 + humidity_offset;

            _temperature = ((data[2] * 10) + (data[3] & 0x7F)) * ((data[3] & 0x80) ? -1 : 1);
            _temperature = double(_temperature) / 10 + temperature_offset;
            return true;

        case DHT21:
        case DHT22:
            _humidity = (uint16_t(data[0] << 8) | data[1]);
            _humidity = double(_humidity) / 10 + humidity_offset;

            _temperature = (uint16_t((data[2] & 0x7F) << 8) | data[3]) * ((data[2] & 0x80) ? -1 : 1);
            _temperature = double(_temperature) / 10 + temperature_offset;
            return true;

        default: return false;
    }
}

#endif