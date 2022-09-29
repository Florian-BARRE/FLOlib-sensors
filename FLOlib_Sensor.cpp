#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "FLOlib_Sensor.h"

/*
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif
*/

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

// II) BME/P 280 sensor

// ###--- Summary ---###
// 1) Constantes
// 1)a. BME280 Definitions 
// 1)b. BME280 Registers 
// 1)c. BME280 Register bitfield positions 
// 1)d. BME280 Register bitfield masks
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
// a. BME280 Definitions 
#define BME280_LIB_VERSION              0x10    
#define BME280_ADDRESS                  0x76   
#define BME280_CHIP_ID                  0x60   

// b. BME280 Registers 
// Temperature
#define BME280_REG_DIG_T1               0x88
#define BME280_REG_DIG_T2               0x8A
#define BME280_REG_DIG_T3               0x8C
// Pressure
#define BME280_REG_DIG_P1               0x8E
#define BME280_REG_DIG_P2               0x90
#define BME280_REG_DIG_P3               0x92
#define BME280_REG_DIG_P4               0x94
#define BME280_REG_DIG_P5               0x96
#define BME280_REG_DIG_P6               0x98
#define BME280_REG_DIG_P7               0x9A
#define BME280_REG_DIG_P8               0x9C
#define BME280_REG_DIG_P9               0x9E
// Humidity
#define BME280_REG_DIG_H1               0xA1
#define BME280_REG_DIG_H2               0xE1
#define BME280_REG_DIG_H3               0xE3
#define BME280_REG_DIG_H4               0xE4
#define BME280_REG_DIG_H5               0xE5
#define BME280_REG_DIG_H6               0xE7

#define BME280_REG_CHIPID               0xD0
#define BME280_REG_VERSION              0xD1
#define BME280_REG_RESET                0xE0
#define BME280_REG_CTRLHUM              0xF2
#define BME280_REG_STATUS               0xF3
#define BME280_REG_CONTROL              0xF4
#define BME280_REG_CONFIG               0xF5
#define BME280_REG_PDATA                0xF7
#define BME280_REG_TDATA                0xFA
#define BME280_REG_HDATA                0xFD

// c. BME280 Register bitfield positions 
#define BME280_BITMASK_HOS_SHIFT        0
#define BME280_BITMASK_POS_SHIFT        2
#define BME280_BITMASK_TOS_SHIFT        5
#define BME280_BITMASK_FILTER_SHIFT     2
#define BME280_BITMASK_MODE_SHIFT       0
#define BME280_BITMASK_FILTER_SHIFT     2
#define BME280_BITMASK_PERIOD_SHIFT     5
#define BME280_BITMASK_MEASURING_SHIFT  3
#define BME280_BITMASK_IMUPDATE_SHIFT   0

// d. BME280 Register bitfield masks 
#define BME280_BITMASK_HOS              (0x07 << BME280_BITMASK_HOS_SHIFT)
#define BME280_BITMASK_POS              (0x07 << BME280_BITMASK_POS_SHIFT)
#define BME280_BITMASK_TOS              (0x07 << BME280_BITMASK_TOS_SHIFT)
#define BME280_BITMASK_MODE             (0x03 << BME280_BITMASK_MODE_SHIFT)
#define BME280_BITMASK_FILTER           (0x07 << BME280_BITMASK_FILTER_SHIFT)
#define BME280_BITMASK_PERIOD           (0x07 << BME280_BITMASK_PERIOD_SHIFT)
#define BME280_BITMASK_MEASURING        (0x01 << BME280_BITMASK_MEASURING_SHIFT)
#define BME280_BITMASK_IMUPDATE         (0x01 << BME280_BITMASK_IMUPDATE_SHIFT)
#define BME280_BITMASK_RESET            0xB6

#define BME280_DEFAULT_HOS      (bmeo4      << BME280_BITMASK_HOS_SHIFT)
#define BME280_DEFAULT_TOS      (bmeo2      << BME280_BITMASK_TOS_SHIFT)
#define BME280_DEFAULT_POS      (bmeo16     << BME280_BITMASK_POS_SHIFT)
#define BME280_DEFAULT_MODE     (bmemNormal << BME280_BITMASK_MODE_SHIFT)
#define BME280_DEFAULT_PERIOD   (bmep250ms  << BME280_BITMASK_PERIOD_SHIFT)
#define BME280_DEFAULT_FILTER   (bmef8      << BME280_BITMASK_FILTER_SHIFT)
// 2) Fonctions utilitaires
uint8_t BME_P280::read8(uint8_t reg) {
  readStart(reg, 1);
  return Wire.read();
}
uint16_t BME_P280::read16(uint8_t reg) {
  readStart(reg, 2);
  uint16_t data;
  data   = (uint8_t)Wire.read();
  data <<= 8;
  data  |= (uint8_t)Wire.read();
  return data;
}
uint16_t BME_P280::read16BE(uint8_t reg) {
  uint16_t data = read16(reg);
  return (data << 8) | (data >> 8);
}
uint32_t BME_P280::read24(uint8_t reg) {
  readStart(reg, 3);
  uint32_t data;
  data   = (uint8_t)Wire.read();
  data <<= 8;
  data  |= (uint8_t)Wire.read();
  data <<= 8;
  data  |= (uint8_t)Wire.read();
  return data >> 4;
}
void BME_P280::send(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
void BME_P280::write(uint8_t reg, uint8_t value) {
  do send(reg, value);
  while (value != read8(reg));
}
void BME_P280::rmw(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
  uint8_t data = read8(reg);

  uint8_t tmp;
  if (reg != BME280_REG_CONTROL)
    tmp = read8(BME280_REG_CONTROL);
  write(BME280_REG_CONTROL, 0);

  value <<=  shift;
  value  &=  mask;
  data   &= ~mask;
  data   |=  value;
  write(reg, data);

  if (reg != BME280_REG_CONTROL)
    write(BME280_REG_CONTROL, tmp);
}
void BME_P280::readStart(uint8_t reg, uint8_t size) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(BME280_ADDRESS, size);
  while (Wire.available() < size);
}
void BME_P280::reset() {
  send(BME280_REG_RESET, BME280_BITMASK_RESET);
  delay(2);
}
void BME_P280::mode(BME280mode_t x) {
  rmw(BME280_REG_CONTROL, BME280_BITMASK_MODE, BME280_BITMASK_MODE_SHIFT, (uint8_t)x);
}
void BME_P280::period(BME280period_t x) {
  rmw(BME280_REG_CONFIG, BME280_BITMASK_PERIOD, BME280_BITMASK_PERIOD_SHIFT, (uint8_t)x);
}
void BME_P280::filter(BME280filter_t x) {
  rmw(BME280_REG_CONFIG, BME280_BITMASK_FILTER, BME280_BITMASK_FILTER_SHIFT, (uint8_t)x);
}
void BME_P280::oversampling(BME280oversampling_t x, char channel) {
  channel &= ~0x20;

  if (channel == 'T')
    rmw(BME280_REG_CONTROL, BME280_BITMASK_TOS, BME280_BITMASK_TOS_SHIFT, (uint8_t)x);

  if (channel == 'P')
    rmw(BME280_REG_CONTROL, BME280_BITMASK_POS, BME280_BITMASK_POS_SHIFT, (uint8_t)x);

  if (channel == 'H')
    rmw(BME280_REG_CTRLHUM, BME280_BITMASK_HOS, BME280_BITMASK_HOS_SHIFT, (uint8_t)x);
}
void BME_P280::oversamplingT(BME280oversampling_t x) {
  rmw(BME280_REG_CONTROL, BME280_BITMASK_TOS, BME280_BITMASK_TOS_SHIFT, (uint8_t)x);
}
void BME_P280::oversamplingP(BME280oversampling_t x) {
  rmw(BME280_REG_CONTROL, BME280_BITMASK_POS, BME280_BITMASK_POS_SHIFT, (uint8_t)x);
}
void BME_P280::oversamplingH(BME280oversampling_t x) {
  rmw(BME280_REG_CTRLHUM, BME280_BITMASK_HOS, BME280_BITMASK_HOS_SHIFT, (uint8_t)x);
}
// 3) Fonctions publiques
bool BME_P280::Begin() {
  Wire.begin();
  delay(2);

  if (read8(BME280_REG_CHIPID) != BME280_CHIP_ID)
    return true;

  reset();

  t1 = (uint16_t)read16BE(BME280_REG_DIG_T1);
  t2 = (int16_t)read16BE(BME280_REG_DIG_T2);
  t3 = (int16_t)read16BE(BME280_REG_DIG_T3);

  p1 = (uint16_t)read16BE(BME280_REG_DIG_P1);
  p2 = (int16_t)read16BE(BME280_REG_DIG_P2);
  p3 = (int16_t)read16BE(BME280_REG_DIG_P3);
  p4 = (int16_t)read16BE(BME280_REG_DIG_P4);
  p5 = (int16_t)read16BE(BME280_REG_DIG_P5);
  p6 = (int16_t)read16BE(BME280_REG_DIG_P6);
  p7 = (int16_t)read16BE(BME280_REG_DIG_P7);
  p8 = (int16_t)read16BE(BME280_REG_DIG_P8);
  p9 = (int16_t)read16BE(BME280_REG_DIG_P9);

  h1 = (uint8_t  )read8(BME280_REG_DIG_H1);
  h2 = (int16_t  )read16BE(BME280_REG_DIG_H2);
  h3 = (uint8_t  )read8(BME280_REG_DIG_H3);
  h4 = ((uint16_t)read8(BME280_REG_DIG_H4 + 0) << 8) + (uint8_t)(read8(BME280_REG_DIG_H4 + 1) << 4);
  h5 = ((uint16_t)read8(BME280_REG_DIG_H5 + 1) << 8) + (uint8_t)(read8(BME280_REG_DIG_H5 + 0) << 0);
  h6 = (int8_t   )read8(BME280_REG_DIG_H6);
  h4 >>= 4;
  h5 >>= 4;

  write(BME280_REG_CTRLHUM, BME280_DEFAULT_HOS );
  write(BME280_REG_CONTROL, BME280_DEFAULT_POS    | BME280_DEFAULT_TOS | BME280_DEFAULT_MODE );
  write(BME280_REG_CONFIG,  BME280_DEFAULT_PERIOD | BME280_DEFAULT_FILTER );
  return false;
}

void BME_P280::Read_Temp_Hum_Pres() {
  
  x = read24(BME280_REG_TDATA);
  x >>= 4;  x -= t1;
  t_fine  = t3;  t_fine *= x;  t_fine >>= 16;
  t_fine += t2;  t_fine *= x;  t_fine >>= 10;
    x = t_fine; x -= 128000L;

  y = r = x;  y *= x;  y >>= 16;  y *= p6;  y >>= 1;
  r *= p5;    y += r;  y >>= 3;   r  = p4;  r <<= 15;  y += r;

  r = 1L << 20;
  r -= read24(BME280_REG_PDATA); //  r -= 306093;
  r <<= 11;  r -= y;

  y = x;   y *= p3;  y >>= 8;  y *= x;  y >>= 12;
  x *= p2; y += x;   y >>= 8;  y += 1L << 27;
  y = ((uint64_t)y * p1) >> 19; //  y >>= 12; y *= p1; y >>= 7;

  r = (uint64_t)r * 3125 / y; //  r *= 3125;  r /= y;
  p = x = r;  p *= p9;  p >>= 18;  p *= r;  x  *= p8;  p += x;
  p >>= 8;    x  = p7;  x <<= 9;   p += x;  r <<= 11;  p += r;  p >>= 13;
 
   x = t_fine;  x -= 76800L;

  r = y = x;  r *= h3;  r += 1L << 26;  r >>= 16;  y *= h6;
  y >>= 16;   r *= y;   r += 1L << 20;  r >>= 12;  r *= h2;  r >>= 8;

  x *= h5;  y = -(uint32_t)h4;  y <<= 6;
  y += read16(BME280_REG_HDATA); //  y += 22123;
  y <<= 14;   y -= x;   y >>= 14;       r *= y;    r >>= 8;

  h = r;     r *= h1;   r = -r;   r += 1L << 27;   r >>= 16;  h *= r;

  if (h < 0             ) humidity = 0;
  if (h > (100UL << 19) ) humidity = 100.0;
  humidity = (float)h / float(1UL << 19);
 
  temperature = (float)t_fine / 5120.0;

  pressure = p;
}

#endif