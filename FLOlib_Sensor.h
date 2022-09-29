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

// II) BME/P 280 sensor
// ###--- Summary ---###
// --- PINs: SDA & SCL, BUS: I2C
// --- Fonctions: bool Begin()
// -------------- void Read_Temp_Hum_Pres()
// --- Attributs: float    humidity
// -------------- float    temperature
// -------------- uint32_t pressure
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

// II) BME/P 280 sensor
class BME_P280 {
  private:
    typedef enum {      
      bmeoSkip = 0,     
      bmeo1,            
      bmeo2,            
      bmeo4,            
      bmeo8,            
      bmeo16            
    } BME280oversampling_t;

    typedef enum {      
      bmemSleep = 0,   
      bmemForced,      
      bmemNormal = 3    
    } BME280mode_t;

    typedef enum {      
      bmefOff = 0,      
      bmef2,        
      bmef4,        
      bmef8,        
      bmef16        
    } BME280filter_t;

    typedef enum {      
      bmep0_5ms = 0,    
      bmep62_5ms,       
      bmep125ms,        
      bmep250ms,        
      bmep500ms,        
      bmep1s,           
      bmep10ms,         
      bmep20ms          
    } BME280period_t;

    int32_t     t_fine, x, y, r, p, h;
    uint8_t     h1, h3;
    int8_t      h6;
    uint16_t    t1, p1;
    int16_t     t2, t3, p2, p3, p4, p5, p6, p7, p8, p9, h2, h4, h5;

    uint8_t     read8(uint8_t reg);
    uint16_t    read16(uint8_t reg);
    uint16_t    read16BE(uint8_t reg);
    uint32_t    read24(uint8_t reg);
    void        readStart(uint8_t reg, uint8_t size);
    void        send(uint8_t reg, uint8_t value);
    void        write(uint8_t reg, uint8_t value);
    void        rmw(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);

    void        reset();                    

    void        mode(BME280mode_t x);       
    void        period(BME280period_t x);   
    void        filter(BME280filter_t x);   

    void        oversampling(BME280oversampling_t x, char channel); 
    void        oversamplingT(BME280oversampling_t x);              
    void        oversamplingP(BME280oversampling_t x);              
    void        oversamplingH(BME280oversampling_t x);              

  public:
    float    temperature;
    float    humidity   ;
    uint32_t pressure   ;

    bool Begin();  
    void Read_Temp_Hum_Pres();     
};
#endif