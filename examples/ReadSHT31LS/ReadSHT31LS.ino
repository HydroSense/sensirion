/**
 * ReadSHT31LS
 *
 * Read temperature and humidity values from an SHT31-LS 
 * May also work with the SHT1x series (SHT10, SHT11, SHT15) sensors.
 *
 * Copyright 2016 Alan Marchiori amm042@bucknell.edu
 * www.hydrosense.net
 */

#include <sensirion.h>

// Specify data and clock connections and instantiate sensirion object
#define dataPin  2
#define clockPin 3
#define ledPin 13

sensirion sht(dataPin, clockPin);

void setup()
{
   Serial.begin(9600); // Open serial connection to report values to host
   Serial.println("Starting up");
   pinMode(ledPin, OUTPUT);
}

void loop()
{
  float temp_c;
  float temp_f;
  float humidity;

  // blink
  digitalWrite(ledPin, (digitalRead(ledPin) == HIGH) ^ 1);

  if (1){
    uint16_t status = sht.readStatus();
    Serial.print("Status: 0x"); Serial.println(status, HEX);
  }

  if (1){
    humidity = sht.readHumidity();
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
  }

  if (1){
    temp_c = sht.readTemperatureC(); 
    Serial.print("Temperature: "); Serial.print(temp_c, DEC); Serial.println(" C");
  }
  
  if (1){
    temp_f = sht.readTemperatureF(); 
    Serial.print("Temperature: "); Serial.print(temp_f, DEC); Serial.println(" F");
  }
  
  delay(2000);
}
