/**
 * sensirion_h Library
 *
 * Hydrosense 2016 / hydrosense.net
 *
 * Based on previous work by:
 *    Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15, SHT31)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#ifndef sensirion_h
#define sensirion_h

#include <Arduino.h>

#define SHT_ACK 1
#define SHT_NOACK 0
#define SHT_RESET 1
#define SHT_NORESET 0
#define SHT_SUCCESS 0
#define SHT_FAIL 0xff

// if less than this many milliseconds,
// report the last reading.
#define SHT_CACHE_MILLIS 1000

class sensirion
{
  public:
	sensirion(int dataPin, int clockPin);
    float readHumidity();
    float readTemperatureC();
    float readTemperatureF();
    uint8_t readStatus();
  private:
    int _dataPin;
    int _clockPin;
    int _numBits;
    unsigned long _lastTempMillis;
    unsigned long _lastHumMillis;
    uint16_t _lastTempRaw;
    float _lastHumVal;

    void shtDelay(uint16_t);
    uint16_t readTemperatureRaw();
    uint8_t shiftIn(int _dataPin, int _clockPin, int _bitoder);
    uint8_t shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val, uint8_t ack);
    uint8_t sendCommandSHT_internal(uint8_t _command, int _dataPin, int _clockPin);
    uint8_t sendCommandSHT(uint8_t _command, int _dataPin, int _clockPin);
    void waitForResultSHT(int _dataPin);
    uint16_t getData16SHT(int _dataPin, int _clockPin);
    uint8_t readCRC(int _dataPin, int _clockPin);
    uint8_t rev8bits(uint8_t v);
    uint8_t crc8add(uint8_t crc, const uint16_t data);
    uint8_t crc8add(uint8_t crc, const uint8_t data);
};

#endif
