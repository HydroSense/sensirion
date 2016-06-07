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

#include "sensirion.h"

sensirion::sensirion(int dataPin, int clockPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;

  _lastTempMillis = millis() - SHT_CACHE_MILLIS;
  _lastHumMillis = _lastTempMillis;

  // turn off gpio's
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, INPUT);
}


/* ================  Public methods ================ */

/**
 * Reads the current temperature in degrees Celsius
 */
float sensirion::readTemperatureC()
{
  uint16_t _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;  // for 14 Bit @ 5V
  const float D2 =   0.01; // for 14 Bit DEGC

  // Fetch raw value
  _val = readTemperatureRaw();

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads the current temperature in degrees Fahrenheit
 */
float sensirion::readTemperatureF()
{
  uint16_t _val;                 // Raw value returned from sensor
  float _temperature;       // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;   // for 14 Bit @ 5V
  const float D2 =   0.018; // for 14 Bit DEGF

  // Fetch raw value
  _val = readTemperatureRaw();

  // Convert raw value to degrees Fahrenheit
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float sensirion::readHumidity()
{
  uint16_t _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float C1 = -4.0;       // for 12 Bit
  const float C2 =  0.0405;    // for 12 Bit
  const float C3 = -0.0000028; // for 12 Bit
  const float T1 =  0.01;      // for 14 Bit @ 5V
  const float T2 =  0.00008;   // for 14 Bit @ 5V

  // Command to send to the SHT1x to request humidity
  const uint8_t _gHumidCmd = 0b00000101;

  // check cache
  if ( millis() - _lastHumMillis < SHT_CACHE_MILLIS){
	  return _lastHumVal;

  } else{
	  // Fetch the value from the sensor
	  if ( sendCommandSHT(_gHumidCmd, _dataPin, _clockPin) == SHT_SUCCESS ){
		  waitForResultSHT(_dataPin);
		  _val = getData16SHT(_dataPin, _clockPin);

		  uint8_t rxcrc = rev8bits( readCRC(_dataPin, _clockPin ) );
		  uint8_t mycrc = crc8add( 0, _gHumidCmd );
		  mycrc = crc8add( mycrc, _val );
		  if (mycrc != rxcrc){
			Serial.println("SHT: crc error");
			Serial.print(" Got:      0x"); Serial.println(rxcrc, HEX);
			Serial.print(" Expected: 0x"); Serial.println(mycrc, HEX);
			return NAN;
		  }

		  // Apply linear conversion to raw value
		  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

			// Get current temperature for humidity correction
		  _temperature = readTemperatureC();

		  // Correct humidity value for current temperature
		  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

		  // cache
		  _lastHumVal = _correctedHumidity;
		  _lastHumMillis = millis();

		  return (_correctedHumidity);
	  }
  }
  //error
  return NAN;
}


/* ================  Private methods ================ */

void sensirion::shtDelay(uint16_t arg)
{
	// shtDelay in each clock state. clock period is twice this time.
	// 100 us = 5 KHz
	// 500 us = 0.5 KHz
	// the datasheet says you can go to 1 MHz, but in practice
	// a few KHz seems to be the most reliable.
	delayMicroseconds(200);
}

/**
 * Reads the current raw temperature value
 */
uint16_t sensirion::readTemperatureRaw()
{
  uint16_t _val;

  // Command to send to the SHT1x to request Temperature
  const uint8_t _gTempCmd  = 0b00000011;

  if (millis() - _lastTempMillis < SHT_CACHE_MILLIS ){
	  return _lastTempRaw;
  } else {
	  if ( sendCommandSHT(_gTempCmd, _dataPin, _clockPin) == SHT_SUCCESS ){
		  waitForResultSHT(_dataPin);
		  _val = getData16SHT(_dataPin, _clockPin);
		  uint8_t rxcrc = rev8bits( readCRC(_dataPin, _clockPin ) );
		  uint8_t mycrc = crc8add( 0, _gTempCmd );
		  mycrc = crc8add( mycrc, _val );
		  if (mycrc != rxcrc){
			Serial.println("SHT: crc error");
			Serial.print(" Got:      0x"); Serial.println(rxcrc, HEX);
			Serial.print(" Expected: 0x"); Serial.println(mycrc, HEX);
			return 0;
		  }
		  // cache
		  _lastTempMillis = millis();
		  _lastTempRaw = _val;
		  return (_val);
	  }
  }
  return 0; // error
}

uint8_t sensirion::readStatus()
{
  uint16_t _val;

  // Command to send to the SHT1x to request Status
  const uint8_t _gStatCmd  = 0b00000111;

  sendCommandSHT(_gStatCmd, _dataPin, _clockPin);
  // result is immediate with read status.
  //waitForResultSHT(_dataPin);

  shtDelay(1);
  _val = getData16SHT(_dataPin, _clockPin);

  /*
  // Send the required ack
  digitalWrite(_dataPin, LOW);
  pinMode(_dataPin, OUTPUT);
  shtDelay(1);
  digitalWrite(_clockPin, HIGH);
  shtDelay(1);
  digitalWrite(_clockPin, LOW);
  shtDelay(1);
  pinMode(_dataPin, INPUT);
  */

  // end transmission, turn off gpio's
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, INPUT);

  uint8_t rx_val = _val >> 8;
  uint8_t rx_crc = _val & 0xff;
  uint8_t local_crc = crc8add( 0, _gStatCmd );
  local_crc = crc8add(local_crc, rx_val);

  if (rx_crc != rev8bits(local_crc)){
	Serial.println("SHT: crc error");
	Serial.print(" Got:      0x"); Serial.println(rx_crc, HEX);
	Serial.print(" Expected: 0x"); Serial.println(local_crc, HEX);
  }

  return rx_val;
}

/**
*/
uint8_t sensirion::shiftIn(int _dataPin, int _clockPin, int _bitorder)
{
  uint8_t ret = 0;
  uint8_t i;
  uint8_t _numBits = 8;

  for (i=0; i<_numBits; ++i)
  {
     digitalWrite(_clockPin, HIGH);
     shtDelay(1);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + digitalRead(_dataPin);

     digitalWrite(_clockPin, LOW);
     shtDelay(1);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
  }

  return(ret);
}

/*
 * if ack == SHT_ACK, the return value is 0 on success, 1 if there was no ack.
 */
uint8_t sensirion::shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val, uint8_t ack)
{
	uint8_t i;
	uint8_t rval = SHT_SUCCESS;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));

		shtDelay(1);
		digitalWrite(clockPin, HIGH);
		shtDelay(1);
		if ((ack == SHT_ACK) && (i == 7)) { // last byte prepare for ACK
			pinMode(dataPin, INPUT);
		}
		digitalWrite(clockPin, LOW);
	}

	if (SHT_ACK == ack){
	  shtDelay(1);
	  // Verify we get the correct ack

	  // sensor should be pulling data pin LOW
	  ack = 0;
	  while ((ack < 10) && (digitalRead(_dataPin) != LOW)) {
	    ack++;
	    shtDelay(1);
	  }
	  if (ack>=10){
	    Serial.println("Ack error 0");
	    rval = SHT_FAIL;
	  }
	  digitalWrite(_clockPin, HIGH);
	  shtDelay (1);
	  digitalWrite(_clockPin, LOW);
	}
	return rval;
}

/**
 */
uint8_t sensirion::sendCommandSHT_internal(uint8_t _command, int _dataPin, int _clockPin)
{
  // Transmission Start
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // start sequence clock LOW and data HIGH
  digitalWrite(_clockPin, LOW);
  digitalWrite(_dataPin, HIGH);
  shtDelay(2);

  // DATA falls while SCK high
  // SCK falls while DATA low
  // DATA raises while SCK high
  digitalWrite(_clockPin, HIGH);
  shtDelay(1);
  digitalWrite(_dataPin, LOW);
  shtDelay(1);
  digitalWrite(_clockPin, LOW);
  shtDelay(1);
  digitalWrite(_clockPin, HIGH);
  shtDelay(1);
  digitalWrite(_dataPin, HIGH);
  shtDelay(1);

  // idle clock now goes LOW
  digitalWrite(_clockPin, LOW);
  shtDelay(2);

  return shiftOut(_dataPin, _clockPin, MSBFIRST, _command, SHT_ACK);
}
/**
 */
uint8_t sensirion::sendCommandSHT(uint8_t _command, int _dataPin, int _clockPin)
{
	if (sendCommandSHT_internal(_command, _dataPin, _clockPin) != 0){
	  // no ack, reset the interface and retry ONE time
	  digitalWrite(_dataPin, HIGH);
	  digitalWrite(_clockPin, LOW);
	  pinMode(_dataPin, OUTPUT);
	  pinMode(_clockPin, OUTPUT);
	  digitalWrite(_dataPin, HIGH);
	  digitalWrite(_clockPin, LOW);

	  shtDelay(1);
	  // send >9 clocks to reset interface
	  shiftOut(_dataPin, _clockPin, MSBFIRST, 0xff, SHT_NOACK);
	  shtDelay(1);
	  shiftOut(_dataPin, _clockPin, MSBFIRST, 0xff, SHT_NOACK);
	  // if we reset, do a real 1 ms delay
	  delay(1);

	  return sendCommandSHT_internal(_command, _dataPin, _clockPin);
	} else{
	  // success
	  return SHT_SUCCESS;
	}
}
/**
 */
void sensirion::waitForResultSHT(int _dataPin)
{
  int i;
  int ack;

  pinMode(_dataPin, INPUT);

  for(i= 0; i < 100; ++i)
  {
    shtDelay(10);
    ack = digitalRead(_dataPin);

    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    Serial.println("Ack Error 2"); // Can't do serial stuff here, need another way of reporting errors
  }
}

/**
 */
uint16_t sensirion::getData16SHT(int _dataPin, int _clockPin)
{
  uint16_t val;
  // start with idle colock
  digitalWrite(_clockPin, LOW);
  // Get the most significant bits
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  //val = shiftIn(_dataPin, _clockPin, 8);
  val = shiftIn(_dataPin, _clockPin, MSBFIRST );
  val *= 256;

  // Send the required ack
  digitalWrite(_dataPin, LOW);
  pinMode(_dataPin, OUTPUT);
  shtDelay(1);
  digitalWrite(_clockPin, HIGH);
  shtDelay(1);
  digitalWrite(_clockPin, LOW);
  shtDelay(1);
  pinMode(_dataPin, INPUT);

  // Get the least significant bits
  //val |= shiftIn(_dataPin, _clockPin, 8);
  val |= shiftIn(_dataPin, _clockPin, MSBFIRST);

  return val;
}

/**
 */
uint8_t sensirion::readCRC(int _dataPin, int _clockPin )
{
	uint8_t crc;

	// Send the required ack
	digitalWrite(_dataPin, LOW);
	pinMode(_dataPin, OUTPUT);
	shtDelay(1);
	digitalWrite(_clockPin, HIGH);
	shtDelay(1);
	digitalWrite(_clockPin, LOW);
	shtDelay(1);
	pinMode(_dataPin, INPUT);

	// Get the crc value

	crc = shiftIn(_dataPin, _clockPin, MSBFIRST);

	// end comm, turn off gpio's
	pinMode(_dataPin, INPUT);
	pinMode(_clockPin, INPUT);

	return crc;
	/*
	uint8_t local_crc = crc8( (uint8_t*)&data, 2);

	if (crc != local_crc){
		Serial.println("SHT: crc error");
		Serial.print(" Got:      0x"); Serial.println(crc, HEX);
		Serial.print(" Expected: 0x"); Serial.println(local_crc, HEX);
	}
	*/
}
// ref http://contiki.sourceforge.net/docs/2.6/a00178_source.html
uint8_t sensirion::rev8bits(uint8_t v)
{
	uint8_t r = v;
	uint8_t s = 7;

	for (v >>= 1; v; v >>= 1) {
		r <<= 1;
		r |= v & 1;
		s--;
	}
	r <<= s;                  /* Shift when v's highest bits are zero */
	return r;
}
uint8_t sensirion::crc8add(uint8_t crc, const uint16_t data)
{
	crc = crc8add(crc, (uint8_t)(data >> 8));
	return crc8add(crc, (uint8_t)(data & 0xff));
}
uint8_t sensirion::crc8add(uint8_t crc, const uint8_t data)
{
/*
 * based on contiki's sht11 crc
 * http://contiki.sourceforge.net/docs/2.6/a00178_source.html
 *
 */
  const uint8_t POLYNOMIAL = 0x31;
  uint8_t i;

  crc ^= data;
  for ( i = 8; i > 0; --i ) {
	  crc = ( crc & 0x80 )
				? (crc << 1) ^ POLYNOMIAL
				: (crc << 1);
  }

  return crc;
}
