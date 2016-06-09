/*************************************************** 
  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_BMP085.h>
#include <Arduino.h>
#include <math.h>
#include <Print.h>
#include <stdbool.h>
#include <stdint.h>
#include <Wire.h>

Adafruit_BMP085::Adafruit_BMP085() {
}

#ifdef ENABLE_ONLY_HIGH_RES
uint8_t Adafruit_BMP085::begin() {
    //Wire.begin();

    uint8_t err = 0;
    if (read8(0xD0, err) != 0x55)
        return 10;

    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1, err);
    ac2 = read16(BMP085_CAL_AC2, err);
    ac3 = read16(BMP085_CAL_AC3, err);
    ac4 = read16(BMP085_CAL_AC4, err);
    ac5 = read16(BMP085_CAL_AC5, err);
    ac6 = read16(BMP085_CAL_AC6, err);

    b1 = read16(BMP085_CAL_B1, err);
    b2 = read16(BMP085_CAL_B2, err);

    mb = read16(BMP085_CAL_MB, err);
    mc = read16(BMP085_CAL_MC, err);
    md = read16(BMP085_CAL_MD, err);

    return err;
}
#else
uint8_t Adafruit_BMP085::begin(uint8_t mode) {
    if (mode > BMP085_ULTRAHIGHRES)
    mode = BMP085_ULTRAHIGHRES;
    oversampling = mode;

    //Wire.begin();

    uint8_t err = 0;
    if (read8(0xD0, err) != 0x55)
        return 10;

    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1, err);
    ac2 = read16(BMP085_CAL_AC2, err);
    ac3 = read16(BMP085_CAL_AC3, err);
    ac4 = read16(BMP085_CAL_AC4, err);
    ac5 = read16(BMP085_CAL_AC5, err);
    ac6 = read16(BMP085_CAL_AC6, err);

    b1 = read16(BMP085_CAL_B1, err);
    b2 = read16(BMP085_CAL_B2, err);

    mb = read16(BMP085_CAL_MB, err);
    mc = read16(BMP085_CAL_MC, err);
    md = read16(BMP085_CAL_MD, err);
#if (BMP085_DEBUG == 1)
    Serial.print("ac1 = "); Serial.println(ac1, DEC);
    Serial.print("ac2 = "); Serial.println(ac2, DEC);
    Serial.print("ac3 = "); Serial.println(ac3, DEC);
    Serial.print("ac4 = "); Serial.println(ac4, DEC);
    Serial.print("ac5 = "); Serial.println(ac5, DEC);
    Serial.print("ac6 = "); Serial.println(ac6, DEC);

    Serial.print("b1 = "); Serial.println(b1, DEC);
    Serial.print("b2 = "); Serial.println(b2, DEC);

    Serial.print("mb = "); Serial.println(mb, DEC);
    Serial.print("mc = "); Serial.println(mc, DEC);
    Serial.print("md = "); Serial.println(md, DEC);
#endif

    return true;
}
#endif

uint16_t Adafruit_BMP085::readRawTemperature(uint8_t& err) {
    write8(BMP085_CONTROL, BMP085_READTEMPCMD, err);
    delay(5);
#if BMP085_DEBUG == 1
    Serial.print("Raw temp: "); Serial.println(read16(BMP085_TEMPDATA));
#endif
    return read16(BMP085_TEMPDATA, err);
}

uint32_t Adafruit_BMP085::readRawPressure(uint8_t& err) {
    uint32_t raw;

#ifdef ENABLE_ONLY_HIGH_RES
    write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (BMP085_ULTRAHIGHRES << 6), err);
    delay(26);
#else
    write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

    if (oversampling == BMP085_ULTRALOWPOWER)
    delay(5);
    else if (oversampling == BMP085_STANDARD)
    delay(8);
    else if (oversampling == BMP085_HIGHRES)
    delay(14);
    else
    delay(26);
#endif

    raw = read16(BMP085_PRESSUREDATA, err);
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA + 2, err);
#ifdef ENABLE_ONLY_HIGH_RES
    raw >>= (8 - BMP085_HIGHRES);
#else
    raw >>= (8 - oversampling);
#endif

    /* this pull broke stuff, look at it later?
     if (oversampling==0) {
     raw <<= 8;
     raw |= read8(BMP085_PRESSUREDATA+2);
     raw >>= (8 - oversampling);
     }
     */

#if BMP085_DEBUG == 1
    Serial.print("Raw pressure: "); Serial.println(raw);
#endif
    return raw;
}

int32_t Adafruit_BMP085::correctPressure(int32_t UT, int32_t UP) {
    int32_t B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

#if BMP085_DEBUG == 1
    // use datasheet numbers!
    UT = 27898;
    UP = 23843;
    ac6 = 23153;
    ac5 = 32757;
    mc = -8711;
    md = 2868;
    b1 = 6190;
    b2 = 4;
    ac3 = -14383;
    ac2 = -72;
    ac1 = 408;
    ac4 = 32741;
    oversampling = 0;
#endif

    // do temperature calculations
    X1 = (UT - (int32_t) (ac6)) * ((int32_t) (ac5)) / pow(2, 15);
    X2 = ((int32_t) mc * pow(2, 11)) / (X1 + (int32_t) md);
    B5 = X1 + X2;

#if BMP085_DEBUG == 1
    Serial.print("X1 = "); Serial.println(X1);
    Serial.print("X2 = "); Serial.println(X2);
    Serial.print("B5 = "); Serial.println(B5);
#endif

    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t) b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t) ac2 * B6) >> 11;
    X3 = X1 + X2;
#ifdef ENABLE_ONLY_HIGH_RES
    B3 = ((((int32_t) ac1 * 4 + X3) << BMP085_HIGHRES) + 2) / 4;
#else
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
#endif

#if BMP085_DEBUG == 1
    Serial.print("B6 = "); Serial.println(B6);
    Serial.print("X1 = "); Serial.println(X1);
    Serial.print("X2 = "); Serial.println(X2);
    Serial.print("B3 = "); Serial.println(B3);
#endif

    X1 = ((int32_t) ac3 * B6) >> 13;
    X2 = ((int32_t) b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t) ac4 * (uint32_t) (X3 + 32768)) >> 15;

#ifdef ENABLE_ONLY_HIGH_RES
    B7 = ((uint32_t) UP - B3) * (uint32_t) (50000UL >> BMP085_HIGHRES);
#else
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );
#endif

#if BMP085_DEBUG == 1
    Serial.print("X1 = "); Serial.println(X1);
    Serial.print("X2 = "); Serial.println(X2);
    Serial.print("B4 = "); Serial.println(B4);
    Serial.print("B7 = "); Serial.println(B7);
#endif

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

#if BMP085_DEBUG == 1
    Serial.print("p = "); Serial.println(p);
    Serial.print("X1 = "); Serial.println(X1);
    Serial.print("X2 = "); Serial.println(X2);
#endif

    p = p + ((X1 + X2 + (int32_t) 3791) >> 4);
#if BMP085_DEBUG == 1
    Serial.print("p = "); Serial.println(p);
#endif
    return p;
}

float Adafruit_BMP085::correctTemperature(int32_t UT) {
    int32_t X1, X2, B5;     // following ds convention
    float temp;

#if BMP085_DEBUG == 1
    // use datasheet numbers!
    UT = 27898;
    ac6 = 23153;
    ac5 = 32757;
    mc = -8711;
    md = 2868;
#endif

    // step 1
    X1 = (UT - (int32_t) ac6) * ((int32_t) ac5) / pow(2, 15);
    X2 = ((int32_t) mc * pow(2, 11)) / (X1 + (int32_t) md);
    B5 = X1 + X2;
    temp = (B5 + 8) / pow(2, 4);
    temp /= 10;

    return temp;
}

/**************************************************************************/
/*!
 Calculates the altitude (in meters) from the specified atmospheric
 pressure (in hPa), and sea-level pressure (in hPa).
 @param  seaLevel      Sea-level pressure in hPa
 @param  atmospheric   Atmospheric pressure in hPa
 */
/**************************************************************************/
float Adafruit_BMP085::pressureToAltitude(float seaLevel, float atmospheric) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
 Calculates the pressure at sea level (in hPa) from the specified altitude
 (in meters), and atmospheric pressure (in hPa).
 @param  altitude      Altitude in meters
 @param  atmospheric   Atmospheric pressure in hPa
 */
/**************************************************************************/
float Adafruit_BMP085::seaLevelForAltitude(float altitude, float atmospheric) {
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*********************************************************************/

uint8_t Adafruit_BMP085::read8(uint8_t a, uint8_t& err) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    err = Wire.endTransmission(); // end transmission
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 1); // send data n-bytes read
    uint8_t ret = Wire.read(); // receive DATA
    err = Wire.endTransmission(); // end transmission

    return ret;
}

uint16_t Adafruit_BMP085::read16(uint8_t a, uint8_t& err) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    err = Wire.endTransmission(); // end transmission

    Wire.requestFrom(BMP085_I2CADDR, 2); // send data n-bytes read
    Wire.endTransmission(); // end transmission
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    uint16_t ret = Wire.read(); // receive DATA
    ret <<= 8;
    ret |= Wire.read(); // receive DATA
    err = Wire.endTransmission(); // end transmission

    return ret;
}

void Adafruit_BMP085::write8(uint8_t a, uint8_t d, uint8_t& err) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.write(d);  // write data
    err = Wire.endTransmission(); // end transmission
}
