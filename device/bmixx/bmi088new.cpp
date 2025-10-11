/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "bmi088new.hpp"
#include "delay.h"
#include "mklog.hpp"

bmi088::bmi088(const char* name, mDev::mSpi* bus,mDev::mGpio* accel_cs,mDev::mGpio* gyro_cs) :
mDev::mImu(name),_accelCsPin(accel_cs), _gyroCsPin(gyro_cs), _spi(bus)
{

}

/* Re-implement this function to define customized rotation */
void bmi088_rotate_to_frd(float* data, uint32_t dev_id)
{
    /* do nothing */
    (void)data;
    (void)dev_id;
}

mResult bmi088::writeCheckedReg(mDev::mGpio* cspin, uint8_t reg, uint8_t val)
{
    uint8_t r_val;
    if(_spi->writeReg(reg, &val, 1, cspin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(_spi->readReg(reg, &r_val, 1, cspin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(_spi->readReg(reg, &r_val, 1, cspin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    } 
    return (r_val == val) ? M_RESULT_EOK : M_RESULT_ERROR;
}

mResult bmi088::modifyReg(mDev::mGpio* cspin, uint8_t reg, regVal regval)
{
    uint8_t value;

    /* In case of read operations of the accelerometer part, the requested data is not sent 
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual 
    reqested register content is transmitted. */
    if(_spi->readReg(reg, &value, 1, cspin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(_spi->readReg(reg, &value, 1, cspin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    } 
    value &= ~regval.clearbits;
    value |= regval.setbits;

    if(writeCheckedReg(cspin, reg, value)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}

mResult bmi088::gyroSetSampleRate(uint32_t frequencyHz)
{
    regVal regval;

    if (frequencyHz <= 150) {
        regval = BMI088_GYRO_RATE_100;
    } else if (frequencyHz <= 300) {
        regval = BMI088_GYRO_RATE_200;
    } else if (frequencyHz <= 700) {
        regval = BMI088_GYRO_RATE_400;
    } else if (frequencyHz <= 1500) {
        regval = BMI088_GYRO_RATE_1000;
    } else if (frequencyHz <= 2000) {
        regval = BMI088_GYRO_RATE_2000;
    } else {
        return M_RESULT_EINVAL;
    }
    if(modifyReg(_gyroCsPin, BMI088_BW_ADDR, regval)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

mResult bmi088::gyroSetDlpfFilter(uint16_t frequencyHz)
{
    /* lpf bw is set by BMI088_BW_ADDR */
    (void)frequencyHz;

    return M_RESULT_EOK;
}

mResult bmi088::gyroSetRange(unsigned maxDps)
{
    regVal regval;
    float lsb_per_dps;

    if (maxDps == 0) {
        maxDps = 2000;
    }

    if (maxDps <= 187) {
        regval = BMI088_GYRO_RANGE_125_DPS;
        lsb_per_dps = 262.4;
    } else if (maxDps <= 375) {
        regval = BMI088_GYRO_RANGE_250_DPS;
        lsb_per_dps = 131.2;
    } else if (maxDps <= 750) {
        regval = BMI088_GYRO_RANGE_500_DPS;
        lsb_per_dps = 65.6;
    } else if (maxDps <= 1500) {
        regval = BMI088_GYRO_RANGE_1000_DPS;
        lsb_per_dps = 32.8;
    } else if (maxDps <= 2000) {
        regval = BMI088_GYRO_RANGE_2000_DPS;
        lsb_per_dps = 16.4;
    } else {
        return M_RESULT_EINVAL;
    }
    if(modifyReg(_gyroCsPin, BMI088_RANGE_ADDR, regval)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }

    _gyroRangesSale = (M_PI_F / (180.0f * lsb_per_dps));

    return M_RESULT_EOK;
}

mResult bmi088::gyroReadRaw()
{
    uint8_t buffer[6];
    if(_spi->readReg(BMI088_RATE_X_LSB_ADDR, (uint8_t*)buffer, 6, _gyroCsPin) != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    gyr[0] = ((buffer[1] << 8) | buffer[0]);
    gyr[1] = ((buffer[3] << 8) | buffer[2]);
    gyr[2] = ((buffer[5] << 8) | buffer[4]);
    return M_RESULT_EOK;
}

mResult bmi088::gyroReadRad()
{
    if(gyroReadRaw()!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    gyrRad[0] = _gyroRangesSale * gyr[0];
    gyrRad[1] = _gyroRangesSale * gyr[1];
    gyrRad[2] = _gyroRangesSale * gyr[2];
    return M_RESULT_EOK;
}

mResult bmi088::gyroScopeInit(uint32_t gyroRange, uint32_t gyroRate)
{
    uint8_t gyroId;

    _spi->readReg(BMI088_CHIP_ID_ADDR, &gyroId, 1, _gyroCsPin);
    if (gyroId != BMI088_GRRO_CHIP_ID)
    {
        KLOGE("Warning: not found BMI088 gyro id: %02x\r\n", gyroId);
        return M_RESULT_ERROR;
    }

    /* soft reset */
    gyroId = 0xB6;
    if(_spi->writeReg(BMI088_BGW_SOFT_RST_ADDR, &gyroId, 1, _gyroCsPin) != M_RESULT_EOK)
    {
        KLOGE("Error: soft reset fail\r\n");
        return M_RESULT_ERROR;
    }
    delay_ms(30);

    if(gyroSetRange(gyroRange)!= M_RESULT_EOK)/* 2000dps */
    {
        KLOGE("Error: setRange fail\r\n");
        return M_RESULT_ERROR;
    }

    if(gyroSetSampleRate(gyroRate)!= M_RESULT_EOK)/* OSR 1000KHz, Filter BW: 116Hz */
    {
        KLOGE("Error: setRange fail\r\n");
        return M_RESULT_ERROR;
    }
    /* enable gyroscope */
    if(modifyReg(_gyroCsPin, BMI088_MODE_LPM1_ADDR, REG_VAL(0, BIT(7) | BIT(5)))!=M_RESULT_EOK)/* {0; 0}  NORMAL mode */
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    delay_ms(1);

    return M_RESULT_EOK;
}

mResult bmi088::accelSetSampleRate(uint32_t frequencyHz)
{
    regVal regval;

    if (frequencyHz <= (125 / 10)) {
        regval = BMI088_ACCEL_RATE_12_5;
        _sampleRate = 12.5;
    } else if (frequencyHz <= 25) {
        regval = BMI088_ACCEL_RATE_25;
        _sampleRate = 25;
    } else if (frequencyHz <= 50) {
        regval = BMI088_ACCEL_RATE_50;
        _sampleRate = 50;
    } else if (frequencyHz <= 100) {
        regval = BMI088_ACCEL_RATE_100;
        _sampleRate = 100;
    } else if (frequencyHz <= 200) {
        regval = BMI088_ACCEL_RATE_200;
        _sampleRate = 200;
    } else if (frequencyHz <= 400) {
        regval = BMI088_ACCEL_RATE_400;
        _sampleRate = 400;
    } else if (frequencyHz <= 800) {
        regval = BMI088_ACCEL_RATE_800;
        _sampleRate = 800;
    } else if (frequencyHz <= 1600) {
        regval = BMI088_ACCEL_RATE_1600;
        _sampleRate = 1600;
    } else {
        return M_RESULT_EINVAL;
    }
    if(modifyReg(_accelCsPin, BMI088_ACC_CONF, regval)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}

mResult bmi088::accelSetBwpOdr(uint32_t dlpfFreqHz)
{
    regVal regval;

    if (_sampleRate <= 12.5) {
        if (dlpfFreqHz <= 1) {
            // 1Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 3) {
            // 2Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 5Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 25) {
        if (dlpfFreqHz <= 4) {
            // 3Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 7) {
            // 5Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 10Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 50) {
        if (dlpfFreqHz <= 7) {
            // 5Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 14) {
            // 9Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 20Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 100) {
        if (dlpfFreqHz <= 14) {
            // 10Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 29) {
            // 19Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 40Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 200) {
        if (dlpfFreqHz <= 29) {
            // 20Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 59) {
            // 38Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 80Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 400) {
        if (dlpfFreqHz <= 52) {
            // 40Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 110) {
            // 75Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 145Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 800) {
        if (dlpfFreqHz <= 110) {
            // 80Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 175) {
            // 140Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 230Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else if (_sampleRate <= 1600) {
        if (dlpfFreqHz <= 199) {
            // 145Hz
            regval = BMI088_ACCEL_OSR_4;
        }
        else if (dlpfFreqHz <= 257) {
            // 234Hz
            regval = BMI088_ACCEL_OSR_2;
        } else {
            // 280Hz
            regval = BMI088_ACCEL_OSR_0;
        }
    } else {
        return M_RESULT_EINVAL;
    }
    if(modifyReg(_accelCsPin, BMI088_ACC_CONF, regval)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

mResult bmi088::accelSetRange(uint32_t maxG)
{
    uint8_t regval;

    if (maxG == 0) {
        maxG = 24;
    }

    if (maxG <= 3) {
        regval = BMI088_ACCEL_RANGE_3_G;
        _accelRangeScale = (3 * BMI088_ONE_G / 32768);
        _adcAcc1G = 32768 / 3;
    } else if (maxG <= 6) {
        regval = BMI088_ACCEL_RANGE_6_G;
        _accelRangeScale = (6 * BMI088_ONE_G / 32768);
        _adcAcc1G = 32768 / 6;
    } else if (maxG <= 12) {
        regval = BMI088_ACCEL_RANGE_12_G;
        _accelRangeScale = (12 * BMI088_ONE_G / 32768);
        _adcAcc1G = 32768 / 12;
    } else if (maxG <= 24) {
        regval = BMI088_ACCEL_RANGE_24_G;
        _accelRangeScale = (24 * BMI088_ONE_G / 32768);
        _adcAcc1G = 32768 / 24;
    } else {
        return M_RESULT_EINVAL;
    }
    if(_spi->writeReg(BMI088_ACC_RANGE, &regval, 1, _accelCsPin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

mResult bmi088::accelErometerInit(uint32_t sampleRate, uint32_t dlpfFreqHz, uint32_t Grange)
{
    uint8_t accelId;
    uint8_t regval;

    /* dummy read to let accel enter SPI mode */
    _spi->readReg(BMI088_ACC_BGW_CHIPID, &accelId, 1, _accelCsPin);

    /* read accel id */
    _spi->readReg(BMI088_ACC_BGW_CHIPID, &accelId, 1, _accelCsPin);
    if (accelId != BMI088_ACC_BGW_CHIPID_VALUE)
    {
        KLOGE("Warning: not found BMI088 accel id: %02x\n", accelId);
        return M_RESULT_ERROR;
    }

    /* soft reset */
    regval = 0xB6;
    if(_spi->writeReg(BMI088_ACC_SOFTRESET, &regval, 1, _accelCsPin)!=M_RESULT_EOK)
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    delay_ms(2);

    /* enter active mode */
    regval = 0x00;
    if(_spi->writeReg(BMI088_ACC_PWR_CONF, &regval, 1, _accelCsPin)!=M_RESULT_EOK)
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    delay_ms(1);

    /* enter normal mode */
    regval = 0x04;
    if(_spi->writeReg(BMI088_ACC_PWR_CTRL, &regval, 1, _accelCsPin)!=M_RESULT_EOK)
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    delay_ms(55);

    /* set default range and bandwidth */
    if(accelSetRange(Grange)!=M_RESULT_EOK)         /* 24g */
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(accelSetSampleRate(sampleRate)!=M_RESULT_EOK)      /* 1600Hz sample rate */
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(accelSetBwpOdr(dlpfFreqHz)!=M_RESULT_EOK)           /* Normal BW */
    {
        KLOGE("Error: %s()%d return\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}

mResult bmi088::accelReadRaw()
{
    uint8_t buffer[7];

    /* In case of read operations of the accelerometer part, the requested data is not sent 
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual 
    reqested register content is transmitted. */

    if(_spi->readReg(BMI088_ACCD_X_LSB, buffer, 7, _accelCsPin) != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    acc[0] = (buffer[2] << 8 | buffer[1]);
    acc[1] = (buffer[4] << 8 | buffer[3]);
    acc[2] = (buffer[6] << 8 | buffer[5]);

    return M_RESULT_EOK;
}

mResult bmi088::accelReadMs2()
{
    if(accelReadRaw()!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }

    accgMs[0] = _accelRangeScale * acc[0];
    accgMs[1] = _accelRangeScale * acc[1];
    accgMs[2] = _accelRangeScale * acc[2];

    return M_RESULT_EOK;
}
float bmi088::readTemp()
{
    /* temperature data */
    uint8_t _buffer[3];
    uint16_t temp_uint11;
    int16_t temp_int11;
    if(_spi->readReg(BMI088_INT_TEMP_MSB, _buffer, 3, _accelCsPin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }

    temp_uint11 = (_buffer[1] * 8) + (_buffer[2] / 32);
    if (temp_uint11 > 1023)
    {
    temp_int11 = temp_uint11 - 2048;
    } 
    else
    {
    temp_int11 = temp_uint11;
    }
    return (float) temp_int11 * 0.125f + 23.0f;
}
uint32_t bmi088::readTime()
{
    uint8_t _buffer[4];
    if(_spi->readReg(BMI088_SENSORTIME_0, _buffer, 4, _accelCsPin)!=M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    /* time data */
    return (_buffer[3] << 16) | (_buffer[2] << 8) | _buffer[1];
}

mResult bmi088::init(uint32_t gyroRange, uint32_t gyroRate, uint32_t sampleRate, uint32_t dlpfFreqHz, uint32_t Grange)
{
    if(gyroScopeInit(gyroRange, gyroRate)!= M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(accelErometerInit(sampleRate, dlpfFreqHz, Grange) != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}