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
#pragma once

#include <stdint.h>
#include "mspidrv.hpp"
#include "mi2cdrv.hpp"
#include "mimudrv.hpp"
#include "mgpiodrv.hpp"
#include "mmagnetmetordrv.hpp"
#include "sys.h"
#ifdef BIT
    #undef BIT
#endif

#define BIT(_idx) (1 << _idx)
#define REG_VAL(_setbits, _clearbits) \
    (regVal) { .setbits = (_setbits), .clearbits = (_clearbits) }

// #define BMI088_ACC_I2C_ADDR1        0x18                 //SDO is low(GND)
// #define BMI088_ACC_I2C_ADDR2        0x19                 //SDO is high(VCC)
// #define BMI088_ACC_DEFAULT_ADDRESS  BMI088_ACC_I2C_ADDR2 //in the LPC54102 SPM-S

#define BMI088_ACC_BGW_CHIPID_VALUE 0x1E
#define BMI088_ACC_BGW_CHIPID       0x00

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACC_STATUS  0x03

#define BMI088_ACCD_X_LSB   0x12
#define BMI088_ACCD_X_MSB   0x13
#define BMI088_ACCD_Y_LSB   0x14
#define BMI088_ACCD_Y_MSB   0x15
#define BMI088_ACCD_Z_LSB   0x16
#define BMI088_ACCD_Z_MSB   0x17
#define BMI088_SENSORTIME_0 0x18
#define BMI088_SENSORTIME_1 0x19
#define BMI088_SENSORTIME_2 0x1A

#define BMI088_INT_STAT_1 0x1D

#define BMI088_INT_TEMP_MSB 0x22
#define BMI088_INT_TEMP_LSB 0x23

#define BMI088_ACC_CONF  0x40
#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_PWR_CONF  0x7C
#define BMI088_ACC_PWR_CTRL  0x7D
#define BMI088_ACC_SOFTRESET 0x7E

// #define BMI088_GYRO_I2C_ADDR1        0x68 //SDO is low(GND)
// #define BMI088_GYRO_I2C_ADDR2        0x69 //SDO is high(VCC)
// #define BMI088_GYRO_DEFAULT_ADDRESS  BMI088_GYRO_I2C_ADDR2

#define BMI088_GRRO_CHIP_ID 0x0F
#define BMI088_CHIP_ID_ADDR 0x00

#define BMI088_RATE_X_LSB_ADDR 0x02
#define BMI088_RATE_X_MSB_ADDR 0x03
#define BMI088_RATE_Y_LSB_ADDR 0x04
#define BMI088_RATE_Y_MSB_ADDR 0x05
#define BMI088_RATE_Z_LSB_ADDR 0x06
#define BMI088_RATE_Z_MSB_ADDR 0x07

#define BMI088_INTR_STAT1_ADDR 0x0A

#define BMI088_RANGE_ADDR 0x0F

#define BMI088_BW_ADDR 0x10

#define BMI088_MODE_LPM1_ADDR 0x11

#define BMI088_BGW_SOFT_RST_ADDR 0x14

#define BMI088_INTR_ENABLE0_ADDR 0x15
#define BMI088_INTR_ENABLE1_ADDR 0x16

#define BMI088_SELECTF_TEST_ADDR 0x3C

#define BMI088_GYRO_RANGE_2000_DPS REG_VAL(0, BIT(2) | BIT(1) | BIT(0))
#define BMI088_GYRO_RANGE_1000_DPS REG_VAL(BIT(0), BIT(2) | BIT(1))
#define BMI088_GYRO_RANGE_500_DPS  REG_VAL(BIT(1), BIT(2) | BIT(0))
#define BMI088_GYRO_RANGE_250_DPS  REG_VAL(BIT(1) | BIT(0), BIT(2))
#define BMI088_GYRO_RANGE_125_DPS  REG_VAL(BIT(2), BIT(1) | BIT(0))

#define BMI088_GYRO_RATE_100  REG_VAL(BIT(2) | BIT(1) | BIT(0), BIT(3))
#define BMI088_GYRO_RATE_200  REG_VAL(BIT(2) | BIT(1), BIT(3) | BIT(0))
#define BMI088_GYRO_RATE_400  REG_VAL(BIT(1) | BIT(0), BIT(3) | BIT(2))
#define BMI088_GYRO_RATE_1000 REG_VAL(BIT(1), BIT(3) | BIT(2) | BIT(0))
#define BMI088_GYRO_RATE_2000 REG_VAL(BIT(0), BIT(3) | BIT(2) | BIT(1))

#define BMI088_ACCEL_RANGE_3_G  0x00
#define BMI088_ACCEL_RANGE_6_G  0x01
#define BMI088_ACCEL_RANGE_12_G 0x02
#define BMI088_ACCEL_RANGE_24_G 0x03

#define BMI088_ACCEL_BW_12_5 0xA5
#define BMI088_ACCEL_BW_25   0xA6
#define BMI088_ACCEL_BW_50   0xA7
#define BMI088_ACCEL_BW_100  0xA8
#define BMI088_ACCEL_BW_200  0xA9
#define BMI088_ACCEL_BW_400  0xAA
#define BMI088_ACCEL_BW_800  0xAB
#define BMI088_ACCEL_BW_1600 0xAC

#define BMI088_ACCEL_RATE_12_5 REG_VAL(BIT(0) | BIT(2), BIT(1) | BIT(3))
#define BMI088_ACCEL_RATE_25   REG_VAL(BIT(1) | BIT(2), BIT(0) | BIT(3))
#define BMI088_ACCEL_RATE_50   REG_VAL(BIT(0) | BIT(1) | BIT(2), BIT(3))
#define BMI088_ACCEL_RATE_100  REG_VAL(BIT(3), BIT(0) | BIT(1) | BIT(2))
#define BMI088_ACCEL_RATE_200  REG_VAL(BIT(0) | BIT(3), BIT(1) | BIT(2))
#define BMI088_ACCEL_RATE_400  REG_VAL(BIT(1) | BIT(3), BIT(0) | BIT(2))
#define BMI088_ACCEL_RATE_800  REG_VAL(BIT(0) | BIT(1) | BIT(3), BIT(2))
#define BMI088_ACCEL_RATE_1600 REG_VAL(BIT(2) | BIT(3), BIT(0) | BIT(1))

#define BMI088_ACCEL_OSR_0 REG_VAL(BIT(5) | BIT(7), BIT(4) | BIT(6))
#define BMI088_ACCEL_OSR_2 REG_VAL(BIT(4) | BIT(7), BIT(5) | BIT(6))
#define BMI088_ACCEL_OSR_4 REG_VAL(BIT(7), BIT(4) | BIT(5) | BIT(6))

#define DIR_READ     0x80
#define DIR_WRITE    0x00
#define M_PI_F       3.1415926f
#define BMI088_ONE_G 9.80665f

struct regVal{
    uint8_t setbits;
    uint8_t clearbits;
};

class bmi088 : public mDev::mImu
{
public:
    bmi088(const char* name, mDev::mSpi* bus,mDev::mGpio* accel_cs,mDev::mGpio* gyro_cs);
    ~bmi088() = default;


    virtual float getTemp()override{return readTemp();};
    virtual int16_t getAccelX()override{return acc[0];};
    virtual int16_t getAccelY()override{return acc[1];};
    virtual int16_t getAccelZ()override{return acc[2];};
    virtual int16_t getGyroX()override{return gyr[0];};
    virtual int16_t getGyroY()override{return gyr[1];};
    virtual int16_t getGyroZ()override{return gyr[2];};
    virtual float getAccelXms2()override{return accgMs[0];};
    virtual float getAccelYms2()override{return accgMs[1];};
    virtual float getAccelZms2()override{return accgMs[2];};
    virtual float getGyroXrad()override{return gyrRad[0];};
    virtual float getGyroYrad()override{return gyrRad[1];};
    virtual float getGyroZrad()override{return gyrRad[2];};
    virtual bool updateData()override
    {
        gyroReadRad();
        accelReadMs2();
        return true;
    }
    mResult init(uint32_t gyroRange = 2000, uint32_t gyroRate = 1000, uint32_t sampleRate = 1600, uint32_t dlpfFreqHz = 145, uint32_t Grange = 12);
private:
    mResult writeCheckedReg(mDev::mGpio* cspin, uint8_t reg, uint8_t val);
    mResult modifyReg(mDev::mGpio* cspin, uint8_t reg, regVal reg_val);
    mResult gyroSetSampleRate(uint32_t frequency_hz);
    mResult gyroSetDlpfFilter(uint16_t frequency_hz);
    mResult gyroSetRange(unsigned max_dps);
    mResult gyroReadRaw();
    mResult gyroReadRad();
    mResult gyroScopeInit(uint32_t gyroRange = 2000, uint32_t gyroRate = 1000);

    //ACC
    mResult accelSetSampleRate(uint32_t frequencyHz);
    mResult accelSetBwpOdr(uint32_t dlpf_freq_hz);
    mResult accelSetRange(uint32_t maxG);
    mResult accelErometerInit(uint32_t sampleRate = 1600, uint32_t dlpfFreqHz = 145, uint32_t Grange = 12);
    mResult accelReadRaw();
    mResult accelReadMs2();
    float readTemp();
    uint32_t readTime();
private:
    int16_t gyr[3];
    float   gyrRad[3];
    int16_t acc[3];
    float   accgMs[3];
    float _gyroRangesSale;
    float _accelRangeScale;
    float _sampleRate;
    mDev::mGpio* _accelCsPin;
    mDev::mGpio* _gyroCsPin;
    mDev::mSpi *_spi;
};

