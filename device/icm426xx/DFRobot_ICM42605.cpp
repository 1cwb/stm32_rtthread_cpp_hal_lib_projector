/*!
 * @file DFRobot_ICM42605.cpp
 * @brief Define basic structure of DFRobot_ICM42605 class, the implementation of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangfeng]<feng.yang@dfrobot.com>
 * @version V1.0
 * @date 2021-05-13
 * @url  https://github.com/DFRobot/DFRobot_ICM42605
 */
#include <DFRobot_ICM42605.h>
#include<math.h>
#include "delay.h"
#include "mbase.hpp"
#include "containers.hpp"

DFRobot_ICM42605::DFRobot_ICM42605()
{
  accelConfig0.accelODR = 6;
  accelConfig0.accelFsSel = 0;
  gyroConfig0.gyroODR = 6;
  gyroConfig0.gyroFsSel = 0;
  _gyroRange = 4000/65535.0;
  _accelRange = 0.488f;
  FIFOMode = false;
}

int DFRobot_ICM42605::begin(void)
{
  uint8_t bank = 0;
  printf("begin to write reg\r\n");
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  delay_ms(100);
  uint8_t id=0;
  readReg(ICM42605_WHO_AM_I,&id,1);

  DBG("real sensor id= %d\r\n",id)
  if(id != DFRobot_ICM42605_ID){
    DBG("bus data access error");
    return ERR_IC_VERSION;
  }
  uint8_t reset = 0;
  writeReg(ICM42605_DEVICE_CONFIG,&reset,1);
  delay_ms(2);
  return ERR_OK;
}

float DFRobot_ICM42605::getTemperature(void)
{
  float value;
  if(FIFOMode){
    value = (_temp/2.07) + 25;
  } else{
    uint8_t data[2];
    int16_t value2;
    readReg(ICM42605_TEMP_DATA1, data, 2);
    value2 = ((uint16_t )data[0]<<8) | (uint16_t )data[1];
    value = value2/132.48 + 25;
  }
  return value;
}

float DFRobot_ICM42605::getAccelDataX(void)
{
  float value;
  if(FIFOMode){
    value = _accelX;
  } else{
    uint8_t data[2];
    readReg(ICM42605_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float DFRobot_ICM42605::getAccelDataY(void)
{
  float value;
  if(FIFOMode){
    value = _accelY;
  } else{
    uint8_t data[2];
    readReg(ICM42605_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float DFRobot_ICM42605::getAccelDataZ(void)
{
  float value;
  if(FIFOMode){
    value = _accelZ;
  } else{
    uint8_t data[2];
    readReg(ICM42605_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_accelRange;
}

float DFRobot_ICM42605::getGyroDataX(void)
{
  float value;
  if(FIFOMode){
    value = _gyroX;
  } else{
    uint8_t data[2];
    readReg(ICM42605_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

float DFRobot_ICM42605::getGyroDataY(void)
{
  float value;
  if(FIFOMode){
    value = _gyroY;
  } else{
    uint8_t data[2];
    readReg(ICM42605_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

float DFRobot_ICM42605::getGyroDataZ(void)
{
  float value;
  if(FIFOMode){
    value = _gyroZ;
  } else{
    uint8_t data[2];
    readReg(ICM42605_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*_gyroRange;
}

void DFRobot_ICM42605:: tapDetectionInit(uint8_t accelMode)
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  if(accelMode == 0){
    accelConfig0.accelODR = 15;
    writeReg(ICM42605_ACCEL_CONFIG0,&accelConfig0,1);
    PWRMgmt0.accelMode = 2;
    writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
    delay_ms(1);
    INTFConfig1.accelLpClkSel = 0;
    writeReg(ICM42605_INTF_CONFIG1,&INTFConfig1,1);
    accelConfig1.accelUIFiltORD = 2;
    writeReg(ICM42605_ACCEL_CONFIG1,&accelConfig1,1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    writeReg(ICM42605_GYRO_ACCEL_CONFIG0,&gyroAccelConfig0,1);
  } else if(accelMode == 1){
    accelConfig0.accelODR = 6;
    writeReg(ICM42605_ACCEL_CONFIG0,&accelConfig0,1);
    PWRMgmt0.accelMode = 3;
    writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
    delay_ms(1);
    accelConfig1.accelUIFiltORD = 2;
    writeReg(ICM42605_ACCEL_CONFIG1,&accelConfig1,1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    writeReg(ICM42605_GYRO_ACCEL_CONFIG0,&gyroAccelConfig0,1);
  } else{
    DBG("accelMode invalid !");
    return;
  }
  delay_ms(1);
  bank = 4;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  APEXConfig8.tapTmin = 3;
  APEXConfig8.tapTavg = 3;
  APEXConfig8.tapTmax = 2;
  writeReg(ICM42605_APEX_CONFIG8,&APEXConfig8,1);
  APEXConfig7.tapMinJerkThr = 17;
  APEXConfig7.tapMaxPeakTol = 1;
  writeReg(ICM42605_APEX_CONFIG7,&APEXConfig7,1);
  delay_ms(1);
  INTSource.tapDetIntEn = 1;
  if(_INTPin==1){
    writeReg(ICM42605_INT_SOURCE6,&INTSource,1);
  } else {
    writeReg(ICM42605_INT_SOURCE7,&INTSource,1);
  }
  delay_ms(50);
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  APEXConfig0.tapEnable = 1;
  writeReg(ICM42605_APEX_CONFIG0,&APEXConfig0,1);
}
void DFRobot_ICM42605::getTapInformation()
{
  uint8_t data;
  readReg(ICM42605_APEX_DATA4, &data, 1);
  _tapNum = data & 0x18;
  _tapAxis = data & 0x06;
  _tapDir = data & 0x01;
}
uint8_t DFRobot_ICM42605:: numberOfTap()
{
  return _tapNum;
}
uint8_t DFRobot_ICM42605:: axisOfTap()
{
  return _tapAxis;
}
void DFRobot_ICM42605:: wakeOnMotionInit()
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  accelConfig0.accelODR = 9;
  writeReg(ICM42605_ACCEL_CONFIG0,&accelConfig0,1);
  PWRMgmt0.accelMode = 2;
  writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
  delay_ms(1);
  INTFConfig1.accelLpClkSel = 0;
  writeReg(ICM42605_INTF_CONFIG1,&INTFConfig1,1);
  delay_ms(1);
}
void DFRobot_ICM42605:: setWOMTh(uint8_t axis,uint8_t threshold)
{
  uint8_t bank = 4;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  uint8_t womValue = threshold;
  if(axis == X_AXIS){
    writeReg(ICM42605_ACCEL_WOM_X_THR,&womValue,1);
  } else if(axis == Y_AXIS){
    writeReg(ICM42605_ACCEL_WOM_Y_THR,&womValue,1);
  } else if(axis == Z_AXIS){
    writeReg(ICM42605_ACCEL_WOM_Z_THR,&womValue,1);
  } else if(axis == ALL){
    writeReg(ICM42605_ACCEL_WOM_X_THR,&womValue,1);
    writeReg(ICM42605_ACCEL_WOM_Y_THR,&womValue,1);
    writeReg(ICM42605_ACCEL_WOM_Z_THR,&womValue,1);
  }
  delay_ms(1);
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
}
void DFRobot_ICM42605:: setWOMInterrupt(uint8_t axis)
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  if(_INTPin == 1){
    writeReg(ICM42605_INT_SOURCE1,&axis,1);
  } else {
    writeReg(ICM42605_INT_SOURCE4,&axis,1);
  }
  delay_ms(50);
  SMDConfig.SMDMode = 1;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  writeReg(ICM42605_SMD_CONFIG,&SMDConfig,1);
}
void DFRobot_ICM42605::enableSMDInterrupt(uint8_t mode)
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  uint8_t INT = 1<<3 ;
  if(mode != 0){
    if(_INTPin == 1){
      writeReg(ICM42605_INT_SOURCE1,&INT,1);
    } else {
      writeReg(ICM42605_INT_SOURCE4,&INT,1);
    }
  }
  delay_ms(50);
  SMDConfig.SMDMode = mode;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  writeReg(ICM42605_SMD_CONFIG,&SMDConfig,1);
}

uint8_t DFRobot_ICM42605::readInterruptStatus(uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  readReg(reg,&status,1);
  return status;
}

bool DFRobot_ICM42605::setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  float lsb_per_dps = 0.0f;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
      writeReg(ICM42605_GYRO_CONFIG0,&gyroConfig0,1);
      switch(FSR){
        case FSR_0:
            lsb_per_dps = 16.4f;  // ±2000dps
            break;
        case FSR_1:
            lsb_per_dps = 32.8f;  // ±1000dps
            break;
        case FSR_2:
            lsb_per_dps = 65.6f;  // ±500dps
            break;
        case FSR_3:
            lsb_per_dps = 131.2f; // ±250dps
            break;
        case FSR_4:
            lsb_per_dps = 262.4f; // ±125dps
            break;
        case FSR_5:
            lsb_per_dps = 524.8f; // ±62.5dps
            break;
        case FSR_6:
            lsb_per_dps = 1049.6f; // ±31.25dps
            break;
        case FSR_7:
            lsb_per_dps = 2099.2f; // ±15.625dps
            break;
      }
      _gyroRange = (M_PI_F / (180.0f * lsb_per_dps));
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
      writeReg(ICM42605_ACCEL_CONFIG0,&accelConfig0,1);
      switch(FSR){
        case FSR_0:
          _accelRange = (16 * ICM42605_ONE_G / 32768);
          _adcAcc1G = 32768 / 16;
          break;
        case FSR_1:
          _accelRange = (8 * ICM42605_ONE_G / 32768);
          _adcAcc1G = 32768 / 8;
          break;
        case FSR_2:
          _accelRange = (4 * ICM42605_ONE_G / 32768);
          _adcAcc1G = 32768 / 4;
          break;
        case FSR_3:
          _accelRange = (2 * ICM42605_ONE_G / 32768);
          _adcAcc1G = 32768 / 2;
          break;
      }
    }
  } 
  return ret;
}

void DFRobot_ICM42605::setFIFODataMode()
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  FIFOConfig1.FIFOHiresEn = 0;
  FIFOConfig1.FIFOAccelEn = 1;
  FIFOConfig1.FIFOGyroEn = 1;
  FIFOConfig1.FIFOTempEn = 1;
  FIFOConfig1.FIFOTmstFsyncEn = 0;
  writeReg(ICM42605_FIFO_CONFIG1,&FIFOConfig1,1);

}

void DFRobot_ICM42605::startFIFOMode()
{
  uint8_t bank = 0;
  FIFOMode = true;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  setFIFODataMode();
  uint8_t start = 1<<6;
  writeReg(ICM42605_FIFO_CONFIG,&start,1);
  getFIFOData();
}
void DFRobot_ICM42605::getFIFOData()
{
  uint8_t data[16];
  readReg(ICM42605_FIFO_DATA,data,16);
  _accelX = (uint16_t)data[1]<<8 | (uint16_t)data[2];
  //DBG("_accelX");DBG(_accelX);
  _accelY = (uint16_t)data[3]<<8 | (uint16_t)data[4];
  //DBG("_accelY");DBG(_accelY);
  _accelZ = (uint16_t)data[5]<<8 | (uint16_t)data[6];
  //DBG("_accelZ");DBG(_accelZ);
  _gyroX = (uint16_t)data[7]<<8 | (uint16_t)data[8];
  //DBG("_gyroX");DBG(_gyroX);
  _gyroY = (uint16_t)data[9]<<8 | (uint16_t)data[10];
  //DBG("_gyroY");DBG(_gyroY);
  _gyroZ = (uint16_t)data[11]<<8 | (uint16_t)data[12];
  //DBG("_gyroZ");DBG(_gyroZ);
  _temp = (uint8_t)data[13];
  //DBG("_temp");DBG(data[13]);
}
void DFRobot_ICM42605::sotpFIFOMode()
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  uint8_t start = 1<<7;
  writeReg(ICM42605_FIFO_CONFIG,&start,1);
}

void DFRobot_ICM42605::setINTMode(uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  if(INTPin == 1){
    _INTPin = 1;
    INTConfig.INT1Mode = INTmode;
    INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    INTConfig.INT1Polarity = INTPolarity;
  } else if(INTPin == 2){
    _INTPin = 2;
    INTConfig.INT2Mode = INTmode;
    INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    INTConfig.INT2Polarity = INTPolarity;
  }
  writeReg(ICM42605_INT_CONFIG,&INTConfig,1);
}

void DFRobot_ICM42605::startTempMeasure()
{
  PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
  delay_ms(1);
}
void DFRobot_ICM42605::startGyroMeasure(uint8_t mode)
{
  PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
  delay_ms(1);
}

void DFRobot_ICM42605::startAccelMeasure(uint8_t mode)
{
  PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  writeReg(ICM42605_PWR_MGMT0,&PWRMgmt0,1);
  delay_ms(10);
}
void DFRobot_ICM42605:: setGyroNotchFilterFHz(double freq,uint8_t axis)
{
  uint8_t bank = 1;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  double fdesired = freq * 1000;
  double coswz = cos(2*3.14*fdesired/32);
  int16_t nfCoswz;
  uint8_t nfCoswzSel;
  if(abs(coswz)<=0.875){
    nfCoswz = round(coswz*256);
    nfCoswzSel = 0;
  } else {
    nfCoswzSel = 1;
    if(coswz> 0.875){
      nfCoswz = round(8*(1-coswz)*256);
    } else if(coswz < -0.875){
      nfCoswz = round(-8*(1+coswz)*256);
    }
  }
  if(axis == X_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    writeReg(ICM42605_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC9,&gyroConfigStatic9,1);
  } else if(axis == Y_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    writeReg(ICM42605_GYRO_CONFIG_STATIC7,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC9,&gyroConfigStatic9,1);
  } else if(axis == Z_AXIS){
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    writeReg(ICM42605_GYRO_CONFIG_STATIC8,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC9,&gyroConfigStatic9,1);
  } else if(axis == ALL)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    writeReg(ICM42605_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC7,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC8,&nfCoswz,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC9,&gyroConfigStatic9,1);
  }
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
}

void DFRobot_ICM42605::setGyroNFbandwidth(uint8_t bw)
{
  uint8_t bank = 1;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  uint8_t bandWidth = (bw<<4) | 0x01;
  writeReg(ICM42605_GYRO_CONFIG_STATIC10,&bandWidth,1);
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
}

void DFRobot_ICM42605::setGyroNotchFilter(bool mode)
{
  if(mode){
    gyroConfigStatic2.gyroNFDis = 0;
  } else {
    gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  writeReg(ICM42605_GYRO_CONFIG_STATIC2,&gyroConfigStatic2,1);
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
}
void DFRobot_ICM42605::setAAFBandwidth(uint8_t who,uint8_t BWIndex)
{
  uint8_t bank = 0;
  uint16_t AAFDeltsqr = BWIndex*BWIndex;
  if(who == GYRO){
    bank = 1;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC3,&BWIndex,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(ICM42605_GYRO_CONFIG_STATIC5,&gyroConfigStatic5,1);
    bank = 0;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  } else if(who == ACCEL){
    bank = 2;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(ICM42605_ACCEL_CONFIG_STATIC2,&accelConfigStatic2,1);
    writeReg(ICM42605_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(ICM42605_ACCEL_CONFIG_STATIC4,&accelConfigStatic4,1);

    bank = 0;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  } else if(who == ALL){
    bank = 1;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC3,&BWIndex,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(ICM42605_GYRO_CONFIG_STATIC5,&gyroConfigStatic5,1);
    bank = 2;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(ICM42605_ACCEL_CONFIG_STATIC2,&accelConfigStatic2,1);
    writeReg(ICM42605_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(ICM42605_ACCEL_CONFIG_STATIC4,&accelConfigStatic4,1);
    bank = 0;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  }
}
void DFRobot_ICM42605::setAAF(uint8_t who,bool mode)
{
  uint8_t bank = 0;
  if(who == GYRO){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC2,&gyroConfigStatic2,1);
  }else if(who == ACCEL){
    if(mode){
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_ACCEL_CONFIG_STATIC2,&accelConfigStatic2,1);
  } else if(who == ALL){
    if(mode){
      gyroConfigStatic2.gyroAAFDis = 0;
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_GYRO_CONFIG_STATIC2,&gyroConfigStatic2,1);
    bank = 2;
    writeReg(ICM42605_REG_BANK_SEL,&bank,1);
    writeReg(ICM42605_ACCEL_CONFIG_STATIC2,&accelConfigStatic2,1);
  }
  bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
}

bool DFRobot_ICM42605::setUIFilter(uint8_t who,uint8_t filterOrder ,uint8_t UIFilterIndex)
{
  bool ret = true;
  uint8_t bank = 0;
  writeReg(ICM42605_REG_BANK_SEL,&bank,1);
  if(filterOrder > 3 || UIFilterIndex > 15){
    ret = false;
  } else{
    if(who == GYRO){
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(ICM42605_GYRO_CONFIG1,&gyroConfig1,1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      writeReg(ICM42605_GYRO_ACCEL_CONFIG0,&gyroAccelConfig0,1);
    } else if(who == ACCEL){
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(ICM42605_ACCEL_CONFIG1,&accelConfig1,1);
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(ICM42605_GYRO_ACCEL_CONFIG0,&gyroAccelConfig0,1);
    } else if(who == ALL){
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(ICM42605_GYRO_CONFIG1,&gyroConfig1,1);
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(ICM42605_ACCEL_CONFIG1,&accelConfig1,1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(ICM42605_GYRO_ACCEL_CONFIG0,&gyroAccelConfig0,1);
    }
  }
  return ret;
}

DFRobot_ICM42605_SPI::DFRobot_ICM42605_SPI(const char* name, mDev::mSpi* spix, mDev::mGpio* cs):mImu(name),mspi(spix),_cs(cs)
{
  if(!mspi)
  {
    printf("Error: spi4 not init yet\r\n");
  }
  init();
  begin();
}

int DFRobot_ICM42605_SPI::begin(void)
{
  if(mspi)
  {
  }
  return DFRobot_ICM42605::begin();
}

void DFRobot_ICM42605_SPI::writeReg(uint8_t reg, void* pBuf, size_t size)
{
  if(pBuf == NULL){
     DBG("pBuf ERROR!! : null pointer");
  }
  if(!mspi)
  {
    printf("Error: spi4 not init yet\r\n");
    return;
  }
  if(mspi->writeReg(reg, (uint8_t*)pBuf, size, _cs) != M_RESULT_EOK)
  {
    printf("ERROR: SPI WRITE REG ERROR\r\n");
  }
}

uint8_t DFRobot_ICM42605_SPI::readReg(uint8_t reg, void* pBuf, size_t size)
{
  uint8_t ret = 0;
  if(pBuf == NULL){
	  DBG("pBuf ERROR!! : null pointer");
  }
  if(!mspi)
  {
    printf("Error: spi4 not init yet\r\n");
    return 0;
  }
  ret = (uint8_t)mspi->readReg(reg, (uint8_t*)pBuf, size, _cs);
  if(ret != (uint8_t)M_RESULT_EOK)
  {
    printf("ERROR: SPI READ REG ERROR\r\n");
  }
  return ret;
}

void DFRobot_ICM42605_SPI::getAccelGyroData()
{
    setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
    setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);
    startTempMeasure();
    startGyroMeasure(/* mode= */LN_MODE);
    startAccelMeasure(/* mode= */LN_MODE);
}

void DFRobot_ICM42605_SPI::getDataByFIFO()
{
  /**
   * Set ODR and Full-scale range of gyroscope or accelerometer
   * who  GYRO/ACCEL/ALL
   *      GYRO: indicate only set gyroscope
   *      ACCEL: indicate only set accelerometer
   * ODR  Output data rate
   *      ODR_32KHZ         Support: Gyro/Accel(LN mode)
   *      ODR_16KHZ         Support: Gyro/Accel(LN mode)
   *      ODR_8KHZ          Support: Gyro/Accel(LN mode)
   *      ODR_4KHZ          Support: Gyro/Accel(LN mode)
   *      ODR_2KHZ          Support: Gyro/Accel(LN mode)
   *      ODR_1KHZ          Support: Gyro/Accel(LN mode)
   *      ODR_200HZ         Support: Gyro/Accel(LP or LN mode)
   *      ODR_100HZ         Support: Gyro/Accel(LP or LN mode)
   *      ODR_50HZ          Support: Gyro/Accel(LP or LN mode)
   *      ODR_25KHZ         Support: Gyro/Accel(LP or LN mode)
   *      ODR_12_5KHZ       Support: Gyro/Accel(LP or LN mode)
   *      ODR_6_25KHZ       Support: Accel(LP mode)
   *      ODR_3_125HZ       Support: Accel(LP mode)
   *      ODR_1_5625HZ      Support: Accel(LP mode)
   *      ODR_500HZ         Support: Accel(LP or LN mode)
   * FSR  Full-scale range
   *      FSR_0      Gyro:±2000dps   /   Accel: ±16g
   *      FSR_1      Gyro:±1000dps   /   Accel: ±8g
   *      FSR_2      Gyro:±500dps    /   Accel: ±4g
   *      FSR_3      Gyro:±250dps    /   Accel: ±2g
   *      FSR_4      Gyro:±125dps    /   Accel: not optional
   *      FSR_5      Gyro:±62.5dps   /   Accel: not optional
   *      FSR_6      Gyro:±31.25dps  /   Accel: not optional
   *      FSR_7      Gyro:±15.625dps /   Accel: not optional
   * Note：
   * In FIFO mode, set gyroscope and accelerometer ODR to be the same and the selected ODR should be no more than 8KHz. Otherwise, the temperature data integration rate will not match reading rate.
  */
  setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
  setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
  /**
   * Set gyroscope and accelerometer working mode
   * mode 
   *      OFF_MODE   0              Disable
   *      STANDBY_MODE_ONLY_GYRO 1  Set stanby mode, only support gyroscope
   *      LP_MODE_ONLY_ACCEL  2     Set low-power mode, only support accelerometer
   *      LN_MODE  3                Set low-noise mode
   */
  startTempMeasure();
  startGyroMeasure(/* mode= */LN_MODE);
  startAccelMeasure(/* mode= */LN_MODE);
  startFIFOMode();  //Enable FIFO
}

void DFRobot_ICM42605_SPI::interruptMode()
{
  /**
   * Set interrupt mode
   * INTPin  Interrupt pin : 1 represents using INT1 interrupt pin; 2 represents using INT2 interrupt pin
   * INTmode Set interrupt mode, 1 represents interrupt lock mode (polarity remain unchanged when interrupt triggerred, and restore after clearing interrupt); 0 represents pulse mode
   * INTPolarity Interrupt output level polarity, 0 represents interrupt pin polarity is LOW when producing interrupt, 1 represents interrupt pin polarity is HIGH when producing interrupt
   * INTDriveCircuit  0 represents Open drain  1 represents Push pull
   */
  setINTMode(/*INTPin=*/1, /*INTmode=*/0, /*INTPolarity=*/0, /*INTDriveCircuit=*/1);
  /**
   * @brief Wake on motion init
   */
  wakeOnMotionInit();

  /**
   * Set wake on motion interrupt threshold of axis accelerometer
   * axis
   *       X_AXIS_WOM
   *       Y_AXIS_WOM
   *       Z_AXIS_WOM
   *       ALL
   * threshold  Range(0-255) [WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg]
   */
  setWOMTh(/*axis=*/ALL,/*threshold=*/98);

  /**
   * Set essential motion detection mode and enable SMD interrupt
   * mode  0: disable SMD
   *       2 : SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart
   *       3 : SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
   */
  enableSMDInterrupt(/*mode=*/3);
}
mResult DFRobot_ICM42605_SPI::init()
{
    getDataByFIFO();
    return M_RESULT_EOK;
}
# if 0
int init42605()
{
  static DFRobot_ICM42605_SPI* icm42605 = new DFRobot_ICM42605_SPI();
  icm42605->getDataByFIFO();
  return 0;
}
INIT_EXPORT(init42605, "3");
#endif