#pragma once
#include "sys.h"
#include "mi2cdrv.hpp"
#include "mmagnetmetordrv.hpp"

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

class QMC5883LCompass : public mDev::mMagnetmetor
{
public:
	enum MODE
	{
		MODE_STANDBY		=	0x00,
		MODE_CONTINUES	    =	0x01
	};
	enum OUTPUT_DATA_RATE_ODR
	{
		ODR_10Hz     =   	0x00,
		ODR_50Hz     =   	0x04,
		ODR_100Hz    =   	0x08,
		ODR_200Hz    =   	0x0C
	};
	enum SCALE_RANGE
	{
		SCALE_RANGE_2G        = 	0x00,
		SCALE_RANGE_8G        = 	0x10
	};
	enum OVER_SAMPLE_RATIO_OSR
	{
		OSR_512      =   	0x00,
		OSR_256      =   	0x40,
		OSR_128      =   	0x80,
		OSR_64       =   	0xC0, 
	};
public:
    QMC5883LCompass(const char*name, mDev::mI2c* i2cx);
	void init();
    void setADDR(uint8_t b);
    void setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);
	void setMagneticDeclination(int degrees, uint8_t minutes);
	void setSmoothing(uint8_t steps, bool adv);
	void calibrate();
	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
	void setCalibrationOffsets(float x_offset, float y_offset, float z_offset);
	void setCalibrationScales(float x_scale, float y_scale, float z_scale);
    float getCalibrationOffset(uint8_t index);
	float getCalibrationScale(uint8_t index);
	void clearCalibration();
	void setReset();
    void read();
	int getX();
	int getY();
	int getZ();
	int getAzimuthData();
	uint8_t getBearing(int azimuth);
	void getDirection(char* myArray, int azimuth);
	uint8_t whoAmi()
	{
		uint8_t data;
		_i2cx->readReg(_ADDR, QMC5883L_STATUS, &data, 1);
		return data;
	}
	void calibrateAndSmooth();
    virtual float getTemp()override{return 0.0f;};
    virtual int getMageX()override{return getX();};
    virtual int getMageY()override{return getY();};
    virtual int getMageZ()override{return getZ();};
	virtual bool prepareData()override{read(); return true;}
    virtual bool updateData()override{calibrateAndSmooth(); return true;}
    virtual int getAzimuth()override {return  QMC5883LCompass::getAzimuthData();}
	virtual void getOrignalData(uint8_t* ordata, uint32_t len)override {memcpy(ordata, _orignalData, len > sizeof(_orignalData) ? sizeof(_orignalData) : len);}
  private:
    void _writeReg(uint8_t reg,uint8_t val);
	int _get(int index);
	float _magneticDeclinationDegrees = 0;
	bool _smoothUse = false;
	uint8_t _smoothSteps = 5;
	bool _smoothAdvanced = false;
    uint8_t _ADDR = 0x0D;
	int _vRaw[3] = {0,0,0};
	int _vHistory[10][3];
	int _vScan = 0;
	long _vTotals[3] = {0,0,0};
	int _vSmooth[3] = {0,0,0};
	void _smoothing();
	float _offset[3] = {0.,0.,0.};
	float _scale[3] = {1.,1.,1.};
	int _vCalibrated[3];
	uint8_t _orignalData[6] = {0};
	void _applyCalibration();
	const char _bearings[16][3] =  {
		{' ', ' ', 'N'},
		{'N', 'N', 'E'},
		{' ', 'N', 'E'},
		{'E', 'N', 'E'},
		{' ', ' ', 'E'},
		{'E', 'S', 'E'},
		{' ', 'S', 'E'},
		{'S', 'S', 'E'},
		{' ', ' ', 'S'},
		{'S', 'S', 'W'},
		{' ', 'S', 'W'},
		{'W', 'S', 'W'},
		{' ', ' ', 'W'},
		{'W', 'N', 'W'},
		{' ', 'N', 'W'},
		{'N', 'N', 'W'},
	};
	mDev::mI2c* _i2cx = nullptr;
};