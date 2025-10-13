#include "rtoscommon.hpp"
#include "mtimer.hpp"
#include "mthread.hpp"
#include "mmagnetmetordrv.hpp"
#include "mimudrv.hpp"
#include "mbarometordrv.hpp"
#include "qmc5883.hpp"
#include "datapublish.hpp"
#include "systeminfo.hpp"
#include "workqueue.hpp"
#include "workqueuemanager.hpp"
#include "mklog.hpp"
#include "project.hpp"
#include "musbdevicedrv.hpp"
#include "mgpiodrv.hpp"
#include "mahony.hpp"
#include "acccalibration.hpp"
#include "gyrocalibration.hpp"
#include "magcalibration.hpp"

using namespace bfimu;

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
uint8_t data_to_send[64] D2_MEM_ALIGN(4);

float mapAngleFloat(float angle) {
    if (angle > 180.0f) {
        angle -= 360.0f;
    }
    return angle;
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	volatile uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2 = alt;
	memset(data_to_send, 0, 64);
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(mapAngleFloat(angle_yaw)*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

    ((mDev::mUsbHidDevice*)mDev::mDeviceManager::getInstance()->getDevice(DEV_VCOM))->send(data_to_send,_cnt);
}

int sensorCalTask(void)
{
    mthread* sensorCal = mthread::create("sensorcal", 2048*2, 1, 20, [&](void* p){
        mDev::mGpio* gpiox = (mDev::mGpio*)mDev::mDeviceManager::getInstance()->getDevice("pb1");
        gpiox->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
        mDev::mGpio* gpiox1 = (mDev::mGpio*)mDev::mDeviceManager::getInstance()->getDevice("pb0");
        gpiox1->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
        mDev::mImu* imu1 = (mDev::mImu*)mDev::mDeviceManager::getInstance()->getDevice(DEV_IMU1);
        mDev::mMagnetmetor* mag1 = (mDev::mMagnetmetor*)mDev::mDeviceManager::getInstance()->getDevice(DEV_MAG1);
        mDev::mBarometor* mb1 = (mDev::mBarometor*)mDev::mDeviceManager::getInstance()->getDevice(DEV_BARO1);
        mDev::mSystick* systickx = (mDev::mSystick*)mDev::mDeviceManager::getInstance()->getDevice(DEV_SYSTICK);
        Mahony filter1;
        StandaloneAccCalibration accCalibration;
        StandaloneGyroCalibration gyroCalibration;
        StandaloneMagCalibration magCalibration;
        filter1.imuInit();
        accCalibration.startCalibration();
        gyroCalibration.startCalibration();
        magCalibration.startCalibration(systickx->systimeNowUs());

        workItem* senscal = new workItem("imucal", 0, 1, [&](void* param){
            gpiox1->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
            float pressure = 0.0;
            float ahrsData[7] = {0.0};

            float accelGyroBias1[6];   // 原来就有，保留
            float magBias[3];          // 原来就有，保留
            gpiox->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
            if(imu1)
            {
                imu1->updateData();
                // 使用新的 Vector3 类
                Vector3 accData(static_cast<float>(imu1->getAccelX()), 
                               static_cast<float>(imu1->getAccelY()), 
                               static_cast<float>(imu1->getAccelZ()));
                Vector3 gyroData(static_cast<float>(imu1->getGyroX()), 
                               static_cast<float>(imu1->getGyroY()), 
                               static_cast<float>(imu1->getGyroZ()));
                if(!gyroCalibration.isCalibrationComplete())
                {
                    gyroCalibration.performGyroCalibration(gyroData);
                }
                else
                {
                    // 使用新的applyCalibration方法进行补偿
                    Vector3 calibratedGyroData = gyroCalibration.applyCalibration(gyroData);
                    accelGyroBias1[0] = calibratedGyroData.x() * imu1->getGyroRangeScale();
                    accelGyroBias1[1] = calibratedGyroData.y() * imu1->getGyroRangeScale();
                    accelGyroBias1[2] = calibratedGyroData.z() * imu1->getGyroRangeScale();
                }

                if(!accCalibration.isCalibrationComplete())
                {
                    accCalibration.performAccelerationCalibration(accData, imu1->getAdcAcc1G());
                }
                else
                {
                    // 使用新的applyCalibration方法进行补偿
                    Vector3 calibratedAccData = accCalibration.applyCalibration(accData);
                    accelGyroBias1[3] = calibratedAccData.x() * imu1->getAccelRangeScale();
                    accelGyroBias1[4] = calibratedAccData.y() * imu1->getAccelRangeScale();
                    accelGyroBias1[5] = calibratedAccData.z() * imu1->getAccelRangeScale();
                    
                    //printf("x:%.4f, y:%.4f, z:%.4f\r\n", calResult.x(), calResult.y(), calResult.z());
                    //printf("GYR:%.4f,%.4f,%.4f,ACC:%.4f,%.4f,%.4f\r\n", imu1->getGyroXrad(), imu1->getGyroYrad(), imu1->getGyroZrad(), imu1->getAccelXms2(), imu1->getAccelYms2(), imu1->getAccelZms2());
                    //printf("GYR:%d,%.d,%d,ACC:%d,%d,%d\r\n",imu1->getGyroX(),imu1->getGyroY(),imu1->getGyroZ(),imu1->getAccelX(),imu1->getAccelY(),imu1->getAccelZ());
                }
                if(accCalibration.isCalibrationComplete() && gyroCalibration.isCalibrationComplete())
                {
                    imu1Hub->publish(accelGyroBias1);
                }
            }

            if(mag1)
            {
                mag1->updateData();
                Vector3 magData(static_cast<float>(mag1->getMageX()), 
                               static_cast<float>(mag1->getMageY()), 
                               static_cast<float>(mag1->getMageZ()));
                Vector3 gyroData(static_cast<float>(imu1->getGyroX()), 
                                static_cast<float>(imu1->getGyroY()), 
                                static_cast<float>(imu1->getGyroZ()));
                if(!magCalibration.isCalibrationComplete())
                {
                    magCalibration.performMagnetometerCalibration(magData, gyroData, systickx->systimeNowUs());
                }
                else
                {
                    // 应用磁罗盘校准偏差补偿
                    Vector3 calibratedMagData = magCalibration.applyCalibration(magData);
                    
                    // 更新magBias数组用于发布和后续处理
                    magBias[0] = calibratedMagData.x();
                    magBias[1] = calibratedMagData.y();
                    magBias[2] = calibratedMagData.z();
                    mag1Hub->publish(magBias);
                }
            }
            if(mb1)
            {
                mb1->updateData();
                pressure = mb1->getPressure();
                mb1Hub->publish(&pressure);
            }
            gpiox->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
            if(imu1 && mag1 && accCalibration.isCalibrationComplete() && gyroCalibration.isCalibrationComplete() && magCalibration.isCalibrationComplete())
            {
                filter1.update(systickx->systimeNowUs(), 
                              accelGyroBias1[0], accelGyroBias1[1], accelGyroBias1[2], 
                              accelGyroBias1[3], accelGyroBias1[4], accelGyroBias1[5], 
                              magBias[0], magBias[1], magBias[2]);
                ahrsData[0] = filter1.getYaw() / 10.0f;
                ahrsData[1] = filter1.getRoll() / 10.0f;
                ahrsData[2] = filter1.getPitch() / 10.0f;
                ahrsData[6] = pressure;
                ahrsHub->publish(ahrsData);
                gpiox1->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
            }
        }, nullptr);
        workQueueManager::getInstance()->find(WORKQUEUE_HP_WORK)->scheduleWork(senscal);

        while(true)
        {
            float ahrsData[7] = {0.0};
            if(ahrsHub->poll(ahrsNode))
            {
                ahrsHub->copy(ahrsNode, ahrsData);
                ALOGI("YAW:%10f ROLL:%10f PITCH:%10f \r\n", ahrsData[0], ahrsData[1], ahrsData[2]);
                ANO_DT_Send_Status(ahrsData[1], ahrsData[2], ahrsData[0], static_cast<int32_t>(ahrsData[6]), 0, 0);
            }
            if(mag1Hub->poll(mag1Node))
            {
                float mag1Data[3] = {0.0};
                mag1Hub->copy(mag1Node, mag1Data);
                //ALOGI("MAG1:%10f %10f %10f\r\n", mag1Data[0], mag1Data[1], mag1Data[2]);
            }
            mthread::threadMdelay(10);
        }
    }, nullptr);
    sensorCal->startup();
    return 0;
}

TASK_EXPORT(sensorCalTask, "0.1");