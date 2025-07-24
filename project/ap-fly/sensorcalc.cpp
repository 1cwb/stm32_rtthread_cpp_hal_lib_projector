#include "rtoscommon.hpp"
#include "mtimer.hpp"
#include "mthread.hpp"
#include "mmagnetmetordrv.hpp"
#include "mimudrv.hpp"
#include "mbarometordrv.hpp"
#include "MadgwickAHRS.hpp"
#include "qmc5883.hpp"
#include "datapublish.hpp"
#include "systeminfo.hpp"
#include "workqueue.hpp"
#include "workqueuemanager.hpp"
#include "mklog.hpp"
#include "project.hpp"
#include "MadgwickAHRS.hpp"
#include "musbdevicedrv.hpp"
#include "bfmahony.hpp"

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

    ((mDev::mUsbHidDevice*)mDev::mPlatform::getInstance()->getDevice(DEV_VCOM))->send(data_to_send,_cnt);
}

int sensorCalTask(void)
{

    mthread* sensorCal = mthread::create("sensorcal", 2048, 1, 20, [&](void* p){
        //MadgMahony imu1AHRS;
        MadgMahony imu2AHRS;
        bfim::BetaflightImu bfimu1;
        //mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice(DEV_TIMER1);
        mDev::mImu* imu1 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice(DEV_IMU1);
        mDev::mImu* imu2 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice(DEV_IMU2);
        mDev::mMagnetmetor* mag1 = (mDev::mMagnetmetor*)mDev::mPlatform::getInstance()->getDevice(DEV_MAG1);
        mDev::mBarometor* mb1 = (mDev::mBarometor*)mDev::mPlatform::getInstance()->getDevice(DEV_BARO1);
        //mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED0);
        //mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED1);
        //mDev::mLed* led2 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED2);
        //mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice(DEV_TIMER2);
        //mDev::mSystick* systickx = (mDev::mSystick*)mDev::mPlatform::getInstance()->getDevice(DEV_SYSTICK);
        workItem* senscal = new workItem("imucal", 0, 5, [&](void* param){
            float accelGyroBias1[6] = {0};
            float accelGyroBias2[6] = {0};
            float magBias[3] = {0.0};
            float pressure = 0.0;
            float ahrsData[7] = {0.0};
            if(imu1)
            {
                imu1->updateData();
                accelGyroBias1[0] = imu1->getGyroXrad();
                accelGyroBias1[1] = imu1->getGyroYrad();
                accelGyroBias1[2] = imu1->getGyroZrad();
                accelGyroBias1[3] = imu1->getAccelXms2();
                accelGyroBias1[4] = imu1->getAccelYms2();
                accelGyroBias1[5] = imu1->getAccelZms2();
                imu1Hub->publish(accelGyroBias1);
            }
            if(imu2)
            {
                imu2->updateData();
                accelGyroBias2[0] = imu2->getGyroXrad();
                accelGyroBias2[1] = imu2->getGyroYrad();
                accelGyroBias2[2] = imu2->getGyroZrad();
                accelGyroBias2[3] = imu2->getAccelXms2();
                accelGyroBias2[4] = imu2->getAccelYms2();
                accelGyroBias2[5] = imu2->getAccelZms2();
                imu2Hub->publish(accelGyroBias2);
            }
            if(mag1)
            {
                mag1->updateData();
                magBias[0] = mag1->getMageX();
                magBias[1] = mag1->getMageY();
                magBias[2] = mag1->getMageZ();
                mag1Hub->publish(magBias);
            }
            if(mb1)
            {
                mb1->updateData();
                pressure = mb1->getPressure();
                mb1Hub->publish(&pressure);
            }
            if(imu1 && mag1)
            {
                #if 0
                imu1AHRS.update(accelGyroBias1[0],
                                accelGyroBias1[1],
                                accelGyroBias1[2],
                                accelGyroBias1[3],
                                accelGyroBias1[4],
                                accelGyroBias1[5],magBias[0],magBias[1],magBias[2]);
                #endif
                bfimu1.update(0.005f,accelGyroBias1[0],
                                accelGyroBias1[1],
                                accelGyroBias1[2],
                                true,
                                accelGyroBias1[3],
                                accelGyroBias1[4],
                                accelGyroBias1[5],
                                true,
                                magBias[0],magBias[1],magBias[2]);
            }
            else if(imu1)
            {
                #if 0
                imu1AHRS.updateIMU( accelGyroBias1[0],
                                    accelGyroBias1[1],
                                    accelGyroBias1[2],
                                    accelGyroBias1[3],
                                    accelGyroBias1[4],
                                    accelGyroBias1[5]);
                #endif
            }
            if(imu2 && mag1)
            {
                imu2AHRS.update(accelGyroBias2[0],
                accelGyroBias2[1],
                accelGyroBias2[2],
                accelGyroBias2[3],
                accelGyroBias2[4],
                accelGyroBias2[5],magBias[0],magBias[1],magBias[2]);
            }
            else if(imu2)
            {
                imu2AHRS.updateIMU(accelGyroBias2[0],
                                    accelGyroBias2[1],
                                    accelGyroBias2[2],
                                    accelGyroBias2[3],
                                    accelGyroBias2[4],
                                    accelGyroBias2[5]);
            }
            /*
            ahrsData[0] = imu1AHRS.getYaw();
            ahrsData[1] = imu1AHRS.getRoll();
            ahrsData[2] = imu1AHRS.getPitch();
            */
            auto [roll, pitch, yaw] = bfimu1.getEulerDeg();
            ahrsData[0] = yaw;
            ahrsData[1] = roll;
            ahrsData[2] = pitch;
            ahrsData[3] = imu2AHRS.getYaw();
            ahrsData[4] = imu2AHRS.getRoll();
            ahrsData[5] = imu2AHRS.getPitch();
            ahrsData[6] = pressure;
            ahrsHub->publish(&ahrsData);
        }, nullptr);
        workQueueManager::getInstance()->find(WORKQUEUE_HP_WORK)->scheduleWork(senscal);

        while(true)
        {
            float ahrsData[7] = {0.0};
            if(ahrsHub->poll(ahrsNode))
            {
                ahrsHub->copy(ahrsNode, ahrsData);
                //ALOGI("YAW:%10f ROLL:%10f PITCH:%10f \r\n",ahrsData[0]*0.1f, ahrsData[1]*0.1f, ahrsData[2]*0.1f);
                ANO_DT_Send_Status(ahrsData[1]*0.1f, ahrsData[2]*0.1f, ahrsData[0]*0.1f, ahrsData[6], 0, 0);
            }
            if(mag1Hub->poll(mag1Node))
            {
                float mag1Data[3] = {0.0};
                mag1Hub->copy(mag1Node, mag1Data);
                //ALOGI("MAG1:%10f %10f %10f\r\n", mag1Data[0], mag1Data[1], mag1Data[2]);
            }
            mthread::threadMdelay(10);
        }
    },nullptr);
    sensorCal->startup();
    return 0;
}
TASK_EXPORT(sensorCalTask, "0.1");