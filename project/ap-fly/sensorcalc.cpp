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

int sensorCalTask(void)
{

    mthread* sensorCal = mthread::create("sensorcal", 2048, 1, 20, [&](void* p){
        MadgMahony imu1AHRS;
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
            float ahrsData[4] = {0.0};
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
            imu1AHRS.update((accelGyroBias1[0]+accelGyroBias2[0])/2.0f,
            (accelGyroBias1[1]+accelGyroBias2[1])/2.0f,
            (accelGyroBias1[2]+accelGyroBias2[2])/2.0f,
            (accelGyroBias1[3]+accelGyroBias2[3])/2.0f,
            (accelGyroBias1[4]+accelGyroBias2[4])/2.0f,
            (accelGyroBias1[5]+accelGyroBias2[5])/2.0f,magBias[0],magBias[1],magBias[2]);
            ahrsData[0] = imu1AHRS.getYaw();
            ahrsData[1] = imu1AHRS.getRoll();
            ahrsData[2] = imu1AHRS.getPitch();
            ahrsData[3] = pressure;
            ahrsHub->publish(&ahrsData);
        }, nullptr);
        workQueueManager::getInstance()->find(WORKQUEUE_HP_WORK)->scheduleWork(senscal);

        while(true)
        {
            float ahrsData[4] = {0.0};
            if(ahrsHub->poll(ahrsNode))
            {
                ahrsHub->copy(ahrsNode, ahrsData);
                //ALOGI("YAW:%10f ROLL:%10f PITCH:%10f P%10f\r\n",ahrsData[0], ahrsData[1], ahrsData[2], ahrsData[3]);
            }
            mthread::threadMdelay(10);
        }
    },nullptr);
    sensorCal->startup();
    return 0;
}
TASK_EXPORT(sensorCalTask, "0.1");