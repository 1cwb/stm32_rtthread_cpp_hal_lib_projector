#include "systeminfo.hpp"
mResult systemInfo::init()
{
    idle = mIdle::getInstance();
    if(!idle)
    {
        printf("Error: idleThread not found\r\n");
        return M_RESULT_ENOSYS;
    }
    systime = (mDev::mSystick*)mDev::mPlatform::getInstance()->getDevice("systick");
    if(!systime)
    {
        printf("Error: systime driver not found\r\n");
        return M_RESULT_EINVAL;
    }
    idle->regitserHookCallback([this](){
        uint64_t tick = 0;
        uint64_t count = 0;
        volatile uint32_t loop = 0;

    if (totalCount == 0)
    {                             
        /* get total count */
        mSchedule::getInstance()->enterCritical();                             
        tick = mClock::getInstance()->tickGet();                               
        while (mClock::getInstance()->tickGet() - tick < CPU_USAGE_CALC_TICK)
        {
                totalCount ++;                         
                loop = 0;
                while (loop < CPU_USAGE_LOOP) loop ++;
        }
        mSchedule::getInstance()->exitCritical();  
    }
    count = 0;
    /* get CPU usage */
    tick = mClock::getInstance()->tickGet();                           
    while (mClock::getInstance()->tickGet() - tick < CPU_USAGE_CALC_TICK)
    {
            count ++;                                   
            loop  = 0;
            while (loop < CPU_USAGE_LOOP) loop ++;
    }

    /* calculate major and minor */
    if (count < totalCount) 
    {                          
        count = totalCount - count;
        cpuUsageMajor = (count * 100) / totalCount;
        cpuUsageMinor = ((count * 100) % totalCount) * 100 / totalCount;
    }
    else
    {
            totalCount = count;                                

            /* no CPU usage */
            cpuUsageMajor = 0;
            cpuUsageMinor = 0;
        }
    });
    
    return M_RESULT_EOK;
}
void systemInfo::showAllThreadStackSizeInfo()
{
    mList_t* node{nullptr};
    mObject_t* object{nullptr};
    mthread* pthread{nullptr};
    struct mObjectInformation_t *information = nullptr;

    information = mObject::getInstance()->objectGetInformation(M_OBJECT_CLASS_THREAD);
    if (information == nullptr) return ;

    /* retrieve pointer of object */
    for(node = information->objectList.next; node != &(information->objectList); node = node->next)
    {
        object = listEntry(node, mObject_t, list);
        if(object)
        {
            pthread = reinterpret_cast<mthread*>(object);
            printf("thread %s totalStackSize = %ld, usedStackSize = %ld, freeStackSize = %ld\r\n",   pthread->getThTimer_t()->name,
                                                                                pthread->getTotalStackSize(),
                                                                                pthread->getTotalStackSize() - pthread->getFreeStackSize(),
                                                                                pthread->getFreeStackSize());
        }
    }
}