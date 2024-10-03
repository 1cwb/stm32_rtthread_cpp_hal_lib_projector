#include "mtask.hpp"
#include "mclock.hpp"
#define MAX_INIT_TIME (5000)
#define TASK_TICKET (5)
void mTaskManager::init()
{
    extern int _task_start;
    extern int _task_end;
    uint32_t initDone = 0;
    mClock* clock =  mClock::getInstance();
    uint32_t timeStartInit =clock->tickGet();
    task_t* taskList = nullptr;

    taskList = reinterpret_cast<task_t*>(&_task_start);
    _taskNum = reinterpret_cast<task_t*>(&_task_end) - taskList;
    if(_taskNum == 0)
    {
        printf("there no task find\r\n");
        return;
    }
    _tid = new mthread*[_taskNum];
    _taskTable = new mTask[_taskNum];
    MASSERT(_tid);

    for (size_t i = 0; i < _taskNum; i++)
    {
        _taskTable[i].addTask(&taskList[i]);
        MASSERT(_taskTable[i].bNameInit());
        MASSERT(_taskTable[i].bInitFuncSet());
        MASSERT(_taskTable[i].bEntryFuncSet());
    }
    while (initDone < _taskNum && (clock->tickGet() - timeStartInit) < MAX_INIT_TIME)
    {
        for(uint32_t i = 0; i < _taskNum; i++)
        {
            bool dependMet = true;
            for(uint32_t n = 0; _taskTable[i].getDependecy() != nullptr && _taskTable[i].getDependecy()[n] != nullptr; n++)
            {
                for(uint32_t k = 0; k < _taskNum; k++)
                {
                    if(strcmp(_taskTable[i].getDependecy()[n], _taskTable[k].getName()) == 0)
                    {
                        if(_taskTable[k].getStatus() == TASK_STATUS_IDLE)
                        {
                            dependMet = false;
                            break;
                        }
                        else if(_taskTable[k].getStatus() == TASK_STATUS_FAIL)
                        {
                            dependMet = false;
                            _taskTable[i].setStatus(TASK_STATUS_FAIL);
                            initDone++;
                        }
                    }
                }
            }

            if(dependMet && _taskTable[i].getStatus() == TASK_STATUS_IDLE)
            {
                if(_taskTable[i].init() == M_RESULT_EOK)
                {
                    _tid[i] = mthread::create(_taskTable[i].getName(),
                            _taskTable[i].getStackSize(),
                            _taskTable[i].getPriority(),
                            TASK_TICKET,
                            _taskTable[i].getEnteryFunc(),
                            (void*)_taskTable[i].getParam()
                            );
                    if(_tid[i] != nullptr)
                    {
                        _taskTable[i].setStatus(TASK_STATUS_READY);
                        _taskTable[i].setTid(_tid[i]);
                    }
                }
                else
                {
                    _taskTable[i].setStatus(TASK_STATUS_FAIL);
                }
                initDone++;
            }
        }
    }
}
void mTaskManager::start()
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(_taskTable[i].getStatus() == TASK_STATUS_READY && _taskTable[i].bAutoStart())
        {
            if(_taskTable[i].getTid())
            {
                if(_taskTable[i].getTid()->startup()!=M_RESULT_EOK)
                {
                    return;
                }
                _taskTable[i].setStatus(TASK_STATUS_RUNNING);
            }
        }
    }
}
mResult mTaskManager::startTask(const char* name)
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            if(_taskTable[i].getStatus() != TASK_STATUS_READY)
            {
                return M_RESULT_ENOSYS;
            }
            if(_taskTable[i].getTid())
            {
                if(_taskTable[i].getTid()->startup()!=M_RESULT_EOK)
                {
                    return M_RESULT_ERROR;
                }
                _taskTable[i].setStatus(TASK_STATUS_RUNNING);
            }
            break;
        }
    }
    return M_RESULT_ERROR;
}
mTaskStatus mTaskManager::getTaskStatus(const char* name) const
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            return _taskTable[i].getStatus();
        }
    }
    return TASK_STATUS_UNKNOW;
}
mthread* mTaskManager::getTaskTid(const char* name)
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            return _taskTable[i].getTid();
        }
    }
    return nullptr;
}
uint32_t mTaskManager::getTaskNum() const
{
    return _taskNum;
}
mResult mTaskManager::suspend(const char* name, uint32_t timeout)
{
    mClock* clock =  mClock::getInstance();
    uint32_t startTime = clock->tickGet();

    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            if(_taskTable[i].getStatus() != TASK_STATUS_RUNNING)
            {
                return M_RESULT_ENOSYS;
            }
            if(_taskTable[i].getTid())
            {
                while(clock->tickGet() - startTime < timeout)
                {
                    if(_taskTable[i].getTid()->threadSuspend(reinterpret_cast<thread_t*>(_taskTable[i].getTid())) == M_RESULT_EOK)
                    {
                        _taskTable[i].setStatus(TASK_STATUS_SUSPEND);
                        return M_RESULT_EOK;
                    }
                }
            }
            break;
        }
    }
    return M_RESULT_ERROR;
}
mResult mTaskManager::resume(const char* name)
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            if(_taskTable[i].getStatus() != TASK_STATUS_SUSPEND)
            {
                return M_RESULT_ENOSYS;
            }
            if(_taskTable[i].getTid())
            {
                if(_taskTable[i].getTid()->threadResume() == M_RESULT_EOK)
                {
                    _taskTable[i].setStatus(TASK_STATUS_RUNNING);
                    return M_RESULT_EOK;
                }
            }
            break;
        }
    }
    return M_RESULT_ERROR;
}
mResult mTaskManager::kill(const char* name)
{
    for(uint32_t i = 0; i < _taskNum; i++)
    {
        if(strcmp(name, _taskTable[i].getName()) == 0)
        {
            if(_taskTable[i].getTid())
            {
                if(_taskTable[i].getStatus() == TASK_STATUS_RUNNING)
                {
                    if(_taskTable[i].getTid()->threadDelete() == M_RESULT_EOK)
                    {
                        _taskTable[i].setStatus(TASK_STATUS_ZOMBIE);
                    }
                }
                else if(_taskTable[i].getStatus() == TASK_STATUS_READY)
                {
                    _taskTable[i].setStatus(TASK_STATUS_ZOMBIE);
                }
                else
                {
                    return M_RESULT_ENOSYS;
                }
                return M_RESULT_EOK;
            }
            break;
        }
    }
    return M_RESULT_ERROR;
}

#if 0
mResult testinitf(void)
{
printf("testxxxxx init  now \r\n"); return M_RESULT_EOK;
}
void testEnter(void* p)
{
    while(1)
    {
        printf("tony run entry+++++++++++\r\n");
    }
}
TASK_EXPORT testx = {
    ._name = "TEST1",
    ._init = testinitf,
    ._entry = testEnter,
    ._priority = 3,
    ._autoStart = true,
    ._stackSize = 1024,
    ._param = nullptr,
    ._dependency = nullptr
};
#endif