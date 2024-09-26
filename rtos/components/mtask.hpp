#pragma once
#include "mthread.hpp"
//#include <functional>
#include <list>
#include <cstring>
#include "containers.hpp"

using TASK_INIT_FUNC = mResult(*)(void);
using TASK_ENTRY_FUNC = void(*)(void*);

enum mTaskStatus
{
    TASK_STATUS_IDLE = 0,
    TASK_STATUS_FAIL,
    TASK_STATUS_READY,
    TASK_STATUS_RUNNING,
    TASK_STATUS_SUSPEND,
    TASK_STATUS_ZOMBIE,
    TASK_STATUS_UNKNOW
};

struct task_t
{
    const char* _name;
    TASK_INIT_FUNC _init;
    TASK_ENTRY_FUNC _entry;
    uint8_t _priority;
    bool _autoStart;
    uint32_t _stackSize;
    void* _param;
    char** _dependency;
};

class mTask
{
public:
    mTask():_status(TASK_STATUS_IDLE)
    {
    }
    ~mTask()
    {

    }
    mTask(const mTask&) = delete;
    mTask(mTask&&) = delete;
    mTask& operator=(const mTask&) = delete;
    mTask& operator=(mTask&&) = delete;
    void addTask(task_t* task) {_task = task;}
    mResult init() {if(_task->_init){return _task->_init();} return M_RESULT_EINVAL;}
    void setStatus(const mTaskStatus status) {_status = status;}

    const char* getName() const {return _task->_name;}
    char** getDependecy() {return _task->_dependency;}
    mTaskStatus getStatus() const {return _status;}
    
    TASK_ENTRY_FUNC& getEnteryFunc() {return _task->_entry;}
    void* getParam() {return _task->_param;}
    uint8_t getPriority() const {return _task->_priority;}
    bool bAutoStart() const {return _task->_autoStart;}
    uint32_t getStackSize()const {return _task->_stackSize;}

    bool bNameInit() const {return _task->_name != nullptr;}
    bool bInitFuncSet() const {return _task->_init ? true : false;}
    bool bEntryFuncSet() const {return _task->_entry ? true : false;}

    void setTid(mthread* tid) {_tid = tid;}
    mthread* getTid() {return _tid;}
private:
    task_t* _task;
    mTaskStatus _status;
    mthread* _tid = nullptr;
};

#define TASK_EXPORT M_USED static const task_t SECTION("TaskTab")

class mTaskManager
{
public:
    static mTaskManager* getInstance()
    {
        static mTaskManager taskManager;
        return &taskManager;
    }
    void init();
    void start();
    mTaskStatus getTaskStatus(const char* name) const;
    mthread* getTaskTid(const char* name);
    uint32_t getTaskNum() const;
    mResult startTask(const char* name);
    mResult suspend(const char* name, uint32_t timeout);
    mResult resume(const char* name);
    mResult kill(const char* name);
private:
    mTaskManager(){};
    ~mTaskManager()=default;
    mTaskManager(const mTaskManager&) = delete;
    mTaskManager(mTaskManager&&) = delete;
    mTaskManager& operator=(const mTaskManager&) = delete;
    mTaskManager& operator=(mTaskManager&&) = delete;

private:
    //std::list<mTask*, mMemAllocator<mTask*>> _taskList;
    mTask* _taskTable = nullptr;
    uint32_t _taskNum = 0;
    mthread** _tid = nullptr;
};