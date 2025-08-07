#pragma once
#include "mipc.hpp"
#include <string>
namespace mDev
{
class IMutex {
public:
    virtual ~IMutex() = default;
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

class Mutex : public IMutex
{
public:
    Mutex(){
        static int _mutexid;
        std::string idname("mutex_");
        _mutex.init((idname+std::to_string(++_mutexid)).c_str(), mIpcFlag::IPC_FLAG_FIFO);
    }

    ~Mutex(){_mutex.detach();}
    void lock() override{
        _mutex.mutexTake(WAITING_FOREVER);
    }
    void unlock() override {
        _mutex.mutexRelease();
    }
private:
    
    mMutex _mutex;
};
}