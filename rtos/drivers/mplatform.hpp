#pragma once
#include "containers.hpp"
#include "mipc.hpp"
#include <list>
namespace mDev
{
class mDevice;
struct devBase
{
    const char* _devname = nullptr;
    mDev::mDevice* _mdev = nullptr;
    devBase(const char* name, mDev::mDevice* dev):_devname(name), _mdev(dev)
    {

    }
};

using devList = std::list<devBase*, mMemAllocator<devBase*>>;

class mPlatform
{
public:
    static mPlatform* getInstance()
    {
        static mPlatform platform;
        return &platform;
    }
    mResult registerDevice(const char* name, mDev::mDevice* dev)
    {
        if(dev == nullptr)
        {
            return M_RESULT_ERROR;
        }
        _mutex.mutexTake(WAITING_FOREVER);
        for(auto& it : _devList)
        {
            if(strcmp(it->_devname, name) == 0)
            {
                _mutex.mutexRelease();
                return M_RESULT_EXIST;
            }
        }

        devBase* pdev = new devBase(name, dev);
        if(!pdev)
        {
            _mutex.mutexRelease();
            return M_RESULT_ENOMEM;
        }
        _devList.emplace_back(pdev);
        _mutex.mutexRelease();
        return M_RESULT_EOK;
    }
    void unregisterDevice(const char* name)
    {
        if(name == nullptr)
        {
            return;
        }
        _mutex.mutexTake(WAITING_FOREVER);
        for(auto it = _devList.begin(); it != _devList.end(); ++it)
        {
            if(strcmp((*it)->_devname, name) == 0)
            {
                _devList.erase(it);
                delete (*it);
                break;
            }
        }
        _mutex.mutexRelease();
    }
    mDev::mDevice* getDevice(const char* name)
    {
        _mutex.mutexTake(WAITING_FOREVER);
        for(auto it = _devList.begin(); it != _devList.end(); ++it)
        {
            if(strcmp((*it)->_devname, name) == 0)
            {
                _mutex.mutexRelease();
                return (*it)->_mdev;
            }
        }
        _mutex.mutexRelease();
        printf("Error:\r\nthe device %s is nullptr\r\n",name);
        return nullptr;
    }
private:
    mPlatform():_mutex("pmutex", IPC_FLAG_FIFO)
    {

    }
    ~mPlatform()
    {
        _mutex.detach();
    }
    mPlatform(const mPlatform&) = delete;
    mPlatform(mPlatform&&) = delete;
    mPlatform& operator=(const mPlatform&) = delete;
    mPlatform& operator=(mPlatform&&) = delete;
private:
    devList _devList;
    mMutex _mutex;
};
}