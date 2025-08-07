#pragma once
#include "containers.hpp"
#include <memory>
#include <list>
#include <string>
#include <mutex>
#include "minternal.hpp"

namespace mDev
{
class mDevice;
struct devBase
{
    const std::string _devname = nullptr;
    mDev::mDevice* _mdev = nullptr;
    devBase(const std::string& name, mDev::mDevice* dev):_devname(name), _mdev(dev)
    {

    }
};

class mDeviceManager
{
public:
    static mDeviceManager* getInstance()
    {
        static mDeviceManager platform;
        return &platform;
    }
    mResult registerDevice(const std::string& name, mDev::mDevice* dev)
    {
        if(dev == nullptr)
        {
            return M_RESULT_ERROR;
        }
        std::lock_guard<IMutex> lock(*_mutex);
        for(auto& it : _devList)
        {
            if(it->_devname.compare(name) == 0)
            {
                return M_RESULT_EXIST;
            }
        }

        devBase* pdev = new devBase(name, dev);
        if(!pdev)
        {
            return M_RESULT_ENOMEM;
        }
        _devList.emplace_back(pdev);
        return M_RESULT_EOK;
    }
    void unregisterDevice(const char* name)
    {
        if(name == nullptr)
        {
            return;
        }
        std::lock_guard<IMutex> lock(*_mutex);
        for(auto it = _devList.begin(); it != _devList.end(); ++it)
        {
            if((*it)->_devname.compare(name) == 0)
            {
                _devList.erase(it);
                delete (*it);
                break;
            }
        }
    }
    mDev::mDevice* getDevice(const char* name)
    {
        std::lock_guard<IMutex> lock(*_mutex);
        for(auto it = _devList.begin(); it != _devList.end(); ++it)
        {
            if((*it)->_devname.compare(name) == 0)
            {
                return (*it)->_mdev;
            }
        }
        printf("Warning:\r\nthe device %s is nullptr\r\n",name);
        return nullptr;
    }
private:
    mDeviceManager(std::unique_ptr<IMutex> mutex = std::make_unique<Mutex>()):_mutex(std::move(mutex))
    {

    }
    ~mDeviceManager()
    {

    }
    mDeviceManager(const mDeviceManager&) = delete;
    mDeviceManager(mDeviceManager&&) = delete;
    mDeviceManager& operator=(const mDeviceManager&) = delete;
    mDeviceManager& operator=(mDeviceManager&&) = delete;
private:
    std::list<devBase*> _devList;
    std::unique_ptr<IMutex> _mutex; // 多态互斥锁
};
}