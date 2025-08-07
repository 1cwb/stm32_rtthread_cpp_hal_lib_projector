#pragma once
#include "containers.hpp"
#include <memory>
#include <list>
#include <string>
#include <mutex>
#include <unordered_map>
#include "minternal.hpp"

namespace mDev
{
class mDevice;

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
        if(dev == nullptr || name.empty())
        {
            return M_RESULT_ERROR;
        }
        std::lock_guard<IMutex> lock(*_mutex);
        auto it =_mapDev.find(name);
        if(it != _mapDev.end())
        {
            return M_RESULT_EXIST;
        }
        _mapDev[name] = dev;
        return M_RESULT_EOK;
    }

    void unregisterDevice(const std::string& name)
    {
        if(name.empty())
        {
            return;
        }
        std::lock_guard<IMutex> lock(*_mutex);
        auto it =_mapDev.find(name);
        if(it != _mapDev.end())
        {
            _mapDev.erase(it);
        }
    }
    mDev::mDevice* getDevice(const std::string& name)
    {
        std::lock_guard<IMutex> lock(*_mutex);
        auto it = _mapDev.find(name);
        if(it != _mapDev.end())
        {
            return (*it).second;
        }
        printf("Warning:\r\nthe device %s is nullptr\r\n",name.c_str());
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
    std::unordered_map<std::string, mDevice*> _mapDev;
    std::unique_ptr<IMutex> _mutex; // 多态互斥锁
};
}