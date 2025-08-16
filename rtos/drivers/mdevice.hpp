#pragma once
#include "mdevicemanager.hpp"
#include <functional>
#include <string>
namespace mDev
{
enum class transferMode
{
    TRANSFER_MODE_NOMAL,
    TRANSFER_MODE_IT,
    TRANSFER_MODE_DMA,
};
enum class recvMode
{
    RECV_MODE_NOMAL,
    RECV_MODE_IT,
    RECV_MODE_DMA,
    RECV_MODE_IT_RECV_IDLE,
    RECV_MODE_DMA_RECV_IDLE
};

template<typename T, typename TYPE = int32_t>
struct devCbData
{
    TYPE type;
    T data;
    uint32_t dataPerSize;
    uint32_t dataOfobjCount;
    uint32_t len;
};

using initCallbackExt = std::function<void(bool binit)>;

class mDevice
{
    using interruptCallback = std::function<void(mDevice*, void*)>;
public:
    explicit mDevice(const std::string& name):_devname(name)
    {
        mDeviceManager::getInstance()->registerDevice(_devname,this);
    }
    explicit mDevice(std::string&& name):_devname(std::move(name))
    {
        mDeviceManager::getInstance()->registerDevice(_devname,this);
    }
    virtual ~mDevice()
    {
        mDeviceManager::getInstance()->unregisterDevice(_devname);
    }
    mDevice(const mDevice&) = delete;
    mDevice(mDevice&&) = delete;
    mDevice& operator=(const mDevice&) = delete;
    mDevice& operator=(mDevice&&) = delete;
    std::string getDeviceName() const {return _devname;}

    void registerInterruptCb(const interruptCallback& cb){_cb = cb;}
    void unregisterInterrupt(){_cb = nullptr;}
    void runInterruptCb(void* p){if(_cb)_cb(this,p);}
    void runInitCallback(bool binit){if(_initcb)_initcb(binit);}

protected:
    template<typename ClassType, typename MemberType>
    static ClassType* GetObjectFromMember(MemberType ClassType::* member_ptr, MemberType* member_address) {
        // 临时存储用于计算偏移量（不调用构造函数）
        alignas(ClassType) char temp_buf[sizeof(ClassType)]{};
        ClassType* temp_obj = reinterpret_cast<ClassType*>(temp_buf);
        
        // 计算成员变量的偏移量
        uintptr_t offset = reinterpret_cast<uintptr_t>(&(temp_obj->*member_ptr)) - 
                        reinterpret_cast<uintptr_t>(temp_obj);
        
        // 返回所属对象地址
        return reinterpret_cast<ClassType*>(reinterpret_cast<uintptr_t>(member_address) - offset);
    }
protected:
    std::string _devname;
    initCallbackExt _initcb;
private:
    interruptCallback _cb;
};
}