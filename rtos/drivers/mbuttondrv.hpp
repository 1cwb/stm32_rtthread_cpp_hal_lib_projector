#pragma once
#include "mdevice.hpp"
#include "mgpiodrv.hpp"
#include <list>

namespace mDev
{
enum mButtonEvent
{
    BTN_PRESS_DOWN = 0,
    BTN_PRESS_CLICK,
    BTN_PRESS_DOUBLE_CLICK,
    BTN_PRESS_REPEAT_CLICK,
    BTN_PRESS_SHORT_START,
    BTN_PRESS_SHORT_UP,
    BTN_PRESS_LONG_START,
    BTN_PRESS_LONG_UP,
    BTN_PRESS_LONG_HOLD,
    BTN_PRESS_LONG_HOLD_UP,
    BTN_PRESS_MAX,
    BTN_PRESS_NONE,
};
enum mButtonStatus
{
    BTN_STAGE_DEFAULT = 0,
    BTN_STAGE_DOWN    = 1,
    BTN_STAGE_MULTIPLE_CLICK = 2
};
class mButton : public mDevice
{
    using buttonEventCallback = std::function<void(mButton*, mButtonEvent)>;
public:
    mButton() = delete;
    mButton(const char* name, mGpio* gpiox, uint8_t activeLevel = 0, uint16_t shortPressStartTick = 1500, uint16_t longPressStartTick = 3000, uint16_t longHoldStartTick = 4500):
    mDevice(name), _scanCnt(0), _clickCnt(0), _maxMultipleClicksInterval(MAX_MULTIPLE_CLICKS_INTERVAL), _debounceTick(msToScanCnt(20)), 
    _shortPressStartTick(msToScanCnt(shortPressStartTick)), _longPressStartTick(msToScanCnt(longPressStartTick)), _longHoldStartTick(msToScanCnt(longHoldStartTick)),
    _pressedLogicLevel(activeLevel),_bpressed(false),_event(BTN_PRESS_NONE), _status(BTN_STAGE_DEFAULT),
    _gpiox(gpiox) {};
    virtual ~mButton(){};
    static uint16_t msToScanCnt(uint16_t ms)
    {
        return ms / (1000 / BTN_SCAN_FREQ_HZ);
    }
    void scan()
    {
        if(_pressedLogicLevel == 0)
        {
            _bpressed = _gpiox->getLevel() == mDev::mGpio::LEVEL_LOW ? true : false;
        }
        else 
        {
            _bpressed = _gpiox->getLevel() == mDev::mGpio::LEVEL_HIGH ? true : false;
        }
    }
    void process()
    {
        if (_status != BTN_STAGE_DEFAULT)
        {
            _scanCnt ++;
            if (_scanCnt >= std::numeric_limits<decltype(_scanCnt)>::max())
            {
                _scanCnt = _longHoldStartTick;
            }
        }
        switch (_status)
        {
        case BTN_STAGE_DEFAULT:
            if(_bpressed)
            {
                _scanCnt = 0;
                _clickCnt = 0;
                _event = BTN_PRESS_DOWN;
                runEventCb(_event);
                _status = BTN_STAGE_DOWN;
            }
            else
            {
                _event = BTN_PRESS_NONE;
            }
            break;
        case BTN_STAGE_DOWN:
            if(_bpressed)
            {
                if(_clickCnt > 0)/* multiple click */
                {
                    if (_scanCnt > _maxMultipleClicksInterval)
                    {
                        _event = _clickCnt <  (uint16_t)BTN_PRESS_REPEAT_CLICK ? (mButtonEvent)_clickCnt : BTN_PRESS_REPEAT_CLICK;
                        runEventCb(_event);

                        /* swtich to button down stage */
                        _status = BTN_STAGE_DOWN;
                        _scanCnt = 0;
                        _clickCnt = 0;
                    }
                }
                else if (_scanCnt >= _longHoldStartTick)
                {
                    if (_event != BTN_PRESS_LONG_HOLD)
                    {
                        _event = BTN_PRESS_LONG_HOLD;
                        runEventCb(_event);
                    }
                }
                else if (_scanCnt >= _longPressStartTick)
                {
                    if (_event != BTN_PRESS_LONG_START)
                    {
                        _event = BTN_PRESS_LONG_START;
                        runEventCb(_event);
                    }
                }
                else if (_scanCnt >= _shortPressStartTick)
                {
                    if (_event != BTN_PRESS_SHORT_START)
                    {
                        _event = BTN_PRESS_SHORT_START;
                        runEventCb(_event);
                    }
                }
            }
            else /* button up */
            {
                if (_scanCnt >= _longHoldStartTick)
                {
                    _event = BTN_PRESS_LONG_HOLD_UP;
                    runEventCb(_event);
                    _status = BTN_STAGE_DEFAULT;
                }
                else if (_scanCnt >= _longPressStartTick)
                {
                    _event = BTN_PRESS_LONG_UP;
                    runEventCb(_event);
                    _status = BTN_STAGE_DEFAULT;
                }
                else if (_scanCnt >= _shortPressStartTick)
                {
                    _event = BTN_PRESS_SHORT_UP;
                    runEventCb(_event);
                    _status = BTN_STAGE_DEFAULT;
                }
                else
                {
                    /* swtich to multiple click stage */
                    _status = BTN_STAGE_MULTIPLE_CLICK;
                    _clickCnt ++;
                }
            }
            break;
            case BTN_STAGE_MULTIPLE_CLICK: /* stage: multiple click */
            if (_bpressed) /* is pressed */
            {
                /* swtich to button down stage */
                _status = BTN_STAGE_DOWN;
                _scanCnt = 0;
            }
            else
            {
                if (_scanCnt > _maxMultipleClicksInterval)
                {
                    _event = _clickCnt <  (uint16_t)BTN_PRESS_REPEAT_CLICK ? (mButtonEvent)_clickCnt : BTN_PRESS_REPEAT_CLICK;
                    runEventCb(_event);
                    /* swtich to default stage */
                    _status = BTN_STAGE_DEFAULT;
                }
            }
            break;
        default:
            break;
        }
    }
    void registerEventCb(const buttonEventCallback& cb){_cb = cb;}
    void unregisterEventCb(){_cb = nullptr;}
    void runEventCb(mButtonEvent ev){if(_cb)_cb(this,ev);}
    bool isPressed() const {return _bpressed;}
private:
    static constexpr uint16_t BTN_SCAN_FREQ_HZ = 50;
    static constexpr uint16_t MAX_MULTIPLE_CLICKS_INTERVAL = (300 / (1000 / BTN_SCAN_FREQ_HZ));
private:
    uint16_t _scanCnt;
    uint16_t _clickCnt;
    uint16_t _maxMultipleClicksInterval;

    uint16_t _debounceTick;
    uint16_t _shortPressStartTick;
    uint16_t _longPressStartTick;
    uint16_t _longHoldStartTick;

    uint8_t _pressedLogicLevel;
    uint8_t _bpressed;
    mButtonEvent _event;
    mButtonStatus _status;
    buttonEventCallback _cb;
    mGpio* _gpiox;
};

class mButtonManager final : public mDevice
{
public:
    mButtonManager() = delete;
    mButtonManager(const char* name):mDevice(name){_sem.init("btmase", 1, mIpcFlag::IPC_FLAG_FIFO);};
    virtual ~mButtonManager(){_sem.detach();};
    void addButton(mButton* button)
    {
        _sem.semTake(WAITING_FOREVER);
        for(auto& it : _buttonlist)
        {
            if(it->getDeviceName().compare(button->getDeviceName()) == 0)
            {
                return;
            }
        }
        _buttonlist.emplace_back(button);
        _sem.semRelease();
    }
    void removeButton(mButton* button)
    {
        _sem.semTake(WAITING_FOREVER);
        _buttonlist.remove(button);
        _sem.semRelease();
    }
    void removeButton(const char* name)
    {
        _sem.semTake(WAITING_FOREVER);
        for(auto it = _buttonlist.begin(); it != _buttonlist.end(); it++)
        {
            if((*it)->getDeviceName().compare(name) == 0)
            {
                _buttonlist.remove(*it);
                break;
            }
        }
        _sem.semRelease();
    }

    void process()
    {
        _sem.semTake(WAITING_FOREVER);
        for(auto& it : _buttonlist)
        {
            it->scan();
        }
        for(auto& it : _buttonlist)
        {
            it->process();
        }
        _sem.semRelease();
    }
private:
    std::list<mButton*, mMemAllocator<mButton*>> _buttonlist;
    mSemaphore _sem;
};
}