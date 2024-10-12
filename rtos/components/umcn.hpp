#pragma once
#include "mipc.hpp"
#include "mscheduler.hpp"
#include "mtimer.hpp"
#include <functional>
#include <list>
#include "containers.hpp"
#include "atomic.hpp"

#define MCN_PUB_EVENT            (1 << 0)
#define MCN_MAX_LINK_NUM         30
#define MCN_FREQ_EST_WINDOW_LEN  5

using mcnPubCallback = std::function<void (void*)>;
using mcnPubEchoCallback = std::function<int (void*)>;

class mcnNode;
class mcnHub
{
public:
    mcnHub(const char* objname, const uint32_t objsize):
    _objName(objname),
    _objSize(objsize),
    _pdata(nullptr),
    _linkHead(nullptr),
    _linkTail(nullptr),
    _linkNum(0),
    _bpublished(false),
    _bsuspend(false),
    _freq(0.0F),
    _windowIndex(0)
    {
        
    }
    ~mcnHub()
    {
        
    }
    mcnHub(const mcnHub& mq) = delete;
    mcnHub(mcnHub&& mq) = delete;
    mcnHub& operator=(const mcnHub& mq) = delete;
    mcnHub& operator=(mcnHub&& mq) = delete;
    mResult init(const mcnPubEchoCallback& cb);
    mResult deInit();
    const char* getObjName() const {return _objName;}
    const uint32_t getObjSize() const {return _objSize;}
    void* getData() {return _pdata;}
    mcnNode* getLinkHead() {return _linkHead;}
    mcnNode* getLinkT() {return _linkTail;}
    uint32_t getLinkNum() const {return _linkNum;}
    bool bPublished() const {return _bpublished;}
    bool bSuspend() const {return _bsuspend;}
    void suspend() {_bsuspend = true;}
    void resume() {_bsuspend = false;}
    float getFreq() const {return _freq;}
    uint16_t getWindowIndex() const {return _windowIndex;}
    mcnNode* subscribe(const char* nodeName, const mcnPubCallback& cb);
    mResult unSubscribe(mcnNode* node);
    mResult unSubscribe(const char* nodeName);
    mcnNode* getNode(const char* nodeName);
    mResult publish(const void* data, bool bsync = true);
    bool poll(mcnNode* node);
    bool wait(int32_t timeout);
    mResult copy(mcnNode* node, void* buffer);
    /* This function will directly copy topic data from hub no matter it has been updated or not and won't clear the renewal flag*/
    mResult copyDirectly(void* buffer);
    void clear(mcnNode* node);
    void clear(const char* nodeName);
    static mcnHub* getObject(const char* objname);
private:
    const char* _objName;
    const uint32_t _objSize;
    void* _pdata;
    mcnNode* _linkHead;
    mcnNode* _linkTail;
    uint32_t _linkNum;
    bool _bpublished;
    bool _bsuspend;
    float _freq;
    uint16_t _freqEstWindow[MCN_FREQ_EST_WINDOW_LEN] = {0};
    uint16_t _windowIndex;
    mcnPubEchoCallback _echoCb;
    mEvent _event;
    static std::list<mcnHub*, mMemAllocator<mcnHub*>> _mcnHubList;
    static mTimer _freqTimer;
    static mAtomic<bool> _btimerInit;
};

class mcnNode
{
public:
    mcnNode(const char* name, mcnHub* hub, mcnPubCallback cb = nullptr):
    _hub(hub),
    _renewal(0),
    _next(nullptr),
    _cb(cb),
    _name(name)
    {

    }
    ~mcnNode()
    {

    }
    mcnNode(const mcnNode& mq) = delete;
    mcnNode(mcnNode&& mq) = delete;
    mcnNode& operator=(const mcnNode& mq) = delete;
    mcnNode& operator=(mcnNode&& mq) = delete;
    mcnHub* getHub() {return _hub;}
    uint8_t getRenewal() const {return _renewal;}
    void setRenewal(uint8_t renewal) {_renewal = renewal;}
    void runPubCallback(void* param) {if(_cb){_cb(param);}}
    void setNext(mcnNode* next) {_next = next;}
    mcnNode* getNext() {return _next;}
    const char* getName() const {return _name;}
private:
    mcnHub* _hub;
    volatile uint8_t _renewal;
    mcnNode* _next;
    mcnPubCallback _cb;
    const char* _name;
};