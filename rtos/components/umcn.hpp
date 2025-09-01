#pragma once
#include "mipc.hpp"
#include "mscheduler.hpp"
#include "mtimer.hpp"
#include <functional>
#include <unordered_map>
#include "containers.hpp"
#include "atomic.hpp"
#include "doublebuffer.hpp"

#define MCN_PUB_EVENT            (1 << 0)
#define MCN_MAX_LINK_NUM         30

class mcnNode;
class mcnHub
{
public:
    mcnHub(const std::string& objname, const uint32_t objsize):
    _objName(objname),
    _objSize(objsize),
    _doubleBuffer(nullptr),
    _linkHead(nullptr),
    _linkTail(nullptr),
    _linkNum(0),
    _bpublished(false),
    _bsuspend(false)
    {
        
    }
    ~mcnHub()
    {
        
    }
    mcnHub(const mcnHub& mq) = delete;
    mcnHub(mcnHub&& mq) = delete;
    mcnHub& operator=(const mcnHub& mq) = delete;
    mcnHub& operator=(mcnHub&& mq) = delete;
    mResult init();
    mResult deInit();
    const std::string& getObjName() const {return _objName;}
    const uint32_t getObjSize() const {return _objSize;}
    void* getData() {    
        if (!_doubleBuffer) return nullptr;
        return _doubleBuffer->getReadBuffer();
    }
    mcnNode* getLinkHead() {return _linkHead;}
    mcnNode* getLinkT() {return _linkTail;}
    uint32_t getLinkNum() const {return _linkNum;}
    bool bPublished() const {return _bpublished;}
    bool bSuspend() const {return _bsuspend;}
    void suspend() {_bsuspend = true;}
    void resume() {_bsuspend = false;}
    mcnNode* subscribe(const char* nodeName);
    mResult unSubscribe(mcnNode* node);
    mResult unSubscribe(const std::string& nodeName);
    mcnNode* getNode(const std::string& nodeName);
    mResult publish(const void* data, bool bsync = false);
    bool poll(mcnNode* node);
    bool wait(int32_t timeout);
    mResult copy(mcnNode* node, void* buffer);
    /* This function will directly copy topic data from hub no matter it has been updated or not and won't clear the renewal flag*/
    mResult copyDirectly(void* buffer);
    void clear(mcnNode* node);
    void clear(const char* nodeName);
    static mcnHub* getObject(const char* objname);
private:
    const std::string _objName;
    const uint32_t _objSize;
    DoubleBuffer* _doubleBuffer;
    mcnNode* _linkHead;
    mcnNode* _linkTail;
    uint32_t _linkNum;
    bool _bpublished;
    bool _bsuspend;
    mEvent _event;
    static std::unordered_map<std::string, mcnHub*> _mcnHubList;
};

class mcnNode
{
public:
    mcnNode(const std::string& name, mcnHub* hub):
    _hub(hub),
    _renewal(0),
    _next(nullptr),
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
    void setNext(mcnNode* next) {_next = next;}
    mcnNode* getNext() {return _next;}
    const std::string& getName() const {return _name;}
private:
    mcnHub* _hub;
    volatile uint8_t _renewal;
    mcnNode* _next;
    const std::string _name;
};