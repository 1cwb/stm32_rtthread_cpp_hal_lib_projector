#include "umcn.hpp"
#include <cstring>
std::list<mcnHub*, mMemAllocator<mcnHub*>> mcnHub::_mcnHubList;
mTimer mcnHub::_freqTimer;
mAtomic<bool> mcnHub::_btimerInit(false);

mResult mcnHub::init(const mcnPubEchoCallback& cb)
{
    void* pdata = nullptr;

    if(_pdata != nullptr)
    {
        return M_RESULT_EINVAL;
    }
    pdata = new uint8_t[_objSize];
    if(!pdata)
    {
        return M_RESULT_ENOMEM;
    }
    memset(pdata, 0, _objSize);

    _event.init(_objName, IPC_FLAG_FIFO);

    mSchedule::getInstance()->enterCritical();
    _pdata = pdata;
    _echoCb = cb;
    _mcnHubList.emplace_back(this);
    memset(_freqEstWindow, 0, sizeof(_freqEstWindow));
    _windowIndex = 0;
    mSchedule::getInstance()->exitCritical();
    
    if(!_btimerInit.load())
    {
        printf("%s()%d \r\n",__FUNCTION__,__LINE__);
        _freqTimer.init("UMCNIPC",1000,TIMER_FLAG_PERIODIC,[this](){
            for(auto it = this->_mcnHubList.begin(); it != this->_mcnHubList.end(); ++it)
            {
                if(*it)
                {
                    uint32_t cnt = 0;
                    for(int i = 0; i < MCN_FREQ_EST_WINDOW_LEN; i++)
                    {
                        cnt += (*it)->_freqEstWindow[i];
                        (*it)->_freq = (float)cnt / MCN_FREQ_EST_WINDOW_LEN;
                    }
                    (*it)->_windowIndex = ((*it)->_windowIndex+1) % MCN_FREQ_EST_WINDOW_LEN;
                    (*it)->_freqEstWindow[(*it)->_windowIndex] = 0;
                }
            }
        });
        if(_freqTimer.start() != M_RESULT_EOK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInit.store(true);
    }
    return M_RESULT_EOK;
}
mResult mcnHub::deInit()
{
    if(_btimerInit.load()&&_mcnHubList.empty())
    {
        _freqTimer.stop();
        _freqTimer.timerDetach();
        _btimerInit.store(false);
    }
    _event.detach();
    mSchedule::getInstance()->enterCritical();
    _mcnHubList.remove(this);
    delete [](uint8_t*)_pdata;
    _pdata = nullptr;
    mSchedule::getInstance()->exitCritical();
    return M_RESULT_EOK;
}

mcnNode* mcnHub::subscribe(const char* nodeName, const mcnPubCallback& cb)
{
    if(_linkNum >= MCN_MAX_LINK_NUM)
    {
        printf("Error: mcn link num is already full\r\n");
        return nullptr;
    }
    mcnNode* node = new mcnNode(nodeName, this, cb);

    mSchedule::getInstance()->enterCritical();
    if(_linkTail == nullptr)
    {
        _linkHead = _linkTail = node;
    }
    else
    {
        _linkTail->setNext(node);
        _linkTail = node;
    }

    ++_linkNum;
    mSchedule::getInstance()->exitCritical();
    if(_bpublished)
    {
        node->setRenewal(1);
        node->runPubCallback(getData());
    }
    return node;
}
mResult mcnHub::unSubscribe(mcnNode* node)
{
    mcnNode* curNode = _linkHead;
    mcnNode* preNode = nullptr;

    while(curNode)
    {
        if(curNode == node)
        {
            break;
        }
        preNode = curNode;
        curNode = preNode->getNext();
    }
    if(curNode == nullptr)
    {
        return M_RESULT_EEMPTY;
    }
    mSchedule::getInstance()->enterCritical();
    if(_linkNum == 1)
    {
        _linkHead = _linkTail = nullptr;
    }
    else
    {
        if(curNode == _linkHead)
        {
            _linkHead = curNode->getNext();
        }
        else if(curNode == _linkTail)
        {
            if(preNode)
            {
                    preNode->setNext(nullptr);
            }
            _linkTail = preNode;
        }
        else
        {
            preNode->setNext(curNode->getNext());
        }
    }
    _linkNum --;
    mSchedule::getInstance()->exitCritical();
    delete node;
    return M_RESULT_EOK;
}
mResult mcnHub::unSubscribe(const char* nodeName)
{
    return unSubscribe(getNode(nodeName));
}
mcnNode* mcnHub::getNode(const char* nodeName)
{
    mcnNode* node = _linkHead;
    while(node != nullptr)
    {
        if(strcmp(node->getName(), nodeName) == 0)
        {
            break;
        }
        node = node->getNext();
    }
    return node;
}
mcnHub* mcnHub::getObject(const char* objname)
{
    for(auto& it : _mcnHubList)
    {
        if(strcmp(it->getObjName(), objname))
        {
            return it;
        }
    }
    return nullptr;
}
mResult mcnHub::publish(const void* data, bool bsync)
{
    if(!data)
    {
        return M_RESULT_EINVAL;
    }
    if(_bsuspend)
    {
        return M_RESULT_EINVAL;
    }
    _freqEstWindow[_windowIndex] ++;
    mSchedule::getInstance()->enterCritical();
    memcpy(_pdata, data, _objSize);
    mcnNode* node = _linkHead;
    while(node != nullptr)
    {
        node->setRenewal(1);
        node = node->getNext();
    }
    _bpublished = true;
    mSchedule::getInstance()->exitCritical();

    node = _linkHead;
    while (node != nullptr)
    {
        node->runPubCallback(_pdata);
        node = node->getNext();
    }
    if(bsync)
    {
        _event.send(MCN_PUB_EVENT);
        reinterpret_cast<mEvent_t*>(&_event)->set &= ~MCN_PUB_EVENT;
    }
    return M_RESULT_EOK;
}

bool mcnHub::poll(mcnNode* node)
{
    bool renewal = false;
    if(!node)
    {
        return false;
    }
    mSchedule::getInstance()->enterCritical();
    renewal = node->getRenewal();
    mSchedule::getInstance()->exitCritical();
    return renewal;
}
bool mcnHub::wait(int32_t timeout)
{
    uint32_t recvSet = 0;
    if(_event.recv(MCN_PUB_EVENT,EVENT_FLAG_OR,timeout,&recvSet) != M_RESULT_EOK)
    {
        return false;
    }
    if(recvSet & MCN_PUB_EVENT)
    {
        return true;
    }
    else
    {
        return false;
    }
}
mResult mcnHub::copy(mcnNode* node, void* buffer)
{
    if(!node || !buffer)
    {
        return M_RESULT_EINVAL;
    }
    if(_pdata == nullptr)
    {
        return M_RESULT_ERROR;
    }
    if(!_bpublished)
    {
        return M_RESULT_BADF;
    }
    mSchedule::getInstance()->enterCritical();
    memcpy(buffer, _pdata, _objSize);
    node->setRenewal(0);
    mSchedule::getInstance()->exitCritical();
    return M_RESULT_EOK;
}

mResult mcnHub::copyDirectly(void* buffer)
{
    if(!buffer)
    {
        return M_RESULT_EINVAL;
    }
    if(!_pdata)
    {
        return M_RESULT_ERROR;
    }
    if(!_bpublished)
    {
        return M_RESULT_BADF;
    }
    mSchedule::getInstance()->enterCritical();
    memcpy(buffer, _pdata, _objSize);
    mSchedule::getInstance()->exitCritical();
    return M_RESULT_EOK;
}

void mcnHub::clear(mcnNode* node)
{
    if(!node)
    {
        return;
    }
    mSchedule::getInstance()->enterCritical();
    node->setRenewal(0);
    mSchedule::getInstance()->exitCritical();
}
void mcnHub::clear(const char* nodeName)
{
    if(!nodeName)
    {
        return;
    }
    clear(getNode(nodeName));
}