#include "mdev.hpp"
#include "containers.hpp"
namespace mdev
{
    mDev::mDev(const char* devname):_devname(devname)
    {
        printf("DBG: mDev::mDev\r\n");
        mResult ret = _lock.init(devname,1,IPC_FLAG_FIFO);
        if(ret != M_RESULT_EOK)
        {
            printf("sem init fail %s()\r\n",__FUNCTION__);
        }
    }
    mDev::~mDev()
    {
        if(_registered)
        {
            unregisterDriver(_devname);
        }
        if(_pollset)
        {
            delete[] _pollset;
        }
        _lock.detach();
    }
	mResult mDev::registerClassDevName(const char* classDevName, int* instance)
    {
        if(!classDevName)
        {
            return M_RESULT_EINVAL;
        }
        int classInstance = 0;
        mResult ret = M_RESULT_ERROR;
        while(classInstance < 4)
        {
            char name[32];
            snprintf(name, sizeof(name), "%s%d",classDevName, classInstance);
            ret = registerDriver(name, &fops, 0666, (void*)this);
            if(ret == M_RESULT_EOK)
            {
                break;
            }
            classInstance++;
        }
        if(classInstance == 4)
        {
            return ret;
        }
        *instance = classInstance;
        return M_RESULT_EOK;
    }
	mResult mDev::unregisterClassDevName(const char* classDevName, unsigned classInstance)
    {
        char name[32];
        snprintf(name, sizeof(name), "%s%u", classDevName, classInstance);
        return unregisterDriver(name);
    }
    mResult mDev::init()
    {
        mResult ret = M_RESULT_ERROR;
        if(_devname != nullptr)
        {
            ret = registerDriver(_devname, &fops, 0666, (void*)this);
            if(ret == M_RESULT_EOK)
            {
                _registered = true;
            }
            else
            {
                ret = M_RESULT_NODEV;
            }
        }
        return ret;
    }
    mResult mDev::open(file_t* filep)
    {
        mResult ret = M_RESULT_ERROR;
        lock();
        _openCount ++;
        if(_openCount == 1)
        {
            ret = openFirst(filep);
            if(ret != M_RESULT_EOK)
            {
                _openCount --;
            }
        }
        unlock();
        return ret;
    }
	mResult mDev::close(file_t* filep)
    {
        mResult ret = M_RESULT_ERROR;
        lock();
        if(_openCount > 0)
        {
            _openCount --;
            if(_openCount == 0)
            {
                ret = closeLast(filep);
            }
        }
        else
        {
            ret = M_RESULT_BADF;
        }
        unlock();
        return ret;
    }
	mResult	mDev::poll(file_t *filep, pollfd_t *fds, bool setup)
    {
        mResult ret = M_RESULT_ERROR;
        if(setup)
        {
            fds->priv = (void*) filep;
        
            lock();
            while((ret = storePollWaiter(fds)) == M_RESULT_NOFILE)
            {
                if(_maxPollWaiters >= 256/2)
                {
                    ret = M_RESULT_ENOMEM;
                    break;
                }
                const uint8_t newcount = _maxPollWaiters > 0 ? _maxPollWaiters * 2 : 1;
                pollfd_t** prevPollset = _pollset;
                pollfd_t** newPollset = new pollfd_t*[newcount];

                if(prevPollset == _pollset)
                {
                    if(!newPollset)
                    {
                        ret = M_RESULT_ENOMEM;
                        break;
                    }
                    if(_maxPollWaiters > 0)
                    {
                        memset(newPollset + _maxPollWaiters, 0, sizeof(pollfd_t*)*(newcount - _maxPollWaiters));
                        memcpy(newPollset, _pollset, sizeof(pollfd_t*)*_maxPollWaiters);
                    }
                    _pollset = newPollset;
                    _pollset[_maxPollWaiters] = fds;
                    _maxPollWaiters = newcount;

                    if(prevPollset)
                    {
                        delete [] prevPollset;
                    }
                    ret = M_RESULT_EOK;
                    break;
                }
                //we have to retry
                delete[] newPollset;
            }
            if(ret == M_RESULT_EOK)
            {
                fds->revents |= fds->events & pollState(filep);
                if(fds->revents != 0)
                {
                    reinterpret_cast<mSemaphore*>(fds->sem)->semRelease();
                }
            }
            unlock();
        }
        else
        {
            lock();
            ret = removePollWaiter(fds);
            unlock();
        }
        return ret;
    }
    void mDev::pollNotifyOne(pollfd_t* fds, short events)
    {
        fds->revents |= fds->events & events;
        if(fds->revents != 0)
        {
            reinterpret_cast<mSemaphore*>(fds->sem)->semRelease();
        }
    }
    void mDev::pollNotify(short events)
    {
        lock();
        for(unsigned i = 0; i < _maxPollWaiters; i++)
        {
            if(nullptr != _pollset[i])
            {
                pollNotifyOne(_pollset[i], events);
            }
        }
        unlock();
    }

    mResult mDev::storePollWaiter(pollfd_t* fds)
    {
        for(unsigned i = 0; i < _maxPollWaiters; i++)
        {
            if(_pollset[i] == nullptr)
            {
                _pollset[i] = fds;
                return M_RESULT_EOK;
            }
        }
        return M_RESULT_NOFILE;
    }
	mResult mDev::removePollWaiter(pollfd_t* fds)
    {
        for(unsigned i = 0; i < _maxPollWaiters; i++)
        {
            if(fds == _pollset[i])
            {
                _pollset[i] = nullptr;
                return M_RESULT_EOK;
            }
        }
        return M_RESULT_EINVAL;
    }
};