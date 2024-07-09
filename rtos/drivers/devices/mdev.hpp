#pragma once
#include "platform.hpp"
#include "mipc.h"
#include "mpoll.hpp"

namespace mdev
{

class mDev
{
public:
    explicit mDev(const char* devname);

    // no copy, assignment, move, move assignment
	mDev(const mDev &) = delete;
	mDev &operator=(const mDev &) = delete;
	mDev(mDev &&) = delete;
	mDev &operator=(mDev &&) = delete;

	virtual ~mDev();
    virtual mResult	init();
	
	virtual mResult open(file_t* filep);
	virtual mResult close(file_t* filep);
	virtual ssize_t read(file_t *filep, char *buffer, size_t buflen){return -1;};
	virtual ssize_t	write(file_t *filep, const char *buffer, size_t buflen) { return -1; }
	virtual off_t	seek(file_t *filep, off_t offset, int whence) { return -1; }
	virtual mResult	ioctl(file_t *filep, int cmd, unsigned long arg) { return M_RESULT_ENOSYS; };
	mResult	poll(file_t *filep, pollfd_t *fds, bool setup);

	const char* getDevName() const {return _devname;}
protected:
	virtual short pollState(file_t* filep) {return 0;};
	virtual void pollNotifyOne(pollfd_t* fds, short events);
	virtual mResult openFirst(file_t* filep) {return M_RESULT_EOK;}
	virtual mResult closeLast(file_t* filep) {return M_RESULT_EOK;}
	mResult registerClassDevName(const char* classDevName, int* instance);
	mResult unregisterClassDevName(const char* classDevName, unsigned classInstance);
	void lock() {do{}while(_lock.semTake(WAITING_FOREVER) != M_RESULT_EOK);}
	void unlock() {_lock.semRelease();};
	void pollNotify(short events);
	static const fileOperations_t fops;
	mSemaphore _lock;
private:
	inline mResult storePollWaiter(pollfd_t* fds);
	inline mResult removePollWaiter(pollfd_t* fds);

	const char* _devname{nullptr};
	pollfd_t** _pollset{nullptr};
	bool _registered{false};
	uint8_t _maxPollWaiters{0};
	uint16_t _openCount{0};
};

};