#pragma once

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
    virtual int	init();
    
};

};