#pragma once
#include "rtoscommon.hpp"

class mKservice
{
public:
    static mKservice* getInstance()
    {
        static mKservice service;
        return &service;
    }

/**
 * This function finds the first bit set (beginning with the least significant bit)
 * in value and return the index of that bit.
 *
 * Bits are numbered starting at 1 (the least significant bit).  A return value of
 * zero from any of these functions means that the argument was zero.
 *
 * @return return the index of the first bit set. If value is 0, then this function
 * shall return 0.
 */
int __rt_ffs(int value);

private:
    mKservice()
    {
    }
    ~mKservice()
    {

    }
    mKservice(const mKservice&) = delete;
    mKservice(mKservice&&) = delete;
    mKservice& operator=(const mKservice&) = delete;
    mKservice& operator=(mKservice&&) = delete;

    static const uint8_t __lowest_bit_bitmap[];
};