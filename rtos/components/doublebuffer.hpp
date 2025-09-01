#pragma once
#include "atomic.hpp"

// 动态大小的双缓冲类
class DoubleBuffer {
public:
    DoubleBuffer(uint32_t bufferSize) : _bufferSize(bufferSize) {
        _buffers[0] = new uint8_t[bufferSize];
        _buffers[1] = new uint8_t[bufferSize];
        memset(_buffers[0], 0, bufferSize);
        memset(_buffers[1], 0, bufferSize);
        _writeIndex.store(0);
        _readIndex.store(0);
    }
    
    ~DoubleBuffer() {
        delete[] _buffers[0];
        delete[] _buffers[1];
    }
    
    void* getWriteBuffer() { 
        return _buffers[_writeIndex.load()];
    }
    
    void* getReadBuffer() { 
        return _buffers[_readIndex.load()];
    }
    
    void swap() {
        int writeIdx = _writeIndex.load();
        int newReadIdx = writeIdx;
        int newWriteIdx = 1 - writeIdx;
        
        _writeIndex.store(newWriteIdx);
        _readIndex.store(newReadIdx);
    }
    
    uint32_t getBufferSize() const { return _bufferSize; }
    
    // 禁止拷贝
    DoubleBuffer(const DoubleBuffer&) = delete;
    DoubleBuffer& operator=(const DoubleBuffer&) = delete;

private:
    uint8_t* _buffers[2];
    mAtomic<int> _writeIndex;
    mAtomic<int> _readIndex;
    uint32_t _bufferSize;
};