#include "datapublish.hpp"
#include "umcn.hpp"

mcnHub* mcnJoyStickData = nullptr;

int joyStickDataInit()
{
    mcnJoyStickData = new mcnHub("joyStickData",sizeof(uint32_t)*4);
    return 0;
}