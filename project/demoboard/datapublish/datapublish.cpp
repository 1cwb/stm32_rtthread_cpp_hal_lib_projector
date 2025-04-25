#include "datapublish.hpp"
#include "rtoscommon.hpp"

mcnHub* mcnJoyStickData = nullptr;
mcnNode* mcnJoyStickNode = nullptr;

int joyStickDataInit()
{
    mcnJoyStickData = new mcnHub("joyStickData",sizeof(uint32_t)*3);
    if(!mcnJoyStickData)
    {
        return -1;
    }
    mcnJoyStickData->init();
    mcnJoyStickNode = mcnJoyStickData->subscribe("joyStickNode");
    if(!mcnJoyStickNode)
    {
        return -1;
    }
    return 0;
}

INIT_EXPORT(joyStickDataInit,"5");