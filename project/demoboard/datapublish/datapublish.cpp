#include "datapublish.hpp"
#include "rtoscommon.hpp"

mcnHub* mcnJoyStickData = nullptr;
mcnNode* mcnJoyStickNode = nullptr;
mcnHub* ahrsHub = nullptr;
mcnNode* ahrsSendBackToRemoteNode = nullptr;
int joyStickDataInit()
{
    mcnJoyStickData = new mcnHub("joyStickData",sizeof(uint32_t)*5);
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
    ahrsHub = new mcnHub("ahrsHub",sizeof(float)*3);
    if(!ahrsHub)
    {
        return -1;
    }
    ahrsHub->init();
    ahrsSendBackToRemoteNode = ahrsHub->subscribe("ahrsSendBackToRemoteNode");
    if(!ahrsSendBackToRemoteNode)
    {
        return -1;
    }
    return 0;
}

INIT_EXPORT(joyStickDataInit,"5");