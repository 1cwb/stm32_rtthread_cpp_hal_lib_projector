#include "datapublish.hpp"
#include "rtoscommon.hpp"

mcnHub* imu1Hub = nullptr;
mcnHub* imu2Hub = nullptr;
mcnHub* mag1Hub = nullptr;
mcnHub* mb1Hub = nullptr;
mcnHub* ahrsHub = nullptr;
mcnNode* imu1Node = nullptr;
mcnNode* imu2Node = nullptr;
mcnNode* mag1Node = nullptr;
mcnNode* mb1Node = nullptr;
mcnNode* ahrsNode = nullptr;
mcnNode* ahrsSendBackToRemoteNode = nullptr;
int mcnInit()
{
    imu1Hub = new mcnHub("imu1Hub",sizeof(float)*6);
    if(!imu1Hub)
    {
        return -1;
    }
    imu1Hub->init();
    imu1Node = imu1Hub->subscribe("imu1Node");
    if(!imu1Node)
    {
        return -1;
    }
    imu2Hub = new mcnHub("imu2Hub",sizeof(float)*6);
    if(!imu2Hub)
    {
        return -1;
    }
    imu2Hub->init();
    imu2Node = imu2Hub->subscribe("imu2Node");
    if(!imu2Node)
    {
        return -1;
    }
    mag1Hub = new mcnHub("mag1Hub",sizeof(float)*3);
    if(!mag1Hub)
    {
        return -1;
    }
    mag1Hub->init();
    mag1Node = mag1Hub->subscribe("mag1Node");
    if(!mag1Node)
    {
        return -1;
    }
    mb1Hub = new mcnHub("mb1Hub",sizeof(float));
    if(!mb1Hub)
    {
        return -1;
    }
    mb1Hub->init();
    mb1Node = mb1Hub->subscribe("mb1Node");
    if(!mb1Node)
    {
        return -1;
    }
    ahrsHub = new mcnHub("ahrsHub",sizeof(float)*4);
    if(!ahrsHub)
    {
        return -1;
    }
    ahrsHub->init();
    ahrsNode = ahrsHub->subscribe("ahrsNode");
    if(!ahrsNode)
    {
        return -1;
    }
    ahrsSendBackToRemoteNode = ahrsHub->subscribe("ahrsSendBackToRemoteNode");
    if(!ahrsSendBackToRemoteNode)
    {
        return -1;
    }
    return 0;
}

INIT_EXPORT(mcnInit,"5");