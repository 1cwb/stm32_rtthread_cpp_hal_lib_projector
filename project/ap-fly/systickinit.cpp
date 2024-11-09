#include "systick.hpp"

int systickInit()
{
    systick* msystick = new systick;
    if(msystick)
    {
        msystick->init();//do no thing
    }
    return 0;
}
INIT_EXPORT(systickInit, "0.4");