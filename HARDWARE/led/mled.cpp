#include "mled.hpp"
#include "rtoscommon.h"

mResult mled::init()
{
	mResult ret = mDev::init();
     if(ret != M_RESULT_EOK)
	 {
		return ret;
	 }
    led0Init();
	return M_RESULT_EOK;
}
mResult mled::ioctl(mdev::file_t *filep, int cmd, unsigned long arg)
{
    mResult result = M_RESULT_EOK;

	switch (cmd) {
	case LED_ON:
		led0On();
		break;

	case LED_OFF:
		led0Off();
		break;

	case LED_TOGGLE:
		led0Toggle();
		break;

	default:
		result = mDev::ioctl(filep, cmd, arg);
	}

	return result;
}
mled* led = nullptr;
static int ledInitProbe(void)
{
	if(!led)
	{
		led = new mled;
		led->init();
	}
	return 0;
}

INIT_EXPORT(ledInitProbe, "1");