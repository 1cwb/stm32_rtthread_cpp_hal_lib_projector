#include "rtoscommon.hpp"
#include "lvgl.h"
#include "mtimer.hpp"
#include "lv_port_disp_template.h"
#include "mthread.hpp"

int lvglInit()
{
    lv_init();
    lv_port_disp_init();
    mTimer* lvglTimer = mTimer::create("lvgltm", 1, mTimerStateFlag::TIMER_FLAG_PERIODIC, [](){
        lv_tick_inc(1);
    });
    lvglTimer->start();
    return 0;
}
INIT_EXPORT(lvglInit, "0.8");