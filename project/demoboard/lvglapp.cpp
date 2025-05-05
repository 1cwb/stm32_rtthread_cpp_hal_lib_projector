#include "rtoscommon.hpp"
#include "lvgl.h"
#include "mtimer.hpp"
#include "lv_port_disp_template.h"
#include "mthread.hpp"
#include "datapublish.hpp"

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


int lvglAppTask(void)
{
    mthread* lvglthreadapp = mthread::create("lvglth", 2048, 5, 20, [&](void* p){

        // 设置屏幕背景样式
        static lv_style_t screen_style;
        lv_style_init(&screen_style);
        lv_style_set_bg_color(&screen_style, lv_color_black());
        lv_obj_add_style(lv_scr_act(), &screen_style, 0);

        // 定义两个静态样式变量，用于背景和指示器的样式
        static lv_style_t style_bg;
        static lv_style_t style_indic;
        // 初始化背景样式
        lv_style_init(&style_bg);
        // 设置背景样式的边框颜色为蓝色
        lv_style_set_border_color(&style_bg, lv_palette_main(LV_PALETTE_BLUE));
        // 设置背景样式的边框宽度为2
        lv_style_set_border_width(&style_bg, 2);
        // 设置背景样式的内边距为6，使指示器变小
        lv_style_set_pad_all(&style_bg, 4); /*To make the indicator smaller*/
        // 设置背景样式的圆角半径为6
        lv_style_set_radius(&style_bg, 2);

        lv_style_set_bg_color(&style_bg, lv_color_white());
        // 设置背景样式的动画时间为1000毫秒
        //lv_style_set_anim_time(&style_bg, 1000);

        // 初始化指示器样式
        lv_style_init(&style_indic);
        // 设置指示器样式的背景不透明度为完全不透明
        lv_style_set_bg_opa(&style_bg, LV_OPA_COVER);
        // 设置指示器样式的背景不透明度为完全不透明
        lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
        // 设置指示器样式的背景颜色为蓝色
        lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN));
        // 设置指示器样式的圆角半径为0
        lv_style_set_radius(&style_indic, 0);

        lv_obj_t * bar1 = lv_bar_create(lv_scr_act());
        // 移除进度条对象的所有样式，确保干净的开始
        lv_obj_remove_style_all(bar1);  /*To have a clean start*/
        // 为进度条对象添加背景样式
        lv_obj_add_style(bar1, &style_bg, 0);
        // 为进度条对象的指示器部分添加指示器样式
        lv_obj_add_style(bar1, &style_indic, LV_PART_INDICATOR);

        lv_obj_set_size(bar1, 240, 20);
        lv_obj_center(bar1);
        lv_bar_set_range(bar1, 0, 65535); // 设置进度条范围为-50到+50
        lv_bar_set_mode(bar1, LV_BAR_MODE_SYMMETRICAL);
        lv_bar_set_value(bar1, 0, LV_ANIM_OFF);


        lv_obj_t * bar2 = lv_bar_create(lv_scr_act());
        // 移除进度条对象的所有样式，确保干净的开始
        lv_obj_remove_style_all(bar2);  /*To have a clean start*/
        // 为进度条对象添加背景样式
        lv_obj_add_style(bar2, &style_bg, 0);
        // 为进度条对象的指示器部分添加指示器样式
        lv_obj_add_style(bar2, &style_indic, LV_PART_INDICATOR);

        lv_obj_set_size(bar2, 20, 280);
        lv_obj_center(bar2);
        lv_bar_set_range(bar2, 0, 65535); // 设置进度条范围为-50到+50
        lv_bar_set_mode(bar2, LV_BAR_MODE_SYMMETRICAL);
        lv_bar_set_value(bar2, 0, LV_ANIM_OFF);
        int i = 0;
        uint32_t buff[5];
        while(true)
        {
            if(mcnJoyStickData && mcnJoyStickNode)
            {
                if(mcnJoyStickData->poll(mcnJoyStickNode))
                {
                    mcnJoyStickData->copy(mcnJoyStickNode, buff);
                    lv_bar_set_value(bar1, buff[3], LV_ANIM_OFF);
                    lv_bar_set_value(bar2, buff[4], LV_ANIM_OFF);
                }
            }

            //lv_bar_set_value(bar1, i++, LV_ANIM_OFF);
            //lv_bar_set_value(bar2, i++, LV_ANIM_OFF);
            if(i > 50)
            {
                i = -50;
            }
            mthread::threadMdelay(5);
        }
    },nullptr);
    lvglthreadapp->startup();

    mthread* lvglthread = mthread::create("lvglth", 2048, 5, 20, [&](void* p){
        while(true)
        {
            lv_timer_handler();
            mthread::threadMdelay(5);
        }
    },nullptr);
    lvglthread->startup();
    return 0;
}
TASK_EXPORT(lvglAppTask, "0.1");