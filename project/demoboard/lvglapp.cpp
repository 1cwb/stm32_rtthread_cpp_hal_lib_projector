#include "rtoscommon.hpp"
#include "lvgl.h"
#include "mtimer.hpp"
#include "lv_port_disp_template.h"
#include "mthread.hpp"
#include "datapublish.hpp"
#include "systeminfo.hpp"
#include "workqueue.hpp"
#include "workqueuemanager.hpp"

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

#define SAMPLE_BOX_WIDTH  80
#define SAMPLE_BOX_HEIGHT   80
#define SAMPLE_BALL_WIDTH  20
int lvglAppTask(void)
{
    mthread* lvglthreadapp = mthread::create("lvglth", 2048, 5, 20, [&](void* p){

        // 新增：CPU使用率标签
        lv_obj_t * cpu_label = lv_label_create(lv_scr_act());
        lv_obj_align(cpu_label, LV_ALIGN_TOP_LEFT, 10, 10);
        lv_obj_set_style_text_color(cpu_label, lv_color_white(), 0);
        lv_obj_set_style_text_font(cpu_label, &lv_font_montserrat_14, 0);
        lv_obj_move_foreground(cpu_label);  // 新增：置顶显示

        // 新增：创建容器用于水平排列两个方框
        lv_obj_t * container = lv_obj_create(lv_scr_act());
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        lv_obj_set_style_bg_color(container, lv_color_black(), 0);  // 改为继承父对象颜色
        lv_obj_set_size(container, lv_obj_get_width(lv_scr_act()), 88);
        lv_obj_align(container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        // 第一个正方形+圆球
        lv_obj_t * box1 = lv_obj_create(container);
        lv_obj_set_size(box1, SAMPLE_BOX_WIDTH, SAMPLE_BOX_HEIGHT);
        lv_obj_set_style_border_color(box1, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_pad_all(box1, 0, 0);  // 清除父容器内边距
        lv_obj_set_style_border_width(box1, 2, 0);
        lv_obj_set_style_bg_opa(box1, LV_OPA_TRANSP, 0);

        lv_obj_t * ball1 = lv_obj_create(box1);
        lv_obj_set_size(ball1, SAMPLE_BALL_WIDTH, SAMPLE_BALL_WIDTH);
        lv_obj_align(ball1, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_radius(ball1, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(ball1, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_shadow_width(ball1, 10, 0);
        lv_obj_set_style_shadow_color(ball1, lv_palette_darken(LV_PALETTE_RED, 2), 0);

        // 第二个正方形+圆球
        lv_obj_t * box2 = lv_obj_create(container);
        lv_obj_set_size(box2, SAMPLE_BOX_WIDTH, SAMPLE_BOX_HEIGHT);
        lv_obj_set_style_border_color(box2, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_pad_all(box2, 0, 0);  // 清除父容器内边距
        lv_obj_set_style_border_width(box2, 2, 0);
        lv_obj_set_style_bg_opa(box2, LV_OPA_TRANSP, 0);

        lv_obj_t * ball2 = lv_obj_create(box2);
        lv_obj_set_size(ball2, SAMPLE_BALL_WIDTH, SAMPLE_BALL_WIDTH);
        lv_obj_align(ball2, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_radius(ball2, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(ball2, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_shadow_width(ball2, 10, 0);
        lv_obj_set_style_shadow_color(ball2, lv_palette_darken(LV_PALETTE_RED, 2), 0);
        lv_refr_now(NULL);  // 立即执行刷新

        uint32_t buff[5];
        lv_coord_t border_width = lv_obj_get_style_border_width(box1, 0);  // 获取边框宽度
        printf("lv_obj_get_width(box1) = %d\r\n",lv_obj_get_width(box1));
        printf("lv_obj_get_width(ball1) = %d\r\n",lv_obj_get_width(ball1));
        int16_t max_range = lv_obj_get_width(box1) - lv_obj_get_width(ball1) - (border_width * 2);  // 实际可用空间 = 容器尺寸 - 圆球尺寸 - 两侧边框
        int16_t x = 0;  // 假设摇杆值范围0-65535
        int16_t y = 0;
        int16_t x1 = 0;  // 假设摇杆值范围0-65535
        int16_t y1 = 0;

        workItem* lvglwork = new workItem("lvglwork", 0, 1000, [&](void* param){
            lv_label_set_text_fmt(cpu_label, "CPU: %.1f%%", systemInfo::getInstance()->getCpuUsage());
        }, nullptr);
        workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(lvglwork);
        while(true)
        {
            if(mcnJoyStickData && mcnJoyStickNode)
            {
                if(mcnJoyStickData->poll(mcnJoyStickNode))
                {
                    mcnJoyStickData->copy(mcnJoyStickNode, buff);
                    mcnJoyStickData->copy(mcnJoyStickNode, buff);
                    x = (buff[1] * max_range) / 65535;  // 假设摇杆值范围0-1000
                    y = (buff[2] * max_range) / 65535;
                    x1 = (buff[3] * max_range) / 65535;  // 假设摇杆值范围0-1000
                    y1 = (buff[4] * max_range) / 65535;
                    // 限制坐标范围并移动圆球
                    x = (x < 0) ? 0 : (x > max_range) ? max_range : x;
                    y = (y < 0) ? 0 : (y > max_range) ? max_range : y;
                    x1 = (x1 < 0)? 0 : (x1 > max_range)? max_range : x1;
                    y1 = (y1 < 0)? 0 : (y1 > max_range)? max_range : y1;
                    lv_obj_set_pos(ball1, x, y);
                    lv_obj_set_pos(ball2, x1, y1);
                }
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