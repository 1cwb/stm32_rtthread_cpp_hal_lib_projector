#include "stm32h7xx_hal.h"
#include "led.h"
#include "mthread.h"

int main(void)
{
    led_init();
    led_off();
     
    mthread* th3 = mthread::create("th3",512,0,20,[&](){
        //using MyString = std::basic_string<char, std::char_traits<char>, mMemAllocator<char>>;
        int m = 100;
        while(1)
        {
            mthread::threadDelay(1000);
            printf("get event\r\n");
        }
    });
    if(th3)
    {
        th3->startup();
    }

    mthread* th4 = mthread::create("th4",512,0,20,[&](){
        //using MyString = std::basic_string<char, std::char_traits<char>, mMemAllocator<char>>;
        int m = 0;
        while(1)
        {
            printf("recv ... m = %d\r\n",m);
        }
    });
    if(th4)
    {
        th4->startup();
    }

    int i = 0;
    mTimer* tim1 = mTimer::create("tim1", 100, TIMER_FLAG_PERIODIC, [&](){
        i++;
        if(i == 4)
        {
            i = 0;
        }
        tim1->setTimerAndStart(200);
        printf("timeout\r\n");
    });
    tim1->start();
    while(1)
    {
       mthread::threadSleep(1000);
       led_on();
       mthread::threadSleep(1000);
       led_off();
       printf("thread run now\r\n");
    }
    return 0;
}
