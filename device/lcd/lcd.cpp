#include "lcd.hpp"
#include "delay.h"
Lcd169::Lcd169(const char* name, uint16_t width, uint16_t height, uint8_t xOffset, uint8_t yOffset, uint8_t bitPerPixel) :
mDev::mDisplay(name,width,height,xOffset,yOffset,bitPerPixel),spix(nullptr),dc(nullptr),rst(nullptr),bl(nullptr),cs(nullptr)
{
    
}
Lcd169::~Lcd169()
{
    spix = nullptr;
}
void Lcd169::init(mDev::mSpi* spix, mDev::mGpio* dc, mDev::mGpio* rst, mDev::mGpio* bl, mDev::mGpio* cs)
{
    this->spix = spix;
    this->dc = dc;
    this->rst = rst;
    this->bl = bl;
    this->cs = cs;
    writeCommand(0x36);       // 显存访问控制 指令，用于设置访问显存的方式
	writeData8bit(0x00);     // 配置成 从上到下、从左到右，RGB像素格式

	writeCommand(0x3A);			// 接口像素格式 指令，用于设置使用 12位、16位还是18位色
	writeData8bit(0x05);     // 此处配置成 16位 像素格式

    // 接下来很多都是电压设置指令，直接使用厂家给设定值
 	writeCommand(0xB2);			
	writeData8bit(0x0C);
	writeData8bit(0x0C); 
	writeData8bit(0x00); 
	writeData8bit(0x33); 
	writeData8bit(0x33); 			

	writeCommand(0xB7);		   // 栅极电压设置指令	
	writeData8bit(0x35);     // VGH = 13.26V，VGL = -10.43V

	writeCommand(0xBB);			// 公共电压设置指令
	writeData8bit(0x19);     // VCOM = 1.35V

	writeCommand(0xC0);
	writeData8bit(0x2C);

	writeCommand(0xC2);       // VDV 和 VRH 来源设置
	writeData8bit(0x01);     // VDV 和 VRH 由用户自由配置

	writeCommand(0xC3);			// VRH电压 设置指令  
	writeData8bit(0x12);     // VRH电压 = 4.6+( vcom+vcom offset+vdv)
				
	writeCommand(0xC4);		   // VDV电压 设置指令	
	writeData8bit(0x20);     // VDV电压 = 0v

	writeCommand(0xC6); 		// 正常模式的帧率控制指令
	writeData8bit(0x0F);   	// 设置屏幕控制器的刷新帧率为60帧    

	writeCommand(0xD0);			// 电源控制指令
	writeData8bit(0xA4);     // 无效数据，固定写入0xA4
	writeData8bit(0xA1);     // AVDD = 6.8V ，AVDD = -4.8V ，VDS = 2.3V

	writeCommand(0xE0);       // 正极电压伽马值设定
	writeData8bit(0xD0);
	writeData8bit(0x04);
	writeData8bit(0x0D);
	writeData8bit(0x11);
	writeData8bit(0x13);
	writeData8bit(0x2B);
	writeData8bit(0x3F);
	writeData8bit(0x54);
	writeData8bit(0x4C);
	writeData8bit(0x18);
	writeData8bit(0x0D);
	writeData8bit(0x0B);
	writeData8bit(0x1F);
	writeData8bit(0x23);

	writeCommand(0xE1);      // 负极电压伽马值设定
	writeData8bit(0xD0);
	writeData8bit(0x04);
	writeData8bit(0x0C);
	writeData8bit(0x11);
	writeData8bit(0x13);
	writeData8bit(0x2C);
	writeData8bit(0x3F);
	writeData8bit(0x44);
	writeData8bit(0x51);
	writeData8bit(0x2F);
	writeData8bit(0x1F);
	writeData8bit(0x1F);
	writeData8bit(0x20);
	writeData8bit(0x23);

	writeCommand(0x21);       // 打开反显，因为面板是常黑型，操作需要反过来

 // 退出休眠指令，LCD控制器在刚上电、复位时，会自动进入休眠模式 ，因此操作屏幕之前，需要退出休眠  
	writeCommand(0x11);       // 退出休眠 指令
    delay_ms(120);
                   // 需要等待120ms，让电源电压和时钟电路稳定下来

 // 打开显示指令，LCD控制器在刚上电、复位时，会自动关闭显示 
	writeCommand(0x29);       // 打开显示   	
	
// 以下进行一些驱动的默认设置
    setDirection(mDev::DIRECTION_V);  	      //	设置显示方向
	setBackColor(LCD_BLUE);           // 设置背景色
 	setColor(LCD_WHITE);               // 设置画笔色  
	clear();                           // 清屏
    setFont(FONT_TYPE_24);
    setShowNumMode(mDev::Fill_ZERO);
    bl->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
}
void Lcd169::setAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    writeCommand(0x2a);			//	列地址设置，即X坐标
	writeData16bit(x1+xOffset);
	writeData16bit(x2+xOffset);

	writeCommand(0x2b);			//	行地址设置，即Y坐标
	writeData16bit(y1+yOffset);
	writeData16bit(y2+yOffset);

	writeCommand(0x2c);			//	开始写入显存，即要显示的颜色数据
}
void Lcd169::writeCommand(uint8_t command)
{
    cmdEnable();
    if(spix)
    {
        spix->write(&command, 1);
    }
}
void Lcd169::writeData8bit(uint8_t data)
{
    dataEnable();
    if(spix)
    {
        spix->write(&data, 1);
    }
}
void Lcd169::writeData16bit(uint16_t data)
{
    uint8_t lcd_data_buff[2];    // 数据发送区
    dataEnable();                // 数据指令选择 引脚输出高电平，代表本次传输 数据
  
    lcd_data_buff[0] = data>>8;  // 将数据拆分
    lcd_data_buff[1] = data;
    if(spix)
    {
        spix->write(lcd_data_buff, 2);
    }
}
void Lcd169::writeBuff(uint8_t *DataBuff, uint16_t DataSize)
{
	dataEnable();      // 数据指令选择 引脚输出高电平，代表本次传输 数据	
    if(spix)
    {
        spix->write(DataBuff, DataSize);
    }
}
void Lcd169::internelSetDirection(mDev::Direction dir)
{
    if( direction == mDev::DIRECTION_H )   // 横屏显示
    {
        writeCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
        writeData8bit(0x70);        // 横屏显示
    }
    else if( direction == mDev::DIRECTION_V )
    {
        writeCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
        writeData8bit(0x00);        // 垂直显示				
    }
    else if( direction == mDev::DIRECTION_H_FLIP )
    {
        writeCommand(0x36);   			 // 显存访问控制 指令，用于设置访问显存的方式
        writeData8bit(0xA0);         // 横屏显示，并上下翻转，RGB像素格式			
    }
    else if( direction == mDev::DIRECTION_V_FLIP )
    {
        writeCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
        writeData8bit(0xC0);        // 垂直显示 ，并上下翻转，RGB像素格式			
    }
}