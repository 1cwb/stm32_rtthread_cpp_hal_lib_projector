#pragma once
#include "mdevice.hpp"
#include "fonts.hpp"
#include "mipc.hpp"

namespace mDev
{
#define ABS(X)  ((X) > 0 ? (X) : -(X))   
/*---------------------------------------- 常用颜色 ------------------------------------------------------

 1. 这里为了方便用户使用，定义的是24位 RGB888颜色，然后再通过代码自动转换成 16位 RGB565 的颜色
 2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色通道
 3. 用户可以在电脑用调色板获取24位RGB颜色，再将颜色输入LCD_SetColor()或LCD_SetBackColor()就可以显示出相应的颜色 
 */                                                  						
#define 	LCD_WHITE       0xFFFFFF	 // 纯白色
#define 	LCD_BLACK       0x000000    // 纯黑色
                        
#define 	LCD_BLUE        0x0000FF	 //	纯蓝色
#define 	LCD_GREEN       0x00FF00    //	纯绿色
#define 	LCD_RED         0xFF0000    //	纯红色
#define 	LCD_CYAN        0x00FFFF    //	蓝绿色
#define 	LCD_MAGENTA     0xFF00FF    //	紫红色
#define 	LCD_YELLOW      0xFFFF00    //	黄色
#define 	LCD_GREY        0x2C2C2C    //	灰色
												
#define 	LIGHT_BLUE      0x8080FF    //	亮蓝色
#define 	LIGHT_GREEN     0x80FF80    //	亮绿色
#define 	LIGHT_RED       0xFF8080    //	亮红色
#define 	LIGHT_CYAN      0x80FFFF    //	亮蓝绿色
#define 	LIGHT_MAGENTA   0xFF80FF    //	亮紫红色
#define 	LIGHT_YELLOW    0xFFFF80    //	亮黄色
#define 	LIGHT_GREY      0xA3A3A3    //	亮灰色
												
#define 	DARK_BLUE       0x000080    //	暗蓝色
#define 	DARK_GREEN      0x008000    //	暗绿色
#define 	DARK_RED        0x800000    //	暗红色
#define 	DARK_CYAN       0x008080    //	暗蓝绿色
#define 	DARK_MAGENTA    0x800080    //	暗紫红色
#define 	DARK_YELLOW     0x808000    //	暗黄色
#define 	DARK_GREY       0x404040    //	暗灰色
enum Direction
{
    DIRECTION_H,			
    DIRECTION_H_FLIP,	   
    DIRECTION_V,				
    DIRECTION_V_FLIP,
};
enum DispNumMode
{
// 设置变量显示时多余位补0还是补空格
    Fill_ZERO,		//填充0
    Fill_SPACE,		//填充空格
};
class mDisplay : public mDevice
{
public:
    mDisplay(const char* name, uint16_t width, uint16_t height, uint8_t xOffset, uint8_t yOffset, uint8_t bitPerPixel) :
    mDevice(name), pwidth(width),pheight(height),pXOffset(xOffset),pYOffset(yOffset),lcdBuff(nullptr),lcdBuffSize(0),
    color(0),backColor(0),bitPerPixel(bitPerPixel),bytesPerPixel(bitPerPixel/8),width(width),height(height),xOffset(xOffset),yOffset(yOffset)
    {
        lcdBuffSize = bytesPerPixel*1024;
        lcdBuff = new alignas(32) uint8_t[lcdBuffSize];
    }
    virtual ~mDisplay()
    {
        if(lcdBuff)
        {
            delete[] lcdBuff;
            lcdBuff = nullptr;
        }
    }
    void setColor(uint32_t color)
    {
        uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值
        if(bitPerPixel == 16)
        {
            Red_Value   = (uint16_t)((color&0x00F80000)>>8);   // 转换成 16位 的RGB565颜色
            Green_Value = (uint16_t)((color&0x0000FC00)>>5);
            Blue_Value  = (uint16_t)((color&0x000000F8)>>3);
        }
        else if(bitPerPixel == 24)
        {
            Red_Value   = (uint16_t)((color&0x00FF0000)>>16);
            Green_Value = (uint16_t)((color&0x0000FF00)>>8);
            Blue_Value  = (uint16_t)((color&0x000000FF));
        }
        this->color = (uint32_t)(Red_Value | Green_Value | Blue_Value);  // 将颜色写入全局LCD参数	
    }
    void setBackColor(uint32_t color)
    {
        uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值
        if(bitPerPixel == 16)
        {
            Red_Value   = (uint16_t)((color&0x00F80000)>>8);   // 转换成 16位 的RGB565颜色
            Green_Value = (uint16_t)((color&0x0000FC00)>>5);
            Blue_Value  = (uint16_t)((color&0x000000F8)>>3);
        }
        else if(bitPerPixel == 24)
        {
            Red_Value   = (uint16_t)((color&0x00FF0000)>>16);
            Green_Value = (uint16_t)((color&0x0000FF00)>>8);
            Blue_Value  = (uint16_t)((color&0x000000FF));
        }
        this->backColor = (uint32_t)(Red_Value | Green_Value | Blue_Value);  // 将颜色写入全局LCD参数
    }
    void setShowNumMode(DispNumMode mode)
    {
        this->showNumMode = mode;
    }
    void setDirection(Direction dir)
    {
    	this->direction = dir;

        if( direction == DIRECTION_H )   // 横屏显示
        {
            xOffset   = pYOffset;             // 设置控制器坐标偏移量
            yOffset   = pXOffset;   
            width      = pheight;		// 重新赋值长、宽
            height     = pwidth;		
        }
        else if( direction == DIRECTION_V )
        {
            xOffset   = pXOffset;              // 设置控制器坐标偏移量
            yOffset   = pYOffset;     
            width      = pwidth;		// 重新赋值长、宽
            height     = pheight;						
        }
        else if( direction == DIRECTION_H_FLIP )
        {
            xOffset   = pYOffset;              // 设置控制器坐标偏移量
            yOffset   = pXOffset;      
            width      = pheight;		 // 重新赋值长、宽
            height     = pwidth;				
        }
        else if( direction == DIRECTION_V_FLIP )
        {
            xOffset   = pXOffset;              // 设置控制器坐标偏移量
            yOffset   = pYOffset;     
            width      = pwidth;		// 重新赋值长、宽
            height     = pheight;				
        }
        internelSetDirection(direction);
    }
    void setFont(FontType type)
    {
        fontType = type;
        getFonts(fontType, &asciiFonts);
    }
    FontType getFontType(){return fontType;}
    void clear()
    {
        uint32_t size = lcdBuffSize;
        uint32_t count = 0;
        for(uint32_t i = 0; i < size; i += bytesPerPixel)
        {
            if(bytesPerPixel == 2)
            {
                // 正确设置16位颜色值
                lcdBuff[i] = (backColor >> 8) & 0xFF;  // 高字节
                lcdBuff[i + 1] = backColor & 0xFF;         // 低字节
            }
            else if(bytesPerPixel == 3)
            {
                // 设置24位颜色值
                lcdBuff[i] = (backColor >> 16) & 0xFF; // 红色分量
                lcdBuff[i + 1] = (backColor >> 8) & 0xFF;  // 绿色分量
                lcdBuff[i + 2] = backColor & 0xFF;         // 蓝色分量 
            }
        }
        setAddress(0,0,width-1,height-1);	// 设置坐标
        count = width*height*bytesPerPixel/lcdBuffSize;
        if(count == 0)
        {
            writeBuff(lcdBuff, size);
        }
        else
        {
            for(uint32_t i = 0; i < count; i++)
            {
                writeBuff(lcdBuff, lcdBuffSize);
            }
            count = width*height*bytesPerPixel%lcdBuffSize;
            if(count != 0)
            {
                writeBuff(lcdBuff, count);
            }
        }
    }
    void clearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
    {
        uint32_t size = width*height*bytesPerPixel;
        uint32_t count = 0;
        for(uint32_t i = 0; i < lcdBuffSize; i += bytesPerPixel)
        {
            if(bytesPerPixel == 2)
            {
                // 正确设置16位颜色值
                lcdBuff[i] = (backColor >> 8) & 0xFF;  // 高字节
                lcdBuff[i + 1] = backColor & 0xFF;         // 低字节
            }
            else if(bytesPerPixel == 3)
            {
                // 设置24位颜色值
                lcdBuff[i] = (backColor >> 16) & 0xFF; // 红色分量
                lcdBuff[i + 1] = (backColor >> 8) & 0xFF;  // 绿色分量
                lcdBuff[i + 2] = backColor & 0xFF;         // 蓝色分量
            }
        }
        setAddress( x, y, x+width-1, y+height-1);	// 设置坐标
        count = width*height*bytesPerPixel/lcdBuffSize;
        if(count == 0)
        {
            writeBuff(lcdBuff, size);
        }
        else
        {
            for(uint32_t i = 0; i < count; i++)
            {
                writeBuff(lcdBuff, lcdBuffSize);
            }
            count = width*height*bytesPerPixel%lcdBuffSize;
            if(count != 0)
            {
                writeBuff(lcdBuff, count);
            }
        }
    }
    void drawPoint(uint16_t x, uint16_t y, uint32_t color)
    {
        setAddress(x,y,x,y);	//	设置坐标 
        writeData16bit(color);
    }
    void showText(uint16_t x, uint16_t y, char *pText)
    {
        if(!asciiFonts)
        {
            return;
        }
        while(*pText != 0)	// 判断是否为空字符
        {
            if(*pText<=0x7F)	// 判断是否为ASCII码
            {
                showChar(x,y,*pText);	// 显示ASCII
                x+=asciiFonts->width;				// 水平坐标调到下一个字符处
                pText++;								// 字符串地址+1
            }
        }	
    }
    void showChar(uint16_t x, uint16_t y, uint8_t c)
    {
        if(!asciiFonts)
        {
            return;
        }
        uint16_t  index = 0, counter = 0 ,i = 0, w = 0;		// 计数变量
        uint8_t   disChar;		//存储字符的地址
     
        c = c - 32; 	// 计算ASCII字符的偏移
    
        for(index = 0; index < asciiFonts->sizes; index++)	
        {
            disChar = asciiFonts->pTable[c*asciiFonts->sizes + index]; //获取字符的模值
            for(counter = 0; counter < 8; counter++)
            { 
                if(disChar & 0x01)	
                {		
                    if(bytesPerPixel == 2)
                    {
                        lcdBuff[i] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+1] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
                    }
                    else if(bytesPerPixel == 3)
                    {
                        lcdBuff[i] =  (color>>16)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+1] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+2] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
                    }
                }
                else		
                {		
                    if(bytesPerPixel == 2)
                    {
                        lcdBuff[i] =  (backColor>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+1] =  backColor&0xff;			// 当前模值不为0时，使用画笔色绘点
                    }
                    else if(bytesPerPixel == 3)
                    {
                        lcdBuff[i] =  (backColor>>16)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+1] =  (backColor>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                        lcdBuff[i+2] =  backColor&0xff;			// 当前模值不为0时，使用画笔色绘点
                    }
                }
                disChar >>= 1;
                i+=bytesPerPixel;
                w++;
                if( w == asciiFonts->width ) // 如果写入的数据达到了字符宽度，则退出当前循环
                {								   // 进入下一字符的写入的绘制
                    w = 0;
                    break;
                }        
            }	
        }		
        setAddress( x, y, x+asciiFonts->width-1, y+asciiFonts->height-1);	   // 设置坐标	
        writeBuff(lcdBuff,asciiFonts->width*asciiFonts->height*bytesPerPixel);          // 写入显存
    }
    void showString(uint16_t x, uint16_t y, char *p)
    {
        if(!asciiFonts)
        {
            return;
        }
        while ((x < width) && (*p != 0))	//判断显示坐标是否超出显示区域并且字符是否为空字符
        {
            showChar( x,y,*p);
             x += asciiFonts->width; //显示下一个字符
             p++;	//取下一个字符地址
        }
    }
    void showNum(uint16_t x, uint16_t y, int32_t number, uint8_t len)
    {
        char Number_Buffer[15];				// 用于存储转换后的字符串

        if( showNumMode == Fill_ZERO)	// 多余位补0
        {
            sprintf( Number_Buffer , "%.*ld",len, number );	// 将 number 转换成字符串，便于显示		
        }
        else			// 多余位补空格
        {	
            sprintf( Number_Buffer , "%*ld",len, number );	// 将 number 转换成字符串，便于显示		
        }
        
        showString( x, y,(char *)Number_Buffer) ;  // 将转换得到的字符串显示出来
    }
    void showDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs)
    {
        char  Number_Buffer[20];				// 用于存储转换后的字符串
	
        if( showNumMode == Fill_ZERO)	// 多余位填充0模式
        {
            sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
        }
        else		// 多余位填充空格
        {
            sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
        }
        
        showString( x, y,(char *)Number_Buffer) ;	// 将转换得到的字符串显示出来
    }
    void drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
    {
        int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
        yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
        curpixel = 0;
    
        deltax = ABS(x2 - x1);        /* The difference between the x's */
        deltay = ABS(y2 - y1);        /* The difference between the y's */
        x = x1;                       /* Start x off at the first pixel */
        y = y1;                       /* Start y off at the first pixel */
    
        if (x2 >= x1)                 /* The x-values are increasing */
        {
         xinc1 = 1;
         xinc2 = 1;
        }
        else                          /* The x-values are decreasing */
        {
         xinc1 = -1;
         xinc2 = -1;
        }
    
        if (y2 >= y1)                 /* The y-values are increasing */
        {
         yinc1 = 1;
         yinc2 = 1;
        }
        else                          /* The y-values are decreasing */
        {
         yinc1 = -1;
         yinc2 = -1;
        }
    
        if (deltax >= deltay)         /* There is at least one x-value for every y-value */
        {
         xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
         yinc2 = 0;                  /* Don't change the y for every iteration */
         den = deltax;
         num = deltax / 2;
         numadd = deltay;
         numpixels = deltax;         /* There are more x-values than y-values */
        }
        else                          /* There is at least one y-value for every x-value */
        {
         xinc2 = 0;                  /* Don't change the x for every iteration */
         yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
         den = deltay;
         num = deltay / 2;
         numadd = deltax;
         numpixels = deltay;         /* There are more y-values than x-values */
        }
        for (curpixel = 0; curpixel <= numpixels; curpixel++)
        {
         drawPoint(x,y,color);             /* Draw the current pixel */
         num += numadd;              /* Increase the numerator by the top of the fraction */
         if (num >= den)             /* Check if numerator >= denominator */
         {
            num -= den;               /* Calculate the new numerator value */
            x += xinc1;               /* Change the x as appropriate */
            y += yinc1;               /* Change the y as appropriate */
         }
         x += xinc2;                 /* Change the x as appropriate */
         y += yinc2;                 /* Change the y as appropriate */
        } 
    }
    void drawLineV(uint16_t x, uint16_t y, uint16_t height)
    {
        uint16_t i, j ; // 计数变量

        for (i = 0, j = 0; i < height; i++, j+=bytesPerPixel)
        {
            if(bytesPerPixel == 2)
            {
                lcdBuff[j] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+1] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
            }
            else if(bytesPerPixel == 3)
            {
                lcdBuff[j] =  (color>>16)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+1] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+2] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
            }
        }   
        setAddress( x, y, x, y+height-1);	     // 设置坐标	
    
        writeBuff(lcdBuff,height*bytesPerPixel);          // 写入显存
    }
    void drawLineH(uint16_t x, uint16_t y, uint16_t width)
    {
        uint16_t i, j; // 计数变量

        for (i = 0, j = 0; i < width; i++, j+=bytesPerPixel)
        {
            if(bytesPerPixel == 2)
            {
                lcdBuff[j] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+1] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
            }
            else if(bytesPerPixel == 3)
            {
                lcdBuff[j] =  (color>>16)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+1] =  (color>>8)&0xff;	// 当前模值不为0时，使用画笔色绘点
                lcdBuff[j+2] =  color&0xff;			// 当前模值不为0时，使用画笔色绘点
            }
        }   
        setAddress( x, y, x+width-1, y);	     // 设置坐标	
    
        writeBuff(lcdBuff,width*bytesPerPixel);          // 写入显存
    }
    void drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
    {
        // 绘制水平线
        drawLineH( x,  y,  width);           
        drawLineH( x,  y+height-1,  width);

        // 绘制垂直线
        drawLineV( x,  y,  height);
        drawLineV( x+width-1,  y,  height);
    }
    void drawCircle(uint16_t x, uint16_t y, uint16_t r)
    {
        int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
        do {   
    
            drawPoint(x-Xadd,y+Yadd,color);
            drawPoint(x+Xadd,y+Yadd,color);
            drawPoint(x+Xadd,y-Yadd,color);
            drawPoint(x-Xadd,y-Yadd,color);
            
            e2 = err;
            if (e2 <= Yadd) {
                err += ++Yadd*2+1;
                if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
            }
            if (e2 > Xadd) err += ++Xadd*2+1;
        }
        while (Xadd <= 0);   
    }							
    void drawEllipse(int x, int y, int r1, int r2)
    {
        int Xadd = -r1, Yadd = 0, err = 2-2*r1, e2;
        float K = 0, rad1 = 0, rad2 = 0;
         
        rad1 = r1;
        rad2 = r2;
        
        if (r1 > r2)
        { 
          do {
            K = (float)(rad1/rad2);
               
              drawPoint(x-Xadd,y+(uint16_t)(Yadd/K),color);
              drawPoint(x+Xadd,y+(uint16_t)(Yadd/K),color);
              drawPoint(x+Xadd,y-(uint16_t)(Yadd/K),color);
              drawPoint(x-Xadd,y-(uint16_t)(Yadd/K),color);     
               
            e2 = err;
            if (e2 <= Yadd) {
              err += ++Yadd*2+1;
              if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
            }
            if (e2 > Xadd) err += ++Xadd*2+1;
          }
          while (Xadd <= 0);
        }
        else
        {
          Yadd = -r2; 
          Xadd = 0;
          do { 
            K = (float)(rad2/rad1);
      
              drawPoint(x-(uint16_t)(Xadd/K),y+Yadd,color);
              drawPoint(x+(uint16_t)(Xadd/K),y+Yadd,color);
              drawPoint(x+(uint16_t)(Xadd/K),y-Yadd,color);
              drawPoint(x-(uint16_t)(Xadd/K),y-Yadd,color);  
               
            e2 = err;
            if (e2 <= Xadd) {
              err += ++Xadd*3+1;
              if (-Yadd == Xadd && e2 <= Yadd) e2 = 0;
            }
            if (e2 > Yadd) err += ++Yadd*3+1;     
          }
          while (Yadd <= 0);
        }
    }										
    void fillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
    {
        uint32_t size = width * height *bytesPerPixel;
        for(uint32_t i = 0; i < size; i += bytesPerPixel)
        {
            if(bytesPerPixel == 2)
            {
                // 正确设置16位颜色值
                lcdBuff[i] = (color >> 8) & 0xFF;  // 高字节
                lcdBuff[i + 1] = color & 0xFF;         // 低字节
            }
            else if(bytesPerPixel == 3)
            {
                // 设置24位颜色值
                lcdBuff[i] = (color >> 16) & 0xFF; // 红色分量
                lcdBuff[i + 1] = (color >> 8) & 0xFF;  // 绿色分量
                lcdBuff[i + 2] = color & 0xFF;         // 蓝色分量
            }
        }
        setAddress(x, y, x + width - 1, y + height - 1);  // 设置坐标
        writeBuff(lcdBuff, width * height * bytesPerPixel);
    }
    void fillCircle(uint16_t x, uint16_t y, uint16_t r)
    {
        int32_t  D;    /* Decision Variable */ 
        uint32_t  CurX;/* Current X Value */
        uint32_t  CurY;/* Current Y Value */ 
        
        D = 3 - (r << 1);
        
        CurX = 0;
        CurY = r;
        
        while (CurX <= CurY)
        {
          if(CurY > 0) 
          { 
            drawLineV(x - CurX, y - CurY,2*CurY);
            drawLineV(x + CurX, y - CurY,2*CurY);
          }
          
          if(CurX > 0) 
          {
            drawLineV(x - CurY, y - CurX,2*CurX);
            drawLineV(x + CurY, y - CurX,2*CurX);
          }
          if (D < 0)
          { 
            D += (CurX << 2) + 6;
          }
          else
          {
            D += ((CurX - CurY) << 2) + 10;
            CurY--;
          }
          CurX++;
        }
        drawCircle(x, y, r);  
    }
    void drawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)
    {
        uint8_t   disChar;	         // 字模的值
        uint16_t  Xaddress = x;       // 水平坐标
        uint16_t  Yaddress = y;       // 垂直坐标  
        uint16_t  i=0,j=0,m=0;        // 计数变量
        uint16_t  BuffCount = 0;      // 缓冲区计数
        uint16_t  Buff_Height = 0;    // 缓冲区的行数
    
        // 因为缓冲区大小有限，需要分多次写入
        Buff_Height = (sizeof(lcdBuff)/2) / height;    // 计算缓冲区能够写入图片的多少行
    
        for(i = 0; i <height; i++)             // 循环按行写入
        {
            for(j = 0; j <(float)width/8; j++)  
            {
                disChar = *pImage;
    
                for(m = 0; m < 8; m++)
                { 
                    if(disChar & 0x01)	
                    {		
                        if(bytesPerPixel == 2)
                        {
                            // 正确设置16位颜色值
                            lcdBuff[BuffCount] = (color >> 8) & 0xFF;  // 高字节
                            lcdBuff[BuffCount + 1] = color & 0xFF;         // 低字节
                        }
                        else if(bytesPerPixel == 3)
                        {
                            // 设置24位颜色值
                            lcdBuff[BuffCount] = (color >> 16) & 0xFF; // 红色分量
                            lcdBuff[BuffCount + 1] = (color >> 8) & 0xFF;  // 绿色分量
                            lcdBuff[BuffCount + 2] = color & 0xFF;         // 蓝色分量
                        }
                    }
                    else		
                    {	
                        if(bytesPerPixel == 2)
                        {
                            // 正确设置16位颜色值
                            lcdBuff[BuffCount] = (backColor >> 8) & 0xFF;  // 高字节
                            lcdBuff[BuffCount + 1] = backColor & 0xFF;         // 低字节
                        }
                        else if(bytesPerPixel == 3)
                        {
                            // 设置24位颜色值
                            lcdBuff[BuffCount] = (backColor >> 16) & 0xFF; // 红色分量
                            lcdBuff[BuffCount + 1] = (backColor >> 8) & 0xFF;  // 绿色分量
                            lcdBuff[BuffCount + 2] = backColor & 0xFF;         // 蓝色分量
                        }	
                    }
                    disChar >>= 1;     // 模值移位
                    Xaddress++;        // 水平坐标自加
                    BuffCount+=bytesPerPixel;       // 缓冲区计数       
                    if( (Xaddress - x)==width ) // 如果水平坐标达到了字符宽度，则退出当前循环,进入下一行的绘制		
                    {											 
                        Xaddress = x;				                 
                        break;
                    }
                }	
                pImage++;			
            }
          if( BuffCount == Buff_Height*width  )  // 达到缓冲区所能容纳的最大行数时
          {
             BuffCount = 0; // 缓冲区计数清0
    
             setAddress( x, Yaddress , x+width-1, Yaddress+Buff_Height-1);	// 设置坐标	
             writeBuff(lcdBuff,width*Buff_Height*bytesPerPixel);          // 写入显存     
    
             Yaddress = Yaddress+Buff_Height;    // 计算行偏移，开始写入下一部分数据
          }     
          if( (i+1)== height ) // 到了最后一行时
          {
             setAddress( x, Yaddress , x+width-1,i+y);	   // 设置坐标	
             writeBuff(lcdBuff,width*(i+1+y-Yaddress)*bytesPerPixel);    // 写入显存     
          }
        }
    }
    void copyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint8_t *DataBuff)
    {
        setAddress(x,y,x+width-1,y+height-1);
        writeBuff(DataBuff,width*height*bytesPerPixel);
    }
    uint16_t getpWidth() const
    {
        return pwidth;
    }
    uint16_t getpHeight() const
    {
        return pheight;
    }
    uint16_t getWidth() const
    {
        return width;
    }
    uint16_t getHeight() const
    {
        return height;
    }
protected:
    virtual void setAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) = 0;
    virtual void writeCommand(uint8_t command) = 0;
    virtual void writeData8bit(uint8_t data) = 0;
    virtual void writeData16bit(uint16_t data) = 0;
    virtual void writeBuff(uint8_t *DataBuff, uint16_t DataSize) = 0;
    virtual void internelSetDirection(Direction dir) = 0;
protected:
    const uint16_t pwidth;
    const uint16_t pheight;
    const uint8_t  pXOffset;
    const uint8_t  pYOffset;
    uint8_t*     lcdBuff;    // Display缓冲区，16位宽（每个像素点占2字节）
    uint32_t     lcdBuffSize;
    uint32_t     color;  		    // display当前画笔颜色
    uint32_t     backColor;		    // 背景色
    uint8_t      bitPerPixel;
    uint8_t      bytesPerPixel;
    uint16_t     width;            // 屏幕像素长度
    uint16_t     height;           // 屏幕像素宽度	
    uint8_t      xOffset;          // X坐标偏移，用于设置屏幕控制器的显存写入方式
    uint8_t      yOffset;          // Y坐标偏移，用于设置屏幕控制器的显存写入方式
    Fonts        *asciiFonts = nullptr;	    // 英文字体，ASCII字符集
    FontType      fontType = FONT_TYPE_MAX;
    DispNumMode   showNumMode = Fill_SPACE;		// 数字显示模式
    Direction     direction = DIRECTION_H;		// 显示方向
    mSemaphore    _sem;
};

}