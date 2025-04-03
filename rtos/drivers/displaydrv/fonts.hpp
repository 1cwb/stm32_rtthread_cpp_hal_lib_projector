#pragma once
#include <stdint.h>

// 字体相关结构定义
struct Fonts
{    
	const uint8_t 		*pTable;  		//	字模数组地址
	uint16_t 			width; 		 	//	单个字符的字模宽度
	uint16_t 			height; 			//	单个字符的字模长度
	uint16_t 			sizes;	 		//	单个字符的字模数据个数
	uint16_t			tableRows;		// 该参数只有汉字字模用到，表示二维数组的行大小
};
enum FontType
{
    FONT_TYPE_32 = 0,
    FONT_TYPE_24,
    FONT_TYPE_20,
    FONT_TYPE_16,
    FONT_TYPE_12,
    FONT_TYPE_MAX,
};

void getFonts(FontType type, Fonts** asccifonts);
