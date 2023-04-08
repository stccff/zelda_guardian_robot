#ifndef _OLED_DRV_H_
#define _OLED_DRV_H_

#include <stdio.h>
#include <stdlib.h>


extern void OLED_WR_Byte(uint8_t dat, bool DC);
extern void OLED_Set_Pos(unsigned char x, unsigned char y);

// 开启OLED显示
extern void OLED_Display_On(void);
// 关闭OLED显示
extern void OLED_Display_Off(void);
// 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
extern void OLED_Clear(void);

// 在指定位置显示一个字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
// size:选择字体 16/12
extern void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);

// 在指定位置显示一个6x8字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
extern void OLED_ShowChar8(uint8_t x, uint8_t y, uint8_t chr);

// 显示一个字符号串
extern void OLED_ShowString(uint8_t x, uint8_t y, char *chr);

/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
extern void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);


// 初始化oled
extern void OLED_Init(void);

#endif
