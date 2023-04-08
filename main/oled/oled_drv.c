
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "oled_drv.h"
#include "font.h"

/*********************define gpio1->DC,gpio2->RST************************/

#define USR_SPI_CHANNAL (1)

#define SIZE 16
#define XLevelL 0x00
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF
#define X_WIDTH 128
#define Y_WIDTH 64

#define OLED_CMD 0  // 写命令
#define OLED_DATA 1 // 写数据

/* OLED屏幕硬件相关宏定义 */
#define LCD_HOST SPI2_HOST

#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK 36
#define PIN_NUM_CS 39
#define PIN_NUM_DC 37
#define PIN_NUM_RST 38

#define OLED_RST_Clr() gpio_set_level(PIN_NUM_RST, 0);
#define OLED_RST_Set() gpio_set_level(PIN_NUM_RST, 1);


spi_device_handle_t g_spi;

// /****************************************************************************
// *spi_usr_type = 0 is analog spi, spi_usr_type = 1 is hardware spi.
// ****************************************************************************/
// uint8_t spi_usr_type = 0;

// /***************************************************************************/
// //是否清除首页显示内容标志位0=未清理，1=已清理
// bool CLRorNOT = 0;

// //spi device select
// void spi_oled_cs(bool CS)
// {
//     //if (!spi_usr_type)
//     {
//         if (CS)
//             Ql_GPIO_SetLevel(PINNAME_SPI_CS,PINLEVEL_HIGH);
//         else
//             Ql_GPIO_SetLevel(PINNAME_SPI_CS,PINLEVEL_LOW);
//     }
// }

static void spi_write_data(uint8_t *data, int len)
{
    esp_err_t ret = 0;

    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void *)1;
    ret = spi_device_transmit(g_spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

// data/cmd选择
static void lcd_spi_pre_trans_callback_dc(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

// oled data/command select
// 0,表示命令;1,表示数据;
// void spi_oled_dc(bool DC)
// {
//     // if (!spi_usr_type)
//     {
//         if (DC) // set gpio1 as DC
//             Ql_GPIO_SetLevel(PINNAME_GPIO1, PINLEVEL_HIGH);
//         else
//             Ql_GPIO_SetLevel(PINNAME_GPIO1, PINLEVEL_LOW);
//     }
// }

// write a byte to oled
// dat:要写入的数据/命令
// cmd:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(uint8_t dat, bool DC)
{
    esp_err_t ret = 0;
    // spi_oled_dc(DC);

    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &dat;
    t.user = (void *)DC;
    ret = spi_device_transmit(g_spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f) | 0x01, OLED_CMD);
}

// 开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
// 关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
// 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    uint8_t i, n, a[128];
    for (n = 0; n < 128; n++)
    {
        a[n] = 0;
    }
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置显示位置—列高地址
        // for(n=0;n<128;n++)
        //     OLED_WR_Byte(0,OLED_DATA);
        spi_write_data(a, 128);
    } // 更新显示
}

// 在指定位置显示一个字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
// size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    if (x > Max_Column - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (SIZE == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    /*
    else {
        OLED_Set_Pos(x,y+1);
        for(i=0;i<6;i++)
        OLED_WR_Byte(F6x8[c][i],OLED_DATA);
    }
    */
}

// 在指定位置显示一个6x8字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
void OLED_ShowChar8(uint8_t x, uint8_t y, uint8_t chr)
{
    uint8_t c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    OLED_Set_Pos(x, y + 1);
    for (i = 0; i < 6; i++)
        OLED_WR_Byte(F6x8[c][i], OLED_DATA);
}

// 在指定位置显示一个16*32字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
void OLED_ShowChar32(uint8_t x, uint8_t y, uint8_t p)
{
    uint8_t i = 0, j = 0;

    OLED_Set_Pos(x, y);
    for (j = 0; j < 4; j++)
    {
        OLED_Set_Pos(x, y + j);
        for (i = 0; i < 16; i++)
        {
            OLED_WR_Byte(F16X32[p * 64 + i + j * 16], OLED_DATA);
        }
    }
}

// 显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, char *chr)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j]);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    
    for (unsigned int y = y0; y < y1 + 1; y++)
    {
        OLED_Set_Pos(x0, y);
        // unsigned int j = 0;
        // for(unsigned int x=x0;x<x1;x++) {
        //     OLED_WR_Byte(BMP[j++],OLED_DATA);
        // }
        spi_write_data(&BMP[y * (x1 - x0 + 1)], (x1 - x0 + 1));
    }
    // printf("draw complete\r\n");
}

static void OLED_1in51_InitReg(void)
{
    OLED_WR_Byte(0xAE, OLED_CMD);//--turn off oled panel

    OLED_WR_Byte(0x00, OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10, OLED_CMD);//---set high column address

	OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);

    OLED_WR_Byte(0xFF, OLED_CMD);

    OLED_WR_Byte(0xA6, OLED_CMD);

    OLED_WR_Byte(0xA8, OLED_CMD); 
    OLED_WR_Byte(0x3F, OLED_CMD);

    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);

    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);

    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0x22, OLED_CMD);

    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);

    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
}

static void OLED_0in96_InitReg(void)
{
    OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)

    OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    // OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xFF, OLED_CMD);

    OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8, OLED_CMD); // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00, OLED_CMD); //-not offset

    OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    // OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xF0, OLED_CMD);

    OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
    OLED_WR_Byte(0x40, OLED_CMD); // Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
}

static void OLED_Reset(void)
{
    OLED_RST_Set();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    OLED_RST_Clr();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    OLED_RST_Set();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

// 初始化SSD1315
void OLED_Init(void)
{
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {0};
    buscfg.miso_io_num = -1;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0; // dma模式？
    
    spi_device_interface_config_t devcfg = {0};
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.clock_speed_hz = 10 * 1000 * 1000;      // Clock out at 10 MHz
    devcfg.mode = 0;                               // SPI mode 0
    devcfg.spics_io_num = PIN_NUM_CS;              // CS pin
    devcfg.queue_size = 7;                         // We want to be able to queue 7 transactions at a time
    devcfg.pre_cb = lcd_spi_pre_trans_callback_dc; // Specify pre-transfer callback to handle D/C line

    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &g_spi);
    ESP_ERROR_CHECK(ret);

    // Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);


    OLED_Reset();
    OLED_0in96_InitReg();
    // OLED_1in51_InitReg();

    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel
    

    OLED_Clear();
    // OLED_Set_Pos(0, 0);
}
