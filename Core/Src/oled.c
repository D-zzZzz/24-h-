#include "oled.h"
#include "oled_bus.h"
#include "oledfont.h"          //头文件
#include <string.h>

#define OLED_WIDTH 128U
#define OLED_PAGES 8U

/* Foreground-only display staging and an ACK-confirmed copy of display RAM. */
static uint8_t oled_stage[OLED_WIDTH];
static uint8_t oled_shadow[OLED_PAGES][OLED_WIDTH];
static uint8_t oled_valid_pages;

static const uint8_t CMD_Data[] = {
    0xAE,             /* display off */
    0xD5, 0x80,       /* clock divide ratio */
    0xA8, 0x3F,       /* multiplex ratio: 1/64 */
    0xD3, 0x00,       /* display offset */
    0x40,             /* display start line */
    0x8D, 0x14,       /* charge pump on */
    0x20, 0x02,       /* page addressing mode */
    0xA1,             /* segment remap */
    0xC8,             /* COM scan direction */
    0xDA, 0x12,       /* COM pin configuration */
    0x81, 0xFF,       /* contrast */
    0xD9, 0xF1,       /* pre-charge period */
    0xDB, 0x30,       /* VCOMH deselect level */
    0xA4,             /* use display RAM */
    0xA6,             /* normal display */
    0xAF              /* display on */
};      //初始化命令


static HAL_StatusTypeDef OLED_SetPosition(uint8_t x, uint8_t page)
{
    uint8_t commands[3];
    commands[0] = (uint8_t)(0xb0U + page);
    commands[1] = (uint8_t)(0x10U | (x >> 4));
    commands[2] = (uint8_t)(x & 0x0fU);
    return OLED_I2C_Write(0x00U, commands, sizeof(commands));
}

static HAL_StatusTypeDef OLED_WriteSpan(uint8_t x, uint8_t page,
                                      const uint8_t *data, uint16_t size)
{
    uint16_t first = 0U, last;
    uint8_t page_mask;

    if (x >= OLED_WIDTH || page >= OLED_PAGES || data == NULL || size == 0U)
        return HAL_ERROR;
    if (size > OLED_WIDTH - x)
        size = OLED_WIDTH - x;
    last = size;
    page_mask = (uint8_t)(1U << page);
    if ((oled_valid_pages & page_mask) != 0U)
    {
        while (first < size && oled_shadow[page][x + first] == data[first])
            first++;
        if (first == size)
            return HAL_OK;
        while (last > first && oled_shadow[page][x + last - 1U] == data[last - 1U])
            last--;
    }
    if (OLED_SetPosition((uint8_t)(x + first), page) != HAL_OK ||
        OLED_I2C_Write(0x40U, data + first, last - first) != HAL_OK)
    {
        /* A partially transferred page must be resent, not treated as current. */
        oled_valid_pages &= (uint8_t)~page_mask;
        return HAL_ERROR;
    }
    memcpy(oled_shadow[page] + x, data, size);
    if (x == 0U && size == OLED_WIDTH)
        oled_valid_pages |= page_mask;
    return HAL_OK;
}

void WriteCmd(void)
{
    (void)OLED_I2C_Write(0x00U, CMD_Data, sizeof(CMD_Data));
    oled_valid_pages = 0U;
}
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd)
{
    /* Raw commands can change addressing or the interpretation of display RAM. */
    oled_valid_pages = 0U;
    OLED_I2C_Write(0x00U, &cmd, 1U);
}
//向设备写数据
void OLED_WR_DATA(uint8_t data)
{
    oled_valid_pages = 0U;
    OLED_I2C_Write(0x40U, &data, 1U);
}
//初始化oled屏幕
void OLED_Init(void)
{
    HAL_Delay(200);

    WriteCmd();
    HAL_Delay(100);
}
//清屏
void OLED_Clear(void)
{
    uint8_t page;
    memset(oled_stage, 0, sizeof(oled_stage));
    oled_valid_pages = 0U;
    for (page = 0U; page < OLED_PAGES; page++)
        (void)OLED_WriteSpan(0U, page, oled_stage, OLED_WIDTH);
}
//开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_CMD(0X8D);  //SET DCDC命令
    OLED_WR_CMD(0X14);  //DCDC ON
    OLED_WR_CMD(0XAF);  //DISPLAY ON
}
//关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_CMD(0X8D);  //SET DCDC命令
    OLED_WR_CMD(0X10);  //DCDC OFF
    OLED_WR_CMD(0XAE);  //DISPLAY OFF
}
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    if (x < OLED_WIDTH && y < OLED_PAGES)
        (void)OLED_SetPosition(x, y);
}

void OLED_On(void)
{
    uint8_t page;
    memset(oled_stage, 1, sizeof(oled_stage));
    for (page = 0U; page < OLED_PAGES; page++)
        (void)OLED_WriteSpan(0U, page, oled_stage, OLED_WIDTH);
}
unsigned int oled_pow(uint8_t m,uint8_t n)
{
    unsigned int result=1;
    while(n--)result*=m;
    return result;
}
//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    uint8_t char_width = (size2 == 16U) ? 8U : 6U;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+char_width*t,y,' ',size2);
                continue;
            } else enshow=1;

        }
        OLED_ShowChar(x+char_width*t,y,temp+'0',size2);
    }
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:页地址 0~7，每页高 8 像素
//mode:0,反白显示;1,正常显示
//size:16 为 8x16 字体，8 为 6x8 字体
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{
    uint8_t text[2];
    if (chr < ' ' || chr > '~')
        return;
    text[0] = chr;
    text[1] = 0U;
    OLED_ShowString(x, y, text, Char_Size);
}

//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
    uint8_t char_width = (Char_Size == 16U) ? 8U : 6U;
    uint8_t char_pages = (Char_Size == 16U) ? 2U : 1U;
    uint8_t count, i, font_page;

    if (chr == NULL)
        return;
    while (*chr != 0U)
    {
        if (x > OLED_WIDTH - char_width)
        {
            x = 0U;
            y += char_pages;
        }
        if (y > OLED_PAGES - char_pages)
            break;
        count = 0U;
        while (chr[count] != 0U && x + (count + 1U) * char_width <= OLED_WIDTH)
            count++;
        for (font_page = 0U; font_page < char_pages; font_page++)
        {
            for (i = 0U; i < count; i++)
            {
                uint8_t c = (chr[i] >= ' ' && chr[i] <= '~') ? chr[i] - ' ' : 0U;
                const uint8_t *glyph = (char_pages == 2U) ?
                    F8X16 + c * 16U + font_page * 8U : F6x8[c];
                memcpy(oled_stage + i * char_width, glyph, char_width);
            }
            (void)OLED_WriteSpan(x, y + font_page, oled_stage, count * char_width);
        }
        chr += count;
        x = (uint8_t)(x + count * char_width);
    }
}
//显示汉字
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{
    if (x > OLED_WIDTH - 16U || y > OLED_PAGES - 2U ||
        no >= sizeof(Hzk) / (2U * sizeof(Hzk[0])))
        return;
    (void)OLED_WriteSpan(x, y, Hzk[2U * no], 16U);
    (void)OLED_WriteSpan(x, y + 1U, Hzk[2U * no + 1U], 16U);
}
