/******************************************************************************/
/** \file lcd_D61593A.h
 **
 ** Header file for lcd D6A593A Converter functions
 ** @link HDMCU for LCD mode 0 @endlink
 **
 **   - 2021-02-03      First Version
 **
 ******************************************************************************/

#ifndef __LCD_D61593A_H__
#define __LCD_D61593A_H__
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include <stdint.h>
#include "ddl.h"


/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/
/******************************************************************************
 ** 字符0~9和对应码段的关系，真值表中数字码段有两种情况
 ******************************************************************************
#define NUM_1_11_SEG_A 0x02
#define NUM_1_11_SEG_B 0x08
#define NUM_1_11_SEG_C 0x40
#define NUM_1_11_SEG_D 0x80
#define NUM_1_11_SEG_E 0x20
#define NUM_1_11_SEG_F 0x04
#define NUM_1_11_SEG_G 0x10

#define NUM_12_21_SEG_A 0x8000
#define NUM_12_21_SEG_B 0x4000
#define NUM_12_21_SEG_C 0x2000
#define NUM_12_21_SEG_D 0x1000
#define NUM_12_21_SEG_E 0x0020
#define NUM_12_21_SEG_F 0x0080
#define NUM_12_21_SEG_G 0x0040

0 = SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F
1 = SEG_B+SEG_C
2 = SEG_A+SEG_B+SEG_D+SEG_E+SEG_G
3 = SEG_A+SEG_B+SEG_C+SEG_D+SEG_G
4 = SEG_B+SEG_C+SEG_F+SEG_G
5 = SEG_A+SEG_C+SEG_D+SEG_F+SEG_G
6 = SEG_A+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G
7 = SEG_A+SEG_B+SEG_C
8 = SEG_A+SEG_B+SEG_C+SEG_D+SEG_E+SEG_F+SEG_G
9 = SEG_A+SEG_B+SEG_C+SEG_D+SEG_F+SEG_G
*****************************************************************************/

#define NUM_1_11_CHAR_0    0xee
#define NUM_1_11_CHAR_1    0x48
#define NUM_1_11_CHAR_2    0xba
#define NUM_1_11_CHAR_3    0xda
#define NUM_1_11_CHAR_4    0x5c
#define NUM_1_11_CHAR_5    0xd6
#define NUM_1_11_CHAR_6    0xf6
#define NUM_1_11_CHAR_7    0x4a
#define NUM_1_11_CHAR_8    0xfe
#define NUM_1_11_CHAR_9    0xde

#define NUM_12_21_CHAR_0    0xf0a0
#define NUM_12_21_CHAR_1    0x6000
#define NUM_12_21_CHAR_2    0xd060
#define NUM_12_21_CHAR_3    0xf040
#define NUM_12_21_CHAR_4    0x60c0
#define NUM_12_21_CHAR_5    0xb0c0
#define NUM_12_21_CHAR_6    0xb0e0
#define NUM_12_21_CHAR_7    0xe000
#define NUM_12_21_CHAR_8    0xf0e0
#define NUM_12_21_CHAR_9    0xf0c0

#define CHAR_T1    0x01

#define LCDRAM_INDEX_0    0
#define LCDRAM_INDEX_1    1
#define LCDRAM_INDEX_2    2
#define LCDRAM_INDEX_3    3
#define LCDRAM_INDEX_4    4
#define LCDRAM_INDEX_5    5


/******************************************************************************
 ** Global type definitions
 *****************************************************************************/
/******************************************************************************
 ** \brief LCDRAM
 *****************************************************************************/
typedef union{
    uint8_t u8_dis[4];
    uint16_t u16_dis[2];
    uint32_t u32_dis;
}un_Ram_Data;

/******************************************************************************
 ** \brief LCDRAM 返回结构体
 *****************************************************************************/
typedef struct stc_lcd_d61593a_lcdram
{
    uint8_t u8LcdRamIdx;    //LCDRAM index
    un_Ram_Data unRamData;    //写入到 LCDRAM 的值
}stc_lcd_d61593a_lcdram_t;


/******************************************************************************
 ** \brief 智能模式选择
 *****************************************************************************/
typedef enum
{
    SmartModeDry = 0,    //干
    SmartModeMid = 1,    //适中
    SmartModeWet = 2,    //湿
}en_smart_mode_t;


/******************************************************************************
** Global function prototypes (definition in C source)
******************************************************************************/
extern stc_lcd_d61593a_lcdram_t LCD_D61593A_GenRam_Channel(uint8_t u8Val, boolean_t bDisplay);


//@} // LCDGroup

#ifdef __cplusplus
#endif

#endif /* __LCD_D61593A_H__ */
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
