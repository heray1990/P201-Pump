/*****************************************************************************/
/** \file lcd_D61593A.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2021-02-02  1.0    First version.
 **
 *****************************************************************************/

/******************************************************************************
 * Include files
 *****************************************************************************/
#include "lcd_D61593A.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 *****************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 *****************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 *****************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 *****************************************************************************/

/******************************************************************************
 * Local variable definitions ('const')                                       *
 *****************************************************************************/
const uint8_t u8Num1To11Table[] = {
                NUM_1_11_CHAR_0,    // 0
                NUM_1_11_CHAR_1,    // 1
                NUM_1_11_CHAR_2,    // 2
                NUM_1_11_CHAR_3,    // 3
                NUM_1_11_CHAR_4,    // 4
                NUM_1_11_CHAR_5,    // 5
                NUM_1_11_CHAR_6,    // 6
                NUM_1_11_CHAR_7,    // 7
                NUM_1_11_CHAR_8,    // 8
                NUM_1_11_CHAR_9     // 9
};

const uint16_t u16Num12To21Table[] = {
                NUM_12_21_CHAR_0,    // 0
                NUM_12_21_CHAR_1,    // 1
                NUM_12_21_CHAR_2,    // 2
                NUM_12_21_CHAR_3,    // 3
                NUM_12_21_CHAR_4,    // 4
                NUM_12_21_CHAR_5,    // 5
                NUM_12_21_CHAR_6,    // 6
                NUM_12_21_CHAR_7,    // 7
                NUM_12_21_CHAR_8,    // 8
                NUM_12_21_CHAR_9     // 9
};


/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 *****************************************************************************/

/******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 *****************************************************************************/
/******************************************************************************
 ** \brief 生成 T1 通道显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Val: T1 通道需要显示的数字
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 **
 ** \retval LCDRAM 的内容
 *****************************************************************************/
void Lcd_D61593A_GenRam_Channel(un_Ram_Data* punRamData, uint8_t u8Val, boolean_t bDisplay)
{
    punRamData[LCDRAM_INDEX_2].u32_dis &= MASK_LCDRAM2_CHANNEL;    // Clean T1 and 1 in LCDRAM2.

    if(TRUE == bDisplay || u8Val <= 9)
    {
        punRamData[LCDRAM_INDEX_2].u8_dis[1] = (u8Num1To11Table[u8Val] | 0x01);    // Set value for 1 and display T1
    }
}

/******************************************************************************
 ** \brief 生成浇水时长显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Val: T2 浇水时长需要显示的数字
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 **
 *****************************************************************************/
void Lcd_D61593A_GenRam_Watering_Time(un_Ram_Data* punRamData, uint8_t u8Val, boolean_t bDisplay)
{
    uint8_t u8Single, u8Ten, u8Hundred;

    u8Single = u8Val % 10;
	u8Ten = u8Val / 10 % 10;
	u8Hundred = u8Val / 100 % 10;

    punRamData[LCDRAM_INDEX_1].u32_dis &= MASK_LCDRAM1_WATER_TIME;    // Clean RAM of T2, T7, 8 and 9 in LCDRAM1.
    punRamData[LCDRAM_INDEX_2].u32_dis &= MASK_LCDRAM2_WATER_TIME;    // Clean RAM of 7 in LCDRAM2.

    if(TRUE == bDisplay)
    {
        // LCDRAM1
        punRamData[LCDRAM_INDEX_1].u8_dis[1] |= 0x01;    // Display T7.
        punRamData[LCDRAM_INDEX_1].u8_dis[2] = u8Num1To11Table[u8Single] | 0x01;    // Set value for 9 and display T2
        punRamData[LCDRAM_INDEX_1].u8_dis[3] |= u8Num1To11Table[u8Ten];    // Set value for 8

        // LCDRAM2
        punRamData[LCDRAM_INDEX_2].u8_dis[0] |= u8Num1To11Table[u8Hundred];    // Set value for 7
    }
}

/******************************************************************************
 ** \brief 生成 T8 组数显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Val: T8 组数需要显示的数字
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 **
 *****************************************************************************/
void Lcd_D61593A_GenRam_T8(un_Ram_Data* punRamData, uint8_t u8Val, boolean_t bDisplay)
{
    // Clean RAM of T8 and 12 in LCDRAM4 and LCDRAM5.
    punRamData[LCDRAM_INDEX_4].u32_dis &= MASK_LCDRAM4_T8;
    punRamData[LCDRAM_INDEX_5].u32_dis &= MASK_LCDRAM5_T8;

    if(TRUE == bDisplay || u8Val <= 9)
    {
        // LCDRAM4
        punRamData[LCDRAM_INDEX_4].u8_dis[3] |= 0x01;    // Display T8.
        punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[u8Val] & 0x00f0) << 4;    // Set value for 12 in LCDRAM4

        // LCDRAM5
        punRamData[LCDRAM_INDEX_5].u16_dis[0] |= (u16Num12To21Table[u8Val] & 0xf000) >> 12;    // Set value for 12 in LCDRAM5
    }
}

/******************************************************************************
 ** \brief 生成智能1显示的 LCDRAM 值 T9 ~ T15
 **
 ** \input punRamData: LCDRAM 的值
 **        enMode: 智能1的模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 **
 *****************************************************************************/
void Lcd_D61593A_GenRam_Smart1(un_Ram_Data* punRamData, en_smart_mode_t enMode, boolean_t bDisplay)
{
    // Clean RAM of T9 ~ T15 in LCDRAM3, LCDRAM4 and LCDRAM5.
    punRamData[LCDRAM_INDEX_3].u32_dis &= MASK_LCDRAM3_SMART1;
    punRamData[LCDRAM_INDEX_4].u32_dis &= MASK_LCDRAM4_SMART1;
    punRamData[LCDRAM_INDEX_5].u32_dis &= MASK_LCDRAM5_SMART1;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x80;    // Display T15.

        if(SmartModeDry == enMode)
        {
            punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x18;    // Display T13 and T14
        }
        else if(SmartModeMid == enMode)
        {
            punRamData[LCDRAM_INDEX_3].u8_dis[0] |= 0x01;    // Display T11
            punRamData[LCDRAM_INDEX_4].u8_dis[1] |= 0x01;    // Display T12
        }
        else
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x06;    // Display T9 and T10
        }
    }
}

/******************************************************************************
 ** \brief 生成智能2显示的 LCDRAM 值 P1 ~ P7
 **
 ** \input punRamData: LCDRAM 的值
 **        enMode: 智能2的模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 **
 *****************************************************************************/
void Lcd_D61593A_GenRam_Smart2(un_Ram_Data* punRamData, en_smart_mode_t enMode, boolean_t bDisplay)
{
    // Clean RAM of P1 ~ P7 in LCDRAM0 ~ LCDRAM3.
    punRamData[LCDRAM_INDEX_0].u32_dis &= MASK_LCDRAM0_SMART2;
    punRamData[LCDRAM_INDEX_1].u32_dis &= MASK_LCDRAM1_SMART2;
    punRamData[LCDRAM_INDEX_2].u32_dis &= MASK_LCDRAM2_SMART2;
    punRamData[LCDRAM_INDEX_3].u32_dis &= MASK_LCDRAM3_SMART2;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x01;    // Display P1.

        if(SmartModeDry == enMode)
        {
            punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x06;    // Display P2 and P3
        }
        else if(SmartModeMid == enMode)
        {
            punRamData[LCDRAM_INDEX_1].u8_dis[3] |= 0x01;    // Display P5
            punRamData[LCDRAM_INDEX_2].u8_dis[0] |= 0x01;    // Display P4
        }
        else
        {
            punRamData[LCDRAM_INDEX_0].u8_dis[1] |= 0x01;    // Display P6
            punRamData[LCDRAM_INDEX_0].u8_dis[2] |= 0x01;    // Display P6
        }
    }
}

