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
                NUM_1_11_CHAR_0,        // 0
                NUM_1_11_CHAR_1,        // 1
                NUM_1_11_CHAR_2,        // 2
                NUM_1_11_CHAR_3,        // 3
                NUM_1_11_CHAR_4,        // 4
                NUM_1_11_CHAR_5,        // 5
                NUM_1_11_CHAR_6,        // 6
                NUM_1_11_CHAR_7,        // 7
                NUM_1_11_CHAR_8,        // 8
                NUM_1_11_CHAR_9,        // 9
                NUM_1_11_CHAR_ERROR     // Error
};

const uint16_t u16Num12To21Table[] = {
                NUM_12_21_CHAR_0,       // 0
                NUM_12_21_CHAR_1,       // 1
                NUM_12_21_CHAR_2,       // 2
                NUM_12_21_CHAR_3,       // 3
                NUM_12_21_CHAR_4,       // 4
                NUM_12_21_CHAR_5,       // 5
                NUM_12_21_CHAR_6,       // 6
                NUM_12_21_CHAR_7,       // 7
                NUM_12_21_CHAR_8,       // 8
                NUM_12_21_CHAR_9,       // 9
                NUM_12_21_CHAR_ERROR    // Error
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

    if(TRUE == bDisplay)
    {
        if(u8Val >= 0 && u8Val <= 9)
        {
            punRamData[LCDRAM_INDEX_2].u8_dis[1] = (u8Num1To11Table[u8Val] | 0x01);    // Set value for 1 and display T1
        }
        else
        {
            punRamData[LCDRAM_INDEX_2].u8_dis[1] = (u8Num1To11Table[10] | 0x01);    // Display "E"(Error)
        }
    }
}

/******************************************************************************
 ** \brief 生成浇水时长显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Val: T2 浇水时长需要显示的数字
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
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
        punRamData[LCDRAM_INDEX_1].u8_dis[1] |= 0x01;    // Display T7.
        punRamData[LCDRAM_INDEX_1].u8_dis[2] = u8Num1To11Table[u8Single] | 0x01;    // Set value for 9 and display T2

        if(u8Hundred > 0 || (u8Hundred == 0 && u8Ten > 0))
        {
            punRamData[LCDRAM_INDEX_1].u8_dis[3] |= u8Num1To11Table[u8Ten];    // Set value for 8
        }

        if(u8Hundred > 0)
        {
            punRamData[LCDRAM_INDEX_2].u8_dis[0] |= u8Num1To11Table[u8Hundred];    // Set value for 7
        }
    }
}

/******************************************************************************
 ** \brief 生成 T8 组数显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Val: T8 组数需要显示的数字
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Sets(un_Ram_Data* punRamData, uint8_t u8Val, boolean_t bDisplay)
{
    // Clean RAM of T8 and 12 in LCDRAM4 and LCDRAM5.
    punRamData[LCDRAM_INDEX_4].u32_dis &= MASK_LCDRAM4_T8;
    punRamData[LCDRAM_INDEX_5].u32_dis &= MASK_LCDRAM5_T8;

    if(TRUE == bDisplay)
    {
        if(u8Val >= 0 && u8Val <= 9)
        {
            punRamData[LCDRAM_INDEX_4].u8_dis[3] |= 0x01;    // Display T8.
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[u8Val] & 0x00f0) << 4;    // Set value for 12 in LCDRAM4
            punRamData[LCDRAM_INDEX_5].u16_dis[0] |= (u16Num12To21Table[u8Val] & 0xf000) >> 12;    // Set value for 12 in LCDRAM5
        }
    }
}

/******************************************************************************
 ** \brief 生成智能1显示的 LCDRAM 值 T9 ~ T15
 **
 ** \input punRamData: LCDRAM 的值
 **        enMode: 智能1的模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
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
 **        enSmartMode: 智能2的模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Smart2(un_Ram_Data* punRamData, en_smart_mode_t enSmartMode, boolean_t bDisplay)
{
    // Clean RAM of P1 ~ P7 in LCDRAM0 ~ LCDRAM3.
    punRamData[LCDRAM_INDEX_0].u32_dis &= MASK_LCDRAM0_SMART2;
    punRamData[LCDRAM_INDEX_1].u32_dis &= MASK_LCDRAM1_SMART2;
    punRamData[LCDRAM_INDEX_2].u32_dis &= MASK_LCDRAM2_SMART2;
    punRamData[LCDRAM_INDEX_3].u32_dis &= MASK_LCDRAM3_SMART2;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x01;    // Display P1.

        if(SmartModeDry == enSmartMode)
        {
            punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x06;    // Display P2 and P3
        }
        else if(SmartModeMid == enSmartMode)
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

/******************************************************************************
 ** \brief 生成启动时间显示的 LCDRAM 值
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Hour: 启动时间需要显示的小时
 **        u8Minute: 启动时间需要显示的分钟
 **        enWorkingMode: 目前选定的工作模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Starting_Time(
                                un_Ram_Data* punRamData,
                                uint8_t u8Hour,
                                uint8_t u8Minute,
                                en_working_mode_t enWorkingMode,
                                boolean_t bDisplay)
{
    uint8_t u8HourSingle, u8HourTen, u8MinuteSingle, u8MinuteTen;

    u8HourSingle = u8Hour % 10;
    u8HourTen = u8Hour / 10 % 10;
    u8MinuteSingle = u8Minute % 10;
    u8MinuteTen = u8Minute / 10 % 10;

    punRamData[LCDRAM_INDEX_0].u32_dis &= MASK_LCDRAM0_START_TIME;    // Clean RAM of 5, 6 and COL3 in LCDRAM0.
    punRamData[LCDRAM_INDEX_1].u32_dis &= MASK_LCDRAM1_START_TIME;    // Clean RAM of 3, 4 and T3 in LCDRAM1.

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_0].u8_dis[3] |= 0x01;    // Display COL3.
        punRamData[LCDRAM_INDEX_1].u8_dis[0] |= 0x01;    // Display T3.

        if(ModeAutomatic == enWorkingMode)
        {
            if(u8Minute >= 0 && u8Minute <= 59)
            {
                punRamData[LCDRAM_INDEX_0].u8_dis[2] |= u8Num1To11Table[u8MinuteSingle];    // 6
                punRamData[LCDRAM_INDEX_0].u8_dis[3] |= u8Num1To11Table[u8MinuteTen];    // 5
            }
            else
            {
                punRamData[LCDRAM_INDEX_0].u8_dis[2] |= u8Num1To11Table[10];    // Display "E"(Error) in 6
                punRamData[LCDRAM_INDEX_0].u8_dis[3] |= u8Num1To11Table[10];    // Display "E"(Error) in 5
            }

            if(u8Hour >= 0 && u8Hour <= 23)
            {
                punRamData[LCDRAM_INDEX_1].u8_dis[0] |= u8Num1To11Table[u8HourSingle];    // 4
                if(u8HourTen > 0)
                {
                    punRamData[LCDRAM_INDEX_1].u8_dis[1] |= u8Num1To11Table[u8HourTen];    // 3
                }
            }
            else
            {
                punRamData[LCDRAM_INDEX_1].u8_dis[0] = u8Num1To11Table[10];    // Display "E"(Error) in 4
                punRamData[LCDRAM_INDEX_1].u8_dis[1] = u8Num1To11Table[10];    // Display "E"(Error) in 3
            }
        }
        else
        {
            // Display "--:--"
            punRamData[LCDRAM_INDEX_0].u8_dis[2] |= 0x10;
            punRamData[LCDRAM_INDEX_0].u8_dis[3] |= 0x10;
            punRamData[LCDRAM_INDEX_1].u8_dis[0] |= 0x10;
            punRamData[LCDRAM_INDEX_1].u8_dis[1] |= 0x10;
        }
    }
}

/******************************************************************************
 ** \brief 生成间隔天数显示的 LCDRAM 值 T4, 10 和 11
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Day: 间隔天数
 **        enWorkingMode: 目前选定的工作模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Days_Apart(
                                un_Ram_Data* punRamData,
                                uint8_t u8Day,
                                en_working_mode_t enWorkingMode,
                                boolean_t bDisplay)
{
    uint8_t u8DaySingle, u8DayTen;

    u8DaySingle = u8Day % 10;
    u8DayTen = u8Day / 10 % 10;

    punRamData[LCDRAM_INDEX_0].u32_dis &= MASK_LCDRAM0_DAYS_APART;    // Clean RAM of 10, 11 and T4 in LCDRAM0.

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_0].u8_dis[0] |= 0x01;    // Display T4.

        if(ModeAutomatic == enWorkingMode)
        {
            if(u8Day >= 0 && u8Day <= 99)
            {
                punRamData[LCDRAM_INDEX_0].u8_dis[0] |= u8Num1To11Table[u8DaySingle];    // 11

                if(u8DayTen > 0)
                {
                    punRamData[LCDRAM_INDEX_0].u8_dis[1] |= u8Num1To11Table[u8DayTen];    // 10
                }
            }
            else
            {
                punRamData[LCDRAM_INDEX_0].u8_dis[0] |= u8Num1To11Table[10];    // Display "E"(Error) in 11
                punRamData[LCDRAM_INDEX_0].u8_dis[1] |= u8Num1To11Table[10];    // Display "E"(Error) in 10
            }
        }
        else
        {
            // Display "--"
            punRamData[LCDRAM_INDEX_0].u8_dis[0] |= 0x10;
            punRamData[LCDRAM_INDEX_0].u8_dis[1] |= 0x10;
        }
    }
}


/******************************************************************************
 ** \brief 生成工作模式显示的 LCDRAM 值 T16 和 T17
 **
 ** \input punRamData: LCDRAM 的值
 **        enSmartMode: 智能2的模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_WorkingMode(
                                un_Ram_Data* punRamData,
                                en_working_mode_t enWrokingMode,
                                boolean_t bDisplay)
{
    // Clean RAM of T16 and T17 in LCDRAM5.
    punRamData[LCDRAM_INDEX_5].u8_dis[1] &= 0xef;    // T17
    punRamData[LCDRAM_INDEX_5].u8_dis[2] &= 0xdf;    // T16

    if(TRUE == bDisplay)
    {
        if(ModeAutomatic == enWrokingMode)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x10;
        }
        else
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x20;
        }
    }
}

/******************************************************************************
 ** \brief 生成"暂停"显示的 LCDRAM 值 T18
 **
 ** \input punRamData: LCDRAM 的值
 **        bStop - TRUE: 显示"暂停"
 **                FALSE：不显示"暂停"
 *****************************************************************************/
void Lcd_D61593A_GenRam_Stop(un_Ram_Data* punRamData, boolean_t bStop)
{
    // Clean RAM of T18 in LCDRAM5.
    punRamData[LCDRAM_INDEX_5].u8_dis[2] &= 0xef;

    if(TRUE == bStop)
    {
        punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x10;
    }
}

/******************************************************************************
 ** \brief 生成"锁"图标显示状态的 LCDRAM 值 X1~X3
 **
 ** \input punRamData: LCDRAM 的值
 **        enLockStatus: 锁或者解锁
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Lock_Icon(
                un_Ram_Data* punRamData,
                en_lock_status_t enLockStatus,
                boolean_t bDisplay)
{
    // Clean RAM of X1~X3 in LCDRAM5.
    punRamData[LCDRAM_INDEX_5].u8_dis[1] &= 0x9f;
    punRamData[LCDRAM_INDEX_5].u8_dis[2] &= 0xbf;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x20;

        if(Lock == enLockStatus)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x40;
        }
        else
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x40;
        }
    }
}

/******************************************************************************
 ** \brief 生成WiFi信号强度显示的 LCDRAM 值 S3 S4
 **
 ** \input punRamData: LCDRAM 的值
 **        enWifiSignal: WiFi 信号强弱
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Wifi_Icon(
                un_Ram_Data* punRamData,
                en_wifi_signal_strength_t enWifiSignal,
                boolean_t bDisplay)
{
    // Clean RAM of S3 and S4 in LCDRAM5.
    punRamData[LCDRAM_INDEX_5].u8_dis[1] &= 0x7f;
    punRamData[LCDRAM_INDEX_5].u8_dis[2] &= 0x7f;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x80;

        if(WifiSignalStrong == enWifiSignal)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x80;
        }
    }
}

/******************************************************************************
 ** \brief 生成电池图标显示的 LCDRAM 值 Z1~Z5
 **
 ** \input punRamData: LCDRAM 的值
 **        enWifiSignal: WiFi 信号强弱
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Battery_Icon(
                un_Ram_Data* punRamData,
                en_remaining_battery_t enBatteryPercent,
                boolean_t bDisplay)
{
    // Clean RAM of Z1~Z5 in LCDRAM5.
    punRamData[LCDRAM_INDEX_5].u8_dis[1] &= 0xf7;
    punRamData[LCDRAM_INDEX_5].u8_dis[2] &= 0xf0;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x01;

        if(BatteryPercent25 == enBatteryPercent)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x08;
        }

        if(BatteryPercent50 == enBatteryPercent)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x08;
            punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x08;
        }

        if(BatteryPercent75 == enBatteryPercent)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x08;
            punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x0C;
        }

        if(BatteryPercent100 == enBatteryPercent)
        {
            punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x08;
            punRamData[LCDRAM_INDEX_5].u8_dis[2] |= 0x0e;
        }
    }
}

/******************************************************************************
 ** \brief 生成日期和时间显示的 LCDRAM 值. 13~21, T26~T29, COL1, ZZ,
 **        YY, S1 and S2
 **
 ** \input punRamData: LCDRAM 的值
 **        u8Hour: 启动时间需要显示的小时
 **        u8Minute: 启动时间需要显示的分钟
 **        enWorkingMode: 目前选定的工作模式
 **        bDisplay - TRUE: 显示
 **                   FALSE：不显示
 *****************************************************************************/
void Lcd_D61593A_GenRam_Date_And_Time(
                                un_Ram_Data* punRamData,
                                uint8_t u8Year,
                                uint8_t u8Month,
                                uint8_t u8Day,
                                uint8_t u8Hour,
                                uint8_t u8Minute,
                                boolean_t bDisplay)
{
    uint8_t u8YearSingle, u8YearTen, u8MonthSingle, u8MonthTen, u8DaySingle, u8DayTen;
    uint8_t u8HourSingle, u8HourTen, u8MinuteSingle, u8MinuteTen;

    u8YearSingle = u8Year % 10;
    u8YearTen = u8Year / 10 % 10;
    u8MonthSingle = u8Month % 10;
    u8MonthTen = u8Month / 10 % 10;
    u8DaySingle = u8Day % 10;
    u8DayTen = u8Day / 10 % 10;

    u8HourSingle = u8Hour % 10;
    u8HourTen = u8Hour / 10 % 10;
    u8MinuteSingle = u8Minute % 10;
    u8MinuteTen = u8Minute / 10 % 10;

    punRamData[LCDRAM_INDEX_2].u32_dis &= MASK_LCDRAM2_DATE_TIME;
    punRamData[LCDRAM_INDEX_3].u32_dis &= MASK_LCDRAM3_DATE_TIME;
    punRamData[LCDRAM_INDEX_4].u32_dis &= MASK_LCDRAM4_DATE_TIME;
    punRamData[LCDRAM_INDEX_5].u32_dis &= MASK_LCDRAM5_DATE_TIME;

    if(TRUE == bDisplay)
    {
        punRamData[LCDRAM_INDEX_2].u8_dis[2] |= 0x11;    // Display ZZ and T29.
        punRamData[LCDRAM_INDEX_3].u16_dis[1] |= 0x0140;    // Display COL1 and T28.
        punRamData[LCDRAM_INDEX_4].u8_dis[1] |= 0x10;    // Display T27.
        punRamData[LCDRAM_INDEX_4].u8_dis[3] |= 0x10;    // Display T26.
        punRamData[LCDRAM_INDEX_5].u8_dis[1] |= 0x01;    // Display YY.

        if(u8Year >= 0 && u8Year <= 99)
        {
            punRamData[LCDRAM_INDEX_2].u16_dis[1] |= u16Num12To21Table[u8YearTen];    // 17
            punRamData[LCDRAM_INDEX_3].u16_dis[0] |= u16Num12To21Table[u8YearSingle];    // 18
        }
        else
        {
            punRamData[LCDRAM_INDEX_2].u16_dis[1] |= u16Num12To21Table[10];    // "E" in 17
            punRamData[LCDRAM_INDEX_3].u16_dis[0] |= u16Num12To21Table[10];    // "E" in 18
        }

        if(u8Month >= 1 && u8Month <= 12)
        {
            punRamData[LCDRAM_INDEX_3].u16_dis[1] |= (u16Num12To21Table[u8MonthSingle] & 0x00f0) << 8;    // 19
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[u8MonthSingle] & 0xf000) >> 8;    // 19
            punRamData[LCDRAM_INDEX_3].u8_dis[3] |= 0x10;    // Display S2

            if(0 == u8MonthTen)
            {
                punRamData[LCDRAM_INDEX_3].u8_dis[2] |= 0x20;    // Display S1
            }
        }
        else
        {
            punRamData[LCDRAM_INDEX_3].u16_dis[1] |= (u16Num12To21Table[10] & 0x00f0) << 8;    // "E" in 19
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[10] & 0xf000) >> 8;    // "E" in 19
        }

        if(u8Day >= 1 && u8Day <= 31)
        {
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[u8DayTen] & 0x00f0) << 8;    // 20
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[u8DayTen] & 0xf000) >> 8;    // 20
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[u8DaySingle] & 0x00f0) << 8;    // 21
            punRamData[LCDRAM_INDEX_5].u16_dis[0] |= (u16Num12To21Table[u8DaySingle] & 0xf000) >> 8;    // 21
        }
        else
        {
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[10] & 0x00f0) << 8;    // "E" in 20
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[10] & 0xf000) >> 8;    // "E" in 20
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[10] & 0x00f0) << 8;    // "E" in 21
            punRamData[LCDRAM_INDEX_5].u16_dis[0] |= (u16Num12To21Table[10] & 0xf000) >> 8;    // "E" in 21
        }

        if(u8Hour >= 0 && u8Hour <= 23)
        {
            punRamData[LCDRAM_INDEX_3].u16_dis[0] |= u16Num12To21Table[u8HourSingle] >> 4;    // 15
            if(u8HourTen > 0)
            {
                punRamData[LCDRAM_INDEX_2].u16_dis[1] |= u16Num12To21Table[u8HourTen] >> 4;    // 16
            }
        }
        else
        {
            punRamData[LCDRAM_INDEX_3].u16_dis[0] |= u16Num12To21Table[10] >> 4;    // "E" in 15
            punRamData[LCDRAM_INDEX_2].u16_dis[1] |= u16Num12To21Table[10] >> 4;    // "E" in 16
        }

        if(u8Minute >= 0 && u8Minute <= 59)
        {
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[u8MinuteSingle] & 0x00f0) << 4;    // 13
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[u8MinuteSingle] & 0xf000) >> 12;    // 13
            punRamData[LCDRAM_INDEX_3].u16_dis[1] |= (u16Num12To21Table[u8MinuteTen] & 0x00f0) << 4;    // 14
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[u8MinuteTen] & 0xf000) >> 12;    // 14
        }
        else
        {
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[10] & 0x00f0) << 4;    // "E" in 13
            punRamData[LCDRAM_INDEX_4].u16_dis[1] |= (u16Num12To21Table[10] & 0xf000) >> 12;    // "E" in 13
            punRamData[LCDRAM_INDEX_3].u16_dis[1] |= (u16Num12To21Table[10] & 0x00f0) << 4;    // "E" in 14
            punRamData[LCDRAM_INDEX_4].u16_dis[0] |= (u16Num12To21Table[10] & 0xf000) >> 12;    // "E" in 14
        }
    }
}

