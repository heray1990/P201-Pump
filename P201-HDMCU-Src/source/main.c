/*****************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 *****************************************************************************/

/******************************************************************************
 * Include files
 *****************************************************************************/
#include "lcd.h"
#include "lpm.h"
#include "bt.h"
#include "gpio.h"
#include "flash.h"
#include "rtc.h"
#include "lcd_D61593A.h"
#include "flash_manager.h"
#include "general_define.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 *****************************************************************************/
#define MIN_KEY_COUNT 0
#define KEY_LONG_PRESS_CNT 25 // 250ms
#define LCD_CONTENT_STROBE_DURATION 50  // 500ms
//#define CLEAR_FALSH

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 *****************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 *****************************************************************************/
typedef union
{
    unsigned char Full;
    struct
    {
        unsigned char Power :1;
        unsigned char Mode :1;
        unsigned char Set :1;
        unsigned char OK :1;
        unsigned char Down :1;
        unsigned char Up :1;
        unsigned char Lock :1;
        unsigned char SetHold :1;
    };
}un_key_type;

typedef enum
{
    Waiting,
    Detected,
    WaitForRelease,
    Update,
    UpdateForLongPress
}en_key_states;

// sizeof(stc_user_data_t) = 52
typedef struct stc_user_datat
{
    uint8_t u8StartCode;
    unsigned int u8GroupNum;
    en_working_mode_t enWorkingMode;
    uint8_t u8StopFlag;
    uint16_t u16WateringTimeManul;
    uint8_t u8CheckSumBCC;
} stc_user_data_t;

/******************************************************************************
 * Local function prototypes ('static')
 *****************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 *****************************************************************************/
un_Ram_Data u32LcdRamData[LCDRAM_INDEX_MAX];
__IO un_key_type unKeyPress;
__IO en_focus_on enFocusOn;
__IO en_lock_status_t enLockStatus;
__IO uint8_t u8PowerOnFlag, u8RtcFlag, u8KeyLongPressCnt;
__IO uint8_t u8StopFlag, u8GroupNum;
__IO en_working_mode_t enWorkingMode;
__IO uint16_t stcGroupDataAuto[GROUP_NUM_MAX][AUTOMODE_GROUP_DATA_ELEMENT_MAX];
__IO uint16_t u16WateringTimeManual;
__IO uint8_t u8RtcSecond, u8RtcMinute, u8RtcHour, u8RtcDay, u8RtcMonth, u8RtcYear;
__IO stc_user_data_t stcUserData;

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 *****************************************************************************/

/******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 *****************************************************************************/
void App_ClkInit(void);
void App_KeyInit(void);
un_key_type App_KeyDetect(void);
void App_KeyStateChkSet(void);
void App_KeyHandler(void);
void App_RtcCfg(void);
void App_RtcTime(void);
void App_PortCfg(void);
void App_LcdCfg(void);
void App_LcdRam_Init(un_Ram_Data* pu32Data);
void App_Lcd_Display_Update(un_Ram_Data* pu32Data);
void App_LcdStrobeControl(void);
void App_Timer0Cfg(uint16_t u16Period);
void App_UserDataSetDefaultVal(void);
void App_ConvertFlashData2UserData(void);
void App_ConvertUserData2FlashData(void);


int32_t main(void)
{
#ifdef CLEAR_FALSH
    Flash_SectorErase(FLASH_MANAGER_DATA_SECTOR_0_HEAD_ADDR);
    Flash_SectorErase(FLASH_MANAGER_DATA_SECTOR_1_HEAD_ADDR);
    Flash_SectorErase(FLASH_MANAGER_DATA_SECTOR_2_HEAD_ADDR);
#endif
    uint8_t u8PartIdx = 0;

    DDL_ZERO_STRUCT(stcUserData);

    if(Ok == Flash_Manager_Init())
    {
        if(stcFlashManager.bFlashEmpty)
        {
            // Flash分区都为空, 则设置默认值
            App_UserDataSetDefaultVal();
        }
        else
        {
            App_ConvertFlashData2UserData();
        }
    #if 0//ndef CLEAR_FALSH
        Flash_Manager_Update();
    #endif
    }

    App_LcdRam_Init(u32LcdRamData);

    unKeyPress.Full = 0x00;
    u8PowerOnFlag = 1;
    enFocusOn = Nothing;
    enLockStatus = Unlock;
    u8KeyLongPressCnt = 0;

    u8RtcYear = 21;
    u8RtcMonth = 2;
    u8RtcDay = 22;
    u8RtcHour = 17;
    u8RtcMinute = 21;

    App_ClkInit(); //设置RCH为4MHz内部时钟初始化配置
    App_KeyInit();

    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);            ///< 使能RCL时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);                ///< 配置内部低速时钟频率为32.768kHz
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd, TRUE);   ///< 开启LCD时钟
    App_PortCfg();               ///< LCD端口配置
    App_LcdCfg();                ///< LCD模块配置
    Lcd_ClearDisp();             ///< 清屏

    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
    Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
    if(ModeAutomatic == enWorkingMode)
    {
        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
    }
    else
    {
        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
    }
    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
    Lcd_D61593A_GenRam_Smart1(u32LcdRamData, SmartModeDry, FALSE);
    Lcd_D61593A_GenRam_Smart2(u32LcdRamData, SmartModeWet, FALSE);
    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
    Lcd_D61593A_GenRam_Lock_Icon(u32LcdRamData, enLockStatus, TRUE);
    Lcd_D61593A_GenRam_Wifi_Icon(u32LcdRamData, WifiSignalStrong, FALSE);
    Lcd_D61593A_GenRam_Battery_Icon(u32LcdRamData, BatteryPercent100, TRUE);
    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);

    App_Lcd_Display_Update(u32LcdRamData);

    App_Timer0Cfg(160);   //周期 = 160*(1/(4*1024)*256 = 10ms
    Bt_M0_Run(TIM0);    // Timer0 运行

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//GPIO外设时钟打开
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    App_RtcCfg();

    while(1)
    {
        if(1 == u8RtcFlag)
        {
            u8RtcFlag = 0;
            App_RtcTime();
            Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
            App_Lcd_Display_Update(u32LcdRamData);
        }

        if(unKeyPress.Full != 0x00)    // Key pressed detected
        {
            App_KeyHandler();
            unKeyPress.Full = 0x00;
        }
    }
}

//时钟初始化配置
void App_ClkInit(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;

    ///< 开启FLASH外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);

    ///<========================== 时钟初始化配置 ===================================
    ///< 因要使用的时钟源HCLK小于24M：此处设置FLASH 读等待周期为0 cycle(默认值也为0 cycle)
    Flash_WaitCycle(FlashWaitCycle0);

    ///< 时钟初始化前，优先设置要使用的时钟源：此处设置RCH为4MHz
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);

    ///< 选择内部RCH作为HCLK时钟源;
    stcCfg.enClkSrc    = SysctrlClkRCH;
    ///< HCLK SYSCLK/1
    stcCfg.enHClkDiv   = SysctrlHclkDiv1;
    ///< PCLK 为HCLK/1
    stcCfg.enPClkDiv   = SysctrlPclkDiv1;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcCfg);
}

void App_KeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    ///< GPIO IO KEY初始化
    Gpio_Init(GpioPortD, GpioPin0, &stcGpioCfg);    // POWER
    Gpio_Init(GpioPortD, GpioPin1, &stcGpioCfg);    // MODE
    Gpio_Init(GpioPortD, GpioPin4, &stcGpioCfg);    // SET
    Gpio_Init(GpioPortD, GpioPin6, &stcGpioCfg);    // OK
    Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg);    // DW
    Gpio_Init(GpioPortD, GpioPin7, &stcGpioCfg);    // UP
}

un_key_type App_KeyDetect(void)
{
    un_key_type unKeyTypeTemp;

    unKeyTypeTemp.Full = 0x00;

    ///< 检测各按键是否按下(低电平)
    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin0))
    {
        unKeyTypeTemp.Power = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin1))
    {
        unKeyTypeTemp.Mode = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin4))
    {
        unKeyTypeTemp.Set = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin6))
    {
        unKeyTypeTemp.OK = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin5))
    {
        unKeyTypeTemp.Down = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin7))
    {
        unKeyTypeTemp.Up = 1;
        return unKeyTypeTemp;
    }

    return unKeyTypeTemp;
}

// 状态机按键消抖, 每10ms检查或设置1次状态
void App_KeyStateChkSet(void)
{
    static en_key_states enKeyState = Waiting;
    static uint8_t u8Tim0Cnt = 0;
    static un_key_type unKeyPressTemp;  //stores temporary key values
    un_key_type unKeyPressDetected;     //stores which key was pressed

    unKeyPressDetected = App_KeyDetect();

    switch(enKeyState)
    {
        case Waiting:
            if(unKeyPressDetected.Full)   //if any key press detected
            {
                enKeyState = Detected;        //change state
                u8Tim0Cnt = 0;
                u8KeyLongPressCnt = 0;
                unKeyPressTemp = unKeyPressDetected;  //Record the temporary value
            }
            break;
        case Detected:
            if(unKeyPressTemp.Full == unKeyPressDetected.Full)
            {
                ++u8Tim0Cnt;    //Guarded state action, if same value increase key count
                if(u8Tim0Cnt > MIN_KEY_COUNT)
                {
                    u8Tim0Cnt = 0;
                    enKeyState = WaitForRelease;   //guarded state transition
                }
            }
            else
            {
                enKeyState = Waiting;   //state transition
            }
            break;
        case WaitForRelease:
            ++u8Tim0Cnt;
            // 长按超过 (KEY_LONG_PRESS_CNT * 10)ms, 响应一次长按操作
            if(u8Tim0Cnt > KEY_LONG_PRESS_CNT)
            {
                if(unKeyPressTemp.Lock && unKeyPressDetected.OK)
                {
                    // 识别为Lock键, 等待OK键松开
                    u8Tim0Cnt = 0;
                    enKeyState = WaitForRelease;
                }
                else if(unKeyPressTemp.SetHold && unKeyPressDetected.Set)
                {
                    // 识别为SetHold键, 等待Set键松开
                    u8Tim0Cnt = 0;
                    enKeyState = WaitForRelease;
                }
                else
                {
                    if(unKeyPressTemp.Full == unKeyPressDetected.Full)
                    {
                        enKeyState = UpdateForLongPress;
                    }
                    else
                    {
                        enKeyState = Update;
                    }
                }
            }
            else
            {
                if(!unKeyPressDetected.Full)
                {
                    enKeyState = Update;   //state transition when all buttons released
                }
            }
            break;
        case Update:
            unKeyPress = unKeyPressTemp;    //state action    HERE the Key value is updated
            enKeyState = Waiting;           //state transition
            u8Tim0Cnt = 0;                  //state action
            unKeyPressTemp.Full = 0x00;     //state action
            u8KeyLongPressCnt = 0;
            break;
        case UpdateForLongPress:
            if(unKeyPressTemp.OK)
            {
                // 长按OK, 即触发"Lock"键, 跳转到WaitForRelease等待OK键释放
                unKeyPressTemp.OK = 0;
                unKeyPressTemp.Lock = 1;
                u8Tim0Cnt = 0;
                enKeyState = WaitForRelease;
            }
            else if(unKeyPressTemp.Set)
            {
                // 长按Set, 即触发"SetHold"键, 跳转到WaitForRelease等待Set键释放
                unKeyPressTemp.Set = 0;
                unKeyPressTemp.SetHold = 1;
                u8Tim0Cnt = 0;
                enKeyState = WaitForRelease;
            }
            else
            {
                unKeyPress = unKeyPressTemp;    //state action    HERE the Key value is updated
                u8Tim0Cnt = 0;
                enKeyState = WaitForRelease;    //state transition
                u8KeyLongPressCnt++;
            }
            break;
        default:
            enKeyState = Waiting;
            u8Tim0Cnt = 0;
            unKeyPressTemp.Full = 0x00;
            unKeyPress.Full = 0x00;
            break;
	}
}

void App_KeyHandler(void)
{
    static uint8_t j = 0;

    if(enLockStatus < Lock  && unKeyPress.Power && 0 == u8KeyLongPressCnt)
    {
        if(0 == u8PowerOnFlag)
        {
            u8PowerOnFlag = 1;
            enLockStatus = Unlock;
            M0P_LCD->CR0_f.EN = LcdEnable;
            Gpio_SetIO(GpioPortC, GpioPin0);
        }
        else
        {
            u8PowerOnFlag = 0;
            enLockStatus = LockExceptPowerKey;
            M0P_LCD->CR0_f.EN = LcdDisable;
            Gpio_ClrIO(GpioPortC, GpioPin0);
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Mode && 0 == u8KeyLongPressCnt)
    {
        enFocusOn = Nothing;
        if(ModeAutomatic == enWorkingMode)
        {
            enWorkingMode = ModeManual;
            u8StopFlag = 1;
            Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
        }
        else
        {
            enWorkingMode = ModeAutomatic;
            u8StopFlag = 0;
            Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
        }
        Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
        Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
    }

    if(Unlock == enLockStatus && unKeyPress.Set)
    {
        if(0 == u8KeyLongPressCnt)
        {
            if(ModeAutomatic == enWorkingMode)
            {
                switch(enFocusOn)
                {
                    case Nothing:
                        enFocusOn = Channel;
                        u8StopFlag = 1;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        break;

                    case Channel:
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        break;

                    case WateringTime:
                        enFocusOn = Nothing;
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                        break;

                    case StartingTimeH:
                    case StartingTimeM:
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        break;

                    case DaysApart:
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        break;

                    default:
                        enFocusOn = Nothing;
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        break;
                }
            }
            else
            {
                u8StopFlag = 1;
                Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                if(Nothing == enFocusOn)
                {
                    enFocusOn = WateringTime;
                }
                else
                {
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
                }
            }
        }
    }

    if(Unlock == enLockStatus && unKeyPress.OK)
    {
        if(0 == u8KeyLongPressCnt)
        {
            if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
            {
                switch(enFocusOn)
                {
                    case RtcYear:
                        enFocusOn = RtcMonth;
                        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                        break;

                    case RtcMonth:
                        enFocusOn = RtcDay;
                        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                        break;

                    case RtcDay:
                        enFocusOn = RtcHour;
                        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                        break;

                    case RtcHour:
                        enFocusOn = RtcMin;
                        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                        break;

                    case RtcMin:
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                        if(ModeAutomatic == enWorkingMode)
                        {
                            u8StopFlag = 0;
                            Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        }
                        break;

                    default:
                        enFocusOn = Nothing;
                        break;
                }
            }
            else
            {
                if(ModeAutomatic == enWorkingMode)
                {
                    switch(enFocusOn)
                    {
                        case Channel:
                            enFocusOn = WateringTime;
                            Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                            break;

                        case WateringTime:
                            enFocusOn = StartingTimeH;
                            Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                            break;

                        case StartingTimeH:
                            enFocusOn = StartingTimeM;
                            Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                            break;

                        case StartingTimeM:
                            enFocusOn = DaysApart;
                            Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                            break;

                        case DaysApart:
                            enFocusOn = Nothing;
                            Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                            u8StopFlag = 0;
                            Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                            break;

                        default:
                            enFocusOn = Nothing;
                            break;
                    }
                }
                else
                {
                    if(WateringTime == enFocusOn)
                    {
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
                    }
                    else
                    {
                        enFocusOn = Nothing;
                        u8StopFlag = !u8StopFlag;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    }
                }
            }
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Down)
    {
        if(0 == u8KeyLongPressCnt)
        {
            switch(enFocusOn)
            {
                case Nothing:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(u8GroupNum == GROUP_NUM_MIN)
                        {
                            u8GroupNum = GROUP_NUM_MAX - 1;
                        }
                        else
                        {
                            --u8GroupNum;
                        }
                        // 组数变化了, 通道、浇水市场、启动时间和间隔天数也需要跟着变化
                        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
                        Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case Channel:
                    if(stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] == 0)
                    {
                        stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] = 9;
                    }
                    else
                    {
                        --stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL];
                    }
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                    break;

                case WateringTime:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] == 0)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] = 999;
                        }
                        else
                        {
                            --stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME];
                        }
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                    }
                    else
                    {
                        if(u16WateringTimeManual == 0)
                        {
                            u16WateringTimeManual = 999;
                        }
                        else
                        {
                            --u16WateringTimeManual;
                        }
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
                    }
                    break;

                case StartingTimeH:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] == 0)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] = 23;
                        }
                        else
                        {
                            --stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR];
                        }
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case StartingTimeM:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] == 0)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] = 59;
                        }
                        else
                        {
                            --stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN];
                        }
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case DaysApart:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] == 0)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] = 99;
                        }
                        else
                        {
                            --stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART];
                        }
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case RtcYear:
                    if(u8RtcYear == 0)
                    {
                        u8RtcYear = 99;
                    }
                    else
                    {
                        --u8RtcYear;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcMonth:
                    if(--u8RtcMonth < 1)
                    {
                        u8RtcMonth = 12;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcDay:
                    if(--u8RtcDay < 1)
                    {
                        u8RtcDay = 31;  // 不同月份有不同的天数, 后续区分
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcHour:
                    if(u8RtcHour == 0)
                    {
                        u8RtcHour = 23;
                    }
                    else
                    {
                        --u8RtcHour;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcMin:
                    if(u8RtcMinute == 0)
                    {
                        u8RtcMinute = 59;
                    }
                    else
                    {
                        --u8RtcMinute;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                default:
                    break;
            }
        }
        else
        {
            // 长按处理
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Up)
    {
        if(0 == u8KeyLongPressCnt)
        {
            switch(enFocusOn)
            {
                case Nothing:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(++u8GroupNum > GROUP_NUM_MAX - 1)
                        {
                            u8GroupNum = GROUP_NUM_MIN;
                        }
                        // 组数变化了, 通道、浇水市场、启动时间和间隔天数也需要跟着变化
                        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
                        Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case Channel:
                    if(++stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] > 9)
                    {
                        stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] = 0;
                    }
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], TRUE, enFocusOn);
                    break;

                case WateringTime:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(++stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] > 999)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] = 0;
                        }
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], TRUE, enFocusOn);
                    }
                    else
                    {
                        if(++u16WateringTimeManual > 999)
                        {
                            u16WateringTimeManual = 0;
                        }
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, TRUE, enFocusOn);
                    }
                    break;

                case StartingTimeH:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(++stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] > 23)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] = 0;
                        }
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case StartingTimeM:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(++stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] > 59)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] = 0;
                        }
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case DaysApart:
                    if(ModeAutomatic == enWorkingMode)
                    {
                        if(++stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] > 99)
                        {
                            stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] = 0;
                        }
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, TRUE, enFocusOn);
                    }
                    break;

                case RtcYear:
                    if(++u8RtcYear > 99)
                    {
                        u8RtcYear = 0;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcMonth:
                    if(++u8RtcMonth > 12)
                    {
                        u8RtcMonth = 1;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcDay:
                    if(++u8RtcDay > 31)
                    {
                        u8RtcDay = 1;  // 不同月份有不同的天数, 后续区分
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcHour:
                    if(++u8RtcHour > 23)
                    {
                        u8RtcHour = 0;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                case RtcMin:
                    if(++u8RtcMinute > 59)
                    {
                        u8RtcMinute = 0;
                    }
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, TRUE, enFocusOn);
                    break;

                default:
                    break;
            }
        }
        else
        {
            // 长按处理
        }
    }

    if(enLockStatus != LockExceptPowerKey && unKeyPress.Lock)
    {
        // 开机状态下长按"确定"锁定/解锁按键
        if(Unlock == enLockStatus)
        {
            enLockStatus = Lock;
        }
        else
        {
            enLockStatus = Unlock;
        }
        Lcd_D61593A_GenRam_Lock_Icon(u32LcdRamData, enLockStatus, TRUE);
    }

    if(Unlock == enLockStatus && unKeyPress.SetHold)
    {
        if(enFocusOn != RtcYear)
        {
            enFocusOn = RtcYear;
            u8StopFlag = 1;
            Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
        }
    }

    App_Lcd_Display_Update(u32LcdRamData);
}

void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;

    RtcInitStruct.rtcAmpm = RtcPm;        //12小时制
    RtcInitStruct.rtcClksrc = RtcClkRcl;  //内部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrds;  //周期中断类型PRDS
    RtcInitStruct.rtcPrdsel.rtcPrds = Rtc1S;      //周期中断事件间隔
    RtcInitStruct.rtcTime.u8Second = 0x55;
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;
    RtcInitStruct.rtcCompValue = 0;//补偿值根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                     //使能闹钟中断
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);  //使能RTC中断向量
    Rtc_Cmd(TRUE);                          //使能RTC开始计数
    //Rtc_StartWait();                      //启动RTC计数，如果要立即切换到低功耗，需要执行此函数
}

void App_RtcTime(void)
{
    stc_rtc_time_t stcRtcTime;

    Rtc_ReadDateTime(&stcRtcTime);
    u8RtcSecond = stcRtcTime.u8Second;
    u8RtcMinute = stcRtcTime.u8Minute;
    u8RtcHour   = stcRtcTime.u8Hour;
    u8RtcDay    = stcRtcTime.u8Day;
    u8RtcMonth  = stcRtcTime.u8Month;
    u8RtcYear   = stcRtcTime.u8Year;
}

void Rtc_IRQHandler(void)
{
    if(Rtc_GetPridItStatus() == TRUE)
    {
        u8RtcFlag = 1;
        Rtc_ClearPrdfItStatus();             //清除中断标志位
    }
}

/******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 *****************************************************************************/
void App_PortCfg(void)
{
    Gpio_SetAnalogMode(GpioPortA, GpioPin9);  //COM0
    Gpio_SetAnalogMode(GpioPortA, GpioPin10); //COM1
    Gpio_SetAnalogMode(GpioPortA, GpioPin11); //COM2
    Gpio_SetAnalogMode(GpioPortA, GpioPin12); //COM3
    Gpio_SetAnalogMode(GpioPortC, GpioPin10); //COM4
    Gpio_SetAnalogMode(GpioPortC, GpioPin11); //COM5
    Gpio_SetAnalogMode(GpioPortC, GpioPin12); //COM6
    Gpio_SetAnalogMode(GpioPortD, GpioPin2); //COM7

    Gpio_SetAnalogMode(GpioPortA, GpioPin8);  //SEG0
    Gpio_SetAnalogMode(GpioPortC, GpioPin9);  //SEG1
    Gpio_SetAnalogMode(GpioPortC, GpioPin8);  //SEG2
    Gpio_SetAnalogMode(GpioPortC, GpioPin7);  //SEG3
    Gpio_SetAnalogMode(GpioPortC, GpioPin6);  //SEG4
    Gpio_SetAnalogMode(GpioPortB, GpioPin15); //SEG5
    Gpio_SetAnalogMode(GpioPortB, GpioPin14); //SEG6
    Gpio_SetAnalogMode(GpioPortB, GpioPin13); //SEG7
    Gpio_SetAnalogMode(GpioPortB, GpioPin12); //SEG8
    Gpio_SetAnalogMode(GpioPortB, GpioPin11); //SEG9
    Gpio_SetAnalogMode(GpioPortB, GpioPin10); //SEG10
    Gpio_SetAnalogMode(GpioPortB, GpioPin2); //SEG11
    Gpio_SetAnalogMode(GpioPortB, GpioPin1); //SEG12
    Gpio_SetAnalogMode(GpioPortB, GpioPin0); //SEG13
    Gpio_SetAnalogMode(GpioPortC, GpioPin5); //SEG14
    Gpio_SetAnalogMode(GpioPortC, GpioPin4); //SEG15
    Gpio_SetAnalogMode(GpioPortA, GpioPin7); //SEG16
    Gpio_SetAnalogMode(GpioPortA, GpioPin6); //SEG17
    Gpio_SetAnalogMode(GpioPortA, GpioPin5); //SEG18
    Gpio_SetAnalogMode(GpioPortA, GpioPin4); //SEG19
    Gpio_SetAnalogMode(GpioPortA, GpioPin3); //SEG20
    Gpio_SetAnalogMode(GpioPortA, GpioPin2); //SEG21
    Gpio_SetAnalogMode(GpioPortA, GpioPin1); //SEG22
    Gpio_SetAnalogMode(GpioPortB, GpioPin3); //SEG32/VLCD1
    Gpio_SetAnalogMode(GpioPortB, GpioPin4); //SEG33/VLCD2
    Gpio_SetAnalogMode(GpioPortB, GpioPin5); //SEG34/VLCD3
    Gpio_SetAnalogMode(GpioPortB, GpioPin6); //SEG35/VLCDH
}

/******************************************************************************
 ** \brief  配置LCD
 **
 ** \return 无
 *****************************************************************************/
void App_LcdCfg(void)
{
    stc_lcd_cfg_t LcdInitStruct;
    stc_lcd_segcom_t LcdSegCom;
    stc_gpio_cfg_t stcLcdBlGpioCfg;

    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcLcdBlGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcLcdBlGpioCfg.enPu = GpioPuDisable;
    stcLcdBlGpioCfg.enPd = GpioPdEnable;

    Gpio_SetIO(GpioPortC, GpioPin0);
    ///< GPIO IO LCD BL_ON 端口初始化
    Gpio_Init(GpioPortC, GpioPin0, &stcLcdBlGpioCfg);

    LcdSegCom.u32Seg0_31 = 0xff800000;                              ///< 配置LCD_POEN0寄存器 开启SEG0~SEG22
    LcdSegCom.stc_seg32_51_com0_8_t.seg32_51_com0_8 = 0xffffffff;   ///< 初始化LCD_POEN1寄存器 全部关闭输出端口
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg36Com7 = 0;       ///< 使能COM0~COM7
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg37Com6 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg38Com5 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg39Com4 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Com0_3 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Mux = 0;             ///< Mux=0,Seg32_35=0,BSEL=1表示:选择外部电容工作模式，内部电阻断路
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg32_35 = 0;
    Lcd_SetSegCom(&LcdSegCom);                                      ///< LCD COMSEG端口配置

    LcdInitStruct.LcdBiasSrc = LcdExtCap;                          ///< 电容分压模式，需要外部电路配合
    LcdInitStruct.LcdDuty = LcdDuty8;                              ///< 1/8 Duty
    LcdInitStruct.LcdBias = LcdBias3;                              ///< 1/3 BIAS
    LcdInitStruct.LcdCpClk = LcdClk2k;                             ///< 电压泵时钟频率选择2kHz
    LcdInitStruct.LcdScanClk = LcdClk512hz;                        ///< LCD扫描频率选择512Hz
    LcdInitStruct.LcdMode = LcdMode0;                              ///< 选择模式0
    LcdInitStruct.LcdClkSrc = LcdRCL;                              ///< LCD时钟选择RCL
    LcdInitStruct.LcdEn   = LcdEnable;                             ///< 使能LCD模块
    Lcd_Init(&LcdInitStruct);
}

void App_LcdRam_Init(un_Ram_Data* pu32Data)
{
    uint8_t u8Idx = 0;

    for(u8Idx = 0; u8Idx <= LCDRAM_INDEX_MAX; u8Idx++)
    {
        pu32Data[u8Idx].u32_dis = 0x00000000;
    }
}

void App_Lcd_Display_Update(un_Ram_Data* pu32Data)
{
    uint8_t u8Idx = 0;

    for(u8Idx = 0; u8Idx <= LCDRAM_INDEX_MAX; u8Idx++)
    {
        Lcd_WriteRam(u8Idx, pu32Data[u8Idx].u32_dis);
    }
}

void App_LcdStrobeControl(void)
{
    static uint8_t u8LcdContentSDCnt = 0;
    static boolean_t bFlipFlag = TRUE;

    if(++u8LcdContentSDCnt > LCD_CONTENT_STROBE_DURATION)
    {
        u8LcdContentSDCnt = 0;

        switch(enFocusOn)
        {
            case Channel:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Channel(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL], bFlipFlag, enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case WateringTime:
                bFlipFlag = !bFlipFlag;
                if(ModeAutomatic == enWorkingMode)
                {
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME], bFlipFlag, enFocusOn);
                }
                else
                {
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual, bFlipFlag, enFocusOn);
                }
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case StartingTimeH:
            case StartingTimeM:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR], (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN], enWorkingMode, bFlipFlag, enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case DaysApart:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, (uint8_t)stcGroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART], enWorkingMode, bFlipFlag, enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case RtcYear:
            case RtcMonth:
            case RtcDay:
            case RtcHour:
            case RtcMin:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, u8RtcYear, u8RtcMonth, u8RtcDay, u8RtcHour, u8RtcMinute, bFlipFlag, enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            default:
                bFlipFlag = TRUE;
                break;
        }
    }
}

/******************************************************************************
 ** \brief  Timer0配置初始化
 ** 周期 = u16Period*(1/内部时钟频率)*256
 ** \return 无
 ******************************************************************************/
void App_Timer0Cfg(uint16_t u16Period)
{
    uint16_t                  u16ArrValue;
    uint16_t                  u16CntValue;
    stc_bt_mode0_cfg_t     stcBtBaseCfg;

    DDL_ZERO_STRUCT(stcBtBaseCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能

    stcBtBaseCfg.enWorkMode = BtWorkMode0;                  //定时器模式
    stcBtBaseCfg.enCT       = BtTimer;                      //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS      = BtPCLKDiv256;                 //PCLK/256
    stcBtBaseCfg.enCntMode  = Bt16bitArrMode;               //自动重载16位计数器/定时器
    stcBtBaseCfg.bEnTog     = FALSE;
    stcBtBaseCfg.bEnGate    = FALSE;
    stcBtBaseCfg.enGateP    = BtGatePositive;
    Bt_Mode0_Init(TIM0, &stcBtBaseCfg);                     //TIM0 的模式0功能初始化

    u16ArrValue = 0x10000 - u16Period;
    Bt_M0_ARRSet(TIM0, u16ArrValue);                        //设置重载值(ARR = 0x10000 - 周期)

    u16CntValue = 0x10000 - u16Period;
    Bt_M0_Cnt16Set(TIM0, u16CntValue);                      //设置计数初值

    Bt_ClearIntFlag(TIM0,BtUevIrq);                         //清中断标志
    Bt_Mode0_EnableIrq(TIM0);                               //使能TIM0中断(模式0时只有一个中断)
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //TIM0中断使能
}

void Tim0_IRQHandler(void)
{
    //Timer0 模式0 溢出中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        App_KeyStateChkSet();

        App_LcdStrobeControl();

        Bt_ClearIntFlag(TIM0, BtUevIrq); //中断标志清零
    }
}

void App_UserDataSetDefaultVal(void)
{
    uint8_t u8Idx = 0;

    stcUserData.u8StartCode = FLASH_DATA_START_CODE;
    u8GroupNum = 0;
    enWorkingMode = ModeAutomatic;
    u8StopFlag = 0;
    u16WateringTimeManual = 0;

    for(u8Idx = 0; u8Idx < GROUP_NUM_MAX; u8Idx++)
    {
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_CHANNEL] = 0;
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTHOUR] = 0;
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTMIN] = 0;
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_DAYSAPART] = 0;
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_WATER_TIME] = 0;
    }

    App_ConvertUserData2FlashData();
}


void App_ConvertFlashData2UserData(void)
{
    uint8_t u8Idx = 0;

    stcUserData.u8StartCode = stcFlashManager.u8FlashManagerData[0];
    u8GroupNum = stcFlashManager.u8FlashManagerData[1] & 0x0F;
    enWorkingMode = (stcFlashManager.u8FlashManagerData[1] & 0x10) >> 4;
    u8StopFlag = (stcFlashManager.u8FlashManagerData[1] & 0x20) >> 5;
    u16WateringTimeManual = stcFlashManager.u8FlashManagerData[2] | (stcFlashManager.u8FlashManagerData[3] << 8);

    for(u8Idx = 0; u8Idx < GROUP_NUM_MAX; u8Idx++)
    {
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_CHANNEL] = stcFlashManager.u8FlashManagerData[4 + GROUP_NUM_MAX * u8Idx];
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTHOUR] = stcFlashManager.u8FlashManagerData[5 + GROUP_NUM_MAX * u8Idx];
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTMIN] = stcFlashManager.u8FlashManagerData[6 + GROUP_NUM_MAX * u8Idx];
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_DAYSAPART] = stcFlashManager.u8FlashManagerData[7 + GROUP_NUM_MAX * u8Idx];
        stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_WATER_TIME] = stcFlashManager.u8FlashManagerData[8 + GROUP_NUM_MAX * u8Idx]
                                                        | (stcFlashManager.u8FlashManagerData[9 + GROUP_NUM_MAX * u8Idx] << 8);
    }

    stcUserData.u8CheckSumBCC = stcFlashManager.u8FlashManagerData[FLASH_MANAGER_DATA_LEN - 1];
}

void App_ConvertUserData2FlashData(void)
{
    uint8_t u8Idx = 0;

    stcFlashManager.u8FlashManagerData[0] = FLASH_DATA_START_CODE;
    stcFlashManager.u8FlashManagerData[1] = (u8GroupNum | (enWorkingMode << 4) | (u8StopFlag << 5)) & 0x3F;
    stcFlashManager.u8FlashManagerData[2] = (uint8_t)(u16WateringTimeManual & 0x00FF);
    stcFlashManager.u8FlashManagerData[3] = (uint8_t)(u16WateringTimeManual & 0xFF00 >> 8);

    for(u8Idx = 0; u8Idx < GROUP_NUM_MAX; u8Idx++)
    {
        stcFlashManager.u8FlashManagerData[4 + GROUP_NUM_MAX * u8Idx] = (uint8_t)stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_CHANNEL];
        stcFlashManager.u8FlashManagerData[5 + GROUP_NUM_MAX * u8Idx] = (uint8_t)stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTHOUR];
        stcFlashManager.u8FlashManagerData[6 + GROUP_NUM_MAX * u8Idx] = (uint8_t)stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_STARTMIN];
        stcFlashManager.u8FlashManagerData[7 + GROUP_NUM_MAX * u8Idx] = (uint8_t)stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_DAYSAPART];
        stcFlashManager.u8FlashManagerData[8 + GROUP_NUM_MAX * u8Idx] = (uint8_t)(stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0x00FF);
        stcFlashManager.u8FlashManagerData[9 + GROUP_NUM_MAX * u8Idx] = (uint8_t)(stcGroupDataAuto[u8Idx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0xFF00 >> 8);
    }

    stcUserData.u8CheckSumBCC = Flash_Manager_Data_BCC_Checksum(stcFlashManager.u8FlashManagerData, FLASH_MANAGER_DATA_LEN);
    stcFlashManager.u8FlashManagerData[FLASH_MANAGER_DATA_LEN - 1] = stcUserData.u8CheckSumBCC;
}
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
