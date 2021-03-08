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
#include "core_cm0plus.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 *****************************************************************************/
#define MIN_KEY_COUNT 0
#define KEY_LONG_PRESS_CNT 25 // 250ms
#define LCD_CONTENT_STROBE_DURATION 50  // 500ms
#define MODE_KEY_LONG_PRESS_CNT 600 // 6000ms
#define SET_OK_KEY_LONG_PRESS_CNT 200 // 2000ms

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 *****************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 *****************************************************************************/
typedef union
{
    uint16_t Full;
    struct
    {
        uint8_t Power   :1;
        uint8_t Mode    :1;
        uint8_t Set     :1;
        uint8_t OK      :1;
        uint8_t Down    :1;
        uint8_t Up      :1;
        uint8_t Lock    :1;
        uint8_t SetHold :1;
        uint8_t Reset   :1;
        uint8_t         :7;
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
__IO uint8_t u8PowerOnFlag, u8RtcFlag;
__IO uint32_t u32UpDownCnt;
__IO uint8_t u8StopFlag, u8GroupNum, u8ChannelManual;
__IO en_working_mode_t enWorkingMode;
__IO uint32_t u32GroupDataAuto[GROUP_NUM_MAX][AUTOMODE_GROUP_DATA_ELEMENT_MAX];
__IO uint16_t u16WateringTimeManual[CHANNEL_NUM_MAX] = {0, 0};
__IO stc_rtc_time_t stcRtcTime;

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
boolean_t App_GetRtcTime(void);
uint8_t App_DaysInAMonth(stc_rtc_time_t *time);
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
    }

    App_LcdRam_Init(u32LcdRamData);

    unKeyPress.Full = 0x0000;
    u8PowerOnFlag = 1;
    enFocusOn = Nothing;
    enLockStatus = Unlock;
    u32UpDownCnt = 0;

    DDL_ZERO_STRUCT(stcRtcTime);

    App_ClkInit(); //设置RCH为4MHz内部时钟初始化配置
    App_KeyInit();

    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);            ///< 使能RCL时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);                ///< 配置内部低速时钟频率为32.768kHz
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd, TRUE);   ///< 开启LCD时钟
    App_PortCfg();               ///< LCD端口配置
    App_LcdCfg();                ///< LCD模块配置
    Lcd_ClearDisp();             ///< 清屏

    App_Timer0Cfg(160);   //周期 = 160*(1/(4*1024)*256 = 10ms
    Bt_M0_Run(TIM0);    // Timer0 运行

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//GPIO外设时钟打开
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    App_RtcCfg();

    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
    if(ModeAutomatic == enWorkingMode)
    {
        Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                            TRUE,
                            enFocusOn);
        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                    (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                    TRUE,
                                    enFocusOn);
    }
    else
    {
        Lcd_D61593A_GenRam_Channel(u32LcdRamData, u8ChannelManual + 1, TRUE, enFocusOn);
        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
    }
    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                enWorkingMode,
                                TRUE,
                                enFocusOn);
    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                            enWorkingMode,
                            TRUE,
                            enFocusOn);
    Lcd_D61593A_GenRam_Smart1(u32LcdRamData, SmartModeDry, FALSE);
    Lcd_D61593A_GenRam_Smart2(u32LcdRamData, SmartModeWet, FALSE);
    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
    Lcd_D61593A_GenRam_Lock_Icon(u32LcdRamData, enLockStatus, TRUE);
    Lcd_D61593A_GenRam_Wifi_Icon(u32LcdRamData, WifiSignalStrong, FALSE);
    Lcd_D61593A_GenRam_Battery_Icon(u32LcdRamData, BatteryPercent100, TRUE);
    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);

    App_Lcd_Display_Update(u32LcdRamData);

    while(1)
    {
        if(1 == u8RtcFlag)
        {
            u8RtcFlag = 0;
            if(TRUE == App_GetRtcTime())
            {
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
            }
        }

        if(unKeyPress.Full != 0x0000)    // Key pressed detected
        {
            App_KeyHandler();
            unKeyPress.Full = 0x0000;
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

    unKeyTypeTemp.Full = 0x0000;

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
    static uint32_t u32Tim0Cnt = 0;
    static un_key_type unKeyPressTemp;  //stores temporary key values
    un_key_type unKeyPressDetected;     //stores which key was pressed

    unKeyPressDetected = App_KeyDetect();

    switch(enKeyState)
    {
        case Waiting:
            if(unKeyPressDetected.Full)   //if any key press detected
            {
                enKeyState = Detected;        //change state
                u32Tim0Cnt = 0;
                u32UpDownCnt = 0;
                unKeyPressTemp = unKeyPressDetected;  //Record the temporary value
            }
            break;
        case Detected:
            if(unKeyPressTemp.Full == unKeyPressDetected.Full)
            {
                ++u32Tim0Cnt;    //Guarded state action, if same value increase key count
                if(u32Tim0Cnt > MIN_KEY_COUNT)
                {
                    u32Tim0Cnt = 0;
                    enKeyState = WaitForRelease;   //guarded state transition
                }
            }
            else
            {
                enKeyState = Waiting;   //state transition
            }
            break;
        case WaitForRelease:
            ++u32Tim0Cnt;

            if(u32Tim0Cnt > KEY_LONG_PRESS_CNT)  // 长按超过 (KEY_LONG_PRESS_CNT * 10)ms, 响应一次长按操作(Ok, Set, Up, Down)
            {
                if(unKeyPressDetected.OK)
                {
                    // 识别为Lock键, 先执行功能, 等待OK键松开. 等待过程中不再执行动作
                    if(unKeyPressTemp.Lock)
                    {
                        enKeyState = WaitForRelease;
                        u32Tim0Cnt = 0;
                    }
                    else
                    {
                        if(u32Tim0Cnt >= SET_OK_KEY_LONG_PRESS_CNT)
                        {
                            enKeyState = Update;
                            unKeyPressTemp.Lock = 1;
                        }
                    }
                }
                else if(unKeyPressDetected.Set)
                {
                    // 识别为SetHold键, 先执行功能, 等待Set键松开. 等待过程中不再执行动作
                    if(unKeyPressTemp.SetHold)
                    {
                        enKeyState = WaitForRelease;
                        u32Tim0Cnt = 0;
                    }
                    else
                    {
                        if(u32Tim0Cnt >= SET_OK_KEY_LONG_PRESS_CNT)
                        {
                            enKeyState = Update;
                            unKeyPressTemp.SetHold = 1;
                        }
                    }
                }
                else if(unKeyPressDetected.Mode)
                {
                    // 识别为Reset键, 先执行功能, 等待Mode键松开. 等待过程中不再执行动作
                    if(unKeyPressTemp.Reset)
                    {
                        enKeyState = WaitForRelease;
                        u32Tim0Cnt = 0;
                    }
                    else
                    {
                        if(u32Tim0Cnt >= MODE_KEY_LONG_PRESS_CNT)
                        {
                            enKeyState = Update;
                            unKeyPressTemp.Reset = 1;
                        }
                    }
                }
                else if(unKeyPressDetected.Up || unKeyPressDetected.Down)
                {
                    // 识别到长按上/下键
                    u32UpDownCnt++;
                    enKeyState = Update;
                }
                else if(!unKeyPressDetected.Full)
                {
                    if(unKeyPressTemp.Lock || unKeyPressTemp.SetHold || unKeyPressTemp.Reset)
                    {
                        enKeyState = Waiting;
                        u32Tim0Cnt = 0;
                        unKeyPressTemp.Full = 0x0000;
                    }
                    else
                    {
                        enKeyState = Update;
                        u32UpDownCnt = 0;
                    }
                }
            }
            else
            {
                if(!unKeyPressDetected.Full)
                {
                    if(unKeyPressTemp.Lock || unKeyPressTemp.SetHold || unKeyPressTemp.Reset)
                    {
                        enKeyState = Waiting;
                        u32Tim0Cnt = 0;
                        unKeyPressTemp.Full = 0x0000;
                    }
                    else
                    {
                        enKeyState = Update;   //state transition when all buttons released
                        u32UpDownCnt = 0;
                    }
                }
            }
            break;
        case Update:
            unKeyPress = unKeyPressTemp;    // HERE the Key value is updated
            u32Tim0Cnt = 0;
            if(unKeyPressTemp.Lock || unKeyPressTemp.SetHold || unKeyPressTemp.Reset)
            {
                enKeyState = WaitForRelease;
            }
            else
            {
                if(u32UpDownCnt > 0)
                {
                    enKeyState = WaitForRelease;
                    /* 识别到长按上/下键, 按的时间越长, 执行动作的速度越快, 最终以最快的速度匀速执行
                     * 在 (30 * KEY_LONG_PRESS_CNT * 10)ms 时达到最快速度.
                     * u32Tim0Cnt = 15 表示每隔 ((KEY_LONG_PRESS_CNT - 15) * 10)ms 就响应一次按键*/
                    if(u32UpDownCnt <= 30)
                    {
                        u32Tim0Cnt = u32UpDownCnt / 2;
                    }
                    else
                    {
                        u32Tim0Cnt = 15;
                    }
                }
                else
                {
                    enKeyState = Waiting;
                    unKeyPressTemp.Full = 0x0000;
                }
            }
            break;
        default:
            enKeyState = Waiting;
            u32Tim0Cnt = 0;
            u32UpDownCnt = 0;
            unKeyPressTemp.Full = 0x0000;
            unKeyPress.Full = 0x0000;
            break;
	}
}

void App_KeyHandler(void)
{
    static uint8_t j = 0;

    if(enLockStatus < Lock  && unKeyPress.Power)
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

    if(Unlock == enLockStatus && unKeyPress.Mode)
    {
        enFocusOn = Nothing;
        if(ModeAutomatic == enWorkingMode)
        {
            enWorkingMode = ModeManual;
            u8StopFlag = 1;
            Lcd_D61593A_GenRam_Channel(u32LcdRamData, u8ChannelManual + 1, TRUE, enFocusOn);
            Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
        }
        else
        {
            enWorkingMode = ModeAutomatic;
            u8StopFlag = 0;
            Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                TRUE,
                                enFocusOn);
            Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                    (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                    TRUE,
                                    enFocusOn);
        }
        Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                    enWorkingMode,
                                    TRUE,
                                    enFocusOn);
        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                enWorkingMode,
                                TRUE,
                                enFocusOn);
        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
    }

    if(Unlock == enLockStatus && unKeyPress.Set)
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
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                        (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                        TRUE,
                                        enFocusOn);
                    u8StopFlag = 0;
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    break;

                case WateringTime:
                    enFocusOn = Nothing;
                    u8StopFlag = 0;
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                TRUE,
                                                enFocusOn);
                    break;

                case StartingTimeH:
                case StartingTimeM:
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                    u8StopFlag = 0;
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    break;

                case DaysApart:
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                            enWorkingMode,
                                            TRUE,
                                            enFocusOn);
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
                Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
            }
        }
    }

    if(Unlock == enLockStatus && unKeyPress.OK)
    {
        if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
        {
            switch(enFocusOn)
            {
                case RtcYear:
                    enFocusOn = RtcMonth;
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    break;

                case RtcMonth:
                    enFocusOn = RtcDay;
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    break;

                case RtcDay:
                    enFocusOn = RtcHour;
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    break;

                case RtcHour:
                    enFocusOn = RtcMin;
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    break;

                case RtcMin:
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
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
                        Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                            TRUE,
                                            enFocusOn);
                        break;

                    case WateringTime:
                        enFocusOn = StartingTimeH;
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                    (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                    TRUE,
                                                    enFocusOn);
                        break;

                    case StartingTimeH:
                        enFocusOn = StartingTimeM;
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                    enWorkingMode,
                                                    TRUE,
                                                    enFocusOn);
                        break;

                    case StartingTimeM:
                        enFocusOn = DaysApart;
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                    enWorkingMode,
                                                    TRUE,
                                                    enFocusOn);
                        break;

                    case DaysApart:
                        enFocusOn = Nothing;
                        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                        u8StopFlag = 0;
                        Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        App_ConvertUserData2FlashData();
                        Flash_Manager_Update();
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
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
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

    if(Unlock == enLockStatus && unKeyPress.Down)
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
                    // 组数变化了, 通道、浇水时长、启动时间和间隔天数也需要跟着变化
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                        (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                        TRUE,
                                        enFocusOn);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                TRUE,
                                                enFocusOn);
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                            enWorkingMode,
                                            TRUE,
                                            enFocusOn);
                }
                else
                {
                    if(u8ChannelManual == 0)
                    {
                        u8ChannelManual = 1;
                    }
                    else
                    {
                        --u8ChannelManual;
                    }
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, u8ChannelManual + 1, TRUE, enFocusOn);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                }
                break;

            case Channel:
                if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] == 0)
                {
                    u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] = 1;
                }
                else
                {
                    --u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL];
                }
                Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                    TRUE,
                                    enFocusOn);
                break;

            case WateringTime:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] == 0)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] = 999;
                    }
                    else
                    {
                        --u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME];
                    }
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                TRUE,
                                                enFocusOn);
                }
                else
                {
                    if(u16WateringTimeManual[u8ChannelManual] == 0)
                    {
                        u16WateringTimeManual[u8ChannelManual] = 999;
                    }
                    else
                    {
                        --u16WateringTimeManual[u8ChannelManual];
                    }
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                }
                break;

            case StartingTimeH:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] == 0)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] = 23;
                    }
                    else
                    {
                        --u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR];
                    }
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                }
                break;

            case StartingTimeM:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] == 0)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] = 59;
                    }
                    else
                    {
                        --u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN];
                    }
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                }
                break;

            case DaysApart:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] == 0)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] = 99;
                    }
                    else
                    {
                        --u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART];
                    }
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                            enWorkingMode,
                                            TRUE,
                                            enFocusOn);
                }
                break;

            case RtcYear:
                if(stcRtcTime.u8Year == 0)
                {
                    stcRtcTime.u8Year = 99;
                }
                else
                {
                    stcRtcTime.u8Year--;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMonth:
                if(stcRtcTime.u8Month <= 1)
                {
                    stcRtcTime.u8Month = 12;
                }
                else
                {
                    stcRtcTime.u8Month--;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcDay:
                if(stcRtcTime.u8Day <= 1)
                {
                    stcRtcTime.u8Day = App_DaysInAMonth(&stcRtcTime);
                }
                else
                {
                    stcRtcTime.u8Day--;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcHour:
                if(stcRtcTime.u8Hour == 0)
                {
                    stcRtcTime.u8Hour = 23;
                }
                else
                {
                    stcRtcTime.u8Hour--;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMin:
                if(stcRtcTime.u8Minute == 0)
                {
                    stcRtcTime.u8Minute = 59;
                }
                else
                {
                    stcRtcTime.u8Minute--;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            default:
                break;
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Up)
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
                    // 组数变化了, 通道、浇水时长、启动时间和间隔天数也需要跟着变化
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode);
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                        (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                        TRUE,
                                        enFocusOn);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                TRUE,
                                                enFocusOn);
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                            enWorkingMode,
                                            TRUE,
                                            enFocusOn);
                }
                else
                {
                    if(++u8ChannelManual > 1)
                    {
                        u8ChannelManual = 0;
                    }
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, u8ChannelManual + 1, TRUE, enFocusOn);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                }
                break;

            case Channel:
                if(++u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] > 1)
                {
                    u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] = 0;
                }
                Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                    TRUE,
                                    enFocusOn);
                break;

            case WateringTime:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(++u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] > 999)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] = 0;
                    }
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                TRUE,
                                                enFocusOn);
                }
                else
                {
                    if(++u16WateringTimeManual[u8ChannelManual] > 999)
                    {
                        u16WateringTimeManual[u8ChannelManual] = 0;
                    }
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                }
                break;

            case StartingTimeH:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(++u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] > 23)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR] = 0;
                    }
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                }
                break;

            case StartingTimeM:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(++u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] > 59)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN] = 0;
                    }
                    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                enWorkingMode,
                                                TRUE,
                                                enFocusOn);
                }
                break;

            case DaysApart:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(++u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] > 99)
                    {
                        u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] = 0;
                    }
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                            enWorkingMode,
                                            TRUE,
                                            enFocusOn);
                }
                break;

            case RtcYear:
                if(++stcRtcTime.u8Year > 99)
                {
                    stcRtcTime.u8Year = 0;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMonth:
                if(++stcRtcTime.u8Month > 12)
                {
                    stcRtcTime.u8Month = 1;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcDay:
                if(++stcRtcTime.u8Day > App_DaysInAMonth(&stcRtcTime))
                {
                    stcRtcTime.u8Day = 1;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcHour:
                if(++stcRtcTime.u8Hour > 23)
                {
                    stcRtcTime.u8Hour = 0;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMin:
                if(++stcRtcTime.u8Minute > 59)
                {
                    stcRtcTime.u8Minute = 0;
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            default:
                break;
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

    if(Unlock == enLockStatus && unKeyPress.Reset)
    {
        Flash_Manager_Erase_All_Data();
        delay1ms(10);
        // Reset MCU
        NVIC_SystemReset();
    }

    App_Lcd_Display_Update(u32LcdRamData);
}

void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;

    RtcInitStruct.rtcAmpm = RtcPm;        //24小时制
    RtcInitStruct.rtcClksrc = RtcClkRcl;  //内部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrds;  //周期中断类型PRDS
    RtcInitStruct.rtcPrdsel.rtcPrds = Rtc1S;      //周期中断事件间隔
    RtcInitStruct.rtcTime.u8Second = 0x00;
    RtcInitStruct.rtcTime.u8Minute = 0x00;
    RtcInitStruct.rtcTime.u8Hour   = 0x00;
    RtcInitStruct.rtcTime.u8Day    = 0x01;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x01;
    RtcInitStruct.rtcTime.u8Month  = 0x01;
    RtcInitStruct.rtcTime.u8Year   = 0x00;
    RtcInitStruct.rtcCompen = RtcCompenEnable;
    RtcInitStruct.rtcCompValue = 0;//补偿值根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                     //使能闹钟中断
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);  //使能RTC中断向量
    Rtc_Cmd(TRUE);                          //使能RTC开始计数
    Rtc_StartWait();                      //启动RTC计数，如果要立即切换到低功耗，需要执行此函数
}

// 当分钟, 小时, 日月年有更新时, 才返回 TRUE
boolean_t App_GetRtcTime(void)
{
    stc_rtc_time_t stcRtcTimeTmp;

    Rtc_ReadDateTime(&stcRtcTimeTmp);
    if(stcRtcTime.u8Minute == stcRtcTimeTmp.u8Minute &&
        stcRtcTime.u8Hour == stcRtcTimeTmp.u8Hour &&
        stcRtcTime.u8Day == stcRtcTimeTmp.u8Day &&
        stcRtcTime.u8Month == stcRtcTimeTmp.u8Month &&
        stcRtcTime.u8Year == stcRtcTimeTmp.u8Year)
    {
        stcRtcTime.u8Second = stcRtcTimeTmp.u8Second;
        return FALSE;
    }
    else
    {
        stcRtcTime.u8Second = stcRtcTimeTmp.u8Second;
        stcRtcTime.u8Minute = stcRtcTimeTmp.u8Minute;
        stcRtcTime.u8Hour = stcRtcTimeTmp.u8Hour;
        stcRtcTime.u8Day = stcRtcTimeTmp.u8Day;
        stcRtcTime.u8Month = stcRtcTimeTmp.u8Month;
        stcRtcTime.u8Year = stcRtcTimeTmp.u8Year;
        return TRUE;
    }
}

uint8_t App_DaysInAMonth(stc_rtc_time_t *time)
{
    uint8_t u8DaysInAMonth = 0;

    if(time->u8Month == 2)
    {
        u8DaysInAMonth = Get_Month2_Day(time->u8Year);
    }
    else
    {
        if(time->u8Month == 1 ||
            time->u8Month == 3 ||
            time->u8Month == 5 ||
            time->u8Month == 7 ||
            time->u8Month == 8 ||
            time->u8Month == 10 ||
            time->u8Month == 12)
        {
            u8DaysInAMonth = 31;
        }
        else
        {
            u8DaysInAMonth = 30;
        }
    }
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
                Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                    bFlipFlag,
                                    enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case WateringTime:
                bFlipFlag = !bFlipFlag;
                if(ModeAutomatic == enWorkingMode)
                {
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                bFlipFlag,
                                                enFocusOn);
                }
                else
                {
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], bFlipFlag, enFocusOn);
                }
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case StartingTimeH:
            case StartingTimeM:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                            enWorkingMode,
                                            bFlipFlag,
                                            enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case DaysApart:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                        (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                        enWorkingMode,
                                        bFlipFlag,
                                        enFocusOn);
                App_Lcd_Display_Update(u32LcdRamData);
                break;

            case RtcYear:
            case RtcMonth:
            case RtcDay:
            case RtcHour:
            case RtcMin:
                bFlipFlag = !bFlipFlag;
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, bFlipFlag, enFocusOn);
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
    uint8_t u8GroupIdx = 0;

    u8GroupNum = 0;
    enWorkingMode = ModeAutomatic;
    u8StopFlag = 0;
    u8ChannelManual = 0;
    u16WateringTimeManual[0] = 0;
    u16WateringTimeManual[1] = 0;

    for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
    {
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART] = 0;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL] = 0;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR] = 0;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN] = 0;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] = 0;
    }

    App_ConvertUserData2FlashData();
}

void App_ConvertFlashData2UserData(void)
{
    uint8_t u8GroupIdx = 0;

    u8GroupNum = stcFlashManager.u8FlashManagerData[1] & 0x0F;
    enWorkingMode = (stcFlashManager.u8FlashManagerData[1] & 0x10) >> 4;
    u8StopFlag = (stcFlashManager.u8FlashManagerData[1] & 0x20) >> 5;
    u8ChannelManual = (stcFlashManager.u8FlashManagerData[1] & 0x40) >> 6;
    u16WateringTimeManual[0] = stcFlashManager.u8FlashManagerData[2] |
                                ((stcFlashManager.u8FlashManagerData[3] & 0x03) << 8);
    u16WateringTimeManual[1] = ((stcFlashManager.u8FlashManagerData[3] & 0xC0) >> 6) |
                                (stcFlashManager.u8FlashManagerData[4] << 2);

    for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
    {
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART]
            = stcFlashManager.u8FlashManagerData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x7F;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL]
            = (stcFlashManager.u8FlashManagerData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x80) >> 7;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR]
            = stcFlashManager.u8FlashManagerData[6 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx];
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN]
            = stcFlashManager.u8FlashManagerData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x3F;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME]
            = ((stcFlashManager.u8FlashManagerData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0xC0) >> 6) |
                (stcFlashManager.u8FlashManagerData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] << 2);
    }
}

void App_ConvertUserData2FlashData(void)
{
    uint8_t u8GroupIdx = 0;

    stcFlashManager.u8FlashManagerData[0] = FLASH_DATA_START_CODE;
    stcFlashManager.u8FlashManagerData[1] = (u8GroupNum | (enWorkingMode << 4) | (u8StopFlag << 5) | (u8ChannelManual << 6)) & 0x7F;
    stcFlashManager.u8FlashManagerData[2] = (uint8_t)(u16WateringTimeManual[0] & 0x00FF);
    stcFlashManager.u8FlashManagerData[3] = (uint8_t)(((u16WateringTimeManual[0] & 0x0300) >> 8) |
                                            ((u16WateringTimeManual[1] & 0x0003) << 6)) & 0xC3;
    stcFlashManager.u8FlashManagerData[4] = (uint8_t)((u16WateringTimeManual[1] & 0x03FC) >> 2);

    for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
    {
        stcFlashManager.u8FlashManagerData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART] & 0x7F) |
                ((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL] & 0x01) << 7));
        stcFlashManager.u8FlashManagerData[6 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR];
        stcFlashManager.u8FlashManagerData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = ((uint8_t)u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN] & 0x3F) |
                ((uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0x03) << 6));
        stcFlashManager.u8FlashManagerData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0x03FC) >> 2);
    }

    stcFlashManager.u8FlashManagerData[FLASH_MANAGER_DATA_LEN - 1] = Flash_Manager_Data_BCC_Checksum(stcFlashManager.u8FlashManagerData, FLASH_MANAGER_DATA_LEN);
}
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
