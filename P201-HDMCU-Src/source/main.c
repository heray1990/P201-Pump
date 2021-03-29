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
#include "wdt.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 *****************************************************************************/
#define MIN_KEY_COUNT               0
#define KEY_LONG_PRESS_CNT          25  // 250ms
#define LCD_CONTENT_FLASH_FREQ      50  // 500ms
#define LCD_CONTENT_FLASH_DURATION  1000    // 10s
#define MODE_KEY_LONG_PRESS_CNT     600 // 6s
#define SET_OK_KEY_LONG_PRESS_CNT   200 // 2s
#define TIMER0_CNT_WATER_TIME       100 // 1s
#define AUTO_DEEP_SLEEP_CNT         1000    // 10s

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
    Waiting = 0u,
    Detected = 1u,
    WaitForRelease = 2u,
    Update = 3u
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
__IO uint8_t u8PowerOnFlag, u8RtcFlag, u8DeepSleepFlag;
__IO uint32_t u32UpDownCnt;
__IO uint8_t u8StopFlag, u8GroupNum, u8ChannelManual;
__IO en_working_mode_t enWorkingMode;
__IO uint32_t u32GroupDataAuto[GROUP_NUM_MAX][AUTOMODE_GROUP_DATA_ELEMENT_MAX];
__IO uint16_t u16WateringTimeManual[CHANNEL_NUM_MAX] = {0, 0};
__IO uint8_t u8DaysAddUp[GROUP_NUM_MAX] = {0, 0, 0, 0, 0, 0};
__IO stc_rtc_time_t stcRtcTime;
__IO boolean_t bLcdUpdate, bPortDIrFlag;
__IO uint16_t u16LcdFlickerCnt, u16RtcCnt, u16NoKeyPressedCnt, u16WTPump1, u16WTPump2;
static en_key_states enKeyState = Waiting;
__IO uint8_t u8PumpCtrl;

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
void App_RtcInit(void);
boolean_t App_GetRtcTime(void);
uint8_t App_DaysInAMonth(stc_rtc_time_t *time);
boolean_t IsTimeToWater(boolean_t bJustWatered);
void App_LcdPortInit(void);
void App_LcdInit(void);
void App_LcdBlInit(void);
void App_LcdRam_Init(un_Ram_Data* pu32Data);
void App_Lcd_Display_Update(un_Ram_Data* pu32Data);
void App_PumpInit(void);
void App_PumpCtrl(void);
void App_LcdRamFlipCtrl(boolean_t bFlipFlag);
void App_LcdStrobeControl(void);
void App_WateringTimeCntDown(void);
void App_AutoDeepSleepCnt(void);
void App_Timer0Init(uint16_t u16Period);
void App_UserDataSetDefaultVal(void);
void App_ConvertFlashData2UserData(void);
boolean_t App_IsFlashDataUpdate(void);
void App_ConvertUserData2FlashData(void);
void App_WdtInit(void);
void App_SysInit(void);
void App_SysInitWakeUp(void);
void App_DeepSleepModeEnter(void);


void Rtc_IRQHandler(void)
{
    if(Rtc_GetPridItStatus() == TRUE)
    {
        u8RtcFlag = 1;
        Rtc_ClearPrdfItStatus();             //清除中断标志位
    }
}

void Tim0_IRQHandler(void)
{
    //Timer0 模式0 溢出中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        App_KeyStateChkSet();

        if(enFocusOn > Nothing)
        {
            App_LcdStrobeControl();
        }

        if(0 == u8StopFlag)
        {
            App_WateringTimeCntDown();
        }

        if((enFocusOn == Nothing) && (enKeyState < WaitForRelease) && 0x00 == u8PumpCtrl)
        {
            // 无操作10s进入深度休眠.
            App_AutoDeepSleepCnt();
        }

        Bt_ClearIntFlag(TIM0, BtUevIrq); //中断标志清零
    }
}

void PortD_IRQHandler(void)
{
    if(1 == u8PowerOnFlag)
    {
        if(TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER) ||
            TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE) ||
            TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_SET) ||
            TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_OK) ||
            TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN) ||
            TRUE == Gpio_GetIrqStatus(GPIO_PORT_KEY, GPIO_PIN_KEY_UP))
        {
            enLockStatus = Unlock;
            bPortDIrFlag = TRUE;
            bLcdUpdate = TRUE;
            M0P_LCD->CR0_f.EN = LcdEnable;
            Gpio_SetIO(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL);

            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER, GpioIrqFalling);
            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE, GpioIrqFalling);
            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_SET, GpioIrqFalling);
            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_OK, GpioIrqFalling);
            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN, GpioIrqFalling);
            Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_UP, GpioIrqFalling);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_SET);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_OK);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_UP);
            Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER);
        }
    }
    else
    {
        u8PowerOnFlag = 1;
        enLockStatus = Unlock;
        bPortDIrFlag = TRUE;
        bLcdUpdate = TRUE;
        M0P_LCD->CR0_f.EN = LcdEnable;
        Gpio_SetIO(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL);

        Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER, GpioIrqFalling);
        Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER);
    }
}

int32_t main(void)
{
    static boolean_t bJustWatered = FALSE;

    App_SysInit();

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

    DDL_ZERO_STRUCT(stcRtcTime);
    App_GetRtcTime();

    App_LcdRam_Init(u32LcdRamData);

    unKeyPress.Full = 0x0000;
    u8PowerOnFlag = 1;
    enFocusOn = Nothing;
    enLockStatus = Unlock;
    u32UpDownCnt = 0;
    u8PumpCtrl = 0x00;
    bLcdUpdate = TRUE;
    u16RtcCnt = 0;
    u8RtcFlag = 0;
    bPortDIrFlag = FALSE;
    u16WTPump1 = 0;
    u16WTPump2 = 0;

    //while(Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE) == TRUE);
    Wdt_Start();
    Bt_M0_Run(TIM0);    // Timer0 运行

    while(1)
    {
        if(unKeyPress.Full != 0x0000)    // Key pressed detected
        {
            Wdt_Feed();
            App_KeyHandler();
            unKeyPress.Full = 0x0000;
            u16LcdFlickerCnt = 0;
            u16NoKeyPressedCnt = 0;
        }

        if(Nothing == enFocusOn && u16LcdFlickerCnt != 0)
        {
            u16LcdFlickerCnt = 0;
        }

        if(1 == u8RtcFlag)
        {
            u8RtcFlag = 0;
            Wdt_Feed();

            if(enFocusOn < RtcYear || enFocusOn > RtcMin)
            {
                if(TRUE == App_GetRtcTime())
                {
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    bLcdUpdate = TRUE;

                    if(TRUE == bJustWatered)
                    {
                        bJustWatered = FALSE;
                    }
                }

                if(TRUE == IsTimeToWater(bJustWatered) && 1 == u8PowerOnFlag)
                {
                    bJustWatered = TRUE;
                    u8StopFlag = 0;

                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, ModeAutomatic, TRUE);
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, ModeAutomatic, TRUE, enFocusOn);
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
                                            ModeAutomatic,
                                            TRUE,
                                            enFocusOn);
                    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART] - u8DaysAddUp[u8GroupNum],
                                            ModeAutomatic,
                                            TRUE,
                                            enFocusOn);

                    bLcdUpdate = TRUE;
                    M0P_LCD->CR0_f.EN = LcdEnable;
                    Gpio_SetIO(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL);

                    bPortDIrFlag = FALSE;

                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER, GpioIrqFalling);
                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE, GpioIrqFalling);
                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_SET, GpioIrqFalling);
                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_OK, GpioIrqFalling);
                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN, GpioIrqFalling);
                    Gpio_DisableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_UP, GpioIrqFalling);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_SET);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_OK);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_UP);
                    Gpio_ClearIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER);
                }
            }
        }

        if(1 == u8DeepSleepFlag)
        {
            App_DeepSleepModeEnter();
            App_SysInitWakeUp();
        }
        else
        {
            if(TRUE == bLcdUpdate)
            {
                App_Lcd_Display_Update(u32LcdRamData);
                bLcdUpdate = FALSE;
            }

            App_PumpCtrl();
        }
    }
}

//时钟初始化配置
void App_ClkInit(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;
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
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER, &stcGpioCfg);  // POWER
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE, &stcGpioCfg);   // MODE
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_SET, &stcGpioCfg);    // SET
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_OK, &stcGpioCfg);     // OK
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN, &stcGpioCfg);   // DW
    Gpio_Init(GPIO_PORT_KEY, GPIO_PIN_KEY_UP, &stcGpioCfg);     // UP
}

un_key_type App_KeyDetect(void)
{
    un_key_type unKeyTypeTemp;

    unKeyTypeTemp.Full = 0x0000;

    ///< 检测各按键是否按下(低电平)
    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER))
    {
        unKeyTypeTemp.Power = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE))
    {
        unKeyTypeTemp.Mode = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_SET))
    {
        unKeyTypeTemp.Set = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_OK))
    {
        unKeyTypeTemp.OK = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN))
    {
        unKeyTypeTemp.Down = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(GPIO_PORT_KEY, GPIO_PIN_KEY_UP))
    {
        unKeyTypeTemp.Up = 1;
        return unKeyTypeTemp;
    }

    return unKeyTypeTemp;
}

// 状态机按键消抖, 每10ms检查或设置1次状态
void App_KeyStateChkSet(void)
{
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
                        if(u32Tim0Cnt >= SET_OK_KEY_LONG_PRESS_CNT && FALSE == bPortDIrFlag)
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
                        if(u32Tim0Cnt >= SET_OK_KEY_LONG_PRESS_CNT && FALSE == bPortDIrFlag)
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
                        if(u32Tim0Cnt >= MODE_KEY_LONG_PRESS_CNT && FALSE == bPortDIrFlag)
                        {
                            enKeyState = Update;
                            unKeyPressTemp.Reset = 1;
                        }
                    }
                }
                else if((unKeyPressDetected.Up || unKeyPressDetected.Down) && FALSE == bPortDIrFlag)
                {
                    // 识别到长按上/下键
                    u32UpDownCnt++;
                    enKeyState = Update;
                }

                if(!unKeyPressDetected.Full)
                {
                    if(FALSE == bPortDIrFlag)
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
                    else
                    {
                        bPortDIrFlag = FALSE;
                        enKeyState = Waiting;
                        u32Tim0Cnt = 0;
                        unKeyPressTemp.Full = 0x0000;
                    }
                }
            }
            else
            {
                if(!unKeyPressDetected.Full)
                {
                    if(FALSE == bPortDIrFlag)
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
                    else
                    {
                        bPortDIrFlag = FALSE;
                        enKeyState = Waiting;
                        u32Tim0Cnt = 0;
                        unKeyPressTemp.Full = 0x0000;
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
                     * 在 (20 * KEY_LONG_PRESS_CNT * 10)ms 时达到最快速度.
                     * u32Tim0Cnt = 15 表示每隔 ((KEY_LONG_PRESS_CNT - 15) * 10)ms 就响应一次按键*/
                    if(u32UpDownCnt <= 20)
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
    if(enLockStatus < Lock  && unKeyPress.Power)
    {
        if(0 == u8PowerOnFlag)
        {
            u8PowerOnFlag = 1;
            enLockStatus = Unlock;
        }
        else
        {
            if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
            {
                Rtc_SetTime(&stcRtcTime);
                Rtc_Cmd(TRUE);
            }

            u8PowerOnFlag = 0;
            enLockStatus = LockExceptPowerKey;
            u8DeepSleepFlag = 1;
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Mode)
    {
        if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
        {
            Rtc_SetTime(&stcRtcTime);
            Rtc_Cmd(TRUE);
        }

        enFocusOn = Mode;

        if(ModeAutomatic == enWorkingMode)
        {
            enWorkingMode = ModeManual;
            u8PumpCtrl = 0x00;
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
        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
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
                case Mode:
                    if(Mode == enFocusOn)
                    {
                        Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
                    }
                    enFocusOn = Group;
                    u8StopFlag = 1;
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                    break;

                case Group:
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
                    u8StopFlag = 0;
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

                case RtcYear:
                case RtcMonth:
                case RtcDay:
                case RtcHour:
                case RtcMin:
                    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                    Rtc_SetTime(&stcRtcTime);
                    Rtc_Cmd(TRUE);
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
            if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
            {
                Rtc_SetTime(&stcRtcTime);
                Rtc_Cmd(TRUE);
            }

            u8StopFlag = 1;
            Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
            if(Nothing == enFocusOn)
            {
                enFocusOn = WateringTime;
            }
            else if(Mode == enFocusOn)
            {
                enFocusOn = WateringTime;
                Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
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
                    Rtc_SetTime(&stcRtcTime);
                    Rtc_Cmd(TRUE);
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
                    case Mode:
                        break;

                    case Group:
                        enFocusOn = Channel;
                        if(0 == u8StopFlag)
                        {
                            u8StopFlag = 1;
                            Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                        }
                        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
                        break;

                    case Channel:
                        enFocusOn = WateringTime;
                        Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                                            TRUE,
                                            enFocusOn);
                        if(TRUE == App_IsFlashDataUpdate())
                        {
                            App_ConvertUserData2FlashData();
                            Flash_Manager_Update();
                        }
                        break;

                    case WateringTime:
                        enFocusOn = StartingTimeH;
                        Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                                    (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                                    TRUE,
                                                    enFocusOn);
                        if(TRUE == App_IsFlashDataUpdate())
                        {
                            App_ConvertUserData2FlashData();
                            Flash_Manager_Update();
                        }
                        break;

                    case StartingTimeH:
                        enFocusOn = StartingTimeM;
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                    enWorkingMode,
                                                    TRUE,
                                                    enFocusOn);
                        if(TRUE == App_IsFlashDataUpdate())
                        {
                            App_ConvertUserData2FlashData();
                            Flash_Manager_Update();
                        }
                        break;

                    case StartingTimeM:
                        enFocusOn = DaysApart;
                        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                                    enWorkingMode,
                                                    TRUE,
                                                    enFocusOn);
                        if(TRUE == App_IsFlashDataUpdate())
                        {
                            App_ConvertUserData2FlashData();
                            Flash_Manager_Update();
                        }
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
                        if(TRUE == App_IsFlashDataUpdate())
                        {
                            App_ConvertUserData2FlashData();
                            Flash_Manager_Update();
                        }
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
                    if(TRUE == App_IsFlashDataUpdate())
                    {
                        App_ConvertUserData2FlashData();
                        Flash_Manager_Update();
                    }
                }
                else
                {
                    enFocusOn = Nothing;
                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
                    u8StopFlag = !u8StopFlag;
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);

                    if(1 == u8StopFlag || 0 == u16WateringTimeManual[u8ChannelManual])
                    {
                        u8PumpCtrl = 0x00;
                    }
                    else
                    {
                        if(0 == u8ChannelManual)
                        {
                            u8PumpCtrl = 0x01;
                        }
                        else
                        {
                            u8PumpCtrl = 0x10;
                        }
                    }
                }
            }
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Down)
    {
        switch(enFocusOn)
        {
            case Nothing:
            case Group:
            case Mode:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(enFocusOn != Group)
                    {
                        enFocusOn = Group;
                    }

                    if(u8GroupNum == GROUP_NUM_MIN)
                    {
                        u8GroupNum = GROUP_NUM_MAX - 1;
                    }
                    else
                    {
                        --u8GroupNum;
                    }
                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
                    // 组数变化了, 通道、浇水时长、启动时间和间隔天数也需要跟着变化
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
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
                    u8StopFlag = 1;
                    u8PumpCtrl = 0x00;
                    if(0 == u8ChannelManual)
                    {
                        u16WateringTimeManual[0] = stcFlashManager.u32FlashData[2] |
                                            ((stcFlashManager.u32FlashData[3] & 0x03) << 8);
                    }
                    else
                    {
                        u16WateringTimeManual[1] = ((stcFlashManager.u32FlashData[3] & 0xC0) >> 6) |
                                            (stcFlashManager.u32FlashData[4] << 2);
                    }

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
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
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
                if(stcRtcTime.u8Year == DEC2BCD(0))
                {
                    stcRtcTime.u8Year = DEC2BCD(99);
                }
                else
                {
                    stcRtcTime.u8Year = DEC2BCD(BCD2DEC(stcRtcTime.u8Year) - 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMonth:
                if(stcRtcTime.u8Month <= DEC2BCD(1))
                {
                    stcRtcTime.u8Month = DEC2BCD(12);
                }
                else
                {
                    stcRtcTime.u8Month = DEC2BCD(BCD2DEC(stcRtcTime.u8Month) - 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcDay:
                if(stcRtcTime.u8Day <= DEC2BCD(1))
                {
                    stcRtcTime.u8Day = DEC2BCD(App_DaysInAMonth(&stcRtcTime));
                }
                else
                {
                    stcRtcTime.u8Day = DEC2BCD(BCD2DEC(stcRtcTime.u8Day) - 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcHour:
                if(stcRtcTime.u8Hour == DEC2BCD(0))
                {
                    stcRtcTime.u8Hour = DEC2BCD(23);
                }
                else
                {
                    stcRtcTime.u8Hour = DEC2BCD(BCD2DEC(stcRtcTime.u8Hour) - 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMin:
                if(stcRtcTime.u8Minute == DEC2BCD(0))
                {
                    stcRtcTime.u8Minute = DEC2BCD(59);
                }
                else
                {
                    stcRtcTime.u8Minute = DEC2BCD(BCD2DEC(stcRtcTime.u8Minute) - 1);
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
            case Group:
            case Mode:
                if(ModeAutomatic == enWorkingMode)
                {
                    if(enFocusOn != Group)
                    {
                        enFocusOn = Group;
                    }

                    if(++u8GroupNum > GROUP_NUM_MAX - 1)
                    {
                        u8GroupNum = GROUP_NUM_MIN;
                    }
                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, TRUE);
                    // 组数变化了, 通道、浇水时长、启动时间和间隔天数也需要跟着变化
                    Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
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
                    u8StopFlag = 1;
                    u8PumpCtrl = 0x00;
                    if(0 == u8ChannelManual)
                    {
                        u16WateringTimeManual[0] = stcFlashManager.u32FlashData[2] |
                                            ((stcFlashManager.u32FlashData[3] & 0x03) << 8);
                    }
                    else
                    {
                        u16WateringTimeManual[1] = ((stcFlashManager.u32FlashData[3] & 0xC0) >> 6) |
                                            (stcFlashManager.u32FlashData[4] << 2);
                    }

                    if(++u8ChannelManual > 1)
                    {
                        u8ChannelManual = 0;
                    }
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, u8ChannelManual + 1, TRUE, enFocusOn);
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                    Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
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
                if(stcRtcTime.u8Year >= DEC2BCD(99))
                {
                    stcRtcTime.u8Year = DEC2BCD(0);
                }
                else
                {
                    stcRtcTime.u8Year = DEC2BCD(BCD2DEC(stcRtcTime.u8Year) + 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMonth:
                if(stcRtcTime.u8Month >= DEC2BCD(12))
                {
                    stcRtcTime.u8Month = DEC2BCD(1);
                }
                else
                {
                    stcRtcTime.u8Month = DEC2BCD(BCD2DEC(stcRtcTime.u8Month) + 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcDay:
                if(stcRtcTime.u8Day >= DEC2BCD(App_DaysInAMonth(&stcRtcTime)))
                {
                    stcRtcTime.u8Day = DEC2BCD(1);
                }
                else
                {
                    stcRtcTime.u8Day = DEC2BCD(BCD2DEC(stcRtcTime.u8Day) + 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcHour:
                if(stcRtcTime.u8Hour >= DEC2BCD(23))
                {
                    stcRtcTime.u8Hour = DEC2BCD(0);
                }
                else
                {
                    stcRtcTime.u8Hour = DEC2BCD(BCD2DEC(stcRtcTime.u8Hour) + 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            case RtcMin:
                if(stcRtcTime.u8Minute >= DEC2BCD(59))
                {
                    stcRtcTime.u8Minute = DEC2BCD(0);
                }
                else
                {
                    stcRtcTime.u8Minute = DEC2BCD(BCD2DEC(stcRtcTime.u8Minute) + 1);
                }
                Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, TRUE, enFocusOn);
                break;

            default:
                break;
        }
    }

    if(enLockStatus != LockExceptPowerKey && unKeyPress.Lock)
    {
        enFocusOn = Nothing;
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
            Rtc_Cmd(FALSE);
            stcRtcTime.u8Second = 0;
        }
    }

    if(Unlock == enLockStatus && unKeyPress.Reset)
    {
        Flash_Manager_Erase_All_Data();
        delay1ms(10);
        // Reset MCU
        NVIC_SystemReset();
    }

    bLcdUpdate = TRUE;
}

void App_RtcInit(void)
{
    stc_rtc_initstruct_t RtcInitStruct;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc, TRUE);  //RTC模块时钟打开

    RtcInitStruct.rtcAmpm = RtcPm;                          //24小时制
    RtcInitStruct.rtcClksrc = RtcClkXtl;                    //外部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;            //周期中断类型PRDX
    RtcInitStruct.rtcPrdsel.rtcPrdx = 0x3B;                 //周期中断事件间隔, 0x3B -> 30s
    RtcInitStruct.rtcTime.u8Second = 0x00;
    RtcInitStruct.rtcTime.u8Minute = 0x00;
    RtcInitStruct.rtcTime.u8Hour   = 0x00;
    RtcInitStruct.rtcTime.u8Day    = 0x01;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x01;
    RtcInitStruct.rtcTime.u8Month  = 0x01;
    RtcInitStruct.rtcTime.u8Year   = 0x21;
    RtcInitStruct.rtcCompen = RtcCompenEnable;
    RtcInitStruct.rtcCompValue = 0;                         //补偿值根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                                     //使能闹钟中断
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);                  //使能RTC中断向量
    Rtc_Cmd(TRUE);                                          //使能RTC开始计数
    Rtc_StartWait();                                        //启动RTC计数，如果要立即切换到低功耗，需要执行此函数
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

    if(BCD2DEC(time->u8Month) == 2)
    {
        u8DaysInAMonth = Get_Month2_Day(BCD2DEC(time->u8Year));
    }
    else
    {
        if(BCD2DEC(time->u8Month) == 1 ||
            BCD2DEC(time->u8Month) == 3 ||
            BCD2DEC(time->u8Month) == 5 ||
            BCD2DEC(time->u8Month) == 7 ||
            BCD2DEC(time->u8Month) == 8 ||
            BCD2DEC(time->u8Month) == 10 ||
            BCD2DEC(time->u8Month) == 12)
        {
            u8DaysInAMonth = 31;
        }
        else
        {
            u8DaysInAMonth = 30;
        }
    }

    return u8DaysInAMonth;
}

boolean_t IsTimeToWater(boolean_t bJustWatered)
{
    uint8_t u8GroupIdxTmp = 0;

    if(ModeManual == enWorkingMode)
    {
        return FALSE;
    }

    u16RtcCnt++;

    for(u8GroupIdxTmp = 0; u8GroupIdxTmp < GROUP_NUM_MAX; u8GroupIdxTmp++)
    {
        if(u16RtcCnt >= 2880)    // 24 * 60 * 60 / 30 = 2880
        {
            u16RtcCnt = 0;
            u8DaysAddUp[u8GroupIdxTmp]++;
        }

        if(u8DaysAddUp[u8GroupIdxTmp] >= u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_DAYSAPART] &&
            u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME] != 0)
        {
            if(stcRtcTime.u8Hour == DEC2BCD(u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_STARTHOUR]) &&
                stcRtcTime.u8Minute == DEC2BCD(u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_STARTMIN]) &&
                FALSE == bJustWatered && 1 == u8PowerOnFlag)
            {
                /* 如果有多组是同一时间同一通道进行浇水, 那么激活浇水时间长的那一组
                    如果有多组是同一时间不同通道进行浇水, 那么两路水泵都工作, 并且显示胶水时间长的那一组 */
                if(0 == u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_CHANNEL])
                {
                    u8PumpCtrl |= 0x01;

                    if(u16WTPump1 < u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME])
                    {
                        u16WTPump1 = u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME];

                        if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] <
                            u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME])
                        {
                            u8GroupNum = u8GroupIdxTmp;
                        }
                    }
                }
                else
                {
                    u8PumpCtrl |= 0x10;

                    if(u16WTPump2 < u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME])
                    {
                        u16WTPump2 = u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME];

                        if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] <
                            u32GroupDataAuto[u8GroupIdxTmp][AUTOMODE_GROUP_DATA_WATER_TIME])
                        {
                            u8GroupNum = u8GroupIdxTmp;
                        }
                    }
                }
            }
        }

        if(u8DaysAddUp[u8GroupIdxTmp] > 99)
        {
            u8DaysAddUp[u8GroupIdxTmp] = 0;
        }
    }

    if(0x00 == u8PumpCtrl)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

void App_LcdPortInit(void)
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

void App_LcdInit(void)
{
    stc_lcd_cfg_t LcdInitStruct;
    stc_lcd_segcom_t LcdSegCom;

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

void App_LcdBlInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;

    ///< GPIO IO LCD BL_ON 端口初始化
    Gpio_Init(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL, &stcGpioCfg);
}

void App_LcdRam_Init(un_Ram_Data* pu32Data)
{
    uint8_t u8Idx = 0;

    for(u8Idx = 0; u8Idx <= LCDRAM_INDEX_MAX; u8Idx++)
    {
        pu32Data[u8Idx].u32_dis = 0x00000000;
    }

    Lcd_D61593A_GenRam_WorkingMode(pu32Data, enWorkingMode, TRUE);
    Lcd_D61593A_GenRam_GroupNum(pu32Data, u8GroupNum + 1, enWorkingMode, TRUE, enFocusOn);
    if(ModeAutomatic == enWorkingMode)
    {
        Lcd_D61593A_GenRam_Channel(pu32Data,
                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                            TRUE,
                            enFocusOn);
        Lcd_D61593A_GenRam_Watering_Time(pu32Data,
                                    (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                    TRUE,
                                    enFocusOn);
    }
    else
    {
        Lcd_D61593A_GenRam_Channel(pu32Data, u8ChannelManual + 1, TRUE, enFocusOn);
        Lcd_D61593A_GenRam_Watering_Time(pu32Data, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
    }
    Lcd_D61593A_GenRam_Starting_Time(pu32Data,
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                enWorkingMode,
                                TRUE,
                                enFocusOn);
    Lcd_D61593A_GenRam_Days_Apart(pu32Data,
                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                            enWorkingMode,
                            TRUE,
                            enFocusOn);
    Lcd_D61593A_GenRam_Smart1(pu32Data, SmartModeDry, FALSE);
    Lcd_D61593A_GenRam_Smart2(pu32Data, SmartModeWet, FALSE);
    Lcd_D61593A_GenRam_Stop(pu32Data, u8StopFlag);
    Lcd_D61593A_GenRam_Lock_Icon(pu32Data, enLockStatus, TRUE);
    Lcd_D61593A_GenRam_Wifi_Icon(pu32Data, WifiSignalStrong, FALSE);
    Lcd_D61593A_GenRam_Battery_Icon(pu32Data, BatteryPercent100, TRUE);
    Lcd_D61593A_GenRam_Date_And_Time(pu32Data, &stcRtcTime, TRUE, enFocusOn);
}

void App_Lcd_Display_Update(un_Ram_Data* pu32Data)
{
    uint8_t u8Idx = 0;

    for(u8Idx = 0; u8Idx <= LCDRAM_INDEX_MAX; u8Idx++)
    {
        Lcd_WriteRam(u8Idx, pu32Data[u8Idx].u32_dis);
    }
}

void App_PumpInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;

    Gpio_Init(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1, &stcGpioCfg);
    Gpio_Init(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2, &stcGpioCfg);
}

void App_PumpCtrl(void)
{
    switch (u8PumpCtrl)
    {
    case 0x00:
        Gpio_ClrIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
        Gpio_ClrIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);
        break;

    case 0x01:
        Gpio_SetIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
        Gpio_ClrIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);
        break;

    case 0x10:
        Gpio_ClrIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
        Gpio_SetIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);
        break;

    case 0x11:
        Gpio_SetIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
        Gpio_SetIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);
        break;

    default:
        Gpio_ClrIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
        Gpio_ClrIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);
        break;
    }
}

/**
 * \brief   更新 LCD 控件闪烁的 Ram
 *
 * \param   [in]  bFlipFlag  控制某个空间亮或灭
 *
 * \retval  无
 */
void App_LcdRamFlipCtrl(boolean_t bFlipFlag)
{
    switch(enFocusOn)
    {
    case Mode:
        Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, enWorkingMode, bFlipFlag);
        break;

    case Group:
        Lcd_D61593A_GenRam_GroupNum(u32LcdRamData, u8GroupNum + 1, enWorkingMode, bFlipFlag, enFocusOn);
        break;

    case Channel:
        Lcd_D61593A_GenRam_Channel(u32LcdRamData,
                            (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_CHANNEL] + 1,
                            bFlipFlag,
                            enFocusOn);
        break;

    case WateringTime:
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
        break;

    case StartingTimeH:
    case StartingTimeM:
        Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData,
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTHOUR],
                                    (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_STARTMIN],
                                    enWorkingMode,
                                    bFlipFlag,
                                    enFocusOn);
        break;

    case DaysApart:
        Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData,
                                (uint8_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_DAYSAPART],
                                enWorkingMode,
                                bFlipFlag,
                                enFocusOn);
        break;

    case RtcYear:
    case RtcMonth:
    case RtcDay:
    case RtcHour:
    case RtcMin:
        Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, &stcRtcTime, bFlipFlag, enFocusOn);
        break;

    default:
        break;
    }
}

// 控制 LCD 部分控件闪烁
void App_LcdStrobeControl(void)
{
    static uint8_t u8LcdContentSDCnt = 0;
    static boolean_t bFlipFlag = TRUE;

    u16LcdFlickerCnt++;

    if(u16LcdFlickerCnt < LCD_CONTENT_FLASH_DURATION ||
        (u16LcdFlickerCnt >= LCD_CONTENT_FLASH_DURATION && FALSE == bFlipFlag))
    {
        if(++u8LcdContentSDCnt > LCD_CONTENT_FLASH_FREQ)
        {
            u8LcdContentSDCnt = 0;

            bFlipFlag = !bFlipFlag;

            if(enKeyState > Waiting)
            {
                // 正在处理按键时不闪烁
                bFlipFlag = TRUE;
            }

            App_LcdRamFlipCtrl(bFlipFlag);

            if((enFocusOn > Nothing) && (enKeyState < WaitForRelease))
            {
                // 当焦点处于会闪烁的控件并且没有处理按键时, 刷新LCD显示.
                bLcdUpdate = TRUE;
            }
        }
    }
    else
    {
        if(enFocusOn >= RtcYear && enFocusOn <= RtcMin)
        {
            Rtc_SetTime(&stcRtcTime);
            Rtc_Cmd(TRUE);
        }
        enFocusOn = Nothing;
    }
}

void App_WateringTimeCntDown(void)
{
    static uint8_t u8Tim0Cnt = 0;

    if(0x00 != u8PumpCtrl)
    {
        if(++u8Tim0Cnt > TIMER0_CNT_WATER_TIME)
        {
            u8Tim0Cnt = 0;
            Wdt_Feed();
            // 每隔1s, 浇水倒计时变量减1
            if(ModeAutomatic == enWorkingMode)
            {
                if(u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] > 0)
                {
                    u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME]--;
                }

                if(0 == u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME])
                {
                    //u8PumpCtrl = 0x00;
                    u16NoKeyPressedCnt = 0;

                    u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME] = ((stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupNum] & 0xC0) >> 6) |
                        (stcFlashManager.u32FlashData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupNum] << 2);
                }

                Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData,
                                        (uint16_t)u32GroupDataAuto[u8GroupNum][AUTOMODE_GROUP_DATA_WATER_TIME],
                                        TRUE,
                                        enFocusOn);
                bLcdUpdate = TRUE;

                if(u16WTPump1 > 0)
                {
                    u16WTPump1--;
                }
                if(0 == u16WTPump1)
                {
                    u8PumpCtrl &= 0x10;
                }

                if(u16WTPump2 > 0)
                {
                    u16WTPump2--;
                }
                if(0 == u16WTPump2)
                {
                    u8PumpCtrl &= 0x01;
                }
            }
            else
            {
                if(u16WateringTimeManual[u8ChannelManual] > 0)
                {
                    u16WateringTimeManual[u8ChannelManual]--;
                }

                if(0 == u16WateringTimeManual[u8ChannelManual])
                {
                    u8PumpCtrl = 0x00;
                    u16NoKeyPressedCnt = 0;
                    u8StopFlag = 1;

                    if(0 == u8ChannelManual)
                    {
                        u16WateringTimeManual[0] = stcFlashManager.u32FlashData[2] |
                                            ((stcFlashManager.u32FlashData[3] & 0x03) << 8);
                    }
                    else
                    {
                        u16WateringTimeManual[1] = ((stcFlashManager.u32FlashData[3] & 0xC0) >> 6) |
                                            (stcFlashManager.u32FlashData[4] << 2);
                    }
                }

                Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, u16WateringTimeManual[u8ChannelManual], TRUE, enFocusOn);
                Lcd_D61593A_GenRam_Stop(u32LcdRamData, u8StopFlag);
                bLcdUpdate = TRUE;
            }
        }
    }
}

void App_AutoDeepSleepCnt(void)
{
    u16NoKeyPressedCnt++;
    if(u16NoKeyPressedCnt >= AUTO_DEEP_SLEEP_CNT)
    {
        u16NoKeyPressedCnt = 0;
        if(ModeAutomatic == enWorkingMode)
        {
            u8DeepSleepFlag = 1;
        }
    }
}

/******************************************************************************
 ** \brief  Timer0配置初始化
 ** 周期 = u16Period*(1/内部时钟频率)*256
 ** \return 无
 ******************************************************************************/
void App_Timer0Init(uint16_t u16Period)
{
    uint16_t            u16ArrValue;
    uint16_t            u16CntValue;
    stc_bt_mode0_cfg_t  stcBtBaseCfg;

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

    Bt_ClearIntFlag(TIM0, BtUevIrq);                         //清中断标志
    Bt_Mode0_EnableIrq(TIM0);                               //使能TIM0中断(模式0时只有一个中断)
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //TIM0中断使能
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
        u8DaysAddUp[u8GroupIdx] = 0;
    }

    App_ConvertUserData2FlashData();
}

void App_ConvertFlashData2UserData(void)
{
    uint8_t u8GroupIdx = 0;

    u8GroupNum = stcFlashManager.u32FlashData[1] & 0x0F;
    enWorkingMode = (stcFlashManager.u32FlashData[1] & 0x10) >> 4;
    u8StopFlag = (stcFlashManager.u32FlashData[1] & 0x20) >> 5;
    u8ChannelManual = (stcFlashManager.u32FlashData[1] & 0x40) >> 6;
    u16WateringTimeManual[0] = stcFlashManager.u32FlashData[2] |
                                ((stcFlashManager.u32FlashData[3] & 0x03) << 8);
    u16WateringTimeManual[1] = ((stcFlashManager.u32FlashData[3] & 0xC0) >> 6) |
                                (stcFlashManager.u32FlashData[4] << 2);

    for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
    {
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART]
            = stcFlashManager.u32FlashData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x7F;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL]
            = (stcFlashManager.u32FlashData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x80) >> 7;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR]
            = stcFlashManager.u32FlashData[6 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx];
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN]
            = stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x3F;
        u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME]
            = ((stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0xC0) >> 6) |
                (stcFlashManager.u32FlashData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] << 2);
        u8DaysAddUp[u8GroupIdx] = 0;
    }
}

/**
 * \brief   检测当前UI设置的数据是否需要更新到Flash中, 避免对Flash做不必要的插写
 *
 * \param   无
 *
 * \retval  TRUE: 当前UI设置的数据和Flash中不一样, 需要更新到Flash中
 *          FALSE: 当前UI设置的数据和Flash中一样, 不需要更新到Flash中
 */
boolean_t App_IsFlashDataUpdate(void)
{
    uint8_t u8GroupIdx = 0;

    if(ModeAutomatic == enWorkingMode)
    {
        if(u8GroupNum != (stcFlashManager.u32FlashData[1] & 0x0F))
            return TRUE;

        for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
        {
            if(u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART] !=
                (stcFlashManager.u32FlashData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x7F))
                return TRUE;

            if(u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL] !=
                ((stcFlashManager.u32FlashData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x80) >> 7))
                return TRUE;

            if(u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR] !=
                stcFlashManager.u32FlashData[6 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx])
                return TRUE;

            if(u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN] !=
                (stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0x3F))
                return TRUE;

            if(u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] !=
                (((stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] & 0xC0) >> 6) |
                (stcFlashManager.u32FlashData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx] << 2)))
                return TRUE;
        }
    }
    else
    {
        if(u8ChannelManual != ((stcFlashManager.u32FlashData[1] & 0x40) >> 6))
            return TRUE;

        if(u16WateringTimeManual[0] != (stcFlashManager.u32FlashData[2] | ((stcFlashManager.u32FlashData[3] & 0x03) << 8)))
            return TRUE;

        if(u16WateringTimeManual[1] != (((stcFlashManager.u32FlashData[3] & 0xC0) >> 6) | (stcFlashManager.u32FlashData[4] << 2)))
            return TRUE;
    }

    return FALSE;
}

void App_ConvertUserData2FlashData(void)
{
    uint8_t u8GroupIdx = 0;

    stcFlashManager.u32FlashData[0] = FLASH_DATA_START_CODE;
    stcFlashManager.u32FlashData[1] = (u8GroupNum | (enWorkingMode << 4) | (u8StopFlag << 5) | (u8ChannelManual << 6)) & 0x7F;
    stcFlashManager.u32FlashData[2] = (uint8_t)(u16WateringTimeManual[0] & 0x00FF);
    stcFlashManager.u32FlashData[3] = (uint8_t)(((u16WateringTimeManual[0] & 0x0300) >> 8) |
                                            ((u16WateringTimeManual[1] & 0x0003) << 6)) & 0xC3;
    stcFlashManager.u32FlashData[4] = (uint8_t)((u16WateringTimeManual[1] & 0x03FC) >> 2);

    for(u8GroupIdx = 0; u8GroupIdx < GROUP_NUM_MAX; u8GroupIdx++)
    {
        stcFlashManager.u32FlashData[5 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_DAYSAPART] & 0x7F) |
                ((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_CHANNEL] & 0x01) << 7));
        stcFlashManager.u32FlashData[6 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTHOUR];
        stcFlashManager.u32FlashData[7 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = ((uint8_t)u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_STARTMIN] & 0x3F) |
                ((uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0x03) << 6));
        stcFlashManager.u32FlashData[8 + (AUTOMODE_GROUP_DATA_ELEMENT_MAX - 1) * u8GroupIdx]
            = (uint8_t)((u32GroupDataAuto[u8GroupIdx][AUTOMODE_GROUP_DATA_WATER_TIME] & 0x03FC) >> 2);
    }

    stcFlashManager.u32FlashData[FLASH_MANAGER_DATA_LEN - 1] = Flash_Manager_Data_BCC_Checksum(stcFlashManager.u32FlashData, FLASH_MANAGER_DATA_LEN);
}

void App_WdtInit(void)
{
    ///< 开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt, TRUE);
    ///< WDT 初始化
    Wdt_Init(WdtResetEn, WdtT52s4);
}

void App_SysInit(void)
{
    App_ClkInit();

    App_Timer0Init(160); //周期 = 160*(1/(4*1024)*256 = 10ms

    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);   // 使能RCL时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);        // 配置内部低速时钟频率为32.768kHz

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd, TRUE);  // 开启LCD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); // 开启GPIO外设时钟
    App_LcdPortInit();
    App_LcdInit();
    App_LcdBlInit();
    Gpio_SetIO(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL);
    Lcd_ClearDisp();

    App_RtcInit();
    App_WdtInit();
    App_KeyInit();
    App_PumpInit();
    Gpio_ClrIO(GPIO_PORT_PUMP_1, GPIO_PIN_PUMP_1);
    Gpio_ClrIO(GPIO_PORT_PUMP_2, GPIO_PIN_PUMP_2);

    ///< 确保初始化正确执行后方能进行FLASH编程操作，FLASH初始化（编程时间,休眠模式配置）
    while(Ok != Flash_Init(1, TRUE))
    {
        ;
    }
}

void App_SysInitWakeUp(void)
{
    u16NoKeyPressedCnt = 0;
    u8DeepSleepFlag = 0;

    App_LcdPortInit();

    // 下面两行顺序不能更换. 如果休眠前LCD有控件处于闪烁(灭)的状态, 那么唤醒后强制显示该控件
    App_LcdRamFlipCtrl(TRUE);
    enFocusOn = Nothing;

    Lcd_ClearDisp();
    App_LcdBlInit();
    Wdt_Feed();
    App_PumpInit();
    Bt_M0_Run(TIM0);
}

void App_DeepSleepModeEnter(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); // 打开GPIO外设时钟门控
    Sysctrl_SetFunc(SysctrlSWDUseIOEn, TRUE);   //swd as gpio

    M0P_LCD->CR0_f.EN = LcdDisable;
    Lcd_ClearDisp();
    u16NoKeyPressedCnt = 0;
    Bt_M0_Stop(TIM0);

    // PC14, PC15为RTC晶振输入脚, 进入深度休眠前不改变其状态
    // XTLI和XTLO两个口保持模拟, 其它端口配置为数字(0为数字)
    M0P_GPIO->PAADS = 0;
    M0P_GPIO->PBADS = 0;
    M0P_GPIO->PCADS = 0xC000;
    M0P_GPIO->PDADS = 0;

    // XTLI和XTLO两个口保持输出, 其它端口配置为输入
    M0P_GPIO->PADIR = 0XFFFF;
    M0P_GPIO->PBDIR = 0XFFFF;
    M0P_GPIO->PCDIR = 0X3FFF;
    M0P_GPIO->PDDIR = 0XFFFF;

    // XTLI和XTLO两个口保持, 其它端口配置为端口下拉(1为下拉)
    M0P_GPIO->PAPD = 0xFFFF;
    M0P_GPIO->PBPD = 0xFFFF;
    M0P_GPIO->PCPD = 0x3FFF;
    M0P_GPIO->PDPD = 0xFF0C;

    // 端口清零
    M0P_GPIO->PABCLR=0XFFFF;
    M0P_GPIO->PBBCLR=0XFFFF;
    M0P_GPIO->PCBCLR=0X3FFF;
    M0P_GPIO->PDBCLR=0XFF0C;

    Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_POWER, GpioIrqFalling);
    if(1 == u8PowerOnFlag)
    {
        Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_MODE, GpioIrqFalling);
        Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_SET, GpioIrqFalling);
        Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_OK, GpioIrqFalling);
        Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_DOWN, GpioIrqFalling);
        Gpio_EnableIrq(GPIO_PORT_KEY, GPIO_PIN_KEY_UP, GpioIrqFalling);
    }
    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);

    Rtc_StartWait();
    delay1ms(10);
    Lpm_GotoDeepSleep(FALSE);
    delay1ms(10);
}
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
