/******************************************************************************
* Copyright (C) 2017, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
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
#include "lcd_D61593A.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 *****************************************************************************/
 #define MIN_KEY_COUNT 0

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 *****************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 *****************************************************************************/
typedef struct
{
    en_working_mode_t enWorkingMode;
}stc_status_storage_t;

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
        unsigned char Reserved1 :1;
        unsigned char Reserved2 :1;
    };
}un_key_type;

typedef enum
{
    Waiting,
    Detected,
    WaitForRelease,
    Update
}en_key_states;


/******************************************************************************
 * Local function prototypes ('static')
 *****************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 *****************************************************************************/
static stc_status_storage_t stcStatusVal;
un_key_type unKeyPress;

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
void App_PortCfg(void);
void App_LcdCfg(void);
void App_LcdRam_Init(un_Ram_Data* pu32Data);
void App_Lcd_Display_Update(un_Ram_Data* pu32Data);
void App_Timer0Cfg(uint16_t u16Period);


int32_t main(void)
{
    static uint8_t j = 0;
    un_Ram_Data u32LcdRamData[LCDRAM_INDEX_MAX];

    App_LcdRam_Init(u32LcdRamData);
    DDL_ZERO_STRUCT(stcStatusVal);

    unKeyPress.Full = 0x00;

    App_ClkInit(); //设置RCH为4MHz内部时钟初始化配置
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);            ///< 使能RCL时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);                ///< 配置内部低速时钟频率为32.768kHz

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd, TRUE);   ///< 开启LCD时钟

    App_KeyInit();
    App_PortCfg();               ///< LCD端口配置
    App_LcdCfg();                ///< LCD模块配置

    Lcd_ClearDisp();             ///< 清屏

    Lcd_D61593A_GenRam_Channel(u32LcdRamData, 0, TRUE);
    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, 215, TRUE);
    Lcd_D61593A_GenRam_Sets(u32LcdRamData, 1, TRUE);
    Lcd_D61593A_GenRam_Smart1(u32LcdRamData, SmartModeDry, TRUE);
    Lcd_D61593A_GenRam_Smart2(u32LcdRamData, SmartModeWet, TRUE);
    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, ModeAutomatic, TRUE);
    Lcd_D61593A_GenRam_Starting_Time(u32LcdRamData, 4, 30, ModeAutomatic, TRUE);
    Lcd_D61593A_GenRam_Days_Apart(u32LcdRamData, 99, ModeAutomatic, TRUE);
    Lcd_D61593A_GenRam_Stop(u32LcdRamData, TRUE);
    Lcd_D61593A_GenRam_Lock_Icon(u32LcdRamData, Unlock, TRUE);
    Lcd_D61593A_GenRam_Wifi_Icon(u32LcdRamData, WifiSignalStrong, TRUE);
    Lcd_D61593A_GenRam_Battery_Icon(u32LcdRamData, BatteryPercent100, TRUE);
    Lcd_D61593A_GenRam_Date_And_Time(u32LcdRamData, 2021, 2, 10, 0, 11, TRUE);

    App_Lcd_Display_Update(u32LcdRamData);

    App_Timer0Cfg(160);   //周期 = 160*(1/(4*1024)*256 = 10ms
    Bt_M0_Run(TIM0);    // Timer0 运行

    while(1)
    {
        if(unKeyPress.Full != 0x00)    // Key pressed detected
        {
            if(unKeyPress.Power)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, 0, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_Channel(u32LcdRamData, 0, TRUE);
                }
            }

            if(unKeyPress.Mode)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, 215, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_Watering_Time(u32LcdRamData, 215, TRUE);
                }
            }

            if(unKeyPress.Set)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_Sets(u32LcdRamData, 1, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_Sets(u32LcdRamData, 1, TRUE);
                }
            }

            if(unKeyPress.OK)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_Smart1(u32LcdRamData, SmartModeDry, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_Smart1(u32LcdRamData, SmartModeDry, TRUE);
                }
            }

            if(unKeyPress.Down)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_Smart2(u32LcdRamData, SmartModeWet, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_Smart2(u32LcdRamData, SmartModeWet, TRUE);
                }
            }

            if(unKeyPress.Up)
            {
                if(0 == j)
                {
                    j = 1;
                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, ModeAutomatic, FALSE);
                }
                else
                {
                    j = 0;
                    Lcd_D61593A_GenRam_WorkingMode(u32LcdRamData, ModeAutomatic, TRUE);
                }
            }

            unKeyPress.Full = 0x00;
            App_Lcd_Display_Update(u32LcdRamData);
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
    Gpio_Init(STK_USER_PORT, STK_XTHI_PIN, &stcGpioCfg);        // POWER
    Gpio_Init(STK_USER_PORT, STK_XTHO_PIN, &stcGpioCfg);        // MODE
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);        // SET
    Gpio_Init(STK_USER_PORT, STK_LCD_SEG4_PIN, &stcGpioCfg);    // OK
    Gpio_Init(STK_USER_PORT, STK_LED_PIN, &stcGpioCfg);         // DW
    Gpio_Init(STK_USER_PORT, STK_LCD_SEG3_PIN, &stcGpioCfg);    // UP
}

un_key_type App_KeyDetect(void)
{
    un_key_type unKeyTypeTemp;

    unKeyTypeTemp.Full = 0x00;

    ///< 检测各按键是否按下(低电平)
    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_XTHI_PIN))
    {
        unKeyTypeTemp.Power = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_XTHO_PIN))
    {
        unKeyTypeTemp.Mode = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN))
    {
        unKeyTypeTemp.Set = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_LCD_SEG4_PIN))
    {
        unKeyTypeTemp.OK = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_LED_PIN))
    {
        unKeyTypeTemp.Down = 1;
        return unKeyTypeTemp;
    }

    if(FALSE == Gpio_GetInputIO(STK_USER_PORT, STK_LCD_SEG3_PIN))
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
                unKeyPressTemp = unKeyPressDetected;  //Record the temporary value
            }
            break;
        case Detected:
            if(unKeyPressTemp.Full == unKeyPressDetected.Full)
            {
                ++u8Tim0Cnt;    //Guarded state action, if same value increase key count
                if(u8Tim0Cnt > MIN_KEY_COUNT)
                {
                    enKeyState = WaitForRelease;   //guarded state transition
                }
            }
            else
            {
                enKeyState = Waiting;   //state transition
            }
            break;
        case WaitForRelease:
            if(!unKeyPressDetected.Full)
            {
                enKeyState = Update;   //state transition when all buttons released
            }
            break;
        case Update:
            unKeyPress = unKeyPressTemp;    //state action    HERE the Key value is updated
            enKeyState = Waiting;           //state transition
            u8Tim0Cnt = 0;                  //state action
            unKeyPressTemp.Full=0;          //state action
            break;
        default:
            enKeyState = Waiting;
            u8Tim0Cnt = 0;
            unKeyPressTemp.Full = 0x00;
            unKeyPress.Full = 0x00;
            break;
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

        Bt_ClearIntFlag(TIM0, BtUevIrq); //中断标志清零
    }
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
