/*******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
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
/******************************************************************************/
/** \file general_define.h
 **
 ** General define (#define)
 **
 **   - 2021-03-03
 **
 *****************************************************************************/

#ifndef  __GENERAL_DEFINE_H__
#define  __GENERAL_DEFINE_H__

#define GROUP_NUM_MIN       1
#define GROUP_NUM_MAX       6
#define GROUP_NUM_DEFAULT   0

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
 ** \brief 模式选择
 *****************************************************************************/
typedef enum
{
    ModeAutomatic = 0,
    ModeManual = 1,
}en_working_mode_t;

/******************************************************************************
 ** \brief Lock and Unlock
 *****************************************************************************/
typedef enum
{
    Unlock = 0u,
    LockExceptPowerKey = 1u,
    Lock = 2u
}en_lock_status_t;

/******************************************************************************
 ** \brief Wifi 信号强弱
 *****************************************************************************/
typedef enum
{
    WifiSignalWeek = 0,
    WifiSignalStrong = 1,
}en_wifi_signal_strength_t;

/******************************************************************************
 ** \brief 剩余电量
 *****************************************************************************/
typedef enum
{
    BatteryPercent0 = 0,
    BatteryPercent25 = 1,
    BatteryPercent50 = 2,
    BatteryPercent75 = 3,
    BatteryPercent100 = 4,
}en_remaining_battery_t;

typedef enum
{
    Nothing = 0u,
    Channel = 1u,
    WateringTime = 2u,
    StartingTimeH = 3u,
    StartingTimeM = 4u,
    DaysApart = 5u,
    RtcYear = 6u,
    RtcMonth = 7u,
    RtcDay = 8u,
    RtcHour = 9u,
    RtcMin = 10u
}en_focus_on;

#endif /* __GENERAL_DEFINE_H__ */

/*******************************************************************************
 * EOF (not truncated)                                                        
 ******************************************************************************/
