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

#define GROUP_NUM_MIN   0
#define GROUP_NUM_MAX   6
#define AUTOMODE_GROUP_DATA_ELEMENT_MAX 5
#define AUTOMODE_GROUP_DATA_CHANNEL     0
#define AUTOMODE_GROUP_DATA_STARTHOUR   1
#define AUTOMODE_GROUP_DATA_STARTMIN    2
#define AUTOMODE_GROUP_DATA_DAYSAPART   3
#define AUTOMODE_GROUP_DATA_WATER_TIME  4


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
