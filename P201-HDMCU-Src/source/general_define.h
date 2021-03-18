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
#define CHANNEL_NUM_MAX    2
#define AUTOMODE_GROUP_DATA_ELEMENT_MAX 5
#define AUTOMODE_GROUP_DATA_DAYSAPART   0
#define AUTOMODE_GROUP_DATA_CHANNEL     1
#define AUTOMODE_GROUP_DATA_STARTHOUR   2
#define AUTOMODE_GROUP_DATA_STARTMIN    3
#define AUTOMODE_GROUP_DATA_WATER_TIME  4

#define GPIO_PORT_KEY       GpioPortD
#define GPIO_PIN_KEY_POWER  GpioPin0
#define GPIO_PIN_KEY_MODE   GpioPin1
#define GPIO_PIN_KEY_SET    GpioPin4
#define GPIO_PIN_KEY_OK     GpioPin6
#define GPIO_PIN_KEY_DOWN   GpioPin5
#define GPIO_PIN_KEY_UP     GpioPin7

#define GPIO_PORT_PUMP_1    GpioPortC
#define GPIO_PORT_PUMP_2    GpioPortB
#define GPIO_PIN_PUMP_1     GpioPin13
#define GPIO_PIN_PUMP_2     GpioPin7

#define GPIO_PORT_LCD_BL    GpioPortC
#define GPIO_PIN_LCD_BL     GpioPin0


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
    Group = 1u,
    Channel = 2u,
    WateringTime = 3u,
    StartingTimeH = 4u,
    StartingTimeM = 5u,
    DaysApart = 6u,
    RtcYear = 7u,
    RtcMonth = 8u,
    RtcDay = 9u,
    RtcHour = 10u,
    RtcMin = 11u,
    Mode = 12u
}en_focus_on;

#endif /* __GENERAL_DEFINE_H__ */

/*******************************************************************************
 * EOF (not truncated)                                                        
 ******************************************************************************/
