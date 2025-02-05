## P201-PUMP HDMCU Project

### 简介

本项目是基于华大 MCU 开发的 P201 抽水机项目。MCU 型号是 HC32L136K8TA-LQFP64。此文档主要记录软件相关的笔记。

### 开发环境

* Keil MDK 532
* Arm Compiler V6.14.1 或以下，Arm Compiler V6.15 会有问题，详见 [《Keil MDK 5.33 运行华大官方示例无法进入 Timer0 中断》](https://blog.csdn.net/heray1990/article/details/113838794)

### 入门资料和参考文章

* [华大官网 HC32L136K8TA-LQFP64 文档、驱动库与示例](https://www.hdsc.com.cn/Category82-1404)
* [《华大单片机DDL库与lite库的区别》](https://blog.csdn.net/willOkay/article/details/106535809)
* [《如何用 Keil 新建一个工程》](https://blog.csdn.net/willOkay/article/details/106533167)
* [《Keil MDK 5.33 运行华大官方示例无法进入 Timer0 中断》](https://blog.csdn.net/heray1990/article/details/113838794)
* 状态机按键消抖：[《Debouncing Push-Buttons Using a State Machine Approach》](https://www.eeweb.com/debouncing-push-buttons-using-a-state-machine-approach/)，参考代码：[https://github.com/tommygartlan/Buttons_Debounce_State_Mch](https://github.com/tommygartlan/Buttons_Debounce_State_Mch)

### 烧录模式

#### ISP 串口烧录模式

1. 到华大官网下载“**Cortex-M在线编程器.zip**”：https://www.hdsc.com.cn/Category82-1404

   ![](./华大官网编程器下载界面.png)

2. 解压步骤1下载到的压缩包，打开里面的文档《Cortex-M在线编程器用户手册Rev2.0.pdf》可以找到串口模块与 MCU 引脚连接图。其中 P201 的引脚连接图和实物图如下：

![](./串口模块与MCU引脚接线图.png)

![](./ISP串口烧录模式接线实物图.jpg)

3. 引脚接线 OK 之后，把串口工具插入电脑，然后打开压缩包中的 **hdsc.exe** 程序，按照《Cortex-M在线编程器用户手册Rev2.0.pdf》文档中的说明进行设置即可完成烧录。其中**Hex文件**是 Keil 编译生成的文件。下面是实际烧录过程。

![](./P201_MCU_ISP_PROGRAM.gif)

#### SWD 烧录模式

P201 SWD 烧录模式实物接线图如下：

![](./SWD烧录模式接线实物图.jpg)

### Flash 数据存储结构

HC32L136K8TA-LQFP64 包含一块 64K Bytes 容量的 Flash 存储器（详见芯片用户手册）。本项目利用 0x0000E000~0x0000FFFF 这个区域来保存数据，共 16 个 Sector，每个 Sector 512 Bytes，共 8192 Bytes。每个 Sector 能容纳 17 个分区（Partition）的数据。每个分区保存着程序中需要的数据，共 30 Bytes。

**Sector 地址对照表**

![](./Sectors_Addresses.PNG)

**每个 Sector 分区表**

![](./Data_in_one_Sector_01.PNG)

![](./Data_in_one_Sector_02.PNG)

#### 数据取值范围与最小需要的位数

* **Start Code**: 0x5A, 8bits
* **u8GroupNum**: 0~9, 4bits
* **enWorkingMode**: 0~1, 1bit
* **u8StopFlag**: 0~1, 1bit
* **u8ChannelManual**: 0~1, 1bit
* **Reserved**: 0~1, 1bit
* **u16WateringTimeManual[0]**: 0~999, 10bits
* **u16WateringTimeManual[1]**: 0~999, 10bits
* **u8DaysApart[0]**: 0~99, 7bits
* **u8ChannelAuto[0]**: 0~1, 1bit
* **u8StartHour[0]**: 0~24, 5bits
* **u8StartMin[0]**: 0~60, 6bits
* **u16WateringTimeAuto[0]**: 0~999, 10bits
* ......
* **u8DaysApart[5]**: 0~99, 7bits
* **u8ChannelAuto[5]**: 0~1, 1bit
* **u8StartHour[5]**: 0~24, 5bits
* **u8StartMin[5]**: 0~60, 6bits
* **u16WateringTimeAuto[5]**: 0~999, 10bits
* **u8CheckSumBCC**: 校验位, 8bits

**分区（Partition）数据**

![](./Data_in_one_Partition.PNG)

### 系统状态转换原理

#### 系统状态枚举：

```C
typedef enum
{
    PowerOn = 0u,
    PowerOnCharge = 1u,
    StandBy = 2u,
    StandByChargeEarly = 3u,
    StandByCharge = 4u,
    PowerOff = 5u,
    PowerOffChargeEarly = 6u,
    PowerOffCharge = 7u
}en_sys_states;
```

* **PowerOn**：LCD+BL On；RTC、Timer和水泵正常工作；按键正常响应
* **PowerOnCharge**：LCD+BL On；RTC、Timer和水泵正常工作；按键正常响应；电量动画
* **StandBy**（休眠模式）：LCD+BL Off; RTC、水泵不工作；Timer 停止；按键和充电唤醒
* **StandByChargeEarly**：LCD+BL On；RTC、Timer和水泵正常工作；按键正常响应；电量动画
* **StandByCharge**：LCD On+BL Off；RTC、Timer和水泵正常工作；按键正常响应；电量动画
* **PowerOff**（休眠模式）：LCD+BL Off; RTC正常工作；Timer和水泵不工作；Power按键和充电唤醒
* **PowerOffChargeEarly**：LCD+BL On；RTC和Time正常工作；水泵不工作；只响应Power按键；LCD只显示电量动画
* **PowerOffCharge**：LCD On+BL Off；RTC和Time正常工作；水泵不工作；只响应Power按键；LCD只显示电量动画

#### 系统状态转换触发源：

A：按下 Power Key

B：5VIN IO（PC02）上升沿，即插入充电线

C：5VIN IO（PC02）下降沿，即拔掉充电线

D：10s 无操作倒计时

E：其它按键触发

F：RTC 计时到点

#### 系统状态转换流程图

```mermaid
graph LR
	0u[PowerOn] -. A .-> 5u((PowerOff))
	0u[PowerOn] -. B .-> 1u[PowerOnCharge]
	0u[PowerOn] -. D .-> 2u((StandBy))
	1u[PowerOnCharge] -. A .-> 7u[PowerOffCharge]
	1u[PowerOnCharge] -. C .-> 0u[PowerOn]
	1u[PowerOnCharge] -. D .-> 4u[StandByCharge]
	1u[PowerOnCharge] -. F .-> 1u[PowerOnCharge]
	2u((StandBy)) == A_Wakeup or E_Wakeup or F_Wakeup ==> 0u[PowerOn]
	2u((StandBy)) == B_Wakeup ==> 3u[StandByChargeEarly]
	3u[StandByChargeEarly] -. A .-> 7u[PowerOffCharge]
	3u[StandByChargeEarly] -. C .-> 2u((StandBy))
	3u[StandByChargeEarly] -. D .-> 4u[StandByCharge]
	3u[StandByChargeEarly] -. F .-> 1u[PowerOnCharge]
	4u[StandByCharge] -. A .-> 7u[PowerOffCharge]
	4u[StandByCharge] -. C .-> 2u((StandBy))
	4u[StandByCharge] -. E .-> 3u[StandByChargeEarly]
	4u[StandByCharge] -. F .-> 1u[PowerOnCharge]
	5u((PowerOff)) == A_Wakeup ==> 0u[PowerOn]
	5u((PowerOff)) == B_Wakeup ==> 6u[PowerOffChargeEarly]
	6u[PowerOffChargeEarly] -. A or D .-> 7u[PowerOffCharge]
	6u[PowerOffChargeEarly] -. C .-> 5u((PowerOff))
	7u[PowerOffCharge] -. A .-> 1u[PowerOnCharge]
	7u[PowerOffCharge] -. C .-> 5u((PowerOff))
```

