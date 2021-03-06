/**
 * Store data into internal flash of HDMCU HC32L136K8TA-LQFP64.
 * 
 * 1. Data stored in one Sector
 *    Total: 512 Bytes
 *    | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes | 51 Bytes |  1 Byte  |  1 Byte  |
 *    |   data   |   data   |   data   |   data   |   data   |   data   |   data   |   data   |   data   |   data   | reserved |   0xA5   |
 * 2. Data stored in one partition
 *    Total: 51 Bytes
 *    | start code | 40 Bytes data | 10 Bytes Reserved | checksum |
 *    |    0x5A    | 40 Bytes data |        0xFF       |   BCC    |
 *    Detail
 *    | 0x5A | <- start code, 1 Byte
 *    | u8GroupNum(4 bites) enWorkingMode(1 bit) bStopFlag(1 bits) reserved(2 bits) | <- 1 Byte
 *    | u16WateringTimeManul(16bits) | <- 2 Bytes
 *    | u8Channel | u8StartHour | u8StartMin | u8DaysApart | u16WateringTimeAuto(16 bits) | <- 6 groups, 60 Bytes
 *    | reserved | <- 10 Bytes
 *    | checksum | <- checksum code, 1 Byte 
 */

#ifndef __FLASH_MANAGER_H__
#define __FLASH_MANAGER_H__


#include <stdint.h>
#include "flash.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

#define FLASH_MANAGER_BYTES_ONE_SECTOR 512
#define FLASH_MANAGER_DATA_LEN 51   // 1bytes起始码 + 39bytes数据 + 10bytes预留空间 + 1bytes校验码(末尾)
#define FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR 10

// Using 16 sectors of flash to store data. Addresses are frome 0xE000 to 0xFFFF
#define FLASH_MANAGER_SECTORS_QUANTITY 16
#define FLASH_MANAGER_DATA_SECTOR_0_HEAD_ADDR 0xE000
#define FLASH_MANAGER_DATA_SECTOR_0_TAIL_ADDR 0xE1FF
#define FLASH_MANAGER_DATA_SECTOR_1_HEAD_ADDR 0xE200
#define FLASH_MANAGER_DATA_SECTOR_1_TAIL_ADDR 0xE3FF
#define FLASH_MANAGER_DATA_SECTOR_2_HEAD_ADDR 0xE400
#define FLASH_MANAGER_DATA_SECTOR_2_TAIL_ADDR 0xE5FF
#define FLASH_MANAGER_DATA_SECTOR_3_HEAD_ADDR 0xE600
#define FLASH_MANAGER_DATA_SECTOR_3_TAIL_ADDR 0xE7FF
#define FLASH_MANAGER_DATA_SECTOR_4_HEAD_ADDR 0xE800
#define FLASH_MANAGER_DATA_SECTOR_4_TAIL_ADDR 0xE9FF
#define FLASH_MANAGER_DATA_SECTOR_5_HEAD_ADDR 0xEA00
#define FLASH_MANAGER_DATA_SECTOR_5_TAIL_ADDR 0xEBFF
#define FLASH_MANAGER_DATA_SECTOR_6_HEAD_ADDR 0xEC00
#define FLASH_MANAGER_DATA_SECTOR_6_TAIL_ADDR 0xEDFF
#define FLASH_MANAGER_DATA_SECTOR_7_HEAD_ADDR 0xEE00
#define FLASH_MANAGER_DATA_SECTOR_7_TAIL_ADDR 0xEFFF
#define FLASH_MANAGER_DATA_SECTOR_8_HEAD_ADDR 0xF000
#define FLASH_MANAGER_DATA_SECTOR_8_TAIL_ADDR 0xF1FF
#define FLASH_MANAGER_DATA_SECTOR_9_HEAD_ADDR 0xF200
#define FLASH_MANAGER_DATA_SECTOR_9_TAIL_ADDR 0xF3FF
#define FLASH_MANAGER_DATA_SECTOR_A_HEAD_ADDR 0xF400
#define FLASH_MANAGER_DATA_SECTOR_A_TAIL_ADDR 0xF5FF
#define FLASH_MANAGER_DATA_SECTOR_B_HEAD_ADDR 0xF600
#define FLASH_MANAGER_DATA_SECTOR_B_TAIL_ADDR 0xF7FF
#define FLASH_MANAGER_DATA_SECTOR_C_HEAD_ADDR 0xF800
#define FLASH_MANAGER_DATA_SECTOR_C_TAIL_ADDR 0xF9FF
#define FLASH_MANAGER_DATA_SECTOR_D_HEAD_ADDR 0xFA00
#define FLASH_MANAGER_DATA_SECTOR_D_TAIL_ADDR 0xFBFF
#define FLASH_MANAGER_DATA_SECTOR_E_HEAD_ADDR 0xFC00
#define FLASH_MANAGER_DATA_SECTOR_E_TAIL_ADDR 0xFDFF
#define FLASH_MANAGER_DATA_SECTOR_F_HEAD_ADDR 0xFE00
#define FLASH_MANAGER_DATA_SECTOR_F_TAIL_ADDR 0xFFFF

#define FLASH_DATA_START_CODE 0x5A
#define FLASH_DATA_END_CODE_SECTOR 0xA5
#define FLASH_DATA_INIT_CODE 0xFF

typedef struct stc_flash_manager
{
    boolean_t bFlashEmpty;
    /*存储用户数据,用户把数据存储在此数组*/
    uint32_t u8FlashManagerData[FLASH_MANAGER_DATA_LEN];
    /*记录数据要保存到新Partition的首地址*/
    uint32_t u32DataStoredHeadAddr;
} stc_flash_manager_t;

extern stc_flash_manager_t stcFlashManager;

extern uint8_t Flash_Manager_Data_BCC_Checksum(uint8_t *u8pData, uint16_t u16DataLen);
extern en_result_t Flash_Manager_Update(void);
extern en_result_t Flash_Manager_Init(void);
extern en_result_t Flash_Manager_Erase_All_Data(void);

#ifdef __cplusplus
}
#endif

#endif
