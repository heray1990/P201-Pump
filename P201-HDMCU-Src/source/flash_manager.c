#include "flash_manager.h"
#include "ddl.h"


const uint32_t u32SectorHeadAddrTable[FLASH_MANAGER_SECTORS_QUANTITY] = {
        FLASH_MANAGER_DATA_SECTOR_0_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_1_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_2_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_3_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_4_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_5_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_6_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_7_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_8_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_9_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_A_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_B_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_C_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_D_HEAD_ADDR,
        FLASH_MANAGER_DATA_SECTOR_E_HEAD_ADDR, FLASH_MANAGER_DATA_SECTOR_F_HEAD_ADDR};

const uint32_t u32SectorTailAddrTable[FLASH_MANAGER_SECTORS_QUANTITY] = {
        FLASH_MANAGER_DATA_SECTOR_0_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_1_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_2_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_3_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_4_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_5_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_6_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_7_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_8_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_9_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_A_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_B_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_C_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_D_TAIL_ADDR,
        FLASH_MANAGER_DATA_SECTOR_E_TAIL_ADDR, FLASH_MANAGER_DATA_SECTOR_F_TAIL_ADDR};

stc_flash_manager_t stcFlashManager;

boolean_t Flash_Manager_Is_Partition_Valid(uint32_t u32PartHeadAddr)
{
    uint8_t u8DataAddrOffset = 0;

    for(u8DataAddrOffset = 0; u8DataAddrOffset < FLASH_MANAGER_DATA_LEN; u8DataAddrOffset++)
    {
        if(*((volatile uint8_t*)(u32PartHeadAddr + u8DataAddrOffset)) != FLASH_DATA_INIT_CODE)
        {
            return FALSE;
        }
    }

    return TRUE;
}

uint8_t Flash_Manager_Data_BCC_Checksum(uint8_t *u8pData, uint16_t u16DataLen)
{
    uint8_t u8Idx = 0;
    uint8_t u8BccCheckSum = *u8pData;

    for(u8Idx = 1; u8Idx < u16DataLen - 1; u8Idx++)
    {
        u8BccCheckSum ^= *(u8pData + u8Idx);
    }

    return u8BccCheckSum;
}

uint32_t Flash_Manager_Find_Latest_Data_Head_Addr(uint32_t u32SectorHeadAddr)
{
    uint8_t u8PartIdx = FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR;
    uint32_t u32DataAddrTmp = 0xFFFF;

    for(u8PartIdx = FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR; u8PartIdx > 0; u8PartIdx--)
    {
        /* Sector 从右到左, 查询每个Partition的首尾地址内容, 直到找到首位为0x5A, 末位为校验码,
         * 并且校验码符合异或校验结果, 说明这个Partition内容是最新保存的 */
        u32DataAddrTmp = u32SectorHeadAddr + (uint32_t)((u8PartIdx - 1) * FLASH_MANAGER_DATA_LEN);
        if(FLASH_DATA_START_CODE == *((volatile uint8_t*)u32DataAddrTmp) &&
            Flash_Manager_Data_BCC_Checksum((volatile uint8_t*)u32DataAddrTmp, FLASH_MANAGER_DATA_LEN) == *((volatile uint8_t*)(u32DataAddrTmp + FLASH_MANAGER_DATA_LEN - 1)))
        {
            return u32DataAddrTmp;
        }
    }

    return u32DataAddrTmp;
}

void Flash_Manager_Load_Latest_Data(uint32_t u32DataHeadAddr)
{
    uint8_t u8PartIdx = 0;

    for(u8PartIdx = 0; u8PartIdx < FLASH_MANAGER_DATA_LEN; u8PartIdx++)
    {
        stcFlashManager.u8FlashManagerData[u8PartIdx] = *((volatile uint8_t*)(u32DataHeadAddr + u8PartIdx));
    }
}


uint32_t Flash_Manager_Find_Valid_Partition_In_Sector(uint32_t u32SectorHeadAddr)
{
    uint8_t u8PartIdx = 0;
    uint32_t u32DataAddrTmp = 0xFFFF;

    for(u8PartIdx = 0; u8PartIdx < FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR; u8PartIdx++)
    {
        // 查询每个Partition的首地址内容, 直到找到首地址内容为0xFF, 说明这个Partition未写
        u32DataAddrTmp = u32SectorHeadAddr + (uint32_t)(u8PartIdx * FLASH_MANAGER_DATA_LEN);
        if(FLASH_DATA_INIT_CODE == *((volatile uint8_t*)u32DataAddrTmp))
        {
            // 确认该Partition是否可用, 即整个Partition内容都是0xFF
            if(TRUE == Flash_Manager_Is_Partition_Valid(u32DataAddrTmp))
            {
                // 找到可用的Partition, 返回该Partition的首地址
                return u32DataAddrTmp;
            }
            else
            {
                if(u8PartIdx == (FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR - 1))
                {
                    // 检测到当前Sector最后一个Partition未写入, 但是不可用(不全是0xFF).
                    // 把当前Sector看作写满了, 继续查询下一个Sector.
                    Flash_WriteByte((u32SectorHeadAddr + FLASH_MANAGER_BYTES_ONE_SECTOR - 1), FLASH_DATA_END_CODE_SECTOR);
                    return 0xFFFF;
                }
            }
        }
    }

    return u32DataAddrTmp;
}

en_result_t Flash_Manager_Init(void)
{
    uint8_t u8SectorIdx;
    uint8_t u8DataHeadSector[FLASH_MANAGER_SECTORS_QUANTITY], u8DataTailSector[FLASH_MANAGER_SECTORS_QUANTITY];
    uint32_t u32ValidPartHeadAddr;

    DDL_ZERO_STRUCT(stcFlashManager);

    stcFlashManager.bFlashEmpty = FALSE;

    for(u8SectorIdx = 0; u8SectorIdx < FLASH_MANAGER_SECTORS_QUANTITY; u8SectorIdx++)
    {
        u8DataHeadSector[u8SectorIdx] = *((volatile uint8_t*)u32SectorHeadAddrTable[u8SectorIdx]);
        u8DataTailSector[u8SectorIdx] = *((volatile uint8_t*)u32SectorTailAddrTable[u8SectorIdx]);

        if(FLASH_DATA_START_CODE == u8DataHeadSector[u8SectorIdx] &&
            FLASH_DATA_END_CODE_SECTOR == u8DataTailSector[u8SectorIdx])
        {
            // 这一页在上次保存时已经写满, 下次再写需要写到下一页中, 并且写成功后把本页擦除
            if((FLASH_MANAGER_SECTORS_QUANTITY - 1) == u8SectorIdx)
            {
                u32ValidPartHeadAddr = Flash_Manager_Find_Valid_Partition_In_Sector(u32SectorHeadAddrTable[0]);
                if(0xFFFF == u32ValidPartHeadAddr)
                {
                    // 当前 Sector 没有可用的 Partition
                    return Error;
                }
                else
                {
                    stcFlashManager.u32DataStoredHeadAddr = u32ValidPartHeadAddr;
                }
                Flash_Manager_Load_Latest_Data(u32SectorHeadAddrTable[u8SectorIdx] + FLASH_MANAGER_DATA_LEN * (FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR - 1));
                return Ok;
            }
            else
            {
                u32ValidPartHeadAddr = Flash_Manager_Find_Valid_Partition_In_Sector(u32SectorHeadAddrTable[u8SectorIdx + 1]);
                if(0xFFFF == u32ValidPartHeadAddr)
                {
                    // 当前 Sector 没有可用的 Partition
                    return Error;
                }
                else
                {
                    stcFlashManager.u32DataStoredHeadAddr = u32ValidPartHeadAddr;
                }
                Flash_Manager_Load_Latest_Data(u32SectorHeadAddrTable[u8SectorIdx] + FLASH_MANAGER_DATA_LEN * (FLASH_MANAGER_DATA_PARTITIONS_ONE_SECTOR - 1));
                return Ok;
            }
        }
        else if(FLASH_DATA_START_CODE == u8DataHeadSector[u8SectorIdx] &&
            FLASH_DATA_INIT_CODE == u8DataTailSector[u8SectorIdx])
        {
            u32ValidPartHeadAddr = Flash_Manager_Find_Valid_Partition_In_Sector(u32SectorHeadAddrTable[u8SectorIdx]);
            if(0xFFFF == u32ValidPartHeadAddr)
            {
                // 当前 Sector 没有可用的 Partition
                return Error;
            }
            else
            {
                stcFlashManager.u32DataStoredHeadAddr = u32ValidPartHeadAddr;
            }
            Flash_Manager_Load_Latest_Data(Flash_Manager_Find_Latest_Data_Head_Addr(u32SectorHeadAddrTable[u8SectorIdx]));
            return Ok;
        }
        else if(FLASH_DATA_INIT_CODE == u8DataHeadSector[u8SectorIdx] &&
            FLASH_DATA_INIT_CODE == u8DataTailSector[u8SectorIdx])
        {
            if(u8SectorIdx == FLASH_MANAGER_SECTORS_QUANTITY - 1)
            {
                stcFlashManager.u32DataStoredHeadAddr = u32SectorHeadAddrTable[0];
                stcFlashManager.bFlashEmpty = TRUE;
                return Ok;
            }
        }
    }

    return Ok;
}

en_result_t Flash_Manager_Update(void)
{
    uint8_t u8Idx;
    en_result_t enRetVal = Ok;

    for(u8Idx = 0; u8Idx < FLASH_MANAGER_DATA_LEN; u8Idx++)
    {
        if(Ok != Flash_WriteByte((stcFlashManager.u32DataStoredHeadAddr + u8Idx), stcFlashManager.u8FlashManagerData[u8Idx]))
        {
            enRetVal = Error;
        }
    }

    for(u8Idx = 0; u8Idx < FLASH_MANAGER_SECTORS_QUANTITY; u8Idx++)
    {
        if(stcFlashManager.u32DataStoredHeadAddr + FLASH_MANAGER_DATA_LEN + 1 == u32SectorTailAddrTable[u8Idx])
        {
            // 该Sector写满了, 则在最后一位写入0xA5
            if(Ok != Flash_WriteByte(u32SectorTailAddrTable[u8Idx], FLASH_DATA_END_CODE_SECTOR))
            {
                enRetVal = Error;
            }
        }

        if(stcFlashManager.u32DataStoredHeadAddr == u32SectorHeadAddrTable[u8Idx] && Ok == enRetVal)
        {
            // 写完数据之后, 当该Partition是Sector的第一个时, 判断是否需要把上一个Sector擦除
            if(0 == u8Idx)
            {
                if(FLASH_DATA_END_CODE_SECTOR == *((volatile uint8_t*)u32SectorTailAddrTable[FLASH_MANAGER_SECTORS_QUANTITY - 1]))
                {
                    Flash_SectorErase(u32SectorHeadAddrTable[FLASH_MANAGER_SECTORS_QUANTITY - 1]);
                }
            }
            else
            {
                if(FLASH_DATA_END_CODE_SECTOR == *((volatile uint8_t*)u32SectorTailAddrTable[u8Idx - 1]))
                {
                    Flash_SectorErase(u32SectorHeadAddrTable[u8Idx - 1]);
                }
            }
        }
    }

    return enRetVal;
}
