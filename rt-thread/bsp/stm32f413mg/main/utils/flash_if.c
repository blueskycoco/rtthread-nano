#include "flash_if.h"

static uint32_t GetSector(uint32_t Address);

uint32_t FLASH_If_Erase(uint32_t StartSector, uint32_t EndSector)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError = 0;
	uint32_t FirstSector = GetSector(StartSector);
	uint32_t NbOfSectors = GetSector(EndSector) - FirstSector + 1;
	
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
		rt_kprintf("erase failed, %d\r\n", SECTORError);

	return (0);
}

uint32_t FLASH_If_Write(uint32_t FlashAddress, uint32_t* Data ,uint32_t DataLength)
{
	uint32_t i = 0;
	uint32_t addr = FlashAddress;
	for (i = 0; (i < DataLength) && (addr <= (USER_FLASH_END_ADDRESS-4)); i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint32_t)Data[i]) == HAL_OK)
		{
			if (*(__IO uint32_t*)addr != *(uint32_t*)(Data+i))
			{
				rt_kprintf("%d %x <> %x\r\n", i, *(uint32_t *)addr, *(uint32_t *)(Data+i));
				return(2);
			}
			addr += 4;
		}
		else
		{
			rt_kprintf("FLASH_ProgramWord failed\r\n");
			return (1);
		}
	}

	return (0);
}

static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
	{
		sector = FLASH_SECTOR_7;
	}
	return sector;
}

static uint32_t GetSectorSize(uint32_t Sector)
{
	uint32_t sectorsize = 0x00;
	if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
	{
		sectorsize = 16 * 1024;
	}
	else if(Sector == FLASH_SECTOR_4)
	{
		sectorsize = 64 * 1024;
	}
	else
	{
		sectorsize = 128 * 1024;
	}  
	return sectorsize;
}
