/**
  ******************************************************************************
  * @file    stm32_extmem.c
  * @author  MCD Application Team
  * @brief   This file includes a driver for external memory support
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/**
 * @brief macro to get memory table definition
 */
#define EXTMEM_C

/* Includes ------------------------------------------------------------------*/
#include <string.h>  /* memset - required as an error by GCC >= 14 */
#include "stm32_extmem.h"
#include "stm32_extmem_conf.h"

#if EXTMEM_DRIVER_NOR_SFDP == 1
#include "nor_sfdp/stm32_sfdp_driver_api.h"
#endif /* EXTMEM_DRIVER_NOR_SFDP */

#if EXTMEM_DRIVER_PSRAM == 1
#include "psram/stm32_psram_driver_api.h"
#endif /* EXTMEM_DRIVER_PSRAM */

#if EXTMEM_DRIVER_SDCARD == 1
#include "sdcard/stm32_sdcard_driver_api.h"
#endif /* EXTMEM_DRIVER_SDCARD */

#if EXTMEM_DRIVER_USER == 1
#include "user/stm32_user_driver_api.h"
#endif /* EXTMEM_DRIVER_USER */

/**
  * @defgroup EXTMEM_DRIVER
  * @{
  */

/**
  * @}
 */

/**
  * @defgroup EXTMEM_SAL
  * @{
  */

/**
  * @}
 */

/**
  * @defgroup EXTMEM
  * @{
  */

/* Private Macros ------------------------------------------------------------*/
/**
  *  @defgroup EXTMEM_Private_Macros External Memory Private Macros
  * @{
  */
/**
  * @brief EXTMEM debug function
  */
/**
  * @brief Macro used to trace EXTMEM function calls
  */

#if defined(EXTMEM_DEBUG_LEVEL) && defined(EXTMEM_MACRO_DEBUG)
#if EXTMEM_DEBUG_LEVEL > 0
#define EXTMEM_DEBUG(_MSG_)  EXTMEM_MACRO_DEBUG(_MSG_)
#if EXTMEM_DEBUG_LEVEL > 1
#define EXTMEM_FUNC_CALL()  \
  do                        \
  {                         \
    EXTMEM_DEBUG(__func__); \
    EXTMEM_DEBUG("\n");     \
  } while (0);
#else
#define EXTMEM_FUNC_CALL()
#endif /* EXTMEM_DEBUG_LEVEL > 1 */
#else
#define EXTMEM_DEBUG(_MSG_)
#define EXTMEM_FUNC_CALL()
#endif /* EXTMEM_DEBUG_LEVEL > 0 */
#else
#define EXTMEM_DEBUG(_MSG_)
#define EXTMEM_FUNC_CALL()
#endif /* defined(EXTMEM_DEBUG_LEVEL) && defined(EXTMEM_MACRO_DEBUG) */

/**
  * @brief Macro used to trace EXTMEM debug information
  */
#define EXTMEM_DEBUG_ENDL(_STRING_) \
  do                                \
  {                                 \
    EXTMEM_DEBUG(_STRING_);         \
    EXTMEM_DEBUG("\n");             \
  } while (0);

/**
  * @}
  */

/* Private typedefs ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/** @defgroup EXTMEM_Exported_Functions External Memory Exported Functions
  * @{
  */

EXTMEM_StatusTypeDef EXTMEM_Init(uint32_t MemId, uint32_t ClockInput)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL();

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        /* Initialize the SFDP memory */
        if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_Init(extmem_list_config[MemId].Handle,
                                                                     extmem_list_config[MemId].ConfigType,
                                                                     ClockInput,
                                                                     &extmem_list_config[MemId].NorSfdpObject))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      /* Initialize the SFDP memory */
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_Init(extmem_list_config[MemId].Handle,
                                                               &extmem_list_config[MemId].SdCardObject))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
#if EXTMEM_DRIVER_PSRAM == 1
    case EXTMEM_PSRAM : {
        /* Initialize the SFDP memory */
        if (EXTMEM_DRIVER_PSRAM_OK != EXTMEM_DRIVER_PSRAM_Init(extmem_list_config[MemId].Handle,
                                                               extmem_list_config[MemId].ConfigType,
                                                               ClockInput,
                                                               &extmem_list_config[MemId].PsramObject))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      /* Initialize the SFDP memory */
      EXTMEM_DRIVER_USER_StatusTypeDef status;
      status = EXTMEM_DRIVER_USER_Init(MemId,
                                       &extmem_list_config[MemId].UserObject);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:{
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      }
      case EXTMEM_DRIVER_USER_OK:{
        /* nothing to do */
        retr = EXTMEM_OK;
        break;
      }
      default:{
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
    default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_DeInit(uint32_t MemId)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL();

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        /* UnInitialize the SFDP memory, the return is always OK no need to test the returned value */
        (void)EXTMEM_DRIVER_NOR_SFDP_DeInit(&extmem_list_config[MemId].NorSfdpObject);
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      /* UnInitialize the SDCRAD memory */
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_DeInit(&extmem_list_config[MemId].SdCardObject))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
#if EXTMEM_DRIVER_PSRAM == 1
    case EXTMEM_PSRAM : {
        /* UnInitialize the PSRAM memory, the return is always OK no need to test the returned value */
        (void)EXTMEM_DRIVER_PSRAM_DeInit(&extmem_list_config[MemId].PsramObject);
        break;
      }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      /* Initialize the SFDP memory */
      EXTMEM_DRIVER_USER_StatusTypeDef status = EXTMEM_DRIVER_USER_DeInit(&extmem_list_config[MemId].UserObject);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      case EXTMEM_DRIVER_USER_OK:
        /* nothing to do, the returned status is already set to EXTMEM_OK */
        break;
      default:
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
      default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_Read(uint32_t MemId, uint32_t Address, uint8_t* Data, uint32_t Size)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
    case EXTMEM_NOR_SFDP:{
      if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_Read(&extmem_list_config[MemId].NorSfdpObject,
                                                                   Address, Data, Size))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_Read(&extmem_list_config[MemId].SdCardObject,
                                                               Address, Data, Size))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
#if EXTMEM_DRIVER_PSRAM == 1
    case EXTMEM_PSRAM : {
      retr = EXTMEM_ERROR_NOTSUPPORTED;
      break;
    }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      /* Initialize the SFDP memory */
      EXTMEM_DRIVER_USER_StatusTypeDef status = EXTMEM_DRIVER_USER_Read(&extmem_list_config[MemId].UserObject,
                                                                        Address, Data, Size);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      case EXTMEM_DRIVER_USER_OK:
        /* nothing to do, the returned status is already set to EXTMEM_OK */
        break;
      default:
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
    default:{
      EXTMEM_DEBUG("\terror unknown type\n");
      retr = EXTMEM_ERROR_UNKNOWNMEMORY;
      break;
    }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_Write(uint32_t MemId, uint32_t Address, const uint8_t* Data, uint32_t Size)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_Write(&extmem_list_config[MemId].NorSfdpObject,
                                                                      Address, Data, Size))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      /* Initialize the SFDP memory */
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_Write(&extmem_list_config[MemId].SdCardObject,
                                                                Address, Data, Size))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
#if EXTMEM_DRIVER_PSRAM == 1
      case EXTMEM_PSRAM : {
          retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      /* Initialize the SFDP memory */
      EXTMEM_DRIVER_USER_StatusTypeDef status = EXTMEM_DRIVER_USER_Write(&extmem_list_config[MemId].UserObject,
                                                                         Address, Data, Size);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      case EXTMEM_DRIVER_USER_OK:
        break;
      default:
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
      default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_WriteInMappedMode(uint32_t MemId, uint32_t Address, const uint8_t* const Data, uint32_t Size)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_WriteInMappedMode(&extmem_list_config[MemId].NorSfdpObject,
                                                                                  Address, Data, Size))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
      case EXTMEM_SDCARD :
      case EXTMEM_PSRAM :
      case EXTMEM_USER :
          retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
  }

EXTMEM_StatusTypeDef EXTMEM_EraseSector(uint32_t MemId, uint32_t Address, uint32_t Size)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
    case EXTMEM_NOR_SFDP:{
      const EXTMEM_DRIVER_NOR_SFDP_ObjectTypeDef* const object = &extmem_list_config[MemId].NorSfdpObject;
      EXTMEM_DRIVER_NOR_SFDP_SectorTypeTypeDef sector_type = EXTMEM_DRIVER_NOR_SFDP_SECTOR_TYPE1;
      uint32_t local_address = Address;
      uint32_t local_size = Size;
      uint32_t sector_size = (uint32_t)1u << object->sfdp_private.DriverInfo.EraseType4Size;

      while (local_size != 0u)
      {
        /* if address is a modulo a sector size if sector  */
        if ((object->sfdp_private.DriverInfo.EraseType4Size != 0u)
            && (local_size >= sector_size)
              && ((local_address % sector_size) == 0u))
        {
          sector_type = EXTMEM_DRIVER_NOR_SFDP_SECTOR_TYPE4;
        }
        else
        {
          sector_size = (uint32_t)1u << object->sfdp_private.DriverInfo.EraseType3Size;
          if ((object->sfdp_private.DriverInfo.EraseType3Size != 0u)
              && (local_size >= sector_size)
                && ((local_address % sector_size) == 0u))
          {
            sector_type = EXTMEM_DRIVER_NOR_SFDP_SECTOR_TYPE3;
          }
          else
          {
            sector_size = (uint32_t)1u << object->sfdp_private.DriverInfo.EraseType2Size;
            if ((object->sfdp_private.DriverInfo.EraseType2Size != 0u)
                && (local_size >= sector_size)
                  && ((local_address % sector_size) == 0u))
            {
              sector_type = EXTMEM_DRIVER_NOR_SFDP_SECTOR_TYPE2;
            }
            else
            {
              sector_size = (uint32_t)1u << object->sfdp_private.DriverInfo.EraseType1Size;
              if ((object->sfdp_private.DriverInfo.EraseType1Size != 0u)
                  && ((local_address % sector_size) == 0u))
              {
                sector_type = EXTMEM_DRIVER_NOR_SFDP_SECTOR_TYPE1;
              }
              else
              {
                retr = EXTMEM_ERROR_SECTOR_SIZE;
              }
            }
          }
        }

        if (retr == EXTMEM_OK)
        {
          if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_SectorErase(&extmem_list_config[MemId].NorSfdpObject,
                                                                              local_address, sector_type))
          {
            retr = EXTMEM_ERROR_DRIVER;
          }
        }

        if (retr != EXTMEM_OK)
        {
          local_size = 0u;
        }
        else
        {
          local_address = local_address + sector_size;
          if (sector_size > local_size)
          {
            local_size = 0u;
          }
          else
          {
            local_size = local_size - sector_size;
          }
        }
      }
      break;
    }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      /* Initialize the SFDP memory */
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_EraseBlock(&extmem_list_config[MemId].SdCardObject,
                                                              Address, Size))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
    case EXTMEM_PSRAM : {
      retr = EXTMEM_ERROR_NOTSUPPORTED;
      break;
    }
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      /* Initialize the SFDP memory */
      EXTMEM_DRIVER_USER_StatusTypeDef status = EXTMEM_DRIVER_USER_EraseSector(&extmem_list_config[MemId].UserObject,
                                                                        Address, Size);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      case EXTMEM_DRIVER_USER_OK:
        break;
      default:
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
    default:{
      EXTMEM_DEBUG("\terror unknown type\n");
      retr = EXTMEM_ERROR_UNKNOWNMEMORY;
      break;
     }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_EraseAll(uint32_t MemId)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()

  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        if (EXTMEM_DRIVER_NOR_SFDP_OK != EXTMEM_DRIVER_NOR_SFDP_MassErase(&extmem_list_config[MemId].NorSfdpObject))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
    case EXTMEM_SDCARD:{
      if (EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_Erase(&extmem_list_config[MemId].SdCardObject))
      {
        retr = EXTMEM_ERROR_DRIVER;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_SDCARD == 1 */
      case EXTMEM_PSRAM:{
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      }
#if EXTMEM_DRIVER_USER == 1
    case EXTMEM_USER :{
      EXTMEM_DRIVER_USER_StatusTypeDef status = EXTMEM_DRIVER_USER_MassErase(&extmem_list_config[MemId].UserObject);
      switch(status){
      case EXTMEM_DRIVER_USER_NOTSUPPORTED:
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      case EXTMEM_DRIVER_USER_OK:
        break;
      default:
        retr = EXTMEM_ERROR_DRIVER;
        break;
      }
      break;
    }
#endif /* EXTMEM_DRIVER_USER == 1 */
      default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}

EXTMEM_StatusTypeDef EXTMEM_GetInfo(uint32_t MemId, void *Info)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL()
  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1
      case EXTMEM_NOR_SFDP:{
        EXTMEM_DRIVER_NOR_SFDP_GetFlashInfo(&extmem_list_config[MemId].NorSfdpObject, (EXTMEM_NOR_SFDP_FlashInfoTypeDef *)Info);
        break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
      case EXTMEM_SDCARD : {
        if(EXTMEM_DRIVER_SDCARD_OK != EXTMEM_DRIVER_SDCARD_GetInfo(&extmem_list_config[MemId].SdCardObject, (EXTMEM_DRIVER_SDCARD_InfoTypeDef *)Info))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
      case EXTMEM_USER : {
        if( EXTMEM_DRIVER_USER_OK != EXTMEM_DRIVER_USER_GetInfo(&extmem_list_config[MemId].UserObject, (EXTMEM_USER_MemInfoTypeDef *)Info))
        {
          retr = EXTMEM_ERROR_DRIVER;
        }
        break;
      }
#endif /* EXTMEM_DRIVER_USER == 1 */
      case EXTMEM_PSRAM :
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      default:{
        EXTMEM_DEBUG("\terror unknown type\n");
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}

//EXTMEM_StatusTypeDef EXTMEM_MemoryMappedMode(uint32_t MemId, EXTMEM_StateTypeDef State)
//{
//  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
//  EXTMEM_FUNC_CALL();
//  /* control the memory ID */
//  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
//  {
//    retr = EXTMEM_OK;
//    switch (extmem_list_config[MemId].MemType)
//    {
//#if EXTMEM_DRIVER_NOR_SFDP == 1
//      case EXTMEM_NOR_SFDP:{
//        if (EXTMEM_ENABLE == State)
//        {
//          /* start the memory mapped mode */
//          if (EXTMEM_DRIVER_NOR_SFDP_OK !=
//              EXTMEM_DRIVER_NOR_SFDP_Enable_MemoryMappedMode(&extmem_list_config[MemId].NorSfdpObject))
//          {
//            return EXTMEM_ERROR_DRIVER;
//          }
//        }
//        else
//        {
//          /* stop the memory mapped mode */
//          if (EXTMEM_DRIVER_NOR_SFDP_OK !=
//              EXTMEM_DRIVER_NOR_SFDP_Disable_MemoryMappedMode(&extmem_list_config[MemId].NorSfdpObject))
//          {
//            return EXTMEM_ERROR_DRIVER;
//          }
//        }
//        break;
//      }
//#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 */
//#if EXTMEM_DRIVER_PSRAM == 1
//      case EXTMEM_PSRAM : {
//        if (EXTMEM_ENABLE == State)
//        {
//          /* start the memory mapped mode */
//          if (EXTMEM_DRIVER_PSRAM_OK !=
//              EXTMEM_DRIVER_PSRAM_Enable_MemoryMappedMode(&extmem_list_config[MemId].PsramObject))
//          {
//            return EXTMEM_ERROR_DRIVER;
//          }
//        }
//        else
//        {
//          /* stop the memory mapped mode */
//          if (EXTMEM_DRIVER_PSRAM_OK !=
//              EXTMEM_DRIVER_PSRAM_Disable_MemoryMappedMode(&extmem_list_config[MemId].PsramObject))
//          {
//            return EXTMEM_ERROR_DRIVER;
//          }
//        }
//        break;
//      }
//#endif /* EXTMEM_DRIVER_PSRAM == 1 */
//#if EXTMEM_DRIVER_USER == 1
//     case EXTMEM_USER :{
//       /* Initialize the SFDP memory */
//       EXTMEM_DRIVER_USER_StatusTypeDef status;
//       if (EXTMEM_ENABLE == State)
//       {
//         status = EXTMEM_DRIVER_USER_Enable_MemoryMappedMode(&extmem_list_config[MemId].UserObject);
//       }
//       else
//       {
//         status = EXTMEM_DRIVER_USER_Disable_MemoryMappedMode(&extmem_list_config[MemId].UserObject);
//       }
//       switch(status){
//       case EXTMEM_DRIVER_USER_NOTSUPPORTED:
//         retr = EXTMEM_ERROR_NOTSUPPORTED;
//         break;
//       case EXTMEM_DRIVER_USER_OK:
//         break;
//       default:
//         retr = EXTMEM_ERROR_DRIVER;
//         break;
//       }
//       break;
//    }
//#endif /* EXTMEM_DRIVER_USER == 1 */
//    case EXTMEM_SDCARD :
//      retr = EXTMEM_ERROR_NOTSUPPORTED;
//    break;
//    default:{
//        EXTMEM_DEBUG("\terror unknown type\n");
//        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
//        break;
//      }
//    }
//  }
//  return retr;
//}

EXTMEM_StatusTypeDef EXTMEM_GetMapAddress(uint32_t MemId, uint32_t *BaseAddress)
{
  EXTMEM_StatusTypeDef retr = EXTMEM_ERROR_INVALID_ID;
  EXTMEM_FUNC_CALL();
  /* control the memory ID */
  if (MemId < (sizeof(extmem_list_config) / sizeof(EXTMEM_DefinitionTypeDef)))
  {
    retr = EXTMEM_OK;
    switch (extmem_list_config[MemId].MemType)
    {
#if EXTMEM_DRIVER_NOR_SFDP == 1 || EXTMEM_DRIVER_PSRAM == 1
      case EXTMEM_PSRAM:
      case EXTMEM_NOR_SFDP:{
#if defined(XSPI1)
       if (((XSPI_HandleTypeDef *)extmem_list_config[MemId].Handle)->Instance == XSPI1)
        {
          *BaseAddress = XSPI1_BASE;
       }
#if defined(XSPI2)
       else if (((XSPI_HandleTypeDef *)extmem_list_config[MemId].Handle)->Instance == XSPI2)
       {
          *BaseAddress = XSPI2_BASE;
       }
#if defined(XSPI3)
       else if (((XSPI_HandleTypeDef *)extmem_list_config[MemId].Handle)->Instance == XSPI3)
       {
          *BaseAddress = XSPI3_BASE;
       }
#endif /* XSPI3 */
#endif /* XSPI2 */
#else  /* XSPI1 */
       if (((XSPI_HandleTypeDef *)extmem_list_config[MemId].Handle)->Instance == OCTOSPI1)
       {
          *BaseAddress = OCTOSPI1_BASE;
       }
#endif  /* XSPI1 */
       else
       {
          retr = EXTMEM_ERROR_DRIVER;
       }

       break;
      }
#endif /* EXTMEM_DRIVER_NOR_SFDP == 1 || EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_SDCARD == 1
      case EXTMEM_SDCARD : {
        retr = EXTMEM_ERROR_NOTSUPPORTED;
        break;
      }
#endif /* EXTMEM_DRIVER_PSRAM == 1 */
#if EXTMEM_DRIVER_USER == 1
      case EXTMEM_USER:{
        switch(EXTMEM_DRIVER_USER_GetMapAddress(&extmem_list_config[MemId].UserObject, BaseAddress))
        {
          case EXTMEM_DRIVER_USER_OK:
          break;
          case EXTMEM_DRIVER_USER_NOTSUPPORTED:
            retr = EXTMEM_ERROR_NOTSUPPORTED;
          break;
          default :
            retr = EXTMEM_ERROR_DRIVER;
          break;
        }
      break;
      }
#endif /* EXTMEM_DRIVER_USER == 1 */
      default :
      {
        retr = EXTMEM_ERROR_UNKNOWNMEMORY;
        break;
      }
    }
  }
  return retr;
}
/**
  * @}
  */

/**
  * @}
  */
EXTMEM_StatusTypeDef EXTMEM_MemoryMappedMode(uint32_t MemID, EXTMEM_StateTypeDef State)
{
if (MemID == 0) {
  XSPI_RegularCmdTypeDef   sCommand;
  XSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* If DevID is not needed (single device), just silence the warning */

  if (State == EXTMEM_ENABLE)
  {
    /* -------- Enable memory-mapped mode -------- */

    /* Initialize memory-mapped mode for read operations */
    memset(&sCommand, 0, sizeof(sCommand));
    memset(&sMemMappedCfg, 0, sizeof(sMemMappedCfg));

    sCommand.OperationType      = HAL_XSPI_OPTYPE_READ_CFG;
    sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
    sCommand.DummyCycles        = 19;
    sCommand.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    sCommand.Instruction        = 0xEE11;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

    if (HAL_XSPI_Command(&hxspi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return EXTMEM_ERROR_DRIVER;
    }

    /* Initialize memory-mapped mode for write operations */
    sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    sCommand.Instruction   = 0x12ED;
    sCommand.DummyCycles   = 0;

    if (HAL_XSPI_Command(&hxspi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return EXTMEM_ERROR_DRIVER;
    }

    /* Configure the memory-mapped mode with TimeoutCounter Disabled */
    sMemMappedCfg.NoPrefetchAXI       = HAL_XSPI_AXI_PREFETCH_ENABLE;
    sMemMappedCfg.NoPrefetchData      = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
    sMemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_XSPI_MemoryMapped(&hxspi1, &sMemMappedCfg) != HAL_OK)
    {
      return EXTMEM_ERROR_DRIVER;
    }

    return EXTMEM_OK;
  }
  else
  {
    /* -------- Disable memory-mapped mode -------- */
    /* Abort ongoing memory-mapped operation */
    if (HAL_XSPI_Abort(&hxspi1) != HAL_OK)
    {
      return EXTMEM_ERROR_DRIVER;
    }

    return EXTMEM_OK;
  }
}

else {
	  if (State == EXTMEM_ENABLE)
	  {
	    /* -------- Enable memory-mapped mode -------- */
		  XSPI_RegularCmdTypeDef   sCommand1;
		  XSPI_MemoryMappedTypeDef sMemMappedCfg1;
	    /* Initialize memory-mapped mode for read operations */
	    memset(&sCommand1, 0, sizeof(sCommand1));
	    memset(&sMemMappedCfg1, 0, sizeof(sMemMappedCfg1));

	    sCommand1.OperationType      = HAL_XSPI_OPTYPE_READ_CFG;
	    sCommand1.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
	    sCommand1.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
	    sCommand1.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
	    sCommand1.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
	    sCommand1.Address            = 0x0;
	    sCommand1.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	    sCommand1.DataMode           = HAL_XSPI_DATA_8_LINES;
	    sCommand1.DummyCycles        = 6;
	    sCommand1.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
	    sCommand1.Instruction        = 0x00;
	    sCommand1.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	    sCommand1.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
	    sCommand1.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
	    sCommand1.DQSMode            = HAL_XSPI_DQS_ENABLE;

	    if (HAL_XSPI_Command(&hxspi2, &sCommand1, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	    {
	      return EXTMEM_ERROR_DRIVER;
	    }

	    /* Initialize memory-mapped mode for write operations */
	    sCommand1.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
	    sCommand1.Instruction   = 0x80;
	    sCommand1.DummyCycles   = 6;

	    if (HAL_XSPI_Command(&hxspi2, &sCommand1, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	    {
	      return EXTMEM_ERROR_DRIVER;
	    }

	    /* Configure the memory-mapped mode with TimeoutCounter Disabled */
	    sMemMappedCfg1.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;

	    if (HAL_XSPI_MemoryMapped(&hxspi2, &sMemMappedCfg1) != HAL_OK)
	    {
	      return EXTMEM_ERROR_DRIVER;
	    }

	    return EXTMEM_OK;
	  }
	  else
	  {
	    /* -------- Disable memory-mapped mode -------- */
	    /* Abort ongoing memory-mapped operation */
	    if (HAL_XSPI_Abort(&hxspi2) != HAL_OK)
	    {
	      return EXTMEM_ERROR_DRIVER;
	    }

	    return EXTMEM_OK;
	  }
	}
}
