/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file NV_Flash.c
* Implementation of the non-volatile storage module for the CORTEX-M4 processor
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "EmbeddedTypes.h"
#include "NV_Flash.h"
#include "TimersManager.h"
#include "RNG_Interface.h"
#include "FunctionLib.h"

#include "fsl_os_abstraction.h"
#include "Flash_Adapter.h"

#if (gFsciIncluded_c && (gNvmEnableFSCIRequests_c || gNvmEnableFSCIMonitoring_c))
#include "NV_FsciCommands.h"
#endif

#include "Messaging.h"
/*****************************************************************************
 *****************************************************************************
 * Private macros
 *****************************************************************************
 *****************************************************************************/
#if gNvStorageIncluded_d

 #if (gUnmirroredFeatureSet_d == TRUE)
   #if (gNvUseFlexNVM_d == TRUE)
     #error "*** ERROR: gUnmirroredFeatureSet_d not implemented on FlexNVM"
   #endif
   #if (gNvFragmentation_Enabled_d == FALSE)
     #error "*** ERROR: gNvFragmentation_Enabled_d should be enabled for gUnmirroredFeatureSet_d"
   #endif
 #endif

/*
 * Name: gNvVirtualPagesCount_c
 * Description: the count of virtual pages used
 */
#define gNvVirtualPagesCount_c         2 /* DO NOT MODIFY */

/*
 * Name: gNvGuardValue_c
 * Description: self explanatory
 */
#define gNvGuardValue_c            0xFFFFFFFFFFFFFFFFuL

/*
 * Name: gNvFirstMetaOffset_c
 * Description: the offset of the first meta
 */
#if gNvUseExtendedFeatureSet_d
    #define gNvFirstMetaOffset_c       (sizeof(NVM_TableInfo_t) + mNvTableSizeInFlash + sizeof(NVM_TableInfo_t))
#else
    #define gNvFirstMetaOffset_c       (sizeof(NVM_TableInfo_t))
#endif

/*
 * Name: gNvErasedFlashCellValue_c
 * Description: self explanatory
 */
#define gNvErasedFlashCellValue_c      0xFF


#endif /* gNvStorageIncluded_d */
/*****************************************************************************
 *****************************************************************************
 * Private type definitions
 *****************************************************************************
 *****************************************************************************/
#if PGM_SIZE_BYTE == FTFx_PHRASE_SIZE
typedef uint64_t NV_baseType;
#else
typedef uint32_t NV_baseType;
#endif

/*****************************************************************************
 *****************************************************************************
 * Private prototypes
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

/******************************************************************************
 * Name: __NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element
 *               [IN] overwrite - if an existing table entry shall be
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d
static NVM_Status_t __NvRegisterTableEntry
(
  void* ptrData,
  NvTableEntryId_t uniqueId,
  uint16_t elemCount,
  uint16_t elemSize,
  uint16_t dataEntryType,
  bool_t overwrite
);
/******************************************************************************
 * Name: __NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV
 *              table. The RAM table must be updated previusly.
 * Parameter(s): [IN] entryId - the entry id of the entry that is removed
 *               [IN] tableEntryIndex - the index of the entry in the ram table
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
static NVM_Status_t __NvEraseEntryFromStorage
(
  uint16_t entryId,
  uint16_t tableEntryIndex
);
#endif /* gNvUseExtendedFeatureSet_d */

/******************************************************************************
 * Name: __NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvAtomicSave
(
  bool_t ignoreCriticalSectionFlag
);

/******************************************************************************
 * Name: __NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvSyncSave
(
  void* ptrData,
  bool_t saveAll,
  bool_t ignoreCriticalSectionFlag
);

/******************************************************************************
 * Name: __NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter to active page.
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a
 *                                        critical section
 *****************************************************************************/
static NVM_Status_t __NvFormat
(
  void
);
/******************************************************************************
 * Name: __NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
static void __NvIdle
(
  void
);
/******************************************************************************
 * Name: __NvIsDataSetDirty
 * Description: return TRUE if the element pointed by ptrData is dirty
 * Parameters: [IN] ptrData - pointer to data to be checked
 * Return: TRUE if the element is dirty, FALSE otherwise
 ******************************************************************************/
bool_t __NvIsDataSetDirty
(
  void* ptrData
);
/******************************************************************************
 * Name: __NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
static NVM_Status_t __NvRestoreDataSet
(
  void* ptrData,
  bool_t restoreAll
);

/******************************************************************************
 * Name: __NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
static bool_t __NvTimerTick
(
  bool_t countTick
);
/******************************************************************************
 * Name: __NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnCount
(
  void* ptrData
);
/******************************************************************************
 * Name: __NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnInterval
(
  void* ptrData
);
/******************************************************************************
 * Name: __NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnIdle
(
  void* ptrData,
  bool_t saveAll
);

/******************************************************************************
 * Name: __NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found
 *         gNVM_OK_c - module was successfully initialized
 *****************************************************************************/
static NVM_Status_t __NvModuleInit
(
  void
);

/******************************************************************************
 * Name: InitNVMConfig
 * Description: Initialises the hal driver, and gets the active page.
 * Parameter(s): -
 * Return: -
 *****************************************************************************/
static void InitNVMConfig
(
  void
);
#if gUnmirroredFeatureSet_d

/******************************************************************************
 * Name: __NvmMoveToRam
 * Description: Move from NVM to Ram
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: pointer to Ram location
 *****************************************************************************/
static NVM_Status_t __NvmMoveToRam
(
  void** ppData
);

/******************************************************************************
 * Name: __NvmErase
 * Description: Erase from NVM an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be erased
 * Return: pointer to Ram location
 *****************************************************************************/
static NVM_Status_t __NvmErase
(
  void** ppData
);

/******************************************************************************
 * Name: NvIsNVMFlashAddress
 * Description: check if the address is in Flash
 * Parameter(s): [IN] address
 *
 * Return: TRUE if the table entry is in Flash / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsNVMFlashAddress
(
  void* address
);
#endif

/******************************************************************************
 * Name: NvGetRecordFullSize
 * Description: Computes the size of the specified table entry that will
 *              be written on FLASH memory
 * Parameter(s): [IN] tableEntryIndex - table entry index
 * Return: the computed size
 *****************************************************************************/
static uint32_t NvGetRecordFullSize
(
  NvTableEntryId_t tableEntryIndex
);


/******************************************************************************
 * Name: NvInitPendingSavesQueue
 * Description: Initialize the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 * Return: TRUE if the pointer is valid, FALSE otherwise
 ******************************************************************************/
static bool_t NvInitPendingSavesQueue
(
  NVM_SaveQueue_t *pQueue
);


/******************************************************************************
 * Name: NvPushPendingSave
 * Description: Add a new pending save to the queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [IN] data - data to be saved
 * Return: TRUE if the push operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPushPendingSave
(
  NVM_SaveQueue_t *pQueue,
  NVM_TableEntryInfo_t data
);


/******************************************************************************
 * Name: NvPopPendingSave
 * Description: Retrieves the head element from the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [OUT] pData - pointer to the location where data will be placed
 * Return: TRUE if the pop operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPopPendingSave
(
  NVM_SaveQueue_t *pQueue,
  NVM_TableEntryInfo_t *pData
);

/******************************************************************************
 * Name: NvGetPendingSavesCount
 * Description: self explanatory
 * Parameters: [IN] pQueue - pointer to queue
 * Return: Number of pending saves
 ******************************************************************************/
static uint8_t NvGetPendingSavesCount
(
  NVM_SaveQueue_t *pQueue
);

/*****************************************************************
 * The below functions are compiled only if FlexNVM is NOT used
 *****************************************************************/

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/******************************************************************************
 * Name: NvEraseVirtualPage
 * Description: erase the specified page
 * Parameter(s): [IN] pageID - the ID of the page to be erased
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_SectorEraseFail_c - if the page cannot be erased
 *         gNVM_OK_c - if operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvEraseVirtualPage
(
  NVM_VirtualPageID_t pageID
);


/******************************************************************************
 * Name: NvInitStorageSystem
 * Description: Initialize the storage system, retrieve the active page and
 *              the page counter. Called once by NvModuleInit() function.
 * Parameter(s): -
 * Return: -
 *****************************************************************************/
static void NvInitStorageSystem
(
  void
);


/******************************************************************************
 * Name: NvVirtualPageBlankCheck
 * Description: checks if the specified page is blank (erased)
 * Parameter(s): [IN] pageID - the ID of the page to be checked
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_PageIsNotBlank_c - if the page is not blank
 *         gNVM_OK_c - if the page is blank (erased)
 *****************************************************************************/
static NVM_Status_t NvVirtualPageBlankCheck
(
  NVM_VirtualPageID_t pageID
);


/******************************************************************************
 * Name: NvUpdateLastMetaInfoAddress
 * Description: retrieve and store (update) the last meta information address
 * Parameter(s): -
 * Return: gNVM_MetaNotFound_c - if no meta information has been found
 *         gNVM_OK_c - if the meta was found and stored (updated)
 *****************************************************************************/
static NVM_Status_t NvUpdateLastMetaInfoAddress
(
  void
);


/******************************************************************************
 * Name: NvGetMetaInfo
 * Description: get meta information based on the meta information address
 * Parameter(s): [IN] pageID - the ID of the page
 *               [IN] metaInfoAddress - meta information address
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 requested meta information will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_AddressOutOfRange_c - if the provided address is out of range
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetMetaInfo
(
  NVM_VirtualPageID_t pageId,
  uint32_t metaInfoAddress,
  NVM_RecordMetaInfo_t* pMetaInfo
);


/******************************************************************************
 * Name: NvGetPageFreeSpace
 * Description: return the page free space, in bytes
 * Parameter(s): [OUT] ptrFreeSpace - a pointer to a memory location where the
 *                                    page free space will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if the page is empty
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetPageFreeSpace
(
  uint32_t* ptrFreeSpace
);


/******************************************************************************
 * Name: NvIsMemoryAreaAvailable
 * Description: checks if the specified memory area is blank (erased)
 * Parameter(s): [IN] address - start address
 *               [IN] len - length to be verified
 * Return: TRUE if the area is available (blank), FALSE otherwise
 *****************************************************************************/
static bool_t NvIsMemoryAreaAvailable
(
  uint32_t address,
  uint32_t len
);


/******************************************************************************
 * Name: NvIsRecordCopied
 * Description: Checks if a record or an entire table entry is already copied.
 *              Called by page copy function.
 * Parameter(s): [IN] pageId - the ID of the page where to perform the check
 *               [IN] metaInf - a pointer to source page meta information tag
 * Return: TRUE if the element is already copied, FALSE otherwise
 *****************************************************************************/
static bool_t NvIsRecordCopied
(
  NVM_VirtualPageID_t pageId,
  NVM_RecordMetaInfo_t* metaInf
);


/******************************************************************************
 * Name: NvInternalCopy
 * Description: Performs a copy of an record / entire table entry
 * Parameter(s): [IN] dstAddress - destination record address
 *               [IN] dstMetaAddress - destination meta address
 *               [IN] srcMetaInfo - source meta information
 *               [IN] srcTblEntryIdx - source table entry index
 *               [IN] size - bytes to copy
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvInternalCopy
(
  uint32_t dstAddress,
  uint32_t dstMetaAddress,
  NVM_RecordMetaInfo_t* srcMetaInfo,
  uint16_t srcTblEntryIdx,
  uint16_t size
);



/******************************************************************************
 * Name: NvGetTblEntryMetaAddrFromId
 * Description: Gets the table entry meta address based on table entry ID
 * Parameter(s): [IN] searchStartAddress - the search start address
 *               [IN] dataEntryId - table entry ID
 * Return: the value of the meta address
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static uint32_t NvGetTblEntryMetaAddrFromId
(
  uint32_t searchStartAddress,
  uint16_t dataEntryId
);

/******************************************************************************
 * Name: NvInternalDefragmentedCopy
 * Description: Performs defragmentation and copy from the source page to
 *              the destination one
 * Parameter(s): [IN] srcMetaAddr - source page meta address
 *               [IN] pSrcMetaInf - pointer to source page meta information
 *               [IN] srcTblEntryIdx - source page table entry index
 *               [IN] dstMetaAddr - destination meta address
 *               [IN] dstRecordAddr - destination record address (to copy to)
 * Return: the status of the operation
 *****************************************************************************/
static NVM_Status_t NvInternalDefragmentedCopy
(
  uint32_t srcMetaAddr,
  NVM_RecordMetaInfo_t* pSrcMetaInf,
  uint16_t srcTblEntryIdx,
  uint32_t dstMetaAddr,
  uint32_t dstRecordAddr
);
#endif /* #if gNvFragmentation_Enabled_d */


/******************************************************************************
 * Name: NvCopyPage
 * Description: Copy the active page content to the mirror page. Only the
 *              latest table entries / elements are copied. A merge operation
 *              is performed before copy if an entry has single elements
 *              saved priori and newer than the table entry. If one or more
 *              elements were singular saved and the NV page doesn't has a
 *              full table entry saved, then the elements are copied as they
 *              are.
 * Parameter(s): [IN] skipEntryId - the entry ID to be skipped when page
 *                                  copy is performed
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvCopyPage
(
  NvTableEntryId_t skipEntryId
);


/******************************************************************************
 * Name: NvInternalFormat
 * Description: Format the NV storage system. The function erases in place both
 *              virtual pages and then writes the page counter value to first
 *              virtual page. The provided page counter value is automatically
 *              incremented and then written to first (active) virtual page.
 * Parameter(s): [IN] pageCounterValue - the page counter value that will
 *                                       be incremented and then written to
 *                                       active page
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvInternalFormat
(
  uint32_t pageCounterValue
);

/******************************************************************************
 * Name: NvSaveRamTable
 * Description: Saves the NV table
 * Parameter(s): [IN] pageId - the virtual page ID where the table will be
 *                             saved
 * Return: TRUE if table saved successfully, FALSE otherwise
 ******************************************************************************/
static bool_t NvSaveRamTable
(
    NVM_VirtualPageID_t pageId
);

#if gNvUseExtendedFeatureSet_d

/******************************************************************************
 * Name: GetFlashTableVersion
 * Description: returns the flash table version
 * Parameter(s): -
 * Return: 0 or flash table version
 *****************************************************************************/
uint16_t GetFlashTableVersion
(
    void
);

/******************************************************************************
 * Name: RecoverNvEntry
 * Description: Reads a flash entry so that the application can handle dinamic entries.
 * Parameter(s): [IN] index - the ram entry index
 *               [OUT] entry - the flash entry at the specified index
 * Return: gNVM_OK_c - if the operation completes successfully
           gNVM_RestoreFailure_c - if the operation failed
 *****************************************************************************/
NVM_Status_t RecoverNvEntry
(
    uint16_t index,
    NVM_DataEntry_t *entry
);
/******************************************************************************
 * Name: NvGetFlashTableSize
 * Description: Retrieves the size of the NV table
 * Parameter(s): -
 * Return: the NV table size
 ******************************************************************************/
static uint32_t NvGetFlashTableSize
(
  void
);


/******************************************************************************
 * Name: NvIsRamTableUpdated
 * Description: Checks if the the NV table from RAM memory has changed since
 *              last system reset (e.g. via an OTA transfer)
 * Parameter(s): -
 * Return: TRUE if the NV RAM table has been changed / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsRamTableUpdated
(
  void
);

/******************************************************************************
 * Name: NvGetTableEntry
 * Description: get the NV table entry information stored on FLASH memory
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pDataEntry - a pointer to a memory location where the
 *                                  entry information will be stored
 * Return: TRUE if the has been found / FALSE otherwise
 ******************************************************************************/
static bool_t NvGetTableEntry
(
  uint16_t tblEntryId,
  NVM_DataEntry_t* pDataEntry
);
#endif /* gNvUseExtendedFeatureSet_d */

#endif /* no FlexNVM */


/******************************************************************************
 * Name: NvGetEntryFromDataPtr
 * Description: get table and element indexes based on a generic pointer address
 * Parameter(s): [IN] pData - a pointer to a NVM RAM table
 *               [OUT] pIndex - a pointer to a memory location where the
 *                              requested indexed will be stored
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PointerOutOfRange_c - if the provided pointer cannot be founded
 *                                    within the RAM table
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetEntryFromDataPtr
(
  void* pData,
  NVM_TableEntryInfo_t* pIndex
);


/******************************************************************************
 * Name: NvWriteRecord
 * Description: writes a record
 * Parameter(s): [IN] tblIndexes - a pointer to table and element indexes
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvWriteRecord
(
  NVM_TableEntryInfo_t* tblIndexes
);


/******************************************************************************
 * Name: NvRestoreData
 * Description: restore an element from NVM storage to its original RAM location
 * Parameter(s): [IN] tblIdx - pointer to table and element indexes
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if page is empty
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvRestoreData
(
  NVM_TableEntryInfo_t* tblIdx
);


/******************************************************************************
 * Name: NvGetTableEntryIndex
 * Description: get the table entry index from the provided ID
 * Parameter(s): [IN] entryId - the ID of the table entry
 * Return: table entry index of gNvInvalidTableEntryIndex_c
 *****************************************************************************/
static uint16_t NvGetTableEntryIndexFromId
(
  NvTableEntryId_t entryId
);


/******************************************************************************
 * Name: NvAddSaveRequestToQueue
 * Description: Add save request to save requests queue; if the request is
 *              already stored, ignore the current request
 * Parameter(s): [IN] ptrTblIdx - pointer to table index
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_SaveRequestRejected_c - if the request couldn't be queued
 ******************************************************************************/
static NVM_Status_t NvAddSaveRequestToQueue
(
  NVM_TableEntryInfo_t* ptrTblIdx
);

/******************************************************************************
 * Name: NvIntervalTimerCallback
 * Description: Callback function of the timer used by the NvSaveOnInterval()
 * Parameter(s): [IN] timerID - timer ID
 * Return: -
 ******************************************************************************/
static void NvIntervalTimerCallback
(
  void*
);

/******************************************************************************
 * Name: GetRandomRange
 * Description: Returns a random number between 'low' and 'high'
 * Parameter(s): [IN] low, high - generated number range
 * Return: 0..255
 ******************************************************************************/
static uint8_t GetRandomRange
(
  uint8_t low,
  uint8_t high
);

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /*** FlexNVM ***/

/******************************************************************************
 * Name: NvGetFlexLastMetaInfo
 * Description: Get FlexRAM last meta information address
 * Parameter(s): -
 * Return: the address of the last valid meta information
 ******************************************************************************/
static uint32_t NvGetFlexLastMetaInfo
(
  void
);

/******************************************************************************
 * Name: NvGetFlexMetaInfoFromId
 * Description: Get FlexRAM meta information tag from table entry ID
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 meta information tag will be stored
 * Return: -
 ******************************************************************************/
static void NvGetFlexMetaInfoFromId
(
  NvTableEntryId_t tblEntryId,
  NVM_FlexMetaInfo_t* pMetaInfo
);

/******************************************************************************
 * Name: NvCheckNvmTableForFlexRAMUsage
 * Description: Check if the existing NVM table fits within the FlexRAM window
 * Parameter(s): -
 * Return: gNVM_NvTableExceedFlexRAMSize_c - the table exceed the size of
 *                                           FlexRAM window
 *         gNVM_OK_c - the table fits within the size of window FlexRAM window
 ******************************************************************************/
static NVM_Status_t NvCheckNvmTableForFlexRAMUsage
(
  void
);

#endif /* gNvUseFlexNVM_d */
#endif /* gNvStorageIncluded_d */


/*****************************************************************************
 *****************************************************************************
 * Private memory declarations
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/*
 * Name: mNvActivePageId
 * Description: variable that holds the ID of the active page
 */
#if !gNvDebugEnabled_d
static
#endif
NVM_VirtualPageID_t mNvActivePageId;

/*
 * Name: mNvPageCounter
 * Description: page counter, used to validate the entire virtual page
 *              and also to provide statistical information about
 *              how many times the virtual page was erased
 */
static uint32_t mNvPageCounter = 0;

/*
 * Name: mNvVirtualPageProperty
 * Description: virtual page properties
 */
NVM_VirtualPageProperties_t mNvVirtualPageProperty[gNvVirtualPagesCount_c];

/*
 * Name: mNvCopyOperationIsPending
 * Description: a flag that a indicates that a page copy operation is requested
 */
static bool_t mNvCopyOperationIsPending = FALSE;

/*
 * Name: mNvErasePgCmdStatus
 * Description: a data structure used to erase a virtual page. The erase of a
 *              virtual page is performed in idle task, in a sector-by-sector
 *              manner. When the idle task runs, if the erase pending flag is
 *              set, only one flash sector will be erased. Therefore, the
 *              virtual page will be entirely erased after several runs of
 *              idle task
 */
static NVM_ErasePageCmdStatus_t mNvErasePgCmdStatus;

/*
 * Name: maNvRecordsCpyIdx
 * Description: An array that stores the indexes of the records already copied;
 *              Used by the defragmentation process.
 */
#if gNvFragmentation_Enabled_d
static uint16_t maNvRecordsCpyIdx[gNvRecordsCopiedBufferSize_c];
#endif /* gNvFragmentation_Enabled_d */

#if gNvUseExtendedFeatureSet_d
/*
 * Name: mNvTableSizeInFlash
 * Description: the size of the NV table stored in the FLASH memory
 */
#if !gNvDebugEnabled_d
static
#endif
uint32_t mNvTableSizeInFlash;

/*
 * Name: mNvTableMarker
 * Description: FLASH NV table marker, used only for code readability
 *              (when applying the sizeof() operator to it)
 */
static uint16_t mNvTableMarker = gNvTableMarker_c;

/*
 * Name: mNvTableMarker
 * Description: FLASH NV application version, used for determining when table upgrade
 *              happened
 */
#if !gNvDebugEnabled_d
static
#endif
uint16_t mNvFlashTableVersion = gNvFlashTableVersion_c;

/*
 * Name: mNvTableUpdated
 * Description: boolean flag used to mark if the NV table from the RAM memory
 *              has been changed. Set (or left untouched) only at module initialization,
 *              when the existing NV FLASH table (if any) is compared against
 *              the NV RAM table.
 */
static bool_t mNvTableUpdated;

#endif /* gNvUseExtendedFeatureSet_d */

#endif /* no FlexNVM */

/*
 * Name: mNvModuleInitialized
 * Description: variable that holds the NVM initialisation status
 */
static bool_t mNvModuleInitialized = FALSE;


/*
 * Name: mNvFlashConfigInitialised
 * Description: variable that holds the hal driver and active page initialisation status
 */
static bool_t mNvFlashConfigInitialised = FALSE;
/*
 * Name: mNvCriticalSectionFlag
 * Description: If this counter is != 0, do not save to NV Storage
 */
static uint8_t mNvCriticalSectionFlag = 0;

/*
 * Name: gNvMinimumTicksBetweenSaves
 * Description: Minimum number of calls to NvTimerTick() between saves of a given data set
 */
static NvSaveInterval_t gNvMinimumTicksBetweenSaves = gNvMinimumTicksBetweenSaves_c;

/*
 * Name: gNvCountsBetweenSaves
 * Description: Minimum number of calls to NvSaveOnIdle() between saves of a given data set
 */
static NvSaveCounter_t gNvCountsBetweenSaves = gNvCountsBetweenSaves_c;

/*
 * Name: mNvPendingSavesQueue
 * Description: a queue used for storing information about the pending saves
 */
static NVM_SaveQueue_t mNvPendingSavesQueue;

/*
 * Name: maDatasetInfo
 * Description: Data set info table
 */
static NVM_DatasetInfo_t maDatasetInfo[gNvTableEntriesCountMax_c];

/*
 * Name: mNvSaveOnIntervalEvent
 * Description: flag used to signal an 'SaveOnInterval' event
 */
static bool_t mNvSaveOnIntervalEvent;

/*
 * Name: mNvSaveOnIntervalTimerID
 * Description: the ID of timer used by the Save-On-Interval functionality
 */
static tmrTimerID_t mNvSaveOnIntervalTimerID;
/*
 * Name: mNVMMutexId
 * Description: mutex used to ensure NVM functions thread switch safety
 */
mutex_t mNVMMutex;
#endif /* gNvStorageIncluded_d */

/*****************************************************************************
 *****************************************************************************
 * Public memory declarations
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/*
 * Name: NV_STORAGE_END_ADDRESS
 * Description: NV_STORAGE_END_ADDRESS from linker command file is used by this code
 *              as Raw Sector Start Address. This should not be misleading because
 *              ENVM module writes meta information in address-ascending order
 *              and records in address-descending order.
 */
extern uint32_t NV_STORAGE_END_ADDRESS[];

/*
 * Name: NV_STORAGE_SECTOR_SIZE
 * Description: external symbol from linker command file, it represents the size
 *              of a FLASH sector
 */
extern uint32_t NV_STORAGE_SECTOR_SIZE[];

/*
 * Name:  NV_STORAGE_MAX_SECTORS
 * Description: external symbol from linker command file, it represents the sectors
 *              count used by the ENVM storage system; it has to be a multiple of 2
 */
extern uint32_t  NV_STORAGE_MAX_SECTORS[];

#endif /* no FlexNVM */


#endif /* gNvStorageIncluded_d */


/*****************************************************************************
 *****************************************************************************
 * Private functions
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d
/******************************************************************************
 * Name: __NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element
 *               [IN] dataEntryType - the type of the new entry
 *               [IN] overwrite - if an existing table entry shall be
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_RegisterFailure_c - invalid id or unmirrored data set
 *         gNVM_AlreadyRegistered - the id is allready registered in another entry
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d
static NVM_Status_t __NvRegisterTableEntry
(
    void* ptrData,
    NvTableEntryId_t uniqueId,
    uint16_t elemCount,
    uint16_t elemSize,
    uint16_t dataEntryType,
    bool_t overwrite
)
{

    uint16_t loopCnt = 0;
    uint16_t nullPos = gNvTableEntriesCountMax_c;
    NVM_Status_t status;

#if ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0))
    NVM_TableEntryInfo_t tblIdx;
#endif

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if((gNvInvalidDataEntry_c == uniqueId) || (gNvEndOfTableId_c == uniqueId))
    {
        return gNVM_RegisterFailure_c;
    }

#if ((gNvUseFlexNVM_d == FALSE) && (DEBLOCK_SIZE == 0) && gNvFragmentation_Enabled_d)
    if (elemCount > gNvRecordsCopiedBufferSize_c)
    {
        return gNVM_DefragBufferTooSmall_c;
    }
#endif

    while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
    {
        if(pNVM_DataTable[loopCnt].pData == NULL &&
           !overwrite)
        {
            nullPos = loopCnt;
            break;
        }

        if(pNVM_DataTable[loopCnt].DataEntryID == uniqueId)
        {
            if(overwrite)
            {
                /*make sure that the NvWriteRamTable writes the updated values*/
                pNVM_DataTable[loopCnt].pData = ptrData;
                pNVM_DataTable[loopCnt].ElementsCount = elemCount;
                pNVM_DataTable[loopCnt].ElementSize = elemSize;
                pNVM_DataTable[loopCnt].DataEntryType = dataEntryType;
                /*force page copy first*/
                __NvEraseEntryFromStorage(uniqueId, loopCnt);
                return gNVM_OK_c;
            }
            else
            {
                return gNVM_AlreadyRegistered;
            }
        }
        /* increment the loop counter */
        loopCnt++;
    }

    if(gNvTableEntriesCountMax_c != nullPos)
    {
        pNVM_DataTable[nullPos].pData= ptrData;
        pNVM_DataTable[nullPos].DataEntryID = uniqueId;
        pNVM_DataTable[nullPos].ElementsCount = elemCount;
        pNVM_DataTable[nullPos].ElementSize = elemSize;
        pNVM_DataTable[nullPos].DataEntryType = dataEntryType;

        #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
        {
            /*update the flash table*/
            #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
                FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
                FSCI_MsgNVVirtualPageMonitoring(FALSE,status=NvCopyPage(gNvCopyAll_c));
            #else
                status = NvCopyPage(gNvCopyAll_c);
            #endif
        }
        #else
        {
            /* format the FlexRAM window */
            status = NvFormat();
            if (status != gNVM_OK_c)
                return status;
            status = NvAtomicSave(FALSE);
        }
        #endif
        return status;
    }

    return gNVM_RegisterFailure_c;
}
/******************************************************************************
 * Name: GetFlashTableVersion
 * Description: returns the flash table version
 * Parameter(s): -
 * Return: 0 or flash table version
 *****************************************************************************/
uint16_t GetFlashTableVersion
(
    void
)
{
    InitNVMConfig();
    if (mNvActivePageId == gVirtualPageNone_c)
        return 0;
    return (*(NVM_TableInfo_t*)(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress)).fields.NvTableVersion;
}

/******************************************************************************
 * Name: __NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV
 *              table. The RAM table must be updated before this call.
 * Parameter(s): [IN] entryId - the entry id of the entry that is removed
 *               [IN] tableEntryIndex - the index of the entry in the ram table
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
static NVM_Status_t __NvEraseEntryFromStorage
(
    uint16_t entryId,
    uint16_t tableEntryIndex
)
{
    index_t loopCnt;
    NVM_Status_t status;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(mNvCriticalSectionFlag)
    {
        return gNVM_CriticalSectionActive_c;
    }

    /* Check if is in pendding queue - if yes than remove it */
    if (NvGetPendingSavesCount(&mNvPendingSavesQueue))
    {
        /* Start from the queue's head */
        loopCnt = mNvPendingSavesQueue.Head;

        while(loopCnt != mNvPendingSavesQueue.Tail)
        {
            if(entryId == mNvPendingSavesQueue.QData[loopCnt].entryId)
            {
                mNvPendingSavesQueue.QData[loopCnt].entryId = gNvInvalidDataEntry_c;
                break;
            }
            /* increment and wrap the loop index */
            loopCnt = (loopCnt + 1)  & ((uint8_t)(gNvPendingSavesQueueSize_c - 1));
        }
    }
    maDatasetInfo[tableEntryIndex].countsToNextSave = gNvCountsBetweenSaves;
    maDatasetInfo[tableEntryIndex].saveNextInterval = FALSE;

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    /*if a previous call to nvidle is required before erasing this entry*/
    if (mNvCopyOperationIsPending)
        __NvIdle();
    /* erase the table entry by making a copy of the active page to the inactive one,
    * but skipping while copying the table entry to be erased */
    if (gEmptyPageMetaAddress_c != mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
            FSCI_MsgNVVirtualPageMonitoring(FALSE, status = NvCopyPage(entryId));
        #else
            status = NvCopyPage(entryId);
        #endif
    }
#else /* FlexNVM */
    /* format the FlexRAM window */
    status = NvFormat();
    if (gNVM_OK_c != status)
        return status;
    /* re-write the entire NVM table */
    status = NvAtomicSave(FALSE);
#endif /* FlexNVM */
    return status;
}

/******************************************************************************
 * Name: RecoverNvEntry
 * Description: Reads a flash entry so that the application can handle dinamic entries.
 * Parameter(s): [IN] index - the ram entry index
 *               [OUT] entry - the flash entry at the specified index
 * Return: gNVM_OK_c - if the operation completes successfully
           gNVM_RestoreFailure_c - if the operation failed
 *****************************************************************************/
NVM_Status_t RecoverNvEntry
(
    uint16_t index,
    NVM_DataEntry_t *entry
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    uint32_t addr;
    NVM_EntryInfo_t entryInfo;

    entry->pData = NULL;
    InitNVMConfig();
    if (mNvActivePageId == gVirtualPageNone_c)
    {
        return gNVM_RestoreFailure_c;
    }

    addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(NVM_TableInfo_t) + index * sizeof(NVM_EntryInfo_t);
    NV_FlashRead(addr, (uint8_t*)&entryInfo, sizeof(NVM_EntryInfo_t));
    entry->DataEntryID   = entryInfo.fields.NvDataEntryID;
    entry->DataEntryType = entryInfo.fields.NvDataEntryType;
    entry->ElementsCount = entryInfo.fields.NvElementsCount;
    entry->ElementSize   = entryInfo.fields.NvElementSize;
    return gNVM_OK_c;
#else
    return gNVM_RestoreFailure_c;
#endif
}


/******************************************************************************
 * Name: NvIsRamTableUpdated
 * Description: Checks if the the NV table from RAM memory has changed since
 *              last system reset (e.g. via an OTA transfer)
 * Parameter(s): -
 * Return: TRUE if the NV RAM table has been changed / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsRamTableUpdated
(
    void
)
{
    uint16_t idx;
    uint32_t addr;
    uint32_t endAddr;
    bool_t idFound;
    NVM_EntryInfo_t entryInfo;

    /* address = page raw sector start address + page counter size + table marker + table version */
    addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(NVM_TableInfo_t);

    /* compute the search end address */
    endAddr = addr + mNvTableSizeInFlash;

    do
    {
        /* read ID */
        NV_FlashRead(addr, (uint8_t*)&entryInfo, sizeof(NVM_EntryInfo_t));

        idFound = FALSE;
        idx = 0;

        while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
        {
            if(entryInfo.fields.NvDataEntryID == pNVM_DataTable[idx].DataEntryID)
            {
                idFound = TRUE;
                break;
            }

            /* increment the index */
            idx++;
        }

        if(!idFound)
        {
            return TRUE;
        }
        if ((entryInfo.fields.NvDataEntryType != pNVM_DataTable[idx].DataEntryType) ||
            (entryInfo.fields.NvElementsCount != pNVM_DataTable[idx].ElementsCount) ||
            (entryInfo.fields.NvElementSize   != pNVM_DataTable[idx].ElementSize))
        {
            return TRUE;
        }

        /* increment the address */
        addr += sizeof(NVM_EntryInfo_t);

    } while(addr < endAddr);

    return FALSE;
}

/******************************************************************************
 * Name: NvGetTableEntry
 * Description: get the NV table entry information stored on FLASH memory
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pDataEntry - a pointer to a memory location where the
 *                                  entry information will be stored
 * Return: TRUE if the has been found / FALSE otherwise
 ******************************************************************************/
static bool_t NvGetTableEntry
(
    uint16_t tblEntryId,
    NVM_DataEntry_t* pDataEntry
)
{
    uint32_t addr;
    uint64_t tmp;

    pDataEntry->pData = NULL; /* the data pointer is not saved on FLASH table and
    * shall not be used by the caller of this function */

    addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;

    NV_FlashRead(addr, (uint8_t*)&tmp, sizeof(NVM_TableInfo_t));
    if (((NVM_TableInfo_t*)&tmp)->fields.NvTableMarker != mNvTableMarker)
    {
        pDataEntry->DataEntryType = 0;
        pDataEntry->ElementsCount = 0;
        pDataEntry->ElementSize = 0;
        pDataEntry->DataEntryID = gNvInvalidDataEntry_c;
        return FALSE;
    }

    /* increment address */
    addr += sizeof(NVM_TableInfo_t);

    do
    {
        NV_FlashRead(addr, (uint8_t*)&tmp, sizeof(NVM_EntryInfo_t));

        if(((NVM_TableInfo_t*)&tmp)->fields.NvTableMarker == mNvTableMarker)
        {
            /* reached end of table */
            break;
        }
        if(((NVM_EntryInfo_t*)&tmp)->fields.NvDataEntryID == tblEntryId)
        {
            pDataEntry->DataEntryID   = ((NVM_EntryInfo_t*)&tmp)->fields.NvDataEntryID;
            pDataEntry->DataEntryType = ((NVM_EntryInfo_t*)&tmp)->fields.NvDataEntryType;
            pDataEntry->ElementsCount = ((NVM_EntryInfo_t*)&tmp)->fields.NvElementsCount;
            pDataEntry->ElementSize   = ((NVM_EntryInfo_t*)&tmp)->fields.NvElementSize;
            return TRUE;
        }

        /* continue searching */
        addr += sizeof(NVM_EntryInfo_t);
    } while(addr < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress);

    pDataEntry->DataEntryType = 0;
    pDataEntry->ElementsCount = 0;
    pDataEntry->ElementSize = 0;
    pDataEntry->DataEntryID = gNvInvalidDataEntry_c;
    return FALSE;
}

/******************************************************************************
 * Name: NvGetFlashTableSize
 * Description: Retrieves the size of the NV table
 * Parameter(s): -
 * Return: the NV table size
 ******************************************************************************/
static uint32_t NvGetFlashTableSize
(
    void
)
{
    uint32_t addr;
    uint32_t size = 0;
    NVM_TableInfo_t tableInfo;

    /* compute the size of the table stored in Flash memory */
    addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;

    NV_FlashRead(addr, (uint8_t*)&tableInfo, sizeof(NVM_TableInfo_t));

    if(gNvTableMarker_c != tableInfo.fields.NvTableMarker)
    {
        return 0;
    }

    addr += sizeof(NVM_TableInfo_t);

    do
    {
        NV_FlashRead(addr, (uint8_t*)&tableInfo, sizeof(NVM_TableInfo_t));

        if(gNvTableMarker_c == tableInfo.fields.NvTableMarker)
        {
            return size;
        }
        size += sizeof(NVM_TableInfo_t);
        addr += sizeof(NVM_TableInfo_t);

    } while(addr < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress);

    return 0;
}
#endif /* gNvUseExtendedFeatureSet_d */

/******************************************************************************
 * Name: __NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvAtomicSave
(
    bool_t ignoreCriticalSectionFlag
)
{
    NVM_Status_t status = gNVM_OK_c;
    index_t loopCnt = 0;
#if gUnmirroredFeatureSet_d
    index_t loopCnt2 = 0;
#endif

    while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
    {
        #if gUnmirroredFeatureSet_d
        if (pNVM_DataTable[loopCnt].DataEntryType == gNVM_NotMirroredInRam_c)
        {
            for (loopCnt2 = 0; loopCnt2<pNVM_DataTable[loopCnt].ElementsCount; loopCnt2++)
            {
                status = NvSyncSave(&((uint8_t**)pNVM_DataTable[loopCnt].pData)[loopCnt2], FALSE, ignoreCriticalSectionFlag);
                if((gNVM_CriticalSectionActive_c == status) || (gNVM_NullPointer_c == status))
                {
                    /* skip */
                    continue;
                }

                if(gNVM_OK_c != status)
                {
                    return status;
                }
            }
        }
        else
        #endif
        {
            status = NvSyncSave(pNVM_DataTable[loopCnt].pData, TRUE, ignoreCriticalSectionFlag);
            if((gNVM_CriticalSectionActive_c == status) || (gNVM_NullPointer_c == status))
            {
                /* skip */
                loopCnt++;
                continue;
            }

            if(gNVM_OK_c != status)
            {
                return status;
            }
        }

        /* increment the loop counter */
        loopCnt++;
    }
    return status;
}
/******************************************************************************
 * Name: __NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvSyncSave
(
    void* ptrData,
    bool_t saveAll,
    bool_t ignoreCriticalSectionFlag
)
{
    NVM_TableEntryInfo_t tblIdx;
    NVM_Status_t status;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(mNvCriticalSectionFlag && !ignoreCriticalSectionFlag)
    {
        NvAddSaveRequestToQueue(&tblIdx);
        return gNVM_CriticalSectionActive_c;
    }

    if(NULL == ptrData)
    {
        return gNVM_NullPointer_c;
    }

    if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    /* write the save all flag */
    #if gNvFragmentation_Enabled_d
    tblIdx.saveRestoreAll = saveAll;
    #else
    tblIdx.saveRestoreAll = TRUE;
    #endif /* gNvFragmentation_Enabled_d */

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    if((status = NvWriteRecord(&tblIdx)) == gNVM_PageCopyPending_c)
    {
        /* copy active page */
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
            FSCI_MsgNVVirtualPageMonitoring(FALSE,status=NvCopyPage(gNvCopyAll_c));
        #else
            status = NvCopyPage(gNvCopyAll_c);
        #endif
        if(status != gNVM_OK_c)
        {
            return status;
        }
        mNvCopyOperationIsPending = FALSE;

        /* erase old page */
        status = NvEraseVirtualPage(mNvErasePgCmdStatus.NvPageToErase);
        if (gNVM_OK_c != status)
            return status;
        mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
        mNvErasePgCmdStatus.NvErasePending = FALSE;
        /* write record */
        status = NvWriteRecord(&tblIdx);
    }
#else /* FlexNVM */
    /* write record */
    status = NvWriteRecord(&tblIdx);
    /* wait for EEPROM system to be ready (fix@ENGR283453) */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET,  FTFx_SSD_FCNFG_EEERDY)));
#endif
    return status;
}


/******************************************************************************
 * Name: __NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter/ram table to active page.
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a
 *                                        critical section
 *****************************************************************************/
static NVM_Status_t __NvFormat
(
    void
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    NVM_Status_t status;
    NVM_TableInfo_t tableInfo;
    #if gUnmirroredFeatureSet_d
    uint16_t loopCnt;
    uint16_t loopCnt2;
    #endif

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(mNvCriticalSectionFlag)
    {
        return gNVM_CriticalSectionActive_c;
    }

    NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress, (uint8_t*)&tableInfo, sizeof(NVM_TableInfo_t));

    if((status = NvInternalFormat(tableInfo.fields.NvPageCounter)) == gNVM_OK_c)
    {
        /* update last meta info address */
        (void)NvUpdateLastMetaInfoAddress();
    }
    #if gUnmirroredFeatureSet_d
    {
        for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
        {
            if((NULL == pNVM_DataTable[loopCnt].pData) && (gNvEndOfTableId_c == pNVM_DataTable[loopCnt].DataEntryID))
            {
                break;
            }
            if(pNVM_DataTable[loopCnt].DataEntryType == gNVM_NotMirroredInRam_c)
            {
                for (loopCnt2 = 0; loopCnt2 < pNVM_DataTable[loopCnt].ElementsCount; loopCnt2++)
                {
                    if (NvIsNVMFlashAddress(((void**)pNVM_DataTable[loopCnt].pData)[loopCnt2]))
                    {
                        ((void**)pNVM_DataTable[loopCnt].pData)[loopCnt2] = NULL;
                    }
                }
            }
        }
    }
    #endif
    return status;

#else /* FlexNVM */

    uint8_t buff[gNvFlexFormatBufferSize_c];
    uint32_t addr;
    uint16_t size;

    FLib_MemSet(buff, gNvErasedFlashCellValue_c, gNvFlexFormatBufferSize_c);

    addr = gFlashConfig.EERAMBase;
    size = gFlashConfig.EEESize;

    while(size)
    {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET , FTFx_SSD_FCNFG_EEERDY)));

        if(FTFx_OK != EEEWrite(&gFlashConfig, addr, sizeof(buff), buff))
        {
            return gNVM_FormatFailure_c;
        }
        size -= gNvFlexFormatBufferSize_c;
        addr += gNvFlexFormatBufferSize_c;
    }
    return gNVM_OK_c;
#endif /* gNvUseFlexNVM_d */
}

/******************************************************************************
 * Name: __NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
static void __NvIdle
(
    void
)
{

    NVM_TableEntryInfo_t tblIdx;
    #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    uint32_t status;
    #endif

    if (!mNvModuleInitialized || mNvCriticalSectionFlag)
    {
        return;
    }

    #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    if(mNvCopyOperationIsPending)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
            FSCI_MsgNVVirtualPageMonitoring(FALSE,status = NvCopyPage(gNvCopyAll_c));
        #else
            status = NvCopyPage(gNvCopyAll_c);
        #endif
        if (gNVM_OK_c == status)
        {
            mNvCopyOperationIsPending = FALSE;
        }
    }

    if(mNvErasePgCmdStatus.NvErasePending)
    {
        if(mNvErasePgCmdStatus.NvSectorAddress >= mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvRawSectorEndAddress)
        {
            /* all sectors of the page had been erased */
            mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
            mNvErasePgCmdStatus.NvErasePending = FALSE;
            #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvRawSectorStartAddress, gNVM_OK_c);
            #endif
            return;
        }

        /* erase */
        status = NV_FlashEraseSector(&gFlashConfig,
                                     mNvErasePgCmdStatus.NvSectorAddress,
                                     (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE),
                                     gFlashLaunchCommand);

        /* blank check */
        #if gNvDisableIntCmdSeq_c
        OSA_EnterCritical(kCriticalDisableInt);
        #endif
        if(FTFx_OK == FlashVerifySection(&gFlashConfig, mNvErasePgCmdStatus.NvSectorAddress,
                                         (uint32_t)(NV_STORAGE_SECTOR_SIZE) / FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT, READ_NORMAL_MARGIN, gFlashLaunchCommand))
        {
            mNvErasePgCmdStatus.NvSectorAddress += (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
            #if gNvDisableIntCmdSeq_c
            OSA_ExitCritical(kCriticalDisableInt);
            #endif
            return;
        }
        #if gNvDisableIntCmdSeq_c
        OSA_ExitCritical(kCriticalDisableInt);
        #endif
    }
    #endif

    /* process the save-on-interval requests */
    if(mNvSaveOnIntervalEvent)
    {
        __NvTimerTick(TRUE);
        mNvSaveOnIntervalEvent = FALSE;
    }

    /* process the save-on-idle requests */
    if(NvGetPendingSavesCount(&mNvPendingSavesQueue))
    {
        while(NvPopPendingSave(&mNvPendingSavesQueue, &tblIdx))
        {
            if(gNvInvalidDataEntry_c == tblIdx.entryId)
            {
                continue;
            }

            #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
            if(NvWriteRecord(&tblIdx) == gNVM_PageCopyPending_c)
            {
                NvAddSaveRequestToQueue(&tblIdx);
                break;
            }
            #else /* FlexNVM */
            NvWriteRecord(&tblIdx);
            #endif
        }
    }
}
/******************************************************************************
 * Name: __NvIsDataSetDirty
 * Description: return TRUE if the element pointed by ptrData is dirty
 * Parameters: [IN] ptrData - pointer to data to be checked
 * Return: TRUE if the element is dirty, FALSE otherwise
 ******************************************************************************/
bool_t __NvIsDataSetDirty
(
    void* ptrData
)
{
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);

    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIdx;
    index_t loopIdx;

    if(!mNvModuleInitialized)
    {
        return FALSE;
    }

    if(NULL == ptrData)
    {
        return gNVM_NullPointer_c;
    }

    if(gNVM_OK_c != NvGetEntryFromDataPtr(ptrData, &tblIdx))
    {
        return FALSE;
    }
    else
    {
        tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

        if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
        {
            return FALSE;
        }
        /* Check if is in pendding queue - if yes than remove it */
        if (mNvPendingSavesQueue.EntriesCount)
        {
            /* Start from the queue's head */
            loopIdx = mNvPendingSavesQueue.Head;

            while(loopIdx != mNvPendingSavesQueue.Tail)
            {
                if(mNvPendingSavesQueue.QData[loopIdx].entryId == tblIdx.entryId)
                {
                    return TRUE;
                }
                /* increment and wrap the loop index */
                loopIdx = (loopIdx + 1)  & ((uint8_t)(gNvPendingSavesQueueSize_c - 1));
            }
        }
        return maDatasetInfo[tableEntryIdx].saveNextInterval;
    }
}
/******************************************************************************
 * Name: __NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
static NVM_Status_t __NvRestoreDataSet
(
void* ptrData,
bool_t restoreAll
)
{
    NVM_TableEntryInfo_t tblIdx;
    #if gUnmirroredFeatureSet_d
    uint16_t tableEntryIdx;
    #endif
    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    NVM_Status_t nvmStatus;
    #endif

    if(!mNvModuleInitialized)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
        FSCI_MsgNVRestoreMonitoring(0, TRUE, (uint8_t)gNVM_ModuleNotInitialized_c);
        #endif
        return gNVM_ModuleNotInitialized_c;
    }

    if(NULL == ptrData)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
        FSCI_MsgNVRestoreMonitoring(0, TRUE, (uint8_t)gNVM_NullPointer_c);
        #endif
        return gNVM_NullPointer_c;
    }

    #if gNvFragmentation_Enabled_d
    tblIdx.saveRestoreAll = restoreAll;
    #else
    tblIdx.saveRestoreAll = TRUE;
    #endif /* gNvFragmentation_Enabled_d */

    if(NvGetEntryFromDataPtr(ptrData, &tblIdx) != gNVM_OK_c)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
        FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, TRUE, (uint8_t)gNVM_NullPointer_c);
        #endif
        return gNVM_PointerOutOfRange_c;
    }

    #if gUnmirroredFeatureSet_d
    /*make sure you can't request a full backup for unmirrored data sets*/
    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
    {
        tblIdx.saveRestoreAll = FALSE;
    }
    #endif

    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    {
        FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, TRUE, (uint8_t)gNVM_OK_c);
        nvmStatus=NvRestoreData(&tblIdx);
        FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, FALSE, (uint8_t)nvmStatus);
        return nvmStatus;
    }
    #else
        return NvRestoreData(&tblIdx);
    #endif
}

/******************************************************************************
 * Name: __NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
static bool_t __NvTimerTick
(
    bool_t countTick
)
{
    bool_t fTicksLeft;
    NVM_TableEntryInfo_t tblIdx;
    uint16_t idx;
    uint8_t timerJitter;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    idx = 0;
    fTicksLeft = FALSE;

    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
        if(countTick)
        {
            if(maDatasetInfo[idx].ticksToNextSave)
            {
                --maDatasetInfo[idx].ticksToNextSave;
            }
        }

        if(maDatasetInfo[idx].saveNextInterval)
        {
            if (maDatasetInfo[idx].ticksToNextSave)
            {
                fTicksLeft = TRUE;
            }
            else
            {
                tblIdx.entryId = pNVM_DataTable[idx].DataEntryID;
                #if gUnmirroredFeatureSet_d
                if (pNVM_DataTable[idx].DataEntryType == gNVM_NotMirroredInRam_c)
                {
                    tblIdx.elementIndex = maDatasetInfo[idx].elementIndex;
                    tblIdx.saveRestoreAll = FALSE;
                }
                else
                #endif
                {
                    tblIdx.elementIndex = 0;
                    tblIdx.saveRestoreAll = TRUE;
                }
                maDatasetInfo[idx].saveNextInterval = FALSE;
                if(!mNvCriticalSectionFlag)
                {
                    #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
                    if(NvWriteRecord(&tblIdx) == gNVM_PageCopyPending_c)
                    {
                        NvAddSaveRequestToQueue(&tblIdx);
                    }
                    #else /* FlexNVM */
                    NvWriteRecord(&tblIdx);
                    #endif
                }
                else
                {
                    /* push the pending save to pending queue */
                    NvAddSaveRequestToQueue(&tblIdx);
                }
            }
        }

        /* increment the loop counter */
        idx++;
    }
    if (fTicksLeft && !TMR_IsTimerActive(mNvSaveOnIntervalTimerID))
    {
        timerJitter = GetRandomRange(0,255);
        TMR_StartSingleShotTimer(mNvSaveOnIntervalTimerID,
                                 TmrSeconds(1) + timerJitter - 128,
                                 NvIntervalTimerCallback,
                                 NULL);
    }
    return fTicksLeft;
}/* NvTimerTick() */


/******************************************************************************
 * Name: __NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnCount
(
    void* ptrData
)
{
    NVM_Status_t status;
    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIdx;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(NULL == ptrData)
    {
        return gNVM_NullPointer_c;
    }

    /* get the NVM table entry */
    if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    if(gNvInvalidDataEntry_c == tblIdx.entryId)
    {
        return gNVM_InvalidTableEntry_c;
    }

    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }

    if(maDatasetInfo[tableEntryIdx].countsToNextSave)
    {
        --maDatasetInfo[tableEntryIdx].countsToNextSave;
    }
    else
    {
        /* all the elements of the NVM table entry will be saved */
        tblIdx.saveRestoreAll = TRUE;
        maDatasetInfo[tableEntryIdx].countsToNextSave = gNvCountsBetweenSaves;
        status = NvAddSaveRequestToQueue(&tblIdx);
    }

    return status;
}                                       /* NvSaveOnCount() */


/******************************************************************************
 * Name: __NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnInterval
(
    void* ptrData
)
{
    NVM_Status_t status;
    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIdx;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(NULL == ptrData)
    {
        return gNVM_NullPointer_c;
    }

    if(gTmrInvalidTimerID_c == mNvSaveOnIntervalTimerID)
    {
        /* try to allocate the timer used by the save-on-interval functionality */
        mNvSaveOnIntervalTimerID = TMR_AllocateTimer();

        if(gTmrInvalidTimerID_c == mNvSaveOnIntervalTimerID)
        {
            return gNVM_InvalidTimerID_c;
        }
    }

    /* get the NVM table entry */
    if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    if(gNvInvalidDataEntry_c == tblIdx.entryId)
    {
        return gNVM_InvalidTableEntry_c;
    }

    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }

    if(maDatasetInfo[tableEntryIdx].saveNextInterval == FALSE)
    {
        /* ticksToNextSave is +1 because mNvSaveOnIntervalEvent will cause NvIdle
        * to decrement the value before the timer ticks*/
        maDatasetInfo[tableEntryIdx].ticksToNextSave = gNvMinimumTicksBetweenSaves+1;
        maDatasetInfo[tableEntryIdx].saveNextInterval = TRUE;
#if gUnmirroredFeatureSet_d
        if (pNVM_DataTable[tableEntryIdx].DataEntryType == gNVM_NotMirroredInRam_c)
        {
            maDatasetInfo[tableEntryIdx].elementIndex = tblIdx.elementIndex;
        }
#endif
        mNvSaveOnIntervalEvent = TRUE;
    }

    return status;
}

/******************************************************************************
 * Name: __NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
static NVM_Status_t __NvSaveOnIdle
(
    void* ptrData,
    bool_t saveAll
)
{
    NVM_Status_t status;
    NVM_TableEntryInfo_t tblIdx;

    if(!mNvModuleInitialized)
    {
        return gNVM_ModuleNotInitialized_c;
    }

    if(NULL == ptrData)
    {
        return gNVM_NullPointer_c;
    }

    /* get the NVM table entry */
    if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    if(gNvInvalidDataEntry_c == tblIdx.entryId)
    {
        return gNVM_InvalidTableEntry_c;
    }

    /* write the save all flag */
    #if gNvFragmentation_Enabled_d
    tblIdx.saveRestoreAll = saveAll;
    #else
    tblIdx.saveRestoreAll = TRUE;
    #endif /* gNvFragmentation_Enabled_d */

    return NvAddSaveRequestToQueue(&tblIdx);
}

/******************************************************************************
 * Name: InitNVMConfig
 * Description: Initialises the hal driver, and gets the active page.
 * Parameter(s): -
 * Return: -
 *****************************************************************************/
static void InitNVMConfig
(
    void
)
{
    if (mNvFlashConfigInitialised)
        return;
    mNvFlashConfigInitialised = TRUE;
    /* Initialize flash HAL driver */
    NV_Init();

    /* Initialize the active page ID */
    mNvActivePageId = gVirtualPageNone_c;

    /* First virtual page initialisation */
    mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress = (uint32_t)((uint8_t*)NV_STORAGE_END_ADDRESS);
    mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorsCount = (uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1;
    mNvVirtualPageProperty[gFirstVirtualPage_c].NvTotalPageSize = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorsCount *
        (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
    mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress +
        mNvVirtualPageProperty[gFirstVirtualPage_c].NvTotalPageSize - 1;

    /* Second virtual page initialisation */
    mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress + 1;
    mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorsCount = (uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1;
    mNvVirtualPageProperty[gSecondVirtualPage_c].NvTotalPageSize = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorsCount *
        (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
    mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorEndAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress +
        mNvVirtualPageProperty[gSecondVirtualPage_c].NvTotalPageSize - 1;

    /* Initialize the storage system: get active page and page counter */
    NvInitStorageSystem();
}
/******************************************************************************
 * Name: __NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found
 *         gNVM_OK_c - module was successfully initialized
 *****************************************************************************/
static NVM_Status_t __NvModuleInit
(
    void
)
{
    index_t loopCnt;
    bool_t eot = FALSE; /* end of table marker flag */
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    uint32_t pageFreeSpace;
    NVM_Status_t status;
#endif

    if(mNvModuleInitialized)
    {
        return gNVM_ModuleAlreadyInitialized_c;
    }

    /* check the RAM table to have 'End-Of-Table' terminator */
    for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
    {
        if((NULL == pNVM_DataTable[loopCnt].pData) && (gNvEndOfTableId_c == pNVM_DataTable[loopCnt].DataEntryID))
        {
            eot = TRUE;
            break;
        }
        #if ((gNvUseFlexNVM_d == FALSE) && (DEBLOCK_SIZE == 0) && (gNvFragmentation_Enabled_d == TRUE))
        if (pNVM_DataTable[loopCnt].ElementsCount > gNvRecordsCopiedBufferSize_c)
        {
            return gNVM_DefragBufferTooSmall_c;
        }
        #endif
    }
    if(!eot)
    {
        return gNVM_MissingEndOfTableMarker_c;
    }

    /* Initialize the pending saves queue */
    NvInitPendingSavesQueue(&mNvPendingSavesQueue);

    /* Initialize the data set info table */
    for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
    {
        maDatasetInfo[loopCnt].saveNextInterval = FALSE;
        maDatasetInfo[loopCnt].countsToNextSave = gNvCountsBetweenSaves;
    }

    /* initialize the event used by save-on-interval functionality */
    mNvSaveOnIntervalEvent = FALSE;

    /* initialize the timer used by the save-on-interval functionality */
    mNvSaveOnIntervalTimerID = gTmrInvalidTimerID_c;

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */
    /* check if the flash registers are configured to support FlexNVM as EEPROM,
    if not enable this feature*/
    if (0 == gFlashConfig.EEESize)
    {
        DEFlashPartition(&gFlashConfig, gNvEepromDatasetSizeCode_c, gNvEepromBackupSizeCode_c, gFlashLaunchCommand);
    }
    /* check if the NVM table fits within the size of the FlexRAM window */
    if(gNVM_OK_c != NvCheckNvmTableForFlexRAMUsage())
    {
        return gNVM_NvTableExceedFlexRAMSize_c;
    }

    /* check data flash IFR map */
    if(gFlashConfig.EEESize == 0)
    {
        return gNVM_NvWrongFlashDataIFRMap_c;
    }

    /* Enable the EERAM */
    SetEEEEnable(&gFlashConfig, EEE_ENABLE, FlashCommandSequence);

    /* NVM module is now initialized */
    mNvModuleInitialized = TRUE;
    return gNVM_OK_c;

#else /* no FlexNVM */

    /* check linker file symbol definition for sector count; it should be multiple of 2 */
    if (((uint32_t)NV_STORAGE_MAX_SECTORS) & 0x1)
    {
        return gNVM_InvalidSectorsCount_c;
    }
    InitNVMConfig();

    /* no pending erase operations on system initialisation */
    mNvErasePgCmdStatus.NvErasePending = FALSE;

    /* both pages are not valid, format the NV storage system */
    if (mNvActivePageId == gVirtualPageNone_c)
    {
        mNvActivePageId = gFirstVirtualPage_c;
        status = NvInternalFormat(0);
        if (status != gNVM_OK_c)
            return status;
    }
    #if gNvUseExtendedFeatureSet_d
    {
        /* get the size of the NV table stored in FLASH memory */
        mNvTableSizeInFlash = NvGetFlashTableSize();

        if(0 == mNvTableSizeInFlash) /* no NV table found in FLASH, format the system */
        {
            status = NvInternalFormat(0); /* will also save the NV table to FLASH memory */
            if (status != gNVM_OK_c)
                return status;
        }
        else /* found an valid NV table in FLASH memory */
        {
            /* check if the RAM table was updated (e.g. new binary image via OTA) */
            mNvTableUpdated = (GetFlashTableVersion() != mNvFlashTableVersion) || NvIsRamTableUpdated();
            if( mNvTableUpdated )
            {
                if(gNVM_OK_c == NvUpdateLastMetaInfoAddress())
                {
                    /* copy the new RAM table and the page content */
                    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
                        FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
                        status = NvCopyPage(gNvCopyAll_c);
                        FSCI_MsgNVVirtualPageMonitoring(FALSE,status);
                    #else
                        status = NvCopyPage(gNvCopyAll_c);
                    #endif

                    /* NVM module is now initialised */
                    mNvModuleInitialized = TRUE;
                    mNvTableUpdated = FALSE;
                    return status;
                }

                /* format the system */
                if(gNVM_OK_c == NvInternalFormat(0))
                {
                    if(gNVM_OK_c == NvUpdateLastMetaInfoAddress())
                    {
                        /* NVM module is now initialised */
                        mNvModuleInitialized = TRUE;
                        return gNVM_OK_c;
                    }
                }
                return gNVM_FormatFailure_c;
            }
        }
    }
    #endif /* gNvUseExtendedFeatureSet_d */

    /* get the last meta information address */
    if(gNVM_OK_c == (status = NvUpdateLastMetaInfoAddress()))
    {
        /* NVM module is now initialized */
        mNvModuleInitialized = TRUE;

        /* get active page free space */
        if(gNVM_OK_c != (status = NvGetPageFreeSpace(&pageFreeSpace)))
        {
            return status;
        }
        if(pageFreeSpace < gNvMinimumFreeBytesCountStart_c )
        {
            #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
                FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
                status = NvCopyPage(gNvCopyAll_c);
                FSCI_MsgNVVirtualPageMonitoring(FALSE,status);
            #else
                status = NvCopyPage(gNvCopyAll_c);
            #endif
        }
        return status;
    }
    else
    {
        /* format the system */
        if(gNVM_OK_c == NvInternalFormat(0))
        {
            if(FTFx_OK == NvUpdateLastMetaInfoAddress())
            {
                /* NVM module is now initialised */
                mNvModuleInitialized = TRUE;
                return gNVM_OK_c;
            }
        }
        return gNVM_FormatFailure_c;
    }
#endif
}

#if gUnmirroredFeatureSet_d

/******************************************************************************
 * Name: NvIsRecordErased
 * Description: Checks the most recent metas to see if the unmirrored element
 *              was erased or is just uninitialised
 * Parameter(s): [IN] srcTableEntryIdx - the index of the entry to be checked
 *               [IN] srcTableEntryElementIdx - the element index
 *               [IN] srcMetaAddress - the starting address of the search
 * Return: TRUE if the element was erased with NvErase or FALSE otherwise
 *****************************************************************************/
bool_t NvIsRecordErased
(
    uint16_t srcTableEntryIdx,
    uint16_t srcTableEntryElementIdx,
    uint32_t srcMetaAddress
)
{
    NVM_RecordMetaInfo_t srcMetaInfo;
    while(srcMetaAddress < (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress))
    {
        (void)NvGetMetaInfo(mNvActivePageId, srcMetaAddress, &srcMetaInfo);
        if (gNvGuardValue_c == srcMetaInfo.rawValue)
        {
            break;
        }
        if ((srcMetaInfo.fields.NvmRecordOffset == 0) &&
            (srcMetaInfo.fields.NvmElementIndex == srcTableEntryElementIdx) &&
            (srcMetaInfo.fields.NvmDataEntryID == pNVM_DataTable[srcTableEntryIdx].DataEntryID))
        {
            return TRUE;
        }
        srcMetaAddress += sizeof(NVM_RecordMetaInfo_t);
    }
    return FALSE;
}

/******************************************************************************
 * Name: __NvmMoveToRam
 * Description: Move from NVM to Ram an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: pointer to Ram location
 *****************************************************************************/
static NVM_Status_t __NvmMoveToRam
(
    void** ppData
)
{
    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIndex;
    NVM_Status_t status;
    void* pData=NULL;
    index_t loopIdx;

    /* Get entry from NVM table */
    if((status = NvGetEntryFromDataPtr(ppData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    if((tableEntryIndex = NvGetTableEntryIndexFromId(tblIdx.entryId)) == gNvInvalidTableEntryIndex_c)
    {
        return gNVM_InvalidTableEntry_c;
    }

    if(gNVM_NotMirroredInRam_c != pNVM_DataTable[tableEntryIndex].DataEntryType)
    {
        return gNVM_IsMirroredDataSet_c;
    }

    /* Check if entry is in ram  */
    if(!NvIsNVMFlashAddress(*ppData)&&(*ppData != NULL))
    {
        /* Check if is in pendding queue - if yes than remove it */
        if (NvGetPendingSavesCount(&mNvPendingSavesQueue))
        {
            /* Start from the queue's head */
            loopIdx = mNvPendingSavesQueue.Head;

            while(loopIdx != mNvPendingSavesQueue.Tail)
            {
                if((tblIdx.entryId == mNvPendingSavesQueue.QData[loopIdx].entryId)&&
                   (tblIdx.elementIndex == mNvPendingSavesQueue.QData[loopIdx].elementIndex))
                {
                    mNvPendingSavesQueue.QData[loopIdx].entryId = gNvInvalidDataEntry_c;
                    break;
                }
                /* increment and wrap the loop index */
                loopIdx = (loopIdx + 1) & ((uint8_t)(gNvPendingSavesQueueSize_c - 1));
            }
        }
        maDatasetInfo[tableEntryIndex].saveNextInterval = FALSE;
        return gNVM_Error_c;
    }

    /* Allocate a buffer for the data set */
    pData = MSG_Alloc(pNVM_DataTable[tableEntryIndex].ElementSize);
    if(pData == NULL)
    {
        return gNVM_NoMemory_c;
    }

    /* Write from Flash to Ram */
    if(*ppData != NULL)
    {
        FLib_MemCpy(pData, *ppData, pNVM_DataTable[tableEntryIndex].ElementSize);
    }

    OSA_EnterCritical(kCriticalDisableInt);
    *ppData = pData;
    OSA_ExitCritical(kCriticalDisableInt);
    /* Check if the address is in ram */
    return gNVM_OK_c;
}

/******************************************************************************
 * Name: __NvmErase
 * Description: Erase from NVM an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be erased
 * Return: pointer to Ram location
 *****************************************************************************/
static NVM_Status_t __NvmErase
(
    void** ppData
)
{
    NVM_Status_t status;
    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIndex;

    /* Get entry from NVM table */
    if((status = NvGetEntryFromDataPtr(ppData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }

    if((tableEntryIndex = NvGetTableEntryIndexFromId(tblIdx.entryId)) == gNvInvalidTableEntryIndex_c)
    {
        return gNVM_InvalidTableEntry_c;
    }

    if(gNVM_NotMirroredInRam_c != pNVM_DataTable[tableEntryIndex].DataEntryType)
    {
        return gNVM_IsMirroredDataSet_c;
    }

    if(!NvIsNVMFlashAddress(*ppData))
    {
        if(*ppData != NULL)
        {
            MSG_Free(*ppData);
        }
        OSA_EnterCritical(kCriticalDisableInt);
        *ppData = NULL;
        OSA_ExitCritical(kCriticalDisableInt);
        return gNVM_OK_c;
    }

    OSA_EnterCritical(kCriticalDisableInt);
    *ppData = NULL;
    OSA_ExitCritical(kCriticalDisableInt);
    return __NvSyncSave(ppData,FALSE,TRUE);
}

/******************************************************************************
 * Name: NvIsNVMFlashAddress
 * Description: check if the address is in Flash
 * Parameter(s): [IN] address
 *
 * Return: TRUE if the table entry is in Flash / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsNVMFlashAddress
(
    void* address
)
{
    uint8_t idx;
    for(idx=0; idx < gNvVirtualPagesCount_c; idx ++)
    {
        if( ((uint32_t)address > mNvVirtualPageProperty[idx].NvRawSectorStartAddress) &&
           ((uint32_t)address < mNvVirtualPageProperty[idx].NvRawSectorEndAddress))
        {
            return TRUE;
        }
    }
    return FALSE;
}
#endif

/******************************************************************************
 * Name: NvGetRecordFullSize
 * Description: Computes the size of the specified table entry that will
 *              be written on FLASH memory
 * Parameter(s): [IN] tableEntryIndex - table entry index
 * Return: the computed size
 *****************************************************************************/
static uint32_t NvGetRecordFullSize
(
    NvTableEntryId_t tableEntryIndex
)
{
    uint32_t size;
    uint8_t paddingBytes;

    /* compute the RAM size */
    size = pNVM_DataTable[tableEntryIndex].ElementSize * pNVM_DataTable[tableEntryIndex].ElementsCount;

    /* compute the size that will be actually written on FLASH memory */
    paddingBytes = size % (uint8_t)PGM_SIZE_BYTE;

    if(paddingBytes)
    {
        size += (uint8_t)((uint8_t)PGM_SIZE_BYTE-paddingBytes);
    }

    return size;
}
/******************************************************************************
 * Name: NvInitPendingSavesQueue
 * Description: Initialize the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 * Return: TRUE if the pointer is valid, FALSE otherwise
 ******************************************************************************/
static bool_t NvInitPendingSavesQueue
(
    NVM_SaveQueue_t *pQueue
)
{
    if(NULL == pQueue)
    {
        return FALSE;
    }

    pQueue->Head = 0;
    pQueue->Tail = 0;
    pQueue->EntriesCount = 0;

    return TRUE;
}


/******************************************************************************
 * Name: NvPushPendingSave
 * Description: Add a new pending save to the queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [IN] data - data to be saved
 * Return: TRUE if the push operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPushPendingSave
(
    NVM_SaveQueue_t *pQueue,
    NVM_TableEntryInfo_t data
)
{
    if(NULL == pQueue)
    {
        return FALSE;
    }

    if((pQueue->Tail == pQueue->Head) && (pQueue->EntriesCount > 0))
    {
#if gFifoOverwriteEnabled_c
        /* increment the head (read index) */
        pQueue->Head = (pQueue->Head + 1) & ((unsigned char) (gNvPendingSavesQueueSize_c - 1));
#else
        return FALSE;
#endif
    }

    /* Add the item to queue */
    pQueue->QData[pQueue->Tail] = data;

    /* Reset the tail when it reach gNvPendingSavesQueueSize_c */
    pQueue->Tail = (pQueue->Tail + 1) % ((unsigned char) (gNvPendingSavesQueueSize_c));

    /* Increment the entries count */
    if(pQueue->EntriesCount < (unsigned char) (gNvPendingSavesQueueSize_c))
    {
        pQueue->EntriesCount++;
    }

    return TRUE;
}


/******************************************************************************
 * Name: NvPopPendingSave
 * Description: Retrieves the head element from the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [OUT] pData - pointer to the location where data will be placed
 * Return: TRUE if the pop operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPopPendingSave
(
    NVM_SaveQueue_t *pQueue,
    NVM_TableEntryInfo_t *pData
)
{
    if( (NULL == pQueue) || (pQueue->EntriesCount <= 0) || ((NULL == pData)) )
    {
        return FALSE;
    }

    *pData = pQueue->QData[pQueue->Head];

    /* Reset the head when it reach gNvPendingSavesQueueSize_c */
    pQueue->Head = (pQueue->Head + 1) % ((unsigned char) (gNvPendingSavesQueueSize_c ));

    /* Decrement the entries count */
    pQueue->EntriesCount--;

    return TRUE;
}
/******************************************************************************
 * Name: NvGetPendingSavesCount
 * Description: self explanatory
 * Parameters: [IN] pQueue - pointer to queue
 * Return: Number of pending saves
 ******************************************************************************/
static uint8_t NvGetPendingSavesCount
(
    NVM_SaveQueue_t *pQueue
)
{
    if( NULL == pQueue )
    {
        return 0;
    }
    return pQueue->EntriesCount;
}


/*****************************************************************
 * The below functions are compiled only if FlexNVM is NOT used
 *****************************************************************/

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/******************************************************************************
 * Name: NvEraseVirtualPage
 * Description: erase the specified page
 * Parameter(s): [IN] pageID - the ID of the page to be erased
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_SectorEraseFail_c - if the page cannot be erased
 *         gNVM_OK_c - if operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvEraseVirtualPage
(
    NVM_VirtualPageID_t pageID
)
{
    uint32_t status;

    if(pageID > gSecondVirtualPage_c)
        return gNVM_InvalidPageID_c;

    /* erase virtual page */
    status = NV_FlashEraseSector(&gFlashConfig,
                                 mNvVirtualPageProperty[pageID].NvRawSectorStartAddress,
                                 mNvVirtualPageProperty[pageID].NvTotalPageSize,
                                 gFlashLaunchCommand);
    if(FTFx_OK != status)
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[pageID].NvRawSectorStartAddress, status);
        #endif
        return gNVM_SectorEraseFail_c;
    }

    if(gNVM_OK_c != (status = NvVirtualPageBlankCheck(pageID)))
    {
        #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
            FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[pageID].NvRawSectorStartAddress, status);
        #endif
        return (NVM_Status_t)status;
    }
    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
        FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[pageID].NvRawSectorStartAddress, status);
    #endif
    return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvInitStorageSystem
 * Description: Initialize the storage system, retrieve the active page and
 *              the page counter. Called once by NvModuleInit() function.
 * Parameter(s): -
 * Return: -
 *****************************************************************************/
static void NvInitStorageSystem
(
    void
)
{
    NVM_TableInfo_t tableInfo;
    uint32_t firstPageCounterTopValue;
    uint32_t firstPageCounterBottomValue;
    uint32_t secondPageCounterTopValue;
    uint32_t secondPageCounterBottomValue;

    /* read both pages counter values */
    NV_FlashRead(mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress, (uint8_t*)&tableInfo,
                 sizeof(NVM_TableInfo_t));
    firstPageCounterTopValue = tableInfo.fields.NvPageCounter;
    NV_FlashRead(mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress - sizeof(NVM_TableInfo_t) + 1,
                 (uint8_t*)&tableInfo, sizeof(NVM_TableInfo_t));
    firstPageCounterBottomValue = tableInfo.fields.NvPageCounter;

    NV_FlashRead(mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress, (uint8_t*)&tableInfo,
                 sizeof(NVM_TableInfo_t));
    secondPageCounterTopValue = tableInfo.fields.NvPageCounter;
    NV_FlashRead(mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorEndAddress - sizeof(NVM_TableInfo_t) + 1,
                 (uint8_t*)&tableInfo, sizeof(NVM_TableInfo_t));
    secondPageCounterBottomValue = tableInfo.fields.NvPageCounter;

    mNvActivePageId = gVirtualPageNone_c;

    /* get the active page */
    if((firstPageCounterTopValue == firstPageCounterBottomValue) && (gPageCounterMaxValue_c != firstPageCounterTopValue)) /* first page is valid */
    {
        if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
        {
            if(firstPageCounterTopValue >= secondPageCounterTopValue)
            {
                /* first page is active */
                mNvPageCounter = firstPageCounterTopValue;
                mNvActivePageId = gFirstVirtualPage_c;
                return;
            }

            /* second page is active */
            mNvPageCounter = secondPageCounterTopValue;
            mNvActivePageId = gSecondVirtualPage_c;
            return;
        }

        if(secondPageCounterTopValue != secondPageCounterBottomValue)
        {
            /* first page is active */
            mNvPageCounter = firstPageCounterTopValue;
            mNvActivePageId = gFirstVirtualPage_c;
            /* request the erase of the second page */
            mNvErasePgCmdStatus.NvPageToErase = gSecondVirtualPage_c;
            mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress;
            mNvErasePgCmdStatus.NvErasePending = TRUE;
            return;
        }

        /* first page is active */
        mNvPageCounter = firstPageCounterTopValue;
        mNvActivePageId = gFirstVirtualPage_c;

        if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(gSecondVirtualPage_c))
        {
            /* request the erase of the second page */
            mNvErasePgCmdStatus.NvPageToErase = gSecondVirtualPage_c;
            mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress;
            mNvErasePgCmdStatus.NvErasePending = TRUE;
        }

        return;
    }

    if(firstPageCounterTopValue != firstPageCounterBottomValue) /* first page is not valid */
    {
        if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
        {
            /* second page is active */
            mNvPageCounter = secondPageCounterTopValue;
            mNvActivePageId = gSecondVirtualPage_c;
            /* request the erase of the first page */
            mNvErasePgCmdStatus.NvPageToErase = gFirstVirtualPage_c;
            mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress;
            mNvErasePgCmdStatus.NvErasePending = TRUE;
            return;
        }

        mNvActivePageId = gVirtualPageNone_c;
        return;
    }

    if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
    {
        /* second page is active */
        mNvPageCounter = secondPageCounterTopValue;
        mNvActivePageId = gSecondVirtualPage_c;

        if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(gFirstVirtualPage_c))
        {
            /* request the erase of the first page */
            mNvErasePgCmdStatus.NvPageToErase = gFirstVirtualPage_c;
            mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress;
            mNvErasePgCmdStatus.NvErasePending = TRUE;
        }
        return;
    }

    mNvActivePageId = gVirtualPageNone_c;
}

/******************************************************************************
 * Name: NvVirtualPageBlankCheck
 * Description: checks if the specified page is blank (erased)
 * Parameter(s): [IN] pageID - the ID of the page to be checked
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_PageIsNotBlank_c - if the page is not blank
 *         gNVM_OK_c - if the page is blank (erased)
 *****************************************************************************/
static NVM_Status_t NvVirtualPageBlankCheck
(
    NVM_VirtualPageID_t pageID
)
{
    if(pageID > gSecondVirtualPage_c)
        return gNVM_InvalidPageID_c;

    #if gNvDisableIntCmdSeq_c
    OSA_EnterCritical(kCriticalDisableInt);
    #endif

    if(FTFx_OK != FlashVerifySection(&gFlashConfig, mNvVirtualPageProperty[pageID].NvRawSectorStartAddress,
                                     (mNvVirtualPageProperty[pageID].NvTotalPageSize / FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT), READ_NORMAL_MARGIN, gFlashLaunchCommand))
    {
        #if gNvDisableIntCmdSeq_c
        OSA_ExitCritical(kCriticalDisableInt);
        #endif
        return gNVM_PageIsNotBlank_c;
    }
    #if gNvDisableIntCmdSeq_c
    OSA_ExitCritical(kCriticalDisableInt);
    #endif
    return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvUpdateLastMetaInfoAddress
 * Description: retrieve and store (update) the last meta information address
 * Parameter(s): -
 * Return: gNVM_MetaNotFound_c - if no meta information has been found
 *         gNVM_OK_c - if the meta was found and stored (updated)
 *****************************************************************************/
static NVM_Status_t NvUpdateLastMetaInfoAddress
(
    void
)
{
    uint32_t readAddress;
    NVM_RecordMetaInfo_t metaValue;

    readAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c;

    while(readAddress < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress)
    {
        NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));

        if(gNvGuardValue_c == metaValue.rawValue)
        {
            if(readAddress == (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
            {
                mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
                #if gUnmirroredFeatureSet_d
                    mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = gEmptyPageMetaAddress_c;
                #endif
                return gNVM_OK_c;
            }

            readAddress -= sizeof(NVM_RecordMetaInfo_t);

            while(readAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
            {
                NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));

                if((metaValue.fields.NvValidationStartByte == metaValue.fields.NvValidationEndByte) &&
                   ((gValidationByteSingleRecord_c == metaValue.fields.NvValidationStartByte) ||
                    (gValidationByteAllRecords_c == metaValue.fields.NvValidationStartByte)))
                {
                    mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = readAddress;
                    #if gUnmirroredFeatureSet_d
                    {
                        while(readAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
                        {
                            if(metaValue.fields.NvmRecordOffset == 0)
                            {
                                readAddress -= sizeof(NVM_RecordMetaInfo_t);
                                NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));
                            }
                            else
                            {
                                mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = readAddress;
                                break;
                            }
                        }
                    }
                    #endif
                    return gNVM_OK_c;
                }
                readAddress -= sizeof(NVM_RecordMetaInfo_t);
            }
            return gNVM_MetaNotFound_c;
        }
        readAddress += sizeof(NVM_RecordMetaInfo_t);
    }
    return gNVM_MetaNotFound_c;
}


/******************************************************************************
 * Name: NvGetMetaInfo
 * Description: get meta information based on the meta information address
 * Parameter(s): [IN] pageID - the ID of the page
 *               [IN] metaInfoAddress - meta information address
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 requested meta information will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_AddressOutOfRange_c - if the provided address is out of range
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetMetaInfo
(
    NVM_VirtualPageID_t pageId,
    uint32_t metaInfoAddress,
    NVM_RecordMetaInfo_t* pMetaInfo
)
{
    /* check address range */
    if(metaInfoAddress < (mNvVirtualPageProperty[pageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c) ||
       metaInfoAddress > mNvVirtualPageProperty[pageId].NvRawSectorEndAddress)
    {
        return gNVM_AddressOutOfRange_c;
    }

    /* read the meta information tag */
    NV_FlashRead(metaInfoAddress, (uint8_t*)pMetaInfo, sizeof(NVM_RecordMetaInfo_t));

    return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvGetPageFreeSpace
 * Description: return the page free space, in bytes
 * Parameter(s): [OUT] ptrFreeSpace - a pointer to a memory location where the
 *                                    page free space will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if the page is empty
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetPageFreeSpace
(
    uint32_t* ptrFreeSpace
)
{
    NVM_RecordMetaInfo_t metaInfo;
    NVM_Status_t retVal;
    #if gUnmirroredFeatureSet_d
    NVM_RecordMetaInfo_t metaInfoUndeleted;
    NVM_Status_t ret;
    #endif

    if(gEmptyPageMetaAddress_c == mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress)
    {
        #if gNvUseExtendedFeatureSet_d
            *ptrFreeSpace = mNvVirtualPageProperty[mNvActivePageId].NvTotalPageSize - mNvTableSizeInFlash - 3*sizeof(NVM_TableInfo_t);
        #else
            *ptrFreeSpace = mNvVirtualPageProperty[mNvActivePageId].NvTotalPageSize - 2*sizeof(NVM_TableInfo_t);
        #endif /* gNvUseExtendedFeatureSet_d */
        retVal = gNVM_OK_c;
    }
    else
    {
        retVal = NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress, &metaInfo);

        if(gNVM_OK_c == retVal)
        {
            #if gUnmirroredFeatureSet_d
            {
                if(metaInfo.fields.NvmRecordOffset == 0)
                {
                    ret=NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress, &metaInfoUndeleted);
                    if(gNVM_OK_c != ret)
                    {
                        *ptrFreeSpace = 0;
                        return ret;
                    }
                    else
                    {
                        metaInfo.fields.NvmRecordOffset = metaInfoUndeleted.fields.NvmRecordOffset;
                    }
                }
            }
            #endif
            *ptrFreeSpace = (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset) -
                (mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress +
                 sizeof(NVM_RecordMetaInfo_t));
        }
        else
        {
            *ptrFreeSpace = 0;
        }
    }
    return retVal;
}


/******************************************************************************
 * Name: NvIsMemoryAreaAvailable
 * Description: checks if the specified memory area is blank (erased)
 * Parameter(s): [IN] address - start address
 *               [IN] len - length to be verified
 * Return: TRUE if the area is available (blank), FALSE otherwise
 *****************************************************************************/
static bool_t NvIsMemoryAreaAvailable
(
    uint32_t address,
    uint32_t len
)
{
    NV_baseType readBuffer;

    if(len % (uint8_t)PGM_SIZE_BYTE != 0)
    {
        return FALSE;
    }


    if(address < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress ||
       address > mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress)
    {
        return FALSE;
    }

    if((address + len) > mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress + 1)
    {
        return FALSE;
    }

    while(len)
    {

        readBuffer = *(NV_baseType*)address;
        /*the buffer must have only 1 bits*/
        if (~readBuffer)
            return FALSE;
        len-=(uint8_t)PGM_SIZE_BYTE;
        address+=(uint8_t)PGM_SIZE_BYTE;
    }
    return TRUE;
}


/******************************************************************************
 * Name: NvIsRecordCopied
 * Description: Checks if a record or an entire table entry is already copied.
 *              Called by page copy function.
 * Parameter(s): [IN] pageId - the ID of the page where to perform the check
 *               [IN] metaInf - a pointer to source page meta information tag
 * Return: TRUE if the element is already copied, FALSE otherwise
 *****************************************************************************/
static bool_t NvIsRecordCopied
(
    NVM_VirtualPageID_t pageId,
    NVM_RecordMetaInfo_t* metaInf
)
{
    uint32_t loopAddress;
    NVM_RecordMetaInfo_t metaValue;
    bool_t retVal;

    loopAddress = mNvVirtualPageProperty[pageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c;
    retVal = FALSE;

    do
    {
        /* read the meta information tag */
        NV_FlashRead(loopAddress, (uint8_t*)&metaValue, sizeof(NVM_RecordMetaInfo_t));

        if(gNvGuardValue_c == metaValue.rawValue)
        {
            /* reached last meta */
            break;
        }

        if(metaValue.fields.NvValidationStartByte != metaValue.fields.NvValidationEndByte)
        {
            /* invalid meta */
            loopAddress += sizeof(NVM_RecordMetaInfo_t);
            continue;
        }

        if(metaInf->fields.NvmDataEntryID == metaValue.fields.NvmDataEntryID)
        {
            if(metaInf->fields.NvValidationStartByte == gValidationByteSingleRecord_c)
            {
                if(metaValue.fields.NvValidationStartByte == gValidationByteSingleRecord_c)
                {
                    if(metaValue.fields.NvmElementIndex == metaInf->fields.NvmElementIndex)
                    {
                        retVal = TRUE;
                        break;
                    }

                    /* skip */
                    loopAddress += sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }
                retVal = TRUE;
                break;
            }

            if(metaInf->fields.NvValidationStartByte == gValidationByteAllRecords_c)
            {
                if(metaValue.fields.NvValidationStartByte == gValidationByteSingleRecord_c)
                {
                    /* skip */
                    loopAddress += sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }
                retVal = TRUE;
                break;
            }

            /* skip */
            loopAddress += sizeof(NVM_RecordMetaInfo_t);
            continue;
        }

        loopAddress += sizeof(NVM_RecordMetaInfo_t);

    } while(loopAddress < mNvVirtualPageProperty[pageId].NvRawSectorEndAddress);

    return retVal;
}


/******************************************************************************
 * Name: NvInternalCopy
 * Description: Performs a copy of an record / entire table entry
 * Parameter(s): [IN] dstAddress - destination record address
 *               [IN] dstMetaAddress - destination meta address
 *               [IN] srcMetaInfo - source meta information
 *               [IN] srcTblEntryIdx - source table entry index
 *               [IN] size - bytes to copy
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvInternalCopy
(
    uint32_t dstAddress,
    uint32_t dstMetaAddress,
    NVM_RecordMetaInfo_t* srcMetaInfo,
    uint16_t srcTblEntryIdx,
    uint16_t size
)
{
    uint16_t innerOffset;
    uint8_t cacheBuffer[gNvCacheBufferSize_c];
    NVM_RecordMetaInfo_t dstMetaInfo;
    uint16_t diffSize = 0;
    uint16_t diffIdx = 0;
    uint16_t ramSize = 0;
    uint8_t misalignedBytes;
    uint8_t loopIdx;
    uint16_t loopEnd;

    /* Initialize the inner offset*/
    innerOffset = 0;

    /* prepare destination page meta info tag and write if after the record is entirely written.
    * the preparation is made here because the 'dstAddress' may change afterwards
    */
    dstMetaInfo.fields = srcMetaInfo->fields;
    dstMetaInfo.fields.NvmRecordOffset = dstAddress - mNvVirtualPageProperty[(mNvActivePageId+1)%2].NvRawSectorStartAddress;

    ramSize = pNVM_DataTable[srcTblEntryIdx].ElementsCount * pNVM_DataTable[srcTblEntryIdx].ElementSize;

    /* if the bytes to copy are less then RAM table entry space, the supplementary bytes to write on the destination page
    * will be retrieved from RAM table entry. This is the case when the RAM table has been updated and the new
    * entry's elements count is greater then the one existing in the previous RAM table, now stored in the FLASH active page
    * (source page) */
    if(size < ramSize)
    {
        #if gUnmirroredFeatureSet_d
        if(gNVM_NotMirroredInRam_c == pNVM_DataTable[srcTblEntryIdx].DataEntryType)
        {
            diffSize = 0;
        }
        else
        #endif
        {
            diffSize = ramSize - size;
        }
        diffIdx = size / pNVM_DataTable[srcTblEntryIdx].ElementSize;
    }

    while(size)
    {
        if(size > (uint16_t)gNvCacheBufferSize_c)
        {
            /* copy from FLASH to cache buffer */
            NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + srcMetaInfo->fields.NvmRecordOffset + innerOffset,
                         (uint8_t*)&cacheBuffer[0], (uint16_t)gNvCacheBufferSize_c);

            /* write to destination page */
            if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, (uint16_t)gNvCacheBufferSize_c, cacheBuffer, gFlashLaunchCommand))
            {
                /* update the destination record address copy */
                dstAddress += (uint16_t)gNvCacheBufferSize_c;
                /* update the record size */
                size -= (uint16_t)gNvCacheBufferSize_c;
                /* update the inner offset value */
                innerOffset += (uint16_t)gNvCacheBufferSize_c;

                continue;
            }
            return gNVM_RecordWriteError_c;
        }
        else
        {
            /* copy from FLASH to cache buffer */
            NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + srcMetaInfo->fields.NvmRecordOffset + innerOffset,
                         (uint8_t*)&cacheBuffer[0], size);
            /* write to destination page */
            if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, (uint16_t)size, cacheBuffer, gFlashLaunchCommand))
            {
                break;
            }
            return gNVM_RecordWriteError_c;
        }
    }

    if(diffSize)
    {
        /* update the destination record address copy */
        dstAddress += size;

        /* check alignment and adjust it if necessary */
        misalignedBytes = dstAddress - (dstAddress & ((uint32_t)~(PGM_SIZE_BYTE-1)));

        /* initialise the inner offset */
        innerOffset = 0;

        /* check if the destination is longword aligned or not */
        if(misalignedBytes)
        {
            /* align to previous 32 bit boundary */
            dstAddress &= ((uint32_t)~(PGM_SIZE_BYTE-1));

            /* compute the loop end */
            loopEnd = PGM_SIZE_BYTE - misalignedBytes;

            /* read from destination page to cache buffer */
            NV_FlashRead(dstAddress, (uint8_t*)&cacheBuffer[0], PGM_SIZE_BYTE);

            /* update with data from RAM */
            for(loopIdx = 0; loopIdx < loopEnd; loopIdx++)
            {
                cacheBuffer[misalignedBytes] = *((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData +
                                                 (diffIdx * pNVM_DataTable[srcTblEntryIdx].ElementSize) + innerOffset);
                innerOffset++;
                misalignedBytes++;
                if (innerOffset == diffSize)
                    break;
            }

            /* write to Flash destination page */
            if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, PGM_SIZE_BYTE, cacheBuffer, gFlashLaunchCommand))
            {
                return gNVM_RecordWriteError_c;
            }

            /* align to next 32 bit boundary */
            dstAddress += PGM_SIZE_BYTE;
        }

        /* write to Flash destination page the rest of the alligned data */
        if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, diffSize - innerOffset,
                                               ((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData + (diffIdx * pNVM_DataTable[srcTblEntryIdx].ElementSize) + innerOffset),
                                               gFlashLaunchCommand))
        {
            return gNVM_RecordWriteError_c;
        }
    }

    /* write the associated record meta information */
    if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstMetaAddress, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&dstMetaInfo), gFlashLaunchCommand))
    {
        return gNVM_MetaInfoWriteError_c;
    }
    return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvGetTblEntryMetaAddrFromId
 * Description: Gets the table entry meta address based on table entry ID
 * Parameter(s): [IN] searchStartAddress - the search start address
 *               [IN] dataEntryId - table entry ID
 * Return: the value of the meta address
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static uint32_t NvGetTblEntryMetaAddrFromId
(
    uint32_t searchStartAddress,
    uint16_t dataEntryId
)
{
    NVM_RecordMetaInfo_t metaInfo;

    while(searchStartAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
    {
        (void)NvGetMetaInfo(mNvActivePageId, searchStartAddress, &metaInfo);

        if((metaInfo.fields.NvValidationStartByte != gValidationByteAllRecords_c) ||
           (metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte))
        {
            searchStartAddress -= sizeof(NVM_RecordMetaInfo_t);
            continue;
        }

        if(metaInfo.fields.NvmDataEntryID == dataEntryId)
        {
            /* found it */
            return searchStartAddress;
        }

        searchStartAddress -= sizeof(NVM_RecordMetaInfo_t);
    }
    return 0;
}

/******************************************************************************
 * Name: NvInternalDefragmentedCopy
 * Description: Performs defragmentation and copy from the source page to
 *              the destination one
 * Parameter(s): [IN] srcMetaAddr - source page meta address
 *               [IN] pSrcMetaInf - pointer to source page meta information
 *               [IN] srcTblEntryIdx - source page table entry index
 *               [IN] dstMetaAddr - destination meta address
 *               [IN] dstRecordAddr - destination record address (to copy to)
 * Return: the status of the operation
 *****************************************************************************/
static NVM_Status_t NvInternalDefragmentedCopy
(
    uint32_t srcMetaAddr,
    NVM_RecordMetaInfo_t* pSrcMetaInf,
    uint16_t srcTblEntryIdx,
    uint32_t dstMetaAddr,
    uint32_t dstRecordAddr
)
{
    uint32_t metaAddress;
    uint32_t tblEntryMetaAddress;
    NVM_RecordMetaInfo_t dstMetaInfo;
    NVM_RecordMetaInfo_t metaInfo;
    uint32_t destination;
    uint16_t elemSize;
    uint16_t elemSizeCopy;
    uint16_t innerOffset;
    uint8_t misalignedBytes;
    uint16_t recordsCopiedCurrentIdx;
    uint16_t recordIdx;
    bool_t recordIsCopied;
    /* copy buffers */
    uint8_t srcBuffer[PGM_SIZE_BYTE];
    uint8_t dstBuffer[PGM_SIZE_BYTE];
    /* loop control variables */
    uint16_t loopIdx;
    uint8_t loopEnd;
    #if gNvUseExtendedFeatureSet_d
    NVM_DataEntry_t flashDataEntry;
    bool_t fillFromRAM = FALSE;
    #endif /* gNvUseExtendedFeatureSet_d */
    /* status variable */
    NVM_Status_t status = gNVM_OK_c;

    /* search for a full table entry that owns the record */
    tblEntryMetaAddress = NvGetTblEntryMetaAddrFromId(srcMetaAddr, pSrcMetaInf->fields.NvmDataEntryID);

    if(tblEntryMetaAddress != 0)
    {
        /* found it */
        metaAddress = srcMetaAddr;

        /* reset the copied records index */
        recordsCopiedCurrentIdx = 0;

        /* clear the records copied buffer */
        for(loopIdx = 0; loopIdx < (uint16_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
        {
            maNvRecordsCpyIdx[loopIdx] = (uint16_t)gNvInvalidElementIndex_c;
        }

        while(metaAddress > tblEntryMetaAddress)
        {
            /* get meta information */
            NvGetMetaInfo(mNvActivePageId, metaAddress, &metaInfo);

            /* skip invalid entries and full table records */
            if((metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte) ||
               (metaInfo.fields.NvValidationStartByte != gValidationByteSingleRecord_c))
            {
                metaAddress -= sizeof(NVM_RecordMetaInfo_t);
                continue;
            }

            if(metaInfo.fields.NvmDataEntryID == pSrcMetaInf->fields.NvmDataEntryID) /* found it */
            {
                /* check if record is already copied */
                recordIsCopied = FALSE;

                for(loopIdx = 0; loopIdx < (uint16_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
                {
                    if(metaInfo.fields.NvmElementIndex == maNvRecordsCpyIdx[loopIdx])
                    {
                        recordIsCopied = TRUE;
                        break;
                    }
                }
                if(recordIsCopied)
                {
                    /* skip elements already copied */
                    metaAddress -= sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }

                /* check if the element still belongs to an valid RAM table entry */
                if(metaInfo.fields.NvmElementIndex >= pNVM_DataTable[srcTblEntryIdx].ElementsCount)
                {
                    /* the FLASH element is no longer a current RAM table entry element */
                    metaAddress -= sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }

                /* get element size */
                elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;

                /* make a copy not to alter the original value and compute destination address where the element will be written */
                destination = dstRecordAddr + (metaInfo.fields.NvmElementIndex * elemSize);

                /* check alignment and adjust it if necessary */
                misalignedBytes = destination - (destination & ((uint32_t)~(PGM_SIZE_BYTE-1)));

                /* check if the destination is longword aligned or not */
                if(misalignedBytes)
                {
                    /* align to previous 32 bit boundary */
                    destination &= ((uint32_t)~(PGM_SIZE_BYTE-1));
                }

                innerOffset = 0;

                /* compute the loop end */
                if(elemSize < (PGM_SIZE_BYTE-misalignedBytes))
                {
                    loopEnd = elemSize;
                }
                else
                {
                    loopEnd = PGM_SIZE_BYTE - misalignedBytes;
                }

                while(elemSize)
                {
                    /* read (destination) */
                    NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], PGM_SIZE_BYTE);

                    /* read (source) */
                    NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + innerOffset,
                                 (uint8_t*)&srcBuffer[0], loopEnd);

                    /* modify */
                    for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
                    {
                        dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
                    }

                    /* write (destination) */
                    if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, PGM_SIZE_BYTE, dstBuffer, gFlashLaunchCommand))
                    {
                        elemSize -= loopEnd;
                        innerOffset += loopEnd;
                        destination += PGM_SIZE_BYTE;
                        misalignedBytes = 0;
                        if(elemSize >= PGM_SIZE_BYTE)
                        {
                            loopEnd = PGM_SIZE_BYTE;
                        }
                        else
                        {
                            loopEnd = elemSize;
                        }
                    }
                    else
                    {
                        return gNVM_RecordWriteError_c;
                    }
                }

                /* save the copied record offset */
                maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = metaInfo.fields.NvmElementIndex;
                /* increment (and wrap if necessary) the index */
                recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((uint16_t) (gNvRecordsCopiedBufferSize_c - 1));
            }

            /* continue searching */
            metaAddress -= sizeof(NVM_RecordMetaInfo_t);
        }

        /*
        * now copy the elements from table entry, except the one already copied from single records
        */

        NvGetMetaInfo(mNvActivePageId, tblEntryMetaAddress, &metaInfo);

        #if gNvUseExtendedFeatureSet_d
        fillFromRAM = FALSE;

        if(mNvTableUpdated) /* RAM table was updated */
        {
            if(NvGetTableEntry(pNVM_DataTable[srcTblEntryIdx].DataEntryID, &flashDataEntry))
            {
                if(pNVM_DataTable[srcTblEntryIdx].ElementsCount > flashDataEntry.ElementsCount)
                {
                    /* fill the FLASH destination page with the default RAM value for the missing element(s) */
                    fillFromRAM = TRUE;
                }
            }
        }
        #endif /* gNvUseExtendedFeatureSet_d */

        for(recordIdx = 0; recordIdx < pNVM_DataTable[srcTblEntryIdx].ElementsCount; recordIdx++)
        {
            #if gNvUseExtendedFeatureSet_d
            if(mNvTableUpdated)
            {
                if(recordIdx >= flashDataEntry.ElementsCount)
                    break;
            }
            #endif

            recordIsCopied = FALSE;

            for(loopIdx = 0; loopIdx < (uint16_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
            {
                if(recordIdx == maNvRecordsCpyIdx[loopIdx])
                {
                    recordIsCopied = TRUE;
                    break;
                }
            }
            if(recordIsCopied)
            {
                /* skip already copied elements */
                continue;
            }

            /*
            * copy the element
            */

            /* refresh the element size */
            elemSizeCopy = elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;
            /* make a copy not to alter the original value and compute destination address where the element will be written */
            destination = dstRecordAddr + (recordIdx * elemSize);
            /* check alignment and adjust it if necessary */
            misalignedBytes = destination - (destination & ((uint32_t)~(PGM_SIZE_BYTE-1)));
            /* check if the destination is longword aligned or not */
            if(misalignedBytes)
            {
                /* align to previous 32 bit boundary */
                destination &= ((uint32_t)~(PGM_SIZE_BYTE-1));
            }

            innerOffset = 0;

            /* compute the loop end */
            if(elemSizeCopy < (PGM_SIZE_BYTE-misalignedBytes))
            {
                loopEnd = elemSizeCopy;
            }
            else
            {
                loopEnd = PGM_SIZE_BYTE - misalignedBytes;
            }

            while(elemSizeCopy)
            {
                /* read (destination) */
                NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], PGM_SIZE_BYTE);

                /* read (FLASH source) */
                NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset +
                             (recordIdx * elemSize) + innerOffset, (uint8_t*)&srcBuffer[0], loopEnd);

                /* modify */
                for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
                {
                    dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
                }

                /* write (destination) */
                if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, PGM_SIZE_BYTE, dstBuffer, gFlashLaunchCommand))
                {
                    elemSizeCopy -= loopEnd;
                    innerOffset += loopEnd;
                    destination += PGM_SIZE_BYTE;
                    misalignedBytes = 0;
                    if(elemSizeCopy >= PGM_SIZE_BYTE)
                    {
                        loopEnd = PGM_SIZE_BYTE;
                    }
                    else
                    {
                        loopEnd = elemSizeCopy;
                    }
                }
                else
                {
                    return gNVM_RecordWriteError_c;
                }
            }

            /* save the copied record offset */
            maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = recordIdx;
            /* increment (and wrap if necessary) the index */
            recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((uint16_t) (gNvRecordsCopiedBufferSize_c - 1));
        }

        #if gNvUseExtendedFeatureSet_d
        /* append the elements that were not previously stored in FLASH memory, if any */
        if(fillFromRAM)
        {
            for(; recordIdx < pNVM_DataTable[srcTblEntryIdx].ElementsCount; recordIdx++)
            {
                /* refresh the element size */
                elemSizeCopy = elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;

                /* make a copy not to alter the original value and compute destination address where the element will be written */
                destination = dstRecordAddr + (recordIdx * elemSize);

                /* check alignment and adjust it, if necessary */
                misalignedBytes = destination - (destination & ((uint32_t)~(PGM_SIZE_BYTE-1)));

                /* check if the destination is longword aligned or not */
                if(misalignedBytes)
                {
                    /* align to previous 32 bit boundary */
                    destination &= ((uint32_t)~(PGM_SIZE_BYTE-1));
                }

                innerOffset = 0;

                /* compute the loop end */
                if(elemSizeCopy < (PGM_SIZE_BYTE-misalignedBytes))
                {
                    loopEnd = elemSizeCopy;
                }
                else
                {
                    loopEnd = PGM_SIZE_BYTE - misalignedBytes;
                }

                while(elemSizeCopy)
                {
                    /* read (destination) */
                    NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], PGM_SIZE_BYTE);

                    /* read (RAM source) */
                    NV_FlashRead((uint32_t)((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData + (recordIdx * elemSize) + innerOffset),
                                 (uint8_t*)&srcBuffer[0], loopEnd);

                    /* modify */
                    for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
                    {
                        dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
                    }

                    /* write (destination) */
                    if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, PGM_SIZE_BYTE, dstBuffer, gFlashLaunchCommand))
                    {
                        elemSizeCopy -= loopEnd;
                        innerOffset += loopEnd;
                        destination += PGM_SIZE_BYTE;
                        misalignedBytes = 0;
                        if(elemSizeCopy >= PGM_SIZE_BYTE)
                        {
                            loopEnd = PGM_SIZE_BYTE;
                        }
                        else
                        {
                            loopEnd = elemSizeCopy;
                        }
                    }
                    else
                    {
                        return gNVM_RecordWriteError_c;
                    }
                }
                /* save the copied record offset */
                maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = recordIdx;
                /* increment (and wrap if necessary) the index */
                recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((uint16_t) (gNvRecordsCopiedBufferSize_c - 1));
            }
        }
        #endif /* gNvUseExtendedFeatureSet_d */

        /* write meta information tag */
        dstMetaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
        dstMetaInfo.fields.NvmDataEntryID = pSrcMetaInf->fields.NvmDataEntryID;
        dstMetaInfo.fields.NvmElementIndex = 0;
        dstMetaInfo.fields.NvmRecordOffset = dstRecordAddr - mNvVirtualPageProperty[(mNvActivePageId+1)%2].NvRawSectorStartAddress;
        dstMetaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;

        /* write the associated record meta information */
        if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstMetaAddr, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&dstMetaInfo), gFlashLaunchCommand))
        {
            return gNVM_MetaInfoWriteError_c;
        }
    }
    else
    {
        /* an entire table entry was not found, so copy the record as it is */
        if((status = NvInternalCopy(dstRecordAddr, dstMetaAddr, pSrcMetaInf, srcTblEntryIdx, pNVM_DataTable[srcTblEntryIdx].ElementSize)) != gNVM_OK_c)
        {
            return status;
        }
        #if gUnmirroredFeatureSet_d
        if(gNVM_NotMirroredInRam_c == pNVM_DataTable[srcTblEntryIdx].DataEntryType)
        {
            OSA_EnterCritical(kCriticalDisableInt);
            /* set the pointer to the flash data */
            if (NvIsNVMFlashAddress(((void**)pNVM_DataTable[srcTblEntryIdx].pData)[pSrcMetaInf->fields.NvmElementIndex]))
            {
                ((uint8_t**)pNVM_DataTable[srcTblEntryIdx].pData)[pSrcMetaInf->fields.NvmElementIndex] = (uint8_t*)dstRecordAddr;
            }
            OSA_ExitCritical(kCriticalDisableInt);
        }
        #endif
    }
    return status;
}


#endif /* gNvFragmentation_Enabled_d */

/******************************************************************************
 * Name: NvCopyPage
 * Description: Copy the active page content to the mirror page. Only the
 *              latest table entries / elements are copied. A merge operation
 *              is performed before copy if an entry has single elements
 *              saved priori and newer than the table entry. If one or more
 *              elements were singular saved and the NV page doesn't has a
 *              full table entry saved, then the elements are copied as they
 *              are.
 * Parameter(s): [IN] skipEntryId - the entry ID to be skipped when page
 *                                  copy is performed
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvCopyPage
(
    NvTableEntryId_t skipEntryId
)
{
    /* source page related variables */
    uint32_t srcMetaAddress;
    NVM_RecordMetaInfo_t srcMetaInfo;
    uint16_t srcTableEntryIdx;

    /* destination page related variables */
    uint32_t dstMetaAddress;
    uint32_t firstMetaAddress;
    #if gUnmirroredFeatureSet_d
    uint8_t paddingBytes;
    uint16_t recordSize;
    #endif
    NVM_VirtualPageID_t dstPageId;
    uint32_t dstRecordAddress;

    #if gNvUseExtendedFeatureSet_d
    uint16_t idx;
    bool_t entryFound;
    NVM_DataEntry_t flashDataEntry;
    bool_t tableUpgraded;
    #endif /* gNvUseExtendedFeatureSet_d */
    uint32_t bytesToCopy;

    /* status variable */
    NVM_Status_t status;

    dstPageId = (NVM_VirtualPageID_t)((mNvActivePageId+1)%2);

    /* Check if the destination page is blank. If not, erase it. */
    if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(dstPageId))
    {
        status = NvEraseVirtualPage(dstPageId);
        if(gNVM_OK_c != status)
        {
            return status;
        }
    }
    /* initialise the destination page meta info start address */
    dstMetaAddress = mNvVirtualPageProperty[dstPageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c;
    #if gNvUseExtendedFeatureSet_d
    if (mNvTableUpdated)
        tableUpgraded = (GetFlashTableVersion() != mNvFlashTableVersion);
    #endif

    firstMetaAddress = dstMetaAddress;
    /*if src is an empty page, just copy the table and make the initialisations*/
    srcMetaAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
    if (srcMetaAddress != gEmptyPageMetaAddress_c)
    {
        /* initialise the destination page record start address */
        dstRecordAddress = mNvVirtualPageProperty[dstPageId].NvRawSectorEndAddress - sizeof(NVM_TableInfo_t) + 1;

        while(srcMetaAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
        {
            /* get current meta information */
            (void)NvGetMetaInfo(mNvActivePageId, srcMetaAddress, &srcMetaInfo);

            #if gNvUseExtendedFeatureSet_d
            /* NV RAM table has been updated */
            if(mNvTableUpdated)
            {
                idx = 0;
                entryFound = FALSE;

                /* check if the saved entry is still present in the new RAM table */
                while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
                {
                    if(srcMetaInfo.fields.NvmDataEntryID == pNVM_DataTable[idx].DataEntryID)
                    {
                        entryFound = TRUE;
                        break;
                    }
                    idx++;
                }

                if(!entryFound)
                {
                    /* move to the next meta info */
                    srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }
            }
            #endif /* gNvUseExtendedFeatureSet_d */

            /* get table entry index */
            srcTableEntryIdx = NvGetTableEntryIndexFromId(srcMetaInfo.fields.NvmDataEntryID);

            if((srcMetaInfo.fields.NvValidationStartByte != srcMetaInfo.fields.NvValidationEndByte) ||
               (srcTableEntryIdx == gNvInvalidDataEntry_c) ||
               (srcMetaInfo.fields.NvmDataEntryID == skipEntryId) ||
               NvIsRecordCopied(dstPageId, &srcMetaInfo))
            {
                /* go to the next meta information tag */
                srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                continue;
            }

            if((srcMetaInfo.fields.NvValidationStartByte != gValidationByteSingleRecord_c) &&
               (srcMetaInfo.fields.NvValidationStartByte != gValidationByteAllRecords_c))
            {
                /* go to the next meta information tag */
                srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                continue;
            }

            #if gUnmirroredFeatureSet_d
            if(gNVM_NotMirroredInRam_c == pNVM_DataTable[srcTableEntryIdx].DataEntryType)
            {
                /*check if the data was erased using NvErase or is just uninitialised*/
                if (NULL == ((void**)pNVM_DataTable[srcTableEntryIdx].pData)[srcMetaInfo.fields.NvmElementIndex] &&
                    NvIsRecordErased(srcTableEntryIdx, srcMetaInfo.fields.NvmElementIndex, srcMetaAddress))
                {
                    /* go to the next meta information tag */
                    srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                    continue;
                }
                else
                    /* compute the destination record start address */
                {
                    /* compute the 'real record size' taking into consideration that the FTFL controller only writes in burst of 4 bytes */
                    recordSize = pNVM_DataTable[srcTableEntryIdx].ElementSize;
                    paddingBytes = recordSize % (uint8_t)PGM_SIZE_BYTE;
                    if(paddingBytes)
                    {
                        recordSize += (uint8_t)((uint8_t)PGM_SIZE_BYTE-paddingBytes);
                    }

                    dstRecordAddress -= recordSize;

                    bytesToCopy = pNVM_DataTable[srcTableEntryIdx].ElementSize;
                }
            }
            else
            #endif
            {
                /* compute the destination record start address */
                dstRecordAddress -= NvGetRecordFullSize(srcTableEntryIdx);

                bytesToCopy = pNVM_DataTable[srcTableEntryIdx].ElementsCount * pNVM_DataTable[srcTableEntryIdx].ElementSize;
            }

            #if gNvUseExtendedFeatureSet_d
            /* NV RAM table has been updated */
            if(mNvTableUpdated)
            {
                if(NvGetTableEntry(pNVM_DataTable[srcTableEntryIdx].DataEntryID, &flashDataEntry))
                {
                    if ((flashDataEntry.DataEntryType != pNVM_DataTable[srcTableEntryIdx].DataEntryType) ||
                        (flashDataEntry.ElementSize != pNVM_DataTable[srcTableEntryIdx].ElementSize))
                    {
                        srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                        continue;
                    }

                    if(flashDataEntry.ElementsCount != pNVM_DataTable[srcTableEntryIdx].ElementsCount)
                    {
                        if (tableUpgraded)
                        {
                            if (flashDataEntry.ElementsCount < pNVM_DataTable[srcTableEntryIdx].ElementsCount)
                            {
                                /* copy only the bytes that were previously written to FLASH virtual page */
                                bytesToCopy = flashDataEntry.ElementsCount * flashDataEntry.ElementSize;
                            }
                        }
                        else
                        {
                            srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
                            continue;
                        }
                    }
                }
            }
            #endif /* gNvUseExtendedFeatureSet_d */

            #if gNvFragmentation_Enabled_d
            /*
            * single element record
            */
            if(srcMetaInfo.fields.NvValidationStartByte == gValidationByteSingleRecord_c)
            {
                if((status = NvInternalDefragmentedCopy(srcMetaAddress, &srcMetaInfo, srcTableEntryIdx, dstMetaAddress, dstRecordAddress)) != gNVM_OK_c)
                {
                    return status;
                }
            }
            else
            #endif /* gNvFragmentation_Enabled_d */
            /*
            * full table entry
            */
            if((status = NvInternalCopy(dstRecordAddress, dstMetaAddress, &srcMetaInfo, srcTableEntryIdx, bytesToCopy)) != gNVM_OK_c)
            {
                return status;
            }

            /* update destination meta information address */
            dstMetaAddress += sizeof(NVM_RecordMetaInfo_t);

            /* move to the next meta info */
            srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
        };
    }
    /* make a request to erase the old page */
    mNvErasePgCmdStatus.NvPageToErase = mNvActivePageId;
    mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
    mNvErasePgCmdStatus.NvErasePending = TRUE;

    /* update the the active page ID */
    mNvActivePageId = dstPageId;

    /* update the last meta info address */
    if(dstMetaAddress == firstMetaAddress)
    {
        mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
    }
    else
    {
        mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = dstMetaAddress - sizeof(NVM_RecordMetaInfo_t);
    }

    #if gUnmirroredFeatureSet_d
    mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
    #endif

    mNvPageCounter++;
    /* save the current RAM table */
    if(!NvSaveRamTable(dstPageId))
    {
        return gNVM_Error_c;
    }

    #if gNvUseExtendedFeatureSet_d
    if(mNvTableUpdated)
    {
        /* update the size of the NV table stored in FLASH */
        mNvTableSizeInFlash = NvGetFlashTableSize();

        /* clear the flag */
        mNvTableUpdated = FALSE;
    }
    #endif /* gNvUseExtendedFeatureSet_d */
    return gNVM_OK_c;
}

/******************************************************************************
 * Name: NvInternalFormat
 * Description: Format the NV storage system. The function erases in place both
 *              virtual pages and then writes the page counter value to first
 *              virtual page. The provided page counter value is automatically
 *              incremented and then written to first (active) virtual page.
 * Parameter(s): [IN] pageCounterValue - the page counter value that will
 *                                       be incremented and then written to
 *                                       active page
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvInternalFormat
(
    uint32_t pageCounterValue
)
{
    uint8_t retryCount = gNvFormatRetryCount_c;

    /* increment the page counter value */
    if(pageCounterValue == (uint32_t)gPageCounterMaxValue_c - 1)
    {
        pageCounterValue = 1;
    }
    else
    {
        pageCounterValue++;
    }
    mNvPageCounter = pageCounterValue;

    while(retryCount--)
    {
        /* erase first page */
        if (gNVM_OK_c == NvEraseVirtualPage(gFirstVirtualPage_c) &&
            gNVM_OK_c == NvEraseVirtualPage(gSecondVirtualPage_c))
            break;
    }

    /* active page after format = first virtual page */
    mNvActivePageId = gFirstVirtualPage_c;

    /* save NV table from RAM memory to FLASH memory */
    if (FALSE == NvSaveRamTable(mNvActivePageId))
        return gNVM_FormatFailure_c;

    #if gNvUseExtendedFeatureSet_d
    /* update the size of the NV table stored in FLASH */
    mNvTableSizeInFlash = NvGetFlashTableSize();
    #endif

    /* update the page counter value */
    mNvPageCounter = pageCounterValue;

    return gNVM_OK_c;
}

/******************************************************************************
 * Name: NvSaveRamTable
 * Description: Saves the NV table
 * Parameter(s): [IN] pageId - the virtual page ID where the table will be
 *                             saved
 * Return: TRUE if table saved successfully, FALSE otherwise
 ******************************************************************************/
static bool_t NvSaveRamTable
(
    NVM_VirtualPageID_t pageId
)
{
    uint64_t tmp;
    uint32_t addr;
#if gNvUseExtendedFeatureSet_d
    uint16_t idx;
#endif

    if(NULL == pNVM_DataTable)
        return FALSE;

    /* write table qualifier start */
    addr = mNvVirtualPageProperty[pageId].NvRawSectorStartAddress;

#if gNvUseExtendedFeatureSet_d
    tmp = ((NVM_TableInfo_t){.fields.NvPageCounter  = mNvPageCounter,
                             .fields.NvTableMarker  = mNvTableMarker,
                             .fields.NvTableVersion = mNvFlashTableVersion}).rawValue;
#else
    tmp = ((NVM_TableInfo_t){.fields.NvPageCounter  = mNvPageCounter}).rawValue;
#endif

    /*write page counter, table amrker, table version top*/
    if(FTFx_OK != FlashProgram(&gFlashConfig, addr, sizeof(NVM_TableInfo_t), (uint8_t*)(&tmp), gFlashLaunchCommand))
    {
        return FALSE;
    }

    #if gNvUseExtendedFeatureSet_d
    addr += sizeof(NVM_TableInfo_t);
    idx = 0;
    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
        /* write data entry ID */
        tmp = ((NVM_EntryInfo_t){.fields.NvDataEntryID   = pNVM_DataTable[idx].DataEntryID,
               .fields.NvDataEntryType = pNVM_DataTable[idx].DataEntryType,
               .fields.NvElementsCount = pNVM_DataTable[idx].ElementsCount,
               .fields.NvElementSize   = pNVM_DataTable[idx].ElementSize}).rawValue;

        if(FTFx_OK != FlashProgram(&gFlashConfig, addr, sizeof(NVM_EntryInfo_t), (uint8_t*)(&tmp), gFlashLaunchCommand))
        {
            return FALSE;
        }
        /* increment address */
        addr += sizeof(NVM_EntryInfo_t);

        /* increment table entry index */
        idx++;
    }

    tmp = ((NVM_TableInfo_t){.fields.NvTableMarker = mNvTableMarker}).rawValue;
    /* write table qualifier end, the rest 6 bytes are left 0x00 */
    if(FTFx_OK != FlashProgram(&gFlashConfig, addr, sizeof(NVM_EntryInfo_t), (uint8_t*)(&tmp), gFlashLaunchCommand))
    {
        return FALSE;
    }
    #endif

    /*write page counter bottom*/
    tmp = ((NVM_TableInfo_t){.fields.NvPageCounter = mNvPageCounter}).rawValue;
    if(FTFx_OK != FlashProgram(&gFlashConfig, (mNvVirtualPageProperty[pageId].NvRawSectorEndAddress - sizeof(NVM_TableInfo_t) + 1),
                               sizeof(NVM_TableInfo_t), (uint8_t*)&tmp, gFlashLaunchCommand))
        return FALSE;
    return TRUE;
}


#endif /* no FlexNVM */


/******************************************************************************
 * Name: NvGetEntryFromDataPtr
 * Description: get table and element indexes based on a generic pointer address
 * Parameter(s): [IN] pData - a pointer to a NVM RAM table
 *               [OUT] pIndex - a pointer to a memory location where the
 *                              requested indexed will be stored
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PointerOutOfRange_c - if the provided pointer cannot be founded
 *                                    within the RAM table
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetEntryFromDataPtr
(
    void* pData,
    NVM_TableEntryInfo_t* pIndex
)
{
    uint16_t idx = 0;

    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
        if(((uint8_t*)pData >= (uint8_t*)pNVM_DataTable[idx].pData))
        {
            #if gUnmirroredFeatureSet_d
            if(gNVM_NotMirroredInRam_c == pNVM_DataTable[idx].DataEntryType)
            {
                if ((uint8_t*)pData < ((uint8_t*)pNVM_DataTable[idx].pData + (sizeof(void*) * pNVM_DataTable[idx].ElementsCount)))
                {
                    pIndex->elementIndex = (((uint32_t)pData - (uint32_t)pNVM_DataTable[idx].pData) / sizeof(void*));
                    pIndex->entryId = pNVM_DataTable[idx].DataEntryID;
                    return gNVM_OK_c;
                }
                idx++;
                continue;
            }
            else
            #endif
            if ((uint8_t*)pData < ((uint8_t*)pNVM_DataTable[idx].pData + (pNVM_DataTable[idx].ElementSize * pNVM_DataTable[idx].ElementsCount)))
            {
                pIndex->elementIndex = (((uint32_t)pData - (uint32_t)pNVM_DataTable[idx].pData)/(pNVM_DataTable[idx].ElementSize));
                pIndex->entryId = pNVM_DataTable[idx].DataEntryID;
                return gNVM_OK_c;
            }
        }
        /* increment the loop counter */
        idx++;
    }
    return gNVM_PointerOutOfRange_c;
}


/******************************************************************************
 * Name: NvWriteRecord
 * Description: writes a record
 * Parameter(s): [IN] tblIndexes - a pointer to table and element indexes
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvWriteRecord
(
    NVM_TableEntryInfo_t* tblIndexes
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    uint32_t metaInfoAddress;
    uint32_t lastRecordAddress;
    uint32_t newRecordAddress;
    NVM_RecordMetaInfo_t metaInfo;
    uint32_t realRecordSize;
    uint32_t totalRecordSize; /* record + meta */
    uint32_t pageFreeSpace;
    uint8_t paddingBytes;
    bool_t doWrite;
    uint32_t srcAddress;
#else /* FlexNVM */
    uint32_t lastFlexMetaInfoAddress;
    NVM_FlexMetaInfo_t lastFlexMetaInfo;
    NVM_FlexMetaInfo_t flexMetaInfo;
    uint32_t destRecordEndAddress;
#endif

    uint16_t tableEntryIdx;
    uint32_t recordSize;

    tableEntryIdx = NvGetTableEntryIndexFromId(tblIndexes->entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */

    recordSize = pNVM_DataTable[tableEntryIdx].ElementsCount * pNVM_DataTable[tableEntryIdx].ElementSize;

    NvGetFlexMetaInfoFromId(tblIndexes->entryId, &flexMetaInfo);

    if(flexMetaInfo.rawValue == gNvFlexGuardValue_c) /* no meta found for this table entry ID */
    {
        /* set entry ID */
        flexMetaInfo.fields.NvDataEntryID = tblIndexes->entryId;
        /* get last meta info tag address */
        lastFlexMetaInfoAddress = NvGetFlexLastMetaInfo();

        if(lastFlexMetaInfoAddress < gFlashConfig.EERAMBase) /* FlexRAM empty */
        {
            flexMetaInfo.fields.NvDataOffset = gFlashConfig.EEESize - recordSize;
            destRecordEndAddress = gFlashConfig.EERAMBase + gFlashConfig.EEESize;
            lastFlexMetaInfoAddress = gFlashConfig.EERAMBase;
        }
        else
        {
            /* wait for EEPROM system to be ready */
            while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
            /* read last meta tag */
            NV_FlashRead(lastFlexMetaInfoAddress, (uint8_t*)&lastFlexMetaInfo, sizeof(NVM_FlexMetaInfo_t));
            /* compute record destination end address */
            destRecordEndAddress = gFlashConfig.EERAMBase + lastFlexMetaInfo.fields.NvDataOffset;
            /* compute record offset */
            flexMetaInfo.fields.NvDataOffset = lastFlexMetaInfo.fields.NvDataOffset - recordSize;
            /* increment the last meta info address and reused it as address of the current meta info tag */
            lastFlexMetaInfoAddress += sizeof(NVM_FlexMetaInfo_t);
        }

        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

        /* write record */
        if(FTFx_OK != EEEWrite(&gFlashConfig, destRecordEndAddress - recordSize, recordSize, ((uint8_t*)(pNVM_DataTable[tableEntryIdx].pData))))
        {
            return gNVM_RecordWriteError_c;
        }

        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

        /* write meta */
        if(FTFx_OK != EEEWrite(&gFlashConfig, lastFlexMetaInfoAddress, sizeof(NVM_FlexMetaInfo_t), (uint8_t *)(&flexMetaInfo.rawValue)))
        {
            return gNVM_RecordWriteError_c;
        }
    }
    else /* table entry ID already in FlexRAM, update the corresponding record */
    {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

        if(FTFx_OK != EEEWrite(&gFlashConfig, (uint32_t)(gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset), recordSize,
                               ((uint8_t*)(pNVM_DataTable[tableEntryIdx].pData))))
        {
            return gNVM_RecordWriteError_c;
        }
    }
    /* Empty macro when nvm monitoring is not enabled */
    #if (gFsciIncluded_c && (gNvmEnableFSCIRequests_c || gNvmEnableFSCIMonitoring_c))
    FSCI_MsgNVWriteMonitoring(flexMetaInfo.fields.NvDataEntryID,tblIndexes->elementIndex,tblIndexes->saveRestoreAll);
    #endif
    return gNVM_OK_c;

#else /* No FlexNVM */

    #if gUnmirroredFeatureSet_d
    /* For data sets not mirrored in ram a table entry is saved separate */
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
    {
        tblIndexes->saveRestoreAll = FALSE;
    }
    #endif

    if(tblIndexes->saveRestoreAll)
    {
        realRecordSize = recordSize = pNVM_DataTable[tableEntryIdx].ElementSize * pNVM_DataTable[tableEntryIdx].ElementsCount;
    }
    else
    {
        realRecordSize = recordSize = pNVM_DataTable[tableEntryIdx].ElementSize;
    }

    #if gUnmirroredFeatureSet_d
    /* Check if is an erase for unmirrored dataset*/
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
    {
        if(NULL == ((void**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex])
        {
            realRecordSize = recordSize = 0;
        }
        /*if the dataset is allready in flash, ignore it*/
        else if(NvIsNVMFlashAddress(((void**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex]))
        {
            /*it returns OK, because atomic save must not fail, this is not an error*/
            return gNVM_OK_c;
        }
    }
    #endif

    /* get active page free space */
    NvGetPageFreeSpace(&pageFreeSpace);

    /* compute the 'real record size' taking into consideration that the FTFL controller only writes in burst of 4 bytes */
    paddingBytes = recordSize % (uint8_t)PGM_SIZE_BYTE;
    if(paddingBytes)
    {
        realRecordSize += (uint8_t)((uint8_t)PGM_SIZE_BYTE-paddingBytes);
    }

    /* compute the total size (record + meta info) */
    totalRecordSize = realRecordSize + sizeof(NVM_RecordMetaInfo_t);

    /* check if the record fits the page's free space.
    * one extra meta info space must be kept always free, to be able to perform the meta info search */
    if(totalRecordSize + sizeof(NVM_RecordMetaInfo_t) > pageFreeSpace)
    {
        /* there is no space to save the record, try to copy the current active page latest records
        * to the other page
        */
        mNvCopyOperationIsPending = TRUE;
        return gNVM_PageCopyPending_c;
    }
    else
    {
        metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;

        if(gEmptyPageMetaAddress_c == metaInfoAddress)
        {
            /* empty page, first write */

            /* set new record address */
            newRecordAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress - sizeof(NVM_TableInfo_t) - realRecordSize + 1;

            /* set associated meta info */
            if(tblIndexes->saveRestoreAll)
            {
                metaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
                metaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;
            }
            else
            {
                metaInfo.fields.NvValidationStartByte = gValidationByteSingleRecord_c;
                metaInfo.fields.NvValidationEndByte = gValidationByteSingleRecord_c;
            }

            metaInfo.fields.NvmDataEntryID = pNVM_DataTable[tableEntryIdx].DataEntryID;
            metaInfo.fields.NvmElementIndex = tblIndexes->elementIndex;
            metaInfo.fields.NvmRecordOffset = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress -
                mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress - realRecordSize - sizeof(NVM_TableInfo_t) + 1;

            /* gEmptyPageMetaAddress_c is not a valid address and it is used only as an empty page marker;
            * therefore, set the valid value of meta information address */
            metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c;
            #if gUnmirroredFeatureSet_d
            if(realRecordSize)
            {
                mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = metaInfoAddress;
            }
            #endif
        }
        else
        {
            /* get the meta information of the last successfully written record */
            #if gUnmirroredFeatureSet_d
                NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress, &metaInfo);
            #else
                /* get the last record start address (the address is always aligned) */
                NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);
            #endif
            lastRecordAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset;
            /* set new record address */
            newRecordAddress = lastRecordAddress - realRecordSize;

            /* set associated meta info */
            if(tblIndexes->saveRestoreAll)
            {
                metaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
                metaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;
            }
            else
            {
                metaInfo.fields.NvValidationStartByte = gValidationByteSingleRecord_c;
                metaInfo.fields.NvValidationEndByte = gValidationByteSingleRecord_c;
            }
            metaInfo.fields.NvmDataEntryID = pNVM_DataTable[tableEntryIdx].DataEntryID;
            metaInfo.fields.NvmElementIndex = tblIndexes->elementIndex;
            metaInfo.fields.NvmRecordOffset = newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
            metaInfoAddress += sizeof(NVM_RecordMetaInfo_t);
            #if gUnmirroredFeatureSet_d
            if(realRecordSize)
            {
                mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = metaInfoAddress;
            }
            #endif
        }

        /* check if the space needed by the record is really free (erased).
        * this check is necessary because it may happens that a record to be successfully written,
        * but the system fails (e.g. POR) before the associated meta information has been written.
        * the theoretically free space is computed as the difference between the last meta info
        * address and the start address of the last successfully written record. This information
        * is valuable but may not reflect the reality, as mentioned in the explanation above */

        doWrite = FALSE;

        while(totalRecordSize + sizeof(NVM_RecordMetaInfo_t) < pageFreeSpace)
        {
            if(!NvIsMemoryAreaAvailable(newRecordAddress, realRecordSize))
            {
                /* the memory space is not blank */
                pageFreeSpace -= realRecordSize;
                newRecordAddress -= realRecordSize;
            }
            else
            {
                /* the memory space is blank */
                doWrite = TRUE;
                break;
            }
        }

        /* update the meta info offset, if changed */
        if(newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress != metaInfo.fields.NvmRecordOffset)
        {
            metaInfo.fields.NvmRecordOffset = newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
        }

        /* Write the record and associated meta information */
        if(FALSE == doWrite)
        {
            /* there is no space to save the record, try to copy the current active page latest records
            * to the other page
            */
            mNvCopyOperationIsPending = TRUE;
            return gNVM_PageCopyPending_c;
        }
        #if gUnmirroredFeatureSet_d
        if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
        {
            srcAddress = (uint32_t)(uint8_t*)((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex];
        }
        else
        #endif
        if(tblIndexes->saveRestoreAll)
        {
            srcAddress = (uint32_t)((uint8_t*)(((uint8_t*)(pNVM_DataTable[tableEntryIdx]).pData)));
        }
        else
        {
            srcAddress = (uint32_t)((uint8_t*)(((uint8_t*)(pNVM_DataTable[tableEntryIdx]).pData)) + (tblIndexes->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize));
        }

        #if gUnmirroredFeatureSet_d
        if(0 == srcAddress)
        {
            /* It's an erased unmirrored dataset */
            metaInfo.fields.NvmRecordOffset = 0;
        }
        if(FTFx_OK == (srcAddress ? NV_FlashProgramUnaligned(&gFlashConfig, newRecordAddress, recordSize, (uint8_t*)srcAddress, gFlashLaunchCommand):FTFx_OK))
        #else
        if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, newRecordAddress, recordSize, (uint8_t*)srcAddress, gFlashLaunchCommand))
        #endif
        {
            /* record successfully written, now write the associated record meta information */
            if(FTFx_OK == FlashProgram(&gFlashConfig, metaInfoAddress, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&metaInfo), gFlashLaunchCommand))
            {
                /* update the last record meta information */
                mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = metaInfoAddress;
                /* Empty macro when nvm monitoring is not enabled */
                #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
                FSCI_MsgNVWriteMonitoring(metaInfo.fields.NvmDataEntryID,tblIndexes->elementIndex,tblIndexes->saveRestoreAll);
                #endif

                #if gUnmirroredFeatureSet_d
                if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
                {
                    if(0 != metaInfo.fields.NvmRecordOffset)
                    {
                        MSG_Free((uint8_t*)((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex]);
                        ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex] = (uint8_t*)newRecordAddress;
                    }
                }
                #endif
                return gNVM_OK_c;
            }
            else
            {
                return gNVM_MetaInfoWriteError_c;
            }
            }
            else
            {
                return gNVM_RecordWriteError_c;
            }
    }
#endif /* gNvUseFlexNVM_d */
}


/******************************************************************************
 * Name: NvRestoreData
 * Description: restore an element from NVM storage to its original RAM location
 * Parameter(s): [IN] tblIdx - pointer to table and element indexes
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if page is empty
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvRestoreData
(
    NVM_TableEntryInfo_t* tblIdx
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    NVM_RecordMetaInfo_t metaInfo;
    uint32_t metaInfoAddress;
    NVM_Status_t status;
    uint32_t buff_markers;
    uint16_t index;
    uint16_t index_final;
    uint16_t cnt;
    uint8_t pos_buff;
    uint8_t single_changes_left;
    uint8_t all_read;
#else
    NVM_FlexMetaInfo_t flexMetaInfo;
    uint32_t EERamAddress;
#endif

    uint16_t tableEntryIdx;

    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx->entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */

    /* restore data from EERAM */
    EERamAddress = gFlashConfig.EERAMBase;

    do
    {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
        /* read meta info tag */
        NV_FlashRead(EERamAddress, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));

        if(flexMetaInfo.rawValue == gNvFlexGuardValue_c) /* end of meta info space */
        {
            break;
        }

        if(tblIdx->entryId == flexMetaInfo.fields.NvDataEntryID)
        {
            if(tblIdx->saveRestoreAll)
            {
                /* wait for EEPROM system to be ready */
                while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
                /* read all elements */
                NV_FlashRead((gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset),
                             (uint8_t*)(pNVM_DataTable[tableEntryIdx].pData),
                             pNVM_DataTable[tableEntryIdx].ElementSize * pNVM_DataTable[tableEntryIdx].ElementsCount);
                return gNVM_OK_c;
            }
            else
            {
                /* wait for EEPROM system to be ready */
                while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
                /* read element */
                NV_FlashRead(gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset + (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize),
                             (uint8_t*)(((uint8_t*)pNVM_DataTable[tableEntryIdx].pData) + (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
                             pNVM_DataTable[tableEntryIdx].ElementSize);
                return gNVM_OK_c;
            }
        }

        /* go to next meta tag */
        EERamAddress += sizeof(flexMetaInfo);

    } while(EERamAddress < (gFlashConfig.EERAMBase + gFlashConfig.EEESize));

    return gNVM_MetaNotFound_c;

#else /* FlexNVM */
    /* get the last meta information address */
    if((metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress) == gEmptyPageMetaAddress_c)
    {
        /* blank page, no data to restore */
        return gNVM_PageIsEmpty_c;
    }

    if(tblIdx->entryId == gNvInvalidDataEntry_c)
    {
        /* invalid table entry */
        return gNVM_InvalidTableEntry_c;
    }

    /*
    * If the meta info is found, the associated record is restored,
    * otherwise the gNVM_MetaNotFound_c will be returned
    */
    status = gNVM_MetaNotFound_c;


    /*** restore all ***/
    if(tblIdx->saveRestoreAll)
    {
        index = 0;
        index_final = pNVM_DataTable[tableEntryIdx].ElementsCount;
        single_changes_left = TRUE;
        all_read = FALSE;

        while(index < index_final && single_changes_left)
        {
            buff_markers = 0;
            single_changes_left = FALSE;
            /* parse meta info backwards until the element is found */
            while(~buff_markers &&
                  (metaInfoAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c)))
            {
                /* get the meta information */
                NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);

                if(metaInfo.fields.NvValidationStartByte == metaInfo.fields.NvValidationEndByte &&
                   metaInfo.fields.NvmDataEntryID == tblIdx->entryId)
                {
                    pos_buff = metaInfo.fields.NvmElementIndex & 0x1F;

                    if(metaInfo.fields.NvValidationStartByte == gValidationByteSingleRecord_c &&
                       (metaInfo.fields.NvmElementIndex >= index) &&
                           (metaInfo.fields.NvmElementIndex < index_final))
                    {
                        if(metaInfo.fields.NvmElementIndex < index+32 &&
                           !(buff_markers & (1<<pos_buff)))
                        {
                            buff_markers |= (1<<pos_buff);

                            NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset,
                                         ((uint8_t*)pNVM_DataTable[tableEntryIdx].pData)+(index+pos_buff)*pNVM_DataTable[tableEntryIdx].ElementSize,
                                         pNVM_DataTable[tableEntryIdx].ElementSize);
                            status = gNVM_OK_c;
                        }
                        else if (metaInfo.fields.NvmElementIndex >= index+32)
                        {
                            /*bigger index change exists*/
                            single_changes_left = TRUE;
                        }
                    }
                    else if(metaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
                    {
                        if (!all_read)
                        {
                            for (cnt=0; cnt<32; cnt++)
                            {
                                if (buff_markers & (1<<cnt))
                                    continue;
                                if (index+cnt >= index_final)
                                    break;
                                buff_markers |= (1<<cnt);

                                NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + (index + cnt) * pNVM_DataTable[tableEntryIdx].ElementSize,
                                             ((uint8_t*)pNVM_DataTable[tableEntryIdx].pData)+(index+cnt)*pNVM_DataTable[tableEntryIdx].ElementSize,
                                             pNVM_DataTable[tableEntryIdx].ElementSize);
                                status = gNVM_OK_c;
                            }
                            /*read to end*/
                            if (index_final - (index + 32) > 0)
                            {
                                NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + (index + 32) * pNVM_DataTable[tableEntryIdx].ElementSize,
                                             ((uint8_t*)pNVM_DataTable[tableEntryIdx].pData)+(index+32)*pNVM_DataTable[tableEntryIdx].ElementSize,
                                             (index_final - (index + 32)) * pNVM_DataTable[tableEntryIdx].ElementSize);
                                all_read = TRUE;
                                status = gNVM_OK_c;
                            }
                        }
                        break;
                    }
                }
                /* move to the previous meta info */
                metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
            }
            /*if all flags are 1, I never reached the end, so I dont know reliably if all the entries have been tested*/
            if (!(~buff_markers))
                single_changes_left = TRUE;

            index += 32;
            metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
        }
        return status;
    }

    /*** restore single ***/

    /* parse meta info backwards until the element is found */
    while(metaInfoAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + gNvFirstMetaOffset_c))
    {
        /* get the meta information */
        NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);

        if(metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte)
        {
            /* invalid meta info, move to the previous meta info */
            metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
            continue;
        }

        if(metaInfo.fields.NvmDataEntryID == tblIdx->entryId)
        {
            if(metaInfo.fields.NvValidationStartByte == gValidationByteSingleRecord_c && metaInfo.fields.NvmElementIndex == tblIdx->elementIndex)
            {
                #if gUnmirroredFeatureSet_d
                if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
                {
                    if(!metaInfo.fields.NvmRecordOffset)
                    {
                        ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIdx->elementIndex]=NULL;
                    }
                    else
                    {
                        ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIdx->elementIndex] =
                            (uint8_t*)mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset;
                    }
                    status = gNVM_OK_c;
                    break;
                }
                else
                #endif
                {
                    /* restore the element */
                    NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset,
                                 (uint8_t*)((uint8_t*)pNVM_DataTable[tableEntryIdx].pData +
                                            (metaInfo.fields.NvmElementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
                                 pNVM_DataTable[tableEntryIdx].ElementSize);
                    status = gNVM_OK_c;
                    break;
                }
            }

            if(metaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
            {
                /* restore the single element from the entire table entry record */
                NV_FlashRead((mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset +
                              (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
                ((uint8_t*)pNVM_DataTable[tableEntryIdx].pData + (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
                pNVM_DataTable[tableEntryIdx].ElementSize);
                status = gNVM_OK_c;
                break;
            }
        }

        /* move to the previous meta info */
        metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
    }
    return status;

#endif /* gNvUseFlexNVM_d */
}


/******************************************************************************
 * Name: NvGetTableEntryIndex
 * Description: get the table entry index from the provided ID
 * Parameter(s): [IN] entryId - the ID of the table entry
 * Return: table entry index of gNvInvalidTableEntryIndex_c
 *****************************************************************************/
static uint16_t NvGetTableEntryIndexFromId
(
    NvTableEntryId_t entryId
)
{
    uint16_t loopCnt = 0;

    while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
    {
        if(pNVM_DataTable[loopCnt].DataEntryID == entryId)
        {
            return loopCnt;
        }
        /* increment the loop counter */
        loopCnt++;
    }
    return gNvInvalidTableEntryIndex_c;
}


/******************************************************************************
 * Name: NvAddSaveRequestToQueue
 * Description: Add save request to save requests queue; if the request is
 *              already stored, ignore the current request
 * Parameter(s): [IN] ptrTblIdx - pointer to table index
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_SaveRequestRejected_c - if the request couldn't be queued
 ******************************************************************************/
static NVM_Status_t NvAddSaveRequestToQueue
(
    NVM_TableEntryInfo_t* ptrTblIdx
)
{
    uint8_t loopIdx;
    bool_t  isQueued = FALSE;
    bool_t  isInvalidEntry = FALSE;
    uint8_t lastInvalidIdx;

    if(mNvPendingSavesQueue.EntriesCount == 0)
    {
        /* add request to queue */
        if(NvPushPendingSave(&mNvPendingSavesQueue, *ptrTblIdx))
        {
            return gNVM_OK_c;
        }
        return gNVM_SaveRequestRejected_c;
    }

    /* start from the queue's head */
    loopIdx = mNvPendingSavesQueue.Head;

    /* check if the request is not already stored in queue */
    while(loopIdx != mNvPendingSavesQueue.Tail)
    {
        if(ptrTblIdx->entryId == mNvPendingSavesQueue.QData[loopIdx].entryId)
        {
            if(mNvPendingSavesQueue.QData[loopIdx].saveRestoreAll == TRUE) /* full table entry already queued */
            {
                /* request is already queued */
                isQueued = TRUE;
                break;
            }

            /* single element from table entry is queued */
            if(ptrTblIdx->saveRestoreAll == TRUE) /* a full table entry is requested to be saved */
            {
                /* update only the flag of the already queued request */
                mNvPendingSavesQueue.QData[loopIdx].saveRestoreAll = TRUE;
                /* request is already queued */
                isQueued = TRUE;
                break;
            }

            /* The request is for a single element and the queued request is also for a single element;
            * Check if the request is for the same element. If the request is for a different element,
            * add the new request to queue.
            */
            if(ptrTblIdx->elementIndex == mNvPendingSavesQueue.QData[loopIdx].elementIndex)
            {
                /* request is already queued */
                isQueued = TRUE;
                break;
            }

        }
        /* Check if in the queue is an invalid entryId that can be used*/
        if((gNvInvalidDataEntry_c == mNvPendingSavesQueue.QData[loopIdx].entryId)&&
           (isInvalidEntry == FALSE))
        {
            isInvalidEntry = TRUE;
            lastInvalidIdx = loopIdx;
        }
        /* increment and wrap the loop index */
        loopIdx = (loopIdx + 1)  & ((uint8_t)(gNvPendingSavesQueueSize_c - 1));
    }

    if(!isQueued)
    {
        /* Reuse an invalid entry from the queue*/
        if(TRUE == isInvalidEntry)
        {
            mNvPendingSavesQueue.QData[lastInvalidIdx] = *ptrTblIdx;
            return gNVM_OK_c;
        }
        /* push the request to save operation pending queue */
        if(NvPushPendingSave(&mNvPendingSavesQueue, *ptrTblIdx))
        {
            return gNVM_OK_c;
        }
        return gNVM_SaveRequestRejected_c;
    }

    return gNVM_OK_c;
}

/******************************************************************************
 * Name: NvIntervalTimerCallback
 * Description: Callback function of the timer used by the NvSaveOnInterval()
 * Parameter(s): [IN] timerID - timer ID
 * Return: -
 ******************************************************************************/
static void NvIntervalTimerCallback
(
    void* pV
)
{
    mNvSaveOnIntervalEvent = TRUE;
}

/******************************************************************************
 * Name: GetRandomRange
 * Description: Returns a random number between 'low' and 'high'
 * Parameter(s): [IN] low, high - generated number range
 * Return: 0..255
 ******************************************************************************/
static uint8_t GetRandomRange
(
    uint8_t low,
    uint8_t high
)
{
    uint32_t random;
    RNG_GetRandomNo(&random);

    if(high <= low)
    {
        return low;
    }
    return low + (uint8_t)(random % (high - low + 1));
};

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */
/******************************************************************************
 * Name: NvGetFlexLastMetaInfo
 * Description: Get FlexRAM last meta information address
 * Parameter(s): -
 * Return: the address of the last valid meta information
 ******************************************************************************/
static uint32_t NvGetFlexLastMetaInfo
(
    void
)
{
    uint32_t address, size;
    NVM_FlexMetaInfo_t flexMetaInfo;

    address = gFlashConfig.EERAMBase;
    size = gFlashConfig.EEESize;

    while(size)
    {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
        /* read meta info tag */
        NV_FlashRead(address, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));
        if(flexMetaInfo.rawValue == gNvFlexGuardValue_c)
        {
            break;
        }
        address += sizeof(flexMetaInfo);
        size -= sizeof(flexMetaInfo);
    }
    return address - sizeof(flexMetaInfo);
}

/******************************************************************************
 * Name: NvGetFlexMetaInfoFromId
 * Description: Get FlexRAM meta information tag from table entry ID
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 meta information tag will be stored
 * Return: -
 ******************************************************************************/
static void NvGetFlexMetaInfoFromId
(
    NvTableEntryId_t tblEntryId,
    NVM_FlexMetaInfo_t* pMetaInfo
)
{
    uint32_t address, size;
    NVM_FlexMetaInfo_t flexMetaInfo;

    address = gFlashConfig.EERAMBase;
    size = gFlashConfig.EEESize;

    while(size)
    {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

        NV_FlashRead(address, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));
        if(flexMetaInfo.rawValue == gNvFlexGuardValue_c)
        {
            break;
        }

        if(flexMetaInfo.fields.NvDataEntryID == tblEntryId)
        {
            pMetaInfo->fields.NvDataEntryID = flexMetaInfo.fields.NvDataEntryID;
            pMetaInfo->fields.NvDataOffset = flexMetaInfo.fields.NvDataOffset;
            return;
        }

        address += sizeof(flexMetaInfo);
        size -= sizeof(flexMetaInfo);
    }

    pMetaInfo->rawValue = gNvFlexGuardValue_c;
}

/******************************************************************************
 * Name: NvCheckNvmTableForFlexRAMUsage
 * Description: Check if the existing NVM table fits within the FlexRAM window
 * Parameter(s): -
 * Return: gNVM_NvTableExceedFlexRAMSize_c - the table exceed the size of
 *                                           FlexRAM window
 *         gNVM_OK_c - the table fits within the size of window FlexRAM window
 ******************************************************************************/
static NVM_Status_t NvCheckNvmTableForFlexRAMUsage
(
    void
)
{
    index_t loopCnt = 0;
    uint32_t allDatasetSize = 0;

    while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
    {
        if(gNvInvalidDataEntry_c == pNVM_DataTable[loopCnt].DataEntryID)
        {
            loopCnt++;
            continue;
        }
        /* add the record size */
        allDatasetSize += (pNVM_DataTable[loopCnt].ElementsCount * pNVM_DataTable[loopCnt].ElementSize);
        /* add the meta size */
        allDatasetSize += sizeof(NVM_FlexMetaInfo_t);
        /* increment the loop counter */
        loopCnt++;
    }

    /* add the safe guard space (equal to meta size) */
    allDatasetSize += sizeof(NVM_FlexMetaInfo_t);

    if(allDatasetSize > gFlashConfig.EEESize)
    {
        return gNVM_NvTableExceedFlexRAMSize_c;
    }

    return gNVM_OK_c;
}

#endif /* gNvUseFlexNVM_d */

#endif /* gNvStorageIncluded_d */



/*****************************************************************************
 *****************************************************************************
 * Public functions
 *****************************************************************************
 *****************************************************************************/


/******************************************************************************
 * Name: NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found
 *         gNVM_OK_c - module was successfully initialized
 *         gNVM_CannotCreateMutex_c - no mutex available
 *****************************************************************************/
NVM_Status_t NvModuleInit
(
    void
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    status = __NvModuleInit();
    if(status != gNVM_OK_c)
    {
        return status;
    }

    if( kStatus_OSA_Success != OSA_MutexCreate(&mNVMMutex) )
    {
        mNvModuleInitialized = FALSE;
        return gNVM_CannotCreateMutex_c;
    }

    #if (gFsciIncluded_c && gNvmEnableFSCIRequests_c)
    FSCI_RegisterOpGroup(gNV_FsciReqOG_d,
                         gFsciMonitorMode_c,
                         NV_FsciMsgHandler,
                         NULL,
                         gNvmDefaultFsciInterface_c);
    #endif
    return gNVM_OK_c;
#else
    return gNVM_Error_c;
#endif /* #if gNvStorageIncluded_d */
}

/******************************************************************************
 * Name: NvMoveToRam
 * Description: Move from NVM to Ram an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_NoMemory_c - in case there is not a memory block free
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 *****************************************************************************/

NVM_Status_t NvMoveToRam
(
    void** ppData
)
{
#if gNvStorageIncluded_d && gUnmirroredFeatureSet_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvmMoveToRam(ppData);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ppData;
    return gNVM_Error_c;
#endif
}

/******************************************************************************
 * Name: NvErase
 * Description: Erase from NVM an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_NoMemory_c - in case there is not a memory block free
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 *****************************************************************************/

NVM_Status_t NvErase
(
    void** ppData
)
{
#if gNvStorageIncluded_d && gUnmirroredFeatureSet_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvmErase(ppData);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ppData;
    return gNVM_Error_c;
#endif
}


/******************************************************************************
 * Name: NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
NVM_Status_t NvSaveOnIdle
(
    void* ptrData,
    bool_t saveAll
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvSaveOnIdle(ptrData,saveAll);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ptrData;
    (void)saveAll;
    return gNVM_Error_c;
#endif /* # gNvStorageIncluded_d */
}

/******************************************************************************
 * Name: NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
NVM_Status_t NvSaveOnInterval
(
    void* ptrData
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvSaveOnInterval(ptrData);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ptrData;
    return gNVM_Error_c;
#endif
}                                       /* NvSaveOnInterval() */


/******************************************************************************
 * Name: NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 ******************************************************************************/
NVM_Status_t NvSaveOnCount
(
    void* ptrData
)
{
#if gNvStorageIncluded_d

    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvSaveOnCount(ptrData);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;

#else
    (void)ptrData;
    return gNVM_Error_c;
#endif
}                                       /* NvSaveOnCount() */


/******************************************************************************
 * Name: NvSetMinimumTicksBetweenSaves
 * Description: Set the timer used by NvSaveOnInterval(). Takes effect after
 *              the next save.
 * Parameters: [IN] newInterval - new time interval
 * Return: -
 ******************************************************************************/
void NvSetMinimumTicksBetweenSaves
(
    NvSaveInterval_t newInterval
)
{
#if gNvStorageIncluded_d
    gNvMinimumTicksBetweenSaves = newInterval;
#else
    (void)newInterval;
#endif
}                                       /* NvSetMinimumTicksBetweenSaves() */


/******************************************************************************
 * Name: NvSetCountsBetweenSaves
 * Description: Set the counter trigger value used by NvSaveOnCount().
 *              Takes effect after the next save.
 * Parameters: [IN] newCounter - new counter value
 * Return: -
 ******************************************************************************/
void NvSetCountsBetweenSaves
(
    NvSaveCounter_t newCounter
)
{
#if gNvStorageIncluded_d
    gNvCountsBetweenSaves = newCounter;
#else
    (void)newCounter;
#endif
}                                       /* NvSetCountsBetweenSaves() */


/******************************************************************************
 * Name: NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
bool_t NvTimerTick
(
    bool_t countTick
)
{
#if gNvStorageIncluded_d
    bool_t fTicksLeft;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    fTicksLeft = __NvTimerTick(countTick);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return fTicksLeft;
#else
    (void)countTick;
    return FALSE;
#endif /* #if gNvStorageIncluded_d */
}                                       /* NvTimerTick() */


/******************************************************************************
 * Name: NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
NVM_Status_t NvRestoreDataSet
(
    void* ptrData,
    bool_t restoreAll
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvRestoreDataSet(ptrData,restoreAll);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ptrData;
    (void)restoreAll;
    return gNVM_Error_c;
#endif
}

/******************************************************************************
 * Name: NvClearCriticalSection
 * Description: leave critical section
 * Parameters: -
 * Return: -
 ******************************************************************************/
void NvClearCriticalSection
(
    void
)
{
#if (gNvStorageIncluded_d && gNvEnableCriticalSection_c)
    if(mNvCriticalSectionFlag)  /* in case of set/clear mismatch */
        --mNvCriticalSectionFlag;
#endif
}


/******************************************************************************
 * Name: NvSetCriticalSection
 * Description: enter critical section
 * Parameters: -
 * Return: -
 ******************************************************************************/
void NvSetCriticalSection
(
    void
)
{
#if (gNvStorageIncluded_d && gNvEnableCriticalSection_c)
    ++mNvCriticalSectionFlag;
#endif
}


/******************************************************************************
 * Name: NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
void NvIdle
(
    void
)
{
#if gNvStorageIncluded_d
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    __NvIdle();
    (void)OSA_MutexUnlock(&mNVMMutex);

#endif
}/* NvIdle() */


/******************************************************************************
 * Name: NvIsDataSetDirty
 * Description: return TRUE if the element pointed by ptrData is dirty
 * Parameters: [IN] ptrData - pointer to data to be checked
 * Return: TRUE if the element is dirty, FALSE otherwise
 ******************************************************************************/
bool_t NvIsDataSetDirty
(
    void* ptrData
)
{
#if gNvStorageIncluded_d
    bool_t res;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    res = __NvIsDataSetDirty(ptrData);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return res;
#else
    (void)ptrData;
    return FALSE;
#endif
}

/******************************************************************************
 * Name: NvGetStatistics
 * Description:
 * Parameter(s): [OUT] ptrStat - pointer to a memory location where the pages
 *                               statistics (erase cycles of each page) will
 *                               be stored
 * Return: -
 *****************************************************************************/
void NvGetPagesStatistics
(
    NVM_Statistics_t* ptrStat
)
{
#if gNvStorageIncluded_d
    #if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
    if(!mNvModuleInitialized)
    {
        return;
    }

    if(NULL == ptrStat)
    {
        return;
    }

    if(mNvPageCounter%2)
    {
        ptrStat->FirstPageEraseCyclesCount = ptrStat->SecondPageEraseCyclesCount = (mNvPageCounter-1)/2;
    }
    else
    {
        ptrStat->FirstPageEraseCyclesCount = mNvPageCounter/2;
        ptrStat->SecondPageEraseCyclesCount = (mNvPageCounter-2)/2;
    }

    #else /* FlexNVM */
    ptrStat->FirstPageEraseCyclesCount = 0;
    ptrStat->SecondPageEraseCyclesCount = 0;
    return;
    #endif
#else
    (void)ptrStat;
    return;
#endif
}

/******************************************************************************
 * Name: NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter/ram table to active page.
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a
 *                                        critical section
 *****************************************************************************/
NVM_Status_t NvFormat
(
    void
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvFormat();
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    return gNVM_Error_c;
#endif /* gNvStorageIncluded_d */
}


/******************************************************************************
 * Name: NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element
 *               [IN] dataEntryType - the type of the new entry
 *               [IN] overwrite - if an existing table entry shall be
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
NVM_Status_t NvRegisterTableEntry
(
    void* ptrData,
    NvTableEntryId_t uniqueId,
    uint16_t elemCount,
    uint16_t elemSize,
    uint16_t dataEntryType,
    bool_t overwrite
)
{
#if gNvStorageIncluded_d && gNvUseExtendedFeatureSet_d

    NVM_Status_t status;
    OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvRegisterTableEntry(ptrData,uniqueId,elemCount,elemSize, dataEntryType, overwrite);
    OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ptrData;
    (void)uniqueId;
    (void)elemCount;
    (void)elemSize;
    (void)overwrite;
    return gNVM_Error_c;
#endif
}


/******************************************************************************
 * Name: NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV
 *              table.
 * Parameter(s): [IN] ptrData - a pointer to an existing RAM data that is
 *                              managed by the NV storage system
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
NVM_Status_t NvEraseEntryFromStorage
(
    void* ptrData
)
{
#if gNvStorageIncluded_d && gNvUseExtendedFeatureSet_d
    NVM_Status_t status;
    NVM_TableEntryInfo_t tblIdx;
    uint16_t tableEntryIdx;
    OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);

    if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
    {
        return status;
    }
    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);
    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
        return gNVM_InvalidTableEntry_c;
    }

    /* invalidate the table entry */
    pNVM_DataTable[tableEntryIdx].pData = NULL;
    pNVM_DataTable[tableEntryIdx].ElementsCount = 0;
    pNVM_DataTable[tableEntryIdx].ElementSize = 0;
    status = __NvEraseEntryFromStorage(tblIdx.entryId, tableEntryIdx);
    OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    (void)ptrData;
    return gNVM_Error_c;
#endif
}

/******************************************************************************
 * Name: NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
NVM_Status_t NvSyncSave
(
    void* ptrData,
    bool_t saveAll,
    bool_t ignoreCriticalSectionFlag
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvSyncSave(ptrData,saveAll,ignoreCriticalSectionFlag);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;

#else
    (void)ptrData;
    (void)saveAll;
    return gNVM_Error_c;
#endif
}


/******************************************************************************
 * Name: NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
NVM_Status_t NvAtomicSave
(
    bool_t ignoreCriticalSectionFlag
)
{
#if gNvStorageIncluded_d
    NVM_Status_t status ;
    (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
    status = __NvAtomicSave(ignoreCriticalSectionFlag);
    (void)OSA_MutexUnlock(&mNVMMutex);
    return status;
#else
    return gNVM_Error_c;
#endif
}
