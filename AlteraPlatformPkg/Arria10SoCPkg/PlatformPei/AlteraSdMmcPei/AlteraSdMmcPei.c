/** @file

  Copyright (c) 2015, Altera Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.

**/

//
// Include files
//
#include <Uefi.h>
#include <IndustryStandard/Mbr.h>
#include <Library/AlteraSdMmcLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "AlteraSdMmcPei.h"
#include "Diagnostics.h"
#include "Mmc.h"
#include "SdMmcHostProtocol.h"

#if (FixedPcdGet32(PcdDebugMsg_SdMmc) == 0)
//  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDumpEx(BaseAddr, Data32Size, PrintLineStartAddress)   /* do nothing */
#else
  #define ProgressPrint  SerialPortPrint
  #define InfoPrint      SerialPortPrint
  #define MmioHexDumpEx  SerialPortMmioHexDumpEx
#endif

//
// Extract UINT32 from char array
//
#define UNPACK_UINT32(a) (UINT32)( (((UINT8 *) a)[0] <<  0) |    \
                                   (((UINT8 *) a)[1] <<  8) |    \
                                   (((UINT8 *) a)[2] << 16) |    \
                                   (((UINT8 *) a)[3] << 24) )

PEI_SDMMC_INSTANCE*  MmcHostInstance;

// MBR variables
UINT32   mAlteraBootImagePartitionLba = 0;
UINT32   mFat32PartitionLba = 0;
UINT32   mLinuxPartitionLba = 0;

// FAT32 variables
BOOLEAN  mIsValidFAT32 = FALSE;
UINT32   mBytePerSector = 0;
UINT32   mSectorPerCluster = 0;
UINT32   mFirstFatSector = 0;
UINT32   mFirstDataSector = 0;
UINT32   mClusterSize = 0;
UINT32   mRootDirectoryCluster = 0;

// Maximum FAT32 Partition Volumn size:
// according to support.microsoft.com/en-us/kb/140365
//  4096 bytes cluster = Maximum FAT32 Partition Size of  8 GiB
//  8192 bytes cluster = Maximum FAT32 Partition Size of 16 GiB
// 16384 bytes cluster = Maximum FAT32 Partition Size of 32 GiB
// Note that the FAT32 file system does not support drives smaller than 512 MB. (MS kb/192322)
#define  MAX_SUPPORTED_CLUSTER_SIZE    FixedPcdGet32(PcdMaxFAT32ClusterSize)
//#define  MAX_SUPPORTED_CLUSTER_SIZE    4096
//#define  MAX_SUPPORTED_CLUSTER_SIZE    8192
//#define  MAX_SUPPORTED_CLUSTER_SIZE    16384
// 32KB is possible when FAT32X MBR Type 0x0C, to use 32KB turn off debug message to free up OCRAM space
//#define  MAX_SUPPORTED_CLUSTER_SIZE    32768
// The MAX_SUPPORTED_CLUSTER_SIZE will affect the driver file size
// because it will compile into BSS Data Segment of the driver
// if this array is delcare inside a function it will used stack
// but our on-chip memory stack space is not big enough to support 16KB cluster
// Most FAT32 partition will uses 4 kiB cluster,
// set MAX_SUPPORTED_CLUSTER_SIZE = 4096 to reduce driver file size.
UINT32   mFatEntryBuffer[MAX_SUPPORTED_CLUSTER_SIZE / 4];
UINT8    mDataBuffer[MAX_SUPPORTED_CLUSTER_SIZE];
UINT32   mClusterFATSecNumCachedInFatEntryBuffer = 0;
UINT32   mClusterDataSecNumCachedInDataBuffer = 0;

// Cluster File Operation
UINT32   mFileStartClusterNumber = 0;
UINT32   mNextClusterNumber = 0;
BOOLEAN  mReachedEndOfCluster = TRUE;


// The first important data structure on a FAT volume
// is called the BPB (BIOS Parameter Block)
#pragma pack(1)
typedef struct {
  UINT16  BytsPerSec ;
  UINT8   SecPerClus;
  UINT16  RsvdSecCnt;
  UINT8   NumFATs;
  UINT16  RootEntCnt;
  UINT16  TotSec16;
  UINT8   Media;
  UINT16  FATSz16;
  UINT16  SecPerTrk;
  UINT16  NumHeads;
  UINT32  HiddSec;
  UINT32  TotSec32;
} FAT_COMMON_BIOS_PARAMETER_BLOCK;

typedef struct {
  UINT32  FATSz32;
  UINT16  ExtFlags;
  UINT16  FSVer;
  UINT32  RootClus;
  UINT16  FSInfo;
  UINT16  BkBootSec;
  UINT8   Reserved[12];
} FAT32_BIOS_PARAMETER_BLOCK;

// Boot Sector and BPB
typedef struct {
  UINT8                            BS_jmpBoot[3];
  CHAR8                            BS_OEMName [8];
  FAT_COMMON_BIOS_PARAMETER_BLOCK  Bpb;
  FAT32_BIOS_PARAMETER_BLOCK       BpbEx;
  UINT8                            BS_DrvNum;
  UINT8                            BS_Reserved1;
  UINT8                            BS_BootSig;
  CHAR8                            BS_VolID[4];
  CHAR8                            BS_VolLab[11];
  CHAR8                            BS_FilSysType[8];
} FAT32_BOOT_SECTOR;
#pragma pack()


//
// FAT entry values
//
#define FAT_CLUSTER_SPECIAL           ((0xFFFFFFFF &~0xF) | 0x07)
#define FAT_CLUSTER_FREE              0
#define FAT_CLUSTER_RESERVED          (FAT_CLUSTER_SPECIAL)
#define FAT_CLUSTER_LAST              (0xFFFFFFFF)
#define FAT_END_OF_FAT_CHAIN(Cluster) ((Cluster) > (FAT_CLUSTER_SPECIAL))

//
// FAT attribute define
//
#define FAT_ATTRIBUTE_READ_ONLY 0x01
#define FAT_ATTRIBUTE_HIDDEN    0x02
#define FAT_ATTRIBUTE_SYSTEM    0x04
#define FAT_ATTRIBUTE_VOLUME_ID 0x08
#define FAT_ATTRIBUTE_DIRECTORY 0x10
#define FAT_ATTRIBUTE_ARCHIVE   0x20
#define FAT_ATTRIBUTE_DEVICE    0x40
#define FAT_ATTRIBUTE_LFN       0x0F

//
// Directory Entry
//
#pragma pack(1)
typedef struct {
  UINT16  Day : 5;
  UINT16  Month : 4;
  UINT16  Year : 7;                 // From 1980
} FAT_DATE;

typedef struct {
  UINT16  DoubleSecond : 5;
  UINT16  Minute : 6;
  UINT16  Hour : 5;
} FAT_TIME;

typedef struct {
  FAT_TIME  Time;
  FAT_DATE  Date;
} FAT_DATE_TIME;

typedef struct {
  CHAR8         FileName[11];       // 8.3 filename
  UINT8         Attributes;
  UINT8         CaseFlag;
  UINT8         CreateMillisecond;  // (creation milliseconds - ignored)
  FAT_DATE_TIME FileCreateTime;
  FAT_DATE      FileLastAccess;
  UINT16        FileClusterHigh;    // >= FAT32
  FAT_DATE_TIME FileModificationTime;
  UINT16        FileCluster;
  UINT32        FileSize;
} FAT_DIRECTORY_ENTRY;

typedef struct {
  UINT8   Ordinal;
  CHAR16  Name1[5];
  UINT8   Attributes;
  UINT8   Type;
  UINT8   Checksum;
  CHAR16  Name2[6];
  UINT16  MustBeZero;
  CHAR16  Name3[2];
} FAT_DIRECTORY_LFN;

typedef union {
  FAT_DIRECTORY_ENTRY DirEntry;
  FAT_DIRECTORY_LFN   LfnEntry;
} FAT_DIRECTORY_UNION;
#pragma pack()


//
// Functions
//

//-------------------------------------------------------------------
//
// PARTITION TABLE AND BOOT SECTOR DECODING LAYER
//
//-------------------------------------------------------------------
/**
  Test to see if the Mbr buffer is a valid MBR.

  @param  Mbr         Parent Handle.
  @param  LastLba     Last Lba address on the device.

  @retval TRUE        Mbr is a Valid MBR.
  @retval FALSE       Mbr is not a Valid MBR.

**/
BOOLEAN
PartitionValidMbr (
  IN  MASTER_BOOT_RECORD      *Mbr,
  IN  EFI_LBA                 LastLba
  )
{
  UINT32  StartingLBA;
  UINT32  EndingLBA;
  UINT32  NewEndingLBA;
  INTN    Index1;
  INTN    Index2;
  BOOLEAN MbrValid;

  if (Mbr->Signature != MBR_SIGNATURE) {
    InfoPrint ("MBR signature mismatch\r\n");
    return FALSE;
  }
  //
  // The BPB also has this signature, so it can not be used alone.
  //
  MbrValid = FALSE;
  for (Index1 = 0; Index1 < MAX_MBR_PARTITIONS; Index1++) {
    if (Mbr->Partition[Index1].OSIndicator == 0x00 || UNPACK_UINT32 (Mbr->Partition[Index1].SizeInLBA) == 0) {
      continue;
    }

    MbrValid    = TRUE;
    StartingLBA = UNPACK_UINT32 (Mbr->Partition[Index1].StartingLBA);
    EndingLBA   = StartingLBA + UNPACK_UINT32 (Mbr->Partition[Index1].SizeInLBA) - 1;
    if (EndingLBA > LastLba) {
      //
      // Compatibility Errata:
      //  Some systems try to hide drive space with their INT 13h driver
      //  This does not hide space from the OS driver. This means the MBR
      //  that gets created from DOS is smaller than the MBR created from
      //  a real OS (NT & Win98). This leads to BlockIo->LastBlock being
      //  wrong on some systems FDISKed by the OS.
      //
      // return FALSE since no block devices on a system are implemented
      // with INT 13h
      //
      InfoPrint ("MBR region ending LBA > Last LBA\r\n");
      return FALSE;
    }

    for (Index2 = Index1 + 1; Index2 < MAX_MBR_PARTITIONS; Index2++) {
      if (Mbr->Partition[Index2].OSIndicator == 0x00 || UNPACK_UINT32 (Mbr->Partition[Index2].SizeInLBA) == 0) {
        continue;
      }

      NewEndingLBA = UNPACK_UINT32 (Mbr->Partition[Index2].StartingLBA) + UNPACK_UINT32 (Mbr->Partition[Index2].SizeInLBA) - 1;
      if (NewEndingLBA >= StartingLBA && UNPACK_UINT32 (Mbr->Partition[Index2].StartingLBA) <= EndingLBA) {
        //
        // This region overlaps with the Index1'th region
        //
        InfoPrint ("MBR regions overlapped\r\n");
        return FALSE;
      }
    }
  }
  //
  // None of the regions overlapped so MBR is O.K.
  //
  return MbrValid;
}


EFI_STATUS
EFIAPI
DiscoverPartitionTableType (
  VOID
  )
{
  EFI_STATUS          Status;
  UINTN               Index;
  UINTN               PartitionNumber;
  UINT8               ReadBufferArray[512];
  UINT8               PartBufferArray[512];
  VOID*               ReadBuffer;
  VOID*               PartBuffer;
  MASTER_BOOT_RECORD* Mbr;
  UINT32              PartitionFirstLbaWithData;
  FAT32_BOOT_SECTOR*  Fat32VolBs;
  UINT32              FATSz;
  UINT32              TotSec;
  UINT32              DataSec;
  UINT32              RootDirSectors;
  UINT32              CountofClusters;

  ReadBuffer  = &ReadBufferArray[0];
  PartBuffer  = &PartBufferArray[0];
  Mbr = (MASTER_BOOT_RECORD*) &ReadBufferArray[0];
  Fat32VolBs = (FAT32_BOOT_SECTOR*) &PartBufferArray[0];
  PartitionNumber = 1;
  mAlteraBootImagePartitionLba = 0;
  mFat32PartitionLba = 0;
  mLinuxPartitionLba = 0;

  // Check if a Media is Present
  if ((!MmcHostInstance->BlockIo.Media->MediaPresent) ||
       (MmcHostInstance->BlockIo.Media->LastBlock == 0))
  {
    InfoPrint ("ERROR: No Media Present\r\n");
    return EFI_NO_MEDIA;
  }

  if (MmcHostInstance->State != MmcTransferState) {
    InfoPrint ("ERROR: Not ready for Transfer state\r\n");
    return EFI_NOT_READY;
  }

  // Read the buffer at the same location
  InfoPrint ("Reading LBA 0\r\n");
  Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, 0, 512, ReadBuffer);
  if (Status != EFI_SUCCESS) {
    InfoPrint ("ERROR: Fail to Read Block\r\n");
    return Status;
  }
  if (!PartitionValidMbr (Mbr, MmcHostInstance->BlockIo.Media->LastBlock)) {
    InfoPrint ("Not a valid MBR:\r\n");
    MmioHexDumpEx((UINTN)ReadBuffer, 512/4, 0);
    return EFI_VOLUME_CORRUPTED;
  }
  //
  // We have a valid MBR - dump each partition
  //
  InfoPrint ("Master Boot Record (MBR) Info:\r\n");
  MmioHexDumpEx((UINTN)ReadBuffer, 512/4, 0);

  for (Index = 0; Index < MAX_MBR_PARTITIONS; Index++) {
    if (Mbr->Partition[Index].OSIndicator == 0x00 || UNPACK_UINT32 (Mbr->Partition[Index].SizeInLBA) == 0) {
      //
      // Don't use null MBR entries
      //
      continue;
    }

    if (Mbr->Partition[Index].OSIndicator == PMBR_GPT_PARTITION) {
      //
      // This is the guard MBR for the GPT. If you ever see a GPT disk with zero partitions you can get here.
      continue;
    }

    InfoPrint ("Partition No.%d:\r\n", PartitionNumber++);
    InfoPrint ("\tType         = 0x%02x ", Mbr->Partition[Index].OSIndicator);
    PartitionFirstLbaWithData = UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA);
    switch (Mbr->Partition[Index].OSIndicator)
    {
      // Two partition types have been reserved for FAT32 partitions, 0x0B and 0x0C.
      case 0x0B:
      case 0x0C:
        InfoPrint ("(FAT32)\r\n");
        mFat32PartitionLba = UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA);
        break;
      case 0x83:
        InfoPrint ("(Linux)\r\n");
        mLinuxPartitionLba = UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA);
        // 1st 1024 bytes are unused, the Ext4 superblock start at offset 1024 bytes.
        PartitionFirstLbaWithData = UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA) + (1024/512);
        break;
      case 0xA2:
        InfoPrint ("(Altera Boot Image)\r\n");
        mAlteraBootImagePartitionLba = UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA);
        break;
      default: InfoPrint ("\r\n"); break;
    }
    InfoPrint (
      "\tStarting LBA = %d\r\n"
      "\tSize In LBA  = %d (%Ld MB)\r\n",
      UNPACK_UINT32 (Mbr->Partition[Index].StartingLBA),
      (UINT32)UNPACK_UINT32 (Mbr->Partition[Index].SizeInLBA),
      (UINT64)((UNPACK_UINT32 (Mbr->Partition[Index].SizeInLBA) * 512)/1024/1024));

    // Dump first block with data of each partition
    InfoPrint ("Reading LBA %d\r\n", PartitionFirstLbaWithData);
    Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, PartitionFirstLbaWithData, 512, PartBuffer);
    if (Status != EFI_SUCCESS) {
      InfoPrint ("ERROR: Fail to Read Block\r\n");
    } else {
      MmioHexDumpEx((UINTN)PartBuffer, 512/4,(UINTN)(PartitionFirstLbaWithData * 512));
    }

    // If Linux File System
    if (Mbr->Partition[Index].OSIndicator == 0x83)
    {
      // Validate the Super Block magic number
      if ((PartBufferArray[0x38] != 0x53) ||
          (PartBufferArray[0x39] != 0xEF))
      {
        InfoPrint ("Invalid ext2/ext3/ext4 partition\r\n");
      }
    }

    // If FAT32: Dump Boot Sector Info
    if ((Mbr->Partition[Index].OSIndicator == 0x0B) || (Mbr->Partition[Index].OSIndicator == 0x0C))
    {
      // FAT Type Determination
      // The FAT type is determined by the count of clusters on the volume and nothing else
      // according to fatgen103 spec published by Microsoft.

      if(Fat32VolBs->Bpb.FATSz16 != 0)
          FATSz = Fat32VolBs->Bpb.FATSz16;
      else
          FATSz = Fat32VolBs->BpbEx.FATSz32;
      if(Fat32VolBs->Bpb.TotSec16 != 0)
          TotSec = Fat32VolBs->Bpb.TotSec16;
      else
          TotSec = Fat32VolBs->Bpb.TotSec32;

      // RootDirSectors count of sectors occupied by the root directory for FAT32 is always 0
      RootDirSectors = ((Fat32VolBs->Bpb.RootEntCnt * 32) + (Fat32VolBs->Bpb.BytsPerSec - 1)) / Fat32VolBs->Bpb.BytsPerSec;
      mFirstDataSector = (Fat32VolBs->Bpb.RsvdSecCnt + (Fat32VolBs->Bpb.NumFATs * FATSz) + RootDirSectors);
      DataSec = TotSec - mFirstDataSector;
      CountofClusters = DataSec / Fat32VolBs->Bpb.SecPerClus;

      mClusterSize = Fat32VolBs->Bpb.BytsPerSec * Fat32VolBs->Bpb.SecPerClus;

      // Now we can determine the FAT type.
      if(CountofClusters < 4085) {
        /* Volume is FAT12 */
        mIsValidFAT32 = FALSE;
      } else if(CountofClusters < 65525) {
        /* Volume is FAT16 */
        mIsValidFAT32 = FALSE;
      } else {
          if (Fat32VolBs->BS_BootSig == 0x29) {
            /* Volume is FAT32 */
            mIsValidFAT32 = TRUE;
          } else {
            mIsValidFAT32 = FALSE;
          }
      }

      if (mIsValidFAT32 == FALSE)
      {
        InfoPrint ("ERROR: FAT32 boot sector not found!\r\n");
      }
      else
      {

        mFirstFatSector = Fat32VolBs->Bpb.RsvdSecCnt;
        mBytePerSector = Fat32VolBs->Bpb.BytsPerSec;
        mSectorPerCluster = Fat32VolBs->Bpb.SecPerClus;
        mRootDirectoryCluster = Fat32VolBs->BpbEx.RootClus;

        InfoPrint (
          "FAT32 Boot Sector found:\r\n"
          "\t Bytes Per Sector   = %d\r\n"
          "\t Sector Per Cluster = %d\r\n"
          "\t Cluster Size       = %d KB\r\n"
          "\t Count of Clusters  = %d\r\n"
          "\t Total Sectors      = %d\r\n"
          "\t Data Sectors       = %d\r\n"
          "\t 1st FAT Sectors    = %d\r\n"
          "\t 1st Data Sectors   = %d\r\n"
          "\t Volumn Size        = %d MB\r\n",
          mBytePerSector,
          mSectorPerCluster,
          mClusterSize / 1024,
          CountofClusters,
          TotSec,
          DataSec,
          mFirstFatSector,
          mFirstDataSector,
          DataSec/2048); // MiB calculation is assuming 512 bytes per sector

        if (mClusterSize > MAX_SUPPORTED_CLUSTER_SIZE)
        {
          mIsValidFAT32 = FALSE;
          InfoPrint ("FAT32 Cluster Size of %d KB is not supported! Max = %d KB (PcdMaxFAT32ClusterSize)\r\n", mClusterSize / 1024, MAX_SUPPORTED_CLUSTER_SIZE  / 1024);
        }

        // // Dump first block of FAT
        // InfoPrint ("Reading 1st FAT Sector at LBA %d\r\n", mFat32PartitionLba + mFirstFatSector);
        // Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, mFat32PartitionLba + mFirstFatSector, 512, PartBuffer);
        // if (Status != EFI_SUCCESS) {
        //   InfoPrint ("ERROR: Fail to Read Block\r\n");
        // } else {
        //   MmioHexDumpEx((UINTN)PartBuffer, 512/4,(UINTN)(mFirstFatSector * 512));
        // }
        //
        // // Dump first block of DATA
        // InfoPrint ("Reading 1st DATA Sector at LBA %d\r\n", mFat32PartitionLba + mFirstDataSector);
        // Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, mFat32PartitionLba + mFirstDataSector, 512, PartBuffer);
        // if (Status != EFI_SUCCESS) {
        //   InfoPrint ("ERROR: Fail to Read Block\r\n");
        // } else {
        //   MmioHexDumpEx((UINTN)PartBuffer, 512/4,(UINTN)(mFirstDataSector * 512));
        // }

      }

    }

  }

  return EFI_SUCCESS;
}

//-------------------------------------------------------------------
//
// FAT32 CLUSTER CHAINING LAYER
//
//-------------------------------------------------------------------

VOID
EFIAPI
OpenFileByCluster (
  UINT32  FileStartClusterNumber
  )
{

  // Open file
  mFileStartClusterNumber = FileStartClusterNumber;
  mNextClusterNumber = mFileStartClusterNumber;
  mReachedEndOfCluster = FALSE;

}

EFI_STATUS
EFIAPI
ReadFileNextCluster (
  VOID
  )
{
  EFI_STATUS          Status;
  UINT32              ThisClusterNumber;
  UINT32              ThisClusterDataSecNum;
  UINT32              ThisClusterFATOffset;
  UINT32              ThisClusterFATSecNum;
  UINT32              ThisClusterFATEntOffset;
  UINT32              ThisClusterFATEntRawData;
  UINT32              ThisClusterFATEnt28BitData;

  // Read first/next/last cluster of a file
  ThisClusterNumber       = mNextClusterNumber;
  ThisClusterDataSecNum   = ((ThisClusterNumber - 2) * mSectorPerCluster) + mFirstDataSector;
  ThisClusterFATOffset    = ThisClusterNumber * 4; // assuming FAT32 where each entry for a cluster is 4 bytes
  ThisClusterFATSecNum    = mFirstFatSector + (ThisClusterFATOffset / mBytePerSector);
  ThisClusterFATEntOffset = ThisClusterFATOffset % mBytePerSector;

  // InfoPrint ("Locating FAT entry and DATA for Cluster %d:\r\n", ThisClusterNumber);
  // InfoPrint ("Sector of FAT for Cluster %d       \t= %d\r\n", ThisClusterNumber, ThisClusterFATSecNum);
  // InfoPrint ("Entry Offset in FAT for Cluster %d \t= %d\r\n", ThisClusterNumber, ThisClusterFATEntOffset);
  // InfoPrint ("Sector of DATA for Cluster %d      \t= %d\r\n", ThisClusterNumber, ThisClusterDataSecNum);

  // Block Read one Cluster of FAT to get Entry of this cluster
  if (ThisClusterFATSecNum != mClusterFATSecNumCachedInFatEntryBuffer)
  {
    mClusterFATSecNumCachedInFatEntryBuffer = ThisClusterFATSecNum;

    //InfoPrint ("Reading FAT Sector %d at LBA %d\r\n", ThisClusterFATSecNum, mFat32PartitionLba + ThisClusterFATSecNum);
    Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, mFat32PartitionLba +
                            ThisClusterFATSecNum, mClusterSize, mFatEntryBuffer);
    if (Status != EFI_SUCCESS) {
      InfoPrint ("ERROR: Fail to Read FAT Sector %d at LBA %d\r\n", ThisClusterFATSecNum, mFat32PartitionLba + ThisClusterFATSecNum);
      return EFI_DEVICE_ERROR;
    }
    // Dump block of FAT
    // MmioHexDumpEx((UINTN)mFatEntryBuffer, mClusterSize/4,(UINTN)(ThisClusterFATSecNum * mClusterSize));

  }

  ThisClusterFATEntRawData = mFatEntryBuffer[ThisClusterFATEntOffset/4];
  ThisClusterFATEnt28BitData = ThisClusterFATEntRawData & 0x0FFFFFFF;
  ThisClusterFATEnt28BitData = ThisClusterFATEnt28BitData | ((ThisClusterFATEnt28BitData >= 0x0FFFFFF7) ? (-1 &~0xF) : 0);

  // Get meaning of FAT Entry
  //InfoPrint ("Cluster %d Entry = 0x%08x = ", ThisClusterNumber, ThisClusterFATEnt28BitData);
  mReachedEndOfCluster = TRUE; //Assuming this is LAST cluster unless we successfully found NEXT cluster number.
  switch (ThisClusterFATEnt28BitData)
  {
    case FAT_CLUSTER_FREE:
      //InfoPrint ("Free\r\n");
      break;
    case FAT_CLUSTER_RESERVED:
      //InfoPrint ("Rsvd\r\n");
      break;
    case FAT_CLUSTER_LAST:
      //InfoPrint ("Last\r\n");
      break;
    default:
      if (FAT_END_OF_FAT_CHAIN (ThisClusterFATEnt28BitData)) {
        //InfoPrint ("LAST\r\n");
      } else {
        //InfoPrint ("Next Cluster %d\r\n", ThisClusterFATEnt28BitData);
        mNextClusterNumber = ThisClusterFATEnt28BitData;
        mReachedEndOfCluster = FALSE;
      }
      break;
  }

  // Block Read one Cluster of DATA into buffer
  if (ThisClusterDataSecNum != mClusterDataSecNumCachedInDataBuffer)
  {
    mClusterDataSecNumCachedInDataBuffer = ThisClusterDataSecNum;

    // Dump first block of DATA
    //InfoPrint ("Reading DATA Sector %d at LBA %d\r\n", ThisClusterDataSecNum, mFat32PartitionLba + ThisClusterDataSecNum);
    Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId, mFat32PartitionLba + ThisClusterDataSecNum, mClusterSize, mDataBuffer);
    if (Status != EFI_SUCCESS) {
      InfoPrint ("ERROR: Fail to Read DATA Sector %d at LBA %d\r\n", ThisClusterDataSecNum, mFat32PartitionLba + ThisClusterDataSecNum);
      return EFI_DEVICE_ERROR;
    }

    // Dump first 512 bytes of file
    // if (ThisClusterNumber == mFileStartClusterNumber)
    // {
    //   InfoPrint ("Hex Dump of first 512 bytes of data cluster(s):\r\n");
    //   MmioHexDumpEx((UINTN)mDataBuffer, 512/4,(UINTN)(ThisClusterDataSecNum * mClusterSize));
    // }

    // Dump last cluster of file
    // if (mReachedEndOfCluster == TRUE)
    // {
    //   InfoPrint ("Hex Dump of last cluster of file:\r\n");
    //   MmioHexDumpEx((UINTN)mDataBuffer, mClusterSize/4,(UINTN)(ThisClusterDataSecNum * mClusterSize));
    // }

  }

  return EFI_SUCCESS;
}

//-------------------------------------------------------------------
//
// DIRECTORY LAYER
//
//-------------------------------------------------------------------

VOID
EFIAPI
ListFileInDirectory (
  UINT32                  StartOfDirectoryCluster
  )
{
  UINTN                NumberOfDirectoryEntriesInACluster;
  FAT_DIRECTORY_UNION* DirectoryEntries;
  FAT_DIRECTORY_UNION* LfnDirectoryEntries;
  FAT_DIRECTORY_UNION  PreviousClusterLastFewDirectoryEntries[260/13];
  UINTN                i, j, LfnIndex;
  UINT8                ShortFileName[13];  //12345678.123 + NULL
  UINT8                LongFileName[261]; //260 + NULL
  UINTN                TotalFileCount = 0;
  UINT64               TotalFileSize = 0;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  DirectoryEntries = (FAT_DIRECTORY_UNION*)&mDataBuffer[0];
  NumberOfDirectoryEntriesInACluster = (mClusterSize / 32);

  // Looking for file in root directory structure
  OpenFileByCluster (StartOfDirectoryCluster);
  while (mReachedEndOfCluster == FALSE)
  {
    // Backup previous cluster last few directory entries data if any
    // This is needed in case long filename crossed a cluster boundary during backward array scan
    CopyMem (&PreviousClusterLastFewDirectoryEntries[0],
             &DirectoryEntries[NumberOfDirectoryEntriesInACluster - (260/13)],
             sizeof(PreviousClusterLastFewDirectoryEntries));

    // Read a cluster block of data for processing
    ReadFileNextCluster ();

    // Process each root directory entries
    for (i = 0; i < NumberOfDirectoryEntriesInACluster; i++)
    {
      // Check if it is not an empty entry or deleted entry or long filename entry or volume label
      if ((DirectoryEntries[i].LfnEntry.Ordinal != 0x00) &&
          (DirectoryEntries[i].LfnEntry.Ordinal != 0xE5) &&
          (DirectoryEntries[i].LfnEntry.Attributes != FAT_ATTRIBUTE_LFN) &&
          (DirectoryEntries[i].LfnEntry.Attributes != FAT_ATTRIBUTE_VOLUME_ID))
      {
        // Print date and time
        Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
          "%04d-%02d-%02d  %02d:%02d:%02d\t",
           DirectoryEntries[i].DirEntry.FileModificationTime.Date.Year+1980,
           DirectoryEntries[i].DirEntry.FileModificationTime.Date.Month,
           DirectoryEntries[i].DirEntry.FileModificationTime.Date.Day,
           DirectoryEntries[i].DirEntry.FileModificationTime.Time.Hour,
           DirectoryEntries[i].DirEntry.FileModificationTime.Time.Minute,
           DirectoryEntries[i].DirEntry.FileModificationTime.Time.DoubleSecond*2);
        // Print filesize or <DIR> prefix
        if (DirectoryEntries[i].DirEntry.Attributes == FAT_ATTRIBUTE_DIRECTORY) {
          Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
            "<DIR>       ");
        } else {
          Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
            "%10d  ", DirectoryEntries[i].DirEntry.FileSize);
          // Total up for Summary
          TotalFileSize += (UINT64) DirectoryEntries[i].DirEntry.FileSize;
          TotalFileCount++;
        }
        // Construct the short filename
        for (j=0;j<8;j++)
        {
          if (DirectoryEntries[i].DirEntry.FileName[j] != 0x20)
            ShortFileName[j] = DirectoryEntries[i].DirEntry.FileName[j];
          else
            break;
        }
        if ((DirectoryEntries[i].DirEntry.FileName[8] != 0x20) ||
            (DirectoryEntries[i].DirEntry.FileName[9] != 0x20) ||
            (DirectoryEntries[i].DirEntry.FileName[10]!= 0x20))
        {
          ShortFileName[j++] = '.';
          ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[8];
          if (DirectoryEntries[i].DirEntry.FileName[9] != 0x20)
            ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[9];
          if (DirectoryEntries[i].DirEntry.FileName[10] != 0x20)
            ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[10];
        }
        ShortFileName[j] = 0; // ASCII NULL terminalted string

        // Print short filename
        Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
          "%12a\t", ShortFileName);

        // Construct the long filename
        for (j=0; j <sizeof(LongFileName); j++)
        {
          LongFileName[j] = 0;
        }
        if (i >= 1) {
          // Backward scaning the array
          if ((DirectoryEntries[i-1].LfnEntry.Ordinal == 0x01) || (DirectoryEntries[i-1].LfnEntry.Ordinal == 0x41))
          {
            // Long filename exist
            LfnIndex = 0;
            // j = LFN sequence
            j = 0;
            do {
              j++; // LFN sequence start at 1

              // Not supporting long filename crossed a cluster boundary during backward array scan
              if (j > i) {
                LfnDirectoryEntries = &PreviousClusterLastFewDirectoryEntries[(260/13)-(j-i)];
              } else {
                LfnDirectoryEntries = &DirectoryEntries[i-j];
              }

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[1];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[2];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[3];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[4];

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[1];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[2];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[3];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[4];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[5];

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name3[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name3[1];

            } while(LfnDirectoryEntries->LfnEntry.Ordinal < 0x40);
          }
        }

        // Print long filename
        Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
          "%a\r\n", LongFileName);

        // Flush the string buffer to serial port if it is going to be full
        if (((UINTN)Char8Ptr + 400) > ((UINTN)&Char8Str[0] + sizeof(Char8Str)))
        {
          SerialPortPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
          Char8Str[0] = 0;
        }

      } //END if is file or is dir

    } //END for (i = 0; i < NumberOfDirectoryEntriesInACluster; i++)

  } //END while (mReachedEndOfCluster == FALSE)

  // Print the remaining message in string buffer
  SerialPortPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // Print Summary
  InfoPrint ("%6d File(s)  %18Ld bytes\r\n", TotalFileCount, (UINT64) TotalFileSize);

}


VOID
EFIAPI
FindFileInDirectory (
  UINT32                  StartOfDirectoryCluster,
  CHAR8*                  pFindByFileName,
  UINT32*                 pFirstClusterOfFile,
  UINT32*                 pFileSize
  )
{
  UINTN                NumberOfDirectoryEntriesInACluster;
  FAT_DIRECTORY_UNION* DirectoryEntries;
  FAT_DIRECTORY_UNION* LfnDirectoryEntries;
  FAT_DIRECTORY_UNION  PreviousClusterLastFewDirectoryEntries[260/13];
  UINTN                i, j, LfnIndex;
  UINT8                ShortFileName[13];  //12345678.123 + NULL
  UINT8                LongFileName[261]; //260 + NULL
  UINT8                FileNameSearchUpper[261]; //260 + NULL

  // Return 0 when file not file
  *pFirstClusterOfFile = 0;
  *pFileSize = 0;

  DirectoryEntries = (FAT_DIRECTORY_UNION*)&mDataBuffer[0];
  NumberOfDirectoryEntriesInACluster = (mClusterSize / 32);

  // Looking for file in root directory structure
  OpenFileByCluster (StartOfDirectoryCluster);
  while (mReachedEndOfCluster == FALSE)
  {
    // Backup previous cluster last few directory entries data if any
    // This is needed in case long filename crossed a cluster boundary during backward array scan
    CopyMem (&PreviousClusterLastFewDirectoryEntries[0],
             &DirectoryEntries[NumberOfDirectoryEntriesInACluster - (260/13)],
             sizeof(PreviousClusterLastFewDirectoryEntries));

    // Read a cluster block of data for processing
    ReadFileNextCluster ();

    // Process each root directory entries
    for (i = 0; i < NumberOfDirectoryEntriesInACluster; i++)
    {
      // Check if it is not an empty entry or deleted entry or long filename entry or volume label
      if ((DirectoryEntries[i].LfnEntry.Ordinal != 0x00) &&
          (DirectoryEntries[i].LfnEntry.Ordinal != 0xE5) &&
          (DirectoryEntries[i].LfnEntry.Attributes != FAT_ATTRIBUTE_LFN) &&
          (DirectoryEntries[i].LfnEntry.Attributes != FAT_ATTRIBUTE_VOLUME_ID))
      {
        // Construct the short filename
        for (j=0;j<8;j++)
        {
          if (DirectoryEntries[i].DirEntry.FileName[j] != 0x20)
            ShortFileName[j] = DirectoryEntries[i].DirEntry.FileName[j];
          else
            break;
        }
        if ((DirectoryEntries[i].DirEntry.FileName[8] != 0x20) ||
            (DirectoryEntries[i].DirEntry.FileName[9] != 0x20) ||
            (DirectoryEntries[i].DirEntry.FileName[10]!= 0x20))
        {
          ShortFileName[j++] = '.';
          ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[8];
          if (DirectoryEntries[i].DirEntry.FileName[9] != 0x20)
            ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[9];
          if (DirectoryEntries[i].DirEntry.FileName[10] != 0x20)
            ShortFileName[j++] = DirectoryEntries[i].DirEntry.FileName[10];
        }
        ShortFileName[j] = 0; // ASCII NULL terminalted string

        // Convert Short Filename To Upper Case
        for (j=0; j <sizeof(ShortFileName); j++)
        {
          if (ShortFileName[j] >= 'a' && ShortFileName[j] <= 'z') {
             ShortFileName[j] = (ShortFileName[j] - ('a' - 'A'));
          }
        }

        // Construct the long filename
        for (j=0; j <sizeof(LongFileName); j++)
        {
          LongFileName[j] = 0;
        }
        if (i >= 1) {
          // Backward scaning the array
          if ((DirectoryEntries[i-1].LfnEntry.Ordinal == 0x01) || (DirectoryEntries[i-1].LfnEntry.Ordinal == 0x41))
          {
            // Long filename exist
            LfnIndex = 0;
            // j = LFN sequence
            j = 0;
            do {
              j++; // LFN sequence start at 1

              // Not supporting long filename crossed a cluster boundary during backward array scan
              if (j > i) {
                LfnDirectoryEntries = &PreviousClusterLastFewDirectoryEntries[(260/13)-(j-i)];
              } else {
                LfnDirectoryEntries = &DirectoryEntries[i-j];
              }

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[1];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[2];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[3];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name1[4];

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[1];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[2];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[3];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[4];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name2[5];

              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name3[0];
              LongFileName[LfnIndex++] = (UINT8) LfnDirectoryEntries->LfnEntry.Name3[1];

            } while(LfnDirectoryEntries->LfnEntry.Ordinal < 0x40);
          }
        }

        // Convert Long Filename To Upper Case
        for (j=0; j <sizeof(LongFileName); j++)
        {
          if (LongFileName[j] >= 'a' && LongFileName[j] <= 'z') {
             LongFileName[j] = (LongFileName[j] - ('a' - 'A'));
          }
        }

        // We can now search for file name macthing here.

        // Convert filename search string to Upper Case
        if (pFindByFileName != NULL)
        {
          j = 0;
          while ((*(pFindByFileName + j) != '\0')) {
            FileNameSearchUpper[j] = *(pFindByFileName + j);
            if (FileNameSearchUpper[j] >= 'a' && FileNameSearchUpper[j] <= 'z') {
               FileNameSearchUpper[j] = (FileNameSearchUpper[j] - ('a' - 'A'));
            }
            j++;
            if (j >= sizeof(FileNameSearchUpper)) break;
          }
          FileNameSearchUpper[j] = 0;

          //if (AsciiStrCmp ((CONST CHAR8*)FileNameSearchUpper, (CONST CHAR8*)ShortFileName) == 0) {
          //  InfoPrint ("Short filename matached.\r\n");
          //}
          //
          //if (AsciiStrCmp ((CONST CHAR8*)FileNameSearchUpper, (CONST CHAR8*)LongFileName) == 0) {
          //  InfoPrint ("Long filename matached.\r\n");
          //}

          if ((AsciiStrCmp ((CONST CHAR8*)FileNameSearchUpper, (CONST CHAR8*)ShortFileName) == 0) ||
              (AsciiStrCmp ((CONST CHAR8*)FileNameSearchUpper, (CONST CHAR8*)LongFileName) == 0))
          {
            *pFirstClusterOfFile = ((UINT32)DirectoryEntries[i].DirEntry.FileClusterHigh << 16) |
                                    (UINT32)DirectoryEntries[i].DirEntry.FileCluster;
            *pFileSize = DirectoryEntries[i].DirEntry.FileSize;

            // Print file info
            ProgressPrint ("Opening file:\r\n"
                             "\t Filename : %a\r\n"
                             //"\t Cluster  : %d\r\n"
                             "\t Date     : %04d-%02d-%02d\r\n"
                             "\t Time     : %02d:%02d:%02d\r\n"
                             "\t Size     : ",
                             pFindByFileName,
                             //*pFirstClusterOfFile,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Date.Year+1980,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Date.Month,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Date.Day,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Time.Hour,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Time.Minute,
                             DirectoryEntries[i].DirEntry.FileModificationTime.Time.DoubleSecond*2);

            if (DirectoryEntries[i].DirEntry.FileSize >= (1024*1024*10)) {
              ProgressPrint ( "%d MB\r\n", DirectoryEntries[i].DirEntry.FileSize / 1024 / 1024);
            } else if (DirectoryEntries[i].DirEntry.FileSize >= (1024*10)) {
              ProgressPrint ( "%d KB\r\n", DirectoryEntries[i].DirEntry.FileSize / 1024);
            } else {
              ProgressPrint ( "%d Bytes\r\n", DirectoryEntries[i].DirEntry.FileSize);
            }

            // Stop searching
            return;
          }
        }

        // Print filename
        // InfoPrint ("%a (%a) is not the file we are looking for.\r\n", ShortFileName, LongFileName);

      }
    } // END for (i = 0; i < NumberOfDirectoryEntriesInACluster; i++)
  } // END while (mReachedEndOfCluster == FALSE)
}


VOID
EFIAPI
ListFileInRootDirectory (
  VOID
  )
{
  if ((mFat32PartitionLba != 0) && (mIsValidFAT32 == TRUE))
  {
    InfoPrint ("Directory of Root:\r\n");
    ListFileInDirectory (mRootDirectoryCluster);
  }
}


VOID
EFIAPI
FindFileInRootDirectory (
  CHAR8*                  pFindByFileName,
  UINT32*                 pFirstClusterOfFile,
  UINT32*                 pFileSize
  )
{
  FindFileInDirectory (mRootDirectoryCluster, pFindByFileName, pFirstClusterOfFile, pFileSize);
}

//-------------------------------------------------------------------
//
// FILE LAYER
//
//-------------------------------------------------------------------

VOID
EFIAPI
OpenFileInRootDirectory (
  FAT32_FILE*             File
  )
{
  // Init variables
  File->Found = FALSE;
  File->EndOfFile = FALSE;
  File->NextFilePos = 0;
  File->LastReadDataSize = 0;
  File->FileData = &mDataBuffer[0];

  if ((mFat32PartitionLba != 0) && (mIsValidFAT32 == TRUE))
  {
    // Find found in root directory
    FindFileInRootDirectory(File->FileName, &(File->FileFirstCluster), &(File->FileSize));

    // File exists?
    if ((File->FileFirstCluster > 0) && (File->FileSize > 0))
    {
      // Yes, found
      File->Found = TRUE;
      OpenFileByCluster (File->FileFirstCluster);
    }
  }
}

EFI_STATUS
EFIAPI
ReadFileNextData (
  FAT32_FILE*             File
  )
{
  EFI_STATUS           Status;

  // Read a block of data
  Status = ReadFileNextCluster ();

  // Calculate how big is the block of data we just read
  if (mReachedEndOfCluster == TRUE) {
    File->LastReadDataSize = File->FileSize % mClusterSize;
    File->EndOfFile = TRUE;
  } else {
    File->LastReadDataSize = mClusterSize;
  }

  // NextFilePos point to the start of next block of data
  File->NextFilePos += File->LastReadDataSize;

  return Status;
}

VOID
EFIAPI
DumpTextFile (
  CHAR8*               TextFileFilename
  )
{
  FAT32_FILE           MyFile;

  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);
  if (MyFile.Found == TRUE)
  {
    while (MyFile.EndOfFile == FALSE)
    {
      ReadFileNextData (&MyFile);
      //MmioHexDumpEx((UINTN)MyFile.FileData, MyFile.LastReadDataSize/4, MyFile.NextFilePos - MyFile.LastReadDataSize);
      SerialPortWrite ((UINT8 *) MyFile.FileData, MyFile.LastReadDataSize);
    }
  } else {
    InfoPrint ("File not found %a\r\n", TextFileFilename);
  }
}


//-------------------------------------------------------------------
//
// INIT FUNCTION
//
//-------------------------------------------------------------------

EFI_STATUS
EFIAPI
AlteraSdMmcPeiInit(
  VOID
  )
{
  MmcPeiInitialize ();

  MmcHostInstance = GetMmcHostInstance();

  if (MmcHostInstance->BlockIo.Media->LastBlock != 0) {
    //InfoPrint ("Card Detected!\r\n");

    DiscoverPartitionTableType ();

    InfoPrint ("Altera Boot Image Partition (Type 0xA2) ");
    if (mAlteraBootImagePartitionLba != 0)
    {
      InfoPrint ("found at LBA %d\r\n", mAlteraBootImagePartitionLba);
    } else {
      InfoPrint ("not found.\r\n");
    }

    InfoPrint ("FAT32 Partition ");
    if ((mFat32PartitionLba != 0) && (mIsValidFAT32 == TRUE))
    {
      InfoPrint ("found at LBA %d\r\n", mFat32PartitionLba);

      // List Root Directory files and folders
      ListFileInRootDirectory();

      //DumpTextFile ("README.TXT");

    } else {
      InfoPrint ("not found.\r\n");
    }

#ifdef SDMMC_DIAGNOSTICS
    PeiSdMmcRunDiagnostics (MmcHostInstance);
#endif

    return EFI_SUCCESS;
  } else {
    return EFI_NO_MEDIA;
  }
}

