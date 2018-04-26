/** @file
  A shell application that triggers I2c control.

  Copyright (c) 2016 - 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/PrintLib.h>
#include <AlteraPlatform.h>
#include "I2c.h"

#define CAPSULE_HEADER_SIZE  0x20

#define NESTED_CAPSULE_HEADER_SIZE  SIZE_4KB
#define SYSTEM_FIRMWARE_FLAG 0x50000
#define DEVICE_FIRMWARE_FLAG 0x78010

#define MAJOR_VERSION   1
#define MINOR_VERSION   0

#define MAX_CAPSULE_NUM 10
#define MAX_READ_BYTES  256
#define MAX_WRITE_BYTES 256


extern UINTN  Argc;
extern CHAR16 **Argv;

I2C_CTLR_t  I2cCtrl = ALT_I2C_I2C1;


/**

  This function parse application ARG.

  @return Status
**/
EFI_STATUS
GetArg (
  VOID
  );

/**
  Print APP usage.
**/
VOID
PrintUsage (
  VOID
  );

/**
  Initialization of the I2C Controller

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
I2cInit (
  IN UINT32 BusIndex
  );
  
/**
  Initialization of the I2C Controller

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
GetClkFreq (
  );
  
/**
  Get current bus

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cGetCurrentBus (
  IN UINT32 BusIndex
  );

/**
  Get current bus speed

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cGetCurrentBusSpeed (
  IN UINT32 BusIndex
  );  
  
/**
  Get current bus speed

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cSetCurrentBusSpeed (
  IN UINT32 BusIndex,
  IN UINT32 BusSpeed  
  );

/**
  Initialization of the I2C Target Address

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
I2cSetTargetAddress (
  IN UINT32 BusIndex,
  IN UINT32 DevAddr
  );
  
/**
  Read I2c

  @retval  EFI_SUCCESS  Successfully read from I2c
  @retval  Other        Failed to read from I2c
**/
EFI_STATUS
EFIAPI
I2cReadByte (
  IN UINT32 BusIndex,
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  OUT UINT8 *Buffer
  );

/**
  Write I2c

  @retval  EFI_SUCCESS  Successfully write to I2c
  @retval  Other        Failed to write to I2c
**/
EFI_STATUS
EFIAPI
I2cWriteByte (
  IN UINT32 BusIndex,
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  IN UINT8* Buffer
  );
  
/**
  Handling "I2C bus" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cBusHandler (
  VOID
  )
{
  UINT32       BusIndex;
  I2C_CTLR_t  I2cCtrl;
  
  if ( Argc == 2 ) {
    //show all bus
    for ( I2cCtrl=ALT_I2C_I2C0; I2cCtrl < ALT_I2C_MAX; I2cCtrl+=0x100 ){
        I2cGetCurrentBus(I2cCtrl);
    }
    
  } else {
    //show specific bus
    BusIndex = (UINT32) StrHexToUintn(Argv[2]);
    switch (BusIndex) {
        case 0:
            I2cGetCurrentBus(ALT_I2C_I2C0);
            break;
        case 1:
            I2cGetCurrentBus(ALT_I2C_I2C1);
            break;
        case 2:
            I2cGetCurrentBus(ALT_I2C_I2C2);
            break;
        case 3:
            I2cGetCurrentBus(ALT_I2C_I2C3);
            break;
        case 4:
            I2cGetCurrentBus(ALT_I2C_I2C4);
            break;
        default:
            Print(L"Invalid I2c bus");
            break;
    }
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C crc32" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cCrc32Handler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       ReadAddr;
  UINT32       CrcChecksum;  
  UINT8        ReadLength;
  UINT8        BufferData[MAX_READ_BYTES];

  if ( Argc == 5 ) {

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    ReadAddr = (UINT32)StrHexToUintn(Argv[3]);
    ReadLength = (UINT8)StrHexToUintn(Argv[4]);       
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cReadByte((UINT32)I2cCtrl, 
                (UINT16)DEFAULT_ADDR_LENGTH, 
                ReadAddr, 
                ReadLength, 
                BufferData);
    CrcChecksum = CalculateCrc32(BufferData, ReadLength);
    //show crc32 
    Print(L"CRC32 is %x\n", CrcChecksum);  
  
    
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C dev" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cDevHandler (
  VOID
  )
{
  UINT32       BusIndex;
  
  if ( Argc == 2 ) {
    //show current I2C bus
    Print(L"Current I2c bus is %x\n", I2cCtrl);  
    
  } else {
    //set I2C bus
    BusIndex = (UINT32)StrHexToUintn(Argv[2]);
    switch (BusIndex) {
        case 0:
            I2cCtrl = ALT_I2C_I2C0;
            break;
        case 1:
            I2cCtrl = ALT_I2C_I2C1;
            break;
        case 2:
            I2cCtrl = ALT_I2C_I2C2;
            break;
        case 3:
            I2cCtrl = ALT_I2C_I2C3;
            break;
        case 4:
            I2cCtrl = ALT_I2C_I2C4;           
            break;
        default:
            Print(L"Invalid I2c bus\n");
            return EFI_INVALID_PARAMETER;
    }
    I2cInit(I2cCtrl);
  }
  
  return EFI_SUCCESS;
}

/**
  Handling "I2C loop" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cLoopHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       ReadAddr;
  UINT32       ReadLength;
  UINT8        BufferData[MAX_READ_BYTES];
  UINT8        i;
  
  if (Argc == 5){

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    ReadAddr = (UINT32)StrHexToUintn(Argv[3]);
    ReadLength = (UINT32)StrHexToUintn(Argv[4]);    
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cReadByte((UINT32)I2cCtrl, 
                (UINT16)DEFAULT_ADDR_LENGTH, 
                ReadAddr, 
                ReadLength, 
                BufferData);
    for (i=0; i<ReadLength; i++){
        Print(L"I2c address %x: %x\n",ReadAddr+i ,BufferData[i]);
    }
    
  } else {
    //invalid parameter
    PrintUsage();      
  }
  
  return EFI_SUCCESS;
}

/**
  Handling "I2C md" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cMdHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       ReadAddr;  
  UINT8        ReadLength;
  UINT8        BufferData[MAX_READ_BYTES];
  UINT8        i;

  
  if ( Argc == 5 ) {

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    ReadAddr = (UINT32)StrHexToUintn(Argv[3]);
    ReadLength = (UINT8)StrHexToUintn(Argv[4]);       
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cReadByte((UINT32)I2cCtrl, 
                (UINT16)DEFAULT_ADDR_LENGTH, 
                ReadAddr, 
                ReadLength, 
                BufferData);
    for (i=0; i<ReadLength; i++){
        Print(L"I2c address %x: %x\n",ReadAddr+i ,BufferData[i]);
    }
  
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C mw" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cMwHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       WriteAddr;  
  UINT8        WriteLength;
  UINT8        BufferData[MAX_WRITE_BYTES];
  UINT8        i;

  
  if ( Argc == 6 ) {

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    WriteAddr = (UINT32)StrHexToUintn(Argv[3]);
    WriteLength = (UINT8)StrHexToUintn(Argv[5]);
    if (WriteLength > MAX_WRITE_BYTES){    
       return EFI_INVALID_PARAMETER; 
    }
    for (i=0; i<WriteLength; i++) {
        BufferData[i] = (UINT32)StrHexToUintn(Argv[4]);
    }    
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cWriteByte((UINT32)I2cCtrl, 
                 (UINT16)DEFAULT_ADDR_LENGTH, 
                  WriteAddr, 
                  WriteLength, 
                  BufferData);
  
    
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C nm" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cNmHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       WriteAddr;  
  UINT8        WriteLength;
  UINT8        BufferData[MAX_WRITE_BYTES];
  UINT8        i;

  
  if ( Argc >= 5 ) {

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    WriteAddr = (UINT32)StrHexToUintn(Argv[3]);
    WriteLength = Argc - 4;
    for (i=0; i<WriteLength; i++) {
        BufferData[i] = (UINT32)StrHexToUintn(Argv[i+4]);
    }    
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cWriteByte((UINT32)I2cCtrl, 
                 (UINT16)DEFAULT_ADDR_LENGTH, 
                  WriteAddr, 
                  WriteLength, 
                  BufferData);
  
    
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C probe" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cProbeHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  EFI_STATUS   Status;
  UINT8        BufferData[MAX_READ_BYTES];
  UINT16       ProbeAddr;
  UINT8        ProbeByteLength;
  
  ProbeAddr = 0;
  ProbeByteLength = 1;
  for (ChipIndex=0; ChipIndex<128; ChipIndex++) {
      
      I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
      Status = I2cReadByte((UINT32)I2cCtrl, 
                    (UINT16)DEFAULT_ADDR_LENGTH, 
                    ProbeAddr, 
                    ProbeByteLength, 
                    BufferData);
      if (Status == EFI_SUCCESS) {
           Print(L"DevAddr: Dev address is 0x%02x\n", ChipIndex); 
      } else {
           I2cInit(I2cCtrl);
      } 
  }  

  return EFI_SUCCESS;
}

/**
  Handling "I2C read" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cReadHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       ReadAddr;
  UINT8        * MemAddr;  
  UINT8        ReadLength;
  UINT8        BufferData[MAX_READ_BYTES];
  UINT8        i;
  
  if ( Argc == 6 ) {

    ChipIndex = (UINT32)StrHexToUintn(Argv[2]);
    ReadAddr  = (UINT32)StrHexToUintn(Argv[3]);
    ReadLength = (UINT8)StrHexToUintn(Argv[4]);
    if (ReadLength > MAX_READ_BYTES){    
       return EFI_INVALID_PARAMETER; 
    }    
    MemAddr    = (UINT8*)StrHexToUintn(Argv[5]);    
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cReadByte((UINT32)I2cCtrl, 
                (UINT16)DEFAULT_ADDR_LENGTH, 
                ReadAddr, 
                ReadLength, 
                BufferData);
    for (i=0; i<ReadLength; i++) {
        *(MemAddr + i) = BufferData[i] ;
    }    
    
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C write" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cWriteHandler (
  VOID
  )
{
  UINT32       ChipIndex;
  UINT32       WriteAddr;
  UINT8        * MemAddr;
  UINT8        WriteLength;
  UINT8        BufferData[MAX_WRITE_BYTES];
  UINT8        i;

  
  if ( Argc == 6 ) {

    MemAddr   = (UINT8*)StrHexToUintn(Argv[2]);
    ChipIndex = (UINT32)StrHexToUintn(Argv[3]);
    WriteAddr = (UINT32)StrHexToUintn(Argv[4]);
    WriteLength = (UINT8)StrHexToUintn(Argv[5]);
    if (WriteLength > MAX_WRITE_BYTES){    
       return EFI_INVALID_PARAMETER; 
    }
    for (i=0; i<WriteLength; i++) {
        BufferData[i] = *(MemAddr + i);
    }    
    I2cSetTargetAddress((UINT32)I2cCtrl,ChipIndex);
    I2cWriteByte((UINT32)I2cCtrl, 
                 (UINT16)DEFAULT_ADDR_LENGTH, 
                  WriteAddr, 
                  WriteLength, 
                  BufferData);
  
    
  } else {

    //invalid parameter
    PrintUsage();
  }
  return EFI_SUCCESS;
}

/**
  Handling "I2C speed" command

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cSpeedHandler (
  VOID
  )
{
  UINTN       BusSpeed;
  
  if ( Argc == 2 ) {
    //show current I2C bus speed
    I2cGetCurrentBusSpeed(I2cCtrl);
    
  } else {
    //set I2C bus speed
    BusSpeed = StrDecimalToUintn(Argv[2]);
    I2cSetCurrentBusSpeed(I2cCtrl, BusSpeed);

  }
  return EFI_SUCCESS;
}  
  

/**
  Print APP usage.
**/
VOID
PrintUsage (
  VOID
  )
{
     
  Print(L"I2c:  usage\n");
  Print(L"  i2c bus [muxtype:muxaddr:muxchannel] - show I2C bus info\n");
  Print(L"  i2c crc32 chip address[.0, .1, .2] count - compute CRC32 checksum\n");
  Print(L"  i2c dev [dev] - show or set current I2C bus\n");
  Print(L"  i2c loop chip address[.0, .1, .2] [# of objects] - looping read of device\n");
  Print(L"  i2c md chip address[.0, .1, .2] [# of objects] - read from I2C device\n");
  Print(L"  i2c mw chip address[.0, .1, .2] value [count] - write to I2C device (fill)\n");
  Print(L"  i2c nm chip address[.0, .1, .2] - write to I2C device (constant address)\n");
  Print(L"  i2c probe [address] - test for and show device(s) on the I2C bus\n");
  Print(L"  i2c read chip address[.0, .1, .2] length memaddress - read to memory\n");
  Print(L"  i2c write memaddress chip address[.0, .1, .2] length  - write memory\n");
  Print(L"            to I2C;\n");
  Print(L"  i2c speed [speed] - show or set I2C bus speed\n");

}

/**
  Update Capsule image.

  @param[in]  ImageHandle     The image handle.
  @param[in]  SystemTable     The system table.

  @retval EFI_SUCCESS            Command completed successfully.
  @retval EFI_INVALID_PARAMETER  Command usage error.
  @retval EFI_NOT_FOUND          The input file can't be found.
**/
EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS                    Status;

  Status = GetArg();
  if (EFI_ERROR(Status)) {
    Print(L"Please use UEFI SHELL to run this application!\n", Status);
    return Status;
  }
  if (Argc < 2) {
    PrintUsage();
    return EFI_INVALID_PARAMETER;
  }

  if (StrCmp(Argv[1], L"bus") == 0) {
    Status = I2cBusHandler();
  }
  if (StrCmp(Argv[1], L"crc32") == 0) {
    Status = I2cCrc32Handler();
  }
  if (StrCmp(Argv[1], L"dev") == 0) {
    Status = I2cDevHandler();
  }
  if (StrCmp(Argv[1], L"loop") == 0) {
    Status = I2cLoopHandler();
  }  
  if (StrCmp(Argv[1], L"md") == 0) {
    Status = I2cMdHandler();
  }
  if (StrCmp(Argv[1], L"mw") == 0) {
    Status = I2cMwHandler();
  }
  if (StrCmp(Argv[1], L"nm") == 0) {
    Status = I2cNmHandler();
  }
  if (StrCmp(Argv[1], L"probe") == 0) {
    Status = I2cProbeHandler();
  }
  if (StrCmp(Argv[1], L"read") == 0) {
    Status = I2cReadHandler();
  }
  if (StrCmp(Argv[1], L"write") == 0) {
    Status = I2cWriteHandler();
  }
  if (StrCmp(Argv[1], L"speed") == 0) {
    Status = I2cSpeedHandler();
  }  
 
  

  return Status;
}
