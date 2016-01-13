/** @file
  Altera HPS SD/MMC controller PEI Driver header file

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

#ifndef __ALTERA_SDMMC_PEI_H
#define __ALTERA_SDMMC_PEI_H

// File operation structure
typedef struct {
  CHAR8*               FileName;
  UINT8*               FileData;
  UINT32               FileSize;
  UINT32               FileFirstCluster;
  UINT32               NextFilePos;
  UINT32               LastReadDataSize;
  BOOLEAN              Found;
  BOOLEAN              EndOfFile;
} FAT32_FILE;

EFI_STATUS
EFIAPI
AlteraSdMmcPeiInit (
  VOID
  );

VOID
EFIAPI
OpenFileByCluster (
  UINT32  FileStartClusterNumber
  );

EFI_STATUS
EFIAPI
ReadFileNextCluster (
  VOID
  );

VOID
EFIAPI
OpenFileInRootDirectory (
  FAT32_FILE*             File
  );

EFI_STATUS
EFIAPI
ReadFileNextData (
  FAT32_FILE*             File
  );

VOID
EFIAPI
DumpTextFile (
  CHAR8*                  TextFileFilename
  );

VOID
EFIAPI
ListFileInRootDirectory (
  VOID
  );

EFI_STATUS
EFIAPI
MmcReadLba (
  IN EFI_LBA                  Lba,
  IN UINTN                    BufferSize,
  OUT VOID                    *Buffer
  );

EFI_STATUS
EFIAPI
MmcWriteLba (
  IN EFI_LBA                  Lba,
  IN UINTN                    BufferSize,
  IN VOID                     *Buffer
  );

#endif

