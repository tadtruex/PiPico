#include "ff.h"
#include "disk.h"
#include "ramdisk.h"

#define KBYTES_HERE 64

BYTE ramDiskBuffer[KBYTES_HERE * 1024];


DSTATUS rd_initialize(){ return 0; }
DSTATUS rd_status(){ return 0; }

DRESULT rd_read  (BYTE* buff, LBA_t sector, UINT count){ return RES_ERROR; }
DRESULT rd_write (const BYTE* buff, LBA_t sector, UINT count){ return RES_ERROR; }


DRESULT rd_ioctl (BYTE cmd, void* buff){
  switch (cmd) {
  case CTRL_SYNC: return RES_OK;
  case GET_SECTOR_COUNT:
    *((LBA_t *)buff) = KBYTES_HERE * 2;
    return RES_OK;
  case GET_SECTOR_SIZE:
    *((WORD *)buff) = 512;
    return RES_OK;
  case GET_BLOCK_SIZE:
    *((DWORD*)buff) = 1;
    return RES_OK;
  case CTRL_TRIM:
    return RES_OK;

    
  default : return RES_ERROR;
  }
  
  return RES_ERROR;
}

const disk_desc RamDisk0 = {
  rd_initialize,
  rd_status,
  rd_read,
  rd_write,
  rd_ioctl,
};

void initRamDisk(){
  ramDisk = &RamDisk0;
}




