#ifndef _DISK_H
#define _DISK_H

#include "ff.h"
#include "diskio.h"

// Generic typedef for any disk
typedef struct {
	DSTATUS (*disk_initialize) ();
	DSTATUS (*disk_status) ();
	DRESULT (*disk_read) (BYTE* buff, LBA_t sector, UINT count);
	DRESULT (*disk_write) (const BYTE* buff, LBA_t sector, UINT count);
	DRESULT (*disk_ioctl) (BYTE cmd, void* buff);
} disk_desc;

extern const disk_desc *ramDisk;

#endif
