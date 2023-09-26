#include <stddef.h>

#include "ff.h"
#include "diskio.h"

#include "disk.h"


const disk_desc *ramDisk = NULL;

// Default functions
DSTATUS def_disk_initialize (void){ return STA_NOINIT; }
DSTATUS def_disk_status (void){ return STA_NODISK; }
DRESULT def_disk_read (BYTE* buff, LBA_t sector, UINT count){return RES_NOTRDY; }
DRESULT def_disk_write (const BYTE* buff, LBA_t sector, UINT count){ return RES_NOTRDY; }
DRESULT def_disk_ioctl (BYTE cmd, void* buff){ return RES_NOTRDY; }


DSTATUS disk_initialize (BYTE pdrv){
	if ( ramDisk ){
		return ramDisk->disk_initialize();
	}
	return def_disk_initialize();
}

DSTATUS disk_status (BYTE pdrv){
	if ( ramDisk ){
		return ramDisk->disk_status();
	}
	return def_disk_status();
}

DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count){
	if ( ramDisk ){
		return ramDisk->disk_read( buff, sector, count );
	}
	return def_disk_read(buff, sector, count );
}

DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count){
	if ( ramDisk ) {
		return ramDisk->disk_write( buff, sector, count );
	}
	return def_disk_write(buff, sector, count);
}

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff){
	if ( ramDisk ) {
		return ramDisk->disk_ioctl( cmd, buff );
	}
	return def_disk_ioctl(cmd, buff);
}


