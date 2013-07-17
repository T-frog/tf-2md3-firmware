#ifndef __IO_H__
#define __IO_H__

void set_io_dir( unsigned char io_dir );
RAMFUNC void set_io_data( unsigned char io_data );
RAMFUNC unsigned char get_io_data( void );

#endif
