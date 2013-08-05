#ifndef __ERRORS_H__
#define __ERRORS_H__


const unsigned char error_pat[ ERROR_NUM + 1 ] =
{
	0xCC, // 11001100 low voltage
	0xBF, // 10111111 hall1
	0xAF, // 10101111 hall2
	0xAA, // 10101010 watchdog
	0x00, // 00000000 none
};


#endif

