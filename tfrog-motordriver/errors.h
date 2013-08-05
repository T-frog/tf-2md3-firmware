#ifndef __ERRORS_H__
#define __ERRORS_H__


const unsigned char error_pat[ ERROR_NUM + 1 ] =
{
	0xCC, // 11001100 low voltage
	0xFD, // 11111101 hall1
	0xF5, // 11110101 hall2
	0xAA, // 10101010 watchdog
	0x00, // 00000000 none
};


#endif

