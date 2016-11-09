#ifndef __FILTER_H__
#define __FILTER_H__

#define FIXED_POINT 4096

typedef struct
{
	int k[4];
	int x;
} Filter1st;

int Filter1st_Filter( Filter1st *filter, int input );
int Filter1st_CreateLPF( Filter1st *filter, float timeconst );


#endif
