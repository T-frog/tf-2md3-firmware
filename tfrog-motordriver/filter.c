#include "filter.h"

int Filter1st_Filter( Filter1st *filter, int input )
{
	filter->x = ( filter->k[0] * input + filter->k[1] * filter->x ) / 256;
	return ( filter->k[2] * input + filter->k[3] * filter->x ) / 256;
}

int Filter1st_CreateLPF( Filter1st *filter, float timeconst )
{
	filter->k[3] = (int)( ( -1.0 / ( 1.0 + 2.0 * timeconst ) ) * 256.0 );
	filter->k[2] = - filter->k[3];
	filter->k[1] = (int)( ( ( 1.0 - 2.0 * timeconst ) * ( -1.0 / ( 1.0 + 2.0 * timeconst ) ) ) * 256.0 );
	filter->k[0] = - filter->k[1] - 256;
	filter->x = 0;
}
