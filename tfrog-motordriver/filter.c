#include "filter.h"

int Filter1st_Filter( Filter1st *filter, int input )
{
	filter->x = (filter->k[0] * input + filter->k[1] * filter->x) / FIXED_POINT;
	return ( filter->k[2] * input + filter->k[3] * filter->x ) / FIXED_POINT;
}

int Filter1st_CreateLPF( Filter1st *filter, float timeconst )
{
	filter->k[3] = (int)( ( -1.0 / ( 1.0 + 2.0 * timeconst ) ) * FIXED_POINT );
	filter->k[2] = - filter->k[3];
	filter->k[1] = (int)( ( ( 1.0 - 2.0 * timeconst ) * ( -1.0 / ( 1.0 + 2.0 * timeconst ) ) ) * FIXED_POINT );
	filter->k[0] = - filter->k[1] - FIXED_POINT;
	filter->x = 0;
	return 1;
}
