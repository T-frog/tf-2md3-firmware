/**
	@file fixp.h
	@brief Fixed point value operation
 */

#ifndef __FIX_POINT_AWD__
#define __FIX_POINT_AWD__


#define INLINE	inline

typedef int fixp4;

#ifndef FP4_POINTBIT
# define FP4_POINTBIT 17
#endif

# define FP4_POINTBIT2		(FP4_POINTBIT*2)
# define FP4_DIV			(1.0/(double)(1<<FP4_POINTBIT))
# define FP4_MUL			((double)(1<<FP4_POINTBIT))
# define FP4_ONE			(1<<FP4_POINTBIT)

#define DOUBLE2FP4(a)		(fixp4)( a * FP4_MUL + 0.5 )
#define INT2FP4(a)			(fixp4)( a << FP4_POINTBIT )
	
INLINE fixp4 fp4mul( fixp4 a, fixp4 b );
INLINE fixp4 fp4mulf( fixp4 a, fixp4 b );
INLINE fixp4 fp4div( fixp4 a, fixp4 b );
INLINE fixp4 double2fp4( double a );
INLINE fixp4 int2fp4( int a );
INLINE double fp42double( fixp4 a );


#endif
