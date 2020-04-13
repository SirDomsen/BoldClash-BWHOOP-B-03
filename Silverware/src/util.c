/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include <math.h>
#include "util.h"
#include "drv_time.h"


// calculates the coefficient for lpf filter, times in the same units
float lpfcalc(float sampleperiod, float filtertime) {
float ga = 1.0f - sampleperiod / filtertime;
if (ga > 1.0f)
	ga = 1.0f;
if (ga < 0.0f)
	ga = 0.0f;
return ga;
}


// calculates the coefficient for lpf filter
float lpfcalc_hz(float sampleperiod, float filterhz) {
float ga = 1.0f - sampleperiod * filterhz;
if (ga > 1.0f)
	ga = 1.0f;
if (ga < 0.0f)
	ga = 0.0f;
return ga;
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{

return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;

}


void lpf( float *out, float in , float coeff)
{
	*out = ( *out )* coeff + in * ( 1-coeff);
}


void limitf ( float *input , const float limit)
{
	if (*input > limit) *input = limit;
	if (*input < - limit) *input = - limit;
}

float rcexpo ( float in , float exp )
{
	if ( exp > 1 ) exp = 1;
	if ( exp < -1 ) exp = -1;
	float ans = in*in*in * exp + in * ( 1 - exp );
	limitf( &ans , 1.0);
	return ans;
}


// timing routines for debugging
static unsigned long timestart;
unsigned long timeend;

// timestart
void TS( void)
{
	timestart = gettime();
}
// timeend
void TE( void)
{
	timeend =( gettime() - timestart );
}



float fastsin( float x )
{
 //always wrap input angle to -PI..PI
while (x < -3.14159265f)
    x += 6.28318531f;

while (x >  3.14159265f)
    x -= 6.28318531f;
float sin1;

//compute sine
if (x < 0)
   sin1 = (1.27323954f + .405284735f * x) *x;
else
   sin1 = (1.27323954f - .405284735f * x) *x;


return sin1;

}


float fastcos( float x )
{
 x += 1.57079632f;
	return fastsin(x);
}


// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06
#if 0
	#define sinPolyCoef3 -1.666568107e-1f
	#define sinPolyCoef5  8.312366210e-3f
	#define sinPolyCoef7 -1.849218155e-4f
	#define sinPolyCoef9  0
#else
	#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
	#define sinPolyCoef5  8.333017292e-3f // Double:  8.333017291562218127986291618761571373087e-3
	#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
	#define sinPolyCoef9  2.600054768e-6f // Double:  2.600054767890361277123254766503271638682e-6
#endif

#define PI_F 3.14159265358979323846f

float sin_approx( float x )
{
	while ( x > PI_F ) { x -= 2.0f * PI_F; } // always wrap input angle to -PI..PI
	while ( x < -PI_F ) { x += 2.0f * PI_F; }
	if ( x > 0.5f * PI_F ) { // We just pick -90 .. +90 Degree
		x = PI_F - x;
	} else if ( x < -0.5f * PI_F ) {
		x = -PI_F - x;
	}
	const float x2 = x * x;
	return x + x * x2 * ( sinPolyCoef3 + x2 * ( sinPolyCoef5 + x2 * ( sinPolyCoef7 + x2 * sinPolyCoef9 ) ) );
}

float cos_approx( float x )
{
	return sin_approx( x + 0.5f * PI_F );
}


#include <inttypes.h>
uint32_t seed = 7;
uint32_t random( void)
{
  seed ^= seed << 13;
  seed ^= seed >> 17;
  seed ^= seed << 5;
  return seed;
}


// serial print routines
#ifdef SERIAL_ENABLE

extern void buffer_add(int val );
#include <stdlib.h>

// print a 32bit signed int
void print_int( int val )
{
#define SP_INT_BUFFERSIZE 12
char buffer2[SP_INT_BUFFERSIZE];

	if (val < 0)
	{
		buffer_add( (char) '-' );
		val = abs(val);
	}

int power = SP_INT_BUFFERSIZE;

do
{
	power--;
	int quotient = val/(10);
	int remainder = val-quotient*10;
	val = quotient;
	buffer2[power] = remainder+'0';
}
while (( val ) && power >=0) ;


	for (  ; power <= SP_INT_BUFFERSIZE-1 ; power++)
	{
		buffer_add(buffer2[power] );
	}
}

// print float with 2 decimal points
// this does not handle Nans inf and values over 32bit signed int
void print_float( float val )
{

	int ival = (int) val;

	if ( val < 0 && ival == 0 ) buffer_add( (char) '-' );

	print_int( ival );

	buffer_add( (char) '.' );

	val = val - (int) val;

	int decimals = val * 100;

	decimals = abs(decimals);

	if (decimals < 10) buffer_add( (char) '0' );
	print_int( decimals );

}

void print_str(const char *str)
{
	int count = 0;
	// a 64 character limit so we don't print the entire flash by mistake
	while (str[count]&&!(count>>6) )
	{
	buffer_add( (char) str[count] );
	count++;
	}
}

#endif

