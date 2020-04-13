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


//#define RECTANGULAR_RULE_INTEGRAL
#define MIDPOINT_RULE_INTEGRAL
// #define SIMPSON_RULE_INTEGRAL


//#define NORMAL_DTERM
// #define NEW_DTERM
//#define MAX_FLAT_LPF_DIFF_DTERM
//#define DTERM_LPF_1ST_HZ 100
// #define  DTERM_LPF_2ND_HZ 50
#define  DYNAMIC_DTERM_LPF_2ND

#define AA_filterBaseFrequency ( 40 * ( aux[FN_INVERTED] ? 0.8f : 1.0f ) )
#define AA_filterMaxFrequency 60 // A higher filter frequency than 333 causes ripples.
#define AA_filterScaler 2 * throttle * ( AA_filterMaxFrequency - AA_filterBaseFrequency ) // Reaches AA_filterMaxFrequency at 1/2 throttle.

// aux_analog[0] -- aux_analog[1]

#define AA_pdScaleYawStabilizer 1.2f // multiply pdScaleValue by this value at full yaw
static float pdScaleValue; // updated in pid_precalc()

#define AA_pidkp ( x < 2 ? pdScaleValue * aux_analog[0] : 1.0f ) // Scale Kp and Kd only for roll and pitch.
#define AA_pidki 1.0f
#define AA_pidkd ( x < 2 ? pdScaleValue * aux_analog[1] : 1.0f ) // Scale Kp and Kd only for roll and pitch.

#define AA_pidScaleInverted 1.2f // multiply by this value when flying inverted

// if ( aux[DEVO_CHAN_11] ) { // 3S
// } else { // 4S
// }

// if ( aux[DEVO_CHAN_8]) { // PID2
// } else { // PID1
// }

// if ( aux[LEDS_ON]) { // ON
// } else { // OFF
// }


//#define ANTI_WINDUP_DISABLE

#include <stdbool.h>
#include <stdlib.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "led.h"
#include "defines.h"
#include <math.h>

float pidkp[PIDNUMBER] = { 0.04, 0.04, 0.04 };
float pidki[PIDNUMBER] = { 0.40, 0.40, 2.0 };
float pidkd[PIDNUMBER] = { 0.20, 0.20, 0.0 };

// "setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
// #define ENABLE_SETPOINT_WEIGHTING
float b[3] = { 1.0 , 1.0 , 1.0};




// output limit
const float outlimit[PIDNUMBER] = { 0.5, 0.5, 0.25 };

// limit of integral term (abs)
const float integrallimit[PIDNUMBER] = { 0.1, 0.1, 0.25 };


// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f H101
#define PID_VC_FACTOR 1.33f


// non changable things below
float * pids_array[3] = {pidkp, pidki, pidkd};
int number_of_increments[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
int current_pid_axis = 0;
int current_pid_term = 0;
float * current_pid_term_pointer = pidkp;

float ierror[PIDNUMBER] = { 0 , 0 , 0};
float pidoutput[PIDNUMBER];
float setpoint[PIDNUMBER];

static float lasterror[PIDNUMBER];
extern float error[PIDNUMBER];
extern float looptime;
extern float gyro[3];
extern float gyro_unfiltered[3];
extern int onground;
extern float looptime;
// extern float vbattfilt;
extern float vbatt_comp;
extern float battery_scale_factor;
extern float rxcopy[];
extern float aux_analog[];
extern float mixmax;
extern char aux[AUXNUMBER];

#ifdef NORMAL_DTERM
static float lastrate[PIDNUMBER];
#endif

#ifdef NEW_DTERM
static float lastratexx[PIDNUMBER][2];
#endif

#ifdef MAX_FLAT_LPF_DIFF_DTERM
static float lastratexx[PIDNUMBER][4];
#endif

#ifdef SIMPSON_RULE_INTEGRAL
static float lasterror2[PIDNUMBER];
#endif

float timefactor;
float v_compensation = 1.00;

// pid calculation for acro ( rate ) mode
// input: error[x] = setpoint - gyro
// output: pidoutput[x] = change required from motors
float pid(int x )
{
    const float out_limit = outlimit[x] * v_compensation * battery_scale_factor;

    if (onground)
    {
    ierror[x] *= 0.98f;
    }


    int iwindup = 0;
    if (( pidoutput[x] >= out_limit )&& ( error[x] > 0) )
    {
        iwindup = 1;
    }

    if (( pidoutput[x] <= -out_limit)&& ( error[x] < 0) )
    {
        iwindup = 1;
    }


    #ifdef ANTI_WINDUP_DISABLE
    iwindup = 0;
    #endif

#ifdef TRANSIENT_WINDUP_PROTECTION
    static float avgSetpoint[ 2 ];
    if ( x < 2 ) { // Only for roll and pitch.
        lpf( &avgSetpoint[ x ], setpoint[ x ], FILTERCALC( LOOPTIME, 1e6f / 20.0f ) ); // 20 Hz
        const float hpfSetpoint = setpoint[ x ] - avgSetpoint[ x ]; // HPF = input - average_input
        if ( fabsf( hpfSetpoint ) > 0.1f ) { // 5.7 °/s
            iwindup = 1;
        }
    }
#endif

#ifdef DYNAMIC_ITERM_RESET
    static float lastGyro[ 3 ];
    if ( fabsf( ierror[ x ] ) > integrallimit[ x ] * 0.2f * battery_scale_factor && ( gyro[ x ] < 0.0f != lastGyro[ x ] < 0.0f ) ) { // gyro crossed zero
        ierror[ x ] *= 0.2f;
    }
    lastGyro[ x ] = gyro[ x ];
#endif // DYNAMIC_ITERM_RESET

    if ( !iwindup)
    {
        #ifdef MIDPOINT_RULE_INTEGRAL
         // trapezoidal rule instead of rectangular
        ierror[x] = ierror[x] + (error[x] + lasterror[x]) * 0.5f *  pidki[x] * looptime * AA_pidki;
        lasterror[x] = error[x];
        #endif

        #ifdef RECTANGULAR_RULE_INTEGRAL
        ierror[x] = ierror[x] + error[x] *  pidki[x] * looptime * AA_pidki;
        lasterror[x] = error[x];
        #endif

        #ifdef SIMPSON_RULE_INTEGRAL
        // assuming similar time intervals
        ierror[x] = ierror[x] + 0.166666f* (lasterror2[x] + 4*lasterror[x] + error[x]) *  pidki[x] * looptime * AA_pidki;
        lasterror2[x] = lasterror[x];
        lasterror[x] = error[x];
        #endif
    }

    limitf( &ierror[x] , integrallimit[x] * battery_scale_factor );


    #ifdef ENABLE_SETPOINT_WEIGHTING
    // P term
    pidoutput[x] = error[x] * ( b[x])* pidkp[x] * AA_pidkp;
    // b
    pidoutput[x] +=  - ( 1.0f - b[x])* pidkp[x] * gyro[x];
    #else
    // P term with b disabled
    pidoutput[x] = error[x] * pidkp[x] * AA_pidkp;
    #endif

	if ( x < 2 ) { // Only for roll and pitch.

// https://www.rcgroups.com/forums/showpost.php?p=39606684&postcount=13846
// https://www.rcgroups.com/forums/showpost.php?p=39667667&postcount=13956
#ifdef FEED_FORWARD_STRENGTH

#ifdef RX_SMOOTHING
		static float lastSetpoint[2];
		float ff = ( setpoint[x] - lastSetpoint[x] ) * timefactor * FEED_FORWARD_STRENGTH * pidkd[x] * AA_pidkd;
		lastSetpoint[x] = setpoint[x];
#else
		static float lastSetpoint[2];
		static float setpointDiff[2];
		static int ffCount[2];
		if ( setpoint[x] != lastSetpoint[x] ) {
			setpointDiff[x] = ( setpoint[x] - lastSetpoint[x] ) / 5; // Spread it evenly over 5 ms (PACKET_PERIOD)
			ffCount[x] = 5;
			lastSetpoint[x] = setpoint[x];
		}

		float ff = 0.0f;
		if ( ffCount[x] > 0 ) {
			--ffCount[x];
			ff = setpointDiff[x] * timefactor * FEED_FORWARD_STRENGTH * pidkd[x] * AA_pidkd;
		}
#endif // RX_SMOOTHING

		// // 16 point moving average filter to smooth out the 5 ms steps:
		// #define MA_SIZE ( 1 << 4 ) // power of two
		// static float ma_value[2];
		// static float ma_array[2][ MA_SIZE ];
		// static uint8_t ma_index[2];
		// ma_value[x] -= ma_array[x][ ma_index[x] ];
		// ma_value[x] += ff;
		// ma_array[x][ ma_index[x] ] = ff;
		// ++ma_index[x];
		// ma_index[x] &= MA_SIZE - 1;
		// ff = ma_value[x] / MA_SIZE; // dividing by a power of two is handled efficiently by the compiler (__ARM_scalbnf)

		// Attenuate FF towards stick center position (aka FF Transition):
		const float absStickDeflection = fabsf( rxcopy[ x ] );
#if 0
		const float scalingValue = 0.0f; // Scale FF by that much at stick center.
		const float scalingBreakpoint = 0.5f * AA_ffScalingBreakpoint; // No scaling when the stick deflection is above that value.
		if ( absStickDeflection < scalingBreakpoint ) {
			ff *= 1.0f + ( 1.0f - absStickDeflection / scalingBreakpoint ) * ( scalingValue - 1.0f );
		}
#else
		ff *= absStickDeflection; // Scale linearly from center to max deflection.
#endif

#ifdef SMART_FF
		if ( ff < 0.0f == pidoutput[x] < 0.0f ) {
			if ( fabsf( ff ) > fabsf( pidoutput[x] ) ) {
				pidoutput[x] = ff; // Take the larger of P or FF as long as P and FF have the same sign.
			}
		} else {
			pidoutput[x] += ff; // Always add FF if the signs are opposite.
		}
#else
		pidoutput[x] += ff;
#endif // SMART_FF

#endif // FEED_FORWARD_STRENGTH

    } else { // x == 2: yaw

#ifdef FEED_FORWARD_YAW
    #ifndef RX_SMOOTHING
        #error "FEED_FORWARD_YAW only works with RX_SMOOTHING enabled."
    #endif
        static float lastSetpointYaw;
        float ff = ( setpoint[ x ] - lastSetpointYaw ) * timefactor * FEED_FORWARD_YAW;
        lastSetpointYaw = setpoint[ x ];

        static float avgFFyaw;
        lpf( &avgFFyaw, ff, FILTERCALC( 0.001, 1.0f / 20.0f ) ); // 20 Hz
        ff = avgFFyaw;

        pidoutput[ x ] += ff;
#endif // FEED_FORWARD_YAW

    }

    // I term
    pidoutput[x] += ierror[x];

    // D term
    // skip yaw D term if not set
    if ( pidkd[x] > 0 )
    {
        #ifdef NORMAL_DTERM
        pidoutput[x] = pidoutput[x] - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor  ;
        lastrate[x] = gyro[x];
        #endif

        #ifdef NEW_DTERM
        pidoutput[x] = pidoutput[x] - ( ( 0.5f) *gyro[x]
                    - (0.5f) * lastratexx[x][1] ) * pidkd[x] * timefactor  ;

        lastratexx[x][1] = lastratexx[x][0];
        lastratexx[x][0] = gyro[x];
        #endif

        #ifdef MAX_FLAT_LPF_DIFF_DTERM
        pidoutput[x] = pidoutput[x] - ( + 0.125f *gyro[x] + 0.250f * lastratexx[x][0]
                    - 0.250f * lastratexx[x][2] - ( 0.125f) * lastratexx[x][3]) * pidkd[x] * timefactor 						;

        lastratexx[x][3] = lastratexx[x][2];
        lastratexx[x][2] = lastratexx[x][1];
        lastratexx[x][1] = lastratexx[x][0];
        lastratexx[x][0] = gyro[x];
        #endif


        #ifdef DTERM_LPF_1ST_HZ
        float dterm;
        static float lastrate[3];
        static float dlpf[3] = {0};

        dterm = - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor;
        lastrate[x] = gyro[x];

        lpf( &dlpf[x], dterm, FILTERCALC( 0.001 , 1.0f/DTERM_LPF_1ST_HZ ) );

        pidoutput[x] += dlpf[x];
        #endif

        #if defined DTERM_LPF_2ND_HZ || defined DYNAMIC_DTERM_LPF_2ND
        float dterm;
        static float lastrate[3];
        float lpf2( float in, int num);
        if ( pidkd[x] > 0)
        {
#if 0 // CASCADE_GYRO_AND_DTERM_FILTER
            dterm = - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor * AA_pidkd;
            lastrate[x] = gyro[x];
#else
            dterm = - (gyro_unfiltered[x] - lastrate[x]) * pidkd[x] * timefactor * AA_pidkd;
            lastrate[x] = gyro_unfiltered[x];
#endif
            dterm = lpf2(  dterm, x );
            pidoutput[x] += dterm;
        }
        #endif
    }

#ifdef PID_VOLTAGE_COMPENSATION
    pidoutput[x] *= v_compensation;
#endif

    pidoutput[x] *= battery_scale_factor;

    if ( aux[FN_INVERTED] ) {
        pidoutput[x] *= AA_pidScaleInverted;
    }

    limitf(  &pidoutput[x] , out_limit);

return pidoutput[x];
}

// calculate change from ideal loop time
// 0.0032f is there for legacy purposes, should be 0.001f = looptime
// this is called in advance as an optimization because it has division
void pid_precalc()
{
	timefactor = 0.0032f / looptime;

#ifdef PID_VOLTAGE_COMPENSATION
    // v_compensation = mapf ( vbattfilt , 3.00 , 4.00 , PID_VC_FACTOR , 1.00);
    v_compensation = mapf ( vbatt_comp , 3.00 , 4.00 , PID_VC_FACTOR , 1.00);
    if( v_compensation > PID_VC_FACTOR) v_compensation = PID_VC_FACTOR;
    if( v_compensation < 1.00f) v_compensation = 1.00;
#endif

    pdScaleValue = 1.0f; // constant (no throttle dependent scaling)

#ifdef AA_pdScaleYawStabilizer
	const float absyaw = fabsf( rxcopy[2] );
	// const float absyaw = fabsf( gyro[2] / ( (float)MAX_RATEYAW * DEGTORAD ) );
	pdScaleValue *= 1 + absyaw * ( AA_pdScaleYawStabilizer - 1 ); // Increase Kp and Kd on high yaw speeds to avoid iTerm Rotation related wobbles.
#endif

}


// i vector rotation by joelucid
// https://www.rcgroups.com/forums/showpost.php?p=39354943&postcount=13468

void rotateErrors()
{
	// // rotation around x axis:
	// float temp = gyro[0] * looptime;
	// ierror[1] -= ierror[2] * temp;
	// ierror[2] += ierror[1] * temp;

	// // rotation around y axis:
	// temp = gyro[1] * looptime;
	// ierror[2] -= ierror[0] * temp;
	// ierror[0] += ierror[2] * temp;

	// // rotation around z axis:
	// temp = gyro[2] * looptime;
	// ierror[0] -= ierror[1] * temp;
	// ierror[1] += ierror[0] * temp;

	// ---

	// http://fooplot.com/#W3sidHlwZSI6MCwiZXEiOiJ4KjAuMDMxIiwiY29sb3IiOiIjODA4MDgwIn0seyJ0eXBlIjoyLCJlcXgiOiJjb3MocykiLCJlcXkiOiJzaW4ocykiLCJjb2xvciI6IiMwMDAwMDAiLCJzbWluIjoiMCIsInNtYXgiOiIwLjAzMSIsInNzdGVwIjoiLjAwMDEifSx7InR5cGUiOjIsImVxeCI6IjEiLCJlcXkiOiJzIiwiY29sb3IiOiIjZmYwMDAwIiwic21pbiI6IjAiLCJzbWF4IjoiMC4wMzEiLCJzc3RlcCI6Ii4wMDAxIn0seyJ0eXBlIjoyLCJlcXgiOiIxLXMqcy8yIiwiZXF5IjoicyIsImNvbG9yIjoiI0ZGMDBGRiIsInNtaW4iOiIwIiwic21heCI6IjAuMDMxIiwic3N0ZXAiOiIuMDAwMSJ9LHsidHlwZSI6MTAwMCwid2luZG93IjpbIjAuOTg0IiwiMS4wMTYiLCIwIiwiMC4wMzIiXSwic2l6ZSI6WzY1MCw2NTBdfV0-
	// yaw angle (at 1800?/s -> a2 = 1800/180*3.1416*0.001 = 0.031)
	const float a2 = gyro[2] * looptime;
	// small angle approximations
	const float COS_a2 = 1 - a2*a2/2;
	const float SIN_a2 = a2; // Adding -a2*a2*a2/6 yields no further improvement.
	const float ierror0 = ierror[0] * COS_a2 - ierror[1] * SIN_a2;
	const float ierror1 = ierror[1] * COS_a2 + ierror[0] * SIN_a2;
	ierror[0] = ierror0;
	ierror[1] = ierror1;
}


#if defined DTERM_LPF_2ND_HZ

//the compiler calculates these
static float two_one_minus_alpha = 2*FILTERCALC( 0.001 , (1.0f/DTERM_LPF_2ND_HZ) );
static float one_minus_alpha_sqr = (FILTERCALC( 0.001 , (1.0f/DTERM_LPF_2ND_HZ) ) )*(FILTERCALC( 0.001 , (1.0f/DTERM_LPF_2ND_HZ) ));
static float alpha_sqr = (1 - FILTERCALC( 0.001 , (1.0f/DTERM_LPF_2ND_HZ) ))*(1 - FILTERCALC( 0.001 , (1.0f/DTERM_LPF_2ND_HZ) ));

static float last_out[3], last_out2[3];
static int holdoff[3];

float lpf2( float in, int num)
 {
  if ( holdoff[num] > 0 ) {
    --holdoff[num];
    return 0.0;
  }

  float ans = in * alpha_sqr + two_one_minus_alpha * last_out[num]
      - one_minus_alpha_sqr * last_out2[num];

  last_out2[num] = last_out[num];
  last_out[num] = ans;

  return ans;
 }

#elif defined DYNAMIC_DTERM_LPF_2ND

static float last_out[3], last_out2[3];
static int holdoff[3];

float lpf2( float in, int num )
{
    if ( holdoff[num] > 0 ) {
        --holdoff[num];
        return 0.0;
    }

	const float throttle = rxcopy[3];

    static float two_one_minus_alpha;
    static float one_minus_alpha_sqr;
    static float alpha_sqr;
    if ( num == 0 ) {
        float filterHz = AA_filterBaseFrequency + AA_filterScaler;
        if ( filterHz > AA_filterMaxFrequency ) {
            filterHz = AA_filterMaxFrequency;
        } else if ( filterHz < AA_filterBaseFrequency ) {
            filterHz = AA_filterBaseFrequency;
        }
        const float one_minus_alpha = FILTERCALC( 0.001, 1.0f / filterHz );
        two_one_minus_alpha = 2 * one_minus_alpha;
        one_minus_alpha_sqr = one_minus_alpha * one_minus_alpha;
        alpha_sqr = ( 1 - one_minus_alpha ) * ( 1 - one_minus_alpha );
    }
    const float ans = in * alpha_sqr + two_one_minus_alpha * last_out[num] - one_minus_alpha_sqr * last_out2[num];

    last_out2[num] = last_out[num];
    last_out[num] = ans;

    return ans;
}

#endif

#if defined DTERM_LPF_2ND_HZ || defined DYNAMIC_DTERM_LPF_2ND
void lpf2_reset()
{
	last_out[0] = last_out[1] = last_out[2] = 0.0f;
	last_out2[0] = last_out2[1] = last_out2[2] = 0.0f;
    holdoff[0] = holdoff[1] = holdoff[2] = 0; // ms
}
#else
void lpf2_reset()
{
}
#endif

// below are functions used with gestures for changing pids by a percentage

// Cycle through P / I / D - The initial value is P
// The return value is the currently selected TERM (after setting the next one)
// 1: P
// 2: I
// 3: D
// The return value is used to blink the leds in main.c
int next_pid_term()
{
//	current_pid_axis = 0;

	switch (current_pid_term)
	{
		case 0:
			current_pid_term_pointer = pidki;
			current_pid_term = 1;
			break;
		case 1:
			if ( pidkd[ current_pid_axis ] == 0.0f ) { // Skip a zero D term, and go directly to P
				current_pid_term_pointer = pidkp;
				current_pid_term = 0;
			} else {
				current_pid_term_pointer = pidkd;
				current_pid_term = 2;
			}
			break;
		case 2:
			current_pid_term_pointer = pidkp;
			current_pid_term = 0;
			break;
	}

	return current_pid_term + 1;
}

// Cycle through the axis - Initial is Roll
// Return value is the selected axis, after setting the next one.
// 1: Roll
// 2: Pitch
// 3: Yaw
// The return value is used to blink the leds in main.c
int next_pid_axis()
{
	const int size = 3;
	if (current_pid_axis == size - 1) {
		current_pid_axis = 0;
	}
	else {
		#ifdef COMBINE_PITCH_ROLL_PID_TUNING
		if (current_pid_axis <2 ) {
			// Skip axis == 1 which is roll, and go directly to 2 (Yaw)
			current_pid_axis = 2;
		}
		#else
		current_pid_axis++;
		#endif
	}

	return current_pid_axis + 1;
}

#define PID_GESTURES_MULTI 1.1f

int change_pid_value(int increase)
{
	float multiplier = 1.0f/(float)PID_GESTURES_MULTI;
	if (increase) {
		multiplier = (float)PID_GESTURES_MULTI;
		number_of_increments[current_pid_term][current_pid_axis]++;
	}
	else {
		number_of_increments[current_pid_term][current_pid_axis]--;
	}

	current_pid_term_pointer[current_pid_axis] = current_pid_term_pointer[current_pid_axis] * multiplier;

    #ifdef COMBINE_PITCH_ROLL_PID_TUNING
	if (current_pid_axis == 0) {
		current_pid_term_pointer[current_pid_axis+1] = current_pid_term_pointer[current_pid_axis+1] * multiplier;
	}
	#endif

	return abs(number_of_increments[current_pid_term][current_pid_axis]);
}

// Increase currently selected term, for the currently selected axis, (by functions above) by 10%
// The return value, is absolute number of times the specific term/axis was increased or decreased.  For example, if P for Roll was increased by 10% twice,
// And then reduced by 10% 3 times, the return value would be 1  -  The user has to rememeber he has eventually reduced the by 10% and not increased by 10%
// I guess this can be improved by using the red leds for increments and blue leds for decrements or something, or just rely on SilverVISE
int increase_pid()
{
	return change_pid_value(1);
}

// Same as increase_pid but... you guessed it... decrease!
int decrease_pid()
{
	return change_pid_value(0);
}


