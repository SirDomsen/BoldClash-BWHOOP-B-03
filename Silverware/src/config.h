
#include "defines.h"
#include "hardware.h"

// pids in pid.c

// rate in deg/sec
// for acro mode
#define MAX_RATE 1800.0
#define MAX_RATEYAW 1800.0
// use if your tx has no expo function
// 0.00 to 1.00 , 0 = no exp
// positive = less sensitive near center
#define ACRO_EXPO_XY 0.0
#define ACRO_EXPO_YAW 0.0



// max angle for level mode
#define LEVEL_MAX_ANGLE 80.0f
#define EXPO_XY 0.0
#define EXPO_YAW 0.0

#define LOW_RATES_MULTI 0.5f




// battery saver
// do not start software if battery is too low
// flashes 2 times repeatedly at startup
//#define STOP_LOWBATTERY

// voltage to start warning
// volts
#define VBATTLOW 3.6

// compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// calculate above factor automatically
#define AUTO_VDROP_FACTOR

// voltage hysteresis
// in volts
#define HYST 0.10



// lower throttle when battery below threshold
#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

#define PID_VOLTAGE_COMPENSATION


// Gyro LPF filter frequency
// gyro filter 0 = 250hz delay 0.97mS
// gyro filter 1 = 184hz delay 2.9mS
// gyro filter 2 = 92hz delay 3.9mS
// gyro filter 3 = 41hz delay 5.9mS (Default)
#define GYRO_LOW_PASS_FILTER 0


// software gyro lpf ( iir )
// set only one below
//#define SOFT_LPF_1ST_043HZ
//#define SOFT_LPF_1ST_100HZ
//#define SOFT_LPF_2ND_043HZ
//#define SOFT_LPF_2ND_088HZ
//#define SOFT_LPF_4TH_088HZ
//#define SOFT_LPF_4TH_160HZ
//#define SOFT_LPF_4TH_250HZ
// #define SOFT_LPF_1ST_HZ_BASE 80
// #define SOFT_LPF_1ST_HZ_MAX 300
// #define SOFT_LPF_1ST_HZ ( SOFT_LPF_1ST_HZ_BASE + 4 * throttle * ( SOFT_LPF_1ST_HZ_MAX - SOFT_LPF_1ST_HZ_BASE ) ) // * 4 .. MAX reached at 1/4 throttle
//#define SOFT_LPF_1ST_HZ 100
// #define SOFT_LPF_2ND_HZ
// #define SOFT_LPF_2ND_HZ_BASE 100 * ( aux[FN_INVERTED] ? 0.75f : 1.0f )
// #define SOFT_LPF_2ND_HZ_MAX 300 // A higher filter frequency than 333 causes ripples.
// #define SOFT_LPF_2ND_HZ_THROTTLE 0.25
//#define SOFT_KALMAN_GYRO KAL1_HZ_90
#define SOFT_BIQUAD_NOTCH_HZ 260
#define SOFT_BIQUAD_NOTCH_Q 6
// #define SOFT_LPF_NONE

// transmitter type
//#define USE_STOCK_TX
#define USE_DEVO
//#define USE_MULTI

// switch function selection

// H8 protocol channels
// CH_FLIP - flip,  CH_HEADFREE - headfree, CH_RTH - headingreturn
// CH_EXPERT , CH_INV (inv h101 tx)
// CH_RLL_TRIM , CH_PIT_TRIM - trim buttons pitch, roll
// CH_EMG , CH_TO - boldclash stock tx

// DEVO channels (bayang protocol)
// DEVO_CHAN_5 - DEVO_CHAN_10

// Multiprotocol can use MULTI_CHAN_5 - MULTI_CHAN_10  (bayang protocol)

// CH_ON - on always ( all protocols)
// CH_OFF - off always ( all protocols)

// CHAN_5 - CHAN_10 - auto based on tx selection

// rates / expert mode
#define RATES DEVO_CHAN_9

#define LEVELMODE DEVO_CHAN_10

#define STARTFLIP CH_OFF

#define LEDS_ON DEVO_CHAN_7

// switch for fpv / other, requires fet
// comment out to disable
//#define FPV_ON CH_ON

// Airmode keeps the PID loop stabilizing the quads orientation even at zero throttle.
// To stop the motors on ground a switch on the remote control is necessary.
#define AIRMODE_HOLD_SWITCH DEVO_CHAN_5

// enable inverted flight code ( brushless only )
#define INVERTED_ENABLE
#define FN_INVERTED DEVO_CHAN_6 //for brushless only
// #define LEVEL_MODE_INVERTED_ENABLE // be careful when enabling this

// aux1 channel starts on if this is defined, otherwise off.
// #define AUX1_START_ON

// automatically remove center bias ( needs throttle off for 1 second )
//#define STOCK_TX_AUTOCENTER

// enable motor filter - select one
// motorfilter1: hanning 3 sample fir filter
// motorfilter2: 1st lpf, 0.2 - 0.6 , 0.6 = less filtering
//#define MOTOR_FILTER
// #define MOTOR_FILTER2_ALPHA 0.3
//#define MOTOR_KAL KAL1_HZ_70
//#define MOTOR_KAL_2ND KAL1_HZ_90

// clip feedforward attempts to resolve issues that occur near full throttle
//#define CLIP_FF

// pwm frequency for motor control
// a higher frequency makes the motors more linear
// in Hz
#define PWMFREQ 24000

// motor curve to use
// the pwm frequency has to be set independently
// 720motors - use 8khz and curve none.
#define MOTOR_CURVE_NONE
//#define MOTOR_CURVE_6MM_490HZ
//#define MOTOR_CURVE_85MM_8KHZ
//#define MOTOR_CURVE_85MM_32KHZ
//#define BOLDCLASH_716MM_8K
// #define BOLDCLASH_716MM_24K

// a filter which makes throttle feel faster
#define THROTTLE_TRANSIENT_COMPENSATION
// if the quad resets , or for brushless ,try a lower value
#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 3.0

// For smoother motor reversing in 3D flight
#define THROTTLE_REVERSING_KICK 0.2f
#define THROTTLE_REVERSING_DEADTIME 20000 // 20 ms

// lost quad beeps using motors (30 sec timeout)
#define MOTOR_BEEPS

// throttle angle compensation in level mode
// comment out to disable
//#define AUTO_THROTTLE

// enable auto lower throttle near max throttle to keep control
// mix3 works better with brushless
// comment out to disable
//#define MIX_LOWER_THROTTLE
//#define MIX_INCREASE_THROTTLE

//#define MIX_LOWER_THROTTLE_3
//#define MIX_INCREASE_THROTTLE_3

// Betaflight like mix scaling (aka Airmode)
#define MIX_SCALING
// Mix increasing yields a more crisp response but also a more jumpy quad at low RPM
#define ALLOW_MIX_INCREASING
// A higher value means a shorter active increasing period (shorter bouncy period)
#define TRANSIENT_MIX_INCREASING_HZ 2.0
// Can be used to limit maximum motor RPM, i.e. tone down a too fast quad.
#define MIX_RANGE_LIMIT aux[ DEVO_CHAN_11 ] ? 0.75f : 1.0f

// Radio protocol selection
// select only one

#define RX_BAYANG_PROTOCOL_TELEMETRY
//#define RX_NRF24_BAYANG_TELEMETRY
//#define RX_BAYANG_BLE_APP
//#define RX_SBUS
//#define RX_SUMD

// 0 - 7 - power for telemetry
#define TX_POWER 7


// Flash saving features
#define DISABLE_FLIP_SEQUENCER
//#define DISABLE_GESTURES2

// led brightness in-flight ( solid lights only)
// 0- 15 range
#define LED_BRIGHTNESS 15


// external buzzer - pins in hardware.h
//#define BUZZER_ENABLE



// Comment out to disable pid tuning gestures
#define PID_GESTURE_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// flash save method
// flash_save 1: pids + accel calibration
// flash_save 2: accel calibration to option bytes
#define FLASH_SAVE1
//#define FLASH_SAVE2


// Rotate I-term vector for a stable yaw axis (aka iTerm Rotation)
#define PID_ROTATE_ERRORS

// Removes roll and pitch bounce back after flips (aka iTerm Relax)
#define TRANSIENT_WINDUP_PROTECTION
// Remove bounce back when quickly stopping a roll/pitch/yaw movement (but it is mostly there for yaw)
#define DYNAMIC_ITERM_RESET

// Feed fast roll/pitch-stick changes directly to the motors to give a snappier response
// 0.0f (or commented out) equates D-term on measurement, 1.0f equates D-term on error.
//#define FEED_FORWARD_STRENGTH 1.0f
//#define SMART_FF
// Feedforward for yaw. It's an absolute value, not related to the 0 .. 1 from above.
//#define FEED_FORWARD_YAW 0.2f

// Add linear interpolation between the otherwise 5 ms staircase steps of the RX signal
#define RX_SMOOTHING

// Use a square root motor curve to counteract thrust ~ RPM^2
// 0.0f .. no compensation, 1.0f .. full square root curve
#define THRUST_LINEARIZATION 0.33f


//##################################
// debug / other things
// this should not be usually changed


// level mode "manual" trims ( in degrees)
// pitch positive forward
// roll positive right
#define TRIM_PITCH 0.0
#define TRIM_ROLL 0.0

// disable motors for testing
//#define NOMOTORS

// throttle direct to motors for thrust measure
// #define MOTORS_TO_THROTTLE

// throttle direct to motors for thrust measure as a flight mode
#define MOTORS_TO_THROTTLE_MODE CH_AUX1


// loop time in uS
// this affects soft gyro lpf frequency if used
#define LOOPTIME 1000

// Failsafe time in us. Sets stick inputs to zero after FAILSAFETIME no RX signal. Keeps quad stabilized.
#define FAILSAFETIME 100000  // 0.1 seconds
// Motors failsafe time in us. Shuts motors down after additional MOTORS_FAILSAFETIME.
#define MOTORS_FAILSAFETIME 3000000 // 3 seconds

// max rate used by level pid ( limit )
#define LEVEL_MAX_RATE 1800

// invert yaw pid for hubsan motors
#define INVERT_YAW_PID

// debug things ( debug struct and other)
//#define DEBUG

// rxdebug structure
//#define RXDEBUG

// enable motors if pitch / roll controls off center (at zero throttle)
// possible values: 0 / 1
// use in acro build only
#define ENABLESTIX 0
#define ENABLESTIX_TRESHOLD 0.3
#define ENABLESTIX_TIMEOUT 1e6

// A deadband can be used to eliminate stick center jitter and non-returning to exactly 0.
#define STICKS_DEADBAND 0.02f

// overclock to 64Mhz
#define ENABLE_OVERCLOCK


// limit minimum motor output to a value (0.0 - 1.0)
//#define MOTOR_MIN_ENABLE
#define MOTOR_MIN_VALUE 0.05





#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260

//--fpmode=fast





// define logic - do not change
///////////////

// used for pwm calculations
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#endif



#ifdef MOTOR_BEEPS
#ifdef USE_ESC_DRIVER
#warning "MOTOR BEEPS_WORKS WITH BRUSHED MOTORS ONLY"
#endif
#endif



// for the ble beacon to work after in-flight reset
#ifdef RX_BAYANG_PROTOCOL_BLE_BEACON
#undef STOP_LOWBATTERY
#endif


// gcc warnings in main.c
