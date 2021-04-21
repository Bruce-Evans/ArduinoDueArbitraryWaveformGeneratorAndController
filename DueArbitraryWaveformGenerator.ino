// Due Arbitrary Waveform Generator by Bruce Evans 2017. (Some code adapted from the Magician, Kerry D. Wong, Mark T, MartinL, ard_newie and possibly others. Many thanks!)
// Mostly, Capitals have been used for global variables and lower case for local variables. Any constants have ALL capitals. Although unconventional, I've personally found this more helpful.
//
// To use:
// 1) Upload this sketch to Arduino Due.
// 2) If additional board protection desired then connect pins 3 & 7 first to a resistor (approx 47 to 100 ohms) then to circuit.
// 3) The synchronised square wave output is always taken from pin 3. The UNsyncronised square wave output is taken from pin 7, except below 163Hz when it's taken from pin 3 to maximize wave specifications.
// 4) The analogue wave output is at DAC0 (This may be marked as DAC2 on some Arduino Due boards).
//
// See:  https://create.arduino.cc/projecthub/BruceEvans/arduino-due-arbitrary-waveform-generator-a9d180?f=1 and https://github.com/Bruce-Evans/ArduinoDueArbitraryWaveformGeneratorAndController/releases
// For more background see: https://makezine.com/projects/make-35/advanced-arduino-sound-synthesis/
//
// User Defined Equation code modifications developed by Michael Szoke 2021 which builds upon and increases functionality.
// Type "x" in Serial Monitor to access Extra commands for each wave shape.
// See Github fork:  https://github.com/mszoke01/ArduinoDueArbitraryWaveformGeneratorAndController
//
//
// Use the accompanying separate program to control the Arduino from your PC. See its help file (in its data folder) for more info.
//
// or alternatively...
// If you want to use 2 pots to control frequency & duty cycle, use any value pots between 1k & 50k, and connect them between 0V & 3.3V (NOT 5V!) with their wipers connected to A0 & A1.
//  If you want to use switches to control pot operation:
//    Connect a momentary switch between the following pins: (no pull-up resistors required)
//      pin 22 & ground - toggle: enable / disable pots (& LEDs)
//      pin 24 & ground - toggle: Unsync'ed square wave: pot FREQ or PERIOD
//      pin 26 & ground - cycle thru: Unsync'ed square wave: pot freq / period RANGE
//      pin 28 & ground - toggle: Sync'ed waves: pot FREQ or PERIOD
//      pin 30 & ground - cycle thru: Sync'ed waves: pot freq / period RANGE
//      pin 32 & ground - toggle: Unsync'ed square wave: pot DUTY CYCLE or PULSE WIDTH
//      pin 34 & ground - cycle thru: Unsync'ed square wave: pot pulse width RANGE (duty cycle range is fixed at 0 - 100%)
//      pin 36 & ground - toggle: Sync'ed waves: pot DUTY CYCLE or PULSE WIDTH
//      pin 38 & ground - cycle thru: Sync'ed waves: pot pulse width RANGE (duty cycle range is fixed at 0 - 100%)
//      pin 40 & ground - cycle thru: WAVE SHAPE
//      pin 42 & ground - toggle: Exact Freq Mode ON / OFF
//      pin 44 & ground - toggle: Square Wave Sync ON / OFF
//      pin 46 & ground - cycle thru: Which wave/s to control
//  If you want to use LEDs to display pot operation: (not range or wave-shape)
//    Connect a single LED or a dual colour LED or 2 back to back LEDs (in parallel) via a single resistor (approx 560 ohms) between the following pins:
//      pins 23 & ground - indicates pots enabled (use a single LED only)
//      pins 25 & 27 - indicates: Unsync'ed square wave: pot FREQ or PERIOD
//      pins 29 & 31 - indicates: Sync'ed wave: pot FREQ or PERIOD
//      pins 33 & 35 - indicates: Unsync'ed square wave: pot DUTY CYCLE or PULSE WIDTH
//      pins 37 & 39 - indicates: Sync'ed wave: pot DUTY CYCLE or PULSE WIDTH
//      pins 41 & ground - indicates: Exact Freq Mode ON (use a single LED only) (pin 43 is low current, so not used here)
//      pins 45 & ground - indicates: Square Wave Sync ON (use a single LED only)
//      pins 47 & ground - indicates: Analogue wave is being controlled (use a single LED only)
//      pins 49 & ground - indicates: Unsynch'ed Square wave is being controlled (use a single LED only)
//  Note: Sweep and Timer functions are not controllable by the pots & switches.
//  Also note that when pots are enabled, the Arduino will accept commands from the pots when touched AND from the PC via USB when a signal is received.
//
// or alternatively...
// TO CONTROL FROM THE ARDUINO SERIAL MONITOR:
// Set your serial monitor to 115200 baud.
// Type the following then press enter (or Send)
// Type:   a   to create a new Arbitrary wave. Enter the value (between 0 and 4095) for each waypoint. Up to 4096 points. Stepped points can also be easily entered. On-screen instructions will be given. The points will be evenly spread over the period of the wave.
// Type:   w   to toggle the analogue Wave shape between sine, triangle and arbitrary
// Type:   v   to toggle between Viewing synchronized or unsynchronized square wave                       } to toggle view and control simultaneously:
// Type:  ' '  [spacebar] to toggle between controlling synchronized waves or unsynchronized square wave  } both 'v' & ' ' can be typed (followed by enter)
// Type:   b   to control Both synchronized waves and unsynchronized square wave simultaneously
// Type:   h   to change frequency of wave/s, type required frequency in Hz followed by h. The freq range is: (synchronized waves limit: 0.00005Hz - 100kHz) (unsynchronized square wave limit: 0.00005Hz - 42MHz) via serial cconnection, or slightly less using pots
// Type:   m   to change period of wave/s, type required period in Milliseconds followed by m (synchronized waves limit: 5hrs 33mins 20secs or 20,0000 Secs - 10 MicroSecs) (unsynchronized square wave limit: 10,737Secs - 23.8 nanoSecs)
// Type:   d   to change Duty-cycle type required percentage duty-cycle (0 - 100) followed by d.
// Type:   u   to set pulse width. Type required pulse width in µseconds followed by u. PULSE WIDTH WILL REMAIN FIXED until duty-cycle (above) is set instead.
// Type:   e   to toggle Exact Freq Mode on/off (synchronized waves only) eliminating freq steps, but has lower sample rate, & dithering on synchronized sq. wave & sharp edges (view on oscilliscope with HF filter on)
// Type:   s   to set up the frequency Sweep feature. Follow on-screen instructions.
// Type:   t   to enter or exit the Timer. Follow on-screen instructions.
// Type:   p   to enable or disable the pots
// Type:   P   to toggle between pot controlling duty-cycle Percent, or Pulse width of wave. Synchronized and unsynchronized waves can be set independently by using the ' ' or 'b' command shown above
// Type:   f   to toggle between pot controlling Freq of wave, or period of wave. Synchronized and unsynchronized waves can be set independently by using the ' ' or 'b' command shown above
// Type:   r   to toggle frequency/period Range of the pots: x 1, x10, x100, x1000 & x 10000 (synchronized & unsynchronized ranges are adjustable independently by using the ' ' or 'b' command shown above)
// Type:   R   to toggle pulse width Range of the pot. (does not affect duty-cycle percent): x 1, x10, x100, x1000 & x 10000."); (synchronized & unsynchronized ranges are adjustable independently by using the ' ' or 'b' command shown above)
// Type:   ?   to display the help screen. (shortened version of the above)
// Type:   x   to access Extra commands for each wave type. Close and re-open Serial Monitor to reset all wave variables to default values.

/********************************************************/
// Do not adjust if working okay: Type number before delimiter (if altered, will return to default unless variables: MinMaxDuty, Delay1, 2 & 3 are altered accordingly)
// Type:   M   for setup only. Min/Max duty-cycle percentage, expressed in samples (when synchronized & above 1kHz with Exact Freq Mode off) - will return to default (4) when switching from unsynchronized to synchronized square wave (by typing 'v') [settings below 4 cause polarity reversal due to DMA timing]
// Type:   H   for setup only. delay of square wave for best phase synchronization at High sample rate. ie: 10kHz, 20kHz, 40khz & 100kHz [delay (high or low) can't be adjusted below 1.01kHz] - default is 10
// Type:   L   for setup only. delay of square wave for best phase synchronization at Low sample rate. ie: 1.1kHz, 11kHz, 21kHz, & 41kHz (adj high sample rate delay 1st) - default is 36
// Type:   D   for setup only. delay of square wave for best phase synchronization at Low duty-cycle, but not 0%. - default is 110

/********************************************************/

// Common to all Waveforms:
bool UsingGUI;  //  1 = using the Arbitrary Waveform GUI on the PC
bool PotsEnabled = 0; //  <<<<<<<<<<<<<<<<<<<<<<<< CHANGE TO 1 TO START-UP WITH POTS ENABLED <<<<<<<<<<<<<<<<<<<<<<
bool PotPulseWidth[3];   // enable / disable pulse width input from pot instead of duty-cycle % {unsynchronized wave, synchronized waves}
// Range of pots:
int   Range[]  = {1, 100, 1, 1}; // {unsynchronized wave freq/period, synchronized waves freq/period, unsynchronized wave Pulse Width Range, synchronized waves Pulse Width Range}
// Unsynchronized ^        // if PotPeriodMode[0] = 1: Range x1 = low period (24.0 nS - 40.9 µS), x10 = per. x 10, x100 = per. x 100, x1000 = per. x 1000, x10000 = per. x 10000 (up to 409.6 mS)
// Unsynchronized ^        // if PotPeriodMode[0] = 0: Range x1 = low freq   (1.26 Hz - 4.0 kHz), x10 = freq x 10, x100 = freq x 100, x1000 = freq x 1000, x10000 = freq x 10000 (up to 42 MHz)
// Sine/triangle wave  ^   // if PotPeriodMode[1] = 0: Range x1 = low freq   (10 mHz - 40.9  Hz), x10 = freq x 10, x100 = freq x 100, x1000 = freq x 1000, x10000 = freq x 10000 (up to 100 kHz)
// Sine/triangle wave  ^   // if PotPeriodMode[1] = 1: Range x1 = low period  (10 µS - 409.6 µS), x10 = per. x 10, x100 = per. x 100, x1000 = per. x 1000, x10000 = per. x 10000 (up to 4.096 Secs)
volatile boolean SquareWaveSync = LOW; // HIGH;
byte  Control  = 2;        // control of synchronized waves or unsynchronized square wave: 0 = unsync'ed, 1 = sync'ed, 2 = both
int   Pot0     = 1000;     // reading from freq pot <<<<<<<<<<<<<<<<<<<<<< DETERMINES START-UP FREQUENCY, even though pots may be disabled <<<<<<<<<<<<<<<<<<<<<
int   Pot1     = 2000;     // reading from duty-cycle pot
float DutyReading[] = {50, 50}; // duty-cycle reading from the pot {unsynchronized wave, synchronized waves}
float OldReading[3];       // freq/period reading from the pot {unsynchronized wave, synchronized waves}
bool  PotPeriodMode[3];    // {unsynchronized wave, synchronized waves}: 0 = pot adjusts freq of wave, 1 = pot adjusts period of wave
unsigned long SwitchPressedTime;    // for debouncing pot switches
unsigned long LEDupdateTime;        // update pot function LED indicators every 300 mSecs (not every cycle of the loop)
double UserInput       = 0;         // Numbers read from serial connection
char   UserChars[5] = "    ";        // 1st serial character following UserInput number (above) read into UserChars[0]. If that character was 'x' and more serial characters available they are read into rest of array
//int   numDecimalPlaces = -1;        // counts number of input digits read from serial connection after '.' detected. Equals -1 until '.' detected
unsigned long TouchedTime = 0; // detects when enter pressed twice within 500 mSecs for triggering Status message
byte  SweepStep;                    // 0 = Sweep off, 1 =  Set SweepMinFreq, 2 = Set SweepMaxFreq, 3 = Set SweepRiseTime, 4 = Set SweepFallTime, 5 = Run
float SweepMinFreq;
float SweepMaxFreq;
int   SweepRiseTime;
int   SweepFallTime;
/********************************************************/
// For the Unsynchronized Square Wave:
uint32_t clkAFreq = 42000000ul;   // system clock 1
//uint32_t pwmFreq  = 42000000ul; // enabled later in program if freq is 1300Hz min (lowest poss is 1281)  - min (duty-cycle) pulse width:  12nSec
uint32_t pwmFreq  = 10500000ul;   // enabled later in program if freq is  650Hz min                (~641)  - min (duty-cycle) pulse width:  24nSec
//uint32_t pwmFreq  = 2;          // enabled later in program if freq is  325Hz min                (~321)  - min (duty-cycle) pulse width:  48nSec
//uint32_t pwmFreq  = 4;          // enabled later in program if freq is  163Hz min                (~161)  - min (duty-cycle) pulse width:  96nSec
volatile uint32_t PulsePeriod[] = {1010, 2020}; // {Pulse, Period}  -  if freq is < 163Hz, used by interupt handler - min (duty-cycle) pulse width: 96nSec
bool     PeriodHalf;     // identifies which half -  if freq is < 163Hz, used by interupt handler
int      uPeriodMultiplier = 2;   // for calculating microSec period at different pwm freqs
double   Period      = 42000;     // period in 'timer counts' of 24 nSecs per count, so 42,000 = 1 mSec or 1 kHz
double   Pulse       = 21000;     // pulse width in 'timer counts' of 24 nSecs per count
int      FreqReading = 1000;      // unsynchronized square wave freq (reading from pot)
double   TargetFreq  = 1000;      // unsynchronized square wave target freq
double   ActualFreq  = 1000;      // unsynchronized square wave calculated actual freq
float    TargetDuty  = 50;        // unsynchronized square wave target duty-cycle
float    ActualDuty  = 50;       // unsynchronized square wave actual duty-cycle
bool     MinOrMaxDuty;          // if duty-cycle is 0% or 100% only 1 half of unsynchronized square wave is displayed - if freq is < 163Hz, used by interupt handler
float    TargetPulseWidth;     // unsynchronized square wave target pulse width in microsecs (0 if not set)
float    uPulseWidth;         // pulse width calculated in microSeconds
float    uPeriod;            // period calculated in microSeconds
byte     TimerMode;         // time period can be entered - in days, hours, minutes, seconds (max period is 4294967296 days, or 11,767,033.6 years!) - if freq is < 163Hz, used by interupt handler
bool     TimerInvert;      // sets timer ouput neg instead of pos
bool     TimerRun;        // reset or start timer running
bool     TimeUp;         // output goes positive when time entered has passed
uint32_t TimeIncrement; // 200kHz, 5 microseconds clock rate - if freq is < 163Hz, used by interupt handler
byte     TimerSecs;    // time that has passed since resetting timer
byte     TimerMins;   // as above
byte     TimerHours;
uint32_t TimerDays;
byte     PeriodS;  // seconds - Target time period
byte     PeriodM; // minutes
byte     PeriodH; // hours
uint32_t PeriodD; // days
int      d, h, m, s, a; // used for timer input (& arbitrary uploading)
uint32_t OldTime;
bool     SecChanged; // indicates when second changed
byte     LowFreqDisplay = 0; // if freq below 0.5Hz. 1 = info ready to display, 2 = display info
byte     OldSec;

/********************************************************/

// For Sine, Triangle & Arbitrary (Analogue) Waveforms: (can be accompanied by synchronized square wave)
#define  NWAVEFULL   4096           // size of Full wave-table for slow mode
#define  NWAVETABLE   160           // size of wave-table for fast mode
int      SamplesPerCycle[] = {160, 80, 40, 16}; // {FastMode0, FastMode1, FastMode2, FastMode3}
volatile boolean WaveHalf = LOW;    // identifies current 1/2 cycle
//(1/2 cycle ID -> v) (v <---- 0,  1,  2, 3     0,  1,  2, 3 <-- FastMode)
volatile int  Duty[ ] [4] = {{80, 40, 20, 8}, {80, 40, 20, 8}}; // Samples per 1/2 cycle - changes with duty cycle
// here we are storing 12 bit samples in 16 bit ints:
uint16_t WaveFull[NWAVEFULL + 1];   // wave-table for slow mode used up to 1kHz, and in ExactFreqMode at all freq's (4096 bits)
uint16_t WaveTable[NWAVETABLE + 1]; // wave-table for copying into Wave0 to Wave3 for fast mode use (160 bits)
uint16_t Wave0[2][NWAVETABLE];      // wave used for direct DMA access up to 10kHz  - FastMode0 (160 bits)
uint16_t Wave1[2][NWAVETABLE /  2]; // wave used for direct DMA access up to 20kHz  - FastMode1 (80 bits)
uint16_t Wave2[2][NWAVETABLE /  4]; // wave used for direct DMA access up to 40kHz  - FastMode2 (40 bits)
uint16_t Wave3[2][NWAVETABLE / 10]; // wave used for direct DMA access up to 100kHz - FastMode3 (16 bits)
byte     WaveShape        = 0;      // 0 = Sinewave, 1 = Triangle / Sawtooth, 2 = Arbitrary, etc...
int      TimerCounts      = 26;     // divides 42MHz (CPU timer 1) by 26 counts to produce 1.61538 MHz for DMA handler
int      Delay1           = 10;     // square wave sync delay set at high sample rate
float    Delay2           = 0.55;   // square wave sync delay set at low sample rate (set high 1st)
float    Delay3           = 110;     // square wave sync delay set at low duty
volatile int SyncDelay    = (TimerCounts - Delay1) * Delay2; // delay of square wave for synchronization with peaks of analogue wave
volatile bool MinOrMaxWaveDuty;     // if duty-cycle is 0% or 100% only 1 half of wave is displayed
volatile int FastMode     = 0;      // -1 = up to 1kHz (slow mode), 0 = up to 10kHz, 1 = up to 20kHz, 2 = up to 40khz, 3 = up to 100kHz
int      OldFastMode;               // indicates when FastMode changes
byte     MinMaxDuty       = 1;      // min & max duty-cycle limit for waves (in samples) due to DMA - will be set to 4 when viewing synchronized square wave to prevent polarity reversal due to DMA timing. (set to 1 with unsynchronized square wave as polarity reversal irrelevant)
bool     PotAdjFreq[]     = {1, 1}; // {unsynchronized wave, synchronized waves}: toggles freq adjustment by pot(1) or serial(0)
bool     PotAdjDuty[]     = {1, 1}; // {unsynchronized wave, synchronized waves}: toggles duty-cycle adjustment by pot(1) or serial(0)
float    WaveReading      = 1000;   // Target freq / period (reading from pot)
double   TargetWaveFreq   = 1000;   // synchronized waves Target freq
float    TargetWaveDuty   = 50;     // synchronized waves Target duty-cycle
double   ActualWaveFreq;            // synchronized waves Actual freq
float    ActualWaveDuty;            // synchronized waves Actual duty-cycle
float    TargetWavePulseWidth;      // synchronized waves Target pulse width in mirosecs
float    AnaPulseWidth;             // Actual Pulse Width in mSeconds
float    LastAllowedWaveDuty = 50;
volatile uint32_t WaveBit;          // the bit/sample of the wave curently being processed
double   FreqIncrement = 21475000;  // the number of wave-table bits to skip in slow mode for required freq (multiplied, so we can use an int (below) instead of a float, which is faster)
double   FreqIncrmt[3] = {21475000, 21475000}; // as above {1st half of wave, 2nd half of wave} (21475000 = half wave for 1kHz) used for calculating
volatile uint32_t Increment[]  = {21475000, 21475000}; // as above {1st half of wave, 2nd half of wave} (21475000 = half wave for 1kHz)
double   IncrProportion[] = {1.00, 1.00};   // faster way used to calculate Increment[] when freq changed but duty-cycle not changed
int16_t  DitherPoint;                       // used at very low freq's to dither the Increment to produce more accurate intermediate freq's (between freq steps)
unsigned long DitherTime;                   // indicates time to dither Increment
#define  NARBWAVE   4096                    // max number of waypoints in arbitrary wave
int16_t  ArbitraryPointNumber;              // total number of waypoints entered
int16_t  ArbitraryWave[NARBWAVE + 1];       // variable to store arbitrary wave point data - all points stored (but not the extra data for stepped points)
int16_t  ArbitraryWaveStep[NARBWAVE + 1];   // variable to store arbitrary wave point data - extra data for stepped points stored here
volatile boolean ExactFreqMode = LOW;       // high indicates Exact freq mode is on. Exact at 50% duty-cycle only, causes dithering synchronized square wave
volatile boolean ExactFreqDutyNot50 = LOW;  // high indicates when in Exact freq mode and Not at 50% duty-cycle (and not 0% or 100%) - slightly reduces the workload for the interupt handler when not at 50% duty-cycle
double   ExactFreqModeAccuracy = 0.9999925; //32; // ADJUST ACCURACY of Exact Freq Mode <<<<<<<<<<<<<<<<<<<<<<<< can be tweaked here <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int      DutyMultiplier[3];                 // used when in ExactFreqMode if not at 50% duty-cycle (& not at 0 or 100%), to TRY to maintain freq

/***********************************************************************************************/
//For User Defined Equation based Waveforms
//Initial default values

//  General variables
byte       NumWS = 5;       // Total number of different WaveShapes (6 counting wave 0)

// WaveShape = 0 - Sine Wave variables
// y = A*sin(B(x + C)) + D
float      WS0a = 1.0; //A - Amplitude
float      WS0b = 1.0; //B - "Wave Period". Does not change frequency. Only wave shape within set frequency. 0.5 doubles period and shortens wave higher number of bumps w/in single wavelength, 2 halfs period and elongates wave
float      WS0c = 0.5; //C - Phase shift. O.5 shifts quarter wave to right and wave starts at maximum positive (e.g. cosine or "sine" wave), 0.0 = wave starts at "0" but only top half of wave
float      WS0d = 0.5; //D - Vertical shift
bool       WS0sh = 0;   //Sample and hold toggle (default is off = 0)
float      WS0s = 0.1; //Sample and hold number of holding step fraction (default = 0.1, range is 0.05 to less than 0.5)
int        WS0seg;     //Sample and Hold segment number (used by code)
uint16_t   WS0volt;    //Sample and hold voltage (used by code)

// WaveShape = 1 - Triangle Wave variables
// y = Ax+B
float      WS1a = 1.0; //A - Line slope Negative number is a decending line. 0.0 is horizontal line. Positive number is increasing line.
float      WS1b = 0.5; //B - Y intercept or Vertical shift.
bool       WS1sh = 0;   //Sample and hold toggle (default is off = 0)
float      WS1s = 0.1; //Sample and hold number of holding step fraction (default = 0.1, range is 0.05 to less than 0.5)
int        WS1seg;     //Sample and Hold segment number (used by code)
uint16_t   WS1volt;    //Sample and hold voltage (used by code)

// WaveShape = 2 - Arbitrary Wave variables
bool       WS2sh = 0;   //Sample and hold toggle (default is off = 0)
float      WS2s = 0.1; //Sample and hold number of holding step fraction (default = 0.1, range is 0.05 to less than 0.5)
int        WS2seg;     //Sample and Hold segment number (used by code)
uint16_t   WS2volt;    //Sample and hold voltage (used by code)

// WaveShape = 3 - Step Square wave variables (10 steps)
//y = A0 if Index <= B0;
//y = A1 if Index > B0 && Index <= B1;
//y = A2 if Index > B1 && Index <= B2;
//y = A3 if Index > B2 && Index <= B3;
//y = A4 if Index > B3 && Index <= B4;
//y = A5 if Index > B4 && Index <= B5;
//y = A6 if Index > B5 && Index <= B6;
//y = A7 if Index > B6 && Index <= B7;
//y = A8 if Index > B7 && Index <= B8;
//y = A9 else

float      WS3a0 = 0.000; //A0
float      WS3a1 = 0.111; //A1
float      WS3a2 = 0.222; //A2
float      WS3a3 = 0.333; //A3
float      WS3a4 = 0.444; //A4
float      WS3a5 = 0.555; //A5
float      WS3a6 = 0.666; //A6
float      WS3a7 = 0.777; //A7
float      WS3a8 = 0.888; //A8
float      WS3a9 = 1.000; //A9

float      WS3b0 = 0.10; //B0
float      WS3b1 = 0.20; //B1
float      WS3b2 = 0.30; //B2
float      WS3b3 = 0.40; //B3
float      WS3b4 = 0.50; //B4
float      WS3b5 = 0.60; //B5
float      WS3b6 = 0.70; //B6
float      WS3b7 = 0.80; //B7
float      WS3b8 = 0.90; //B8
////WS3b9 //B9 - Not used (i.e. else)

int        Nextb;   //used in Wave 3 randomiser function
long       RandNum; //used in Wave 3 randomiser function

// WaveShape = 4 - Half Square Root function variables
//y = A*(x + B)^0.5 + C
float       WS4a = 50.0; //A
float       WS4b = 0.0;  //B
float       WS4c = 0.0;  //C (Fraction of Y max between 0.0 and 1.0)
bool        WS4up = 1;   //Up portion of wave
bool        WS4dn = 1;   //Down portion of wave second half
bool        WS4sh = 0;    //Sample and hold toggle (default is off = 0)
float       WS4s = 0.1;  //Sample and hold number of holding step fraction (default = 0.1, range is 0.05 to less than 0.5)
int         WS4seg;      //Sample and Hold segment number (used by code)
uint16_t    WS4volt;     //Sample and hold voltage (used by code)

//// WaveShape = 5 - Full Square Root function variables
//y = A*(x + B)^0.5 + C
float       WS5a = 50.0; //A
float       WS5b = 0.0;  //B
float       WS5c = 0.0;  //C (Fraction of Y max between 0.0 and 1.0)
bool        WS5sh = 0;    //Sample and hold toggle (default is off = 0)
float       WS5s = 0.1;  //Sample and hold number of holding step fraction (default = 0.1, range is 0.05 to less than 0.5)
int         WS5seg;      //Sample and Hold segment number (used by code)
uint16_t    WS5volt;     //Sample and hold voltage (used by code)

// Add more waveshape parameters below here

/***********************************************************************************************/

void setup()
{
  analogReadResolution(12);
  analogWriteResolution(12);
  Serial.begin (115200);
  Serial.setTimeout(50);
  Serial.println("\n     ************** Due Waveform Generator **************\n\n                Type and enter '?' for help.\n\n");
  pinMode(22, INPUT_PULLUP); // for switches
  pinMode(24, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(30, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(34, INPUT_PULLUP);
  pinMode(36, INPUT_PULLUP);
  pinMode(38, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP);
  pinMode(42, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);
  pinMode(23, OUTPUT); // for LEDs
  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(7, OUTPUT); // Square wave PWM output
  CreateWaveFull();
  CreateWaveTable();
  CreateNewWave();
  dac_setup();
  TimerCounts = freqToTc(TargetWaveFreq);
  TC_setup();
  TC_setup4();
  TC_setup5();
  if (!PotsEnabled) SetWaveFreq(1);
  CalculateWaveDuty(1);
  if (!PotsEnabled)
  {
    SetFreqAndDuty(0, 0);
    Serial.print("   Unsync'ed Sq.Wave Freq: ");
    PrintUnsyncedSqWaveFreq(); Serial.print(", Target: "); Serial.print(TargetFreq, 3);
    Serial.print(" Hz\n   Unsync'ed Sq.Wave Period: ");
    PrintUnsyncedSqWavePeriod();
    Serial.print("   Unsync'ed Sq.Wave Duty-cycle: ");
    Serial.print(ActualDuty);
    Serial.println(" %\n");
  }
  //needed for randomiser function
  randomSeed(analogRead(0));
}

void CreateWaveFull() // WaveFull: (actually half a wave in full resolution) for low freq use; prevents 'sample' noise at very low audio freq's (sample-skipping used without DMA)
{
  if (WaveShape == 2) // Arbitrary wave creation - Create the wave from serial data
  {
    //    Serial.print("ArbitraryPointNumber = "); Serial.println(ArbitraryPointNumber);
    float pointSpacing = 4096.00 / float(ArbitraryPointNumber); // ArbitraryPointNumber = 300 if touch screen used
    //   Serial.print("pointSpacing = "); Serial.println(pointSpacing);
    for (int16_t point = 0; point < ArbitraryPointNumber; point++) // changes for each defined point
    {
      int16_t nextPointValue;
      int16_t lastPointValue;
      float level = ArbitraryWave[point];
      if (ArbitraryWaveStep[point + 1] > -1) nextPointValue = ArbitraryWaveStep[point + 1]; // if next point is a wave-step, read level from 'step' variable
      else nextPointValue = ArbitraryWave[point + 1];                                      // otherwise it's a normal point, so read level from usual variable
      if (ArbitraryWaveStep[point] > -1 && point != 0) lastPointValue = ArbitraryWaveStep[point];      // if this point is a wave-step & not 1st point, read level from 'step' variable
      else lastPointValue = ArbitraryWave[max(0, point - 1)];                            // otherwise read level from previous point's usual variable
      byte waveStepPeak = 0; // if sharp wave peak detected (if direction changed sharply) at start of wave step value will depend on freq
      if (ArbitraryWaveStep[point] > -1) // if this point is a wave-step,
      {
        if (abs(constrain((ArbitraryWaveStep[point] - ArbitraryWave[max(0, point - 1)]) / 100, -2, 2) - constrain((ArbitraryWave[point] - ArbitraryWaveStep[point]) / 100, -2, 2)) > 1) waveStepPeak = min(int(TargetWaveFreq / 100), min(12, int(pointSpacing))); // if sharp wave peak detected (if direction changed sharply) at start of wave step
      }
      byte wavePeak = 0; // if sharp wave peak detected (if direction changed) value will depend on freq
      if (abs(constrain((ArbitraryWave[point] - lastPointValue) / 100, -2, 2) - constrain((nextPointValue - ArbitraryWave[point]) / 100, -2, 2)) > 1) wavePeak = min(int(TargetWaveFreq / 100), min(12, int(pointSpacing))); // if sharp wave peak detected (if direction changed sharply)
      float stepValue = float(nextPointValue - ArbitraryWave[point]) / (pointSpacing - min(1, ArbitraryWaveStep[point] + 1) - wavePeak); // calculate size of steps needed to join defined points - less needed if sharp wave peak detected (if direction changed sharply)
      if (waveStepPeak > 0 && wavePeak > 0)
      {
        waveStepPeak /= 2;
        wavePeak -= waveStepPeak;
      }
      int16_t currentPointLocation = round((float(point) / ArbitraryPointNumber) * 4096); // means location of current defined point in full wave
      int16_t nextPointLocation = round((float(point + 1) / ArbitraryPointNumber) * 4096); // means location of next defined point in full wave
      for (int i = currentPointLocation; i < nextPointLocation; i++) // changes for each bit, joining defined points
      {
        if (i == currentPointLocation)
        {
          if (ArbitraryWaveStep[point] > -1 && point != 0)
          {
            if (waveStepPeak > 0) // if sharp wave peak detected at start of wave step (& freq is high now), repeat peak value (create a plateau) to make it more visible:
            {
              int16_t cl = i;
              for (i = i; i < cl + waveStepPeak; i++)
              {
                WaveFull[i] = ArbitraryWaveStep[point];
              }
            }
            WaveFull[i] = ArbitraryWaveStep[point]; // if step present, write 1st half of step
            i++; // move to next wave bit to write 2nd half of step
          }
          if (wavePeak > 0) // if sharp wave peak detected (& freq is high now), repeat peak value (create a plateau) to make it more visible:
          {
            int16_t cl = i;
            for (i = i; i < cl + wavePeak; i++)
            {
              WaveFull[i] = round(level);
            }
          }
        }
        WaveFull[i] = round(level);
        level += stepValue;
      }
    }
  }
  else
  {
    for (int Index = 0; Index < NWAVEFULL; Index++) // create the individual samples for the wave - half cycle, 4096 steps, 12 bit range: 4095 - 0
    {
      if (WaveShape == 0) // Sine wave
      {
        // y = A*sin(B(x + C)) + D
        WaveFull[Index] = (uint16_t) ((((WS0d * 2) + WS0a * sin((((2.0 * PI) / (WS0b * NWAVEFULL)) / 2) * (Index + (WS0c * NWAVEFULL)))) * 4095.0) / 2);
        // Sample and Hold function (i.e. sample rate reduction)
        if (WS0sh == 1)  
        {
          if (Index == 0) // Initialise
          {
            WS0volt = WaveFull[Index];
            WS0seg = 1; //Segment number
          }
          else if (Index == (int) (WS0seg * (WS0s * (NWAVEFULL - 1)))) // Sample
          {
            WS0seg = WS0seg + 1;
            WS0volt = WaveFull[Index];
          }
          else // Hold
          {
             WaveFull[Index] = WS0volt;
          }
        }  
      }
      else if (WaveShape == 1)  // Triangle wave
      {
        // y = A/2+B-A*x // y = Ax+B
        WaveFull[Index] = ((WS1b * (NWAVEFULL - 1)) + (WS1a * (NWAVEFULL / 2))) - (WS1a * Index); // NWAVEFULL - 1 - Index;
        // Sample and Hold function (i.e. sample rate reduction)
        if (WS1sh == 1)  
        {
          if (Index == 0) // Initialise
          {
            WS1volt = WaveFull[Index];
            WS1seg = 1; //Segment number
          }
          else if (Index == (int) (WS1seg * (WS1s * (NWAVEFULL - 1)))) // Sample
          {
            WS1seg = WS1seg + 1;
            WS1volt = WaveFull[Index];
          }
          else // Hold
          {
             WaveFull[Index] = WS1volt;
          }
        }     
      }
      else if (WaveShape == 3)  // Staircase "wave" (maximum 10 steps)
      {
        //y = A0 if Index <= B0;
        //y = A1 if Index > B0 && Index <= B1;
        //y = A2 if Index > B1 && Index <= B2;
        //y = A3 if Index > B2 && Index <= B3;
        //y = A4 if Index > B3 && Index <= B4;
        //y = A5 if Index > B4 && Index <= B5;
        //y = A6 if Index > B5 && Index <= B6;
        //y = A7 if Index > B6 && Index <= B7;
        //y = A8 if Index > B7 && Index <= B8;
        //y = A9 else

        if ((WS3b0 != 0.0) && (Index >= 0) && (Index < (WS3b0 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a0 * (NWAVEFULL - 1));
        else if ((WS3b1 != 0.0) && (Index >= (WS3b0 * (NWAVEFULL - 1))) && (Index < (WS3b1 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a1 * (NWAVEFULL - 1));
        else if ((WS3b2 != 0.0) && (Index >= (WS3b1 * (NWAVEFULL - 1))) && (Index < (WS3b2 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a2 * (NWAVEFULL - 1));
        else if ((WS3b3 != 0.0) && (Index >= (WS3b2 * (NWAVEFULL - 1))) && (Index < (WS3b3 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a3 * (NWAVEFULL - 1));
        else if ((WS3b4 != 0.0) && (Index >= (WS3b3 * (NWAVEFULL - 1))) && (Index < (WS3b4 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a4 * (NWAVEFULL - 1));
        else if ((WS3b5 != 0.0) && (Index >= (WS3b4 * (NWAVEFULL - 1))) && (Index < (WS3b5 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a5 * (NWAVEFULL - 1));
        else if ((WS3b6 != 0.0) && (Index >= (WS3b5 * (NWAVEFULL - 1))) && (Index < (WS3b6 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a6 * (NWAVEFULL - 1));
        else if ((WS3b7 != 0.0) && (Index >= (WS3b6 * (NWAVEFULL - 1))) && (Index < (WS3b7 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a7 * (NWAVEFULL - 1));
        else if ((WS3b8 != 0.0) && (Index >= (WS3b7 * (NWAVEFULL - 1))) && (Index < (WS3b8 * (NWAVEFULL - 1)))) WaveFull[Index] = (uint16_t) (WS3a8 * (NWAVEFULL - 1));
        else WaveFull[Index] = (uint16_t) (WS3a9 * (NWAVEFULL - 1));
      }
      else if (WaveShape == 4) // Half Square Root Function "wave"
      {
        //y = A*(x + B)^0.5 + C
        if ((Index + WS4b) < 0) WaveFull[Index] = 0; //Square Root of a negative number is imaginary therefore set to zero
        else
        {
          if ((Index >= 0) && (Index < ((NWAVEFULL - 1) / 2)))
          {
            if (WS4up == 1) WaveFull[Index] = (uint16_t) ((WS4a * pow((Index + WS4b), 0.5)) + (WS4c * NWAVEFULL - 1));
            else WaveFull[Index] = 0;
          }
          else
          {
            if (WS4dn == 1) WaveFull[Index] = (uint16_t) (-(WS4a * pow((Index + WS4b), 0.5) + (WS4c * NWAVEFULL - 1)));
            else WaveFull[Index] = 0;
          }
        }
        // Sample and Hold function (i.e. sample rate reduction)
        if (WS4sh == 1)  
        {
          if (Index == 0) // Initialise
          {
            WS4volt = WaveFull[Index];
            WS4seg = 1; //Segment number
          }
          else if (Index == (int) (WS4seg * (WS4s * (NWAVEFULL - 1)))) // Sample
          {
            WS4seg = WS4seg + 1;
            WS4volt = WaveFull[Index];
          }
          else // Hold
          {
             WaveFull[Index] = WS4volt;
          }
        } 
      }
      else if (WaveShape == 5)  // Full Square Root Function "wave"
      {
        //y = A*(x + B)^0.5 + C
        if ((Index + WS5b) < 0) WaveFull[Index] = 0; // Square Root of a negative number is imaginary therefore set to zero
        else
        {
          WaveFull[Index] = (uint16_t) ((WS5a * pow((Index + WS5b), 0.5)) + (WS5c * NWAVEFULL - 1));
        }
        // Sample and Hold function (i.e. sample rate reduction)
        if (WS5sh == 1)  
        {
          if (Index == 0) // Initialise
          {
            WS5volt = WaveFull[Index];
            WS5seg = 1; //Segment number
          }
          else if (Index == (int) (WS5seg * (WS5s * (NWAVEFULL - 1)))) // Sample
          {
            WS5seg = WS5seg + 1;
            WS5volt = WaveFull[Index];
          }
          else // Hold
          {
             WaveFull[Index] = WS5volt;
          }
        } 
      }
      WaveFull[Index] = constrain(WaveFull[Index], 0, (NWAVEFULL - 1)); // keep wave clamped between maximum and minium values
      //Serial.print(Index); Serial.print(" WF = "); Serial.println(WaveFull[Index]);
    }
  }
}

void CreateWaveTable() // WaveTable: a smaller half wave for higher freq use; faster to copy from smaller table into NewWave (below) [It takes 50% longer to copy from full wave above!]
{
  for (int Index = 0; Index < NWAVETABLE; Index++) // create the individual samples for the wave table - half cycle, 160 steps, 12 bit range: 4095 - 0
  {
    if (WaveShape == 0) // Sine wave
    {
      // y = A*sin(B*(x + C)) + D
      WaveTable[Index] = (uint16_t) ((((WS0d * 2) + WS0a * sin((((2.0 * PI) / (WS0b * NWAVETABLE)) / 2) * (Index + (WS0c * NWAVETABLE)))) * 4095.0) / 2);
      // Sample and Hold function (i.e. sample rate reduction)
      if (WS0sh == 1)  
      {
        if (Index == 0) // Initialise
        {
          WS0volt = WaveTable[Index];
          WS0seg = 1; //Segment number
        }
        else if (Index == (int) (WS0seg * (WS0s * (NWAVETABLE - 1)))) // Sample
        {
          WS0seg = WS0seg + 1;
          WS0volt = WaveTable[Index];
        }
        else // Hold
        {
           WaveTable[Index] = WS0volt;
        }
      } 
    }
    else if (WaveShape == 1) // Triangle wave
    {
      // y = A/2+B-A*x
      WaveTable[Index] = (uint16_t) (((1.0 / (WS1b * (NWAVEFULL - 1))) + (WS1a * (NWAVEFULL / 2))) - (WS1a * Index) * 4095.0); // (((1.0 / (NWAVETABLE - 1)) * (NWAVETABLE - 1 - Index)) * 4095.0);
      // Sample and Hold function (i.e. sample rate reduction)
      if (WS1sh == 1)  
      {
        if (Index == 0) // Initialise
        {
          WS1volt = WaveTable[Index];
          WS1seg = 1; //Segment number
        }
        else if (Index == (int) (WS1seg * (WS1s * (NWAVETABLE - 1)))) // Sample
        {
          WS1seg = WS1seg + 1;
          WS1volt = WaveTable[Index];
        }
        else // Hold
        {
           WaveTable[Index] = WS1volt;
        }
      }
    }
    else if (WaveShape == 2) // Arbitrary wave
    {
      WaveTable[Index] = (uint16_t) WaveFull[round(Index * 25.6)]; // Arbitrary wave
    }
    else if (WaveShape == 3)  // Staircase "wave" (maximum 10 steps)
    {
      //y = A0 if Index <= B0;
      //y = A1 if Index > B0 && Index <= B1;
      //y = A2 if Index > B1 && Index <= B2;
      //y = A3 if Index > B2 && Index <= B3;
      //y = A4 if Index > B3 && Index <= B4;
      //y = A5 if Index > B4 && Index <= B5;
      //y = A6 if Index > B5 && Index <= B6;
      //y = A7 if Index > B6 && Index <= B7;
      //y = A8 if Index > B7 && Index <= B8;
      //y = A9 else

      if ((WS3b0 != 0.0) && (Index >= 0) && (Index < (WS3b0 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a0 * (NWAVEFULL - 1));
      else if ((WS3b1 != 0.0) && (Index >= (WS3b0 * (NWAVETABLE - 1))) && (Index < (WS3b1 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a1 * (NWAVEFULL - 1));
      else if ((WS3b2 != 0.0) && (Index >= (WS3b1 * (NWAVETABLE - 1))) && (Index < (WS3b2 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a2 * (NWAVEFULL - 1));
      else if ((WS3b3 != 0.0) && (Index >= (WS3b2 * (NWAVETABLE - 1))) && (Index < (WS3b3 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a3 * (NWAVEFULL - 1));
      else if ((WS3b4 != 0.0) && (Index >= (WS3b3 * (NWAVETABLE - 1))) && (Index < (WS3b4 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a4 * (NWAVEFULL - 1));
      else if ((WS3b5 != 0.0) && (Index >= (WS3b4 * (NWAVETABLE - 1))) && (Index < (WS3b5 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a5 * (NWAVEFULL - 1));
      else if ((WS3b6 != 0.0) && (Index >= (WS3b5 * (NWAVETABLE - 1))) && (Index < (WS3b6 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a6 * (NWAVEFULL - 1));
      else if ((WS3b7 != 0.0) && (Index >= (WS3b6 * (NWAVETABLE - 1))) && (Index < (WS3b7 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a7 * (NWAVEFULL - 1));
      else if ((WS3b8 != 0.0) && (Index >= (WS3b7 * (NWAVETABLE - 1))) && (Index < (WS3b8 * (NWAVETABLE - 1)))) WaveTable[Index] = (uint16_t) (WS3a8 * (NWAVEFULL - 1));
      else WaveTable[Index] = (uint16_t) (WS3a9 * (NWAVEFULL - 1));
    }
    else if (WaveShape == 4) // Half Square Root Function "wave"
    {
      //y = A*(x + B)^0.5 + C
      //y = -(A*(x + B)^0.5 + C)
      if ((Index + WS4b) < 0) WaveTable[Index] = 0; // Square Root of a negative number is imaginary therefore set to zero
      else
      {
        if ((Index >= 0) && (Index < ((NWAVETABLE - 1) / 2)))
        {
          if (WS4up == 1) WaveTable[Index] = (uint16_t) ((WS4a * pow((Index + WS4b), 0.5)) + (WS4c * NWAVETABLE - 1));
          else WaveTable[Index] = 0;
        }
        else
        {
          if (WS4dn == 1) WaveTable[Index] = (uint16_t) (-(WS4a * pow((Index + WS4b), 0.5) + (WS4c * NWAVETABLE - 1)));
          else WaveTable[Index] = 0;
        }
      }
      // Sample and Hold function (i.e. sample rate reduction)
      if (WS4sh == 1)  
      {
        if (Index == 0) // Initialise
        {
          WS4volt = WaveTable[Index];
          WS4seg = 1; //Segment number
        }
        else if (Index == (int) (WS4seg * (WS4s * (NWAVETABLE - 1)))) // Sample
        {
          WS4seg = WS4seg + 1;
          WS4volt = WaveTable[Index];
        }
        else // Hold
        {
           WaveTable[Index] = WS4volt;
        }
      }
    }
    else if (WaveShape == 5) // Full Square Root function "wave"
    {
      //y = A*(x + B)^0.5 + C
      if ((Index + WS5b) < 0) WaveTable[Index] = 0; // Square Root of a negative number is imaginary therefore set to zero
      else
      {
        WaveTable[Index] = (uint16_t) ((WS5a * pow((Index + WS5b), 0.5)) + (WS5c * NWAVETABLE - 1));
      }
      // Sample and Hold function (i.e. sample rate reduction)
      if (WS5sh == 1)  
      {
        if (Index == 0) // Initialise
        {
          WS5volt = WaveTable[Index];
          WS5seg = 1; //Segment number
        }
        else if (Index == (int) (WS5seg * (WS5s * (NWAVETABLE - 1)))) // Sample
        {
          WS5seg = WS5seg + 1;
          WS5volt = WaveTable[Index];
        }
        else // Hold
        {
           WaveTable[Index] = WS5volt;
        }
      }
    }
    WaveTable[Index] = constrain(WaveTable[Index], 0, (NWAVEFULL - 1)); // keep wave clamped between maximum and minium values
    //    Serial.print(Index); Serial.print(" WT = "); Serial.println(WaveTable[Index]);
  }
}

// NewWave: For analogue wave frequencies above 1kHz. Samples from WaveTable to be copied here with selected duty cycle, for copying by DMA
void CreateNewWave() // create the individual samples for each FastMode wave whenever the duty-cycle is changed (copy from WaveTable)
{
  boolean wh = !WaveHalf; // idendifies which half of wave is CURRENTLY being written by DMA - (LOW = 1st half of wave [positive going half])
  byte fm = max(0, FastMode); // capture the current FastMode
  for (int cycle = 0; cycle < 4; cycle++) // update all 4 FastMode waves, current one first
  {
    byte dv = 0;
    if (TargetWaveDuty == 0 || TargetWaveDuty == 100) dv++; // if displaying one half of wave only divide by 1 less (next lines) to maintain full amplitude as both 0 & 4095 will be included in same half
    float inc0 = float(NWAVETABLE - dv) / (Duty[0][fm] - dv); // calculate increment needed for sampling 1st half wave from wavetable the required number of steps (depending on duty-cycle)
    float inc1 = float(NWAVETABLE) / (Duty[1][fm] - dv); // calculate increment needed for sampling 2nd half wave from wavetable the required number of steps (depending on duty-cycle)
    if (wh == HIGH)
    {
      if (TargetWaveDuty >   0) Create1stHalfNewWave(fm, inc0); // update next half wave cycle first (causes less disturbance to wave especially at high freq's)
      if (TargetWaveDuty < 100) Create2ndHalfNewWave(fm, inc1); // update current half wave cycle afterwards
    }
    else // if (wh ==  LOW)
    {
      if (TargetWaveDuty < 100) Create2ndHalfNewWave(fm, inc1); // update next half wave cycle first (causes less disturbance to wave especially at high freq's)
      if (TargetWaveDuty >   0) Create1stHalfNewWave(fm, inc0); // update current half wave cycle afterwards
    }
    if (fm < 3) fm++;
    else fm = 0;
  }
}
void Create1stHalfNewWave(byte fm, float inc0)
{
  //  Serial.println("Half 1");
  float x = NWAVETABLE;
  for (int Index = 0; Index < Duty[0][fm]; Index++) // 1st half of cycle
  { // separate files used (Wave0 - 3) instead of arrays to reduce memory useage: [since Wave3 is much smaller than Wave0]
    if ((TargetWaveDuty > 0 && TargetWaveDuty < 100) || Index != 0) x -= inc0;
    else x = NWAVETABLE - 1; // if displaying one half of wave only start from full amplitude at NWAVETABLE 159 instead of 160
    if      (fm == 0) Wave0[0][Index] = WaveTable[round(x)];
    else if (fm == 1) Wave1[0][Index] = WaveTable[round(x)];
    else if (fm == 2) Wave2[0][Index] = WaveTable[round(x)];
    else if (fm == 3) Wave3[0][Index] = WaveTable[round(x)];
    //    if      (fm == 3) {Serial.print(Index); Serial.print("a = "); Serial.print(round(x)); Serial.print(" val = "); Serial.println(Wave3[0][Index]);}
  }
}
void Create2ndHalfNewWave(int fm, float inc1)
{
  //  Serial.println("Half 2");
  float  x = 0;
  for (int Index = 0; Index < Duty[1][fm]; Index++) // 2nd half of cycle
  { // separate files used (Wave0 - 3) instead of arrays to reduce memory useage: [since Wave3 is much smaller than Wave0]
    if ((TargetWaveDuty > 0 && TargetWaveDuty < 100) || Index != 0) x = min(NWAVETABLE - 1, x + inc1); // if displaying one half of wave only start from full amplitude
    if      (fm == 0) Wave0[1][Index] = WaveTable[round(x)];
    else if (fm == 1) Wave1[1][Index] = WaveTable[round(x)];
    else if (fm == 2) Wave2[1][Index] = WaveTable[round(x)];
    else if (fm == 3) Wave3[1][Index] = WaveTable[round(x)];
    //    if      (fm == 3) {Serial.print(Index); Serial.print("b = "); Serial.print(round(x)); Serial.print(" val = "); Serial.println(Wave3[1][Index]);}
  }
}

void loop()
{
  if (millis() > SwitchPressedTime + 500) // check pot control switch/es:
  {
    if (!digitalRead(22))
    {
      PotsEnabled = !PotsEnabled;
      Serial.print("PotsEnabled = "); Serial.println(PotsEnabled);
      digitalWrite(23, PotsEnabled);
      if (!PotsEnabled)
      {
        digitalWrite(25, LOW);
        digitalWrite(27, LOW);
        digitalWrite(29, LOW);
        digitalWrite(31, LOW);
        digitalWrite(33, LOW);
        digitalWrite(35, LOW);
        digitalWrite(37, LOW);
        digitalWrite(39, LOW);
        digitalWrite(41, LOW);
        digitalWrite(45, LOW);
        digitalWrite(47, LOW);
        digitalWrite(49, LOW);
      }
      SwitchPressedTime = millis();
    }
    if (PotsEnabled) // check other switches:
    {
      if (!digitalRead(24)) // toggle unsync'ed wave freq / period
      {
        PotPeriodMode[0] = !PotPeriodMode[0];
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(26)) // change unsync'ed wave freq / period range
      {
        if (Range[0] < 10000) Range[0] *= 10;
        else Range[0] = 1;
        Serial.print("   Unsync'ed Sq.Wave Pot Freq Range: x "); Serial.println(Range[0]);
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(28)) // toggle sync'ed waves freq / period
      {
        PotPeriodMode[1] = !PotPeriodMode[1];
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(30)) // change unsync'ed wave freq / period range
      {
        if (Range[0] < 10000) Range[0] *= 10;
        else Range[0] = 1;
        Serial.print("   Unsync'ed Sq.Wave Pot Freq Range: x "); Serial.println(Range[0]);
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(32)) // toggle unsync'ed waves Duty Cycle / Pulse Width
      {
        PotPulseWidth[0] = !PotPulseWidth[0];
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(34)) // change unsync'ed wave Pulse Width Range
      {
        if (Range[2] < 10000) Range[2] *= 10;
        else Range[2] = 1;
        Serial.print("   Unsync'ed Sq.Wave Pot Pulse Width Range: x "); Serial.println(Range[2]);
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(36)) // toggle unsync'ed waves Duty Cycle / Pulse Width
      {
        PotPulseWidth[1] = !PotPulseWidth[1];
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(38)) // change sync'ed wave Pulse Width Range
      {
        if (Range[3] < 10000) Range[3] *= 10;
        else Range[3] = 1;
        Serial.print("   Sync'ed Sq.Wave Pot Pulse Width Range: x "); Serial.println(Range[3]);
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(40)) // Change Wave Shape
      {
        if (WaveShape < NumWS) WaveShape++;
        else WaveShape = 0;
        if      (WaveShape == 0) Serial.println("             ********** Sine Wave **********\n");
        else if (WaveShape == 1) Serial.println("             ******** Triangle Wave ********\n");
        else if (WaveShape == 2) Serial.println("             ******** Arbitrary Wave *******\n");
        else if (WaveShape == 3) Serial.println("             ******** Staircase Wave *******\n");
        else if (WaveShape == 4) Serial.println("             **** Half Square Root Wave ****\n");
        else if (WaveShape == 5) Serial.println("             **** Full Square Root Wave ****\n");
        CreateWaveFull();
        CreateWaveTable();
        CreateNewWave();
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(42)) // toggle ExactFreqMode
      {
        ToggleExactFreqMode();
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(44)) // toggle Square Wave Sync
      {
        ToggleSquareWaveSync();
        SwitchPressedTime = millis();
      }
      else if (!digitalRead(46)) // Change which wave/s to control
      {
        if (Control < 2) Control++;
        else Control = 0;
        SwitchPressedTime = millis();
      }
      if (millis() > LEDupdateTime + 300)
      {
        digitalWrite(25, PotPeriodMode[0]);
        digitalWrite(27, !PotPeriodMode[0]);
        digitalWrite(29, PotPeriodMode[1]);
        digitalWrite(31, !PotPeriodMode[1]);
        digitalWrite(33, !PotPulseWidth[0]);
        digitalWrite(35, PotPulseWidth[0]);
        digitalWrite(37, !PotPulseWidth[1]);
        digitalWrite(39, PotPulseWidth[1]);
        digitalWrite(41, ExactFreqMode);
        digitalWrite(45, SquareWaveSync);
        if (Control > 0) digitalWrite(47, HIGH);
        else digitalWrite(47, LOW);
        if (Control != 1) digitalWrite(49, HIGH);
        else digitalWrite(49, LOW);
        LEDupdateTime = millis();
      }
    }
  }
  if (PotsEnabled)
  {
    Pot0 = analogRead(A0);
    Pot1 = analogRead(A1);
  }
  /***********************************************************************************************/
  // UNSYNC'ED SQ.WAVE FREQ / PERIOD ADJUSTMENT:
  float newReading =  Pot0 * Range[0];
  if (PotAdjFreq[0] == 1 || Control == 1)
  {
    OldReading[0] += (newReading - OldReading[0]) * constrain(0.02 * abs(newReading - OldReading[0]), 0.03, 1); // Responsive smoothing
    if (PotPeriodMode[0] == 1) FreqReading = min(84000000 / OldReading[0], 42000000); // if adjusting period convert to freq - max freq = 42MHz
    else FreqReading = min(OldReading[0]  , 42000000); // if adjusting freq - max freq = 42MHz
  }
  if ((abs(FreqReading - TargetFreq) > TargetFreq / 40 && PotAdjFreq[0] == 1)
      || (abs(newReading - OldReading[0]) > OldReading[0] / 20 && PotAdjFreq[0] == 0))
  {
    if (Control != 1)
    {
      PotAdjFreq[0] = 1;
      if (PotPulseWidth[0] && TargetPulseWidth > 0) TargetDuty = min(100, 100 * (TargetPulseWidth / (1000000 / min(FreqReading, 42000000)))); // convert pulse width input into %
      else TargetPulseWidth = 0;

      TargetFreq = FreqReading;
      if (TargetFreq >= 100) Period = 84000000 / TargetFreq;
      else Period = 200000 / TargetFreq;
      SetFreqAndDuty(1, 1);
      Serial.print("   Unsync'ed Sq.Wave Freq: ");
      PrintUnsyncedSqWaveFreq();
      Serial.print(", Unsync'ed Sq.Wave Duty Cycle: "); Serial.print(ActualDuty);
      Serial.print(" %\n   Unsync'ed Sq.Wave Period: ");
      PrintUnsyncedSqWavePeriod();
      Serial.println("");
    }
  }
  // PULSE DUTY CYCLE ADJUSTMENT:
  float newRead;
  if (PotPulseWidth[0]) newRead = (Pot1 / 40) * Range[2];
  else newRead = Pot1 / 40;
  if (PotAdjDuty[0] == 1 || Control == 1) DutyReading[0] += (newRead - DutyReading[0]) * constrain(0.04 * abs(newRead - DutyReading[0]), 0.05, 1); // Responsive smoothing
  if ((abs(DutyReading[0] - TargetDuty) > 0.5 && PotAdjDuty[0] == 1)
      || (abs(newRead - DutyReading[0]) > 1 && PotAdjDuty[0] == 0))
  {
    if (Control != 1)
    {
      PotAdjDuty[0] = 1;
      if (PotPulseWidth[0])
      {
        TargetPulseWidth = round(DutyReading[0]);
        TargetDuty = 100 * (round(DutyReading[0]) / (1000000 / ActualFreq)); // convert pulse width input into %
      }
      else TargetDuty = round(DutyReading[0]);
      SetFreqAndDuty(0, 1);
      Serial.print("   Unsync'ed Sq.Wave Duty-cycle: "); Serial.print(ActualDuty); Serial.print(" %,  Actual: "); Serial.print(TargetDuty); Serial.println(" %\n");
    }
  }
  /***********************************************************************************************/
  // SYNC'ED WAVES FREQ ADJUSTMENT:
  float newWaveReading = max(Pot0 * Range[1] * 0.01, 0.01);
  if (PotAdjFreq[1] == 1 || Control == 0)
  {
    OldReading[1] += (newWaveReading - OldReading[1]) * constrain(0.02 * abs(newWaveReading - OldReading[1]), 0.03, 1); // Responsive smoothing
    if (PotPeriodMode[1] == 1) WaveReading = min(100000 / OldReading[1], 100000); // if adjusting period convert to freq
    else WaveReading = min(OldReading[1], 100000); // if adjust freq
  }
  if ((abs(TargetWaveFreq - WaveReading) > TargetWaveFreq / 40 && PotAdjFreq[1] == 1)
      || (abs(newWaveReading - OldReading[1]) > OldReading[1] / 20 && PotAdjFreq[1] == 0))
  {
    if (Control > 0)
    {
      PotAdjFreq[1] = 1;
      if (PotPulseWidth[1] && TargetWavePulseWidth > 0) TargetWaveDuty = min(100, 100 * (TargetWavePulseWidth / (1000000 / min(WaveReading, 100961)))); // convert pulse width input into %
      else TargetWavePulseWidth = 0;
      TargetWaveFreq = min(WaveReading, 100961);

      FreqIncrement = WaveReading * 21475;
      //      TargetWaveFreq = WaveReading;
      SetWaveFreq(0);
      Serial.print("   Analogue Wave Freq: ");
      PrintSyncedWaveFreq();
      Serial.print(", Analogue Wave Duty Cycle: "); Serial.print(ActualWaveDuty);
      Serial.print(" %\n   Analogue Wave Period: ");
      PrintSyncedWavePeriod();
      Serial.println("");
    }
  }
  // SYNC'ED WAVES DUTY CYCLE ADJUSTMENT:
  if (PotPulseWidth[1]) newRead = (Pot1 / 40) * Range[3];
  else newRead = Pot1 / 40;
  if (PotAdjDuty[1] == 1 || Control == 0) DutyReading[1] += (newRead - DutyReading[1]) * constrain(0.04 * abs(newRead - DutyReading[1]), 0.05, 1); // Responsive smoothing
  if ((abs(DutyReading[1] - TargetWaveDuty) > 0.5 && PotAdjDuty[1] == 1)
      || (abs(newRead - DutyReading[1]) > 1 && PotAdjDuty[1] == 0))
  {
    if (Control > 0)
    {
      PotAdjDuty[1] = 1;
      if (PotPulseWidth[1])
      {
        TargetWavePulseWidth = round(DutyReading[1]);
        TargetWaveDuty = 100 * (round(DutyReading[1]) / (1000000 / ActualWaveFreq)); // convert pulse width input into %
      }
      else TargetWaveDuty = round(DutyReading[1]);
      CalculateWaveDuty(0);
      Serial.print("   Analogue Wave Duty-cycle: "); Serial.print(ActualWaveDuty); Serial.print(" %,  Target: "); Serial.print(TargetWaveDuty); Serial.println(" %\n");
      CreateNewWave();
    }
  }
  /***********************************************************************************************/
  if (Serial.available())
  {
    int maxNum = 1;
    UserInput = Serial.parseFloat(SKIP_NONE); // read float related characters until other character appears, then stop.
    if (Serial.peek() == 'x') maxNum = 4; // look ahead to see what the next character will be - if it's 'x' maxNum is 4, otherwise it's 1
    Serial.readBytesUntil('\n', UserChars, maxNum); // read all characters (including numbers) after initial (float) numbers up to maxNum of characters (only 1 character if next character is not 'x')
    if (UserChars[0] == 'x') {Serial.print("   You sent: "); Serial.print(UserInput); Serial.print(UserChars[0]); Serial.print(UserChars[1]); Serial.print(UserChars[2]); Serial.println(UserChars[3]);}
    // else Serial.print("   UserChars[0] = "); Serial.println(UserChars[0]);
    if (a == 1 || UserChars[0] == ',' || UserChars[0] == '-') // If currently receiving arbitrary wave. UserChars[0] will be ',' or '-' if starting to add more points to an existing arbitrary wave.
    {
      if (a == 0) // only occurs when starting to add more points to an existing arbitrary wave
      {
        a = 1;
        Serial.print("   "); // indent start of numbers in serial monitor if starting to add more points to an existing arbitrary wave
      }
      if (UserChars[0] == ',') // point separator / counter
      {
        if (ArbitraryPointNumber > 0 && ArbitraryPointNumber % 10 == 0) Serial.print("\n   "); // start a new line after every 10 points entered
        ArbitraryWave[ArbitraryPointNumber] = UserInput; // save Wave point data
        if (ArbitraryPointNumber == 0 && ArbitraryWaveStep[ArbitraryPointNumber] == -1) Serial.print("   You entered:\n   ");
        Serial.print(ArbitraryWave[ArbitraryPointNumber]); Serial.print(",");
        if (ArbitraryPointNumber < NARBWAVE - 1) ArbitraryPointNumber++;
      }
      else if (UserChars[0] == '-') // stepped point separator
      {
        ArbitraryWaveStep[ArbitraryPointNumber] = UserInput; // save Wave Step data
        if (ArbitraryPointNumber == 0) Serial.print("   You entered:\n   ");
        Serial.print(ArbitraryWaveStep[ArbitraryPointNumber]); Serial.print("-");
      }
      else if (UserChars[0] == ';') // last point detector / signal to create wave
      {
        ArbitraryWave[ArbitraryPointNumber] = UserInput;
        Serial.print(ArbitraryWave[ArbitraryPointNumber]); Serial.println(";");
        if (ArbitraryPointNumber < NARBWAVE) ArbitraryPointNumber++;
        ArbitraryWave[ArbitraryPointNumber] = ArbitraryWave[0]; // make last point same as first
        ArbitraryWaveStep[ArbitraryPointNumber] = ArbitraryWaveStep[0];
        Serial.print("   ..... a total of "); Serial.print(ArbitraryPointNumber);
        if (ArbitraryPointNumber < NARBWAVE - 1) Serial.println(" points.\n");
        else Serial.println(" points. THIS IS THE MAXIMUM LIMIT\n");
        WaveShape = 2;
        CreateWaveFull();
        CreateWaveTable();
        CreateNewWave();
        Serial.println("   Set duty-cycle to 0% (Type 0d). Other duty-cycle settings create a 'mirrored' effect.");
        Serial.println("   Typing 'a' again (to enter a new arbitrary wave) will clear the current wave from memory.");
        if (ArbitraryPointNumber < 300) Serial.println("   Or you can just add more points!\n   ");
        else Serial.println("\n   ");
        a = 0;
      }
    }
    else // if (a == 0) // If NOT currently receiving arbitrary wave
    {
      delay(1); // short delay to ensure UserChars[0] is read correctly next
      if (UserChars[0] == 'a' && TimerMode == 0) // Arbitrary wave creation - deletes old wave data, ready for new wave data
      {
        a = 1;
        ArbitraryPointNumber = 0;
        for (int16_t i = 0; i <= 4096; i++) {
          ArbitraryWaveStep[i] = -1; // resets any stepped points to 'unstepped' state
        }
        Serial.println("");
        Serial.println("         ************************* NEW ARBITRARY WAVE CREATION *************************        \n");
        Serial.println("   Please type the value of each point you wish to define. (Any old data has been deleted.)");
        Serial.println("   Separate each value (0 to 4095) with a comma. Use no spaces. Finish with a semi-colon.\n");
        Serial.println("   For example:\n   2047,2150,3800,4095,3800,400,200,400,2510,2700,2510,1800,1700,1800,2040,2150,2050,1980,1960,2000;\n");
        Serial.println("   You can create steps in the wave by dividing points into two values.\n   For example, an 'M' wave:  0,0-4095,1600,4095-0;\n");
        Serial.println("   You can define up to 300 points, but enter them no more than 40 at a time, ending with a comma.");
        Serial.println("   Use a semi-colon at the end to trigger wave creation. Points can also be added later. (300 total)\n");
      }
//********************************** Block of extra parameter code: ****************************************/
      else if (UserChars[0] == 'x')
      {
        if (UserChars[1] == 'x') // if received xx Update Wave Tables:
        {
          CreateWaveFull();
          CreateWaveTable();
          CreateNewWave();
          Serial.println("\n   Wave Tables updated and refreshed.\n");
        }
        else if (UserChars[1] == 'w') // if received xw List Wave shapes
        {
          Serial.println("\n   Wave Shape 0 = Sine wave.");
          Serial.println(  "   Wave Shape 1 = Triangle wave");
          Serial.println(  "   Wave Shape 3 = Staircase wave function wave");
          Serial.println(  "   Wave Shape 4 = Half Square Root function wave");
          Serial.println(  "   Wave Shape 5 = Full Square Root function wave\n");
        }
        else if (UserChars[1] == '0')      //  if received 0 - Wave Shape 0 variables:
        {
          if (UserChars[2] == 'a') // if received x0a
          {
            WS0a = UserInput;
            Serial.print("   Wave Shape 0 A - Sine Amplitude changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'b') // if received x0b
          {
            WS0b = UserInput;
            Serial.print("   Wave Shape 0 B - Sine Waveshape period changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'c') // if received x0c
          {
            WS0c = UserInput;
            Serial.print("   Wave Shape 0 C - Sine Phase Shift changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'd') // if received x0d
          {
            WS0d = UserInput;
            Serial.print("   Wave Shape 0 D - Sine Vertical Shift changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'h') // if received x0h - Sample and Hold toggle
          {
            if ((UserInput == 1) || (UserInput == 0))
            {
              WS0sh = UserInput;
              Serial.print("   Sample and Hold toggle on/off changed to: ");
              Serial.println(UserInput);    
            }
            else Serial.println("   Not valid input for x0h. Must enter either 1 or 0 only (e.g. 1x0h).\n");    
          }
          else if (UserChars[2] == 's') // if received x0s - Sample and Hold fraction
          {
             if ((UserInput >= 0.05) && (UserInput < 0.5))
             {
                WS0s = UserInput;
                Serial.print("   Sample and Hold fraction changed to: ");
                Serial.println(UserInput);
             }
             else Serial.println("   Not valid input for x0s. Must enter value between 0.05 and less than 0.5 only (e.g. 0.2x0s).\n");
          }
          else if (UserChars[2] == 'r')   //if recieved x0r
          {
            //randomise all variables
            // random number set for each variable acceptable range  //NEEDS TWEAKING
            WS0a = random(-500,501)/100.0; // This is fixed from -5 to 5
            WS0b = random(-500,501)/100.0; // This is fixed from -5 to 5
            WS0c = random(-500,501)/100.0; // This is fixed from -5 to 5
            WS0d = random(101)/70.0; //this is fixed from 0 to about 1.43
            Serial.println("   Wave Shape 0 - Sine Wave all equation parameters randomised."); 
            Serial.println("   Note this may result in wave off scale hi/low (i.e. flat line). Tweak variables or re-randomise.");        
          }     
          else // if received x0 with no more char's after it - Wave Shape 0 variables:
          {
            Serial.println("\n   Wave Shape 0 - Sine Wave variables.");
            Serial.println(  "        y = A*sin(B(x + C)) + D\n");
            Serial.println(  "   x0a = A - Amplitude (default = 1.0)");
            Serial.println(  "   x0b = B - Waveshape Period (default = 1.0)");
            Serial.println(  "   x0c = C - Phase Shift (default = 0.5)");
            Serial.println(  "   x0d = D - Vertical Offset (default = 0.5)");
            Serial.println(  "   x0r     - To randomise all equation variables (i.e x0a to x0d).");
            Serial.println(  "   x0h     - Toggle Sample and Hold on/off (default = 0)");
            Serial.println(  "   x0s     - Sample and Hold fractional length (default = 0.1, range 0.05 to less than 0.5)\n");
            Serial.println(  "             Current values: ");
            Serial.print(    "   A = "); Serial.print(WS0a); Serial.print(", B = "); Serial.print(WS0b); Serial.print(", C = "); Serial.print(WS0c); Serial.print(", D = "); Serial.println(WS0d);
            Serial.print(    "   Sample and Hold: x0h = "); Serial.print(WS0sh); Serial.print(", x0s = "); Serial.print(WS0s); Serial.println("");
          }
        }
        else if (UserChars[1] == '1')    // if recieved 1 - Wave Shape 1 variables:
        {
          if (UserChars[2] == 'a') // if received x1a
          {
            WS1a = UserInput;
            Serial.print("   Wave Shape 1 A - Triangle slope changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'b') // if received x1b
          {
            WS1b = UserInput;
            Serial.print("   Wave Shape 1 B - Triangle vertical shift changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'h') // if received x1h - Sample and Hold toggle
          {
            if ((UserInput == 1) || (UserInput == 0))
            {
              WS1sh = UserInput;
              Serial.print("   Sample and Hold toggle on/off changed to: ");
              Serial.println(UserInput);  
            }
            else Serial.println("   Not valid input for x1h. Must enter either 1 or 0 only (e.g. 1x1h).\n"); 
          }
          else if (UserChars[2] == 's') // if received x1s - Sample and Hold fraction
          {
              if ((UserInput >= 0.05) && (UserInput < 0.5))
              {
                WS1s = UserInput;
                Serial.print("   Sample and Hold fraction changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input for x1s. Must enter value between 0.05 and less than 0.5 only (e.g. 0.2x1s).\n");  
          }  
          else if (UserChars[2] == 'r')   //if recieved x1r
          {
            //randomise all variables
            // random number set for each variable acceptable range  //NEEDS TWEAKING
            WS1a = random(-300,301)/100.0; // This is fixed from -3 to 3
            WS1b = random(-100,101)/100.0; // This is fixed from -1 to 1
            Serial.println("   Wave Shape 1 - Triangle Wave all equation parameters randomised."); 
            Serial.println("   Note this may result in wave off scale hi/low (i.e. flat line). Tweak variables or re-randomise.");        
          }            
          else // if received x1 with no more char's after it - Wave Shape 1 variables:
          {
            Serial.println("\n   Wave Shape 1 - Triangle Wave variables.");
            Serial.println(  "             y = A/2+B-A*x\n"); // y = Ax+B\n");
            Serial.println(  "   x1a = A - Slope (default = 1.0)");
            Serial.println(  "   x1b = B - Fractional Vertical Shift value usually between 0.0 to 1.0 (default = 0.5)");
            Serial.println(  "   x1r     - To randomise all equation variables (i.e x1a and x1b).");
            Serial.println(  "   x1h     - Toggle Sample and Hold on/off (default = 0)");
            Serial.println(  "   x1s     - Sample and Hold fractional length (default = 0.1, range 0.05 to less than 0.5)");
            Serial.println(  "   Hint: Try A = -1 to invert triangle. Also try A = 2 and B = 0.75 for trapezoid wave.\n");
            Serial.println(  "             Current values: ");
            Serial.print(    "   A = "); Serial.print(WS1a); Serial.print(", B = "); Serial.println(WS1b);
            Serial.print(    "   Sample and Hold: x1h = "); Serial.print(WS1sh); Serial.print(", x1s = "); Serial.print(WS1s); Serial.println("");
          }
        }
        else if (UserChars[1] == '3')    // if recieved 3 - Wave Shape 3 variables:
        {
          if (UserChars[2] == 'a')       // if recieved x3a
          {
            if (UserChars[3] == '0')     // if recieved x3a0
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a0 = UserInput;
                Serial.print("   Wave Shape 3 a0 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '1')    // if recieved x3a1
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a1 = UserInput;
                Serial.print("   Wave Shape 3 a1 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '2')    // if recieved x3a2
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a2 = UserInput;
                Serial.print("   Wave Shape 3 a2 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '3')    //if recieved x3a3
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a3 = UserInput;
                Serial.print("   Wave Shape 3 a3 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '4')    //if recieved x3a4
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a4 = UserInput;
                Serial.print("   Wave Shape 3 a4 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '5')    //if recieved x3a5
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a5 = UserInput;
                Serial.print("   Wave Shape 3 a5 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '6')    //if recieved x3a6
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a6 = UserInput;
                Serial.print("   Wave Shape 3 a6 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '7')    //if recieved x3a7
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a7 = UserInput;
                Serial.print("   Wave Shape 3 a7 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '8')    //if recieved x3a8
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a8 = UserInput;
                Serial.print("   Wave Shape 3 a8 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == '9')    //if recieved x3a9
            {
              if ((UserInput >= 0) && (UserInput <= 1.0))
              {
                WS3a9 = UserInput;
                Serial.print("   Wave Shape 3 a9 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0.\n");
            }
            else if (UserChars[3] == 'r')   //if recieved x3ar
            {
              //randomise all a values
              // random number from 0 to 100 divided by 100 for random value between 0.0 and 1.0
              WS3a0 = random(101)/100.0;
              WS3a1 = random(101)/100.0;
              WS3a2 = random(101)/100.0;
              WS3a3 = random(101)/100.0;
              WS3a4 = random(101)/100.0;
              WS3a5 = random(101)/100.0;
              WS3a6 = random(101)/100.0;
              WS3a7 = random(101)/100.0;
              WS3a8 = random(101)/100.0;
              WS3a9 = random(101)/100.0;
              Serial.print("   Wave Shape 3 - Staircase Wave all a values randomised.");          
            }
            else Serial.println("   Not valid input - No number after 'a' detected.\n");
          }
          else if (UserChars[2] == 'b') // if recieved x3b
          {
            if (UserChars[3] == '0')     // if recieved x3b0
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput < WS3b1)))
              {
                WS3b0 = UserInput;
                Serial.print("   Wave Shape 3 b0 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and less than next x range value.");
            }
            else if (UserChars[3] == '1')    // if recieved x3b1
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput < WS3b2)))
              {
                WS3b1 = UserInput;
                Serial.print("   Wave Shape 3 b1 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and less than next x range value.");
            }
            else if (UserChars[3] == '2')   // if recieved x3b2
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput < WS3b3)))
              {
                WS3b2 = UserInput;
                Serial.print("   Wave Shape 3 b2 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and less than next x range value.");
            }
            else if (UserChars[3] == '3')   // if recieved x3b3
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b2)))
              {
                WS3b3 = UserInput;
                Serial.print("   Wave Shape 3 b3 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == '4')   // if recieved x3b4
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b3)))
              {
                WS3b4 = UserInput;
                Serial.print("   Wave Shape 3 b4 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == '5')   // if recieved x3b5
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b4)))
              {
                WS3b5 = UserInput;
                Serial.print("   Wave Shape 3 b5 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == '6')   // if recieved x3b6
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b5)))
              {
                WS3b6 = UserInput;
                Serial.print("   Wave Shape 3 b6 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == '7')   // if recieved x3b7
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b6)))
              {
                WS3b7 = UserInput;
                Serial.print("   Wave Shape 3 b7 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == '8')   // if recieved x3b8
            {
              if ((UserInput == 0) || ((UserInput > 0) && (UserInput <= 1.0) && (UserInput > WS3b7)))
              {
                WS3b8 = UserInput;
                Serial.print("   Wave Shape 3 b8 - Staircase Wave changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input. Must enter number between 0.0 and 1.0 and is greater than the previous x range value.");
            }
            else if (UserChars[3] == 'r')   //if recieved x3br
            {
              //randomise all b values
              // random number from 0 to 100 divided by 100 for random value between 0.0 and 1.0
              // ensure no overlap and increasing b values.
              RandNum = random(0,101);
              WS3b0 = RandNum/100.0;
              Nextb = RandNum + 1;
              if (WS3b0 < 0.99)
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b1 = RandNum/100.0;
                Nextb = RandNum + 1;
              }              
              else WS3b1 = 0; // turn off segment
              if (WS3b1 < 0.99 && WS3b1 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b2 = RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b2 = 0; // turn off segment
              if (WS3b2 < 0.99 && WS3b2 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b3 =RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b3 = 0; // turn off segment
              if (WS3b3 < 0.99 && WS3b3 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b4 = RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b4 = 0; // turn off segment
              if (WS3b4 < 0.99 && WS3b4 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b5 = RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b5 = 0; // turn off segment
              if (WS3b5 < 0.99 && WS3b5 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b6 = RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b6 = 0; // turn off segment
              if (WS3b6 < 0.99 && WS3b6 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b7 = RandNum/100.0;
                Nextb = RandNum + 1;
              }
              else WS3b7 = 0; // turn off segment
              if (WS3b7 < 0.99 && WS3b7 != 0) 
              {
                RandNum = random(Nextb,101); // ensures no overlap (i.e. starts from previous random number + 1)
                WS3b8 = RandNum/100.0;
              }
              else WS3b8 = 0; // turn off segment
              Serial.print("   Wave Shape 3 - Staircase Wave all b values randomised.");          
            }                
            else Serial.println("   Not valid input - No number after 'b' detected.\n");
          }
          else  // if received x3 with no more char's after it - Wave Shape 3 variables:
          {
            Serial.println("\n   Wave Shape 3 - Staircase wave variables.");
            Serial.println(  "   Enter values between 0.0 and 1.0 for all variables.");         
            Serial.println(  "   Values for x range must be increasing from left to right (see default values for example).");
            Serial.println(  "          y = a0, a1, a2, a3, a4, a5, a6, a7, a8, a9");
            Serial.println(  "          x range = b0, b1, b2, b3, b4, b5, b6, b7, b8\n");
            Serial.println(  "          x3a0, x3a1, x3a2, x3a3, x3a4, x3a5, x3a6, x3a7, x3a8, x3a9 (default values = 0.000, 0.111, 0.222, 0.333, 0.444, 0.555, 0.666, 0.777, 0.888, 1.000)");
            Serial.println(  "          x3b0, x3b1, x3b2, x3b3, x3b4, x3b5, x3b6, x3b7, x3b8 (default values = 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9. x3b9 is not used.)");
            Serial.println(  "   Maximum 10 segments. To turn off a segment set b to 0 (e.g. use command 0x3b0)");
            Serial.println(  "   To randomise all a variables (i.e a0 to a9) enter command x3ar.");
            Serial.println(  "   To randomise all b variables (i.e b0 to b8) enter command x3br.\n");          
            Serial.println(  "          Current values: ");
            Serial.print(  "   y: a0 = "); Serial.print(WS3a0); Serial.print(", a1 = "); Serial.print(WS3a1); Serial.print(", a2 = "); Serial.print(WS3a2); Serial.print(", a3 = "); Serial.print(WS3a3); Serial.print(", a4 = "); Serial.print(WS3a4); Serial.print(", a5 = "); Serial.print(WS3a5); Serial.print(", a6 = "); Serial.print(WS3a6); Serial.print(", a7 = "); Serial.print(WS3a7); Serial.print(", a8 = "); Serial.print(WS3a8); Serial.print(", a9 = "); Serial.print(WS3a9); Serial.println("");
            Serial.print(  "   x: b0 = "); Serial.print(WS3b0); Serial.print(", b1 = "); Serial.print(WS3b1); Serial.print(", b2 = "); Serial.print(WS3b2); Serial.print(", b3 = "); Serial.print(WS3b3); Serial.print(", b4 = "); Serial.print(WS3b4); Serial.print(", b5 = "); Serial.print(WS3b5); Serial.print(", b6 = "); Serial.print(WS3b6); Serial.print(", b7 = "); Serial.print(WS3b7); Serial.print(", b8 = "); Serial.print(WS3b8); Serial.println("");
          }
        }
        else if (UserChars[1] == '4')    // if recieved 4 - Wave Shape 4 variables:
        {
          if (UserChars[2] == 'a') // if recieved x4a
          {
            WS4a = UserInput;
            Serial.print("   Wave Shape 4 - Half Square Root function A changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'b')    // if recieved x4b
          {
            WS4b = UserInput;
            Serial.print("   Wave Shape 4 - Half Square Root function B changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'c')    // if recieved x4c
          {
            WS4c = UserInput;
            Serial.print("   Wave Shape 4 - Half Square Root function C Vertical Shift changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'u')    //if recieved x4u
          {
            if ((UserInput == 1) || (UserInput == 0))
            {
              WS4up = UserInput;
              Serial.print("   Wave Shape 4 - Half Square Root function Up changed to: ");
              Serial.println(UserInput);
            }
            else Serial.println("   Not valid input for x4u. Must enter either 1 or 0 only (e.g. 1x4u).\n");
          }
          else if (UserChars[2] == 'd')    //if recieved x4d
          {
            if ((UserInput == 1) || (UserInput == 0))
            {
              WS4dn = UserInput;
              Serial.print("   Wave Shape 4 - Half Square Root function Down changed to: ");
              Serial.println(UserInput);
            }
            else Serial.println("   Not valid input for x4d. Must enter either 1 or 0 only (e.g. 0x4d).\n");
          }
          else if (UserChars[2] == 'h') // if received x4h - Sample and Hold toggle
          {
              if ((UserInput == 1) || (UserInput == 0))
              {
                WS4sh = UserInput;
                Serial.print("   Sample and Hold toggle on/off changed to: ");
                Serial.println(UserInput);   
              }
              else Serial.println("   Not valid input for x4h. Must enter either 1 or 0 only (e.g. 1x4h).\n");
          }
          else if (UserChars[2] == 's') // if received x4s - Sample and Hold fraction
          {
              if ((UserInput >= 0.05) && (UserInput < 0.5))
              {
                WS4s = UserInput;
                Serial.print("   Sample and Hold fraction changed to: ");
                Serial.println(UserInput);
              }
              else Serial.println("   Not valid input for x4s. Must enter value between 0.05 and less than 0.5 only (e.g. 0.2x4s).\n");
          }
          else if (UserChars[2] == 'r')   //if recieved x4r
          {
            //randomise all variables
            // random number set for each variable acceptable range  //NEEDS TWEAKING
            WS4a = random(100,1000)/10.0;  // This is fixed from 10 to 100
            WS4b = random(-5000,5001)/100.0; // This is fixed from -50 to 50
            WS4c = random(-200,201)/100.0; // This is fixed from -2 to 2
            WS4up = random(0,2); // This is fixed either 0 or 1
            if (WS4up == 0) WS4dn = 1; // both WS4up and WS4dn cannot be 0
            else WS4dn = random(0,2); // This is fixed either 0 or 1
            Serial.println("   Wave Shape 4 - Half Square Root Wave all equation parameters randomised."); 
            Serial.println("   Note this may result in wave off scale hi/low (i.e. flat line). Tweak variables or re-randomise.");        
          }        
          else  // if recieved x4 with no more char's after it - Wave Shape 4 variables:
          {
            Serial.println("\n   Wave Shape 4 - Half Square Root function wave variables.");
            Serial.println(  "          Upper portion: y = A(x + B)^0.5 + C");
            Serial.println(  "          Lower portion: y = -(A(x + B)^0.5 + C)\n");
            Serial.println(  "   x4a = A (default = 50.0)");
            Serial.println(  "   x4b = B (default = 0.0)");
            Serial.println(  "   x4c = C Vertical Shift (default = 0.0)");
            Serial.println(  "   x4u = Upper portion of the function (default = 1). Value must be 1 or 0 only (boolean).");
            Serial.println(  "   x4d = Lower portion of the function (default = 1). Value must be 1 or 0 only (boolean).");
            Serial.println(  "   x4r     - To randomise all equation variables (i.e x4a to x4c including x4u and x4d).");            
            Serial.println(  "   x4h     - Toggle Sample and Hold on/off (default = 0)");
            Serial.println(  "   x4s     - Sample and Hold fractional length (default = 0.1, range 0.05 to less than 0.5)");
            Serial.println(  "   Hint: Try A = -50 and C = 0.5 and change values of x4u and x4d.\n");
            Serial.println(  "   Current values: ");
            Serial.print(  "   A = "); Serial.print(WS4a); Serial.print(", B = "); Serial.print(WS4b); Serial.print(", C = "); Serial.print(WS4c); Serial.print(", Up = "); Serial.print(WS4up); Serial.print(", Down = "); Serial.println(WS4dn);
            Serial.print(  "   Sample and Hold: x4h = "); Serial.print(WS4sh); Serial.print(", x4s = "); Serial.print(WS4s); Serial.println("");
          }
        }
        else if (UserChars[1] == '5')   // if recieved 5 - Wave Shape 5 variables:
        {
          if (UserChars[2] == 'a')   // if recieved x5a
          {
            WS5a = UserInput;
            Serial.print("   Wave Shape 5 - Full Square Root function A changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'b')    // if recieved x5b
          {
            WS5b = UserInput;
            Serial.print("   Wave Shape 5 - Full Square Root function B changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'c')   // if recieved x5c
          {
            WS5c = UserInput;
            Serial.print("   Wave Shape 5 - Full Square Root function C Vertical Shift changed to: ");
            Serial.println(UserInput);
          }
          else if (UserChars[2] == 'h') // if received x5h - Sample and Hold toggle
          {
            if ((UserInput == 1) || (UserInput == 0))
            {
              WS5sh = UserInput;
              Serial.print("   Sample and Hold toggle on/off changed to: ");
              Serial.println(UserInput);      
            }
            else Serial.println("   Not valid input for x5h. Must enter either 1 or 0 only (e.g. 1x5h).\n");
          }
          else if (UserChars[2] == 's') // if received x5s - Sample and Hold fraction
          {
              if ((UserInput >= 0.05) && (UserInput < 0.5))
              {
                WS5s = UserInput;
                Serial.print("   Sample and Hold fraction changed to: ");
                Serial.println(UserInput);                
              }
              else Serial.println("   Not valid input for x5s. Must enter value between 0.05 and less than 0.5 only (e.g. 0.2x5s).\n");
          }           
          else if (UserChars[2] == 'r')   //if recieved x5r
          {
            //randomise all variables
            // random number set for each variable acceptable range  //NEEDS TWEAKING
            WS5a = random(100,1000)/10.0;  // This is fixed from 10 to 100
            WS5b = random(-5000,5001)/100.0; // This is fixed from -50 to 50
            WS5c = random(-200,201)/100.0; // This is fixed from -2 to 2
            Serial.println("   Wave Shape 5 - Full Square Root Wave all equation parameters randomised."); 
            Serial.println("   Note this may result in wave off scale hi/low (i.e. flat line). Tweak variables or re-randomise.");        
          }      
          else  // if revieved x5 with no more char's after it - Wave Shape 5 variables:
          {
            Serial.println("\n   Wave Shape 5 - Full Square Root function wave variables.");
            Serial.println(  "          y = A(x + B)^0.5 + C\n");
            Serial.println(  "   x5a = A (default = 50.0)");
            Serial.println(  "   x5b = B (default = 0.0)");
            Serial.println(  "   x5c = C Vertical Shift (default = 0.0)");
            Serial.println(  "   x5r     - To randomise all equation variables (i.e x5a to x5c)."); 
            Serial.println(  "   x5h     - Toggle Sample and Hold on/off (default = 0)");
            Serial.println(  "   x5s     - Sample and Hold fractional length (default = 0.1, range 0.05 to less than 0.5)");
            Serial.println(  "   Hint: Try A = -50, C = 1.\n");
            Serial.println(  "   Current values: ");
            Serial.print(  "   A = "); Serial.print(WS5a); Serial.print(", B = "); Serial.print(WS5b); Serial.print(", C = "); Serial.println(WS5c);
            Serial.print(  "   Sample and Hold: x5h = "); Serial.print(WS5sh); Serial.print(", x5s = "); Serial.print(WS5s); Serial.println("");
          }
        }
        else // if received x followed by line feed or x plus any other invalid or unused characters
        {
          Serial.println("   HELP for Extra parameters for Waves with equations and variables.\n   Type the following, then press enter:\n");
          //      Serial.println("   Type:    x  to list all Extra commands."); // NOT NEEDED - ALREADY USED TO GET HERE
          Serial.println("   Type:   xw  to list Wave shapes.");
          Serial.println("   Type:   x0  to list Wave Shape 0 variables. (Change number for other wave shapes (e.g. x1, x3, x4, etc.). Note x2 is Arbitrary wave with no variables.");
          Serial.println("   Type:   xx  to update all wave tables (e.g. after variable changes completed).\n");
        }
 //       delay(1500);
        // Serial.print("1158   Received = "); Serial.print(UserInput); Serial.println(UserChars);
        for (byte i = 0; i < 4; i++) UserChars[i] = ' '; // Change to characters not used in the x block. A space will go unnoticed so will look better in the serial monitor
        // Serial.print("1160   UserChars[0] = "); Serial.println(UserChars[0]);
      }
//*********************************** END of Block of extra parameter modifying code ***********************************************/
      else if ((UserChars[0] == 't' && !UsingGUI && TimerMode == 0) || (UserChars[0] == 'T' && UsingGUI)) // if entering Timer Mode:
      {
        if (SquareWaveSync) TimerMode = 2; // if SquareWaveSync was HIGH before entering timer mode
        else TimerMode = 1; // if SquareWaveSync was LOW before entering timer mode
        SquareWaveSync = LOW; // stop Synchronized Square wave
        TimerRun = 0;
        REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
        REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
        PWMC_DisableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
        NVIC_DisableIRQ(TC1_IRQn);
        pinMode(7, OUTPUT);
        if (TimerInvert) digitalWrite(7, HIGH);
        else digitalWrite(7, LOW);
        TimeUp = 0;
        if (UsingGUI) Serial.println("TimerMode");
        else
        {
          TC_setup5(); // (use timer in GUI if running)
          Serial.println(String("           ********** Timer Mode **********\n   The current time period is: ") + d + String(" days, ") + h + String(" hours, ") + m + String(" mins, ") + s + String(" secs.\n"));
          Serial.println("   To make changes enter the desired number followed by:\n   d for Days, h for Hours, m for Minutes or s for Seconds\n\n   Then type r to start running. Or type t again to exit.\n");
        }
      }
      else if (TimerMode > 0) // If in Timer mode:
      {
        if (UserChars[0] == 't' ) // if exiting Timer Mode
        {
          if (TimerMode == 2 || TargetFreq < 163) // if SquareWaveSync was HIGH before entering timer mode or low freq
          {
            pinMode(7, INPUT);
            PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC28B_TIOA7, PIO_DEFAULT);  // pin 3
            if (TimerMode == 2) SquareWaveSync = HIGH; // if SquareWaveSync was HIGH before entering timer mode start Synchronized Square wave
            else NVIC_EnableIRQ(TC1_IRQn); // if SquareWaveSync was LOW & freq was low before entering timer mode start Unsynchronized Square wave
          }
          else SetFreqAndDuty(1, 1); // if SquareWaveSync was LOW & freq was high before entering timer mode start Unsynchronized Square wave (PWM)
          TimerSecs  = 0;
          TimerMins  = 0;
          TimerHours = 0;
          TimerDays  = 0;
          TimerMode = 0;
          TimerRun = 0;
          Serial.println("   You have exited the Timer.\n");
        }
        else if (UsingGUI && (UserChars[0] == 'D' || UserChars[0] == 'H' || UserChars[0] == 'M' || UserChars[0] == 'S'))
        {
          if (UserChars[0] == 'D')
          {
            if (TimerRun) TimerDays = int(UserInput);
            else            PeriodD = int(UserInput);
          }
          else if (UserChars[0] == 'H')
          {
            if (TimerRun) TimerHours = int(UserInput);
            else             PeriodH = int(UserInput);
          }
          else if (UserChars[0] == 'M')
          {
            if (TimerRun) TimerMins = int(UserInput);
            else            PeriodM = int(UserInput);
          }
          else if (UserChars[0] == 'S')
          {
            if (TimerRun) TimerSecs = int(UserInput);
            else            PeriodS = int(UserInput);
          }
        }
        else if (!UsingGUI && (UserChars[0] == 'd' || UserChars[0] == 'h' || UserChars[0] == 'm' || UserChars[0] == 's'))
        {
          if      (UserChars[0] == 'd') d = int(UserInput);
          else if (UserChars[0] == 'h') h = int(UserInput);
          else if (UserChars[0] == 'm') m = int(UserInput);
          else if (UserChars[0] == 's') s = int(UserInput);
          Serial.println(String(" * You typed: ") + d + String(" days, ") + h + String(" hours, ") + m + String(" mins, ") + s + String(" secs."));
          if (h > 23 || m > 59 || s > 59) Serial.println("   Sorry but that is invalid. Please try again.\n");
          else Serial.println("   To enter this time period type e\n");
        }
        else if (!UsingGUI && UserChars[0] == 'e') // TIMER PERIOD ADJUSTMENT:
        {
          // PERIOD ADJUSTMENT: in Time Period Entry Mode
          PeriodD = d;
          if (h <= 23) PeriodH = h;
          if (m <= 59) PeriodM = m;
          if (s <= 59) PeriodS = s;
          Serial.println(String("** You entered: ") + PeriodD + String(" days, ") + PeriodH + String(" hours, ") + PeriodM + String(" mins, ") + PeriodS + String(" secs.\n\n   Type r to start Running. Type r again to Reset.\n"));
        }
        else if (UsingGUI && UserChars[0] == 'U')
        {
          TimeUp = 1;
          Serial.println("   Time Up!");
          if (TimerInvert) digitalWrite(7, LOW);
          else digitalWrite(7, HIGH);
        }
        else if (UsingGUI && UserChars[0] == 'R')
        {
          TimerRun = 1;
          Serial.println("   Timer Running...");
        }
        else if (UserChars[0] == 'r')
        {
          TimeUp = 0;
          if (TimerInvert) digitalWrite(7, HIGH); // Restart cycle
          else digitalWrite(7, LOW);
          TimerSecs    = 0; // Reset timers
          TimerMins   = 0;
          TimerHours = 0;
          TimerDays = 0;
          if (UsingGUI) TimerRun = 0; // reset only
          else
          {
            TimerRun = !TimerRun;
            TC_Start(TC1, 1);
          }
          if (TimerRun == 1)
          {
            Serial.print("   Timer Running...");
            if (PeriodD + PeriodH + PeriodM + PeriodS == 0) Serial.println("  WARNING! Timer is set to zero!\n");
            else Serial.println("\n");
          }
          else Serial.println("   Timer Reset!\n");
        }
        else if (UserChars[0] == 'i')
        {
          TimerInvert = false;
          digitalWrite(7, LOW);
          Serial.println("  Positive Timer Mode.\n");
        }
        else if (UserChars[0] == 'I')
        {
          TimerInvert = true;
          digitalWrite(7, HIGH);
          Serial.println("  Negative Timer Mode.\n");
        }
      }
      else if (UserChars[0] == 'h' || UserChars[0] == 'm') // FREQ or PERIOD ADJUSTMENT:  in Hertz or Milliseconds
      {
        //        UserInput = UserInput * pow(10, min(0, -numDecimalPlaces)); // multiply input by 10 to the power of minus the number of decimal places
        //        numDecimalPlaces = -1; // reset
        if (UserChars[0] == 'm') UserInput = 1000 / UserInput; // convert period into freq
        if (UserInput >= 0.00001999 && UserInput <= 42000000)
        {
          if (Control > 0) // sync'ed waves
          {
            PotAdjFreq[1] = 0;
            if (TargetWavePulseWidth > 0) TargetWaveDuty = min(100, 100 * (TargetWavePulseWidth / (1000000 / min(UserInput, 100961.54)))); // convert pulse width input into %
            TargetWaveFreq = min(UserInput, 100961.54);
            SetWaveFreq(1);
          }
          if (Control != 1) // unsync'ed sq.wave
          {
            PotAdjFreq[0] = 0;
            if (TargetPulseWidth > 0) TargetDuty = min(100, 100 * (TargetPulseWidth / (1000000 / min(UserInput, 42000000)))); // convert pulse width input into %
            TargetFreq = UserInput;
            SetFreqAndDuty(1, 1);
            Serial.print("   Unsync'ed Sq.Wave Freq: "); PrintUnsyncedSqWaveFreq(); Serial.print(", Target: "); Serial.print(UserInput, 3);
            Serial.print(" Hz\n   Unsync'ed Sq.Wave Period: ");
            PrintUnsyncedSqWavePeriod(); // PrintUnsyncedSqWaveDuty();
            Serial.print("   Unsync'ed Sq.Wave Duty-cycle: "); Serial.print(ActualDuty); Serial.println(" %\n");
          }
          if ((!SquareWaveSync && TargetFreq < 0.5) || (SquareWaveSync && TargetWaveFreq < 0.5))
          {
            if (millis() - 500 < TouchedTime) // if freq or period entered twice (if double clicked)
            {
              if (SquareWaveSync) // sync'ed waves
              {
                WaveBit = 1; // reset sync'ed waves
                if (TargetWaveDuty == 0)
                {
                  WaveHalf = 0;
                  TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
                }
                else
                {
                  WaveHalf = 1;
                  TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
                }
              }
              else // unsync'ed sq.wave
              {
                TimeIncrement = 0; // reset unsync'ed sq wave
                if (TargetDuty < 100) PeriodHalf = 0;
                else PeriodHalf = 1;
                if (TargetDuty == 0) TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
                else TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
              }
              TC_Start(TC1, 1); // Reset timer
              TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // the counter is reset and the clock is started
              LowFreqDisplay = 1;
              TouchedTime = 0;
            }
            else TouchedTime = millis();
          }
        }
        else if (UserChars[0] == 'h') {
          Serial.print("   ");
          Serial.print(UserInput);
          Serial.println(" Hz is outside required freq range\n");
        }
        else                       {
          Serial.print("   ");
          Serial.print(1 / UserInput);
          Serial.println(" Secs is outside required period range\n");
        }
      }
      else if (UserChars[0] == 'd' || UserChars[0] == 'u') // DUTY CYCLE or PULSE WIDTH ADJUSTMENT: in % or microseconds
      {
        //        UserInput = UserInput * pow(10, min(0, -numDecimalPlaces)); // multiply input by 10 to the power of minus the number of decimal places
        //        numDecimalPlaces = -1; // reset
        if ((UserChars[0] == 'd' && UserInput >= 0 && UserInput <= 100) || UserChars[0] == 'u')
        {
          if (Control > 0) // synchronized waves
          {
            PotAdjDuty[1] = 0;
            if (UserChars[0] == 'u')
            {
              TargetWavePulseWidth = UserInput;
              TargetWaveDuty = 100 * (UserInput / (1000000 / ActualWaveFreq)); // convert pulse width input into %
            }
            else
            {
              TargetWavePulseWidth = 0;
              TargetWaveDuty = UserInput;
            }
            CalculateWaveDuty(0);
            if (MinOrMaxWaveDuty) CalculateWaveDuty(0); // if at 0 or 100% duty-cycle re-calculate (for stability)
            CreateNewWave();
            Serial.print("   Analogue Wave Duty-cycle: "); Serial.print(ActualWaveDuty); Serial.print(" %, Target: "); Serial.print(TargetWaveDuty); Serial.println(" %");
            Serial.print("   Analogue Wave Period: "); PrintSyncedWavePeriod(); Serial.println("\n");
          }
          if (Control != 1) // unsynchronized Square wave
          {
            PotAdjDuty[0] = 0;
            if (UserChars[0] == 'u')
            {
              TargetPulseWidth = UserInput;
              TargetDuty = 100 * (UserInput / (1000000 / ActualFreq)); // convert pulse width input into %
            }
            else
            {
              TargetPulseWidth = 0;
              TargetDuty = UserInput;
            }
            SetFreqAndDuty(0, 1);
            Serial.print("   Unsync'ed Sq.Wave Duty-cycle: "); Serial.print(ActualDuty); Serial.print(" %, Target: "); Serial.print(TargetDuty); Serial.println(" %");
            Serial.print("   Unsync'ed Sq.Wave Period: "); PrintUnsyncedSqWavePeriod(); Serial.println("\n");
          }
        }
      }
      else if (UserChars[0] == 'w') // Change Wave Shape
      {
        if      (WaveShape < NumWS && UserInput == 0) WaveShape++;
        else if (WaveShape > 0 && UserInput == 1) WaveShape--;
        else if (UserInput == 0) WaveShape = 0;
        else if (UserInput == 1) WaveShape = NumWS;
        CreateWaveFull();
        CreateWaveTable();
        CreateNewWave();
        if      (WaveShape == 0) Serial.println("             ********** Sine Wave **********\n");
        else if (WaveShape == 1) Serial.println("             ******** Triangle Wave ********\n");
        else if (WaveShape == 2) Serial.println("             ******** Arbitrary Wave *******\n");
        else if (WaveShape == 3) Serial.println("             ******** Staircase Wave *******\n");
        else if (WaveShape == 4) Serial.println("             **** Half Square Root Wave ****\n");
        else if (WaveShape == 5) Serial.println("             **** Full Square Root Wave ****\n");
      }
      else if (UserChars[0] == 'M') // Min & max duty
      {
        if (UserInput >= 1 && UserInput <= 7) MinMaxDuty = UserInput;
        else MinMaxDuty = 4;
        Serial.print("   You have temporarily set MinMaxDuty to "); Serial.print(MinMaxDuty); Serial.println("  It will be reset to 4 by typing 'M' or 'v')\n");
      }
      else if (UserChars[0] == 'H') // square wave sync delay at High sample rate
      {
        if (UserInput >= 1) Delay1 = min(UserInput, 25);
        else Delay1 = 10; // 10;
        SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty));
        Serial.print("   You have set Delay1 to "); Serial.print(Delay1); Serial.println("  It should be set to 10 (Type 'H')\n");
      }
      else if (UserChars[0] == 'L') // square wave sync delay at Low sample rate
      {
        if (UserInput >= 1) Delay2 = min(UserInput * 0.01, 50);
        else Delay2 = 0.55; // 0.36;
        SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty));
        Serial.print("   You have set Delay2 to "); Serial.print(Delay2 * 100, 0); Serial.println("  It should be set to 55 (Type 'L')\n");
      }
      else if (UserChars[0] == 'D') // square wave sync delay at High sample rate
      {
        if (UserInput >= 50) Delay3 = min(UserInput, 200);
        else Delay3 = 110;
        SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty));
        Serial.print("   You have set Delay3 to "); Serial.print(Delay3, 0); Serial.println("  It should be set to 110 (Type 'D')\n");
      }
      else if (UserChars[0] == 'v' && TimerMode == 0) // toggle between Viewing sync'ed square wave or unsync'ed square wave
      {
        ToggleSquareWaveSync();
      }
      else if (PotsEnabled && UserChars[0] == 'r') // Range of Freq / Period Pots
      {
        if (Control > 0) // synchronized waves range:
        {
          if (Range[1] < 10000) Range[1] *= 10;
          else Range[1] = 1;
          Serial.print("   Synchronized Waves Pot Freq Range: x "); Serial.println(Range[1]);
        }
        if (Control != 1) // unsynchronized sq.wave range:
        {
          if (Range[0] < 10000) Range[0] *= 10;
          else Range[0] = 1;
          Serial.print("   Unsync'ed Sq.Wave Pot Freq Range: x "); Serial.println(Range[0]);
        }
        SwitchPressedTime = millis();
      }
      else if (PotsEnabled && UserChars[0] == 'R') // Range of Duty-cycle / Pulse Width Pots
      {
        if (Control > 0) // synchronized waves range:
        {
          if (Range[3] < 10000) Range[3] *= 10;
          else Range[3] = 1;
          Serial.print("   Synchronized Waves Pot Pulse Width Range: x "); Serial.println(Range[3]);
        }
        if (Control != 1) // unsynchronized sq.wave range:
        {
          if (Range[2] < 10000) Range[2] *= 10;
          else Range[2] = 1;
          Serial.print("   Unsync'ed Sq.Wave Pot Pulse Width Range: x "); Serial.println(Range[2]);
        }
        SwitchPressedTime = millis();
      }
      else if (PotsEnabled && UserChars[0] == 'f' && TimerMode == 0) // toggle between pot controlling Freq of wave, or period of wave
      {
        if (Control > 0)
        {
          PotPeriodMode[1] = !PotPeriodMode[1]; // toggle PotPeriodMode for synch'ed waves
          Serial.print("   Synchronized Waves Pot Mode is ");
          if      (PotPeriodMode[1] == 0) Serial.println("Freq");
          else if (PotPeriodMode[1] == 1) Serial.println("Period");
        }
        if (Control != 1)
        {
          PotPeriodMode[0] = !PotPeriodMode[0]; // toggle PotPeriodMode for unsynch'ed wave
          Serial.print("   Unsynchronized Wave Pot Mode is ");
          if      (PotPeriodMode[0] == 0) Serial.println("Freq");
          else if (PotPeriodMode[0] == 1) Serial.println("Period");
        }
        Serial.println("");
        SwitchPressedTime = millis();
      }
      else if (UserChars[0] == 'e') // Exact Freq Mode - low resolution at 50% duty-cycle only & with dithering synchronized square wave
      {
        ToggleExactFreqMode();
      }
      else if (UserChars[0] == ' ' && TimerMode == 0) // space-bar - toggle control between sync'ed waves & unsync'ed sq.wave
      {
        if (Control == 1)
        {
          Control = 0;
          Serial.println("   CONTROL >> Unsync'ed Sq.Wave\n");
        }
        else
        {
          Control = 1;
          Serial.println("   CONTROL >> Analogue Wave\n");
        }
      }
      else if (UserChars[0] == 'b' && TimerMode == 0) // control both analogue wave & Unsync'ed sq.wave together
      {
        Control = 2;
        Serial.println("   CONTROL >> Both Waves\n");
      }
      else if (UserChars[0] == 's' && TimerMode == 0) // Sweep freq function
      {
        //        UserInput = UserInput * pow(10, min(0, -numDecimalPlaces)); // multiply input by 10 to the power of minus the number of decimal places
        //        numDecimalPlaces = -1; // reset
        if (SweepStep > 0 && SweepStep != 3 && SweepStep != 4 && UserInput == 0) // cancel / delete entry & stop sweep if running
        {
          SweepStep = 0;
          Serial.println("   Sweep cancelled\n");
          return;
        }
        if (SweepStep == 0)
        {
          if (!UsingGUI)
          {
            Serial.println("       ************ SWEEP FREQUENCY FUNCTION ************");
            Serial.println("   Please type the minimum sweep frequency in Hz followed by s\n   (Typing s again without entering a number first will cancel)");
          }
          SweepStep = 1;
        }
        if (SweepStep == 1 && UserInput > 0)
        {
          SweepMinFreq = UserInput;
          SweepStep = 2;
          if (!UsingGUI)
          {
            Serial.print("   The minimum sweep frequency is "); Serial.print(SweepMinFreq); Serial.println(" Hz");
            Serial.println("   Please type the maximum sweep frequency in Hz followed by s");
          }
        }
        else if (SweepStep == 2 && UserInput > 0)
        {
          if (UserInput > SweepMinFreq)
          {
            SweepMaxFreq = UserInput;
            SweepStep = 3;
            if (!UsingGUI)
            {
              Serial.print("   The maximum sweep frequency is "); Serial.print(SweepMaxFreq); Serial.println(" Hz");
              Serial.println("   Please type the sweep rise time in seconds followed by s");
            }
          }
          else if (!UsingGUI)
          {
            Serial.print("\n   The maximum sweep frequency must be higher than the minimum entered which was "); Serial.print(SweepMinFreq);
            Serial.println(" Hz\n   Please re-enter the maximum sweep frequency\n");
          }
        }
        else if (SweepStep == 3 && UserInput >= 0)
        {
          SweepRiseTime = UserInput;
          SweepStep = 4;
          if (!UsingGUI)
          {
            Serial.print("   The sweep rise time is "); Serial.print(SweepRiseTime); Serial.println(" Secs");
            Serial.println("   Please type the sweep fall time in seconds followed by s");
          }
        }
        else if (SweepStep == 4 && UserInput >= 0)
        {
          if (UserInput > 0 || SweepRiseTime > 0)
          {
            SweepFallTime = UserInput;
            SweepStep = 5;
            if (!UsingGUI)
            {
              Serial.print("   The sweep fall time is "); Serial.print(SweepFallTime); Serial.println(" Secs");
              Serial.print("   To repeat this freq sweep the following string can be entered:\n\n   ");
              Serial.println(SweepMinFreq + String("s") + SweepMaxFreq + String("s") + SweepRiseTime + String("s") + SweepFallTime + String("s\n"));
              Serial.println("   Type s again to stop the sweep\n");
              delay(2000); // time to read above message
            }
            SweepFreq();
          }
          else if (!UsingGUI)
          {
            Serial.print("\n   The Sweep Rise Time and Fall Time can't both be zero");
            Serial.println("\n   Please re-enter the Sweep Fall Time\n");
          }
        }
      }
      else if (UserChars[0] == 'p') PotsEnabled = !PotsEnabled; // enable / disable Pots
      else if (UserChars[0] == 'P')
      {
        if (Control > 0) // synchronized waves range:
        {
          PotPulseWidth[1] = !PotPulseWidth[1]; // enable / disable pulse width input from pot instead of duty-cycle %
        }
        if (Control != 1) // unsynchronized waves range:
        {
          PotPulseWidth[0] = !PotPulseWidth[0]; // enable / disable pulse width input from pot instead of duty-cycle %
        }
        SwitchPressedTime = millis();
      }
      else if (UserChars[0] == '?') // Help
      {
        Serial.println("   HELP:  Type the following, then press enter:");
        Serial.println("   Type:   w   to cycle through the analogue Wave shape.");
        Serial.println("   Type:  1w   to cycle through the analogue Wave shape backwards.");
        Serial.println("   Type:   a   to create a new Arbitrary wave. Follow on-screen instructions.");
        Serial.println("   Type:   v   to toggle between Viewing synchronized or unsynchronized waves.");
        Serial.println("   Type:  ' '  [spacebar] to toggle between controlling synchronized waves or unsync'ed square wave.");
        Serial.println("   Type:   b   to control Both synchronized and unsynchronized waves simultaneously.");
        Serial.println("   Type:   h   to change frequency of wave/s, type required frequency in Hz followed by h.");
        Serial.println("   Type:   m   to change period of wave/s, type required period in Milliseconds followed by m.");
        Serial.println("   Type:   d   to change Duty-cycle type required percentage duty-cycle (0 - 100) followed by d.");
        Serial.println("   Type:   u   to set pulse width. Type required pulse width in microseconds followed by u.");
        Serial.println("   Type:   e   to toggle on/off Exact Freq Mode for synchronized waves, eliminating freq steps.");
        Serial.println("   Type:   s   to set up the frequency Sweep feature. Follow on-screen instructions.");
        Serial.println("   Type:   t   to enter or exit the Timer. Follow on-screen instructions.");
        Serial.println("   Type:   p   to enable (or disable) pots.");
        Serial.println("   Type:   f   to toggle between pot controlling Freq of wave, or period of wave.");
        Serial.println("   Type:   P   to toggle between pot controlling duty-cycle Percent, or Pulse width of wave.");
        Serial.println("   Type:   r   to toggle Range of the frequency/period pot: x 1, x10, x100, x1000 & x 10000.");
        Serial.println("   Type:   R   to toggle Range of the pulse width pot: x 1, x10, x100, x1000 & x 10000.");
        Serial.println("   Type:   x   to access Extra commands for each wave shape.");
        Serial.println("   Type:   S   to display the current status.\n");
      }
      else if (UserChars[0] == 'G' && millis() < 2000) // if connecting to GUI: MOD changed from 1000 to: 2000
      {
        //       Serial.println(millis());
        UsingGUI = 1;
        Serial.println("Hello GUI");
      }
      else if (UserChars[0] == 'S') // CHANGED FROM if Enter pressed
      {
        //      if (millis() - 500 < TouchedTime) // if Enter pressed twice - display status
        //       {
        Serial.println("\n   STATUS:");
        if (TimerMode > 0) Serial.println(String("   Timer period is set to: ") + PeriodD + String(" days, ") + PeriodH + String(" hours, ") + PeriodM + String(" mins, ") + PeriodS + String(" secs."));
        if  (ExactFreqMode) Serial.print("   Exact Mode is ON ");
        else                Serial.print("   Exact Mode is OFF");
        if (SquareWaveSync) Serial.println("  Waves are Synchronized");
        else                Serial.println("  Waves are Unsynchronized");
        //          } ////e
        if (Control > 0 && TimerMode == 0) Serial.print(">> Analogue Wave Freq: ");
        else  Serial.print("   Analogue Wave Freq: ");
        PrintSyncedWaveFreq();
        Serial.print(", Analogue Wave Duty-cycle: "); Serial.print(ActualWaveDuty); Serial.println(" %");
        Serial.print("   Analogue Wave Period: ");
        PrintSyncedWavePeriod();
        if (Control != 1 && TimerMode == 0) Serial.print(">> Unsync'ed Sq.Wave Freq: ");
        else  Serial.print("   Unsync'ed Sq.Wave Freq: ");
        PrintUnsyncedSqWaveFreq();
        Serial.print(", Unsync'ed Sq.Wave Duty-cycle: "); Serial.print(ActualDuty); Serial.println(" %");
        Serial.print("   Unsync'ed Sq.Wave Period: ");
        PrintUnsyncedSqWavePeriod();
        if (PotsEnabled)
        {
          Serial.print("   Pot Mode - Analog Wave: ");
          if      (PotPeriodMode[1] == 0) Serial.print("Freq ");
          else if (PotPeriodMode[1] == 1) Serial.print("Period");
          Serial.print("  Unsync'ed Sq.Wave: ");
          if      (PotPeriodMode[0] == 0) Serial.println("Freq");
          else if (PotPeriodMode[0] == 1) Serial.println("Period");
          Serial.print("   Freq Pot Range - Analog Wave: x "); Serial.print(Range[1]); Serial.print("  Unsync'ed Sq.Wave: x "); Serial.println(Range[0]);
          if (PotPulseWidth) Serial.print("   Pulse Width Range - Analog Wave: x "); Serial.print(Range[3]); Serial.print("  Unsync'ed Sq.Wave: x "); Serial.println(Range[2]);
        }
        else if (millis() < 180000) Serial.println("   Press 'p' to enable / disable pots"); // only shown for 1st 3 minutes
        Serial.println();
        TouchedTime = 0;
        //       }
        //       else TouchedTime = millis();
      }
      UserInput = 0; // reset to 0 ready for the next sequence of digits
      UserChars[0] = 'z';
    }
  }

  //************************************************************************
  if (TimerRun == 1 || (TimerMode == 0 && ((!SquareWaveSync && TargetFreq < 0.5) || (SquareWaveSync && TargetWaveFreq < 0.5)))) // if timer running or if sq.wave is less than 0.5Hz
  {
    bool waveHalfStart = 0;
    if ((SquareWaveSync && WaveBit < OldTime) || (!SquareWaveSync && TimeIncrement < OldTime)) waveHalfStart = 1;
    byte sec = TC1->TC_CHANNEL[1].TC_CV / 656250; // remaining seconds not in timer mode. 656250(Hz) * 60(seconds) = 39375000 clocks per minute
    if ((sec != OldSec) || waveHalfStart) // if remaining seconds have changed or at start of wavehalf then update
    {
      if (!waveHalfStart && LowFreqDisplay == 1)
      {
        SecChanged = 1;
        if (TimerSecs == 59)
        {
          if (TimerMins < 59) TimerMins++;
          else
          {
            TimerMins = 0;
            if (TimerHours < 23) TimerHours++;
            else
            {
              TimerHours = 0;
              TimerDays++;
            }
          }
        }
        LowFreqDisplay = 2;
      }
      byte oldTimerSecs = TimerSecs;
      if (waveHalfStart)
      {
        TimerSecs = 0;
        OldSec = 0;
        sec = 0;
      }
      else TimerSecs = sec;
      if (TimerMode > 0 && !UsingGUI) // if in timer mode & not using GUI
      {
        if (TimeUp == 0 && TimerDays >= PeriodD && TimerHours >= PeriodH && TimerMins >= PeriodM && TimerSecs >= PeriodS && PeriodD + PeriodH + PeriodM + PeriodS > 0) // if time elapsed & if not set to zero
        {
          if (TimerInvert) digitalWrite(7, LOW);
          else digitalWrite(7, HIGH);
          TimeUp = 1; // keeps output high (or low) after time is up in timer mode
          Serial.println("   Time Is Up!\n");
        }
      }
      else if (((!SquareWaveSync && TargetFreq < 0.5) || (SquareWaveSync && TargetWaveFreq < 0.5)) && TimerMode == 0 && a == 0) // a = 1 if arbitrary wave being received
      {
        if (oldTimerSecs + TimerMins + TimerHours + TimerDays > 0 && waveHalfStart) // if at start of wave half
        {
          if ((SquareWaveSync && (WaveHalf || AnaPulseWidth < 1000)) || !SquareWaveSync) // if at start of period
          {
            TimerSecs  = 0; // Reset time at start of each cycle
            TimerMins  = 0;
            TimerHours = 0;
            TimerDays  = 0;
            LowFreqDisplay = 2;
            delay(10); // slightly late sync of timer ensures period resets reliably next time
            TC_Start(TC1, 1); // Reset timer
          }
        }
      }
      if (UsingGUI && TimerMode == 0 && LowFreqDisplay == 2 && a == 0) Serial.print("INFO>");
      if (!UsingGUI && TimerMode > 0) Serial.println(String("   Time Elapsed: ") + TimerDays + String(" days, ") + TimerHours + String(" hours, ") + TimerMins + String(" mins, ") + TimerSecs + String(" secs\n"));
      else if (SweepStep == 0 && TimerMode == 0 && LowFreqDisplay == 2 && a == 0) Serial.println(String("   ") + TimerHours + String(" hours, ") + TimerMins + String(" mins, ") + TimerSecs + String(" secs from start of period\n"));
      if (LowFreqDisplay == 2) LowFreqDisplay = 1; // prevents half-wave being displayed
      if (SquareWaveSync) OldTime = WaveBit;
      else OldTime = TimeIncrement;
      SecChanged = 0;
    }
    OldSec = sec;
  }
  else if (TimerMode == 0) LowFreqDisplay = 0;
  // (To use 64 bit numbers (for Increment[] etc) instead of dithering a slower sample rate (in slow mode) would be needed)
  if (DitherTime > 0 && millis() >= DitherTime && TargetWaveFreq < 0.1) // add increment dithering for very low (intermediate) freq compensation:
  {
    DitherTime = millis() + 100; // how often to dither
    int numDithPoints = min(1 / TargetWaveFreq, 1000); // number of points to dither over
    if (DitherPoint < numDithPoints - 1) DitherPoint++; // current Dither point number
    else DitherPoint = 0;
    if ((round(FreqIncrmt[0] * numDithPoints) % numDithPoints) + DitherPoint > numDithPoints - 1) Increment[0] = int(FreqIncrmt[0]) + 1; // add dither to 1st half of wave Increment[0] if required
    else Increment[0] = int(FreqIncrmt[0]); // add no dither
    if ((round(FreqIncrmt[1] * numDithPoints) % numDithPoints) + DitherPoint > numDithPoints - 1) Increment[1] = int(FreqIncrmt[1]) + 1; // add dither to 2nd half of wave Increment[1] if required
    else Increment[1] = int(FreqIncrmt[1]); // add no dither
  }
  else if (DitherTime > 0 && TargetWaveFreq >= 0.1) DitherTime = 0;
}

void ToggleExactFreqMode()
{
  SyncDelay = 0; // remove square wave sync delay for stability in case of sudden change to high freq
  ExactFreqMode = !ExactFreqMode;
  if (ExactFreqMode) Serial.println("   Exact Mode is ON ");
  else Serial.println("   Exact Mode is OFF");
  if (ExactFreqMode && TargetWaveDuty != 50) ExactFreqDutyNot50 = 1;
  else ExactFreqDutyNot50 = 0; ////e
  SetWaveFreq(1);
  SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty)); // re-apply square wave sync delay
}

void ToggleSquareWaveSync()
{
  if (!SquareWaveSync)
  {
    NVIC_DisableIRQ(TC1_IRQn);
    PWMC_DisableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
    pinMode(7, INPUT);
    PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC28B_TIOA7, PIO_DEFAULT);
    SquareWaveSync = HIGH; // start Synchronized Square wave
    MinMaxDuty = 4; // allow min (& max) duty-cycle of 4 samples
    CalculateWaveDuty(0);
    CreateNewWave();
    WavePolarity();
    Serial.println("   Waves are Synchronized");
  }
  else
  {
    SquareWaveSync = LOW; // stop Synchronized Square wave
    if (TargetFreq < 163)
    {
      pinMode(7, INPUT); // re-enable pin after stopping
      TC_setup3();
    }
    else
    {
      REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
      REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
      SetPWM(7, Period, Pulse); // PWMC_EnableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
    }
    MinMaxDuty = 1; // allow min (& max) duty-cycle of 1 sample
    CalculateWaveDuty(ExactFreqMode);
    CreateNewWave();
    Serial.println("   Waves are Unsynchronized");
  }
  Serial.print("   Analogue Wave Period: ");
  PrintSyncedWavePeriod();
  Serial.print("   Analogue Wave Duty-cycle: "); Serial.print(ActualWaveDuty); Serial.println(" %\n");
  if (ExactFreqMode) SetWaveFreq(0); // ensures exact freq maintained
}

void SweepFreq()
{
  float oldFreq  = TargetFreq;
  float oldWaveFreq = TargetWaveFreq;
  int   SweepUpdatePeriod = 25; // change freq every SweepUpdatePeriod milliseconds
  float RiseIncrement = pow(float(SweepMaxFreq) / SweepMinFreq, 1 / (float(SweepRiseTime) / (SweepUpdatePeriod * 0.001))); // proportional freq increase required each SweepUpdatePeriod
  float FallIncrement = pow(float(SweepMinFreq) / SweepMaxFreq, 1 / (float(SweepFallTime) / (SweepUpdatePeriod * 0.001))); // proportional freq decrease required each SweepUpdatePeriod
  float TargetSweepFreq = SweepMinFreq;
  float SweepIncrement = RiseIncrement;
  unsigned long SweepUpdateTime = millis() + SweepUpdatePeriod;
  while (1)
  {
    if (TargetSweepFreq >= SweepMaxFreq) SweepIncrement = FallIncrement;
    else if (TargetSweepFreq <= SweepMinFreq) SweepIncrement = RiseIncrement;
    if (millis() >= SweepUpdateTime)
    {
      SweepUpdateTime = millis() + SweepUpdatePeriod; // set next SweepUpdateTime
      if (SweepFallTime == 0 && TargetSweepFreq >= SweepMaxFreq) TargetSweepFreq = SweepMinFreq;
      else if (SweepRiseTime == 0 && TargetSweepFreq <= SweepMinFreq) TargetSweepFreq = SweepMaxFreq;
      else TargetSweepFreq *= SweepIncrement;
      if (Control > 0)
      {
        TargetWaveFreq = min(TargetSweepFreq, 100000);
        SetWaveFreq(0);
        if (Control > 1 && !UsingGUI) Serial.println();
        if (!UsingGUI) Serial.print("\n   ");
        if (UsingGUI) Serial.print("AAF ");
        PrintSyncedWaveFreq();
      }
      if (Control != 1)
      {
        TargetFreq = TargetSweepFreq;
        SetFreqAndDuty(1, 1);
        if (UsingGUI) Serial.print("SAF ");
        else Serial.print("\n   ");
        PrintUnsyncedSqWaveFreq();
      }
    }
    if (Serial.available() > 0)
    {
      UserChars[0] = Serial.read();
      if (UserChars[0] == 's') // if signalled to stop
      {
        SweepStep = 0;
        if (Control > 0) // return to freq before sweep
        {
          TargetWaveFreq = oldWaveFreq;
          FreqIncrement = min(TargetWaveFreq, 100000) * 21475;
          SetWaveFreq(0);
          if (UsingGUI) Serial.print("AAF ");
          else Serial.print("\n   ");
          PrintSyncedWaveFreq();
        }
        if (Control != 1) // return to freq before sweep
        {
          TargetFreq = oldFreq;
          SetFreqAndDuty(1, 1);
          if (UsingGUI) Serial.print("SAF ");
          else Serial.print("\n   ");
          PrintUnsyncedSqWaveFreq();
        }
        if (!UsingGUI) Serial.println("\n   Sweep stopped\n");
        return;
      }
    }
  }
}

void PrintSyncedWaveFreq()
{
  if      (ActualWaveFreq < 1)       {
    Serial.print(ActualWaveFreq * 1000, 5);
    Serial.print(" mHz");
  }
  else if (ActualWaveFreq < 1000)    {
    Serial.print(ActualWaveFreq, 5);
    Serial.print(" Hz");
  }
  else                           {
    Serial.print(ActualWaveFreq / 1000, 5);
    Serial.print(" kHz");
  }
}

void PrintSyncedWavePeriod()
{
  float AnaPeriod = 1000 / ActualWaveFreq; // in mSeconds
  if (TargetWaveDuty == 0)
  {
    if (SquareWaveSync) AnaPulseWidth = 0.000048; // in mSeconds
    else AnaPulseWidth = 0.000350; // in mSeconds
  }
  else if (TargetWaveDuty >= 100)
  {
    if (SquareWaveSync) AnaPulseWidth = AnaPeriod - 0.000096; // in mSeconds
    else AnaPulseWidth = AnaPeriod - 0.000350; // in mSeconds
  }
  else AnaPulseWidth = ActualWaveDuty * 0.01 * AnaPeriod; // in mSeconds
  if      (AnaPeriod < 1)    {
    Serial.print(AnaPeriod * 1000, 5);
    Serial.print(" uS, Pulse Width: ");
  }
  else if (AnaPeriod < 1000) {
    Serial.print(AnaPeriod, 5);
    Serial.print(" mS, Pulse Width: ");
  }
  else if (AnaPeriod < 10000000) {
    Serial.print(AnaPeriod * 0.001, 5);
    Serial.print(" Sec, Pulse Width: ");
  }
  else                           {
    Serial.print(AnaPeriod * 0.001, 4);
    Serial.print(" Sec, Pulse Width: ");
  }
  if      (AnaPulseWidth < 0.001) {
    Serial.print(AnaPulseWidth * 1000000, 5);
    Serial.println(" nS");
  }
  else if (AnaPulseWidth < 1)     {
    Serial.print(AnaPulseWidth * 1000, 5);
    Serial.println(" uS");
  }
  else if (AnaPulseWidth < 1000)  {
    Serial.print(AnaPulseWidth, 5);
    Serial.println(" mS");
  }
  else if (AnaPulseWidth < 10000000) {
    Serial.print(AnaPulseWidth / 1000, 5);
    Serial.println(" Sec");
  }
  else                               {
    Serial.print(AnaPulseWidth / 1000, 4);
    Serial.println(" Sec");
  }
}

void PrintUnsyncedSqWaveFreq()
{
  if      (ActualFreq < 1)       {
    Serial.print(ActualFreq * 1000, 5);
    Serial.print(" mHz");
  }
  else if (ActualFreq < 1000)    {
    Serial.print(ActualFreq, 5);
    Serial.print(" Hz");
  }
  else if (ActualFreq < 1000000) {
    Serial.print(ActualFreq / 1000, 5);
    Serial.print(" kHz");
  }
  else                           {
    Serial.print(ActualFreq / 1000000, 5);
    Serial.print(" MHz");
  }
}

void PrintUnsyncedSqWavePeriod()
{
  if      (uPeriod < 1)       {
    Serial.print(uPeriod * 1000, 5);
    Serial.print(" nS, Pulse Width: ");
  }
  else if (uPeriod < 1000)    {
    Serial.print(uPeriod, 5);
    Serial.print(" uS, Pulse Width: ");
  }
  else if (uPeriod < 1000000) {
    Serial.print(uPeriod / 1000, 5);
    Serial.print(" mS, Pulse Width: ");
  }
  else if (uPeriod < 10000000000) {
    Serial.print(uPeriod / 1000000, 5);
    Serial.print(" Sec, Pulse Width: ");
  }
  else                            {
    Serial.print(uPeriod / 1000000, 4);
    Serial.print(" Sec, Pulse Width: ");
  }
  if      (uPulseWidth < 1)       {
    Serial.print(uPulseWidth * 1000, 5);
    Serial.println(" nS");
  }
  else if (uPulseWidth < 1000)    {
    Serial.print(uPulseWidth, 5);
    Serial.println(" uS");
  }
  else if (uPulseWidth < 1000000) {
    Serial.print(uPulseWidth / 1000, 5);
    Serial.println(" mS");
  }
  else if (uPulseWidth < 10000000000) {
    Serial.print(uPulseWidth / 1000000, 5);
    Serial.println(" Sec");
  }
  else                                {
    Serial.print(uPulseWidth / 1000000, 4);
    Serial.println(" Sec");
  }
}

float tcToFreq( int tc_cntr)
{
  float freq_hz;
  if      (tc_cntr == 0) return 1000;
  if      (FastMode == 3) freq_hz = (420000000.00 / tc_cntr) / NWAVETABLE; // display freq fractions
  else if (FastMode == 2) freq_hz = (168000000.00 / tc_cntr) / NWAVETABLE;
  else if (FastMode == 1) freq_hz = ( 84000000.00 / tc_cntr) / NWAVETABLE;
  else if (FastMode <= 0) freq_hz = ( 42000000.00 / tc_cntr) / NWAVETABLE;
  return freq_hz;
}

int freqToTc(float freq_hz)
{
  int tc_cntr = 0;
  if      (freq_hz == 0) return 25;
  if      (FastMode == 3) tc_cntr = (420000000UL / freq_hz) / NWAVETABLE;
  else if (FastMode == 2) tc_cntr = (168000000UL / freq_hz) / NWAVETABLE;
  else if (FastMode == 1) tc_cntr = ( 84000000UL / freq_hz) / NWAVETABLE;
  else if (FastMode <= 0) tc_cntr = ( 42000000UL / freq_hz) / NWAVETABLE;
  return tc_cntr;
}

void WavePolarity() // ensures the same wave polarity is maintained (relative to square wave)
{
  if (FastMode == 0)
  {
    DACC->DACC_TPR  =  (uint32_t)  Wave0[0];      // DMA buffer
    DACC->DACC_TNPR =  (uint32_t)  Wave0[1];      // next DMA buffer
  }
  else if (FastMode == 1)
  {
    DACC->DACC_TPR  =  (uint32_t)  Wave1[0];      // DMA buffer
    DACC->DACC_TNPR =  (uint32_t)  Wave1[1];      // next DMA buffer
  }
  else if (FastMode == 2)
  {
    DACC->DACC_TPR  =  (uint32_t)  Wave2[0];      // DMA buffer
    DACC->DACC_TNPR =  (uint32_t)  Wave2[1];      // next DMA buffer
  }
  else if (FastMode == 3)
  {
    DACC->DACC_TPR  =  (uint32_t)  Wave3[0];      // DMA buffer
    DACC->DACC_TNPR =  (uint32_t)  Wave3[1];      // next DMA buffer
  }
  DACC->DACC_TCR  =  Duty[0][FastMode];
  DACC->DACC_TNCR =  Duty[1][FastMode];
}

void SetWaveFreq(bool show) // FOR ANALOGUE & SYNCHRONIZED SQUARE WAVE:
{
  //  SET FREQUENCY:
  float dutyLimit = 0;
  float allowedWaveDuty = TargetWaveDuty;
  OldFastMode = FastMode; // old FastMode
  if      (!ExactFreqMode && TargetWaveFreq > 40000) FastMode =  3;
  else if (!ExactFreqMode && TargetWaveFreq > 20000) FastMode =  2;
  else if (!ExactFreqMode && TargetWaveFreq > 10000) FastMode =  1;
  else if (!ExactFreqMode && TargetWaveFreq >  1000) FastMode =  0;
  else // if slow mode (sample rate of 400kHz)
  {
    FastMode = -1;
    FreqIncrement = TargetWaveFreq * 21475;
    double FreqIncr = FreqIncrement;
    //  Set Duty (if necessary):
    if (TargetWaveDuty > 0 && TargetWaveDuty < 100) dutyLimit = 1 / (4000 / TargetWaveFreq); // calculate percent duty limit of 1 sample at 400kHz (up to 25% at 100kHz)
    if (ExactFreqMode)
    {
      FreqIncr *= ExactFreqModeAccuracy; // accuracy adjustment
    }
    allowedWaveDuty = constrain(TargetWaveDuty, dutyLimit, 100 - dutyLimit);
    if (TargetWavePulseWidth == 0 && allowedWaveDuty == LastAllowedWaveDuty) // if PulseWidth not fixed & duty limit not changed, quick freq/duty calculation is okay:
    {
      if (allowedWaveDuty == 0)
      {
        FreqIncrmt[0] = FreqIncr / 2;
        WaveHalf = 0;
        MinOrMaxWaveDuty = 1;
      }
      else FreqIncrmt[0] = constrain((1.00 / (           allowedWaveDuty  / 50.000)) * FreqIncr, 0 , 4294967295);
      //      else FreqIncrmt[0] = IncrProportion[0] * FreqIncr;
      if (allowedWaveDuty == 100)
      {
        FreqIncrmt[1] = FreqIncr / 2;
        WaveHalf = 1;
        MinOrMaxWaveDuty = 1;
      }
      else FreqIncrmt[1] = constrain((1.00 / ((100.000 - allowedWaveDuty) / 50.000)) * FreqIncr, 0 , 4294967295);
      //     else FreqIncrmt[1] = IncrProportion[1] * FreqIncr;
      Increment[0] = FreqIncrmt[0]; // update quickly after calculating above
      Increment[1] = FreqIncrmt[1]; // update quickly after calculating above
      if (TargetWaveFreq < 0.1) DitherTime = millis() + 100;
    }
    else CalculateWaveDuty(0); // if duty limit changed, freq/duty will need to be fully re-calculated:
  }
  if (FastMode >= 0)               // if in fast mode
  {
    SyncDelay = 0; // remove square wave sync delay for stability in case of sudden change to high freq
    TimerCounts = freqToTc(TargetWaveFreq);
    if (OldFastMode < 0)           // if exiting slow mode
    {
      NVIC_DisableIRQ(TC0_IRQn);
      dac_setup();
      TC_setup();
      CalculateWaveDuty(0);
      CreateNewWave();
    }
    else
    {
      TC_setup();
      if (TargetWavePulseWidth > 0)
      {
        CalculateWaveDuty(0);
        CreateNewWave();
      }
    }
    if (FastMode == OldFastMode) SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty)); // if fast mode changed calculate later
    else if (FastMode >= 0) ; // do nothing - Very slight delay needed anyway before proceeding
  }
  else // if (FastMode < 0)        // if in slow mode
  {
    TimerCounts = freqToTc(TargetWaveFreq);
    if (OldFastMode  >= 0)         // if exiting fast mode
    {
      dac_setup2();
      TC_setup2();
      NVIC_EnableIRQ(TC0_IRQn);
      CalculateWaveDuty(0);
    }
  }
  //  MEASURE FREQUENCY:
  if (ExactFreqMode) ActualWaveFreq = TargetWaveFreq; // if in ExactFreqMode - absolutely exact only at 50% duty-cycle
  else
  {
    if (TargetWaveFreq > 1000) ActualWaveFreq = tcToFreq(TimerCounts); // if in fast mode
    else                       ActualWaveFreq = 200000 / ceil(200000 / (TargetWaveFreq * 1.0000075)); // if in slow mode - previously 1.0002519
  }                                       //  increase this number if measurement is too low   ^^^

  //  MEASURE DUTY: (if necessary, due to duty limit changing with freq)
  if (FastMode >= 0) // if in fast mode
  {
    if (FastMode != OldFastMode || TargetWaveDuty > 0 || TargetWaveDuty < 100) // duty changes only when fast mode changes or at extreme duty-cycle
    {
      //      if (TargetWaveDuty == 0 || TargetWaveDuty == 100) ActualWaveDuty = TargetWaveDuty;
      if      (TargetWaveDuty == 0)   ActualWaveDuty = 0.0048 / (1000 / ActualWaveFreq);
      else if (TargetWaveDuty == 100) ActualWaveDuty = ((1000 / ActualWaveFreq) - 0.000048) / (10 / ActualWaveFreq);
      else ActualWaveDuty = (100.0 * Duty[0][FastMode]) / (Duty[0][FastMode] + Duty[1][FastMode]); // 1st wave half / (1st wave half + 2nd wave half)
      SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty));
    }
  }
  else // if in slow mode, duty limit was calculated with freq above
  {
    if      (TargetWaveDuty == 0)   ActualWaveDuty = 0.0048 / (1000 / ActualWaveFreq); ////new6
    else if (TargetWaveDuty == 100) ActualWaveDuty = ((1000 / ActualWaveFreq) - 0.000048) / (10 / ActualWaveFreq);
    else ActualWaveDuty = constrain(TargetWaveDuty, dutyLimit, 100 - dutyLimit);
  }
  if (show)
  {
    Serial.print("   Analogue Wave Freq: ");
    PrintSyncedWaveFreq(); Serial.print(", Target: ");
    Serial.print(TargetWaveFreq, 3);
    Serial.print(" Hz\n   Analogue Wave Period: ");
    PrintSyncedWavePeriod();
    Serial.print("   Analogue Wave Duty-cycle: "); Serial.print(ActualWaveDuty); Serial.println(" %\n");
  }
  LastAllowedWaveDuty = allowedWaveDuty;
}

void CalculateWaveDuty(bool updateSlowMode) // FOR ANALOGUE & SYNCHRONIZED SQUARE WAVE:
{
  float dutyLimit = 0;
  float allowedWaveDuty = TargetWaveDuty;
  if (FastMode >= 0)
  {
    //  SET DUTY:
    int du[2][5];
    if (TargetWaveDuty > 0 && TargetWaveDuty < 100)
    {
      du[0][0] = constrain(round(TargetWaveDuty *  1.6), MinMaxDuty, 160 - MinMaxDuty); // 1st half of cycle at FastMode 0 (160 = SamplesPerCycle)
      du[0][1] = constrain(round(TargetWaveDuty *  0.8), MinMaxDuty,  80 - MinMaxDuty); // 1st half of cycle at FastMode 1
      du[0][2] = constrain(round(TargetWaveDuty *  0.4), MinMaxDuty,  40 - MinMaxDuty); // 1st half of cycle at FastMode 2
      du[0][3] = constrain(round(TargetWaveDuty * 0.16), MinMaxDuty,  16 - MinMaxDuty); // 1st half of cycle at FastMode 3
      du[1][0] = 160 - du[0][0]; // 2nd half of cycle at FastMode 0
      du[1][1] =  80 - du[0][1]; // 2nd half of cycle at FastMode 1
      du[1][2] =  40 - du[0][2]; // 2nd half of cycle at FastMode 2
      du[1][3] =  16 - du[0][3]; // 2nd half of cycle at FastMode 3
      for (byte i = 0; i <= 3; i++)
      {
        Duty[0][i] = du[0][i]; // update quickly after calculating above
        Duty[1][i] = du[1][i]; // update quickly after calculating above
      }
      MinOrMaxWaveDuty = 0;
      //      if (SquareWaveSync && (du[0][FastMode] <= 4 || du[1][FastMode] <= 4)) WavePolarity(); // MinMaxDuty = 4
      if (SquareWaveSync && du[0][FastMode] != du[1][FastMode]) WavePolarity();
    }
    else // if TargetWaveDuty = 0 or 100, display only one wave half:
    {
      if (TargetWaveDuty == 0) WaveHalf = 0; // display 2nd wave half only (neg going)
      else WaveHalf = 1; // display 1st wave half only (pos going)
      Duty[!WaveHalf][0] = 160;
      Duty[!WaveHalf][1] =  80;
      Duty[!WaveHalf][2] =  40;
      Duty[!WaveHalf][3] =  16;
      MinOrMaxWaveDuty = 1;
    }
    //  MEASURE DUTY:
    if      (TargetWaveDuty == 0)   ActualWaveDuty = 0.0048 / (1000 / ActualWaveFreq);
    else if (TargetWaveDuty == 100) ActualWaveDuty = ((1000 / ActualWaveFreq) - 0.000048) / (10 / ActualWaveFreq);
    else ActualWaveDuty = (100.0 * Duty[0][FastMode]) / (Duty[0][FastMode] + Duty[1][FastMode]); // 1st wave half / (1st wave half + 2nd wave half)
    SyncDelay = (TimerCounts - Delay1) * Delay2 * max((Delay3 / (abs(ActualWaveDuty - 50.0) + Delay3)), int(MinOrMaxWaveDuty));
  }
  if (FastMode < 0 || updateSlowMode)
  {
    //  SET DUTY:
    MinOrMaxWaveDuty = 0;
    double FreqIncr = FreqIncrement;
    if (TargetWaveDuty > 0 && TargetWaveDuty < 100) dutyLimit = 1 / (4000 / TargetWaveFreq); // calculate percent duty limit of 1 sample at 400kHz (up to 25% at 100kHz)
    if (ExactFreqMode)
    {
      FreqIncr *= ExactFreqModeAccuracy; // accuracy adjustment
      if (TargetWaveDuty != 50) ExactFreqDutyNot50 = 1;
      else ExactFreqDutyNot50 = 0;
    }
    allowedWaveDuty = constrain(TargetWaveDuty, dutyLimit, 100 - dutyLimit);
    if (allowedWaveDuty == 0)
    {
      FreqIncrmt[0] = FreqIncr / 2;
      WaveHalf = 0;
      MinOrMaxWaveDuty = 1;
    }
    else FreqIncrmt[0] = constrain((1.00 / (           allowedWaveDuty  / 50.000)) * FreqIncr, 0 , 4294967295);
    if (allowedWaveDuty == 100)
    {
      FreqIncrmt[1] = FreqIncr / 2;
      WaveHalf = 1;
      MinOrMaxWaveDuty = 1;
    }
    else FreqIncrmt[1] = constrain((1.00 / ((100.000 - allowedWaveDuty) / 50.000)) * FreqIncr, 0 , 4294967295);
    IncrProportion[0] = FreqIncrmt[0] / FreqIncr;
    IncrProportion[1] = FreqIncrmt[1] / FreqIncr;
    int dutyMulti1    = (IncrProportion[1] / IncrProportion[0]) * 1000;
    DutyMultiplier[0] = (IncrProportion[0] / IncrProportion[1]) * 1000;
    DutyMultiplier[1] = dutyMulti1;    // update quickly after calculating above
    Increment[0] = FreqIncrmt[0]; // update quickly after calculating above
    Increment[1] = FreqIncrmt[1]; // update quickly after calculating above
    if (TargetWaveFreq < 0.1) DitherTime = millis() + 100;
    //  MEASURE DUTY:
    if      (TargetWaveDuty == 0)   ActualWaveDuty = 0.0048 / (1000 / ActualWaveFreq);
    else if (TargetWaveDuty == 100) ActualWaveDuty = ((1000 / ActualWaveFreq) - 0.000048) / (10 / ActualWaveFreq);
    else ActualWaveDuty = constrain(TargetWaveDuty, dutyLimit, 100 - dutyLimit);
  }
}

void SetFreqAndDuty(bool setFreq, bool setDuty) // Sets freq & duty-cycle and calculates measurements - FOR UNSYNCHRONIZED SQUARE WAVE:
{
  if (setFreq)
  {
    if (TargetFreq >= 1300) // Period <= 64615) // >= 1300 Hz
    {
      Period = round(84000000 / TargetFreq);
      if (!SquareWaveSync && pwmFreq == 10) // if viewing unsynch'ed sq.wave and exiting slow mode
      {
        REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
        REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
        NVIC_DisableIRQ(TC1_IRQn);
        PWMC_EnableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
      }
      pwmFreq = 42000000;
      uPeriodMultiplier = 1;
    }
    else if (TargetFreq >= 650) // Period <= 129231) // >= 650 Hz
    {
      Period = round(42000000 / TargetFreq);
      if (!SquareWaveSync && pwmFreq == 10) // if viewing unsynch'ed sq.wave and exiting slow mode
      {
        REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
        REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
        NVIC_DisableIRQ(TC1_IRQn);
        PWMC_EnableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
      }
      pwmFreq = 10500000;
      uPeriodMultiplier = 2;
    }
    else if (TargetFreq >= 325) // Period <= 420000) // >= 325 Hz
    {
      Period = round(21000000 / TargetFreq);
      if (!SquareWaveSync && pwmFreq == 10) // if viewing unsynch'ed sq.wave and exiting slow mode
      {
        REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
        REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
        NVIC_DisableIRQ(TC1_IRQn);
        PWMC_EnableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
      }
      pwmFreq = 2;
      uPeriodMultiplier = 4;
    }
    else if (TargetFreq >= 163) // Period <= 420000) // >= 163 Hz
    {
      Period = round(10500000 / TargetFreq);
      if (!SquareWaveSync && pwmFreq == 10) // if viewing unsynch'ed sq.wave and exiting slow mode
      {
        REG_PIOC_PER |= PIO_PER_P28; // PIO takes control of pin 3 from peripheral - similar to pinMode(3, OUTPUT)
        REG_PIOC_ODR |= PIO_ODR_P28; // PIO disables pin 3 (C28) - similar to pinMode(3, INPUT)
        NVIC_DisableIRQ(TC1_IRQn);
        PWMC_EnableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
      }
      pwmFreq = 4;
      uPeriodMultiplier = 8;
    }
    else // if (TargetFreq < 163) // if < 163 Hz, slow mode
    {
      Period = round(200000 / TargetFreq);
      if (!SquareWaveSync && pwmFreq != 10) // if viewing unsynch'ed sq.wave and entering slow mode
      {
        PWMC_DisableChannel(PWM_INTERFACE, g_APinDescription[7].ulPWMChannel);
        pinMode(7, INPUT);
        TC_setup3();
        PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC28B_TIOA7, PIO_DEFAULT);
      }
      pwmFreq = 10; // not used but identifies mode of operation
      uPeriodMultiplier = 420; // 210;
    }
  }
  if (setDuty) Pulse = constrain(round((TargetDuty / 100.0) * Period), 1, Period - 1);
  if (setDuty && !setFreq) ; // if setting duty only skip following freq measurement
  else // if setting freq (or if only measuring. ie: !setDuty && !setFreq)
  {
    uPeriod = (float(Period) * 0.0119047619047619) * uPeriodMultiplier;
    ActualFreq = 1000000.00 / uPeriod;
  }
  if (TargetFreq < 163) // load variables for interupt handler (and calculate uPulseWidth)
  {
    if (TargetDuty > 0 && TargetDuty < 100)
    {
      PulsePeriod[0] = Pulse;
      PulsePeriod[1] = Period;
      MinOrMaxDuty = 0;
      uPulseWidth = (float(Pulse) * 0.0119047619047619) * uPeriodMultiplier;
    }
    else
    {
      PulsePeriod[0] = Period;
      PulsePeriod[1] = Period;
      MinOrMaxDuty = 1;
      if (TargetDuty == 0)
      {
        PeriodHalf = 0;
        uPulseWidth = 0.096;
      }
      else
      {
        PeriodHalf = 1;
        uPulseWidth = uPeriod - 0.12;
      }
    }
  }
  else
  {
    if (!SquareWaveSync) SetPWM(7, Period, Pulse); // if viewing unsynch'ed sq.wave and freq >= 163Hz set up PWM
    uPulseWidth = (float(Pulse) * 0.0119047619047619) * uPeriodMultiplier;
  }
  ActualDuty = (uPulseWidth * 100) / uPeriod;
}

void SetPWM(byte pwmPin, uint32_t maxDutyCount, uint32_t duty) // sets up PWM for Unsynchronized Square Wave when at least 163Hz
{
  if (pwmFreq > 4) // if >= 650Hz
  {
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);
    PIO_Configure(g_APinDescription[pwmPin].pPort,
                  g_APinDescription[pwmPin].ulPinType,
                  g_APinDescription[pwmPin].ulPin,
                  g_APinDescription[pwmPin].ulPinConfiguration);
    uint32_t channel = g_APinDescription[pwmPin].ulPWMChannel;
    PWMC_ConfigureChannel(PWM_INTERFACE, channel , pwmFreq, 0, 0);
    PWMC_SetPeriod(PWM_INTERFACE, channel, maxDutyCount);
    PWMC_EnableChannel(PWM_INTERFACE, channel);
    PWMC_SetDutyCycle(PWM_INTERFACE, channel, maxDutyCount - duty);
    //    pmc_mck_set_prescaler(2); // creates 84MHz output but can't change back!
  }
  else
  {
    // Select Instance = PWM; Signal = PWML6 (channel 6); I/O Line = PC23 (C23, Arduino pin 7, see pinout diagram) ; Peripheral = B
    PMC->PMC_PCER1 |= PMC_PCER1_PID36;                      // Enable PWM
    REG_PIOC_ABSR |= PIO_ABSR_P23;                          // Set PWM pin perhipheral type B
    REG_PIOC_PDR |= PIO_PDR_P23;                            // Set PWM pin to an output
    REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(pwmFreq);  // Set the PWM clock rate to 2MHz (84MHz/42) ; Adjust DIVA for the resolution you are looking for
    REG_PWM_CMR6 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;       // The period is left aligned, clock source as CLKA on channel 6
    REG_PWM_CPRD6 = maxDutyCount;                         // Channel 6 : Set the PWM frequency
    REG_PWM_CDTY6 = duty;                                // Channel 6: Set the PWM duty cycle
    REG_PWM_ENA = PWM_ENA_CHID6;                        // Enable the PWM channel 6 (see datasheet page 973)
  }
}

void TC1_Handler() // used for Unsynchronized Square Wave when below 163Hz (200,000 clocks per Sec)
{
  // We need to get the status to clear it and allow the interrupt to fire again:
  TC_GetStatus(TC0, 1);
  TimeIncrement++; // 200,000 per Sec
  if (TimeIncrement >= PulsePeriod[PeriodHalf]) // if at end of pulse or period
  {
    if (MinOrMaxDuty) // if duty set to 0 or 100
    {
      if (PeriodHalf) // if duty set to 100
      {
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR; // set TIOA (sq. wave on pin 3) LOW when triggered (on next line)
        TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // software trigger also resets the counter and restarts the clock
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET; // set TIOA (sq. wave on pin 3) HIGH again when triggered (after else statement)
      }
      else // if duty set to 0
      {
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
        TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // output is set to 1, the counter is reset and the clock is started
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
      }
      TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // output is reset (creating 96nS pulse), the counter is reset and the clock is started
      TimeIncrement = 0; // if at end of period
    }
    else // if duty not set to 0 or 100
    {
      if (PeriodHalf)
      {
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
        TimeIncrement = 0; // if at end of period
      }
      else TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
      TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG;
      PeriodHalf = !PeriodHalf; // change to other half of period
    }
  }
}

void DACC_Handler(void) // write analogue & synchronized square wave to DAC with DMA - Fast Mode
{
  if      (FastMode == 3) DACC->DACC_TNPR = (uint32_t) Wave3[!WaveHalf]; // if (FastMode == 3) // next DMA buffer
  else if (FastMode == 2) DACC->DACC_TNPR = (uint32_t) Wave2[!WaveHalf]; // if (FastMode == 2) // next DMA buffer
  else if (FastMode == 1) DACC->DACC_TNPR = (uint32_t) Wave1[!WaveHalf]; // if (FastMode == 1) // next DMA buffer
  else if (FastMode == 0) DACC->DACC_TNPR = (uint32_t) Wave0[!WaveHalf]; // if (FastMode == 0) // next DMA buffer
  DACC->DACC_TNCR = Duty[!WaveHalf][max(0, FastMode)]; // number of counts until Handler re-triggered (when next half cycle finished)
  if (MinOrMaxWaveDuty) // if duty set to 0 or 100
  {
    if (SquareWaveSync) // creates squarewave synchronized with triangle or sine wave
    {
      for (int i = 0; i < SyncDelay; i++) {
        while (WaveHalf != WaveHalf); // delays of very short duration. ie: less than 1 microsecond amounts, to sync with analogue wave
      }
      if (WaveHalf)
      {
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR; // set TIOA (sq. wave on pin 3) LOW when triggered (on next line)
        TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG; // software trigger also resets the counter and restarts the clock
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET; // set TIOA (sq. wave on pin 3) HIGH again when triggered (after else statement)
      }
      else
      {
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
        TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG; // the counter is reset and the clock is started
        TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
      }
      TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // output is reset (creating 48nS pulse), the counter is reset and the clock is started
    }
  }
  else // if duty not set to 0 or 100
  {
    if (SquareWaveSync) // creates squarewave synchronized with triangle or sine wave
    {
      for (int i = 0; i < SyncDelay; i++) {
        while (WaveHalf != WaveHalf); // delays of very short duration. ie: less than 1 microsecond amounts, to sync with analogue wave
      }
      if (WaveHalf) TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
      else TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
      TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG; // the counter is reset and the clock is started
    }
    WaveHalf = !WaveHalf; // change to other wave half
  }
}

void TC0_Handler() // write analogue & synchronized square wave to DAC & pin 3 - Slow Mode (400,000 clocks per Sec)
{
  TC_GetStatus(TC0, 0);
  WaveBit += Increment[!WaveHalf]; // add appropriate Increment to WaveBit
  if (WaveBit < Increment[!WaveHalf]) // if end of wave half (if 4294967295 exceeded - highest number for 32 bit int before roll-over to zero)
  {
    if (ExactFreqDutyNot50) // if in exact freq mode and not at 50% duty-cycle
      WaveBit = (WaveBit / 1000) * DutyMultiplier[WaveHalf]; // if not 50% duty-cycle TRY to maintain freq
    else if (!ExactFreqMode) WaveBit = 1; // if not in exact freq mode reset to 1, allowing next update to be lower at 0 (necessary for very low duty cycle)
    if (MinOrMaxWaveDuty) // if duty set to 0 or 100
    {
      if (!WaveHalf) DACC->DACC_CDR = WaveFull[WaveBit / 1048576]; // if displaying 2nd wave half only (just passed end), write to DAC (1048576 = 4294967296 / 4096)
      else           DACC->DACC_CDR = WaveFull[(4294967295 - WaveBit) / 1048576]; // if displaying 1st wave half only (just passed end), write to DAC
      if (SquareWaveSync)
      {
        if (WaveHalf) // if duty set to 100
        {
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR; // set TIOA (sq. wave on pin 3) LOW when triggered (on next line)
          TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // software trigger also resets the counter and restarts the clock
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET; // set TIOA (sq. wave on pin 3) HIGH when triggered (after else statement)
        }
        else // if duty set to 0
        {
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET;
          TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // the counter is reset and the clock is started
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR;
        }
        TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // output is reset (creating 48nS pulse), the counter is reset and the clock is started
      }
    }
    else // if duty not set to 0 or 100
    {
      if (WaveHalf)
      {
        DACC->DACC_CDR = WaveFull[WaveBit / 1048576]; // if 2nd wave half (just passed end of 1st half), write to DAC
        if (SquareWaveSync)
        {
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_CLEAR; // set TIOA (sq. wave on pin 3) LOW when triggered (on next line)
          TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // the counter is reset and the clock is started
        }
      }
      else
      {
        DACC->DACC_CDR = WaveFull[(4294967295 - WaveBit) / 1048576]; // if 1st wave half (just passed end of 2nd half), write to DAC
        if (SquareWaveSync)
        {
          TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_ASWTRG_SET; // set TIOA (sq. wave on pin 3) HIGH when triggered (on next line)
          TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG; // the counter is reset and the clock is started
        }
      }
      WaveHalf = !WaveHalf; // change to other wave half
    }
  }
  else // if not end of wave half
  {
    if (!WaveHalf) DACC->DACC_CDR = WaveFull[WaveBit / 1048576]; // if 2nd wave half, write to DAC
    else          DACC->DACC_CDR = WaveFull[(4294967295 - WaveBit) / 1048576]; // if 1st wave half, write to DAC
  }
}

void TC_setup () // system timer clock set-up for analogue wave & synchronized square wave when in fast mode
{
  pmc_enable_periph_clk(TC_INTERFACE_ID);
  TcChannel * t = &(TC0->TC_CHANNEL)[0];
  t->TC_CCR = TC_CCR_CLKDIS;
  t->TC_IDR = 0xFFFFFFFF;
  t->TC_SR;
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |  TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  t->TC_RC = TimerCounts;
  t->TC_RA = TimerCounts / 2;
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void TC_setup2() // system timer clock set-up for analogue wave & triggering synchronized square wave when in slow mode
{
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC0);   // enable peripheral clock TC0
  // we want wavesel 01 with RC:
  TC_Configure(/* clock */TC0,/* channel */0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1); // select 42 MHz clock
  TC_SetRC(TC0, 0, 105); // select divisor of 105 (42 MHz / 105) - clocks DAC at 400kHz
  TC_Start(TC0, 0);
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // IER = interrupt enable register
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS; // IDR = interrupt disable register
  NVIC_EnableIRQ(TC0_IRQn);
}

void TC_setup3() // system timer clock set-up for Unsynchronized Square Wave when below 163Hz (PWM used above 163Hz)
{
  //  Serial.println("Entering < 163Hz mode");
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC1);   // enable peripheral clock TC1 (TC0, channel 1)
  // we want wavesel 01 with RC:
  TC_Configure(/* clock */TC0,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1); // select 42 MHz clock
  TC_SetRC(TC0, 1, 210); // select divisor of 210 (42 MHz / 210) - clocks at 200kHz
  TC_Start(TC0, 1);
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS; // IER = interrupt enable register
  TC0->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS; // IDR = interrupt disable register
  // Enable the interrupt in the nested vector interrupt controller
  // TC1_IRQn where 1 is the timer number * timer channels (3) + the channel number (=(0*3)+1) for timer 0 channel 1:
  NVIC_EnableIRQ(TC1_IRQn);
}

void TC_setup4() // timer clock set-up for synchronized square wave
{
  pmc_enable_periph_clk (TC_INTERFACE_ID + 2 * 3 + 1); // clock the TC2 channel 1. for pin 3
  //  pmc_enable_periph_clk (TC_INTERFACE_ID + 2*3+2); // clock the TC2 channel 2. for pin 11 instead
  TcChannel * t = &(TC2->TC_CHANNEL)[1];            // pointer to TC2 registers for its channel 1. for pin 3
  //  TcChannel * t = &(TC2->TC_CHANNEL)[2];         // pointer to TC2 registers for its channel 2. for pin 11 instead
  t->TC_CCR = TC_CCR_CLKDIS;                      // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF;                        // disable interrupts
  t->TC_SR;                                     // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |     // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                   // waveform mode
              TC_CMR_WAVSEL_UP |             // count-up without auto RC triggering
              TC_CMR_ASWTRG_SET |           // Software Trigger effect on TIOA
              TC_CMR_CPCTRG |              // enable manual RC triggering
              TC_CMR_ACPA_NONE |          // RA effect on TIOA
              TC_CMR_ACPC_NONE;          // RC effect on TIOA
  TC_Start(TC2, 1);
}

void TC_setup5() // timer clock set-up for timer - when operated from the serial monitor
{
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC4);   // enable peripheral clock TC1, channel 1
  TC_Configure(TC1, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4); // select 656.250 kHz clock
  TC_SetRC(TC1, 1, 39375000); // select divisor of 39375000 (656250Hz / 39375000) - resets at 1 minute
  TC_Start(TC1, 1);
}

void dac_setup() // DAC set-up for analogue wave & triggering synchronized square wave when in fast mode and using DMA
{
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ;   // start clocking DAC
  dacc_reset(DACC);
  dacc_set_transfer_mode(DACC, 0);
  dacc_set_power_save(DACC, 0, 1);              // sleep = 0, fast wakeup = 1
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_set_trigger(DACC, 1);
  //  dacc_set_channel_selection(DACC, 1);
  //  dacc_enable_channel(DACC, 1);
  dacc_set_channel_selection(DACC, 0);
  dacc_enable_channel(DACC, 0);
  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  NVIC_EnableIRQ(DACC_IRQn);
  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
  DACC->DACC_TPR  = (uint32_t) Wave0[0];     // DMA buffer
  DACC->DACC_TCR  = NWAVETABLE / 2;
  DACC->DACC_TNPR = (uint32_t) Wave0[1];     // next DMA buffer
  DACC->DACC_TNCR = NWAVETABLE / 2;
  DACC->DACC_PTCR = 0x00000100;
}

void dac_setup2() // DAC set-up for analogue & synchronized square wave when in slow mode (and Exact Freq Mode at any freq)
{
  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  dacc_disable_interrupt(DACC, DACC_IER_ENDTX); // disable DMA
  DACC->DACC_CR = DACC_CR_SWRST ;               // reset DAC
  DACC->DACC_CHER = DACC_CHER_CH0 << 0 ;        // enable chan0
}
