/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include <avr/pgmspace.h>
#include "cmdbuffer.h"

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#define VERSION_STRING  "1.0.0"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Calibrate print surface with automatic Z probe
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used when printing from SD card)
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float home_offset[3]={0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
char selectedFilename[LONG_FILENAME_LENGTH] = "";
char previousFilename[LONG_FILENAME_LENGTH] = "";
int lock;

// Extruder offset
#if EXTRUDERS > 1
#define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
uint8_t active_extruder = 0;
int atom_version = 2;
char *atom_version_name[] = {
	(char*)MSG_ATOM_2_0,
	(char*)MSG_ATOM_2_5,
	(char*)MSG_ATOM_2_5EX,
	(char*)MSG_ATOM_2_5FX,
};
char *extruder_side[] = {
	(char*)MSG_EXTRUDER_LEFT,
	(char*)MSG_EXTRUDER_RIGHT
};
bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop. 
bool extruder_inverting = false;
int load_filament_length;
int back_to_jct_length;
int unload_filament_length;
int fanSpeed=0;
float input_size=0;
float output_size=0;

#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

#ifdef ULTIPANEL
	bool powersupply = true;
#endif

#ifdef DELTA
float delta[3] = {0.0, 0.0, 0.0};
float delta_radius = DELTA_RADIUS;
float delta_calibration_radius = DELTA_CALIBRATION_RADIUS;
float delta_tower_angle_trim[ABC];
float endstop_adj[ABC] = { 0 };
float delta_tower[ABC][2];
float delta_diagonal_rod = 0;
float delta_diagonal_rod_2 = 0;
#endif

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
float lastPos[NUM_AXIS] = {0, 0, 0, 0}; 
static float offset[3] = {0.0, 0.0, 0.0};
float z_offset = 0.0;
float move_down_rate = 0.0;
float bed_level_rate = 0.0;
float bed_level[7][7] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
};
float home_pos[3] = {MANUAL_X_HOME_POS, MANUAL_Y_HOME_POS, MANUAL_Z_HOME_POS};
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
char cmdbuffer[MAX_CMD_SIZE];
static CMDBUFFER cmds[BUFSIZE];
static CMDBUFFER *curr_cmd = NULL;
static bool fromsd[BUFSIZE];
static volatile int bufindr = 0;
static volatile int bufindw = 0;
static volatile int buflen = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
static char *gcode_pointer;

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long currenttime = 0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;

bool Stopped = false;
bool wait_heating = false;
uint16_t SdfileCount = 0;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

inline void set_current_to_destination() { COPY(current_position, destination); }
inline void set_destination_to_current() { COPY(destination, current_position); }

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

FORCE_INLINE void inc_wr_idx()
{
  bufindw = (bufindw + 1) % BUFSIZE;
  ++buflen;
}

FORCE_INLINE void inc_rd_idx()
{
  --buflen;
  bufindr = (bufindr + 1) % BUFSIZE;
}

void clear_cmds_buffer()
{
	bufindr = bufindw;
	buflen = 0;
}
//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd, bool sd)
{
  if(buflen < BUFSIZE)
  {
	cmds[bufindw].clear();
	char c;
	while ((c = *cmd++) != 0)
	{
		cmds[bufindw].add(c);
	}
	cmds[bufindw].save_value();
	fromsd[bufindw] = sd;
    inc_wr_idx();
  }
}

void enquecommand_P(const char *cmd, bool sd)
{
  if(buflen < BUFSIZE)
  {
	cmds[bufindw].clear();
	char c;
	while ((c = pgm_read_byte(cmd++)) != 0)
	{
		cmds[bufindw].add(c);
	}
	cmds[bufindw].save_value();
	fromsd[bufindw] = sd;
	inc_wr_idx();
  }
}

void update_atom_settings()
{
	char tmp[21] = "";
	snprintf(tmp, 20, "ATOM %s Ready", atom_version_name[atom_version]);
	lcd_setstatus(tmp);

	switch (atom_version)
	{
	case 0:
		Z_MIN_ENDSTOP_INVERTING = false;
		load_filament_length = 1000;
    back_to_jct_length = 0;
		unload_filament_length = -1000;
		break;
	case 1:
		Z_MIN_ENDSTOP_INVERTING = true;
		load_filament_length = 1000;
    back_to_jct_length = 0;
		unload_filament_length = -1000;
		break;
	case 2:
		Z_MIN_ENDSTOP_INVERTING = true;
		load_filament_length = 660;
    back_to_jct_length = -110;
		unload_filament_length = -800;
		break;
	case 3:
		Z_MIN_ENDSTOP_INVERTING = true;
		load_filament_length = 600;
    back_to_jct_length = -160;
		unload_filament_length = -740;
		break;
	default:
		break;
	}
}

void update_delta_diagonal_rod_2()
{
  delta_diagonal_rod_2 = pow(delta_diagonal_rod, 2);
  Config_StoreSettings();
  SERIAL_ECHO("delta_diagonal_rod=");
  SERIAL_ECHOLN(delta_diagonal_rod);
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    pinMode(KILL_PIN,INPUT);
    WRITE(KILL_PIN,HIGH);
  #endif
}

void setup_laserpin() 
{ 
   #if defined(LASER_PIN) && LASER_PIN > -1
    SET_OUTPUT(LASER_PIN); 
    WRITE(LASER_PIN, LOW); 
  #endif 
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
  #endif
}

void setup()
{
  setup_killpin();
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLN_CPGM("start");
  SERIAL_ECHO_START;
  lock=1;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLN_CPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLN_CPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLN_CPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLN_CPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLN_CPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHO_CPGM(MSG_MARLIN);
  SERIAL_ECHOLN_CPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHO_CPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHO_CPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHO_CPGM(MSG_AUTHOR);
      SERIAL_ECHOLN_CPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHO_CPGM("Compiled: ");
      SERIAL_ECHOLN_CPGM(__DATE__);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHO_CPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHO_CPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  setup_laserpin();
  servo_init();

  lcd_init();
  _delay_ms(500);	// wait 1sec to display the splash screen
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
}
FORCE_INLINE bool cmds_can_append()
{
	return buflen < (BUFSIZE - 1);
}
FORCE_INLINE void save_commands()
{
	curr_cmd = &cmds[bufindr];
	if (curr_cmd->code == 'M' && curr_cmd->value == 29)
	{
		card.closefile();
		SERIAL_PROTOCOLLN_CPGM(MSG_FILE_SAVED);
	}
	else
	{
		card.write_command(curr_cmd);
		if (card.logging)
		{
			process_commands();
		}
		else
		{
			SERIAL_PROTOCOLLN_CPGM(MSG_OK);
		}
	}
	inc_rd_idx();
}
void loop()
{
	if (cmds_can_append())
	{
		get_command();
	}
#ifdef SDSUPPORT
	card.checkautostart(false);
#endif
	if (buflen)
	{
#ifdef SDSUPPORT
		if (card.saving)
		{
			save_commands();
		}
		else
		{
			process_commands();
		}
#else
		process_commands();
#endif //SDSUPPORT
	}
	//check heater every n milliseconds
	manage_heater();
	manage_inactivity();
	checkHitEndstops();
	lcd_update();
}
FORCE_INLINE void finished_print()
{
	SERIAL_PROTOCOLLN_CPGM(MSG_FILE_PRINTED);
	stoptime = millis();
	char time[30];
	unsigned long t = (stoptime - starttime) / 1000;
	int hours, minutes;
	minutes = (t / 60) % 60;
	hours = t / 60 / 60;
	sprintf_P(time, PSTR("%i hours %i minutes"), hours, minutes);
	SERIAL_ECHO_START;
	SERIAL_ECHOLN(time);
	lcd_setstatus(time);
  lock=1;
	card.printingHasFinished();
	card.checkautostart(true);
}
FORCE_INLINE bool EOL(char c)
{
	if (c == '\n')
		return true;
	if (c == '\r')
		return true;
	if (c == ':' && comment_mode == false)
		return true;

	return false;
}
FORCE_INLINE void get_command_from_serial()
{
	while (cmds_can_append() && MYSERIAL.available() > 0)
	{
		serial_char = MYSERIAL.read();
		if (EOL(serial_char) ||	serial_count >= (MAX_CMD_SIZE - 1))
		{
			if (!serial_count) //if empty line
			{ 
				comment_mode = false; //for new command
				return;
			}

			if (comment_mode) 
			{
				comment_mode = false; //for new command
				serial_count = 0; //clear buffer
				cmdbuffer[serial_count] = 0; //terminate string
				return;
			}

			cmdbuffer[serial_count] = 0; //terminate string
			strchr_pointer = strchr(cmdbuffer, 'N');
			if (strchr_pointer != NULL)
			{
				gcode_N = strtol(strchr_pointer + 1, &gcode_pointer, 10);
				if (gcode_N != gcode_LastN + 1 && strstr_P(cmdbuffer, PSTR("M110")) == NULL)
				{
					SERIAL_ERROR_START;
					SERIAL_ERROR_CPGM(MSG_ERR_LINE_NO);
					SERIAL_ERRORLN(gcode_LastN);
					//Serial.println(gcode_N);
					FlushSerialRequestResend();
					serial_count = 0;
					return;
				}
				strchr_pointer = strchr(cmdbuffer, '*');
				if (strchr_pointer != NULL)
				{
					byte checksum = 0;
					byte count = 0;
					while (cmdbuffer[count] != '*')
					{
						checksum = checksum^cmdbuffer[count++];
					}
					if ((int)(strtod(strchr_pointer + 1, NULL)) != checksum)
					{
						SERIAL_ERROR_START;
						SERIAL_ERROR_CPGM(MSG_ERR_CHECKSUM_MISMATCH);
						SERIAL_ERRORLN(gcode_LastN);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}
					//if no errors, continue parsing
					*strchr_pointer = 0;
					while (*gcode_pointer == ' ')
					{
						gcode_pointer++;
					}
					enquecommand(gcode_pointer, false);
					serial_count = 0; //clear buffer
					gcode_LastN = gcode_N;
					return;
				}
				else //error
				{
					SERIAL_ERROR_START;
					SERIAL_ERROR_CPGM(MSG_ERR_NO_CHECKSUM);
					SERIAL_ERRORLN(gcode_LastN);
					FlushSerialRequestResend();
					serial_count = 0;
					return;
				}
			}
			else //we don't receive 'N'
			{
				// if we don't receive 'N' but still see '*'
				if ((strchr(cmdbuffer, '*') != NULL))
				{
					SERIAL_ERROR_START;
					SERIAL_ERROR_CPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
					SERIAL_ERRORLN(gcode_LastN);
					serial_count = 0;
					return;
				}
				enquecommand(cmdbuffer, false);
				serial_count = 0; //clear buffer
				return;
			}
			if ((strchr(cmdbuffer, 'G') != NULL))
			{
				strchr_pointer = strchr(cmdbuffer, 'G');
				switch ((int)((strtod(strchr_pointer + 1, NULL))))
				{
				case 0:
				case 1:
				case 2:
				case 3:
					if (Stopped == false)
					{ // If printer is stopped by an error the G[0-3] codes are ignored.
#ifdef SDSUPPORT
						if (card.saving)
							break;
#endif //SDSUPPORT
						SERIAL_PROTOCOLLN_CPGM(MSG_OK);
					}
					else
					{
						SERIAL_ERRORLN_CPGM(MSG_ERR_STOPPED);
						LCD_MESSAGE_CPGM(MSG_STOPPED);
					}
					break;
				default:
					break;
				}
			}
			serial_count = 0; //clear buffer
		}
		else
		{
			if (serial_char == ';') comment_mode = true;
			if (!comment_mode) cmdbuffer[serial_count++] = serial_char;
		}
	}
}

FORCE_INLINE void get_command_from_sd()
{
	while (cmds_can_append() && !card.eof())
	{
		int16_t n = card.get();
		serial_char = (char)n;
		if (EOL(serial_char) || serial_count >= (MAX_CMD_SIZE - 1) || n == -1)
		{
			if (card.eof())
			{
				finished_print();
			}
			if (!serial_count)
			{
				goto quit;
			}
			cmds[bufindw].save_value();
			fromsd[bufindw] = true;
			inc_wr_idx();

		quit:
			comment_mode = false; //for new command
			serial_count = 0; //clear buffer
		}
		else
		{
			if (serial_char == ';')
			{
				CMDBUFFER::comment = true;
				comment_mode = true;
			}
			if (!comment_mode)
			{
				if (!serial_count)
				{
					cmds[bufindw].clear();
				}
				cmds[bufindw].add(serial_char);
				serial_count++;
			}
		}
	}
}

void get_command()
{
	get_command_from_serial();

	#ifdef SDSUPPORT
	if(!card.sdprinting || serial_count != 0)
	{
		return;
	}
	get_command_from_sd();
	#endif //SDSUPPORT
}

FORCE_INLINE float code_value()
{
  return curr_cmd->get_find_value();
}
FORCE_INLINE long code_value_long()
{
  return curr_cmd->get_find_value();
}
FORCE_INLINE bool code_seen(char code)
{
  return curr_cmd->find(code);
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(int axis) 
{
  current_position[axis] = home_pos[axis] + home_offset[axis];
  min_pos[axis] =          base_min_pos(axis) + home_offset[axis];
  max_pos[axis] =          base_max_pos(axis) + home_offset[axis];
}

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);

	// Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (SERVO_ENDSTOPS[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, CURRENT_EXTRUDER);
    st_synchronize();

    enable_endstops(false);  // Ignore Z probe while moving away from the top microswitch.
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, CURRENT_EXTRUDER);
    st_synchronize();
    enable_endstops(true);  // Stop ignoring Z probe while moving up to the top microswitch again.

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis]/10;

    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, CURRENT_EXTRUDER);
    st_synchronize();

    if (endstop_adj[axis])
    {
      current_position[axis] = 0;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      current_position[axis] = endstop_adj[axis];
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, CURRENT_EXTRUDER);
      st_synchronize();
    }
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (SERVO_ENDSTOPS[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
  }
}

void deploy_z_probe() 
{
  feedmultiply = move_down_rate;
  feedrate = 8000;
  destination[X_AXIS] = 0;
  destination[Y_AXIS] = 0;
  destination[Z_AXIS] = 30;
  prepare_move_raw();
  st_synchronize();
  feedrate = 8000/10;
}

void retract_z_probe() {
  feedmultiply = 100;
  feedrate = 8000;
  // destination[Z_AXIS] = current_position[Z_AXIS] + 20;
  prepare_move_raw();

  destination[X_AXIS] = 0;
  destination[Y_AXIS] = 0;
  destination[Z_AXIS] = 100;
  
  //destination[Z_AXIS] = current_position[Z_AXIS] + 150;
  //destination[Z_AXIS] = current_position[Z_AXIS] - 10;
  prepare_move_raw();

  // Move the nozzle below the print surface to push the probe up.
  // feedrate = homing_feedrate[Z_AXIS]/10;
  //destination[Z_AXIS] = current_position[Z_AXIS] - 20;
  //prepare_move_raw();

  //feedrate = homing_feedrate[Z_AXIS];
  //destination[Z_AXIS] = current_position[Z_AXIS] + 30;
  //prepare_move_raw();
  //st_synchronize();
}

float z_probe() 
{
  feedrate = 1800;
  prepare_long_move_raw();
  st_synchronize();

  enable_endstops(true);
  float start_z = current_position[Z_AXIS];
  long start_steps = st_get_position(Z_AXIS);

  destination[Z_AXIS] = -20;
  prepare_move_raw();
  st_synchronize();
  endstops_hit_on_purpose();

  enable_endstops(false);
  long stop_steps = st_get_position(Z_AXIS);

  float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
  current_position[Z_AXIS] = mm;
  calculate_delta(current_position);
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

  destination[Z_AXIS] = mm + 2;
  prepare_move_raw();
  st_synchronize();

  start_z = current_position[Z_AXIS];
  start_steps = st_get_position(Z_AXIS);
  feedrate = 200;
  enable_endstops(true);
  destination[Z_AXIS] = -20;
  prepare_move_raw();
  st_synchronize();
  endstops_hit_on_purpose();
  enable_endstops(false);
  stop_steps = st_get_position(Z_AXIS);
  mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
  current_position[Z_AXIS] = mm;
  calculate_delta(current_position);
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  set_destination_to_current();

  feedrate = 1800;
  destination[Z_AXIS] = mm + 10;
  prepare_move_raw();
  return mm;
}

void calibrate_print_surface() 
{
  feedrate = homing_feedrate[X_AXIS];
  destination[X_AXIS] = 0;
  destination[Y_AXIS] = 0;
  destination[Z_AXIS] = 30;
  prepare_long_move_raw();
  st_synchronize();

  for (int y = 3; y >= -3; y--) 
  {
    int dir = y % 2 ? -1 : 1;
    for (int x = -3*dir; x != 4*dir; x += dir) 
    {
      if (x*x + y*y < 11) 
      {
	      destination[X_AXIS] = AUTOLEVEL_GRID * x;
	      destination[Y_AXIS] = AUTOLEVEL_GRID * y;
	      bed_level[x+3][y+3] = z_probe();
      } 
      else 
      {
	      bed_level[x+3][y+3] = 0.0;
      }
    }
    // For unprobed positions just copy nearest neighbor.
    if (abs(y) >= 3) 
    {
      bed_level[1][y+3] = bed_level[2][y+3];
      bed_level[5][y+3] = bed_level[4][y+3];
    }
    if (abs(y) >=2) 
    {
      bed_level[0][y+3] = bed_level[1][y+3];
      bed_level[6][y+3] = bed_level[5][y+3];
    }
  }
  for (int y = 3; y >= -3; y--) 
  {
    for (int x = -3; x <= 3; x++) 
    {
      // Print calibration results for manual frame adjustment.
      SERIAL_PROTOCOL_F(bed_level[x + 3][y + 3], 3);
      SERIAL_PROTOCOL_CPGM(" ");
    }
    SERIAL_ECHOLN("");
  }
}

void print_bed_level(){
  for (int y = 3; y >= -3; y--) {
    for (int x = -3; x <= 3; x++) {
       SERIAL_PROTOCOL_F(bed_level[x+3][y+3], 3);
       SERIAL_PROTOCOL_CPGM(" ");
    }
    SERIAL_ECHOLN("");
  }  
}

void reset_bed_level(){
  home_pos[X_AXIS] = MANUAL_X_HOME_POS;
  home_pos[Y_AXIS] = MANUAL_Y_HOME_POS;
  home_pos[Z_AXIS] = MANUAL_Z_HOME_POS;
  z_offset = 0;
  move_down_rate = 50;
  bed_level_rate = 20;
  for (int y = 3; y >= -3; y--) {
    for (int x = -3; x <= 3; x++) {
      bed_level[x+3][y+3] = 0.0;
    }
  }
}

void home_delta_axis() 
{
  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 50;
  previous_millis_cmd = millis();
  enable_endstops(true);
  set_destination_to_current();
  feedrate = 0.0;
  current_position[X_AXIS] = 0;
  current_position[Y_AXIS] = 0;
  current_position[Z_AXIS] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  destination[X_AXIS] = 3 * Z_MAX_LENGTH;
  destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
  destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
  feedrate = 1.732 * homing_feedrate[X_AXIS];
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, CURRENT_EXTRUDER);
  st_synchronize();
  endstops_hit_on_purpose();

  current_position[X_AXIS] = destination[X_AXIS];
  current_position[Y_AXIS] = destination[Y_AXIS];
  current_position[Z_AXIS] = destination[Z_AXIS];

  // take care of back off and rehome now we are all at the top
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  homeaxis(Z_AXIS);

  calculate_delta(current_position);
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  #ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  previous_millis_cmd = millis();
  endstops_hit_on_purpose();
}
void home_twice()
{
  home_delta_axis();
  home_delta_axis();
}
void deploy_z_zero() 
{
  home_twice();
  st_synchronize();
  delay(2000);

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedrate = 8000;
  feedmultiply = move_down_rate;

  destination[X_AXIS] = 0;
  destination[Y_AXIS] = 0;
  destination[Z_AXIS] = 10;
  prepare_move();
  st_synchronize();

  feedmultiply = bed_level_rate;

  float start_z = current_position[Z_AXIS];
  long start_steps = st_get_position(Z_AXIS);
  enable_endstops(true);
  destination[X_AXIS] = 0;
  destination[Y_AXIS] = 0;
  destination[Z_AXIS] = 0.1;
  prepare_move();
  st_synchronize();
  enable_endstops(false);
  endstops_hit_on_purpose();

  long stop_steps = st_get_position(Z_AXIS);
  current_position[Z_AXIS] = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
  calculate_delta(current_position);
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  if (((int)(current_position[Z_AXIS] * 10)) > 1)
  {
	  z_offset += current_position[Z_AXIS] + 0.1 * 2;
  }
  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
}
bool z_min_check()
{
	if (READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)
	{
		LCD_MESSAGE_CPGM("Z Min Error");
		lcd_return_to_status();
		lcd_buzz(1000, 300);
		return false;
	}
	return true;
}
void g29()
{
  home_twice();
  saved_feedmultiply = feedmultiply;
  deploy_z_probe();
  calibrate_print_surface();
  retract_z_probe();
  feedrate = 6000;
  feedmultiply = saved_feedmultiply;
  previous_millis_cmd = millis();
  endstops_hit_on_purpose();
  Config_StoreSettings();
  home_delta_axis();
}

float probe_pt(const float &lx, const float &ly)
{
  destination[X_AXIS] = lx;
  destination[Y_AXIS] = ly;
  destination[Z_AXIS] = 5;
  return z_probe();
}
static void saveFeedRate()
{
  saved_feedrate = feedrate;
}
static void restoreFeedRate()
{
  feedrate = saved_feedrate;
}
void print_signed_float(const char * const prefix, const float &f)
{
  SERIAL_PROTOCOL_CPGM("  ");
  serialprintPGM(prefix);
  SERIAL_CHAR(':');
  if (f >= 0) SERIAL_CHAR('+');
  SERIAL_PROTOCOL_F(f, 2);
}
static void print_G33_settings() 
{
  SERIAL_PROTOCOLPAIRCPGM(".Height:", DELTA_HEIGHT + home_offset[Z_AXIS]);
  print_signed_float(PSTR("  Ex"), endstop_adj[A_AXIS]);
  print_signed_float(PSTR("Ey"), endstop_adj[B_AXIS]);
  print_signed_float(PSTR("Ez"), endstop_adj[C_AXIS]);
  SERIAL_PROTOCOLPAIRCPGM("    Radius:", delta_radius);

  SERIAL_PROTOLN();
  SERIAL_PROTOCOL_CPGM(".Tower angle :  ");
  print_signed_float(PSTR("Tx"), delta_tower_angle_trim[A_AXIS]);
  print_signed_float(PSTR("Ty"), delta_tower_angle_trim[B_AXIS]);
  print_signed_float(PSTR("Tz"), delta_tower_angle_trim[C_AXIS]);
  SERIAL_PROTOLN();
}
static void G33_cleanup()
{
  restoreFeedRate();
}
void recalc_delta_settings() 
{
  delta_tower[A_AXIS][X_AXIS] = cos(RADIANS(210 + delta_tower_angle_trim[A_AXIS])) * delta_radius; // front left tower
  delta_tower[A_AXIS][Y_AXIS] = sin(RADIANS(210 + delta_tower_angle_trim[A_AXIS])) * delta_radius;
  delta_tower[B_AXIS][X_AXIS] = cos(RADIANS(330 + delta_tower_angle_trim[B_AXIS])) * delta_radius; // front right tower
  delta_tower[B_AXIS][Y_AXIS] = sin(RADIANS(330 + delta_tower_angle_trim[B_AXIS])) * delta_radius;
  delta_tower[C_AXIS][X_AXIS] = cos(RADIANS(90 + delta_tower_angle_trim[C_AXIS])) * delta_radius; // back middle tower
  delta_tower[C_AXIS][Y_AXIS] = sin(RADIANS(90 + delta_tower_angle_trim[C_AXIS])) * delta_radius;
  delta_diagonal_rod_2 = sq(delta_diagonal_rod);
}
inline void g33()
{
  const int8_t probe_points = 3;
  const float calibration_precision = 0;
  const int8_t force_iterations = 0;
  const bool towers_set = true;
  const static char save_message[] PROGMEM = "Auto Save Delta Setting";
  int8_t iterations = 0;
  float test_precision; 
  float zero_std_dev = 999.0;
  float zero_std_dev_old = zero_std_dev;
  float zero_std_dev_min = zero_std_dev;

  float e_old[ABC] = 
  {
    endstop_adj[A_AXIS],
    endstop_adj[B_AXIS],
    endstop_adj[C_AXIS]
  };
  float dr_old = delta_radius;
  float zh_old = home_offset[Z_AXIS];
  float ta_old[ABC] = 
  {
    delta_tower_angle_trim[A_AXIS],
    delta_tower_angle_trim[B_AXIS],
    delta_tower_angle_trim[C_AXIS]
  };

  SERIAL_PROTOCOLLN_CPGM("G33 Auto Calibrate");
  st_synchronize();
  reset_bed_level();
  saveFeedRate();
  enable_endstops(true); 
  home_twice();

  home_offset[Z_AXIS] = -probe_pt(0, 0);
  SERIAL_PROTOCOLPAIRCPGM("Z Height:", DELTA_HEIGHT + home_offset[Z_AXIS]);
  SERIAL_PROTOLN();

  home_twice();
  SERIAL_PROTOCOLLN_CPGM("Checking... AC");
  print_G33_settings();

  do
  {
    float z_at_pt[13] = { 0.0 };

    test_precision = zero_std_dev_old != 999.0 ? (zero_std_dev + zero_std_dev_old) / 2 : zero_std_dev;
    iterations++;
    feedrate = 6000;

    // probe extra center points
    for (int8_t axis = 9; axis > 0; axis -= 4)
    {
      const float a = RADIANS(180 + 30 * axis);
      const float r = delta_calibration_radius * 0.1;
      z_at_pt[0] += probe_pt(cos(a) * r, sin(a) * r);
      if (isnan(z_at_pt[0]))
      {
        return G33_cleanup();
      }
    }
    z_at_pt[0] /= float(probe_points);

    // probe the radius
    for (uint8_t axis = 1; axis < 13; axis += 2)
    {
      const float a = RADIANS(180 + 30 * axis);
      const float r = delta_calibration_radius;
      z_at_pt[axis] += probe_pt(cos(a) * r, sin(a) * r);
      if (isnan(z_at_pt[axis]))
      {
        return G33_cleanup();
      }
    }

    float S1 = z_at_pt[0];
    float S2 = sq(z_at_pt[0]);
    int16_t N = 1;
    // std dev from zero plane
    for (uint8_t axis = 1; axis < 13; axis += 2)
    {
      S1 += z_at_pt[axis];
      S2 += sq(z_at_pt[axis]);
      N++;
    }
    zero_std_dev_old = zero_std_dev;
    zero_std_dev = round(SQRT(S2 / N) * 1000.0) / 1000.0 + 0.00001;

    // Solve matrices
    if ((zero_std_dev < test_precision || iterations <= force_iterations) && zero_std_dev > calibration_precision)
    {
      if (zero_std_dev < zero_std_dev_min)
      {
        COPY(e_old, endstop_adj);
        dr_old = delta_radius;
        zh_old = home_offset[Z_AXIS];
        COPY(ta_old, delta_tower_angle_trim);
      }

      float e_delta[ABC] = { 0.0 }, r_delta = 0.0, t_delta[ABC] = { 0.0 };
      const float r_diff = delta_radius - delta_calibration_radius;
      const float h_factor = (1.00 + r_diff * 0.001) / 6.0;                                       // 1.02 for r_diff = 20mm
      const float r_factor = (-(1.75 + 0.005 * r_diff + 0.001 * sq(r_diff))) / 6.0;               // 2.25 for r_diff = 20mm
      const float a_factor = (66.66 / delta_calibration_radius) / (iterations == 1 ? 16.0 : 2.0); // 0.83 for cal_rd = 80mm  (Slow down on 1st iteration)

      #define ZP(N,I) ((N) * z_at_pt[I])
      #define Z6(I) ZP(6, I)
      #define Z4(I) ZP(4, I)
      #define Z2(I) ZP(2, I)
      #define Z1(I) ZP(1, I)

      e_delta[A_AXIS] = (Z6(0) + Z2(1) - Z1(5) - Z1(9) - Z2(7) + Z1(11) + Z1(3)) * h_factor;
      e_delta[B_AXIS] = (Z6(0) - Z1(1) + Z2(5) - Z1(9) + Z1(7) - Z2(11) + Z1(3)) * h_factor;
      e_delta[C_AXIS] = (Z6(0) - Z1(1) - Z1(5) + Z2(9) + Z1(7) + Z1(11) - Z2(3)) * h_factor;
      r_delta = (Z6(0) - Z1(1) - Z1(5) - Z1(9) - Z1(7) - Z1(11) - Z1(3)) * r_factor;

      if (towers_set)
      {
        t_delta[A_AXIS] = (-Z2(5) + Z2(9) - Z2(11) + Z2(3)) * a_factor;
        t_delta[B_AXIS] = (Z2(1) - Z2(9) + Z2(7) - Z2(3)) * a_factor;
        t_delta[C_AXIS] = (-Z2(1) + Z2(5) - Z2(7) + Z2(11)) * a_factor;
        e_delta[A_AXIS] += (t_delta[B_AXIS] - t_delta[C_AXIS]) / 4.5;
        e_delta[B_AXIS] += (t_delta[C_AXIS] - t_delta[A_AXIS]) / 4.5;
        e_delta[C_AXIS] += (t_delta[A_AXIS] - t_delta[B_AXIS]) / 4.5;
      }
      LOOP_XYZ(axis) endstop_adj[axis] += e_delta[axis];
      delta_radius += r_delta;
      LOOP_XYZ(axis) delta_tower_angle_trim[axis] += t_delta[axis];
    }
    else if (zero_std_dev >= test_precision) // step one back
    {
      COPY(endstop_adj, e_old);
      delta_radius = dr_old;
      home_offset[Z_AXIS] = zh_old;
      COPY(delta_tower_angle_trim, ta_old);
    }

    // normalise angles to least squares
    float a_sum = 0.0;
    LOOP_XYZ(axis) a_sum += delta_tower_angle_trim[axis];
    LOOP_XYZ(axis) delta_tower_angle_trim[axis] -= a_sum / 3.0;

    // adjust delta_height and endstops by the max amount
    const float z_temp = MAX3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);
    home_offset[Z_AXIS] -= z_temp;
    LOOP_XYZ(axis) endstop_adj[axis] -= z_temp;

    recalc_delta_settings();
    NOMORE(zero_std_dev_min, zero_std_dev);

    if ((zero_std_dev >= test_precision && iterations > force_iterations) || zero_std_dev <= calibration_precision)
    {
      // end iterations
      SERIAL_PROTOCOL_CPGM("Calibration OK");
      SERIAL_PROTOCOL_SP(36);
      if (zero_std_dev >= test_precision)
      {
        SERIAL_PROTOCOL_CPGM("rolling back.");
      }
      else
      {
        SERIAL_PROTOCOL_CPGM("std dev:");
        SERIAL_PROTOCOL_F(zero_std_dev_min, 3);
      }
      SERIAL_PROTOLN();
      char mess[21];
      sprintf_P(mess, PSTR("Calibration sd:"));
      if (zero_std_dev_min < 1)
      {
        sprintf_P(&mess[15], PSTR("0.%03i"), (int)round(zero_std_dev_min * 1000.0));
      }
      else
      {
        sprintf_P(&mess[15], PSTR("%03i.x"), (int)round(zero_std_dev_min));
      }
      lcd_setstatus(mess);
      print_G33_settings();
      serialprintPGM(save_message);
      SERIAL_PROTOLN();
    }
    else // !end iterations
    {                                                     
      char mess[15];
      if (iterations < 31)
      {
        sprintf_P(mess, PSTR("Iteration : %02i"), (int)iterations);
      }
      else
      {
        sprintf_P(mess, PSTR("No convergence"));
      }
      SERIAL_PROTOCOL(mess);
      SERIAL_PROTOCOL_SP(36);
      SERIAL_PROTOCOL_CPGM("std dev:");
      SERIAL_PROTOCOL_F(zero_std_dev, 3);
      SERIAL_PROTOLN();
      print_G33_settings();
    }
    enable_endstops(true);
    home_delta_axis();

  } while (((zero_std_dev < test_precision && iterations < 31) || iterations <= force_iterations) && zero_std_dev > calibration_precision);

  G33_cleanup();
  Config_StoreSettings();
}

FORCE_INLINE void process_gcode(int value)
{
	unsigned long codenum; //throw away variable
	switch (value)
	{
	case 0: // G0 -> G1
	case 1: // G1
		if (Stopped == false) 
		{
			get_coordinates(); // For X Y Z E F
			prepare_move();
			//ClearToSend();
			return;
		}
		//break;
	case 2: // G2  - CW ARC
		if (Stopped == false) {
			get_arc_coordinates();
			prepare_arc_move(true);
			return;
		}
	case 3: // G3  - CCW ARC
		if (Stopped == false) {
			get_arc_coordinates();
			prepare_arc_move(false);
			return;
		}
	case 4: // G4 dwell
		LCD_MESSAGE_CPGM(MSG_DWELL);
		codenum = 0;
		if (code_seen('P')) codenum = code_value(); // milliseconds to wait
		if (code_seen('S')) codenum = code_value() * 1000; // seconds to wait

		st_synchronize();
		codenum += millis();  // keep track of when we started waiting
		previous_millis_cmd = millis();
		while (millis()  < codenum) {
			manage_heater();
			manage_inactivity();
			lcd_update();
		}
		break;
#ifdef FWRETRACT
	case 10: // G10 retract
		if (!retracted)
		{
			destination[X_AXIS] = current_position[X_AXIS];
			destination[Y_AXIS] = current_position[Y_AXIS];
			destination[Z_AXIS] = current_position[Z_AXIS];
			current_position[Z_AXIS] += -retract_zlift;
			destination[E_AXIS] = current_position[E_AXIS] - retract_length;
			feedrate = retract_feedrate;
			retracted = true;
			prepare_move();
		}

		break;
	case 11: // G10 retract_recover
		if (!retracted)
		{
			destination[X_AXIS] = current_position[X_AXIS];
			destination[Y_AXIS] = current_position[Y_AXIS];
			destination[Z_AXIS] = current_position[Z_AXIS];

			current_position[Z_AXIS] += retract_zlift;
			current_position[E_AXIS] += -retract_recover_length;
			feedrate = retract_recover_feedrate;
			retracted = false;
			prepare_move();
		}
		break;
#endif //FWRETRACT
	case 28: //G28 Home all Axis one at a time
		if (code_seen('S') && code_value() == 1)
		{
			home_delta_axis();
		}
		else
		{
			home_twice();
		}
		break;
	case 29: // G29 Calibrate print surface with automatic Z probe.
    if (z_min_check())
    {
      g29();
    }
		break;
  case 33:
    if (z_min_check())
    {
      g33();
    }
    break;
	case 90: // G90
		relative_mode = false;
		break;
	case 91: // G91
		relative_mode = true;
		break;
	case 92: // G92
		if (!code_seen(axis_codes[E_AXIS]))
			st_synchronize();
		for (int8_t i = 0; i < NUM_AXIS; i++) {
			if (code_seen(axis_codes[i])) {
				if (i == E_AXIS) {
					current_position[i] = code_value();
					plan_set_e_position(current_position[E_AXIS]);
				}
				else {
					current_position[i] = code_value() + home_offset[i];
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				}
			}
		}
		break;
	}
}

FORCE_INLINE void process_mcode(int value)
{
	unsigned long codenum; //throw away variable
	switch (value)
	{
#ifdef ULTIPANEL
	case 0: // M0 - Unconditional stop - Wait for user button press on LCD
	case 1: // M1 - Conditional stop - Wait for user button press on LCD
	{
		LCD_MESSAGE_CPGM(MSG_USERWAIT);
		codenum = 0;
		if (code_seen('P')) codenum = code_value(); // milliseconds to wait
		if (code_seen('S')) codenum = code_value() * 1000; // seconds to wait

		st_synchronize();
		previous_millis_cmd = millis();
		if (codenum > 0) {
			codenum += millis();  // keep track of when we started waiting
			while (millis()  < codenum && !lcd_clicked()) {
				manage_heater();
				manage_inactivity();
				lcd_update();
			}
		}
		else {
			while (!lcd_clicked()) {
				manage_heater();
				manage_inactivity();
				lcd_update();
			}
		}
		LCD_MESSAGE_CPGM(MSG_RESUMING);
	}
	break;

#endif
	case 3:
		st_synchronize();
		digitalWrite(LASER_PIN, HIGH);
		SERIAL_ECHOLN("Laser On");
		break;
	case 4:
		analogWrite(LASER_PIN, 8);
		break;
	case 5:
		st_synchronize();
		digitalWrite(LASER_PIN, LOW);
		SERIAL_ECHOLN("Laser Off");
		break;
	case 17:
		LCD_MESSAGE_CPGM(MSG_NO_MOVE);
		enable_x();
		enable_y();
		enable_z();
		enable_e0();
		enable_e1();
		enable_e2();
		break;

#ifdef SDSUPPORT
	case 20: // M20 - list SD card
		SERIAL_PROTOCOLLN_CPGM(MSG_BEGIN_FILE_LIST);
		card.ls();
		SERIAL_PROTOCOLLN_CPGM(MSG_END_FILE_LIST);
		break;
	case 21: // M21 - init SD card
		card.initsd();
		break;
	case 22: //M22 - release SD card
		card.release();
		break;
	case 23: //M23 - Select file
		if (curr_cmd->is_text())
		{
			strncpy(selectedFilename, curr_cmd->text, LONG_FILENAME_LENGTH - 1);
			card.openFile(selectedFilename, true);
		}
		break;
	case 24: //M24 - Start SD print
		card.startFileprint();
    starttime = millis();
		wait_heating = true;
		strncpy(previousFilename, selectedFilename, LONG_FILENAME_LENGTH - 1);
		break;
	case 25: //M25 - Pause SD print
		card.pauseSDPrint();
		break;
	case 26: //M26 - Set SD index
		if (card.cardOK && code_seen('S')) {
			card.setIndex(code_value_long());
		}
		break;
	case 27: //M27 - Get SD status
		card.getStatus();
		break;
	case 28: //M28 - Start SD write
		if (curr_cmd->is_text())
		{
			card.openFile(curr_cmd->text, false);
		}
		break;
	case 29: //M29 - Stop SD write
		//processed in write to file routine above
		 //card,saving = false;
		break;
	case 30: //M30 <filename> Delete File
		if (card.cardOK) 
		{
			card.closefile();
			if (curr_cmd->is_text())
			{
				card.removeFile(curr_cmd->text);
			}
		}
		break;
	case 32: //M32 - Select file and start SD print
		if (card.sdprinting) 
		{
			st_synchronize();
			card.closefile();
			card.sdprinting = false;
		}
		if (curr_cmd->is_text())
		{
			card.openFile(curr_cmd->text, true);
			card.startFileprint();
      starttime = millis();
		}
		break;
	case 928: //M928 - Start SD write
		if (curr_cmd->is_text())
		{
			card.openLogFile(curr_cmd->text);
		}
		break;

#endif //SDSUPPORT

	case 31: //M31 take time since the start of the SD print or an M109 command
	{
		stoptime = millis();
		char time[30];
		unsigned long t = (stoptime - starttime) / 1000;
		int sec, min;
		min = t / 60;
		sec = t % 60;
		sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
		SERIAL_ECHO_START;
		SERIAL_ECHOLN(time);
		lcd_setstatus(time);
		autotempShutdown();
	}
	break;
	case 42: //M42 -Change pin status via gcode
		if (code_seen('S'))
		{
			int pin_status = code_value();
			int pin_number = LED_PIN;
			if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
				pin_number = code_value();
			for (int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
			{
				if (sensitive_pins[i] == pin_number)
				{
					pin_number = -1;
					break;
				}
			}
#if defined(FAN_PIN) && FAN_PIN > -1
			if (pin_number == FAN_PIN)
				fanSpeed = pin_status;
#endif
			if (pin_number > -1)
			{
				pinMode(pin_number, OUTPUT);
				digitalWrite(pin_number, pin_status);
				analogWrite(pin_number, pin_status);
			}
		}
		break;
	case 104: // M104
		if (setTargetedHotend(104)) 
		{
			break;
		}
		if (code_seen('S'))
		{
			SERIAL_ECHO("temp=");
			SERIAL_ECHOLN(code_value());
			setTargetHotend(code_value());
		}
		setWatch();
		break;
	case 140: // M140 set bed temp
		if (code_seen('S')) setTargetBed(code_value());
		break;
	case 105: // M105
		if (setTargetedHotend(105)) {
			break;
		}
#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
		SERIAL_PROTOCOL_CPGM("ok T:");
		SERIAL_PROTOCOL_F(degHotend(), 1);
		SERIAL_PROTOCOL_CPGM(" /");
		SERIAL_PROTOCOL_F(degTargetHotend(), 1);
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
		SERIAL_PROTOCOL_CPGM(" B:");
		SERIAL_PROTOCOL_F(degBed(), 1);
		SERIAL_PROTOCOL_CPGM(" /");
		SERIAL_PROTOCOL_F(degTargetBed(), 1);
#endif //TEMP_BED_PIN
#else
		SERIAL_ERROR_START;
		SERIAL_ERRORLN_CPGM(MSG_ERR_NO_THERMISTORS);
#endif

		SERIAL_PROTOCOL_CPGM(" @:");
		SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));

		SERIAL_PROTOCOL_CPGM(" B@:");
		SERIAL_PROTOCOL(getHeaterPower(-1));

		SERIAL_PROTOCOLLN("");
		return;
		break;
	case 109: // M109 - Wait for extruder heater to reach target.
	{
		wait_heating = true;
		if (setTargetedHotend(109)) {
			break;
		}
		LCD_MESSAGE_CPGM(MSG_NOZZLE_HEATING);
#ifdef AUTOTEMP
		autotemp_enabled = false;
#endif
		if (code_seen('S')) {
			setTargetHotend(code_value());
			CooldownNoWait = true;
		}
		else if (code_seen('R')) {
			setTargetHotend(code_value());
			CooldownNoWait = false;
		}
#ifdef AUTOTEMP
		if (code_seen('S')) autotemp_min = code_value();
		if (code_seen('B')) autotemp_max = code_value();
		if (code_seen('F'))
		{
			autotemp_factor = code_value();
			autotemp_enabled = true;
		}
#endif

		setWatch();
		codenum = millis();

		/* See if we are heating up or cooling down */
		target_direction = isHeatingHotend(); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
		long residencyStart;
		residencyStart = -1;
		/* continue to loop until we have reached the target temp
		_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
		while (wait_heating && ((residencyStart == -1) ||
			(residencyStart >= 0 && (((unsigned int)(millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))))) {
#else
		while (target_direction ? (isHeatingHotend()) : (isCoolingHotend(tmp_extruder) && (CooldownNoWait == false))) {
#endif //TEMP_RESIDENCY_TIME
			if ((millis() - codenum) > 1000UL)
			{ //Print Temp Reading and remaining time every 1 second while heating up/cooling down
				SERIAL_PROTOCOL_CPGM("T:");
				SERIAL_PROTOCOL_F(degHotend(), 1);
				SERIAL_PROTOCOL_CPGM(" E:");
				SERIAL_PROTOCOL((int)tmp_extruder);
#ifdef TEMP_RESIDENCY_TIME
				SERIAL_PROTOCOL_CPGM(" W:");
				if (residencyStart > -1)
				{
					codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
					SERIAL_PROTOCOLLN(codenum);
				}
				else
				{
					SERIAL_PROTOCOLLN("?");
				}
#else
				SERIAL_PROTOCOLLN("");
#endif
				codenum = millis();
			}
			manage_heater();
			manage_inactivity();
			lcd_update();
#ifdef TEMP_RESIDENCY_TIME
			/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
			or when current temp falls outside the hysteresis after target temp was reached */
			if ((residencyStart == -1 && target_direction && (degHotend() >= (degTargetHotend() - TEMP_WINDOW))) ||
				(residencyStart == -1 && !target_direction && (degHotend() <= (degTargetHotend() + TEMP_WINDOW))) ||
				(residencyStart > -1 && labs(degHotend() - degTargetHotend()) > TEMP_HYSTERESIS))
			{
				residencyStart = millis();
			}
#endif //TEMP_RESIDENCY_TIME
		}
		if (wait_heating)
		{
			LCD_MESSAGE_CPGM(MSG_NOZZLE_HEATING_COMPLETE);
		}
		previous_millis_cmd = millis();
		}
	break;
	case 190: // M190 - Wait for bed heater to reach target.
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
		wait_heating = true;
		LCD_MESSAGE_CPGM(MSG_BED_HEATING);
		if (code_seen('S')) {
			setTargetBed(code_value());
			CooldownNoWait = true;
		}
		else if (code_seen('R')) {
			setTargetBed(code_value());
			CooldownNoWait = false;
		}
		codenum = millis();

		target_direction = isHeatingBed(); // true if heating, false if cooling

		while (wait_heating && (target_direction ? (isHeatingBed()) : (isCoolingBed() && (CooldownNoWait == false))))
		{
			if ((millis() - codenum) > 1000) //Print Temp Reading every 1 second while heating up.
			{
				float tt = degHotend();
				SERIAL_PROTOCOL_CPGM("T:");
				SERIAL_PROTOCOL(tt);
				SERIAL_PROTOCOL_CPGM(" E:");
				SERIAL_PROTOCOL((int)active_extruder);
				SERIAL_PROTOCOL_CPGM(" B:");
				SERIAL_PROTOCOL_F(degBed(), 1);
				SERIAL_PROTOCOLLN("");
				codenum = millis();
			}
			manage_heater();
			manage_inactivity();
			lcd_update();
		}
		if (wait_heating)
		{
			LCD_MESSAGE_CPGM(MSG_BED_DONE);
		}
		previous_millis_cmd = millis();
#endif
		break;

#if defined(FAN_PIN) && FAN_PIN > -1
	case 106: //M106 Fan On
		if (code_seen('S')) {
			fanSpeed = constrain(code_value(), 0, 255);
		}
		else {
			fanSpeed = 255;
		}
		break;
	case 107: //M107 Fan Off
		fanSpeed = 0;
		break;
#endif //FAN_PIN
#ifdef BARICUDA
		// PWM for HEATER_1_PIN
#if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
	case 126: //M126 valve open
		if (code_seen('S')) {
			ValvePressure = constrain(code_value(), 0, 255);
		}
		else {
			ValvePressure = 255;
		}
		break;
	case 127: //M127 valve closed
		ValvePressure = 0;
		break;
#endif //HEATER_1_PIN

		// PWM for HEATER_2_PIN
#if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
	case 128: //M128 valve open
		if (code_seen('S')) {
			EtoPPressure = constrain(code_value(), 0, 255);
		}
		else {
			EtoPPressure = 255;
		}
		break;
	case 129: //M129 valve closed
		EtoPPressure = 0;
		break;
#endif //HEATER_2_PIN
#endif

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
	case 80: // M80 - Turn on Power Supply
		SET_OUTPUT(PS_ON_PIN); //GND
		WRITE(PS_ON_PIN, PS_ON_AWAKE);
#ifdef ULTIPANEL
		powersupply = true;
		lcd_update();
#endif
		break;
#endif

	case 81: // M81 - Turn off Power Supply
		disable_heater();
		st_synchronize();
		disable_e0();
		disable_e1();
		disable_e2();
		finishAndDisableSteppers();
		fanSpeed = 0;
		delay(1000); // Wait a little before to switch off
#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
		st_synchronize();
		suicide();
#elif defined(PS_ON_PIN) && PS_ON_PIN > -1
		SET_OUTPUT(PS_ON_PIN);
		WRITE(PS_ON_PIN, PS_ON_ASLEEP);
#endif
#ifdef ULTIPANEL
		powersupply = false;
		LCD_MESSAGE_CPGM(MACHINE_NAME " " MSG_OFF ".");
		lcd_update();
#endif
		break;

	case 82:
		axis_relative_modes[3] = false;
		break;
	case 83:
		axis_relative_modes[3] = true;
		break;
	case 18: //compatibility
	case 84: // M84
		if (code_seen('S')) {
			stepper_inactive_time = code_value() * 1000;
		}
		else
		{
			bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])) || (code_seen(axis_codes[3])));
			if (all_axis)
			{
				st_synchronize();
				disable_e0();
				disable_e1();
				disable_e2();
				finishAndDisableSteppers();
			}
			else
			{
				st_synchronize();
				if (code_seen('X')) disable_x();
				if (code_seen('Y')) disable_y();
				if (code_seen('Z')) disable_z();
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
				if (code_seen('E')) {
					disable_e0();
					disable_e1();
					disable_e2();
				}
#endif
			}
		}
		break;
	case 85: // M85
		code_seen('S');
		max_inactive_time = code_value() * 1000;
		break;
	case 92: // M92
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				if (i == 3) { // E
					float value = code_value();
					if (value < 20.0) {
						float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
						max_e_jerk *= factor;
						max_feedrate[i] *= factor;
						axis_steps_per_sqr_second[i] *= factor;
					}
					axis_steps_per_unit[i] = value;
				}
				else {
					axis_steps_per_unit[i] = code_value();
				}
			}
		}
		break;
	case 115: // M115
		SERIAL_PROTOCOL_CPGM(MSG_M115_REPORT);
		break;
	case 117: // M117 display message
		if (curr_cmd->is_text())
		{
			lcd_setstatus(curr_cmd->text);
		}
		break;
	case 114: // M114
		SERIAL_PROTOCOL_CPGM("X:");
		SERIAL_PROTOCOL(current_position[X_AXIS]);
		SERIAL_PROTOCOL_CPGM("Y:");
		SERIAL_PROTOCOL(current_position[Y_AXIS]);
		SERIAL_PROTOCOL_CPGM("Z:");
		SERIAL_PROTOCOL(current_position[Z_AXIS]);
		SERIAL_PROTOCOL_CPGM("E:");
		SERIAL_PROTOCOL(current_position[E_AXIS]);

		SERIAL_PROTOCOL_CPGM(MSG_COUNT_X);
		SERIAL_PROTOCOL(float(st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS]);
		SERIAL_PROTOCOL_CPGM("Y:");
		SERIAL_PROTOCOL(float(st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS]);
		SERIAL_PROTOCOL_CPGM("Z:");
		SERIAL_PROTOCOL(float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS]);

		SERIAL_PROTOCOLLN("");
		break;
	case 120: // M120
		enable_endstops(false);
		break;
	case 121: // M121
		enable_endstops(true);
		break;
	case 119: // M119
		SERIAL_PROTOCOLLN(MSG_M119_REPORT);
#if defined(X_MIN_PIN) && X_MIN_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_X_MIN);
		SERIAL_PROTOCOLLN(((READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_X_MAX);
		SERIAL_PROTOCOLLN(((READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_Y_MIN);
		SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_Y_MAX);
		SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_Z_MIN);
		SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
		SERIAL_PROTOCOL_CPGM(MSG_Z_MAX);
		SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
#endif
		break;
		//TODO: update for all axis, use for loop
	case 201: // M201
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				max_acceleration_units_per_sq_second[i] = code_value();
			}
		}
		// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
		break;
#if 0 // Not used for Sprinter/grbl gen6
	case 202: // M202
		for (int8_t i = 0; i < NUM_AXIS; i++) {
			if (code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
		}
		break;
#endif
	case 203: // M203 max feedrate mm/sec
		for (int8_t i = 0; i < NUM_AXIS; i++) {
			if (code_seen(axis_codes[i])) max_feedrate[i] = code_value();
		}
		break;
	case 204: // M204 acclereration S normal moves T filmanent only moves
	{
		if (code_seen('S')) acceleration = code_value();
		if (code_seen('T')) retract_acceleration = code_value();
	}
	break;
	case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
	{
		if (code_seen('S')) minimumfeedrate = code_value();
		if (code_seen('T')) mintravelfeedrate = code_value();
		if (code_seen('B')) minsegmenttime = code_value();
		if (code_seen('X')) max_xy_jerk = code_value();
		if (code_seen('Z')) max_z_jerk = code_value();
		if (code_seen('E')) max_e_jerk = code_value();
	}
	break;
	case 206: // M206 additional homeing offset
		for (int8_t i = 0; i < 3; i++)
		{
			if (code_seen(axis_codes[i])) home_offset[i] = code_value();
		}
		break;
#ifdef FWRETRACT
	case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
	{
		if (code_seen('S'))
		{
			retract_length = code_value();
		}
		if (code_seen('F'))
		{
			retract_feedrate = code_value();
		}
		if (code_seen('Z'))
		{
			retract_zlift = code_value();
		}
	}break;
	case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
	{
		if (code_seen('S'))
		{
			retract_recover_length = code_value();
		}
		if (code_seen('F'))
		{
			retract_recover_feedrate = code_value();
		}
	}break;
	case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
	{
		if (code_seen('S'))
		{
			int t = code_value();
			switch (t)
			{
			case 0: autoretract_enabled = false; retracted = false; break;
			case 1: autoretract_enabled = true; retracted = false; break;
			default:
				SERIAL_ECHO_START;
				SERIAL_ECHO_CPGM(MSG_UNKNOWN_COMMAND);
				SERIAL_ECHO(cmdbuffer[bufindr]);
				SERIAL_ECHOLN_CPGM("\"");
			}
		}

	}break;
#endif // FWRETRACT
#if EXTRUDERS > 1
	case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
	{
		if (setTargetedHotend(218)) {
			break;
		}
		if (code_seen('X'))
		{
			extruder_offset[X_AXIS][tmp_extruder] = code_value();
		}
		if (code_seen('Y'))
		{
			extruder_offset[Y_AXIS][tmp_extruder] = code_value();
		}   
		SERIAL_ECHO_START;
		SERIAL_ECHO_CPGM(MSG_HOTEND_OFFSET);
		for (tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
		{
			SERIAL_ECHO(" ");
			SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
			SERIAL_ECHO(",");
			SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
		}
		SERIAL_ECHOLN("");
	}break;
#endif
	case 220: // M220 S<factor in percent>- set speed factor override percentage
	{
		if (code_seen('S'))
		{
			feedmultiply = code_value();
		}
	}
	break;
	case 221: // M221 S<factor in percent>- set extrude factor override percentage
	{
		if (code_seen('S'))
		{
			extrudemultiply = code_value();
		}
	}
	break;

#if NUM_SERVOS > 0
	case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
	{
		int servo_index = -1;
		int servo_position = 0;
		if (code_seen('P'))
			servo_index = code_value();
		if (code_seen('S')) {
			servo_position = code_value();
			if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
				servos[servo_index].write(servo_position);
			}
			else {
				SERIAL_ECHO_START;
				SERIAL_ECHO("Servo ");
				SERIAL_ECHO(servo_index);
				SERIAL_ECHOLN(" out of range");
			}
		}
		else if (servo_index >= 0) {
			SERIAL_PROTOCOL(MSG_OK);
			SERIAL_PROTOCOL(" Servo ");
			SERIAL_PROTOCOL(servo_index);
			SERIAL_PROTOCOL(": ");
			SERIAL_PROTOCOL(servos[servo_index].read());
			SERIAL_PROTOCOLLN("");
		}
	}
	break;
#endif // NUM_SERVOS > 0

#if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
	case 300: // M300
	{
		int beepS = code_seen('S') ? code_value() : 110;
		int beepP = code_seen('P') ? code_value() : 1000;
		if (beepS > 0)
		{
			lcd_buzz(beepS, beepP);
		}
		else
		{
			delay(beepP);
		}
	}
	break;
#endif // M300

#ifdef PIDTEMP
	case 301: // M301
	{
		if (code_seen('P')) Kp = code_value();
		if (code_seen('I')) Ki = scalePID_i(code_value());
		if (code_seen('D')) Kd = scalePID_d(code_value());

#ifdef PID_ADD_EXTRUSION_RATE
		if (code_seen('C')) Kc = code_value();
#endif

		updatePID();
		SERIAL_PROTOCOL(MSG_OK);
		SERIAL_PROTOCOL(" p:");
		SERIAL_PROTOCOL(Kp);
		SERIAL_PROTOCOL(" i:");
		SERIAL_PROTOCOL(unscalePID_i(Ki));
		SERIAL_PROTOCOL(" d:");
		SERIAL_PROTOCOL(unscalePID_d(Kd));
#ifdef PID_ADD_EXTRUSION_RATE
		SERIAL_PROTOCOL(" c:");
		//Kc does not have scaling applied above, or in resetting defaults
		SERIAL_PROTOCOL(Kc);
#endif
		SERIAL_PROTOCOLLN("");
	}
	break;
#endif //PIDTEMP
#ifdef PIDTEMPBED
	case 304: // M304
	{
		if (code_seen('P')) bedKp = code_value();
		if (code_seen('I')) bedKi = scalePID_i(code_value());
		if (code_seen('D')) bedKd = scalePID_d(code_value());

		updatePID();
		SERIAL_PROTOCOL(MSG_OK);
		SERIAL_PROTOCOL(" p:");
		SERIAL_PROTOCOL(bedKp);
		SERIAL_PROTOCOL(" i:");
		SERIAL_PROTOCOL(unscalePID_i(bedKi));
		SERIAL_PROTOCOL(" d:");
		SERIAL_PROTOCOL(unscalePID_d(bedKd));
		SERIAL_PROTOCOLLN("");
	}
	break;
#endif //PIDTEMP
	case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
	{
#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
		const uint8_t NUM_PULSES = 16;
		const float PULSE_LENGTH = 0.01524;
		for (int i = 0; i < NUM_PULSES; i++) {
			WRITE(PHOTOGRAPH_PIN, HIGH);
			_delay_ms(PULSE_LENGTH);
			WRITE(PHOTOGRAPH_PIN, LOW);
			_delay_ms(PULSE_LENGTH);
		}
		delay(7.33);
		for (int i = 0; i < NUM_PULSES; i++) {
			WRITE(PHOTOGRAPH_PIN, HIGH);
			_delay_ms(PULSE_LENGTH);
			WRITE(PHOTOGRAPH_PIN, LOW);
			_delay_ms(PULSE_LENGTH);
		}
#endif
	}
	break;
#ifdef DOGLCD
	case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
	{
		if (code_seen('C')) {
			lcd_setcontrast(((int)code_value()) & 63);
		}
		SERIAL_PROTOCOL_CPGM("lcd contrast value: ");
		SERIAL_PROTOCOL(lcd_contrast);
		SERIAL_PROTOCOLLN("");
	}
	break;
#endif
#ifdef PREVENT_DANGEROUS_EXTRUDE
	case 302: // allow cold extrudes, or set the minimum extrude temperature
	{
		float temp = .0;
		if (code_seen('S')) temp = code_value();
		set_extrude_min_temp(temp);
	}
	break;
#endif
	case 303: // M303 PID autotune
	{
		float temp = 150.0;
		int e = 0;
		int c = 5;
		if (code_seen('E')) e = code_value();
		if (e<0)
			temp = 70;
		if (code_seen('S')) temp = code_value();
		if (code_seen('C')) c = code_value();
		PID_autotune(temp, e, c);
	}
	break;
	case 400: // M400 finish all moves
	{
		st_synchronize();
	}
	break;
	case 500: // M500 Store settings in EEPROM
	{
		Config_StoreSettings();
	}
	break;
	case 501: // M501 Read settings from EEPROM
	{
		Config_RetrieveSettings();
	}
	break;
	case 502: // M502 Revert to default settings
	{
		Config_ResetDefault();
	}
	break;
	case 503: // M503 print settings currently in memory
	{
		Config_PrintSettings();
	}
	break;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
	case 540:
	{
		if (code_seen('S')) abort_on_endstop_hit = code_value() > 0;
	}
	break;
#endif
	case 600://pause move 
		st_synchronize();
		saved_feedmultiply = feedmultiply;
		saved_feedrate = feedrate;
		lastPos[X_AXIS] = current_position[X_AXIS];
		lastPos[Y_AXIS] = current_position[Y_AXIS];
		lastPos[Z_AXIS] = current_position[Z_AXIS];
		lastPos[E_AXIS] = current_position[E_AXIS];
		destination[X_AXIS] = lastPos[X_AXIS];
		destination[Y_AXIS] = lastPos[Y_AXIS];
		destination[Z_AXIS] = lastPos[Z_AXIS] + 32;
		destination[E_AXIS] = lastPos[E_AXIS] - 10;
		prepare_move();
		st_synchronize();
		break;
	case 601://resume (with pause move)
		st_synchronize();
		current_position[E_AXIS] = lastPos[E_AXIS];
		plan_set_e_position(current_position[E_AXIS]);
		st_synchronize();
		feedmultiply = saved_feedmultiply;
		feedrate = saved_feedrate;
		destination[X_AXIS] = lastPos[X_AXIS];
		destination[Y_AXIS] = lastPos[Y_AXIS];
		destination[Z_AXIS] = lastPos[Z_AXIS];
		destination[E_AXIS] = lastPos[E_AXIS];
		prepare_move();
		st_synchronize();
		break;  
  case 666:
    if (code_seen('X'))
      endstop_adj[X_AXIS] = code_value();
    if (code_seen('Y'))
      endstop_adj[Y_AXIS] = code_value();
    if (code_seen('Z'))
      endstop_adj[Z_AXIS] = code_value();
    break;
	case 907: // M907 Set digital trimpot motor current using axis codes.
	{
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
		for (int i = 0; i<NUM_AXIS; i++) if (code_seen(axis_codes[i])) digipot_current(i, code_value());
		if (code_seen('B')) digipot_current(4, code_value());
		if (code_seen('S')) for (int i = 0; i <= 4; i++) digipot_current(i, code_value());
#endif
	}
	break;
	case 908: // M908 Control digital trimpot directly.
	{
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
		uint8_t channel, current;
		if (code_seen('P')) channel = code_value();
		if (code_seen('S')) current = code_value();
		digitalPotWrite(channel, current);
#endif
	}
	break;
	case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
	{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
		if (code_seen('S')) for (int i = 0; i <= 4; i++) microstep_mode(i, code_value());
		for (int i = 0; i<NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_mode(i, (uint8_t)code_value());
		if (code_seen('B')) microstep_mode(4, code_value());
		microstep_readings();
#endif
	}
	break;
	case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
	{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
		if (code_seen('S')) switch ((int)code_value())
		{
		case 1:
			for (int i = 0; i<NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, code_value(), -1);
			if (code_seen('B')) microstep_ms(4, code_value(), -1);
			break;
		case 2:
			for (int i = 0; i<NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, -1, code_value());
			if (code_seen('B')) microstep_ms(4, -1, code_value());
			break;
		}
		microstep_readings();
#endif
	}
	break;
	case 999: // M999: Restart after being stopped
		Stopped = false;
		lcd_reset_alert_level();
		gcode_LastN = Stopped_gcode_LastN;
		FlushSerialRequestResend();
		break;
	}
}

FORCE_INLINE void process_tcode(int value)
{
	tmp_extruder = value;
	if (tmp_extruder >= EXTRUDERS) {
		SERIAL_ECHO_START;
		SERIAL_ECHO("T");
		SERIAL_ECHO(tmp_extruder);
		SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
	}
	else {
		//boolean make_move = false;
		if (code_seen('F')) {
			//make_move = true;
			next_feedrate = code_value();
			if (next_feedrate > 0.0) {
				feedrate = next_feedrate;
			}
		}
#if EXTRUDERS > 1
		if (tmp_extruder != active_extruder) {
			// Save current position to return to after applying extruder offset
			memcpy(destination, current_position, sizeof(destination));
   
			// Set the new active extruder and position
			active_extruder = tmp_extruder;
		}
#endif
		SERIAL_ECHO_START;
		SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
		SERIAL_PROTOCOLLN((int)active_extruder);
		switch (active_extruder)
		{
		case 0:
			LCD_MESSAGE_CPGM("Extruder 0 Active");
			break;
		case 1:
			LCD_MESSAGE_CPGM("Extruder 1 Active");
			break;
		}
	}
}
void process_commands()
{
  curr_cmd = &cmds[bufindr];
  char code = curr_cmd->code;
  int value = curr_cmd->value;
  inc_rd_idx();

  if(code == 'G')
  {
	  process_gcode(value);
  }
  else if(code == 'M')
  {
	  process_mcode(value);
  }
  else if(code == 'T')
  {
	  process_tcode(value);
  }
  else
  {
	  SERIAL_ECHO_START;
	  SERIAL_ECHO_CPGM(MSG_UNKNOWN_COMMAND);
	  SERIAL_ECHO(curr_cmd);
	  SERIAL_ECHOLN_CPGM("\"");
  }
  ClearToSend();
} //end process_commands

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOL_CPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLN_CPGM(MSG_OK);
}

void get_coordinates()
{
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

#ifdef DELTA
void calculate_delta(float cartesian[3])
{
  delta[X_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower[A_AXIS][X_AXIS] - cartesian[X_AXIS])
                       - sq(delta_tower[A_AXIS][Y_AXIS] - cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower[B_AXIS][X_AXIS] - cartesian[X_AXIS])
                       - sq(delta_tower[B_AXIS][Y_AXIS] - cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower[C_AXIS][X_AXIS] - cartesian[X_AXIS])
                       - sq(delta_tower[C_AXIS][Y_AXIS] - cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];

  /*
  SERIAL_ECHO_CPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHO_CPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHO_CPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHO_CPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHO_CPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHO_CPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
}

// Adjust print surface height by linear interpolation over the bed_level array.
void adjust_delta(float cartesian[3])
{
  float grid_x = max(-2.999, min(2.999, cartesian[X_AXIS] / AUTOLEVEL_GRID));
  float grid_y = max(-2.999, min(2.999, cartesian[Y_AXIS] / AUTOLEVEL_GRID));
  int floor_x = floor(grid_x);
  int floor_y = floor(grid_y);
  float ratio_x = grid_x - floor_x;
  float ratio_y = grid_y - floor_y;
  float z1 = bed_level[floor_x+3][floor_y+3] + z_offset;
  float z2 = bed_level[floor_x+3][floor_y+4] + z_offset;
  float z3 = bed_level[floor_x+4][floor_y+3] + z_offset;
  float z4 = bed_level[floor_x+4][floor_y+4] + z_offset;
  float left = (1-ratio_y)*z1 + ratio_y*z2;
  float right = (1-ratio_y)*z3 + ratio_y*z4;
  float offset = (1-ratio_x)*left + ratio_x*right;

  delta[X_AXIS] += offset;
  delta[Y_AXIS] += offset;
  delta[Z_AXIS] += offset;

  /*
  SERIAL_ECHO_CPGM("grid_x="); SERIAL_ECHO(grid_x);
  SERIAL_ECHO_CPGM(" grid_y="); SERIAL_ECHO(grid_y);
  SERIAL_ECHO_CPGM(" floor_x="); SERIAL_ECHO(floor_x);
  SERIAL_ECHO_CPGM(" floor_y="); SERIAL_ECHO(floor_y);
  SERIAL_ECHO_CPGM(" ratio_x="); SERIAL_ECHO(ratio_x);
  SERIAL_ECHO_CPGM(" ratio_y="); SERIAL_ECHO(ratio_y);
  SERIAL_ECHO_CPGM(" z1="); SERIAL_ECHO(z1);
  SERIAL_ECHO_CPGM(" z2="); SERIAL_ECHO(z2);
  SERIAL_ECHO_CPGM(" z3="); SERIAL_ECHO(z3);
  SERIAL_ECHO_CPGM(" z4="); SERIAL_ECHO(z4);
  SERIAL_ECHO_CPGM(" left="); SERIAL_ECHO(left);
  SERIAL_ECHO_CPGM(" right="); SERIAL_ECHO(right);
  SERIAL_ECHO_CPGM(" offset="); SERIAL_ECHOLN(offset);
  */
}
#endif

void prepare_move_raw()
{
  previous_millis_cmd = millis();
  calculate_delta(destination);
  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply / 60 / 100.0, CURRENT_EXTRUDER);
  set_current_to_destination();
}

void prepare_long_move_raw()
{
  previous_millis_cmd = millis();
  float diff[NUM_AXIS];
  int start = 0;
  float real_feedrate = feedrate * feedmultiply;
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    diff[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(diff[X_AXIS]) + sq(diff[Y_AXIS]) + sq(diff[Z_AXIS]));
  if (cartesian_mm < 0.000001)
  {
    cartesian_mm = abs(diff[E_AXIS]);
    start = 3;
    real_feedrate = feedrate * 100;
  }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / real_feedrate;
  int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));
  for (int s = 1; s <= steps; s++)
  {
    float fraction = float(s) / float(steps);
    for (int8_t i = start; i < NUM_AXIS; i++)
    {
      destination[i] = current_position[i] + diff[i] * fraction;
    }
    if (start == 0)
    {
      calculate_delta(destination);
    }
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], real_feedrate / 6000.0, CURRENT_EXTRUDER);
  }
  set_current_to_destination();
}

void prepare_move()
{
  previous_millis_cmd = millis();
  float diff[NUM_AXIS];
  int start = 0;
  float real_feedrate = feedrate * feedmultiply;
  for (int8_t i=0; i < NUM_AXIS; i++) 
  {
	  diff[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(diff[X_AXIS]) + sq(diff[Y_AXIS]) + sq(diff[Z_AXIS]));
  if (cartesian_mm < 0.000001) 
  { 
	  cartesian_mm = abs(diff[E_AXIS]);
	  start = 3;
    real_feedrate = feedrate * 100;
  }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / real_feedrate;
  int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));
  // SERIAL_ECHO_CPGM("mm="); SERIAL_ECHO(cartesian_mm);
  // SERIAL_ECHO_CPGM(" seconds="); SERIAL_ECHO(seconds);
  // SERIAL_ECHO_CPGM(" steps="); SERIAL_ECHOLN(steps);
  for (int s = 1; s <= steps; s++) 
  {
    float fraction = float(s) / float(steps);
    for(int8_t i = start; i < NUM_AXIS; i++) 
	  {
      destination[i] = current_position[i] + diff[i] * fraction;
    }
	  if (start == 0)
	  {
	    calculate_delta(destination);
	    adjust_delta(destination);
	  }
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], real_feedrate/6000.0, CURRENT_EXTRUDER);
  }
  set_current_to_destination();
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, CURRENT_EXTRUDER);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
      #if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
       || !READ(X2_ENABLE_PIN)
      #endif
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  #if defined(KILL_PIN) && KILL_PIN > -1
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                      current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLN_CPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGE_CPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) 
  {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLN_CPGM(MSG_ERR_STOPPED);
    LCD_MESSAGE_CPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}
