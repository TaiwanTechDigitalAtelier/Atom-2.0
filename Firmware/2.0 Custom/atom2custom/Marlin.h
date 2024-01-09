// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
  #define MYSERIAL Serial
#else
  #define MYSERIAL MSerial
#endif
#define SERIAL_CHAR(x) ((void)MYSERIAL.write(x))
#define SERIAL_PROTOLN() (MYSERIAL.write('\n'))
#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print(x,y))
#define SERIAL_PROTOCOL_PGM(x) (serialprintPGM(x))
#define SERIAL_PROTOCOL_CPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.print(x),SERIAL_PROTOLN())
#define SERIAL_PROTOCOLLN_PGM(x) (serialprintPGM(x),SERIAL_PROTOLN())
#define SERIAL_PROTOCOLLN_CPGM(x) (serialprintPGM(PSTR(x)),SERIAL_PROTOLN())
#define SERIAL_PROTOCOL_SP(C) serial_spaces(C)
#define SERIAL_PROTOCOLPAIRCPGM(name, value) (serial_echopair_P(PSTR(name),(value)))

const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERROR_CPGM(x) SERIAL_PROTOCOL_CPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLN_CPGM(x) SERIAL_PROTOCOLLN_CPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHO_CPGM(x) SERIAL_PROTOCOL_CPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLN_CPGM(x) SERIAL_PROTOCOLLN_CPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}
FORCE_INLINE void serial_spaces(uint8_t count)
{ 
  while (count--)
  {
    MYSERIAL.write(' ');
  }
}
void get_command();
void process_commands();

void manage_inactivity();

#if defined(DUAL_X_CARRIAGE) && defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1 \
    && defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
  #define  enable_x() do { WRITE(X_ENABLE_PIN, X_ENABLE_ON); WRITE(X2_ENABLE_PIN, X_ENABLE_ON); } while (0)
  #define disable_x() do { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); WRITE(X2_ENABLE_PIN,!X_ENABLE_ON); } while (0)
#elif defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif

#define UNUSED(x) (void)(x)
#define UNUSED_ __attribute__((unused))

enum AxisEnum {X_AXIS=0, A_AXIS = 0, Y_AXIS=1, B_AXIS = 1, Z_AXIS=2, C_AXIS = 2, E_AXIS=3};

#define ABC 3
#define XYZ 3

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RADIANS(d) ((d)*M_PI/180.0)
#define DEGREES(r) ((r)*180.0/M_PI)

#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)

#define ATAN2(y, x) atan2(y, x)
#define FABS(x)     fabs(x)
#define POW(x, y)   pow(x, y)
#define SQRT(x)     sqrt(x)
#define CEIL(x)     ceil(x)
#define FLOOR(x)    floor(x)
#define LROUND(x)   lround(x)

#define CEILING(x,y) (((x) + (y) - 1) / (y))

#define MIN3(a, b, c)       min(min(a, b), c)
#define MIN4(a, b, c, d)    min(MIN3(a, b, c), d)
#define MIN5(a, b, c, d, e) min(MIN4(a, b, c, d), e)
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)
#define MAX5(a, b, c, d, e) max(MAX4(a, b, c, d), e)

#define ZERO(a) memset(a,0,sizeof(a))
#define COPY(a,b) memcpy(a,b,min(sizeof(a),sizeof(b)))

#define LOOP_S_LE_N(VAR, S, N) for (uint8_t VAR=S; VAR<=N; VAR++)
#define LOOP_XYZ(VAR) LOOP_S_LE_N(VAR, X_AXIS, Z_AXIS)

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
#ifdef DELTA
void calculate_delta(float cartesian[3]);
void adjust_delta(float cartesian[3]);
extern float delta[3];
extern float delta_diagonal_rod;
extern float delta_diagonal_rod_2;
extern float delta_radius;
extern float delta_calibration_radius;
extern float delta_tower_angle_trim[ABC];
extern float endstop_adj[ABC];
extern float delta_tower[ABC][2];
#endif
void prepare_move_raw();
void prepare_long_move_raw();
void prepare_move();
void kill();
void Stop();

bool IsStopped();
void clear_cmds_buffer();
void enquecommand(const char *cmd, bool sd = true); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd, bool sd = true); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
//void clamp_to_software_endstops(float target[3]);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float home_offset[3];
extern float min_pos[3];
extern float max_pos[3];
extern float z_offset;
extern float move_down_rate;
extern float bed_level_rate;
extern float bed_level[7][7];
extern float home_pos[3];
extern char cmdbuffer[MAX_CMD_SIZE];
extern int fanSpeed;
extern float input_size;
extern float output_size;
#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long currenttime;
extern unsigned long stoptime;
extern bool wait_heating;
extern uint16_t SdfileCount;
extern char previousFilename[];
extern char selectedFilename[];

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern int atom_version;
extern char *atom_version_name[];
extern char *extruder_side[];
extern bool Z_MIN_ENDSTOP_INVERTING;
extern bool extruder_inverting;
extern int load_filament_length;
extern int back_to_jct_length;
extern int unload_filament_length;

#define CURRENT_EXTRUDER active_extruder^extruder_inverting

extern void update_atom_settings();
extern void home_twice();
extern void home_delta_axis();
extern bool z_min_check();
extern void g29();
extern void g33();
extern void update_delta_diagonal_rod_2();
extern void reset_bed_level();
extern void print_bed_level();
extern void recalc_delta_settings();
#endif
