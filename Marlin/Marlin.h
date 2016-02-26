// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline
/**
 * Compiler warning on unused varable.
 */
#define UNUSED(x) (void) (x)

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

#ifndef SANITYCHECK_H
  #error Your Configuration.h and Configuration_adv.h files are outdated!
#endif

#include "Arduino.h"

typedef unsigned long millis_t;

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#ifdef USBCON
  #include "HardwareSerial.h"
#endif

#include "MarlinSerial.h"

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef USBCON
  #if ENABLED(BLUETOOTH)
    #define MYSERIAL bluetoothSerial
  #else
    #define MYSERIAL Serial
  #endif // BLUETOOTH
#else
  #define MYSERIAL customizedSerial
#endif

#define SERIAL_CHAR(x) MYSERIAL.write(x)
#define SERIAL_EOL SERIAL_CHAR('\n')

#define SERIAL_PROTOCOLCHAR(x) SERIAL_CHAR(x)
#define SERIAL_PROTOCOL(x) MYSERIAL.print(x)
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y)
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLN(x) do{ MYSERIAL.print(x); SERIAL_EOL; }while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{ serialprintPGM(PSTR(x)); SERIAL_EOL; }while(0)


extern const char errormagic[] PROGMEM;
extern const char echomagic[] PROGMEM;

#define SERIAL_ERROR_START serialprintPGM(errormagic)
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic)
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) do{ serial_echopair_P(PSTR(name),(value)); }while(0)

void serial_echopair_P(const char* s_P, int v);
void serial_echopair_P(const char* s_P, long v);
void serial_echopair_P(const char* s_P, float v);
void serial_echopair_P(const char* s_P, double v);
void serial_echopair_P(const char* s_P, unsigned long v);


// Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char* str) {
  char ch;
  while ((ch = pgm_read_byte(str))) {
    MYSERIAL.write(ch);
    str++;
  }
}

void get_command();

void idle(); // the standard idle routine calls manage_inactivity(false)

void manage_inactivity(bool ignore_stepper_queue = false);

#if HAS_X_ENABLE
  #define  enable_x() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_x() { X_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if HAS_Y_ENABLE
  #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #define  enable_y() { Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #else
    #define  enable_y() Y_ENABLE_WRITE( Y_ENABLE_ON)
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #endif
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if HAS_U_ENABLE
  #define  enable_u() U_ENABLE_WRITE( U_ENABLE_ON)
  #define disable_u() { U_ENABLE_WRITE(!U_ENABLE_ON); axis_known_position[U_AXIS] = false; }
#else
  #define enable_u() ;
  #define disable_u() ;
#endif

#if HAS_V_ENABLE
  #define  enable_v() V_ENABLE_WRITE( V_ENABLE_ON)
  #define disable_v() { V_ENABLE_WRITE(!V_ENABLE_ON); axis_known_position[V_AXIS] = false; }
#else
  #define enable_v() ;
  #define disable_v() ;
#endif


/**
 * The axis order in all axis related arrays is X, Y, U, V
 */
#define NUM_AXIS 4

/**
 * Axis indices as enumerated constants
 *
 * A_AXIS and B_AXIS are used by COREXY printers
 * X_HEAD and Y_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 */
enum AxisEnum {X_AXIS = 0, Y_AXIS = 1, U_AXIS = 2, V_AXIS = 3,
							 X_HEAD = 4, Y_HEAD = 5, U_HEAD = 6, V_HEAD = 7};
enum VirtualAxisEnum {A_AXIS = 0, B_AXIS = 1, C_AXIS = 2, D_AXIS = 3};

enum EndstopEnum {X_MIN = 0, Y_MIN = 1, U_MIN = 2, V_MIN = 3,
									X_MAX = 4, Y_MAX = 5, U_MAX = 6, V_MAX = 7};

void enable_all_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ok_to_send();

void reset_bed_level();
void prepare_move();
void kill(const char*);
void Stop();

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void filrunout();
#endif

/**
 * Debug flags - not yet widely applied
 */
enum DebugFlags {
  DEBUG_ECHO          = BIT(0),
  DEBUG_INFO          = BIT(1),
  DEBUG_ERRORS        = BIT(2),
  DEBUG_DRYRUN        = BIT(3),
  DEBUG_COMMUNICATION = BIT(4),
  DEBUG_LEVELING      = BIT(5)
};
extern uint8_t marlin_debug_flags;

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

bool enqueuecommand(const char* cmd); //put a single ASCII command at the end of the current buffer or return false when it is full
void enqueuecommands_P(const char* cmd); //put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

extern bool axis_relative_modes[];
extern int feedrate_multiplier;
extern bool volumetric_enabled;
extern int extruder_multiplier[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float filament_size[EXTRUDERS]; // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS];
extern float home_offset[4]; // axis[n].home_offset
extern float min_pos[4]; // axis[n].min_pos
extern float max_pos[4]; // axis[n].max_pos
extern bool axis_known_position[4]; // axis[n].is_known

#if ENABLED(DELTA)
  extern float delta[3];
  extern float endstop_adj[3]; // axis[n].endstop_adj
  extern float delta_radius;
  #ifndef DELTA_RADIUS_TRIM_TOWER_1
    #define DELTA_RADIUS_TRIM_TOWER_1 0.0
  #endif
  #ifndef DELTA_RADIUS_TRIM_TOWER_2
    #define DELTA_RADIUS_TRIM_TOWER_2 0.0
  #endif
  #ifndef DELTA_RADIUS_TRIM_TOWER_3
    #define DELTA_RADIUS_TRIM_TOWER_3 0.0
  #endif
  extern float delta_diagonal_rod;
  #ifndef DELTA_DIAGONAL_ROD_TRIM_TOWER_1
    #define DELTA_DIAGONAL_ROD_TRIM_TOWER_1 0.0
  #endif
  #ifndef DELTA_DIAGONAL_ROD_TRIM_TOWER_2
    #define DELTA_DIAGONAL_ROD_TRIM_TOWER_2 0.0
  #endif
  #ifndef DELTA_DIAGONAL_ROD_TRIM_TOWER_3
    #define DELTA_DIAGONAL_ROD_TRIM_TOWER_3 0.0
  #endif
  extern float delta_segments_per_second;
  void calculate_delta(float cartesian[3]);
  void recalc_delta_settings(float radius, float diagonal_rod);
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    extern int delta_grid_spacing[2];
    void adjust_delta(float cartesian[3]);
  #endif
#elif ENABLED(SCARA)
  extern float axis_scaling[3];  // Build size scaling
  void calculate_delta(float cartesian[3]);
  void calculate_SCARA_forward_Transform(float f_scara[3]);
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  extern float z_endstop_adj;
#endif

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  extern float zprobe_zoffset;
#endif

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  extern float extrude_min_temp;
#endif

extern int fanSpeed;

#if ENABLED(BARICUDA)
  extern int ValvePressure;
  extern int EtoPPressure;
#endif

#if ENABLED(FAN_SOFT_PWM)
  extern unsigned char fanSpeedSoftPwm;
#endif

#if ENABLED(FILAMENT_SENSOR)
  extern float filament_width_nominal;  //holds the theoretical filament diameter ie., 3.00 or 1.75
  extern bool filament_sensor;  //indicates that filament sensor readings should control extrusion
  extern float filament_width_meas; //holds the filament diameter as accurately measured
  extern signed char measurement_delay[];  //ring buffer to delay measurement
  extern int delay_index1, delay_index2;  //ring buffer index. used by planner, temperature, and main code
  extern float delay_dist; //delay distance counter
  extern int meas_delay_cm; //delay distance
#endif

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  extern int lpq_len;
#endif

#if ENABLED(FWRETRACT)
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS]; // extruder[n].retracted
  extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

extern millis_t print_job_start_ms;
extern millis_t print_job_stop_ms;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current(int channel, float current);
  extern void digipot_i2c_init();
#endif

extern void calculate_volumetric_multipliers();

#endif //MARLIN_H
