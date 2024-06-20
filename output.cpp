#include "../../communication/SimpleFOCDebug.h"
#include "../common/base_classes/BLDCDriver.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/base_classes/Sensor.h"
#include "../common/base_classes/StepperDriver.h"
#include "../common/defaults.h"
#include "../common/foc_utils.h"
#include "../common/lowpass_filter.h"
#include "../common/pid.h"
#include "../common/time_utils.h"
#include "../communication/SimpleFOCDebug.h"
#include "../defaults.h"
#include "../foc_utils.h"
#include "../lowpass_filter.h"
#include "../pid.h"
#include "../time_utils.h"
#include "Arduino.h"
#include "BLDCDriver3PWM.h"
#include "BLDCDriver6PWM.h"
#include "Commander.h"
#include "CurrentSense.h"
#include "Encoder.h"
#include "FOCMotor.h"
#include "GenericCurrentSense.h"
#include "GenericSensor.h"
#include "HallSensor.h"
#include "InlineCurrentSense.h"
#include "LowsideCurrentSense.h"
#include "MagneticSensorAnalog.h"
#include "MagneticSensorI2C.h"
#include "MagneticSensorPWM.h"
#include "MagneticSensorSPI.h"
#include "Sensor.h"
#include "SimpleFOCDebug.h"
#include "StepDirListener.h"
#include "StepperDriver2PWM.h"
#include "StepperDriver4PWM.h"
#include "commands.h"
#include "foc_utils.h"
#include "hardware_api.h"
#include "time_utils.h"
#include <SPI.h>
#include <Wire.h>
#include <inttypes.h>
#ifndef SIMPLEFOC_H
#define SIMPLEFOC_H
#ifndef BLDCMotor_h
#define BLDCMotor_h
#ifndef FOCMOTOR_H
#define FOCMOTOR_H
#define _MON_TARGET 0b1000000 
#define _MON_VOLT_Q 0b0100000 
#define _MON_VOLT_D 0b0010000 
#define _MON_CURR_Q 0b0001000 
#define _MON_CURR_D 0b0000100 
#define _MON_VEL    0b0000010 
#define _MON_ANGLE  0b0000001 
enum MotionControlType : uint8_t {
torque            = 0x00,     
velocity          = 0x01,     
angle             = 0x02,     
velocity_openloop = 0x03,
angle_openloop    = 0x04
};
enum TorqueControlType : uint8_t {
voltage            = 0x00,     
dc_current         = 0x01,     
foc_current        = 0x02,     
};
enum FOCModulationType : uint8_t {
SinePWM            = 0x00,     
SpaceVectorPWM     = 0x01,     
Trapezoid_120      = 0x02,
Trapezoid_150      = 0x03,
};
enum FOCMotorStatus : uint8_t {
motor_uninitialized = 0x00,     
motor_initializing  = 0x01,     
motor_uncalibrated  = 0x02,     
motor_calibrating   = 0x03,     
motor_ready         = 0x04,     
motor_error         = 0x08,     
motor_calib_failed  = 0x0E,     
motor_init_failed   = 0x0F,     
};
class FOCMotor
{
public:
FOCMotor();
virtual void init()=0;
virtual void disable()=0;
virtual void enable()=0;
void linkSensor(Sensor* sensor);
void linkCurrentSense(CurrentSense* current_sense);
virtual int initFOC()=0;
virtual void loopFOC()=0;
virtual void move(float target = NOT_SET)=0;
virtual void setPhaseVoltage(float Uq, float Ud, float angle_el)=0;
float shaftAngle();
float shaftVelocity();
float electricalAngle();
float target; 
float feed_forward_velocity = 0.0f; 
float shaft_angle;
float electrical_angle;
float shaft_velocity;
float current_sp;
float shaft_velocity_sp;
float shaft_angle_sp;
DQVoltage_s voltage;
DQCurrent_s current;
float voltage_bemf; 
float	Ualpha, Ubeta; 
float voltage_sensor_align;
float velocity_index_search;
float	phase_resistance; 
int pole_pairs;
float KV_rating; 
float	phase_inductance; 
float voltage_limit; 
float current_limit; 
float velocity_limit; 
int8_t enabled = 0;
FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; 
FOCModulationType foc_modulation;
int8_t modulation_centered = 1;
TorqueControlType torque_controller; 
MotionControlType controller; 
PIDController PID_current_q{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
PIDController PID_current_d{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};
PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT};
PIDController P_angle{DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM};	
LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};
LowPassFilter LPF_angle{0.0};
unsigned int motion_downsample = DEF_MOTION_DOWNSMAPLE; 
unsigned int motion_cnt = 0; 
float sensor_offset; 
float zero_electric_angle = NOT_SET;
Direction sensor_direction = Direction::UNKNOWN; 
bool pp_check_result = false; 
void useMonitoring(Print &serial);
void monitor();
unsigned int monitor_downsample = DEF_MON_DOWNSMAPLE; 
char monitor_start_char = '\0'; 
char monitor_end_char = '\0'; 
char monitor_separator = '\t'; 
unsigned int  monitor_decimals = 4; 
uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; 
Sensor* sensor;
CurrentSense* current_sense;
Print* monitor_port; 
private:
unsigned int monitor_cnt = 0 ; 
};
#endif
FOCMotor::FOCMotor()
{
velocity_limit = DEF_VEL_LIM;
voltage_limit = DEF_POWER_SUPPLY;
current_limit = DEF_CURRENT_LIM;
velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;
foc_modulation = FOCModulationType::SinePWM;
target = 0;
voltage.d = 0;
voltage.q = 0;
current_sp = 0;
current.q = 0;
current.d = 0;
voltage_bemf = 0;
Ualpha = 0;
Ubeta = 0;
monitor_port = nullptr;
sensor_offset = 0.0f;
sensor = nullptr;
current_sense = nullptr;
}
void FOCMotor::linkSensor(Sensor* _sensor) {
sensor = _sensor;
}
void FOCMotor::linkCurrentSense(CurrentSense* _current_sense) {
current_sense = _current_sense;
}
float FOCMotor::shaftAngle() {
if(!sensor) return shaft_angle;
return sensor_direction*LPF_angle(sensor->getAngle()) - sensor_offset;
}
float FOCMotor::shaftVelocity() {
if(!sensor) return shaft_velocity;
return sensor_direction*LPF_velocity(sensor->getVelocity());
}
float FOCMotor::electricalAngle(){
if(!sensor) return electrical_angle;
return  _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
}
void FOCMotor::useMonitoring(Print &print){
monitor_port = &print; 
#ifndef SIMPLEFOC_DISABLE_DEBUG
SimpleFOCDebug::enable(&print);
SIMPLEFOC_DEBUG("MOT: Monitor enabled!");
#endif
}
void FOCMotor::monitor() {
if( !monitor_downsample || monitor_cnt++ < (monitor_downsample-1) ) return;
monitor_cnt = 0;
if(!monitor_port) return;
bool printed = 0;
if(monitor_variables & _MON_TARGET){
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
monitor_port->print(target,monitor_decimals);
printed= true;
}
if(monitor_variables & _MON_VOLT_Q) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(voltage.q,monitor_decimals);
printed= true;
}
if(monitor_variables & _MON_VOLT_D) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(voltage.d,monitor_decimals);
printed= true;
}
if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
DQCurrent_s c = current;
if( current_sense && torque_controller != TorqueControlType::foc_current ){
c = current_sense->getFOCCurrents(electrical_angle);
c.q = LPF_current_q(c.q);
c.d = LPF_current_d(c.d);
}
if(monitor_variables & _MON_CURR_Q) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(c.q*1000, monitor_decimals); 
printed= true;
}
if(monitor_variables & _MON_CURR_D) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(c.d*1000, monitor_decimals); 
printed= true;
}
}
if(monitor_variables & _MON_VEL) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(shaft_velocity,monitor_decimals);
printed= true;
}
if(monitor_variables & _MON_ANGLE) {
if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
else if(printed) monitor_port->print(monitor_separator);
monitor_port->print(shaft_angle,monitor_decimals);
printed= true;
}
if(printed){
if(monitor_end_char) monitor_port->println(monitor_end_char);
else monitor_port->println("");
}
}
#ifndef SENSOR_H
#define SENSOR_H
enum Direction : int8_t {
CW      = 1,  
CCW     = -1, 
UNKNOWN = 0   
};
enum Pullup : uint8_t {
USE_INTERN = 0x00, 
USE_EXTERN = 0x01  
};
class Sensor{
friend class SmoothingSensor;
public:
virtual float getMechanicalAngle();
virtual float getAngle();
virtual double getPreciseAngle();
virtual float getVelocity();
virtual int32_t getFullRotations();
virtual void update();
virtual int needsSearch();
float min_elapsed_time = 0.000100; 
protected:
virtual float getSensorAngle()=0;
virtual void init();
float velocity=0.0f;
float angle_prev=0.0f; 
long angle_prev_ts=0; 
float vel_angle_prev=0.0f; 
long vel_angle_prev_ts=0; 
int32_t full_rotations=0; 
int32_t vel_full_rotations=0; 
};
#endif
void Sensor::update() {
float val = getSensorAngle();
if (val<0) 
return; 
angle_prev_ts = _micros();
float d_angle = val - angle_prev;
if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
angle_prev = val;
}
float Sensor::getVelocity() {
float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6f;
if (Ts < 0.0f) {    
vel_angle_prev = angle_prev;
vel_full_rotations = full_rotations;
vel_angle_prev_ts = angle_prev_ts;
return velocity;
}
if (Ts < min_elapsed_time) return velocity; 
velocity = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
vel_angle_prev = angle_prev;
vel_full_rotations = full_rotations;
vel_angle_prev_ts = angle_prev_ts;
return velocity;
}
void Sensor::init() {
getSensorAngle(); 
delayMicroseconds(1);
vel_angle_prev = getSensorAngle(); 
vel_angle_prev_ts = _micros();
delay(1);
getSensorAngle(); 
delayMicroseconds(1);
angle_prev = getSensorAngle(); 
angle_prev_ts = _micros();
}
float Sensor::getMechanicalAngle() {
return angle_prev;
}
float Sensor::getAngle(){
return (float)full_rotations * _2PI + angle_prev;
}
double Sensor::getPreciseAngle() {
return (double)full_rotations * (double)_2PI + (double)angle_prev;
}
int32_t Sensor::getFullRotations() {
return full_rotations;
}
int Sensor::needsSearch() {
return 0; 
}
#ifndef BLDCDRIVER_H
#define BLDCDRIVER_H
enum PhaseState : uint8_t {
PHASE_OFF = 0, 
PHASE_ON = 1,  
PHASE_HI = 2,  
PHASE_LO = 3,  
};
class BLDCDriver{
public:
virtual int init() = 0;
virtual void enable() = 0;
virtual void disable() = 0;
long pwm_frequency; 
float voltage_power_supply; 
float voltage_limit; 
float dc_a; 
float dc_b; 
float dc_c; 
bool initialized = false; 
void* params = 0; 
virtual void setPwm(float Ua, float Ub, float Uc) = 0;
virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;
};
#endif
#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f
#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC (NOT_SET)
#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)
struct DQCurrent_s
{
float d;
float q;
};
struct PhaseCurrent_s
{
float a;
float b;
float c;
};
struct DQVoltage_s
{
float d;
float q;
};
struct ABCurrent_s
{
float alpha;
float beta;
};
float _sin(float a);
float _cos(float a);
void _sincos(float a, float* s, float* c);
float _atan2(float y, float x);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _sqrtApprox(float value);
#endif
__attribute__((weak)) float _sin(float a){
static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
unsigned int i = (unsigned int)(a * (64*4*256.0f/_2PI));
int t1, t2, frac = i & 0xff;
i = (i >> 8) & 0xff;
if (i < 64) {
t1 = sine_array[i]; t2 = sine_array[i+1];
}
else if(i < 128) {
t1 = sine_array[128 - i]; t2 = sine_array[127 - i];
}
else if(i < 192) {
t1 = -sine_array[-128 + i]; t2 = -sine_array[-127 + i];
}
else {
t1 = -sine_array[256 - i]; t2 = -sine_array[255 - i];
}
return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}
__attribute__((weak)) float _cos(float a){
float a_sin = a + _PI_2;
a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
return _sin(a_sin);
}
__attribute__((weak)) void _sincos(float a, float* s, float* c){
*s = _sin(a);
*c = _cos(a);
}
__attribute__((weak)) float _atan2(float y, float x) {
float abs_y = fabsf(y);
float abs_x = fabsf(x);
float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
float s = a * a;
float r =
((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
if (abs_y > abs_x) r = 1.57079637f - r;
if (x < 0.0f) r = 3.14159274f - r;
if (y < 0.0f) r = -r;
return r;
}
__attribute__((weak)) float _normalizeAngle(float angle){
float a = fmod(angle, _2PI);
return a >= 0 ? a : (a + _2PI);
}
float _electricalAngle(float shaft_angle, int pole_pairs) {
return (shaft_angle * pole_pairs);
}
__attribute__((weak)) float _sqrtApprox(float number) {
union {
float    f;
uint32_t i;
} y = { .f = number };
y.i = 0x5f375a86 - ( y.i >> 1 );
return number * y.f;
}
#ifndef TIME_UTILS_H
#define TIME_UTILS_H
void _delay(unsigned long ms);
unsigned long _micros();
#endif
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
unsigned long t = _micros() + ms*1000;
while( _micros() < t ){};
#else
delay(ms);
#endif
}
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
else return (micros());
#else
return micros();
#endif
}
#define DEF_POWER_SUPPLY 12.0f 
#define DEF_PID_VEL_P 0.5f 
#define DEF_PID_VEL_I 10.0f 
#define DEF_PID_VEL_D 0.0f 
#define DEF_PID_VEL_RAMP 1000.0f 
#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY) 
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__)
#define DEF_PID_CURR_P 2 
#define DEF_PID_CURR_I 100 
#define DEF_PID_CURR_D 0.0f 
#define DEF_PID_CURR_RAMP 1000.0f 
#define DEF_PID_CURR_LIMIT (DEF_POWER_SUPPLY) 
#define DEF_CURR_FILTER_Tf 0.01f 
#else
#define DEF_PID_CURR_P 3 
#define DEF_PID_CURR_I 300.0f 
#define DEF_PID_CURR_D 0.0f 
#define DEF_PID_CURR_RAMP 0  
#define DEF_PID_CURR_LIMIT (DEF_POWER_SUPPLY) 
#define DEF_CURR_FILTER_Tf 0.005f 
#endif
#define DEF_CURRENT_LIM 2.0f 
#define DEF_MON_DOWNSMAPLE 100 
#define DEF_MOTION_DOWNSMAPLE 0 
#define DEF_P_ANGLE_P 20.0f 
#define DEF_VEL_LIM 20.0f 
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1.0f 
#define DEF_VOLTAGE_SENSOR_ALIGN 3.0f 
#define DEF_VEL_FILTER_Tf 0.005f 
#define DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf 0.0f  
class BLDCMotor: public FOCMotor
{
public:
BLDCMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
virtual void linkDriver(BLDCDriver* driver);
BLDCDriver* driver;
void init() override;
void disable() override;
void enable() override;
int initFOC() override;
void loopFOC() override;
void move(float target = NOT_SET) override;
float Ua, Ub, Uc;
void setPhaseVoltage(float Uq, float Ud, float angle_el) override;
private:
int alignSensor();
int alignCurrentSense();
int absoluteZeroSearch();
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
long open_loop_timestamp;
};
#endif
#ifndef __SIMPLEFOCDEBUG_H__
#define __SIMPLEFOCDEBUG_H__
#ifndef SIMPLEFOC_DISABLE_DEBUG
class SimpleFOCDebug {
public:
static void enable(Print* debugPrint = &Serial);
static void println(const __FlashStringHelper* msg);
static void println(const char* msg);
static void println(const __FlashStringHelper* msg, float val);
static void println(const char* msg, float val);
static void println(const __FlashStringHelper* msg, int val);
static void println(const char* msg, int val);
static void println(const char* msg, char val);
static void println();
static void println(int val);
static void println(float val);
static void print(const char* msg);
static void print(const __FlashStringHelper* msg);
static void print(int val);
static void print(float val);
protected:
static Print* _debugPrint;
};
#define SIMPLEFOC_DEBUG(msg, ...) \
SimpleFOCDebug::println(F(msg), ##__VA_ARGS__)
#else 
#define SIMPLEFOC_DEBUG(msg, ...)
#endif 
#endif
#ifndef SIMPLEFOC_DISABLE_DEBUG
Print* SimpleFOCDebug::_debugPrint = NULL;
void SimpleFOCDebug::enable(Print* debugPrint) {
_debugPrint = debugPrint;
}
void SimpleFOCDebug::println(int val) {
if (_debugPrint != NULL) {
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(float val) {
if (_debugPrint != NULL) {
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(const char* str) {
if (_debugPrint != NULL) {
_debugPrint->println(str);
}
}
void SimpleFOCDebug::println(const __FlashStringHelper* str) {
if (_debugPrint != NULL) {
_debugPrint->println(str);
}
}
void SimpleFOCDebug::println(const char* str, float val) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(const __FlashStringHelper* str, float val) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(const char* str, int val) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(const char* str, char val) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
_debugPrint->println(val);
}
}
void SimpleFOCDebug::println(const __FlashStringHelper* str, int val) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
_debugPrint->println(val);
}
}
void SimpleFOCDebug::print(const char* str) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
}
}
void SimpleFOCDebug::print(const __FlashStringHelper* str) {
if (_debugPrint != NULL) {
_debugPrint->print(str);
}
}
void SimpleFOCDebug::print(int val) {
if (_debugPrint != NULL) {
_debugPrint->print(val);
}
}
void SimpleFOCDebug::print(float val) {
if (_debugPrint != NULL) {
_debugPrint->print(val);
}
}
void SimpleFOCDebug::println() {
if (_debugPrint != NULL) {
_debugPrint->println();
}
}
#endif
int trap_120_map[6][3] = {
{_HIGH_IMPEDANCE,1,-1},
{-1,1,_HIGH_IMPEDANCE},
{-1,_HIGH_IMPEDANCE,1},
{_HIGH_IMPEDANCE,-1,1},
{1,-1,_HIGH_IMPEDANCE},
{1,_HIGH_IMPEDANCE,-1}
};
int trap_150_map[12][3] = {
{_HIGH_IMPEDANCE,1,-1},
{-1,1,-1},
{-1,1,_HIGH_IMPEDANCE},
{-1,1,1},
{-1,_HIGH_IMPEDANCE,1},
{-1,-1,1},
{_HIGH_IMPEDANCE,-1,1},
{1,-1,1},
{1,-1,_HIGH_IMPEDANCE},
{1,-1,-1},
{1,_HIGH_IMPEDANCE,-1},
{1,1,-1}
};
BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _inductance)
: FOCMotor()
{
pole_pairs = pp;
phase_resistance = _R;
KV_rating = NOT_SET;
if (_isset(_KV))
KV_rating = _KV;
phase_inductance = _inductance;
torque_controller = TorqueControlType::voltage;
}
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
driver = _driver;
}
void BLDCMotor::init() {
if (!driver || !driver->initialized) {
motor_status = FOCMotorStatus::motor_init_failed;
SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
return;
}
motor_status = FOCMotorStatus::motor_initializing;
SIMPLEFOC_DEBUG("MOT: Init");
if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
if(current_sense){
PID_current_q.limit = voltage_limit;
PID_current_d.limit = voltage_limit;
}
if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
PID_velocity.limit = current_limit;
}else{
PID_velocity.limit = voltage_limit;
}
P_angle.limit = velocity_limit;
if ((controller==MotionControlType::angle_openloop
||controller==MotionControlType::velocity_openloop)
&& (sensor_direction == Direction::UNKNOWN)) {
sensor_direction = Direction::CW;
}
_delay(500);
SIMPLEFOC_DEBUG("MOT: Enable driver.");
enable();
_delay(500);
motor_status = FOCMotorStatus::motor_uncalibrated;
}
void BLDCMotor::disable()
{
if(current_sense) current_sense->disable();
driver->setPwm(0, 0, 0);
driver->disable();
enabled = 0;
}
void BLDCMotor::enable()
{
driver->enable();
driver->setPwm(0, 0, 0);
if(current_sense) current_sense->enable();
PID_velocity.reset();
P_angle.reset();
PID_current_q.reset();
PID_current_d.reset();
enabled = 1;
}
int  BLDCMotor::initFOC() {
int exit_flag = 1;
motor_status = FOCMotorStatus::motor_calibrating;
if(sensor){
exit_flag *= alignSensor();
sensor->update();
shaft_angle = shaftAngle();
if(exit_flag){
if(current_sense){
if (!current_sense->initialized) {
motor_status = FOCMotorStatus::motor_calib_failed;
SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
exit_flag = 0;
}else{
exit_flag *= alignCurrentSense();
}
}
else { SIMPLEFOC_DEBUG("MOT: No current sense."); }
}
} else {
SIMPLEFOC_DEBUG("MOT: No sensor.");
if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)){
exit_flag = 1;
SIMPLEFOC_DEBUG("MOT: Openloop only!");
}else{
exit_flag = 0; 
}
}
if(exit_flag){
SIMPLEFOC_DEBUG("MOT: Ready.");
motor_status = FOCMotorStatus::motor_ready;
}else{
SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
motor_status = FOCMotorStatus::motor_calib_failed;
disable();
}
return exit_flag;
}
int BLDCMotor::alignCurrentSense() {
int exit_flag = 1; 
SIMPLEFOC_DEBUG("MOT: Align current sense.");
exit_flag = current_sense->driverAlign(voltage_sensor_align);
if(!exit_flag){
SIMPLEFOC_DEBUG("MOT: Align error!");
exit_flag = 0;
}else{
SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
}
return exit_flag > 0;
}
int BLDCMotor::alignSensor() {
int exit_flag = 1; 
SIMPLEFOC_DEBUG("MOT: Align sensor.");
if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
if(!exit_flag) return exit_flag;
float voltage_align = voltage_sensor_align;
if(sensor_direction==Direction::UNKNOWN){
for (int i = 0; i <=500; i++ ) {
float angle = _3PI_2 + _2PI * i / 500.0f;
setPhaseVoltage(voltage_align, 0,  angle);
sensor->update();
_delay(2);
}
sensor->update();
float mid_angle = sensor->getAngle();
for (int i = 500; i >=0; i-- ) {
float angle = _3PI_2 + _2PI * i / 500.0f ;
setPhaseVoltage(voltage_align, 0,  angle);
sensor->update();
_delay(2);
}
sensor->update();
float end_angle = sensor->getAngle();
_delay(200);
float moved =  fabs(mid_angle - end_angle);
if (moved<MIN_ANGLE_DETECT_MOVEMENT) { 
SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
return 0; 
} else if (mid_angle < end_angle) {
SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
sensor_direction = Direction::CCW;
} else{
SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
sensor_direction = Direction::CW;
}
pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f); 
if( pp_check_result==false ) {
SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI/moved);
} else {
SIMPLEFOC_DEBUG("MOT: PP check: OK!");
}
} else { SIMPLEFOC_DEBUG("MOT: Skip dir calib."); }
if(!_isset(zero_electric_angle)){
setPhaseVoltage(voltage_align, 0,  _3PI_2);
_delay(700);
sensor->update();
zero_electric_angle = 0;
zero_electric_angle = electricalAngle();
_delay(20);
if(monitor_port){
SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
}
setPhaseVoltage(0, 0, 0);
_delay(200);
} else { SIMPLEFOC_DEBUG("MOT: Skip offset calib."); }
return exit_flag;
}
int BLDCMotor::absoluteZeroSearch() {
SIMPLEFOC_DEBUG("MOT: Index search...");
float limit_vel = velocity_limit;
float limit_volt = voltage_limit;
velocity_limit = velocity_index_search;
voltage_limit = voltage_sensor_align;
shaft_angle = 0;
while(sensor->needsSearch() && shaft_angle < _2PI){
angleOpenloop(1.5f*_2PI);
sensor->update();
}
setPhaseVoltage(0, 0, 0);
velocity_limit = limit_vel;
voltage_limit = limit_volt;
if(monitor_port){
if(sensor->needsSearch()) { SIMPLEFOC_DEBUG("MOT: Error: Not found!"); }
else { SIMPLEFOC_DEBUG("MOT: Success!"); }
}
return !sensor->needsSearch();
}
void BLDCMotor::loopFOC() {
if (sensor) sensor->update();
if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
if(!enabled) return;
electrical_angle = electricalAngle();
switch (torque_controller) {
case TorqueControlType::voltage:
break;
case TorqueControlType::dc_current:
if(!current_sense) return;
current.q = current_sense->getDCCurrent(electrical_angle);
current.q = LPF_current_q(current.q);
voltage.q = PID_current_q(current_sp - current.q);
if(_isset(phase_inductance)) voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
else voltage.d = 0;
break;
case TorqueControlType::foc_current:
if(!current_sense) return;
current = current_sense->getFOCCurrents(electrical_angle);
current.q = LPF_current_q(current.q);
current.d = LPF_current_d(current.d);
voltage.q = PID_current_q(current_sp - current.q);
voltage.d = PID_current_d(-current.d);
break;
default:
SIMPLEFOC_DEBUG("MOT: no torque control selected!");
break;
}
setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
void BLDCMotor::move(float new_target) {
if(motion_cnt++ < motion_downsample) return;
motion_cnt = 0;
if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop )
shaft_angle = shaftAngle(); 
shaft_velocity = shaftVelocity(); 
if(!enabled) return;
if(_isset(new_target)) target = new_target;
if (_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;
switch (controller) {
case MotionControlType::torque:
if(torque_controller == TorqueControlType::voltage){ 
if(!_isset(phase_resistance))  voltage.q = target;
else  voltage.q =  target*phase_resistance + voltage_bemf;
voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
}else{
current_sp = target; 
}
break;
case MotionControlType::angle:
shaft_angle_sp = target;
shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
shaft_velocity_sp = _constrain(shaft_velocity_sp,-velocity_limit, velocity_limit);
current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
if(torque_controller == TorqueControlType::voltage){
if(!_isset(phase_resistance))  voltage.q = current_sp;
else  voltage.q =  _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
}
break;
case MotionControlType::velocity:
shaft_velocity_sp = target;
current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
if(torque_controller == TorqueControlType::voltage){
if(!_isset(phase_resistance))  voltage.q = current_sp;
else  voltage.q = _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
}
break;
case MotionControlType::velocity_openloop:
shaft_velocity_sp = target;
voltage.q = velocityOpenloop(shaft_velocity_sp); 
voltage.d = 0;
break;
case MotionControlType::angle_openloop:
shaft_angle_sp = target;
voltage.q = angleOpenloop(shaft_angle_sp); 
voltage.d = 0;
break;
}
}
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
float center;
int sector;
float _ca,_sa;
switch (foc_modulation)
{
case FOCModulationType::Trapezoid_120 :
sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); 
center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
Ua= center;
Ub = trap_120_map[sector][1] * Uq + center;
Uc = trap_120_map[sector][2] * Uq + center;
driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); 
}else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
Ua = trap_120_map[sector][0] * Uq + center;
Ub = center;
Uc = trap_120_map[sector][2] * Uq + center;
driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);
}else{
Ua = trap_120_map[sector][0] * Uq + center;
Ub = trap_120_map[sector][1] * Uq + center;
Uc = center;
driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);
}
break;
case FOCModulationType::Trapezoid_150 :
sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); 
center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
Ua= center;
Ub = trap_150_map[sector][1] * Uq + center;
Uc = trap_150_map[sector][2] * Uq + center;
driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); 
}else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
Ua = trap_150_map[sector][0] * Uq + center;
Ub = center;
Uc = trap_150_map[sector][2] * Uq + center;
driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); 
}else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
Ua = trap_150_map[sector][0] * Uq + center;
Ub = trap_150_map[sector][1] * Uq + center;
Uc = center;
driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); 
}else{
Ua = trap_150_map[sector][0] * Uq + center;
Ub = trap_150_map[sector][1] * Uq + center;
Uc = trap_150_map[sector][2] * Uq + center;
driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); 
}
break;
case FOCModulationType::SinePWM :
case FOCModulationType::SpaceVectorPWM :
_sincos(angle_el, &_sa, &_ca);
Ualpha =  _ca * Ud - _sa * Uq;  
Ubeta =  _sa * Ud + _ca * Uq;    
Ua = Ualpha;
Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;
center = driver->voltage_limit/2;
if (foc_modulation == FOCModulationType::SpaceVectorPWM){
float Umin = min(Ua, min(Ub, Uc));
float Umax = max(Ua, max(Ub, Uc));
center -= (Umax+Umin) / 2;
}
if (!modulation_centered) {
float Umin = min(Ua, min(Ub, Uc));
Ua -= Umin;
Ub -= Umin;
Uc -= Umin;
}else{
Ua += center;
Ub += center;
Uc += center;
}
break;
}
driver->setPwm(Ua, Ub, Uc);
}
float BLDCMotor::velocityOpenloop(float target_velocity){
unsigned long now_us = _micros();
float Ts = (now_us - open_loop_timestamp) * 1e-6f;
if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
shaft_velocity = target_velocity;
float Uq = voltage_limit;
if(_isset(phase_resistance)){
Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
}
setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
open_loop_timestamp = now_us;
return Uq;
}
float BLDCMotor::angleOpenloop(float target_angle){
unsigned long now_us = _micros();
float Ts = (now_us - open_loop_timestamp) * 1e-6f;
if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
shaft_velocity = velocity_limit;
}else{
shaft_angle = target_angle;
shaft_velocity = 0;
}
float Uq = voltage_limit;
if(_isset(phase_resistance)){
Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
}
setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));
open_loop_timestamp = now_us;
return Uq;
}
#ifndef StepperMotor_h
#define StepperMotor_h
#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H
#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H
#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif
#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true
#endif
#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true
#endif
#define SIMPLEFOC_DRIVER_INIT_FAILED ((void*)-1)
typedef struct GenericDriverParams {
int pins[6];
long pwm_frequency;
float dead_zone;
} GenericDriverParams;
void* _configure1PWM(long pwm_frequency, const int pinA);
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB);
void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);
void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B);
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l);
void _writeDutyCycle1PWM(float dc_a, void* params);
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params);
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params);
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params);
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params);
#endif
class StepperDriver{
public:
virtual int init() = 0;
virtual void enable() = 0;
virtual void disable() = 0;
long pwm_frequency; 
float voltage_power_supply; 
float voltage_limit; 
bool initialized = false; 
void* params = 0; 
virtual void setPwm(float Ua, float Ub) = 0;
};
#endif
class StepperMotor: public FOCMotor
{
public:
StepperMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
void linkDriver(StepperDriver* driver);
StepperDriver* driver;
void init() override;
void disable() override;
void enable() override;
int initFOC() override;
void loopFOC() override;
void move(float target = NOT_SET) override;
void setPhaseVoltage(float Uq, float Ud, float angle_el) override;
private:
int alignSensor();
int absoluteZeroSearch();
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
long open_loop_timestamp;
};
#endif
StepperMotor::StepperMotor(int pp, float _R, float _KV, float _inductance)
: FOCMotor()
{
pole_pairs = pp;
phase_resistance = _R;
KV_rating = _KV;
phase_inductance = _inductance;
torque_controller = TorqueControlType::voltage;
}
void StepperMotor::linkDriver(StepperDriver* _driver) {
driver = _driver;
}
void StepperMotor::init() {
if (!driver || !driver->initialized) {
motor_status = FOCMotorStatus::motor_init_failed;
SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
return;
}
motor_status = FOCMotorStatus::motor_initializing;
SIMPLEFOC_DEBUG("MOT: Init");
if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
if(_isset(phase_resistance)){
PID_velocity.limit = current_limit;
}else{
PID_velocity.limit = voltage_limit;
}
P_angle.limit = velocity_limit;
if ((controller==MotionControlType::angle_openloop
||controller==MotionControlType::velocity_openloop)
&& (sensor_direction == Direction::UNKNOWN)) {
sensor_direction = Direction::CW;
}
_delay(500);
SIMPLEFOC_DEBUG("MOT: Enable driver.");
enable();
_delay(500);
motor_status = FOCMotorStatus::motor_uncalibrated;
}
void StepperMotor::disable()
{
driver->setPwm(0, 0);
driver->disable();
enabled = 0;
}
void StepperMotor::enable()
{
driver->enable();
driver->setPwm(0, 0);
enabled = 1;
}
int  StepperMotor::initFOC() {
int exit_flag = 1;
motor_status = FOCMotorStatus::motor_calibrating;
_delay(500);
if(sensor){
exit_flag *= alignSensor();
sensor->update();
shaft_angle = sensor->getAngle();
} else {
SIMPLEFOC_DEBUG("MOT: No sensor.");
if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)){
exit_flag = 1;
SIMPLEFOC_DEBUG("MOT: Openloop only!");
}else{
exit_flag = 0; 
}
}
if(exit_flag){
SIMPLEFOC_DEBUG("MOT: Ready.");
motor_status = FOCMotorStatus::motor_ready;
}else{
SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
motor_status = FOCMotorStatus::motor_calib_failed;
disable();
}
return exit_flag;
}
int StepperMotor::alignSensor() {
int exit_flag = 1; 
SIMPLEFOC_DEBUG("MOT: Align sensor.");
float voltage_align = voltage_sensor_align;
if(sensor_direction == Direction::UNKNOWN){
if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
if(!exit_flag) return exit_flag;
for (int i = 0; i <=500; i++ ) {
float angle = _3PI_2 + _2PI * i / 500.0f;
setPhaseVoltage(voltage_align, 0,  angle);
sensor->update();
_delay(2);
}
sensor->update();
float mid_angle = sensor->getAngle();
for (int i = 500; i >=0; i-- ) {
float angle = _3PI_2 + _2PI * i / 500.0f ;
setPhaseVoltage(voltage_align, 0,  angle);
sensor->update();
_delay(2);
}
sensor->update();
float end_angle = sensor->getAngle();
setPhaseVoltage(0, 0, 0);
_delay(200);
if (mid_angle == end_angle) {
SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
return 0; 
} else if (mid_angle < end_angle) {
SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
sensor_direction = Direction::CCW;
} else{
SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
sensor_direction = Direction::CW;
}
float moved =  fabs(mid_angle - end_angle);
pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f);  
if( pp_check_result==false ) {
SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI/moved);
} else {
SIMPLEFOC_DEBUG("MOT: PP check: OK!");
}
} else {
SIMPLEFOC_DEBUG("MOT: Skip dir calib.");
}
if(!_isset(zero_electric_angle)){
setPhaseVoltage(voltage_align, 0,  _3PI_2);
_delay(700);
sensor->update();
zero_electric_angle = 0;
zero_electric_angle = electricalAngle();
_delay(20);
if(monitor_port){
SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
}
setPhaseVoltage(0, 0, 0);
_delay(200);
} else { SIMPLEFOC_DEBUG("MOT: Skip offset calib."); }
return exit_flag;
}
int StepperMotor::absoluteZeroSearch() {
SIMPLEFOC_DEBUG("MOT: Index search...");
float limit_vel = velocity_limit;
float limit_volt = voltage_limit;
velocity_limit = velocity_index_search;
voltage_limit = voltage_sensor_align;
shaft_angle = 0;
while(sensor->needsSearch() && shaft_angle < _2PI){
angleOpenloop(1.5f*_2PI);
sensor->update();
}
setPhaseVoltage(0, 0, 0);
velocity_limit = limit_vel;
voltage_limit = limit_volt;
if(monitor_port){
if(sensor->needsSearch()) SIMPLEFOC_DEBUG("MOT: Error: Not found!");
else { SIMPLEFOC_DEBUG("MOT: Success!"); }
}
return !sensor->needsSearch();
}
void StepperMotor::loopFOC() {
if (sensor) sensor->update();
if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
shaft_angle = shaftAngle();
if(!enabled) return;
electrical_angle = electricalAngle();
setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
void StepperMotor::move(float new_target) {
if(motion_cnt++ < motion_downsample) return;
motion_cnt = 0;
if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop )
shaft_angle = shaftAngle(); 
shaft_velocity = shaftVelocity(); 
if(!enabled) return;
if(_isset(new_target) ) target = new_target;
if (_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;
switch (controller) {
case MotionControlType::torque:
if(!_isset(phase_resistance))  voltage.q = target; 
else  voltage.q =  target*phase_resistance + voltage_bemf;
voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
break;
case MotionControlType::angle:
shaft_angle_sp = target;
shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
shaft_velocity_sp = _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
if(!_isset(phase_resistance))  voltage.q = current_sp;
else  voltage.q =  _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
break;
case MotionControlType::velocity:
shaft_velocity_sp = target;
current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); 
if(!_isset(phase_resistance))  voltage.q = current_sp;
else  voltage.q = _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
if(!_isset(phase_inductance)) voltage.d = 0;
else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
break;
case MotionControlType::velocity_openloop:
shaft_velocity_sp = target;
voltage.q = velocityOpenloop(shaft_velocity_sp); 
voltage.d = 0; 
break;
case MotionControlType::angle_openloop:
shaft_angle_sp = target;
voltage.q = angleOpenloop(shaft_angle_sp); 
voltage.d = 0; 
break;
}
}
void StepperMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
float _sa, _ca;
_sincos(angle_el, &_sa, &_ca);
Ualpha =  _ca * Ud - _sa * Uq;  
Ubeta =  _sa * Ud + _ca * Uq;    
driver->setPwm(Ualpha, Ubeta);
}
float StepperMotor::velocityOpenloop(float target_velocity){
unsigned long now_us = _micros();
float Ts = (now_us - open_loop_timestamp) * 1e-6f;
if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
shaft_velocity = target_velocity;
float Uq = voltage_limit;
if(_isset(phase_resistance)){
Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
}
setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
open_loop_timestamp = now_us;
return Uq;
}
float StepperMotor::angleOpenloop(float target_angle){
unsigned long now_us = _micros();
float Ts = (now_us - open_loop_timestamp) * 1e-6f;
if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
shaft_velocity = velocity_limit;
}else{
shaft_angle = target_angle;
shaft_velocity = 0;
}
float Uq = voltage_limit;
if(_isset(phase_resistance)){
Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
}
setPhaseVoltage(Uq,  0, _electricalAngle((shaft_angle), pole_pairs));
open_loop_timestamp = now_us;
return Uq;
}
#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H
enum Quadrature : uint8_t {
ON    = 0x00, 
OFF   = 0x01  
};
class Encoder: public Sensor{
public:
Encoder(int encA, int encB , float ppr, int index = 0);
void init() override;
void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doIndex)() = nullptr);
void handleA();
void handleB();
void handleIndex();
int pinA; 
int pinB; 
int index_pin; 
Pullup pullup; 
Quadrature quadrature;
float cpr;
float getSensorAngle() override;
float getVelocity() override;
virtual void update() override;
int needsSearch() override;
private:
int hasIndex(); 
volatile long pulse_counter;
volatile long pulse_timestamp;
volatile int A_active; 
volatile int B_active; 
volatile int I_active; 
volatile bool index_found = false; 
float prev_Th, pulse_per_second;
volatile long prev_pulse_counter, prev_timestamp_us;
};
#endif
Encoder::Encoder(int _encA, int _encB , float _ppr, int _index){
pinA = _encA;
pinB = _encB;
pulse_counter = 0;
pulse_timestamp = 0;
cpr = _ppr;
A_active = 0;
B_active = 0;
I_active = 0;
index_pin = _index; 
prev_Th = 0;
pulse_per_second = 0;
prev_pulse_counter = 0;
prev_timestamp_us = _micros();
pullup = Pullup::USE_EXTERN;
quadrature = Quadrature::ON;
}
void Encoder::handleA() {
bool A = digitalRead(pinA);
switch (quadrature){
case Quadrature::ON:
if ( A != A_active ) {
pulse_counter += (A_active == B_active) ? 1 : -1;
pulse_timestamp = _micros();
A_active = A;
}
break;
case Quadrature::OFF:
if(A && !digitalRead(pinB)){
pulse_counter++;
pulse_timestamp = _micros();
}
break;
}
}
void Encoder::handleB() {
bool B = digitalRead(pinB);
switch (quadrature){
case Quadrature::ON:
if ( B != B_active ) {
pulse_counter += (A_active != B_active) ? 1 : -1;
pulse_timestamp = _micros();
B_active = B;
}
break;
case Quadrature::OFF:
if(B && !digitalRead(pinA)){
pulse_counter--;
pulse_timestamp = _micros();
}
break;
}
}
void Encoder::handleIndex() {
if(hasIndex()){
bool I = digitalRead(index_pin);
if(I && !I_active){
index_found = true;
long tmp = pulse_counter;
pulse_counter = round((double)pulse_counter/(double)cpr)*cpr;
prev_pulse_counter += pulse_counter - tmp;
}
I_active = I;
}
}
void Encoder::update() {
noInterrupts();
angle_prev_ts = pulse_timestamp;
long copy_pulse_counter = pulse_counter;
interrupts();
full_rotations = copy_pulse_counter / (int)cpr;
angle_prev = _2PI * ((copy_pulse_counter) % ((int)cpr)) / ((float)cpr);
}
float Encoder::getSensorAngle(){
return _2PI * (pulse_counter) / ((float)cpr);
}
float Encoder::getVelocity(){
noInterrupts();
long copy_pulse_counter = pulse_counter;
long copy_pulse_timestamp = pulse_timestamp;
interrupts();
long timestamp_us = _micros();
float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
float Th = (timestamp_us - copy_pulse_timestamp) * 1e-6f;
long dN = copy_pulse_counter - prev_pulse_counter;
float dt = Ts + prev_Th - Th;
pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;
if ( Th > 0.1f) pulse_per_second = 0;
float velocity = pulse_per_second / ((float)cpr) * (_2PI);
prev_timestamp_us = timestamp_us;
prev_Th = Th;
prev_pulse_counter = copy_pulse_counter;
return velocity;
}
int Encoder::needsSearch(){
return hasIndex() && !index_found;
}
int Encoder::hasIndex(){
return index_pin != 0;
}
void Encoder::init(){
if(pullup == Pullup::USE_INTERN){
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
if(hasIndex()) pinMode(index_pin,INPUT_PULLUP);
}else{
pinMode(pinA, INPUT);
pinMode(pinB, INPUT);
if(hasIndex()) pinMode(index_pin,INPUT);
}
pulse_counter = 0;
pulse_timestamp = _micros();
prev_Th = 0;
pulse_per_second = 0;
prev_pulse_counter = 0;
prev_timestamp_us = _micros();
if(quadrature == Quadrature::ON) cpr = 4*cpr;
}
void Encoder::enableInterrupts(void (*doA)(), void(*doB)(), void(*doIndex)()){
switch(quadrature){
case Quadrature::ON:
if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
break;
case Quadrature::OFF:
if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, RISING);
if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, RISING);
break;
}
if(hasIndex() && doIndex != nullptr) attachInterrupt(digitalPinToInterrupt(index_pin), doIndex, CHANGE);
}
#ifndef MAGNETICSENSORSPI_LIB_H
#define MAGNETICSENSORSPI_LIB_H
#define DEF_ANGLE_REGISTER 0x3FFF
struct MagneticSensorSPIConfig_s  {
int spi_mode;
long clock_speed;
int bit_resolution;
int angle_register;
int data_start_bit;
int command_rw_bit;
int command_parity_bit;
};
extern MagneticSensorSPIConfig_s AS5147_SPI,AS5048_SPI,AS5047_SPI, MA730_SPI;
class MagneticSensorSPI: public Sensor{
public:
MagneticSensorSPI(int cs, int bit_resolution, int angle_register = 0);
MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs);
void init(SPIClass* _spi = &SPI);
float getSensorAngle() override;
int spi_mode;
long clock_speed;
private:
float cpr; 
int angle_register; 
int chip_select_pin; 
SPISettings settings; 
void close();
word read(word angle_register);
byte spiCalcEvenParity(word value);
int getRawCount();
int bit_resolution; 
int command_parity_bit; 
int command_rw_bit; 
int data_start_bit; 
SPIClass* spi;
};
#endif
MagneticSensorSPIConfig_s AS5147_SPI = {
.spi_mode = SPI_MODE1,
.clock_speed = 1000000,
.bit_resolution = 14,
.angle_register = 0x3FFF,
.data_start_bit = 13,
.command_rw_bit = 14,
.command_parity_bit = 15
};
MagneticSensorSPIConfig_s AS5048_SPI = AS5147_SPI;
MagneticSensorSPIConfig_s AS5047_SPI = AS5147_SPI;
MagneticSensorSPIConfig_s MA730_SPI = {
.spi_mode = SPI_MODE0,
.clock_speed = 1000000,
.bit_resolution = 14,
.angle_register = 0x0000,
.data_start_bit = 15,
.command_rw_bit = 0,  
.command_parity_bit = 0 
};
MagneticSensorSPI::MagneticSensorSPI(int cs, int _bit_resolution, int _angle_register){
chip_select_pin = cs;
angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTER;
cpr = _powtwo(_bit_resolution);
spi_mode = SPI_MODE1;
clock_speed = 1000000;
bit_resolution = _bit_resolution;
command_parity_bit = 15; 
command_rw_bit = 14; 
data_start_bit = 13; 
}
MagneticSensorSPI::MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs){
chip_select_pin = cs;
angle_register = config.angle_register ? config.angle_register : DEF_ANGLE_REGISTER;
cpr = _powtwo(config.bit_resolution);
spi_mode = config.spi_mode;
clock_speed = config.clock_speed;
bit_resolution = config.bit_resolution;
command_parity_bit = config.command_parity_bit; 
command_rw_bit = config.command_rw_bit; 
data_start_bit = config.data_start_bit; 
}
void MagneticSensorSPI::init(SPIClass* _spi){
spi = _spi;
settings = SPISettings(clock_speed, MSBFIRST, spi_mode);
pinMode(chip_select_pin, OUTPUT);
spi->begin();
digitalWrite(chip_select_pin, HIGH);
this->Sensor::init(); 
}
float MagneticSensorSPI::getSensorAngle(){
return (getRawCount() / (float)cpr) * _2PI;
}
int MagneticSensorSPI::getRawCount(){
return (int)MagneticSensorSPI::read(angle_register);
}
byte MagneticSensorSPI::spiCalcEvenParity(word value){
byte cnt = 0;
byte i;
for (i = 0; i < 16; i++)
{
if (value & 0x1) cnt++;
value >>= 1;
}
return cnt & 0x1;
}
word MagneticSensorSPI::read(word angle_register){
word command = angle_register;
if (command_rw_bit > 0) {
command = angle_register | (1 << command_rw_bit);
}
if (command_parity_bit > 0) {
command |= ((word)spiCalcEvenParity(command) << command_parity_bit);
}
spi->beginTransaction(settings);
digitalWrite(chip_select_pin, LOW);
spi->transfer16(command);
digitalWrite(chip_select_pin,HIGH);
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) 
delayMicroseconds(50); 
#else
delayMicroseconds(1); 
#endif
digitalWrite(chip_select_pin, LOW);
word register_value = spi->transfer16(0x00);
digitalWrite(chip_select_pin, HIGH);
spi->endTransaction();
register_value = register_value >> (1 + data_start_bit - bit_resolution);  
const static word data_mask = 0xFFFF >> (16 - bit_resolution);
return register_value & data_mask;  
}
void MagneticSensorSPI::close(){
spi->end();
}
#ifndef MAGNETICSENSORI2C_LIB_H
#define MAGNETICSENSORI2C_LIB_H
struct MagneticSensorI2CConfig_s  {
int chip_address;
int bit_resolution;
int angle_register;
int data_start_bit;
};
extern MagneticSensorI2CConfig_s AS5600_I2C,AS5048_I2C;
#if defined(TARGET_RP2040)
#define SDA I2C_SDA
#define SCL I2C_SCL
#endif
class MagneticSensorI2C: public Sensor{
public:
MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);
MagneticSensorI2C(MagneticSensorI2CConfig_s config);
static MagneticSensorI2C AS5600();
void init(TwoWire* _wire = &Wire);
float getSensorAngle() override;
int checkBus(byte sda_pin , byte scl_pin );
uint8_t currWireError = 0;
private:
float cpr; 
uint16_t lsb_used; 
uint8_t lsb_mask;
uint8_t msb_mask;
uint8_t angle_register_msb; 
uint8_t chip_address; 
int read(uint8_t angle_register_msb);
int getRawCount();
TwoWire* wire;
};
#endif
MagneticSensorI2CConfig_s AS5600_I2C = {
.chip_address = 0x36,
.bit_resolution = 12,
.angle_register = 0x0C,
.data_start_bit = 11
};
MagneticSensorI2CConfig_s AS5048_I2C = {
.chip_address = 0x40,  
.bit_resolution = 14,
.angle_register = 0xFE,
.data_start_bit = 15
};
MagneticSensorI2C::MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _bits_used_msb){
chip_address = _chip_address;
angle_register_msb = _angle_register_msb;
cpr = _powtwo(_bit_resolution);
lsb_used = _bit_resolution - _bits_used_msb;
lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
wire = &Wire;
}
MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config){
chip_address = config.chip_address;
angle_register_msb = config.angle_register;
cpr = _powtwo(config.bit_resolution);
int bits_used_msb = config.data_start_bit - 7;
lsb_used = config.bit_resolution - bits_used_msb;
lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
msb_mask = (uint8_t)( (2 << bits_used_msb) - 1 );
wire = &Wire;
}
MagneticSensorI2C MagneticSensorI2C::AS5600() {
return {AS5600_I2C};
}
void MagneticSensorI2C::init(TwoWire* _wire){
wire = _wire;
wire->begin();
this->Sensor::init(); 
}
float MagneticSensorI2C::getSensorAngle(){
return  ( getRawCount() / (float)cpr) * _2PI ;
}
int MagneticSensorI2C::getRawCount(){
return (int)MagneticSensorI2C::read(angle_register_msb);
}
int MagneticSensorI2C::read(uint8_t angle_reg_msb) {
byte readArray[2];
uint16_t readValue = 0;
wire->beginTransmission(chip_address);
wire->write(angle_reg_msb);
currWireError = wire->endTransmission(false);
wire->requestFrom(chip_address, (uint8_t)2);
for (byte i=0; i < 2; i++) {
readArray[i] = wire->read();
}
readValue = ( readArray[1] &  lsb_mask );
readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
return readValue;
}
int MagneticSensorI2C::checkBus(byte sda_pin, byte scl_pin) {
pinMode(scl_pin, INPUT_PULLUP);
pinMode(sda_pin, INPUT_PULLUP);
delay(250);
if (digitalRead(scl_pin) == LOW) {
return 1;
}
if(digitalRead(sda_pin) == LOW) {
pinMode(scl_pin, OUTPUT);
for (byte i = 0; i < 16; i++) {
digitalWrite(scl_pin, LOW);
delayMicroseconds(20);
digitalWrite(scl_pin, HIGH);
delayMicroseconds(20);
}
pinMode(sda_pin, INPUT);
delayMicroseconds(20);
if (digitalRead(sda_pin) == LOW) {
return 2;
}
_delay(1000);
}
pinMode(sda_pin, INPUT);
pinMode(scl_pin, INPUT);
return 0;
}
#ifndef MAGNETICSENSORANALOG_LIB_H
#define MAGNETICSENSORANALOG_LIB_H
class MagneticSensorAnalog: public Sensor{
public:
MagneticSensorAnalog(uint8_t _pinAnalog, int _min = 0, int _max = 0);
void init();
int pinAnalog; 
Pullup pullup;
float getSensorAngle() override;
int raw_count;
private:
int min_raw_count;
int max_raw_count;
int cpr;
int read();
int getRawCount();
};
#endif
MagneticSensorAnalog::MagneticSensorAnalog(uint8_t _pinAnalog, int _min_raw_count, int _max_raw_count){
pinAnalog = _pinAnalog;
cpr = _max_raw_count - _min_raw_count;
min_raw_count = _min_raw_count;
max_raw_count = _max_raw_count;
if(pullup == Pullup::USE_INTERN){
pinMode(pinAnalog, INPUT_PULLUP);
}else{
pinMode(pinAnalog, INPUT);
}
}
void MagneticSensorAnalog::init(){
raw_count = getRawCount();
this->Sensor::init(); 
}
float MagneticSensorAnalog::getSensorAngle(){
raw_count = getRawCount();
return ( (float) (raw_count) / (float)cpr) * _2PI;
}
int MagneticSensorAnalog::getRawCount(){
return analogRead(pinAnalog);
}
#ifndef MAGNETICSENSORPWM_LIB_H
#define MAGNETICSENSORPWM_LIB_H
class MagneticSensorPWM: public Sensor{
public:
MagneticSensorPWM(uint8_t _pinPWM,int _min = 0, int _max = 0);
MagneticSensorPWM(uint8_t _pinPWM, int freqHz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks);
void init();
int pinPWM;
void update() override;
float getSensorAngle() override;
void handlePWM();
void enableInterrupt(void (*doPWM)());
unsigned long pulse_length_us;
unsigned int timeout_us = 1200;
private:
int raw_count;
int min_raw_count;
int max_raw_count;
int cpr;
bool is_interrupt_based;
int read();
int getRawCount();
unsigned long last_call_us;
unsigned long pulse_timestamp;
};
#endif
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pinPWM, int _min_raw_count, int _max_raw_count){
pinPWM = _pinPWM;
cpr = _max_raw_count - _min_raw_count + 1;
min_raw_count = _min_raw_count;
max_raw_count = _max_raw_count;
is_interrupt_based = false;
last_call_us = _micros();
}
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pinPWM, int freqHz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks){
pinPWM = _pinPWM;
min_raw_count = lroundf(1000000.0f/freqHz/_total_pwm_clocks*_min_pwm_clocks);
max_raw_count = lroundf(1000000.0f/freqHz/_total_pwm_clocks*_max_pwm_clocks);
cpr = max_raw_count - min_raw_count + 1;
is_interrupt_based = false;
min_elapsed_time = 1.0f/freqHz; 
last_call_us = _micros();
}
void MagneticSensorPWM::init(){
pinMode(pinPWM, INPUT);
raw_count = getRawCount();
pulse_timestamp = _micros();
this->Sensor::init(); 
}
void MagneticSensorPWM::update() {
if (is_interrupt_based)
noInterrupts();
Sensor::update();
angle_prev_ts = pulse_timestamp; 
if (is_interrupt_based)
interrupts();
}
float MagneticSensorPWM::getSensorAngle(){
raw_count = getRawCount();
if (raw_count > max_raw_count) raw_count = max_raw_count;
if (raw_count < min_raw_count) raw_count = min_raw_count;
return( (float) (raw_count - min_raw_count) / (float)cpr) * _2PI;
}
int MagneticSensorPWM::getRawCount(){
if (!is_interrupt_based){ 
pulse_timestamp = _micros(); 
pulse_length_us = pulseIn(pinPWM, HIGH, timeout_us); 
}
return pulse_length_us;
}
void MagneticSensorPWM::handlePWM() {
unsigned long now_us = _micros();
if (!digitalRead(pinPWM)) {
pulse_length_us = now_us - last_call_us;
pulse_timestamp = last_call_us; 
}
last_call_us = now_us;
is_interrupt_based = true; 
}
void MagneticSensorPWM::enableInterrupt(void (*doPWM)()){
is_interrupt_based  = true;
attachInterrupt(digitalPinToInterrupt(pinPWM), doPWM, CHANGE);
}
#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H
const int8_t ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };
class HallSensor: public Sensor{
public:
HallSensor(int encA, int encB, int encC, int pp);
void init();
void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doC)() = nullptr);
void handleA();
void handleB();
void handleC();
int pinA; 
int pinB; 
int pinC; 
Pullup pullup; 
int cpr;
void update() override;
float getSensorAngle() override;
float getVelocity() override;
Direction direction;
Direction old_direction;
void attachSectorCallback(void (*onSectorChange)(int a) = nullptr);
volatile int8_t hall_state;
volatile int8_t electric_sector;
volatile long electric_rotations;
volatile long total_interrupts;
float velocity_max = 1000.0f;
private:
Direction decodeDirection(int oldState, int newState);
void updateState();
volatile unsigned long pulse_timestamp;
volatile int A_active; 
volatile int B_active; 
volatile int C_active; 
void (*onSectorChange)(int sector) = nullptr;
volatile long pulse_diff;
};
#endif
HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp){
pinA = _hallA;
pinB = _hallB;
pinC = _hallC;
cpr = _pp * 6;
pullup = Pullup::USE_EXTERN;
}
void HallSensor::handleA() {
A_active= digitalRead(pinA);
updateState();
}
void HallSensor::handleB() {
B_active = digitalRead(pinB);
updateState();
}
void HallSensor::handleC() {
C_active = digitalRead(pinC);
updateState();
}
void HallSensor::updateState() {
long new_pulse_timestamp = _micros();
int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);
if (new_hall_state == hall_state) {
return;
}
hall_state = new_hall_state;
int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
if (new_electric_sector - electric_sector > 3) {
direction = Direction::CCW;
electric_rotations += direction;
} else if (new_electric_sector - electric_sector < (-3)) {
direction = Direction::CW;
electric_rotations += direction;
} else {
direction = (new_electric_sector > electric_sector)? Direction::CW : Direction::CCW;
}
electric_sector = new_electric_sector;
if (direction == old_direction) {
pulse_diff = new_pulse_timestamp - pulse_timestamp;
} else {
pulse_diff = 0;
}
pulse_timestamp = new_pulse_timestamp;
total_interrupts++;
old_direction = direction;
if (onSectorChange != nullptr) onSectorChange(electric_sector);
}
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
onSectorChange = _onSectorChange;
}
void HallSensor::update() {
noInterrupts();
angle_prev_ts = pulse_timestamp;
long last_electric_rotations = electric_rotations;
int8_t last_electric_sector = electric_sector;
interrupts();
angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI ;
full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
}
float HallSensor::getSensorAngle() {
return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI ;
}
float HallSensor::getVelocity(){
noInterrupts();
long last_pulse_timestamp = pulse_timestamp;
long last_pulse_diff = pulse_diff;
interrupts();
if (last_pulse_diff == 0 || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff*2) ) { 
return 0;
} else {
return direction * (_2PI / (float)cpr) / (last_pulse_diff / 1000000.0f);
}
}
void HallSensor::init(){
electric_rotations = 0;
if(pullup == Pullup::USE_INTERN){
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
pinMode(pinC, INPUT_PULLUP);
}else{
pinMode(pinA, INPUT);
pinMode(pinB, INPUT);
pinMode(pinC, INPUT);
}
A_active= digitalRead(pinA);
B_active = digitalRead(pinB);
C_active = digitalRead(pinC);
updateState();
pulse_timestamp = _micros();
}
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()){
if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
if(doC != nullptr) attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE);
}
#ifndef GENERIC_SENSOR_LIB_H
#define GENERIC_SENSOR_LIB_H
class GenericSensor: public Sensor{
public:
GenericSensor(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr);
float (*readCallback)() = nullptr; 
void (*initCallback)() = nullptr; 
void init() override;
float getSensorAngle() override;
};
#endif
GenericSensor::GenericSensor(float (*readCallback)(), void (*initCallback)()){
if(readCallback != nullptr) this->readCallback = readCallback;
if(initCallback != nullptr) this->initCallback = initCallback;
}
void GenericSensor::init(){
if(initCallback != nullptr) this->initCallback();
this->Sensor::init(); 
}
float GenericSensor::getSensorAngle(){
return this->readCallback();
}
#ifndef BLDCDriver3PWM_h
#define BLDCDriver3PWM_h
class BLDCDriver3PWM: public BLDCDriver
{
public:
BLDCDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);
int init() override;
void disable() override;
void enable() override;
int pwmA; 
int pwmB; 
int pwmC; 
int enableA_pin; 
int enableB_pin; 
int enableC_pin; 
bool enable_active_high = true;
void setPwm(float Ua, float Ub, float Uc) override;
virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;
private:
};
#endif
BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
pwmA = phA;
pwmB = phB;
pwmC = phC;
enableA_pin = en1;
enableB_pin = en2;
enableC_pin = en3;
voltage_power_supply = DEF_POWER_SUPPLY;
voltage_limit = NOT_SET;
pwm_frequency = NOT_SET;
}
void  BLDCDriver3PWM::enable(){
if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
setPwm(0,0,0);
}
void BLDCDriver3PWM::disable()
{
setPwm(0, 0, 0);
if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);
}
int BLDCDriver3PWM::init() {
pinMode(pwmA, OUTPUT);
pinMode(pwmB, OUTPUT);
pinMode(pwmC, OUTPUT);
if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);
if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
}
}
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {
Ua = _constrain(Ua, 0.0f, voltage_limit);
Ub = _constrain(Ub, 0.0f, voltage_limit);
Uc = _constrain(Uc, 0.0f, voltage_limit);
dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
_writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}
#ifndef BLDCDriver6PWM_h
#define BLDCDriver6PWM_h
class BLDCDriver6PWM: public BLDCDriver
{
public:
BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en = NOT_SET);
int init() override;
void disable() override;
void enable() override;
int pwmA_h,pwmA_l; 
int pwmB_h,pwmB_l; 
int pwmC_h,pwmC_l; 
int enable_pin; 
bool enable_active_high = true;
float dead_zone; 
PhaseState phase_state[3]; 
void setPwm(float Ua, float Ub, float Uc) override;
virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;
private:
};
#endif
BLDCDriver6PWM::BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en){
pwmA_h = phA_h;
pwmB_h = phB_h;
pwmC_h = phC_h;
pwmA_l = phA_l;
pwmB_l = phB_l;
pwmC_l = phC_l;
enable_pin = en;
voltage_power_supply = DEF_POWER_SUPPLY;
voltage_limit = NOT_SET;
pwm_frequency = NOT_SET;
dead_zone = 0.02f;
}
void  BLDCDriver6PWM::enable(){
if ( _isset(enable_pin) ) digitalWrite(enable_pin, enable_active_high);
setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
setPwm(0, 0, 0);
}
void BLDCDriver6PWM::disable()
{
setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
setPwm(0, 0, 0);
if ( _isset(enable_pin) ) digitalWrite(enable_pin, !enable_active_high);
}
int BLDCDriver6PWM::init() {
pinMode(pwmA_h, OUTPUT);
pinMode(pwmB_h, OUTPUT);
pinMode(pwmC_h, OUTPUT);
pinMode(pwmA_l, OUTPUT);
pinMode(pwmB_l, OUTPUT);
pinMode(pwmC_l, OUTPUT);
if(_isset(enable_pin)) pinMode(enable_pin, OUTPUT);
if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
phase_state[0] = PhaseState::PHASE_OFF;
phase_state[1] = PhaseState::PHASE_OFF;
phase_state[2] = PhaseState::PHASE_OFF;
dc_a = dc_b = dc_c = 0;
params = _configure6PWM(pwm_frequency, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}
void BLDCDriver6PWM::setPwm(float Ua, float Ub, float Uc) {
Ua = _constrain(Ua, 0, voltage_limit);
Ub = _constrain(Ub, 0, voltage_limit);
Uc = _constrain(Uc, 0, voltage_limit);
dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
_writeDutyCycle6PWM(dc_a, dc_b, dc_c, phase_state, params);
}
void BLDCDriver6PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
phase_state[0] = sa;
phase_state[1] = sb;
phase_state[2] = sc;
}
#ifndef STEPPER_DRIVER_4PWM_h
#define STEPPER_DRIVER_4PWM_h
class StepperDriver4PWM: public StepperDriver
{
public:
StepperDriver4PWM(int ph1A,int ph1B,int ph2A,int ph2B, int en1 = NOT_SET, int en2 = NOT_SET);
int init() override;
void disable() override;
void enable() override;
int pwm1A; 
int pwm1B; 
int pwm2A; 
int pwm2B; 
int enable_pin1; 
int enable_pin2; 
void setPwm(float Ua, float Ub) override;
private:
};
#endif
StepperDriver4PWM::StepperDriver4PWM(int ph1A,int ph1B,int ph2A,int ph2B,int en1, int en2){
pwm1A = ph1A;
pwm1B = ph1B;
pwm2A = ph2A;
pwm2B = ph2B;
enable_pin1 = en1;
enable_pin2 = en2;
voltage_power_supply = DEF_POWER_SUPPLY;
voltage_limit = NOT_SET;
pwm_frequency = NOT_SET;
}
void  StepperDriver4PWM::enable(){
if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
setPwm(0,0);
}
void StepperDriver4PWM::disable()
{
setPwm(0, 0);
if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);
}
int StepperDriver4PWM::init() {
pinMode(pwm1A, OUTPUT);
pinMode(pwm1B, OUTPUT);
pinMode(pwm2A, OUTPUT);
pinMode(pwm2B, OUTPUT);
if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);
if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
params = _configure4PWM(pwm_frequency, pwm1A, pwm1B, pwm2A, pwm2B);
initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}
void StepperDriver4PWM::setPwm(float Ualpha, float Ubeta) {
float duty_cycle1A(0.0f),duty_cycle1B(0.0f),duty_cycle2A(0.0f),duty_cycle2B(0.0f);
Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
if( Ualpha > 0 )
duty_cycle1B = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
else
duty_cycle1A = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
if( Ubeta > 0 )
duty_cycle2B = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
else
duty_cycle2A = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
_writeDutyCycle4PWM(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, params);
}
#ifndef STEPPER_DRIVER_2PWM_h
#define STEPPER_DRIVER_2PWM_h
class StepperDriver2PWM: public StepperDriver
{
public:
StepperDriver2PWM(int pwm1, int* in1, int pwm2, int* in2, int en1 = NOT_SET, int en2 = NOT_SET);
StepperDriver2PWM(int pwm1, int dir1, int pwm2, int dir2, int en1 = NOT_SET, int en2 = NOT_SET);
int init() override;
void disable() override;
void enable() override;
int pwm1; 
int dir1a; 
int dir1b; 
int pwm2; 
int dir2a; 
int dir2b; 
int enable_pin1; 
int enable_pin2; 
void setPwm(float Ua, float Ub) override;
private:
};
#endif
StepperDriver2PWM::StepperDriver2PWM(int _pwm1, int* _in1, int _pwm2, int* _in2, int en1, int en2){
pwm1 = _pwm1; 
dir1a = _in1[0]; 
dir1b = _in1[1]; 
pwm2 = _pwm2; 
dir2a = _in2[0]; 
dir2b = _in2[1]; 
enable_pin1 = en1;
enable_pin2 = en2;
voltage_power_supply = DEF_POWER_SUPPLY;
voltage_limit = NOT_SET;
pwm_frequency = NOT_SET;
}
StepperDriver2PWM::StepperDriver2PWM(int _pwm1, int _dir1, int _pwm2, int _dir2, int en1, int en2){
pwm1 = _pwm1; 
dir1a = _dir1; 
pwm2 = _pwm2; 
dir2a = _dir2; 
dir1b = NOT_SET;
dir2b = NOT_SET;
enable_pin1 = en1;
enable_pin2 = en2;
voltage_power_supply = DEF_POWER_SUPPLY;
voltage_limit = NOT_SET;
pwm_frequency = NOT_SET;
}
void  StepperDriver2PWM::enable(){
if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, HIGH);
if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, HIGH);
setPwm(0,0);
}
void StepperDriver2PWM::disable()
{
setPwm(0, 0);
if ( _isset(enable_pin1) ) digitalWrite(enable_pin1, LOW);
if ( _isset(enable_pin2) ) digitalWrite(enable_pin2, LOW);
}
int StepperDriver2PWM::init() {
pinMode(pwm1, OUTPUT);
pinMode(pwm2, OUTPUT);
pinMode(dir1a, OUTPUT);
pinMode(dir2a, OUTPUT);
if( _isset(dir1b) ) pinMode(dir1b, OUTPUT);
if( _isset(dir2b) ) pinMode(dir2b, OUTPUT);
if( _isset(enable_pin1) ) pinMode(enable_pin1, OUTPUT);
if( _isset(enable_pin2) ) pinMode(enable_pin2, OUTPUT);
if( !_isset(voltage_limit)  || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;
params = _configure2PWM(pwm_frequency, pwm1, pwm2);
initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}
void StepperDriver2PWM::setPwm(float Ua, float Ub) {
float duty_cycle1(0.0f),duty_cycle2(0.0f);
Ua = _constrain(Ua, -voltage_limit, voltage_limit);
Ub = _constrain(Ub, -voltage_limit, voltage_limit);
duty_cycle1 = _constrain(abs(Ua)/voltage_power_supply,0.0f,1.0f);
duty_cycle2 = _constrain(abs(Ub)/voltage_power_supply,0.0f,1.0f);
digitalWrite(dir1a, Ua >= 0 ? LOW : HIGH);
if( _isset(dir1b) ) digitalWrite(dir1b, Ua >= 0 ? HIGH : LOW );
digitalWrite(dir2a, Ub >= 0 ? LOW : HIGH);
if( _isset(dir2b) ) digitalWrite(dir2b, Ub >= 0 ? HIGH : LOW );
_writeDutyCycle2PWM(duty_cycle1, duty_cycle2, params);
}
#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H
class InlineCurrentSense: public CurrentSense{
public:
InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET);
int init() override;
PhaseCurrent_s getPhaseCurrents() override;
int driverAlign(float align_voltage) override;
float gain_a; 
float gain_b; 
float gain_c; 
float offset_ia; 
float offset_ib; 
float offset_ic; 
private:
int pinA; 
int pinB; 
int pinC; 
float shunt_resistor; 
float amp_gain; 
float volts_to_amps_ratio; 
void calibrateOffsets();
};
#endif
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
pinA = _pinA;
pinB = _pinB;
pinC = _pinC;
shunt_resistor = _shunt_resistor;
amp_gain  = _gain;
volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; 
gain_a = volts_to_amps_ratio;
gain_b = volts_to_amps_ratio;
gain_c = volts_to_amps_ratio;
};
InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
pinA = _pinA;
pinB = _pinB;
pinC = _pinC;
volts_to_amps_ratio = 1000.0f / _mVpA; 
gain_a = volts_to_amps_ratio;
gain_b = volts_to_amps_ratio;
gain_c = volts_to_amps_ratio;
};
int InlineCurrentSense::init(){
void* drv_params = driver ? driver->params : nullptr;
params = _configureADCInline(drv_params,pinA,pinB,pinC);
if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;
calibrateOffsets();
initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
return 1;
}
void InlineCurrentSense::calibrateOffsets(){
const int calibration_rounds = 1000;
offset_ia = 0;
offset_ib = 0;
offset_ic = 0;
for (int i = 0; i < calibration_rounds; i++) {
if(_isset(pinA)) offset_ia += _readADCVoltageInline(pinA, params);
if(_isset(pinB)) offset_ib += _readADCVoltageInline(pinB, params);
if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC, params);
_delay(1);
}
if(_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
if(_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
PhaseCurrent_s current;
current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageInline(pinA, params) - offset_ia)*gain_a;
current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageInline(pinB, params) - offset_ib)*gain_b;
current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC, params) - offset_ic)*gain_c; 
return current;
}
int InlineCurrentSense::driverAlign(float voltage){
int exit_flag = 1;
if(skip_align) return exit_flag;
if (driver==nullptr) {
SIMPLEFOC_DEBUG("CUR: No driver linked!");
return 0;
}
if (!initialized) return 0;
if(_isset(pinA)){
driver->setPwm(voltage, 0, 0);
_delay(2000);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
if(_isset(pinB) && ab_ratio > 1.5f ){ 
gain_a *= _sign(c.a);
}else if(_isset(pinC) && ac_ratio > 1.5f ){ 
gain_a *= _sign(c.a);
}else if(_isset(pinB) && ab_ratio < 0.7f ){ 
int tmp_pinA = pinA;
pinA = pinB;
pinB = tmp_pinA;
float tmp_offsetA = offset_ia;
offset_ia = offset_ib;
offset_ib = tmp_offsetA;
gain_a *= _sign(c.b);
exit_flag = 2; 
}else if(_isset(pinC) &&  ac_ratio < 0.7f ){ 
int tmp_pinA = pinA;
pinA = pinC;
pinC= tmp_pinA;
float tmp_offsetA = offset_ia;
offset_ia = offset_ic;
offset_ic = tmp_offsetA;
gain_a *= _sign(c.c);
exit_flag = 2;
}else{
return 0;
}
}
if(_isset(pinB)){
driver->setPwm(0, voltage, 0);
_delay(200);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
if(_isset(pinA) && ba_ratio > 1.5f ){ 
gain_b *= _sign(c.b);
}else if(_isset(pinC) && bc_ratio > 1.5f ){ 
gain_b *= _sign(c.b);
}else if(_isset(pinA) && ba_ratio < 0.7f ){ 
int tmp_pinB = pinB;
pinB = pinA;
pinA = tmp_pinB;
float tmp_offsetB = offset_ib;
offset_ib = offset_ia;
offset_ia = tmp_offsetB;
gain_b *= _sign(c.a);
exit_flag = 2; 
}else if(_isset(pinC) && bc_ratio < 0.7f ){ 
int tmp_pinB = pinB;
pinB = pinC;
pinC = tmp_pinB;
float tmp_offsetB = offset_ib;
offset_ib = offset_ic;
offset_ic = tmp_offsetB;
gain_b *= _sign(c.c);
exit_flag = 2; 
}else{
return 0;
}
}
if(_isset(pinC)){
driver->setPwm(0, 0, voltage);
_delay(200);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
if(_isset(pinA) && ca_ratio > 1.5f ){ 
gain_c *= _sign(c.c);
}else if(_isset(pinB) && cb_ratio > 1.5f ){ 
gain_c *= _sign(c.c);
}else if(_isset(pinA) && ca_ratio < 0.7f ){ 
int tmp_pinC = pinC;
pinC = pinA;
pinA = tmp_pinC;
float tmp_offsetC = offset_ic;
offset_ic = offset_ia;
offset_ia = tmp_offsetC;
gain_c *= _sign(c.a);
exit_flag = 2; 
}else if(_isset(pinB) && cb_ratio < 0.7f ){ 
int tmp_pinC = pinC;
pinC = pinB;
pinB = tmp_pinC;
float tmp_offsetC = offset_ic;
offset_ic = offset_ib;
offset_ib = tmp_offsetC;
gain_c *= _sign(c.b);
exit_flag = 2; 
}else{
return 0;
}
}
if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
return exit_flag;
}
#ifndef LOWSIDE_CS_LIB_H
#define LOWSIDE_CS_LIB_H
class LowsideCurrentSense: public CurrentSense{
public:
LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);
int init() override;
PhaseCurrent_s getPhaseCurrents() override;
int driverAlign(float align_voltage) override;
float gain_a; 
float gain_b; 
float gain_c; 
float offset_ia; 
float offset_ib; 
float offset_ic; 
private:
int pinA; 
int pinB; 
int pinC; 
float shunt_resistor; 
float amp_gain; 
float volts_to_amps_ratio; 
void calibrateOffsets();
};
#endif
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
pinA = _pinA;
pinB = _pinB;
pinC = _pinC;
shunt_resistor = _shunt_resistor;
amp_gain  = _gain;
volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; 
gain_a = volts_to_amps_ratio;
gain_b = volts_to_amps_ratio;
gain_c = volts_to_amps_ratio;
}
LowsideCurrentSense::LowsideCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
pinA = _pinA;
pinB = _pinB;
pinC = _pinC;
volts_to_amps_ratio = 1000.0f / _mVpA; 
gain_a = volts_to_amps_ratio;
gain_b = volts_to_amps_ratio;
gain_c = volts_to_amps_ratio;
}
int LowsideCurrentSense::init(){
if (driver==nullptr) {
SIMPLEFOC_DEBUG("CUR: Driver not linked!");
return 0;
}
params = _configureADCLowSide(driver->params,pinA,pinB,pinC);
if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;
_driverSyncLowSide(driver->params, params);
calibrateOffsets();
initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
return 1;
}
void LowsideCurrentSense::calibrateOffsets(){
const int calibration_rounds = 2000;
offset_ia = 0;
offset_ib = 0;
offset_ic = 0;
for (int i = 0; i < calibration_rounds; i++) {
_startADC3PinConversionLowSide();
if(_isset(pinA)) offset_ia += (_readADCVoltageLowSide(pinA, params));
if(_isset(pinB)) offset_ib += (_readADCVoltageLowSide(pinB, params));
if(_isset(pinC)) offset_ic += (_readADCVoltageLowSide(pinC, params));
_delay(1);
}
if(_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
if(_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}
PhaseCurrent_s LowsideCurrentSense::getPhaseCurrents(){
PhaseCurrent_s current;
_startADC3PinConversionLowSide();
current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageLowSide(pinA, params) - offset_ia)*gain_a;
current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageLowSide(pinB, params) - offset_ib)*gain_b;
current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageLowSide(pinC, params) - offset_ic)*gain_c; 
return current;
}
int LowsideCurrentSense::driverAlign(float voltage){
int exit_flag = 1;
if(skip_align) return exit_flag;
if (!initialized) return 0;
if(_isset(pinA)){
driver->setPwm(voltage, 0, 0);
_delay(2000);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
if(_isset(pinB) && ab_ratio > 1.5f ){ 
gain_a *= _sign(c.a);
}else if(_isset(pinC) && ac_ratio > 1.5f ){ 
gain_a *= _sign(c.a);
}else if(_isset(pinB) && ab_ratio < 0.7f ){ 
int tmp_pinA = pinA;
pinA = pinB;
pinB = tmp_pinA;
float tmp_offsetA = offset_ia;
offset_ia = offset_ib;
offset_ib = tmp_offsetA;
gain_a *= _sign(c.b);
exit_flag = 2; 
}else if(_isset(pinC) &&  ac_ratio < 0.7f ){ 
int tmp_pinA = pinA;
pinA = pinC;
pinC= tmp_pinA;
float tmp_offsetA = offset_ia;
offset_ia = offset_ic;
offset_ic = tmp_offsetA;
gain_a *= _sign(c.c);
exit_flag = 2;
}else{
return 0;
}
}
if(_isset(pinB)){
driver->setPwm(0, voltage, 0);
_delay(200);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
if(_isset(pinA) && ba_ratio > 1.5f ){ 
gain_b *= _sign(c.b);
}else if(_isset(pinC) && bc_ratio > 1.5f ){ 
gain_b *= _sign(c.b);
}else if(_isset(pinA) && ba_ratio < 0.7f ){ 
int tmp_pinB = pinB;
pinB = pinA;
pinA = tmp_pinB;
float tmp_offsetB = offset_ib;
offset_ib = offset_ia;
offset_ia = tmp_offsetB;
gain_b *= _sign(c.a);
exit_flag = 2; 
}else if(_isset(pinC) && bc_ratio < 0.7f ){ 
int tmp_pinB = pinB;
pinB = pinC;
pinC = tmp_pinB;
float tmp_offsetB = offset_ib;
offset_ib = offset_ic;
offset_ic = tmp_offsetB;
gain_b *= _sign(c.c);
exit_flag = 2; 
}else{
return 0;
}
}
if(_isset(pinC)){
driver->setPwm(0, 0, voltage);
_delay(200);
PhaseCurrent_s c = getPhaseCurrents();
for (int i = 0; i < 100; i++) {
PhaseCurrent_s c1 = getPhaseCurrents();
c.a = c.a*0.6f + 0.4f*c1.a;
c.b = c.b*0.6f + 0.4f*c1.b;
c.c = c.c*0.6f + 0.4f*c1.c;
_delay(3);
}
driver->setPwm(0, 0, 0);
float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
if(_isset(pinA) && ca_ratio > 1.5f ){ 
gain_c *= _sign(c.c);
}else if(_isset(pinB) && cb_ratio > 1.5f ){ 
gain_c *= _sign(c.c);
}else if(_isset(pinA) && ca_ratio < 0.7f ){ 
int tmp_pinC = pinC;
pinC = pinA;
pinA = tmp_pinC;
float tmp_offsetC = offset_ic;
offset_ic = offset_ia;
offset_ia = tmp_offsetC;
gain_c *= _sign(c.a);
exit_flag = 2; 
}else if(_isset(pinB) && cb_ratio < 0.7f ){ 
int tmp_pinC = pinC;
pinC = pinB;
pinB = tmp_pinC;
float tmp_offsetC = offset_ic;
offset_ic = offset_ib;
offset_ib = tmp_offsetC;
gain_c *= _sign(c.b);
exit_flag = 2; 
}else{
return 0;
}
}
if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
return exit_flag;
}
#ifndef GENERIC_CS_LIB_H
#define GENERIC_CS_LIB_H
class GenericCurrentSense: public CurrentSense{
public:
GenericCurrentSense(PhaseCurrent_s (*readCallback)() = nullptr, void (*initCallback)() = nullptr);
int init() override;
PhaseCurrent_s getPhaseCurrents() override;
int driverAlign(float align_voltage) override;
PhaseCurrent_s (*readCallback)() = nullptr; 
void (*initCallback)() = nullptr; 
private:
void calibrateOffsets();
float offset_ia; 
float offset_ib; 
float offset_ic; 
};
#endif
GenericCurrentSense::GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)()){
if(readCallback != nullptr) this->readCallback = readCallback;
if(initCallback != nullptr) this->initCallback = initCallback;
}
int GenericCurrentSense::init(){
if(initCallback != nullptr) initCallback();
calibrateOffsets();
initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
return 1;
}
void GenericCurrentSense::calibrateOffsets(){
const int calibration_rounds = 1000;
offset_ia = 0;
offset_ib = 0;
offset_ic = 0;
for (int i = 0; i < calibration_rounds; i++) {
PhaseCurrent_s current = readCallback();
offset_ia += current.a;
offset_ib += current.b;
offset_ic += current.c;
_delay(1);
}
offset_ia = offset_ia / calibration_rounds;
offset_ib = offset_ib / calibration_rounds;
offset_ic = offset_ic / calibration_rounds;
}
PhaseCurrent_s GenericCurrentSense::getPhaseCurrents(){
PhaseCurrent_s current = readCallback();
current.a = (current.a - offset_ia); 
current.b = (current.b - offset_ib); 
current.c = (current.c - offset_ic); 
return current;
}
int GenericCurrentSense::driverAlign(float voltage){
_UNUSED(voltage) ; 
int exit_flag = 1;
if(skip_align) return exit_flag;
if (!initialized) return 0;
return exit_flag;
}
#ifndef COMMANDS_H
#define COMMANDS_H
#define MAX_COMMAND_LENGTH 20
enum VerboseMode : uint8_t {
nothing       = 0x00, 
on_request    = 0x01, 
user_friendly = 0x02,  
machine_readable = 0x03 
};
typedef void (* CommandCallback)(char*); 
class Commander
{
public:
Commander(Stream &serial, char eol = '\n', bool echo = false);
Commander(char eol = '\n', bool echo = false);
void run();
void run(Stream &reader, char eol = '\n');
void run(char* user_input);
void add(char id , CommandCallback onCommand, const char* label = nullptr);
VerboseMode verbose = VerboseMode::user_friendly; 
uint8_t decimal_places = 3; 
Stream* com_port = nullptr; 
char eol = '\n'; 
bool echo = false; 
void motor(FOCMotor* motor, char* user_cmd);
void lpf(LowPassFilter* lpf, char* user_cmd);
void pid(PIDController* pid, char* user_cmd);
void scalar(float* value, char* user_cmd);
void target(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");
void motion(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");
bool isSentinel(char ch);
private:
CommandCallback call_list[20];
char call_ids[20]; 
char* call_label[20]; 
int call_count = 0;
char received_chars[MAX_COMMAND_LENGTH] = {0}; 
int rec_cnt = 0; 
void printVerbose(const char* message);
void printVerbose(const __FlashStringHelper *message);
void print(const float number);
void print(const int number);
void print(const char* message);
void print(const __FlashStringHelper *message);
void print(const char message);
void println(const float number);
void println(const int number);
void println(const char* message);
void println(const __FlashStringHelper *message);
void println(const char message);
void printMachineReadable(const float number);
void printMachineReadable(const int number);
void printMachineReadable(const char* message);
void printMachineReadable(const __FlashStringHelper *message);
void printMachineReadable(const char message);
void printlnMachineReadable(const float number);
void printlnMachineReadable(const int number);
void printlnMachineReadable(const char* message);
void printlnMachineReadable(const __FlashStringHelper *message);
void printlnMachineReadable(const char message);
void printError();
};
#endif
Commander::Commander(Stream& serial, char eol, bool echo){
com_port = &serial;
this->eol = eol;
this->echo = echo;
}
Commander::Commander(char eol, bool echo){
this->eol = eol;
this->echo = echo;
}
void Commander::add(char id, CommandCallback onCommand, const char* label ){
call_list[call_count] = onCommand;
call_ids[call_count] = id;
call_label[call_count] = (char*)label;
call_count++;
}
void Commander::run(){
if(!com_port) return;
run(*com_port, eol);
}
void Commander::run(Stream& serial, char eol){
Stream* tmp = com_port; 
char eol_tmp = this->eol;
this->eol = eol;
com_port = &serial;
while (serial.available()) {
int ch = serial.read();
received_chars[rec_cnt++] = (char)ch;
if(echo)
print((char)ch);
if (isSentinel(ch)) {
run(received_chars);
received_chars[0] = 0;
rec_cnt=0;
}
if (rec_cnt>=MAX_COMMAND_LENGTH) { 
received_chars[0] = 0;
rec_cnt=0;
}
}
com_port = tmp; 
this->eol = eol_tmp;
}
void Commander::run(char* user_input){
char id = user_input[0];
switch(id){
case CMD_SCAN:
for(int i=0; i < call_count; i++){
printMachineReadable(CMD_SCAN);
print(call_ids[i]);
print(":");
if(call_label[i]) println(call_label[i]);
else println("");
}
break;
case CMD_VERBOSE:
if(!isSentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
printVerbose(F("Verb:"));
printMachineReadable(CMD_VERBOSE);
switch (verbose){
case VerboseMode::nothing:
println(F("off!"));
break;
case VerboseMode::on_request:
case VerboseMode::user_friendly:
println(F("on!"));
break;
case VerboseMode::machine_readable:
printlnMachineReadable(F("machine"));
break;
}
break;
case CMD_DECIMAL:
if(!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
printVerbose(F("Decimal:"));
printMachineReadable(CMD_DECIMAL);
println(decimal_places);
break;
default:
for(int i=0; i < call_count; i++){
if(id == call_ids[i]){
printMachineReadable(user_input[0]);
call_list[i](&user_input[1]);
break;
}
}
break;
}
}
void Commander::motor(FOCMotor* motor, char* user_command) {
if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+' || isSentinel(user_command[0])){
target(motor, user_command);
return;
}
char cmd = user_command[0];
char sub_cmd = user_command[1];
int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ||  (sub_cmd == '#') ?  2 :  1;
bool GET = isSentinel(user_command[value_index]);
float value = atof(&user_command[value_index]);
printMachineReadable(cmd);
if (sub_cmd >= 'A'  && sub_cmd <= 'Z') {
printMachineReadable(sub_cmd);
}
switch(cmd){
case CMD_C_Q_PID:      
printVerbose(F("PID curr q| "));
if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
else pid(&motor->PID_current_q,&user_command[1]);
break;
case CMD_C_D_PID:      
printVerbose(F("PID curr d| "));
if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
else pid(&motor->PID_current_d, &user_command[1]);
break;
case CMD_V_PID:      
printVerbose(F("PID vel| "));
if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
else pid(&motor->PID_velocity, &user_command[1]);
break;
case CMD_A_PID:      
printVerbose(F("PID angle| "));
if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
else pid(&motor->P_angle, &user_command[1]);
break;
case CMD_LIMITS:      
printVerbose(F("Limits| "));
switch (sub_cmd){
case SCMD_LIM_VOLT:      
printVerbose(F("volt: "));
if(!GET) {
motor->voltage_limit = value;
motor->PID_current_d.limit = value;
motor->PID_current_q.limit = value;
if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
}
println(motor->voltage_limit);
break;
case SCMD_LIM_CURR:      
printVerbose(F("curr: "));
if(!GET){
motor->current_limit = value;
if(_isset(motor->phase_resistance) || motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
}
println(motor->current_limit);
break;
case SCMD_LIM_VEL:      
printVerbose(F("vel: "));
if(!GET){
motor->velocity_limit = value;
motor->P_angle.limit = value;
}
println(motor->velocity_limit);
break;
default:
printError();
break;
}
break;
case CMD_MOTION_TYPE:
case CMD_TORQUE_TYPE:
case CMD_STATUS:
motion(motor, &user_command[0]);
break;
case CMD_PWMMOD:
printVerbose(F("PWM Mod | "));
switch (sub_cmd){
case SCMD_PWMMOD_TYPE:      
printVerbose(F("type: "));
if(!GET) motor->foc_modulation = (FOCModulationType)value;
switch(motor->foc_modulation){
case FOCModulationType::SinePWM:
println(F("SinePWM"));
break;
case FOCModulationType::SpaceVectorPWM:
println(F("SVPWM"));
break;
case FOCModulationType::Trapezoid_120:
println(F("Trap 120"));
break;
case FOCModulationType::Trapezoid_150:
println(F("Trap 150"));
break;
}
break;
case SCMD_PWMMOD_CENTER:      
printVerbose(F("center: "));
if(!GET) motor->modulation_centered = value;
println(motor->modulation_centered);
break;
default:
printError();
break;
}
break;
case CMD_RESIST:
printVerbose(F("R phase: "));
if(!GET){
motor->phase_resistance = value;
if(motor->torque_controller==TorqueControlType::voltage)
motor->PID_velocity.limit= motor->current_limit;
}
if(_isset(motor->phase_resistance)) println(motor->phase_resistance);
else println(0);
break;
case CMD_INDUCTANCE:
printVerbose(F("L phase: "));
if(!GET){
motor->phase_inductance = value;
}
if(_isset(motor->phase_inductance)) println(motor->phase_inductance);
else println(0);
break;
case CMD_KV_RATING:
printVerbose(F("Motor KV: "));
if(!GET){
motor->KV_rating = value;
}
if(_isset(motor->KV_rating)) println(motor->KV_rating);
else println(0);
break;
case CMD_SENSOR:
printVerbose(F("Sensor | "));
switch (sub_cmd){
case SCMD_SENS_MECH_OFFSET:      
printVerbose(F("offset: "));
if(!GET) motor->sensor_offset = value;
println(motor->sensor_offset);
break;
case SCMD_SENS_ELEC_OFFSET:      
printVerbose(F("el. offset: "));
if(!GET) motor->zero_electric_angle = value;
println(motor->zero_electric_angle);
break;
default:
printError();
break;
}
break;
case CMD_MONITOR:     
printVerbose(F("Monitor | "));
switch (sub_cmd){
case SCMD_GET:      
switch((uint8_t)value){
case 0: 
printVerbose(F("target: "));
println(motor->target);
break;
case 1: 
printVerbose(F("Vq: "));
println(motor->voltage.q);
break;
case 2: 
printVerbose(F("Vd: "));
println(motor->voltage.d);
break;
case 3: 
printVerbose(F("Cq: "));
println(motor->current.q);
break;
case 4: 
printVerbose(F("Cd: "));
println(motor->current.d);
break;
case 5: 
printVerbose(F("vel: "));
println(motor->shaft_velocity);
break;
case 6: 
printVerbose(F("angle: "));
println(motor->shaft_angle);
break;
case 7: 
printVerbose(F("all: "));
print(motor->target);
print(";");
print(motor->voltage.q);
print(";");
print(motor->voltage.d);
print(";");
print(motor->current.q);
print(";");
print(motor->current.d);
print(";");
print(motor->shaft_velocity);
print(";");
println(motor->shaft_angle);
break;
default:
printError();
break;
}
break;
case SCMD_DOWNSAMPLE:
printVerbose(F("downsample: "));
if(!GET) motor->monitor_downsample = value;
println((int)motor->monitor_downsample);
break;
case SCMD_CLEAR:
motor->monitor_variables = (uint8_t) 0;
println(F("clear"));
break;
case CMD_DECIMAL:
printVerbose(F("decimal: "));
motor->monitor_decimals = value;
println((int)motor->monitor_decimals);
break;
case SCMD_SET:
if(!GET){
motor->monitor_variables = (uint8_t) 0;
for(int i = 0; i < 7; i++){
if(isSentinel(user_command[value_index+i])) break;
motor->monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
}
}
for(int i = 0; i < 7; i++){
print( (motor->monitor_variables & (1 << (6-i))) >> (6-i));
}
println("");
break;
default:
printError();
break;
}
break;
default:  
printVerbose(F("unknown cmd "));
printError();
}
}
void Commander::motion(FOCMotor* motor, char* user_cmd, char* separator){
char cmd = user_cmd[0];
char sub_cmd = user_cmd[1];
bool GET  = isSentinel(user_cmd[1]);
float value = atof(&user_cmd[(sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1]);
switch(cmd){
case CMD_MOTION_TYPE:
printVerbose(F("Motion:"));
switch(sub_cmd){
case SCMD_DOWNSAMPLE:
printVerbose(F(" downsample: "));
if(!GET) motor->motion_downsample = value;
println((int)motor->motion_downsample);
break;
default:
if(!GET && value >= 0 && (int)value < 5) 
motor->controller = (MotionControlType)value;
switch(motor->controller){
case MotionControlType::torque:
println(F("torque"));
break;
case MotionControlType::velocity:
println(F("vel"));
break;
case MotionControlType::angle:
println(F("angle"));
break;
case MotionControlType::velocity_openloop:
println(F("vel open"));
break;
case MotionControlType::angle_openloop:
println(F("angle open"));
break;
}
break;
}
break;
case CMD_TORQUE_TYPE:
printVerbose(F("Torque: "));
if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)
motor->torque_controller = (TorqueControlType)value;
switch(motor->torque_controller){
case TorqueControlType::voltage:
println(F("volt"));
if( !_isset(motor->phase_resistance) ) motor->PID_velocity.limit = motor->voltage_limit;
break;
case TorqueControlType::dc_current:
println(F("dc curr"));
motor->PID_velocity.limit = motor->current_limit;
break;
case TorqueControlType::foc_current:
println(F("foc curr"));
motor->PID_velocity.limit = motor->current_limit;
break;
}
break;
case CMD_STATUS:
printVerbose(F("Status: "));
if(!GET) (bool)value ? motor->enable() : motor->disable();
println(motor->enabled);
break;
default:
target(motor,  user_cmd, separator);
break;
}
}
void Commander::pid(PIDController* pid, char* user_cmd){
char cmd = user_cmd[0];
bool GET  = isSentinel(user_cmd[1]);
float value = atof(&user_cmd[1]);
switch (cmd){
case SCMD_PID_P:      
printVerbose("P: ");
if(!GET) pid->P = value;
println(pid->P);
break;
case SCMD_PID_I:      
printVerbose("I: ");
if(!GET) pid->I = value;
println(pid->I);
break;
case SCMD_PID_D:      
printVerbose("D: ");
if(!GET) pid->D = value;
println(pid->D);
break;
case SCMD_PID_RAMP:      
printVerbose("ramp: ");
if(!GET) pid->output_ramp = value;
println(pid->output_ramp);
break;
case SCMD_PID_LIM:      
printVerbose("limit: ");
if(!GET) pid->limit = value;
println(pid->limit);
break;
default:
printError();
break;
}
}
void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
char cmd = user_cmd[0];
bool GET  = isSentinel(user_cmd[1]);
float value = atof(&user_cmd[1]);
switch (cmd){
case SCMD_LPF_TF:      
printVerbose(F("Tf: "));
if(!GET) lpf->Tf = value;
println(lpf->Tf);
break;
default:
printError();
break;
}
}
void Commander::scalar(float* value,  char* user_cmd){
bool GET  = isSentinel(user_cmd[0]);
if(!GET) *value = atof(user_cmd);
println(*value);
}
void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
if(isSentinel(user_cmd[0])) {
printlnMachineReadable(motor->target);
return;
};
float pos, vel, torque;
char* next_value;
switch(motor->controller){
case MotionControlType::torque: 
torque = atof(strtok (user_cmd, separator));
motor->target = torque;
break;
case MotionControlType::velocity: 
vel= atof(strtok (user_cmd, separator));
motor->target = vel;
next_value = strtok (NULL, separator);
if (next_value){
torque = atof(next_value);
motor->PID_velocity.limit = torque;
if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
else  motor->current_limit = torque;
}
break;
case MotionControlType::angle: 
pos= atof(strtok (user_cmd, separator));
motor->target = pos;
next_value = strtok (NULL, separator);
if( next_value ){
vel = atof(next_value);
motor->velocity_limit = vel;
motor->P_angle.limit = vel;
next_value = strtok (NULL, separator);
if( next_value ){
torque= atof(next_value);
motor->PID_velocity.limit = torque;
if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
else  motor->current_limit = torque;
}
}
break;
case MotionControlType::velocity_openloop: 
vel= atof(strtok (user_cmd, separator));
motor->target = vel;
next_value = strtok (NULL, separator);
if (next_value ){
torque = atof(next_value);
if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
else  motor->current_limit = torque;
}
break;
case MotionControlType::angle_openloop: 
pos= atof(strtok (user_cmd, separator));
motor->target = pos;
next_value = strtok (NULL, separator);
if( next_value ){
vel = atof(next_value);
motor->velocity_limit = vel;
next_value = strtok (NULL, separator);
if (next_value ){
torque = atof(next_value);
if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
else  motor->current_limit = torque;
}
}
break;
}
printVerbose(F("Target: "));
println(motor->target);
}
bool Commander::isSentinel(char ch)
{
if(ch == eol)
return true;
else if (ch == '\r')
{
printVerbose(F("Warn: \\r detected! \n"));
return true; 
}
return false;
}
void Commander::print(const int number){
if( !com_port || verbose == VerboseMode::nothing ) return;
com_port->print(number);
}
void Commander::print(const float number){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->print((float)number,(int)decimal_places);
}
void Commander::print(const char* message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->print(message);
}
void Commander::print(const __FlashStringHelper *message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->print(message);
}
void Commander::print(const char message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->print(message);
}
void Commander::println(const int number){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->println(number);
}
void Commander::println(const float number){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->println((float)number, (int)decimal_places);
}
void Commander::println(const char* message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->println(message);
}
void Commander::println(const __FlashStringHelper *message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->println(message);
}
void Commander::println(const char message){
if(!com_port || verbose == VerboseMode::nothing ) return;
com_port->println(message);
}
void Commander::printVerbose(const char* message){
if(verbose == VerboseMode::user_friendly) print(message);
}
void Commander::printVerbose(const __FlashStringHelper *message){
if(verbose == VerboseMode::user_friendly) print(message);
}
void Commander::printMachineReadable(const int number){
if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::printMachineReadable(const float number){
if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::printMachineReadable(const char* message){
if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::printMachineReadable(const __FlashStringHelper *message){
if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::printMachineReadable(const char message){
if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::printlnMachineReadable(const int number){
if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::printlnMachineReadable(const float number){
if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::printlnMachineReadable(const char* message){
if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::printlnMachineReadable(const __FlashStringHelper *message){
if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::printlnMachineReadable(const char message){
if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::printError(){
println(F("err"));
}
#ifndef STEPDIR_H
#define STEPDIR_H
class StepDirListener
{
public:
StepDirListener(int pinStep, int pinDir, float counter_to_value = 1);
void enableInterrupt(void (*handleStep)());
void init();
void handle();
float getValue();
void attach(float* variable);
int pin_step; 
int pin_dir; 
long count; 
decltype(RISING) polarity = RISING; 
private:
float* attached_variable = nullptr; 
float counter_to_value; 
};
#endif
StepDirListener::StepDirListener(int _pinStep, int _pinDir, float _counter_to_value){
pin_step = _pinStep;
pin_dir = _pinDir;
counter_to_value = _counter_to_value;
}
void StepDirListener::init(){
pinMode(pin_dir, INPUT);
pinMode(pin_step, INPUT_PULLUP);
count = 0;
}
void StepDirListener::enableInterrupt(void (*doA)()){
attachInterrupt(digitalPinToInterrupt(pin_step), doA, polarity);
}
void StepDirListener::attach(float* variable){
attached_variable = variable;
}
void StepDirListener::handle(){
if(digitalRead(pin_dir))
count++;
else
count--;
if(attached_variable) *attached_variable = getValue();
}
float StepDirListener::getValue(){
return (float) count * counter_to_value;
}
#endif
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_8, PA_9, PA_10, PA_7);
float target_angle = 0;
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void setup() {
sensor.init();
motor.linkSensor(&sensor);
driver.voltage_power_supply = 8;
driver.init();
motor.linkDriver(&driver);
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
motor.controller = MotionControlType::velocity;
motor.PID_velocity.P = 0.2f;
motor.PID_velocity.I = 1.6f;
motor.PID_velocity.D = 0;
motor.voltage_limit = 6;
motor.PID_velocity.output_ramp = 1000;
motor.LPF_velocity.Tf = 0.01f;
Serial.begin(115200);
motor.useMonitoring(Serial);
motor.init();
motor.initFOC();
command.add('T', doTarget, "target velocity");
Serial.println(F("Motor ready."));
Serial.println(F("Set the target angle using serial terminal:"));
_delay(1000);
}
void loop() {
motor.loopFOC();
motor.move(target_velocity);
command.run();
}