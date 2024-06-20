#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#ifndef SIMPLEFOC_H

#define SIMPLEFOC_H

#ifndef BLDCMotor_h

#define BLDCMotor_h

#ifndef FOCMOTOR_H

#define FOCMOTOR_H

#ifndef SENSOR_H

#define SENSOR_H

enum Direction : int8_t
{
    CW = 1,
    CCW = -1,
    UNKNOWN = 0
};

enum Pullup : uint8_t
{
    USE_INTERN = 0x00,
    USE_EXTERN = 0x01
};

class Sensor
{
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
    virtual float getSensorAngle() = 0;

    virtual void init();

    float velocity = 0.0f;
    float angle_prev = 0.0f;
    long angle_prev_ts = 0;
    float vel_angle_prev = 0.0f;
    long vel_angle_prev_ts = 0;
    int32_t full_rotations = 0;
    int32_t vel_full_rotations = 0;
};

#endif

#ifndef CURRENTSENSE_H

#define CURRENTSENSE_H

#ifndef BLDCDRIVER_H

#define BLDCDRIVER_H

enum PhaseState : uint8_t
{
    PHASE_OFF = 0,
    PHASE_ON = 1,
    PHASE_HI = 2,
    PHASE_LO = 3,
};

class BLDCDriver
{
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
    void *params = 0;

    virtual void setPwm(float Ua, float Ub, float Uc) = 0;

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;
};

#endif

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

class CurrentSense
{
public:
    virtual int init() = 0;

    void linkDriver(BLDCDriver *driver);

    bool skip_align = false;

    BLDCDriver *driver = nullptr;
    bool initialized = false;
    void *params = 0;

    virtual int driverAlign(float align_voltage) = 0;

    virtual PhaseCurrent_s getPhaseCurrents() = 0;

    virtual float getDCCurrent(float angle_el = 0);

    DQCurrent_s getFOCCurrents(float angle_el);

    ABCurrent_s getABCurrents(PhaseCurrent_s current);

    DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el);

    virtual void enable();

    virtual void disable();
};

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

#ifndef PID_H

#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator()(float error);
    void reset();

    float P;
    float I;
    float D;
    float output_ramp;
    float limit;

protected:
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
};

#endif

#ifndef LOWPASS_FILTER_H

#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif

#define _MON_TARGET 0b1000000

#define _MON_VOLT_Q 0b0100000

#define _MON_VOLT_D 0b0010000

#define _MON_CURR_Q 0b0001000

#define _MON_CURR_D 0b0000100

#define _MON_VEL 0b0000010

#define _MON_ANGLE 0b0000001

enum MotionControlType : uint8_t
{
    torque = 0x00,
    velocity = 0x01,
    angle = 0x02,
    velocity_openloop = 0x03,
    angle_openloop = 0x04
};

enum TorqueControlType : uint8_t
{
    voltage = 0x00,
    dc_current = 0x01,
    foc_current = 0x02,
};

enum FOCModulationType : uint8_t
{
    SinePWM = 0x00,
    SpaceVectorPWM = 0x01,
    Trapezoid_120 = 0x02,
    Trapezoid_150 = 0x03,
};

enum FOCMotorStatus : uint8_t
{
    motor_uninitialized = 0x00,
    motor_initializing = 0x01,
    motor_uncalibrated = 0x02,
    motor_calibrating = 0x03,
    motor_ready = 0x04,
    motor_error = 0x08,
    motor_calib_failed = 0x0E,
    motor_init_failed = 0x0F,
};

class FOCMotor
{
public:
    FOCMotor();

    virtual void init() = 0;

    virtual void disable() = 0;

    virtual void enable() = 0;

    void linkSensor(Sensor *sensor);

    void linkCurrentSense(CurrentSense *current_sense);

    virtual int initFOC() = 0;

    virtual void loopFOC() = 0;

    virtual void move(float target = NOT_SET) = 0;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;

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
    float Ualpha, Ubeta;

    float voltage_sensor_align;
    float velocity_index_search;

    float phase_resistance;
    int pole_pairs;
    float KV_rating;
    float phase_inductance;

    float voltage_limit;
    float current_limit;
    float velocity_limit;

    int8_t enabled = 0;
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized;

    FOCModulationType foc_modulation;
    int8_t modulation_centered = 1;

    TorqueControlType torque_controller;
    MotionControlType controller;

    PIDController PID_current_q{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    PIDController PID_current_d{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};
    PIDController PID_velocity{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT};
    PIDController P_angle{DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM};
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
    unsigned int monitor_decimals = 4;

    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

    Sensor *sensor;

    CurrentSense *current_sense;

    Print *monitor_port;

private:
    unsigned int monitor_cnt = 0;
};

#endif

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

class BLDCMotor : public FOCMotor
{
public:
    BLDCMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);

    virtual void linkDriver(BLDCDriver *driver);

    BLDCDriver *driver;

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

#ifndef StepperMotor_h

#define StepperMotor_h

#ifndef STEPPERDRIVER_H

#define STEPPERDRIVER_H

class StepperDriver
{
public:
    virtual int init() = 0;

    virtual void enable() = 0;

    virtual void disable() = 0;

    long pwm_frequency;
    float voltage_power_supply;
    float voltage_limit;

    bool initialized = false;
    void *params = 0;

    virtual void setPwm(float Ua, float Ub) = 0;
};

#endif

class StepperMotor : public FOCMotor
{
public:
    StepperMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);

    void linkDriver(StepperDriver *driver);

    StepperDriver *driver;

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

#ifndef ENCODER_LIB_H

#define ENCODER_LIB_H

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#ifndef SENSOR_H

#define SENSOR_H

enum Direction : int8_t
{
    CW = 1,
    CCW = -1,
    UNKNOWN = 0
};

enum Pullup : uint8_t
{
    USE_INTERN = 0x00,
    USE_EXTERN = 0x01
};

class Sensor
{
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
    virtual float getSensorAngle() = 0;

    virtual void init();

    float velocity = 0.0f;
    float angle_prev = 0.0f;
    long angle_prev_ts = 0;
    float vel_angle_prev = 0.0f;
    long vel_angle_prev_ts = 0;
    int32_t full_rotations = 0;
    int32_t vel_full_rotations = 0;
};

#endif

enum Quadrature : uint8_t
{
    ON = 0x00,
    OFF = 0x01
};

class Encoder : public Sensor
{
public:
    Encoder(int encA, int encB, float ppr, int index = 0);

    void init() override;

    void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr, void (*doIndex)() = nullptr);

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

#ifndef MAGNETICSENSORSPI_LIB_H

#define MAGNETICSENSORSPI_LIB_H

#define DEF_ANGLE_REGISTER 0x3FFF

struct MagneticSensorSPIConfig_s
{
    int spi_mode;
    long clock_speed;
    int bit_resolution;
    int angle_register;
    int data_start_bit;
    int command_rw_bit;
    int command_parity_bit;
};

extern MagneticSensorSPIConfig_s AS5147_SPI, AS5048_SPI, AS5047_SPI, MA730_SPI;

class MagneticSensorSPI : public Sensor
{
public:
    MagneticSensorSPI(int cs, int bit_resolution, int angle_register = 0);

    MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs);

    void init(SPIClass *_spi = &SPI);

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

    SPIClass *spi;
};

#endif

#ifndef MAGNETICSENSORI2C_LIB_H

#define MAGNETICSENSORI2C_LIB_H

struct MagneticSensorI2CConfig_s
{
    int chip_address;
    int bit_resolution;
    int angle_register;
    int data_start_bit;
};

extern MagneticSensorI2CConfig_s AS5600_I2C, AS5048_I2C;

#if defined(TARGET_RP2040)

#define SDA I2C_SDA

#define SCL I2C_SCL

#endif

class MagneticSensorI2C : public Sensor
{
public:
    MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);

    MagneticSensorI2C(MagneticSensorI2CConfig_s config);

    static MagneticSensorI2C AS5600();

    void init(TwoWire *_wire = &Wire);

    float getSensorAngle() override;

    int checkBus(byte sda_pin, byte scl_pin);

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

    TwoWire *wire;
};

#endif

#ifndef MAGNETICSENSORANALOG_LIB_H

#define MAGNETICSENSORANALOG_LIB_H

class MagneticSensorAnalog : public Sensor
{
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

#ifndef MAGNETICSENSORPWM_LIB_H

#define MAGNETICSENSORPWM_LIB_H

class MagneticSensorPWM : public Sensor
{
public:
    MagneticSensorPWM(uint8_t _pinPWM, int _min = 0, int _max = 0);

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

#ifndef HALL_SENSOR_LIB_H

#define HALL_SENSOR_LIB_H

const int8_t ELECTRIC_SECTORS[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

class HallSensor : public Sensor
{
public:
    HallSensor(int encA, int encB, int encC, int pp);

    void init();

    void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr, void (*doC)() = nullptr);

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

#ifndef GENERIC_SENSOR_LIB_H

#define GENERIC_SENSOR_LIB_H

class GenericSensor : public Sensor
{
public:
    GenericSensor(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr);

    float (*readCallback)() = nullptr;
    void (*initCallback)() = nullptr;

    void init() override;

    float getSensorAngle() override;
};

#endif

#ifndef BLDCDriver3PWM_h

#define BLDCDriver3PWM_h

#ifndef BLDCDRIVER_H

#define BLDCDRIVER_H

enum PhaseState : uint8_t
{
    PHASE_OFF = 0,
    PHASE_ON = 1,
    PHASE_HI = 2,
    PHASE_LO = 3,
};

class BLDCDriver
{
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
    void *params = 0;

    virtual void setPwm(float Ua, float Ub, float Uc) = 0;

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;
};

#endif

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

#ifndef HARDWARE_UTILS_DRIVER_H

#define HARDWARE_UTILS_DRIVER_H

#ifndef __SIMPLEFOCDEBUG_H__

#define __SIMPLEFOCDEBUG_H__

#ifndef SIMPLEFOC_DISABLE_DEBUG

class SimpleFOCDebug
{
public:
    static void enable(Print *debugPrint = &Serial);

    static void println(const __FlashStringHelper *msg);
    static void println(const char *msg);
    static void println(const __FlashStringHelper *msg, float val);
    static void println(const char *msg, float val);
    static void println(const __FlashStringHelper *msg, int val);
    static void println(const char *msg, int val);
    static void println(const char *msg, char val);
    static void println();
    static void println(int val);
    static void println(float val);

    static void print(const char *msg);
    static void print(const __FlashStringHelper *msg);
    static void print(int val);
    static void print(float val);

protected:
    static Print *_debugPrint;
};

#define SIMPLEFOC_DEBUG(msg, ...) \
    SimpleFOCDebug::println(F(msg), ##__VA_ARGS__)

#else

#define SIMPLEFOC_DEBUG(msg, ...)

#endif

#endif

#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH

#define SIMPLEFOC_PWM_ACTIVE_HIGH true

#endif

#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH

#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true

#endif

#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH

#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true

#endif

#define SIMPLEFOC_DRIVER_INIT_FAILED ((void *)-1)

typedef struct GenericDriverParams
{
    int pins[6];
    long pwm_frequency;
    float dead_zone;
} GenericDriverParams;

void *_configure1PWM(long pwm_frequency, const int pinA);

void *_configure2PWM(long pwm_frequency, const int pinA, const int pinB);

void *_configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);

void *_configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B);

void *_configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l, const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l);

void _writeDutyCycle1PWM(float dc_a, void *params);

void _writeDutyCycle2PWM(float dc_a, float dc_b, void *params);

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params);

void _writeDutyCycle4PWM(float dc_1a, float dc_1b, float dc_2a, float dc_2b, void *params);

void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState *phase_state, void *params);

#endif

class BLDCDriver3PWM : public BLDCDriver
{
public:
    BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);

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

#ifndef BLDCDriver6PWM_h

#define BLDCDriver6PWM_h

class BLDCDriver6PWM : public BLDCDriver
{
public:
    BLDCDriver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en = NOT_SET);

    int init() override;

    void disable() override;

    void enable() override;

    int pwmA_h, pwmA_l;
    int pwmB_h, pwmB_l;
    int pwmC_h, pwmC_l;
    int enable_pin;
    bool enable_active_high = true;

    float dead_zone;

    PhaseState phase_state[3];

    void setPwm(float Ua, float Ub, float Uc) override;

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;

private:
};

#endif

#ifndef STEPPER_DRIVER_4PWM_h

#define STEPPER_DRIVER_4PWM_h

#ifndef STEPPERDRIVER_H

#define STEPPERDRIVER_H

class StepperDriver
{
public:
    virtual int init() = 0;

    virtual void enable() = 0;

    virtual void disable() = 0;

    long pwm_frequency;
    float voltage_power_supply;
    float voltage_limit;

    bool initialized = false;
    void *params = 0;

    virtual void setPwm(float Ua, float Ub) = 0;
};

#endif

class StepperDriver4PWM : public StepperDriver
{
public:
    StepperDriver4PWM(int ph1A, int ph1B, int ph2A, int ph2B, int en1 = NOT_SET, int en2 = NOT_SET);

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

#ifndef STEPPER_DRIVER_2PWM_h

#define STEPPER_DRIVER_2PWM_h

class StepperDriver2PWM : public StepperDriver
{
public:
    StepperDriver2PWM(int pwm1, int *in1, int pwm2, int *in2, int en1 = NOT_SET, int en2 = NOT_SET);

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

#ifndef INLINE_CS_LIB_H

#define INLINE_CS_LIB_H

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

#ifndef CURRENTSENSE_H

#define CURRENTSENSE_H

#ifndef BLDCDRIVER_H

#define BLDCDRIVER_H

enum PhaseState : uint8_t
{
    PHASE_OFF = 0,
    PHASE_ON = 1,
    PHASE_HI = 2,
    PHASE_LO = 3,
};

class BLDCDriver
{
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
    void *params = 0;

    virtual void setPwm(float Ua, float Ub, float Uc) = 0;

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;
};

#endif

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

class CurrentSense
{
public:
    virtual int init() = 0;

    void linkDriver(BLDCDriver *driver);

    bool skip_align = false;

    BLDCDriver *driver = nullptr;
    bool initialized = false;
    void *params = 0;

    virtual int driverAlign(float align_voltage) = 0;

    virtual PhaseCurrent_s getPhaseCurrents() = 0;

    virtual float getDCCurrent(float angle_el = 0);

    DQCurrent_s getFOCCurrents(float angle_el);

    ABCurrent_s getABCurrents(PhaseCurrent_s current);

    DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el);

    virtual void enable();

    virtual void disable();
};

#endif

#ifndef LOWPASS_FILTER_H

#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif

#ifndef HARDWARE_UTILS_CURRENT_H

#define HARDWARE_UTILS_CURRENT_H

#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void *)-1)

typedef struct GenericCurrentSenseParams
{
    int pins[3];
    float adc_voltage_conv;
} GenericCurrentSenseParams;

float _readADCVoltageInline(const int pinA, const void *cs_params);

void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC = NOT_SET);

void *_configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC = NOT_SET);

void _startADC3PinConversionLowSide();

float _readADCVoltageLowSide(const int pinA, const void *cs_params);

void _driverSyncLowSide(void *driver_params, void *cs_params);

#endif

class InlineCurrentSense : public CurrentSense
{
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

#ifndef LOWSIDE_CS_LIB_H

#define LOWSIDE_CS_LIB_H

#ifndef FOCMOTOR_H

#define FOCMOTOR_H

#ifndef SENSOR_H

#define SENSOR_H

enum Direction : int8_t
{
    CW = 1,
    CCW = -1,
    UNKNOWN = 0
};

enum Pullup : uint8_t
{
    USE_INTERN = 0x00,
    USE_EXTERN = 0x01
};

class Sensor
{
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
    virtual float getSensorAngle() = 0;

    virtual void init();

    float velocity = 0.0f;
    float angle_prev = 0.0f;
    long angle_prev_ts = 0;
    float vel_angle_prev = 0.0f;
    long vel_angle_prev_ts = 0;
    int32_t full_rotations = 0;
    int32_t vel_full_rotations = 0;
};

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

#ifndef PID_H

#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator()(float error);
    void reset();

    float P;
    float I;
    float D;
    float output_ramp;
    float limit;

protected:
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
};

#endif

#ifndef LOWPASS_FILTER_H

#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif

#define _MON_TARGET 0b1000000

#define _MON_VOLT_Q 0b0100000

#define _MON_VOLT_D 0b0010000

#define _MON_CURR_Q 0b0001000

#define _MON_CURR_D 0b0000100

#define _MON_VEL 0b0000010

#define _MON_ANGLE 0b0000001

enum MotionControlType : uint8_t
{
    torque = 0x00,
    velocity = 0x01,
    angle = 0x02,
    velocity_openloop = 0x03,
    angle_openloop = 0x04
};

enum TorqueControlType : uint8_t
{
    voltage = 0x00,
    dc_current = 0x01,
    foc_current = 0x02,
};

enum FOCModulationType : uint8_t
{
    SinePWM = 0x00,
    SpaceVectorPWM = 0x01,
    Trapezoid_120 = 0x02,
    Trapezoid_150 = 0x03,
};

enum FOCMotorStatus : uint8_t
{
    motor_uninitialized = 0x00,
    motor_initializing = 0x01,
    motor_uncalibrated = 0x02,
    motor_calibrating = 0x03,
    motor_ready = 0x04,
    motor_error = 0x08,
    motor_calib_failed = 0x0E,
    motor_init_failed = 0x0F,
};

class FOCMotor
{
public:
    FOCMotor();

    virtual void init() = 0;

    virtual void disable() = 0;

    virtual void enable() = 0;

    void linkSensor(Sensor *sensor);

    void linkCurrentSense(CurrentSense *current_sense);

    virtual int initFOC() = 0;

    virtual void loopFOC() = 0;

    virtual void move(float target = NOT_SET) = 0;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;

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
    float Ualpha, Ubeta;

    float voltage_sensor_align;
    float velocity_index_search;

    float phase_resistance;
    int pole_pairs;
    float KV_rating;
    float phase_inductance;

    float voltage_limit;
    float current_limit;
    float velocity_limit;

    int8_t enabled = 0;
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized;

    FOCModulationType foc_modulation;
    int8_t modulation_centered = 1;

    TorqueControlType torque_controller;
    MotionControlType controller;

    PIDController PID_current_q{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    PIDController PID_current_d{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};
    PIDController PID_velocity{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT};
    PIDController P_angle{DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM};
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
    unsigned int monitor_decimals = 4;

    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

    Sensor *sensor;

    CurrentSense *current_sense;

    Print *monitor_port;

private:
    unsigned int monitor_cnt = 0;
};

#endif

class LowsideCurrentSense : public CurrentSense
{
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

#ifndef GENERIC_CS_LIB_H

#define GENERIC_CS_LIB_H

class GenericCurrentSense : public CurrentSense
{
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

#ifndef COMMANDS_H

#define COMMANDS_H

#ifndef FOCMOTOR_H

#define FOCMOTOR_H

#ifndef SENSOR_H

#define SENSOR_H

enum Direction : int8_t
{
    CW = 1,
    CCW = -1,
    UNKNOWN = 0
};

enum Pullup : uint8_t
{
    USE_INTERN = 0x00,
    USE_EXTERN = 0x01
};

class Sensor
{
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
    virtual float getSensorAngle() = 0;

    virtual void init();

    float velocity = 0.0f;
    float angle_prev = 0.0f;
    long angle_prev_ts = 0;
    float vel_angle_prev = 0.0f;
    long vel_angle_prev_ts = 0;
    int32_t full_rotations = 0;
    int32_t vel_full_rotations = 0;
};

#endif

#ifndef CURRENTSENSE_H

#define CURRENTSENSE_H

#ifndef BLDCDRIVER_H

#define BLDCDRIVER_H

enum PhaseState : uint8_t
{
    PHASE_OFF = 0,
    PHASE_ON = 1,
    PHASE_HI = 2,
    PHASE_LO = 3,
};

class BLDCDriver
{
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
    void *params = 0;

    virtual void setPwm(float Ua, float Ub, float Uc) = 0;

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;
};

#endif

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

class CurrentSense
{
public:
    virtual int init() = 0;

    void linkDriver(BLDCDriver *driver);

    bool skip_align = false;

    BLDCDriver *driver = nullptr;
    bool initialized = false;
    void *params = 0;

    virtual int driverAlign(float align_voltage) = 0;

    virtual PhaseCurrent_s getPhaseCurrents() = 0;

    virtual float getDCCurrent(float angle_el = 0);

    DQCurrent_s getFOCCurrents(float angle_el);

    ABCurrent_s getABCurrents(PhaseCurrent_s current);

    DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el);

    virtual void enable();

    virtual void disable();
};

#endif

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

void _delay(unsigned long ms);

unsigned long _micros();

#endif

#define DEF_POWER_SUPPLY 12.0f

#define DEF_PID_VEL_P 0.5f

#define DEF_PID_VEL_I 10.0f

#define DEF_PID_VEL_D 0.0f

#define DEF_PID_VEL_RAMP 1000.0f

#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__)

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

#ifndef PID_H

#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator()(float error);
    void reset();

    float P;
    float I;
    float D;
    float output_ramp;
    float limit;

protected:
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
};

#endif

#ifndef LOWPASS_FILTER_H

#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif

#define _MON_TARGET 0b1000000

#define _MON_VOLT_Q 0b0100000

#define _MON_VOLT_D 0b0010000

#define _MON_CURR_Q 0b0001000

#define _MON_CURR_D 0b0000100

#define _MON_VEL 0b0000010

#define _MON_ANGLE 0b0000001

enum MotionControlType : uint8_t
{
    torque = 0x00,
    velocity = 0x01,
    angle = 0x02,
    velocity_openloop = 0x03,
    angle_openloop = 0x04
};

enum TorqueControlType : uint8_t
{
    voltage = 0x00,
    dc_current = 0x01,
    foc_current = 0x02,
};

enum FOCModulationType : uint8_t
{
    SinePWM = 0x00,
    SpaceVectorPWM = 0x01,
    Trapezoid_120 = 0x02,
    Trapezoid_150 = 0x03,
};

enum FOCMotorStatus : uint8_t
{
    motor_uninitialized = 0x00,
    motor_initializing = 0x01,
    motor_uncalibrated = 0x02,
    motor_calibrating = 0x03,
    motor_ready = 0x04,
    motor_error = 0x08,
    motor_calib_failed = 0x0E,
    motor_init_failed = 0x0F,
};

class FOCMotor
{
public:
    FOCMotor();

    virtual void init() = 0;

    virtual void disable() = 0;

    virtual void enable() = 0;

    void linkSensor(Sensor *sensor);

    void linkCurrentSense(CurrentSense *current_sense);

    virtual int initFOC() = 0;

    virtual void loopFOC() = 0;

    virtual void move(float target = NOT_SET) = 0;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;

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
    float Ualpha, Ubeta;

    float voltage_sensor_align;
    float velocity_index_search;

    float phase_resistance;
    int pole_pairs;
    float KV_rating;
    float phase_inductance;

    float voltage_limit;
    float current_limit;
    float velocity_limit;

    int8_t enabled = 0;
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized;

    FOCModulationType foc_modulation;
    int8_t modulation_centered = 1;

    TorqueControlType torque_controller;
    MotionControlType controller;

    PIDController PID_current_q{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    PIDController PID_current_d{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};
    PIDController PID_velocity{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT};
    PIDController P_angle{DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM};
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
    unsigned int monitor_decimals = 4;

    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

    Sensor *sensor;

    CurrentSense *current_sense;

    Print *monitor_port;

private:
    unsigned int monitor_cnt = 0;
};

#endif

#ifndef PID_H

#define PID_H

#ifndef TIME_UTILS_H

#define TIME_UTILS_H

#ifndef FOCUTILS_LIB_H

#define FOCUTILS_LIB_H

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))

#ifndef _round

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))

#endif

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define _sqrt(a) (_sqrtApprox(a))

#define _isset(a) ((a) != (NOT_SET))

#define _UNUSED(v) (void)(v)

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

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

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

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif

void _delay(unsigned long ms);

unsigned long _micros();

#endif

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator()(float error);
    void reset();

    float P;
    float I;
    float D;
    float output_ramp;
    float limit;

protected:
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
};

#endif

#ifndef LOWPASS_FILTER_H

#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator()(float x);
    float Tf;

protected:
    unsigned long timestamp_prev;
    float y_prev;
};

#endif

#ifndef COMMANDS_h

#define COMMANDS_h

#define CMD_C_D_PID 'D'
#define CMD_C_Q_PID 'Q'
#define CMD_V_PID 'V'
#define CMD_A_PID 'A'
#define CMD_STATUS 'E'
#define CMD_LIMITS 'L'
#define CMD_MOTION_TYPE 'C'
#define CMD_TORQUE_TYPE 'T'
#define CMD_SENSOR 'S'
#define CMD_MONITOR 'M'
#define CMD_RESIST 'R'
#define CMD_INDUCTANCE 'I'
#define CMD_KV_RATING 'K'
#define CMD_PWMMOD 'W'

#define CMD_SCAN '?'
#define CMD_VERBOSE '@'
#define CMD_DECIMAL '#'

#define SCMD_PID_P 'P'
#define SCMD_PID_I 'I'
#define SCMD_PID_D 'D'
#define SCMD_PID_RAMP 'R'
#define SCMD_PID_LIM 'L'
#define SCMD_LPF_TF 'F'

#define SCMD_LIM_CURR 'C'
#define SCMD_LIM_VOLT 'U'
#define SCMD_LIM_VEL 'V'

#define SCMD_SENS_MECH_OFFSET 'M'
#define SCMD_SENS_ELEC_OFFSET 'E'

#define SCMD_DOWNSAMPLE 'D'
#define SCMD_CLEAR 'C'
#define SCMD_GET 'G'
#define SCMD_SET 'S'

#define SCMD_PWMMOD_TYPE 'T'
#define SCMD_PWMMOD_CENTER 'C'

#endif

#define MAX_COMMAND_LENGTH 20

enum VerboseMode : uint8_t
{
    nothing = 0x00,
    on_request = 0x01,
    user_friendly = 0x02,
    machine_readable = 0x03
};

typedef void (*CommandCallback)(char *);

class Commander
{
public:
    Commander(Stream &serial, char eol = '\n', bool echo = false);
    Commander(char eol = '\n', bool echo = false);

    void run();

    void run(Stream &reader, char eol = '\n');

    void run(char *user_input);

    void add(char id, CommandCallback onCommand, const char *label = nullptr);

    VerboseMode verbose = VerboseMode::user_friendly;
    uint8_t decimal_places = 3;

    Stream *com_port = nullptr;
    char eol = '\n';
    bool echo = false;

    void motor(FOCMotor *motor, char *user_cmd);

    void lpf(LowPassFilter *lpf, char *user_cmd);

    void pid(PIDController *pid, char *user_cmd);

    void scalar(float *value, char *user_cmd);

    void target(FOCMotor *motor, char *user_cmd, char *separator = (char *)" ");

    void motion(FOCMotor *motor, char *user_cmd, char *separator = (char *)" ");

    bool isSentinel(char ch);

private:
    CommandCallback call_list[20];
    char call_ids[20];
    char *call_label[20];
    int call_count = 0;

    char received_chars[MAX_COMMAND_LENGTH] = {0};
    int rec_cnt = 0;

    void printVerbose(const char *message);

    void printVerbose(const __FlashStringHelper *message);

    void print(const float number);
    void print(const int number);
    void print(const char *message);
    void print(const __FlashStringHelper *message);
    void print(const char message);
    void println(const float number);
    void println(const int number);
    void println(const char *message);
    void println(const __FlashStringHelper *message);
    void println(const char message);

    void printMachineReadable(const float number);
    void printMachineReadable(const int number);
    void printMachineReadable(const char *message);
    void printMachineReadable(const __FlashStringHelper *message);
    void printMachineReadable(const char message);

    void printlnMachineReadable(const float number);
    void printlnMachineReadable(const int number);
    void printlnMachineReadable(const char *message);
    void printlnMachineReadable(const __FlashStringHelper *message);
    void printlnMachineReadable(const char message);

    void printError();
};

#endif

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

    void attach(float *variable);

    int pin_step;
    int pin_dir;
    long count;
    decltype(RISING) polarity = RISING;

private:
    float *attached_variable = nullptr;
    float counter_to_value;
};

#endif

#ifndef __SIMPLEFOCDEBUG_H__

#define __SIMPLEFOCDEBUG_H__

#ifndef SIMPLEFOC_DISABLE_DEBUG

class SimpleFOCDebug
{
public:
    static void enable(Print *debugPrint = &Serial);

    static void println(const __FlashStringHelper *msg);
    static void println(const char *msg);
    static void println(const __FlashStringHelper *msg, float val);
    static void println(const char *msg, float val);
    static void println(const __FlashStringHelper *msg, int val);
    static void println(const char *msg, int val);
    static void println(const char *msg, char val);
    static void println();
    static void println(int val);
    static void println(float val);

    static void print(const char *msg);
    static void print(const __FlashStringHelper *msg);
    static void print(int val);
    static void print(float val);

protected:
    static Print *_debugPrint;
};

#define SIMPLEFOC_DEBUG(msg, ...) \
    SimpleFOCDebug::println(F(msg), ##__VA_ARGS__)

#else

#define SIMPLEFOC_DEBUG(msg, ...)

#endif

#endif

#endif
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_8, PA_9, PA_10, PA_7);

float target_angle = 0;
float target_velocity = 0;
Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&target_velocity, cmd); }

void setup()
{
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

void loop()
{
    motor.loopFOC();

    motor.move(target_velocity);

    command.run();
}
