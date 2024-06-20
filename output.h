    FOCMotor();
    void linkSensor(Sensor* sensor);
    void linkCurrentSense(CurrentSense* current_sense);
    float shaftAngle();
    float shaftVelocity();
    float electricalAngle();
    void useMonitoring(Print &serial);
    void monitor();
        virtual float getMechanicalAngle();
        virtual float getAngle();
        virtual double getPreciseAngle();
        virtual float getVelocity();
        virtual int32_t getFullRotations();
        virtual void update();
        virtual int needsSearch();
        virtual void init();
float _sin(float a);
float _cos(float a);
void _sincos(float a, float* s, float* c);
float _atan2(float y, float x);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _sqrtApprox(float value);
void _delay(unsigned long ms);
unsigned long _micros();
    BLDCMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
    virtual void linkDriver(BLDCDriver* driver);
    int alignSensor();
    int alignCurrentSense();
    int absoluteZeroSearch();
    float velocityOpenloop(float target_velocity);
    float angleOpenloop(float target_angle);
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
    StepperMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
    void linkDriver(StepperDriver* driver);
    int alignSensor();
    int absoluteZeroSearch();
    float velocityOpenloop(float target_velocity);
    float angleOpenloop(float target_angle);
    MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);
    MagneticSensorI2C(MagneticSensorI2CConfig_s config);
    static MagneticSensorI2C AS5600();
    void init(TwoWire* _wire = &Wire);
    int checkBus(byte sda_pin , byte scl_pin );
    int read(uint8_t angle_register_msb);
    int getRawCount();
    GenericSensor(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr);
    BLDCDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
    InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET);
    void calibrateOffsets();
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
    LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);
    void calibrateOffsets();
    GenericCurrentSense(PhaseCurrent_s (*readCallback)() = nullptr, void (*initCallback)() = nullptr);
    void calibrateOffsets();
typedef void (* CommandCallback)(char*); 
    Commander(Stream &serial, char eol = '\n', bool echo = false);
    Commander(char eol = '\n', bool echo = false);
    void run();
    void run(Stream &reader, char eol = '\n');
    void run(char* user_input);
    void add(char id , CommandCallback onCommand, const char* label = nullptr);
    void motor(FOCMotor* motor, char* user_cmd);
    void lpf(LowPassFilter* lpf, char* user_cmd);
    void pid(PIDController* pid, char* user_cmd);
    void scalar(float* value, char* user_cmd);
    void target(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");
    void motion(FOCMotor* motor, char* user_cmd, char* separator = (char *)" ");
    bool isSentinel(char ch);
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
    StepDirListener(int pinStep, int pinDir, float counter_to_value = 1);
    void enableInterrupt(void (*handleStep)());
    void init();
    void handle();
    float getValue();
    void attach(float* variable);
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
    getSensorAngle(); 
    delayMicroseconds(1);
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    return sign*_sqrt(ABcurrent.alpha*ABcurrent.alpha + ABcurrent.beta*ABcurrent.beta);
    _sincos(angle_el, &st, &ct);

  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));


  return sensor_direction*LPF_velocity(sensor->getVelocity());
  return  _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
  SIMPLEFOC_DEBUG("MOT: Monitor enabled!");




  SIMPLEFOC_DEBUG("MOT: Init");
  _delay(500);
  SIMPLEFOC_DEBUG("MOT: Enable driver.");
  enable();
  _delay(500);
  if(current_sense) current_sense->disable();
  if(current_sense) current_sense->enable();
    SIMPLEFOC_DEBUG("MOT: No sensor.");
  SIMPLEFOC_DEBUG("MOT: Align current sense.");
  SIMPLEFOC_DEBUG("MOT: Align sensor.");
  if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
    _delay(200);
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  SIMPLEFOC_DEBUG("MOT: Index search...");
  setPhaseVoltage(0, 0, 0);
  if (sensor) sensor->update();
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
      _sincos(angle_el, &_sa, &_ca);
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
  setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));







  SIMPLEFOC_DEBUG("MOT: Init");
  _delay(500);
  SIMPLEFOC_DEBUG("MOT: Enable driver.");
  enable();
  _delay(500);
  _delay(500);
  SIMPLEFOC_DEBUG("MOT: Align sensor.");
    setPhaseVoltage(0, 0, 0);
    _delay(200);
    SIMPLEFOC_DEBUG("MOT: Skip dir calib.");
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  SIMPLEFOC_DEBUG("MOT: Index search...");
  setPhaseVoltage(0, 0, 0);
  if (sensor) sensor->update();
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
  _sincos(angle_el, &_sa, &_ca);
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
  setPhaseVoltage(Uq,  0, _electricalAngle((shaft_angle), pole_pairs));



	return (int)MagneticSensorI2C::read(angle_register_msb);
  pinMode(scl_pin, INPUT_PULLUP);
  pinMode(sda_pin, INPUT_PULLUP);
  delay(250);
    pinMode(sda_pin, INPUT);
    delayMicroseconds(20);
    _delay(1000);
  pinMode(sda_pin, INPUT);
  pinMode(scl_pin, INPUT);



  if(initCallback != nullptr) this->initCallback();


    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
    setPwm(0,0,0);
  setPwm(0, 0, 0);
  if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);




    calibrateOffsets();





    _driverSyncLowSide(driver->params, params);
    calibrateOffsets();
    _startADC3PinConversionLowSide();




    if(initCallback != nullptr) initCallback();
    calibrateOffsets();



  run(*com_port, eol);
      if(!isSentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
      printVerbose(F("Verb:"));
      printMachineReadable(CMD_VERBOSE);
      if(!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
      printVerbose(F("Decimal:"));
      printMachineReadable(CMD_DECIMAL);
      println(decimal_places);
  printMachineReadable(cmd);
          println(motor->voltage_limit);
          printVerbose(F("curr: "));
          println(motor->current_limit);
          printVerbose(F("vel: "));
          println(motor->velocity_limit);
          printError();
      motion(motor, &user_command[0]);
       printVerbose(F("PWM Mod | "));
          printVerbose(F("center: "));
          println(motor->modulation_centered);
          printError();
      printVerbose(F("R phase: "));
      if(_isset(motor->phase_resistance)) println(motor->phase_resistance);
      else println(0);
      printVerbose(F("L phase: "));
      if(_isset(motor->phase_inductance)) println(motor->phase_inductance);
      else println(0);
      printVerbose(F("Motor KV: "));
      if(_isset(motor->KV_rating)) println(motor->KV_rating);
      else println(0);
       printVerbose(F("Sensor | "));
      printVerbose(F("Monitor | "));
          printVerbose(F("downsample: "));
          println((int)motor->monitor_downsample);
          println(F("clear"));
          printVerbose(F("decimal: "));
          println((int)motor->monitor_decimals);
          println("");
          printError();
      printVerbose(F("unknown cmd "));
      printError();
      printVerbose(F("Torque: "));
      printVerbose(F("Status: "));
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       println(motor->enabled);
      target(motor,  user_cmd, separator);
  if(!GET) *value = atof(user_cmd);
  println(*value);
  printVerbose(F("Target: "));
  println(motor->target);
      printVerbose(F("Warn: \\r detected! \n"));
  if(verbose == VerboseMode::user_friendly) print(message);
  if(verbose == VerboseMode::user_friendly) print(message);
  if(verbose == VerboseMode::machine_readable) print(number);
  if(verbose == VerboseMode::machine_readable) print(number);
  if(verbose == VerboseMode::machine_readable) print(message);
  if(verbose == VerboseMode::machine_readable) print(message);
  if(verbose == VerboseMode::machine_readable) print(message);
  if(verbose == VerboseMode::machine_readable) println(number);
  if(verbose == VerboseMode::machine_readable) println(number);
  if(verbose == VerboseMode::machine_readable) println(message);
  if(verbose == VerboseMode::machine_readable) println(message);
  if(verbose == VerboseMode::machine_readable) println(message);
 println(F("err"));

    pinMode(pin_dir, INPUT);
    pinMode(pin_step, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_step), doA, polarity);
   if(attached_variable) *attached_variable = getValue();


