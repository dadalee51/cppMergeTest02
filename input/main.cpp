#include <SimpleFOC.h>
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_8, PA_9, PA_10, PA_7);
// angle set point variable
float target_angle = 0;
float target_velocity = 0;
Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 8;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.controller = MotionControlType::angle;
  motor.controller = MotionControlType::velocity;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 1.6f;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 6;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f;
  // angle P controller
  // motor.P_angle.P = 20;
  // motor.velocity_limit = 20;
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
  // command.add('T', doTarget, "target angle");
  command.add('T', doTarget, "target velocity");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  // motor.move(target_angle);
  motor.move(target_velocity);
  // motor.monitor();
  command.run();
}
