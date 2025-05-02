#include <SimpleFOC.h>

// Motor0 specs
#define M0_pole_pairs 7
#define M0_phase_resistance 10
#define M0_KV 50
#define M0_motor_current_limit 0.5
#define M0_motor_max_velocity 60

// Motor1 specs
#define M1_pole_pairs 7
#define M1_phase_resistance 10
#define M1_KV 50
#define M1_motor_current_limit 0.5
#define M1_motor_max_velocity 60

// Power Supply Specs
#define power_supply_v 24

// Pins
#define MISO 19
#define SCLK 18
#define MOSI 23
#define SS0 5
#define SS1 22

#define M0_phA 32
#define M0_phB 33
#define M0_phC 25

#define M1_phA 26
#define M1_phB 27
#define M1_phC 14

#define M_En 12

//simpleFOC motor monitoring variables bitmap
//current [A], voltage [V], velocity [rad/s], or position [rad]
#define   _MON_TARGET 0b1000000  // monitor target value
#define   _MON_VOLT_Q 0b0100000  // monitor voltage q value
#define   _MON_VOLT_D 0b0010000  // monitor voltage d value
#define   _MON_CURR_Q 0b0001000  // monitor current q value - ( if current sense available ) or estimated current if no current sense but provided phase resistance
#define   _MON_CURR_D 0b0000100  // monitor current d value - ( if current sense available )
#define   _MON_VEL    0b0000010  // monitor velocity value
#define   _MON_ANGLE  0b0000001  // monitor angle value

// Constants
const float target_position0 = 180.0;  // Motor 0 target position
const float reset_position0 = 0.0;    // Motor 0 reset position
const float target_position1 = 180.0; // Motor 1 target position
const float reset_position1 = 0.0;    // Motor 1 reset position
const int   direction0 = -11;           // Motor 0 direction of rotation: 1 CW, -1 CCW
const int   direction1 = 1;          // Motor 1 direction of rotation: 1 CW, -1 CCW
float full_speed_velocity0 = 1000.0;    // Motor 0 full speed velocity
float full_speed_velocity1 = 1000.0;   // Motor 1 full speed velocity
bool coasting0 = false;               // Motor 0 state: running or coasting
bool coasting1 = false;               // Motor 1 state: running or coasting

// Sensors and I2C
MagneticSensorSPI sensor0 = MagneticSensorSPI(AS5048_SPI, SS0);
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, SS1);
SPIClass SPI_0(HSPI);
SPIClass SPI_1(HSPI);

// Motors and Drivers
BLDCMotor motor0 = BLDCMotor(M0_pole_pairs, M0_phase_resistance);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(M0_phA, M0_phB, M0_phC);

BLDCMotor motor1 = BLDCMotor(M1_pole_pairs, M1_phase_resistance);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_phA, M1_phB, M1_phC);

//Commander interface
Commander commander = Commander();

// Sensor offsets
float offset0 = 0.0;
float offset1 = 0.0;

// // Timing for serial output
 unsigned long previousMillis = 0;
 const long printInterval = 1000;  // Print position every second

// Function prototypes
void controlMotor(BLDCMotor &motor, MagneticSensorSPI &sensor, float &offset,
                  float full_speed_velocity, int direction, bool &coasting, float target_position, float reset_position);
float normalizeAngle(float angle, float offset);
void printPositions();

//commander function prototypes
void doTarget(char* cmd);
void doMotor0(char* cmd);
void doMotor1(char* cmd);

void setup() {
  //Initialize SPI
  SPI_0.begin(SCLK, MISO, MOSI, SS0); //SCLK, MISO, MOSI, SS 
  SPI_1.begin(SCLK, MISO, MOSI, SS1); //SCLK, MISO, MOSI, SS 
  sensor0.init(&SPI_0);
  sensor0.init(&SPI_1);

  // Get initial offsets
  offset0 = sensor0.getAngle() * (180.0 / _PI);
  offset1 = sensor1.getAngle() * (180.0 / _PI);

  // Link sensors to motors
  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  // Initialize drivers
  driver0.voltage_power_supply = power_supply_v;
  driver0.init();
  motor0.linkDriver(&driver0);

  driver1.voltage_power_supply = power_supply_v;
  driver1.init();
  motor1.linkDriver(&driver1);

  // Configure motor settings
  motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Set PID controller for velocity control
  motor0.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  // Set PID parameters
  motor0.PID_velocity.P = 0.2;  // Proportional gain
  motor0.PID_velocity.I = 0.1;  // Integral gain
  motor0.PID_velocity.D = 0.01; // Derivative gain
  motor0.PID_velocity.output_ramp = 100; // Acceleration limit [rad/s^2]

  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 0.1;
  motor1.PID_velocity.D = 0.01;
  motor1.PID_velocity.output_ramp = 100;

  motor0.current_limit = M0_motor_current_limit;
  motor1.current_limit = M1_motor_current_limit;

  // use monitoring with the BLDCMotor
  Serial.begin(115200);

  // Initialize Commander interface
  commander.add('T', doTarget, "target velocity");
  commander.add('M', doMotor0, "motor 0");
  commander.add('N', doMotor1, "motor 1");

  //monitoring port
  motor0.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  // monitor data formatting;
  motor0.monitor_start_char = '\0'; //!< monitor starting character
  motor0.monitor_end_char = '\0'; //!< monitor outputs ending character 
  motor0.monitor_separator = '\t'; //!< monitor outputs separation character
  motor1.monitor_start_char = '\0'; //!< monitor starting character
  motor1.monitor_end_char = '\0'; //!< monitor outputs ending character 
  motor1.monitor_separator = '\t'; //!< monitor outputs separation character

  //display variables
  motor0.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE

  // downsampling
  motor0.monitor_downsample = 100; // default 10
  motor1.monitor_downsample = 100; // default 10

  motor0.init();
  motor1.init();
  // motor0.initFOC();
  // motor1.initFOC();

  // Serial.println("Motors ready. Starting sequence...");

  // //get the motors moving first
  // motor0.voltage_limit = power_supply_v;
  // motor1.voltage_limit = power_supply_v;
  // float steps=1000;
  // for (int i; i<=(steps); i++){

  //   motor0.move(full_speed_velocity0*direction0);
  //   motor1.move(full_speed_velocity1*direction1);

  //   motor0.loopFOC();
  //   motor1.loopFOC();
  // }
}

void loop() {
  unsigned long currentMillis = millis();

  // Control motors
  // controlMotor(motor0, sensor0, offset0, full_speed_velocity0, direction0, coasting0, target_position0, reset_position0);
  // controlMotor(motor1, sensor1, offset1, full_speed_velocity1, direction1, coasting1, target_position1, reset_position1);

  if (currentMillis - previousMillis >= printInterval) {
    previousMillis = currentMillis;
    Serial.println("Looping");
    printPositions();
  }

  // Core FOC loop
  // motor0.loopFOC();
  // motor1.loopFOC();

  // Output motor monitor data, comment out when not debugging, will impact performance.
  //motor0.monitor();
  //motor1.monitor();

  //Process Commander interface commands
  //commander.run();
}

void controlMotor(BLDCMotor &motor, MagneticSensorSPI &sensor, float &offset, float full_speed_velocity, int direction, bool &coasting, float target_position, float reset_position) 
{
  float angle = normalizeAngle(sensor.getAngle() * (180.0 / _PI), offset);
  if(direction > 0){
    if (angle < target_position || angle < reset_position) {
      motor.voltage_limit = power_supply_v;
      motor.move(full_speed_velocity*direction);
    } else {
      motor.voltage_limit = 0;
      motor.move(0);
    }
  } else {
    if (angle < target_position || angle < reset_position) {
      motor.voltage_limit = 0;
      motor.move(0);
    } else {
      motor.voltage_limit = power_supply_v;
      motor.move(full_speed_velocity*direction);
    }
  }
}

float normalizeAngle(float angle, float offset) {
  angle -= offset;
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

void printPositions() {
  sensor0.update(); // If the simpleFOC library version is 2.2.0 or above, you need to uncomment this line
  sensor1.update(); // If the simpleFOC library version is 2.2.0 or above, you need to uncomment this line
  float angle0 = normalizeAngle(sensor0.getAngle() * (180.0 / _PI), offset0);
  float angle1 = normalizeAngle(sensor1.getAngle() * (180.0 / _PI), offset1);

  Serial.print("Sensor0 angle: ");
  Serial.print(angle0, 2);  // Print with 2 decimal places
  Serial.print("°, Sensor1 angle: ");
  Serial.print(angle1, 2);  // Print with 2 decimal places
  Serial.println("°");
}

// Commander callback functions
void doTarget(char* cmd) {
  commander.scalar(&motor0.target, cmd);
  commander.scalar(&motor1.target, cmd);
}

void doMotor0(char* cmd){
  commander.motor(&motor0, cmd);
}

void doMotor1(char* cmd){
  commander.motor(&motor1, cmd);
}
