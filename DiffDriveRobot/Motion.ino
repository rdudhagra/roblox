#include <L298N.h>
#include <L298NX2.h>
#include <Encoder.h>
#include <ArduPID.h>

#define MOTOR_FORWARD L298N::FORWARD
#define MOTOR_BACKWARD L298N::BACKWARD
#define MOTOR_STOP L298N::STOP
#define MOTOR_ODOM_FREQ 5000 // Number of microseconds between each PID loop
#define MOTOR_KP 2500
#define MOTOR_KI 0
#define MOTOR_KD 1000
#define MOTOR_LEFT_MIN_PWR 85
#define MOTOR_RIGHT_MIN_PWR 85

extern float currentX;
extern float currentY;
extern float currentTh;

Encoder leftEnc(14, 15);
Encoder rightEnc(10, 11);

L298NX2 motors(3, 17, 18, 2, 8, 9);

IntervalTimer motor_speed_pid_timer;
ArduPID leftMotorPID;
ArduPID rightMotorPID;

long last_left_enc_val;
long last_right_enc_val;
long last_reading_micros;

volatile double current_left_speed;
volatile double current_right_speed;
double current_left_pwr;
double current_right_pwr;
volatile double target_left_speed;
volatile double target_right_speed;
volatile double pid_out_left;
volatile double pid_out_right;

void motion_setup() {
  leftMotorPID.begin(&current_left_speed, &pid_out_left, &target_left_speed, MOTOR_KP, MOTOR_KI, MOTOR_KD);
  rightMotorPID.begin(&current_right_speed, &pid_out_right, &target_right_speed, MOTOR_KP, MOTOR_KI, MOTOR_KD);
  leftMotorPID.setOutputLimits(-255, 255);
  rightMotorPID.setOutputLimits(-255, 255);
  leftMotorPID.setWindUpLimits(-255, 255);
  rightMotorPID.setWindUpLimits(-255, 255);
  leftMotorPID.start();
  rightMotorPID.start();

  motor_speed_pid_timer.begin(encoder_fbk, MOTOR_ODOM_FREQ);
  last_left_enc_val = leftEnc.read();
  last_right_enc_val = rightEnc.read();
  last_reading_micros = micros();
}

void sendVelocity(float left, float right) {
  // left and right are floating point wheel speeds, measured in m/s
  noInterrupts();
  target_left_speed = left;
  target_right_speed = right;
  interrupts();
}

void stop() {
  sendVelocity(0,0);
}

void _cmd_vel(double left_pwr, double right_pwr) {
  motors.setSpeedA(map(abs(left_pwr), 0, 255, MOTOR_LEFT_MIN_PWR, 255));
  motors.setSpeedB(map(abs(right_pwr), 0, 255, MOTOR_RIGHT_MIN_PWR, 255));
  motors.runA(left_pwr > 0 ? MOTOR_FORWARD : (left_pwr < 0 ? MOTOR_BACKWARD : MOTOR_STOP));
  motors.runB(right_pwr > 0 ? MOTOR_FORWARD : (right_pwr < 0 ? MOTOR_BACKWARD : MOTOR_STOP));
}

void encoder_fbk() {
  // Read new data
  long left_enc_val = leftEnc.read();
  long right_enc_val = rightEnc.read();
  long reading_micros = micros();

  // Do encoder stuff
  motor_pid(left_enc_val, right_enc_val, reading_micros);
  update_odom(left_enc_val, right_enc_val, reading_micros);

  // Update "last data" variables with new data
  last_left_enc_val = left_enc_val;
  last_right_enc_val = right_enc_val;
  last_reading_micros = reading_micros;

  //  Serial.print(target_left_speed * 100, 4);
  //  Serial.print(" ");
  //  Serial.print(target_right_speed * 100, 4);
  //  Serial.print(" ");
  //  Serial.print(current_left_speed * 100, 4);
  //  Serial.print(" ");
  //  Serial.println(current_right_speed * 100, 4);
}

void motor_pid(long left_enc_val, long right_enc_val, long reading_micros) {
  float left_speed = 1000000.0 * ((float)(left_enc_val - last_left_enc_val)) / (float)((reading_micros - last_reading_micros));
  float right_speed = 1000000.0 * ((float)(right_enc_val - last_right_enc_val)) / (float)((reading_micros - last_reading_micros));

  current_left_speed = encoder_ticks_to_m(left_speed);
  current_right_speed = encoder_ticks_to_m(right_speed);

  leftMotorPID.compute();
  rightMotorPID.compute();
  current_left_pwr = constrain(current_left_pwr + pid_out_left, -255, 255);
  current_right_pwr = constrain(current_right_pwr + pid_out_right, -255, 255);
  _cmd_vel(current_left_pwr, current_right_pwr);
}
