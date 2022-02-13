#include <Servo.h>

// Robot 0
#define CLAW_FULLY_OPEN 180
#define CLAW_DROP 140
#define CLAW_GRIP 90

// Robot 1
//#define CLAW_FULLY_OPEN 120
//#define CLAW_DROP 100
//#define CLAW_GRIP 50

// All robots
#define LIFT_UP 110
#define LIFT_DOWN_FAST 75
#define LIFT_DOWN 80
#define LIFT_STOP 95

Servo lift;
Servo claw;

void gripper_setup() {
  lift.attach(4);
  claw.attach(5);
}

void pick() {
  claw.write(CLAW_FULLY_OPEN);
  lift.write(LIFT_DOWN_FAST);
  pause(0.1);
  lift.write(LIFT_DOWN - 2);
  pause(1.5);
  lift.write(LIFT_STOP);
  claw.write(CLAW_GRIP);
  pause(0.25);
  lift.write(LIFT_UP);
  pause(1.5);
  lift.write(LIFT_STOP);
}

void drop() {
  claw.write(CLAW_GRIP);
  lift.write(LIFT_DOWN_FAST);
  pause(0.1);
  lift.write(LIFT_DOWN);
  pause(1.5);
  lift.write(LIFT_STOP);
  claw.write(CLAW_DROP);
  pause(0.25);
  lift.write(LIFT_UP);
  pause(1.5);
  lift.write(LIFT_STOP);
  claw.write(CLAW_FULLY_OPEN);
}
