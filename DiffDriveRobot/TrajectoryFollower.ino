#define TIME_PAUSE 0.5
#define AMAX 0.05
#define VMAX 0.05

float trapezoidalVelocityProfile(float t, float dist) {
  // t: the current time
  // dist: distance to travel
  // backwards: do trajectory in reverse
  // [return] the velocity to command
  float tramp = VMAX / AMAX;
  float tf = (abs(dist) + VMAX * VMAX / AMAX) / VMAX;

  float v;

  t -= TIME_PAUSE;
  if (t < 0)
    v = 0;
  else if (t < tramp)
    v = AMAX * t;
  else if (t < tf - tramp)
    v = VMAX;
  else if (t < tf)
    v = AMAX * (tf - t);
  else
    v = 0;

  if (dist < 0)
    v *= -1;

  return v;
}

float calcTrapVelTrajectoryTime(float dist) {
  return (abs(dist) + VMAX * VMAX / AMAX) / VMAX + TIME_PAUSE * 2;
}

void cmd_diff_drive_kinematics(float V, float w) {
  sendVelocity(V - WHEELBASE_RADIUS * w, V + WHEELBASE_RADIUS * w);
}
