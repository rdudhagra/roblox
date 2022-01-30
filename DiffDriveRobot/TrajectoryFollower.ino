#define TIME_PAUSE 0.1

float trapezoidalVelocityProfile(float t, float dist, float vmax, float amax) {
  // t: the current time
  // dist: distance to travel
  // backwards: do trajectory in reverse
  // [return] the velocity to command
  float tramp = vmax / amax;
  float tf = (abs(dist) + vmax * vmax / amax) / vmax;

  if (tramp > tf / 2.0) {
    // Switch to triangular velocity profile
    tf = sqrt(4.0 * abs(dist) / amax);
    tramp = tf / 2.0;
  }

  float v;

  t -= TIME_PAUSE;
  if (t < 0)
    v = 0;
  else if (t < tramp)
    v = amax * t;
  else if (t < tf - tramp)
    v = vmax;
  else if (t < tf)
    v = amax * (tf - t);
  else
    v = 0;

  if (dist < 0)
    v *= -1;

  return v;
}

float calcTrapVelTrajectoryTime(float dist, float vmax, float amax) {
  return (abs(dist) + vmax * vmax / amax) / vmax + TIME_PAUSE * 2;
}

void cmd_diff_drive_kinematics(float V, float w) {
  sendVelocity(V - WHEELBASE_RADIUS * w, V + WHEELBASE_RADIUS * w);
}
