#define WHEELBASE_RADIUS 0.0385 // Distance between the wheels, divided by 2 (in meters)
#define WHEELBASE_DIAMETER (WHEELBASE_RADIUS * 2.0) // Distance between the wheels (in meters)
#define WHEEL_DIAMETER_METERS 0.032
#define ENCODER_TICKS_PER_REVOLUTION (298.0 * 28.0)

// Current state of the robot measured by odometry
// From the robot's point of view, x points forward
//                                 y points left
//                                 theta (th) starts along the x axis and sweeps counterclockwise
float currentX;
float currentY;
float currentTh;

// Target state of the robot, used in feedback control
float targetX;
float targetY;
float targetTh;

void update_odom(long left_enc_val, long right_enc_val, long reading_micros) {
  float dl = encoder_ticks_to_m(left_enc_val - last_left_enc_val);
  float dr = encoder_ticks_to_m(right_enc_val - last_right_enc_val);
  float ds = (dl + dr) / 2.0;
  float dth = (dr - dl) / WHEELBASE_DIAMETER;
  
  float thetaNext = currentTh + dth / 2;
  float xNext = currentX + ds * cos(thetaNext);
  float yNext = currentY + ds * sin(thetaNext);
  thetaNext = currentTh + dth;

  currentX = xNext;
  currentY = yNext;
  currentTh = normalizeAngle(thetaNext);
}

inline float encoder_ticks_to_m(float encoder_ticks) {
  return encoder_ticks / ENCODER_TICKS_PER_REVOLUTION * WHEEL_DIAMETER_METERS * PI;
}
