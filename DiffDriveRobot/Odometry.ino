#define WHEELBASE_RADIUS 0.0385 // Distance between the wheels, divided by 2 (in meters)
#define WHEELBASE_DIAMETER (WHEELBASE_RADIUS * 2.0) // Distance between the wheels (in meters)
#define WHEEL_DIAMETER_METERS 0.032
#define ENCODER_TICKS_PER_REVOLUTION (298.0 * 28.0)

#define REFINE_POSE_RATIO 0.1 // refined_pose = new_pose * REFINE_POSE_RATIO + old_pose * (1 - REFINE_POSE_RATIO)

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

void refinePose(float x, float y, float th) {
  currentX = x * REFINE_POSE_RATIO + currentX * (1 - REFINE_POSE_RATIO);
  currentY = y * REFINE_POSE_RATIO + currentY * (1 - REFINE_POSE_RATIO);
  currentTh = th * REFINE_POSE_RATIO + currentTh * (1 - REFINE_POSE_RATIO);
}

void overwritePose(float x, float y, float th) {
  currentX = x;
  currentY = y;
  currentTh = th;
  resetFF(x, y, th);
}
