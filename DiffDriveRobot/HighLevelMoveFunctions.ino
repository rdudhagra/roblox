#define AMAX 0.05
#define VMAX 0.05
#define ALPHAMAX 0.2 // alpha = angular acceleration
#define WMAX 0.5

void moveRelDist(float dist) {
  // Don't do anything if dist is 0 (or very close to)
  if (abs(dist) < 0.001) return;

  float V;
  float w;

  float t = getTime();
  float startTime = t;
  float tf = calcTrapVelTrajectoryTime(dist, VMAX, AMAX);

  while (t - startTime < tf) {
    t = getTime();

    calcLinearTrajectory(dist, t - startTime, &V, &w);
    ffTick(V, w);
    add_feedback(&V, &w);
    cmd_diff_drive_kinematics(V, w);

    if (DEBUG)
      //      print_pos();
      print_err();

    pause(0.05);
  }
  stop();
}

void turnRelAngle(float angle) {
  // Don't do anything if angle is 0 (or very close to)
  if (abs(angle) < 0.02) return;

  float V;
  float w;

  float t = getTime();
  float startTime = t;
  float tf = calcTrapVelTrajectoryTime(angle, WMAX, ALPHAMAX);

  while (t - startTime < tf) {
    t = getTime();

    calcLinearAngleTrajectory(angle, t - startTime, &V, &w);
    ffTick(V, w);
    add_feedback(&V, &w);
    cmd_diff_drive_kinematics(V, w);

    if (DEBUG)
      //      print_pos();
      print_err();

    pause(0.05);
  }
  stop();
}

void goToPose(float x, float y, float th) {
  // Turn towards goal
  bool goBackwards = false;

  float angleToTurn = normalizeAngle(atan2(y - currentY, x - currentX) - currentTh);
  if (abs(angleToTurn) > PI / 2) goBackwards = true;

  turnRelAngle(goBackwards ? normalizeAngle(angleToTurn + PI) : angleToTurn);

  // Move in a straight line to goal
  moveRelDist((goBackwards ? -1 : 1) * sqrt(sq(x - currentX) + sq(y - currentY)));

  // Turn to final desired angle
  turnRelAngle(normalizeAngle(th - currentTh));

  resetFF(x, y, th);
}
