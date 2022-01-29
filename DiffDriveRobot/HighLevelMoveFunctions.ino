void moveRelDist(float dist) {
  float V;
  float w;

  float t = getTime();
  float startTime = t;
  float tf = calcTrapVelTrajectoryTime(dist);

  //  resetFF();

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
