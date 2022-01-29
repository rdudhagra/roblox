void calcLinearTrajectory(float dist, float t, float *V, float*w) {
  // dist: total distance to travel
  // t: current time
  // total_time: total time for trajectory
  // *V: pointer to calculated velocity to travel
  // *w: pointer to calculated angular velocity to travel
  *w = 0;
  *V = trapezoidalVelocityProfile(t, dist);
  return;
}
