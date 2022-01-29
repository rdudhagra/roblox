bool printed_legend = false;

inline float getTime() {
  return micros() / 1000000.0;
}

inline void pause(float seconds) {
  delay(seconds * 1000.0);
}

void print_pos() {
  if (!printed_legend) {
    Serial.println("Odom_X(cm), Odom_Y(cm), Odom_Th(rad), FF_X(cm), FF_Y(cm), FF_Th(rad)");
    printed_legend = true;
  }

  Serial.print(currentX * 100, 4);
  Serial.print(" ");
  Serial.print(currentY * 100, 4);
  Serial.print(" ");
  Serial.print(currentTh, 4);
  Serial.print(" ");
  Serial.print(ffX * 100, 4);
  Serial.print(" ");
  Serial.print(ffY * 100, 4);
  Serial.print(" ");
  Serial.println(ffTh, 4);
}

void print_err() {
  if (!printed_legend) {
    Serial.println("Err_X(mm), Err_Y(mm), Err_Th(0.001_rad)");
    printed_legend = true;
  }
  float errX, errY, errTh;
  calcError(ffX, ffY, ffTh, currentX, currentY, currentTh, &errX, &errY, &errTh);

  Serial.print(errX * 1000, 4);
  Serial.print(" ");
  Serial.print(errY * 1000, 4);
  Serial.print(" ");
  Serial.println(errTh * 1000, 4);
}

inline float normalizeAngle(float th) {
  return atan2(sin(th), cos(th));
}
