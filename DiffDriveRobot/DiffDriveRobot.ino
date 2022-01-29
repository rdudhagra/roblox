#define DEBUG true

void setup() {
  Serial.begin(115200);
  motion_setup();
  gripper_setup();
  pause(1);
  pick();
  moveRelDist(0.1);
  drop();
}

float dist = 0.2;

void loop() {
  //  moveRelDist(dist);
  //  moveRelDist(-dist);
}
