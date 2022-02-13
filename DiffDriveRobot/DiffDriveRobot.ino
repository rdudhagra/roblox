#define DEBUG false
#define ROBOT_ID 1

void setup() {
  Serial.begin(115200); // Debug
  Serial5.begin(115200); // XBEE
  motion_setup();
  gripper_setup();
}

bool cmdReady = false;
float cmdX;
float cmdY;
float cmdTh;

void loop() {
  if (cmdReady) {
    cmdReady = false;
    goToPose(cmdX, cmdY, cmdTh);
    print_done();
  }
}
