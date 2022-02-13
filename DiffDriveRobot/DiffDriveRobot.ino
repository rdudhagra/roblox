#define DEBUG false
#define ROBOT_ID 0

void setup() {
  Serial.begin(115200); // Debug
  Serial5.begin(115200); // XBEE
  motion_setup();
  gripper_setup();
}

volatile bool cmdReady = false;
volatile float cmdX;
volatile float cmdY;
volatile float cmdTh;
volatile bool cmdBackwards;

volatile bool doPick = false;
volatile bool doPlace = false;

void loop() {
  if (cmdReady) {
    cmdReady = false;
    goToPose(cmdX, cmdY, cmdTh, cmdBackwards);
    Serial5.println("fooo1");
    print_done();
  } else if (doPick) {
    Serial5.println("fooo2.1");
    doPick = false;
    Serial5.println("fooo2.2");
    pick();
    Serial5.println("fooo2.3");
    print_done();
    Serial5.println("fooo2.4");
  } else if (doPlace) {
    Serial5.println("fooo3.1");
    doPlace = false;
    Serial5.println("fooo3.2");
    drop();
    Serial5.println("fooo3.3");
    print_done();
    Serial5.println("fooo3.4");
  }
}
