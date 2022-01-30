#define SERIAL_BUF_SIZE 256

// What should the robot do?
#define COMMAND_DRIVE 'D' // Drive
#define COMMAND_OVERWRITE 'W' // Overwrite the current pose of the robot
#define COMMAND_UPDATE 'U' // Refine the current pose of the robot using an external ground-truth (do a weighted average of current pose and the new one)
#define COMMAND_PICK 'P' // Pick up object using gripper
#define COMMAND_PLACE 'L' // Place object using gripper

// Example command:
// R1;D;X0.1;Y0.25;T1.570796\n (Tell Robot 1 to drive to pos 0.1m, 0.25m, and end at an angle of pi/2 radians (global coordinates, not relative to prev position)
// R0;P\n (Tell Robot 0 to pick up object)

char buf[SERIAL_BUF_SIZE];
uint32_t pos;

void serialEvent5() {
  if (Serial5.available() > 0) {
    char incomingByte = Serial5.read();
    buf[pos] = incomingByte;

    pos++;
    if (pos >= SERIAL_BUF_SIZE) pos = 0;

    if (incomingByte == '\n') {
      buf[pos] = '\0'; // Null-terminate buffer
      processNewCommand(buf);
      pos = 0;
    }
  }
}

void processNewCommand(char *cmd_str) {
  int robot_num;
  char cmd;
  float x;
  float y;
  float th;

  int num_args_parsed = sscanf(cmd_str, "R%d;%c;X%f;Y%f;T%f\n", &robot_num, &cmd, &x, &y, &th);
  if (num_args_parsed == 5) {
    // Parsed command successfully!
    if (robot_num == ROBOT_ID) {
      if (DEBUG) {
        Serial.print("New command received: ");
        Serial.println(cmd_str);
      }
      switch (cmd) {
        case COMMAND_DRIVE:
          cmdX = x;
          cmdY = y;
          cmdTh = th;
          cmdReady = true;
          break;
        case COMMAND_UPDATE:
          refinePose(x, y, th);
          break;
        case COMMAND_OVERWRITE:
          overwritePose(x, y, th);
          break;
      }
    }
  } else if(num_args_parsed == 2) {
    // Also parsed command successfully!
    if (robot_num == ROBOT_ID) {
      if (DEBUG) {
        Serial.print("New command received: ");
        Serial.println(cmd_str);
      }
      switch (cmd) {
        case COMMAND_PICK:
          pick();
          print_done();
          break;
        case COMMAND_PLACE:
          drop();
          print_done();
          break;
      }
    }
  }
}

void print_done() {
  Serial5.println("R" + String(ROBOT_ID) + ";DONE");
}
