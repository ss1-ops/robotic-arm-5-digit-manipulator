// BCN3D MOVEO - Stepper Control via Serial from ROS 2
// Joints: 0=Shoulder, 1=Elbow (Joint_3), 2=Wrist (Joint_5) for now

#define SHOULDER_STEP_PIN  5
#define SHOULDER_DIR_PIN   4
#define ELBOW_STEP_PIN     7
#define ELBOW_DIR_PIN      6
#define WRIST_STEP_PIN     9
#define WRIST_DIR_PIN      8

void setup() {
  Serial.begin(57600);
  delay(1000);

  pinMode(SHOULDER_STEP_PIN, OUTPUT);
  pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(ELBOW_STEP_PIN, OUTPUT);
  pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(WRIST_STEP_PIN, OUTPUT);
  pinMode(WRIST_DIR_PIN, OUTPUT);

  Serial.println("=== MOVEO Stepper Bridge v2 Ready ===");
  Serial.println("Joints mapped: 0=Shoulder, 2=Elbow, 4=Wrist");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      Serial.print("Raw received: ");
      Serial.println(data);

      // Parse 5 joint values
      float joints[5] = {0};
      int idx = 0;
      char *token = strtok((char*)data.c_str(), ",");
      while (token != NULL && idx < 5) {
        joints[idx++] = atof(token);
        token = strtok(NULL, ",");
      }

      Serial.print("Parsed: ");
      for (int i = 0; i < 5; i++) {
        Serial.print(joints[i], 3);
        Serial.print(" ");
      }
      Serial.println();

      // Simple test move for wired joints
      moveStepper(SHOULDER_STEP_PIN, SHOULDER_DIR_PIN, joints[0]);
      moveStepper(ELBOW_STEP_PIN, ELBOW_DIR_PIN, joints[2]);
      moveStepper(WRIST_STEP_PIN, WRIST_DIR_PIN, joints[4]);
    }
  }
  delay(10);
}

// Very basic stepper move function (blocking for testing)
void moveStepper(int stepPin, int dirPin, float angle_rad) {
  int steps = (int)(angle_rad * 200 * 8); // rough conversion, adjust microsteps later
  if (steps == 0) return;

  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  int absSteps = abs(steps);

  for (int i = 0; i < absSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

  Serial.print("Moved stepper on pin ");
  Serial.print(stepPin);
  Serial.print(" by ");
  Serial.print(steps);
  Serial.println(" steps");
}