#include <SCServo.h>

HLSCL hls;   

#define BAUD_RATE 1000000

// === Change these IDs to match your servos ===
const uint8_t servoIDs[] = {1, 2, 3};        // Add more IDs here
const int numServos = sizeof(servoIDs) / sizeof(servoIDs[0]);

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE);
  hls.pSerial = &Serial1;
  
  delay(1000);
  
  Serial.println("=== Multiple HLS3606M Control ===");
  Serial.println("Enter:  ID  position");
  Serial.println("Example: 1 2048   or   2 0   or   3 4095");
  
  // Enable torque on all servos
  for (int i = 0; i < numServos; i++) {
    hls.EnableTorque(servoIDs[i], 1);
    Serial.print("Torque ON for ID ");
    Serial.println(servoIDs[i]);
    delay(50);
  }
}

void loop() {
  if (Serial.available()) {
    int id = Serial.parseInt();           // Read servo ID
    int position = Serial.parseInt();     // Read position

    while (Serial.available()) Serial.read(); // clear buffer

    position = constrain(position, -1500, 4095);

    Serial.print("→ Servo ID ");
    Serial.print(id);
    Serial.print("  → Position ");
    Serial.println(position);

    hls.WritePosEx(id, position, 150, 50, 500);   // speed, accel, torque limit
  }

  // Show current position of all servos every 2 seconds
  static unsigned long last = 0;
  if (millis() - last > 2000) {
    for (int i = 0; i < numServos; i++) {
      int pos = hls.ReadPos(servoIDs[i]);
      if (pos != -1) {
        float angle = (pos * 360.0) / 4095.0;
        Serial.print("  ID ");
        Serial.print(servoIDs[i]);
        Serial.print(" | Raw: ");
        Serial.print(pos);
        Serial.print("  ≈ ");
        Serial.print(angle, 1);
        Serial.println("°");
      }
    }
    last = millis();
  }
}