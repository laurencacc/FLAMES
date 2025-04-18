// Define stepper motor control pins
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26

// Full-step sequence (8-step "half-step" mode for smoother motion)
const int steps[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

int stepIndex = 0;
const int stepDelayMicros = 900;  // 🔥 Try 1000, 800, 700, 600, etc.

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Set the coil states
  digitalWrite(IN1, steps[stepIndex][0]);
  digitalWrite(IN2, steps[stepIndex][1]);
  digitalWrite(IN3, steps[stepIndex][2]);
  digitalWrite(IN4, steps[stepIndex][3]);

  // Advance to the next step
  stepIndex = (stepIndex + 1) % 8;

  // Short delay to control speed
  delayMicroseconds(stepDelayMicros);
}
