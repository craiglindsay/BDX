#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_NeoPixel.h>

// Pin definitions for Display
#define TFT_CS1   20   // GPI20 (D7) for Display 1
#define TFT_DC1   8    // GPIO8 (D8) for Display 1
#define TFT_MOSI  9    // GPIO9 (D9)
#define TFT_SCLK  10   // GPIO10 (D10)

// Servo pin definitions
#define SERVO_UP_DOWN_PIN 6    // D4 (GPIO6) for Up/Down servo
#define SERVO_LEFT_RIGHT_PIN 7 // D5 (GPIO7) for Left/Right servo
#define SERVO_TILT_PIN 21      // D6 (GPIO21) for Tilt servo
#define SERVO_LEFT_EAR_PIN 4   // D2 (GPIO4) for Left Ear servo
#define SERVO_RIGHT_EAR_PIN 3  // D1 (GPIO3) for Right Ear servo

// DFPlayer Mini pin definitions
#define DFPLAYER_TX_PIN 5 // D3 (GPIO5) for TX to DFPlayer RX
#define DFPLAYER_RX_PIN -1 // Not used, set to -1

// Add with other pin definitions
#define NEOPIXEL_PIN 2 // D0 (GPIO2) for NeoPixel LED
#define NUM_PIXELS 1

// Servo position extents (in microseconds)
#define LOOK_UP 1400
#define LOOK_MIDDLE 1550
#define LOOK_DOWN 1700
#define TURN_LEFT 1625
#define TURN_MIDDLE 1525
#define TURN_RIGHT 1375
#define TILT_LEFT 1375
#define TILT_MIDDLE 1500
#define TILT_RIGHT 1625
#define LEFT_EAR_FORWARD 1075
#define LEFT_EAR_UP 1365
#define LEFT_EAR_BACK 2000
#define RIGHT_EAR_FORWARD 2000
#define RIGHT_EAR_UP 1595
#define RIGHT_EAR_BACK 1000

// Initialize displays
TFT_eSPI tft1 = TFT_eSPI(); // Display 1

// Initialize servos
Servo servoTilt;
Servo servoUpDown;
Servo servoLeftRight;
Servo servoLeftEar;
Servo servoRightEar;

// Initialize SoftwareSerial and DFPlayer
SoftwareSerial dfPlayerSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN); // RX, TX
DFRobotDFPlayerMini dfPlayer;

// Define colors
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_CYAN    0x07FF
#define COLOR_ORANGE  0xFD20
#define COLOR_YELLOW  0xFFE0
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_INDIGO  0x4810
#define COLOR_VIOLET  0x8010


// User-adjustable variables for head pause
#define HEAD_PAUSE_INTERVAL 6000  // Base interval between head pauses (ms)
#define HEAD_PAUSE_DURATION 5000  // Base duration of head pause (ms)
#define HEAD_PAUSE_INTERVAL_RANDOM 4000 // Randomization range for interval (+/- ms)
#define HEAD_PAUSE_DURATION_RANDOM 2000 // Randomization range for duration (+/- ms)

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastLedUpdate = 0;
uint8_t ledState = 0; // 0: off, 1: white, 2: rainbow, 3: red, 4: flash red, 5: flash patriotic
uint8_t rainbowHue = 0;
uint8_t patrioticStep = 0;

unsigned long lastBlink = 0;
unsigned long nextBlinkInterval = 5000;
unsigned long lastEmotion = 0;
unsigned long nextEmotionInterval = 75000;
int emotionState = -1; // -1: normal, 0: scowl, 1: surprised, 2: spiral, 3: color wheel, 4: heart, 5: black-white wheel
int nextEmotionState = 0; // Tracks the next animation to play
unsigned long emotionStartTime = 0;
bool animationActive = false;

// Sound timing variables
unsigned long lastSoundTime = 0;
unsigned long nextSoundInterval = 3300; // Initial interval in ms

// Global servo position variables for randomHeadMovement and animations
  int currentUpDown = LOOK_MIDDLE;
  int currentLeftRight = TURN_MIDDLE;
  int currentTilt = TILT_MIDDLE;
  int currentLeftEar = LEFT_EAR_UP;
  int currentRightEar = RIGHT_EAR_UP;

// Head pause timing variables
unsigned long lastHeadPause = 0;
bool headPaused = false;
unsigned long currentPauseInterval = HEAD_PAUSE_INTERVAL;
unsigned long currentPauseDuration = HEAD_PAUSE_DURATION;



// Modified cubic ease-in-out function for smoother start and end
float easeInOutCubic(float t) {
  // Adjusted to emphasize slower start and end
  if (t < 0.5) {
    return 8 * t * t * t * t; // Accelerate more gradually at the start
  } else {
    return 1 - 8 * pow(1 - t, 4); // Decelerate more gradually at the end
  }
}

// Easing function for continuous motion (smoothstep for natural motion)
float easeContinuous(float t) {
  return t * t * (3 - 2 * t); // Smoothstep for acceleration/deceleration
}

// Function for continuous servo movement with blending
void moveServosContinuously(Servo &servo1, int startUs1, int targetUs1, int nextUs1, 
                           Servo &servo2, int startUs2, int targetUs2, int nextUs2, 
                           Servo &servo3, int startUs3, int targetUs3, int nextUs3, 
                           unsigned long duration, int &currentUs1, int &currentUs2, int &currentUs3) {
  const int steps = 400; // Increased for smoother motion
  float stepTime = duration / (float)steps;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    t = easeContinuous(t);
    
    // Blend between current target and next target starting at 30% progress
    float blendFactor = (t < 0.3) ? 0 : (t - 0.3) / 0.7; // 0 at t=0.3, 1 at t=1
    float invBlendFactor = 1 - blendFactor;

    if (startUs1 != targetUs1 || startUs1 != nextUs1) {
      int blendedTarget1 = (invBlendFactor * targetUs1) + (blendFactor * nextUs1);
      int pos1 = startUs1 + (blendedTarget1 - startUs1) * t;
      servo1.writeMicroseconds(pos1);
      if (i == steps) currentUs1 = pos1; // Update current position at end
    }
    if (startUs2 != targetUs2 || startUs2 != nextUs2) {
      int blendedTarget2 = (invBlendFactor * targetUs2) + (blendFactor * nextUs2);
      int pos2 = startUs2 + (blendedTarget2 - startUs2) * t;
      servo2.writeMicroseconds(pos2);
      if (i == steps) currentUs2 = pos2;
    }
    if (startUs3 != targetUs3 || startUs3 != nextUs3) {
      int blendedTarget3 = (invBlendFactor * targetUs3) + (blendFactor * nextUs3);
      int pos3 = startUs3 + (blendedTarget3 - startUs3) * t;
      servo3.writeMicroseconds(pos3);
      if (i == steps) currentUs3 = pos3;
    }
    delayMicroseconds((int)(stepTime * 1200));
  }
}

void ledOnWhite() {
  ledState = 1;
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // White
  pixels.show();
}
                             
void ledOff() {
  ledState = 0;
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Off
  pixels.show();
}

void ledRainbowCycle() {
  ledState = 2;
  lastLedUpdate = millis(); // Initialize for rainbow cycle
}

void ledRed() {
  ledState = 3;
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
  pixels.show();
}

void ledFlashRed() {
  ledState = 4;
  lastLedUpdate = millis(); // Initialize for flashing
}

void ledFlashPatriotic() {
  ledState = 5;
  patrioticStep = 0;
  lastLedUpdate = millis(); // Initialize for patriotic flashing
}

void ledBlue() {
  ledState = 6;
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue
  pixels.show();
}

void ledFlashWhite() {
  ledState = 7;
  lastLedUpdate = millis(); // Initialize for flashing
}

void updateLedAnimations() {
  unsigned long now = millis();
  if (ledState == 2 && now - lastLedUpdate >= 30) { // Rainbow cycle, update every 30ms
    rainbowHue++;
    uint32_t color = pixels.gamma32(pixels.ColorHSV(rainbowHue * 256)); // Hue increments
    pixels.setPixelColor(0, color);
    pixels.show();
    lastLedUpdate = now;
  } else if (ledState == 4 && now - lastLedUpdate >= 1000) { // Flash red every 1s
    if (pixels.getPixelColor(0) == 0) {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red on
    } else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Off
    }
    pixels.show();
    lastLedUpdate = now;
  } else if (ledState == 7 && now - lastLedUpdate >= 1000) { // Flash white every 1s
    if (pixels.getPixelColor(0) == 0) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // White on
    } else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Off
    }
    pixels.show();
    lastLedUpdate = now;
  } else if (ledState == 5 && now - lastLedUpdate >= 1000) { // Flash red/white/blue/off every 1s
    switch (patrioticStep) {
      case 0: // Red
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        patrioticStep = 1;
        break;
      case 1: // White
        pixels.setPixelColor(0, pixels.Color(255, 255, 255));
        patrioticStep = 2;
        break;
      case 2: // Blue
        pixels.setPixelColor(0, pixels.Color(0, 0, 255));
        patrioticStep = 3;
        break;
      case 3: // Off
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        patrioticStep = 0;
        break;
    }
    pixels.show();
   lastLedUpdate = now;
  }
}




// Function to smoothly move a servo to a target position
void moveServoSmoothly(Servo &servo, int startUs, int targetUs, unsigned long duration) {
  const int steps = 200; // High resolution for smooth motion
  float stepTime = duration / (float)steps;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    t = easeInOutCubic(t); // Apply modified cubic easing
    int pos = startUs + (targetUs - startUs) * t;
    servo.writeMicroseconds(pos);
    delayMicroseconds((int)(stepTime * 1000));
  }
}

// Function to move multiple servos simultaneously
void moveServosSmoothly(Servo &servo1, int startUs1, int targetUs1, Servo &servo2, int startUs2, int targetUs2, Servo &servo3, int startUs3, int targetUs3, unsigned long duration) {
  const int steps = 200;
  float stepTime = duration / (float)steps;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    t = easeInOutCubic(t);
    if (startUs1 != targetUs1) {
      int pos1 = startUs1 + (targetUs1 - startUs1) * t;
      servo1.writeMicroseconds(pos1);
    }
    if (startUs2 != targetUs2) {
      int pos2 = startUs2 + (targetUs2 - startUs2) * t;
      servo2.writeMicroseconds(pos2);
    }
    if (startUs3 != targetUs3) {
      int pos3 = startUs3 + (targetUs3 - startUs3) * t;
      servo3.writeMicroseconds(pos3);
    }
    delayMicroseconds((int)(stepTime * 1000));
  }
}

// Function to move both ears to back position for scowl animation
void earsBack() {
  Serial.println("Moving ears to back position for scowl");
  moveServosSmoothly(servoLeftEar, currentLeftEar, LEFT_EAR_BACK, 
                     servoRightEar, currentRightEar, RIGHT_EAR_BACK, 
                     servoTilt, currentTilt, currentTilt, 1000);
  currentLeftEar = LEFT_EAR_BACK;
  currentRightEar = RIGHT_EAR_BACK;
}


// Function for continuous head and ear movement
void constantHeadAndEarMovement() {
  // Define ranges for head movements
  int smallUpDownRange = (LOOK_DOWN - LOOK_UP) / 3; // 25% of up/down range
  int smallLeftRightRange = (TURN_LEFT - TURN_RIGHT) / 3; // 25% of left/right range
  int smallTiltRange = (TILT_RIGHT - TILT_LEFT) / 3; // 25% of tilt range
  int microUpDownRange = (LOOK_DOWN - LOOK_UP) / 6; // 12.5% for closer movements
  int microLeftRightRange = (TURN_LEFT - TURN_RIGHT) / 6; // 12.5% for closer movements
  int microTiltRange = (TILT_RIGHT - TILT_LEFT) / 10; // 5% for tilt

  // Define ranges for ear movements (smaller for subtle motion)
  int leftEarRange = (LEFT_EAR_BACK - LEFT_EAR_FORWARD) / 3; // ~33% of full range
  int rightEarRange = (RIGHT_EAR_FORWARD - RIGHT_EAR_BACK) / 3; // ~33% of full range
  int microLeftEarRange = leftEarRange / 2; // ~16.5% for smaller movements
  int microRightEarRange = rightEarRange / 2; // ~16.5% for smaller movements

  // Check for head pause
  unsigned long now = millis();
  if (now - lastHeadPause >= currentPauseInterval + currentPauseDuration) {
    lastHeadPause = now;
    headPaused = true;
    // Randomize pause interval and duration for next cycle
    currentPauseInterval = HEAD_PAUSE_INTERVAL + random(-HEAD_PAUSE_INTERVAL_RANDOM, HEAD_PAUSE_INTERVAL_RANDOM + 1);
    currentPauseDuration = HEAD_PAUSE_DURATION + random(-HEAD_PAUSE_DURATION_RANDOM, HEAD_PAUSE_DURATION_RANDOM + 1);
    // Ensure non-negative values with type-consistent comparison
    currentPauseInterval = max(currentPauseInterval, 1000UL); // Minimum 1 second
    currentPauseDuration = max(currentPauseDuration, 500UL);  // Minimum 0.5 second
  } else if (now - lastHeadPause >= currentPauseDuration) {
    headPaused = false;
  }

  // Decide number of head servos to move (1 to 3) if not paused
  int headServoCount = headPaused ? 0 : random(2, 4);
  int targetUpDown = currentUpDown;
  int targetLeftRight = currentLeftRight;
  int targetTilt = currentTilt;
  int nextUpDown = currentUpDown;
  int nextLeftRight = currentLeftRight;
  int nextTilt = currentTilt;

  // Always allow ear movements
  bool moveEars = random(0, 3) == 0;
  int targetLeftEar = currentLeftEar;
  int targetRightEar = currentRightEar;
  int nextLeftEar = currentLeftEar;
  int nextRightEar = currentRightEar;

  // Decide ranges for head if not paused
  bool useMicroRange = random(0, 100) < 80;
  int upDownRange = useMicroRange ? microUpDownRange : smallUpDownRange;
  int leftRightRange = useMicroRange ? microLeftRightRange : smallLeftRightRange;
  int tiltRange = random(0, 100) < 50 ? microTiltRange : 0;

  // Select random directions for head
  int upDownDirection = random(0, 2) * 2 - 1; // 1 or -1
  int leftRightDirection = random(0, 2) * 2 - 1; // 1 or -1
  int tiltDirection = random(0, 2) * 2 - 1; // 1 or -1

  // Select current head target positions if not paused
  if (!headPaused && headServoCount >= 1) {
    targetUpDown = LOOK_MIDDLE + upDownDirection * random(0, upDownRange + 1);
    targetUpDown = constrain(targetUpDown, LOOK_UP, LOOK_DOWN);
    while (targetUpDown == currentUpDown && upDownRange > 0) {
      upDownDirection = random(0, 2) * 2 - 1;
      targetUpDown = LOOK_MIDDLE + upDownDirection * random(0, upDownRange + 1);
      targetUpDown = constrain(targetUpDown, LOOK_UP, LOOK_DOWN);
    }
  }
  if (!headPaused && headServoCount >= 2) {
    targetLeftRight = TURN_MIDDLE + leftRightDirection * random(0, leftRightRange + 1);
    targetLeftRight = constrain(targetLeftRight, TURN_RIGHT, TURN_LEFT);
    while (targetLeftRight == currentLeftRight && leftRightRange > 0) {
      leftRightDirection = random(0, 2) * 2 - 1;
      targetLeftRight = TURN_MIDDLE + leftRightDirection * random(0, leftRightRange + 1);
      targetLeftRight = constrain(targetLeftRight, TURN_RIGHT, TURN_LEFT);
    }
  }
  if (!headPaused && headServoCount == 3 && tiltRange > 0) {
    targetTilt = TILT_MIDDLE + tiltDirection * random(0, tiltRange + 1);
    targetTilt = constrain(targetTilt, TILT_LEFT, TILT_RIGHT);
    while (targetTilt == currentTilt && tiltRange > 0) {
      tiltDirection = random(0, 2) * 2 - 1;
      targetTilt = TILT_MIDDLE + tiltDirection * random(0, tiltRange + 1);
      targetTilt = constrain(targetTilt, TILT_LEFT, TILT_RIGHT);
    }
  }

  // Select ear target positions if moving
  if (moveEars) {
    int movementType = random(0, 100);
    // Prefer micro ranges for ear movements (80% probability)
    int earRangeLeft = random(0, 100) < 80 ? microLeftEarRange : leftEarRange;
    int earRangeRight = random(0, 100) < 80 ? microRightEarRange : rightEarRange;
    if (movementType < 40) { // 40% both up
      targetLeftEar = LEFT_EAR_UP;
      targetRightEar = RIGHT_EAR_UP;
    } else if (movementType < 60) { // 20% both forward
      targetLeftEar = LEFT_EAR_UP + random(-earRangeLeft, earRangeLeft + 1);
      targetRightEar = RIGHT_EAR_UP + random(-earRangeRight, earRangeRight + 1);
      targetLeftEar = constrain(targetLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      targetRightEar = constrain(targetRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    } else if (movementType < 80) { // 20% both back
      targetLeftEar = LEFT_EAR_UP + random(-earRangeLeft, earRangeLeft + 1);
      targetRightEar = RIGHT_EAR_UP + random(-earRangeRight, earRangeRight + 1);
      targetLeftEar = constrain(targetLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      targetRightEar = constrain(targetRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    } else { // 20% opposite
      if (random(0, 2) == 0) {
        targetLeftEar = LEFT_EAR_UP + random(-earRangeLeft, 0);
        targetRightEar = RIGHT_EAR_UP + random(0, earRangeRight + 1);
      } else {
        targetLeftEar = LEFT_EAR_UP + random(0, earRangeLeft + 1);
        targetRightEar = RIGHT_EAR_UP + random(-earRangeRight, 0);
      }
      targetLeftEar = constrain(targetLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      targetRightEar = constrain(targetRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    }
    if (targetLeftEar == currentLeftEar) {
      targetLeftEar = LEFT_EAR_UP;
    }
    if (targetRightEar == currentRightEar) {
      targetRightEar = RIGHT_EAR_UP;
    }
  }

  // Calculate duration based on maximum distance (considering all moving servos)
  int maxDistance = 0;
  if (!headPaused && headServoCount >= 1) {
    maxDistance = max(maxDistance, abs(targetUpDown - currentUpDown));
  }
  if (!headPaused && headServoCount >= 2) {
    maxDistance = max(maxDistance, abs(targetLeftRight - currentLeftRight));
  }
  if (!headPaused && headServoCount == 3 && tiltRange > 0) {
    maxDistance = max(maxDistance, abs(targetTilt - currentTilt));
  }
  if (moveEars) {
    maxDistance = max(maxDistance, abs(targetLeftEar - currentLeftEar));
    maxDistance = max(maxDistance, abs(targetRightEar - currentRightEar));
  }
  unsigned long duration = 200 + (maxDistance * 200 / max(microUpDownRange, max(microLeftRightRange, max(microTiltRange, max(microLeftEarRange, microRightEarRange)))));
  if (maxDistance > microUpDownRange || maxDistance > microLeftRightRange || maxDistance > microTiltRange || maxDistance > microLeftEarRange || maxDistance > microRightEarRange) {
    duration = 200 + (maxDistance * 200 / max(smallUpDownRange, max(smallLeftRightRange, max(smallTiltRange, max(leftEarRange, rightEarRange)))));
  }
  duration = constrain(duration, 200, 500);

  // Select next head target positions if not paused
  headServoCount = headPaused ? 0 : random(1, 4);
  useMicroRange = random(0, 100) < 80;
  upDownRange = useMicroRange ? microUpDownRange : smallUpDownRange;
  leftRightRange = useMicroRange ? microLeftRightRange : smallLeftRightRange;
  tiltRange = random(0, 100) < 50 ? microTiltRange : 0;
  upDownDirection = random(0, 2) * 2 - 1;
  leftRightDirection = random(0, 2) * 2 - 1;
  tiltDirection = random(0, 2) * 2 - 1;
  if (!headPaused && headServoCount >= 1) {
    nextUpDown = LOOK_MIDDLE + upDownDirection * random(0, upDownRange + 1);
    nextUpDown = constrain(nextUpDown, LOOK_UP, LOOK_DOWN);
    while (nextUpDown == targetUpDown && upDownRange > 0) {
      upDownDirection = random(0, 2) * 2 - 1;
      nextUpDown = LOOK_MIDDLE + upDownDirection * random(0, upDownRange + 1);
      nextUpDown = constrain(nextUpDown, LOOK_UP, LOOK_DOWN);
    }
  } else {
    nextUpDown = targetUpDown;
  }
  if (!headPaused && headServoCount >= 2) {
    nextLeftRight = TURN_MIDDLE + leftRightDirection * random(0, leftRightRange + 1);
    nextLeftRight = constrain(nextLeftRight, TURN_RIGHT, TURN_LEFT);
    while (nextLeftRight == targetLeftRight && leftRightRange > 0) {
      leftRightDirection = random(0, 2) * 2 - 1;
      nextLeftRight = TURN_MIDDLE + leftRightDirection * random(0, leftRightRange + 1);
      nextLeftRight = constrain(nextLeftRight, TURN_RIGHT, TURN_LEFT);
    }
  } else {
    nextLeftRight = targetLeftRight;
  }
  if (!headPaused && headServoCount == 3 && tiltRange > 0) {
    nextTilt = TILT_MIDDLE + tiltDirection * random(0, tiltRange + 1);
    nextTilt = constrain(nextTilt, TILT_LEFT, TILT_RIGHT);
    while (nextTilt == targetTilt && tiltRange > 0) {
      tiltDirection = random(0, 2) * 2 - 1;
      nextTilt = TILT_MIDDLE + tiltDirection * random(0, tiltRange + 1);
      nextTilt = constrain(nextTilt, TILT_LEFT, TILT_RIGHT);
    }
  } else {
    nextTilt = targetTilt;
  }

  // Select next ear target positions
  bool nextMoveEars = random(0, 3) == 0;
  if (nextMoveEars) {
    int movementType = random(0, 100);
    // Prefer micro ranges for next ear movements (80% probability)
    int earRangeLeft = random(0, 100) < 80 ? microLeftEarRange : leftEarRange;
    int earRangeRight = random(0, 100) < 80 ? microRightEarRange : rightEarRange;
    if (movementType < 40) { // 40% both up
      nextLeftEar = LEFT_EAR_UP;
      nextRightEar = RIGHT_EAR_UP;
    } else if (movementType < 60) { // 20% both forward
      nextLeftEar = LEFT_EAR_UP + random(-earRangeLeft, earRangeLeft + 1);
      nextRightEar = RIGHT_EAR_UP + random(-earRangeRight, earRangeRight + 1);
      nextLeftEar = constrain(nextLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      nextRightEar = constrain(nextRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    } else if (movementType < 80) { // 20% both back
      nextLeftEar = LEFT_EAR_UP + random(-earRangeLeft, earRangeLeft + 1);
      nextRightEar = RIGHT_EAR_UP + random(-earRangeRight, earRangeRight + 1);
      nextLeftEar = constrain(nextLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      nextRightEar = constrain(nextRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    } else { // 20% opposite
      if (random(0, 2) == 0) {
        nextLeftEar = LEFT_EAR_UP + random(-earRangeLeft, 0);
        nextRightEar = RIGHT_EAR_UP + random(0, earRangeRight + 1);
      } else {
        nextLeftEar = LEFT_EAR_UP + random(0, earRangeLeft + 1);
        nextRightEar = RIGHT_EAR_UP + random(-earRangeRight, 0);
      }
      nextLeftEar = constrain(nextLeftEar, LEFT_EAR_FORWARD, LEFT_EAR_BACK);
      nextRightEar = constrain(nextRightEar, RIGHT_EAR_BACK, RIGHT_EAR_FORWARD);
    }
    if (nextLeftEar == targetLeftEar) {
      nextLeftEar = LEFT_EAR_UP;
    }
    if (nextRightEar == targetRightEar) {
      nextRightEar = RIGHT_EAR_UP;
    }
  } else {
    nextLeftEar = targetLeftEar;
    nextRightEar = targetRightEar;
  }

  // Perform current movement with blending to next target
  unsigned long startTime = millis();
  const int steps = 400;
  float stepTime = duration / (float)steps;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    t = easeContinuous(t);
    float blendFactor = (t < 0.3) ? 0 : (t - 0.3) / 0.7;
    float invBlendFactor = 1 - blendFactor;

    // Head servos (only move if not paused)
    if (!headPaused && targetUpDown != currentUpDown) {
      int blendedTarget = (invBlendFactor * targetUpDown) + (blendFactor * nextUpDown);
      int pos = currentUpDown + (blendedTarget - currentUpDown) * t;
      servoUpDown.writeMicroseconds(pos);
      if (i == steps) currentUpDown = pos;
    }
    if (!headPaused && targetLeftRight != currentLeftRight) {
      int blendedTarget = (invBlendFactor * targetLeftRight) + (blendFactor * nextLeftRight);
      int pos = currentLeftRight + (blendedTarget - currentLeftRight) * t;
      servoLeftRight.writeMicroseconds(pos);
      if (i == steps) currentLeftRight = pos;
    }
    if (!headPaused && targetTilt != currentTilt) {
      int blendedTarget = (invBlendFactor * targetTilt) + (blendFactor * nextTilt);
      int pos = currentTilt + (blendedTarget - currentTilt) * t;
      servoTilt.writeMicroseconds(pos);
      if (i == steps) currentTilt = pos;
    }
    // Ear servos (always move if targeted)
    if (moveEars && targetLeftEar != currentLeftEar) {
      int blendedTarget = (invBlendFactor * targetLeftEar) + (blendFactor * nextLeftEar);
      int pos = currentLeftEar + (blendedTarget - currentLeftEar) * t;
      servoLeftEar.writeMicroseconds(pos);
      if (i == steps) currentLeftEar = pos;
    }
    if (moveEars && targetRightEar != currentRightEar) {
      int blendedTarget = (invBlendFactor * targetRightEar) + (blendFactor * nextRightEar);
      int pos = currentRightEar + (blendedTarget - currentRightEar) * t;
      servoRightEar.writeMicroseconds(pos);
      if (i == steps) currentRightEar = pos;
    }
    delayMicroseconds((int)(stepTime * 1000));
  }
}

void setupDFPlayer() {
  dfPlayerSerial.begin(9600);
  Serial.println("Initializing DFPlayer Mini...");
  dfPlayer.begin(dfPlayerSerial, false); // false to avoid blocking if DFPlayer not detected
  dfPlayer.volume(30); // Set volume (0 to 30)
  Serial.println("DFPlayer Mini initialized");
}

void animateYesNod() {
  Serial.println("Starting yes nod animation: 4 up-down cycles");
  
  // Perform 4 up-down cycles
  for (int i = 0; i < 4; i++) {
    // Move up 30% of the extent from middle to up
    int targetUp = LOOK_MIDDLE - (LOOK_MIDDLE - LOOK_UP) * 0.3;
    moveServosSmoothly(servoUpDown, currentUpDown, targetUp, 
                       servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                       servoTilt, currentTilt, TILT_MIDDLE, 500);
    currentUpDown = targetUp;
    currentLeftRight = TURN_MIDDLE;
    currentTilt = TILT_MIDDLE;
    
    // Move back to middle
    moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                       servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                       servoTilt, currentTilt, TILT_MIDDLE, 500);
    currentUpDown = LOOK_MIDDLE;
    currentLeftRight = TURN_MIDDLE;
    currentTilt = TILT_MIDDLE;
  }
  
  Serial.println("Yes nod animation completed");
}

void animateNoShake() {
  Serial.println("Starting no shake animation: 3 left-right oscillations without middle pause");

  // Calculate 60% of the extent from middle to left and right
  int targetLeft = TURN_MIDDLE + (TURN_LEFT - TURN_MIDDLE) * 0.5;
  int targetRight = TURN_MIDDLE - (TURN_MIDDLE - TURN_RIGHT) * 0.5;
  int segmentTime = 200; // 6000ms / 14 segments (3 oscillations * 4 + 2 for start/end)

  // Move from current position to middle
  moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, segmentTime);
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;

  // Perform 4 oscillations: left -> right -> left
  for (int i = 0; i < 4; i++) {
    // Move to left
    dfPlayer.play(25); // Play the track
    updateLedAnimations();
    moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                       servoLeftRight, currentLeftRight, targetLeft, 
                       servoTilt, currentTilt, TILT_MIDDLE, segmentTime);
    currentLeftRight = targetLeft;

    // Move to right
    dfPlayer.play(25); // Play the track
    updateLedAnimations();
    moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                       servoLeftRight, currentLeftRight, targetRight, 
                       servoTilt, currentTilt, TILT_MIDDLE, segmentTime);
    currentLeftRight = targetRight;
  }
  // Move back to middle
  dfPlayer.play(26); // Play the track
  updateLedAnimations();
  moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, segmentTime);
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;

  Serial.println("No shake animation completed");
}


void drawEye(uint16_t color, int heightPct = 100, bool clear = false) {
  digitalWrite(TFT_CS1, LOW); // Select display
  int centerX = 120, centerY = 120;
  int radius = 120;
  int visibleH = radius * 2 * heightPct / 100;
  int yStart = centerY - visibleH / 2;

  // Clear eye region only if requested
  if (clear) {
    tft1.fillRect(centerX - radius - 5, centerY - radius - 5, radius * 2 + 10, radius * 2 + 10, COLOR_BLACK);
  }
  // Draw new eye
  tft1.fillRect(centerX - radius, yStart, radius * 2, visibleH, color);
}

void animateScowlEye() {
  // Access static variables from randomHeadMovement (assuming they are declared globally or accessible)
  static int currentUpDown = LOOK_MIDDLE; // Persist position from randomHeadMovement
  static int currentLeftRight = TURN_MIDDLE;
  static int currentTilt = TILT_MIDDLE;

  Serial.println("Starting scowl animation: fade to red");

  // Move head down 30% of the extent from middle to down, starting from current position
  int targetDown = LOOK_MIDDLE + (LOOK_DOWN - LOOK_MIDDLE) * 0.3;
  moveServosSmoothly(servoUpDown, currentUpDown, targetDown, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, 2000);
  
  earsBack(); // Move ears to back position
  
  // Update current positions
  currentUpDown = targetDown;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;
  dfPlayer.play(24); // Play the track
  // Fade from white (R=31, G=63, B=31) to red (R=31, G=0, B=0) (2 seconds)
  for (int step = 0; step <= 20; step++) {
    float t = (float)step / 20;
    uint8_t r = 31;
    uint8_t g = 63 * (1 - t);
    uint8_t b = 31 * (1 - t);
    uint16_t color = (r << 11) | (g << 5) | b;
    updateLedAnimations();
    drawEye(color, 100, false);
    delay(35); // 20 steps * 100 ms = 2 seconds
  }

  delay(100); // Hold red
  // Narrow to 5% height (12 pixels) by drawing black boxes (0.5s, 10 steps)
  int centerX = 120, centerY = 120, radius = 120;
  for (int h = 100; h >= 5; h -= 10) {
    int visibleH = radius * 2 * h / 100;
    int yStart = centerY - visibleH / 2;
    updateLedAnimations();
    tft1.fillRect(centerX - radius, yStart, radius * 2, visibleH, COLOR_RED);
    if (h < 100) {
      int prevH = radius * 2 * (h + 10) / 100;
      int prevYStart = centerY - prevH / 2;
      int deltaH = prevH - visibleH;
      tft1.fillRect(centerX - radius - 5, prevYStart - 5, radius * 2 + 10, deltaH / 2 + 10, COLOR_BLACK);
      tft1.fillRect(centerX - radius - 5, yStart + visibleH - 5, radius * 2 + 10, deltaH / 2 + 10, COLOR_BLACK);
    }
    delay(50); // 10 steps * 50 ms = 0.5 seconds
  }

// Perform no head shake and hold to match 9000ms
  animateNoShake(); // Takes 6000ms (3 cycles * 4 moves * 500ms)
  //delay(3000); // Additional delay to total 9000ms

  // Expand back to full height by drawing red boxes (2s, 20 steps)
  for (int h = 5; h <= 100; h += 5) {
    int visibleH = radius * 2 * h / 100;
    int yStart = centerY - visibleH / 2;
    updateLedAnimations();
    tft1.fillRect(centerX - radius, yStart, radius * 2, visibleH, COLOR_RED);
    delay(50); // 20 steps * 100 ms = 2 seconds
  }

  delay(2000); // Hold full red for 2 seconds

  // Fade from red (R=31, G=0, B=0) to white (R=31, G=63, B=31) (2 seconds)
  for (int step = 0; step <= 20; step++) {
    float t = (float)step / 20;
    uint8_t r = 31;
    uint8_t g = 63 * t;
    uint8_t b = 31 * t;
    uint16_t color = (r << 11) | (g << 5) | b;
    updateLedAnimations();
    drawEye(color, 100, false);
    delay(35); // 20 steps * 100 ms = 2 seconds
  }

  // Return head to middle smoothly
  moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, 2000);

  // Reset static variables for randomHeadMovement
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;

  Serial.println("Scowl animation completed");
}

void drawSurprisedEye(int pupilXOffset = 0, int pupilYOffset = 0) {
  int centerX = 120, centerY = 120;
  // Redraw cyan circle
  tft1.fillCircle(centerX, centerY, 96, COLOR_CYAN);
  tft1.fillCircle(centerX + pupilXOffset, centerY + pupilYOffset, 36, COLOR_BLACK);
}

void animateSurprisedEye() {
  int steps = 8; // 8 looks over 20 seconds (1000ms look + 1000ms center)
  int directions[8][2] = {
    {0, -40},  // Up
    {0, 40},   // Down
    {-40, 0},  // Left
    {40, 0},   // Right
    {-35, -35}, // Up-Left
    {35, -35},  // Up-Right
    {-35, 35},  // Down-Left
    {35, 35}    // Down-Right
  };

  Serial.println("Starting surprised animation with random directional looks, 1000ms per look and center");

  for (int i = 0; i < steps; i++) {
    // Randomly select direction
    int dir = random(0, 8);
    int xOffset = directions[dir][0];
    int yOffset = directions[dir][1];

    // Map eye offsets to servo positions (normalized to servo range, inverted for 180-degree display rotation)
    float tX = (float)(xOffset + 40) / 80; // Normalize xOffset (-40 to 40) to 0-1
    float tY = (float)(-yOffset + 40) / 80; // Invert yOffset to reverse up/down
    int targetLeftRight = TURN_RIGHT + (TURN_LEFT - TURN_RIGHT) * tX;
    int targetUpDown = LOOK_DOWN + (LOOK_UP - LOOK_DOWN) * tY;
    int targetTilt = TILT_MIDDLE; // Tilt stays centered during looks

    // Snap pupil to target position
    drawSurprisedEye(xOffset, yOffset);

    // Move servos simultaneously
    moveServosSmoothly(servoLeftRight, currentLeftRight, targetLeftRight, 
                       servoUpDown, currentUpDown, targetUpDown, 
                       servoTilt, currentTilt, targetTilt, 1000);

    // Update current positions
    currentLeftRight = targetLeftRight;
    currentUpDown = targetUpDown;
    currentTilt = targetTilt;

    delay(1000); // Hold look for 1000ms

    // Snap pupil to center position for return
    drawSurprisedEye(0, 0);

    // Snap pupil to next direction (opposite for return to center)
    int nextXOffset = -xOffset; // Opposite direction for return
    int nextYOffset = -yOffset;
    drawSurprisedEye(nextXOffset, nextYOffset);

    // Return servos to center
    moveServosSmoothly(servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                       servoUpDown, currentUpDown, LOOK_MIDDLE, 
                       servoTilt, currentTilt, TILT_MIDDLE, 1000);

    // Update current positions
    currentLeftRight = TURN_MIDDLE;
    currentUpDown = LOOK_MIDDLE;
    currentTilt = TILT_MIDDLE;

    delay(1000); // Hold center for 1000ms
  }

  // Ensure final return to center positions
  moveServosSmoothly(servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, 1000);
  drawSurprisedEye(0, 0); // Ensure pupil is centered

  // Reset global variables for randomHeadMovement to prevent jumping
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;
}

void drawSpiral(uint16_t colors[], int baseRadius, float angleOffset) {
  int centerX = 120, centerY = 120;
  // Draw 10 spiral arms
  for (int i = 0; i < 10; i++) {
    float radius = baseRadius * (1.0 - (float)i / 10); // Decrease radius
    int r = (int)radius;
    if (r < 1) r = 1; // Ensure non-zero radius
    float angle = (float)i * 2 * PI / 10 + angleOffset;
    int x = centerX + (int)(radius * cos(angle));
    int y = centerY + (int)(radius * sin(angle));
    updateLedAnimations();
    tft1.fillCircle(x, y, r / 9, colors[i % 9]); // Use 9 colors
  }
}

void animateSpiral() {
  uint16_t baseColors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_INDIGO, COLOR_VIOLET };
  uint16_t colors[6];
  int totalFrames = 1200; // 10 seconds / 25 ms per frame

  // Initialize with black background
  tft1.fillRect(0, 0, 240, 240, COLOR_BLACK);

  // Animate spiral
  for (int frame = 0; frame < totalFrames; frame++) {
    float t = (float)(frame % 50) / 50; // Smooth color transition over 50 frames
    int cycle = frame / 50;
    for (int i = 0; i < 6; i++) {
      int colorIndex = (i + cycle) % 3; // Match baseColors length
      int nextColorIndex = (colorIndex + 1) % 3;
      uint16_t startColor = baseColors[colorIndex];
      uint16_t endColor = baseColors[nextColorIndex];
      uint8_t r1 = (startColor >> 11) & 0x1F;
      uint8_t g1 = (startColor >> 5) & 0x3F;
      uint8_t b1 = startColor & 0x1F;
      uint8_t r2 = (endColor >> 11) & 0x1F;
      uint8_t g2 = (endColor >> 5) & 0x3F;
      uint8_t b2 = endColor & 0x1F;
      uint8_t r = r1 + (r2 - r1) * t;
      uint8_t g = g1 + (g2 - g1) * t;
      uint8_t b = b1 + (b2 - b1) * t;
      colors[i] = (r << 11) | (g << 5) | b;
    }
    float angleOffset = (float)frame * 2 * PI / 50; // Rotate every ~50 frames
    int baseRadius = 100; // Max radius
    updateLedAnimations();
    drawSpiral(colors, baseRadius, angleOffset);
    delay(10); // 25 ms per frame, 400 frames = 10 seconds
  }
}

void drawColorWheel(uint16_t colors[], float angleOffset) {
  int centerX = 120, centerY = 120;
  int radius = 120;
  int segments = 12;
  // Draw 12 segments
  for (int i = 0; i < segments; i++) {
    float angle1 = (float)i * 2 * PI / segments + angleOffset;
    float angle2 = (float)(i + 1) * 2 * PI / segments + angleOffset;
    int x1 = centerX + (int)(radius * cos(angle1));
    int y1 = centerY + (int)(radius * sin(angle1));
    int x2 = centerX + (int)(radius * cos(angle2));
    int y2 = centerY + (int)(radius * sin(angle2));
    updateLedAnimations();
    tft1.fillTriangle(centerX, centerY, x1, y1, x2, y2, colors[i % 7]);
  }
}

void animateColorWheel() {
  uint16_t baseColors[] = {COLOR_RED, COLOR_ORANGE, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE, COLOR_INDIGO, COLOR_VIOLET};
  uint16_t colors[7];
  int totalFrames = 200; // 5 seconds / 25 ms per frame

  // Initialize with black background
  updateLedAnimations();
  tft1.fillRect(0, 0, 240, 240, COLOR_BLACK);
  
  // Animate color wheel
  for (int frame = 0; frame < totalFrames; frame++) {
    float t = (float)(frame % 50) / 50; // Smooth transition over 50 frames
    int cycle = frame / 50;
    for (int i = 0; i < 7; i++) {
      int colorIndex = (i + cycle) % 7;
      int nextColorIndex = (colorIndex + 1) % 7;
      uint16_t startColor = baseColors[colorIndex];
      uint16_t endColor = baseColors[nextColorIndex];
      uint8_t r1 = (startColor >> 11) & 0x1F;
      uint8_t g1 = (startColor >> 5) & 0x3F;
      uint8_t b1 = startColor & 0x1F;
      uint8_t r2 = (endColor >> 11) & 0x1F;
      uint8_t g2 = (endColor >> 5) & 0x3F;
      uint8_t b2 = endColor & 0x1F;
      uint8_t r = r1 + (r2 - r1) * t;
      uint8_t g = g1 + (g2 - g1) * t;
      uint8_t b = b1 + (b2 - b1) * t;
      colors[i] = (r << 11) | (g << 5) | b;
    }
    float angleOffset = (float)frame * 2 * PI / 50; // Rotate every ~50 frames
    updateLedAnimations();
    drawColorWheel(colors, angleOffset);
    delay(25); // 25 ms per frame, 200 frames = 5 seconds
  }
}

void drawBlackWhiteWheel(float angleOffset) {
  int centerX = 120, centerY = 120;
  int radius = 120;
  int segments = 12;
  // Draw 12 segments, alternating black and white
  for (int i = 0; i < segments; i++) {
    float angle1 = (float)i * 2 * PI / segments + angleOffset;
    float angle2 = (float)(i + 1) * 2 * PI / segments + angleOffset;
    int x1 = centerX + (int)(radius * cos(angle1));
    int y1 = centerY + (int)(radius * sin(angle1));
    int x2 = centerX + (int)(radius * cos(angle2));
    int y2 = centerY + (int)(radius * sin(angle2));
    uint16_t color = (i % 2 == 0) ? COLOR_BLACK : COLOR_WHITE;
    updateLedAnimations();
    tft1.fillTriangle(centerX, centerY, x1, y1, x2, y2, color);
  }
}

void animateBlackWhiteWheel() {
  int totalFrames = 200; // 5 seconds / 25 ms per frame
  Serial.println("Starting black-white wheel animation");
  updateLedAnimations();
  tft1.fillRect(0, 0, 240, 240, COLOR_BLACK);

  // Animate black-white wheel
  for (int frame = 0; frame < totalFrames; frame++) {
    float angleOffset = (float)frame * 2 * PI / 50; // Rotate every ~50 frames
    updateLedAnimations();
    drawBlackWhiteWheel(angleOffset);
    delay(25); // 25 ms per frame, 200 frames = 5 seconds
  }
  Serial.println("Black-white wheel animation completed");
}

void drawHeart(int sizePct, uint16_t color) {
  int centerX = 120, centerY = 120; // Centered
  int radius = 120;
  int size = radius * 2 * sizePct / 100; // Total width/height of heart
  int y = centerY - size / 2;

  // Clear region
  updateLedAnimations();
  tft1.fillRect(centerX - radius - 5, centerY - radius - 5, radius * 2 + 10, radius * 2 + 10, COLOR_BLACK);

  // Draw heart (point down, 180° rotation)
  int w = size; // Width of heart
  int h = size; // Height of heart
  int rectWidth = w / 2; // Each rectangle's width
  int rectHeight = h * 2 / 3; // Rectangle height
  int lobeRadius = rectWidth / 2; // Circle diameter = rectangle width

  // Draw two rectangles tilted ±45°
  float rad = 45 * PI / 180; // 45° tilt
  // Bottom point of heart
  int bottomX = centerX;
  int bottomY = y + h;
  // Left rectangle vertices (tilted +45°)
  int lx1 = bottomX - (int)(rectHeight * cos(rad)); // Left point
  int ly1 = bottomY - (int)(rectHeight * sin(rad));
  int lx2 = lx1 + rectWidth; // Right point
  int ly2 = ly1;
  int lx3 = bottomX; // Bottom point
  int ly3 = bottomY;
  // Right rectangle vertices (tilted -45°)
  int rx1 = bottomX; // Bottom point
  int ry1 = bottomY;
  int rx2 = bottomX + (int)(rectHeight * cos(rad)); // Right point
  int ry2 = bottomY - (int)(rectHeight * sin(rad));
  int rx3 = rx2 - rectWidth; // Left point
  int ry3 = ry2;
  // Draw left rectangle
  updateLedAnimations();
  tft1.fillTriangle(lx1, ly1, lx2, ly2, lx3, ly3, color);
  // Draw right rectangle
  tft1.fillTriangle(rx1, ry1, rx2, ry2, rx3, ry3, color);
  // Draw circles at top center of rectangles
  tft1.fillCircle(lx1 + rectWidth / 2, ly1, lobeRadius, color); // Left lobe
  tft1.fillCircle(rx2 - rectWidth / 2, ry2, lobeRadius, color); // Right lobe
}

void animateHeart() {
  int centerX = 120, centerY = 120;
  int radius = 120;
  Serial.println("Starting heart animation");

  // Tilt head left at the start of the animation
  moveServoSmoothly(servoTilt, TILT_MIDDLE, TILT_LEFT, 1200);

  // Grow from 5% to 40% (1200 ms)
  for (int sizePct = 5; sizePct <= 40; sizePct += 3) {
    drawHeart(sizePct, COLOR_RED);
    updateLedAnimations();
    delay(80); // 15 steps * 80 ms = 1200 ms
  }
  // Blink 10 times (4000 ms)
  for (int i = 0; i < 10; i++) {
    tft1.fillRect(centerX - radius - 5, centerY - radius - 5, radius * 2 + 10, radius * 2 + 10, COLOR_BLACK);
    delay(200); // 200 ms off
    updateLedAnimations();
    drawHeart(40, COLOR_RED);
    delay(200); // 200 ms on
  }
  // Fade to black (3200 ms)
  for (int r = 31; r >= 0; r--) {
    uint16_t color = (r << 11);
    updateLedAnimations();
    drawHeart(40, color);
    delay(100); // 32 steps * 100 ms = 3200 ms
  }

  // Return head to center
  moveServoSmoothly(servoTilt, TILT_LEFT, TILT_MIDDLE, 3200);

  Serial.println("Heart animation completed");
}

void randomHeadMovement() {
  // Randomly decide how many servos to move (1, 2, or 3)
  int servoCount = random(1, 4); // 1 to 3 servos
  int targetUpDown = currentUpDown;
  int targetLeftRight = currentLeftRight;
  int targetTilt = currentTilt;

  // Define smaller range for idle movements (50% of full range)
  int upDownRange = (LOOK_DOWN - LOOK_UP) / 4; // 25% of up/down range
  int leftRightRange = (TURN_LEFT - TURN_RIGHT) / 3.5; // 25% of left/right range
  int tiltRange = (TILT_RIGHT - TILT_LEFT) / 2.5; // 25% of tilt range

  // Randomly select targets for active servos
  if (servoCount >= 1) {
    targetUpDown = LOOK_MIDDLE + random(-upDownRange, upDownRange + 1);
  }
  if (servoCount >= 2) {
    targetLeftRight = TURN_MIDDLE + random(-leftRightRange, leftRightRange + 1);
  }
  if (servoCount == 3) {
    targetTilt = TILT_MIDDLE + random(-tiltRange, tiltRange + 1);
  }

  // Move servos to random positions (500ms)
  moveServosSmoothly(servoUpDown, currentUpDown, targetUpDown, 
                     servoLeftRight, currentLeftRight, targetLeftRight, 
                     servoTilt, currentTilt, targetTilt, 500);

  // Update current positions
  currentUpDown = targetUpDown;
  currentLeftRight = targetLeftRight;
  currentTilt = targetTilt;
}


void animateScanning() {
  Serial.println("Starting scanning animation: 5 left-to-right passes with up/down adjustments");

  // Define 60% of left/right range for scanning
  int targetLeft = TURN_MIDDLE + (TURN_LEFT - TURN_MIDDLE) * 0.6;
  int targetRight = TURN_MIDDLE - (TURN_MIDDLE - TURN_RIGHT) * 0.6;
  // Define 10% of up/down range for slight adjustments
  int upDownStep = (LOOK_DOWN - LOOK_UP) / 10; // 10% of up/down range

  // Start at middle position
  moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, 500);
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;

  // Perform 5 passes
  for (int i = 0; i < 5; i++) {
    // Calculate slight up/down adjustment (+/- 10% range, alternating)
    int targetUpDown = LOOK_MIDDLE + (i % 2 == 0 ? 1 : -1) * upDownStep;

    // Move to left
    moveServosSmoothly(servoUpDown, currentUpDown, targetUpDown, 
                       servoLeftRight, currentLeftRight, targetLeft, 
                       servoTilt, currentTilt, TILT_MIDDLE, 2000);
    currentUpDown = targetUpDown;
    currentLeftRight = targetLeft;

    // Move to right
    moveServosSmoothly(servoUpDown, currentUpDown, targetUpDown, 
                       servoLeftRight, currentLeftRight, targetRight, 
                       servoTilt, currentTilt, TILT_MIDDLE, 2000);
    currentLeftRight = targetRight;
  }

  // Return to middle
  moveServosSmoothly(servoUpDown, currentUpDown, LOOK_MIDDLE, 
                     servoLeftRight, currentLeftRight, TURN_MIDDLE, 
                     servoTilt, currentTilt, TILT_MIDDLE, 500);
  currentUpDown = LOOK_MIDDLE;
  currentLeftRight = TURN_MIDDLE;
  currentTilt = TILT_MIDDLE;

  Serial.println("Scanning animation completed");
}

void blinkEye() {
  int centerX = 120, centerY = 120;
  int radius = 120;
  // Close eye
  for (int h = 100; h >= 0; h -= 10) {
    int visibleH = radius * 2 * h / 100;
    int prevH = (h == 100) ? 240 : radius * 2 * (h + 10) / 100;
    int yStart = centerY - visibleH / 2;
    int prevYStart = centerY - prevH / 2;
    int deltaH = prevH - visibleH;
    tft1.fillRect(centerX - radius - 5, prevYStart - 5, radius * 2 + 10, deltaH / 2 + 10, COLOR_BLACK);
    tft1.fillRect(centerX - radius - 5, yStart + visibleH - 5, radius * 2 + 10, deltaH / 2 + 10, COLOR_BLACK);
    tft1.fillRect(centerX - radius, yStart, radius * 2, visibleH, COLOR_WHITE);
    delay(4); // 4 ms per frame
  }
  delay(8); // Pause at closure
  // Open eye
  for (int h = 0; h <= 100; h += 10) {
    int visibleH = radius * 2 * h / 100;
    int yStart = centerY - visibleH / 2;
    tft1.fillRect(centerX - radius, yStart, radius * 2, visibleH, COLOR_WHITE);
    delay(4);
  }
  Serial.println("Blink completed");
}

void showNormalEye() {
  drawEye(COLOR_WHITE, 100, true);
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  //delay(100);
  Serial.println("Starting GC9A01 dual blinking eye with animated expressions and servo control test...");

  // Initialize Software Serial
  dfPlayerSerial.begin(9600);
  Serial.println("Initializing DFPlayer Mini...");
  dfPlayer.begin(dfPlayerSerial, false); // false to avoid blocking if DFPlayer not detected
  dfPlayer.volume(20); // Set volume (0 to 30)
  Serial.println("DFPlayer Mini initialized");
  
  // Initialize SPI bus
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1); // SCLK, MISO, MOSI, no CS
  Serial.println("SPI bus initialized");

  // Initialize pins
  pinMode(TFT_CS1, OUTPUT);
  digitalWrite(TFT_CS1, HIGH);
  pinMode(TFT_DC1, OUTPUT);
  digitalWrite(TFT_DC1, HIGH);

  pixels.begin();
  pixels.setBrightness(255); // Set to 50/255 to avoid excessive brightness
  pixels.show(); // Initialize LED off
  
  // Initialize Servos
  Serial.println("Using pins: D4 (GPIO6), D5 (GPIO7), D6 (GPIO21)");
  servoTilt.attach(SERVO_TILT_PIN, 1000, 2000);
  servoUpDown.attach(SERVO_UP_DOWN_PIN, 1000, 2000);
  servoLeftRight.attach(SERVO_LEFT_RIGHT_PIN, 1000, 2000);
  servoLeftEar.attach(SERVO_LEFT_EAR_PIN, 1000, 2000);
  servoRightEar.attach(SERVO_RIGHT_EAR_PIN, 1000, 2000);
    // Move servos smoothly to middle positions
  Serial.println("Moving servos to middle positions...");
  moveServoSmoothly(servoUpDown, 1500, LOOK_MIDDLE, 2000); // Assume start at 1500us
  moveServoSmoothly(servoLeftRight, 1500, TURN_MIDDLE, 2000); // Assume start at 1500us
  moveServoSmoothly(servoTilt, 1500, TILT_MIDDLE, 2000); // Assume start at 1500us
  moveServoSmoothly(servoLeftEar, 1500, LEFT_EAR_UP, 2000);
  moveServoSmoothly(servoRightEar, 1500, RIGHT_EAR_UP, 2000);
  Serial.println("Servos moved to middle positions");
    
  // Initialize Displays
  Serial.println("Initializing Display 1...");
  tft1.begin();
  tft1.init();
  tft1.setRotation(2); // Rotate 180 degrees
  tft1.fillScreen(COLOR_BLACK);
  Serial.println("Display 1 initialized");

  // Initial eyes
  showNormalEye();
  randomSeed(analogRead(0));
  lastBlink = millis();
  nextBlinkInterval = 5000 + random(-1500, 1500);
  lastEmotion = millis();
  nextEmotionInterval = 75000 + random(-15000, 15000);

  // Seed random number generator for better randomness
  randomSeed(analogRead(A0));
}

void loop() {
  unsigned long now = millis();

  // Handle blinking when in normal state
  if (emotionState == -1 && now - lastBlink >= nextBlinkInterval) {
    blinkEye();
    lastBlink = now;
    nextBlinkInterval = 5000 + random(-1500, 1500);
  }

  // Play a random sound at a random time
  if (now - lastSoundTime >= nextSoundInterval) {
    int track = random(1, 24); // Select a random track (1 to 24)
    dfPlayer.play(track); // Play the track
    Serial.print("Playing track: ");
    Serial.println(track);
    lastSoundTime = now;    
    nextSoundInterval = random(500, 5500);
  }

  // Handle constant head movement during non-blinking, non-animation period
  if (emotionState == -1 && !animationActive && now - lastBlink < nextBlinkInterval) {
    constantHeadAndEarMovement();
  }

  // Start new expression
  if (emotionState == -1 && !animationActive && now - lastEmotion >= nextEmotionInterval) {
    animationActive = true;
    emotionState = nextEmotionState;
    if (emotionState != 0 && emotionState != 1 && emotionState != 2 && emotionState != 3 && emotionState != 5) {
      tft1.fillScreen(COLOR_BLACK);
    }
    emotionStartTime = now;

    if (emotionState == 0) {
      ledFlashRed();
      animateScowlEye(); // ~11 seconds
      ledOff();
      Serial.println("Scowl animation triggered");
    } else if (emotionState == 1) {
      ledBlue();
      animateSurprisedEye(); // 20 seconds
      ledOff();
      Serial.println("Surprised animation triggered");
    } else if (emotionState == 2) {
      ledRainbowCycle();
      animateSpiral(); // 10 seconds
      ledOff();
      Serial.println("Spiral animation triggered");
    } else if (emotionState == 3) {
      ledRainbowCycle();
      animateColorWheel(); // 5 seconds
      ledOff();
      Serial.println("Color wheel animation triggered");
    } else if (emotionState == 4) {
      ledFlashRed();
      animateHeart(); // ~6.4 seconds
      ledOff();
      Serial.println("Heart animation triggered");
    } else if (emotionState == 5) {
      ledFlashWhite();
      animateBlackWhiteWheel(); // 5 seconds
      ledOff();
      Serial.println("Black-white wheel animation triggered");
    }
  }

  // Return to normal eye after expression duration
  if (animationActive && now - emotionStartTime >= (emotionState == 0 ? 11000 : (emotionState == 1 ? 20000 : (emotionState == 2 ? 10000 : (emotionState == 4 ? 6400 : 5000))))) {
    Serial.println("Returning to normal eye, setting emotionState to -1, next will be: " + String((nextEmotionState + 1) % 6));
    tft1.fillScreen(COLOR_BLACK);
    showNormalEye();
    lastEmotion = now;
    nextEmotionInterval = 65000 + random(-15000, 15000);
    Serial.println("Next emotion scheduled in " + String(nextEmotionInterval) + " ms");
    emotionState = -1; // Return to normal state
    nextEmotionState = (nextEmotionState + 1) % 6; // Increment for next animation
    animationActive = false; // Allow next animation
  }
  updateLedAnimations();
}
