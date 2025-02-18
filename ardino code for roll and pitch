#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int buttonPin = 2;
volatile bool zeroingRequested = false;
volatile unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 200;  // milliseconds

float rollZero = 0, pitchZero = 0;

float rollFiltered = 0, pitchFiltered = 0;
const float filterAlpha = 0.98;
const float toleranceThreshold = 1.0;

unsigned long lastTime = 0;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void calibrateGyro() {
  const int numSamples = 500;
  long sumX = 0, sumY = 0, sumZ = 0;
  digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED during calibration
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }
  gyroOffsetX = sumX / (float)numSamples;
  gyroOffsetY = sumY / (float)numSamples;
  gyroOffsetZ = sumZ / (float)numSamples;
  digitalWrite(LED_BUILTIN, LOW);  // Calibration complete
}

void zeroingButtonPressed() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonPress > debounceDelay) {
    zeroingRequested = true;
    lastButtonPress = currentTime;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), zeroingButtonPressed, FALLING);

  delay(100);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  calibrateGyro();
  lastTime = millis();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float accelRoll = atan2(ay_g, az_g) * 180.0 / PI;
  float accelPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

  rollFiltered = accelRoll;
  pitchFiltered = accelPitch;
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float gyroRollRate = (gx - gyroOffsetX) / 65.5;
  float gyroPitchRate = (gy - gyroOffsetY) / 65.5;

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  if (abs(ax_g) <= 1.5 && abs(ay_g) <= 1.5 && abs(az_g) <= 1.5) {
    float accelRoll = atan2(ay_g, az_g) * 180.0 / PI;
    float accelPitch = 0;
    if (az_g != 0 && (ay_g * ay_g + az_g * az_g) > 0) {
      accelPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;

    rollFiltered = filterAlpha * (rollFiltered + gyroRollRate * dt) + (1 - filterAlpha) * accelRoll;
    pitchFiltered = filterAlpha * (pitchFiltered + gyroPitchRate * dt) + (1 - filterAlpha) * accelPitch;

    lastTime = currentTime;

    if (zeroingRequested) {
      rollZero = rollFiltered;
      pitchZero = pitchFiltered;
      zeroingRequested = false;
      Serial.println("Zeroing complete!");
      delay(500);
    }

    float adjustedRoll = (abs(rollFiltered - rollZero) > toleranceThreshold) ? (rollFiltered - rollZero) : 0;
    float adjustedPitch = (abs(pitchFiltered - pitchZero) > toleranceThreshold) ? (pitchFiltered - pitchZero) : 0;

    Serial.print(adjustedRoll);
    Serial.print(",");
    Serial.println(adjustedPitch);
  }

  delay(33);
}
