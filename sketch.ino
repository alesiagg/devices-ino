#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==== OLED setup ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== IMU setup ====
Adafruit_MPU6050 mpu;

// Step detection variables
unsigned long lastStepTime = 0;
float strideTimes[20]; // store last 20 stride times
int strideIndex = 0;
bool stepDetected = false;

// Parameters
const float stepThreshold = 1.2; // threshold for vertical acceleration (m/s^2)
const int windowSize = 20;

void setup() {
  Serial.begin(115200);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // Initialize IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accZ = a.acceleration.z;

  // Step detection
  if (accZ > stepThreshold && !stepDetected) {
    stepDetected = true;
    unsigned long now = millis();
    if (lastStepTime > 0) {
      float strideTime = (now - lastStepTime) / 1000.0; // seconds
      strideTimes[strideIndex % windowSize] = strideTime;
      strideIndex++;
    }
    lastStepTime = now;
  }
  if (accZ < stepThreshold) {
    stepDetected = false;
  }

  // Compute metrics when enough strides collected
  if (strideIndex >= windowSize) {
    float sum = 0;
    for (int i = 0; i < windowSize; i++) sum += strideTimes[i];
    float meanStride = sum / windowSize;

    float var = 0;
    for (int i = 0; i < windowSize; i++) var += pow(strideTimes[i] - meanStride, 2);
    float strideVar = sqrt(var / windowSize);

    float cadence = 60.0 / meanStride;
    float sway = abs(a.acceleration.x); // simplified lateral sway
    float stabilityIndex = 1.0 / (1.0 + sway);
    float gaitQuality = (0.5 * (1.0 / (1.0 + strideVar))) + (0.5 * stabilityIndex);

    // Print to Serial
    Serial.println("=== Gait Metrics ===");
    Serial.print("Cadence: "); Serial.print(cadence); Serial.println(" steps/min");
    Serial.print("Stride Variability: "); Serial.println(strideVar, 3);
    Serial.print("Stability Index: "); Serial.println(stabilityIndex, 3);
    Serial.print("Gait Quality Index: "); Serial.println(gaitQuality, 3);
    Serial.println("====================");

    // Display on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Gait Metrics:");
    display.print("Cadence: "); display.println(cadence, 1);
    display.print("StrideVar: "); display.println(strideVar, 3);
    display.print("Stability: "); display.println(stabilityIndex, 2);
    display.print("Quality: "); display.println(gaitQuality, 2);
    display.display();

    strideIndex = 0; // reset window
  }

  delay(50); // ~20 Hz sampling
}
