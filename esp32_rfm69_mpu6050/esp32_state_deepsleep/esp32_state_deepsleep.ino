#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "esp_sleep.h"
#include <SPI.h>
#include <RH_RF69.h>

Adafruit_MPU6050 mpu;

// GPIO pin connected to the MPU6050 interrupt pin
#define MPU_INT_PIN 1

// RFM69 Radio Setup
#define RF69_FREQ 434.0

// ESP32 feather w/wing
#define RFM69_CS   10  // "B"
#define RFM69_INT  4  // "A"
#define RFM69_RST  38  // same as LED

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// State machine states
enum State {
  MOVE,
  STOP
};

State currentState = STOP;
unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 3000;  // 3 seconds

/*void IRAM_ATTR handleMotionInterrupt() {
  // Update last motion time whenever motion is detected
  lastMotionTime = millis();
}*/

void setupMPU6050() {
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched. Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);  // Set to high for interrupt
  mpu.setMotionInterrupt(true);

  /*pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), handleMotionInterrupt, FALLING);*/
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Take some time to open up the Serial Monitor

  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Setup MPU6050 for motion detection
  setupMPU6050();

  // Initialize RFM69HCW
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // Manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  // If woke up due to motion interrupt, enter MOVE state
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    currentState = MOVE;
    lastMotionTime = millis();
  } else {
    currentState = STOP;
  }

  // Setup ESP32 to wake up on MPU6050 motion interrupt (external trigger)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_1, 0); // 0 = Low level to wake up
  Serial.println("Setup ESP32 to wake up on MPU6050 motion interrupt");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentState == MOVE) {
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Create a message with the IMU data
    char radiopacket[60];
    snprintf(radiopacket, sizeof(radiopacket), "Accel: %.2f, %.2f, %.2f Gyro: %.2f, %.2f, %.2f",
             a.acceleration.x, a.acceleration.y, a.acceleration.z,
             g.gyro.x, g.gyro.y, g.gyro.z);

    // Send the message
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();

    // Print out the values
    Serial.println(radiopacket);

    // Check if motion is still detected using MPU6050 interrupt status
    if (mpu.getMotionInterruptStatus()) {
      // Motion is detected, update last motion time
      lastMotionTime = currentMillis;
    } else {
      // No motion detected, check timeout
      if (currentMillis - lastMotionTime > motionTimeout) {
        currentState = STOP;
      }
    }

    delay(100);  // Read every 0.1 second

  } else if (currentState == STOP) {
    // Go to sleep now
    Serial.println("No motion detected, going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
}
