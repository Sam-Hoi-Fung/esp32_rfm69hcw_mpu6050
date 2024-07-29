#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "esp_sleep.h"

// MPU6050 Setup
Adafruit_MPU6050 mpu;

// RFM69 Radio Setup
#define RF69_FREQ 434.0

// ESP32 S3 pins
#define RFM69_CS   10  
#define RFM69_INT  4  
#define RFM69_RST  38  

RH_RF69 rf69(RFM69_CS, RFM69_INT);

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30          /* Time ESP32 will go to sleep (in seconds) */
#define WAKE_TIME      5          /* Time ESP32 will stay awake (in seconds) */ 

// GPIO pin connected to the MPU6050 interrupt pin
#define MPU_INT_PIN 1

RTC_DATA_ATTR int bootCount = 0;

// State machine states
enum State {
  MOVE,
  STOP,
  TIMER
};

State currentState = STOP;
unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 3000;  // 3 seconds

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
  mpu.setInterruptPinPolarity(true);  // Set to low for interrupt
  mpu.setMotionInterrupt(true);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by movement");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

void readAndTransmitIMUData(unsigned long duration) {
  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis;

  while (currentMillis - startMillis <= duration) {  // Run for the specified duration
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Create a message with the IMU data
    char radiopacket[100];
    snprintf(radiopacket, sizeof(radiopacket), "Accel: %.2f, %.2f, %.2f Gyro: %.2f, %.2f, %.2f",
             a.acceleration.x, a.acceleration.y, a.acceleration.z,
             g.gyro.x, g.gyro.y, g.gyro.z);

    // Send the message
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();

    // Print out the values
    Serial.println(radiopacket);

    currentMillis = millis();
    delay(100);  // Read every 0.1 second
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Take some time to open up the Serial Monitor

  // Increment boot number and print it every reboot
  //++bootCount;
  //Serial.println("Boot number: " + String(bootCount));

  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // MPU6050 Initialization
  setupMPU6050();

  // RFM69 Initialization
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

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // Handle wakeup
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    currentState = MOVE;
    lastMotionTime = millis();
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    currentState = TIMER;
  } else {
    currentState = STOP;
  }

  // Setup ESP32 to wake up on timer and MPU6050 motion interrupt
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_1, 0); // 0 = Low level to wake up
  Serial.println("Setup ESP32 to wake up on timer and MPU6050 motion interrupt");
}

void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case MOVE:
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
      break;

    case TIMER:
      Serial.println("Handling timer wakeup");
      readAndTransmitIMUData(WAKE_TIME * 1000);  // Handle timer wakeup for specified wake time
      currentState = STOP;
      break;

    case STOP:
      // Go to sleep now
      Serial.println("No motion detected, going to sleep now");
      Serial.flush();
      esp_deep_sleep_start();
      break;
  }
}
