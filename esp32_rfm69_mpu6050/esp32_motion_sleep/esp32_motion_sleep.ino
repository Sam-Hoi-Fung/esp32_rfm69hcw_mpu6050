/*
esp32s3 devkit
MOSI: 11
MISO: 13
SCK: 12
SS: 10

SDA: 8
SCL: 9

pinout
rfm69hcw - esp32s3 devkit
G - G
MISO - 13 
MOSI - 11
SCK - 12
NSS - 10
RST - 38
DIO0 - 4
3.3V - 3.3V

MPU6050 - esp32s3 devkit
VCC - 3.3V
GND - GND
SDA - 8
SCL - 9
INT - 1
*/

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
  mpu.setInterruptPinPolarity(false);  // Set to low for interrupt
  mpu.setMotionInterrupt(true);
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

  //Serial.println("Feather RFM69 TX Test!");
  //Serial.println();

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

  //Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // If woke up due to motion interrupt, read and send sensor data
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    unsigned long startMillis = millis();
    unsigned long currentMillis = startMillis;

    while (currentMillis - startMillis <= 15000) {  // Run for 15 seconds
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

      currentMillis = millis();
      delay(100);  // Read every 0.1 second
    }
  }

  // Setup ESP32 to wake up on MPU6050 motion interrupt (external trigger)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_1, 1); // 0 = Low level to wake up
  Serial.println("Setup ESP32 to wake up on MPU6050 motion interrupt");

  // Go to sleep now
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // This will never be called
}