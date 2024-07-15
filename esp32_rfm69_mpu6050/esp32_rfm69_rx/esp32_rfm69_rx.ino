/*
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
*/

#include <SPI.h>
#include <RH_RF69.h>

// Change to 434.0 or other frequency, must match TX's freq!
#define RF69_FREQ 434.0

// ESP32 S3 pins
#define RFM69_CS   10  
#define RFM69_INT  4  
#define RFM69_RST  38  

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  Serial.begin(115200);
  //pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

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

  rf69.setTxPower(20, true);  // Range from 14-20 for power, 2nd arg must be true for 69HCW

  // Encryption key (must match TX)
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @ ");  
  Serial.print((int)RF69_FREQ);  
  Serial.println(" MHz");
}

void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.recv(buf, &len)) {
      buf[len] = 0;  // Null-terminate string
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "Ack";
      rf69.send(data, sizeof(data));
      rf69.waitPacketSent();
      //Serial.println("Sent a reply");
    } else {
      Serial.println("Receive failed");
    }
  }
}
