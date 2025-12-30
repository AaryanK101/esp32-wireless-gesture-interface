#include <Wire.h>       // I2C communication (MPU6050)
#include <WiFi.h>       // WiFi required for ESP-NOW
#include <esp_now.h>    // ESP-NOW protocol

// ==================== MPU6050 ====================
const uint8_t MPU_ADDR = 0x68; 
// Default MPU6050 I2C address (0x69 if AD0 pin pulled high)

// ==================== TUNING CONSTANTS ====================
const int THRESH = 8000;           // Minimum tilt magnitude to count as a gesture
const int COOLDOWN_MS = 100;       // Minimum time between packets
const int SAMPLE_DELAY_MS = 20;    // Loop sampling rate

// ==================== ESP-NOW PEER ====================
// MAC address of receiver ESP32 (found using WiFi scan/tester code)
uint8_t peerMac[] = {0x84, 0x1F, 0xE8, 0x29, 0x96, 0xA8};

// ==================== DATA PACKET ====================
// Small, fixed-size packet for fast ESP-NOW transmission
typedef struct __attribute__((packed)) {
  int8_t dir;     // 1=RIGHT, 2=LEFT, 3=FORWARD, 4=BACKWARD
  int16_t mag;    // Raw magnitude of tilt (strength of gesture)
} GesturePacket;

GesturePacket pkt;

// ==================== MPU6050 SETUP ====================
// Wakes MPU6050 from sleep mode
void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);     // Power management register
  Wire.write(0x00);     // Set to zero
  Wire.endTransmission();
}

// ==================== ACCELEROMETER READ ====================
// Reads X and Y acceleration values from MPU6050
// Returns false if I2C communication fails
bool readAccelXY(int16_t &ax, int16_t &ay) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  // 0x3B = ACCEL_XOUT_H -> start of X/Y accel data

  // Send register pointer, keep bus active (repeated start)
  if (Wire.endTransmission(false) != 0) return false;

  // Request 4 bytes: X_H, X_L, Y_H, Y_L
  if (Wire.requestFrom((int)MPU_ADDR, 4, true) != 4) return false;

  // Combine high + low bytes into signed 16-bit values
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();

  return true;
}

// ==================== ESP32 SETUP ====================
void setupEspNow() {
  WiFi.mode(WIFI_STA);   // Required for ESP32

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Define peer (receiver ESP32)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;      // Same channel
  peerInfo.encrypt = false; // No encryption (simpler, faster)

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    Serial.println("Failed to add peer");
  else
    Serial.println("Peer added");
}

// ==================== SEND PACKET ====================
void sendPacket(const GesturePacket &p) {
  esp_err_t result = esp_now_send(peerMac, (uint8_t *)&p, sizeof(GesturePacket));

  if (result == ESP_OK) Serial.println("Sent OK");
  else Serial.println("Send FAIL");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1500);   // Allow time for Serial Monitor

  // Initialise I2C (ESP32 default pins)
  Wire.begin(21, 22);
  Wire.setClock(100000);  // Standard 100kHz I2C
  wakeMPU();

  setupEspNow();
  Serial.println("ESP32 A ready. Sending dir + magnitude...");
}

// ==================== MAIN LOOP ====================
void loop() {
  static unsigned long lastSend = 0;     // Timestamp of last sent packet
  static int8_t lastDir = 0;             // Previously sent direction
  static int16_t lastMagBucket = -1;     // Previously sent magnitude bucket

  int16_t ax, ay;

  // Read accelerometer; stop if read fails
  if (!readAccelXY(ax, ay)) {
    Serial.println("MPU read failed");
    delay(200);
    return;
  }

  int8_t dir = 0;
  int16_t mag = 0;

  // Determine dominant axis
  // Whichever axis has larger absolute value defines gesture
  // ax > 0 -> LEFT, ax < 0 -> RIGHT
  // ay < 0 -> FORWARD, ay > 0 -> BACKWARD
  if (abs(ax) > abs(ay)) {
    mag = abs(ax);
    if (mag > THRESH)
      dir = (ax > 0) ? 2 : 1;
  } else {
    mag = abs(ay);
    if (mag > THRESH)
      dir = (ay < 0) ? 3 : 4;
  }

  // No valid gesture -> do nothing
  if (dir == 0) {
    delay(SAMPLE_DELAY_MS);
    return;
  }

  // ==================== MAGNITUDE BUCKETING ====================
  // Convert raw magnitude into discrete levels (1..7)
  // This prevents wireless spam from tiny fluctuations
  int16_t magClamped = mag;
  if (magClamped > 20000) magClamped = 20000;

  int16_t bucket = map(magClamped, THRESH, 20000, 1, 7);
  if (bucket < 1) bucket = 1;
  if (bucket > 7) bucket = 7;

  // ==================== SEND CONDITIONS ====================
  // Send only if:
  // - cooldown passed
  // - direction changed OR strength changed meaningfully
  if (millis() - lastSend > COOLDOWN_MS && (dir != lastDir || bucket != lastMagBucket)) {

    lastSend = millis();
    lastDir = dir;
    lastMagBucket = bucket;

    pkt.dir = dir;
    pkt.mag = mag;  // Send raw magnitude

    // Debug output
    if (dir == 1) Serial.println("RIGHT");
    else if (dir == 2) Serial.println("LEFT");
    else if (dir == 3) Serial.println("FORWARD");
    else if (dir == 4) Serial.println("BACKWARD");

    sendPacket(pkt);
  }

  delay(SAMPLE_DELAY_MS);
}
