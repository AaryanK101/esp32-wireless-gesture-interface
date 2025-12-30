#include <WiFi.h>              // Needed for WiFi mode + ESP-NOW
#include <esp_now.h>           // ESP-NOW wireless protocol (peer-to-peer)
#include <Wire.h>              // I2C (LCD uses I2C)
#include <LiquidCrystal_I2C.h> // LCD library (I2C LCD)

#include <MD_MAX72xx.h>        // LED matrix driver library
#include <SPI.h>               // SPI bus (MAX7219 matrix uses SPI)

// ==================== LEDS ====================
// 7 discrete LEDs wired to these GPIO pins
int leds[] = {12, 14, 27, 26, 25, 33, 32};
const int NUM_LEDS = 7;

// ==================== LCD ====================
// I2C LCD at address 0x27, 16 columns x 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== MATRIX ====================
// MAX7219 LED matrix wiring
#define MATRIX_CLK 19
#define MATRIX_CS  18
#define MATRIX_DIN 23

// FC16_HW is the common 8x8 MAX7219 module wiring layout
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 1  // number of chained 8x8 modules
MD_MAX72XX mx(HARDWARE_TYPE, MATRIX_DIN, MATRIX_CLK, MATRIX_CS, MAX_DEVICES);

// ==================== SERVO ====================
const int servoPin = 4;  // servo signal wire connected to GPIO 4

// Convert servo pulse width (microseconds) into PWM duty cycle
// Servo expects 50Hz signal with pulse 500–2500 us
void writeServoMicroseconds(int us) {
  // 50Hz period = 20,000 us
  // 16-bit PWM duty range = 0..65535
  // duty = (pulse_us / 20000us) * 65535
  uint32_t duty = (uint32_t)us * 65535 / 20000;

  // Send duty to PWM output
  ledcWrite(servoPin, duty);
}

// Calibrated servo endpoints (these match YOUR servo’s real motion)
const int SERVO_BACK_US = 2200;   // full BACKWARD position pulse width
const int SERVO_FWD_US = 300;    // full FORWARD position pulse width
const int SERVO_NEUTRAL_US = (SERVO_BACK_US + SERVO_FWD_US) / 2; // centre pulse

// Magnitude thresholds that control when servo starts moving and when it saturates
const int SERVO_MAG_START = 5000;    // below this, keep servo neutral
const int SERVO_MAG_FULL  = 16000;   // at/above this, servo reaches full travel

// Moves servo only for FORWARD/BACKWARD gestures, scaled by magnitude
void updateServoFromForwardBack(int dir, int16_t mag) {
  int m = abs(mag);  // magnitude should be positive strength

  // If gesture isn't strong enough, keep servo centred
  if (m <= SERVO_MAG_START) {
    writeServoMicroseconds(SERVO_NEUTRAL_US);
    return;
  }

  // Cap strength so the servo doesn't keep increasing beyond your chosen max
  if (m > SERVO_MAG_FULL) m = SERVO_MAG_FULL;

  // Normalise magnitude into 0..1 range
  float t = (float)(m - SERVO_MAG_START) /
            (float)(SERVO_MAG_FULL - SERVO_MAG_START);  // 0..1

  int us = SERVO_NEUTRAL_US;

  if (dir == 3) {
    // FORWARD: move from neutral toward SERVO_FWD_US as strength increases
    us = SERVO_NEUTRAL_US + t * (SERVO_FWD_US - SERVO_NEUTRAL_US);
  }
  else if (dir == 4) {
    // BACKWARD: move from neutral toward SERVO_BACK_US as strength increases
    us = SERVO_NEUTRAL_US - t * (SERVO_NEUTRAL_US - SERVO_BACK_US);
  }

  writeServoMicroseconds(us);
}

// ==================== ARROWS ====================
// 8x8 bitmaps for arrows (each row is 8 bits)
// These are defined in a "human orientation" then rotated for my module
const uint8_t ARROW_LEFT[8] = {
  0b00011000,
  0b00011100,
  0b00011110,
  0b11111111,
  0b11111111,
  0b00011110,
  0b00011100,
  0b00011000
};

const uint8_t ARROW_RIGHT[8] = {
  0b00011000,
  0b00111000,
  0b01111000,
  0b11111111,
  0b11111111,
  0b01111000,
  0b00111000,
  0b00011000
};

const uint8_t ARROW_UP[8] = {
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000
};

const uint8_t ARROW_DOWN[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000
};

// ==================== ESP-NOW PACKET ====================
// Must match transmitter struct exactly (same types, same order)
typedef struct __attribute__((packed)) {
  int8_t dir;     // 1=RIGHT 2=LEFT 3=FORWARD 4=BACKWARD
  int16_t mag;    // magnitude sent by ESP-A
} GesturePacket;

// Values written inside interrupt/callback -> mark volatile so compiler doesn't optimise wrongly
volatile int8_t  lastDir = 0;
volatile int16_t lastMag = 0;

// ==================== ESP-NOW RECEIVE CALLBACK ====================
// This runs whenever a packet arrives
void onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  // Ignore anything that isn't exactly our expected packet size
  if (len != sizeof(GesturePacket)) return;

  // Copy raw bytes into a GesturePacket struct
  GesturePacket p;
  memcpy(&p, data, sizeof(p));

  // Store latest values for the main loop to use
  lastDir = p.dir;
  lastMag = p.mag;
}

// ==================== LCD DISPLAY ====================
// Shows a simple 2-line message: label on line 1, direction on line 2
void lcdShow(const char* txt) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Direction:");
  lcd.setCursor(0,1);
  lcd.print(txt);
}

// ==================== MATRIX DISPLAY ====================
// My matrix is rotated physically vs the bitmap orientation
// This rotates an 8x8 bitmap 90 degrees clockwise
void rotate90CW(const uint8_t in[8], uint8_t out[8]) {
  memset(out, 0, 8);

  // For each pixel (r,c) in input, map it to rotated output
  for (int r=0;r<8;r++)
    for (int c=0;c<8;c++)
      if (in[r] & (1<<(7-c)))          // check if pixel is ON in input
        out[c] |= (1<<(7-r));          // set rotated pixel
}

// Show a bitmap on the LED matrix
void matrixShow(const uint8_t bmp[8]) {
  uint8_t rot[8];
  rotate90CW(bmp, rot);   // rotate to match your hardware orientation

  mx.clear();

  // Set each pixel
  for (int r=0;r<8;r++)
    for (int c=0;c<8;c++)
      mx.setPoint(r, c, (rot[r]>>(7-c)) & 1);

  mx.update();  // push changes to the display
}

// ==================== LED INTENSITY FROM MAGNITUDE ====================
// Each threshold corresponds to turning on another LED (strength meter)
const int16_t LEVELS[NUM_LEDS] = {5000,9000,11000,13000,13800,14500,15000};

// Convert magnitude -> number of LEDs to turn on (0..NUM_LEDS)
int magToLedCount(int16_t mag) {
  mag = abs(mag);

  int count = 0;
  for (int i=0;i<NUM_LEDS;i++) {
    if (mag >= LEVELS[i]) count++;
    else break;  // stop once magnitude is below the next threshold
  }
  return count;
}

// Light LEDs depending on direction + strength
// RIGHT: light from left side up (index 0 upward)
// LEFT:  light from right side down (reverse order)
void setLeds(int dir, int16_t mag) {
  int n = magToLedCount(mag);

  // Clear all LEDs first
  for (int i=0;i<NUM_LEDS;i++) digitalWrite(leds[i], LOW);

  if (dir == 1) {
    // RIGHT: 0..n-1
    for (int i=0;i<n;i++) digitalWrite(leds[i], HIGH);
  }

  if (dir == 2) {
    // LEFT: NUM_LEDS-1 downwards
    for (int i=0;i<n;i++) digitalWrite(leds[NUM_LEDS-1-i], HIGH);
  }

  // FORWARD/BACKWARD do not use this LED bar logic in your current design
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  // Configure LED pins
  for (int i=0;i<NUM_LEDS;i++) pinMode(leds[i], OUTPUT);

  // Start I2C bus (shared by LCD here)
  Wire.begin(21,22);

  // LCD init
  lcd.init();
  lcd.backlight();
  lcdShow("Waiting...");

  // Matrix init
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 8);        // brightness (0..15)
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);

  // Servo PWM init
  // 50Hz frequency, 16-bit resolution
  ledcAttach(servoPin, 50, 16);
  writeServoMicroseconds(SERVO_NEUTRAL_US);    // start centred

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onReceive);         // link callback
}

// ==================== LOOP ====================
void loop() {
  static int lastShown = -1;  // last direction shown on LCD/matrix (avoid flicker)

  // Copy volatile values into normal variables once per loop
  // This avoids reading half-updated values mid-loop
  int dir = lastDir;
  int16_t mag = lastMag;

  // Only update LCD + matrix if direction changed
  if (dir != lastShown) {
    lastShown = dir;

    if (dir == 1) { lcdShow("RIGHT");    matrixShow(ARROW_RIGHT); }
    if (dir == 2) { lcdShow("LEFT");     matrixShow(ARROW_LEFT); }
    if (dir == 3) { lcdShow("FORWARD");  matrixShow(ARROW_UP); }
    if (dir == 4) { lcdShow("BACKWARD"); matrixShow(ARROW_DOWN); }
  }

  // Continuously update LEDs and servo based on latest magnitude
  setLeds(dir, mag);
  updateServoFromForwardBack(dir, mag);

  delay(30);  // small loop delay to reduce flicker + CPU load
}
