#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1500);                 // gives time to open Serial Monitor
  Serial.println("\nBOOT OK");

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 B MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Will reprint MAC every 2 seconds...");
}

void loop() {
  Serial.print("ESP32 B MAC: ");
  Serial.println(WiFi.macAddress());
  delay(2000);
}
