/*
 * ESP32 MAC Address Finder
 * Flash this sketch to your Receiver (Motor Driver ESP32)
 * Open Serial Monitor at 115200 baud to read its MAC address.
 */
#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  
  Serial.println("\n===========================================");
  Serial.println("       ESP32 Board MAC Address Finder      ");
  Serial.println("===========================================");
  Serial.print("ESP32 Station MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.println("\nCopy & Paste directly into ESP32_CAM_V10.ino:");
    Serial.printf("uint8_t receiverAddress[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n",
                  baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  }
  Serial.println("===========================================\n");
}

void loop() {
  // Idle
}
