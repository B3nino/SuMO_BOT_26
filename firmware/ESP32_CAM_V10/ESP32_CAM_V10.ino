/*
 * V10 ESP32-CAM LINE DETECTOR (SENDER)
 * Upload Settings: ESP32 Wrover Module, Huge APP (3MB No OTA)
 */
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>

// --- ESP-NOW CONFIGURATION ---
// ⚠️ REPLACE with the MAC Address of your ESP32 MOTOR DRIVER receiver!
// Use the sketch in tools/Get_MAC_Address to get your board's MAC address.
uint8_t receiverAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct struct_message {
  int16_t error;      // Offset of the line from the center
  uint8_t trackState; // 0 = Following, 1 = Lost/Gap, 2 = Intersection (T-junction/Cross)
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- CAMERA PIN SETTINGS (AI-THINKER ESP32-CAM) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4 // Flash LED

// ==========================================
// 🛠️ VISION SETTINGS (EDIT THESE TO TUNE)
// ==========================================
// Camera tracks lower 50% of screen. Frame size: 160(W) x 120(H)
const int ROI_START_Y = 60;  // Start scanning at Y=60 (middle of frame)
const int ROI_END_Y = 120;   // Stop scanning at Y=120 (bottom of frame)
const int FRAME_WIDTH = 160;
const int FRAME_CENTER = 80; // 160 / 2

// Sensitivity thresholds for Line Breaks vs Intersections
const int MIN_PIXELS_FOR_LINE = 150;    // If black pixels < this -> Track lost
const int MAX_PIXELS_FOR_CROSS = 3500;  // If black pixels > this -> Intersection detected
// ==========================================

void setup() {
  Serial.begin(115200);

  // Turn on Flashlight to max brightness!
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW); // momentarily off for boot

  // Configure WiFi as Station for ESP-NOW (No credentials needed!)
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Camera initialization
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Pure black and white reading (High Speed!)
  config.frame_size = FRAMESIZE_QQVGA;       // 160x120 perfect for quick math
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // CAMERA GEOMETRY CONFIGURATION
  sensor_t * s = esp_camera_sensor_get();
  // 🔥 Vertical flip is REQUIRED for upright camera mounts
  s->set_vflip(s, 1); 
  
  // 🔄 **HORIZONTAL MIRROR TOGGLE** 
  // If your bot naturally drives AWAY from the line / reverses the steering commands, 
  // CHANGE THIS TO `1`. This gives you exactly how human eyes perceive right/left.
  s->set_hmirror(s, 0); 
  
  // Crank up flash after initialization prevents cam brownouts
  digitalWrite(FLASH_LED_PIN, HIGH);
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  // --- 1. AUTO-CALIBRATION ALGORITHM (Replaces manual cv2 Threshold value) ---
  int min_luma = 255;
  int max_luma = 0;
  
  // Sample pixels rapidly jumping by 4 to gauge room lighting conditions
  for (int y = ROI_START_Y; y < ROI_END_Y; y += 4) {
    for (int x = 0; x < FRAME_WIDTH; x += 4) {
      uint8_t pixel = fb->buf[y * FRAME_WIDTH + x];
      if (pixel < min_luma) min_luma = pixel;
      if (pixel > max_luma) max_luma = pixel;
    }
  }
  // Adaptive threshold: We want dark line -> 40% higher than the darkest point recorded
  int thresh = min_luma + ((max_luma - min_luma) * 0.40);

  // --- 2. VISION PROCESSING (Contour Math & Morphological Opening) ---
  long sum_x = 0;
  int total_black_pixels = 0;

  for (int y = ROI_START_Y; y < ROI_END_Y; y++) {
    // Start at x=1, end at x=158 to allow neighbor voting checking
    for (int x = 1; x < FRAME_WIDTH - 1; x++) {
      
      // Morphological Pseudo-Open Noise Filtering: 
      // Instead of reading just 1 pixel, check if at least 2 pixels in a row are dark. 
      // Skips single dots of dirt.
      int dark_neighbors = 0;
      if (fb->buf[y * FRAME_WIDTH + (x - 1)] < thresh) dark_neighbors++;
      if (fb->buf[y * FRAME_WIDTH + x]       < thresh) dark_neighbors++;
      if (fb->buf[y * FRAME_WIDTH + (x + 1)] < thresh) dark_neighbors++;

      if (dark_neighbors >= 2) { 
        sum_x += x;
        total_black_pixels++;
      }
    }
  }

  // --- 3. LOGIC DECISION ALGORITHM ---
  if (total_black_pixels < MIN_PIXELS_FOR_LINE) {
    // ⚠️ Track Broken or Obstacle ahead
    myData.trackState = 1; 
    myData.error = 0; // maintain course
  } 
  else if (total_black_pixels > MAX_PIXELS_FOR_CROSS) {
    // ⚠️ Intersection detected (sudden massive patch of black)
    myData.trackState = 2;
    myData.error = 0; // Go straight across it normally
  } 
  else {
    // ✅ Normal Following Logic
    myData.trackState = 0;
    int cx = sum_x / total_black_pixels; // Center of mass
    myData.error = cx - FRAME_CENTER;    // Compute distance from middle (the Error!)
  }

  // Send packet to Motor Driver instantly
  esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));

  // Release buffer
  esp_camera_fb_return(fb);
}