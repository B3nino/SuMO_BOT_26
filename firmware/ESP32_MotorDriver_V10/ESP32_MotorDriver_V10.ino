/*
 * V10 ESP32 MOTOR DRIVER SCRIPT (RECEIVER) - AUTO-START ON CAMERA SIGNAL
 * Using TB6612FNG Driver configured for N20 500 RPM Motors
 */
#include <WiFi.h>
#include <esp_now.h>

// ─── Motor pins (TB6612FNG) ─────────────────────────────────────────────────
constexpr int PWMA_PIN = 26;   // Right PWM speed control
constexpr int AIN1_PIN = 27;   // Right Direction 1
constexpr int AIN2_PIN = 14;   // Right Direction 2

constexpr int PWMB_PIN = 12;   // Left PWM speed control
constexpr int BIN1_PIN = 13;   // Left Direction 1
constexpr int BIN2_PIN = 15;   // Left Direction 2

constexpr int STBY_PIN = 4;    // Standby pin (Must be HIGH to enable motors)
// ────────────────────────────────────────────────────────────────────────────

// ESP-NOW Variables
typedef struct struct_message {
  int16_t error;
  uint8_t trackState;
} struct_message;

struct_message camData;
volatile bool dataUpdated = false;

// Time tracker to see when we last received a command
volatile unsigned long lastDataTime = 0; 

// ==========================================
// 🛠️ MOVEMENT & PID SETTINGS (EDIT THESE TO TUNE)
// ==========================================
// Base Speed limiter (Max 255) for N20 500rpm motors. 
// Starting slowly at 90 as requested. Max speed capped so it won't outrun the camera.
const int BASE_SPEED = 50;   
const int MAX_SPEED = 60;   

// PID Constants
float Kp = 1.2;   // Proportional reaction
float Kd = 2.0;   // Derivative dampens the oscillations / triggers high speed braking
float Ki = 0.002; // Integral helps center tight curves over time

float previous_error = 0;
float integral = 0;
// ==========================================

// ESP-NOW Receiver Callback
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&camData, incomingData, sizeof(camData));
  dataUpdated = true;
  lastDataTime = millis(); // Record exactly when the camera last spoke to us
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA_PIN, OUTPUT); pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT); pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT); 
  
  digitalWrite(STBY_PIN, HIGH); // ENABLE MOTORS CHIP
  
  // Make absolutely sure motors are stopped at boot!
  motorWrite(0, 0); 

  // Setup WiFi in Station mode to catch ESP-NOW signals
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialize Failed");
    return;
  }
  
  // Register callback
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("System Ready. Awaiting initial commands from ESP32-CAM...");
}

void loop() {
  // -------------------------------------------------------------
  // SIGNAL FAILSAFE:
  // If we have NEVER received data, or if it's been more than 500ms 
  // since the last camera packet, completely freeze the motors.
  // -------------------------------------------------------------
  if (millis() - lastDataTime > 500) {
    motorWrite(0, 0);
    return; // Exit loop here, don't run PID until camera talks again
  }

  // --- Real-time Movement Handling ---
  if (dataUpdated) {
    dataUpdated = false;

    // SCENARIO 1: We lost the line (Track Gap)
    if (camData.trackState == 1) {
      motorWrite(BASE_SPEED, BASE_SPEED);
      return;
    }
    
    // SCENARIO 2: T-Junction or Cross section detected
    if (camData.trackState == 2) {
      motorWrite(BASE_SPEED + 20, BASE_SPEED + 20); 
      return; 
    }

    // SCENARIO 3: Normal Line Following (PID Calculation)
    int error = camData.error;
    
    integral += error; 
    int derivative = error - previous_error;

    // Computations
    float pid_adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    int rightMotorSpeed = BASE_SPEED - pid_adjustment;
    int leftMotorSpeed = BASE_SPEED + pid_adjustment;

    previous_error = error;

    motorWrite(leftMotorSpeed, rightMotorSpeed);
  }
}

// ────────────────────────────────────────────────────────────────────────
// ⚙️ INTELLIGENT HIGH-SPEED MOTOR DRIVER (Supports Hard Braking & Reversing)
// ────────────────────────────────────────────────────────────────────────
void motorWrite(int left_speed, int right_speed) {
  // Right Motor (Motor A) Mapping Logic
  if (right_speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, min(right_speed, MAX_SPEED));
  } else if (right_speed < 0) {
    // ⚠️ HIGH SPEED BRAKING LOGIC
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, min(abs(right_speed), MAX_SPEED));
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, 0);
  }

  // Left Motor (Motor B) Mapping Logic
  if (left_speed > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, min(left_speed, MAX_SPEED));
  } else if (left_speed < 0) {
    // ⚠️ HIGH SPEED BRAKING LOGIC
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, min(abs(left_speed), MAX_SPEED));
  } else {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, 0);
  }
}