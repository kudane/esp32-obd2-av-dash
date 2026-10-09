#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <ESP_8_BIT_GFX.h>
#include "ELMduino.h"

// =========================================================
// Hardware configuration
// =========================================================
constexpr int HC05_RX_PIN = 16;
constexpr int HC05_TX_PIN = 17;
constexpr uint32_t HC05_BAUD = 38400;

// =========================================================
// Application timing
// =========================================================
constexpr uint32_t UI_UPDATE_INTERVAL_MS = 100;       // 10 Hz
constexpr uint32_t SCREEN_READY_DELAY_MS = 5000;
constexpr uint32_t RECONNECT_DELAY_MS = 3000;
constexpr uint8_t MAX_CONSECUTIVE_OBD_ERRORS = 4;

// OBD worker task. The ELMduino begin() path can block while
// protocol detection is running, so keep it outside loop().
constexpr uint32_t OBD_TASK_STACK_SIZE = 8192;
constexpr UBaseType_t OBD_TASK_PRIORITY = 1;

// =========================================================
// Global hardware objects
// =========================================================
HardwareSerial obdSerial(2);
ELM327* elm = nullptr;

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
ESP_8_BIT_GFX videoOut(true, 8);

// =========================================================
// Cached telemetry
// =========================================================
struct ObdData {
  uint32_t runTime = 0; // Time in seconds
  int clt = 0;
  int iat = 0;
  float engineLoad = 0.0f;
  float battery = 0.0f;
};

ObdData obdData;

// =========================================================
// Driving Alert State
// =========================================================
constexpr uint32_t ALERT_INTERVAL_SEC = 1800; // 30 minutes (30 * 60)
constexpr uint32_t ALERT_DURATION_MS = 5000;  // Show alert for 5 seconds

uint32_t lastAlertRunTime = 0;
bool isAlertActive = false;
uint32_t alertStartTimeMs = 0;

// =========================================================
// Connection / screen state
// =========================================================
enum class ConnectionState : uint8_t {
  Connecting,
  Connected,
  Disconnected
};

volatile ConnectionState connectionState = ConnectionState::Connecting;
volatile uint32_t connectedAt = 0;

// =========================================================
// OBD PID scheduler
// =========================================================
// Engine Load is sampled more often than slow-changing values.
enum class Pid : uint8_t {
  RunTime,
  EngineLoad,
  Clt,
  Iat,
  Battery
};

constexpr Pid PID_SEQUENCE[] = {
  Pid::EngineLoad,
  Pid::RunTime,
  Pid::EngineLoad,
  Pid::Clt,
  Pid::EngineLoad,
  Pid::Iat,
  Pid::EngineLoad,
  Pid::Battery
};

constexpr size_t PID_SEQUENCE_COUNT = sizeof(PID_SEQUENCE) / sizeof(PID_SEQUENCE[0]);

size_t pidIndex = 0;
uint8_t consecutiveObdErrors = 0;
uint32_t reconnectAfter = 0;

// =========================================================
// Helpers
// =========================================================
static bool isConnected() {
  return connectionState == ConnectionState::Connected;
}

static void markConnected() {
  connectionState = ConnectionState::Connected;
  connectedAt = millis();
  consecutiveObdErrors = 0;
  pidIndex = 0;

  Serial.println("ELM327 Connected!");
}

static void markDisconnected(const char* reason) {
  if (reason != nullptr) {
    Serial.print("OBD disconnected: ");
    Serial.println(reason);
  }

  connectionState = ConnectionState::Disconnected;
  reconnectAfter = millis() + RECONNECT_DELAY_MS;
  consecutiveObdErrors = 0;
  pidIndex = 0;

  if (elm != nullptr) {
    delete elm;
    elm = nullptr;
  }
}

static bool screenReady() {
  return isConnected() && (millis() - connectedAt >= SCREEN_READY_DELAY_MS);
}

// =========================================================
// Graphics
// =========================================================
void drawDriveCarefullyAlert() {
  // Fill screen with white background
  videoOut.fillScreen(0xFF);
  
  const int cx = videoOut.width() / 2;
  const int cy = videoOut.height() / 2;
  
  const char* alertText = "Drive carefully.";
  
  // Set text color to black (0x00) for contrast against white background
  videoOut.setTextColor(0x00);
  videoOut.setTextSize(2);
  
  // Calculate text position to center it
  // Size 2 text is approx 12 pixels wide and 16 pixels high per character
  const int maxTextWidth = strlen(alertText) * 12;
  videoOut.setCursor(cx - (maxTextWidth / 2), cy - 8);
  videoOut.print(alertText);
  
  // Reset text color back to default white for other screens
  videoOut.setTextColor(0xFF);
}

void drawMitsubishiLogo() {
  const int cx = videoOut.width() / 2;
  const int cy = (videoOut.height() / 2) - 20;
  const int dx = 14;
  const int dy = static_cast<int>(dx * 1.732f);

  videoOut.fillTriangle(cx, cy,
                        cx - dx, cy - dy,
                        cx + dx, cy - dy,
                        0xE0);

  videoOut.fillTriangle(cx - dx, cy - dy,
                        cx + dx, cy - dy,
                        cx, cy - 2 * dy,
                        0xE0);

  videoOut.fillTriangle(cx, cy,
                        cx - 2 * dx, cy,
                        cx - dx, cy + dy,
                        0xE0);

  videoOut.fillTriangle(cx - 2 * dx, cy,
                        cx - 3 * dx, cy + dy,
                        cx - dx, cy + dy,
                        0xE0);

  videoOut.fillTriangle(cx, cy,
                        cx + 2 * dx, cy,
                        cx + dx, cy + dy,
                        0xE0);

  videoOut.fillTriangle(cx + 2 * dx, cy,
                        cx + 3 * dx, cy + dy,
                        cx + dx, cy + dy,
                        0xE0);

  videoOut.setTextSize(1);
  videoOut.setTextColor(0xFF); 

  // ฟังก์ชันนี้จะถูกเรียกใช้เฉพาะตอน Connecting เท่านั้น จึงไม่จำเป็นต้องเช็คสถานะอีก
  const char* baseText = "Connecting OBDII";
  const int maxTextWidth = strlen(baseText) * 6;
  videoOut.setCursor(cx - (maxTextWidth / 2), cy + 55);
  videoOut.print(baseText);
}

void drawDataCell(int x, int y,
                  const char* label,
                  const char* value,
                  const char* unit) {
  videoOut.setTextColor(0xFF); 
  
  videoOut.setTextSize(1);
  videoOut.setCursor(x, y);
  videoOut.print(label);

  videoOut.setTextSize(3);
  videoOut.setCursor(x, y + 12);
  videoOut.print(value);

  const int valueLength = strlen(value);
  videoOut.setTextSize(1);
  videoOut.setCursor(x + (valueLength * 18) + 5, y + 26);
  videoOut.print(unit);
}

void renderDataScreen() {
  const int col1 = 15;
  const int col2 = (videoOut.width() / 2) + 15;
  const int row1 = 15;
  const int row2 = (videoOut.height() / 3) + 15;
  const int row3 = ((videoOut.height() / 3) * 2) + 15;

  char runTimeStr[16]; 
  char cltStr[16];
  char iatStr[16];
  char loadStr[16]; 
  char batStr[16];

  // Snapshot the cache once so one rendered frame does not mix values
  ObdData snapshot;
  portENTER_CRITICAL(&telemetryMux);
  snapshot = obdData;
  portEXIT_CRITICAL(&telemetryMux);

  // Format data into strings
  uint32_t rTime = snapshot.runTime;
  uint32_t hours = rTime / 3600;
  uint32_t minutes = (rTime % 3600) / 60;
  // uint32_t seconds = rTime % 60; // Not needed anymore for HH:MM

  // แก้ไขแสดงผลเป็น HH:MM
  snprintf(runTimeStr, sizeof(runTimeStr), "%02u:%02u", hours, minutes);

  snprintf(cltStr, sizeof(cltStr), "%d", snapshot.clt);
  snprintf(iatStr, sizeof(iatStr), "%d", snapshot.iat);
  snprintf(loadStr, sizeof(loadStr), "%.1f", snapshot.engineLoad);
  snprintf(batStr, sizeof(batStr), "%.1f", snapshot.battery);

  // Draw cells
  drawDataCell(col1, row1, "ECU Voltage", batStr, "V");
  drawDataCell(col2, row1, "Intake Air", iatStr, "C");
  drawDataCell(col1, row2, "Engine Load", loadStr, "%");
  drawDataCell(col2, row2, "Coolant", cltStr, "C");
  
  // Custom draw for Run Time to fit HH:MM
  videoOut.setTextColor(0xFF);
  videoOut.setTextSize(1);
  videoOut.setCursor(col1, row3);
  videoOut.print("Run Time");

  // ปรับขนาดกลับมาเป็น Size 3 ได้แล้วเพราะ 00:00 (5 ตัวอักษร) ใช้พื้นที่น้อยกว่า 00:00:00
  videoOut.setTextSize(3); 
  videoOut.setCursor(col1, row3 + 12);
  videoOut.print(runTimeStr);
  
  // Custom draw for Fuel Type
  // 1. Draw Label
  videoOut.setTextSize(1);
  videoOut.setTextColor(0xFF);
  videoOut.setCursor(col2, row3);
  videoOut.print("Fuel Type");

  // 2. Draw Value ("G95") in WHITE
  videoOut.setTextSize(3);
  videoOut.setTextColor(0xFF);
  videoOut.setCursor(col2, row3 + 12);
  videoOut.print("G95");
  
  // Reset text color back to default for the next frame
  videoOut.setTextColor(0xFF); 
}

void updateVideo() {
  videoOut.waitForFrame();
  videoOut.fillScreen(0x00);

  // Snapshot the runTime to check for alerts
  uint32_t currentRunTime = 0;
  portENTER_CRITICAL(&telemetryMux);
  currentRunTime = obdData.runTime;
  portEXIT_CRITICAL(&telemetryMux);

  const uint32_t nowMs = millis();

  // Check if we should trigger a new alert
  // Condition: Screen is ready, we haven't alerted for this 30-min block, and runTime crossed the threshold
  if (screenReady() && currentRunTime >= ALERT_INTERVAL_SEC) {
      uint32_t currentIntervalCount = currentRunTime / ALERT_INTERVAL_SEC;
      uint32_t lastAlertIntervalCount = lastAlertRunTime / ALERT_INTERVAL_SEC;

      if (currentIntervalCount > lastAlertIntervalCount && !isAlertActive) {
          isAlertActive = true;
          alertStartTimeMs = nowMs;
          lastAlertRunTime = currentRunTime;
      }
  }

  // Manage alert state duration
  if (isAlertActive) {
      if (nowMs - alertStartTimeMs < ALERT_DURATION_MS) {
          drawDriveCarefullyAlert();
          return; // Skip drawing other screens while alert is active
      } else {
          isAlertActive = false; // Alert duration finished
      }
  }

  // Normal screen rendering
  if (screenReady()) {
    renderDataScreen();
  } else if (isConnected()) {
    // ช่วงที่เชื่อมต่อแล้ว แต่กำลังรอให้ครบ 5 วินาที (SCREEN_READY_DELAY_MS)
    // แทนที่จะแสดง "Loading Data..." ให้แสดงหน้าแจ้งเตือนการขับขี่แทน
    drawDriveCarefullyAlert();
  } else {
    // ช่วงที่ยังไม่เชื่อมต่อ แสดงโลโก้
    drawMitsubishiLogo();
  }
}

// =========================================================
// ELMduino PID request helpers
// =========================================================
bool readPid(Pid pid) {
  switch (pid) {
    case Pid::RunTime: {
      const uint32_t value = elm->runTime();
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.runTime = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::EngineLoad: {
      const float value = elm->engineLoad(); // PID 0x04
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.engineLoad = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::Clt: {
      const int value = static_cast<int>(elm->engineCoolantTemp());
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.clt = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::Iat: {
      const int value = static_cast<int>(elm->intakeAirTemp());
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.iat = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::Battery: {
      const float value = elm->batteryVoltage();
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.battery = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }
  }

  return false;
}

void processCurrentPid() {
  if (elm == nullptr) {
    markDisconnected("ELM client is null");
    return;
  }

  const Pid pid = PID_SEQUENCE[pidIndex];

  const bool success = readPid(pid);

  if (success) {
    consecutiveObdErrors = 0;
    pidIndex = (pidIndex + 1) % PID_SEQUENCE_COUNT;
    return;
  }

  if (elm->nb_rx_state == ELM_GETTING_MSG) {
    return;
  }

  ++consecutiveObdErrors;

  Serial.print("OBD PID error, state=");
  Serial.print(static_cast<int>(elm->nb_rx_state));
  Serial.print(", count=");
  Serial.println(consecutiveObdErrors);

  pidIndex = (pidIndex + 1) % PID_SEQUENCE_COUNT;

  if (consecutiveObdErrors >= MAX_CONSECUTIVE_OBD_ERRORS) {
    markDisconnected("too many consecutive PID errors");
  }
}

// =========================================================
// OBD worker task
// =========================================================
void obdTask(void* parameter) {
  (void)parameter;

  for (;;) {
    switch (connectionState) {
      case ConnectionState::Connecting: {
        Serial.println("Connecting to ELM327...");

        if (elm != nullptr) {
          delete elm;
          elm = nullptr;
        }

        elm = new ELM327();
        if (elm == nullptr) {
          Serial.println("ELM327 allocation failed");
          connectionState = ConnectionState::Disconnected;
          reconnectAfter = millis() + RECONNECT_DELAY_MS;
          break;
        }

        if (elm->begin(obdSerial, true, 1500)) {
          markConnected();
        } else {
          Serial.println("ELM327 connection failed");
          delete elm;
          elm = nullptr;
          connectionState = ConnectionState::Disconnected;
          reconnectAfter = millis() + RECONNECT_DELAY_MS;
        }
        break;
      }

      case ConnectionState::Connected:
        processCurrentPid();
        vTaskDelay(pdMS_TO_TICKS(1));
        break;

      case ConnectionState::Disconnected:
        if (static_cast<int32_t>(millis() - reconnectAfter) >= 0) {
          connectionState = ConnectionState::Connecting;
        } else {
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        break;
    }
  }
}

// =========================================================
// Setup / main loop
// =========================================================
void setup() {
  Serial.begin(115200);

  // Reserve video framebuffer first.
  videoOut.begin();

  obdSerial.begin(HC05_BAUD, SERIAL_8N1, HC05_RX_PIN, HC05_TX_PIN);

  // Initial screen.
  videoOut.waitForFrame();
  videoOut.fillScreen(0x00);
  drawMitsubishiLogo();

  // Keep all ELMduino work away from the UI loop.
  xTaskCreatePinnedToCore(
    obdTask,
    "OBD_Task",
    OBD_TASK_STACK_SIZE,
    nullptr,
    OBD_TASK_PRIORITY,
    nullptr,
    0
  );
}

void loop() {
  static uint32_t lastUiUpdateTime = 0;
  const uint32_t now = millis();

  if (now - lastUiUpdateTime >= UI_UPDATE_INTERVAL_MS) {
    updateVideo();
    lastUiUpdateTime = now;
  }

  // Yield to other FreeRTOS tasks and avoid a tight application loop.
  delay(1);
}
