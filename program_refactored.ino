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
  int rpm = 0;
  int map = 0;
  int clt = 0;
  int iat = 0;
  float tps = 0.0f;
  float battery = 0.0f;
};

ObdData obdData;

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
// RPM and TPS are sampled more often than slow-changing values.
// Sequence:
//   RPM -> TPS -> RPM -> MAP -> RPM -> CLT -> RPM -> IAT -> RPM -> BAT
// This keeps the dashboard responsive without issuing concurrent PID requests.
enum class Pid : uint8_t {
  Rpm,
  Tps,
  Map,
  Clt,
  Iat,
  Battery
};

constexpr Pid PID_SEQUENCE[] = {
  Pid::Rpm,
  Pid::Tps,
  Pid::Rpm,
  Pid::Map,
  Pid::Rpm,
  Pid::Clt,
  Pid::Rpm,
  Pid::Iat,
  Pid::Rpm,
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

  // ELMduino owns an internal payload buffer. Destroy the client so a
  // future reconnect starts with a clean allocation.
  if (elm != nullptr) {
    delete elm;
    elm = nullptr;
  }
}

static bool screenReady() {
  return isConnected() &&
         (millis() - connectedAt >= SCREEN_READY_DELAY_MS);
}

// =========================================================
// Graphics
// =========================================================
void drawMitsubishiLogo() {
  const int cx = videoOut.width() / 2;
  const int cy = videoOut.height() / 2 - 20;
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

  char baseText[32];
  if (!isConnected()) {
    strcpy(baseText, "Connecting OBDII");

    const int dotCount = (millis() / 500) % 4;
    for (int i = 0; i < dotCount; ++i) {
      strcat(baseText, ".");
    }
  } else {
    strcpy(baseText, "Connected! Waiting...");
  }

  const int maxTextWidth = strlen(baseText) * 6;
  videoOut.setCursor(cx - (maxTextWidth / 2), cy + 55);
  videoOut.print(baseText);
}

void drawDataCell(int x, int y,
                  const char* label,
                  const char* value,
                  const char* unit) {
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

  char rpmStr[16];
  char mapStr[16];
  char cltStr[16];
  char iatStr[16];
  char tpsStr[16];
  char batStr[16];

  // Snapshot the cache once so one rendered frame does not mix values
  // from multiple reads while the OBD task is updating them.
  ObdData snapshot;
  portENTER_CRITICAL(&telemetryMux);
  snapshot = obdData;
  portEXIT_CRITICAL(&telemetryMux);

  snprintf(rpmStr, sizeof(rpmStr), "%d", snapshot.rpm);
  snprintf(mapStr, sizeof(mapStr), "%d", snapshot.map);
  snprintf(cltStr, sizeof(cltStr), "%d", snapshot.clt);
  snprintf(iatStr, sizeof(iatStr), "%d", snapshot.iat);
  snprintf(tpsStr, sizeof(tpsStr), "%.1f", snapshot.tps);
  snprintf(batStr, sizeof(batStr), "%.1f", snapshot.battery);

  drawDataCell(col1, row1, "Engine Speed", rpmStr, "rpm");
  drawDataCell(col2, row1, "Intake Air", iatStr, "C");
  drawDataCell(col1, row2, "Throttle", tpsStr, "%");
  drawDataCell(col2, row2, "Coolant", cltStr, "C");
  drawDataCell(col1, row3, "Battery", batStr, "V");
  drawDataCell(col2, row3, "Manifold", mapStr, "kPa");
}

void updateVideo() {
  videoOut.waitForFrame();
  videoOut.fillScreen(0x00);

  if (screenReady()) {
    renderDataScreen();
  } else {
    drawMitsubishiLogo();
  }
}

// =========================================================
// ELMduino PID request helpers
// =========================================================
bool readPid(Pid pid) {
  switch (pid) {
    case Pid::Rpm: {
      const float value = elm->rpm();
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.rpm = static_cast<int>(value);
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::Tps: {
      const float value = elm->throttle();
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.tps = value;
        portEXIT_CRITICAL(&telemetryMux);
        return true;
      }
      break;
    }

    case Pid::Map: {
      const int value = static_cast<int>(elm->manifoldPressure());
      if (elm->nb_rx_state == ELM_SUCCESS) {
        portENTER_CRITICAL(&telemetryMux);
        obdData.map = value;
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

  // Important: call the same PID function repeatedly while
  // ELM_GETTING_MSG is active. Do not issue a second query just
  // to retrieve the result.
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

        // Allocate a fresh ELMduino object for every connection attempt.
        // If begin() fails, delete() runs ELMduino's destructor and frees
        // its internal payload buffer before the next attempt.
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

        // ELMduino begin() can block during protocol detection.
        // It is intentionally executed here, not inside loop().
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
