#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
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
// Application timing & limits
// =========================================================
constexpr uint32_t UI_UPDATE_INTERVAL_MS = 100;       // 10 Hz
constexpr uint32_t SCREEN_READY_DELAY_MS = 5000;
constexpr uint32_t RECONNECT_DELAY_MS = 3000;

// จำนวนครั้งที่ยอมให้ Error โดยรวมก่อนตัดการเชื่อมต่อ
constexpr uint8_t MAX_GLOBAL_OBD_ERRORS = 5; 
// จำนวนครั้งที่ยอมให้ Error ต่อ 1 เซ็นเซอร์ ก่อนจะขึ้น -- (Stale data prevention)
constexpr uint8_t MAX_ERRORS_PER_PID = 3;    
// จำนวนครั้งที่ยอมให้ Error ต่อคิว ก่อนจะทำการ Auto-Skip (ข้ามการดึงคิวนี้ไปเลยเพื่อรักษา Bandwidth)
constexpr uint8_t MAX_ERRORS_BEFORE_SKIP = 10; 

constexpr uint32_t OBD_TASK_STACK_SIZE = 8192;
constexpr UBaseType_t OBD_TASK_PRIORITY = 1;

// =========================================================
// Global hardware objects
// =========================================================
HardwareSerial obdSerial(2);

// Best Practice 1: Static Allocation หลีกเลี่ยง Memory Leak/Fragmentation
ELM327 elm; 

// Best Practice 2: ใช้ Mutex แทน portMUX เพื่อหลีกเลี่ยงการ Block Hardware Interrupts
SemaphoreHandle_t telemetryMutex = NULL;
ESP_8_BIT_GFX videoOut(true, 8);

// =========================================================
// Cached telemetry (Data Structure)
// =========================================================
// ใช้ค่า -999 เป็น Magic Number เพื่อบอกหน้าจอว่าเซ็นเซอร์ตัวนี้ขาดการเชื่อมต่อ
constexpr int INVALID_INT = -999;
constexpr float INVALID_FLOAT = -999.0f;

struct ObdData {
  uint32_t runTime = 0; // Time in seconds
  int clt = INVALID_INT;
  int iat = INVALID_INT;
  float engineLoad = INVALID_FLOAT;
  float battery = INVALID_FLOAT;
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

uint32_t baseRunTimeOffset = 0;
uint32_t runTimeSyncMillis = 0;
bool isRunTimeSynced = false;

// =========================================================
// OBD PID scheduler
// =========================================================
enum class Pid : uint8_t {
  EngineLoad,
  Clt,
  Iat,
  Battery
};

// Sequence ของการดึงข้อมูล
constexpr Pid PID_SEQUENCE[] = {
  Pid::EngineLoad,
  Pid::Clt,
  Pid::EngineLoad,
  Pid::Iat,
  Pid::EngineLoad,
  Pid::Battery
};

constexpr size_t PID_SEQUENCE_COUNT = sizeof(PID_SEQUENCE) / sizeof(PID_SEQUENCE[0]);

size_t pidIndex = 0;
uint8_t globalObdErrors = 0;                   
uint8_t pidErrors[PID_SEQUENCE_COUNT] = {0};   
bool skipSequenceSlot[PID_SEQUENCE_COUNT] = {false}; // บันทึกสถานะว่าคิวไหนถูก Auto-Skip ไปแล้วบ้าง
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
  globalObdErrors = 0;
  pidIndex = 0;
  isRunTimeSynced = false; 
  memset(pidErrors, 0, sizeof(pidErrors));
  memset(skipSequenceSlot, 0, sizeof(skipSequenceSlot)); // รีเซ็ตการข้ามทั้งหมดเมื่อเริ่มเชื่อมต่อใหม่

  Serial.println("ELM327 Connected!");
}

static void markDisconnected(const char* reason) {
  if (reason != nullptr) {
    Serial.print("OBD disconnected: ");
    Serial.println(reason);
  }

  connectionState = ConnectionState::Disconnected;
  reconnectAfter = millis() + RECONNECT_DELAY_MS;
  globalObdErrors = 0;
  pidIndex = 0;
  isRunTimeSynced = false;
  memset(pidErrors, 0, sizeof(pidErrors));
  memset(skipSequenceSlot, 0, sizeof(skipSequenceSlot)); // รีเซ็ตการข้ามทั้งหมดเมื่อหลุด
  
  // เคลียร์ค่าแคชให้เป็น Invalid เมื่อหลุด
  if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
      obdData.clt = INVALID_INT;
      obdData.iat = INVALID_INT;
      obdData.engineLoad = INVALID_FLOAT;
      obdData.battery = INVALID_FLOAT;
      xSemaphoreGive(telemetryMutex);
  }
}

static bool screenReady() {
  return isConnected() && (millis() - connectedAt >= SCREEN_READY_DELAY_MS);
}

// =========================================================
// Graphics
// =========================================================
void drawDriveCarefullyAlert() {
  videoOut.fillScreen(0x00); // พื้นหลังสีดำ
  
  const int cx = videoOut.width() / 2;
  const int cy = videoOut.height() / 2;
  const char* alertText = "Drive carefully.";
  
  videoOut.setTextColor(0xFF); // ตัวอักษรสีขาว
  videoOut.setTextSize(2);
  
  const int maxTextWidth = strlen(alertText) * 12;
  videoOut.setCursor(cx - (maxTextWidth / 2), cy - 8);
  videoOut.print(alertText);
}

void drawMitsubishiLogo() {
  const int cx = videoOut.width() / 2;
  const int cy = (videoOut.height() / 2) - 20;
  const int dx = 14;
  const int dy = static_cast<int>(dx * 1.732f);

  videoOut.fillTriangle(cx, cy, cx - dx, cy - dy, cx + dx, cy - dy, 0xE0);
  videoOut.fillTriangle(cx - dx, cy - dy, cx + dx, cy - dy, cx, cy - 2 * dy, 0xE0);
  videoOut.fillTriangle(cx, cy, cx - 2 * dx, cy, cx - dx, cy + dy, 0xE0);
  videoOut.fillTriangle(cx - 2 * dx, cy, cx - 3 * dx, cy + dy, cx - dx, cy + dy, 0xE0);
  videoOut.fillTriangle(cx, cy, cx + 2 * dx, cy, cx + dx, cy + dy, 0xE0);
  videoOut.fillTriangle(cx + 2 * dx, cy, cx + 3 * dx, cy + dy, cx + dx, cy + dy, 0xE0);

  videoOut.setTextSize(1);
  videoOut.setTextColor(0xFF); 

  const char* baseText = "Connecting OBDII";
  const int maxTextWidth = strlen(baseText) * 6;
  videoOut.setCursor(cx - (maxTextWidth / 2), cy + 55);
  videoOut.print(baseText);
}

void drawDataCell(int x, int y, const char* label, const char* value, const char* unit) {
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

  ObdData snapshot;
  // ใช้ Mutex แบบ Best Practice (รอได้สูงสุด 10 Tick ถ้ารอไม่ไหวข้ามไปก่อนเพื่อไม่ให้จอค้าง)
  if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      snapshot = obdData;
      xSemaphoreGive(telemetryMutex);
  }

  // Format Data (ถ้าข้อมูลขาดหาย จะแสดง -- แทน Stale Data)
  uint32_t rTime = snapshot.runTime;
  snprintf(runTimeStr, sizeof(runTimeStr), "%02u:%02u", rTime / 3600, (rTime % 3600) / 60);

  if (snapshot.clt == INVALID_INT) strcpy(cltStr, "--"); 
  else snprintf(cltStr, sizeof(cltStr), "%d", snapshot.clt);

  if (snapshot.iat == INVALID_INT) strcpy(iatStr, "--"); 
  else snprintf(iatStr, sizeof(iatStr), "%d", snapshot.iat);

  if (snapshot.engineLoad == INVALID_FLOAT) strcpy(loadStr, "--"); 
  else snprintf(loadStr, sizeof(loadStr), "%.1f", snapshot.engineLoad);

  if (snapshot.battery == INVALID_FLOAT) strcpy(batStr, "--"); 
  else snprintf(batStr, sizeof(batStr), "%.1f", snapshot.battery);

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

  videoOut.setTextSize(3); 
  videoOut.setCursor(col1, row3 + 12);
  videoOut.print(runTimeStr);
  
  // Custom draw for Fuel Type
  videoOut.setTextSize(1);
  videoOut.setCursor(col2, row3);
  videoOut.print("Fuel Type");

  videoOut.setTextSize(3);
  videoOut.setCursor(col2, row3 + 12);
  videoOut.print("G95");
}

void updateVideo() {
  videoOut.waitForFrame();
  videoOut.fillScreen(0x00);

  uint32_t currentRunTime = 0;
  if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      currentRunTime = obdData.runTime;
      xSemaphoreGive(telemetryMutex);
  }

  const uint32_t nowMs = millis();

  // ตรวจสอบรอบเวลา 30 นาทีสำหรับการแจ้งเตือน
  if (screenReady() && currentRunTime >= ALERT_INTERVAL_SEC) {
      uint32_t currentIntervalCount = currentRunTime / ALERT_INTERVAL_SEC;
      uint32_t lastAlertIntervalCount = lastAlertRunTime / ALERT_INTERVAL_SEC;

      if (currentIntervalCount > lastAlertIntervalCount && !isAlertActive) {
          isAlertActive = true;
          alertStartTimeMs = nowMs;
          lastAlertRunTime = currentRunTime;
      }
  }

  if (isAlertActive) {
      if (nowMs - alertStartTimeMs < ALERT_DURATION_MS) {
          drawDriveCarefullyAlert();
          return; 
      } else {
          isAlertActive = false; 
      }
  }

  if (screenReady()) {
    renderDataScreen();
  } else if (isConnected()) {
    drawDriveCarefullyAlert();
  } else {
    drawMitsubishiLogo();
  }
}

// =========================================================
// ELMduino PID request helpers
// =========================================================
void invalidatePidData(Pid pid) {
  // หากล้มเหลวเกินกำหนด เคลียร์ค่า PID นั้นๆ ให้หน้าจอรู้ตัว
  if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
      switch (pid) {
          case Pid::EngineLoad: obdData.engineLoad = INVALID_FLOAT; break;
          case Pid::Clt: obdData.clt = INVALID_INT; break;
          case Pid::Iat: obdData.iat = INVALID_INT; break;
          case Pid::Battery: obdData.battery = INVALID_FLOAT; break;
      }
      xSemaphoreGive(telemetryMutex);
  }
}

bool readPid(Pid pid) {
  switch (pid) {
    case Pid::EngineLoad: {
      const float value = elm.engineLoad(); 
      if (elm.nb_rx_state == ELM_SUCCESS) {
        if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          obdData.engineLoad = value;
          xSemaphoreGive(telemetryMutex);
        }
        return true;
      }
      break;
    }
    case Pid::Clt: {
      const int value = static_cast<int>(elm.engineCoolantTemp());
      if (elm.nb_rx_state == ELM_SUCCESS) {
        if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          obdData.clt = value;
          xSemaphoreGive(telemetryMutex);
        }
        return true;
      }
      break;
    }
    case Pid::Iat: {
      const int value = static_cast<int>(elm.intakeAirTemp());
      if (elm.nb_rx_state == ELM_SUCCESS) {
        if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          obdData.iat = value;
          xSemaphoreGive(telemetryMutex);
        }
        return true;
      }
      break;
    }
    case Pid::Battery: {
      const float value = elm.batteryVoltage();
      if (elm.nb_rx_state == ELM_SUCCESS) {
        if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          obdData.battery = value;
          xSemaphoreGive(telemetryMutex);
        }
        return true;
      }
      break;
    }
  }
  return false;
}

void processCurrentPid() {
  // 1. Initial Sync Phase: ดึงค่าเวลา Run Time จากรถแค่ครั้งแรกครั้งเดียว
  if (!isRunTimeSynced) {
    const uint32_t value = elm.runTime();
    
    if (elm.nb_rx_state == ELM_SUCCESS) {
      isRunTimeSynced = true;
      runTimeSyncMillis = millis();
      baseRunTimeOffset = value;
      globalObdErrors = 0;
    } else if (elm.nb_rx_state != ELM_GETTING_MSG) {
      ++globalObdErrors;
      if (globalObdErrors >= 3) {
        // Fallback: ถ้ารถส่งค่าไม่ได้เลย ให้ตีค่าเวลาตั้งต้นเป็น 0
        isRunTimeSynced = true;
        runTimeSyncMillis = millis();
        baseRunTimeOffset = 0;
        globalObdErrors = 0;
      }
    }
    return;
  }

  // 2. Normal Polling Phase
  // เช็คและข้ามคิวที่ถูก Auto-Skip ไปแล้ว
  size_t startPidIndex = pidIndex;
  while (skipSequenceSlot[pidIndex]) {
    pidIndex = (pidIndex + 1) % PID_SEQUENCE_COUNT;
    
    // Safety check: ป้องกัน Infinite Loop กรณีพังหมดทุกเซ็นเซอร์
    if (pidIndex == startPidIndex) {
      return; 
    }
  }

  const Pid pid = PID_SEQUENCE[pidIndex];
  const bool success = readPid(pid);

  if (success) {
    // รีเซ็ต Error ของตัวมันเอง และ Error โดยรวม
    pidErrors[pidIndex] = 0;
    globalObdErrors = 0;
    pidIndex = (pidIndex + 1) % PID_SEQUENCE_COUNT;
    return;
  }

  if (elm.nb_rx_state == ELM_GETTING_MSG) {
    return; 
  }

  // หากล้มเหลว
  ++pidErrors[pidIndex];
  ++globalObdErrors;

  // Best Practice 3 & 4: Invalidate stale data & Auto-Skip
  if (pidErrors[pidIndex] == MAX_ERRORS_PER_PID) {
      invalidatePidData(pid);
  } else if (pidErrors[pidIndex] >= MAX_ERRORS_BEFORE_SKIP) {
      skipSequenceSlot[pidIndex] = true;
      Serial.print("Auto-Skip activated for sequence index: ");
      Serial.println(pidIndex);
  }

  pidIndex = (pidIndex + 1) % PID_SEQUENCE_COUNT;

  if (globalObdErrors >= MAX_GLOBAL_OBD_ERRORS) {
    markDisconnected("too many consecutive global PID errors");
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
        
        // ไม่มีการจอง/ทำลาย Object ใหม่แล้ว อาศัยฟังก์ชัน begin() เคลียร์สถานะแทน
        if (elm.begin(obdSerial, true, 1500)) {
          markConnected();
        } else {
          Serial.println("ELM327 connection failed");
          connectionState = ConnectionState::Disconnected;
          reconnectAfter = millis() + RECONNECT_DELAY_MS;
        }
        break;
      }

      case ConnectionState::Connected:
        processCurrentPid();
        
        if (isRunTimeSynced) {
          if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
            obdData.runTime = baseRunTimeOffset + ((millis() - runTimeSyncMillis) / 1000);
            xSemaphoreGive(telemetryMutex);
          }
        }
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
  
  // สร้าง Mutex ให้พร้อมก่อนเริ่มกระบวนการอื่นๆ
  telemetryMutex = xSemaphoreCreateMutex();
  if (telemetryMutex == NULL) {
      Serial.println("Failed to create mutex!");
      while (1) delay(100); 
  }

  videoOut.begin();
  obdSerial.begin(HC05_BAUD, SERIAL_8N1, HC05_RX_PIN, HC05_TX_PIN);

  videoOut.waitForFrame();
  videoOut.fillScreen(0x00);
  drawMitsubishiLogo();

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

  delay(1);
}
