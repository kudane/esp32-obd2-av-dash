#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP_8_BIT_GFX.h>
#include "esp_bt.h" 

// ---------------------------------------------------------
// [CONFIGURATION] การตั้งค่าระบบ
// ---------------------------------------------------------
#define DEBUG_MODE 0            // เปลี่ยนเป็น 0 เมื่อใช้งานจริง (Production) เพื่อปิด Serial Log
#define MAX_OBD_ERRORS 5        // จำนวนครั้งที่อ่านพลาดก่อนรีเซ็ตการเชื่อมต่อ
#define OBD_POLL_DELAY_MS 50    // หน่วงเวลาระหว่างรอบการอ่าน (มิลลิวินาที)

// ตั้งค่า MAC Address ของ ELM327 OBD2
const uint8_t TARGET_MAC[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33}; 
const char* OBD_PIN = "1234";

// Macro สำหรับ Debug
#if DEBUG_MODE
  #define LOG_PRINT(x) Serial.print(x)
  #define LOG_PRINTLN(x) Serial.println(x)
#else
  #define LOG_PRINT(x)
  #define LOG_PRINTLN(x)
#endif

// โครงสร้างเก็บค่าพารามิเตอร์ของเครื่องยนต์
struct EngineData {
  bool isObdAvailable = false; 
  int rpm = -1;           
  int manifold = -1;      
  int coolantTemp = -999; 
  int airTemp = -999;     
  float throttle = -1.0;  
  float battery = -1.0;   
};

// Mutex สำหรับป้องกัน Core 0 และ Core 1 แย่งกันอ่าน/เขียนข้อมูล
SemaphoreHandle_t dataMutex;

class Elm327Connector {
private:
  enum State {
    STATE_CONNECTING,
    STATE_CONFIGURING,
    STATE_READING
  };

  BluetoothSerial SerialBT;
  State currentState;
  uint8_t macAddress[6];
  const char* pinCode;
  
  EngineData data;
  EngineData safeData; 
  uint8_t errorCount;

  // ฟังก์ชันส่งคำสั่งและรับผลลัพธ์ (ใช้ C-String ป้องกัน RAM Fragmentation)
  bool sendOBDCommand(const char* cmd, char* responseBuffer, size_t maxLen) {
    memset(responseBuffer, 0, maxLen);
    
    // ล้างข้อมูลเก่าที่อาจค้างใน Buffer
    while (SerialBT.available()) {
      SerialBT.read(); 
    }

    SerialBT.print(cmd);
    SerialBT.print("\r");

    unsigned long timeout = millis() + 1000; // Timeout 1 วินาทีต่อคำสั่ง
    size_t index = 0;
    bool hasData = false;

    while (millis() < timeout) {
      if (SerialBT.available()) {
        char c = SerialBT.read();
        if (c == '>') {
          break; // จบการตอบกลับ
        }
        if (c != '\r' && c != '\n' && c != ' ') {
          if (index < maxLen - 1) {
            responseBuffer[index++] = c; 
            hasData = true;
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    responseBuffer[index] = '\0'; 
    return hasData && strlen(responseBuffer) > 0 && strchr(responseBuffer, '?') == nullptr;
  }

  int getRPM(bool &success) {
    char buf[32];
    success = sendOBDCommand("010C", buf, sizeof(buf));
    if (!success) return -1;

    char* found = strstr(buf, "410C"); 
    if (found != nullptr && strlen(found) >= 8) {
      char hexA[3] = {found[4], found[5], '\0'};
      char hexB[3] = {found[6], found[7], '\0'};
      long a = strtol(hexA, NULL, 16);
      long b = strtol(hexB, NULL, 16);
      return ((a * 256) + b) / 4;
    }
    return -1;
  }

  int getManifold(bool &success) {
    char buf[32];
    success = sendOBDCommand("010B", buf, sizeof(buf));
    if (!success) return -1;

    char* found = strstr(buf, "410B");
    if (found != nullptr && strlen(found) >= 6) {
      char hexA[3] = {found[4], found[5], '\0'};
      return strtol(hexA, NULL, 16);
    }
    return -1;
  }

  int getCoolantTemp(bool &success) {
    char buf[32];
    success = sendOBDCommand("0105", buf, sizeof(buf));
    if (!success) return -999;

    char* found = strstr(buf, "4105");
    if (found != nullptr && strlen(found) >= 6) {
      char hexA[3] = {found[4], found[5], '\0'};
      return (int)strtol(hexA, NULL, 16) - 40;
    }
    return -999;
  }

  int getAirTemp(bool &success) {
    char buf[32];
    success = sendOBDCommand("010F", buf, sizeof(buf));
    if (!success) return -999;

    char* found = strstr(buf, "410F");
    if (found != nullptr && strlen(found) >= 6) {
      char hexA[3] = {found[4], found[5], '\0'};
      return (int)strtol(hexA, NULL, 16) - 40;
    }
    return -999;
  }

  float getThrottle(bool &success) {
    char buf[32];
    success = sendOBDCommand("0111", buf, sizeof(buf));
    if (!success) return -1.0;

    char* found = strstr(buf, "4111");
    if (found != nullptr && strlen(found) >= 6) {
      char hexA[3] = {found[4], found[5], '\0'};
      return ((float)strtol(hexA, NULL, 16) * 100.0) / 255.0;
    }
    return -1.0;
  }

  float getBattery(bool &success) {
    char buf[32];
    success = sendOBDCommand("ATRV", buf, sizeof(buf));
    if (success) {
      return atof(buf);
    }
    return -1.0;
  }

public:
  Elm327Connector(const uint8_t* mac, const char* pin) : currentState(STATE_CONNECTING), pinCode(pin), errorCount(0) {
    memcpy(macAddress, mac, 6);
  }

  void begin() {
    if (dataMutex == NULL) {
      dataMutex = xSemaphoreCreateMutex();
    }
    SerialBT.begin("ESP32_OBD_Master", true);
    SerialBT.setPin(pinCode);
    currentState = STATE_CONNECTING;
  }

  EngineData getEngineData() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      safeData = data;
      xSemaphoreGive(dataMutex);
    }
    return safeData;
  }

  void process() {
    char dummyBuf[16]; 
    bool cmdSuccess = false;
    
    switch (currentState) {
      case STATE_CONNECTING:
        if (!SerialBT.connected()) {
          LOG_PRINTLN("[OBD] Attempting to connect...");
          if (SerialBT.connect(macAddress)) {
            LOG_PRINTLN("[OBD] Connected successfully!");
            errorCount = 0;
            currentState = STATE_CONFIGURING;
          } else {
            LOG_PRINTLN("[OBD] Connection failed. Retrying in 3s...");
            vTaskDelay(pdMS_TO_TICKS(3000)); 
          }
        } else {
          currentState = STATE_CONFIGURING;
        }
        break;

      case STATE_CONFIGURING:
        LOG_PRINTLN("[OBD] Configuring ELM327...");
        sendOBDCommand("ATZ", dummyBuf, sizeof(dummyBuf));   vTaskDelay(pdMS_TO_TICKS(800));
        sendOBDCommand("ATE0", dummyBuf, sizeof(dummyBuf));  
        sendOBDCommand("ATL0", dummyBuf, sizeof(dummyBuf));  
        sendOBDCommand("ATS0", dummyBuf, sizeof(dummyBuf));  
        sendOBDCommand("ATSP0", dummyBuf, sizeof(dummyBuf)); 
        vTaskDelay(pdMS_TO_TICKS(500));
        
        LOG_PRINTLN("[OBD] Setup Done. Ready to Read!");
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          data.isObdAvailable = true;
          xSemaphoreGive(dataMutex);
        }
        currentState = STATE_READING;
        break;

      case STATE_READING:
        if (!SerialBT.connected()) {
          LOG_PRINTLN("[OBD] Connection lost!");
          handleDisconnect();
        } else {
          // ดึงค่าทีละตัว พร้อมเช็คสถานะ
          int tmp_rpm = getRPM(cmdSuccess);
          if(!cmdSuccess) errorCount++; else errorCount = 0;

          int tmp_manifold = getManifold(cmdSuccess);
          if(!cmdSuccess) errorCount++; else errorCount = 0;

          int tmp_coolantTemp = getCoolantTemp(cmdSuccess);
          if(!cmdSuccess) errorCount++; else errorCount = 0;

          int tmp_airTemp = getAirTemp(cmdSuccess);
          float tmp_throttle = getThrottle(cmdSuccess);
          float tmp_battery = getBattery(cmdSuccess); 
          
          // ถ้ารถดับ หรือดึงสายออกจนเกิด Error ติดต่อกันเกินกำหนด ให้รีเซ็ต
          if (errorCount >= MAX_OBD_ERRORS) {
            LOG_PRINTLN("[OBD] Max errors reached. Resetting connection...");
            SerialBT.disconnect();
            handleDisconnect();
            break;
          }
          
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            data.rpm = tmp_rpm;
            data.manifold = tmp_manifold; 
            data.coolantTemp = tmp_coolantTemp;
            data.airTemp = tmp_airTemp;   
            data.throttle = tmp_throttle;
            data.battery = tmp_battery; 
            xSemaphoreGive(dataMutex);
          }
          
          vTaskDelay(pdMS_TO_TICKS(OBD_POLL_DELAY_MS)); 
        }
        break;
    }
  }

private:
  void handleDisconnect() {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      data.isObdAvailable = false;
      xSemaphoreGive(dataMutex);
    }
    errorCount = 0;
    currentState = STATE_CONNECTING;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
};

class NtscDisplayer {
private:
  ESP_8_BIT_GFX videoOut;

  void drawMitsubishiLogo() {
    int cx = videoOut.width() / 2;
    int cy = videoOut.height() / 2 - 20;
    int dx = 14; 
    int dy = (int)(dx * 1.732); 

    videoOut.fillTriangle(cx, cy, cx - dx, cy - dy, cx + dx, cy - dy, 0xE0);
    videoOut.fillTriangle(cx - dx, cy - dy, cx + dx, cy - dy, cx, cy - 2 * dy, 0xE0);
    videoOut.fillTriangle(cx, cy, cx - 2 * dx, cy, cx - dx, cy + dy, 0xE0);
    videoOut.fillTriangle(cx - 2 * dx, cy, cx - 3 * dx, cy + dy, cx - dx, cy + dy, 0xE0);
    videoOut.fillTriangle(cx, cy, cx + 2 * dx, cy, cx + dx, cy + dy, 0xE0);
    videoOut.fillTriangle(cx + 2 * dx, cy, cx + 3 * dx, cy + dy, cx + dx, cy + dy, 0xE0);

    videoOut.setTextSize(1);
    char baseText[32];
    strcpy(baseText, "Searching OBDII");
    
    int dotCount = (millis() / 500) % 4; 
    for(int i = 0; i < dotCount; i++) {
      strcat(baseText, "."); 
    }
    
    int maxTextWidth = (15 + 3) * 6; 
    videoOut.setCursor(cx - (maxTextWidth / 2), cy + 55);
    videoOut.print(baseText);
  }

  void drawDataCell(int x, int y, const char* label, const char* value, const char* unit) {
    videoOut.setTextSize(1);
    videoOut.setCursor(x, y);
    videoOut.print(label);

    videoOut.setTextSize(3);
    videoOut.setCursor(x, y + 12); 
    videoOut.print(value);

    int valueLength = strlen(value);
    videoOut.setTextSize(1);
    videoOut.setCursor(x + (valueLength * 18) + 5, y + 26);
    videoOut.print(unit);
  }

public:
  NtscDisplayer() : videoOut(true, 8) {}

  void begin() {
    videoOut.begin();
  }

  void waitForSync() {
    videoOut.waitForFrame();
  }

  void showConnectOBDII() {
    videoOut.fillScreen(0x00); 
    drawMitsubishiLogo();
  }

  void showEngineData(const EngineData& data) {
    videoOut.fillScreen(0x00);

    int col1 = 15;
    int col2 = (videoOut.width() / 2) + 15;
    int row1 = 15;
    int row2 = (videoOut.height() / 3) + 15;
    int row3 = ((videoOut.height() / 3) * 2) + 15;

    char rpmStr[16], iatStr[16], tpsStr[16], cltStr[16], batStr[16], mapStr[16];
    
    // UI Polish: ถ้าค่าเป็น Default (เช่นอ่านไม่ได้) ให้โชว์ N/A
    if(data.rpm == -1) strcpy(rpmStr, "N/A"); else snprintf(rpmStr, sizeof(rpmStr), "%d", data.rpm);
    if(data.airTemp == -999) strcpy(iatStr, "N/A"); else snprintf(iatStr, sizeof(iatStr), "%d", data.airTemp);
    if(data.throttle < 0) strcpy(tpsStr, "N/A"); else snprintf(tpsStr, sizeof(tpsStr), "%.1f", data.throttle);
    if(data.coolantTemp == -999) strcpy(cltStr, "N/A"); else snprintf(cltStr, sizeof(cltStr), "%d", data.coolantTemp);
    if(data.battery < 0) strcpy(batStr, "N/A"); else snprintf(batStr, sizeof(batStr), "%.1f", data.battery);
    if(data.manifold == -1) strcpy(mapStr, "N/A"); else snprintf(mapStr, sizeof(mapStr), "%d", data.manifold);

    drawDataCell(col1, row1, "Engine Speed", rpmStr, "rpm");
    drawDataCell(col2, row1, "Intake Air", iatStr, "C");
    drawDataCell(col1, row2, "Throttle", tpsStr, "%");
    drawDataCell(col2, row2, "Coolant", cltStr, "C");
    drawDataCell(col1, row3, "Battery", batStr, "V");
    drawDataCell(col2, row3, "Manifold", mapStr, "kPa");
  }
};

Elm327Connector* obdScanner = nullptr;
NtscDisplayer* displayer = nullptr;

// Task Core 0: จัดการเรื่อง Bluetooth และ Network โดยเฉพาะ
void obdTask(void * pvParameters) {
  while (true) {
    if (obdScanner != nullptr) {
      obdScanner->process();
    }
    // ป้องกัน Watchdog Timer
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

void setup() {
  #if DEBUG_MODE
    Serial.begin(115200);
    delay(1000); 
    LOG_PRINTLN("\n\n--- System Booting (Production) ---");
  #endif

  // [หัวใจสำคัญ 1]: คืน RAM จาก BLE (30KB) เพื่อให้ DMA ของจอและ BT อยู่ร่วมกันได้
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  // [หัวใจสำคัญ 2]: เปิดจอภาพก่อนเพื่อจอง RAM ต่อเนื่อง (Contiguous Memory)
  LOG_PRINTLN("Allocating Video Memory (ESP_8_BIT_GFX)...");
  displayer = new NtscDisplayer();
  displayer->begin();
  
  // รอให้จอภาพสร้างสัญญาณ Sync จนเสถียรก่อนกระชากไฟเปิด Bluetooth
  delay(1500); 

  // [หัวใจสำคัญ 3]: เริ่มระบบ Bluetooth
  LOG_PRINTLN("Allocating Bluetooth Stack...");
  obdScanner = new Elm327Connector(TARGET_MAC, OBD_PIN);
  obdScanner->begin();

  LOG_PRINTLN("Starting Core 0 Task...");
  xTaskCreatePinnedToCore(
    obdTask,        
    "OBD_Task",     
    8192,           // Stack 8KB (ประหยัด RAM)
    NULL,           
    1,              
    NULL,           
    0               // ผูกติด Core 0
  );
}

void loop() {
  if (obdScanner != nullptr && displayer != nullptr) {
    
    // รอจนกว่าจะจบ 1 Frame (60Hz) ป้องกันจอกระพริบและลดภาระ Core 1
    displayer->waitForSync(); 
    
    EngineData currentData = obdScanner->getEngineData();

    if (currentData.isObdAvailable) {
      displayer->showEngineData(currentData);
    } else {
      displayer->showConnectOBDII();
    }
  } else {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
