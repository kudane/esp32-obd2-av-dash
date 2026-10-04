#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP_8_BIT_GFX.h>
#include "ELMduino.h"

// กำหนดขาเชื่อมต่อ HC-05 (ใช้ Serial2)
#define HC05_RX_PIN 16 
#define HC05_TX_PIN 17 
#define HC05_BAUD 38400 

HardwareSerial obdSerial(2);
ELM327 myELM327;
ESP_8_BIT_GFX videoOut(true, 8);

// ---------------------------------------------------------
// ตัวแปรเก็บค่าเครื่องยนต์ (Step 4 & 5)
// ---------------------------------------------------------
int v_rpm = 0;
int v_map = 0;
int v_clt = 0;
int v_iat = 0;
float v_tps = 0.0;
float v_bat = 0.0;

// ---------------------------------------------------------
// State Machine สำหรับระบบ Non-Blocking
// ---------------------------------------------------------
typedef enum {
  ENG_CONNECTING,
  ENG_RPM,
  ENG_MAP,
  ENG_CLT,
  ENG_IAT,
  ENG_TPS,
  ENG_BAT
} obd_states_t;

obd_states_t obd_state = ENG_CONNECTING;

bool isElmConnected = false;
bool showDataScreen = false;
unsigned long lastUiUpdateTime = 0;

// ---------------------------------------------------------
// ฟังก์ชันวาดกราฟิก (Step 1 & 5)
// ---------------------------------------------------------
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
  
  if (!isElmConnected) {
    strcpy(baseText, "Connecting OBDII");
    int dotCount = (millis() / 500) % 4; 
    for(int i = 0; i < dotCount; i++) strcat(baseText, "."); 
  } else {
    // ถ้าต่อติดแล้ว แต่รอเวลาครบ 5 วินาที
    strcpy(baseText, "Connected! Waiting...");
  }
  
  int maxTextWidth = strlen(baseText) * 6; 
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

void renderDataScreen() {
  int col1 = 15;
  int col2 = (videoOut.width() / 2) + 15;
  int row1 = 15;
  int row2 = (videoOut.height() / 3) + 15;
  int row3 = ((videoOut.height() / 3) * 2) + 15;

  char rpmStr[16], mapStr[16], cltStr[16], iatStr[16], tpsStr[16], batStr[16];
  
  snprintf(rpmStr, sizeof(rpmStr), "%d", v_rpm);
  snprintf(mapStr, sizeof(mapStr), "%d", v_map);
  snprintf(cltStr, sizeof(cltStr), "%d", v_clt);
  snprintf(iatStr, sizeof(iatStr), "%d", v_iat);
  snprintf(tpsStr, sizeof(tpsStr), "%.1f", v_tps);
  snprintf(batStr, sizeof(batStr), "%.1f", v_bat);

  drawDataCell(col1, row1, "Engine Speed", rpmStr, "rpm");
  drawDataCell(col2, row1, "Intake Air", iatStr, "C");
  drawDataCell(col1, row2, "Throttle", tpsStr, "%");
  drawDataCell(col2, row2, "Coolant", cltStr, "C");
  drawDataCell(col1, row3, "Battery", batStr, "V");
  drawDataCell(col2, row3, "Manifold", mapStr, "kPa");
}

// ---------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  // จอง RAM สำหรับภาพวิดีโอก่อนเสมอ
  videoOut.begin();

  // เริ่มต้นพอร์ตสื่อสารกับ HC-05
  obdSerial.begin(HC05_BAUD, SERIAL_8N1, HC05_RX_PIN, HC05_TX_PIN);

  // วาดโลโก้ทันทีที่เปิดเครื่อง (Step 1)
  videoOut.waitForFrame();   
  videoOut.fillScreen(0x00); 
  drawMitsubishiLogo();
}

void loop() {
  // ---------------------------------------------------------
  // Step 3: เงื่อนไขการปิดโลโก้ (ต่อสำเร็จ + เปิดเครื่องมาแล้ว 5 วินาที)
  // ---------------------------------------------------------
  if (!showDataScreen && isElmConnected && (millis() > 5000)) {
    showDataScreen = true; // อนุญาตให้เปลี่ยนจอ
  }

  // ---------------------------------------------------------
  // Step 2 & 4: ELMduino Non-Blocking Data Flow (วิธีที่ดีที่สุด)
  // ---------------------------------------------------------
  switch (obd_state) {
    case ENG_CONNECTING:
      // ใช้ Timeout 1.5 วิ ป้องกันลูปค้าง (Step 2)
      if (myELM327.begin(obdSerial, true, 1500)) {
        Serial.println("ELM327 Connected!");
        isElmConnected = true;
        obd_state = ENG_RPM;
      }
      break;

    case ENG_RPM: {
      float tempRpm = myELM327.rpm();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_rpm = (int)tempRpm;
        obd_state = ENG_MAP;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_MAP; // หาก Error ให้ข้ามไปอ่านค่าถัดไปทันที
      }
      break;
    }

    case ENG_MAP: {
      int tempMap = myELM327.manifoldPressure();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_map = tempMap;
        obd_state = ENG_CLT;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_CLT;
      }
      break;
    }

    case ENG_CLT: {
      float tempClt = myELM327.engineCoolantTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_clt = (int)tempClt;
        obd_state = ENG_IAT;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_IAT;
      }
      break;
    }

    case ENG_IAT: {
      float tempIat = myELM327.intakeAirTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_iat = (int)tempIat;
        obd_state = ENG_TPS;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_TPS;
      }
      break;
    }

    case ENG_TPS: {
      float tempTps = myELM327.throttle();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_tps = tempTps;
        obd_state = ENG_BAT;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_BAT;
      }
      break;
    }

    case ENG_BAT: {
      float tempBat = myELM327.batteryVoltage();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        v_bat = tempBat;
        obd_state = ENG_RPM; // วนลูปกลับไปอ่าน RPM ใหม่
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        obd_state = ENG_RPM;
      }
      break;
    }
  }

  // ---------------------------------------------------------
  // Step 5: จัดการการแสดงผลภาพ (Update UI)
  // ---------------------------------------------------------
  // อัปเดตหน้าจอทุกๆ 100ms ป้องกันหน้าจอกระพริบและลดภาระ CPU
  if (millis() - lastUiUpdateTime > 100) {
    videoOut.waitForFrame();
    videoOut.fillScreen(0x00);
    
    if (showDataScreen) {
      renderDataScreen(); // แสดงค่า Data
    } else {
      drawMitsubishiLogo(); // ยังไม่ผ่านเงื่อนไข 5 วิ หรือยังต่อไม่ติด
    }
    
    lastUiUpdateTime = millis();
  }
}
