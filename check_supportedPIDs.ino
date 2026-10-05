#include "ELMduino.h"

#define ELM_PORT Serial2
#define RX2_PIN 16
#define TX2_PIN 17
#define ELM_BAUD 38400   // ต้องตรงกับ AT+UART ที่ตั้งใน HC-05

ELM327 myELM327;

uint8_t step = 0;
uint32_t mask1, mask2, mask3;

void printSupported(uint32_t mask, uint8_t startPid) {
  for (uint8_t i = 0; i < 32; i++) {
    if (mask & (1UL << (31 - i))) {
      Serial.printf("PID 0x%02X\n", startPid + i);
    }
  }
}

void setup() {
  Serial.begin(115200);
  ELM_PORT.begin(ELM_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("กำลังเชื่อมต่อ ELM327 ผ่าน HC-05...");
  // protocol '6' = ISO 15765-4 CAN 11bit 500k
  if (!myELM327.begin(ELM_PORT, true, 2000, '6')) {
    Serial.println("ELM327 begin ไม่สำเร็จ");
    while (1) delay(1000);
  }
  Serial.println("เชื่อมต่อสำเร็จ");
}

void loop() {
  if (step == 0) {
    uint32_t r = myELM327.supportedPIDs_1_20();
    if (myELM327.nb_rx_state == ELM_SUCCESS) { mask1 = r; step = 1; }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { myELM327.printError(); step = 1; }
  }
  else if (step == 1) {
    uint32_t r = myELM327.supportedPIDs_21_40();
    if (myELM327.nb_rx_state == ELM_SUCCESS) { mask2 = r; step = 2; }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { myELM327.printError(); step = 2; }
  }
  else if (step == 2) {
    uint32_t r = myELM327.supportedPIDs_41_60();
    if (myELM327.nb_rx_state == ELM_SUCCESS) { mask3 = r; step = 3; }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { myELM327.printError(); step = 3; }
  }
  else if (step == 3) {
    Serial.println("--- PID 01-20 ---"); printSupported(mask1, 0x01);
    Serial.println("--- PID 21-40 ---"); printSupported(mask2, 0x21);
    Serial.println("--- PID 41-60 ---"); printSupported(mask3, 0x41);
    step = 4;
  }
}
