/* ============================================================
 *  Final Project - Arduino Mega (Handshake Version)
 *  功能：
 *    - RFID (MFRC522) 讀卡，將 UID 傳給 Raspberry Pi
 *    - LCD(I2C 0x27, 20x4) 顯示系統狀態
 *    - BH1750 光照度感測
 *    - DHT11 溫濕度感測 (腳位 2)
 *    - DC 馬達 PWM 開迴路控制 (MOTOR_PWM_PIN)
 *    - ✅ 改為「交握式」感測資料：
 *        RPi 送 REQ,SENSOR
 *        Arduino 量測一次 → 回 SENSOR,LUX=...,H=...,T=...
 * ============================================================ */

#include <SPI.h>
#include <MFRC522.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <BH1750FVI.h>

#include "DHT.h"

// ===========================
//  底層硬體腳位設定
// ===========================

// RFID (MFRC522)
#define RFID_SS_PIN   10   // SDA / SS
#define RFID_RST_PIN   9

// DHT11
#define DHTPIN   2
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

// 馬達 PWM 腳位
#define MOTOR_PWM_PIN 5

// Serial 給 RPi
#define SERIAL_RPI Serial

// ===========================
//  全域物件
// ===========================

// RFID
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);

// LCD: 0x27, 20x4
LiquidCrystal_I2C LCD(0x27, 20, 4);

// BH1750 光感測器
BH1750FVI LightSensor;

// 授權狀態
bool g_isAuthorized = false;

// 收 Serial 指令的緩衝區
const uint8_t CMD_BUF_SIZE = 64;
char cmdBuffer[CMD_BUF_SIZE];
uint8_t cmdIndex = 0;

// 前置宣告
void handleSerialInput();
void processCommand(char *cmd);
void handleRFID();
void sendRFIDFrame(const String &uidHex);
void sendSensorFrame();
void lcdShowLine(uint8_t row, const String &msg);

// ===========================
//  setup()
// ===========================
void setup() {
  SERIAL_RPI.begin(115200);

  // LCD 初始化
  LCD.begin();
  LCD.backlight();
  LCD.clear();
  lcdShowLine(0, "RFID System Init...");
  lcdShowLine(1, "Please wait");

  // RFID 初始化
  SPI.begin();
  rfid.PCD_Init();
  delay(50);

  // BH1750 初始化
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L);
  LightSensor.SetMode(Continuous_H_resolution_Mode);

  // DHT 初始化
  dht.begin();

  // 馬達 PWM 腳位
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);

  // 初始顯示
  LCD.clear();
  lcdShowLine(0, "System Ready");
  lcdShowLine(1, "Waiting for card");
  lcdShowLine(2, "Auth: NONE");
  lcdShowLine(3, "Motor: 0");
}

// ===========================
//  loop()
// ===========================
void loop() {
  // 1. 處理來自 RPi 的指令 (AUTH, MOTOR, REQ,SENSOR ...)
  handleSerialInput();

  // 2. 處理 RFID 刷卡
  handleRFID();
}

// ===========================
//  Serial 指令接收 + 分段
// ===========================
void handleSerialInput() {
  while (SERIAL_RPI.available() > 0) {
    char c = SERIAL_RPI.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else {
      if (cmdIndex < CMD_BUF_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        cmdIndex = 0;
      }
    }
  }
}

// ===========================
//  處理指令內容
//    AUTH,OK / AUTH,FAIL
//    MOTOR,SPD=xxx
//    ✅ REQ,SENSOR
// ===========================
void processCommand(char *cmd) {
  // AUTH,OK / AUTH,FAIL
  if (strncmp(cmd, "AUTH,OK", 7) == 0) {
    g_isAuthorized = true;
    lcdShowLine(2, "Auth: OK   ");
    LCD.setCursor(0, 1);
    LCD.print("Welcome!           ");
    SERIAL_RPI.println("ACK,AUTH=OK");
  }
  else if (strncmp(cmd, "AUTH,FAIL", 9) == 0) {
    g_isAuthorized = false;
    lcdShowLine(2, "Auth: FAIL");
    LCD.setCursor(0, 1);
    LCD.print("Access Denied      ");
    SERIAL_RPI.println("ACK,AUTH=FAIL");
  }
  // MOTOR,SPD=xxx
  else if (strncmp(cmd, "MOTOR,SPD=", 10) == 0) {
    int spd = atoi(cmd + 10);
    if (spd < 0) spd = 0;
    if (spd > 255) spd = 255;

    if (g_isAuthorized) {
      analogWrite(MOTOR_PWM_PIN, spd);

      char buf[20];
      snprintf(buf, sizeof(buf), "Motor: %d  ", spd);
      lcdShowLine(3, String(buf));

      SERIAL_RPI.print("ACK,MOTOR=");
      SERIAL_RPI.println(spd);
    } else {
      SERIAL_RPI.println("ERR,UNAUTHORIZED_MOTOR_CMD");
    }
  }
  // ✅ 新增：REQ,SENSOR → 量測一次感測器 → 回傳 SENSOR,.... 封包
  else if (strncmp(cmd, "REQ,SENSOR", 10) == 0) {
    sendSensorFrame();  // 量測並送出資料
  }
  else {
    SERIAL_RPI.print("WARN,UNKNOWN_CMD=");
    SERIAL_RPI.println(cmd);
  }
}

// ===========================
//  處理 RFID 刷卡
// ===========================
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  String uidHex = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uidHex += "0";
    uidHex += String(rfid.uid.uidByte[i], HEX);
  }
  uidHex.toUpperCase();

  lcdShowLine(0, "Card detected     ");
  lcdShowLine(1, "UID: " + uidHex);

  sendRFIDFrame(uidHex);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void sendRFIDFrame(const String &uidHex) {
  SERIAL_RPI.print("RFID,UID=");
  SERIAL_RPI.println(uidHex);
}

// ===========================
//  量測一次感測器並送出
//  SENSOR,LUX=...,H=...,T=...
// ===========================
void sendSensorFrame() {
  int lux = LightSensor.GetLightIntensity();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    SERIAL_RPI.println("SENSOR,ERROR=DHT_FAIL");
    return;
  }

  SERIAL_RPI.print("SENSOR,LUX=");
  SERIAL_RPI.print(lux);
  SERIAL_RPI.print(",H=");
  SERIAL_RPI.print(h, 1);
  SERIAL_RPI.print(",T=");
  SERIAL_RPI.print(t, 1);
  SERIAL_RPI.println();
}

// ===========================
//  LCD 顯示工具
// ===========================
void lcdShowLine(uint8_t row, const String &msg) {
  if (row > 3) return;
  LCD.setCursor(0, row);
  LCD.print("                    "); // 20 spaces
  LCD.setCursor(0, row);
  LCD.print(msg);
}
