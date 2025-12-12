/* ============================================================
 *  Final Project - Arduino Mega (Handshake Version)
 *  功能：
 *    - RFID (MFRC522) 讀卡，將 UID 傳給 Raspberry Pi
 *    - LCD(I2C 0x27, 20x4) 顯示系統狀態
 *    - BH1750 光照度感測
 *    - DHT11 溫濕度感測 (腳位 2)
 *    - DC 馬達 PWM 開迴路控制 (MOTOR_PWM_PIN)
 *    - 交握式感測資料：REQ,SENSOR -> SENSOR,LUX=...,H=...,T=...
 *    - 一般使用者：MOTOR,SPD=xxx（需要 AUTH,OK 才能動）
 *    - 監督者：SUP_MOTOR,SPD=xxx（不看 AUTH，總是直接動）
 *
 *  ✅ 新增/修正：
 *    - 蜂鳴器：
 *        RFID 任意感應：0.5s 短音
 *        AUTH OK：兩次短音
 *        AUTH FAIL：一次長音
 *    - supervisor block LCD 更新：SYS,BLOCK=0/1（由 arduino_bridge 送過來）
 *    - logout LCD 更新：AUTH,LOGOUT（由 arduino_bridge 送過來）
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
#define RFID_SS_PIN   10
#define RFID_RST_PIN   9

// DHT11
#define DHTPIN   2
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

// 馬達 PWM 腳位
#define MOTOR_PWM_PIN 5

// 蜂鳴器腳位
#define BUZZERPIN 3

// Serial 給 RPi
#define SERIAL_RPI Serial

// ===========================
//  全域物件
// ===========================

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
LiquidCrystal_I2C LCD(0x27, 20, 4);
BH1750FVI LightSensor;

// 授權狀態（一般使用者用）
bool g_isAuthorized = false;

// 系統是否被 supervisor block（由 SYS,BLOCK=0/1 控制）
bool g_isBlocked = false;

// 收 Serial 指令的緩衝區
const uint8_t CMD_BUF_SIZE = 64;
char cmdBuffer[CMD_BUF_SIZE];
uint8_t cmdIndex = 0;

// ===========================
// 蜂鳴器：非阻塞狀態機
// ===========================
enum BuzzerPattern {
  BUZ_NONE = 0,
  BUZ_RFID,
  BUZ_AUTH_OK,
  BUZ_AUTH_FAIL
};

BuzzerPattern g_buz_pattern = BUZ_NONE;
uint8_t g_buz_step = 0;
unsigned long g_buz_step_start_ms = 0;
bool g_buz_on = false;

// pattern（ms）：ON, OFF, ON, OFF... 以 0 結束
const uint16_t PAT_RFID[] = {500, 0};                 // 0.5s on
const uint16_t PAT_OK[]   = {150, 120, 150, 0};       // short, gap, short
const uint16_t PAT_FAIL[] = {2000, 0};                // 2s long（你原本 2e3）

const uint16_t* currentPattern() {
  switch (g_buz_pattern) {
    case BUZ_RFID:     return PAT_RFID;
    case BUZ_AUTH_OK:  return PAT_OK;
    case BUZ_AUTH_FAIL:return PAT_FAIL;
    default:           return nullptr;
  }
}

void buzzerStart(BuzzerPattern p) {
  // 你可以在這裡做優先權：例如 FAIL 蓋過 RFID
  g_buz_pattern = p;
  g_buz_step = 0;
  g_buz_step_start_ms = millis();
  g_buz_on = false;
  digitalWrite(BUZZERPIN, LOW);
}

void buzzerStop() {
  g_buz_pattern = BUZ_NONE;
  g_buz_step = 0;
  g_buz_on = false;
  digitalWrite(BUZZERPIN, LOW);
}

void buzzerUpdate() {
  if (g_buz_pattern == BUZ_NONE) return;
  const uint16_t* pat = currentPattern();
  if (!pat) { buzzerStop(); return; }

  unsigned long now = millis();

  // step 指到 0：結束
  uint16_t dur = pat[g_buz_step];
  if (dur == 0) { buzzerStop(); return; }

  if (!g_buz_on) {
    // OFF waiting 或準備開始 ON
    // 如果目前是 OFF step（奇數 step），就等 OFF 時間到再前進
    if (g_buz_step % 2 == 1) {
      if (now - g_buz_step_start_ms >= dur) {
        g_buz_step++;
        g_buz_step_start_ms = now;
      }
      return;
    }

    // ON step：開啟蜂鳴器
    digitalWrite(BUZZERPIN, HIGH);
    g_buz_on = true;
    g_buz_step_start_ms = now;
    return;
  } else {
    // ON step 計時到：關閉並進入下一步（OFF）
    if (now - g_buz_step_start_ms >= dur) {
      digitalWrite(BUZZERPIN, LOW);
      g_buz_on = false;
      g_buz_step++;
      g_buz_step_start_ms = now;
      return;
    }
  }
}

// ===========================
// 前置宣告
// ===========================
void handleSerialInput();
void processCommand(char *cmd);
void handleRFID();
void sendRFIDFrame(const String &uidHex);
void sendSensorFrame();
void lcdShowLine(uint8_t row, const String &msg);
void lcdShowIdle();
void lcdShowBlocked();
void lcdShowAuthWait();
void lcdUpdateMotorLine(const String &prefix, int spd);

// ===========================
// setup()
// ===========================
void setup() {
  SERIAL_RPI.begin(115200);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);

  pinMode(BUZZERPIN, OUTPUT);
  digitalWrite(BUZZERPIN, LOW);

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

  lcdShowIdle();
}

// ===========================
// loop()
// ===========================
void loop() {
  // 非阻塞蜂鳴器更新（放最前面，確保準時）
  buzzerUpdate();

  // 1. 處理來自 RPi 的指令
  handleSerialInput();

  // 2. 處理 RFID 刷卡
  handleRFID();
}

// ===========================
// Serial 指令接收 + 分段
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
        // overflow，丟掉這一行
        cmdIndex = 0;
      }
    }
  }
}

// ===========================
// 處理指令內容
//   SYS,BLOCK=0/1
//   AUTH,OK / AUTH,FAIL / AUTH,LOGOUT
//   MOTOR,SPD=xxx       （一般使用者，需 AUTH）
//   SUP_MOTOR,SPD=xxx   （監督者，不看 AUTH）
//   REQ,SENSOR
// ===========================
void processCommand(char *cmd) {
  // 1) SYS,BLOCK=0/1（由 arduino_bridge 送過來）
  if (strncmp(cmd, "SYS,BLOCK=", 10) == 0) {
    int v = atoi(cmd + 10);
    g_isBlocked = (v != 0);

    if (g_isBlocked) {
      // block：取消授權、停馬達、顯示 blocked
      g_isAuthorized = false;
      analogWrite(MOTOR_PWM_PIN, 0);
      lcdShowBlocked();
    } else {
      // unblock：回 idle
      g_isAuthorized = false;
      lcdShowIdle();
    }

    SERIAL_RPI.print("ACK,SYS_BLOCK=");
    SERIAL_RPI.println(g_isBlocked ? 1 : 0);
    return;
  }

  // 2) AUTH,OK / AUTH,FAIL / AUTH,LOGOUT
  if (strncmp(cmd, "AUTH,OK", 7) == 0) {
    if (g_isBlocked) {
      // 被 block 時：即使上位機說 OK，也不進入授權
      g_isAuthorized = false;
      lcdShowBlocked();
      SERIAL_RPI.println("ACK,AUTH=BLOCKED");
      buzzerStart(BUZ_AUTH_FAIL);
      return;
    }

    g_isAuthorized = true;
    lcdShowLine(2, "Auth: OK   ");
    LCD.setCursor(0, 1);
    LCD.print("Welcome!           ");
    SERIAL_RPI.println("ACK,AUTH=OK");
    buzzerStart(BUZ_AUTH_OK);
    return;
  }

  if (strncmp(cmd, "AUTH,FAIL", 9) == 0) {
    g_isAuthorized = false;
    if (g_isBlocked) lcdShowBlocked();
    else {
      lcdShowLine(2, "Auth: FAIL");
      LCD.setCursor(0, 1);
      LCD.print("Access Denied      ");
    }
    SERIAL_RPI.println("ACK,AUTH=FAIL");
    buzzerStart(BUZ_AUTH_FAIL);
    return;
  }

  if (strncmp(cmd, "AUTH,LOGOUT", 11) == 0) {
    g_isAuthorized = false;
    if (g_isBlocked) lcdShowBlocked();
    else lcdShowIdle();
    SERIAL_RPI.println("ACK,AUTH=LOGOUT");
    return;
  }

  // 3) 一般使用者馬達：MOTOR,SPD=xxx（需要已授權；block 時禁止）
  if (strncmp(cmd, "MOTOR,SPD=", 10) == 0) {
    int spd = atoi(cmd + 10);
    if (spd < 0) spd = 0;
    if (spd > 255) spd = 255;

    if (g_isBlocked) {
      analogWrite(MOTOR_PWM_PIN, 0);
      SERIAL_RPI.println("ERR,SYSTEM_BLOCKED");
      return;
    }

    if (g_isAuthorized) {
      analogWrite(MOTOR_PWM_PIN, spd);
      lcdUpdateMotorLine("Motor:", spd);
      SERIAL_RPI.print("ACK,MOTOR=");
      SERIAL_RPI.println(spd);
    } else {
      SERIAL_RPI.println("ERR,UNAUTHORIZED_MOTOR_CMD");
    }
    return;
  }

  // 4) 監督者馬達：SUP_MOTOR,SPD=xxx（不看 g_isAuthorized；block 時仍可用）
  if (strncmp(cmd, "SUP_MOTOR,SPD=", 14) == 0) {
    int spd = atoi(cmd + 14);
    if (spd < 0) spd = 0;
    if (spd > 255) spd = 255;

    analogWrite(MOTOR_PWM_PIN, spd);
    lcdUpdateMotorLine("Sup:", spd);
    SERIAL_RPI.print("ACK,SUP_MOTOR=");
    SERIAL_RPI.println(spd);
    return;
  }

  // 5) REQ,SENSOR
  if (strncmp(cmd, "REQ,SENSOR", 10) == 0) {
    sendSensorFrame();
    return;
  }

  SERIAL_RPI.print("WARN,UNKNOWN_CMD=");
  SERIAL_RPI.println(cmd);
}

// ===========================
// 處理 RFID 刷卡
// ===========================
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  // 任意 RFID 感應：0.5 秒短音（非阻塞）
  buzzerStart(BUZ_RFID);

  String uidHex = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uidHex += "0";
    uidHex += String(rfid.uid.uidByte[i], HEX);
  }
  uidHex.toUpperCase();

  if (g_isBlocked) {
    lcdShowBlocked();
    lcdShowLine(1, "Card ignored       ");
    lcdShowLine(2, "Auth: BLOCK        ");
  } else {
    lcdShowLine(0, "Card detected     ");
    lcdShowLine(1, "UID: " + uidHex);
    lcdShowAuthWait();
  }

  sendRFIDFrame(uidHex);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void sendRFIDFrame(const String &uidHex) {
  SERIAL_RPI.print("RFID,UID=");
  SERIAL_RPI.println(uidHex);
}

// ===========================
// 量測一次感測器並送出
// SENSOR,LUX=...,H=...,T=...
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
// LCD 顯示工具
// ===========================
void lcdShowLine(uint8_t row, const String &msg) {
  if (row > 3) return;
  LCD.setCursor(0, row);
  LCD.print("                    "); // 20 spaces
  LCD.setCursor(0, row);
  LCD.print(msg);
}

void lcdShowIdle() {
  LCD.clear();
  lcdShowLine(0, "System Ready");
  lcdShowLine(1, "Waiting for card");
  lcdShowLine(2, "Auth: NONE");
  lcdShowLine(3, "Motor: 0");
}

void lcdShowBlocked() {
  LCD.clear();
  lcdShowLine(0, "SYSTEM BLOCKED");
  lcdShowLine(1, "Supervisor mode");
  lcdShowLine(2, "Auth: BLOCK");
  lcdShowLine(3, "Motor: 0");
}

void lcdShowAuthWait() {
  lcdShowLine(2, "Auth: WAIT PWD     ");
}

void lcdUpdateMotorLine(const String &prefix, int spd) {
  char buf[21];
  // 讓文字不殘留
  snprintf(buf, sizeof(buf), "%s %d            ", prefix.c_str(), spd);
  lcdShowLine(3, String(buf));
}
