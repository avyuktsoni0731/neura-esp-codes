#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <Arduino_GFX_Library.h>

// --------------------------------------------------
// CONFIGURATION
// --------------------------------------------------
#define AP_SSID "Neura"
#define AP_PASSWORD "neura@123"

// I2C pins for MPU6500
#define SDA_PIN 33
#define SCL_PIN 34
#define MPU6500_ADDR 0x68

// TFT pins
#define TFT_DC   13
#define TFT_RST  14
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_CS   -1

// Colors
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Objects
WebSocketsServer webSocket = WebSocketsServer(81);
MPU6500_WE myMPU = MPU6500_WE(MPU6500_ADDR);
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0, true, 240, 240, 0, 0);

// Sensor values
float currentPitch = 0;
float currentRoll = 0;
float currentYaw = 0;
unsigned long currentTimestamp = 0;

// Timing
unsigned long lastSensorUpdate = 0;
unsigned long lastWebSocketSend = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long sendInterval = 100;
const unsigned long displayInterval = 100;

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[+] ESP32-S3 PICO Complete System");
  Serial.println("[+] MPU6500 + WebSocket + TFT Display\n");

  // 1. Init TFT Display
  Serial.println("[+] Initializing TFT...");
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(60, 20);
  gfx->println("NEURA");
  gfx->setTextSize(1);
  gfx->setCursor(20, 60);
  gfx->println("Initializing systems...");
  Serial.println("[+] TFT OK");

  // 2. Init I2C
  Serial.println("[+] Initializing I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  gfx->setCursor(20, 80);
  gfx->println("I2C initialized");
  Serial.println("[+] I2C on GPIO33/34");

  // 3. Init MPU6500
  Serial.println("[+] Initializing MPU6500...");
  gfx->setCursor(20, 100);
  gfx->println("MPU6500...");
  
  if (!myMPU.init()) {
    Serial.println("[-] MPU6500 FAILED!");
    gfx->setTextColor(RED);
    gfx->setCursor(20, 120);
    gfx->println("MPU6500 ERROR!");
    while (1) delay(1000);
  }
  
  myMPU.autoOffsets();
  myMPU.enableGyrDLPF();
  myMPU.setGyrDLPF(MPU6500_DLPF_6);
  myMPU.enableAccDLPF(true);
  myMPU.setAccDLPF(MPU6500_DLPF_6);
  
  gfx->setTextColor(GREEN);
  gfx->setCursor(20, 120);
  gfx->println("MPU6500 OK");
  Serial.println("[+] MPU6500 OK");

  // 4. WiFi AP
  Serial.println("[+] Starting WiFi AP...");
  gfx->setTextColor(WHITE);
  gfx->setCursor(20, 140);
  gfx->println("Starting WiFi AP...");
  
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 6);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[+] AP IP: ");
  Serial.println(IP);
  
  gfx->setTextColor(GREEN);
  gfx->setCursor(20, 160);
  gfx->print("IP: ");
  gfx->println(IP);

  // 5. WebSocket
  Serial.println("[+] Starting WebSocket...");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  gfx->setCursor(20, 180);
  gfx->println("WebSocket :81");
  Serial.println("[+] WebSocket :81");

  delay(2000);
  gfx->fillScreen(BLACK);
  
  Serial.println("\n=================================");
  Serial.println("[+] SYSTEM READY!");
  Serial.println("=================================");
  Serial.println("WiFi SSID: " + String(AP_SSID));
  Serial.println("Password:  " + String(AP_PASSWORD));
  Serial.println("WebSocket: ws://192.168.4.1:81");
  Serial.println("=================================\n");
}

// --------------------------------------------------
// WebSocket Event
// --------------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.print("[WS] Client [");
      Serial.print(num);
      Serial.println("] disconnected");
      break;
    case WStype_CONNECTED:
      Serial.print("[WS] Client [");
      Serial.print(num);
      Serial.println("] connected");
      break;
  }
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop() {
  webSocket.loop();

  // Update sensor values (50ms)
  if (millis() - lastSensorUpdate >= 50) {
    lastSensorUpdate = millis();
    
    xyzFloat angles = myMPU.getAngles();
    currentPitch = angles.x;
    currentRoll = angles.y;
    currentYaw = angles.z;
    currentTimestamp = millis();
  }

  // Send WebSocket data (100ms)
  if (millis() - lastWebSocketSend >= sendInterval) {
    lastWebSocketSend = millis();
    sendSensorData();
  }

  // Update display (100ms)
  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
}

// --------------------------------------------------
// Send WebSocket Data
// --------------------------------------------------
void sendSensorData() {
  StaticJsonDocument<200> doc;
  doc["pitch"] = currentPitch;
  doc["roll"] = currentRoll;
  doc["yaw"] = currentYaw;
  doc["timestamp"] = currentTimestamp;

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
  
  // Print occasionally to avoid spam
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    Serial.println(jsonString);
    lastPrint = millis();
  }
}

// --------------------------------------------------
// Update TFT Display
// --------------------------------------------------
void updateDisplay() {
  // Title
  gfx->setTextSize(2);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(70, 10);
  gfx->println("NEURA");
  
  // Draw boxes
  gfx->drawRect(10, 50, 220, 50, WHITE);
  gfx->drawRect(10, 110, 220, 50, WHITE);
  gfx->drawRect(10, 170, 220, 50, WHITE);
  
  // Pitch
  gfx->setTextSize(1);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(20, 60);
  gfx->println("PITCH");
  gfx->setTextSize(2);
  gfx->setTextColor(GREEN, BLACK);
  gfx->setCursor(20, 75);
  gfx->print("       "); // Clear
  gfx->setCursor(20, 75);
  gfx->print(currentPitch, 1);
  gfx->setTextSize(1);
  gfx->print(" deg");
  
  // Roll
  gfx->setTextSize(1);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(20, 120);
  gfx->println("ROLL");
  gfx->setTextSize(2);
  gfx->setTextColor(YELLOW, BLACK);
  gfx->setCursor(20, 135);
  gfx->print("       "); // Clear
  gfx->setCursor(20, 135);
  gfx->print(currentRoll, 1);
  gfx->setTextSize(1);
  gfx->print(" deg");
  
  // Yaw
  gfx->setTextSize(1);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(20, 180);
  gfx->println("YAW");
  gfx->setTextSize(2);
  gfx->setTextColor(MAGENTA, BLACK);
  gfx->setCursor(20, 195);
  gfx->print("       "); // Clear
  gfx->setCursor(20, 195);
  gfx->print(currentYaw, 1);
  gfx->setTextSize(1);
  gfx->print(" deg");
  
  // Connection indicator
  gfx->fillCircle(215, 15, 3, webSocket.connectedClients() > 0 ? GREEN : RED);
}