#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h> // 引入Json库简化解析（需在platformio.ini添加lib_deps=bblanchon/ArduinoJson@^6.21.0）

// ===================== 配置参数 =====================
// WiFi热点配置
const char* AP_SSID = "ESP32_Gyroscope";
const char* AP_PASS = "fzcnfzcn";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GW(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// 服务器端口
const uint16_t DNS_PORT = 53;
const uint16_t HTTP_PORT = 80;
const uint16_t WS_PORT = 81;

// PWM基础配置（50Hz舵机标准）
const int PWM_FREQUENCY = 50;      // 50Hz固定
const int PWM_RESOLUTION = 12;     // 12位分辨率（4096级）

// ===================== 全局实例 =====================
DNSServer dnsServer;
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WS_PORT);

// 舵机通道缓存（存储最新的引脚和脉宽）
struct ServoChannel {
  int pin = -1;    // 引脚（-1表示未配置）
  int pulseUs = 1500; // 脉宽（默认中位1500us）
  int channel = -1; // PWM通道（动态分配）
} servoPitch, servoRoll, servoYaw;

// 打印频率控制（避免串口刷屏）
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // 100ms打印一次关键信息

// ===================== 工具函数 =====================
// 初始化/更新PWM通道（动态绑定引脚）
void updatePWMChannel(ServoChannel& sc) {
  if (sc.pin == -1) return;
  
  // 未分配通道则分配（0-15可选，这里用0/1/2）
  if (sc.channel == -1) {
    if (&sc == &servoPitch) sc.channel = 0;
    else if (&sc == &servoRoll) sc.channel = 1;
    else if (&sc == &servoYaw) sc.channel = 2;
    ledcSetup(sc.channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(sc.pin, sc.channel);
  }
  
  // 转换脉宽到PWM值：500-2500us对应20ms周期（20000us），12位分辨率4095
  int pwmValue = (sc.pulseUs * 4095) / 20000;
  ledcWrite(sc.channel, pwmValue);
}

// WebServer重定向到GitHub Pages
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>重定向中...</title></head>";
  html += "<body><script>window.location.href='https://fz-cn-114-514.github.io/';</script>";
  html += "<p>正在跳转到控制页面...</p></body></html>";
  server.send(200, "text/html", html);
}

// WebSocket事件处理（核心：解析网页下发的脉宽指令）
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] 客户端 #%u 断开连接\n", num);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[WS] 客户端 #%u 连接 (IP: %s)\n", num, ip.toString().c_str());
      webSocket.sendTXT(num, "ESP32 Servo Controller Ready");
      break;
    }
      
    case WStype_TEXT: {
      String msg = String((char*)payload);
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, msg);
      
      // 解析成功则更新舵机参数
      if (!err) {
        // 解析Pitch通道
        if (doc.containsKey("P-PIN") && doc.containsKey("P-PWM")) {
          servoPitch.pin = doc["P-PIN"];
          servoPitch.pulseUs = doc["P-PWM"];
          updatePWMChannel(servoPitch);
        }
        // 解析Roll通道
        if (doc.containsKey("R-PIN") && doc.containsKey("R-PWM")) {
          servoRoll.pin = doc["R-PIN"];
          servoRoll.pulseUs = doc["R-PWM"];
          updatePWMChannel(servoRoll);
        }
        // 解析Yaw通道
        if (doc.containsKey("Y-PIN") && doc.containsKey("Y-PWM")) {
          servoYaw.pin = doc["Y-PIN"];
          servoYaw.pulseUs = doc["Y-PWM"];
          updatePWMChannel(servoYaw);
        }
        
        // 低频打印调试信息
        unsigned long now = millis();
        if (now - lastPrintTime >= PRINT_INTERVAL) {
          Serial.printf("[PWM] P(%d):%dus | R(%d):%dus | Y(%d):%dus\n",
                        servoPitch.pin, servoPitch.pulseUs,
                        servoRoll.pin, servoRoll.pulseUs,
                        servoYaw.pin, servoYaw.pulseUs);
          lastPrintTime = now;
        }
      }
      
      // 处理指令（映射归零仅需网页端处理，这里仅回执）
      if (msg == "reset_mapping") {
        webSocket.sendTXT(num, "Mapping Reset ACK");
        Serial.println("[CMD] 收到映射归零指令（网页端处理）");
      }
      break;
    }
    
    default: break;
  }
}

// ===================== 初始化 =====================
void setup() {
  // 串口初始化
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== ESP32 舵机控制器启动 ===");
  
  // WiFi热点配置
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);
  Serial.printf("[WiFi] 热点启动: %s (IP: %s)\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  
  // DNS服务器（所有域名重定向到ESP32）
  dnsServer.start(DNS_PORT, "*", AP_IP);
  Serial.println("[DNS] 重定向服务启动");
  
  // WebServer
  server.on("/", handleRoot);
  server.begin();
  Serial.println("[WebServer] 启动 (端口80)");
  
  // WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("[WebSocket] 启动 (端口81)");
  
  // 初始舵机中位（默认引脚12/13/14，可被网页覆盖）
  servoPitch.pin = 12;
  servoRoll.pin = 13;
  servoYaw.pin = 14;
  updatePWMChannel(servoPitch);
  updatePWMChannel(servoRoll);
  updatePWMChannel(servoYaw);
  
  Serial.println("[初始化] 完成，等待网页连接...");
}

// ===================== 主循环 =====================
void loop() {
  dnsServer.processNextRequest();  // 处理DNS请求
  server.handleClient();           // 处理Web请求
  webSocket.loop();                // 处理WebSocket（高频率响应）
  
  // 保证PWM输出稳定性（50Hz固定，无需额外处理，ledc硬件自动生成）
}