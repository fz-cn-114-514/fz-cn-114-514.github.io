#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>

// 配置参数
const char* AP_SSID = "ESP32_Gyroscope";
const char* AP_PASS = "fzcnfzcn";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GW(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// 服务器端口
const uint16_t DNS_PORT = 53;
const uint16_t HTTP_PORT = 80;
const uint16_t WS_PORT = 81;

// 舵机GPIO引脚定义（D12、D13、D14对应GPIO12、13、14）
const int SERVO_PIN_PITCH = 12;
const int SERVO_PIN_ROLL = 13;
const int SERVO_PIN_YAW = 14;

// 舵机PWM配置
const int PWM_FREQUENCY = 50;      // 50Hz
const int PWM_RESOLUTION = 12;     // 12位分辨率
const int PWM_CHANNEL_PITCH = 0;
const int PWM_CHANNEL_ROLL = 1;
const int PWM_CHANNEL_YAW = 2;

// 实例化服务器
DNSServer dnsServer;
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WS_PORT);

// 通道配置结构体
typedef struct {
  float rawValue;          // 原始传感器值
  float mappedValue;       // 映射后的值
  float rate;              // 感度倍率
  float offset;            // 姿态归零偏移量
  int pulseWidth;          // 当前脉宽
  int minPulse;            // 最小脉宽
  int maxPulse;            // 最大脉宽
} ChannelConfig;

// 系统配置
typedef struct {
  bool controlEnabled;     // 启用控制
  bool operationLocked;    // 操作锁定
  ChannelConfig pitch;     // 俯仰角通道
  ChannelConfig roll;      // 横滚角通道
  ChannelConfig yaw;        // 偏航角通道
} SystemConfig;

// 全局配置变量
SystemConfig config;

// 打印频率控制
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000; // 1秒打印一次

// 函数声明
void initPWM();
void updateServoPWM();
void servoReset();
void attitudeReset();
void updateGyroData(float pitchRaw, float rollRaw, float yawRaw);
void parseConfigData(String json);
void handleDNSRequest();
void handleRoot();

// DNS重定向处理
void handleDNSRequest() {
  dnsServer.processNextRequest();
}

// Web服务器处理
void handleRoot() {
  // 返回一个HTML页面，使用JavaScript立即重定向到GitHub Pages
  String html = "<!DOCTYPE html>\n";
  html += "<html>\n";
  html += "<head>\n";
  html += "<meta charset=\"UTF-8\">\n";
  html += "<title>重定向到GitHub Pages</title>\n";
  html += "</head>\n";
  html += "<body>\n";
  html += "<script>\n";
  html += "window.location.href = 'https://fz-cn-114-514.github.io/';\n";
  html += "</script>\n";
  html += "<p>正在重定向到GitHub Pages...</p>\n";
  html += "</body>\n";
  html += "</html>\n";
  
  server.send(200, "text/html", html);
}

// 初始化PWM
void initPWM() {
  // 配置PWM通道
  ledcSetup(PWM_CHANNEL_PITCH, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_ROLL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_YAW, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // 绑定GPIO引脚
  ledcAttachPin(SERVO_PIN_PITCH, PWM_CHANNEL_PITCH);
  ledcAttachPin(SERVO_PIN_ROLL, PWM_CHANNEL_ROLL);
  ledcAttachPin(SERVO_PIN_YAW, PWM_CHANNEL_YAW);
  
  // 初始化为中心位置
  servoReset();
}

// 更新PWM输出
void updateServoPWM() {
  // 将脉宽转换为PWM值（500-2500us对应整个20ms周期）
  // 20ms = 20000us，12位分辨率下4095对应20000us
  // 计算方式：pwmValue = (pulseWidth * 4095) / 20000
  int pwmPitch = (config.pitch.pulseWidth * 4095) / 20000;
  int pwmRoll = (config.roll.pulseWidth * 4095) / 20000;
  int pwmYaw = (config.yaw.pulseWidth * 4095) / 20000;
  
  // 设置PWM输出
  ledcWrite(PWM_CHANNEL_PITCH, pwmPitch);
  ledcWrite(PWM_CHANNEL_ROLL, pwmRoll);
  ledcWrite(PWM_CHANNEL_YAW, pwmYaw);
  
  // 限制打印频率为1秒一次
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    Serial.printf("[舵机脉宽] Pitch: %d, Roll: %d, Yaw: %d\n", 
                 config.pitch.pulseWidth, config.roll.pulseWidth, config.yaw.pulseWidth);
    // 不需要更新lastPrintTime，因为在updateGyroData函数中已经更新了
  }
}

// 舵机回中
void servoReset() {
  config.pitch.pulseWidth = 1500;
  config.roll.pulseWidth = 1500;
  config.yaw.pulseWidth = 1500;
  
  updateServoPWM();
}

// 姿态归零
void attitudeReset() {
  // 设置当前原始值为偏移量
  config.pitch.offset = config.pitch.rawValue;
  config.roll.offset = config.roll.rawValue;
  config.yaw.offset = config.yaw.rawValue;
  
  // 立即更新映射角度（考虑新的偏移量，范围限制在±180°）
  config.pitch.mappedValue = constrain((config.pitch.rawValue - config.pitch.offset), -180.0, 180.0);
  config.roll.mappedValue = constrain((config.roll.rawValue - config.roll.offset), -180.0, 180.0);
  config.yaw.mappedValue = constrain((config.yaw.rawValue - config.yaw.offset), -180.0, 180.0);
  
  // 如果控制启用，更新脉宽
  if (config.controlEnabled) {
    // 计算脉宽
    int pitchPulse = 1500 + (config.pitch.mappedValue * config.pitch.rate);
    int rollPulse = 1500 + (config.roll.mappedValue * config.roll.rate);
    int yawPulse = 1500 + (config.yaw.mappedValue * config.yaw.rate);
    
    // 限制脉宽在最小和最大值之间
    config.pitch.pulseWidth = constrain(pitchPulse, config.pitch.minPulse, config.pitch.maxPulse);
    config.roll.pulseWidth = constrain(rollPulse, config.roll.minPulse, config.roll.maxPulse);
    config.yaw.pulseWidth = constrain(yawPulse, config.yaw.minPulse, config.yaw.maxPulse);
    
    // 更新PWM输出
    updateServoPWM();
  }
  
  // 立即发送更新后的数据到所有WebSocket客户端（使用sprintf优化内存分配）
  char dataBuffer[200];
  sprintf(dataBuffer, "{\"pitch_mapped\":%.1f,\"roll_mapped\":%.1f,\"yaw_mapped\":%.1f,\"pitch_pulse\":%d,\"roll_pulse\":%d,\"yaw_pulse\":%d}",
          config.pitch.mappedValue, config.roll.mappedValue, config.yaw.mappedValue,
          config.pitch.pulseWidth, config.roll.pulseWidth, config.yaw.pulseWidth);
  webSocket.broadcastTXT(dataBuffer);
}

// 更新陀螺仪数据
void updateGyroData(float pitchRaw, float rollRaw, float yawRaw) {
  // 更新原始值
  config.pitch.rawValue = pitchRaw;
  config.roll.rawValue = rollRaw;
  config.yaw.rawValue = yawRaw;
  
  // 计算映射值（考虑偏移量，范围限制在±180°，确保舵机能够正确响应）
  config.pitch.mappedValue = constrain((pitchRaw - config.pitch.offset), -180.0, 180.0);
  config.roll.mappedValue = constrain((rollRaw - config.roll.offset), -180.0, 180.0);
  config.yaw.mappedValue = constrain((yawRaw - config.yaw.offset), -180.0, 180.0);
  
  // 将映射值转换为脉宽：输出脉宽 = 1500 + (映射角度 * rate)
  // 1500us为中位，映射角度范围±180°，rate控制灵敏度
  if (config.controlEnabled) {
    // 计算脉宽
    int pitchPulse = 1500 + (config.pitch.mappedValue * config.pitch.rate);
    int rollPulse = 1500 + (config.roll.mappedValue * config.roll.rate);
    int yawPulse = 1500 + (config.yaw.mappedValue * config.yaw.rate);
    
    // 限制脉宽在最小和最大值之间
    config.pitch.pulseWidth = constrain(pitchPulse, config.pitch.minPulse, config.pitch.maxPulse);
    config.roll.pulseWidth = constrain(rollPulse, config.roll.minPulse, config.roll.maxPulse);
    config.yaw.pulseWidth = constrain(yawPulse, config.yaw.minPulse, config.yaw.maxPulse);
    
    // 更新PWM输出
    updateServoPWM();
  }
  
  // 限制打印频率为1秒一次
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    Serial.printf("[陀螺仪数据] Pitch: %.1f, Roll: %.1f, Yaw: %.1f, Control: %s\n", 
                 pitchRaw, rollRaw, yawRaw, config.controlEnabled ? "Enabled" : "Disabled");
    lastPrintTime = currentTime;
  }
  
  // 发送实时数据到所有WebSocket客户端（使用sprintf优化内存分配）
  char dataBuffer[200];
  sprintf(dataBuffer, "{\"pitch_mapped\":%.1f,\"roll_mapped\":%.1f,\"yaw_mapped\":%.1f,\"pitch_pulse\":%d,\"roll_pulse\":%d,\"yaw_pulse\":%d}",
          config.pitch.mappedValue, config.roll.mappedValue, config.yaw.mappedValue,
          config.pitch.pulseWidth, config.roll.pulseWidth, config.yaw.pulseWidth);
  webSocket.broadcastTXT(dataBuffer);
}

// 辅助函数：从字符串中提取浮点数
float extractFloat(const char* json, int startIndex, int& endIndex) {
  const char* ptr = json + startIndex;
  // 找到冒号位置
  while (*ptr != ':' && *ptr != '\0') ptr++;
  if (*ptr == '\0') return 0.0;
  ptr++;
  // 跳过空格
  while (*ptr == ' ' || *ptr == '\t') ptr++;
  
  // 提取数值，直到遇到非数字字符
  char valueStr[16] = {0};
  int i = 0;
  while ((i < 15) && (*ptr >= '0' && *ptr <= '9' || *ptr == '.' || *ptr == '-' || *ptr == '+')) {
    valueStr[i++] = *ptr++;
  }
  valueStr[i] = '\0';
  
  endIndex = (ptr - json);
  return atof(valueStr);
}

// 辅助函数：从字符串中提取整数
int extractInt(const char* json, int startIndex, int& endIndex) {
  const char* ptr = json + startIndex;
  // 找到冒号位置
  while (*ptr != ':' && *ptr != '\0') ptr++;
  if (*ptr == '\0') return 0;
  ptr++;
  // 跳过空格
  while (*ptr == ' ' || *ptr == '\t') ptr++;
  
  // 提取数值，直到遇到非数字字符
  char valueStr[16] = {0};
  int i = 0;
  while ((i < 15) && (*ptr >= '0' && *ptr <= '9' || *ptr == '-' || *ptr == '+')) {
    valueStr[i++] = *ptr++;
  }
  valueStr[i] = '\0';
  
  endIndex = (ptr - json);
  return atoi(valueStr);
}

// 解析JSON配置数据（使用C风格字符串处理，减少内存分配）
void parseConfigData(String json) {
  const char* jsonStr = json.c_str();
  
  // 解析controlEnabled
  char* enabledPtr = strstr(jsonStr, "controlEnabled");
  if (enabledPtr != nullptr) {
    int endIndex;
    int enabledValue = extractInt(enabledPtr, 0, endIndex);
    config.controlEnabled = (enabledValue == 1);
  }
  
  // 解析operationLocked
  char* lockedPtr = strstr(jsonStr, "operationLocked");
  if (lockedPtr != nullptr) {
    int endIndex;
    int lockedValue = extractInt(lockedPtr, 0, endIndex);
    config.operationLocked = (lockedValue == 1);
  }
  
  // 解析Pitch通道配置
  char* pitchPtr = strstr(jsonStr, "\"pitch\":");
  if (pitchPtr != nullptr) {
    // 解析rate
    char* ratePtr = strstr(pitchPtr, "\"rate\":");
    if (ratePtr != nullptr) {
      int endIndex;
      config.pitch.rate = extractFloat(ratePtr, 0, endIndex);
    }
    
    // 解析minPulse
    char* minPtr = strstr(pitchPtr, "\"minPulse\":");
    if (minPtr != nullptr) {
      int endIndex;
      config.pitch.minPulse = extractInt(minPtr, 0, endIndex);
    }
    
    // 解析maxPulse
    char* maxPtr = strstr(pitchPtr, "\"maxPulse\":");
    if (maxPtr != nullptr) {
      int endIndex;
      config.pitch.maxPulse = extractInt(maxPtr, 0, endIndex);
    }
  }
  
  // 解析Roll通道配置
  char* rollPtr = strstr(jsonStr, "\"roll\":");
  if (rollPtr != nullptr) {
    // 解析rate
    char* ratePtr = strstr(rollPtr, "\"rate\":");
    if (ratePtr != nullptr) {
      int endIndex;
      config.roll.rate = extractFloat(ratePtr, 0, endIndex);
    }
    
    // 解析minPulse
    char* minPtr = strstr(rollPtr, "\"minPulse\":");
    if (minPtr != nullptr) {
      int endIndex;
      config.roll.minPulse = extractInt(minPtr, 0, endIndex);
    }
    
    // 解析maxPulse
    char* maxPtr = strstr(rollPtr, "\"maxPulse\":");
    if (maxPtr != nullptr) {
      int endIndex;
      config.roll.maxPulse = extractInt(maxPtr, 0, endIndex);
    }
  }
  
  // 解析Yaw通道配置
  char* yawPtr = strstr(jsonStr, "\"yaw\":");
  if (yawPtr != nullptr) {
    // 解析rate
    char* ratePtr = strstr(yawPtr, "\"rate\":");
    if (ratePtr != nullptr) {
      int endIndex;
      config.yaw.rate = extractFloat(ratePtr, 0, endIndex);
    }
    
    // 解析minPulse
    char* minPtr = strstr(yawPtr, "\"minPulse\":");
    if (minPtr != nullptr) {
      int endIndex;
      config.yaw.minPulse = extractInt(minPtr, 0, endIndex);
    }
    
    // 解析maxPulse
    char* maxPtr = strstr(yawPtr, "\"maxPulse\":");
    if (maxPtr != nullptr) {
      int endIndex;
      config.yaw.maxPulse = extractInt(maxPtr, 0, endIndex);
    }
  }
}

// WebSocket事件处理
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] 客户端 #%u 断开连接\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WebSocket] 客户端 #%u 连接, IP地址: %s\n", num, ip.toString().c_str());
        // 发送欢迎消息和当前配置
        webSocket.sendTXT(num, "Connected to ESP32 WebSocket Server");
      }
      break;
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        // 限制WebSocket消息打印频率为1秒一次
        unsigned long currentTime = millis();
        if (currentTime - lastPrintTime >= printInterval) {
            Serial.printf("[WebSocket] 客户端 #%u 发送: %s\n", num, message.c_str());
            // 注意：这里不更新lastPrintTime，让陀螺仪数据和WebSocket消息共享同一个打印计时器
        }
        
        // 解析JSON数据
        if (message.startsWith("{")) {
          // 检查是否包含配置数据（具有controlEnabled字段且不包含enabled字段）
          if (message.indexOf("controlEnabled") > 0 && message.indexOf("enabled") < 0) {
            parseConfigData(message);
          }
          // 检查是否包含陀螺仪数据（顶级pitch、roll、yaw字段）
          else if (message.indexOf("pitch") > 0 && message.indexOf("roll") > 0 && message.indexOf("yaw") > 0) {
            // 提取陀螺仪数据
            float pitch = 0.0, roll = 0.0, yaw = 0.0;
            bool enabled = config.controlEnabled; // 默认使用当前控制状态
            
            int pitchIndex = message.indexOf("pitch");
            if (pitchIndex > 0) {
              pitch = message.substring(message.indexOf(":", pitchIndex) + 1, message.indexOf(",", pitchIndex)).toFloat();
            }
            
            int rollIndex = message.indexOf("roll");
            if (rollIndex > 0) {
              roll = message.substring(message.indexOf(":", rollIndex) + 1, message.indexOf(",", rollIndex)).toFloat();
            }
            
            int yawIndex = message.indexOf("yaw");
            if (yawIndex > 0) {
              yaw = message.substring(message.indexOf(":", yawIndex) + 1, message.indexOf(",", yawIndex) > 0 ? message.indexOf(",", yawIndex) : message.indexOf("}", yawIndex)).toFloat();
            }
            
            // 提取enabled字段，用于更新控制状态
            int enabledIndex = message.indexOf("enabled");
            if (enabledIndex > 0) {
              int enabledValue = message.substring(message.indexOf(":", enabledIndex) + 1, message.indexOf(",", enabledIndex) > 0 ? message.indexOf(",", enabledIndex) : message.indexOf("}", enabledIndex)).toInt();
              config.controlEnabled = (enabledValue == 1);
            }
            
            // 更新陀螺仪数据
            updateGyroData(pitch, roll, yaw);
          }
        }
        
        // 处理控制指令
        if (message == "reset_servo") {
          Serial.println("[控制指令] 舵机回中");
          servoReset();
          webSocket.sendTXT(num, "Servo reset");
        } else if (message == "reset_attitude") {
          Serial.println("[控制指令] 姿态归零");
          attitudeReset();
          webSocket.sendTXT(num, "Attitude reset");
        }
      }
      break;
    default:
      break;
  }
}

// 初始化配置
void initConfig() {
  // 系统配置
  config.controlEnabled = true;     // 默认启用控制
  config.operationLocked = true;    // 默认锁定
  
  // Pitch通道配置
  config.pitch.rawValue = 0.0;
  config.pitch.mappedValue = 0.0;
  config.pitch.rate = 5.55;         // 默认感度倍率
  config.pitch.offset = 0.0;
  config.pitch.pulseWidth = 1500;   // 中心位置
  config.pitch.minPulse = 500;      // 最小脉宽
  config.pitch.maxPulse = 2500;     // 最大脉宽
  
  // Roll通道配置
  config.roll.rawValue = 0.0;
  config.roll.mappedValue = 0.0;
  config.roll.rate = 5.55;          // 默认感度倍率
  config.roll.offset = 0.0;
  config.roll.pulseWidth = 1500;    // 中心位置
  config.roll.minPulse = 500;       // 最小脉宽
  config.roll.maxPulse = 2500;      // 最大脉宽
  
  // Yaw通道配置
  config.yaw.rawValue = 0.0;
  config.yaw.mappedValue = 0.0;
  config.yaw.rate = 5.55;           // 默认感度倍率
  config.yaw.offset = 0.0;
  config.yaw.pulseWidth = 1500;     // 中心位置
  config.yaw.minPulse = 500;        // 最小脉宽
  config.yaw.maxPulse = 2500;       // 最大脉宽
}

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println("\nESP32 陀螺仪数据采集系统启动中...");
  
  // 初始化配置
  initConfig();
  
  // 配置PWM
  initPWM();
  
  // 配置WiFi热点
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  // 等待热点启动
  delay(1000);
  
  Serial.printf("[WiFi热点] SSID: %s, IP地址: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  
  // 配置DNS服务器 - 所有域名都重定向到ESP32
  dnsServer.start(DNS_PORT, "*", AP_IP);
  Serial.println("[DNS服务器] 已启动，所有域名重定向到ESP32");
  
  // 配置Web服务器
  server.on("/", handleRoot);
  server.begin();
  Serial.printf("[Web服务器] 已启动，端口: %d\n", HTTP_PORT);
  
  // 配置WebSocket服务器
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.printf("[WebSocket服务器] 已启动，端口: %d\n", WS_PORT);
  
  Serial.println("[系统] 初始化完成，等待客户端连接...");
}

void loop() {
  // 处理DNS请求
  dnsServer.processNextRequest();
  
  // 处理Web服务器请求
  server.handleClient();
  
  // 处理WebSocket事件
  webSocket.loop();
}