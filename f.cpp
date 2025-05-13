#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <DNSServer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
ESP8266WebServer server(80);
DNSServer dnsServer;

// Wi-Fi AP settings
const char* ssid = "HealthMonitor";
const char* password = "12345678";
const byte DNS_PORT = 53;

// Sensor data
volatile int pulse = 0;
volatile int spo2 = 0;
bool beatDetected = false;
uint32_t redBuffer[100], irBuffer[100];
uint32_t irValue = 0;
unsigned long lastBeat = 0;

// Time & Alarm
volatile unsigned long timeBase = 0;
volatile int alarmHour = -1, alarmMinute = -1;
volatile bool alarmTriggered = false;
volatile bool blinkState = true;
volatile unsigned long lastBlink = 0;

// Loops timing
unsigned long lastSensorRead = 0;
unsigned long lastSpO2Check = 0;
unsigned long lastWifiCheck = 0;
const unsigned long sensorInterval = 20;
const unsigned long spo2Interval = 5000;
const unsigned long wifiCheckInterval = 10000;

// WiFi status
bool wifiInitialized = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // MAX30105 init
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor error!");
    display.display();
    while (1);
  }
  
  particleSensor.setup(50, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  setupWiFi();

  // Server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setTime", HTTP_GET, handleSetTime);
  server.on("/setAlarm", HTTP_GET, handleSetAlarm);
  server.on("/clearAlarm", HTTP_GET, handleClearAlarm);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });
  server.begin();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("System ready");
  display.display();
}

void setupWiFi() {
  // Configure and start Wi-Fi AP
  Serial.println("Configuring Wi-Fi AP...");
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  if (WiFi.softAP(ssid, password)) {
    Serial.println("AP setup successful");
    wifiInitialized = true;
    
    // Setup DNS server to redirect all domains to the ESP
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", IPAddress(192,168,4,1));
    
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP address: "); 
    Serial.println(ip);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP:");
    display.println(ssid);
    display.print("IP: ");
    display.println(ip);
    display.display();
    delay(2000);
  } else {
    Serial.println("AP setup failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP failed!");
    display.display();
    delay(2000);
  }
}

void checkWiFi() {
  if (!wifiInitialized || WiFi.softAPgetStationNum() == 0) {
    // If WiFi not initialized or no clients connected, check if it's still active
    if (WiFi.status() != WL_CONNECTED && !WiFi.softAPSSID().equals(ssid)) {
      Serial.println("WiFi AP disconnected. Reconnecting...");
      setupWiFi();
    }
  }
}

void loop() {
  unsigned long now = millis();
  
  // Handle DNS requests for captive portal
  dnsServer.processNextRequest();
  
  // Handle client requests
  server.handleClient();
  
  // Check sensor data at regular intervals
  if (now - lastSensorRead >= sensorInterval) {
    readSensorData();
    lastSensorRead = now;
  }
  
  // Calculate SpO2 at regular intervals
  if (now - lastSpO2Check >= spo2Interval) {
    calculateSpO2();
    lastSpO2Check = now;
  }
  
  // Check WiFi status periodically
  if (now - lastWifiCheck >= wifiCheckInterval) {
    checkWiFi();
    lastWifiCheck = now;
  }
  
  checkAlarmState();
  updateDisplay();
  
  // Allow background processes to run
  yield();
}

void readSensorData() {
  irValue = particleSensor.getIR();
  if (irValue < 25000) {
    beatDetected = false;
    return;
  }
  if (checkForBeat(irValue)) {
    unsigned long delta = millis() - lastBeat;
    lastBeat = millis();
    if (delta > 300 && delta < 2000) {
      pulse = 60000 / delta;
      beatDetected = true;
      Serial.print("BPM: "); Serial.println(pulse);
    }
  }
}

void calculateSpO2() {
  int32_t spo2Val; int8_t validSpO2;
  int32_t hr;      int8_t validHR;
  
  // Ensure the sensor is reporting values before calculating
  if (particleSensor.getIR() < 25000) {
    return;
  }
  
  for (int i = 0; i < 100; ++i) {
    byte retry = 0;
    while (!particleSensor.available() && retry++ < 50) {
      delay(1);
      particleSensor.check();
      yield();
    }
    
    if (particleSensor.available()) {
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    } else {
      // If we can't get enough samples, try again later
      return;
    }
  }
  
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, 100, redBuffer,
    &spo2Val, &validSpO2,
    &hr, &validHR
  );
  
  if (validSpO2 && spo2Val > 0 && spo2Val <= 100) {
    spo2 = spo2Val;
    Serial.print("SpO2: "); Serial.println(spo2);
  }
}

void checkAlarmState() {
  if (alarmTriggered) {
    if (millis() - lastBlink > 500) {
      blinkState = !blinkState;
      lastBlink = millis();
    }
    return;
  }
  
  if (alarmHour >= 0) {
    unsigned long t = millis() - timeBase;
    int h = (t / 3600000) % 24;
    int m = (t / 60000) % 60;
    if (h == alarmHour && m == alarmMinute) {
      alarmTriggered = true;
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  
  if (alarmTriggered) {
    display.setTextSize(2);
    if (blinkState) display.println("ALARM!");
    display.display();
    return;
  }
  
  unsigned long t = millis() - timeBase;
  int h = (t / 3600000) % 24;
  int m = (t / 60000) % 60;
  int s = (t / 1000) % 60;
  
  display.printf("Time: %02d:%02d:%02d\n", h, m, s);
  
  if (!wifiInitialized) {
    display.println("WiFi not connected");
  } else {
    display.printf("WiFi: %d clients\n", WiFi.softAPgetStationNum());
  }
  
  if (irValue < 25000) {
    display.println("Place finger");
  } else {
    display.printf("Pulse: %d bpm\n", beatDetected ? pulse : 0);
    display.printf("SpO2: %d%%\n", spo2);
  }
  
  if (alarmHour >= 0) {
    display.printf("Alarm: %02d:%02d", alarmHour, alarmMinute);
  }
  
  display.display();
}

void handleRoot() {
  String html = F(R"=====(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Health Monitor</title>
<style>
:root{color-scheme:light dark;
--bg:#f0f0f0;--fg:#333;--card:#fff;--bd:#ccc}
@media(prefers-color-scheme:dark){:root{
--bg:#121212;--fg:#eee;--card:#1e1e1e;--bd:#444}}
body{margin:0;padding:0;font-family:Arial,sans-serif;
background:var(--bg);color:var(--fg)}
.container{max-width:500px;margin:20px auto;
padding:20px;background:var(--card);
border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.2)}
h1{margin-bottom:15px}
.section{margin:20px 0;padding:15px;
border:1px solid var(--bd);border-radius:8px}
input[type=number]{width:60px;padding:8px;
margin:5px;border:1px solid var(--bd);
border-radius:4px;background:var(--bg);
color:var(--fg)}
button{padding:10px 20px;
border:none;border-radius:5px;
background:#4CAF50;color:#fff;
cursor:pointer}
button:hover{background:#45a049}
.alarm-btn{background:#f44336}
.alarm-btn:hover{background:#d32f2f}
.data-box{background:var(--bg);
padding:10px;border-radius:5px;
margin-top:10px}
#alarmStatus{font-size:1.2em;
color:#f44336;font-weight:bold;
animation:blink 1s step-start 0s infinite;
display:none;margin-top:10px}
@keyframes blink{50%{opacity:0}}
#connectionStatus {color: #4CAF50; font-weight: bold; margin-top: 10px;}
.error {color: #f44336;}
</style>
<script>
let failedRequests = 0;
function updateData(){
 fetch('/data')
   .then(r => {
     if (!r.ok) throw new Error('Network response was not ok');
     failedRequests = 0;
     document.getElementById('connectionStatus').textContent = 'Connected';
     document.getElementById('connectionStatus').className = '';
     return r.json();
   })
   .then(d => {
     document.getElementById('time').textContent = d.time;
     document.getElementById('pulse').textContent = d.pulse;
     document.getElementById('spo2').textContent = d.spo2;
     document.getElementById('alarm').textContent = d.alarm;
     document.getElementById('alarmStatus').style.display = d.alarmActive == '1' ? 'block' : 'none';
   })
   .catch(err => {
     console.error('Error fetching data:', err);
     failedRequests++;
     if (failedRequests > 3) {
       document.getElementById('connectionStatus').textContent = 'Connection Lost. Retrying...';
       document.getElementById('connectionStatus').className = 'error';
     }
   });
}
setInterval(updateData, 1000);
window.onload = updateData;
</script>
</head><body>
<div class="container">
<h1>🩺 Health Monitor</h1>
<div id="connectionStatus">Connected</div>
<div class="section">
<h3>⏰ Time: <span id="time">--:--:--</span></h3>
<form action="/setTime" method="get">
<input type="number" name="h" min="0" max="23" placeholder="HH" required>
<input type="number" name="m" min="0" max="59" placeholder="MM" required>
<button type="submit">Set Time</button>
</form>
</div>
<div class="section">
<h3>🔔 Alarm: <span id="alarm">Not set</span></h3>
<form action="/setAlarm" method="get">
<input type="number" name="h" min="0" max="23" placeholder="HH" required>
<input type="number" name="m" min="0" max="59" placeholder="MM" required>
<button type="submit">Set Alarm</button>
</form>
<form action="/clearAlarm" method="get" style="margin-top:10px;">
<button class="alarm-btn" type="submit">Clear Alarm</button>
</form>
<div id="alarmStatus">🚨 ALARM TRIGGERED!</div>
</div>
<div class="section data-box">
<h3>📊 Health Data</h3>
<p>❤️ Pulse: <span id="pulse">--</span> bpm</p>
<p>🫁 SpO₂: <span id="spo2">--</span>%</p>
</div>
</div>
</body></html>
)=====");
  server.send(200, "text/html", html);
}

void handleData() {
  unsigned long t = millis() - timeBase;
  int h = (t / 3600000) % 24;
  int m = (t / 60000) % 60;
  int s = (t / 1000) % 60;
  
  String json = "{";
  json += "\"time\":\"" + String(h) + ":" + String(m) + ":" + String(s) + "\",";
  json += "\"pulse\":\"" + String(beatDetected ? pulse : 0) + "\",";
  json += "\"spo2\":\"" + String(spo2) + "\",";
  json += "\"alarm\":\"" + (alarmHour >= 0 ? String(alarmHour) + ":" + String(alarmMinute) : "Not set") + "\",";
  json += "\"alarmActive\":\"" + String(alarmTriggered ? "1" : "0") + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleSetTime() {
  if (server.hasArg("h") && server.hasArg("m")) {
    int h = server.arg("h").toInt();
    int m = server.arg("m").toInt();
    if (h >= 0 && h < 24 && m >= 0 && m < 60) {
      timeBase = millis() - (h * 3600000UL + m * 60000UL);
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetAlarm() {
  if (server.hasArg("h") && server.hasArg("m")) {
    alarmHour = server.arg("h").toInt();
    alarmMinute = server.arg("m").toInt();
    alarmTriggered = false;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleClearAlarm() {
  alarmHour = -1;
  alarmMinute = -1;
  alarmTriggered = false;
  server.sendHeader("Location", "/");
  server.send(303);
}
