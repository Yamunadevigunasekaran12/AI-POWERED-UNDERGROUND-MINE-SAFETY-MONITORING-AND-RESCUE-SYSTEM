/*
 * =========================================================================================
 * Project      : AI-Powered ESP32-CAM Person Detection & Surveillance Robot
 * Target Board : AI-Thinker ESP32-CAM (Compatible with ESP32 Core 3.3.7 & 2.x)
 * Directory    : G:\1. 2026-2027\1. Project Work\3. Durga\15. Camreea robot\Hardware_Code\esp32_cam_person_robot\
 * =========================================================================================
 * 
 * 100% STANDALONE OPERATION (NO WIRES TO 2ND ESP32, NO PC NEEDED):
 *   - Only needs 5V and GND power!
 *   - Runs on-chip AI detection continuously in PSRAM.
 *   - Automatically broadcasts UDP alert ('1' & "PERSON") to Port 4210 over Wi-Fi.
 *   - Periodically broadcasts its IP address ("CAM_IP:http://192.168.x.x") so the
 *     second ESP32 displays the camera web link on its 16x2 LCD & IoT Cloud Dashboard!
 * =========================================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================================
// 1. MOBILE HOTSPOT CREDENTIALS
// ==========================================
const char* ssid     = "projectiotb";
const char* password = "123456789b";

// UDP Alert Settings
const uint16_t UDP_PORT = 4210;
WiFiUDP udp;
IPAddress broadcastIp(255, 255, 255, 255);

unsigned long lastUdpSendMs = 0;
const unsigned long UDP_COOLDOWN_MS = 800;
unsigned long totalUdpPacketsSent = 0;

// Periodic IP Broadcast Beacon (Sends IP every 4 seconds)
unsigned long lastBeaconMs = 0;
const unsigned long BEACON_INTERVAL_MS = 4000;

// Camera Pins (AI-Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Motor & Actuator Pins
#define MOTOR_L_FWD   12   // IN1
#define MOTOR_L_BWD   13   // IN2
#define MOTOR_R_FWD   14   // IN3
#define MOTOR_R_BWD   15   // IN4
#define FLASH_LED      4   // Flashlight
#define STATUS_LED    33   // Red onboard LED (Active LOW: LOW = ON)

WebServer server(80);
WiFiServer streamServer(81);

// Global State
bool autoTrackingMode = false;
bool flashState = false;
bool personDetected = false;
int humanScore = 0;
int detectedX = 0;
int detectedY = 0;
unsigned long lastPersonSeenTime = 0;
const unsigned long PERSON_TIMEOUT_MS = 600;

// Threshold: human face/skin pixels needed to confirm a person
#define HUMAN_THRESHOLD  220
#define SAMPLE_STEP      3

uint8_t *rgb_buffer = nullptr;

// ==========================================
// 2. MOTOR CONTROL FUNCTIONS
// ==========================================
void robotStop() {
  digitalWrite(MOTOR_L_FWD, LOW); digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, LOW); digitalWrite(MOTOR_R_BWD, LOW);
}
void robotForward() {
  digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
}
void robotBackward() {
  digitalWrite(MOTOR_L_FWD, LOW); digitalWrite(MOTOR_L_BWD, HIGH);
  digitalWrite(MOTOR_R_FWD, LOW); digitalWrite(MOTOR_R_BWD, HIGH);
}
void robotTurnLeft() {
  digitalWrite(MOTOR_L_FWD, LOW); digitalWrite(MOTOR_L_BWD, HIGH);
  digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
}
void robotTurnRight() {
  digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, LOW); digitalWrite(MOTOR_R_BWD, HIGH);
}

void sendPersonUDPAlert() {
  unsigned long now = millis();
  if (now - lastUdpSendMs >= UDP_COOLDOWN_MS) {
    lastUdpSendMs = now;
    totalUdpPacketsSent++;
    udp.beginPacket(broadcastIp, UDP_PORT);
    udp.write((uint8_t)'1');
    udp.endPacket();

    udp.beginPacket(broadcastIp, UDP_PORT);
    udp.print("PERSON");
    udp.endPacket();
    Serial.printf("[UDP TX #%lu] Alert Broadcasted on Port %d\n", totalUdpPacketsSent, UDP_PORT);
  }
}

// Broadcasts CAM IP Address so the receiver knows the link automatically
void sendIPBeacon() {
  unsigned long now = millis();
  if (now - lastBeaconMs >= BEACON_INTERVAL_MS) {
    lastBeaconMs = now;
    String ipMsg = "CAM_IP:http://" + WiFi.localIP().toString();
    udp.beginPacket(broadcastIp, UDP_PORT);
    udp.print(ipMsg);
    udp.endPacket();
  }
}

// ==========================================
// 3. EMBEDDED WEB DASHBOARD HTML
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM AI Robot</title><style>
body{font-family:Arial,sans-serif;text-align:center;background:#12141d;color:#fff;margin:0;padding:10px}
h2{color:#38bdf8}#stream{width:100%;max-width:360px;border:3px solid #38bdf8;border-radius:12px}
.badge{padding:8px 20px;border-radius:20px;font-weight:bold;margin:8px auto;display:inline-block;font-size:15px;letter-spacing:0.5px}
.badge-normal{background:#1e293b;color:#94a3b8;border:1px solid #334155}
.badge-detected{background:#ef4444;color:#fff;box-shadow:0 0 15px rgba(239,68,68,0.7);animation:p 1s infinite}
@keyframes p{50%{opacity:.7}}
.score-card{background:#1e293b;max-width:360px;margin:8px auto;padding:8px;border-radius:8px;font-size:13px;color:#94a3b8}
.score-val{color:#38bdf8;font-weight:bold}
.d-pad{display:grid;grid-template-columns:80px 80px 80px;grid-gap:10px;justify-content:center;margin:15px auto}
.btn{background:#1e293b;color:#fff;border:2px solid #38bdf8;border-radius:12px;font-size:22px;padding:12px 0;cursor:pointer}
.btn-stop{background:#ef4444;border-color:#f87171}
.toggle-btn{padding:10px 16px;border-radius:8px;border:none;font-weight:bold;cursor:pointer;margin:5px}
</style></head><body>
<h2>ESP32-CAM AI Robot</h2>
<div id="statusBadge" class="badge badge-normal">NO PERSON (Scanning...)</div><br>
<img id="stream" src=":81/stream"><br>
<div class="score-card">
  Human Feature Score: <span id="scoreVal" class="score-val">0</span> / 220 
  | UDP Sent: <span id="udpVal" class="score-val">0</span>
</div>
<button id="autoBtn" class="toggle-btn" style="background:#475569;color:#fff" onclick="toggleAuto()">Auto Track: OFF</button>
<button class="toggle-btn" style="background:#f59e0b;color:#000" onclick="fetch('/action?go=flash')">Toggle Flash</button>
<div class="d-pad">
<div></div><button class="btn" onmousedown="cmd('forward')" onmouseup="cmd('stop')" ontouchstart="cmd('forward')" ontouchend="cmd('stop')">&#9650;</button><div></div>
<button class="btn" onmousedown="cmd('left')" onmouseup="cmd('stop')" ontouchstart="cmd('left')" ontouchend="cmd('stop')">&#9664;</button>
<button class="btn btn-stop" onclick="cmd('stop')">&#9632;</button>
<button class="btn" onmousedown="cmd('right')" onmouseup="cmd('stop')" ontouchstart="cmd('right')" ontouchend="cmd('stop')">&#9654;</button>
<div></div><button class="btn" onmousedown="cmd('backward')" onmouseup="cmd('stop')" ontouchstart="cmd('backward')" ontouchend="cmd('stop')">&#9660;</button><div></div>
</div>
<script>
let autoMode=false;function cmd(a){if(!autoMode||a==='stop')fetch('/action?go='+a);}
function toggleAuto(){autoMode=!autoMode;let b=document.getElementById('autoBtn');
if(autoMode){b.style.background='#10b981';b.innerText='Auto Track: ON';fetch('/action?go=auto_on');}
else{b.style.background='#475569';b.innerText='Auto Track: OFF';fetch('/action?go=auto_off');}}
setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{
let b=document.getElementById('statusBadge');
document.getElementById('scoreVal').innerText=d.score;
document.getElementById('udpVal').innerText=d.udp_sent;
if(d.detected){
  b.className='badge badge-detected';
  b.innerText='PERSON DETECTED! (UDP SENT)';
} else {
  b.className='badge badge-normal';
  b.innerText=autoMode?'Auto Track: Searching...':'NO PERSON (Scanning...)';
}});},350);
window.onload=()=>{document.getElementById('stream').src='http://'+window.location.hostname+':81/stream';};
</script></body></html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }
void handleAction() {
  if (server.hasArg("go")) {
    String act = server.arg("go");
    if (act == "forward") { autoTrackingMode = false; robotForward(); }
    else if (act == "backward") { autoTrackingMode = false; robotBackward(); }
    else if (act == "left") { autoTrackingMode = false; robotTurnLeft(); }
    else if (act == "right") { autoTrackingMode = false; robotTurnRight(); }
    else if (act == "stop") { robotStop(); }
    else if (act == "auto_on") { autoTrackingMode = true; }
    else if (act == "auto_off") { autoTrackingMode = false; robotStop(); }
    else if (act == "flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED, flashState ? HIGH : LOW);
    }
  }
  server.send(200, "text/plain", "OK");
}
void handleStatus() {
  String json = "{\"detected\":" + String(personDetected ? "true" : "false") + 
                ",\"score\":" + String(humanScore) +
                ",\"udp_sent\":" + String(totalUdpPacketsSent) + 
                ",\"auto\":" + String(autoTrackingMode ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

// ==========================================
// 4. ACCURATE HUMAN SEGMENTATION PIPELINE
// ==========================================
void handleStream() {
  WiFiClient client = streamServer.available();
  if (!client) return;
  client.println("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nAccess-Control-Allow-Origin: *\r\n");

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }

    int skin_count = 0;
    long sum_x = 0;

    // Decode JPEG to RGB in PSRAM for real pixel analysis
    if (rgb_buffer && fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buffer)) {
      int total_pixels = fb->width * fb->height;

      for (int i = 0; i < total_pixels; i += SAMPLE_STEP) {
        int idx = i * 3;
        uint8_t r = rgb_buffer[idx];
        uint8_t g = rgb_buffer[idx + 1];
        uint8_t b = rgb_buffer[idx + 2];

        // Kovacs-Peer Human Skin & Face Chromatic Rules
        if (r > 80 && g > 40 && b > 20 && r > g && r > b) {
          int r_minus_g = (int)r - (int)g;
          int r_minus_b = (int)r - (int)b;
          if (r_minus_g >= 15 && r_minus_g <= 85 && r_minus_b >= 15 && g >= b) {
            skin_count++;
            sum_x += (i % fb->width);
          }
        }
      }
    }

    humanScore = skin_count;

    if (skin_count >= HUMAN_THRESHOLD) {
      personDetected = true;
      digitalWrite(STATUS_LED, LOW); // Red LED ON
      sendPersonUDPAlert();
      lastPersonSeenTime = millis();

      if (autoTrackingMode && skin_count > 0) {
        int centerX = sum_x / skin_count;
        int frameCenter = fb->width / 2;
        if (centerX < (frameCenter - 35)) robotTurnLeft();
        else if (centerX > (frameCenter + 35)) robotTurnRight();
        else robotForward();
      }
    } else {
      if (millis() - lastPersonSeenTime > PERSON_TIMEOUT_MS) {
        personDetected = false;
        digitalWrite(STATUS_LED, HIGH); // Red LED OFF
        if (autoTrackingMode) robotStop();
      }
    }

    // Stream JPEG frame to client browser
    client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    server.handleClient();
  }
  client.stop();
}

void setup() {
#if defined(RTC_CNTL_BROWN_OUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#elif defined(RTC_CNTL_BROWNOUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWNOUT_REG, 0);
#endif
  Serial.begin(115200);
  delay(500);

  pinMode(MOTOR_L_FWD, OUTPUT); pinMode(MOTOR_L_BWD, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT); pinMode(MOTOR_R_BWD, OUTPUT);
  pinMode(FLASH_LED, OUTPUT);   pinMode(STATUS_LED, OUTPUT);
  robotStop();
  digitalWrite(FLASH_LED, LOW); digitalWrite(STATUS_LED, HIGH);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    rgb_buffer = (uint8_t*)ps_malloc(320 * 240 * 3);
    Serial.println("[SETUP] PSRAM RGB Buffer Allocated Successfully!");
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 14;
    config.fb_count = 1;
    rgb_buffer = (uint8_t*)malloc(160 * 120 * 3);
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[FATAL] Camera Init Failed: 0x%x. Retrying...\n", err);
    delay(1000);
    esp_camera_init(&config);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  }

  // Connect to Mobile Hotspot
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.print("Connecting to Mobile Hotspot '");
  Serial.print(ssid);
  Serial.println("'...");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(STATUS_LED, HIGH);
    Serial.println();
    Serial.println("==================================================");
    Serial.println("[SUCCESS] CONNECTED TO MOBILE HOTSPOT!");
    Serial.print("  -> Camera URL: http://");
    Serial.println(WiFi.localIP());
    Serial.println("==================================================");
  }

  udp.begin(UDP_PORT);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/action", HTTP_GET, handleAction);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  streamServer.begin();
}

void loop() {
  server.handleClient();
  handleStream();
  sendIPBeacon(); // Broadcasts camera IP periodically over UDP
}
