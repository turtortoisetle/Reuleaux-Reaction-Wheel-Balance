#include <SimpleFOC.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiUdp.h>

// ============================================
// 自平衡萊洛三角形（單顆反作用輪）
//
// 硬體：ESP32、2804 雲台無刷馬達(7 極對) + SimpleFOC mini、AS5600 磁編碼器、
//       MPU6050、3S 鋰電池。AS5600 與 MPU6050 共用 I2C (SDA 21 / SCL 22)。
// 函式庫：SimpleFOC 2.4.0 以上（用到 estimated_current 扭力模式）
//
// 控制律（輸出單位是電流 A，角度單位是度）：
//   cmd = -( Kp·(pitch - target) + Kd·gyro + Kv·飛輪轉速 )
//   target += Ka·飛輪轉速·dt     平衡點自動修正，等同對飛輪轉速做積分
//
// 操作：序列埠 1000000 baud，或連上 WiFi 熱點後開 http://192.168.4.1，
//       也可以用 wifi_logger.py 透過 UDP 收資料、下指令。指令列表見 handleSerialCommand()。
// ============================================

const char* FW_VERSION = "v32";

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 13);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

const uint8_t MPU_addr = 0x68;
// DLPF_CFG：0x02 ≈ 94Hz(延遲約3ms)、0x03 ≈ 44Hz(延遲約5ms)。振動太大再改 0x03
const uint8_t MPU_DLPF_CFG = 0x02;
int16_t AcX, AcY, AcZ, GyZ;

float pitch = 0.0;
float raw_gyro_z = 0.0;
float angle_offset = 0.0;
float gyro_bias = 0.0;

unsigned long pid_timer = 0;
// 500Hz。設成 1kHz 時讀 IMU 佔掉太多時間，loopFOC 只剩約 1.3kHz，換相不夠快
const unsigned long CTRL_PERIOD_US = 2000;
const int LOG_EVERY = 2;                   // 每2個控制週期記錄一筆(250Hz)

enum RobotStateEnum { STATE_DISABLED, STATE_BALANCING };
// 標 volatile 的變數：網頁伺服器在 Core 0 讀寫，控制迴圈在 Core 1 讀寫
volatile RobotStateEnum robotState = STATE_DISABLED;

// 平衡點：平衡中會被飛輪轉速慢慢修正（見 Ka）
volatile float target_angle = 0.0;
const float TARGET_MAX = 6.0; // 自動修正最多偏離零點 ±6°

// ============================================
// 可調參數（單位：角度用「度」，輸出用「A」）
// 預設值是用 232g、重心距頂點約88mm 估算的起點，不是最佳值
// ============================================
const float DEF_KP = 0.30;     // A/°   (重力約需 0.08 A/°，取約 3.7 倍)
const float DEF_KD = 0.020;    // A/(°/s)
const float DEF_KV = -0.004;   // A/(rad/s) 飛輪轉速回授，必須是負值
const float DEF_KA = 0.02;     // °/s per rad/s 平衡點修正速度，正值（跟 Kv 同方向的積分）
const float DEF_ILIM = 1.0;    // A 電流上限
const float DEF_ALPHA = 0.99;  // 互補濾波每個控制週期的陀螺儀權重，500Hz 下時間常數約0.2秒
const float DEF_ENTRY = 30.0;
const float DEF_ENGAGE = 1.5;
const float DEF_KV_RATING = 127.0; // rpm/V，由規格 Ke=0.0434 換算，建議用 KVTEST 實測

volatile float Kp = DEF_KP;
volatile float Kd = DEF_KD;
volatile float Kv = DEF_KV;
volatile float Ka = DEF_KA;
volatile float Ilim = DEF_ILIM;
volatile float ALPHA = DEF_ALPHA;
volatile float ENTRY_ANGLE = DEF_ENTRY;   // 平衡中超過此角度就退出
volatile float ENGAGE_ANGLE = DEF_ENGAGE; // DISABLED 時要小於此角度才切入
volatile float KV_RATING = DEF_KV_RATING;

const float WHEEL_READY_SPEED = 3.0;  // rad/s，飛輪要先停到這以下才允許切入
const float BRAKE_K = 0.02;           // DISABLED 時的飛輪煞車增益 (A per rad/s)
const float BRAKE_ILIM = 0.4;         // 煞車最大電流

const float SAFETY_MAX_FLYWHEEL_SPEED = 200.0; // rad/s

// 馬達電氣參數
const float MOTOR_R = 2.55;        // 相電阻 (Ω)
const float SUPPLY_V = 12.0;       // 3S
const float MOTOR_VLIMIT = 10.0;   // 要夠高，飛輪轉快時扣掉反電動勢才還有電壓可以出力

unsigned long lastStatusReportMs = 0;
volatile bool safetyTripped = false;

volatile float tele_pitch = 0, tele_gyro = 0, tele_flywheel = 0, tele_volt = 0, tele_curr = 0;
volatile bool sleepRequested = false;
volatile bool recalibrateRequested = false;
volatile bool saveZeroRequested = false;
volatile bool kvTestRequested = false;

uint32_t focLoopCount = 0;
float focHz = 0;

Preferences prefs;

// ============================================
// 參數存在 NVS。key 尾端的數字只是用來作廢舊值
// （增益單位或預設值定義改過時換一個 key），不要隨便改，否則已存的參數會讀不到
// ============================================
void loadParamsFromFlash() {
  prefs.begin("balance", true);
  Kp = prefs.getFloat("kp30", Kp);
  Kd = prefs.getFloat("kd30", Kd);
  Kv = prefs.getFloat("kv30", Kv);
  Ka = prefs.getFloat("ka30", Ka);
  Ilim = prefs.getFloat("ilim30", Ilim);
  KV_RATING = prefs.getFloat("kvr32", KV_RATING);
  ALPHA = prefs.getFloat("alpha31", ALPHA);
  ENTRY_ANGLE = prefs.getFloat("entry_ang", ENTRY_ANGLE);
  ENGAGE_ANGLE = prefs.getFloat("engage_ang", ENGAGE_ANGLE);
  prefs.end();
}

void saveParamsToFlash() {
  prefs.begin("balance", false);
  prefs.putFloat("kp30", Kp);
  prefs.putFloat("kd30", Kd);
  prefs.putFloat("kv30", Kv);
  prefs.putFloat("ka30", Ka);
  prefs.putFloat("ilim30", Ilim);
  prefs.putFloat("kvr32", KV_RATING);
  prefs.putFloat("alpha31", ALPHA);
  prefs.putFloat("entry_ang", ENTRY_ANGLE);
  prefs.putFloat("engage_ang", ENGAGE_ANGLE);
  prefs.end();
}

void resetDefaults() {
  Kp = DEF_KP; Kd = DEF_KD; Kv = DEF_KV; Ka = DEF_KA; Ilim = DEF_ILIM;
  ALPHA = DEF_ALPHA; ENTRY_ANGLE = DEF_ENTRY; ENGAGE_ANGLE = DEF_ENGAGE;
  // KV_RATING 不重設：那是量出來的馬達參數，不是調參
}

// 零點與陀螺儀零飄。沒存過的話 setup() 會自動校準一次（手扶直立3秒）
bool loadZeroOffset() {
  prefs.begin("balance", true);
  bool exists = prefs.isKey("zero_off") && prefs.isKey("gyro_bias_v30");
  if (exists) {
    angle_offset = prefs.getFloat("zero_off", 0.0);
    gyro_bias = prefs.getFloat("gyro_bias_v30", 0.0);
  }
  prefs.end();
  return exists;
}

void saveZeroOffset() {
  prefs.begin("balance", false);
  prefs.putFloat("zero_off", angle_offset);
  prefs.putFloat("gyro_bias_v30", gyro_bias);
  prefs.end();
}

// ============================================
// WiFi 熱點 + 網頁伺服器（Core 0）
// ============================================
const char* AP_SSID = "ReuleauxTune";
const char* AP_PASS = "balance123";

WebServer server(80);
TaskHandle_t webTaskHandle = NULL;

// ============================================
// UDP 資料串流（Core 1，只在loop裡使用）
// ============================================
const uint16_t UDP_PORT = 4210;
const unsigned long UDP_CLIENT_TIMEOUT_MS = 3000;
const unsigned long UDP_FLUSH_MS = 30;
const size_t UDP_FLUSH_BYTES = 1000;
WiFiUDP udp;
IPAddress udpClientIP;
uint16_t udpClientPort = 0;
unsigned long udpLastRxMs = 0;
unsigned long udpLastFlushMs = 0;
String udpBuf;

bool udpClientActive() {
  return udpClientPort != 0 && millis() - udpLastRxMs < UDP_CLIENT_TIMEOUT_MS;
}

// 同一份輸出同時送序列埠與 UDP（有電腦在收的時候才累積到 udpBuf）
class TeePrint : public Print {
public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (udpClientActive()) udpBuf += (char)c;
    return 1;
  }
  size_t write(const uint8_t* buf, size_t size) override {
    Serial.write(buf, size);
    if (udpClientActive()) udpBuf.concat((const char*)buf, size);
    return size;
  }
};
TeePrint Log;

void flushUdp(bool force) {
  if (udpBuf.length() == 0) return;
  if (!udpClientActive()) { udpBuf = ""; return; }
  if (!force && udpBuf.length() < UDP_FLUSH_BYTES && millis() - udpLastFlushMs < UDP_FLUSH_MS) return;
  udp.beginPacket(udpClientIP, udpClientPort);
  udp.write((const uint8_t*)udpBuf.c_str(), udpBuf.length());
  udp.endPacket();
  udpBuf = "";
  udpLastFlushMs = millis();
}

const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Reuleaux 調參</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px}
h2{font-size:16px;margin:14px 0 4px;border-bottom:1px solid #333;padding-bottom:4px}
.row{display:flex;justify-content:space-between;align-items:center;margin:8px 0}
.row label{flex:1;font-size:14px}
.row input[type=range]{flex:2;margin:0 8px}
.row input[type=number]{width:70px;font-size:13px;background:#222;color:#eee;border:1px solid #444;border-radius:4px}
.tele{background:#222;border-radius:8px;padding:12px;margin:10px 0;font-size:14px;line-height:1.7}
button{padding:12px;font-size:14px;background:#2a6;color:#fff;border:none;border-radius:8px;margin:4px 2px;width:100%}
.state{font-weight:bold}
.danger{color:#f33;font-weight:bold}
canvas{background:#1a1a1a;border-radius:8px;width:100%;box-sizing:border-box;margin-top:8px}
</style></head><body>

<h2>即時狀態</h2>
<div class="tele" id="tele">讀取中...</div>
<canvas id="pitchChart" height="120"></canvas>

<h2>控制參數（輸出單位：A）</h2>
<div class="row"><label>Kp (A/°)</label><input type="range" id="kp" min="0" max="1.5" step="0.01"><input type="number" id="kp_n" step="0.01"></div>
<div class="row"><label>Kd (A/(°/s))</label><input type="range" id="kd" min="0" max="0.1" step="0.001"><input type="number" id="kd_n" step="0.001"></div>
<div class="row"><label>Kv 飛輪轉速(負值)</label><input type="range" id="kv" min="-0.03" max="0.01" step="0.0005"><input type="number" id="kv_n" step="0.0005"></div>
<div class="row"><label>Ka 平衡點修正</label><input type="range" id="ka" min="0" max="0.1" step="0.002"><input type="number" id="ka_n" step="0.002"></div>
<div class="row"><label>電流上限(A)</label><input type="range" id="ilim" min="0.2" max="2" step="0.05"><input type="number" id="ilim_n" step="0.05"></div>
<div class="row"><label>濾波α(陀螺儀權重)</label><input type="range" id="alpha" min="0.98" max="0.999" step="0.001"><input type="number" id="alpha_n" step="0.001"></div>
<div class="row"><label>切入角度(±度)</label><input type="range" id="engage_ang" min="0.5" max="5" step="0.1"><input type="number" id="engage_ang_n" step="0.1"></div>
<div class="row"><label>退出角度(±度)</label><input type="range" id="entry_ang" min="5" max="90" step="1"><input type="number" id="entry_ang_n" step="1"></div>
<button onclick="saveParams()" style="background:#37a">儲存目前參數</button>
<button onclick="resetDefaults()" style="background:#666">恢復預設</button>

<h2>零點 / 平衡點</h2>
<div class="tele" id="zero_status">讀取中...</div>
<button onclick="recalibrate()" style="background:#c63">手扶裝置直立後，點此校準</button>
<button onclick="saveZero()" style="background:#a83">把自動學到的平衡點存成零點</button>

<h2>遠程控制</h2>
<button id="sleep_btn" onclick="toggleSleep()" style="background:#a33">休眠</button>

<script>
const ids=['kp','kd','kv','ka','ilim','alpha','entry_ang','engage_ang'];
let updating=false;
function applyValues(d){
  ids.forEach(id=>{
    const s=document.getElementById(id), n=document.getElementById(id+'_n');
    if(!updating){ s.value=d[id]; n.value=d[id]; }
  });
}
function fetchParams(){ fetch('/params').then(r=>r.json()).then(applyValues); }
ids.forEach(id=>{
  const s=document.getElementById(id), n=document.getElementById(id+'_n');
  const send=v=>fetch('/set?'+id+'='+v);
  s.addEventListener('input',e=>{ updating=true; n.value=e.target.value; send(e.target.value); });
  s.addEventListener('change',()=>{updating=false;});
  n.addEventListener('change',e=>{ s.value=e.target.value; send(e.target.value); });
});
function saveParams(){ fetch('/save').then(()=>alert('已儲存！')); }
function resetDefaults(){
  if(!confirm('確定恢復預設值？(按儲存前不會覆蓋flash)')) return;
  fetch('/reset_defaults').then(()=>{ fetchParams(); alert('已恢復預設值，記得按儲存才會永久保留'); });
}
function recalibrate(){
  if(!confirm('請先手扶裝置保持直立，準備好後按確定，接下來3秒鐘請勿移動')) return;
  fetch('/recalibrate').then(()=>setTimeout(()=>alert('校準完成！'),3500));
}
function saveZero(){
  if(!confirm('把目前自動學到的平衡點偏移寫進零點？')) return;
  fetch('/save_zero').then(()=>alert('已寫入'));
}
let isSleeping=false;
function toggleSleep(){
  const action=isSleeping?'wake':'sleep';
  fetch('/sleep_control?action='+action).then(r=>r.json()).then(d=>{ isSleeping=d.sleeping; updateSleepBtn(); });
}
function updateSleepBtn(){
  const btn=document.getElementById('sleep_btn');
  btn.textContent=isSleeping?'啟動(喚醒)':'休眠';
  btn.style.background=isSleeping?'#2a6':'#a33';
}
const pitchHist=[];
const MAXPTS=150;
function drawChart(p){
  pitchHist.push(p);
  if(pitchHist.length>MAXPTS) pitchHist.shift();
  const canvas=document.getElementById('pitchChart');
  const W=canvas.width=canvas.clientWidth, H=canvas.height;
  const c=canvas.getContext('2d');
  c.clearRect(0,0,W,H);
  const range=[-30,30];
  c.strokeStyle='rgba(255,255,255,0.15)';
  const zeroY=H/2;
  c.beginPath(); c.moveTo(0,zeroY); c.lineTo(W,zeroY); c.stroke();
  c.beginPath();
  pitchHist.forEach((v,i)=>{
    const x=i/(MAXPTS-1)*W;
    const vv=Math.max(range[0],Math.min(range[1],v));
    const y=H-(vv-range[0])/(range[1]-range[0])*H;
    i===0?c.moveTo(x,y):c.lineTo(x,y);
  });
  c.strokeStyle='#5af'; c.lineWidth=2; c.stroke();
  c.fillStyle='#5af'; c.font='11px sans-serif';
  c.fillText('Pitch: '+p.toFixed(2)+'°  (±30°)', 4, 14);
}
function fetchTele(){
  fetch('/telemetry').then(r=>r.json()).then(d=>{
    document.getElementById('tele').innerHTML =
      'Pitch: '+d.pitch.toFixed(2)+'°<br>'+
      'GyroZ: '+d.gyro.toFixed(1)+'°/s<br>'+
      'FlyWheel: '+d.flywheel.toFixed(1)+' rad/s<br>'+
      'Iq(估計): '+d.curr.toFixed(2)+' A　Uq: '+d.volt.toFixed(2)+' V<br>'+
      '狀態: <span class="state">'+d.state+'</span>　FOC: '+d.foc_hz.toFixed(0)+' Hz'+
      (d.safety_tripped?'<br><span class="danger">硬體保護已觸發，請按喚醒重置</span>':'');
    document.getElementById('zero_status').innerHTML='零點偏移: '+d.zero.toFixed(2)+'°<br>自動學到的平衡點: '+d.target.toFixed(2)+'°<br>馬達 KV: '+d.kvr.toFixed(1)+' rpm/V';
    isSleeping=d.sleeping;
    updateSleepBtn();
    drawChart(d.pitch);
  });
}
fetchParams();
fetchTele();
setInterval(fetchTele, 100);
</script>
</body></html>
)HTML";

void handleRoot() { server.send(200, "text/html", PAGE_HTML); }

void handleGetParams() {
  String json = "{";
  json += "\"kp\":" + String(Kp, 3) + ",";
  json += "\"kd\":" + String(Kd, 4) + ",";
  json += "\"kv\":" + String(Kv, 4) + ",";
  json += "\"ka\":" + String(Ka, 4) + ",";
  json += "\"ilim\":" + String(Ilim, 2) + ",";
  json += "\"alpha\":" + String(ALPHA, 4) + ",";
  json += "\"entry_ang\":" + String(ENTRY_ANGLE, 1) + ",";
  json += "\"engage_ang\":" + String(ENGAGE_ANGLE, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("kv")) Kv = server.arg("kv").toFloat();
  if (server.hasArg("ka")) Ka = server.arg("ka").toFloat();
  if (server.hasArg("ilim")) Ilim = constrain(server.arg("ilim").toFloat(), 0.1f, 2.5f);
  if (server.hasArg("alpha")) ALPHA = constrain(server.arg("alpha").toFloat(), 0.9f, 0.9995f);
  if (server.hasArg("entry_ang")) ENTRY_ANGLE = server.arg("entry_ang").toFloat();
  if (server.hasArg("engage_ang")) ENGAGE_ANGLE = server.arg("engage_ang").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleSave() { saveParamsToFlash(); server.send(200, "text/plain", "saved"); }
void handleResetDefaults() { resetDefaults(); server.send(200, "text/plain", "ok"); }
void handleRecalibrate() { recalibrateRequested = true; server.send(200, "text/plain", "ok"); }
void handleSaveZero() { saveZeroRequested = true; server.send(200, "text/plain", "ok"); }

void handleTelemetry() {
  String state_str = (robotState == STATE_BALANCING) ? "BALANCING" : "DISABLED";
  String json = "{";
  json += "\"pitch\":" + String(tele_pitch, 2) + ",";
  json += "\"gyro\":" + String(tele_gyro, 1) + ",";
  json += "\"flywheel\":" + String(tele_flywheel, 1) + ",";
  json += "\"volt\":" + String(tele_volt, 2) + ",";
  json += "\"curr\":" + String(tele_curr, 3) + ",";
  json += "\"state\":\"" + state_str + "\",";
  json += "\"sleeping\":" + String(sleepRequested ? "true" : "false") + ",";
  json += "\"safety_tripped\":" + String(safetyTripped ? "true" : "false") + ",";
  json += "\"zero\":" + String(angle_offset, 2) + ",";
  json += "\"target\":" + String(target_angle, 2) + ",";
  json += "\"kvr\":" + String(KV_RATING, 1) + ",";
  json += "\"foc_hz\":" + String(focHz, 0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSleepControl() {
  String action = server.arg("action");
  if (action == "sleep") sleepRequested = true;
  else if (action == "wake") { sleepRequested = false; safetyTripped = false; }
  server.send(200, "application/json", String("{\"sleeping\":") + (sleepRequested ? "true" : "false") + "}");
}

void webServerTask(void* param) {
  server.on("/", handleRoot);
  server.on("/params", handleGetParams);
  server.on("/set", handleSet);
  server.on("/save", handleSave);
  server.on("/reset_defaults", handleResetDefaults);
  server.on("/recalibrate", handleRecalibrate);
  server.on("/save_zero", handleSaveZero);
  server.on("/telemetry", handleTelemetry);
  server.on("/sleep_control", handleSleepControl);
  server.begin();
  for (;;) {
    server.handleClient();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// ============================================
// MPU6050
// ============================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

void initMPU6050() {
  mpuWrite(0x6B, 0x00);          // 先喚醒
  delay(50);
  mpuWrite(0x1A, MPU_DLPF_CFG);  // CONFIG：內建低通濾波
  mpuWrite(0x1B, 0x08);          // GYRO_CONFIG ±500°/s
  mpuWrite(0x1C, 0x00);          // ACCEL_CONFIG ±2g
}

void readMPU6050() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_addr, (size_t)14, (bool)true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  // 暫存器順序：AcX AcY AcZ TEMP GyX GyY GyZ（各2 bytes，共14）
  Wire.read(); Wire.read();  // TEMP
  Wire.read(); Wire.read();  // GyX
  Wire.read(); Wire.read();  // GyY
  GyZ = Wire.read() << 8 | Wire.read();
}

// 正負號約定：加速度計角度 -atan2(AcY,AcX) 與陀螺儀 +GyZ 同方向。
// 改變 MPU6050 安裝方向後要重新確認，兩者反向的話互補濾波會發散
// 65.5 LSB/(°/s) 對應 ±500°/s 量程
float readGyroRateDeg() { return ((float)GyZ / 65.5f); }

// 注意：這兩個函式要在同一次 readMPU6050() 之後呼叫
float rawPitchFromAccel() { return -atan2((float)AcY, (float)AcX) * 180.0f / PI; }

void calibrateZeroAngle() {
  Log.println("# 正在鎖定垂直 0 度，請手扶直立 3 秒...");
  float sum_angle = 0, sum_gyro = 0;
  const int SAMPLES = 300;
  for (int i = 0; i < SAMPLES; i++) {
    readMPU6050();
    sum_angle += rawPitchFromAccel();
    sum_gyro += readGyroRateDeg();
    delay(10);
  }
  angle_offset = sum_angle / SAMPLES;
  gyro_bias = sum_gyro / SAMPLES;
  target_angle = 0;
  pitch = 0;
  Log.printf("# 零點校準完成！偏移量: %.2f  陀螺儀零飄: %.3f\n", angle_offset, gyro_bias);
}

// ============================================
// KV 量測：電壓模式下給兩個電壓，用轉速斜率算 KV（扣掉摩擦造成的偏移）
// 公式對應 SimpleFOC 的 bemf = ω / (KV·√3) / RPM_TO_RADS
// ============================================
float spinAndMeasure(float volts, unsigned long ramp_ms, unsigned long hold_ms, unsigned long avg_ms) {
  unsigned long t0 = millis();
  double sum = 0; long n = 0;
  while (millis() - t0 < ramp_ms + hold_ms) {
    unsigned long el = millis() - t0;
    float v = (el < ramp_ms) ? volts * el / (float)ramp_ms : volts;
    motor.loopFOC();
    motor.move(v);
    if (fabs(motor.shaft_velocity) > SAFETY_MAX_FLYWHEEL_SPEED) { motor.move(0); return NAN; }
    if (el > ramp_ms + hold_ms - avg_ms) { sum += fabs(motor.shaft_velocity); n++; }
  }
  return n ? (float)(sum / n) : NAN;
}

void runKvTest() {
  Log.println("# KVTEST 開始：請手扶穩機身(飛輪會轉到約 90 rad/s)，約 8 秒");
  flushUdp(true);
  robotState = STATE_DISABLED;
  motor.torque_controller = TorqueControlType::voltage;
  const float V1 = 2.0, V2 = 4.0;
  float w1 = spinAndMeasure(V1, 800, 1500, 600);
  float w2 = isnan(w1) ? NAN : spinAndMeasure(V2, 800, 1500, 600);
  // 收尾：慢慢停下
  unsigned long t0 = millis();
  while (millis() - t0 < 2500) {
    motor.loopFOC();
    motor.move(constrain(-0.03f * motor.shaft_velocity, -1.5f, 1.5f));
  }
  motor.move(0);
  motor.torque_controller = TorqueControlType::estimated_current;

  if (isnan(w1) || isnan(w2) || w2 - w1 < 5.0f) {
    Log.printf("# KVTEST 失敗 w1=%.1f w2=%.1f（飛輪有轉嗎？）KV 維持 %.1f\n", w1, w2, KV_RATING);
    return;
  }
  float slope = (w2 - w1) / (V2 - V1);                 // (rad/s)/V
  float kv = slope / (_SQRT3 * _RPM_TO_RADS);           // rpm/V（SimpleFOC 定義）
  Log.printf("# KVTEST w(2V)=%.1f w(4V)=%.1f rad/s → KV=%.1f rpm/V（Ke≈%.4f V/(rad/s)）\n",
             w1, w2, kv, 1.0f / slope);
  if (kv < 30 || kv > 1500) {
    Log.println("# KV 不合理，不套用");
    return;
  }
  KV_RATING = kv;
  motor.KV_rating = kv;
  Log.println("# 已套用新 KV。確認沒問題後送 SAVE 永久保存");
  flushUdp(true);
}

// AS5600 CONF(0x07-0x08)：SF[9:8] 慢速濾波 00=16x(2.2ms) 01=8x 10=4x 11=2x(0.29ms)、FTH[12:10] 快速濾波門檻
// 實測飛輪不論給多少電壓都卡在約88 rad/s，推測是預設16x的延遲讓電角度落後約90°、力矩歸零，
// 所以開機時改成2x。只寫暫存器不燒錄(BURN)，斷電會還原，因此每次開機都要設
void setAS5600FastFilter() {
  const uint8_t AS5600_ADDR = 0x36;
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x07); Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (size_t)2, (bool)true); // 參數明確轉型，否則 ESP32 core 的 requestFrom 重載會有歧義
  uint8_t hi = Wire.read(), lo = Wire.read();
  hi = (hi & ~0x1F) | 0x03; // SF=11(2x)、FTH=000
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x07); Wire.write(hi); Wire.write(lo); Wire.endTransmission();
  // 讀回確認
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x07); Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (size_t)2, (bool)true);
  uint8_t hi2 = Wire.read(), lo2 = Wire.read();
  Serial.printf("# AS5600 CONF=0x%02X%02X %s\n", hi2, lo2, ((hi2 & 0x1F) == 0x03 && lo2 == lo) ? "(濾波2x OK)" : "(寫入失敗，仍是預設濾波)");
}

// ============================================
void setup() {
  Serial.begin(1000000);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }
  delay(500);
  udpBuf.reserve(1400);

  Serial.printf("\n# Reuleaux balance %s\n", FW_VERSION);
  loadParamsFromFlash();

  Wire.begin(21, 22);
  Wire.setClock(400000);

  initMPU6050();
  delay(200);

  if (loadZeroOffset()) {
    Serial.printf("# 已載入上次儲存的零點: %.2f\n", angle_offset);
  } else {
    Serial.println("# （尚未校準過，需要手動校準一次）");
    calibrateZeroAngle();
    saveZeroOffset();
  }

  SimpleFOCDebug::enable(&Serial);

  sensor.init();          // 跟 MPU6050 共用 Wire(21,22)
  Wire.setClock(400000);  // 保險：確保 sensor.init() 之後仍是 400kHz
  setAS5600FastFilter();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_V;
  driver.voltage_limit = MOTOR_VLIMIT;
  driver.init();
  motor.linkDriver(&driver);

  // 驅動板沒有電流感測，用 estimated_current：由相電阻與 KV 估算電壓，等效控制電流並補償反電動勢
  motor.phase_resistance = MOTOR_R;
  motor.KV_rating = KV_RATING;
  motor.voltage_limit = MOTOR_VLIMIT;
  motor.current_limit = Ilim;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::estimated_current;

  driver.enable();
  motor.init();

  motor.voltage_sensor_align = 5.0;
  motor.initFOC(); // 會轉動飛輪對齊電角度，馬達電源要先接上

  Serial.print("# DIAG motor_status=");
  Serial.print(motor.motor_status);
  Serial.println(motor.motor_status == 4 ? " (成功)" : " (異常，非4代表校準失敗)");

  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    Serial.println("# FOC校準失敗，停在這裡。請往上看 MOT: 開頭的訊息找原因：");
    Serial.println("#   Failed to notice movement → 馬達沒轉：檢查馬達電源(不能只插USB)、EN腳(13)、三相線");
    Serial.println("#   下面的AS5600角度：用手轉飛輪時數值不變 → 磁鐵/AS5600/I2C接線問題");
    for (;;) {
      sensor.update();
      Serial.print("# AS5600 角度(rad)="); Serial.println(sensor.getAngle(), 4);
      delay(500);
    }
  }

  pid_timer = micros();

  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  Serial.printf("# WiFi熱點 SSID: %s  密碼: %s\n", AP_SSID, AP_PASS);
  Serial.print("# 手機連上後，瀏覽器打開: http://"); Serial.println(WiFi.softAPIP());
  udp.begin(UDP_PORT);
  Serial.printf("# UDP資料串流埠: %u\n", UDP_PORT);

  xTaskCreatePinnedToCore(webServerTask, "webServer", 8192, NULL, 1, &webTaskHandle, 0);

  Serial.println("==================================================");
  Serial.printf(" 啟動成功（%s）  KV=%.1f rpm/V  Ilim=%.2f A\n", FW_VERSION, KV_RATING, Ilim);
  Serial.println("t_ms,Pitch,GyroZ,FlyWheelVel,Volt,Current_est,Target,State,MotorStatus");
  Serial.println("==================================================");
}

// ============================================
// 指令："SET KP 0.3"、"SET KD 0.02"、"SET KV -0.004"、"SET KA 0.02"、"SET ILIM 1.0"、
// "SET ALPHA 0.995"、"SET ENTRY 30"、"SET ENGAGE 1.5"、"SET KVR 127"、
// "CALIBRATE"、"SAVEZERO"、"KVTEST"、"SAVE"、"GET"、"SLEEP"、"WAKE"、"DEFAULTS"
// ============================================
void handleSerialCommand(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (line == "CALIBRATE") { recalibrateRequested = true; Log.println("# OK CALIBRATE"); return; }
  if (line == "SAVEZERO")  { saveZeroRequested = true; return; }
  if (line == "KVTEST")    { kvTestRequested = true; return; }
  if (line == "SAVE")      { saveParamsToFlash(); Log.println("# OK SAVE"); return; }
  if (line == "DEFAULTS")  { resetDefaults(); Log.println("# OK DEFAULTS (送 SAVE 才會寫入 flash)"); return; }
  if (line == "SLEEP") {
    sleepRequested = true;
    Log.println("# OK SLEEP (馬達完全不出力)");
    return;
  }
  if (line == "WAKE") {
    sleepRequested = false;
    safetyTripped = false;
    Log.println("# OK WAKE");
    return;
  }
  if (line == "GET") {
    Log.printf("# PARAMS %s KP=%.4f KD=%.4f KV=%.5f KA=%.4f ILIM=%.2f ALPHA=%.4f ENTRY=%.1f ENGAGE=%.2f KVR=%.1f TARGET=%.2f ZERO=%.2f\n",
               FW_VERSION, Kp, Kd, Kv, Ka, Ilim, ALPHA, ENTRY_ANGLE, ENGAGE_ANGLE, KV_RATING, target_angle, angle_offset);
    return;
  }

  int sp1 = line.indexOf(' ');
  if (line.startsWith("SET") && sp1 > 0) {
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp2 > 0) {
      String name = line.substring(sp1 + 1, sp2);
      float value = line.substring(sp2 + 1).toFloat();
      name.trim();
      if (name == "KP") Kp = value;
      else if (name == "KD") Kd = value;
      else if (name == "KV") Kv = value;
      else if (name == "KA") Ka = value;
      else if (name == "ILIM") Ilim = constrain(value, 0.1f, 2.5f);
      else if (name == "ALPHA") ALPHA = constrain(value, 0.9f, 0.9995f);
      else if (name == "ENTRY") ENTRY_ANGLE = value;
      else if (name == "ENGAGE") ENGAGE_ANGLE = value;
      else if (name == "KVR") { if (value > 10) { KV_RATING = value; motor.KV_rating = value; } }
      else { Log.print("# ERR unknown param: "); Log.println(name); return; }
      Log.print("# OK "); Log.print(name); Log.print("="); Log.println(value, 5);
      return;
    }
  }
  Log.print("# ERR unrecognized: ");
  Log.println(line);
}

void checkSerialCommands() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleSerialCommand(buf); buf = ""; }
    else if (c != '\r') {
      buf += c;
      if (buf.length() > 64) buf = "";
    }
  }
}

void checkUdpCommands() {
  static unsigned long lastCheckMs = 0;
  if (millis() - lastCheckMs < 10) return;
  lastCheckMs = millis();
  int len = udp.parsePacket();
  if (len <= 0) return;
  udpClientIP = udp.remoteIP();
  udpClientPort = udp.remotePort();
  udpLastRxMs = millis();
  char pkt[129];
  int n = udp.read(pkt, 128);
  if (n <= 0) return;
  pkt[n] = 0;
  String cmd = String(pkt);
  cmd.trim();
  if (cmd != "HELLO") handleSerialCommand(cmd);
}

// ============================================
void loop() {
  motor.loopFOC();
  focLoopCount++;
  checkSerialCommands();
  checkUdpCommands();
  flushUdp(false);

  // 這些動作會阻塞，只在非平衡狀態下執行
  if (robotState != STATE_BALANCING) {
    if (recalibrateRequested) {
      recalibrateRequested = false;
      motor.move(0);
      calibrateZeroAngle();
      saveZeroOffset();
      pid_timer = micros();
    }
    if (kvTestRequested) {
      kvTestRequested = false;
      runKvTest();
      pid_timer = micros();
    }
  }
  if (saveZeroRequested) {
    saveZeroRequested = false;
    angle_offset += target_angle;  // 學到的平衡點併入零點
    pitch -= target_angle;
    target_angle = 0;
    saveZeroOffset();
    Log.printf("# OK SAVEZERO 新零點=%.2f\n", angle_offset);
  }

  if (micros() - pid_timer >= CTRL_PERIOD_US) {
    unsigned long now = micros();
    float dt = (now - pid_timer) * 1e-6f;
    pid_timer = now;
    if (dt > 0.02f) dt = 0.02f; // 阻塞過後避免一次積太多

    readMPU6050();
    float accel_pitch = rawPitchFromAccel() - angle_offset;
    while (accel_pitch > 180.0f)  accel_pitch -= 360.0f;
    while (accel_pitch < -180.0f) accel_pitch += 360.0f;

    raw_gyro_z = readGyroRateDeg() - gyro_bias;
    // 互補濾波：短時間相信陀螺儀積分，長時間用加速度計拉回，消除積分漂移
    pitch = ALPHA * (pitch + raw_gyro_z * dt) + (1.0f - ALPHA) * accel_pitch;

    float flywheel_vel = motor.shaft_velocity; // 上一次 move() 更新的值
    motor.current_limit = Ilim;

    // 飛輪超速保護：直接進入睡眠，要送 WAKE 才會恢復
    bool safetyOverride = false;
    if (fabs(flywheel_vel) > SAFETY_MAX_FLYWHEEL_SPEED) {
      safetyOverride = true;
      safetyTripped = true;
      sleepRequested = true;
      motor.move(0);
    }

    // 遲滯切入：夠直(ENGAGE)而且飛輪已停才開始平衡，倒超過 ENTRY 才放棄。
    // 兩個門檻分開，手扶起來的過程中飛輪才不會先被拉到高轉速
    float error = pitch - target_angle;
    RobotStateEnum prevState = robotState;
    if (sleepRequested) {
      robotState = STATE_DISABLED;
    } else if (robotState == STATE_BALANCING) {
      if (fabs(error) > ENTRY_ANGLE) robotState = STATE_DISABLED;
    } else if (fabs(error) < ENGAGE_ANGLE && fabs(flywheel_vel) < WHEEL_READY_SPEED) {
      robotState = STATE_BALANCING;
    }
    if (robotState != prevState) {
      Log.printf("%s t_ms=%lu pitch=%.2f target=%.2f fly=%.1f\n",
                 robotState == STATE_BALANCING ? "# ENGAGE" : "# DROP",
                 millis(), pitch, target_angle, flywheel_vel);
    }

    float cmd = 0;
    if (!safetyOverride) {
      if (robotState == STATE_BALANCING) {
        // 平衡點自動修正：飛輪一直往同一邊轉 → 平衡點設錯了 → 慢慢挪
        target_angle = constrain(target_angle + Ka * flywheel_vel * dt, -TARGET_MAX, TARGET_MAX);
        error = pitch - target_angle;
        // Kv 必須是負值：對飛輪轉速是正回授，機身會多傾一點讓重力幫忙把飛輪轉速帶回 0
        cmd = -((Kp * error) + (Kd * raw_gyro_z) + (Kv * flywheel_vel)); // 單位：A
      } else if (sleepRequested) {
        cmd = 0;
      } else {
        // 主動煞車：手扶期間把飛輪轉速拉回0
        cmd = constrain(-BRAKE_K * flywheel_vel, -BRAKE_ILIM, BRAKE_ILIM);
      }
      motor.move(cmd); // estimated_current 模式下 loopFOC 會再用 current_limit 夾住
    }

    tele_pitch = pitch;
    tele_gyro = raw_gyro_z;
    tele_flywheel = flywheel_vel;
    tele_volt = motor.voltage.q;
    tele_curr = motor.current.q;

    static int logCnt = 0;
    if (++logCnt >= LOG_EVERY) {
      logCnt = 0;
      // 整行一次 printf，比逐欄位 print 快，減少控制週期抖動
      // 欄位：t_ms,Pitch,GyroZ,FlyWheelVel,Volt,Current_est,Target,State,MotorStatus
      Log.printf("%lu,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%s,%d\n",
                 millis(), pitch, raw_gyro_z, flywheel_vel, motor.voltage.q, motor.current.q,
                 target_angle, robotState == STATE_BALANCING ? "BAL" : "DIS", (int)motor.motor_status);
    }
  }

  if (millis() - lastStatusReportMs >= 1000) {
    unsigned long el = millis() - lastStatusReportMs;
    lastStatusReportMs = millis();
    focHz = focLoopCount * 1000.0f / el;
    focLoopCount = 0;
    Log.printf("# STATUS %s motor_status=%d sleep=%s safety=%s foc_hz=%.0f target=%.2f\n",
               FW_VERSION, (int)motor.motor_status, sleepRequested ? "true" : "false",
               safetyTripped ? "true" : "false", focHz, target_angle);
  }
}
