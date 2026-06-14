// OpenDrones Nav Module v9 - MSP Protocol + Full Sensor Suite
// XIAO ESP32-S3 + BMP280 + QMC5883P Compass + MicoAir M10 GPS
// + Betaflight MSP communication on Serial2

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_BMP280.h>
#include "MSP.h"

Adafruit_BMP280 bmp;
WebServer server(80);

#define I2C_SDA 5
#define I2C_SCL 6
#define GPS_RX  44
#define GPS_TX  43
#define LED_PIN 21
#define COMPASS_ADDR 0x2C

// FC UART (Serial2) - connects to Betaflight flight controller
// Nav module D8=GPIO7 (TX to FC RX), D9=GPIO8 (RX from FC TX)
// NOTE: Try swapping if no MSP response — wiring might be TX→TX/RX→RX
#define FC_RX_PIN 8   // D9/GPIO8 - receives FROM FC TX (swapped)
#define FC_TX_PIN 7   // D8/GPIO7 - sends TO FC RX (swapped)
#define FC_BAUD 115200

// QMC5883P registers
#define QMC5883P_CHIP_ID    0x00
#define QMC5883P_DATA_X     0x01
#define QMC5883P_STATUS     0x09
#define QMC5883P_CTRL1      0x0A
#define QMC5883P_CTRL2      0x0B

// UBX parser state
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_NAV_PVT_CLASS 0x01
#define UBX_NAV_PVT_MSG   0x07
#define UBX_PVT_LEN 92

struct UBX_PVT {
  uint32_t iTOW;
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fixType;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon, lat;
  int32_t height, hMSL;
  uint32_t hAcc, vAcc;
  int32_t velN, velE, velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
};

bool bmpOk = false;
bool compassOk = false;
float temperature = 0, pressure = 0, altitude_bmp = 0;
float compassX = 0, compassY = 0, compassZ = 0;
float heading = 0;

// Compass calibration
bool calibrating = false;
float calMinX = 999, calMaxX = -999;
float calMinY = 999, calMaxY = -999;
float offsetX = 0, offsetY = 0;
float scaleX = 1.0, scaleY = 1.0;
bool calibrated = false;

// =========================================
// MSP / Flight Controller
// =========================================
MSPLib msp;
bool fcConnected = false;
char fcVariant[5] = "----";
uint8_t fcApiMajor = 0, fcApiMinor = 0;
uint16_t rcChannels[RC_CHANNELS];  // Current RC channels from FC
int16_t fcRoll = 0, fcPitch = 0, fcYaw = 0;  // Attitude in decidegrees
bool fcArmed = false;
uint32_t fcFlags = 0;
unsigned long lastFcPoll = 0;
unsigned long lastMspLog = 0;
int fcPollState = 0;  // Round-robin poll state

// GPS data from UBX-PVT
bool gpsFix = false;
double gpsLat = 0, gpsLon = 0;
float gpsAlt = 0, gpsSpeed = 0, gpsHeading = 0;
int gpsSats = 0, gpsFixType = 0;
int gpsYear = 0, gpsMonth = 0, gpsDay = 0;
int gpsHour = 0, gpsMin = 0, gpsSec = 0;
unsigned long lastSensorRead = 0, lastBlink = 0;
const char* ap_ssid = "OpenDrones-Nav";
const char* ap_pass = "opendrones";

// UBX parsing
uint8_t ubxBuf[128];
int ubxIdx = 0;
bool ubxSync = false;
int ubxPayloadLen = 0;
int ubxTotalLen = 0;

// Forward declarations
void readCompass();
void initCompass();
int countI2CDevices();
void parseUBX();
void processPVT(uint8_t* payload, int len);

// --- HTML Dashboard ---
const char dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenDrones Nav</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{min-height:100vh;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
background:linear-gradient(135deg,#0a1628,#1a3a5c 40%,#2d6a9f 70%,#4a9bd9);
display:flex;flex-direction:column;align-items:center;padding:20px;color:#fff}
.hdr{text-align:center;margin-bottom:24px}
.hdr h1{font-size:26px;font-weight:700;text-shadow:0 2px 10px rgba(0,0,0,.3)}
.hdr p{font-size:12px;opacity:.6;margin-top:4px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:18px;width:100%;max-width:880px}
.card{background:rgba(255,255,255,.1);backdrop-filter:blur(20px);-webkit-backdrop-filter:blur(20px);
border:1px solid rgba(255,255,255,.2);border-radius:18px;padding:22px;
box-shadow:0 8px 32px rgba(0,0,0,.2);transition:transform .2s}
.card:hover{transform:translateY(-2px)}
.ct{font-size:11px;text-transform:uppercase;letter-spacing:2px;opacity:.55;margin-bottom:14px;display:flex;align-items:center;gap:6px}
.v{font-size:40px;font-weight:700;line-height:1.1}
.vs{font-size:26px;font-weight:700}
.u{font-size:13px;opacity:.45}
.sr{display:flex;gap:14px;margin-top:10px;flex-wrap:wrap}
.si{flex:1;min-width:70px}
.sl{font-size:10px;opacity:.45;text-transform:uppercase;letter-spacing:1px}
.sv{font-size:18px;font-weight:600;margin-top:2px}
.sb{display:flex;gap:8px;align-items:center;margin-top:10px;font-size:11px;padding:7px 10px;
border-radius:8px;background:rgba(255,255,255,.05)}
.d{width:7px;height:7px;border-radius:50%;flex-shrink:0}
.dg{background:#4ade80;box-shadow:0 0 6px #4ade80}
.dy{background:#facc15;box-shadow:0 0 6px #facc15}
.dr{background:#f87171;box-shadow:0 0 6px #f87171}
.cv{width:130px;height:130px;border-radius:50%;border:2px solid rgba(255,255,255,.2);
margin:10px auto;position:relative;background:rgba(255,255,255,.05)}
.cn{position:absolute;top:50%;left:50%;width:3px;height:55px;
background:linear-gradient(to top,#f87171 50%,#60a5fa 50%);border-radius:2px;
transform-origin:bottom center;transform:translate(-50%,-100%) rotate(0deg);transition:transform .3s}
.cv span{position:absolute;font-size:10px;font-weight:700}
.cv .n{top:6px;left:50%;transform:translateX(-50%);color:#60a5fa}
.cv .s{bottom:6px;left:50%;transform:translateX(-50%);opacity:.35}
.cv .e{right:8px;top:50%;transform:translateY(-50%);opacity:.35}
.cv .w{left:8px;top:50%;transform:translateY(-50%);opacity:.35}
.ft{text-align:center;margin-top:20px;font-size:11px;opacity:.3}
</style></head><body>
<div class="hdr"><h1>🛸 OpenDrones Nav Module</h1><p>XIAO ESP32-S3 Sensor Dashboard</p></div>
<div class="grid">
<div class="card">
<div class="ct">📍 GPS Location</div>
<div class="vs" id="gc">Searching...</div>
<div class="sr">
<div class="si"><div class="sl">Satellites</div><div class="sv" id="gs">0</div></div>
<div class="si"><div class="sl">Altitude</div><div class="sv" id="ga">--</div></div>
<div class="si"><div class="sl">Speed</div><div class="sv" id="gv">--</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">Fix Type</div><div class="sv" id="gf">--</div></div>
<div class="si"><div class="sl">Time</div><div class="sv" id="gt">--</div></div>
</div>
<div class="sb"><div class="d" id="gd"></div><span id="gst">Waiting...</span></div>
</div>
<div class="card">
<div class="ct">🧭 Compass <span id="calSt" style="margin-left:auto;font-size:10px;opacity:.4"></span></div>
<div class="cv">
<div class="cn" id="ndl"></div>
<span class="n">N</span><span class="s">S</span><span class="e">E</span><span class="w">W</span>
</div>
<div style="text-align:center"><div class="vs" id="hd">0&deg;</div><div class="u" id="dr">N</div></div>
<div class="sr">
<div class="si"><div class="sl">X</div><div class="sv" id="cx">0</div></div>
<div class="si"><div class="sl">Y</div><div class="sv" id="cy">0</div></div>
<div class="si"><div class="sl">Z</div><div class="sv" id="cz">0</div></div>
</div>
<div style="text-align:center;margin-top:10px">
<button id="calBtn" onclick="toggleCal()" style="background:rgba(255,255,255,.15);border:1px solid rgba(255,255,255,.3);
color:#fff;padding:8px 16px;border-radius:10px;cursor:pointer;font-size:12px">🎯 Calibrate Compass</button>
</div>
</div>
<div class="card">
<div class="ct">🌡️ Barometer</div>
<div class="v" id="tp">--</div>
<div class="sr" style="margin-top:4px">
<button onclick="toggleTemp()" id="tBtn" style="background:rgba(255,255,255,.1);border:1px solid rgba(255,255,255,.2);
color:#fff;padding:4px 10px;border-radius:6px;cursor:pointer;font-size:11px">Switch to °C</button>
<button onclick="toggleAlt()" id="aBtn" style="background:rgba(255,255,255,.1);border:1px solid rgba(255,255,255,.2);
color:#fff;padding:4px 10px;border-radius:6px;cursor:pointer;font-size:11px">Switch to m</button>
</div>
<div class="sr">
<div class="si"><div class="sl">Pressure</div><div class="sv" id="pr">--</div></div>
<div class="si"><div class="sl">Altitude</div><div class="sv" id="al">--</div></div>
</div>
<div class="sb"><div class="d" id="bd"></div><span id="bs">BMP280</span></div>
</div>
<div class="card">
<div class="ct">📡 System</div>
<div class="sr">
<div class="si"><div class="sl">UBX Pkts</div><div class="sv" id="su">0</div></div>
<div class="si"><div class="sl">Fix</div><div class="sv" id="sf">0</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">Uptime</div><div class="sv" id="supt">0s</div></div>
<div class="si"><div class="sl">Heap</div><div class="sv" id="sh">--</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">I2C Devs</div><div class="sv" id="si">0</div></div>
<div class="si"><div class="sl">Clients</div><div class="sv" id="sw">0</div></div>
</div>
</div>
<div class="card">
<div class="ct">✈️ Flight Controller</div>
<div class="sr">
<div class="si"><div class="sl">Status</div><div class="sv" id="fcStat">--</div></div>
<div class="si"><div class="sl">FC Type</div><div class="sv" id="fcType">--</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">Roll</div><div class="sv" id="fcR">0</div></div>
<div class="si"><div class="sl">Pitch</div><div class="sv" id="fcP">0</div></div>
<div class="si"><div class="sl">Yaw</div><div class="sv" id="fcY">0</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">MSP Tx</div><div class="sv" id="mspTx">0</div></div>
<div class="si"><div class="sl">MSP Rx</div><div class="sv" id="mspRx">0</div></div>
</div>
<div class="sb"><div class="d" id="fcD"></div><span id="fcSt">Not connected</span></div>
</div>
<div class="card">
<div class="ct">🎮 RC Inputs</div>
<div class="sr">
<div class="si"><div class="sl">Throttle</div><div class="sv" id="rcT">--</div></div>
<div class="si"><div class="sl">Roll</div><div class="sv" id="rcR">--</div></div>
<div class="si"><div class="sl">Pitch</div><div class="sv" id="rcP">--</div></div>
<div class="si"><div class="sl">Yaw</div><div class="sv" id="rcYw">--</div></div>
</div>
<div class="sr">
<div class="si"><div class="sl">Arm (CH5)</div><div class="sv" id="rcA">--</div></div>
<div class="si"><div class="sl">CH6</div><div class="sv" id="rc6">--</div></div>
<div class="si"><div class="sl">Nav (CH7)</div><div class="sv" id="rc7">--</div></div>
<div class="si"><div class="sl">CH8</div><div class="sv" id="rc8">--</div></div>
</div>
<div style="margin-top:10px">
<div class="sl" style="margin-bottom:6px">Throttle</div>
<div style="background:rgba(255,255,255,.08);border-radius:6px;height:14px;overflow:hidden">
<div id="thrBar" style="height:100%;background:linear-gradient(90deg,#4ade80,#facc15,#f87171);border-radius:6px;width:0%;transition:width .2s"></div>
</div>
</div>
<div style="margin-top:6px">
<div class="sl" style="margin-bottom:6px">Roll / Pitch</div>
<div style="display:flex;gap:6px">
<div style="flex:1;background:rgba(255,255,255,.08);border-radius:6px;height:10px;overflow:hidden;position:relative">
<div id="rollBar" style="height:100%;background:#60a5fa;width:50%;transition:width .2s;position:absolute"></div>
<div style="position:absolute;left:50%;top:0;bottom:0;width:1px;background:rgba(255,255,255,.3)"></div>
</div>
<div style="flex:1;background:rgba(255,255,255,.08);border-radius:6px;height:10px;overflow:hidden;position:relative">
<div id="pitchBar" style="height:100%;background:#c084fc;width:50%;transition:width .2s;position:absolute"></div>
<div style="position:absolute;left:50%;top:0;bottom:0;width:1px;background:rgba(255,255,255,.3)"></div>
</div>
</div>
<div style="display:flex;gap:6px;margin-top:2px;font-size:9px;opacity:.4">
<div style="flex:1;text-align:center">Roll</div>
<div style="flex:1;text-align:center">Pitch</div>
</div>
</div>
</div>
</div>
<div class="ft">OpenDrones &bull; Molt Studios</div>
<script>
function dir(h){const d=['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];return d[Math.round(h/22.5)%16]}
function upd(){
fetch('/api').then(r=>r.json()).then(d=>{
var ft=['None','DR','2D','3D','GNSS+DR','Time'];
if(d.ft>=2){document.getElementById('gc').textContent=d.lt.toFixed(6)+', '+d.ln.toFixed(6);
document.getElementById('ga').textContent=d.ga.toFixed(0)+'m';
document.getElementById('gd').className='d dg';document.getElementById('gst').textContent=ft[d.ft]+' '+d.st+' sats';}
else{document.getElementById('gc').textContent=d.st>0?'Sats: '+d.st:'Searching...';
document.getElementById('ga').textContent='--';
document.getElementById('gd').className=d.st>0?'d dy':'d dr';
document.getElementById('gst').textContent=d.st>0?ft[d.ft]+' ('+d.st+' sats)':'No signal';}
document.getElementById('gs').textContent=d.st;
document.getElementById('gv').textContent=d.sp>0?d.sp.toFixed(1)+'m/s':'--';
document.getElementById('gf').textContent=ft[d.ft]||'--';
document.getElementById('gt').textContent=d.hr?d.hr+':'+d.mn+':'+d.sc:'--';
document.getElementById('hd').innerHTML=d.hg.toFixed(0)+'&deg;';
document.getElementById('dr').textContent=dir(d.hg);
document.getElementById('ndl').style.transform='translate(-50%,-100%) rotate('+d.hg+'deg)';
document.getElementById('cx').textContent=d.cx.toFixed(3);
document.getElementById('cy').textContent=d.cy.toFixed(3);
document.getElementById('cz').textContent=d.cz.toFixed(3);
document.getElementById('tp').textContent=(useF?d.te:d.tc).toFixed(1)+(useF?'°F':'°C');
document.getElementById('pr').textContent=d.pe.toFixed(1);
document.getElementById('al').textContent=(useFt?d.af:d.at).toFixed(0)+(useFt?'ft':'m');
document.getElementById('bd').className=d.bm?'d dg':'d dr';
document.getElementById('bs').textContent=d.bm?'BMP280 Online':'BMP280 Offline';
document.getElementById('su').textContent=d.up;document.getElementById('sf').textContent=d.ft;
var s=d.upt,m=Math.floor(s/60),sec=s%60;
document.getElementById('supt').textContent=m>0?m+'m '+sec+'s':sec+'s';
document.getElementById('sh').textContent=(d.hk/1024).toFixed(0)+'KB';
document.getElementById('si').textContent=d.ic;document.getElementById('sw').textContent=d.wc;
document.getElementById('fcStat').textContent=d.armed?'ARMED':'Disarmed';
document.getElementById('fcStat').style.color=d.armed?'#4ade80':'#f87171';
document.getElementById('fcType').textContent=d.fc+' v'+d.fa;
document.getElementById('fcR').textContent=(d.fr/10).toFixed(1)+'°';
document.getElementById('fcP').textContent=(d.fp/10).toFixed(1)+'°';
document.getElementById('fcY').textContent=(d.fy/10).toFixed(0)+'°';
document.getElementById('mspTx').textContent=d.mspTx;document.getElementById('mspRx').textContent=d.mspRx;
document.getElementById('fcD').className=d.fc?'d dg':'d dr';
document.getElementById('fcSt').textContent=d.fc?'Connected ('+d.mspRx+' pkts)':'Not connected';
document.getElementById('calSt').textContent=d.cal==='running'?'CALIBRATING...':d.cal==='done'?'Calibrated':'';
document.getElementById('fcStat').textContent=d.armed?'ARMED':'DISARMED';
document.getElementById('fcType').textContent=d.fv||'--';
document.getElementById('fcR').textContent=(d.fr/10).toFixed(1)+'\u00B0';
document.getElementById('fcP').textContent=(d.fp/10).toFixed(1)+'\u00B0';
document.getElementById('fcY').textContent=(d.fy/10).toFixed(0)+'\u00B0';
document.getElementById('mspTx').textContent=d.mspTx;document.getElementById('mspRx').textContent=d.mspRx;
document.getElementById('fcD').className=d.fc?'d dg':'d dr';
document.getElementById('fcSt').textContent=d.fc?'Connected ('+d.fa+')':'Not connected';
// RC Inputs
if(d.fc){
document.getElementById('rcT').textContent=d.rcT;document.getElementById('rcR').textContent=d.rcR;
document.getElementById('rcP').textContent=d.rcP;document.getElementById('rcYw').textContent=d.rcYw;
document.getElementById('rcA').textContent=d.rcA>=1500?'ARMED':'OFF';
document.getElementById('rcA').style.color=d.rcA>=1500?'#4ade80':'#f87171';
document.getElementById('rc6').textContent=d.rc6;document.getElementById('rc7').textContent=d.rc7;
document.getElementById('rc8').textContent=d.rc8;
// Throttle bar (1000-2000 -> 0-100%)
var tp=((d.rcT-1000)/10);tp=Math.max(0,Math.min(100,tp));
document.getElementById('thrBar').style.width=tp+'%';
// Roll bar (1000=full left, 1500=center, 2000=full right)
var rp=((d.rcR-1000)/10);rp=Math.max(0,Math.min(100,rp));
document.getElementById('rollBar').style.width=rp+'%';
// Pitch bar
var pp=((d.rcP-1000)/10);pp=Math.max(0,Math.min(100,pp));
document.getElementById('pitchBar').style.width=pp+'%';
}else{
['rcT','rcR','rcP','rcYw','rcA','rc6','rc7','rc8'].forEach(function(id){document.getElementById(id).textContent='--';});
document.getElementById('thrBar').style.width='0%';
document.getElementById('rollBar').style.width='50%';document.getElementById('pitchBar').style.width='50%';
}
}).catch(e=>console.log(e));setTimeout(upd,1000);}upd();
var calRunning=false;
function toggleCal(){
if(!calRunning){
fetch('/calibrate/start').then(function(r){return r.json()}).then(function(d){calRunning=true;
document.getElementById('calBtn').textContent='Stop Calibration';
document.getElementById('calBtn').style.background='rgba(248,113,113,.3)';});
}else{
fetch('/calibrate/finish').then(function(r){return r.json()}).then(function(d){calRunning=false;
document.getElementById('calBtn').textContent='Calibrate Compass';
document.getElementById('calBtn').style.background='rgba(255,255,255,.15)';
alert('Calibration done! OffX:'+d.offX.toFixed(4)+' OffY:'+d.offY.toFixed(4));});}}
var useF=true,useFt=true;
function toggleTemp(){useF=!useF;document.getElementById('tBtn').textContent=useF?'Switch to C':'Switch to F';}
function toggleAlt(){useFt=!useFt;document.getElementById('aBtn').textContent=useFt?'Switch to m':'Switch to ft';}
</script></body></html>
)rawliteral";

// =========================================
// UBX Parser
// =========================================
int ubxPacketCount = 0;

void parseUBX() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (!ubxSync) {
      if (b == UBX_SYNC1) {
        ubxSync = true;
        ubxIdx = 0;
        ubxBuf[ubxIdx++] = b;
      }
      continue;
    }

    if (ubxIdx == 1 && b != UBX_SYNC2) {
      // Not a valid sync
      ubxSync = false;
      continue;
    }

    if (ubxIdx < 128) ubxBuf[ubxIdx++] = b;

    // At idx 5-6 we have payload length
    if (ubxIdx == 6) {
      ubxPayloadLen = ubxBuf[4] | (ubxBuf[5] << 8);
      ubxTotalLen = 6 + ubxPayloadLen + 2; // header + payload + checksum
    }

    // Full packet received
    if (ubxIdx >= 8 && ubxTotalLen > 0 && ubxIdx >= ubxTotalLen) {
      // Verify checksum
      uint8_t ck_a = 0, ck_b = 0;
      for (int i = 2; i < ubxTotalLen - 2; i++) {
        ck_a += ubxBuf[i];
        ck_b += ck_a;
      }
      if (ck_a == ubxBuf[ubxTotalLen-2] && ck_b == ubxBuf[ubxTotalLen-1]) {
        // Valid packet!
        uint8_t cls = ubxBuf[2];
        uint8_t msg = ubxBuf[3];
        if (cls == UBX_NAV_PVT_CLASS && msg == UBX_NAV_PVT_MSG) {
          processPVT(&ubxBuf[6], ubxPayloadLen);
          ubxPacketCount++;
        }
      }
      ubxSync = false;
      ubxIdx = 0;
      ubxTotalLen = 0;
    }

    // Safety: prevent buffer overflow
    if (ubxIdx >= 128) {
      ubxSync = false;
      ubxIdx = 0;
    }
  }
}

void processPVT(uint8_t* p, int len) {
  if (len < 92) return;

  // Parse UBX-NAV-PVT (u-blox interface spec)
  gpsYear   = p[4] | (p[5] << 8);
  gpsMonth  = p[6];
  gpsDay    = p[7];
  gpsHour   = p[8];
  gpsMin    = p[9];
  gpsSec    = p[10];
  gpsFixType = p[20];        // 0=none, 2=2D, 3=3D
  gpsSats   = p[23];         // numSV

  // lon/lat in degrees * 1e-7
  int32_t rawLon = (int32_t)(p[24] | (p[25]<<8) | (p[26]<<16) | (p[27]<<24));
  int32_t rawLat = (int32_t)(p[28] | (p[29]<<8) | (p[30]<<16) | (p[31]<<24));
  gpsLon = rawLon / 1e7;
  gpsLat = rawLat / 1e7;

  // Height above MSL in mm
  int32_t rawHMSL = (int32_t)(p[36] | (p[37]<<8) | (p[38]<<16) | (p[39]<<24));
  gpsAlt = rawHMSL / 1000.0;

  // Ground speed in mm/s
  int32_t rawSpeed = (int32_t)(p[60] | (p[61]<<8) | (p[62]<<16) | (p[63]<<24));
  gpsSpeed = rawSpeed / 1000.0;

  // Heading of motion in 1e-5 degrees
  int32_t rawHead = (int32_t)(p[64] | (p[65]<<8) | (p[66]<<16) | (p[67]<<24));
  gpsHeading = rawHead / 1e5;

  gpsFix = (gpsFixType >= 2);
}

// =========================================
// Compass Functions (QMC5883P)
// =========================================
void readCompass() {
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(QMC5883P_STATUS);
  if (Wire.endTransmission() != 0) return;
  Wire.requestFrom(COMPASS_ADDR, (byte)1);
  if (!Wire.available()) return;
  byte status = Wire.read();
  if (!(status & 0x01)) return;

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(QMC5883P_DATA_X);
  if (Wire.endTransmission() != 0) return;
  Wire.requestFrom(COMPASS_ADDR, (byte)6);
  if (Wire.available() >= 6) {
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);
    compassX = x / 15000.0;
    compassY = y / 15000.0;
    compassZ = z / 15000.0;

    // Calibration: capture min/max
    if (calibrating) {
      if (compassX < calMinX) calMinX = compassX;
      if (compassX > calMaxX) calMaxX = compassX;
      if (compassY < calMinY) calMinY = compassY;
      if (compassY > calMaxY) calMaxY = compassY;
    }

    // Apply calibration offsets
    float cx = compassX - offsetX;
    float cy = compassY - offsetY;
    if (calibrated) {
      cx *= scaleX;
      cy *= scaleY;
    }

    heading = atan2(cy, -cx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
  }
}

void initCompass() {
  Serial.println("[COMPASS] Init QMC5883P at 0x2C...");
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(QMC5883P_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(COMPASS_ADDR, (byte)1);
  if (Wire.available()) Serial.printf("[COMPASS] Chip ID: 0x%02X\n", Wire.read());

  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(QMC5883P_CTRL1); Wire.write(0x1D);
  Wire.endTransmission();
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(QMC5883P_CTRL2); Wire.write(0x0C);
  Wire.endTransmission();
  compassOk = true;
  delay(100);
  Serial.println("[COMPASS] QMC5883P initialized!");
}

int countI2CDevices() {
  int c = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) c++;
  }
  return c;
}

// =========================================
// Setup
// =========================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(3000);

  Serial.println("\n========================================");
  Serial.println("  OpenDrones Nav Module v8");
  Serial.println("  UBX-NAV-PVT Parser (MicoAir M10)");
  Serial.println("========================================\n");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (bmp.begin(0x76)) {
    bmpOk = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[BMP280] Online");
  } else Serial.println("[BMP280] NOT FOUND");

  initCompass();

  // MicoAir MG-A01: 115200 baud, UBX-PVT default
  Serial1.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] UART @ 115200 baud");

  // MSP via direct UART to FC on D8/D9
  msp.begin(Serial2, FC_RX_PIN, FC_TX_PIN, FC_BAUD);
  Serial.println("[MSP] Serial2 @ 115200 baud on D8(GPIO7)/D9(GPIO8)");
  for (int i = 0; i < RC_CHANNELS; i++) rcChannels[i] = RC_CHANNEL_CENTER;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.printf("[WiFi] AP: %s  Pass: %s\n", ap_ssid, ap_pass);
  Serial.printf("[WiFi] Dashboard: http://%s\n", WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", dashboard_html); });

  // Start compass calibration
  server.on("/calibrate/start", HTTP_GET, [](){
    calibrating = true;
    calMinX = 999; calMaxX = -999;
    calMinY = 999; calMaxY = -999;
    calibrated = false;
    offsetX = 0; offsetY = 0;
    Serial.println("[CAL] Calibration STARTED — rotate in figure-8!");
    server.send(200, "application/json", "{\"status\":\"calibrating\"}");
  });

  // Finish compass calibration
  server.on("/calibrate/finish", HTTP_GET, [](){
    calibrating = false;
    // Calculate hard iron offsets (center of the ellipse)
    offsetX = (calMaxX + calMinX) / 2.0;
    offsetY = (calMaxY + calMinY) / 2.0;
    // Calculate scale to make it circular
    float rangeX = calMaxX - calMinX;
    float rangeY = calMaxY - calMinY;
    if (rangeX > 0 && rangeY > 0) {
      float avgRange = (rangeX + rangeY) / 2.0;
      scaleX = avgRange / rangeX;
      scaleY = avgRange / rangeY;
    }
    calibrated = true;
    Serial.printf("[CAL] DONE! Offsets: X=%.4f Y=%.4f | Scale: X=%.3f Y=%.3f\n",
      offsetX, offsetY, scaleX, scaleY);
    Serial.printf("[CAL] Range: X=[%.4f to %.4f] Y=[%.4f to %.4f]\n",
      calMinX, calMaxX, calMinY, calMaxY);
    String j = "{\"status\":\"done\",\"offX\":" + String(offsetX,4) +
      ",\"offY\":" + String(offsetY,4) +
      ",\"scaleX\":" + String(scaleX,3) +
      ",\"scaleY\":" + String(scaleY,3) + "}";
    server.send(200, "application/json", j);
  });

  server.on("/api", HTTP_GET, [](){
    String j = "{\"te\":" + String(temperature * 9.0 / 5.0 + 32.0, 1) +
      ",\"tc\":" + String(temperature, 1) +
      ",\"pe\":" + String(pressure,1) +
      ",\"at\":" + String(altitude_bmp,1) +
      ",\"af\":" + String(altitude_bmp * 3.28084, 1) +
      ",\"bm\":" + (bmpOk?"true":"false") +
      ",\"lt\":" + String(gpsLat,7) +
      ",\"ln\":" + String(gpsLon,7) +
      ",\"ga\":" + String(gpsAlt,1) +
      ",\"st\":" + String(gpsSats) +
      ",\"ft\":" + String(gpsFixType) +
      ",\"sp\":" + String(gpsSpeed,1) +
      ",\"hg\":" + String(heading,1) +
      ",\"cx\":" + String(compassX,4) +
      ",\"cy\":" + String(compassY,4) +
      ",\"cz\":" + String(compassZ,4) +
      ",\"hr\":" + String(gpsHour) +
      ",\"mn\":" + String(gpsMin) +
      ",\"sc\":" + String(gpsSec) +
      ",\"up\":" + String(ubxPacketCount) +
      ",\"upt\":" + String(millis()/1000) +
      ",\"hk\":" + String(ESP.getFreeHeap()) +
      ",\"ic\":" + String(countI2CDevices()) +
      ",\"wc\":" + String(WiFi.softAPgetStationNum()) +
      ",\"cal\":" + (calibrating?"\"running\"":(calibrated?"\"done\"":"\"none\"")) +
      ",\"offX\":" + String(offsetX,4) +
      ",\"offY\":" + String(offsetY,4) +
      ",\"fc\":" + (fcConnected?"true":"false") +
      ",\"fv\":\"" + String(fcVariant) + "\"" +
      ",\"fa\":\"" + String(fcApiMajor) + "." + String(fcApiMinor) + "\"" +
      ",\"fr\":" + String(fcRoll) +
      ",\"fp\":" + String(fcPitch) +
      ",\"fy\":" + String(fcYaw) +
      ",\"armed\":" + (fcArmed?"true":"false") +
      ",\"mspTx\":" + String(msp.packetsSent) +
      ",\"mspRx\":" + String(msp.packetsReceived) +
      ",\"rcT\":" + String(rcChannels[2]) +
      ",\"rcR\":" + String(rcChannels[0]) +
      ",\"rcP\":" + String(rcChannels[1]) +
      ",\"rcYw\":" + String(rcChannels[3]) +
      ",\"rcA\":" + String(rcChannels[4]) +
      ",\"rc6\":" + String(rcChannels[5]) +
      ",\"rc7\":" + String(rcChannels[6]) +
      ",\"rc8\":" + String(rcChannels[7]) +
      "}";
    server.send(200, "application/json", j);
  });
  server.begin();
  Serial.println("[WEB] Server started\n");
}

// =========================================
// Loop
// =========================================
void loop() {
  // --- GPS UBX parsing ---
  parseUBX();

  // --- MSP communication with FC ---
  // Poll FC every 200ms (round-robin different requests)
  if (millis() - lastFcPoll >= 200) {
    lastFcPoll = millis();
    if (fcConnected) {
      switch (fcPollState) {
        case 0: msp.requestStatus(); break;
        case 1: msp.requestRC(); break;
        case 2: msp.requestAttitude(); break;
        default: msp.requestStatus(); break;
      }
      fcPollState = (fcPollState + 1) % 3;
    } else {
      // Not connected yet - try to discover FC
      msp.requestAPIVersion();
    }
  }

  // Process incoming MSP packets
  while (msp.readPacket()) {
    uint8_t cmd = msp.getCommand();
    switch (cmd) {
      case MSP_API_VERSION: {
        uint8_t maj, min;
        if (msp.parseAPIVersion(maj, min)) {
          fcApiMajor = maj;
          fcApiMinor = min;
          fcConnected = true;
          Serial.printf("[MSP] FC connected! API v%d.%d\n", maj, min);
          // Now get the FC variant
          msp.sendRequest(MSP_FC_VARIANT);
        }
        break;
      }
      case MSP_FC_VARIANT: {
        char var[5];
        if (msp.parseFCVariant(var)) {
          memcpy(fcVariant, var, 5);
          Serial.printf("[MSP] FC Variant: %s\n", fcVariant);
        }
        break;
      }
      case MSP_STATUS: {
        uint16_t cycleTime;
        if (msp.parseStatus(fcFlags, cycleTime)) {
          fcArmed = (fcFlags & (1 << 0)) != 0;  // ARM flag
        }
        break;
      }
      case MSP_RC: {
        if (msp.parseRC(rcChannels)) {
          // Successfully read RC channels from FC
        }
        break;
      }
      case MSP_ATTITUDE: {
        msp.parseAttitude(fcRoll, fcPitch, fcYaw);
        break;
      }
    }
  }

  // Check FC connection
  msp.checkConnection();
  if (!msp.isConnected() && fcConnected) {
    fcConnected = false;
    Serial.println("[MSP] FC disconnected!");
  }

  // --- Sensor reading (every 1 second) ---
  if (millis() - lastSensorRead >= 1000) {
    lastSensorRead = millis();
    if (bmpOk) {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0F;
      altitude_bmp = bmp.readAltitude(1013.25);
    }
    readCompass();

    // Log to serial (every 5 seconds to reduce spam)
    if (millis() - lastMspLog >= 5000) {
      lastMspLog = millis();
      Serial.printf("[S] BMP:%.1fC/%.0fhPa/%.0fm | GPS:%s %.7f,%.7f Alt:%.0fm S:%d Fix:%d | Hdg:%.0f | FC:%s Roll:%.1f Pitch:%.1f Yaw:%.1f MSP:%d/%d\n",
        temperature, pressure, altitude_bmp,
        gpsFix?"FIX!":"     ", gpsLat, gpsLon, gpsAlt,
        gpsSats, gpsFixType,
        heading,
        fcConnected?"ON":"OFF",
        fcRoll/10.0, fcPitch/10.0, fcYaw/10.0,
        msp.packetsSent, msp.packetsReceived);
    }
  }

  server.handleClient();

  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
