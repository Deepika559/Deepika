// FEMORA Health Monitor v1.2 + BLE FINAL
// BlePayload struct is in femora_ble.h
// Arduino pre-scanner cannot reorder #include directives
// so the struct is always visible before bleSend() — no more errors

#include "femora_ble.h"
#include <Wire.h>
#include <SPI.h>
#include "MAX30105.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

// ─── PINS ────────────────────────────────────────────────────────────
#define OLED_DC   9
#define OLED_CS  10
#define OLED_RST  8
#define GSR_PIN   4
#define BTN_PIN  13

Adafruit_SSD1306  display(128,64,&SPI,OLED_DC,OLED_RST,OLED_CS);
MAX30105          sensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ─── GSR ─────────────────────────────────────────────────────────────
#define GSR_NO_TOUCH_MIN          2000
#define GSR_TOUCH_MAX             1800
#define GSR_CHANGE_TH                8
#define GSR_NO_CONTACT_THRESHOLD  2300   // ← ADDED: above this = no skin contact
bool          contact      = false;
unsigned long contactStart = 0;
#define DEBOUNCE_N 5
int touchCnt=0, releaseCnt=0;
float gsrFiltered=0, gsrFast=0;

// ─── BASELINE ────────────────────────────────────────────────────────
#define SETTLE_MS         1000UL
#define INITIAL_SAMPLES      5
#define RECAL_INTERVAL_MS 60000UL
#define RECAL_SAMPLES        15
#define RECAL_BUF_SIZE       15
#define RECAL_TRIM_PCT     0.20f
#define RECAL_SD_MAX       20.0f
#define RECAL_DELTA_MAX    80.0f
#define RECAL_ALPHA         0.15f
#define TREND_WIN             5
#define TREND_RISE_TH       20.0f

enum BaselineState { BL_IDLE,BL_SETTLING,BL_SAMPLING,BL_LOCKED,BL_RECAL_COLLECT };
BaselineState blState=BL_IDLE;
unsigned long blTimer=0;
float baseline=0;
bool  baselineReady=false;
String calStatus="No Contact";
float initSum=0; int initCnt=0;
float recalBuf[RECAL_BUF_SIZE];
int   recalCnt=0; float recalSum=0,recalSumSq=0;
float trendBuf[TREND_WIN]={0}; int trendIdx=0; bool trendFull=false;

// ─── STRESS ──────────────────────────────────────────────────────────
float stressRaw=0,stressSmooth=0;
#define STRESS_CALM_MAX  5
#define STRESS_MILD_MAX 15
#define STRESS_MOD_MAX  30

// ─── MAX30102 ────────────────────────────────────────────────────────
float avgBpm=0; long lastPeak=0,irSmooth=0,irPrev=0;
bool  peakRising=false;
float irDC=0,redDC=0,spo2Avg=0;
#define MIN_PEAK_MS 400
#define IR_ALPHA    0.3f

// ─── SMOOTH BUFFERS ──────────────────────────────────────────────────
#define SMOOTH_SIZE 6
float bpmBuf[SMOOTH_SIZE]={0},spo2Buf[SMOOTH_SIZE]={0},tempBuf[SMOOTH_SIZE]={0};
int smoothIdx=0;

// ─── BBT + CYCLE ─────────────────────────────────────────────────────
#define BBT_ALPHA         0.08f
#define BBT_BASELINE_N      20
#define BBT_OVULATION_RISE 0.20f
#define BBT_LUTEAL_MIN     0.15f
float bbtEMA=0,bbtBase=0,bbtBaseSum=0,bbtPrevEMA=0;
bool  bbtBaseReady=false; int bbtBaseCnt=0;

enum CyclePhase { PHASE_UNKNOWN,PHASE_MENSTRUAL,PHASE_FOLLICULAR,PHASE_OVULATION,PHASE_LUTEAL };
CyclePhase cyclePhase=PHASE_UNKNOWN; int lutealCnt=0;
#define LUTEAL_CONFIRM_N 5

const char* phaseLabel(CyclePhase p){
  switch(p){
    case PHASE_MENSTRUAL:  return "Menstrual";
    case PHASE_FOLLICULAR: return "Follicular";
    case PHASE_OVULATION:  return "Ovulation";
    case PHASE_LUTEAL:     return "Luteal";
    default:               return "Learning...";
  }
}
const char* hormoneNote(CyclePhase p,float stress){
  switch(p){
    case PHASE_MENSTRUAL:  return "Estrogen rising";
    case PHASE_FOLLICULAR: return "Peak estrogen";
    case PHASE_OVULATION:  return "LH surge";
    case PHASE_LUTEAL:
      if(stress>STRESS_MOD_MAX) return "High stress/PMS";
      return "Progesterone hi";
    default: return "Calibrating BBT";
  }
}

void drawPhaseIcon(int x,int y,CyclePhase p){
  switch(p){
    case PHASE_MENSTRUAL:
      display.drawPixel(x+2,y,WHITE);
      display.drawPixel(x+1,y+1,WHITE);display.drawPixel(x+3,y+1,WHITE);
      display.drawPixel(x,y+2,WHITE);display.drawPixel(x+4,y+2,WHITE);
      display.drawPixel(x,y+3,WHITE);display.drawPixel(x+4,y+3,WHITE);
      display.drawPixel(x+1,y+4,WHITE);display.drawPixel(x+3,y+4,WHITE);
      display.drawPixel(x+2,y+5,WHITE);break;
    case PHASE_FOLLICULAR:
      display.drawPixel(x+2,y,WHITE);
      display.drawPixel(x+1,y+1,WHITE);display.drawPixel(x+3,y+1,WHITE);
      display.drawPixel(x+2,y+2,WHITE);display.drawPixel(x+2,y+3,WHITE);
      display.drawPixel(x+2,y+4,WHITE);display.drawPixel(x+2,y+5,WHITE);break;
    case PHASE_OVULATION:
      display.drawCircle(x+2,y+3,2,WHITE);
      display.drawPixel(x+2,y,WHITE);display.drawPixel(x+4,y+1,WHITE);
      display.drawPixel(x,y+1,WHITE);display.drawPixel(x+2,y+6,WHITE);break;
    case PHASE_LUTEAL:
      display.drawCircle(x+2,y+3,3,WHITE);
      display.fillRect(x+3,y,3,7,BLACK);break;
    default:
      display.drawPixel(x+2,y+1,WHITE);display.drawPixel(x+2,y+2,WHITE);
      display.drawPixel(x+2,y+4,WHITE);break;
  }
}

void updateBBT(float tempC){
  if(tempC<20.0f||tempC>42.0f) return;
  if(!bbtBaseReady){
    bbtEMA=(bbtBaseCnt==0)?tempC:BBT_ALPHA*tempC+(1.0f-BBT_ALPHA)*bbtEMA;
    bbtBaseSum+=tempC; bbtBaseCnt++;
    if(bbtBaseCnt>=BBT_BASELINE_N){
      bbtBase=bbtBaseSum/bbtBaseCnt; bbtBaseReady=true; bbtPrevEMA=bbtEMA;
      Serial.print(">> BBT locked: "); Serial.println(bbtBase,3);
    }
    return;
  }
  bbtPrevEMA=bbtEMA;
  bbtEMA=BBT_ALPHA*tempC+(1.0f-BBT_ALPHA)*bbtEMA;
  float delta=bbtEMA-bbtBase, rise=bbtEMA-bbtPrevEMA;
  if(delta<=0.0f){ lutealCnt=0; cyclePhase=(delta<-0.05f)?PHASE_MENSTRUAL:PHASE_FOLLICULAR; }
  else if(rise>=BBT_OVULATION_RISE&&cyclePhase==PHASE_FOLLICULAR){ lutealCnt=0; cyclePhase=PHASE_OVULATION; }
  else if(delta>=BBT_LUTEAL_MIN){ lutealCnt++; if(lutealCnt>=LUTEAL_CONFIRM_N) cyclePhase=PHASE_LUTEAL; }
  else if(cyclePhase==PHASE_OVULATION&&delta>0.0f){ lutealCnt++; if(lutealCnt>=LUTEAL_CONFIRM_N) cyclePhase=PHASE_LUTEAL; }
}

// ─── OUTPUT TIMER ────────────────────────────────────────────────────
unsigned long lastOutput=0;
#define OUTPUT_INTERVAL_MS 200
bool blinkState=false; unsigned long lastBlink=0;
#define BLINK_MS 600
uint8_t uiPage=0; bool btnPrev=HIGH; unsigned long btnDebounce=0;

// ─── HELPERS ─────────────────────────────────────────────────────────
float smoothAvgNonZero(float*b,int s){
  float sum=0; int c=0;
  for(int i=0;i<s;i++) if(b[i]>0){sum+=b[i];c++;}
  return c>0?sum/c:0;
}
float smoothAvgAll(float*b,int s){
  float sum=0; for(int i=0;i<s;i++) sum+=b[i]; return sum/s;
}
float calculateSpO2(long irVal,long redVal){
  irDC=irDC*0.97f+irVal*0.03f; redDC=redDC*0.97f+redVal*0.03f;
  float irAC=fabsf((float)irVal-irDC), redAC=fabsf((float)redVal-redDC);
  if(irAC<300||redAC<300) return 0;
  float R=(redAC/redDC)/(irAC/irDC);
  return constrain(104.0f-17.0f*R,85.0f,100.0f);
}
float trimmedMean(float*buf,int n,float trimPct){
  float tmp[RECAL_BUF_SIZE];
  for(int i=0;i<n;i++) tmp[i]=buf[i];
  for(int i=1;i<n;i++){float key=tmp[i];int j=i-1;while(j>=0&&tmp[j]>key){tmp[j+1]=tmp[j];j--;}tmp[j+1]=key;}
  int drop=max(1,(int)(n*trimPct)); int lo=drop,hi=n-drop;
  if(hi<=lo){lo=0;hi=n;} float sum=0;
  for(int i=lo;i<hi;i++) sum+=tmp[i];
  return sum/(hi-lo);
}
void pushTrend(float v){
  trendBuf[trendIdx]=v; trendIdx=(trendIdx+1)%TREND_WIN; if(trendIdx==0) trendFull=true;
}
bool isTrendingUp(){
  if(!trendFull) return false;
  return (trendBuf[(trendIdx+TREND_WIN-1)%TREND_WIN]-trendBuf[trendIdx])>TREND_RISE_TH;
}
void resetContactState(){
  baselineReady=false; stressRaw=0; stressSmooth=0;
  touchCnt=0; releaseCnt=0; blState=BL_IDLE;
  initSum=0; initCnt=0; recalCnt=0; recalSum=0; recalSumSq=0;
  trendIdx=0; trendFull=false; calStatus="Calibrating";
}
void updateBaseline(){
  if(!contact){ blState=BL_IDLE; baselineReady=false; baseline=0; stressSmooth=0; calStatus="No Contact"; return; }
  pushTrend(gsrFiltered);
  unsigned long now=millis();
  switch(blState){
    case BL_IDLE:
      baselineReady=false; blTimer=now; initSum=0; initCnt=0;
      blState=BL_SETTLING; calStatus="Settling";
      Serial.println(">> BL: IDLE->SETTLING"); break;
    case BL_SETTLING:
      calStatus="Settling"; if(now-blTimer<SETTLE_MS) break;
      blTimer=now; initSum=0; initCnt=0; blState=BL_SAMPLING;
      Serial.println(">> BL: SETTLING->SAMPLING"); break;
    case BL_SAMPLING:
      calStatus="Calibrating"; initSum+=gsrFiltered; initCnt++;
      if(initCnt>=INITIAL_SAMPLES){
        baseline=initSum/initCnt; baselineReady=true; stressSmooth=0;
        blTimer=now; blState=BL_LOCKED; calStatus="Measuring";
        Serial.print(">> BL LOCKED: "); Serial.println(baseline,2);
      } break;
    case BL_LOCKED:
      calStatus="Measuring";
      if(now-blTimer>=RECAL_INTERVAL_MS){
        if(isTrendingUp()){ blTimer=now; break; }
        recalCnt=0; recalSum=0; recalSumSq=0; blTimer=now; blState=BL_RECAL_COLLECT;
        Serial.println(">> BL: LOCKED->RECAL");
      } break;
    case BL_RECAL_COLLECT:
      calStatus="Measuring";
      if(recalCnt<RECAL_BUF_SIZE){ recalBuf[recalCnt]=gsrFiltered; recalSum+=gsrFiltered; recalSumSq+=gsrFiltered*gsrFiltered; recalCnt++; }
      if(recalCnt>=RECAL_SAMPLES){
        float rawMean=recalSum/recalCnt;
        float var=(recalSumSq/recalCnt)-(rawMean*rawMean);
        float sd=sqrtf(var<0?0:var);
        float tM=trimmedMean(recalBuf,recalCnt,RECAL_TRIM_PCT);
        float dlt=fabsf(tM-baseline);
        if(sd<RECAL_SD_MAX&&dlt<RECAL_DELTA_MAX){ baseline=(1.0f-RECAL_ALPHA)*baseline+RECAL_ALPHA*tM; Serial.print(">> RECAL OK: "); Serial.println(baseline,2); }
        else Serial.println(">> RECAL SKIPPED");
        blTimer=millis(); blState=BL_LOCKED;
      } break;
  }
}

// ─── OLED ────────────────────────────────────────────────────────────
void drawStressBar(int x,int y,int w,int h,float pct){
  display.drawRect(x,y,w,h,WHITE);
  int fill=constrain((int)((pct/100.0f)*(w-2)),0,w-2);
  if(fill>0) display.fillRect(x+1,y+1,fill,h-2,WHITE);
}
void drawHeart(int x,int y){
  const int8_t pts[][2]={{1,0},{2,0},{4,0},{5,0},{0,1},{3,1},{6,1},{0,2},{6,2},{1,3},{5,3},{2,4},{4,4},{3,5}};
  for(auto&p:pts) display.drawPixel(x+p[0],y+p[1],WHITE);
}
void drawThermo(int x,int y){ display.drawRect(x+1,y,3,7,WHITE); display.fillCircle(x+2,y+8,2,WHITE); display.fillRect(x+1,y+5,3,3,WHITE); }
void drawWave(int x,int y){ const int8_t dy[]={3,2,1,0,1,2,3,4,5,4}; for(int i=0;i<10;i++) display.drawPixel(x+i,y+dy[i],WHITE); }
void drawFemoramark(int x,int y){ display.fillRect(x,y,5,1,WHITE); display.fillRect(x,y+2,4,1,WHITE); display.fillRect(x,y,1,5,WHITE); }
void drawBBTBar(int x,int y,float pct){
  display.drawRect(x,y,4,20,WHITE);
  int fill=constrain((int)(pct*18),0,18);
  if(fill>0) display.fillRect(x+1,y+19-fill,2,fill,WHITE);
  display.fillCircle(x+1,y+22,3,WHITE);
}

void drawPageVitals(float smBpm,float smSpo2,float smTemp,float stressIndex,String state,int raw){
  display.clearDisplay();
  if(blinkState) display.fillCircle(3,3,2,WHITE); else display.drawCircle(3,3,2,WHITE);
  display.setTextSize(1); display.setCursor(8,0); display.print("FEMORA");
  display.setCursor(70,0);
  if(!contact) display.print("NO TOUCH");
  else if(!baselineReady) display.print("CAL...");
  else if(blState==BL_RECAL_COLLECT) display.print("RECAL");
  else display.print("ACTIVE");
  drawFemoramark(122,0); display.drawFastHLine(0,9,128,WHITE);
  drawHeart(1,11); display.setCursor(10,11); display.print("HR:");
  if(smBpm>0){display.print((int)smBpm);display.print("bpm");}else display.print("---");
  drawWave(70,13); display.setCursor(82,11);
  if(smSpo2>0){display.print((int)smSpo2);display.print("%");}else display.print("--%");
  display.drawFastHLine(0,22,128,WHITE);
  drawThermo(1,23); display.setCursor(10,24); display.print("T:"); display.print(smTemp,1); display.print("\xf7""C");
  display.drawFastVLine(74,23,10,WHITE); display.setCursor(77,24); display.print("G:"); display.print(raw);
  display.drawFastHLine(0,34,128,WHITE);
  display.setCursor(1,36); display.print("STRESS:");
  if(!contact){ display.setCursor(58,36); display.print("--------"); }
  else if(!baselineReady){ display.setCursor(58,36); if(blState==BL_SETTLING) display.print("SETTLE.."); else display.print("CAL..."); }
  else{
    int lblW=0;
    if(state=="Calm")lblW=30; else if(state=="Mild")lblW=24; else if(state=="Moderate")lblW=54; else if(state=="High")lblW=24;
    if(lblW>0){ display.fillRect(57,35,lblW+2,9,WHITE); display.setTextColor(BLACK); display.setCursor(58,36); display.print(state); display.setTextColor(WHITE); }
  }
  float barPct=baselineReady?constrain(stressIndex/40.0f*100.0f,0,100):0;
  drawStressBar(1,46,126,8,barPct);
  display.drawPixel(1+(int)((5.0f/40.0f)*124),45,WHITE);
  display.drawPixel(1+(int)((15.0f/40.0f)*124),45,WHITE);
  display.drawPixel(1+(int)((30.0f/40.0f)*124),45,WHITE);
  display.setCursor(1,56); display.print("SI:"); display.print(stressIndex,1);
  display.setCursor(48,56); display.print("B:"); display.print((int)baseline);
  if(contact&&baselineReady&&blState==BL_LOCKED){
    unsigned long elapsed=millis()-blTimer;
    unsigned long remain=(RECAL_INTERVAL_MS>elapsed)?(RECAL_INTERVAL_MS-elapsed)/1000UL:0;
    display.setCursor(90,56); display.print("r:"); if(remain<10) display.print("0"); display.print(remain); display.print("s");
  }
  display.fillRect(119,61,3,3,WHITE); display.drawRect(124,61,3,3,WHITE); display.display();
}

void drawPageCycle(float smBpm,float smSpo2,float stressIndex,String stressState){
  display.clearDisplay();
  if(blinkState) display.fillCircle(3,3,2,WHITE); else display.drawCircle(3,3,2,WHITE);
  display.setTextSize(1); display.setCursor(8,0); display.print("FEMORA");
  display.setCursor(62,0); display.print("CYCLE MON");
  drawFemoramark(122,0); display.drawFastHLine(0,9,128,WHITE);
  drawPhaseIcon(4,13,cyclePhase);
  display.setCursor(18,13); display.print(phaseLabel(cyclePhase));
  display.setCursor(18,24); display.print(hormoneNote(cyclePhase,stressSmooth));
  display.drawFastHLine(0,33,128,WHITE);
  display.setCursor(1,35); display.print("BBT:");
  if(bbtBaseReady){
    display.print(bbtEMA,2); display.print("\xf7""C");
    float delta=bbtEMA-bbtBase; display.setCursor(75,35); display.print("d:");
    if(delta>=0) display.print("+"); display.print(delta,2);
  } else display.print("Learning...");
  if(bbtBaseReady){
    float pct=constrain((bbtEMA-bbtBase+0.5f)/1.0f,0.0f,1.0f);
    drawBBTBar(1,42,pct); display.setCursor(12,44); display.print("Base:"); display.print(bbtBase,2); display.print("\xf7""C");
  }
  display.drawFastHLine(0,54,128,WHITE);
  display.setCursor(1,56); display.print("HR:"); if(smBpm>0) display.print((int)smBpm); else display.print("--");
  display.setCursor(38,56); display.print("O2:"); if(smSpo2>0){display.print((int)smSpo2);display.print("%");}else display.print("--%");
  display.setCursor(76,56); display.print("S:"); display.print(stressState.substring(0,4));
  display.drawRect(119,61,3,3,WHITE); display.fillRect(124,61,3,3,WHITE); display.display();
}

void drawBootSplash(){
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(14,8); display.print("FEMORA");
  display.setTextSize(1); display.setCursor(8,30); display.print("Health Monitor v1.2");
  display.drawFastHLine(14,26,100,WHITE); display.drawFastHLine(14,27,100,WHITE);
  display.setCursor(16,46); display.print("Wearable  Edition");
  display.drawPixel(0,0,WHITE);display.drawPixel(2,0,WHITE);display.drawPixel(0,2,WHITE);
  display.drawPixel(127,0,WHITE);display.drawPixel(125,0,WHITE);display.drawPixel(127,2,WHITE);
  display.drawPixel(0,63,WHITE);display.drawPixel(0,61,WHITE);display.drawPixel(2,63,WHITE);
  display.drawPixel(127,63,WHITE);display.drawPixel(125,63,WHITE);display.drawPixel(127,61,WHITE);
  display.display();
}

// ─── BLE ─────────────────────────────────────────────────────────────
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789ABC"
#define BLE_CHAR_UUID    "12345678-1234-1234-1234-123456789ABD"
#define BLE_NOTIFY_MS    5000UL

static BlePayload        bleData;
static BleAccum          bleAccum    = {0};
static SemaphoreHandle_t bleMutex   = nullptr;
static bool bleConnected     = false;
static bool bleSendOnConnect = false;
static BLEServer*         pServer   = nullptr;
static BLECharacteristic* pChar     = nullptr;

class FemoraServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected=true; bleSendOnConnect=true;
    Serial.println(">> BLE: connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected=false; bleSendOnConnect=false;
    BLEDevice::startAdvertising();
    Serial.println(">> BLE: disconnected");
  }
};

static void bleSend(const BlePayload& snap){
  char c1[40], c2[40], c3[40];

  // Chunk 1: Heart rate, SpO2, Stress index, Stress state
  snprintf(c1,sizeof(c1),"HR:%d O2:%d%% SI:%.1f SS:%s",
    snap.hr, snap.spo2, snap.si, snap.ss);

  // ── ADDED: if GSR ≥ 2300, TX2 explicitly says NO CONTACT ──────────
  // This tells nRF Connect the sensor is not on skin — prevents
  // logging false data that pollutes the cycle / stress records.
  if(snap.gsr >= GSR_NO_CONTACT_THRESHOLD){
    snprintf(c2,sizeof(c2),"T:%.1fC GSR:%d CT:NO CONTACT",
      snap.temp, snap.gsr);
  } else {
    snprintf(c2,sizeof(c2),"T:%.1fC GSR:%d CT:%d",
      snap.temp, snap.gsr, snap.ct);
  }
  // ── END ADDED ────────────────────────────────────────────────────

  // Chunk 3: Cycle phase and BBT
  snprintf(c3,sizeof(c3),"PH:%s BBT:%.2fC BD:%+.2f",
    snap.ph, snap.bbt, snap.bd);

  pChar->setValue((uint8_t*)c1,strlen(c1)); pChar->notify(); vTaskDelay(pdMS_TO_TICKS(50));
  pChar->setValue((uint8_t*)c2,strlen(c2)); pChar->notify(); vTaskDelay(pdMS_TO_TICKS(50));
  pChar->setValue((uint8_t*)c3,strlen(c3)); pChar->notify(); vTaskDelay(pdMS_TO_TICKS(50));

  Serial.print(">> TX1: "); Serial.println(c1);
  Serial.print(">> TX2: "); Serial.println(c2);
  Serial.print(">> TX3: "); Serial.println(c3);
}

void bleTask(void* param){
  unsigned long lastNotify=0;
  for(;;){
    if(!bleConnected){ vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
    if(bleSendOnConnect){
      bleSendOnConnect=false; vTaskDelay(pdMS_TO_TICKS(2000));
      BlePayload snap;
      if(xSemaphoreTake(bleMutex,pdMS_TO_TICKS(10))==pdTRUE){ snap=bleData; xSemaphoreGive(bleMutex); }
      bleSend(snap); lastNotify=millis();
      Serial.println(">> BLE: connect-burst sent"); continue;
    }
    unsigned long now=millis();
    if(now-lastNotify<BLE_NOTIFY_MS){ vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    lastNotify=now;
    BlePayload snap;
    if(xSemaphoreTake(bleMutex,pdMS_TO_TICKS(10))==pdTRUE){
      if(bleAccum.count>0){
        snap.hr  =(int)(bleAccum.hr_sum  /bleAccum.count);
        snap.spo2=(int)(bleAccum.spo2_sum/bleAccum.count);
        snap.gsr =(int)(bleAccum.gsr_sum /bleAccum.count);
        snap.temp=      bleAccum.temp_sum/bleAccum.count;
        snap.si  =      bleAccum.si_sum  /bleAccum.count;
        snap.bbt =      bleAccum.bbt_sum /bleAccum.count;
        snap.bd  =      bleAccum.bd_sum  /bleAccum.count;
        snap.ct  =(bleAccum.ct_sum>(bleAccum.count/2))?1:0;
        strncpy(snap.ss,bleAccum.ss,11); snap.ss[11]=0;
        strncpy(snap.ph,bleAccum.ph,11); snap.ph[11]=0;
      } else snap=bleData;
      bleAccum={0}; xSemaphoreGive(bleMutex);
    } else { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    bleSend(snap);
  }
}

void initBLE(){
  bleMutex=xSemaphoreCreateMutex();
  BLEDevice::init("FEMORA-v1.2");
  BLEDevice::setPower(ESP_PWR_LVL_N21);
  BLEDevice::setMTU(185);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
  BLESecurity* pSec=new BLESecurity();
  pSec->setAuthenticationMode(ESP_LE_AUTH_NO_BOND);
  pSec->setCapability(ESP_IO_CAP_NONE);
  pServer=BLEDevice::createServer(); pServer->setCallbacks(new FemoraServerCB());
  BLEService* pSvc=pServer->createService(BLE_SERVICE_UUID);
  pChar=pSvc->createCharacteristic(BLE_CHAR_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902()); pSvc->start();
  BLEAdvertising* pAdv=BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID); pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x10); pAdv->setMaxPreferred(0x30);
  BLEDevice::startAdvertising();
  xTaskCreatePinnedToCore(bleTask,"BLE_TASK",4096,nullptr,1,nullptr,0);
  Serial.println(">> BLE init OK TX=-21dBm");
}

void updateBleData(int hr,int spo2,float temp,int rawGsr,float si,const char* ss,float bbt,float bd,uint8_t ct){
  if(xSemaphoreTake(bleMutex,pdMS_TO_TICKS(5))==pdTRUE){
    bleAccum.hr_sum+=hr; bleAccum.spo2_sum+=spo2; bleAccum.gsr_sum+=rawGsr;
    bleAccum.temp_sum+=temp; bleAccum.si_sum+=si; bleAccum.bbt_sum+=bbt;
    bleAccum.bd_sum+=bd; bleAccum.ct_sum+=ct; bleAccum.count++;
    strncpy(bleAccum.ss,ss,11); bleAccum.ss[11]=0;
    strncpy(bleAccum.ph,phaseLabel(cyclePhase),11); bleAccum.ph[11]=0;
    bleData.hr=hr; bleData.spo2=spo2; bleData.gsr=rawGsr;
    bleData.temp=temp; bleData.si=si; bleData.bbt=bbt; bleData.bd=bd; bleData.ct=ct;
    strncpy(bleData.ss,ss,11); bleData.ss[11]=0;
    strncpy(bleData.ph,phaseLabel(cyclePhase),11); bleData.ph[11]=0;
    xSemaphoreGive(bleMutex);
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db); analogReadResolution(12);
  pinMode(BTN_PIN,INPUT_PULLUP); delay(500);
  float s=0; for(int i=0;i<30;i++){s+=analogRead(GSR_PIN);delay(10);}
  gsrFiltered=s/30.0f; gsrFast=gsrFiltered;
  Wire.begin(6,7); SPI.begin(12,-1,11,10);
  if(!display.begin(SSD1306_SWITCHCAPVCC)){ Serial.println("OLED FAIL"); while(1); }
  display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
  drawBootSplash();
  mlx.begin();
  sensor.begin(Wire,I2C_SPEED_STANDARD); sensor.setup();
  sensor.setPulseAmplitudeIR(0x1F); sensor.setPulseAmplitudeRed(0x1F);
  initBLE();
  Serial.println("=== FEMORA v1.2 BLE FINAL ===");
  delay(2500);
}

// ─── LOOP ────────────────────────────────────────────────────────────
void loop(){
  int raw=analogRead(GSR_PIN);
  if(raw<=0||raw>4095) return;
  gsrFiltered=0.9f*gsrFiltered+0.1f*(float)raw;
  gsrFast=0.6f*gsrFast+0.4f*(float)raw;
  float changeRate=fabsf(gsrFast-gsrFiltered);
  bool rawTouching=(raw<GSR_NO_TOUCH_MIN)||(changeRate>GSR_CHANGE_TH*10);
  if(!contact){
    if(rawTouching){ touchCnt++; releaseCnt=0; if(touchCnt>=DEBOUNCE_N){ contact=true; contactStart=millis(); resetContactState(); Serial.println(">> Contact ON"); } }
    else touchCnt=0;
  } else {
    bool rawReleased=(raw>GSR_TOUCH_MAX)&&(changeRate<GSR_CHANGE_TH);
    if(rawReleased){ releaseCnt++; touchCnt=0; if(releaseCnt>=DEBOUNCE_N){ contact=false; releaseCnt=0; Serial.println(">> Contact OFF"); } }
    else releaseCnt=0;
  }
  long irVal=sensor.getIR(), redVal=sensor.getRed();
  if(irVal>50000){
    irSmooth=(long)(IR_ALPHA*irVal+(1.0f-IR_ALPHA)*irSmooth);
    long d=irSmooth-irPrev;
    if(d>0) peakRising=true;
    if(peakRising&&d<0){
      long now2=millis(), interval=now2-lastPeak;
      if(lastPeak&&interval>MIN_PEAK_MS&&interval<1500){ float bpm=60000.0f/interval; if(bpm>40&&bpm<150) avgBpm=avgBpm*0.85f+bpm*0.15f; }
      lastPeak=now2; peakRising=false;
    }
    irPrev=irSmooth;
    float sp=calculateSpO2(irVal,redVal); if(sp>0) spo2Avg=spo2Avg*0.95f+sp*0.05f;
  }
  bool btnNow=digitalRead(BTN_PIN);
  if(btnPrev==HIGH&&btnNow==LOW&&millis()-btnDebounce>50){ uiPage=(uiPage+1)%2; btnDebounce=millis(); }
  btnPrev=btnNow;
  if(millis()-lastOutput>=OUTPUT_INTERVAL_MS){
    lastOutput=millis();
    if(millis()-lastBlink>BLINK_MS){ blinkState=!blinkState; lastBlink=millis(); }
    updateBaseline();
    float stressIndex=0; String state="No Contact";
    if(contact&&baselineReady&&baseline>0){
      float delta=fabsf(gsrFiltered-baseline);
      stressRaw=(delta/baseline)*100.0f;
      stressSmooth=0.85f*stressSmooth+0.15f*stressRaw;
      stressIndex=stressSmooth;
      if(stressIndex<STRESS_CALM_MAX) state="Calm";
      else if(stressIndex<STRESS_MILD_MAX) state="Mild";
      else if(stressIndex<STRESS_MOD_MAX)  state="Moderate";
      else state="High";
    } else if(contact&&!baselineReady) state=calStatus;

    // ── ADDED: override state to "No Contact" when GSR ≥ 2300 ────────
    // This ensures the BLE packet and serial log both clearly mark
    // readings taken with no skin contact — critical for clean data.
    if(raw >= GSR_NO_CONTACT_THRESHOLD){
      state = "No Contact";
    }
    // ── END ADDED ───────────────────────────────────────────────────

    float temp=mlx.readObjectTempC(); updateBBT(temp);
    static int outputCount=0; outputCount++;
    if(outputCount>=3){
      outputCount=0;
      bpmBuf[smoothIdx]=(avgBpm>40&&avgBpm<150)?avgBpm:0;
      spo2Buf[smoothIdx]=(spo2Avg>85&&spo2Avg<100)?spo2Avg:0;
      tempBuf[smoothIdx]=temp; smoothIdx=(smoothIdx+1)%SMOOTH_SIZE;
    }
    float smBpm=smoothAvgNonZero(bpmBuf,SMOOTH_SIZE);
    float smSpo2=smoothAvgNonZero(spo2Buf,SMOOTH_SIZE);
    float smTemp=smoothAvgAll(tempBuf,SMOOTH_SIZE);
    Serial.print("Raw:");Serial.print(raw);Serial.print(" F:");Serial.print(gsrFiltered,1);
    Serial.print(" B:");Serial.print(baseline,1);Serial.print(" SI:");Serial.print(stressIndex,1);
    Serial.print(" Ct:");Serial.print(contact?1:0);Serial.print(" St:");Serial.print(state);
    Serial.print(" BL:");Serial.print((int)blState);
    Serial.print(" HR:");if(smBpm>0)Serial.print(smBpm,1);else Serial.print("--");
    Serial.print(" O2:");if(smSpo2>0)Serial.print(smSpo2,1);else Serial.print("--");
    Serial.print(" T:");Serial.print(smTemp,1);
    Serial.print(" BBT:");Serial.print(bbtEMA,3);
    Serial.print(" Ph:");Serial.println(phaseLabel(cyclePhase));
    if(uiPage==0) drawPageVitals(smBpm,smSpo2,smTemp,stressIndex,state,raw);
    else drawPageCycle(smBpm,smSpo2,stressIndex,state);
    float bbd=bbtBaseReady?(bbtEMA-bbtBase):0.0f;
    updateBleData((int)smBpm,(int)smSpo2,smTemp,raw,stressIndex,state.c_str(),bbtEMA,bbd,(uint8_t)contact);
  }
}
