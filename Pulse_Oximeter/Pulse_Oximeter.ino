#include <Arduino.h>

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================

// --- LED OUTPUTS (4-Wire Setup) ---
#define PIN_IR_WIN     19 
#define PIN_RED_WIN    5  
#define PIN_IR_PWM     18 
#define PIN_RED_PWM    17 

// --- ADC INPUTS ---
#define PIN_ADC_DC     34 // sPH: Raw DC signal (with PWM ripple)
#define PIN_ADC_AC     35 // G Sr_ac: AC signal (Offset ~2.5V)

// =============================================================
// ------------------- CONSTANTS & SETTINGS --------------------
// =============================================================

// --- Timing ---
#define SLOT_PERIOD_US  500  

// --- PWM Settings (20kHz) ---
#define PWM_FREQ        20000
#define PWM_RES_BITS    8    

// --- Calibration Targets ---
// Target: 2.0V (~2480 ADC).
// Safe Range: 0.5V (~620) to 3.0V (~3720).
// 2.0V is chosen to be safely in the middle of your 0.5V-3.0V requirement.
#define DC_TARGET_VAL   2480 
#define DC_TOLERANCE    100  
#define PWM_START_DUTY  50   
#define PWM_MAX_DUTY    240  
#define PWM_MIN_DUTY    10   

// --- Averaging Settings ---
// We take N samples quickly inside the ISR to average out the 20kHz PWM ripple.
// 20kHz period = 50us. 8 ADC reads takes ~100-150us, covering 2-3 PWM cycles.
#define NUM_SAMPLES_AVG 8 

// --- Filter Constants ---
float alpha_hp = 0.9968;
float a0_lp = 0.000241, a1_lp = 0.000482, a2_lp = 0.000241;
float b1_lp = -1.955578, b2_lp = 0.956544;

// =============================================================
// ---------------------- GLOBAL VARIABLES ---------------------
// =============================================================

enum SystemState { STATE_INIT, STATE_CALIBRATING, STATE_MEASURING };
volatile SystemState currentState = STATE_INIT;

volatile int irDuty = PWM_START_DUTY;
volatile int redDuty = PWM_START_DUTY;
volatile int timingPhase = 0; 

// Accumulators for the slow loop (averaged over many frames)
volatile long sumDC_IR = 0;
volatile long sumDC_Red = 0;
volatile int sampleCountDC = 0;

int avgDC_IR = 0;
int avgDC_Red = 0;

struct FilterState {
  float x_hp_old=0, y_hp_old=0;
  float x1_lp=0, x2_lp=0, y1_lp=0, y2_lp=0;
  float filteredOutput=0;
  bool  initialized=false; 
};
volatile FilterState filterIR, filterRed;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================
// -------------------- ISR & FILTER FUNCTIONS -----------------
// =============================================================

void runFilter(volatile FilterState &fs, float rawInput) {
  // Seed filter if new (handle 2.5V AC offset)
  if (!fs.initialized) {
    fs.x_hp_old = rawInput;
    fs.y_hp_old = 0;
    fs.x1_lp = 0; fs.x2_lp = 0; 
    fs.y1_lp = 0; fs.y2_lp = 0;
    fs.initialized = true;
    return; 
  }

  // 1. HPF
  float y_hp = alpha_hp * (fs.y_hp_old + rawInput - fs.x_hp_old);
  fs.x_hp_old = rawInput;
  fs.y_hp_old = y_hp;

  // 2. LPF
  float y_lp = a0_lp * y_hp + a1_lp * fs.x1_lp + a2_lp * fs.x2_lp - b1_lp * fs.y1_lp - b2_lp * fs.y2_lp;
  fs.x2_lp = fs.x1_lp; fs.x1_lp = y_hp;
  fs.y2_lp = fs.y1_lp; fs.y1_lp = y_lp;

  fs.filteredOutput = y_lp;
}

// Helper to perform burst sampling inside ISR
int readAverageDC() {
  long burstSum = 0;
  for (int i = 0; i < NUM_SAMPLES_AVG; i++) {
    burstSum += analogRead(PIN_ADC_DC);
  }
  return (int)(burstSum / NUM_SAMPLES_AVG);
}

void IRAM_ATTR onTimerISR() {
  int currentDC, rawAC;

  portENTER_CRITICAL_ISR(&timerMux);

  switch (timingPhase) {
    case 0: // IR WINDOW
      digitalWrite(PIN_IR_WIN, HIGH);
      
      // NEW: Take 8 samples and average them immediately
      // This smoothes out the PWM ripple you described
      currentDC = readAverageDC();
      sumDC_IR += currentDC;
      
      if (currentState == STATE_MEASURING) {
        rawAC = analogRead(PIN_ADC_AC);
        runFilter(filterIR, rawAC / 4095.0);
      }
      break;

    case 1: // DARK
      digitalWrite(PIN_IR_WIN, LOW);
      break;

    case 2: // RED WINDOW
      digitalWrite(PIN_RED_WIN, HIGH);
      
      // NEW: Take 8 samples and average them immediately
      currentDC = readAverageDC();
      sumDC_Red += currentDC;
      
      if (currentState == STATE_MEASURING) {
        rawAC = analogRead(PIN_ADC_AC);
        runFilter(filterRed, rawAC / 4095.0);
      }
      break;

    case 3: // DARK
      digitalWrite(PIN_RED_WIN, LOW);
      sampleCountDC++;
      break;
  }

  timingPhase = (timingPhase + 1) % 4;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// =============================================================
// ----------------------- SETUP -------------------------------
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("System Init...");

  pinMode(PIN_IR_WIN, OUTPUT);
  pinMode(PIN_RED_WIN, OUTPUT);
  digitalWrite(PIN_IR_WIN, LOW);
  digitalWrite(PIN_RED_WIN, LOW);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // --- PWM SETUP ---
  ledcAttach(PIN_IR_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(PIN_RED_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcWrite(PIN_IR_PWM, irDuty);
  ledcWrite(PIN_RED_PWM, redDuty);

  // --- TIMER SETUP ---
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, SLOT_PERIOD_US, true, 0);

  delay(1000);
  currentState = STATE_CALIBRATING;
  Serial.println("Starting Calibration...");
}

// =============================================================
// ----------------------- MAIN LOOP ---------------------------
// =============================================================

int adjustDuty(int currentDuty, int target, int actual) {
  int step = 1;
  // Increase step size if far from target
  if (abs(target - actual) > 500) step = 5; else step = 1;
  
  if (actual < (target - DC_TOLERANCE)) currentDuty += step; 
  else if (actual > (target + DC_TOLERANCE)) currentDuty -= step; 
  return constrain(currentDuty, PWM_MIN_DUTY, PWM_MAX_DUTY);
}

void loop() {
  static unsigned long lastDebugUpdate = 0;
  static unsigned long lastCalibUpdate = 0;

  // Process data every 100ms
  if (millis() - lastCalibUpdate > 100) {
    lastCalibUpdate = millis();
    if (sampleCountDC > 0) {
      portENTER_CRITICAL(&timerMux);
      avgDC_IR = sumDC_IR / sampleCountDC;
      avgDC_Red = sumDC_Red / sampleCountDC;
      sumDC_IR = 0; sumDC_Red = 0; sampleCountDC = 0;
      portEXIT_CRITICAL(&timerMux);
    }
  }

  switch (currentState) {
    case STATE_CALIBRATING:
      {
        int newIrDuty = adjustDuty(irDuty, DC_TARGET_VAL, avgDC_IR);
        if (newIrDuty != irDuty) {
          irDuty = newIrDuty;
          ledcWrite(PIN_IR_PWM, irDuty);
        }

        int newRedDuty = adjustDuty(redDuty, DC_TARGET_VAL, avgDC_Red);
        if (newRedDuty != redDuty) {
          redDuty = newRedDuty;
          ledcWrite(PIN_RED_PWM, redDuty);
        }

        // Check if both signals are close to 2.0V (Target 2480)
        // This ensures they are inside your 0.5V - 3.0V requirement.
        if (abs(DC_TARGET_VAL - avgDC_IR) < DC_TOLERANCE && 
            abs(DC_TARGET_VAL - avgDC_Red) < DC_TOLERANCE) {
          
          Serial.println("Calibration Complete!");
          portENTER_CRITICAL(&timerMux);
          filterIR.initialized = false;
          filterRed.initialized = false;
          portEXIT_CRITICAL(&timerMux);
          currentState = STATE_MEASURING;
        }
      }
      break;

    case STATE_MEASURING:
      // If signal goes out of the 0.5V-3.0V range (drift), recalibrate.
      // 2480 +/- 300 is roughly 1.7V to 2.3V safe zone.
      // If it hits rails (0.5V or 3.0V), definitely recalibrate.
      if (abs(DC_TARGET_VAL - avgDC_IR) > 400 || 
          abs(DC_TARGET_VAL - avgDC_Red) > 400) {
         Serial.println("Signal Drift. Recalibrating...");
         currentState = STATE_CALIBRATING;
      }
      break;
  }

  if (millis() - lastDebugUpdate > 500) {
    lastDebugUpdate = millis();
    if (currentState == STATE_CALIBRATING) {
      Serial.print("[CALIB] Target:"); Serial.print(DC_TARGET_VAL);
      Serial.print(" | IR DC:"); Serial.print(avgDC_IR);
      Serial.print(" | Red DC:"); Serial.println(avgDC_Red);
    } 
    else if (currentState == STATE_MEASURING) {
      // Plot the AC Heartbeat
      Serial.print("AC_Signal:");
      Serial.print(filterIR.filteredOutput * 1000); 
      Serial.print(",");
      Serial.println(filterRed.filteredOutput * 1000);
    }
  }
}