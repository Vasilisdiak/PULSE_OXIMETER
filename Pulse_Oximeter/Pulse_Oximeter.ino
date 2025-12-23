#include <Arduino.h>

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================

// --- LED OUTPUTS (4-Wire Setup) ---
#define PIN_IR_WIN     19 // IR Window Pulse
#define PIN_RED_WIN    5  // Red Window Pulse
#define PIN_IR_PWM     18 // IR 20kHz Carrier
#define PIN_RED_PWM    17 // Red 20kHz Carrier

// --- ADC INPUTS ---
#define PIN_ADC_DC     34 // Raw signal (Sph)
#define PIN_ADC_AC     35 // AC-coupled signal (Sr_ac)

// =============================================================
// ------------------- CONSTANTS & SETTINGS --------------------
// =============================================================

// --- Timing ---
#define SLOT_PERIOD_US  500  // Four 500us slots per frame

// --- PWM (LEDC) Settings ---
#define PWM_FREQ        20000
#define PWM_RES_BITS    8    // 8-bit resolution
// Note: In ESP32 Core 3.0+, we write to PINS, not Channels.

// --- Calibration Targets ---
#define DC_TARGET_VAL   2480 
#define DC_TOLERANCE    100  
#define PWM_START_DUTY  50   
#define PWM_MAX_DUTY    240  
#define PWM_MIN_DUTY    10   

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

volatile long sumDC_IR = 0;
volatile long sumDC_Red = 0;
volatile int sampleCountDC = 0;

int avgDC_IR = 0;
int avgDC_Red = 0;

struct FilterState {
  float x_hp_old=0, y_hp_old=0;
  float x1_lp=0, x2_lp=0, y1_lp=0, y2_lp=0;
  float filteredOutput=0;
};
volatile FilterState filterIR, filterRed;

// Hardware timer handle
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================
// -------------------- ISR & FILTER FUNCTIONS -----------------
// =============================================================

void runFilter(volatile FilterState &fs, float rawInput) {
  float y_hp = alpha_hp * (fs.y_hp_old + rawInput - fs.x_hp_old);
  fs.x_hp_old = rawInput;
  fs.y_hp_old = y_hp;

  float y_lp = a0_lp * y_hp + a1_lp * fs.x1_lp + a2_lp * fs.x2_lp - b1_lp * fs.y1_lp - b2_lp * fs.y2_lp;
  
  fs.x2_lp = fs.x1_lp; fs.x1_lp = y_hp;
  fs.y2_lp = fs.y1_lp; fs.y1_lp = y_lp;

  fs.filteredOutput = y_lp;
}

void IRAM_ATTR onTimerISR() {
  int rawDC, rawAC;

  portENTER_CRITICAL_ISR(&timerMux);

  switch (timingPhase) {
    case 0: // IR WINDOW
      digitalWrite(PIN_IR_WIN, HIGH);
      rawDC = analogRead(PIN_ADC_DC);
      sumDC_IR += rawDC;
      if (currentState == STATE_MEASURING) {
        rawAC = analogRead(PIN_ADC_AC);
        runFilter(filterIR, rawAC / 4095.0);
      }
      break;

    case 1: // DARK 1
      digitalWrite(PIN_IR_WIN, LOW);
      break;

    case 2: // RED WINDOW
      digitalWrite(PIN_RED_WIN, HIGH);
      rawDC = analogRead(PIN_ADC_DC);
      sumDC_Red += rawDC;
      if (currentState == STATE_MEASURING) {
        rawAC = analogRead(PIN_ADC_AC);
        runFilter(filterRed, rawAC / 4095.0);
      }
      break;

    case 3: // DARK 2
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

  // --- NEW PWM SETUP (ESP32 Core 3.0+) ---
  // ledcAttach(pin, frequency, resolution)
  ledcAttach(PIN_IR_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(PIN_RED_PWM, PWM_FREQ, PWM_RES_BITS);

  // Write initial duty
  ledcWrite(PIN_IR_PWM, irDuty);
  ledcWrite(PIN_RED_PWM, redDuty);

  // --- NEW TIMER SETUP (ESP32 Core 3.0+) ---
  // timerBegin(frequency_hz) -> Set to 1MHz (1 tick = 1us)
  timer = timerBegin(1000000); 
  
  // timerAttachInterrupt(timer, function)
  timerAttachInterrupt(timer, &onTimerISR);
  
  // timerAlarm(timer, alarm_val, autoreload, reload_count)
  // reload_count = 0 means infinite repeat
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
  if (abs(target - actual) > 500) step = 5; else step = 1;

  if (actual < (target - DC_TOLERANCE)) {
    currentDuty += step; 
  } else if (actual > (target + DC_TOLERANCE)) {
    currentDuty -= step; 
  }
  return constrain(currentDuty, PWM_MIN_DUTY, PWM_MAX_DUTY);
}

void loop() 
{
  static unsigned long lastDebugUpdate = 0;
  static unsigned long lastCalibUpdate = 0;

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

  switch (currentState) 
  {
    case STATE_CALIBRATING:
      {
        int newIrDuty = adjustDuty(irDuty, DC_TARGET_VAL, avgDC_IR);
        if (newIrDuty != irDuty) {
          irDuty = newIrDuty;
          // Core 3.0 uses ledcWrite(PIN, value)
          ledcWrite(PIN_IR_PWM, irDuty);
        }

        int newRedDuty = adjustDuty(redDuty, DC_TARGET_VAL, avgDC_Red);
        if (newRedDuty != redDuty) {
          redDuty = newRedDuty;
          // Core 3.0 uses ledcWrite(PIN, value)
          ledcWrite(PIN_RED_PWM, redDuty);
        }

        if (abs(DC_TARGET_VAL - avgDC_IR) < DC_TOLERANCE && 
            abs(DC_TARGET_VAL - avgDC_Red) < DC_TOLERANCE) {
          Serial.println("Calibration Complete!");
          currentState = STATE_MEASURING;
        }
      }
      break;

    case STATE_MEASURING:
      if (abs(DC_TARGET_VAL - avgDC_IR) > (DC_TOLERANCE * 3) || 
          abs(DC_TARGET_VAL - avgDC_Red) > (DC_TOLERANCE * 3)) {
         Serial.println("Drift Detected. Recalibrating...");
         currentState = STATE_CALIBRATING;
      }
      break;
  }

  if (millis() - lastDebugUpdate > 500) {
    lastDebugUpdate = millis();
    if (currentState == STATE_CALIBRATING) {
      Serial.print("[CALIB] IR DC:"); Serial.print(avgDC_IR);
      Serial.print(" PWM:"); Serial.print(irDuty);
      Serial.print(" | Red DC:"); Serial.print(avgDC_Red);
      Serial.print(" PWM:"); Serial.println(redDuty);
    } 
    else if (currentState == STATE_MEASURING) {
      Serial.print("Filter_Out:");
      Serial.print(filterIR.filteredOutput * 1000); 
      Serial.print(",");
      Serial.println(filterRed.filteredOutput * 1000);
    }
  }
}