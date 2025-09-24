/**
 * RailControl_Gemeni2_Optimized_Mega.ino
 * Arduino Mega 2560 - Linear Rail Controller
 * Features:
 * - Manual Run with Pause ($MANUAL_STOP -> $PAUSED) and Abort ($MANUAL_ABORT -> IDLE)
 * - Limited jog in Pause (Left->Start, Right->End), normal moves in IDLE to limits
 * - StopReason separation; Normal moves stop at limits (no auto-return)
 * - Multiplexer channel select (SET_RES_MUX:n) + OLED status (U8g2 SH1106 SW SPI)
 * - HW buttons: ORIGIN(44), STOP(45), LEFT(46), RIGHT(47)
 * - Removed Gain entirely
 */

enum Command { CMD_NONE = 0,
               CMD_CALIBRATE,
               CMD_STOP,
               CMD_SET_RES_MUX,
               CMD_SET_DAC_V,
               CMD_START_LEFT,
               CMD_START_RIGHT,
               CMD_START_LEFT_LIMITED,
               CMD_START_RIGHT_LIMITED,
               CMD_SPEED_FAST,
               CMD_SPEED_SMOOTH,
               CMD_SPEED_SLOW,
               CMD_MANUAL_RUN,
               CMD_MANUAL_STOP,
               CMD_MANUAL_ABORT,
               CMD_MANUAL_RESUME,
               CMD_REC_ON,
               CMD_REC_PAUSE,
               CMD_REC_OFF,
               CMD_UNKNOWN };
struct CmdArgs {
  long a = 0, b = 0, c = 0;
  float f = 0.f;
};

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1110.h>
#include <TimerOne.h>
#include <U8g2lib.h>
#include <string.h>
#include <stdlib.h>

// ---------------- Pins (Mega) ----------------
#define MOTOR_DIR 35
#define MOTOR_STEP 34
#define MOTOR_ENABLE 30
#define M0 31
#define M1 32
#define M2 33

#define LIMIT_SWITCH_DESTINATION 2
#define LIMIT_SWITCH_START 3

#define BUZZER 38

// MUX CD74HC4067
#define MUX_S0_PIN 9
#define MUX_S1_PIN 12
#define MUX_S2_PIN 7
#define MUX_S3_PIN 6
#define MUX_EN_PIN 8  // (Active-Low)



// HW Buttons (active LOW, INPUT_PULLUP)
#define BTN_ORIGIN 44
#define BTN_STOP 45
#define BTN_LEFT 46
#define BTN_RIGHT 47

// OLED SH1106 HW SPI (U8g2)
// Using U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI constructor with hardware SPI
#define OLED_DC 10
#define OLED_CS 53
#define OLED_RESET 11

U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);
// ===== OLED Fonts (fallback-safe) =====
// Bold (recommended safe): ncenB14
#ifndef OLED_FONT_BOLD
#define OLED_FONT_BOLD u8g2_font_ncenB14_tr
#endif
// "Italic" fallback: اگر italic ندارید، Regular 12 را جایگزین کنید
#ifndef OLED_FONT_ITALIC
#define OLED_FONT_ITALIC u8g2_font_ncenR12_tr
#endif
// Small regular already used: u8g2_font_6x10_mf

// --------------- Constants -------------------
#define SERIAL_BAUD_RATE 250000
#define CMD_BUFFER_SIZE 128
#define CMD_TERMINATOR '\n'
#define MCP4725_ADDRESS 0x60
#define MCP4725_VREF 4.096

#define FULL_STEP_DELAY_US 1100
#define HALF_STEP_DELAY_US 800
#define QUARTER_STEP_DELAY_US 700

#define STEPS_PER_MM 100
#define DATA_POINTS_PER_MM 10

#define BUZZER_FREQ 1200
#define BUZZER_DURATION 100
#define BUZZER_INTERVAL 50

#ifndef ADC_READ_INTERVAL_MS
#define ADC_READ_INTERVAL_MS 5UL
#endif
const byte DEV_ADDR = 0x48;
ADS1110 ads1110(DEV_ADDR);

// زمان‌های پیشنهادی:
#define MUX_ADDR_SETTLE_US 10  // زمان تثبیت خطوط آدرس (افزایش یافت)
#define MUX_POST_EN_US 200     // زمان بیشتر بعد از وصل EN تا سیگنال کاملاً روی ADC بنشیند
#define MUX_MUTE_MS 50UL       // Gate نرم‌افزاری طولانی‌تر برای حذف کامل جهش‌ها

#define DAC_MUTE_MS 20UL

volatile unsigned long muxMuteUntilMs = 0;
volatile unsigned long dacMuteUntilMs = 0;

#define DEBUG 0
#if DEBUG
#define LOG(msg) Serial.println(F(msg))
#define LOGV(x) Serial.println(x)
#else
#define LOG(msg) \
  do { \
  } while (0)
#define LOGV(x) \
  do { \
  } while (0)
#endif

// --------------- Globals ---------------------
volatile long currentPosition = 0;
volatile long totalCalibrationSteps = 0;
volatile bool isCalibrated = false;
volatile bool motorEnabled = false;
volatile bool limitDestHit = false;
volatile bool limitStartHit = false;

volatile bool directionRight = true;

// Stop reasons: 0 none, 1 pause, 2 abort, 3 general stop
volatile uint8_t stopReason = 0;
// Limited jog flag (when Pause-jog active)
volatile bool limitedJogActive = false;

enum MoveState { IDLE,
                 MOVING_LEFT,
                 MOVING_RIGHT,
                 MANUAL_PAUSED,
                 MANUAL_MOVING_TO_END,
                 MANUAL_MOVING_TO_START };
MoveState currentMoveState = IDLE;

long microsteps_moved_since_last_datapoint = 0;
long microsteps_for_one_data_point = 0;
int currentMicrostepFactor = 1;
unsigned long currentStepDelay = FULL_STEP_DELAY_US;

// Manual Run
long manualTargetPosition = 0;
long manualStartPosition_us = 0;
long manualEndPosition_us = 0;
int manualTotalCycles = 0;
int manualCompletedCycles = 0;
bool manualGoingToEnd = true;
volatile bool joggedWhilePaused = false;

// Peripherals
float currentDacVoltage = 1.0;

// ADC
unsigned long lastAdcReadTime = 0;
float lastAdcVoltage = 0.0f;  //  for OLED

// Serial buffer
char command_buffer[CMD_BUFFER_SIZE];
byte command_index = 0;

// Objects
Adafruit_MCP4725 dac;
volatile bool dataPointReadyToSend = false;

// MUX channel tracking for OLED
int currentMuxChannel = 0;

// Origin for ΔX
volatile long originPosition_us = 0;
volatile bool originSet = false;
volatile bool recActive = false;
volatile bool recPaused = false;

// Resistor labels for OLED (kΩ)
// ---- Optional: define resistor labels here if not defined elsewhere ----
#ifndef OLED_RES_LABELS_ARRAY
#define OLED_RES_LABELS_ARRAY
const char* RES_LABELS[16] = { "3.9", "5.6", "6.8", "10", "15", "22", "27", "33", "39", "47", "56", "68", "82", "100", "120", "150" };
#endif
static const float RES_VALUES[16] = { 3.9, 5.6, 6.8, 10, 15, 22, 27, 33, 39, 47, 56, 68, 82, 100, 120, 150 };  // kΩ
// Buttons debounce
struct BtnDeb {
  uint8_t pin;
  bool lastStable;
  bool lastRead;
  unsigned long lastChangeMs;
};
BtnDeb btns[4] = {
  { BTN_ORIGIN, false, false, 0 },
  { BTN_STOP, false, false, 0 },
  { BTN_LEFT, false, false, 0 },
  { BTN_RIGHT, false, false, 0 }
};
const unsigned long DEBOUNCE_MS = 25;

// ------------ Fast STEP macros (Mega) ----------
#if defined(__AVR_ATmega2560__)
#define STEP_PORT PORTC
#define STEP_DDR DDRC
#define STEP_BIT 3  // D34 -> PC3
#define STEP_HIGH() (STEP_PORT |= _BV(STEP_BIT))
#define STEP_LOW() (STEP_PORT &= ~_BV(STEP_BIT))
#else
#define STEP_HIGH() digitalWrite(MOTOR_STEP, HIGH)
#define STEP_LOW() digitalWrite(MOTOR_STEP, LOW)
#endif

// ------------ Atomic helpers -------------------
inline long atomicReadLong(volatile long& v) {
  noInterrupts();
  long x = v;
  interrupts();
  return x;
}
inline void atomicWriteLong(volatile long& v, long x) {
  noInterrupts();
  v = x;
  interrupts();
}

// ------------ Buzzer (non-blocking) ------------
enum BuzzerState { BZ_IDLE,
                   BZ_ON,
                   BZ_WAIT };
BuzzerState buzzerState = BZ_IDLE;
uint8_t beepRemaining = 0;
unsigned long buzzerTs = 0;
void serviceBuzzer() {
  if (beepRemaining == 0 && buzzerState == BZ_IDLE) return;
  unsigned long now = millis();
  switch (buzzerState) {
    case BZ_IDLE:
      if (beepRemaining > 0) {
        tone(BUZZER, BUZZER_FREQ, BUZZER_DURATION);
        buzzerTs = now;
        buzzerState = BZ_ON;
      }
      break;
    case BZ_ON:
      if (now - buzzerTs >= BUZZER_DURATION) {
        buzzerTs = now;
        buzzerState = BZ_WAIT;
      }
      break;
    case BZ_WAIT:
      if (now - buzzerTs >= BUZZER_INTERVAL) {
        beepRemaining--;
        if (beepRemaining > 0) {
          tone(BUZZER, BUZZER_FREQ, BUZZER_DURATION);
          buzzerTs = now;
          buzzerState = BZ_ON;
        } else {
          buzzerState = BZ_IDLE;
          noTone(BUZZER);
        }
      }
      break;
  }
}
void beepBuzzer(int count = 1) {
  if (count > 0) beepRemaining += count;
}

// ------------ Converters / OLED helpers --------
inline float us_to_mm(long us) {
  return (us / (float)currentMicrostepFactor) / (float)STEPS_PER_MM;
}
inline float us_to_cm(long us) {
  return us_to_mm(us) / 10.0f;
}

// Snap to nearest E12 nominal within ±5% and round to 2 decimals (kΩ)
static float snap_to_E12(float r) {
  if (r <= 0.0f) return 0.0f;
  const float base[12] = { 1.0f, 1.2f, 1.5f, 1.8f, 2.2f, 2.7f, 3.3f, 3.9f, 4.7f, 5.6f, 6.8f, 8.2f };
  // compute exponent ~ floor(log10(r)) without using log10
  float rr = r;
  int exp = 0;
  while (rr >= 10.0f) {
    rr /= 10.0f;
    exp++;
  }
  while (rr < 1.0f) {
    rr *= 10.0f;
    exp--;
  }
  float nearest = r;
  float best_err = 1e9;
  for (int d = exp - 1; d <= exp + 1; ++d) {
    float scale = 1.0f;
    if (d > 0) {
      for (int i = 0; i < d; i++) scale *= 10.0f;
    } else if (d < 0) {
      for (int i = 0; i < -d; i++) scale /= 10.0f;
    }
    for (int i = 0; i < 12; i++) {
      float cand = base[i] * scale;
      float err = fabs(cand - r);
      if (err < best_err) {
        best_err = err;
        nearest = cand;
      }
    }
  }
  if (nearest > 0.0f && fabs(r - nearest) <= 0.05f * nearest) {
    return roundf(nearest * 100.0f) / 100.0f;
  }
  return roundf(r * 100.0f) / 100.0f;
}
// ---- OLED init (call once in setup) ----
void oledInit() {
  // Initialize OLED once via this function
  u8g2.begin();
  // optional: contrast/flip/mode, etc.
  // u8g2.setContrast(200);
  u8g2.setDisplayRotation(U8G2_R2);  // 180 درجه
}
// ---- OLED draw main screen (call periodically in loop; paging API) ----
void drawOLED() {
  static unsigned long lastDraw = 0;
  unsigned long now = millis();
  if (now - lastDraw < 100) return;  // ~10 FPS
  lastDraw = now;

  // داده‌ها
  /* --- Status icon selection --- */
  uint8_t icon_idx = 0;                                 // 0=STOP,1=PLAY_R,2=PLAY_L,3=PAUSE
  if (currentMoveState == MANUAL_PAUSED) icon_idx = 3;  // Pause
  else if (currentMoveState == IDLE) icon_idx = 0;      // Stop
  else if (directionRight) icon_idx = 1;                // Play ▶
  else icon_idx = 2;                                    // Play ◀

  float curr_cm = us_to_cm(atomicReadLong(currentPosition));
  float start_cm = isCalibrated ? us_to_cm(manualStartPosition_us) : 0.0f;
  float end_cm = 0.0f;
  if (isCalibrated) {
    if (manualEndPosition_us > 0 && manualEndPosition_us <= totalCalibrationSteps * currentMicrostepFactor) {
      end_cm = us_to_cm(manualEndPosition_us);
    } else {
      end_cm = (totalCalibrationSteps / (float)STEPS_PER_MM) / 10.0f;
    }
  }
  // posbuf با dtostrf تا ? نیاید
  char s_start[8], s_curr[8], s_end[8], posbuf[28];
  dtostrf(start_cm, 5, 2, s_start);
  dtostrf(curr_cm, 5, 2, s_curr);
  dtostrf(end_cm, 5, 2, s_end);
  snprintf(posbuf, sizeof(posbuf), "%s:%s:%s", s_start, s_curr, s_end);

  // مقاومت Rt = (Vt * Rm) / (Vi - Vt)
  float Vi = currentDacVoltage;
  if (Vi < 0.05f) Vi = 0.05f;
  float Rm_k = RES_VALUES[currentMuxChannel & 0x0F];  // kΩ
  float Rt_k = 0.0f;
  bool Rt_valid = (Vi - lastAdcVoltage) > 0.001f && (Rm_k > 0.0f);
  if (Rt_valid) Rt_k = (lastAdcVoltage * Rm_k) / (Vi - lastAdcVoltage);
  // اسنپ به سری E12 با تلورانس ±5% و رُند به دو رقم اعشار
  float Rt_disp = Rt_k;
  if (Rt_valid) {
    Rt_disp = snap_to_E12(Rt_k);
  }

  // dX (mm) اگر Origin تنظیم شده
  bool show_dx = originSet;
  float dx_mm = 0.0f;
  if (show_dx) dx_mm = fabs(us_to_mm(atomicReadLong(currentPosition) - originPosition_us));

  // صفحه‌ای
  u8g2.firstPage();
  do {
    // خط 1: وضعیت و پوزیشن‌ها
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.setCursor(0, 10);
    // Draw crisp 12x12 media icon at (0,2)
    int ix = 0, iy = 2;
    switch (icon_idx) {
      case 0:  // STOP
        u8g2.drawBox(ix + 2, iy + 2, 8, 8);
        break;
      case 1:  // PLAY ▶
        u8g2.drawTriangle(ix + 2, iy + 2, ix + 2, iy + 10, ix + 10, iy + 6);
        // thicken outline
        u8g2.drawTriangle(ix + 3, iy + 2, ix + 3, iy + 10, ix + 11, iy + 6);
        break;
      case 2:  // PLAY ◀
        u8g2.drawTriangle(ix + 10, iy + 2, ix + 10, iy + 10, ix + 2, iy + 6);
        u8g2.drawTriangle(ix + 9, iy + 2, ix + 9, iy + 10, ix + 1, iy + 6);
        break;
      case 3:  // PAUSE
        u8g2.drawBox(ix + 2, iy + 2, 3, 8);
        u8g2.drawBox(ix + 7, iy + 2, 3, 8);
        break;
    }
    int w = u8g2.getStrWidth(posbuf);
    int x = 128 - w;
    if (x < 0) x = 0;
    u8g2.setCursor(x, 10);
    u8g2.print(posbuf);

    // --- R: نمایش مقاومت با 2 رقم اعشار و KΩ ---
    u8g2.setFont(OLED_FONT_BOLD);
    char resLine[28];
    if (Rt_valid) {
      char rtbuf[12];
      dtostrf(Rt_disp, 6, 2, rtbuf);
      // نمایش مقدار مقاومت با پسوند K (نماد Ω با بیتی‌مپ جداگانه رسم می‌شود)
      snprintf(resLine, sizeof(resLine), "R: %s K", rtbuf);
    } else {
      snprintf(resLine, sizeof(resLine), "R: --");
    }
    int wres = u8g2.getStrWidth(resLine);
    u8g2.setCursor((128 - wres) / 2, 36);
    u8g2.print(resLine);
    // رسم نماد Ω 8x8 پس از حرف K
    if (Rt_valid) {
      static const uint8_t omega8x8[] PROGMEM = {
        0x3C,  // 00111100
        0x42,  // 01000010
        0x81,  // 10000001
        0x81,  // 10000001
        0x42,  // 01000010 (narrowing bottom of arc)
        0x66,  // 01100110 (legs inside)
        0x42,  // 01000010 (vertical legs)
        0xC3   // 11000011 (feet)
      };
      int xGlyph = u8g2.getCursorX();
      int yGlyph = 29;  // پایه متنی برای 8x8 در خط دوم
      u8g2.drawXBMP(xGlyph, yGlyph, 8, 8, omega8x8);
    }

    // --- dX (centered, "italic" fallback) ---
    if (show_dx) {
      u8g2.setFont(OLED_FONT_ITALIC);
      char dxb[22];
      char dxbuf[10];
      dtostrf(dx_mm, 6, 2, dxbuf);
      snprintf(dxb, sizeof(dxb), "dX = %s mm", dxbuf);
      int wdx = u8g2.getStrWidth(dxb);
      u8g2.setCursor((128 - wdx) / 2, 58);
      u8g2.print(dxb);
    }
    if (recActive) {
      int x_dot = 118, y_dot = 56;
      if (!recPaused) {
        u8g2.drawDisc(x_dot, y_dot, 3);  // ● ضبط در حال اجرا
      } else {
        u8g2.drawCircle(x_dot, y_dot, 3);  // ○ ضبط pause
        // یا: u8g2.setFont(u8g2_font_6x10_mf); u8g2.setCursor(110, 64); u8g2.print("P");
      }
    }
  } while (u8g2.nextPage());
}


// ---- OLED draw "Calibrating..." (call during calibration) ----
void drawCalibratingScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13_mf);
    u8g2.setCursor(10, 36);
    u8g2.print("Calibrating ...");
  } while (u8g2.nextPage());
}

// ---- Optional alias for older calls ----
inline void drawOled() {
  drawOLED();
}

// ------------ Forwards --------------------------
void setMuxChannel(int channel);
void stopMovement(bool triggeredByLimit, bool silent = false);
void calibrateMotor();
void startOrResumeManualMove(bool resuming);
void resetManualState();
void generateStepPulseISR();
void readAdcAndSendData(bool forDataPoint);
void processCommand();
void pollButtons();
void handleButtonPress(uint8_t idx);

// ------------ ISRs ------------------------------
void isrLimitDestination() {
  if (digitalRead(LIMIT_SWITCH_DESTINATION) == LOW) limitDestHit = true;
}
void isrLimitStart() {
  if (digitalRead(LIMIT_SWITCH_START) == LOW) limitStartHit = true;
}
void generateStepPulseISR() {
  STEP_HIGH();
  delayMicroseconds(2);
  STEP_LOW();
  currentPosition += directionRight ? 1 : -1;
  if (currentMoveState == MANUAL_MOVING_TO_END || currentMoveState == MANUAL_MOVING_TO_START) {
    microsteps_moved_since_last_datapoint++;
    if (microsteps_moved_since_last_datapoint >= microsteps_for_one_data_point) {
      dataPointReadyToSend = true;
      microsteps_moved_since_last_datapoint = 0;
    }
  }
}

// ------------ Helpers ---------------------------
void enableMotor(bool enable) {
  digitalWrite(MOTOR_ENABLE, enable ? LOW : HIGH);
  motorEnabled = enable;
  delay(2);
}
void setDirection(bool goRight) {
  directionRight = goRight;
  digitalWrite(MOTOR_DIR, goRight ? LOW : HIGH);
  delayMicroseconds(50);
}
/*
 A4988 microstepping mapping (MS1, MS2, MS3 -> factor):
   LOW,  LOW,  LOW  => Full (1)
   HIGH, LOW,  LOW  => Half (2)
   LOW,  HIGH, LOW  => Quarter (4)
   HIGH, HIGH, LOW  => Eighth (8) [unused]
   HIGH, HIGH, HIGH => Sixteenth (16) [unused]
 Pins M0/M1/M2 are wired to MS1/MS2/MS3 respectively.
*/
void setMicrostepMode(int m0, int m1, int m2, int new_factor, unsigned long new_step_delay_us) {
  if (currentMicrostepFactor == new_factor) {
    LOG("Info: Speed unchanged");
    return;
  }
  if (currentMoveState != IDLE && currentMoveState != MANUAL_PAUSED) {
    LOG("Warning: stop first");
    stopMovement(false, true);
  }
  bool was_paused = (currentMoveState == MANUAL_PAUSED);
  int old_factor = currentMicrostepFactor;
  Timer1.stop();
  enableMotor(false);
  delay(5);
  digitalWrite(M0, m0);
  digitalWrite(M1, m1);
  digitalWrite(M2, m2);
  if (old_factor > 0) {
    long pos = atomicReadLong(currentPosition);
    atomicWriteLong(currentPosition, (long)((double)pos / old_factor * new_factor));
    if (was_paused) {
      manualStartPosition_us = (long)((double)manualStartPosition_us / old_factor * new_factor);
      manualEndPosition_us = (long)((double)manualEndPosition_us / old_factor * new_factor);
      manualTargetPosition = (long)((double)manualTargetPosition / old_factor * new_factor);
    }
  }
  currentMicrostepFactor = new_factor;
  currentStepDelay = new_step_delay_us;
  microsteps_for_one_data_point = (long)(STEPS_PER_MM / DATA_POINTS_PER_MM) * currentMicrostepFactor;
  if (microsteps_for_one_data_point < 1) microsteps_for_one_data_point = 1;
  Serial.print(F("POSITION:"));
  Serial.println(atomicReadLong(currentPosition));
  if (was_paused) currentMoveState = MANUAL_PAUSED;
}
void stopMovement(bool triggeredByLimit, bool silent) {
  Timer1.stop();
  enableMotor(false);
  uint8_t sr = stopReason;
  stopReason = 0;
  if (currentMoveState != IDLE) {
    MoveState previousState = currentMoveState;
    bool wasManualMove = (previousState == MANUAL_MOVING_TO_END || previousState == MANUAL_MOVING_TO_START);
    bool wasJog = (previousState == MOVING_LEFT || previousState == MOVING_RIGHT) && (manualTargetPosition != 0);

    if (triggeredByLimit) {
      if (previousState == MOVING_LEFT || previousState == MANUAL_MOVING_TO_START) {
        atomicWriteLong(currentPosition, 0);
        if (!silent) beepBuzzer(1);
        if (previousState == MANUAL_MOVING_TO_START) {
          manualCompletedCycles++;
          Serial.print(F("CYCLE_DONE:"));
          Serial.println(manualCompletedCycles);
          if (manualCompletedCycles >= manualTotalCycles) {
            Serial.println(F("All manual cycles completed."));
            currentMoveState = IDLE;
            Serial.print(F("POSITION:"));
            Serial.println(currentPosition);
            beepBuzzer(3);
            return;
          } else {
            manualTargetPosition = manualEndPosition_us;
            manualGoingToEnd = true;
            setDirection(true);
            enableMotor(true);
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
            currentMoveState = MANUAL_MOVING_TO_END;
            Serial.print(F("Next cycle: "));
            Serial.println(manualCompletedCycles + 1);
            Serial.print(F("POSITION:"));
            Serial.println(currentPosition);
            return;
          }
        } else {
          currentMoveState = IDLE;
          Serial.print(F("POSITION:"));
          Serial.println(currentPosition);
          return;
        }
      }
      if (previousState == MOVING_RIGHT || previousState == MANUAL_MOVING_TO_END) {
        atomicWriteLong(currentPosition, (long)totalCalibrationSteps * currentMicrostepFactor);
        if (!silent) beepBuzzer(1);
        if (previousState == MANUAL_MOVING_TO_END) {
          manualTargetPosition = manualStartPosition_us;
          manualGoingToEnd = false;
          setDirection(false);
          enableMotor(true);
          Timer1.setPeriod(currentStepDelay);
          Timer1.start();
          currentMoveState = MANUAL_MOVING_TO_START;
          Serial.println(F("Manual target leg reached (by limit)."));
          Serial.print(F("POSITION:"));
          Serial.println(currentPosition);
          return;
        } else {
          currentMoveState = IDLE;
          Serial.print(F("POSITION:"));
          Serial.println(currentPosition);
          return;
        }
      }
    }

    if (wasManualMove) {
      if (sr == 1) {
        currentMoveState = MANUAL_PAUSED;
        if (!silent) {
          Serial.println(F("Manual run paused."));
          Serial.println(F("$PAUSED"));
        }
      } else if (sr == 2) {
        currentMoveState = IDLE;
        manualTargetPosition = 0;
        microsteps_moved_since_last_datapoint = 0;
        if (!silent) Serial.println(F("Manual run aborted."));
      } else {
        currentMoveState = IDLE;
        if (!silent) Serial.println(F("Movement stopped."));
      }
    } else {
      currentMoveState = IDLE;
      if (!silent) {
        if (wasJog) Serial.println(F("Jog stopped."));
        else Serial.println(F("Movement stopped."));
      }
    }
    Serial.print(F("POSITION:"));
    Serial.println(currentPosition);
  }
  limitStartHit = false;
  limitDestHit = false;
  limitedJogActive = false;
}
void resetManualState() {
  manualTargetPosition = 0;
  manualStartPosition_us = 0;
  manualEndPosition_us = 0;
  manualTotalCycles = 0;
  manualCompletedCycles = 0;
  manualGoingToEnd = true;
  microsteps_moved_since_last_datapoint = 0;
  limitedJogActive = false;
  LOG("Manual state reset");
}
void startOrResumeManualMove(bool resuming) {
  if (!isCalibrated) {
    Serial.println(F("Error: Not calibrated."));
    beepBuzzer(1);
    return;
  }
  if (resuming) {
    if (currentMoveState != MANUAL_PAUSED) {
      Serial.println(F("Error: Cannot resume, not paused."));
      return;
    }
    if (joggedWhilePaused) {
      long pos = atomicReadLong(currentPosition);
      if (pos == manualStartPosition_us) {
        manualTargetPosition = manualEndPosition_us;
        manualGoingToEnd = true;
        setDirection(true);
      } else if (pos == manualEndPosition_us) {
        manualTargetPosition = manualStartPosition_us;
        manualGoingToEnd = false;
        setDirection(false);
      } else {
        setDirection(pos < manualTargetPosition);
      }
      joggedWhilePaused = false;
    } else {
      setDirection(atomicReadLong(currentPosition) < manualTargetPosition);
    }
    currentMoveState = manualGoingToEnd ? MANUAL_MOVING_TO_END : MANUAL_MOVING_TO_START;
  } else {
    microsteps_moved_since_last_datapoint = 0;
    manualCompletedCycles = 0;
    manualTargetPosition = manualEndPosition_us;
    manualGoingToEnd = true;
    setDirection(atomicReadLong(currentPosition) < manualTargetPosition);
    currentMoveState = MANUAL_MOVING_TO_END;
  }
  if (atomicReadLong(currentPosition) != manualTargetPosition) {
    enableMotor(true);
    Timer1.setPeriod(currentStepDelay);
    Timer1.start();
  } else {
    LOG("Already at target. Loop will advance.");
  }
}
void setMuxChannel(int channel) {
  channel = constrain(channel, 0, 15);

  // اگر کانال تغییر نکرده، کاری نکن
  if (channel == currentMuxChannel) {
    return;
  }

  // جدا کردن مالتی‌پلکسر از ADC
  digitalWrite(MUX_EN_PIN, HIGH);  // قطع (Active-Low)
  delayMicroseconds(5);            // کمی صبر برای قطع کامل

  // Apply bits: S0=LSB ... S3=MSB
  digitalWrite(MUX_S0_PIN, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1_PIN, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2_PIN, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3_PIN, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(MUX_ADDR_SETTLE_US);

  digitalWrite(MUX_EN_PIN, LOW);  // وصل (Active-Low)
  delayMicroseconds(MUX_POST_EN_US);

  currentMuxChannel = channel;
  muxMuteUntilMs = millis() + MUX_MUTE_MS;  // Gate طولانی‌تر

  // ارسال اطلاعات کانال جدید
  Serial.print(F("MUX_PINS:"));
  Serial.print((channel & 0x01) ? 1 : 0);
  Serial.print(F(","));
  Serial.print((channel & 0x02) ? 1 : 0);
  Serial.print(F(","));
  Serial.print((channel & 0x04) ? 1 : 0);
  Serial.print(F(","));
  Serial.println((channel & 0x08) ? 1 : 0);
}
void readAdcAndSendData(bool forDataPoint) {
  unsigned long now = millis();
  bool mute = (now < muxMuteUntilMs) || (now < dacMuteUntilMs);

  if (mute && lastAdcVoltage > 0.0f) {
    // ارسال مقدار قبلی (Hold) یا Skip؛ پیشنهاد: Hold تا نمودار پیوسته بماند
    float voltage = lastAdcVoltage;
    if (forDataPoint) {
      Serial.print(F("D:"));
      Serial.print(atomicReadLong(currentPosition));
      Serial.print(F(":"));
      Serial.println(voltage, 3);
    } else {
      Serial.print(F("ADC:"));
      Serial.println(voltage, 3);
    }
    return;
  }

  int mVolts = ads1110.getVolt();
  float voltage = mVolts / 1000.0f;
  lastAdcVoltage = voltage;

  if (forDataPoint) {
    Serial.print(F("D:"));
    Serial.print(atomicReadLong(currentPosition));
    Serial.print(F(":"));
    Serial.println(voltage, 3);
  } else {
    Serial.print(F("ADC:"));
    Serial.println(voltage, 3);
  }
}

// ------------ Buttons (polling + debounce) ----
void handleButtonPress(uint8_t idx) {
  switch (idx) {
    case 0:  // ORIGIN
      originPosition_us = atomicReadLong(currentPosition);
      originSet = true;
      beepBuzzer(1);
      Serial.println(F("HW_BTN:ORIGIN"));
      break;

    case 1:  // STOP -> Abort everything, go to IDLE
      Serial.println(F("HW_BTN:STOP"));
      Timer1.stop();
      enableMotor(false);
      stopReason = 2;  // Abort
      stopMovement(false);
      limitedJogActive = false;
      break;

    case 2:  // LEFT
      Serial.println(F("HW_BTN:LEFT"));
      if (isCalibrated) {
        if (currentMoveState == MANUAL_PAUSED) {
          long target_us = manualStartPosition_us;
          if (atomicReadLong(currentPosition) > target_us && digitalRead(LIMIT_SWITCH_START) != LOW) {
            limitedJogActive = true;
            setDirection(false);
            enableMotor(true);
            currentMoveState = MOVING_LEFT;
            manualTargetPosition = target_us;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else {
            beepBuzzer(1);
          }
        } else if (currentMoveState == IDLE) {
          if (digitalRead(LIMIT_SWITCH_START) != LOW) {
            limitedJogActive = false;
            manualTargetPosition = 0;
            setDirection(false);
            enableMotor(true);
            currentMoveState = MOVING_LEFT;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else {
            beepBuzzer(1);
          }
        }
      }
      break;
    case 3:  // RIGHT
      Serial.println(F("HW_BTN:RIGHT"));
      if (isCalibrated) {
        if (currentMoveState == MANUAL_PAUSED) {
          long target_us = manualEndPosition_us;
          if (atomicReadLong(currentPosition) < target_us && digitalRead(LIMIT_SWITCH_DESTINATION) != LOW) {
            limitedJogActive = true;
            setDirection(true);
            enableMotor(true);
            currentMoveState = MOVING_RIGHT;
            manualTargetPosition = target_us;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else {
            beepBuzzer(1);
          }
        } else if (currentMoveState == IDLE) {
          if (digitalRead(LIMIT_SWITCH_DESTINATION) != LOW) {
            limitedJogActive = false;
            manualTargetPosition = (long)totalCalibrationSteps * currentMicrostepFactor;
            setDirection(true);
            enableMotor(true);
            currentMoveState = MOVING_RIGHT;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else {
            beepBuzzer(1);
          }
        }
      }
      break;
  }
}
void pollButtons() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    bool r = (digitalRead(btns[i].pin) == LOW);  // active LOW
    if (r != btns[i].lastRead) {
      btns[i].lastRead = r;
      btns[i].lastChangeMs = now;
    }
    if ((now - btns[i].lastChangeMs) > DEBOUNCE_MS) {
      if (r != btns[i].lastStable) {
        bool prev = btns[i].lastStable;
        btns[i].lastStable = r;
        if (r == true) {
          // pressed
          handleButtonPress(i);
        } else {
          // released
          if (i == 2 || i == 3) {  // LEFT or RIGHT
            Timer1.stop();
            enableMotor(false);
            if (currentMoveState == MANUAL_MOVING_TO_START || currentMoveState == MANUAL_MOVING_TO_END || currentMoveState == MOVING_LEFT || currentMoveState == MOVING_RIGHT) {
              currentMoveState = MANUAL_PAUSED;
              manualTargetPosition = 0;
              joggedWhilePaused = true;
              limitedJogActive = false;
              Serial.println(F("Manual run paused."));
              Serial.println(F("$PAUSED"));
              Serial.print(F("POSITION:"));
              Serial.println(atomicReadLong(currentPosition));
            }
          }
        }
      }
    }
  }
}


// -------------------- setup --------------------
static inline char* nextToken(char* p) {
  char* q = strchr(p, ':');
  return q ? (q + 1) : NULL;
}
static Command parseCommandId(char* buf, CmdArgs& out) {
  if (!buf || !buf[0]) return CMD_NONE;
  if (buf[0] == '$') {
    if (!strcmp(buf, "$CALIBRATE")) return CMD_CALIBRATE;
    if (!strcmp(buf, "$STOP")) return CMD_STOP;
    if (!strcmp(buf, "$START_LEFT")) return CMD_START_LEFT;
    if (!strcmp(buf, "$START_RIGHT")) return CMD_START_RIGHT;
    if (!strcmp(buf, "$SPEED_FAST")) return CMD_SPEED_FAST;
    if (!strcmp(buf, "$SPEED_SMOOTH")) return CMD_SPEED_SMOOTH;
    if (!strcmp(buf, "$SPEED_SLOW")) return CMD_SPEED_SLOW;
    if (!strcmp(buf, "$MANUAL_STOP")) return CMD_MANUAL_STOP;
    if (!strcmp(buf, "$MANUAL_ABORT")) return CMD_MANUAL_ABORT;
    if (!strcmp(buf, "$MANUAL_RESUME")) return CMD_MANUAL_RESUME;
    if (!strcmp(buf, "$REC_ON")) return CMD_REC_ON;
    if (!strcmp(buf, "$REC_PAUSE")) return CMD_REC_PAUSE;
    if (!strcmp(buf, "$REC_OFF")) return CMD_REC_OFF;
    if (!strncmp(buf, "$START_LEFT_LIMITED:", 20)) {
      out.a = atol(buf + 20);
      return CMD_START_LEFT_LIMITED;
    }
    if (!strncmp(buf, "$START_RIGHT_LIMITED:", 21)) {
      out.a = atol(buf + 21);
      return CMD_START_RIGHT_LIMITED;
    }
    if (!strncmp(buf, "$MANUAL_RUN:", 12)) {
      char* p = buf + 12;
      char* s1 = p;
      char* s2 = nextToken(s1);
      char* s3 = s2 ? nextToken(s2) : NULL;
      if (s1 && s2 && s3) {
        out.a = atol(s1);
        out.b = atol(s2);
        out.c = atol(s3);
        return CMD_MANUAL_RUN;
      }
      return CMD_UNKNOWN;
    }
    return CMD_UNKNOWN;
  } else {
    if (!strncmp(buf, "SET_RES_MUX:", 12)) {
      out.a = atoi(buf + 12);
      return CMD_SET_RES_MUX;
    }
    if (!strncmp(buf, "SET_DAC_V:", 10)) {
      out.f = atof(buf + 10);
      return CMD_SET_DAC_V;
    }
    return CMD_UNKNOWN;
  }
}
void processCommand() {
  CmdArgs args;
  Command cmd = parseCommandId(command_buffer, args);
  switch (cmd) {
    case CMD_CALIBRATE:
      Serial.println(F("-> CALIBRATE request"));
      calibrateMotor();
      break;
    case CMD_STOP:
      {
        Serial.println(F("-> General STOP request"));
        if (currentMoveState == MOVING_LEFT || currentMoveState == MOVING_RIGHT) stopMovement(false);
        else if (currentMoveState == MANUAL_MOVING_TO_START || currentMoveState == MANUAL_MOVING_TO_END) Serial.println(F("Info: For Manual, use $MANUAL_STOP or $MANUAL_ABORT. Ignoring $STOP."));
        else Serial.println(F("Info: Already stopped."));
      }
      break;
    case CMD_SET_RES_MUX:
      {
        int ch = (int)args.a;
        setMuxChannel(ch);
      }
      break;

    case CMD_SET_DAC_V:
      {
        float voltage = constrain(args.f, 0.1f, (float)MCP4725_VREF);
        Serial.print(F("-> SET_DAC_V to: "));
        Serial.println(voltage, 1);
        uint16_t dacValue = (uint16_t)((voltage / MCP4725_VREF) * 4095.0f);
        dacValue = constrain(dacValue, 0, 4095);
        dac.setVoltage(dacValue, false);
        currentDacVoltage = voltage;
        dacMuteUntilMs = millis() + DAC_MUTE_MS;  // Gate کوتاه بعد از تغییر DAC
        Serial.print(F("DAC set (Raw: "));
        Serial.print(dacValue);
        Serial.println(F(")"));
      }
      break;
    case CMD_START_LEFT:
      {
        if (!isCalibrated) {
          Serial.println(F("!! Error: Not calibrated."));
          break;
        }
        if (currentMoveState == IDLE && digitalRead(LIMIT_SWITCH_START) != LOW) {
          Serial.println(F("-> START_LEFT (Normal)"));
          manualTargetPosition = 0;
          setDirection(false);
          enableMotor(true);
          currentMoveState = MOVING_LEFT;
          Timer1.setPeriod(currentStepDelay);
          Timer1.start();
        } else {
          Serial.println(F("Cannot move left (At limit or busy)."));
          beepBuzzer(1);
        }
      }
      break;
    case CMD_START_RIGHT:
      {
        if (!isCalibrated) {
          Serial.println(F("!! Error: Not calibrated."));
          break;
        }
        if (currentMoveState == IDLE && digitalRead(LIMIT_SWITCH_DESTINATION) != LOW) {
          Serial.println(F("-> START_RIGHT (Normal)"));
          manualTargetPosition = (long)totalCalibrationSteps * currentMicrostepFactor;
          setDirection(true);
          enableMotor(true);
          currentMoveState = MOVING_RIGHT;
          Timer1.setPeriod(currentStepDelay);
          Timer1.start();
        } else {
          Serial.println(F("Cannot move right (At limit or busy)."));
          beepBuzzer(1);
        }
      }
      break;
    case CMD_START_LEFT_LIMITED:
      {
        if (!isCalibrated) {
          Serial.println(F("!! Error: Not calibrated."));
          break;
        }
        if (currentMoveState == MANUAL_PAUSED) {
          long target_us = args.a;
          if (atomicReadLong(currentPosition) > target_us && digitalRead(LIMIT_SWITCH_START) != LOW) {
            Serial.print(F("-> Limited Jog Left to us: "));
            Serial.println(target_us);
            limitedJogActive = true;
            setDirection(false);
            enableMotor(true);
            currentMoveState = MOVING_LEFT;
            manualTargetPosition = target_us;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else Serial.println(F("Cannot jog further left (target or limit)."));
        } else Serial.println(F("Limited jog only allowed when Manual Run is PAUSED."));
      }
      break;
    case CMD_START_RIGHT_LIMITED:
      {
        if (!isCalibrated) {
          Serial.println(F("!! Error: Not calibrated."));
          break;
        }
        if (currentMoveState == MANUAL_PAUSED) {
          long target_us = args.a;
          if (atomicReadLong(currentPosition) < target_us && digitalRead(LIMIT_SWITCH_DESTINATION) != LOW) {
            Serial.print(F("-> Limited Jog Right to us: "));
            Serial.println(target_us);
            limitedJogActive = true;
            setDirection(true);
            enableMotor(true);
            currentMoveState = MOVING_RIGHT;
            manualTargetPosition = target_us;
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else Serial.println(F("Cannot jog further right (target or limit)."));
        } else Serial.println(F("Limited jog only allowed when Manual Run is PAUSED."));
      }
      break;
    case CMD_SPEED_FAST:
      Serial.println(F("-> SPEED_FAST request"));
      setMicrostepMode(LOW, LOW, LOW, 1, FULL_STEP_DELAY_US);
      break;
    case CMD_SPEED_SMOOTH:
      Serial.println(F("-> SPEED_SMOOTH request"));
      setMicrostepMode(HIGH, LOW, LOW, 2, HALF_STEP_DELAY_US);
      break;
    case CMD_SPEED_SLOW:
      Serial.println(F("-> SPEED_SLOW request"));
      setMicrostepMode(LOW, HIGH, LOW, 4, QUARTER_STEP_DELAY_US);
      break;
    case CMD_MANUAL_RUN:
      {
        if (!isCalibrated) {
          Serial.println(F("!! Error: Not calibrated."));
          break;
        }
        stopMovement(false, true);
        long startPosFull = args.a, endPosFull = args.b;
        int cycles = (int)args.c;
        if (cycles < 1) {
          Serial.println(F("!! Error: MANUAL_RUN cycles < 1."));
          break;
        }
        manualStartPosition_us = startPosFull * currentMicrostepFactor;
        manualEndPosition_us = endPosFull * currentMicrostepFactor;
        manualTotalCycles = cycles;
        long max_us = totalCalibrationSteps * currentMicrostepFactor;
        if (manualStartPosition_us < 0 || manualStartPosition_us > max_us || manualEndPosition_us < 0 || manualEndPosition_us > max_us || manualStartPosition_us == manualEndPosition_us) {
          Serial.println(F("!! Error: Invalid Start/End for MANUAL_RUN."));
        } else startOrResumeManualMove(false);
      }
      break;
    case CMD_MANUAL_STOP:
      {
        Serial.println(F("-> MANUAL_STOP (Pause) request"));
        if (currentMoveState == MANUAL_MOVING_TO_END || currentMoveState == MANUAL_MOVING_TO_START) {
          stopReason = 1;
          stopMovement(false);
        } else Serial.println(F("Info: Not in an active manual move to pause."));
      }
      break;
    case CMD_MANUAL_ABORT:
      {
        Serial.println(F("-> MANUAL_ABORT request"));
        if (currentMoveState != IDLE || currentMoveState == MANUAL_PAUSED) {
          stopReason = 2;
          stopMovement(false);
          resetManualState();
          enableMotor(false);
          Serial.println(F("Manual Run fully aborted."));
          Serial.println(F("$ABORTED"));
        } else Serial.println(F("Info: No active or paused manual run to abort."));
      }
      break;
    case CMD_MANUAL_RESUME:
      {
        Serial.println(F("-> MANUAL_RESUME request"));
        if (currentMoveState == MANUAL_PAUSED) startOrResumeManualMove(true);
        else Serial.println(F("Info: Not paused to resume."));
      }
      break;
    case CMD_REC_ON:
      recActive = true;
      recPaused = false;
      Serial.println(F("REC:ON"));
      break;
    case CMD_REC_PAUSE:
      if (recActive) { recPaused = true; }
      Serial.println(F("REC:PAUSE"));
      break;
    case CMD_REC_OFF:
      recActive = false;
      recPaused = false;
      Serial.println(F("REC:OFF"));
      break;
    case CMD_NONE: break;
    case CMD_UNKNOWN:
    default:
      {
        if (isCalibrated) {
          Serial.print(F("!! Warning: Unknown command (calibrated): "));
          Serial.println(command_buffer);
        } else {
          Serial.println(F("!! Error: Motor not calibrated. Please $CALIBRATE first. !!"));
          Serial.println(F("   (Only $CALIBRATE, SET_DAC_V, SET_RES_MUX allowed before calibration)"));
          beepBuzzer(1);
        }
      }
      break;
  }
  command_index = 0;
  command_buffer[0] = '\0';
}
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println(F("\n--- Arduino Linear Rail Controller (Optimized - MEGA) ---"));
  Wire.begin();
  Wire.setClock(400000);  // Fast I2C 400kHz
  // 1) ریست کانفیگ به حالت پیش‌فرض
  ads1110.reset();
  Serial.println(F("[ADS1110] Reset done"));

  // (اختیاری) حالت تبدیل مداوم
  ads1110.setConMode(CONT);
  int mode = ads1110.getConMode();  // ممکن است کتابخانه 0/1 را متفاوت تعریف کرده باشد
  Serial.print(F("[ADS1110] Conversion Mode: "));
  Serial.println(mode ? F("SINGLE-SHOT") : F("CONTINUOUS"));  // براساس مثال کتابخانه

  // 2) رزولوشن: 16 بیت
  ads1110.setRes(RES_16);
  byte currentRes = ads1110.getRes();
  Serial.print(F("[ADS1110] Resolution set to: "));
  Serial.print(currentRes);
  Serial.println(F("-BIT"));

  // 3) نرخ نمونه‌برداری: 240 SPS
  ads1110.setSampleRate(SPS_240);
  byte currentRate = ads1110.getSampleRate();
  Serial.print(F("[ADS1110] Sample Rate set to: "));
  Serial.print(currentRate);
  Serial.println(F(" SPS"));

  // (اختیاری) مرجع ولتاژ: EXTERNAL (مطابق سخت‌افزار شما، مرجع 2.048V)
  ads1110.setVref(EXT_REF);
  bool vrefExt = ads1110.getVref();  // بسته به کتابخانه 1=EXT, 0=INT
  Serial.print(F("[ADS1110] Vref: "));
  Serial.println(vrefExt ? F("EXTERNAL") : F("INTERNAL"));
  if (Wire.endTransmission() == 0) LOG("ADS1110 configured (128SPS cont.)");
  else Serial.println(F("!! Error: Failed to configure ADS1110 !!"));
  if (!dac.begin(MCP4725_ADDRESS)) {
    Serial.println(F("!! Warning: MCP4725 DAC not found !!"));
  } else {
    uint16_t defaultDacRawValue = (uint16_t)((1.0 / MCP4725_VREF) * 4095.0);
    dac.setVoltage(defaultDacRawValue, false);
    currentDacVoltage = 1.0;
    Serial.print(F("Default DAC voltage set to: "));
    Serial.print(currentDacVoltage, 1);
    Serial.print(F("V (Raw: "));
    Serial.print(defaultDacRawValue);
    Serial.println(F(")"));
  }
  Timer1.initialize();
  Timer1.attachInterrupt(generateStepPulseISR);
  Timer1.stop();
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_STEP, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);

#if defined(__AVR_ATmega2560__)
  STEP_DDR |= _BV(STEP_BIT);
#endif

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(LIMIT_SWITCH_DESTINATION, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_START, INPUT_PULLUP);
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S3_PIN, OUTPUT);

  pinMode(MUX_EN_PIN, OUTPUT);
  digitalWrite(MUX_EN_PIN, LOW);  // پیش‌فرض: وصل (فعال)
  setMuxChannel(0);

  pinMode(BTN_ORIGIN, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  setMicrostepMode(LOW, LOW, LOW, 1, FULL_STEP_DELAY_US);
  microsteps_for_one_data_point = (long)STEPS_PER_MM / DATA_POINTS_PER_MM * currentMicrostepFactor;
  isCalibrated = false;
  atomicWriteLong(currentPosition, 0);
  totalCalibrationSteps = 0;
  resetManualState();
  // Initialize OLED (single begin inside oledInit)
  oledInit();
  drawOLED();
  Serial.println(F("--- Setup Complete. Waiting for $CALIBRATE ---"));
  lastAdcReadTime = millis();
}

// ------------------- loop ----------------------
void loop() {
  serviceBuzzer();
  // Serial processing
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == CMD_TERMINATOR) {
      command_buffer[command_index] = '\0';
      if (command_index > 0) processCommand();
      command_index = 0;
      command_buffer[0] = '\0';
    } else if (command_index < CMD_BUFFER_SIZE - 1) {
      if (ch >= ' ' && ch <= '~') command_buffer[command_index++] = ch;
    } else {
      Serial.println(F("!! Error: Command buffer overflow! Discarding. !!"));
      command_index = 0;
      command_buffer[0] = '\0';
      while (Serial.available() > 0 && Serial.read() != CMD_TERMINATOR) {}
    }
  }

  // Buttons & OLED
  pollButtons();
  drawOLED();

  // Data point emission
  if (dataPointReadyToSend) {
    dataPointReadyToSend = false;
    readAdcAndSendData(true);
  }

  // Movement state machine
  if (currentMoveState != IDLE && currentMoveState != MANUAL_PAUSED) {
    bool hitLimit = false;
    if ((currentMoveState == MOVING_LEFT || currentMoveState == MANUAL_MOVING_TO_START) && (limitStartHit || digitalRead(LIMIT_SWITCH_START) == LOW)) hitLimit = true;
    else if ((currentMoveState == MOVING_RIGHT || currentMoveState == MANUAL_MOVING_TO_END) && (limitDestHit || digitalRead(LIMIT_SWITCH_DESTINATION) == LOW)) hitLimit = true;

    if (hitLimit) {
      stopMovement(true);
    } else {
      bool targetReached = false;
      long currentActualTargetForThisLeg = 0;
      if (currentMoveState == MANUAL_MOVING_TO_END || currentMoveState == MANUAL_MOVING_TO_START)
        currentActualTargetForThisLeg = manualGoingToEnd ? manualEndPosition_us : manualStartPosition_us;
      else if (currentMoveState == MOVING_LEFT || currentMoveState == MOVING_RIGHT)
        currentActualTargetForThisLeg = manualTargetPosition;

      long pos = atomicReadLong(currentPosition);
      if ((directionRight && pos >= currentActualTargetForThisLeg) || (!directionRight && pos <= currentActualTargetForThisLeg) || (pos == currentActualTargetForThisLeg)) {
        if (currentActualTargetForThisLeg != 0 || (currentMoveState == MANUAL_MOVING_TO_START && currentActualTargetForThisLeg == 0)) {
          atomicWriteLong(currentPosition, currentActualTargetForThisLeg);
          targetReached = true;
        }
      }

      if (targetReached) {
        Timer1.stop();
        enableMotor(false);
        if (currentMoveState == MOVING_LEFT || currentMoveState == MOVING_RIGHT) {
          if (limitedJogActive) {
            limitedJogActive = false;
            currentMoveState = MANUAL_PAUSED;
            manualTargetPosition = 0;
            joggedWhilePaused = true;
            Serial.println(F("Manual run paused."));
            Serial.println(F("$PAUSED"));
            Serial.print(F("POSITION:"));
            Serial.println(atomicReadLong(currentPosition));
          } else {
            currentMoveState = IDLE;
            Serial.print(F("POSITION:"));
            Serial.println(atomicReadLong(currentPosition));
          }
        } else if (currentMoveState == MANUAL_MOVING_TO_END || currentMoveState == MANUAL_MOVING_TO_START) {
          Serial.print(F("Manual target leg reached: "));
          Serial.println(atomicReadLong(currentPosition));
          beepBuzzer(1);
          microsteps_moved_since_last_datapoint = 0;
          if (manualGoingToEnd) {
            manualTargetPosition = manualStartPosition_us;
            manualGoingToEnd = false;
            setDirection(false);
          } else {
            manualCompletedCycles++;
            Serial.print(F("CYCLE_DONE:"));
            Serial.println(manualCompletedCycles);
            if (manualCompletedCycles >= manualTotalCycles) {
              Serial.println(F("All manual cycles completed."));
              stopMovement(false);
              beepBuzzer(3);
              return;
            } else {
              manualTargetPosition = manualEndPosition_us;
              manualGoingToEnd = true;
              setDirection(true);
              Serial.print(F("Next cycle: "));
              Serial.println(manualCompletedCycles + 1);
            }
          }
          if (atomicReadLong(currentPosition) != manualTargetPosition) {
            setDirection(atomicReadLong(currentPosition) < manualTargetPosition);
            enableMotor(true);
            Timer1.setPeriod(currentStepDelay);
            Timer1.start();
          } else {
            Serial.println(F("Manual Start == End. Stopping."));
            stopMovement(false);
            return;
          }
        }
      }
    }
  }

  if (limitStartHit && !(currentMoveState == MOVING_LEFT || currentMoveState == MANUAL_MOVING_TO_START)) limitStartHit = false;
  if (limitDestHit && !(currentMoveState == MOVING_RIGHT || currentMoveState == MANUAL_MOVING_TO_END)) limitDestHit = false;

  unsigned long now_millis = millis();
  if (now_millis - lastAdcReadTime >= ADC_READ_INTERVAL_MS) {
    lastAdcReadTime = now_millis;
    readAdcAndSendData(false);
  }
}

// ---------------- Calibration ------------------
void calibrateMotor() {
  Serial.println(F("------------------------------------"));
  Serial.println(F("Starting Calibration Process..."));
  stopMovement(false, true);

  isCalibrated = false;
  limitStartHit = false;
  limitDestHit = false;
  atomicWriteLong(currentPosition, 0);
  totalCalibrationSteps = 0;

  Serial.println(F("Setting to FULL STEP mode for calibration..."));
  setMicrostepMode(LOW, LOW, LOW, 1, FULL_STEP_DELAY_US);
  delay(30);

  drawCalibratingScreen();

  enableMotor(true);
  Serial.println(F("Motor Enabled for Calibration."));
  Serial.println(F("Phase 1: Moving towards DESTINATION (Right)..."));
  setDirection(true);
  limitDestHit = false;
  detachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_START));
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_DESTINATION), isrLimitDestination, FALLING);

  Timer1.setPeriod(currentStepDelay);
  Timer1.start();
  while (!limitDestHit && digitalRead(LIMIT_SWITCH_DESTINATION) != LOW) { delay(1); }
  Timer1.stop();
  detachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_DESTINATION));
  Serial.println(F("Destination Limit Reached."));
  beepBuzzer(1);
  atomicWriteLong(currentPosition, 0);

  Serial.println(F("Phase 2: Moving towards START (Left) and counting steps..."));
  setDirection(false);
  limitStartHit = false;
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_START), isrLimitStart, FALLING);
  Timer1.setPeriod(currentStepDelay);
  Timer1.start();
  while (!limitStartHit && digitalRead(LIMIT_SWITCH_START) != LOW) { delay(1); }
  Timer1.stop();
  detachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_START));

  long steps = atomicReadLong(currentPosition);
  if (steps < 0) steps = -steps;
  totalCalibrationSteps = steps;
  atomicWriteLong(currentPosition, 0);
  isCalibrated = true;

  Serial.print(F("Start Limit Reached. Total Steps: "));
  Serial.println(totalCalibrationSteps);
  Serial.print(F("TOTAL_STEPS:"));
  Serial.println(totalCalibrationSteps);
  Serial.print(F("POSITION:"));
  Serial.println(atomicReadLong(currentPosition));
  Serial.println(F("$CALIBRATION_DONE"));
  beepBuzzer(2);

  enableMotor(false);
  delay(50);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_DESTINATION), isrLimitDestination, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_START), isrLimitStart, FALLING);
  Serial.println(F("------------------------------------"));
}