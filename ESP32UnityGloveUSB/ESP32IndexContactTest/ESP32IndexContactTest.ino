// OFF-HAND sphere-to-index actuation test. Manual CAL2 retained.
// Arm mounting (2026-11-12): 0 deg = 500us = arm just touching the pot reel (inner limit);
// higher pulses move the arm away from the reel. Usable range 500..2500us.
// HAP mode starts/releases at 2500us (180 deg, full clearance). Targets 500..2500us.
// Manual setup reference1000; HAP mode is separate and starts at 2500.
// CAL2 STOP detaches in place; HAP STOP/timeout returns to 2500 then detaches.
// GPIO17 index only; interpolation is provisional and force is not measured.
// Power only one servo. ARM can initially jump to 1000us (CAL2) or 2500us (HAP/CAL3/CAL4).
// Original five-potentiometer packet, pins and averaging preserved.
// CAL3 = true-degree moves: 0..180 deg -> DEG0_US..DEG180_US.
//   CAL3,DEG,<0-180>                       move and hold (keepalive CAL3,KEEP < 500ms)
//   CAL3,SWEEP,<from>,<to>,<step>,<settleMs>  firmware steps, settles, prints
//   SWEEP3,<deg>,<us>,<thumb>,<index>,<middle>,<ring>,<little> per point, then SWEEP3,DONE
//   CAL3,STOP                              detach in place
// CAL4 = WORN fist sweep (stays inside 500..2500us). User holds a fist against the arm;
// the arm steps from <from> to <to>, and at every step the index pot is sampled 10 times.
//   CAL4,SWEEP,<fromUs>,<toUs>,<stepUs>,<settleMs>,<lowRaw>,<highRaw>
//   The arm moves in at 400us/s until the pot changes (FISTACK,CONTACT,<us>), then steps.
//   FIST4,<us>,<mean>,<min>,<max>,<stable 0/1>  per step; FIST4,DONE at the end
//   FIST4,LIMIT,<us>,<raw>  pot reached <=lowRaw or >=highRaw: stop and return to rest (2500)
//   CAL4,KEEP keepalive (<500ms) / CAL4,STOP: return to rest (2500), then detach
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
// Thumb, index, middle, ring, little: adjacent headers VP,VN,D34,D35,D32.
const uint8_t PINS[5] = {36, 39, 34, 35, 32};
uint32_t lastSample = 0;
char line[64];
uint8_t length = 0;
bool discard = false, limited = false;
uint32_t lastValid = 0;

const uint8_t SERVO_PIN = 17;
// MIN_US = arm at the pot reel (0 deg). MAX_US = furthest from the reel (180 deg).
const int RELEASE_US = 1000, MIN_US = 500, MAX_US = 2500;
bool armed = false, returning = false;
bool contactMode = false;
const int CONTACT_RELEASE_US = 2500; // rest/release = 180 deg (more clearance; was 2050)
// This index assembly approaches the stop with decreasing pulse width.
// 20ms steps: approach 800us/s; release 1200us/s. Not a torque limit.
int contactApproachStepUs = 16;
int contactReleaseStepUs = 24;
int pulseUs = RELEASE_US, targetUs = RELEASE_US;
uint32_t lastCal = 0, lastStep = 0;

// Standard hobby servo: 500us = 0 deg, 2500us = 180 deg. Adjust for your servo.
const int DEG0_US = 500, DEG180_US = 2500;
const int DEG_STEP_US = 10; // per 20ms = 500us/s, about 45 deg/s
bool degMode = false, sweeping = false;
int curDeg = 0, sweepEnd = 180, sweepStep = 1;
uint32_t sweepSettleMs = 300, settleStart = 0;
int degToUs(long d) { return DEG0_US + (int)(((long)(DEG180_US - DEG0_US) * d + 90) / 180); }

// CAL4 worn fist sweep state.
const int FIST_STEP_US = 4;          // per 20ms = 200us/s toward the finger
const int FIST_RELEASE_STEP_US = 24; // per 20ms = 1200us/s back to rest
const int FIST_SAMPLES = 10, FIST_STABLE_RAW = 80, FIST_RETRIES = 3;
const int FIST_FREE_STEP_US = 8;     // per 20ms = 400us/s while the arm is not touching yet
const int FIST_CONTACT_RAW = 40;     // pot change that means the arm reached the finger
bool fistFree = false, fistContact = false;
int fistStartRaw = 0;
bool fistMode = false, fistSweeping = false;
int fistTo = 500, fistStep = 20, fistLow = 5, fistHigh = 4090;
uint32_t fistSettleMs = 250, fistSettleStart = 0, fistLastCheck = 0;

void writePulse() {
  uint32_t duty = ((uint32_t)pulseUs * 65535UL + 10000UL) / 20000UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}
void detachOutput() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(SERVO_PIN);
#else
  ledcDetachPin(SERVO_PIN);
#endif
  digitalWrite(SERVO_PIN, LOW);
  bool wasContact = contactMode, wasDeg = degMode, wasFist = fistMode;
  armed = false;
  returning = false;
  contactMode = false;
  degMode = sweeping = false;
  fistMode = fistSweeping = false;
  if (wasContact) Serial.println("HAPACK,OFF");
  if (wasDeg) Serial.println("DEGACK,OFF");
  if (wasFist) Serial.println("FISTACK,OFF");
  Serial.println("BENCHACK,OFF");
}
void releaseOutput() {
  if (armed && (contactMode || fistMode)) { targetUs = CONTACT_RELEASE_US; returning = true; fistSweeping = false; return; }
  if (armed) detachOutput(); // No automatic movement: new release position not yet calibrated.
}
void calCommand() {
  if (contactMode || degMode || fistMode) return;
  if (!strcmp(line, "CAL2,STOP")) { releaseOutput(); return; }
  if (!strcmp(line, "CAL2,ARM") || !strcmp(line, "CAL2,ARM2000")) {
    int startUs = !strcmp(line, "CAL2,ARM2000") ? 2000 : RELEASE_US;
    if (returning) return;
    if (!armed) {
      pulseUs = targetUs = startUs;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      if (!ledcAttach(SERVO_PIN, 50, 16)) { Serial.println("BENCHACK,ERROR_ATTACH"); return; }
#else
      if (!ledcSetup(0, 50, 16)) { Serial.println("BENCHACK,ERROR_ATTACH"); return; }
      ledcAttachPin(SERVO_PIN, 0);
#endif
      armed = true;
      writePulse();
    }
    lastCal = millis();
    Serial.print("BENCHACK,ARMED,"); Serial.println(targetUs);
    return;
  }
  if (!armed || returning) return;
  if (!strcmp(line, "CAL2,KEEP")) { lastCal = millis(); return; }
  if (!strncmp(line, "CAL2,TARGET,", 12)) {
    char* end;
    long value = strtol(line + 12, &end, 10);
    if (end == line + 12 || *end || value < MIN_US || value > MAX_US) {
      Serial.println("BENCHACK,REJECT_RANGE_500_2500"); return;
    }
    targetUs = value;
    lastCal = millis();
    Serial.print("BENCHACK,TARGET,"); Serial.println(targetUs);
  }
}
void contactCommand() {
  if (!strcmp(line, "HAP1,INFO")) {
    Serial.print("HAPINFO,2,"); Serial.print(contactApproachStepUs * 50);
    Serial.print(","); Serial.println(contactReleaseStepUs * 50);
    return;
  }
  if (!strncmp(line, "HAP1,RATE,", 10)) {
    char* end;
    long approach = strtol(line + 10, &end, 10);
    if (end == line + 10 || *end != ',') return;
    char* second = end + 1;
    long release = strtol(second, &end, 10);
    if (end == second || *end || approach < 100 || approach > 2000 ||
        release < 100 || release > 2000 || approach % 50 || release % 50) return;
    contactApproachStepUs = approach / 50;
    contactReleaseStepUs = release / 50;
    Serial.println("HAPACK,RATE_APPLIED");
    return; // Configuration does not arm or refresh the motion watchdog.
  }
  if (!strcmp(line, "HAP1,STOP")) {
    if (contactMode) releaseOutput();
    return;
  }
  if (!strcmp(line, "HAP1,ARM")) {
    if (armed) { Serial.println("HAPACK,ERROR_ALREADY_ACTIVE"); return; }
    pulseUs = targetUs = CONTACT_RELEASE_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(SERVO_PIN, 50, 16)) { Serial.println("HAPACK,ERROR_ATTACH"); return; }
#else
    if (!ledcSetup(0, 50, 16)) { Serial.println("HAPACK,ERROR_ATTACH"); return; }
    ledcAttachPin(SERVO_PIN, 0);
#endif
    contactMode = armed = true;
    returning = false;
    lastCal = millis();
    writePulse();
    Serial.print("HAPACK,ARMED,"); Serial.println(CONTACT_RELEASE_US);
    return;
  }
  if (!contactMode || !armed || returning) return;
  if (!strncmp(line, "HAP1,TARGET,", 12)) {
    char* end;
    long value = strtol(line + 12, &end, 10);
    if (end == line + 12 || *end || value < MIN_US || value > CONTACT_RELEASE_US) return;
    targetUs = value;
    lastCal = millis();
    Serial.print("HAPACK,TARGET,"); Serial.println(targetUs);
  }
}
// Comma-separated integers, exactly n of them, nothing after.
bool readInts(const char* s, long* out, int n) {
  for (int i = 0; i < n; i++) {
    char* end;
    out[i] = strtol(s, &end, 10);
    if (end == s) return false;
    if (i < n - 1) { if (*end != ',') return false; s = end + 1; }
    else if (*end) return false;
  }
  return true;
}
void degCommand() {
  if (!strcmp(line, "CAL3,STOP")) { if (degMode) detachOutput(); return; }
  if (!strcmp(line, "CAL3,KEEP")) { if (degMode) lastCal = millis(); return; }
  long v[4];
  bool move = !strncmp(line, "CAL3,DEG,", 9) && readInts(line + 9, v, 1);
  bool sweep = !move && !strncmp(line, "CAL3,SWEEP,", 11) && readInts(line + 11, v, 4);
  if (!move && !sweep) return;
  if (armed && !degMode) { Serial.println("DEGACK,ERROR_BUSY"); return; }
  if (returning) return;
  if (v[0] < 0 || v[0] > 180 || (sweep && (v[1] < 0 || v[1] > 180 || v[2] < 1 || v[2] > 90 ||
      v[3] < 50 || v[3] > 5000))) { Serial.println("DEGACK,REJECT_RANGE"); return; }
  if (!armed) {
    // Start from the normal rest pulse (HAP leaves the servo at CONTACT_RELEASE_US), then slew.
    pulseUs = targetUs = CONTACT_RELEASE_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(SERVO_PIN, 50, 16)) { Serial.println("DEGACK,ERROR_ATTACH"); return; }
#else
    if (!ledcSetup(0, 50, 16)) { Serial.println("DEGACK,ERROR_ATTACH"); return; }
    ledcAttachPin(SERVO_PIN, 0);
#endif
    armed = degMode = true;
    returning = false;
    writePulse();
  }
  curDeg = v[0];
  targetUs = degToUs(curDeg);
  lastCal = millis();
  sweeping = sweep;
  if (sweep) {
    sweepEnd = v[1]; sweepStep = v[2]; sweepSettleMs = v[3];
    settleStart = millis();
    Serial.printf("DEGACK,SWEEP,%d,%d,%d\n", curDeg, sweepEnd, sweepStep);
  } else {
    Serial.printf("DEGACK,TARGET,%d,%d\n", curDeg, targetUs);
  }
}
// Runs the CAL3 sweep: wait until the pulse reaches the target, settle, sample, next.
void updateSweep() {
  if (!sweeping || !armed) return;
  uint32_t now = millis();
  if (pulseUs != targetUs) { settleStart = now; return; }
  if (now - settleStart < sweepSettleMs) return;
  long sum[5] = {0, 0, 0, 0, 0};
  for (int n = 0; n < 32; n++)
    for (int i = 0; i < 5; i++) sum[i] += analogRead(PINS[i]);
  Serial.printf("SWEEP3,%d,%d,%d,%d,%d,%d,%d\n", curDeg, pulseUs,
    (int)(sum[0] / 32), (int)(sum[1] / 32), (int)(sum[2] / 32), (int)(sum[3] / 32), (int)(sum[4] / 32));
  if (curDeg == sweepEnd) { sweeping = false; Serial.println("SWEEP3,DONE"); return; }
  curDeg = curDeg < sweepEnd ? min(curDeg + sweepStep, sweepEnd) : max(curDeg - sweepStep, sweepEnd);
  targetUs = degToUs(curDeg);
  settleStart = now;
}

int readIndexPot() {
  uint32_t sum = 0;
  for (int n = 0; n < 4; n++) sum += analogRead(PINS[1]);
  return (int)(sum / 4);
}
// Pot at an end: stop pushing, report, and pull the arm back to rest.
bool fistLimitHit(int raw) {
  if (raw > fistLow && raw < fistHigh) return false;
  Serial.printf("FIST4,LIMIT,%d,%d\n", pulseUs, raw);
  releaseOutput();
  return true;
}
void fistCommand() {
  if (!strcmp(line, "CAL4,STOP")) { if (fistMode) releaseOutput(); return; }
  if (!strcmp(line, "CAL4,KEEP")) { if (fistMode) lastCal = millis(); return; }
  long v[6];
  if (strncmp(line, "CAL4,SWEEP,", 11) || !readInts(line + 11, v, 6)) return;
  if (armed) { Serial.println("FISTACK,ERROR_BUSY"); return; }
  if (v[0] < MIN_US || v[0] > MAX_US || v[1] < MIN_US || v[1] > MAX_US || v[1] >= v[0] ||
      v[2] < 5 || v[2] > 200 || v[3] < 50 || v[3] > 3000 || v[4] < 0 || v[5] > 4095 || v[4] >= v[5]) {
    Serial.println("FISTACK,REJECT_RANGE"); return;
  }
  fistLow = v[4]; fistHigh = v[5];
  int raw = readIndexPot();
  if (raw <= fistLow || raw >= fistHigh) { Serial.printf("FISTACK,REJECT_POT_AT_LIMIT,%d\n", raw); return; }
  pulseUs = targetUs = CONTACT_RELEASE_US; // normal rest; slew from here to <from>
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(SERVO_PIN, 50, 16)) { Serial.println("FISTACK,ERROR_ATTACH"); return; }
#else
  if (!ledcSetup(0, 50, 16)) { Serial.println("FISTACK,ERROR_ATTACH"); return; }
  ledcAttachPin(SERVO_PIN, 0);
#endif
  armed = fistMode = fistSweeping = true;
  fistFree = fistContact = false;
  returning = false;
  writePulse();
  targetUs = v[0];
  fistTo = v[1]; fistStep = v[2]; fistSettleMs = v[3];
  fistSettleStart = lastCal = millis();
  Serial.printf("FISTACK,SWEEP,%d,%d,%d\n", (int)v[0], fistTo, fistStep);
}
void updateFist() {
  if (!fistMode || !fistSweeping || returning || !armed) return;
  uint32_t now = millis();
  if (now - fistLastCheck >= 5) {   // check the pot all the time, also while moving
    fistLastCheck = now;
    int raw = readIndexPot();
    if (fistLimitHit(raw)) return;
    // Free approach: the first real pot change means the arm reached the finger.
    // Back off one step and measure from there in small steps.
    if (fistFree && abs(raw - fistStartRaw) > FIST_CONTACT_RAW) {
      fistFree = false; fistContact = true;
      Serial.printf("FISTACK,CONTACT,%d\n", pulseUs);
      targetUs = min(CONTACT_RELEASE_US, pulseUs + fistStep);
    }
  }
  if (pulseUs != targetUs) { fistSettleStart = now; return; }
  // Reached <to> without ever touching: settle, take one sample there, finish.
  if (fistFree) { fistFree = false; fistContact = true; return; }
  if (now - fistSettleStart < fistSettleMs) return;
  // Ten samples, 5ms apart. Retry if the finger was still moving.
  int mean = 0, lo = 4095, hi = 0;
  for (int attempt = 0; attempt < FIST_RETRIES; attempt++) {
    long sum = 0; lo = 4095; hi = 0;
    for (int n = 0; n < FIST_SAMPLES; n++) {
      int raw = readIndexPot();
      if (fistLimitHit(raw)) return;
      sum += raw; lo = min(lo, raw); hi = max(hi, raw);
      delay(5);
    }
    mean = (int)(sum / FIST_SAMPLES);
    if (hi - lo <= FIST_STABLE_RAW) break;
  }
  Serial.printf("FIST4,%d,%d,%d,%d,%d\n", pulseUs, mean, lo, hi, hi - lo <= FIST_STABLE_RAW ? 1 : 0);
  if (!fistContact) {               // first point (arm free): move in steadily until contact
    fistStartRaw = mean;
    fistFree = true;
    targetUs = fistTo;
    return;
  }
  if (pulseUs <= fistTo) { Serial.println("FIST4,DONE"); releaseOutput(); return; }
  targetUs = max(fistTo, pulseUs - fistStep);
  fistSettleStart = now;
}

void updateServo() {
  uint32_t now = millis();
  if (!armed) return;
  if (!returning && now - lastCal > 500) {
    releaseOutput();
    Serial.println(contactMode ? "HAPACK,TIMEOUT_RELEASE" : "BENCHACK,TIMEOUT_OFF");
    if (!armed) return;
  }
  if (now - lastStep < 20) return;
  lastStep = now;
  int stepUs = contactMode ? (targetUs > pulseUs ? contactReleaseStepUs : contactApproachStepUs) :
               fistMode ? (targetUs > pulseUs ? FIST_RELEASE_STEP_US : fistFree ? FIST_FREE_STEP_US : FIST_STEP_US) :
               degMode ? DEG_STEP_US : 2;
  if (pulseUs < targetUs) pulseUs += min(stepUs, targetUs - pulseUs);
  if (pulseUs > targetUs) pulseUs -= min(stepUs, pulseUs - targetUs);
  writePulse();
  if (returning && pulseUs == CONTACT_RELEASE_US) detachOutput();
}

void parse() {
  if (!strncmp(line, "HAP1,", 5)) { contactCommand(); return; }
  if (!strncmp(line, "CAL2,", 5)) { calCommand(); return; }
  if (!strncmp(line, "CAL3,", 5)) { degCommand(); return; }
  if (!strncmp(line, "CAL4,", 5)) { fistCommand(); return; }
  if (!strcmp(line, "SIM1,RELEASE")) {
    limited = false;
    lastValid = millis();
    Serial.println("ACK1,RELEASE");
  } else if (!strncmp(line, "SIM1,LIMIT,", 11)) {
    char* end;
    double curl = strtod(line + 11, &end);
    if (end == line + 11 || *end || !isfinite(curl) || curl < 0 || curl > 1) return;
    limited = true;
    lastValid = millis();
    Serial.print("ACK1,LIMIT,"); Serial.println(curl, 3);
  }
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  analogReadResolution(12);
  for (int i = 0; i < 5; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);
  }
  Serial.begin(115200);
  Serial.println("SIM_READY,CONTACT_TEST_DEFAULT_OFF");
}

void loop() {
  for (int n = 0; n < 64 && Serial.available(); n++) {
    char ch = Serial.read();
    if (ch == '\n') {
      line[length] = 0;
      if (!discard && length) parse();
      length = 0; discard = false;
    } else if (ch != '\r' && !discard) {
      if (length < sizeof(line) - 1) line[length++] = ch;
      else discard = true;
    }
  }
  if (limited && millis() - lastValid > 500) {
    limited = false;
    Serial.println("ACK1,RELEASE_TIMEOUT");
  }
  updateServo();
  updateSweep();
  updateFist();
  uint32_t now = millis();
  if (now - lastSample >= 20) {
    lastSample = now;
    int raw[5];
    for (int i = 0; i < 5; i++) {
      uint32_t sum = 0;
      for (int n = 0; n < 8; n++) sum += analogRead(PINS[i]);
      raw[i] = sum / 8;
    }
    Serial.printf("GLOVE1,%d,%d,%d,%d,%d\n", raw[0], raw[1], raw[2], raw[3], raw[4]);
  }
}
