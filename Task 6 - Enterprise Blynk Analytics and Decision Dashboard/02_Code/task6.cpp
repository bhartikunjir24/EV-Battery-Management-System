#include <Arduino.h>
#include "secret.h"

#include <WiFi.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD assumes a 16x2 I2C module at address 0x27, SDA=GPIO21, SCL=GPIO22.
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL_MS = 500;

// Status LEDs: assumed wiring from the earlier circuit: green=GPIO18, yellow=GPIO19, red=GPIO23.
const int GREEN_LED_PIN = 18;
const int YELLOW_LED_PIN = 19;
const int RED_LED_PIN = 23;
const unsigned long GREEN_LED_BLINK_INTERVAL_MS = 500;
unsigned long lastGreenLedToggle = 0;
bool greenLedLevel = false;

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>

// =====================================================
// TASK 4 - STAGE 4
// Structured Logging + Verified Fault Recovery
// ESP32 DevKit C V4 / Wokwi
// Existing wiring unchanged
// =====================================================

enum SystemState { NORMAL, DEGRADED, FAILSAFE, SHUTDOWN };

enum FaultID {
  NO_FAULT,
  F001_BATTERY,
  F002_ADC,
  F003_RELAY,
  F004_COMMUNICATION,
  F005_SYSTEM
};

struct FaultRecord {
  FaultID id;
  const char* source;
  bool active;
};

FaultRecord faults[] = {
  {F001_BATTERY, "BATTERY", false},
  {F002_ADC, "ADC", false},
  {F003_RELAY, "RELAY", false},
  {F004_COMMUNICATION, "COMMUNICATION", false},
  {F005_SYSTEM, "SYSTEM", false}
};

const int FAULT_COUNT = 5;
const int CELL_PINS[] = {34, 35, 32};
const int RELAY_PIN = 4;

// Non-blocking timing
const unsigned long SAMPLE_INTERVAL_MS = 250;
const unsigned long ADC_FREEZE_TIME_MS = 5000;
const unsigned long RELAY_MISMATCH_TIME_MS = 1000;
const unsigned long RECOVERY_VERIFY_MS = 3000;
const unsigned long STATUS_INTERVAL_MS = 3000;

// ADC freeze-test monitoring
int previousADC = -1;
unsigned long lastADCChange = 0;
unsigned long lastSample = 0;
int adcValue = 0;
bool simulateADCFrozen = false;
int frozenADCValue = 0;

// Relay control and software-simulated feedback
bool simulateRelayMismatch = false;
bool relayCommand = false;
bool simulatedRelayFeedback = false;
unsigned long relayMismatchStart = 0;
bool relayMismatchTimerStarted = false;

// System state and recovery verification
SystemState currentState = NORMAL;
bool shutdownLatched = false;
bool recoveryPending = false;
unsigned long recoveryStart = 0;
unsigned long lastStatus = 0;

// =====================================================
// TASK 5 - Event-driven telemetry + bounded offline queue
// =====================================================
const uint8_t TELEMETRY_QUEUE_CAPACITY = 32;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
const unsigned long BLYNK_RETRY_INTERVAL_MS = 5000;
const float CELL_CHANGE_THRESHOLD_V = 0.05f;
const int RSSI_CHANGE_THRESHOLD_DBM = 5;
const unsigned long RSSI_CHECK_INTERVAL_MS = 5000;

struct TelemetryEvent {
  unsigned long timestampMs;
  char eventName[32];
  float cellVoltage[3];
  int weakestCell;
  int strongestCell;
  bool relayOn;
  char faultState[32];
  char wifiHealth[20];
  int rssi;
};

TelemetryEvent telemetryQueue[TELEMETRY_QUEUE_CAPACITY];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueCount = 0;
unsigned long droppedTelemetryEvents = 0;

float cellVoltage[3] = {0.0f, 0.0f, 0.0f};
float lastPublishedCellVoltage[3] = {-100.0f, -100.0f, -100.0f};
unsigned long lastWifiAttempt = 0;
unsigned long lastBlynkAttempt = 0;
unsigned long lastRssiCheck = 0;
int lastReportedRssi = -127;
bool wifiWasConnected = false;
bool blynkWasConnected = false;
bool liveRefreshPending = false;

void enqueueTelemetryEvent(const char* eventName);
void serviceTelemetry();
void serviceConnectivity();
void updateCellVoltageReadings(int raw1, int raw2, int raw3);
int weakestCellNumber();
int strongestCellNumber();
void publishTelemetry(const TelemetryEvent& event, const char* source);
void publishLiveSnapshot();

// TASK 6 analytics globals
unsigned long systemStartMs = 0;
unsigned long lastTask6Publish = 0;
const unsigned long TASK6_PUBLISH_INTERVAL_MS = 3000;
uint32_t cumulativeFaultCount = 0;
char latestTransition[64] = "BOOT -> NORMAL";
float task6RiskScore = 0.0f;
float task6SoC = 0.0f; // Simulation estimate only; not a calibrated battery SoC.
float task6Imbalance = 0.0f;
void updateTask6Analytics();
void publishTask6Dashboard();
const char* task6Recommendation();
const char* task6BatteryHealth();


void printFaults();

// -----------------------------------------------------
// Names and fault lookup
// -----------------------------------------------------
const char* stateName(SystemState s) {
  switch (s) {
    case NORMAL: return "NORMAL";
    case DEGRADED: return "DEGRADED";
    case FAILSAFE: return "FAILSAFE";
    case SHUTDOWN: return "SHUTDOWN";
    default: return "UNKNOWN";
  }
}

// Non-blocking LCD refresh: shows the live state from the existing state machine.
void updateLCD() {
  const unsigned long now = millis();
  if (now - lastLCDUpdate < LCD_UPDATE_INTERVAL_MS) return;
  lastLCDUpdate = now;

  lcd.setCursor(0, 0);
  lcd.print("EV BMS MONITOR "); // 15 characters, padded for a 16x2 display

  lcd.setCursor(0, 1);
  lcd.print("State: ");
  lcd.print(stateName(currentState));
  lcd.print("        "); // clear any leftover characters from longer previous states
}

const char* faultName(FaultID id) {
  switch (id) {
    case F001_BATTERY: return "F001";
    case F002_ADC: return "F002";
    case F003_RELAY: return "F003";
    case F004_COMMUNICATION: return "F004";
    case F005_SYSTEM: return "F005";
    default: return "NONE";
  }
}

FaultRecord* findFault(FaultID id) {
  for (int i = 0; i < FAULT_COUNT; i++) {
    if (faults[i].id == id) return &faults[i];
  }
  return nullptr;
}

bool anyFaultActive() {
  for (int i = 0; i < FAULT_COUNT; i++) {
    if (faults[i].active) return true;
  }
  return false;
}

bool criticalFaultActive() {
  FaultRecord* adc = findFault(F002_ADC);
  FaultRecord* relay = findFault(F003_RELAY);
  return (adc && adc->active) || (relay && relay->active);
}

FaultID highestPriorityFault() {
  const FaultID priority[] = {
    F005_SYSTEM, F002_ADC, F003_RELAY,
    F001_BATTERY, F004_COMMUNICATION
  };
  for (int i = 0; i < 5; i++) {
    FaultRecord* f = findFault(priority[i]);
    if (f && f->active) return f->id;
  }
  return NO_FAULT;
}


// Forward declarations used by Task 5 helpers before Task 4 implementations.
const char* stateName(SystemState s);
const char* faultName(FaultID id);
FaultRecord* findFault(FaultID id);
FaultID highestPriorityFault();
bool anyFaultActive();

// -----------------------------------------------------
// Task 5 - telemetry queue and dashboard helpers
// -----------------------------------------------------
int weakestCellNumber() {
  int index = 0;
  for (int i = 1; i < 3; i++) {
    if (cellVoltage[i] < cellVoltage[index]) index = i;
  }
  return index + 1;
}

int strongestCellNumber() {
  int index = 0;
  for (int i = 1; i < 3; i++) {
    if (cellVoltage[i] > cellVoltage[index]) index = i;
  }
  return index + 1;
}

void updateCellVoltageReadings(int raw1, int raw2, int raw3) {
  const int raw[3] = {raw1, raw2, raw3};
  bool significantChange = false;

  for (int i = 0; i < 3; i++) {
    // Wokwi potentiometer ADC reading converted to ESP32 ADC pin voltage.
    // This is a simulation input, not a calibrated physical battery-cell measurement.
    cellVoltage[i] = (raw[i] / 4095.0f) * 3.3f;
    if (fabsf(cellVoltage[i] - lastPublishedCellVoltage[i]) >= CELL_CHANGE_THRESHOLD_V) {
      significantChange = true;
    }
  }

  if (significantChange) {
    enqueueTelemetryEvent("CELL_VOLTAGE_CHANGE");
    for (int i = 0; i < 3; i++) lastPublishedCellVoltage[i] = cellVoltage[i];
  }
}

void enqueueTelemetryEvent(const char* eventName) {
  TelemetryEvent event = {};
  event.timestampMs = millis();
  snprintf(event.eventName, sizeof(event.eventName), "%s", eventName ? eventName : "EVENT");
  for (int i = 0; i < 3; i++) event.cellVoltage[i] = cellVoltage[i];
  event.weakestCell = weakestCellNumber();
  event.strongestCell = strongestCellNumber();
  event.relayOn = relayCommand;

  FaultID activeFault = highestPriorityFault();
  if (activeFault == NO_FAULT) {
    snprintf(event.faultState, sizeof(event.faultState), "%s", stateName(currentState));
  } else {
    FaultRecord* fault = findFault(activeFault);
    snprintf(event.faultState, sizeof(event.faultState), "%s %s",
             faultName(activeFault), fault ? fault->source : "UNKNOWN");
  }

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  snprintf(event.wifiHealth, sizeof(event.wifiHealth), "%s",
           wifiConnected ? "CONNECTED" : "DISCONNECTED");
  event.rssi = wifiConnected ? WiFi.RSSI() : -100;

  if (queueCount >= TELEMETRY_QUEUE_CAPACITY) {
    // Explicit overflow policy: discard the oldest queued event and retain the newest.
    queueHead = (queueHead + 1) % TELEMETRY_QUEUE_CAPACITY;
    queueCount--;
    droppedTelemetryEvents++;
    Serial.printf("[TELEMETRY] Queue full; oldest event dropped. Total dropped=%lu\r\n",
                  droppedTelemetryEvents);
  }

  telemetryQueue[queueTail] = event;
  queueTail = (queueTail + 1) % TELEMETRY_QUEUE_CAPACITY;
  queueCount++;
}

void publishTelemetry(const TelemetryEvent& event, const char* source) {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, event.cellVoltage[0]);
  Blynk.virtualWrite(V1, event.cellVoltage[1]);
  Blynk.virtualWrite(V2, event.cellVoltage[2]);
  Blynk.virtualWrite(V3, event.weakestCell);
  Blynk.virtualWrite(V4, event.strongestCell);
  Blynk.virtualWrite(V5, event.relayOn ? 1 : 0);
  Blynk.virtualWrite(V6, event.faultState);
  Blynk.virtualWrite(V7, event.wifiHealth);
  Blynk.virtualWrite(V8, event.rssi);
  Blynk.virtualWrite(V9, queueCount);
  Blynk.virtualWrite(V10, source);
}

void publishLiveSnapshot() {
  TelemetryEvent event = {};
  event.timestampMs = millis();
  snprintf(event.eventName, sizeof(event.eventName), "LIVE_SNAPSHOT");
  for (int i = 0; i < 3; i++) event.cellVoltage[i] = cellVoltage[i];
  event.weakestCell = weakestCellNumber();
  event.strongestCell = strongestCellNumber();
  event.relayOn = relayCommand;

  FaultID activeFault = highestPriorityFault();
  if (activeFault == NO_FAULT) {
    snprintf(event.faultState, sizeof(event.faultState), "%s", stateName(currentState));
  } else {
    FaultRecord* fault = findFault(activeFault);
    snprintf(event.faultState, sizeof(event.faultState), "%s %s",
             faultName(activeFault), fault ? fault->source : "UNKNOWN");
  }

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  snprintf(event.wifiHealth, sizeof(event.wifiHealth), "%s",
           wifiConnected ? "CONNECTED" : "DISCONNECTED");
  event.rssi = wifiConnected ? WiFi.RSSI() : -100;
  publishTelemetry(event, "LIVE");
}

void serviceTelemetry() {
  if (!Blynk.connected()) return;

  // FIFO: one stored event is transmitted per loop pass, oldest first.
  if (queueCount > 0) {
    TelemetryEvent event = telemetryQueue[queueHead];
    publishTelemetry(event, "QUEUED");
    queueHead = (queueHead + 1) % TELEMETRY_QUEUE_CAPACITY;
    queueCount--;
    Serial.printf("[TELEMETRY] Sent queued event: %s; remaining=%u\r\n",
                  event.eventName, queueCount);
    liveRefreshPending = true;
    return;
  }

  if (liveRefreshPending) {
    publishLiveSnapshot();
    liveRefreshPending = false;
    Serial.println("[TELEMETRY] Queue drained; dashboard refreshed with LIVE snapshot.");
  }
}

void serviceConnectivity() {
  const unsigned long now = millis();
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (!wifiConnected && now - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiAttempt = now;
    Serial.println("[NETWORK] Starting non-blocking Wi-Fi connection attempt.");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS,6);
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected && !wifiWasConnected) {
    wifiWasConnected = true;
    Serial.printf("[NETWORK] Wi-Fi connected. IP=%s RSSI=%d dBm\r\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    enqueueTelemetryEvent("WIFI_CONNECTED");
  } else if (!wifiConnected && wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[NETWORK] Wi-Fi connection lost.");
    enqueueTelemetryEvent("WIFI_DISCONNECTED");
  }

  if (wifiConnected && !Blynk.connected() &&
      now - lastBlynkAttempt >= BLYNK_RETRY_INTERVAL_MS) {
    lastBlynkAttempt = now;
    Serial.println("[BLYNK] Attempting cloud connection.");
    // A very short timeout prevents a long blocking reconnect loop.
    Blynk.connect(3000);
  }

  bool blynkConnected = Blynk.connected();
  if (blynkConnected && !blynkWasConnected) {
    blynkWasConnected = true;
    Serial.println("[BLYNK] Connected to Blynk Cloud.");
    enqueueTelemetryEvent("BLYNK_CONNECTED");
    liveRefreshPending = true;
  } else if (!blynkConnected && blynkWasConnected) {
    blynkWasConnected = false;
    Serial.println("[BLYNK] Cloud connection lost; events will be queued.");
    enqueueTelemetryEvent("BLYNK_DISCONNECTED");
  }

  if (wifiConnected && now - lastRssiCheck >= RSSI_CHECK_INTERVAL_MS) {
    lastRssiCheck = now;
    int rssi = WiFi.RSSI();
    if (abs(rssi - lastReportedRssi) >= RSSI_CHANGE_THRESHOLD_DBM) {
      lastReportedRssi = rssi;
      enqueueTelemetryEvent("RSSI_CHANGE");
      Serial.printf("[NETWORK] RSSI changed to %d dBm\r\n", rssi);
    }
  }
}

// Non-blocking status LEDs synchronized with the state machine.
// NORMAL: green blinks; DEGRADED/FAILSAFE: yellow ON; SHUTDOWN: red ON.
void updateGreenStatusLED() {
  const unsigned long now = millis();

  // Yellow indicates a warning or critical failsafe state.
  digitalWrite(YELLOW_LED_PIN,
               (currentState == DEGRADED || currentState == FAILSAFE) ? HIGH : LOW);

  // Red indicates latched shutdown.
  digitalWrite(RED_LED_PIN, currentState == SHUTDOWN ? HIGH : LOW);

  // Green blinks only while the system is NORMAL.
  if (currentState != NORMAL) {
    greenLedLevel = false;
    digitalWrite(GREEN_LED_PIN, LOW);
    lastGreenLedToggle = now;
    return;
  }

  if (now - lastGreenLedToggle >= GREEN_LED_BLINK_INTERVAL_MS) {
    lastGreenLedToggle = now;
    greenLedLevel = !greenLedLevel;
    digitalWrite(GREEN_LED_PIN, greenLedLevel ? HIGH : LOW);
  }
}

// -----------------------------------------------------
// Structured event and transition logging
// -----------------------------------------------------
void logEvent(const char* event, FaultID id, const char* detail) {
  FaultRecord* f = findFault(id);
  const char* source = f ? f->source : "NONE";
  Serial.printf(
    "{\"timestamp_ms\":%lu,\"event\":\"%s\",\"fault_id\":\"%s\","
    "\"fault_source\":\"%s\",\"detail\":\"%s\"}\r\n",
    millis(), event, faultName(id), source, detail
  );
  enqueueTelemetryEvent(event);
}

void logTransition(SystemState previous, SystemState next, FaultID trigger) {
  FaultRecord* f = findFault(trigger);
  const char* source = f ? f->source : "NONE";
  snprintf(latestTransition, sizeof(latestTransition), "%s -> %s",
           stateName(previous), stateName(next));
  Serial.printf(
    "{\"timestamp_ms\":%lu,\"event\":\"STATE_TRANSITION\","
    "\"previous_state\":\"%s\",\"new_state\":\"%s\","
    "\"fault_id\":\"%s\",\"fault_source\":\"%s\"}\r\n",
    millis(), stateName(previous), stateName(next),
    faultName(trigger), source
  );
  enqueueTelemetryEvent("STATE_TRANSITION");
}

void transitionTo(SystemState next, FaultID trigger) {
  if (next == currentState) return;

  if (shutdownLatched && next != SHUTDOWN) {
    logEvent("TRANSITION_REJECTED", trigger, "shutdown_latch_active");
    return;
  }

  SystemState previous = currentState;
  currentState = next;
  logTransition(previous, next, trigger);
}

// -----------------------------------------------------
// State selection and recovery verification
// -----------------------------------------------------
SystemState desiredState() {
  FaultRecord* system = findFault(F005_SYSTEM);
  if (shutdownLatched || (system && system->active)) return SHUTDOWN;
  if (criticalFaultActive()) return FAILSAFE;
  if (anyFaultActive()) return DEGRADED;
  return NORMAL;
}

void evaluateState(FaultID trigger) {
  SystemState target = desiredState();

  if (target == SHUTDOWN) {
    recoveryPending = false;
    if (target != currentState) {
      shutdownLatched = true;
      transitionTo(SHUTDOWN, F005_SYSTEM);
    }
    return;
  }

  // A critical fault must be cleared before recovery can begin.
  if (target == FAILSAFE) {
    recoveryPending = false;
    transitionTo(FAILSAFE, trigger);
    return;
  }

  // If leaving FAILSAFE, hold that state during timed verification.
  if (currentState == FAILSAFE && target != FAILSAFE) {
    if (!recoveryPending) {
      recoveryPending = true;
      recoveryStart = millis();
      logEvent("RECOVERY_STARTED", trigger, "verification_timer_started");
      Serial.printf("[RECOVERY] Verification started; wait %lu ms.\r\n",
                    RECOVERY_VERIFY_MS);
    }
    return;
  }

  // If already verifying, do not skip the timer.
  if (recoveryPending) return;

  transitionTo(target, trigger);
}

void serviceRecovery() {
  if (!recoveryPending) return;

  // Any renewed critical fault cancels the recovery attempt.
  if (shutdownLatched || criticalFaultActive()) {
    recoveryPending = false;
    logEvent("RECOVERY_CANCELLED", highestPriorityFault(),
             "fault_returned_or_shutdown_latched");
    Serial.println("[RECOVERY] Verification cancelled; fault is active.");
    evaluateState(highestPriorityFault());
    return;
  }

  if (millis() - recoveryStart < RECOVERY_VERIFY_MS) return;

  // Verify the conditions again at the end of the timer.
  SystemState target = desiredState();
  if (target == FAILSAFE || target == SHUTDOWN) {
    recoveryPending = false;
    logEvent("RECOVERY_FAILED", highestPriorityFault(),
             "verification_conditions_not_met");
    evaluateState(highestPriorityFault());
    return;
  }

  recoveryPending = false;
  logEvent("RECOVERY_VERIFIED", NO_FAULT, "timed_verification_passed");
  Serial.println("[RECOVERY] Verification passed.");
  transitionTo(target, highestPriorityFault());
}

// -----------------------------------------------------
// Fault retention
// -----------------------------------------------------
void injectFault(FaultID id) {
  FaultRecord* f = findFault(id);
  if (!f) return;

  if (f->active) return; // Avoid repeated "already active" messages.

  f->active = true;
  cumulativeFaultCount++;
  recoveryPending = false;
  logEvent("FAULT_DETECTED", id, "fault_marked_active");
  Serial.printf("[FAULT DETECTED] ID=%s SOURCE=%s\r\n",
                faultName(id), f->source);

  evaluateState(id);
  printFaults();
}

void clearFault(FaultID id) {
  FaultRecord* f = findFault(id);
  if (!f) return;

  if (id == F005_SYSTEM) {
    logEvent("CLEAR_REJECTED", id, "use_shutdown_reset");
    Serial.println("[REJECTED] Use shutdown reset.");
    return;
  }

  if (!f->active) {
    Serial.printf("[INFO] %s is not active.\r\n", faultName(id));
    return;
  }

  f->active = false;
  logEvent("FAULT_CLEARED", id, "fault_marked_inactive");
  Serial.printf("[FAULT CLEARED] ID=%s SOURCE=%s\r\n",
                faultName(id), f->source);

  evaluateState(id);
  printFaults();
}

void printFaults() {
  Serial.println("[FAULTS] Active fault list:");
  bool found = false;

  for (int i = 0; i < FAULT_COUNT; i++) {
    if (faults[i].active) {
      Serial.printf(
        "{\"timestamp_ms\":%lu,\"fault_id\":\"%s\","
        "\"fault_source\":\"%s\",\"active\":true}\r\n",
        millis(), faultName(faults[i].id), faults[i].source
      );
      found = true;
    }
  }

  if (!found) Serial.println("No active faults.");
}

void resetShutdown() {
  if (!shutdownLatched) {
    Serial.println("[INFO] Shutdown is not active.");
    return;
  }

  FaultRecord* f = findFault(F005_SYSTEM);
  if (f) f->active = false;

  shutdownLatched = false;
  logEvent("SHUTDOWN_RESET_TEST_ONLY", F005_SYSTEM,
           "system_fault_cleared_for_simulation");
  Serial.println("[TEST ONLY] Shutdown reset requested.");

  // Any other active faults remain active and determine the next state.
  evaluateState(F005_SYSTEM);
  printFaults();
}

// -----------------------------------------------------
// ADC freeze monitoring (test simulation)
// -----------------------------------------------------
void monitorADC() {
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  int raw = analogRead(CELL_PINS[0]);
  int raw2 = analogRead(CELL_PINS[1]);
  int raw3 = analogRead(CELL_PINS[2]);
  if (simulateADCFrozen) raw = frozenADCValue;
  adcValue = raw;
  updateCellVoltageReadings(raw, raw2, raw3);

  if (previousADC < 0) {
    previousADC = raw;
    lastADCChange = now;
    Serial.printf("[ADC] Initial reading: %d\r\n", raw);
    return;
  }

  if (raw != previousADC) {
    previousADC = raw;
    lastADCChange = now;
  } else if (simulateADCFrozen &&
             now - lastADCChange >= ADC_FREEZE_TIME_MS) {
    FaultRecord* adcFault = findFault(F002_ADC);
    if (adcFault && !adcFault->active) {
      injectFault(F002_ADC);
    }
    // Do not reset lastADCChange; this prevents repeated detection logs.
  }

  Serial.printf("[ADC] Cell1=%d, freeze_test=%s\r\n",
                raw, simulateADCFrozen ? "ON" : "OFF");
}

// -----------------------------------------------------
// Relay mismatch monitoring (feedback is simulated)
// -----------------------------------------------------
void monitorRelay() {
  unsigned long now = millis();

  // No new feedback wire is used in this software simulation.
  simulatedRelayFeedback =
      simulateRelayMismatch ? !relayCommand : relayCommand;

  FaultRecord* relayFault = findFault(F003_RELAY);
  bool relayFaultAlreadyActive = relayFault && relayFault->active;

  if (relayCommand != simulatedRelayFeedback) {
    if (!relayMismatchTimerStarted && !relayFaultAlreadyActive) {
      relayMismatchStart = now;
      relayMismatchTimerStarted = true;
      Serial.println("[RELAY] Mismatch timer started.");
    }

    if (relayMismatchTimerStarted &&
        now - relayMismatchStart >= RELAY_MISMATCH_TIME_MS &&
        !relayFaultAlreadyActive) {
      relayMismatchTimerStarted = false;
      injectFault(F003_RELAY);
    }
  } else {
    relayMismatchTimerStarted = false;
  }
}

// -----------------------------------------------------
// Serial commands
// -----------------------------------------------------
void printMenu() {
  Serial.println();
  Serial.println("===== TASK 4 + TASK 5 =====");
  Serial.println("z = Inject frozen ADC test");
  Serial.println("n = Stop frozen ADC test");
  Serial.println("v = Inject relay mismatch test");
  Serial.println("b = Stop relay mismatch test");
  Serial.println("o = Command relay OFF");
  Serial.println("p = Command relay ON");
  Serial.println("d = Inject battery fault");
  Serial.println("f = Inject ADC fault manually");
  Serial.println("r = Inject relay fault manually");
  Serial.println("m = Inject communication fault");
  Serial.println("s = Inject system shutdown fault");
  Serial.println("1 = Clear battery fault");
  Serial.println("2 = Clear ADC fault");
  Serial.println("3 = Clear relay fault");
  Serial.println("4 = Clear communication fault");
  Serial.println("x = Reset shutdown (test only)");
  Serial.println("i = Print state and faults");
  Serial.println("Recovery verification time = 3000 ms");
  Serial.println("Task 5: Blynk event telemetry + 32-event RAM queue");
  Serial.println("==========================");
}

void printStatus() {
  Serial.printf("[STATUS] State=%s, ShutdownLatched=%s, RecoveryPending=%s\r\n",
                stateName(currentState),
                shutdownLatched ? "YES" : "NO",
                recoveryPending ? "YES" : "NO");
  Serial.printf("[RELAY] Command=%s, SimulatedFeedback=%s\r\n",
                relayCommand ? "ON" : "OFF",
                simulatedRelayFeedback ? "ON" : "OFF");
  printFaults();
}

// TASK 6: analytics, recommendations, and dashboard publishing
void updateTask6Analytics() {
  float highest = cellVoltage[0], lowest = cellVoltage[0], average = 0.0f;
  for (int i = 0; i < 3; i++) {
    if (cellVoltage[i] > highest) highest = cellVoltage[i];
    if (cellVoltage[i] < lowest) lowest = cellVoltage[i];
    average += cellVoltage[i];
  }
  average /= 3.0f;
  task6Imbalance = highest - lowest;

  // Demo-only proxy based on 0-3.3 V potentiometer scaling, not real battery SoC.
  task6SoC = (average / 3.3f) * 100.0f;
  if (task6SoC < 0.0f) task6SoC = 0.0f;
  if (task6SoC > 100.0f) task6SoC = 100.0f;

  float imbalanceRisk = (task6Imbalance / 0.50f) * 100.0f;
  if (imbalanceRisk > 100.0f) imbalanceRisk = 100.0f;
  if (imbalanceRisk < 0.0f) imbalanceRisk = 0.0f;
  float faultRisk = (float)cumulativeFaultCount * 10.0f;
  if (faultRisk > 100.0f) faultRisk = 100.0f;
  float socRisk = 100.0f - task6SoC;
  task6RiskScore = 0.40f * imbalanceRisk + 0.35f * faultRisk + 0.25f * socRisk;
  if (task6RiskScore < 0.0f) task6RiskScore = 0.0f;
  if (task6RiskScore > 100.0f) task6RiskScore = 100.0f;
}

const char* task6Recommendation() {
  if (currentState == SHUTDOWN) return "SHUTDOWN LATCHED: inspect system before safe reset";
  if (currentState == FAILSAFE) {
    FaultID id = highestPriorityFault();
    if (id == F002_ADC) return "FAILSAFE: inspect ADC/sensor wiring and verify readings";
    if (id == F003_RELAY) return "FAILSAFE: inspect relay command and feedback";
    return "FAILSAFE: isolate fault and verify system before recovery";
  }
  if (task6Imbalance >= 0.25f) return "High cell imbalance: inspect readings and connections";
  if (task6SoC < 25.0f) return "Estimated charge low: check simulated cell inputs";
  if (anyFaultActive()) return "Fault active: inspect fault source and resolve it";
  if (task6RiskScore >= 50.0f) return "Elevated risk: inspect imbalance and recent faults";
  return "No active fault: continue monitoring cell trends and state";
}

const char* task6BatteryHealth() {
  if (currentState == SHUTDOWN) return "SHUTDOWN";
  if (currentState == FAILSAFE) return "CRITICAL";
  if (currentState == DEGRADED) return "ATTENTION";
  if (task6RiskScore >= 75.0f) return "CRITICAL RISK";
  if (task6RiskScore >= 50.0f) return "POOR";
  if (task6RiskScore >= 25.0f) return "FAIR";
  return "GOOD";
}

void publishTask6Dashboard() {
  if (!Blynk.connected()) return;
  updateTask6Analytics();
  uint32_t uptimeSeconds = (uint32_t)((millis() - systemStartMs) / 1000UL);
  Blynk.virtualWrite(V11, task6RiskScore);
  Blynk.virtualWrite(V12, stateName(currentState));
  Blynk.virtualWrite(V13, uptimeSeconds);
  Blynk.virtualWrite(V14, cumulativeFaultCount);
  Blynk.virtualWrite(V15, task6Imbalance);
  Blynk.virtualWrite(V16, task6SoC);
  Blynk.virtualWrite(V17, task6Recommendation());
  Blynk.virtualWrite(V18, latestTransition);
  Blynk.virtualWrite(V19, task6BatteryHealth());
  // Task 6: State severity indicators
Blynk.virtualWrite(V20, currentState == NORMAL ? 1 : 0);
Blynk.virtualWrite(V21, currentState == DEGRADED ? 1 : 0);
Blynk.virtualWrite(V22, currentState == FAILSAFE ? 1 : 0);
Blynk.virtualWrite(V23, currentState == SHUTDOWN ? 1 : 0);
  Serial.printf("[TASK6] Risk=%.1f Imbalance=%.3fV DemoSoC=%.1f%% Faults=%lu State=%s\r\n",
                task6RiskScore, task6Imbalance, task6SoC,
                (unsigned long)cumulativeFaultCount, stateName(currentState));
}

void setup() {
  Serial.begin(115200);

  // LCD initialization is separate from Blynk and does not alter telemetry.
  Wire.begin(21, 22); // ESP32 default I2C wiring used by this patch
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EV BMS MONITOR");
  lcd.setCursor(0, 1);
  lcd.print("State: NORMAL");

  systemStartMs = millis();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  analogReadResolution(12);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS,6);
  lastWifiAttempt = millis();
  Blynk.config(BLYNK_AUTH_TOKEN);
  Serial.println("[TASK 5] Wi-Fi/Blynk telemetry initialized.");

  Serial.println();
  Serial.println("TASK 4 - STAGE 4");
  Serial.println("Structured Logging + Verified Fault Recovery");
  Serial.println("[INIT] State = NORMAL");
  logEvent("SYSTEM_INITIALIZED", NO_FAULT, "stage4_ready");
  printMenu();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    switch (c) {
      case 'z':
        frozenADCValue = analogRead(CELL_PINS[0]);
        simulateADCFrozen = true;
        previousADC = frozenADCValue;
        lastADCChange = millis();
        logEvent("ADC_FREEZE_TEST_STARTED", F002_ADC,
                 "software_simulation_enabled");
        Serial.printf("[TEST] ADC freeze injected at %d\r\n",
                      frozenADCValue);
        break;

      case 'n':
        simulateADCFrozen = false;
        clearFault(F002_ADC);
        logEvent("ADC_FREEZE_TEST_STOPPED", F002_ADC,
                 "software_simulation_disabled");
        Serial.println("[TEST] ADC freeze disabled.");
        break;

      case 'v':
        simulateRelayMismatch = true;
        relayMismatchTimerStarted = false;
        logEvent("RELAY_MISMATCH_TEST_STARTED", F003_RELAY,
                 "software_feedback_mismatch_enabled");
        Serial.println("[TEST] Relay mismatch injected.");
        break;

      case 'b':
        simulateRelayMismatch = false;
        relayMismatchTimerStarted = false;
        clearFault(F003_RELAY);
        logEvent("RELAY_MISMATCH_TEST_STOPPED", F003_RELAY,
                 "software_feedback_mismatch_disabled");
        Serial.println("[TEST] Relay mismatch disabled.");
        break;

      case 'o':
        relayCommand = false;
        digitalWrite(RELAY_PIN, LOW);
        logEvent("RELAY_COMMAND", F003_RELAY, "OFF");
        Serial.println("[RELAY] Commanded OFF.");
        break;

      case 'p':
        relayCommand = true;
        digitalWrite(RELAY_PIN, HIGH);
        logEvent("RELAY_COMMAND", F003_RELAY, "ON");
        Serial.println("[RELAY] Commanded ON.");
        break;

      case 'd': injectFault(F001_BATTERY); break;
      case 'f': injectFault(F002_ADC); break;
      case 'r': injectFault(F003_RELAY); break;
      case 'm': injectFault(F004_COMMUNICATION); break;
      case 's': injectFault(F005_SYSTEM); break;

      case '1': clearFault(F001_BATTERY); break;
      case '2':
        simulateADCFrozen = false;
        clearFault(F002_ADC);
        break;
      case '3':
        simulateRelayMismatch = false;
        relayMismatchTimerStarted = false;
        clearFault(F003_RELAY);
        break;
      case '4': clearFault(F004_COMMUNICATION); break;
      case 'x': resetShutdown(); break;
      case 'i': printStatus(); break;

      case '\r':
      case '\n':
        break;

      default:
        Serial.println("[INFO] Unknown command.");
        break;
    }
  }

  monitorADC();
  monitorRelay();
  serviceRecovery();
  serviceConnectivity();
  if (Blynk.connected()) Blynk.run();
  serviceTelemetry();

  unsigned long now = millis();
  if (now - lastStatus >= STATUS_INTERVAL_MS) {
    lastStatus = now;
    printStatus();
  }

  if (Blynk.connected() && now - lastTask6Publish >= TASK6_PUBLISH_INTERVAL_MS) {
    lastTask6Publish = now;
    publishTask6Dashboard();
  }

  updateLCD();
  updateGreenStatusLED();
}
