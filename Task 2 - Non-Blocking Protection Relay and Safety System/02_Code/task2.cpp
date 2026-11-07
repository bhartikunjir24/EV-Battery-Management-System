

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);


// ==========================================
// GPIO PINS
// ==========================================
const int NUM_CELLS = 3;
const int cellPins[NUM_CELLS] = {34, 35, 32};


#define GREEN_LED 18
#define YELLOW_LED 19
#define RED_LED 23
#define BUZZER 5
#define RELAY 4


// ==========================================
// NON-BLOCKING TIMERS
// ==========================================
const unsigned long UPDATE_INTERVAL = 500;
const unsigned long LOG_INTERVAL = 2000;
const unsigned long DEBOUNCE_TIME = 1000;
const unsigned long RECOVERY_TIME = 3000; // Stage 5: safe readings must persist for 3 seconds


unsigned long lastUpdateTime = 0;
unsigned long lastLogTime = 0;
unsigned long stateStartTime = 0;


// ==========================================
// BATTERY VARIABLES
// ==========================================
float cellVoltage[NUM_CELLS] = {0};
const float TRIP_THRESHOLD = 0.30;
const float CLEAR_THRESHOLD = 0.20;


// ==========================================
// STAGE 3: ANOMALY SETTINGS
// ==========================================
const float MIN_VALID_VOLTAGE = 0.10;
const float MAX_VALID_VOLTAGE = 3.30;
const float MAX_VOLTAGE_JUMP = 0.80;


const int WINDOW_SIZE = 5;


// ==========================================
// STAGE 4: MOVING-AVERAGE FILTER
// ==========================================
float filteredVoltage[NUM_CELLS] = {0};
float filterHistory[NUM_CELLS][WINDOW_SIZE] = {{0}};
int filterIndex = 0;
int filterCount = 0;
const float FROZEN_VARIATION_LIMIT = 0.005;
const float MEANINGFUL_CHANGE_LIMIT = 0.02;

// Statistical change classification (does not replace safety protection).
const float SMALL_VARIATION_LIMIT = 0.03;
const float SIGNIFICANT_DEVIATION_LIMIT = 0.05;
const int SUSTAINED_CHANGE_SAMPLES = 3;
int significantDeviationCount[NUM_CELLS] = {0};
const char* changeClassification[NUM_CELLS] = {"INITIALIZING", "INITIALIZING", "INITIALIZING"};


// Previous readings for jump detection
float previousVoltage[NUM_CELLS] = {0};
bool previousReadingAvailable[NUM_CELLS] = {false};


// Five-sample history for frozen detection
float voltageHistory[NUM_CELLS][WINDOW_SIZE] = {{0}};
int historyIndex = 0;
int historyCount = 0;


// Current anomaly status
bool sensorAnomaly = false;


// Prevent the same anomaly message from printing
// repeatedly while the condition remains present.
bool outOfRangeReported[NUM_CELLS] = {false};
bool jumpReported[NUM_CELLS] = {false};
bool frozenReported[NUM_CELLS] = {false};


// ==========================================
// PROTECTION STATE MACHINE
// ==========================================
enum ProtectionState {
  NORMAL,
  DEBOUNCE,
  TRIPPED,
  RECOVERY
};


ProtectionState protectionState = NORMAL;


// ==========================================
// STATE NAME
// ==========================================
const char* getStateName() {
  switch (protectionState) {
    case NORMAL:   return "NORMAL";
    case DEBOUNCE: return "DEBOUNCE";
    case TRIPPED:  return "TRIPPED";
    case RECOVERY: return "RECOVERY";
    default:       return "UNKNOWN";
  }
}


// ==========================================
// CHANGE STATE AND LOG TRANSITION
// ==========================================
void changeState(ProtectionState newState) {
  if (protectionState != newState) {
    protectionState = newState;
    stateStartTime = millis();


    Serial.print("[STATE] -> ");
    Serial.println(getStateName());
  }
}


// ==========================================
// READ CELL VOLTAGES
// ==========================================
void readCellVoltages() {
  for (int i = 0; i < NUM_CELLS; i++) {
    int adc = analogRead(cellPins[i]);
    cellVoltage[i] = adc * (3.3f / 4095.0f);
  }
}
// ==========================================
// STAGE 4: UPDATE MOVING AVERAGE
// ==========================================
void updateMovingAverage() {
  // Store the latest raw reading for each cell.
  for (int i = 0; i < NUM_CELLS; i++) {
    filterHistory[i][filterIndex] = cellVoltage[i];
  }


  // Advance the circular window.
  filterIndex = (filterIndex + 1) % WINDOW_SIZE;


  if (filterCount < WINDOW_SIZE) {
    filterCount++;
  }


  // Calculate the average of available samples.
  for (int i = 0; i < NUM_CELLS; i++) {
    float sum = 0;


    for (int j = 0; j < filterCount; j++) {
      sum += filterHistory[i][j];
    }


    filteredVoltage[i] = sum / filterCount;
  }
}


// ==========================================
// STAGE 4: STATISTICAL CHANGE CLASSIFICATION
// ==========================================
void classifyVoltageChanges() {
  for (int i = 0; i < NUM_CELLS; i++) {
    // Compare raw input with the moving average of the recent sample window.
    float deviation = fabsf(cellVoltage[i] - filteredVoltage[i]);

    if (deviation <= SMALL_VARIATION_LIMIT) {
      changeClassification[i] = "SMALL VARIATION";
      significantDeviationCount[i] = 0;
    } else if (deviation < SIGNIFICANT_DEVIATION_LIMIT) {
      changeClassification[i] = "UNCERTAIN";
      significantDeviationCount[i] = 0;
    } else {
      if (significantDeviationCount[i] < SUSTAINED_CHANGE_SAMPLES) {
        significantDeviationCount[i]++;
      }

      if (significantDeviationCount[i] >= SUSTAINED_CHANGE_SAMPLES) {
        changeClassification[i] = "SUSTAINED CHANGE";
      } else {
        changeClassification[i] = "CHANGE CANDIDATE";
      }
    }
  }
}


// ==========================================
// BATTERY CALCULATIONS
// ==========================================
float calculateAverageVoltage() {
  float sum = 0;


  for (int i = 0; i < NUM_CELLS; i++) {
    sum += filteredVoltage[i];
  }


  return sum / NUM_CELLS;
}


float getHighestVoltage() {
  float highest = filteredVoltage[0];


  for (int i = 1; i < NUM_CELLS; i++) {
    if (filteredVoltage[i] > highest) {
      highest = filteredVoltage[i];
    }
  }


  return highest;
}


float getLowestVoltage() {
  float lowest = filteredVoltage[0];


  for (int i = 1; i < NUM_CELLS; i++) {
    if (filteredVoltage[i] < lowest) {
      lowest = filteredVoltage[i];
    }
  }


  return lowest;
}


// ==========================================
// STAGE 3: SENSOR ANOMALY DETECTION
// ==========================================
void checkSensorAnomalies() {
  bool currentAnomaly = false;
  bool outOfRange[NUM_CELLS] = {false};
  bool unrealisticJump[NUM_CELLS] = {false};


  // 1. OUT-OF-RANGE AND UNREALISTIC-JUMP CHECKS
  for (int i = 0; i < NUM_CELLS; i++) {
    // Out-of-range check
    if (cellVoltage[i] < MIN_VALID_VOLTAGE ||
        cellVoltage[i] > MAX_VALID_VOLTAGE) {


      outOfRange[i] = true;
      currentAnomaly = true;


      if (!outOfRangeReported[i]) {
        Serial.print("[ANOMALY] Out-of-range reading - Cell ");
        Serial.println(i + 1);
      }
    }


    outOfRangeReported[i] = outOfRange[i];


    // Unrealistic jump check
    if (previousReadingAvailable[i] &&
        fabsf(cellVoltage[i] - previousVoltage[i])
            > MAX_VOLTAGE_JUMP) {


      unrealisticJump[i] = true;
      currentAnomaly = true;


      if (!jumpReported[i]) {
        Serial.print("[ANOMALY] Unrealistic voltage jump - Cell ");
        Serial.println(i + 1);
      }
    }


    jumpReported[i] = unrealisticJump[i];
  }


  // 2. STORE READINGS IN THE FIVE-SAMPLE WINDOW
  for (int i = 0; i < NUM_CELLS; i++) {
    voltageHistory[i][historyIndex] = cellVoltage[i];
  }


  historyIndex = (historyIndex + 1) % WINDOW_SIZE;


  if (historyCount < WINDOW_SIZE) {
    historyCount++;
  }


  // 3. FROZEN-SENSOR CHECK
  // Compare window variation. Require both other cells to vary before
  // suspecting a frozen sensor, reducing false positives in the simulator.
  bool frozenNow[NUM_CELLS] = {false};


  if (historyCount == WINDOW_SIZE) {
    float variation[NUM_CELLS] = {0};


    for (int cell = 0; cell < NUM_CELLS; cell++) {
      float minValue = voltageHistory[cell][0];
      float maxValue = voltageHistory[cell][0];


      for (int j = 1; j < WINDOW_SIZE; j++) {
        if (voltageHistory[cell][j] < minValue) {
          minValue = voltageHistory[cell][j];
        }


        if (voltageHistory[cell][j] > maxValue) {
          maxValue = voltageHistory[cell][j];
        }
      }


      variation[cell] = maxValue - minValue;
    }


    for (int i = 0; i < NUM_CELLS; i++) {
      int otherCellsChanging = 0;


      for (int j = 0; j < NUM_CELLS; j++) {
        if (i != j &&
            variation[j] >= MEANINGFUL_CHANGE_LIMIT) {
          otherCellsChanging++;
        }
      }


      // With three independently adjustable simulated cells, one cell
      // changing alone is not enough evidence that the others are frozen.
      // Suspect a frozen sensor only when BOTH other cells are changing.
      if (variation[i] < FROZEN_VARIATION_LIMIT &&
          otherCellsChanging == NUM_CELLS - 1) {


        frozenNow[i] = true;
        currentAnomaly = true;


        if (!frozenReported[i]) {
          Serial.print("[ANOMALY] Frozen sensor suspected - Cell ");
          Serial.println(i + 1);
        }
      }
    }
  }


  for (int i = 0; i < NUM_CELLS; i++) {
    frozenReported[i] = frozenNow[i];
  }


  // 4. UPDATE PREVIOUS READINGS
  // Ignore out-of-range readings as jump baselines.
  for (int i = 0; i < NUM_CELLS; i++) {
    if (!outOfRange[i]) {
      previousVoltage[i] = cellVoltage[i];
      previousReadingAvailable[i] = true;
    }
  }


  sensorAnomaly = currentAnomaly;
}


// ==========================================
// STAGE 3: PROTECTION WITH ANOMALY INPUT
// ==========================================
void updateProtection(float imbalance) {
  unsigned long now = millis();


  bool imbalanceFault = imbalance > TRIP_THRESHOLD;


  // Sensor anomalies also request protection.
  bool faultPresent = imbalanceFault || sensorAnomaly;


  // Reset requires safe imbalance AND healthy readings.
  bool faultCleared =
      imbalance < CLEAR_THRESHOLD && !sensorAnomaly;


  switch (protectionState) {


    case NORMAL:
      if (faultPresent) {
        if (sensorAnomaly) {
          Serial.println("[FAULT] Sensor anomaly detected");
        }


        if (imbalanceFault) {
          Serial.println("[FAULT] Imbalance above trip threshold");
        }


        Serial.println("[DEBOUNCE] Confirmation timer started");
        changeState(DEBOUNCE);
      }
      break;


    case DEBOUNCE:
      if (!faultPresent) {
        Serial.println("[DEBOUNCE] Fault cleared before confirmation");
        changeState(NORMAL);
      }
      else if (now - stateStartTime >= DEBOUNCE_TIME) {
        Serial.println("[PROTECTION] Fault confirmed");
        changeState(TRIPPED);
      }
      break;


    case TRIPPED:
      if (faultCleared) {
        Serial.println("[RECOVERY] Fault cleared; starting timed recovery");
        changeState(RECOVERY);
      }
      break;

    case RECOVERY:
      // Keep relay disabled throughout recovery. Any returning fault cancels recovery.
      if (faultPresent) {
        Serial.println("[RECOVERY] Fault returned; recovery cancelled");
        changeState(TRIPPED);
      }
      else if (now - stateStartTime >= RECOVERY_TIME) {
        Serial.println("[RECOVERY] Safe readings maintained; restoring normal operation");
        changeState(NORMAL);
      }
      break;
  }
}


// ==========================================
// UPDATE OUTPUTS
// ==========================================
void updateOutputs() {
  switch (protectionState) {


    case NORMAL:
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);


      ledcWrite(0, 0);
      digitalWrite(RELAY, HIGH);
      break;


    case DEBOUNCE:
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);


      ledcWrite(0, 0);
      digitalWrite(RELAY, HIGH);
      break;


    case TRIPPED:
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, HIGH);

      ledcWrite(0, 128);
      digitalWrite(RELAY, LOW);
      break;

    case RECOVERY:
      // Yellow indicates recovery is in progress; keep relay disconnected.
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);

      ledcWrite(0, 0);
      digitalWrite(RELAY, LOW);
      break;
  }
}


// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);


  lcd.init();
  lcd.backlight();


  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);


  // ESP32 LEDC buzzer configuration
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZER, 0);
  ledcWrite(0, 0);


  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(RELAY, LOW);


  lcd.setCursor(0, 0);
  lcd.print("EV Battery BMS");


  Serial.println("==============================");
  Serial.println("TASK 2 FINAL: NON-BLOCKING EV BMS PROTECTION SYSTEM");
  Serial.println("==============================");
  Serial.println("[STATE] -> NORMAL");
}


// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();


  if (now - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = now;


    readCellVoltages();


    // Stage 3: Check raw sensor readings.
    checkSensorAnomalies();


    // Stage 4: Filter readings for battery calculations.
    updateMovingAverage();

    // Compare each raw reading with its moving average to classify
    // small variation versus a persistent significant change.
    classifyVoltageChanges();


    float averageVoltage = calculateAverageVoltage();
    float highestVoltage = getHighestVoltage();
    float lowestVoltage = getLowestVoltage();
    float imbalance = highestVoltage - lowestVoltage;


    updateProtection(imbalance);
    updateOutputs();


    // LCD display
    lcd.setCursor(0, 0);
    lcd.print("Imbalance:");
    lcd.print(imbalance, 2);
    lcd.print("V   ");


    lcd.setCursor(0, 1);
    lcd.print("State: ");
    lcd.print(getStateName());
    lcd.print("       ");


    // Periodic Serial Monitor logging
    if (now - lastLogTime >= LOG_INTERVAL) {
      lastLogTime = now;


      Serial.println("------------------------------");


      for (int i = 0; i < NUM_CELLS; i++) {
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.print(" Raw: ");
        Serial.print(cellVoltage[i], 3);
        Serial.print(" V | Filtered: ");
        Serial.print(filteredVoltage[i], 3);
        Serial.println(" V");

        Serial.print("  Change classification: ");
        Serial.println(changeClassification[i]);
      }


      Serial.print("Average Voltage (filtered): ");
      Serial.println(averageVoltage, 2);


      Serial.print("Highest Voltage: ");
      Serial.println(highestVoltage, 2);


      Serial.print("Lowest Voltage: ");
      Serial.println(lowestVoltage, 2);


      Serial.print("Imbalance: ");
      Serial.println(imbalance, 2);


      Serial.print("Sensor Anomaly: ");
      Serial.println(sensorAnomaly ? "YES" : "NO");


      Serial.print("Protection State: ");
      Serial.println(getStateName());
    }
  }
}



