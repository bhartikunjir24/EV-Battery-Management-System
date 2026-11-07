#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// TASK 3: FLICKER-FREE LCD DISPLAY ENGINE
// ESP32 DevKit C V4 + Wokwi / PlatformIO
// =====================================================

// LCD configuration
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Three simulated battery cells (potentiometers)
const int CELL1_PIN = 34;
const int CELL2_PIN = 35;
const int CELL3_PIN = 32;

// Connect a pushbutton between GPIO25 and GND
const int FAULT_CLEAR_BUTTON = 25;

// Timing configuration
const unsigned long LCD_REFRESH_MS = 200;
const unsigned long PAGE_INTERVAL_MS = 3000;
const unsigned long BUTTON_DEBOUNCE_MS = 40;

// Demonstration threshold only; not a validated battery limit
const float CRITICAL_MIN_VOLTAGE = 2.50f;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Screen buffers
char previousScreen[LCD_ROWS][LCD_COLUMNS + 1];
char currentScreen[LCD_ROWS][LCD_COLUMNS + 1];

// Timers
unsigned long lastLCDRefresh = 0;
unsigned long lastPageChange = 0;
unsigned long lastButtonChange = 0;

// Page: 0 = Battery, 1 = System, 2 = Telemetry
int currentPage = 0;

// Critical fault latch
bool criticalFaultLatched = false;

// Button debounce state
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

// Latest measurements
float cell1 = 0.0f;
float cell2 = 0.0f;
float cell3 = 0.0f;
float soc = 0.0f;

// =====================================================
// READ SIMULATED CELL VOLTAGE
// =====================================================
float readCellVoltage(int pin) {
  const int raw = analogRead(pin);
  return (raw / 4095.0f) * 4.20f;
}

// =====================================================
// PREPARE FIXED-WIDTH LCD LINE
// =====================================================
void prepareLine(int row, const char *message) {
  for (int col = 0; col < LCD_COLUMNS; col++) {
    currentScreen[row][col] = ' ';
  }

  currentScreen[row][LCD_COLUMNS] = '\0';

  for (int col = 0;
       col < LCD_COLUMNS && message[col] != '\0';
       col++) {
    currentScreen[row][col] = message[col];
  }
}

// =====================================================
// FLICKER-FREE RENDERING
// Writes only contiguous regions of changed characters
// =====================================================
void renderLCD() {
  for (int row = 0; row < LCD_ROWS; row++) {
    int col = 0;

    while (col < LCD_COLUMNS) {
      if (currentScreen[row][col] ==
          previousScreen[row][col]) {
        col++;
        continue;
      }

      const int startCol = col;

      while (col < LCD_COLUMNS &&
             currentScreen[row][col] !=
             previousScreen[row][col]) {
        col++;
      }

      lcd.setCursor(startCol, row);

      for (int i = startCol; i < col; i++) {
        lcd.write(static_cast<uint8_t>(currentScreen[row][i]));
        previousScreen[row][i] = currentScreen[row][i];
      }
    }
  }
}

// =====================================================
// SAMPLE CELLS AND UPDATE FAULT LATCH
// =====================================================
void updateMeasurements() {
  cell1 = readCellVoltage(CELL1_PIN);
  cell2 = readCellVoltage(CELL2_PIN);
  cell3 = readCellVoltage(CELL3_PIN);

  const float averageVoltage = (cell1 + cell2 + cell3) / 3.0f;

  // Illustrative voltage-based estimate, not calibrated SoC
  soc = constrain((averageVoltage / 4.20f) * 100.0f,
                  0.0f, 100.0f);

  const bool lowCell =
      cell1 < CRITICAL_MIN_VOLTAGE ||
      cell2 < CRITICAL_MIN_VOLTAGE ||
      cell3 < CRITICAL_MIN_VOLTAGE;

  // Latch the fault until the user explicitly clears it.
  if (lowCell && !criticalFaultLatched) {
    criticalFaultLatched = true;

    Serial.println("[FAULT] Critical low cell detected");
    Serial.println("[STATE] Fault screen activated");
  }
}

// =====================================================
// NON-BLOCKING DEBOUNCED CLEAR BUTTON
// =====================================================
void checkFaultClearButton() {
  const unsigned long now = millis();
  const bool reading = digitalRead(FAULT_CLEAR_BUTTON);

  if (reading != lastButtonReading) {
    lastButtonChange = now;
    lastButtonReading = reading;
  }

  if (now - lastButtonChange >= BUTTON_DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      // INPUT_PULLUP: pressed = LOW
      if (stableButtonState == LOW) {
        const bool allCellsRecovered =
            cell1 >= CRITICAL_MIN_VOLTAGE &&
            cell2 >= CRITICAL_MIN_VOLTAGE &&
            cell3 >= CRITICAL_MIN_VOLTAGE;

        if (criticalFaultLatched && allCellsRecovered) {
          criticalFaultLatched = false;

          // Restart at Page 1 and begin a fresh 3-second interval.
          currentPage = 0;
          lastPageChange = now;

          Serial.println("[FAULT] Explicitly cleared by button");
          Serial.println("[STATE] Returning to normal pages");
        } else if (criticalFaultLatched) {
          Serial.println("[FAULT] Clear rejected: cell voltage low");
        } else {
          Serial.println("[FAULT] No active fault");
        }
      }
    }
  }
}

// =====================================================
// PREPARE CURRENT PAGE
// =====================================================
void updateDisplayContent() {
  char line1[17];
  char line2[17];

  // Critical fault screen always takes priority.
  if (criticalFaultLatched) {
    snprintf(line1, sizeof(line1), "!! CRITICAL !!");
    snprintf(line2, sizeof(line2), "CELL VOLTAGE LOW");
  } else if (currentPage == 0) {
    // PAGE 1: Battery status
    snprintf(line1, sizeof(line1), "BATTERY STATUS");
    snprintf(line2, sizeof(line2), "SOC: %d%%",
             static_cast<int>(soc + 0.5f));
  } else if (currentPage == 1) {
    // PAGE 2: System state
    snprintf(line1, sizeof(line1), "SYSTEM STATE");

    const bool lowCell =
        cell1 < CRITICAL_MIN_VOLTAGE ||
        cell2 < CRITICAL_MIN_VOLTAGE ||
        cell3 < CRITICAL_MIN_VOLTAGE;

    snprintf(line2, sizeof(line2), "%s",
             lowCell ? "STATE: LOW CELL" : "STATE: NORMAL");
  } else {
    // PAGE 3: Cell telemetry
    snprintf(line1, sizeof(line1),
             "C1:%1.2f C2:%1.2f", cell1, cell2);
    snprintf(line2, sizeof(line2), "C3:%1.2f V", cell3);
  }

  prepareLine(0, line1);
  prepareLine(1, line2);
  renderLCD();
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(FAULT_CLEAR_BUTTON, INPUT_PULLUP);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  // Force initial rendering by making previous characters differ.
  for (int row = 0; row < LCD_ROWS; row++) {
    for (int col = 0; col < LCD_COLUMNS; col++) {
      previousScreen[row][col] = '\0';
      currentScreen[row][col] = ' ';
    }

    previousScreen[row][LCD_COLUMNS] = '\0';
    currentScreen[row][LCD_COLUMNS] = '\0';
  }

  updateMeasurements();

  prepareLine(0, "TASK 3 LCD");
  prepareLine(1, "INITIALIZING...");
  renderLCD();

  const unsigned long now = millis();
  lastLCDRefresh = now;
  lastPageChange = now;
  lastButtonChange = now;

  Serial.println("TASK 3: FINAL LCD ENGINE");
  Serial.println("Stage 1: Changed-character rendering");
  Serial.println("Stage 2: Three rotating pages");
  Serial.println("Stage 3: Critical fault override");
  Serial.println("Stage 4: Timing optimization");
  Serial.println("LCD refresh: 200 ms");
  Serial.println("Page interval: 3000 ms");
  Serial.println("Fault-clear button: GPIO25 to GND");
  Serial.println("Press clear button after all cells recover");
}

// =====================================================
// MAIN LOOP: NON-BLOCKING; NO delay()
// =====================================================
void loop() {
  const unsigned long now = millis();

  // Check the clear button continuously.
  checkFaultClearButton();

  // Periodic measurements and display refresh.
  if (now - lastLCDRefresh >= LCD_REFRESH_MS) {
    lastLCDRefresh = now;

    updateMeasurements();
    updateDisplayContent();

    Serial.printf(
        "C1: %.2f V | C2: %.2f V | C3: %.2f V | "
        "SOC: %.0f%% | Page: %d | Fault: %s\r\n",
        cell1, cell2, cell3, soc, currentPage + 1,
        criticalFaultLatched ? "ACTIVE" : "CLEAR"
    );
  }

  // Rotate pages only when no critical fault is latched.
  if (!criticalFaultLatched &&
      now - lastPageChange >= PAGE_INTERVAL_MS) {
    lastPageChange = now;
    currentPage = (currentPage + 1) % 3;

    Serial.printf("[LCD] Page changed to: %d\r\n",
                  currentPage + 1);

    updateDisplayContent();
  }
}
