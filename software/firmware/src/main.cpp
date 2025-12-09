/**
 * @file main.cpp
 * @brief ATtiny1616 firmware for TheSun lamp controller.
 *
 * Responsibilities:
 *  - Drive an LED on PB5 (Arduino pin 4) with PWM using TCA0 WO2.
 *  - Read an AHT20 temperature/humidity sensor over I2C.
 *  - Optionally read one or two NTC thermistors on analog inputs.
 *  - Handle four push-buttons with debounce and percentage steps.
 *  - Present status on a 128x64 SSD1306 OLED.
 *
 * Notes:
 *  - takeOverTCA0() is called; TCA0 is removed from core control. Ensure
 *    millis/timebase does not depend on TCA0 (prefer TCB/RTC in core opts).
 *  - PWM routing to PB5 depends on build flag -DTCA_PORTMUX=PORTMUX_TCA02_bm.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Adafruit_AHTX0.h>

#define I2C_ADDRESS         0x3C  ///< SSD1306 I2C address

//#define NTC1                2
#define NTC2                3

#define FP16(__var__)       static_cast<float>(__var__)

#define LED_PWM_PIN         4              ///< Arduino pin for LED (PB5 / WO2)
#define REF_RES             FP16(10000.0f) ///< Series resistor for NTC (ohms)
#define NOM_RES             FP16(10000.0f) ///< NTC nominal resistance at 25C
#define NOM_TMP             FP16(25.0f)    ///< NTC nominal temperature (C)
#define BETA                3950           ///< NTC beta coefficient
#define ADC_RES             FP16(1023.0f)  ///< 10-bit ADC max count
#define NTC_BUFFER          100            ///< Samples to average per NTC read
#define TO_KELVIN(__C__)    FP16((FP16(__C__)) + 273.15f)
#define TO_CELSIUS(__K__)   FP16((FP16(__K__)) - 273.15f)

#define P0                  0
#define P1                  100
#define MIN_P0              2
#define MAX_P1              50

// Define switch pins
#define SWITCH_A_PIN        10 // PC0
#define SWITCH_B_PIN        11 // PC1
#define SWITCH_C_PIN        12 // PC2
#define SWITCH_D_PIN        13 // PC3

// Timing constants
#define DEBOUNCE_DELAY      50    ///< ms between switch debouncing checks
#define TEMP_READ_DELAY     5000  ///< ms between sensor reads
#define SCREEN_UPDATE_DELAY 5000  ///< ms between OLED updates (unless forced)

SSD1306AsciiWire display;
Adafruit_AHTX0 aht;

/** Claim and configure TCA0 for single-slope PWM on WO2 (PB5). */
void configureTimer(void);
/** Read NTC temperature (Celsius) using a Steinhart–Hart approximation. */
[[nodiscard]] float readTemperatureNTC(const int pin);
/** Map user percentage [P0..P1] to output clamp [MIN_P0..MAX_P1]. */
[[nodiscard]] uint8_t mapPercentage(int8_t x);
/** Debounce and handle a switch; updates percentage when pressed. */
[[nodiscard]] bool handleSwitch(int pin, int increment, uint32_t& lastTime, bool& lastState);
/** Refresh the OLED contents. */
void updateScreen(void);
/** Read AHT20 and optional NTC sensors. */
void readTemperatures(void);
/** Write duty (0–255) to TCA0 WO2. */
void setLedPwm(const uint8_t duty);

// Variables
int8_t percentage         = P0;
uint8_t mapped_percentage = MIN_P0;
uint8_t pwmValue          = (mapped_percentage * 255) / 100;
float tempC               = FP16(0.0f);
float humdP               = FP16(0.0f);
float ntc1_tmp            = FP16(0.0f);
float ntc2_tmp            = FP16(0.0f);

// Last time markers
uint32_t lastSwitchCheck  = 0U;
uint32_t lastTempRead     = 0U;
uint32_t lastScreenUpdate = 0U;

// Switch states and debounce times
bool lastStateA = HIGH, lastStateB = HIGH, lastStateC = HIGH, lastStateD = HIGH;
uint32_t lastDebounceTimeA = 0, lastDebounceTimeB = 0, lastDebounceTimeC = 0, lastDebounceTimeD = 0;

/**
 * @brief Initialize MCU peripherals, display, sensors, and PWM.
 *
 * Sequence:
 *  - Bring up I2C and GPIO.
 *  - Init OLED and show splash.
 *  - Take over and configure TCA0 for PWM on PB5.
 *  - Init AHT20 sensor (blocks forever on failure).
 *  - Program initial PWM duty based on mapped percentage.
 */
void setup() {
  delay(50);
  Wire.begin();
  delay(100);
  pinMode(LED_PWM_PIN, OUTPUT);
  pinMode(SWITCH_A_PIN, INPUT_PULLUP);
  pinMode(SWITCH_B_PIN, INPUT_PULLUP);
  pinMode(SWITCH_C_PIN, INPUT_PULLUP);
  pinMode(SWITCH_D_PIN, INPUT_PULLUP);
  digitalWrite(LED_PWM_PIN, HIGH);

  delay(500);
  digitalWrite(LED_PWM_PIN, LOW);
  display.begin(&Adafruit128x64, I2C_ADDRESS);
  delay(100);
  display.setFont(System5x7);
  display.clear();

  display.println(F("Init TheSun."));
  display.println(F(""));
  display.println(F("By Niccolo' Ferrari"));

  delay(1000);
  
  configureTimer();
  digitalWrite(LED_PWM_PIN, LOW);

  if (!aht.begin()) {
    display.println(F("Error init AHT20"));
    while (1) {
      delay(250);
    }
  }
  
  digitalWrite(LED_PWM_PIN, LOW);

  pwmValue = (mapPercentage(percentage) * 255) / 100;
  setLedPwm(pwmValue);
}

/**
 * @brief Cooperative task loop.
 *
 * Tasks:
 *  - Debounce/process buttons every DEBOUNCE_DELAY ms.
 *  - Constrain/map percentage to PWM duty.
 *  - Read sensors every TEMP_READ_DELAY ms.
 *  - Refresh OLED every SCREEN_UPDATE_DELAY ms or when data changed.
 *  - Apply PWM to LED via TCA0.
 */
void loop() {
  bool updated = false;

  // Handle switches every 50ms
  if (millis() - lastSwitchCheck >= DEBOUNCE_DELAY) {
    lastSwitchCheck = millis();
    updated |= handleSwitch(SWITCH_A_PIN, 1, lastDebounceTimeA, lastStateA);
    updated |= handleSwitch(SWITCH_B_PIN, -1, lastDebounceTimeB, lastStateB);
    updated |= handleSwitch(SWITCH_C_PIN, 10, lastDebounceTimeC, lastStateC);
    updated |= handleSwitch(SWITCH_D_PIN, -10, lastDebounceTimeD, lastStateD);
  }

  percentage = constrain(percentage, min(P0, P1), max(P0, P1));
  mapped_percentage = mapPercentage(percentage);
  pwmValue = (mapped_percentage * 255) / 100;

  // Read temperatures every 5000ms
  if (millis() - lastTempRead >= TEMP_READ_DELAY) {
    lastTempRead = millis();
    readTemperatures();
    updated = true;
  }

  // Update screen every 5000ms or if there's an update
  if (millis() - lastScreenUpdate >= SCREEN_UPDATE_DELAY || updated) {
    lastScreenUpdate = millis();
    updateScreen();

    // Set the PWM on pin 4
    setLedPwm(pwmValue);
  }

  delay(50);
}

/**
 * @brief Claim and configure TCA0 for single-slope PWM on WO2 (PB5).
 * @details Calls takeOverTCA0(), sets PER for ~250 Hz at 16 MHz / 64 prescale,
 *          enables CMP2, and leaves PORTMUX routing to build flags.
 */
void configureTimer(void) {
  // We explicitly take over TCA0 from megaTinyCore.
  // This:
  //  1. Resets TCA0 to power-on defaults.
  //  2. Tells the core not to touch TCA0 anymore
  //     (no analogWrite()/digitalWrite() side effects on TCA pins).
  takeOverTCA0();

  // At this point TCA0 is in SINGLE mode, counter = 0, prescaler = 1, disabled.
  // PORTMUX for TCA0 output pins is left as configured by the core.
  // In platformio.ini you set: -DTCA_PORTMUX=PORTMUX_TCA02_bm
  // so WO2 is routed to PB5 (Arduino pin 4).

  // We want single-slope PWM on WO2 (PB5 = Arduino pin 4).
  // -> enable compare channel 2 (CMP2EN) and select single-slope mode.
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc  // single-slope PWM
                    | TCA_SINGLE_CMP2EN_bm;             // enable WO2 output

  // Set TOP value (PER). With:
  //   F_CPU = 16 MHz
  //   prescaler = 64
  //   PER = 1000
  // frequency = F_CPU / (prescaler * (PER + 1))
  //           ≈ 16e6 / (64 * 1001) ≈ 249.6 Hz
  TCA0.SINGLE.PER = 1000U;

  // Start with 0 duty until you set pwmValue explicitly.
  TCA0.SINGLE.CMP2 = 0U;

  // Finally, set prescaler = 64 and enable the timer.
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc  // clock = F_CPU / 64
                    | TCA_SINGLE_ENABLE_bm;       // enable TCA0
}

/**
 * @brief Map duty [0..255] to TCA0 PER range and update CMP2 (WO2 => PB5).
 * @param duty Duty cycle 0..255 (0%..100%).
 */
void setLedPwm(const uint8_t duty) {
  // Read PER once (16-bit)
  const uint16_t per = TCA0.SINGLE.PER;

  // 16x16 multiplication would overflow; promote to 32-bit.
  // We map duty in [0..255] to [0..per].
  uint32_t tmp = static_cast<uint32_t>(duty) * static_cast<uint32_t>(per);

  // Integer division; result in [0..per].
  uint16_t cmp = static_cast<uint16_t>(tmp / 255U);

  // Write compare register for channel 2 (WO2).
  TCA0.SINGLE.CMP2 = cmp;
}

/**
 * @brief Read temperature from an NTC thermistor.
 * @param pin Analog pin the NTC divider is connected to.
 * @return Temperature in Celsius (float).
 * @details Uses incremental averaging over NTC_BUFFER samples and the
 *          simplified Steinhart–Hart equation with clamp to avoid NaNs.
 */
float readTemperatureNTC(const int pin) {
  const float nominalTemperature = TO_KELVIN(NOM_TMP); // To Kelvin

  // Calculate the average ADC value using incremental mean
  float adcValue = 0.0;
  for (uint8_t i = 0; i < NTC_BUFFER; ++i) {
      const int _adcValue = analogRead(pin);
      adcValue += (static_cast<float>(_adcValue) - adcValue) / ((float) i + 1.0);
  }

  // Calculate the resistance of the thermistor
  // Clamp adcValue to avoid division by zero and log domain errors
  if (adcValue <= 0.0f) {
    adcValue = 1.0f;
  } else if (adcValue >= (ADC_RES - 1.0f)) {
    adcValue = ADC_RES - 1.0f;
  }
  const float resistance = REF_RES / ((ADC_RES / adcValue) - 1);

  
  const float inverseKelvin = 1.0 / nominalTemperature + log(resistance / NOM_RES) / BETA;
  const float kelvin = (1.0 / inverseKelvin);

  return TO_CELSIUS(kelvin);
}

/**
 * @brief Map user percentage [P0..P1] to bounded output [MIN_P0..MAX_P1].
 * @details Provides a nonlinear clamp that limits minimum and maximum
 *          output regardless of user input extremes.
 */
uint8_t mapPercentage(int8_t x) {
    // Constrain x to the input range [P0, P1]
    x = constrain(x, min(P0, P1), max(P0, P1));

    // Map x from the range [P0, P1] to the range [MIN_P0, MAX_P1]
    float y = MIN_P0 + ((float)(x - P0) * (MAX_P1 - MIN_P0)) / (P1 - P0);

    // Round the result and constrain it to the output range [MIN_P0, MAX_P1]
    int roundedY = round(y);
    return static_cast<uint8_t>(constrain(roundedY, MIN_P0, MAX_P1));
}

// Handle a switch press with debouncing, returns true if updated
/**
 * @brief Debounce a switch and adjust percentage on new press.
 * @param pin Digital input pin.
 * @param increment Amount to add/subtract when pressed.
 * @param lastTime Mutable reference to debounce timer (ms).
 * @param lastState Mutable reference to last pin state.
 * @return true if percentage updated.
 */
bool handleSwitch(int pin, int increment, uint32_t& lastTime, bool& lastState) {
  bool currentState = digitalRead(pin);

  // If the state has changed, check the debounce
  if (currentState != lastState) {
    if ((millis() - lastTime) > DEBOUNCE_DELAY) {
      // Only if the debounce delay has passed, update the state and the variable
      if (currentState == LOW && lastState == HIGH) {
        percentage += increment;
        lastState = currentState; // Update the button state
        lastTime = millis(); // Update the debounce timer
        return true;
      }
      else {
        lastState = currentState; // Update the button state
        lastTime = millis(); // Update the debounce timer
        return false;
      }
    } else {
      // Only update the state without modifying lastTime
      lastState = currentState;
    }
  }

  return false;
}

/** @brief Refresh all OLED lines with current readings and PWM state. */
void updateScreen(void) {
  display.setCursor(0, 0);
  display.clear();
  display.print(F("PWM = "));
  display.print(pwmValue);
  display.print(F("; Perc = "));
  display.print(percentage);
  display.println(F("%"));
  display.println();
  
  display.print(F("Tm: "));
  display.print(tempC);
  display.println(F(" C"));
  display.print(F("Rh': "));
  display.print(humdP);
  display.println(F(" %"));
  display.println();

#ifdef NTC1
  display.print(F("NTC1: "));
  display.print(ntc1_tmp);
  display.println(F(" °C"));
#endif

#ifdef NTC2
  display.print(F("NTC2: "));
  display.print(ntc2_tmp);
  display.println(F(" °C"));
#endif
}

/**
 * @brief Read AHT20 (and NTCs if enabled) into global state.
 * @details Keeps last good values if AHT20 read fails.
 */
void readTemperatures(void) {
  sensors_event_t humidity, temp;
  if (aht.getEvent(&humidity, &temp)) {
    tempC = static_cast<float>(temp.temperature);
    humdP = static_cast<float>(humidity.relative_humidity);
  }

#ifdef NTC1
  ntc1_tmp = readTemperatureNTC(NTC1);
#endif

#ifdef NTC2
  ntc2_tmp = readTemperatureNTC(NTC2);
#endif
}
