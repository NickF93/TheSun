#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Adafruit_AHTX0.h>

#define I2C_ADDRESS         0x3C

//#define NTC1                2
#define NTC2                3

#define LED_PWM_PIN         4
#define REF_RES             10000.0
#define NOM_RES             10000.0
#define NOM_TMP             25.0
#define BETA                3950
#define ADC_RES             1024.0
#define NTC_BUFFER          100
#define TO_KELVIN(__C__)    (((float) __C__) + 273.15)
#define TO_CELSIUS(__K__)   (((float) __K__) - 273.15)

#define P0                  0
#define P1                  100
#define MIN_P0              5
#define MAX_P1              50

// Define switch pins
#define SWITCH_A_PIN        10 // PC0
#define SWITCH_B_PIN        11 // PC1
#define SWITCH_C_PIN        12 // PC2
#define SWITCH_D_PIN        13 // PC3

// Timing constants
#define DEBOUNCE_DELAY      50
#define TEMP_READ_DELAY     5000
#define SCREEN_UPDATE_DELAY 5000

SSD1306AsciiWire display;
Adafruit_AHTX0 aht;

void configureTimer(void);
float readTemperatureNTC(const int pin);
int mapPercentage(int x);
bool handleSwitch(int pin, int increment, unsigned long& lastTime, bool& lastState);
void updateScreen(void);
void readTemperatures(void);

// Variables
int percentage = P0;
int mapped_percentage = MIN_P0;
int pwmValue = (mapped_percentage * 255) / 100;
float tempC = 0.0;
float humdP = 0.0;
float ntc1_tmp = 0.0;
float ntc2_tmp = 0.0;

// Last time markers
unsigned long lastSwitchCheck = 0;
unsigned long lastTempRead = 0;
unsigned long lastScreenUpdate = 0;

// Switch states and debounce times
bool lastStateA = HIGH, lastStateB = HIGH, lastStateC = HIGH, lastStateD = HIGH;
unsigned long lastDebounceTimeA = 0, lastDebounceTimeB = 0, lastDebounceTimeC = 0, lastDebounceTimeD = 0;

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
  analogWrite(LED_PWM_PIN, 127);
  display.begin(&Adafruit128x64, I2C_ADDRESS);
  delay(100);
  display.setFont(System5x7);
  display.clear();

  display.println(F("Init TheSun."));
  display.println(F(""));
  display.println(F("By Niccolo' Ferrari"));

  delay(1000);
  
  configureTimer();

  if (!aht.begin()) {
    display.println(F("Error init AHT20"));
    while (1) {
      delay(250);
    }
  }
  
  digitalWrite(LED_PWM_PIN, LOW);

  pwmValue = (mapPercentage(percentage) * 255) / 100;
}

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
    analogWrite(LED_PWM_PIN, pwmValue);
  }

  delay(50);
}

void configureTimer(void) {
  // Temporarily disable the timer
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;

  // Configure the prescaler (e.g., divide by 64)
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc;

  // Set the TOP value (lower frequency)
  TCA0.SINGLE.PER = 0x03E8; // TOP = 1000 -> f_PWM = (16 MHz) / (64 * (1000 + 1)) = ~250 Hz

  // Enable PWM output on pin PB5 (WO1)
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;

  // Re-enable the timer
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

// Function to read temperature from an NTC thermistor
float readTemperatureNTC(const int pin) {
  const float nominalTemperature = TO_KELVIN(NOM_TMP); // To Kelvin

  // Calculate the average ADC value using incremental mean
  float adcValue = 0.0;
  for (int i = 0; i < NTC_BUFFER; ++i) {
      const int _adcValue = analogRead(pin);
      adcValue += (static_cast<float>(_adcValue) - adcValue) / ((float) i + 1.0);
  }

  // Calculate the resistance of the thermistor
  const float resistance = REF_RES / (ADC_RES / adcValue - 1);

  
  const float inverseKelvin = 1.0 / nominalTemperature + log(resistance / NOM_RES) / BETA;
  const float kelvin = (1.0 / inverseKelvin);

  return TO_CELSIUS(kelvin);
}

int mapPercentage(int x) {
    // Constrain x to the input range [P0, P1]
    x = constrain(x, min(P0, P1), max(P0, P1));

    // Map x from the range [P0, P1] to the range [MIN_P0, MAX_P1]
    float y = MIN_P0 + ((float)(x - P0) * (MAX_P1 - MIN_P0)) / (P1 - P0);

    // Round the result and constrain it to the output range [MIN_P0, MAX_P1]
    int roundedY = round(y);
    return constrain(roundedY, MIN_P0, MAX_P1);
}

// Handle a switch press with debouncing, returns true if updated
bool handleSwitch(int pin, int increment, unsigned long& lastTime, bool& lastState) {
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

void readTemperatures(void) {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  tempC = static_cast<float>(temp.temperature);
  humdP = static_cast<float>(humidity.relative_humidity);

#ifdef NTC1
  ntc1_tmp = readTemperatureNTC(NTC1);
#endif

#ifdef NTC2
  ntc2_tmp = readTemperatureNTC(NTC2);
#endif
}
