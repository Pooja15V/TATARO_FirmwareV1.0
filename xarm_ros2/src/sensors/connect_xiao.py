'''
#include <ArduinoBLE.h>
#include "nrf_gpio.h"
#include "nrf_soc.h"

const int BUTTON_PIN = D0;

// Hold the button this long (continuously) right after waking to actually turn on.
// A brief/accidental bump that doesn't hold for the full duration goes straight
// back to sleep instead of powering up BLE + LEDs.
const unsigned long TURN_ON_HOLD_MS = 10000;

// If ON and no button press happens for this long, power back off.
const unsigned long IDLE_TIMEOUT_MS = 5UL * 60UL * 1000UL; // 5 minutes

// Light-cycle timing (unchanged from before)
const unsigned long GREEN_DURATION_MS = 3000;
const unsigned long WARNING_DURATION_MS = 2000;
const unsigned long FLASH_INTERVAL_MS = 250;
const unsigned long DEBOUNCE_DELAY_MS = 30;

BLEService trafficLightService("19B10000-E8F2-537E-4F6C-D104768A1214");
// 0 = idle/off, 1 = Green, 2 = Warning
BLEByteCharacteristic stateCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

void ledsOff() {
  analogWrite(LEDR, 255); // active-low PWM: 255 = fully off
  analogWrite(LEDG, 255);
}

void goToSleep() {
  BLE.stopAdvertise();
  BLE.end();
  ledsOff();

  // Wait for release first -- SENSE is level-triggered, so arming it while the
  // pin is still LOW would wake the chip right back up immediately.
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10);
  }
  delay(30); // debounce on release

  uint32_t nrfPin = g_ADigitalPinMap[BUTTON_PIN];
  nrf_gpio_cfg_sense_input(nrfPin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  Serial.println("Powering off (System OFF). Hold the button 10s to turn on.");
  Serial.flush();

  sd_power_system_off(); // does not return; next press causes a reset
}

// Runs the 3s green + 2s flashing-warning sequence, polling BLE throughout.
void runLightCycle() {
  stateCharacteristic.writeValue(1);
  analogWrite(LEDR, 255);
  analogWrite(LEDG, 0);

  unsigned long phaseStart = millis();
  while (millis() - phaseStart < GREEN_DURATION_MS) {
    BLE.poll();
  }

  stateCharacteristic.writeValue(2);
  phaseStart = millis();
  unsigned long flashTimer = phaseStart;
  bool flashToggle = false;

  while (millis() - phaseStart < WARNING_DURATION_MS) {
    BLE.poll();
    if (millis() - flashTimer >= FLASH_INTERVAL_MS) {
      flashTimer = millis();
      flashToggle = !flashToggle;
      analogWrite(LEDG, flashToggle ? 0 : 255);
    }
  }

  stateCharacteristic.writeValue(0);
  ledsOff();
  BLE.poll();
}

void setup() {
  Serial.begin(115200);
  delay(100); // let USB-serial settle if connected

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  analogWrite(LEDB, 255); // blue unused, stays off

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("XIAO-TATARO");
  BLE.setAdvertisedService(trafficLightService);
  trafficLightService.addCharacteristic(stateCharacteristic);
  BLE.addService(trafficLightService);
  // Not advertising yet -- only once the long-press is confirmed below.

  // --- Confirm this is a deliberate 10s hold, not a brief bump ---
  Serial.println("Woke up. Hold the button to turn on...");
  unsigned long holdStart = millis();
  bool confirmed = true;
  while (millis() - holdStart < TURN_ON_HOLD_MS) {
    if (digitalRead(BUTTON_PIN) == HIGH) {
      confirmed = false;
      break;
    }
    delay(10);
  }

  if (!confirmed) {
    Serial.println("Released early -- back to sleep.");
    goToSleep();
    return; // unreachable, sd_power_system_off() never returns
  }

  Serial.println("Turned ON.");
  BLE.advertise();
  stateCharacteristic.writeValue(0);
  ledsOff();

  // --- Active session: stay on until IDLE_TIMEOUT_MS passes with no press ---
  bool lastButtonState = HIGH;
  bool debouncedState = HIGH;
  unsigned long lastDebounceTime = millis();
  unsigned long lastActivity = millis();

  while (millis() - lastActivity < IDLE_TIMEOUT_MS) {
    BLE.poll();

    bool reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS && reading != debouncedState) {
      debouncedState = reading;
      if (debouncedState == LOW) { // new press edge
        lastActivity = millis();
        Serial.println("Button pressed -- running light cycle.");
        runLightCycle();
        lastActivity = millis(); // count the cycle itself as activity too
        lastDebounceTime = millis();
      }
    }
    lastButtonState = reading;
  }

  Serial.println("Idle for 5 minutes -- powering off.");
  goToSleep();
}

void loop() {
  // Never reached -- goToSleep() powers the chip off before setup() returns.
}
'''