#include <Arduino.h>

// Pin Assignment

const uint8_t PIN_PROBE_LOW = 12;
const uint8_t PIN_PROBE_FULL = 11;
// drives the reference probe, pulsed only during sampling
const uint8_t PIN_PROBE_EXCITE = 9;

const uint8_t PIN_RELAY_START_1 = 2;
const uint8_t PIN_RELAY_START_2 = 3;
const uint8_t PIN_RELAY_STOP = 4;

const uint8_t PIN_LOW_PROBE_LED = 6;
const uint8_t PIN_FULL_PROBE_LED = 7;
const uint8_t PIN_MOTOR_STATUS_LED = 5;

// Configurations

// most relay modules trigger on LOW; set false for active-HIGH boards
const bool RELAY_ACTIVE_LOW = true;

// how long to "press" the start/stop button
const unsigned long RELAY_PULSE_MS = 600;

// probe must be stable this long before it's trusted
const unsigned long DEBOUNCE_MS = 300;

// how often to sample the probes
const unsigned long SAMPLE_INTERVAL_MS = 50;

// how long to hold the reference probe HIGH before reading
const unsigned long EXCITE_SETTLE_MS = 5;

// how often to print status over Serial
const unsigned long STATUS_PRINT_MS = 1000;

// States

bool motorRunning = false;

// Debounced, trustworthy probe states (true = wet)
bool lowWet = false, fullWet = false;

// Debounce tracking
bool lastRawLow = false, lastRawFull = false;
unsigned long lastChangeLow = 0, lastChangeFull = 0;

unsigned long lastSampleTime = 0;
unsigned long lastStatusPrint = 0;

// Non-blocking relay pulse tracking
bool startPulseActive = false;
bool stopPulseActive = false;
unsigned long startPulseBegin = 0;
unsigned long stopPulseBegin = 0;

// Helpers Functions

void relayWrite(uint8_t pin, bool energize) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, energize ? LOW : HIGH);
  } else {
    digitalWrite(pin, energize ? HIGH : LOW);
  }
}

void beginStartPulse() {
  startPulseActive = true;
  startPulseBegin = millis();
  relayWrite(PIN_RELAY_START_1, true);
  relayWrite(PIN_RELAY_START_2, true);
  Serial.println(
      F("[MOTOR] LOW probe dry -> START pulse fired (both start relays)"));
}

void beginStopPulse() {
  stopPulseActive = true;
  stopPulseBegin = millis();
  relayWrite(PIN_RELAY_STOP, true);
  Serial.println(F("[MOTOR] FULL probe wet -> STOP pulse fired"));
}

// Ends relay pulses once RELAY_PULSE_MS has elapsed. Non-blocking.
void serviceRelayPulses() {
  unsigned long now = millis();
  if (startPulseActive && (now - startPulseBegin >= RELAY_PULSE_MS)) {
    relayWrite(PIN_RELAY_START_1, false);
    relayWrite(PIN_RELAY_START_2, false);
    startPulseActive = false;
  }
  if (stopPulseActive && (now - stopPulseBegin >= RELAY_PULSE_MS)) {
    relayWrite(PIN_RELAY_STOP, false);
    stopPulseActive = false;
  }
}

// Debounces one probe pin, updating stableState only after the raw
// reading has held steady for DEBOUNCE_MS. Returns the current stable state.
bool debounceProbe(uint8_t pin, bool& stableState, bool& lastRaw,
                   unsigned long& lastChangeTime) {
  // LOW = wet (probe bridged to GND through water)
  bool raw = (digitalRead(pin) == LOW);

  if (raw != lastRaw) {
    lastChangeTime = millis();
    lastRaw = raw;
  }

  if ((millis() - lastChangeTime) >= DEBOUNCE_MS) {
    stableState = raw;
  }

  return stableState;
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_PROBE_LOW, INPUT_PULLUP);
  pinMode(PIN_PROBE_FULL, INPUT_PULLUP);
  pinMode(PIN_PROBE_EXCITE, OUTPUT);

  // reference probe stays de-energized except during sampling
  digitalWrite(PIN_PROBE_EXCITE, LOW);

  pinMode(PIN_RELAY_START_1, OUTPUT);
  pinMode(PIN_RELAY_START_2, OUTPUT);
  pinMode(PIN_RELAY_STOP, OUTPUT);
  pinMode(PIN_MOTOR_STATUS_LED, OUTPUT);
  pinMode(PIN_LOW_PROBE_LED, OUTPUT);
  pinMode(PIN_FULL_PROBE_LED, OUTPUT);

  // Relays start de-energized
  relayWrite(PIN_RELAY_START_1, false);
  relayWrite(PIN_RELAY_START_2, false);
  relayWrite(PIN_RELAY_STOP, false);
  digitalWrite(PIN_MOTOR_STATUS_LED, LOW);

  Serial.println(F("Water tank controller booted."));
}

void loop() {
  unsigned long now = millis();

  // 1. Sample probes at a fixed interval
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    // Energize the reference probe only for this brief sampling window,
    // then de-energize it again — minimizes DC exposure time on the
    // probes to reduce electrolytic corrosion.
    digitalWrite(PIN_PROBE_EXCITE, HIGH);
    delay(EXCITE_SETTLE_MS);

    debounceProbe(PIN_PROBE_LOW, lowWet, lastRawLow, lastChangeLow);
    debounceProbe(PIN_PROBE_FULL, fullWet, lastRawFull, lastChangeFull);

    digitalWrite(PIN_PROBE_EXCITE, LOW);

    // 2. Decide on motor action — edge-triggered, fires once per transition
    if (!motorRunning && !lowWet) {
      // Water has dropped below the LOW probe -> start filling
      motorRunning = true;
      digitalWrite(PIN_MOTOR_STATUS_LED, HIGH);
      beginStartPulse();
    } else if (motorRunning && fullWet) {
      // Water has reached the FULL probe -> stop filling
      motorRunning = false;
      digitalWrite(PIN_MOTOR_STATUS_LED, LOW);
      beginStopPulse();
    }
  }

  // 3. Periodic status print
  if (now - lastStatusPrint >= STATUS_PRINT_MS) {
    lastStatusPrint = now;
    Serial.print(F("LOW="));
    Serial.print(lowWet ? F("WET") : F("DRY"));
    Serial.print(F(" FULL="));
    Serial.print(fullWet ? F("WET") : F("DRY"));
    Serial.print(F(" MOTOR="));
    Serial.println(motorRunning ? F("RUNNING") : F("STOPPED"));
  }

  // 4. LED Status of Probe touching Water
  lowWet ? (digitalWrite(PIN_LOW_PROBE_LED, HIGH))
         : digitalWrite(PIN_LOW_PROBE_LED, LOW);

  fullWet ? (digitalWrite(PIN_FULL_PROBE_LED, HIGH))
          : (digitalWrite(PIN_FULL_PROBE_LED, LOW));

  // 5. Always service any active relay pulse (non-blocking timing)
  serviceRelayPulses();
}