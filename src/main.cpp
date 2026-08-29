/*
  ===========================================================================
  Water Tank Level Controller — 2-Probe Sensor (LOW + FULL)
  Board:     Arduino Nano
  Framework: Arduino (PlatformIO)
  ===========================================================================

  WIRING
  ------
    COMMON probe    -> GND         (mount at the very bottom of the tank;
                                     this must stay submerged whenever there
                                     is ANY water in the tank)
    LOW probe       -> D5          (lowest level — "tank empty" trigger)
    FULL probe      -> D6          (highest level — "tank full" trigger)

    START relay 1   -> D8          (both start relays pulse together, e.g.
    START relay 2   -> D9           one across the starter's START button,
                                     the other for a second contactor/
                                     interlock/indicator — wire per your
                                     panel's needs)
    STOP relay      -> D10         (wire across the STOP button)
    Status LED      -> D13         (onboard LED, lit while motor is running)

  HOW THE SENSING WORKS
  ----------------------
    Each probe pin uses the internal pull-up resistor, so it reads HIGH
    (dry) when open, and is pulled LOW (wet) when water bridges it to the
    COMMON/GND probe. Water conducts just well enough to pull the pin low
    through the pull-up.

  CONTROL LOGIC
  -------------
    - Motor is started (a brief simultaneous pulse on BOTH start relays)
      when the LOW probe goes dry — the tank has emptied below the low
      mark.
    - Motor is stopped (a pulse on the STOP relay) when the FULL probe
      becomes wet — the tank has filled to the top mark.
    - Both probes are debounced (must hold a steady reading for
      DEBOUNCE_MS) to reject noise from splashing, corrosion, or mineral
      buildup on the probes.
    - Relay pulses are timed with millis(), not delay(), so the loop never
      blocks and can't miss a state change.

  IMPORTANT SAFETY NOTES
  -----------------------
    - This code drives a relay MODULE, not the motor directly. Never wire
      Arduino GPIO straight to mains/motor power.
    - The relays here are meant to be wired in parallel with the existing
      manual START/STOP pushbuttons on your motor starter/contactor panel
      (a momentary "button press" simulation) — NOT to switch motor
      current itself. If your setup instead needs to switch mains power
      to a contactor coil directly, use a properly rated relay/contactor
      and have a qualified electrician verify the wiring.
    - Add a flyback/snubber per your relay module's documentation if it
      doesn't already have one.
    - Set RELAY_ACTIVE_LOW below to match your relay module (most cheap
      modules are active-LOW).
  ===========================================================================
*/

#include <Arduino.h>

// ---------------- Pin assignment ----------------
const uint8_t PIN_PROBE_LOW  = 5;
const uint8_t PIN_PROBE_FULL = 6;

const uint8_t PIN_RELAY_START_1 = 8;
const uint8_t PIN_RELAY_START_2 = 9;
const uint8_t PIN_RELAY_STOP    = 10;

const uint8_t PIN_STATUS_LED  = 13;

// ---------------- Config ----------------
const bool RELAY_ACTIVE_LOW = true;           // most relay modules trigger on LOW; set false for active-HIGH boards
const unsigned long RELAY_PULSE_MS   = 600;   // how long to "press" the start/stop button
const unsigned long DEBOUNCE_MS      = 300;   // probe must be stable this long before it's trusted
const unsigned long SAMPLE_INTERVAL_MS = 50;  // how often to sample the probes
const unsigned long STATUS_PRINT_MS  = 1000;  // how often to print status over Serial

// ---------------- State ----------------
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
bool stopPulseActive  = false;
unsigned long startPulseBegin = 0;
unsigned long stopPulseBegin  = 0;

// ---------------- Helpers ----------------
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
  Serial.println(F("[MOTOR] LOW probe dry -> START pulse fired (both start relays)"));
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
bool debounceProbe(uint8_t pin, bool &stableState, bool &lastRaw, unsigned long &lastChangeTime) {
  bool raw = (digitalRead(pin) == LOW); // LOW = wet (probe bridged to GND through water)

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

  pinMode(PIN_RELAY_START_1, OUTPUT);
  pinMode(PIN_RELAY_START_2, OUTPUT);
  pinMode(PIN_RELAY_STOP, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Relays start de-energized
  relayWrite(PIN_RELAY_START_1, false);
  relayWrite(PIN_RELAY_START_2, false);
  relayWrite(PIN_RELAY_STOP, false);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.println(F("Water tank controller booted."));
}

void loop() {
  unsigned long now = millis();

  // 1. Sample probes at a fixed interval
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    debounceProbe(PIN_PROBE_LOW,  lowWet,  lastRawLow,  lastChangeLow);
    debounceProbe(PIN_PROBE_FULL, fullWet, lastRawFull, lastChangeFull);

    // 2. Decide on motor action — edge-triggered, fires once per transition
    if (!motorRunning && !lowWet) {
      // Water has dropped below the LOW probe -> start filling
      motorRunning = true;
      digitalWrite(PIN_STATUS_LED, HIGH);
      beginStartPulse();
    } else if (motorRunning && fullWet) {
      // Water has reached the FULL probe -> stop filling
      motorRunning = false;
      digitalWrite(PIN_STATUS_LED, LOW);
      beginStopPulse();
    }
  }

  // 3. Periodic status print (kept separate from sampling so it's easy to slow down/remove)
  if (now - lastStatusPrint >= STATUS_PRINT_MS) {
    lastStatusPrint = now;
    Serial.print(F("LOW="));   Serial.print(lowWet  ? F("WET") : F("DRY"));
    Serial.print(F(" FULL=")); Serial.print(fullWet ? F("WET") : F("DRY"));
    Serial.print(F(" MOTOR=")); Serial.println(motorRunning ? F("RUNNING") : F("STOPPED"));
  }

  // 4. Always service any active relay pulse (non-blocking timing)
  serviceRelayPulses();
}