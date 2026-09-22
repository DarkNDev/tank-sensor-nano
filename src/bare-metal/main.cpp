#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Pin Assignment

#define PROBE_LOW_PORT PORTB
#define PROBE_LOW_DDR DDRB
#define PROBE_LOW_PIN PINB
#define PROBE_LOW_BIT PB4  // D12

#define PROBE_FULL_PORT PORTB
#define PROBE_FULL_DDR DDRB
#define PROBE_FULL_PIN PINB
#define PROBE_FULL_BIT PB3  // D11

#define RELAY_START_1_PORT PORTD
#define RELAY_START_1_DDR DDRD
#define RELAY_START_1_BIT PD6  // D6

#define RELAY_START_2_PORT PORTD
#define RELAY_START_2_DDR DDRD
#define RELAY_START_2_BIT PD7  // D7

#define RELAY_STOP_PORT PORTB
#define RELAY_STOP_DDR DDRB
#define RELAY_STOP_BIT PB0  // D8

#define LOW_PROBE_LED_PORT PORTB
#define LOW_PROBE_LED_DDR DDRB
#define LOW_PROBE_LED_BIT PB2  // D10

#define FULL_PROBE_LED_PORT PORTB
#define FULL_PROBE_LED_DDR DDRB
#define FULL_PROBE_LED_BIT PB1  // D9

#define MOTOR_STATUS_LED_PORT PORTD
#define MOTOR_STATUS_LED_DDR DDRD
#define MOTOR_STATUS_LED_BIT PD2  // D2

// Small bit-manipulation helpers (macros, no function-call/struct overhead)
#define SET_BIT(reg, bit) ((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit) ((reg) &= ~(1 << (bit)))
#define READ_BIT(reg, bit) ((reg) & (1 << (bit)))

// Configurations

// most relay modules trigger on LOW; set false for active-HIGH boards
const bool RELAY_ACTIVE_LOW = true;

// how long to "press" the start/stop button
const uint32_t RELAY_PULSE_MS = 600;

// probe must be stable this long before it's trusted
const uint32_t DEBOUNCE_MS = 300;

// how often to sample the probes
const uint32_t SAMPLE_INTERVAL_MS = 50;

// ---------------- millis() replacement (Timer0, CTC, 1ms tick)
// 16 MHz / prescaler 64 / (OCR0A+1 = 250) = exactly 1000 interrupts/sec,
// so each tick is exactly 1.000 ms — no fractional-error correction needed.

volatile uint32_t g_millis = 0;

ISR(TIMER0_COMPA_vect) { g_millis++; }

static void millisTimerInit() {
  TCCR0A = (1 << WGM01);  // CTC mode
  OCR0A = 249;            // 16e6/64/250 = 1000 Hz
  TIMSK0 = (1 << OCIE0A);
  TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler = 64, timer starts here
}

static uint32_t millis() {
  uint32_t m;
  uint8_t oldSREG = SREG;
  cli();
  m = g_millis;
  SREG = oldSREG;
  return m;
}

// States

bool motorRunning = false;

// Debounced, trustworthy probe states (true = wet)
bool lowWet = false, fullWet = false;

// Debounce tracking
bool lastRawLow = false, lastRawFull = false;
uint32_t lastChangeLow = 0, lastChangeFull = 0;

uint32_t lastSampleTime = 0;

// Non-blocking relay pulse tracking
bool startPulseActive = false;
bool stopPulseActive = false;
uint32_t startPulseBegin = 0;
uint32_t stopPulseBegin = 0;

// Helpers Functions

void beginStartPulse() {
  startPulseActive = true;
  startPulseBegin = millis();
  if (RELAY_ACTIVE_LOW) {
    CLEAR_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    CLEAR_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
  } else {
    SET_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    SET_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
  }
}

void endStartPulse() {
  if (RELAY_ACTIVE_LOW) {
    SET_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    SET_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
  } else {
    CLEAR_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    CLEAR_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
  }
  startPulseActive = false;
}

void beginStopPulse() {
  stopPulseActive = true;
  stopPulseBegin = millis();
  if (RELAY_ACTIVE_LOW) {
    CLEAR_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  } else {
    SET_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  }
}

void endStopPulse() {
  if (RELAY_ACTIVE_LOW) {
    SET_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  } else {
    CLEAR_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  }
  stopPulseActive = false;
}

// Ends relay pulses once RELAY_PULSE_MS has elapsed. Non-blocking.
void serviceRelayPulses() {
  uint32_t now = millis();
  if (startPulseActive && (now - startPulseBegin >= RELAY_PULSE_MS)) {
    endStartPulse();
  }
  if (stopPulseActive && (now - stopPulseBegin >= RELAY_PULSE_MS)) {
    endStopPulse();
  }
}

// Debounces the LOW probe.
bool debounceLow() {
  bool raw = !READ_BIT(PROBE_LOW_PIN, PROBE_LOW_BIT);  // LOW pin = wet

  if (raw != lastRawLow) {
    lastChangeLow = millis();
    lastRawLow = raw;
  }
  if ((millis() - lastChangeLow) >= DEBOUNCE_MS) {
    lowWet = raw;
  }
  return lowWet;
}

// Debounces the FULL probe.
bool debounceFull() {
  bool raw = !READ_BIT(PROBE_FULL_PIN, PROBE_FULL_BIT);  // LOW pin = wet

  if (raw != lastRawFull) {
    lastChangeFull = millis();
    lastRawFull = raw;
  }
  if ((millis() - lastChangeFull) >= DEBOUNCE_MS) {
    fullWet = raw;
  }
  return fullWet;
}

void setup() {
  millisTimerInit();
  sei();  // enable global interrupts (needed for the millis() tick)

  // Probes: inputs with internal pull-up
  CLEAR_BIT(PROBE_LOW_DDR, PROBE_LOW_BIT);
  SET_BIT(PROBE_LOW_PORT, PROBE_LOW_BIT);
  CLEAR_BIT(PROBE_FULL_DDR, PROBE_FULL_BIT);
  SET_BIT(PROBE_FULL_PORT, PROBE_FULL_BIT);

  // Outputs
  SET_BIT(RELAY_START_1_DDR, RELAY_START_1_BIT);
  SET_BIT(RELAY_START_2_DDR, RELAY_START_2_BIT);
  SET_BIT(RELAY_STOP_DDR, RELAY_STOP_BIT);
  SET_BIT(MOTOR_STATUS_LED_DDR, MOTOR_STATUS_LED_BIT);
  SET_BIT(LOW_PROBE_LED_DDR, LOW_PROBE_LED_BIT);
  SET_BIT(FULL_PROBE_LED_DDR, FULL_PROBE_LED_BIT);

  // Relays start de-energized
  if (RELAY_ACTIVE_LOW) {
    SET_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    SET_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
    SET_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  } else {
    CLEAR_BIT(RELAY_START_1_PORT, RELAY_START_1_BIT);
    CLEAR_BIT(RELAY_START_2_PORT, RELAY_START_2_BIT);
    CLEAR_BIT(RELAY_STOP_PORT, RELAY_STOP_BIT);
  }
  CLEAR_BIT(MOTOR_STATUS_LED_PORT, MOTOR_STATUS_LED_BIT);
}

void loop() {
  uint32_t now = millis();

  // 1. Sample probes at a fixed interval
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    debounceLow();
    debounceFull();

    // 2. Decide on motor action — edge-triggered, fires once per transition
    if (!motorRunning && !lowWet) {
      // Water has dropped below the LOW probe -> start filling
      motorRunning = true;
      SET_BIT(MOTOR_STATUS_LED_PORT, MOTOR_STATUS_LED_BIT);
      beginStartPulse();
    } else if (motorRunning && fullWet) {
      // Water has reached the FULL probe -> stop filling
      motorRunning = false;
      CLEAR_BIT(MOTOR_STATUS_LED_PORT, MOTOR_STATUS_LED_BIT);
      beginStopPulse();
    }
  }

  // 3. LED status of probe touching water
  if (lowWet) {
    SET_BIT(LOW_PROBE_LED_PORT, LOW_PROBE_LED_BIT);
  } else {
    CLEAR_BIT(LOW_PROBE_LED_PORT, LOW_PROBE_LED_BIT);
  }

  if (fullWet) {
    SET_BIT(FULL_PROBE_LED_PORT, FULL_PROBE_LED_BIT);
  } else {
    CLEAR_BIT(FULL_PROBE_LED_PORT, FULL_PROBE_LED_BIT);
  }

  // 4. Always service any active relay pulse (non-blocking timing)
  serviceRelayPulses();
}

int main() {
  setup();
  while (1) {
    loop();
  }
  return 0;
}