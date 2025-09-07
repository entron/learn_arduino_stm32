// HC-SR04 example for Arduino API (PlatformIO / STM32)
// Wiring:
//  - VCC  -> 5V (typical). Some newer HC-SR04 variants can run at 3.3V; this example assumes 5V VCC.
//  - GND  -> GND (common ground with MCU)
//  - TRIG -> MCU output pin (3.3V is fine)
//  - ECHO -> MCU input pin
// STM32F103 5V-tolerance notes:
//  - Many pins are 5V‑tolerant (marked FT in the datasheet) when configured as digital input/AF input.
//  - NOT 5V‑tolerant in analog (ADC) mode. Avoid non‑FT pins: PC13–PC15, PA11/PA12 (USB), NRST, VBAT.
//  - If you use a confirmed FT pin (e.g. PA5 used here) as a digital input, you may connect ECHO directly.
//  - If unsure, use a level shifter or a simple divider (e.g. 5k/10k) on ECHO to ~3.3V for safety.

#include <Arduino.h>
#include <HardwareSerial.h>

// Instantiate Serial2 on USART2 if not already provided by the core/variant
#if !defined(SERIAL_PORT_HARDWARE2) && !defined(Serial2)
HardwareSerial Serial2(USART2); // PA2=TX2, PA3=RX2 on Blue Pill
#endif

// Recommended Blue Pill pins (STM32F103):
// TRIG -> PA4 (MCU output, drives sensor trigger)
// ECHO -> PA5 (MCU input, must be level-shifted to 3.3V)
// Note: In Arduino/PlatformIO for STM32 the pins can be referenced by their
// port names (PA4/PA5) where supported by the variant headers.
const uint8_t TRIG_PIN = PA4; // PA4 recommended for trigger
const uint8_t ECHO_PIN = PA5; // PA5 recommended for echo (use divider/level-shifter)

// Measurement settings
const unsigned long MEASURE_INTERVAL_MS = 100; // how often to take a measurement
const unsigned long PULSE_TIMEOUT_US = 30000UL; // 30ms timeout -> ~5 meters

void setup() {
  // Use Serial2 on PA2 (TX) / PA3 (RX) at 115200 to match project wiring
  Serial2.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
  delay(50);

  Serial2.println("HC-SR04 example starting");
  Serial2.println("Wiring: VCC=5V, GND=GND, TRIG=PA4, ECHO=PA5 (FT pin)");
  Serial2.println("Note: On STM32F103, many pins are 5V-tolerant in digital input mode.");
  Serial2.println("If not using an FT pin or if unsure, add a divider on ECHO (e.g. 5k/10k).");
}

float measureDistanceCm() {
  // send 10us pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // read the pulse length on ECHO (microseconds)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) {
    // timeout / no echo
    return -1.0f;
  }

  // Convert microseconds to centimeters
  // Sound speed ~343 m/s -> 29.1 us per cm round-trip -> common formula uses 58
  float distanceCm = duration / 58.0f;
  return distanceCm;
}

void loop() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < MEASURE_INTERVAL_MS) return;
  lastMs = now;

  float d = measureDistanceCm();
  if (d < 0) {
  Serial2.println("Timeout: no echo");
  } else {
  Serial2.print("Distance: ");
  Serial2.print(d, 2);
  Serial2.println(" cm");
  }
}
