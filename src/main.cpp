// SimpleFOC open-loop example for sensorless BLDC on SimpleFOC Mini + Blue Pill
// PWM pins (TIM1): PA8, PA9, PA10  | EN pin: PB12  | Debug serial: USART2 (PA2/PA3)

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SimpleFOC.h>

// Instantiate Serial2 on USART2 if not already defined (PA2=TX, PA3=RX)
#if !defined(SERIAL_PORT_HARDWARE2) && !defined(Serial2)
HardwareSerial Serial2(USART2);
#endif

// --------- User configurable parameters ---------
// Estimate or set your motor pole pairs (number of magnet pairs). Adjust for smoother running.
// Common small gimbal/low-KV hobby motors: 7 (14 magnets), 11 (22 magnets), etc.
static const uint8_t MOTOR_POLE_PAIRS = 7; // TODO: change to your motor's actual pole pair count.

// Supply voltage actually applied to VM (you said 12V). Used for voltage limiting mapping.
static const float SUPPLY_VOLTAGE = 12.0f;

// Initial open-loop target velocity (rad/s). 1 rad/s ~ 9.55 rpm
static float target_velocity = 10.0f; // ~95 rpm mechanical if pole pairs correct

// Voltage limit for safety (<= SUPPLY_VOLTAGE). Raising increases startup torque/heat.
static const float VOLTAGE_LIMIT = 4.0f; // start modest; raise if it stalls

// EN (nSLEEP) pin
static const uint8_t PIN_EN = PB12; // connected to driver EN

// PWM pins (3PWM driver)
static const uint8_t PIN_PWM_A = PA8;
static const uint8_t PIN_PWM_B = PA9;
static const uint8_t PIN_PWM_C = PA10;

// --------- SimpleFOC objects ---------
BLDCMotor motor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_EN); // EN optional, provided here

// Simple helper to parse serial commands:  v <value>  sets velocity rad/s
void handleSerialInput() {
    if (!Serial2.available()) return;
    char c = Serial2.peek();
    if (c == 'v' || c == 'V') {
        String line = Serial2.readStringUntil('\n'); // consume the line
        // format: v 12.3
        int spaceIdx = line.indexOf(' ');
        if (spaceIdx > 0) {
            float val = line.substring(spaceIdx + 1).toFloat();
            if (fabs(val) < 500.0f) { // arbitrary sanity limit
                target_velocity = val;
                Serial2.print(F("[OK] target_velocity = "));
                Serial2.println(target_velocity, 3);
            } else {
                Serial2.println(F("[ERR] velocity out of range"));
            }
        } else {
            Serial2.println(F("Usage: v <rad_per_s>"));
        }
    } else { // unknown command - consume line
        String dummy = Serial2.readStringUntil('\n');
    }
}

void setup() {
    // Serial debug
    Serial2.begin(115200);
    delay(400); // small settle
    Serial2.println(F("SimpleFOC Sensorless Open-loop Demo"));
    Serial2.println(F("Commands: v <rad_per_s>  (example: v 30)"));

    // EN pin will be managed by driver; still ensure it's output LOW first for safety
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, LOW); // keep disabled during config

    // Driver configuration
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = VOLTAGE_LIMIT; // additional safety limit
    driver.pwm_frequency = 25000; // ~25 kHz to reduce audible noise
    if (driver.init()) {
        Serial2.println(F("Driver init OK"));
    } else {
        Serial2.println(F("Driver init FAIL"));
    }

    // Motor configuration
    motor.linkDriver(&driver);
    motor.voltage_limit = VOLTAGE_LIMIT; // ensures move() doesn't exceed
    motor.foc_modulation = FOCModulationType::SinePWM; // default
    motor.controller = MotionControlType::velocity_openloop; // sensorless open-loop

    motor.init(); // no sensor -> open-loop only
    Serial2.println(F("Motor init done (open-loop)"));

    // Enable driver outputs AFTER init (driver.init already may have set EN high; ensure explicit)
    digitalWrite(PIN_EN, HIGH);
    Serial2.println(F("Driver enabled."));
}

unsigned long lastInfo = 0;

void loop() {
    // Process user commands
    handleSerialInput();

    // Open-loop velocity move (no loopFOC required for open-loop)
    motor.move(target_velocity);

    unsigned long now = millis();
    if (now - lastInfo > 1000) {
        lastInfo = now;
        Serial2.print(F("vel target(rad/s): "));
        Serial2.print(target_velocity, 2);
        Serial2.print(F("  est mech rpm ~"));
        // Rough rpm estimate: rad/s * 60 /(2*pi)
        float rpm = target_velocity * 60.0f / (2.0f * PI);
        Serial2.println(rpm, 1);
    }
}
