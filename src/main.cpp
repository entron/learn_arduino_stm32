// Mode selection (set one of these to 1, others to 0)
// 0: disabled
// Exactly one should be 1 at a time.
#define MODE_POLEPAIR_TEST 1      // automatic estimation of motor pole pairs using encoder
#define MODE_ENCODER_TEST 0       // encoder wiring / basic angle+velocity view
#define MODE_OPEN_LOOP 0          // original open-loop velocity demo (no encoder)

#if ( (MODE_POLEPAIR_TEST + MODE_ENCODER_TEST + MODE_OPEN_LOOP) != 1 )
#error "Exactly one MODE_XXX macro must be set to 1"
#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SimpleFOC.h>

// Instantiate Serial2 on USART2 if not already defined (PA2=TX, PA3=RX)
#if !defined(SERIAL_PORT_HARDWARE2) && !defined(Serial2)
HardwareSerial Serial2(USART2);
#endif

#if MODE_POLEPAIR_TEST
// ============================= POLE PAIR DETECTION TEST =============================
// Requires: Encoder mounted and wired (A=PA0, B=PA1). Motor phases connected. Driver EN on PB12.
// Principle: Sweep a known span of electrical angle while applying a small voltage.
// Measure mechanical angle change via encoder.  pole_pairs ≈ electrical_angle_span / mechanical_angle_span.

// Pin assignments (same as other modes)
static const uint8_t PIN_PWM_A = PA8;
static const uint8_t PIN_PWM_B = PA9;
static const uint8_t PIN_PWM_C = PA10;
static const uint8_t PIN_EN    = PB12;

// Encoder pins
static const uint8_t PIN_ENC_A = PA0;
static const uint8_t PIN_ENC_B = PA1;

// Detection parameters
static const float SUPPLY_VOLTAGE = 12.0f;     // motor supply
static const float DETECT_VOLTAGE = 2.5f;      // phase voltage magnitude during sweep (raise carefully if motor doesn't move)
static const int   MAX_ELECTRICAL_REV = 14;    // sweep up to this many electrical revolutions
static const float STEP_E_ANGLE = 0.02f;       // electrical angle step (rad)
static const uint32_t SETTLE_US = 2000;        // microseconds settle each step (2ms)

// Encoder CPR (quadrature counts per mechanical revolution)
static const uint32_t ENCODER_CPR = 1024; // set to your encoder configuration for informational prints

// We construct a minimal BLDCMotor just to reuse setPhaseVoltage(). Pole pairs placeholder=1 (not used for manual angle).
BLDCMotor motor_dummy(1);
BLDCDriver3PWM driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_EN);
Encoder encoder(PIN_ENC_A, PIN_ENC_B, ENCODER_CPR);
void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

// Helper to apply an electrical angle with desired q-axis voltage (sine voltage vector)
static inline void applyElectricalAngle(float angle_el){
    // Use SimpleFOC motor API: Uq = DETECT_VOLTAGE, Ud = 0
    motor_dummy.setPhaseVoltage(DETECT_VOLTAGE, 0.0f, angle_el);
}

void setup(){
    Serial2.begin(115200);
    delay(300);
    Serial2.println(F("Pole Pair Detection Test"));
    Serial2.println(F("Ensure rotor is free. It will rotate slowly."));
    Serial2.println(F("If it stalls: increase DETECT_VOLTAGE (code) modestly."));

    // Init driver
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = DETECT_VOLTAGE + 0.5f; // small headroom
    driver.pwm_frequency = 25000;
    driver.init();

    // Link motor_dummy to driver
    motor_dummy.linkDriver(&driver);
    motor_dummy.voltage_limit = DETECT_VOLTAGE + 0.5f;
    motor_dummy.init(); // does not perform FOC since no sensor linked

    // Init encoder
    encoder.quadrature = Quadrature::ON;
    encoder.pullup = Pullup::USE_EXTERN;
    encoder.init();
    encoder.enableInterrupts(doA, doB);
    delay(100);

    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, HIGH);
    Serial2.println(F("Starting sweep..."));

    float start_angle = encoder.getAngle();
    float prev_angle = start_angle;
    float mech_unwrapped = 0.0f; // accumulated mechanical angle change (can be >2π)

    const float total_e_span = TWO_PI * (float)MAX_ELECTRICAL_REV;
    float e_angle = 0.0f;
    uint32_t lastPrint = millis();
    uint32_t steps = 0;

    while(e_angle <= total_e_span){
        applyElectricalAngle(fmodf(e_angle, TWO_PI));
        delayMicroseconds(SETTLE_US);
        encoder.update();
        float a = encoder.getAngle();
        float delta = a - prev_angle;
        if(delta > PI) delta -= TWO_PI;
        else if(delta < -PI) delta += TWO_PI;
        mech_unwrapped += delta;
        prev_angle = a;
        e_angle += STEP_E_ANGLE;
        steps++;
        if(millis() - lastPrint > 500){
            lastPrint = millis();
            Serial2.print(F("Progress: e_rev="));
            Serial2.print(e_angle / TWO_PI, 2);
            Serial2.print(F(" mech_span(rad)="));
            Serial2.print(fabs(mech_unwrapped), 3);
            Serial2.println();
        }
        // Early exit if we've already spanned > 1.2 mechanical revs (enough for estimate)
        if(fabs(mech_unwrapped) >= (2.0f * TWO_PI * 1.2f)) break;
    }

    // Remove drive
    motor_dummy.setPhaseVoltage(0,0,0);
    Serial2.println(F("Sweep done."));

    float mech_span = fabs(mech_unwrapped);
    if(mech_span < 0.2f){
        Serial2.println(F("[FAIL] Mechanical span too small - increase DETECT_VOLTAGE."));
        return;
    }
    float e_span_used = min(e_angle, total_e_span);
    float pole_pairs_est = e_span_used / mech_span;
    int pole_pairs_round = (int)lroundf(pole_pairs_est);
    if(pole_pairs_round < 1) pole_pairs_round = 1;
    if(pole_pairs_round > 60) pole_pairs_round = 60; // sanity

    Serial2.print(F("Electrical span used (rad): ")); Serial2.println(e_span_used, 3);
    Serial2.print(F("Mechanical span (rad): ")); Serial2.println(mech_span, 3);
    Serial2.print(F("Estimated pole pairs: ")); Serial2.println(pole_pairs_est, 3);
    Serial2.print(F("Rounded pole pairs: ")); Serial2.println(pole_pairs_round);
    Serial2.println(F("Verify by running again or by closed-loop test later."));
    Serial2.println(F("Set your MOTOR_POLE_PAIRS to the rounded value."));
}

void loop(){
    // Nothing continuous required; test completes in setup.
}

#elif MODE_ENCODER_TEST
// ============================= ENCODER WIRING TEST =============================
// MT6701 in ABZ (incremental) mode
// Recommended wiring:
//   A -> PA0   (TIM2 CH1)
//   B -> PA1   (TIM2 CH2)
//   Z -> PB4   (index)  (optional, can leave disconnected if not used)
//   VCC 3.3V, GND common
// If MT6701 outputs are push-pull (default) no pullups required.

static const uint8_t PIN_ENC_A = PA0;
static const uint8_t PIN_ENC_B = PA1;
static const uint8_t PIN_ENC_Z = PB4;   // index pin (optional - not used directly in this test)

// Set CPR (counts per revolution) expected by SimpleFOC: this is the quadrature edge count.
// For a PPR (A channel pulses/rev) value of P, CPR = 4*P in quadrature mode.
// Adjust to your MT6701 configuration (common: 2048 or 4096 CPR). Using 8192 as placeholder.
static const uint32_t ENCODER_CPR = 8192; // CHANGE to your actual configured resolution.

Encoder encoder = Encoder(PIN_ENC_A, PIN_ENC_B, ENCODER_CPR);

void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

// Optional: you can watch PIN_ENC_Z manually if you route index (Z). Not used by SimpleFOC Encoder class directly.

void setup(){
    Serial2.begin(115200);
    delay(200);
    Serial2.println(F("MT6701 Encoder Test (ABZ)"));
    Serial2.println(F("Output columns: angle_rad\tvelocity_rad_s\traw_count"));
    Serial2.print(F("Configured CPR: ")); Serial2.println(ENCODER_CPR);
    Serial2.println(F("Send 'z' to zero angle / reset count. Send 'r' to toggle rate."));

    encoder.quadrature = Quadrature::ON;           // use full 4x decoding
    encoder.pullup = Pullup::USE_EXTERN;           // we assume push-pull outputs
    encoder.init();
    encoder.enableInterrupts(doA, doB);

    Serial2.println(F("Encoder initialised."));
    _delay(500);
}

bool fastPrint = false; // toggle high-rate streaming
float zeroAngle = 0.0f; // software zero reference
unsigned long lastPrint = 0;

void handleCmd(){
    if(!Serial2.available()) return;
    char c = Serial2.read();
        if(c=='z' || c=='Z'){
            // software zero: capture current angle as reference
            zeroAngle = encoder.getAngle();
            Serial2.println(F("[Zero] Angle reference captured"));
    } else if(c=='r' || c=='R'){
        fastPrint = !fastPrint;
        Serial2.print(F("[Rate] fastPrint=")); Serial2.println(fastPrint?"ON":"OFF");
    } else {
        // ignore others
    }
}

void loop(){
    handleCmd();
    encoder.update();

    unsigned long now = micros();
    // choose print interval: fast (~2ms) when fastPrint, else 50ms
    static uint32_t intervalFast = 2000;   // 2ms ~500Hz
    static uint32_t intervalSlow = 50000;  // 50ms 20Hz
    static uint32_t lastMicros = 0;
    uint32_t interval = fastPrint ? intervalFast : intervalSlow;
    if((now - lastMicros) >= interval){
        lastMicros = now;
        float angle = encoder.getAngle() - zeroAngle; // radians relative
        float vel   = encoder.getVelocity();    // rad/s
        // raw tick count not exposed; approximate using angle * CPR /(2π)
        long rawApprox = (long) ( (angle) * (float)ENCODER_CPR / (2.0f*PI) );
        Serial2.print(angle, 6); Serial2.print('\t');
        Serial2.print(vel, 4);   Serial2.print('\t');
        Serial2.println(rawApprox);
    }
}

#else
// ============================= ORIGINAL OPEN-LOOP MOTOR EXAMPLE =============================
// (Set MODE_ENCODER_TEST to 0 to use this block)

// User configurable parameters
static const uint8_t MOTOR_POLE_PAIRS = 7;
static const float SUPPLY_VOLTAGE = 12.0f;
static float target_velocity = 10.0f; // rad/s
static const float VOLTAGE_LIMIT = 4.0f;
static const uint8_t PIN_EN = PB12;
static const uint8_t PIN_PWM_A = PA8;
static const uint8_t PIN_PWM_B = PA9;
static const uint8_t PIN_PWM_C = PA10;

BLDCMotor motor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_EN);

void handleSerialInput() {
    if (!Serial2.available()) return;
    char c = Serial2.peek();
    if (c == 'v' || c == 'V') {
        String line = Serial2.readStringUntil('\n');
        int spaceIdx = line.indexOf(' ');
        if (spaceIdx > 0) {
            float val = line.substring(spaceIdx + 1).toFloat();
            if (fabs(val) < 500.0f) {
                target_velocity = val;
                Serial2.print(F("[OK] target_velocity = ")); Serial2.println(target_velocity, 3);
            } else {
                Serial2.println(F("[ERR] velocity out of range"));
            }
        } else {
            Serial2.println(F("Usage: v <rad_per_s>"));
        }
    } else {
        String dummy = Serial2.readStringUntil('\n');
    }
}

void setup(){
    Serial2.begin(115200);
    delay(400);
    Serial2.println(F("SimpleFOC Sensorless Open-loop Demo"));
    Serial2.println(F("Commands: v <rad_per_s>  (example: v 30)"));
    pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, LOW);
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.pwm_frequency = 25000;
    driver.init();
    motor.linkDriver(&driver);
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.foc_modulation = FOCModulationType::SinePWM;
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();
    digitalWrite(PIN_EN, HIGH);
}

unsigned long lastInfo = 0;
void loop(){
    handleSerialInput();
    motor.move(target_velocity);
    unsigned long now = millis();
    if(now - lastInfo > 1000){
        lastInfo = now;
        float rpm = target_velocity * 60.0f / (2.0f * PI);
        Serial2.print(F("vel target(rad/s): ")); Serial2.print(target_velocity,2);
        Serial2.print(F("  est mech rpm ~")); Serial2.println(rpm,1);
    }
}

#endif
