#include <Arduino.h>
#include <SimpleFOC.h>

// Openloop on this 76 mΩ motor is bounded by the ESC1's ~15 A OCP at standstill
// and by Kv=300 at speed — only a narrow viable window. Closed-loop with
// LowsideCurrentSense is the real fix; this just proves the control path turns
// the rotor.
const int POLE_PAIRS = 14;
const float PSU_VOLTAGE = 16.0f;
const float MOTOR_VOLTAGE_LIMIT = 1.0f;
const float MAX_VEL_RAD_S = 15.0f;
const uint32_t FOC_HZ = 10000;
const float DT_ISR = 1.0f / FOC_HZ;

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

HardwareTimer* foc_timer = nullptr;

// Shared state main↔ISR. 32-bit float/bool reads/writes are atomic on
// Cortex-M4.
volatile bool shared_enabled = false;
volatile float shared_target_vel = 0.0f;
float elec_angle = 0;

void foc_isr() {
  if (!shared_enabled) return;
  elec_angle += shared_target_vel * POLE_PAIRS * DT_ISR;
  if (elec_angle > 6.2832f) elec_angle -= 6.2832f;
  if (elec_angle < -6.2832f) elec_angle += 6.2832f;
  motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT, 0, elec_angle);
}

bool enabled = false;
uint32_t last_btn_ms = 0;
int last_btn_state = HIGH;
uint32_t last_print_ms = 0;
int g_dr = -1;
int g_mr = -1;

void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("==== ESC1 SimpleFOC open-loop ====");

  pinMode(USER_BTN, INPUT_PULLUP);
  analogReadResolution(12);

  driver.voltage_power_supply = PSU_VOLTAGE;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  driver.dead_zone = 0.005f;
  g_dr = driver.init();
  Serial.print("driver.init() -> ");
  Serial.println(g_dr);

  motor.linkDriver(&driver);
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.controller = MotionControlType::velocity_openloop;
  g_mr = motor.init();
  Serial.print("motor.init()  -> ");
  Serial.println(g_mr);

  motor.disable();

  // TIM6 is a basic timer on G4 — no conflict with TIM1 (PWM) or TIM2 (micros).
  foc_timer = new HardwareTimer(TIM6);
  foc_timer->setOverflow(FOC_HZ, HERTZ_FORMAT);
  foc_timer->attachInterrupt(foc_isr);
  foc_timer->resume();

  Serial.println("Press USER button to enable. Pot sets velocity.");
}

void loop() {
  int btn = digitalRead(USER_BTN);
  if (btn == LOW && last_btn_state == HIGH && (millis() - last_btn_ms) > 200) {
    enabled = !enabled;
    if (enabled) {
      motor.enable();
      shared_enabled = true;
    } else {
      shared_enabled = false;  // stop ISR from touching TIM1 first
      motor.disable();
      elec_angle = 0;
    }
    Serial.println(enabled ? "ENABLED" : "DISABLED");
    last_btn_ms = millis();
  }
  last_btn_state = btn;

  int raw = analogRead(A_POTENTIOMETER);
  float pot = raw / 4095.0f;
  float target = pot * MAX_VEL_RAD_S;
  shared_target_vel = target;

  static uint32_t loop_n = 0;
  loop_n++;

  if (millis() - last_print_ms >= 1000) {
    Serial.print("loop_hz=");
    Serial.print(loop_n);
    Serial.print(" en=");
    Serial.print(enabled ? 1 : 0);
    Serial.print(" pot=");
    Serial.print(raw);
    Serial.print(" tgt=");
    Serial.println(target, 2);
    loop_n = 0;
    last_print_ms = millis();
  }
}
