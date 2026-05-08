#include <Arduino.h>
#include <SimpleFOC.h>

// Step (b) closed-loop bring-up: velocity control with AMT103 encoder,
// voltage torque mode (no current loops yet). Goal is to prove sensor +
// alignment + velocity PI before adding LowsideCurrentSense and Iq/Id loops.
//
// Hardware: B-G431B-ESC1 + 300Kv 14pp 76mΩ prop motor, AMT103 on J8
//   (PB6=A, PB7=B, PB8=Z wired but unused, 2048 PPR -> 8192 cpr x4).
//   PSU 16 V, 1 mΩ low-side shunts (unused in this step).
//
// Z (index) is intentionally NOT passed to the Encoder ctor: SimpleFOC's
// initFOC will then skip absoluteZeroSearch(), which spins the rotor at
// 1 rad/s in voltage open-loop — too slow on this low-R motor without
// tripping the ESC1 OCP. Velocity control doesn't need absolute zero;
// alignSensor() (which already passes) is sufficient.

const int   POLE_PAIRS    = 14;
const int   ENCODER_PPR   = 2048;
const float PSU_VOLTAGE   = 16.0f;
const float VOLTAGE_CEIL  = 3.0f;     // ceiling on Uq from the velocity PI
// During initFOC's index search the motor spins at ~1 rad/s in voltage
// open-loop. At that speed BEMF ≈ 0, so this voltage is dropped almost
// entirely across phase resistance (76 mΩ) — keep it at the same level we
// proved safe in the open-loop sketch or the ESC1 OCP will trip.
const float ALIGN_VOLTAGE = 1.0f;
const float MAX_VEL_RAD_S = 20.0f;

BLDCMotor      motor   = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver  = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL,
                                        A_PHASE_VH, A_PHASE_VL,
                                        A_PHASE_WH, A_PHASE_WL);
Encoder        encoder = Encoder(A_ENCODER_A, A_ENCODER_B, ENCODER_PPR);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

bool     enabled        = false;
uint32_t last_btn_ms    = 0;
int      last_btn_state = HIGH;
uint32_t last_print_ms  = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);
  Serial.println("==== ESC1 closed-loop velocity, voltage torque mode ====");

  pinMode(USER_BTN, INPUT_PULLUP);
  analogReadResolution(12);

  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = PSU_VOLTAGE;
  driver.voltage_limit        = VOLTAGE_CEIL;
  driver.dead_zone            = 0.005f;     // verified on this board
  if (!driver.init()) {
    Serial.println("driver.init FAILED");
    while (1) delay(1000);
  }
  motor.linkDriver(&driver);

  motor.voltage_limit        = VOLTAGE_CEIL;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.velocity_limit       = MAX_VEL_RAD_S;

  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  // Conservative starting gains for a 14pp motor — expect to tune.
  motor.PID_velocity.P            = 0.05f;
  motor.PID_velocity.I            = 2.0f;
  motor.PID_velocity.D            = 0.0f;
  motor.PID_velocity.output_ramp  = 100.0f;   // V/s slew on Uq
  motor.LPF_velocity.Tf           = 0.01f;    // 10 ms LPF on encoder velocity

  if (!motor.init()) {
    Serial.println("motor.init FAILED");
    while (1) delay(1000);
  }

  // initFOC spins the rotor a fraction of a turn for electrical-zero alignment
  // and to detect encoder direction. Make sure nothing is binding the shaft.
  Serial.println("Aligning rotor (will move)...");
  if (!motor.initFOC()) {
    Serial.println("initFOC FAILED");
    while (1) delay(1000);
  }
  Serial.println("Alignment OK.");

  motor.disable();
  Serial.println("Press USER button to enable. Pot sets target velocity.");
}

void loop() {
  int btn = digitalRead(USER_BTN);
  if (btn == LOW && last_btn_state == HIGH && (millis() - last_btn_ms) > 200) {
    enabled = !enabled;
    if (enabled) motor.enable();
    else         motor.disable();
    Serial.println(enabled ? "ENABLED" : "DISABLED");
    last_btn_ms = millis();
  }
  last_btn_state = btn;

  int   raw    = analogRead(A_POTENTIOMETER);
  float pot    = raw / 4095.0f;
  float target = pot * MAX_VEL_RAD_S;

  motor.loopFOC();
  motor.move(target);

  if (millis() - last_print_ms >= 500) {
    Serial.print("en=");       Serial.print(enabled ? 1 : 0);
    Serial.print(" tgt=");     Serial.print(target, 2);
    Serial.print(" vel=");     Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Uq=");      Serial.print(motor.voltage.q, 2);
    Serial.print(" angle=");   Serial.println(motor.shaft_angle, 2);
    last_print_ms = millis();
  }
}
