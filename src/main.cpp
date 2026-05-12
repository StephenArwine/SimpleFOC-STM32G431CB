#include <Arduino.h>
#include <SimpleFOC.h>
#include <string.h>

// Step (e) closed-loop FOC + CAN reception. The control architecture from
// step (d) is unchanged (velocity outer loop -> Iq inner loop, foc_current
// torque mode). Velocity setpoint arrives over CAN at 1 Mbps, classic
// frames, ID = CAN_ID_TARGET, payload = 4-byte little-endian float32 in
// rad/s. Motor is enabled at boot and tracks the latest CAN setpoint; if
// no fresh frame has been seen within CAN_RX_TIMEOUT_MS the target is
// forced to zero so the rig coasts to a stop instead of running away.
//
// Hardware: B-G431B-ESC1 + 300Kv 14pp 76mΩ prop motor, AMT103 on J8
//   (PB6=A, PB7=B, PB8=Z wired but unused, 2048 PPR -> 8192 cpr x4).
//   PSU 16 V, 3 mΩ low-side shunts via on-chip OPAMPs (PGA gain -64/7).
//   FDCAN1 to onboard TJA1051 transceiver: PA11=RX, PB9=TX,
//                                          PC11=SHDN (low=run),
//                                          PC14=TERM (high=120Ω on).

const int   POLE_PAIRS    = 14;
const int   ENCODER_PPR   = 2048;
const float PSU_VOLTAGE   = 16.0f;
const float VOLTAGE_CEIL  = 2.0f;
const float ALIGN_VOLTAGE = 1.0f;
const float MAX_VEL_RAD_S = 20.0f;
const float CURRENT_LIMIT = 15.0f;

// CAN config — change CAN_ID_TARGET per board to address left vs right ESC.
constexpr uint32_t CAN_ID_TARGET     = 0x100;   // float32 LE velocity [rad/s]
constexpr uint32_t CAN_RX_TIMEOUT_MS = 200;     // target -> 0 if frame older

BLDCMotor      motor   = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver  = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL,
                                        A_PHASE_VH, A_PHASE_VL,
                                        A_PHASE_WH, A_PHASE_WL);
Encoder        encoder = Encoder(A_ENCODER_A, A_ENCODER_B, ENCODER_PPR);
LowsideCurrentSense current_sense = LowsideCurrentSense(
  0.003f, -64.0f / 7.0f,
  A_OP1_OUT, A_OP2_OUT, A_OP3_OUT
);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

uint32_t last_print_ms = 0;

// === CAN reception ===
FDCAN_HandleTypeDef hfdcan1;
volatile float      can_target     = 0.0f;
volatile uint32_t   can_last_rx_ms = 0;
volatile uint32_t   can_rx_count   = 0;

static void can_init() {
  // FDCAN kernel clock from PCLK1 (= 170 MHz on this board's stm32duino default)
  RCC_PeriphCLKInitTypeDef pclk = {};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&pclk);
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // PA11 = FDCAN1_RX, PB9 = FDCAN1_TX (AF9 on STM32G4)
  GPIO_InitTypeDef g = {};
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_FDCAN1;
  g.Pin       = GPIO_PIN_11; HAL_GPIO_Init(GPIOA, &g);
  g.Pin       = GPIO_PIN_9;  HAL_GPIO_Init(GPIOB, &g);

  // Enable onboard transceiver and turn on bus terminator. If this ESC is
  // *not* at a physical end of the bus, drive A_CAN_TERM low instead.
  pinMode(A_CAN_SHDN, OUTPUT); digitalWrite(A_CAN_SHDN, LOW);
  pinMode(A_CAN_TERM, OUTPUT); digitalWrite(A_CAN_TERM, HIGH);

  // 1 Mbps timing with FDCAN kernel clock = 170 MHz:
  //   tq = 100 ns (prescaler 17), 10 tq/bit (1 sync + 7 tseg1 + 2 tseg2),
  //   sample point = 80%.
  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission   = ENABLE;
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;
  hfdcan1.Init.NominalPrescaler     = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1      = 7;
  hfdcan1.Init.NominalTimeSeg2      = 2;
  hfdcan1.Init.StdFiltersNbr        = 1;
  hfdcan1.Init.ExtFiltersNbr        = 0;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Serial.println("FDCAN init FAILED");
    return;
  }

  // Accept exactly CAN_ID_TARGET into RX FIFO 0; reject everything else.
  FDCAN_FilterTypeDef f = {};
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = CAN_ID_TARGET;
  f.FilterID2    = CAN_ID_TARGET;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1,
                                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

extern "C" void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs) {
  if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
  FDCAN_RxHeaderTypeDef h;
  uint8_t buf[8];
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, buf) == HAL_OK) {
    if (h.Identifier == CAN_ID_TARGET && h.DataLength >= FDCAN_DLC_BYTES_4) {
      float v;
      memcpy(&v, buf, sizeof(float));
      can_target     = v;
      can_last_rx_ms = HAL_GetTick();
      can_rx_count++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);
  Serial.println("==== ESC1 closed-loop velocity + foc_current + CAN ====");

  // Diagnostic: FDCAN baud math in can_init() assumes PCLK1 = 170 MHz.
  Serial.print("HCLK=");   Serial.print(HAL_RCC_GetHCLKFreq());
  Serial.print(" PCLK1="); Serial.print(HAL_RCC_GetPCLK1Freq());
  Serial.print(" PCLK2="); Serial.println(HAL_RCC_GetPCLK2Freq());

  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = PSU_VOLTAGE;
  driver.voltage_limit        = VOLTAGE_CEIL;
  driver.dead_zone            = 0.005f;
  if (!driver.init()) {
    Serial.println("driver.init FAILED");
    while (1) delay(1000);
  }
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    Serial.println("current_sense.init FAILED");
    while (1) delay(1000);
  }
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  motor.voltage_limit        = VOLTAGE_CEIL;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.velocity_limit       = MAX_VEL_RAD_S;
  motor.current_limit        = CURRENT_LIMIT;

  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_velocity.P           = 0.5f;
  motor.PID_velocity.I           = 20.0f;
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 2000.0f;
  motor.LPF_velocity.Tf          = 0.015f;

  motor.PID_current_q.P           = 0.5f;
  motor.PID_current_q.I           = 50.0f;
  motor.PID_current_q.D           = 0.0f;
  motor.PID_current_q.output_ramp = 0.0f;
  motor.LPF_current_q.Tf          = 0.005f;
  motor.PID_current_d.P           = 0.5f;
  motor.PID_current_d.I           = 50.0f;
  motor.PID_current_d.D           = 0.0f;
  motor.PID_current_d.output_ramp = 0.0f;
  motor.LPF_current_d.Tf          = 0.005f;

  if (!motor.init()) {
    Serial.println("motor.init FAILED");
    while (1) delay(1000);
  }

  Serial.println("Aligning rotor (will move)...");
  if (!motor.initFOC()) {
    Serial.println("initFOC FAILED");
    while (1) delay(1000);
  }
  Serial.println("Alignment OK.");

  // Bring up CAN after FOC is ready so an early frame can't surprise us.
  can_init();
  Serial.print("CAN listening on ID 0x");
  Serial.println(CAN_ID_TARGET, HEX);

  motor.enable();
  Serial.println("Motor enabled. Tracking CAN setpoint; target -> 0 if stale.");
}

void loop() {
  uint32_t now = millis();
  bool can_fresh = (can_last_rx_ms != 0) &&
                   (now - can_last_rx_ms < CAN_RX_TIMEOUT_MS);
  float target = can_fresh ? can_target : 0.0f;
  target = _constrain(target, -MAX_VEL_RAD_S, MAX_VEL_RAD_S);

  motor.loopFOC();
  motor.move(target);

  if (now - last_print_ms >= 500) {
    Serial.print("src=");     Serial.print(can_fresh ? 'C' : '-');
    Serial.print(" rx=");     Serial.print(can_rx_count);
    Serial.print(" tgt=");    Serial.print(target, 2);
    Serial.print(" vel=");    Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Iq_sp=");  Serial.print(motor.current_sp, 2);
    Serial.print(" Iq=");     Serial.print(motor.current.q, 2);
    Serial.print(" Uq=");     Serial.println(motor.voltage.q, 2);
    last_print_ms = now;
  }
}
