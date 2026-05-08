#include <Arduino.h>
#include <SimpleFOC.h>

// Step (a) of closed-loop bring-up: encoder sanity check.
// No driver, no motor, no PWM — just prove the AMT103 is wired correctly
// and SimpleFOC reads it the way we expect. Spin the shaft by hand.
//
// AMT103 wired to J8: A=PB6, B=PB7, Z=PB8. 2048 PPR (8192 counts/rev x4).

const int ENCODER_PPR = 2048;

Encoder encoder = Encoder(A_ENCODER_A, A_ENCODER_B, ENCODER_PPR, A_ENCODER_Z);

void doA()     { encoder.handleA(); }
void doB()     { encoder.handleB(); }
void doIndex() { encoder.handleIndex(); }

uint32_t last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("==== ESC1 encoder sanity check ====");
  Serial.print("PPR=");
  Serial.print(ENCODER_PPR);
  Serial.println(" (x4 quadrature -> 8192 counts/rev)");

  encoder.init();
  encoder.enableInterrupts(doA, doB, doIndex);

  Serial.println("Rotate the shaft by hand. CW should give increasing angle.");
}

void loop() {
  encoder.update();

  if (millis() - last_print_ms >= 200) {
    float angle    = encoder.getAngle();
    float velocity = encoder.getVelocity();
    float revs     = angle / _2PI;

    Serial.print("angle=");
    Serial.print(angle, 3);
    Serial.print(" rad  rev=");
    Serial.print(revs, 3);
    Serial.print("  vel=");
    Serial.print(velocity, 2);
    Serial.print(" rad/s  idx_found=");
    Serial.println(encoder.indexFound() ? 1 : 0);

    last_print_ms = millis();
  }
}
