// =============================================================================
// rc_drive.ino - Open-loop RC wheel drive test for Teensy 4.1  (NO balancing)
//
//  >>> RUN WITH THE WHEELS OFF THE GROUND <<<
//
// Proves the whole radio -> motor path before we combine it with the balancer:
//   - CRSF channels (same parser as crsf_test)
//   - differential mix: left = drive + turn, right = drive - turn
//   - ARM (SB) gates the motors; disarm OR link-loss -> motors OFF (kill switch)
//   - throttle (CH3) = top-speed cap
//
// Wiring: RP1 on Serial1 (TX->pin0, RX->pin1, 5V, GND). Motors via IBT-2:
//   LEFT  RPWM/LPWM = pins 6 / 9    RIGHT RPWM/LPWM = pins 22 / 23
//   R_EN/L_EN tied to 5V. (See MIGRATION_TEENSY.md 3.1.)
//
// USB serial @ 115200. Flash: flash --teensy --sketch rc_drive --all
// =============================================================================

// ---- Motor pins (match balance_v2 / MIGRATION_TEENSY.md 3.1) ----------------
#define LEFT_MOTOR_FORWARD_PIN   6
#define LEFT_MOTOR_REVERSE_PIN   9
#define RIGHT_MOTOR_FORWARD_PIN  22
#define RIGHT_MOTOR_REVERSE_PIN  23

// If a wheel spins the WRONG way, flip that side's sign (+1 <-> -1). This is the
// whole point of the test -- get both wheels driving "forward" on stick-up.
#define LEFT_DIR   (+1)
#define RIGHT_DIR  (+1)

const int   MAX_PWM        = 110;   // ceiling (0-255), same as balance_v2
// Per-side stiction kick: the LEFT motor (pins 6/9) runs stiffer, so it needs a
// bigger kick or it lags on small commands. Tune per wheel.
const int   LEFT_DEADBAND  = 15;
const int   RIGHT_DEADBAND = 11;
const float CMD_DEADZONE   = 0.02f; // ignore tiny mixed commands

// ---- CRSF ------------------------------------------------------------------
#define CRSF_PORT   Serial1
#define CRSF_BAUD   420000
#define DEBUG_BAUD  115200

static const uint8_t  CRSF_ADDR_FC          = 0xC8;
static const uint8_t  CRSF_TYPE_RC_CHANNELS = 0x16;
static const uint8_t  CRSF_FRAME_MAX        = 64;
static const uint8_t  CRSF_PAYLOAD_RC       = 22;
static const uint32_t LINK_TIMEOUT_MS       = 500;

uint16_t rcChannels[16];
uint32_t lastFrameMs = 0;
bool     linkUp      = false;

// ---- Channel map (TX16S) ----------------------------------------------------
#define CH_TURN    0   // CH1 right stick L/R
#define CH_DRIVE   1   // CH2 right stick U/D (self-centering) -> direction
#define CH_SPEED   2   // CH3 throttle -> speed cap
#define CH_ARM     5   // CH6 SB -> arm (up only)

static uint8_t crc8_dvbs2(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}
static inline uint16_t crsfToUs(uint16_t v) { return (uint16_t)((5UL * v) / 8) + 880; }

static void unpackChannels(const uint8_t* p) {
  uint32_t bits = 0; uint8_t nbits = 0, ch = 0;
  for (uint8_t i = 0; i < CRSF_PAYLOAD_RC && ch < 16; i++) {
    bits |= (uint32_t)p[i] << nbits; nbits += 8;
    while (nbits >= 11 && ch < 16) { rcChannels[ch++] = bits & 0x7FF; bits >>= 11; nbits -= 11; }
  }
}

static uint8_t frame[CRSF_FRAME_MAX];
static uint8_t framePos = 0, frameTotal = 0;

static void readCrsf() {
  while (CRSF_PORT.available()) {
    uint8_t b = CRSF_PORT.read();
    if (framePos == 0) {
      if (b == CRSF_ADDR_FC) frame[framePos++] = b;
    } else if (framePos == 1) {
      if (b >= 2 && b <= CRSF_FRAME_MAX - 2) { frame[framePos++] = b; frameTotal = 2 + b; }
      else framePos = 0;
    } else {
      frame[framePos++] = b;
      if (framePos >= frameTotal) {
        uint8_t len = frame[1];
        if (crc8_dvbs2(&frame[2], len - 1) == frame[frameTotal - 1] &&
            frame[2] == CRSF_TYPE_RC_CHANNELS) {
          unpackChannels(&frame[3]); lastFrameMs = millis(); linkUp = true;
        }
        framePos = 0;
      }
    }
  }
}

// ---- Control helpers (same mapping as crsf_test) ----------------------------
static float stickNorm(uint8_t ch) {                 // self-centering axis -> -1..+1
  float n = (crsfToUs(rcChannels[ch]) - 1500.0f) / 512.0f;
  if (fabsf(n) < 0.03f) n = 0.0f;
  return constrain(n, -1.0f, 1.0f);
}
static float throttle01(uint8_t ch) {                // throttle (bottom-rest) -> 0..1
  float d = (crsfToUs(rcChannels[ch]) - 988) / 1024.0f;
  if (d < 0.03f) d = 0.0f;
  return constrain(d, 0.0f, 1.0f);
}
static bool isArmed(uint8_t ch) { return crsfToUs(rcChannels[ch]) > 1700; }  // SB up

// ---- Motor output -----------------------------------------------------------
// cmd in -1..+1: sign = direction, magnitude -> PWM past the deadband.
static void motorWrite(int fwdPin, int revPin, float cmd, int deadband) {
  int pwm = 0;
  if (fabsf(cmd) > CMD_DEADZONE)
    pwm = constrain((int)(deadband + fabsf(cmd) * (MAX_PWM - deadband)), 0, MAX_PWM);
  if (cmd >= 0) { analogWrite(fwdPin, pwm); analogWrite(revPin, 0);   }
  else          { analogWrite(fwdPin, 0);   analogWrite(revPin, pwm); }
}
static void motorsOff() {
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0); analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);  pinMode(LEFT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT); pinMode(RIGHT_MOTOR_REVERSE_PIN, OUTPUT);
  analogWriteFrequency(LEFT_MOTOR_FORWARD_PIN, 20000);   // 20 kHz -> quiet (covers pin 9 too)
  analogWriteFrequency(RIGHT_MOTOR_FORWARD_PIN, 20000);  // covers pin 23 too
  motorsOff();
  CRSF_PORT.begin(CRSF_BAUD);
  delay(300);
  Serial.println("rc_drive: WHEELS UP. Arm with SB(up); CH2 drive, CH1 turn, CH3 speed cap.");
}

void loop() {
  readCrsf();
  if (millis() - lastFrameMs > LINK_TIMEOUT_MS) linkUp = false;

  bool armed = linkUp && isArmed(CH_ARM);

  float left = 0, right = 0;
  if (armed) {
    float cap   = 0.35f + 0.65f * throttle01(CH_SPEED);   // top-speed cap 0.35..1.0
    float drive = -stickNorm(CH_DRIVE) * cap;             // inverted: stick UP = +forward
    float turn  = -stickNorm(CH_TURN) * cap;              // inverted: stick RIGHT = turn right
    left  = constrain(drive + turn, -1.0f, 1.0f);
    right = constrain(drive - turn, -1.0f, 1.0f);
    motorWrite(LEFT_MOTOR_FORWARD_PIN,  LEFT_MOTOR_REVERSE_PIN,  LEFT_DIR  * left,  LEFT_DEADBAND);
    motorWrite(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_REVERSE_PIN, RIGHT_DIR * right, RIGHT_DEADBAND);
  } else {
    motorsOff();   // disarmed or link lost -> hard stop
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    if (!linkUp)      Serial.println("LINK DOWN -> motors OFF");
    else if (!armed)  Serial.println("DISARMED (SB up to arm) -> motors OFF");
    else {
      Serial.print("ARMED  L:"); Serial.print(left, 2);
      Serial.print(" R:");       Serial.print(right, 2);
      Serial.print("   drive "); Serial.print(-stickNorm(CH_DRIVE), 2);
      Serial.print(" turn ");    Serial.print(-stickNorm(CH_TURN), 2);
      Serial.print(" cap ");     Serial.println(0.35f + 0.65f * throttle01(CH_SPEED), 2);
    }
  }
}
