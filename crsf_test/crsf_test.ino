// =============================================================================
// crsf_test.ino - Standalone ExpressLRS / CRSF receiver reader for Teensy 4.1
//
// Purpose: prove the RadioMaster RP1 (ELRS) link works BEFORE wiring CRSF into
// the balancer. Reads RC channels on Serial1 and streams them over USB serial.
// Mirrors the project's test-first pattern (ENCODER_TEST / IMU_TEST / ...).
//
// Wiring (RP1 -> Teensy 4.1):
//   RP1 5V  -> 5V rail
//   RP1 GND -> GND (common)
//   RP1 TX  -> Teensy pin 0  (RX1)   <- channel data in
//   RP1 RX  -> Teensy pin 1  (TX1)   -> telemetry back (unused in this test)
//
// CRSF link: 420000 baud, 8N1, NOT inverted. ELRS signals are 3.3 V -> direct
// to Teensy, no level shifting. (Only the 5V pad is 5 V.)
//
// USB serial monitor @ 115200. Flash with:
//   flash --teensy --sketch crsf_test --all
// =============================================================================

#define CRSF_PORT   Serial1
#define CRSF_BAUD   420000
#define DEBUG_BAUD  115200

// ---- CRSF protocol constants ------------------------------------------------
static const uint8_t  CRSF_ADDR_FC          = 0xC8;  // RX frames are addressed to the "flight controller"
static const uint8_t  CRSF_TYPE_RC_CHANNELS = 0x16;  // packed RC-channels frame
static const uint8_t  CRSF_FRAME_MAX        = 64;    // max total frame bytes
static const uint8_t  CRSF_PAYLOAD_RC       = 22;    // 16 ch * 11 bits = 176 bits = 22 bytes
static const uint32_t LINK_TIMEOUT_MS       = 500;   // no valid frame this long -> link considered down

uint16_t rcChannels[16];        // raw 11-bit CRSF values (172..1811, center ~992)
uint32_t lastFrameMs = 0;
bool     linkUp      = false;

// ---- CRC8, poly 0xD5 (DVB-S2) : CRSF computes this over [type + payload] ----
static uint8_t crc8_dvbs2(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

// 11-bit CRSF value -> microseconds (988..2012 us, 1500 center). Betaflight map.
static inline uint16_t crsfToUs(uint16_t v) { return (uint16_t)((5UL * v) / 8) + 880; }

// ---- Control mapping (0-based channel indices; TX16S AETR) -------------------
//   CH1 = left/right (roll)   -> TURN
//   CH2 = right stick up/down -> DRIVE direction (fwd/back, self-centering -1..+1)
//   CH3 = throttle stick      -> SPEED governor (0..1, scales drive magnitude)
//   CH6 = SB switch           -> ARM / DISARM
//   CH7 = SC 3-pos switch     -> selects WHICH gain the knob tunes
//   CH8 = S1 knob             -> live gain value
// Effective command: drive = direction(CH2) * speed(CH3).
#define CH_TURN      0
#define CH_DRIVE     1
#define CH_SPEED     2
#define CH_ARM       5
#define CH_GAINSEL   6
#define CH_TUNE      7

// Stick channel -> -1..+1 with a small center deadband (kills drift at center).
// For SELF-CENTERING axes (right stick CH1/CH2 rest at 1500).
static float stickNorm(uint8_t ch) {
  float n = (crsfToUs(rcChannels[ch]) - 1500.0f) / 512.0f;   // ~ -1 (988) .. +1 (2012)
  if (fabsf(n) < 0.03f) n = 0.0f;
  return constrain(n, -1.0f, 1.0f);
}

// SPEED governor from the THROTTLE stick (CH3), rests at bottom ~988 -> 0..1.
// Used as a top-speed CAP on the drive command (not a hard multiply), so CH2
// alone can always drive.
static float throttle01(uint8_t ch) {
  float d = (crsfToUs(rcChannels[ch]) - 988) / 1024.0f;      // 0 (988) .. 1 (2012)
  if (d < 0.03f) d = 0.0f;                                   // bottom deadband
  return constrain(d, 0.0f, 1.0f);
}

// ---- Live PID tuning: CH7 switch picks the gain, CH8 (S1 knob) sets it -------
// Each entry is a SAFE clamped window around the shipped value -- the knob can
// only move the gain inside this band, never through 0 / into a topple.
struct TuneGain { const char* name; float lo; float hi; };
static const TuneGain TUNE_GAINS[3] = {
  { "Kp",   1.0f, 3.0f },   // switch pos 0  (shipped 2.0)
  { "Kd",   0.0f, 0.8f },   // switch pos 1  (shipped 0.3) -- needs a 3-pos switch
  { "Kvel", 2.0f, 6.0f },   // switch pos 2  (shipped 4.0)
};
// 3-position switch -> 0 / 1 / 2 (a 2-pos switch reaches only 0 and 2).
static uint8_t switchPos3(uint8_t ch) {
  uint16_t us = crsfToUs(rcChannels[ch]);
  if (us < 1300) return 0;
  if (us < 1700) return 1;
  return 2;
}
// Pot/slider channel -> 0..1.
static float knob01(uint8_t ch) {
  return constrain((crsfToUs(rcChannels[ch]) - 988) / 1024.0f, 0.0f, 1.0f);
}
// ARM switch (SB): armed ONLY in the up position (deliberate). Down/mid = disarmed.
static bool isArmed(uint8_t ch) { return crsfToUs(rcChannels[ch]) > 1700; }

// 22 payload bytes -> 16 channels of 11 bits, little-endian bit order.
static void unpackChannels(const uint8_t* p) {
  uint32_t bits  = 0;
  uint8_t  nbits = 0, ch = 0;
  for (uint8_t i = 0; i < CRSF_PAYLOAD_RC && ch < 16; i++) {
    bits |= (uint32_t)p[i] << nbits;
    nbits += 8;
    while (nbits >= 11 && ch < 16) {
      rcChannels[ch++] = bits & 0x7FF;
      bits >>= 11;
      nbits -= 11;
    }
  }
}

// ---- Frame reader state machine: sync 0xC8 -> length -> body -> CRC ----------
static uint8_t frame[CRSF_FRAME_MAX];
static uint8_t framePos = 0;
static uint8_t frameTotal = 0;   // total bytes expected once length is known

static void readCrsf() {
  while (CRSF_PORT.available()) {
    uint8_t b = CRSF_PORT.read();

    if (framePos == 0) {                       // waiting for sync/address byte
      if (b == CRSF_ADDR_FC) frame[framePos++] = b;
    } else if (framePos == 1) {                // length byte (bytes after it: type+payload+crc)
      if (b >= 2 && b <= CRSF_FRAME_MAX - 2) {
        frame[framePos++] = b;
        frameTotal = 2 + b;                    // addr + len + (b)
      } else {
        framePos = 0;                          // bad length -> resync
      }
    } else {                                   // collecting the rest of the frame
      frame[framePos++] = b;
      if (framePos >= frameTotal) {
        uint8_t len = frame[1];
        uint8_t crc = crc8_dvbs2(&frame[2], len - 1);   // over type + payload
        if (crc == frame[frameTotal - 1]) {             // CRC is the last byte
          if (frame[2] == CRSF_TYPE_RC_CHANNELS) {
            unpackChannels(&frame[3]);
            lastFrameMs = millis();
            linkUp = true;
          }
          // (other frame types -- telemetry requests, link stats -- ignored here)
        }
        framePos = 0;                          // ready for the next frame
      }
    }
  }
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  CRSF_PORT.begin(CRSF_BAUD);                   // Serial1 = pins 0 (RX) / 1 (TX)
  delay(300);
  Serial.println("crsf_test: reading ELRS/CRSF on Serial1 @ 420000. Move the sticks.");
}

void loop() {
  readCrsf();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {           // 10 Hz console update
    lastPrint = millis();
    if (millis() - lastFrameMs > LINK_TIMEOUT_MS) linkUp = false;

    if (!linkUp) {
      // FAILSAFE: no link -> zero motion (what the balancer will act on).
      Serial.println("LINK DOWN -> FAILSAFE (drive=0 turn=0)");
    } else {
      bool  armed = isArmed(CH_ARM);
      // In this TEST the sticks always show so you can verify the mapping. (In the
      // balancer, disarm will force drive/turn to 0 AND cut the motors.)
      // CH2 = full bidirectional drive; CH3 throttle caps top speed but never
      // below SPEED_FLOOR, so CH2 alone always drives.
      const float SPEED_FLOOR = 0.35f;
      float dir   = -stickNorm(CH_DRIVE);                                      // inverted: stick UP = +forward
      float cap   = SPEED_FLOOR + (1.0f - SPEED_FLOOR) * throttle01(CH_SPEED); // 0.35..1.0
      float drive = dir * cap;                                                 // scaled command
      float turn  = -stickNorm(CH_TURN);                                       // inverted: stick RIGHT = turn right
      uint8_t sel = switchPos3(CH_GAINSEL);               // SC: 0/1/2 -> Kp/Kd/Kvel
      const TuneGain& g = TUNE_GAINS[sel];
      float gval  = g.lo + knob01(CH_TUNE) * (g.hi - g.lo);

      Serial.print("ARM:");    Serial.print(armed ? "Y" : "-");
      Serial.print(" DRIVE:"); Serial.print(drive, 2);
      Serial.print(" (dir ");  Serial.print(dir, 2); Serial.print(" cap "); Serial.print(cap, 2); Serial.print(')');
      Serial.print(" TURN:");  Serial.print(turn, 2);
      Serial.print("   TUNE ["); Serial.print(g.name); Serial.print("]=");
      Serial.print(gval, 3);
      Serial.print("  (SC "); Serial.print(sel);
      Serial.print(", knob ");  Serial.print(knob01(CH_TUNE), 2); Serial.print(')');
      // DIAGNOSTIC: raw CH1-8 in us -- push a stick and see which number moves.
      Serial.print("   raw");
      for (uint8_t i = 0; i < 8; i++) { Serial.print(' '); Serial.print(i + 1); Serial.print(':'); Serial.print(crsfToUs(rcChannels[i])); }
      Serial.println();
    }
  }
}
