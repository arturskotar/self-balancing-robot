// =============================================================================
// crsf.h - minimal ELRS/CRSF reader + TX16S control mapping for balance_v2.
//
// Reads a RadioMaster RP1 on Serial1 (pins 0/1) @ 420000 8N1. Same parser that
// was hardware-verified in the crsf_test sketch. Non-blocking: call update()
// every loop pass. All state is file-local; the balancer uses the crsf::* API.
//
// Channel map (TX16S):  CH1 turn, CH2 drive, CH3 speed, CH6(SB) arm,
//                       CH7(SC) gain-select, CH8(S1) tune knob.
// =============================================================================
#pragma once

namespace crsf {

static const uint8_t  ADDR_FC         = 0xC8;   // RX frames are addressed to the FC
static const uint8_t  TYPE_RC         = 0x16;   // packed RC-channels frame
static const uint8_t  FRAME_MAX       = 64;
static const uint8_t  PAYLOAD_RC      = 22;     // 16 ch * 11 bits
static const uint32_t LINK_TIMEOUT_MS = 500;

enum { CH_TURN = 0, CH_DRIVE = 1, CH_SPEED = 2, CH_ARM = 5, CH_GAINSEL = 6, CH_TUNE = 7 };

static uint16_t ch[16];
static uint32_t lastFrameMs = 0;
static bool     up = false;

static uint8_t crc8(const uint8_t* d, uint8_t n) {   // poly 0xD5 (DVB-S2)
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}
static inline uint16_t toUs(uint16_t v) { return (uint16_t)((5UL * v) / 8) + 880; }

static void unpack(const uint8_t* p) {   // 22 bytes -> 16 x 11-bit, little-endian
  uint32_t bits = 0; uint8_t nb = 0, c = 0;
  for (uint8_t i = 0; i < PAYLOAD_RC && c < 16; i++) {
    bits |= (uint32_t)p[i] << nb; nb += 8;
    while (nb >= 11 && c < 16) { ch[c++] = bits & 0x7FF; bits >>= 11; nb -= 11; }
  }
}

static uint8_t buf[FRAME_MAX], pos = 0, total = 0;
static void readBytes() {   // sync 0xC8 -> length -> body -> CRC
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (pos == 0) { if (b == ADDR_FC) buf[pos++] = b; }
    else if (pos == 1) {
      if (b >= 2 && b <= FRAME_MAX - 2) { buf[pos++] = b; total = 2 + b; }
      else pos = 0;
    } else {
      buf[pos++] = b;
      if (pos >= total) {
        uint8_t len = buf[1];
        if (crc8(&buf[2], len - 1) == buf[total - 1] && buf[2] == TYPE_RC) {
          unpack(&buf[3]); lastFrameMs = millis(); up = true;
        }
        pos = 0;
      }
    }
  }
}

// ---- public API -------------------------------------------------------------
static void begin()  { Serial1.begin(420000); }
static void update() { readBytes(); if (millis() - lastFrameMs > LINK_TIMEOUT_MS) up = false; }
static bool linkUp() { return up; }
// Armed ONLY when linked AND SB is up -> hard kill on disarm OR link loss.
static bool armed()  { return up && toUs(ch[CH_ARM]) > 1700; }
// SC 3-pos -> 0/1/2 (gain select); S1 knob -> 0..1.
static uint8_t gainSel() { uint16_t u = toUs(ch[CH_GAINSEL]); return u < 1300 ? 0 : (u < 1700 ? 1 : 2); }
static float   knob01()  { return constrain((toUs(ch[CH_TUNE]) - 988) / 1024.0f, 0.0f, 1.0f); }
// CH3 throttle (rests at bottom) -> 0..1 speed cap.
static float   speed()   { float d = (toUs(ch[CH_SPEED]) - 988) / 1024.0f; if (d < 0.03f) d = 0; return constrain(d, 0.0f, 1.0f); }
// Drive/turn. Contract: UP = +forward, RIGHT = +.
// HARDWARE-VERIFIED 2026-08-13: on THIS TX16S setup CH2 reads ABOVE 1500 when the
// stick is pushed forward, so the raw (us-1500) term is already positive there and
// needs NO negation. The extra minus that used to be here made drive() return -1.00
// on a forward stick -- the opposite of the contract one line above -- which showed
// up as TVEL -0.60 (a reverse command) while the pilot was pushing forward.
// NOTE turn() below is a SEPARATE channel with its own TX direction setting; its
// polarity does not follow from this one and has not been verified the same way.
static float drive() { float n = (toUs(ch[CH_DRIVE]) - 1500.0f) / 512.0f; if (fabsf(n) < 0.03f) n = 0; return constrain(n, -1.0f, 1.0f); }
static float turn()  { float n = -(toUs(ch[CH_TURN])  - 1500.0f) / 512.0f; if (fabsf(n) < 0.03f) n = 0; return constrain(n, -1.0f, 1.0f); }

}  // namespace crsf
