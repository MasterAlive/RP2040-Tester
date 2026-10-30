// RP Pico tester firmware: USB serial throughput + core-1 CPU/RAM stress + RAM test + flash surface test
// Core: "Raspberry Pi Pico/RP2040" by Earle F. Philhower III (arduino-pico)
//
// Arduino IDE settings:
//   Tools > Board      : Raspberry Pi Pico (or Pico W)
//   Tools > Flash Size : the size your board is SUPPOSED to have (e.g. 2MB). A filesystem partition is NOT required.
//   Tools > USB Stack  : Pico SDK (default)
//
// Commands (text lines from the PC over USB CDC):
//   STREAM      echo frames back (USB throughput test)
//   STREAM CPU  same, plus the CPU/RAM stress test running on core 1
//   FLASH       RAM test + flash surface test + flash size report, ends with "DONE"
//   BOOT        reboot into BOOTSEL (USB mass-storage) mode
//   PING        answers PONG
//
// STREAM mode uses 512-byte frames: seq(4) + payload(504) + CRC32(4).
//   - PC -> Pico frames are CRC-checked here and echoed back (bad frames are counted and dropped).
//   - Once per second the Pico adds a status frame (seq has bit 31 set, payload is ASCII "S key=value ...").
//
// The flash test erases and rewrites everything AFTER the firmware image, up to the real end of the chip.
// The firmware itself is not touched, but anything stored in LittleFS / EEPROM emulation WILL be wiped.

#include <Arduino.h>
#include <pico/bootrom.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef XIP_NOCACHE_NOALLOC_BASE
#define XIP_NOCACHE_NOALLOC_BASE 0x13000000u
#endif

// Linker symbols (weak: a missing symbol becomes address 0 instead of a link error)
extern uint8_t _FS_start __attribute__((weak));
extern uint8_t _FS_end __attribute__((weak));
extern uint8_t __flash_binary_end __attribute__((weak));

// (defined at the top: the Arduino IDE generates function prototypes above everything else)
struct FlashInfo {
  uint8_t id[3];
  uint32_t jedecSize;     // from the JEDEC id (0 = unreadable)
  uint32_t wrapAt;        // where addresses wrap onto the start of the chip (0 = none found, UNKNOWN = could not check)
  uint32_t configured;    // size selected in the Arduino IDE
  uint32_t real;          // detected size
  uint32_t fwEnd;         // end of the firmware image
  uint32_t start, end;    // tested region
  bool fake;              // JEDEC id claims more than the chip really has
  bool ok;
};

// ------------------------------------------------------------------ globals
enum Mode { CMD, STREAM };
static Mode mode = CMD;
static char line[32];
static uint8_t lineLen = 0;

static const size_t FRAME = 512;
static uint32_t crcTab[256];

static uint32_t crc32(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) c = crcTab[(c ^ *p++) & 0xFF] ^ (c >> 8);
  return ~c;
}

// ------------------------------------------------- core 1: CPU / RAM stress
volatile bool cpuEnabled = false;
volatile uint32_t c1Pass = 0, c1eInt = 0, c1eCrc = 0, c1eFlt = 0, c1eRam = 0;

static const uint32_t EXPECT_INT = 0x88C1A6C3u;   // computed on a PC with the same code
static const uint32_t EXPECT_CRC = 0x78ADAC3Cu;

static inline uint32_t rotl32(uint32_t x, int r) { return (x << r) | (x >> (32 - r)); }

// integer ALU, hardware divider (signed + unsigned), 64-bit multiply
static uint32_t cpuIntTest() {
  uint32_t s = 0x12345678u, acc = 0;
  uint64_t a = 0x9E3779B97F4A7C15ull;
  for (int i = 0; i < 4096; ++i) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    uint32_t d = (s >> 7) | 1u;
    acc += s / d;
    acc ^= (s % d) * 2654435761u;
    int32_t sd = (int32_t)((s >> 9) | 1u);
    acc += (uint32_t)((int32_t)s / sd);
    acc ^= (uint32_t)((int32_t)s % sd);
    a = a * 6364136223846793005ull + 1442695040888963407ull;
    acc = rotl32(acc, 5) + (uint32_t)(a >> 32);
    acc ^= (uint32_t)a;
  }
  return acc;
}

// bitwise CRC32 (a different code path than the table CRC used for the stream)
static uint32_t cpuCrcTest() {
  static uint8_t b[1024];
  uint32_t s = 0xC0FFEE11u;
  for (int i = 0; i < 1024; ++i) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; b[i] = (uint8_t)(s >> 11); }
  uint32_t c = 0xFFFFFFFFu;
  for (int i = 0; i < 1024; ++i) {
    c ^= b[i];
    for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
  }
  return ~c;
}

// soft-float double + float (no constant known in advance: compared with the first pass)
static void cpuFloatTest(uint64_t& dbits, uint32_t& fbits) {
  double d = 0.0;
  float f = 0.0f;
  for (int i = 1; i <= 1000; ++i) {
    d += (double)i / (double)(i + 3) * 1.000001 - 0.5 / (double)i;
    f = f * 0.999f + (float)i / (float)(i * 7 + 1);
  }
  memcpy(&dbits, &d, 8);
  memcpy(&fbits, &f, 4);
}

static uint32_t ramBlk[2048];   // 8 KB owned by the core-1 test

static uint32_t ramVal(uint32_t pat, uint32_t i, uint32_t& rs) {
  switch (pat) {
    case 0: return 0x00000000u;
    case 1: return 0xFFFFFFFFu;
    case 2: return 0x55555555u;
    case 3: return 0xAAAAAAAAu;
    case 4: return (uint32_t)(uintptr_t)&ramBlk[i];
    default: rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs;
  }
}

void setup1() {}

void loop1() {
  static bool was = false, haveGold = false;
  static uint64_t goldD = 0;
  static uint32_t goldF = 0;
  if (!cpuEnabled) { was = false; delay(5); return; }
  if (!was) {                       // rising edge: fresh counters
    was = true; haveGold = false;
    c1Pass = c1eInt = c1eCrc = c1eFlt = c1eRam = 0;
  }
  uint32_t pass = c1Pass;
  uint32_t pat = pass % 6;
  uint32_t rs = 0xA5A5A5A5u ^ pass;
  volatile uint32_t* p = ramBlk;
  for (uint32_t i = 0; i < 2048; ++i) p[i] = ramVal(pat, i, rs);

  if (cpuIntTest() != EXPECT_INT) c1eInt++;
  if (cpuCrcTest() != EXPECT_CRC) c1eCrc++;
  uint64_t d; uint32_t f;
  cpuFloatTest(d, f);
  if (!haveGold) { goldD = d; goldF = f; haveGold = true; }
  else if (d != goldD || f != goldF) c1eFlt++;

  rs = 0xA5A5A5A5u ^ pass;          // verify what was written before the other tests ran
  for (uint32_t i = 0; i < 2048; ++i)
    if (p[i] != ramVal(pat, i, rs)) c1eRam++;
  c1Pass = pass + 1;
}

// ------------------------------------------------------------- stream mode
static uint8_t rb[FRAME * 8];
static size_t rbLen = 0;
static uint32_t rxOk = 0, rxBad = 0, statusSeq = 0, lastStatusMs = 0;
static bool inBad = false;

static void sendStatus() {
  uint8_t f[FRAME];
  memset(f, 0, sizeof(f));
  uint32_t seq = 0x80000000u | (statusSeq++ & 0x7FFFFFFFu);
  memcpy(f, &seq, 4);
  int t10 = (int)(analogReadTemp() * 10.0f);
  snprintf((char*)f + 4, FRAME - 8,
           "S up=%lu c1=%d pass=%lu eInt=%lu eCrc=%lu eFlt=%lu eRam=%lu rxok=%lu rxbad=%lu temp=%d.%d",
           (unsigned long)(millis() / 1000), cpuEnabled ? 1 : 0, (unsigned long)c1Pass, (unsigned long)c1eInt,
           (unsigned long)c1eCrc, (unsigned long)c1eFlt, (unsigned long)c1eRam, (unsigned long)rxOk,
           (unsigned long)rxBad, t10 / 10, abs(t10 % 10));
  uint32_t c = crc32(f, FRAME - 4);
  memcpy(f + FRAME - 4, &c, 4);
  Serial.write(f, FRAME);
}

static void startStream(bool cpu) {
  rbLen = 0; rxOk = rxBad = 0; inBad = false; statusSeq = 0;
  lastStatusMs = millis();
  cpuEnabled = cpu;
  mode = STREAM;
}

static void streamStep() {
  int n = Serial.available();
  if (n > 0) {
    size_t room = sizeof(rb) - rbLen;
    if ((size_t)n > room) n = (int)room;
    n = Serial.readBytes((char*)rb + rbLen, n);
    if (n > 0) rbLen += n;
    size_t pos = 0;
    while (rbLen - pos >= FRAME) {
      const uint8_t* f = rb + pos;
      uint32_t c;
      memcpy(&c, f + FRAME - 4, 4);
      if (crc32(f, FRAME - 4) == c) {
        Serial.write(f, FRAME);        // echo the whole good frame
        rxOk++;
        inBad = false;
        pos += FRAME;
      } else {                         // slide one byte to re-sync; count a bad stretch once
        if (!inBad) { rxBad++; inBad = true; }
        pos++;
      }
    }
    if (pos) { memmove(rb, rb + pos, rbLen - pos); rbLen -= pos; }
    if (Serial.available() == 0) Serial.flush();
  }
  uint32_t now = millis();
  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    sendStatus();
  }
}

// -------------------------------------------------------------- RAM test
static uint32_t ramFillCheck(volatile uint32_t* p, size_t w, uint32_t v) {
  for (size_t i = 0; i < w; ++i) p[i] = v;
  uint32_t e = 0;
  for (size_t i = 0; i < w; ++i) if (p[i] != v) e++;
  return e;
}

static uint32_t ramTestBlock(volatile uint32_t* p, size_t w) {
  uint32_t err = 0;
  static const uint32_t pats[] = {0x00000000u, 0xFFFFFFFFu, 0x55555555u, 0xAAAAAAAAu,
                                  0x33333333u, 0xCCCCCCCCu, 0x0F0F0F0Fu, 0xF0F0F0F0u};
  for (size_t k = 0; k < sizeof(pats) / sizeof(pats[0]); ++k) err += ramFillCheck(p, w, pats[k]);
  for (int b = 0; b < 32; ++b) {                       // walking ones / zeros
    err += ramFillCheck(p, w, 1u << b);
    err += ramFillCheck(p, w, ~(1u << b));
  }
  for (size_t i = 0; i < w; ++i) p[i] = (uint32_t)(uintptr_t)&p[i];   // address lines
  for (size_t i = 0; i < w; ++i) if (p[i] != (uint32_t)(uintptr_t)&p[i]) err++;
  uint32_t s = 0x2545F491u;                            // pseudo-random data
  for (size_t i = 0; i < w; ++i) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; p[i] = s; }
  s = 0x2545F491u;
  for (size_t i = 0; i < w; ++i) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; if (p[i] != s) err++; }
  // March C-
  for (size_t i = 0; i < w; ++i) p[i] = 0;
  for (size_t i = 0; i < w; ++i) { if (p[i] != 0) err++; p[i] = 0xFFFFFFFFu; }
  for (size_t i = 0; i < w; ++i) { if (p[i] != 0xFFFFFFFFu) err++; p[i] = 0; }
  for (size_t i = w; i-- > 0;) { if (p[i] != 0) err++; p[i] = 0xFFFFFFFFu; }
  for (size_t i = w; i-- > 0;) { if (p[i] != 0xFFFFFFFFu) err++; p[i] = 0; }
  for (size_t i = 0; i < w; ++i) if (p[i] != 0) err++;
  return err;
}

static uint32_t runRamTest() {
  const size_t CH = 8192, RESERVE = 24 * 1024;
  static uint32_t* blk[64];
  int nb = 0;
  while (nb < 64 && (size_t)rp2040.getFreeHeap() > RESERVE + CH) {
    void* p = malloc(CH);
    if (!p) break;
    blk[nb++] = (uint32_t*)p;
  }
  Serial.printf("INFO RAM test: %d blocks of 8 KB allocated from the free heap (%lu KB heap in total)\n", nb,
                (unsigned long)(rp2040.getTotalHeap() / 1024));
  Serial.flush();
  uint32_t err = 0;
  for (int round = 0; round < 2; ++round)
    for (int i = 0; i < nb; ++i) {
      uint32_t e = ramTestBlock(blk[i], CH / 4);
      if (e) Serial.printf("FAIL RAM block %d at 0x%08lX: %lu errors (round %d)\n", i,
                           (unsigned long)(uintptr_t)blk[i], (unsigned long)e, round + 1);
      err += e;
    }
  for (int i = 0; i < nb; ++i) free(blk[i]);
  Serial.printf("RAM tested %lu KB of 264 KB (the rest holds the firmware, stacks and USB buffers): %lu errors -> %s\n",
                (unsigned long)(nb * CH / 1024), (unsigned long)err, err ? "FAIL" : "PASS");
  return err;
}

// ------------------------------------------------------------- flash test
static const uint32_t SECTOR = FLASH_SECTOR_SIZE;   // 4096
static const uint32_t MAX_FLASH = 16u * 1024u * 1024u;
static const uint32_t UNKNOWN = 0xFFFFFFFFu;
static uint8_t pat[FLASH_SECTOR_SIZE];
static uint8_t badMap[MAX_FLASH / FLASH_SECTOR_SIZE / 8];

static inline const volatile uint8_t* flashPtr(uint32_t off) {
  return (const volatile uint8_t*)(uintptr_t)(XIP_NOCACHE_NOALLOC_BASE + off);   // bypasses the XIP cache
}

static void flashErase(uint32_t off) {
  rp2040.idleOtherCore();
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(off, SECTOR);
  restore_interrupts(ints);
  rp2040.resumeOtherCore();
}
static void flashProgram(uint32_t off, const uint8_t* data) {
  rp2040.idleOtherCore();
  uint32_t ints = save_and_disable_interrupts();
  flash_range_program(off, data, SECTOR);
  restore_interrupts(ints);
  rp2040.resumeOtherCore();
}

// Reads a sector 'reads' times straight from the chip (no cache); returns wrong BITS,
// 'unstable' counts bytes that changed between reads.
static uint32_t verify(uint32_t off, const uint8_t* expect, uint8_t fill, int reads, int32_t& firstBad,
                       uint32_t& unstable) {
  static uint8_t first[FLASH_SECTOR_SIZE];
  uint32_t bits = 0;
  firstBad = -1;
  unstable = 0;
  const volatile uint8_t* p = flashPtr(off);
  for (int r = 0; r < reads; ++r) {
    for (uint32_t i = 0; i < SECTOR; ++i) {
      uint8_t v = p[i];
      uint8_t e = expect ? expect[i] : fill;
      if (r == 0) first[i] = v;
      else if (v != first[i]) unstable++;
      if (v != e) {
        bits += __builtin_popcount((uint8_t)(v ^ e));
        if (firstBad < 0) firstBad = (int32_t)i;
      }
    }
  }
  return bits;
}

static void makeUnique(uint32_t off, uint8_t* out) {
  uint32_t s = (off ^ 0xA5C3E1F7u) * 2654435761u | 1u;
  for (uint32_t i = 0; i < SECTOR; ++i) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; out[i] = (uint8_t)(s >> 8); }
}


static const char* vendorName(uint8_t m) {
  switch (m) {
    case 0xEF: return "Winbond";
    case 0xC8: return "GigaDevice";
    case 0xC2: return "Macronix";
    case 0x85: return "Puya";
    case 0x20: return "XMC/Micron";
    case 0x9D: return "ISSI";
    case 0x0B: return "XTX";
    case 0x68: return "Boya";
    case 0x5E: return "Zbit";
    case 0x1F: return "Adesto";
    case 0x01: return "Spansion/Cypress";
    default: return "unknown";
  }
}

static uint32_t detectWrap() {
  const volatile uint8_t* b0 = flashPtr(0);
  bool useful = false;
  for (int i = 0; i < 256; ++i) if (b0[i] != 0xFF && b0[i] != 0x00) { useful = true; break; }
  if (!useful) return UNKNOWN;
  for (uint32_t n = 64u * 1024u; n < MAX_FLASH; n <<= 1) {
    const volatile uint8_t* b = flashPtr(n);
    bool same = true;
    for (int i = 0; i < 256; ++i) if (b[i] != b0[i]) { same = false; break; }
    if (same) return n;
  }
  return 0;
}

static void probeFlash(FlashInfo& fi) {
  memset(&fi, 0, sizeof(fi));
  uint8_t tx[4] = {0x9F, 0, 0, 0}, rx[4] = {0, 0, 0, 0};
  rp2040.idleOtherCore();
  uint32_t ints = save_and_disable_interrupts();
  flash_do_cmd(tx, rx, 4);
  restore_interrupts(ints);
  rp2040.resumeOtherCore();
  fi.id[0] = rx[1]; fi.id[1] = rx[2]; fi.id[2] = rx[3];
  fi.jedecSize = (fi.id[2] >= 16 && fi.id[2] <= 24) ? (1u << fi.id[2]) : 0;
  fi.wrapAt = detectWrap();
#ifdef PICO_FLASH_SIZE_BYTES
  fi.configured = (uint32_t)(PICO_FLASH_SIZE_BYTES);
#else
  fi.configured = (&_FS_end != nullptr) ? (uint32_t)((uintptr_t)&_FS_end - XIP_BASE) : 0;
#endif
  uint32_t real = fi.jedecSize;
  if (fi.wrapAt && fi.wrapAt != UNKNOWN && (!real || fi.wrapAt < real)) {
    if (real) fi.fake = true;
    real = fi.wrapAt;
  }
  if (!real) real = fi.configured;
  if (real > MAX_FLASH) real = MAX_FLASH;
  fi.real = real;
  fi.fwEnd = (&__flash_binary_end != nullptr) ? (uint32_t)((uintptr_t)&__flash_binary_end - XIP_BASE) : 0;
  if (fi.fwEnd) fi.start = ((fi.fwEnd + SECTOR - 1) / SECTOR) * SECTOR + SECTOR;   // one spare sector
  else if (&_FS_start != nullptr) fi.start = (uint32_t)((uintptr_t)&_FS_start - XIP_BASE);
  else fi.start = 0;
  fi.end = real & ~(SECTOR - 1);
  fi.ok = fi.start > 0 && fi.start + SECTOR <= fi.end;
}

static void printFlashSize(const FlashInfo& fi, uint32_t testedBytes) {
  Serial.println("FLASHSIZE ======== flash size report ========");
  Serial.printf("FLASHSIZE JEDEC id %02X %02X %02X (%s), capacity code 0x%02X = %lu KB\n", fi.id[0], fi.id[1], fi.id[2],
                vendorName(fi.id[0]), fi.id[2], (unsigned long)(fi.jedecSize / 1024));
  if (fi.wrapAt == UNKNOWN) Serial.println("FLASHSIZE address-wrap check: skipped (flash start is blank)");
  else if (fi.wrapAt == 0) Serial.println("FLASHSIZE address-wrap check: no wrap-around found up to 16384 KB");
  else Serial.printf("FLASHSIZE address-wrap check: addresses wrap at %lu KB (= the real chip size)\n", (unsigned long)(fi.wrapAt / 1024));
  Serial.printf("FLASHSIZE size selected in Arduino IDE: %lu KB\n", (unsigned long)(fi.configured / 1024));
  if (fi.fake)
    Serial.println("FLASHSIZE WARNING: the JEDEC id claims more flash than the chip really has (fake/relabelled chip?)");
  if (fi.configured > fi.real)
    Serial.println("FLASHSIZE WARNING: the IDE flash size is larger than the real chip. Select the correct size.");
  if (fi.configured && fi.configured < fi.real)
    Serial.println("FLASHSIZE note: the chip is larger than the IDE flash size; the extra space was tested too.");
  Serial.printf("FLASHSIZE DETECTED FLASH SIZE: %lu KB (%lu.%02lu MB)\n", (unsigned long)(fi.real / 1024),
                (unsigned long)(fi.real / (1024 * 1024)), (unsigned long)((fi.real % (1024 * 1024)) * 100 / (1024 * 1024)));
  Serial.printf("FLASHSIZE firmware area: first %lu KB (cannot be erased while running); surface-tested: %lu KB at 0x%06lX-0x%06lX\n",
                (unsigned long)(fi.start / 1024), (unsigned long)(testedBytes / 1024), (unsigned long)fi.start,
                (unsigned long)fi.end);
}

static bool markBad(uint32_t s) {
  bool was = badMap[s >> 3] & (1 << (s & 7));
  badMap[s >> 3] |= (1 << (s & 7));
  return !was;
}

static void runFlashTest() {
  uint8_t uid[8];
  rp2040.idleOtherCore();
  uint32_t ints = save_and_disable_interrupts();
  flash_get_unique_id(uid);
  restore_interrupts(ints);
  rp2040.resumeOtherCore();
  Serial.printf("INFO board flash unique id: %02X%02X%02X%02X%02X%02X%02X%02X\n", uid[0], uid[1], uid[2], uid[3], uid[4],
                uid[5], uid[6], uid[7]);

  uint32_t ramErr = runRamTest();

  FlashInfo fi;
  probeFlash(fi);
  Serial.printf("INFO flash detected: %lu KB (JEDEC %lu KB, IDE %lu KB), firmware ends at 0x%06lX\n",
                (unsigned long)(fi.real / 1024), (unsigned long)(fi.jedecSize / 1024), (unsigned long)(fi.configured / 1024),
                (unsigned long)fi.fwEnd);
  if (!fi.ok) {
    printFlashSize(fi, 0);
    Serial.println("ERROR nothing to test (could not locate the end of the firmware, or the chip is too small).");
    Serial.println("DONE");
    return;
  }
  const uint32_t sectors = (fi.end - fi.start) / SECTOR;
  Serial.printf("INFO surface test: %lu sectors (%lu KB) from 0x%06lX to 0x%06lX, about 0.5 s per sector\n",
                (unsigned long)sectors, (unsigned long)(sectors * SECTOR / 1024), (unsigned long)fi.start,
                (unsigned long)fi.end);
  Serial.println("INFO per sector: erase+FF check, random, ~random, 0x55, 0xAA, 0x00, unique-address pattern (every write read back 4x from the chip)");
  Serial.println("INFO then: re-read all unique patterns (finds address aliasing / data loss), final erase of everything");
  Serial.flush();

  memset(badMap, 0, sizeof(badMap));
  uint32_t badSectors = 0, totalBits = 0, totalUnstable = 0;
  uint32_t eMin = 0xFFFFFFFFu, eMax = 0, pMin = 0xFFFFFFFFu, pMax = 0;
  uint64_t eSum = 0, pSum = 0, eCnt = 0, pCnt = 0;
  uint32_t t0 = millis();

  // ---- phase A: pattern tests
  for (uint32_t s = 0; s < sectors; ++s) {
    uint32_t off = fi.start + s * SECTOR;
    uint32_t secBits = 0, secUnstable = 0;
    int32_t firstBad = -1, fb;
    uint32_t us;
    const char* failStep = nullptr;
    for (int step = 0; step <= 6; ++step) {
      uint32_t a = micros();
      flashErase(off);
      uint32_t te = micros() - a;
      eMin = min(eMin, te); eMax = max(eMax, te); eSum += te; eCnt++;
      uint32_t bits = verify(off, nullptr, 0xFF, 4, fb, us);
      if ((bits || us) && !failStep) failStep = "erase";
      secBits += bits; secUnstable += us;
      if (firstBad < 0 && fb >= 0) firstBad = fb;
      if (step == 0) continue;
      uint32_t seed = ((off * 2654435761u) ^ ((uint32_t)step * 0x9E3779B9u)) | 1u;
      if (step == 6) makeUnique(off, pat);
      else for (uint32_t i = 0; i < SECTOR; ++i) {
        switch (step) {
          case 1: seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; pat[i] = (uint8_t)(seed >> 8); break;
          case 2: seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; pat[i] = (uint8_t)~(seed >> 8); break;
          case 3: pat[i] = 0x55; break;
          case 4: pat[i] = 0xAA; break;
          default: pat[i] = 0x00; break;
        }
      }
      a = micros();
      flashProgram(off, pat);
      uint32_t tp = micros() - a;
      pMin = min(pMin, tp); pMax = max(pMax, tp); pSum += tp; pCnt++;
      bits = verify(off, pat, 0, 4, fb, us);
      if ((bits || us) && !failStep) failStep = "program";
      secBits += bits; secUnstable += us;
      if (firstBad < 0 && fb >= 0) firstBad = fb;
    }
    if (secBits || secUnstable) {
      if (markBad(s)) badSectors++;
      Serial.printf("FAIL sector %lu (0x%06lX): %s step, %lu bad bits, %lu unstable reads, first bad byte +0x%03lX\n",
                    (unsigned long)s, (unsigned long)off, failStep ? failStep : "?", (unsigned long)secBits,
                    (unsigned long)secUnstable, (unsigned long)(firstBad < 0 ? 0 : firstBad));
    }
    totalBits += secBits; totalUnstable += secUnstable;
    if (((s + 1) % 8) == 0 || s + 1 == sectors)
      Serial.printf("PROGRESS phase 1/3: %lu/%lu sectors (%lu%%), bad sectors so far: %lu\n", (unsigned long)(s + 1),
                    (unsigned long)sectors, (unsigned long)((s + 1) * 100 / sectors), (unsigned long)badSectors);
  }

  // ---- phase B: every sector still holds its own unique data (aliasing / retention)
  uint32_t aliasErr = 0;
  for (uint32_t s = 0; s < sectors; ++s) {
    uint32_t off = fi.start + s * SECTOR, us;
    int32_t fb;
    makeUnique(off, pat);
    uint32_t bits = verify(off, pat, 0, 2, fb, us);
    if (bits || us) {
      aliasErr++;
      totalBits += bits; totalUnstable += us;
      if (markBad(s)) badSectors++;
      Serial.printf("FAIL sector %lu (0x%06lX): unique pattern changed (address aliasing or data loss), %lu bad bits\n",
                    (unsigned long)s, (unsigned long)off, (unsigned long)bits);
    }
    if (((s + 1) % 64) == 0 || s + 1 == sectors)
      Serial.printf("PROGRESS phase 2/3: %lu/%lu sectors\n", (unsigned long)(s + 1), (unsigned long)sectors);
  }

  // ---- phase C: erase everything, must read back 0xFF
  for (uint32_t s = 0; s < sectors; ++s) {
    uint32_t off = fi.start + s * SECTOR, us;
    int32_t fb;
    uint32_t a = micros();
    flashErase(off);
    uint32_t te = micros() - a;
    eMin = min(eMin, te); eMax = max(eMax, te); eSum += te; eCnt++;
    uint32_t bits = verify(off, nullptr, 0xFF, 2, fb, us);
    if (bits || us) {
      totalBits += bits; totalUnstable += us;
      if (markBad(s)) badSectors++;
      Serial.printf("FAIL sector %lu (0x%06lX): final erase check, %lu bad bits\n", (unsigned long)s, (unsigned long)off,
                    (unsigned long)bits);
    }
    if (((s + 1) % 64) == 0 || s + 1 == sectors)
      Serial.printf("PROGRESS phase 3/3: %lu/%lu sectors\n", (unsigned long)(s + 1), (unsigned long)sectors);
  }

  bool flashPass = (badSectors == 0 && totalUnstable == 0 && totalBits == 0);
  Serial.printf("INFO erase time   min/avg/max: %lu / %lu / %lu us per 4KB sector\n", (unsigned long)eMin,
                (unsigned long)(eSum / (eCnt ? eCnt : 1)), (unsigned long)eMax);
  Serial.printf("INFO program time min/avg/max: %lu / %lu / %lu us per 4KB sector\n", (unsigned long)pMin,
                (unsigned long)(pSum / (pCnt ? pCnt : 1)), (unsigned long)pMax);
  Serial.printf("SUMMARY flash: %lu KB tested in %lu s: %lu bad sectors, %lu bad bits, %lu unstable reads, %lu aliasing errors -> %s\n",
                (unsigned long)(sectors * SECTOR / 1024), (unsigned long)((millis() - t0) / 1000), (unsigned long)badSectors,
                (unsigned long)totalBits, (unsigned long)totalUnstable, (unsigned long)aliasErr, flashPass ? "PASS" : "FAIL");
  Serial.printf("SUMMARY RAM: %lu errors -> %s\n", (unsigned long)ramErr, ramErr ? "FAIL" : "PASS");
  Serial.println("INFO each tested sector saw 8 erase cycles (flash is rated about 100000); the region is left erased.");
  printFlashSize(fi, sectors * SECTOR);
  Serial.printf("RESULT %s\n", (flashPass && ramErr == 0 && !fi.fake) ? "PASS" : "FAIL");
  Serial.println("DONE");
  Serial.flush();
}

// ------------------------------------------------------------ command mode
static void handleLine(const char* l) {
  if (!strcmp(l, "STREAM")) startStream(false);
  else if (!strcmp(l, "STREAM CPU")) startStream(true);
  else if (!strcmp(l, "FLASH")) runFlashTest();
  else if (!strcmp(l, "PING")) Serial.println("PONG");
  else if (!strcmp(l, "BOOT")) {
    Serial.println("INFO rebooting into BOOTSEL mode");
    Serial.flush();
    delay(100);
    reset_usb_boot(0, 0);
  }
}

void setup() {
  Serial.begin(115200);       // baud rate is irrelevant for USB CDC
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crcTab[i] = c;
  }
}

void loop() {
  if (!Serial) {              // host closed the port (DTR low): back to command mode
    mode = CMD;
    cpuEnabled = false;
    lineLen = 0;
    rbLen = 0;
    return;
  }
  if (mode == STREAM) {
    streamStep();
    return;
  }
  while (Serial.available() > 0 && mode == CMD) {
    int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = 0;
      lineLen = 0;
      handleLine(line);
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = (char)c;
    } else {
      lineLen = 0;            // overlong garbage
    }
  }
}
