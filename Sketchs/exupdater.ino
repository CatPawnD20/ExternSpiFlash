#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <string.h>

// Mega2560 + TXB0106 + SST25VF016B — EXUP runtime updater
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t  FLASH_CS   = 44; // CE#
static const uint8_t  FLASH_WP   = 46; // WP#
static const uint8_t  FLASH_HOLD = 48; // HOLD#
static const uint32_t FLASH_SIZE = 2UL * 1024UL * 1024UL; // 2MB
static const uint16_t SECTOR_SZ  = 4096;
static const uint32_t MAX_IMAGE_SIZE = FLASH_SIZE - SECTOR_SZ; // last sector reserved for footer

// Footer location (last 4KB sector)
static const uint32_t FOOTER_SECTOR_BASE = FLASH_SIZE - SECTOR_SZ;
static const uint32_t FOOTER_ADDR        = FOOTER_SECTOR_BASE;
static const char     FOOTER_MAGIC[]     = "EXUPv1";
static const uint8_t  FOOTER_LEN         = 16; // [MAGIC6][RES2][SIZE4][FNV4]

// EEPROM flag that remembers "an update was requested" between resets
static const uint8_t  EEPROM_FLAG_ADDR = 0;
static const uint8_t  EEPROM_FLAG_VALUE = 0xA5;

SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0);

// SPI flash commands
#define CMD_WREN      0x06
#define CMD_RDSR      0x05
#define CMD_READ      0x03
#define CMD_SE        0x20
#define CMD_JEDECID   0x9F

inline void fsel()   { digitalWrite(FLASH_CS, LOW); }
inline void fdesel() { digitalWrite(FLASH_CS, HIGH); }

uint8_t flashSR() {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_RDSR);
  uint8_t status = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
  return status;
}

bool flashWaitBusyTimeout(uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (flashSR() & 0x01) {
    if (millis() - t0 > timeoutMs) {
      return false;
    }
  }
  return true;
}

bool flashWREN_and_check() {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_WREN);
  fdesel();
  SPI.endTransaction();

  for (uint8_t i = 0; i < 8; i++) {
    if (flashSR() & 0x02) {
      return true;
    }
    delayMicroseconds(50);
  }
  return false;
}

void flashRead(uint32_t addr, uint8_t* buf, uint16_t len) {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_READ);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0xFF);
  }
  fdesel();
  SPI.endTransaction();
}

bool flashEraseSector(uint32_t addr) {
  if (!flashWREN_and_check()) {
    return false;
  }

  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_SE);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  fdesel();
  SPI.endTransaction();

  return flashWaitBusyTimeout(1200UL);
}

void readJEDEC(uint8_t &m, uint8_t &t, uint8_t &c) {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_JEDECID);
  m = SPI.transfer(0xFF);
  t = SPI.transfer(0xFF);
  c = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
}

// Fowler–Noll–Vo hash helpers
static const uint32_t FNV_OFFSET = 0x811C9DC5UL;
static const uint32_t FNV_PRIME  = 0x01000193UL;

uint32_t fnv1a_flash(uint32_t size) {
  const uint16_t CHUNK = 256;
  uint8_t tmp[CHUNK];
  uint32_t hash = FNV_OFFSET;
  uint32_t addr = 0;

  while (size) {
    uint16_t n = (size > CHUNK) ? CHUNK : (uint16_t)size;
    flashRead(addr, tmp, n);
    for (uint16_t i = 0; i < n; i++) {
      hash ^= tmp[i];
      hash *= FNV_PRIME;
    }
    addr += n;
    size -= n;
  }
  return hash;
}

bool readFooter(uint32_t &size, uint32_t &hash) {
  uint8_t buf[FOOTER_LEN];
  flashRead(FOOTER_ADDR, buf, FOOTER_LEN);

  if (memcmp(buf, FOOTER_MAGIC, 6) != 0) {
    return false;
  }
  if (buf[6] != 0 || buf[7] != 0) {
    return false;
  }

  size = (uint32_t)buf[8]  | ((uint32_t)buf[9]  << 8) |
         ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);
  hash = (uint32_t)buf[12] | ((uint32_t)buf[13] << 8) |
         ((uint32_t)buf[14] << 16) | ((uint32_t)buf[15] << 24);

  if (size == 0 || size > MAX_IMAGE_SIZE) {
    return false;
  }
  return true;
}

bool eraseFooterSector() {
  return flashEraseSector(FOOTER_SECTOR_BASE);
}

void markUpdateRequested() {
  EEPROM.update(EEPROM_FLAG_ADDR, EEPROM_FLAG_VALUE);
}

void clearUpdateFlag() {
  EEPROM.update(EEPROM_FLAG_ADDR, 0xFF);
}

bool updateWasRequested() {
  return EEPROM.read(EEPROM_FLAG_ADDR) == EEPROM_FLAG_VALUE;
}

void triggerBootloaderUpdate() {
  Serial.println(F("[EXUP] Valid image detected → triggering bootloader"));
  markUpdateRequested();
  delay(50); // allow the log to flush
  wdt_enable(WDTO_15MS);
  while (true) { }
}

void postUpdateCleanup() {
  Serial.println(F("[EXUP] Post-update cleanup running"));
  uint32_t size = 0, hash = 0;
  if (readFooter(size, hash)) {
    uint32_t calc = fnv1a_flash(size);
    if (calc != hash) {
      Serial.println(F("[EXUP] Footer hash mismatch; keeping image for safety"));
      clearUpdateFlag();
      return;
    }
    Serial.println(F("[EXUP] Erasing footer sector to finalize update"));
    if (!eraseFooterSector()) {
      Serial.println(F("[EXUP] Footer erase failed"));
      clearUpdateFlag();
      return;
    }
  } else {
    Serial.println(F("[EXUP] Footer already cleared"));
  }
  clearUpdateFlag();
}

void setup() {
  wdt_disable();
  Serial.begin(115200);
  SPI.begin();

  pinMode(FLASH_CS, OUTPUT);
  pinMode(FLASH_WP, OUTPUT);
  pinMode(FLASH_HOLD, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(FLASH_WP, HIGH);
  digitalWrite(FLASH_HOLD, HIGH);

  uint8_t m = 0, t = 0, c = 0;
  readJEDEC(m, t, c);
  Serial.print(F("[EXUP] JEDEC: 0x"));
  Serial.print(m, HEX);
  Serial.print(' ');
  Serial.print(t, HEX);
  Serial.print(' ');
  Serial.println(c, HEX);

  if (updateWasRequested()) {
    postUpdateCleanup();
  }

  uint32_t size = 0, hash = 0;
  if (!readFooter(size, hash)) {
    Serial.println(F("[EXUP] No update image found"));
    clearUpdateFlag();
    return;
  }

  Serial.print(F("[EXUP] Image size: "));
  Serial.print(size);
  Serial.println(F(" bytes"));
  Serial.print(F("[EXUP] Stored FNV: 0x"));
  Serial.println(hash, HEX);

  uint32_t calc = fnv1a_flash(size);
  Serial.print(F("[EXUP] Calculated FNV: 0x"));
  Serial.println(calc, HEX);

  if (calc != hash) {
    Serial.println(F("[EXUP] Hash mismatch — aborting"));
    clearUpdateFlag();
    return;
  }

  if (!updateWasRequested()) {
    triggerBootloaderUpdate();
  }
}

void loop() {
  // Nothing to do; wait for reset or manual interaction
}
