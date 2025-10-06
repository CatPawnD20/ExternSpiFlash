#include <SPI.h>
#include <string.h>

// Mega2560 + TXB0106 + SST25VF016B (2MB)
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t FLASH_CS   = 44; // CE#
static const uint8_t FLASH_WP   = 46; // WP#
static const uint8_t FLASH_HOLD = 48; // HOLD#

SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0);

// Bellek düzeni (bootloader ile eşleşir)
#define FLASH_SIZE  (2UL * 1024UL * 1024UL)
#define SECTOR_SZ   4096UL
#define FOOTER_ADDR (FLASH_SIZE - SECTOR_SZ)

// Komutlar
#define CMD_WREN      0x06
#define CMD_RDSR      0x05
#define CMD_EWSR      0x50
#define CMD_WRSR      0x01
#define CMD_SE        0x20
#define CMD_READ      0x03
#define CMD_JEDECID   0x9F

static const uint8_t FOOTER_MAGIC[6] = {'E','X','U','P','v','1'};

inline void fsel()   { digitalWrite(FLASH_CS, LOW); }
inline void fdesel() { digitalWrite(FLASH_CS, HIGH); }

uint8_t flashSR() {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_RDSR);
  uint8_t s = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
  return s;
}

bool flashWaitBusyTimeout(uint32_t ms) {
  unsigned long t0 = millis();
  while (flashSR() & 0x01) { // WIP=1
    if (millis() - t0 > ms) return false;
  }
  return true;
}

bool flashWREN() {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_WREN); fdesel();
  SPI.endTransaction();
  delayMicroseconds(10);
  return (flashSR() & 0x02); // WEL=1?
}

bool flashEraseSector(uint32_t addr) {
  if (!flashWREN()) return false;
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

void flashRead(uint32_t addr, uint8_t *buf, uint16_t len) {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_READ);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) buf[i] = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
}

bool footerIsValid(const uint8_t *footer, uint32_t &size_out, uint32_t &fnv_out) {
  if (memcmp(footer, FOOTER_MAGIC, sizeof(FOOTER_MAGIC)) != 0) return false;
  uint32_t sz = (uint32_t)footer[8]  | ((uint32_t)footer[9]  << 8) |
                ((uint32_t)footer[10] << 16) | ((uint32_t)footer[11] << 24);
  uint32_t hv = (uint32_t)footer[12] | ((uint32_t)footer[13] << 8) |
                ((uint32_t)footer[14] << 16) | ((uint32_t)footer[15] << 24);
  if (sz == 0 || sz > (FLASH_SIZE - SECTOR_SZ)) return false;
  size_out = sz;
  fnv_out = hv;
  return true;
}

// Global unprotect (EWSR + WRSR(0x00))
bool flashUnprotect() {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_EWSR); fdesel();
  SPI.endTransaction();

  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_WRSR); SPI.transfer(0x00); fdesel();
  SPI.endTransaction();

  delayMicroseconds(50);
  return ((flashSR() & 0x3C) == 0); // BP bitleri temiz?
}

void setup() {
  Serial.begin(115200);
  SPI.begin();

  pinMode(FLASH_CS, OUTPUT);
  pinMode(FLASH_WP, OUTPUT);
  pinMode(FLASH_HOLD, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(FLASH_WP, HIGH);
  digitalWrite(FLASH_HOLD, HIGH);

  uint8_t m,t,c;
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_JEDECID);
  m=SPI.transfer(0xFF); t=SPI.transfer(0xFF); c=SPI.transfer(0xFF);
  fdesel(); SPI.endTransaction();

  Serial.print("JEDEC "); Serial.print(m,HEX); Serial.print(" ");
  Serial.print(t,HEX); Serial.print(" "); Serial.println(c,HEX);

  if (!flashUnprotect()) {
    Serial.println("Unprotect failed!");
    return;
  }
  Serial.println("Unprotected");

  Serial.println("Chip erase replaced with sector erase below footer.");
  Serial.println("Send 'F' within 5s to ERASE footer metadata sector as well.");
  Serial.println("Any other input (or timeout) preserves footer.");

  unsigned long waitStart = millis();
  bool eraseFooter = false;
  while (millis() - waitStart < 5000UL) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'F' || c == 'f') {
        eraseFooter = true;
        break;
      }
    }
  }

  if (eraseFooter) Serial.println("Operator request: footer sector WILL be erased.");
  else Serial.println("Footer sector preserved (metadata should remain).");

  Serial.println("Erasing data sectors...");
  for (uint32_t addr = 0; addr < FOOTER_ADDR; addr += SECTOR_SZ) {
    Serial.print(" - Sector 0x");
    Serial.print(addr, HEX);
    if (flashEraseSector(addr)) Serial.println(" erased");
    else {
      Serial.println(" ERASE TIMEOUT/FAIL");
      return;
    }
  }

  if (eraseFooter) {
    Serial.print(" - Footer sector 0x");
    Serial.print(FOOTER_ADDR, HEX);
    if (flashEraseSector(FOOTER_ADDR)) Serial.println(" erased");
    else {
      Serial.println(" ERASE TIMEOUT/FAIL");
      return;
    }
  } else {
    Serial.println("Footer sector left intact.");
  }

  uint8_t footer[16];
  uint32_t footerSize = 0;
  uint32_t footerFnv = 0;
  flashRead(FOOTER_ADDR, footer, sizeof(footer));
  if (footerIsValid(footer, footerSize, footerFnv)) {
    Serial.print("Footer status: EXUPv1 metadata present, size=");
    Serial.print(footerSize);
    Serial.print(" bytes, fnv=0x");
    Serial.println(footerFnv, HEX);
  } else {
    Serial.println("Footer status: no valid EXUPv1 record detected.");
  }
}

void loop() {
  // boş
}
