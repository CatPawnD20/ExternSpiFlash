#include <SPI.h>

// Mega2560 + TXB0106 + SST25VF016B — FOOTER Reader/Verifier
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t  FLASH_CS   = 44; // CE#
static const uint8_t  FLASH_WP   = 46; // WP#
static const uint8_t  FLASH_HOLD = 48; // HOLD#
static const uint32_t FLASH_SIZE = 2UL * 1024UL * 1024UL; // 2MB
static const uint16_t SECTOR_SZ  = 4096;
static const uint32_t MAX_IMAGE_SIZE = FLASH_SIZE - SECTOR_SZ;

SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0);

// Komutlar
#define CMD_READ      0x03
#define CMD_RDSR      0x05
#define CMD_JEDECID   0x9F

// Footer
static const uint32_t FOOTER_BASE   = FLASH_SIZE - SECTOR_SZ;
static const uint32_t FOOTER_ADDR   = FOOTER_BASE;
static const char     FOOTER_MAGIC[] = "EXUPv1"; // 6B
static const uint8_t  FOOTER_LEN     = 16;       // [MAGIC6][RES2][SIZE4][FNV4]
static const char*    g_footerError  = nullptr;

// ---- SPI helpers ----
inline void fsel()   { digitalWrite(FLASH_CS, LOW); }
inline void fdesel() { digitalWrite(FLASH_CS, HIGH); }

void flashRead(uint32_t addr, uint8_t* buf, uint16_t len) {
  SPI.beginTransaction(flashSPI);
  fsel();
  SPI.transfer(CMD_READ);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i=0;i<len;i++) buf[i]=SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
}

void readJEDEC(uint8_t &m, uint8_t &t, uint8_t &c) {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_JEDECID);
  m = SPI.transfer(0xFF); t = SPI.transfer(0xFF); c = SPI.transfer(0xFF);
  fdesel(); SPI.endTransaction();
}

// ---- FNV-1a ----
static const uint32_t FNV_OFFSET = 0x811C9DC5UL;
static const uint32_t FNV_PRIME  = 0x01000193UL;

uint32_t fnv1a_flash(uint32_t size) {
  const uint16_t CH = 256;
  uint8_t tmp[CH];
  uint32_t h = FNV_OFFSET;
  uint32_t addr = 0;
  while (size) {
    uint16_t n = (size > CH) ? CH : size;
    flashRead(addr, tmp, n);
    for (uint16_t i=0;i<n;i++) { h ^= tmp[i]; h *= FNV_PRIME; }
    addr += n;
    size -= n;
  }
  return h;
}

// ---- Footer parse ----
bool readFooter(uint32_t &size, uint32_t &hash) {
  uint8_t f[FOOTER_LEN];
  flashRead(FOOTER_ADDR, f, FOOTER_LEN);
  g_footerError = "FOOTER NOT FOUND or MAGIC MISMATCH";
  if (memcmp(f, FOOTER_MAGIC, 6)!=0) return false;

  if (f[6] != 0 || f[7] != 0) {
    g_footerError = "FOOTER INVALID: RESERVED BYTES ARE NON-ZERO";
    return false;
  }

  size = (uint32_t)f[8]  | ((uint32_t)f[9]  <<8) | ((uint32_t)f[10] <<16) | ((uint32_t)f[11] <<24);
  hash = (uint32_t)f[12] | ((uint32_t)f[13] <<8) | ((uint32_t)f[14] <<16) | ((uint32_t)f[15] <<24);
  if (size == 0) {
    g_footerError = "FOOTER INVALID: SIZE FIELD IS ZERO";
    return false;
  }
  if (size > MAX_IMAGE_SIZE) {
    g_footerError = "FOOTER INVALID: SIZE EXCEEDS ALLOWABLE FLASH RANGE";
    return false;
  }

  g_footerError = nullptr;
  return true;
}

void hexdumpHead(uint32_t n) { // opsiyonel görsel doğrulama
  const uint16_t W=16; uint8_t b[W]; uint32_t a=0;
  while (n) {
    uint16_t take = (n>W)? W : n;
    flashRead(a, b, take);
    Serial.print("0x"); Serial.print(a, HEX); Serial.print(": ");
    for (uint16_t i=0;i<take;i++){ if (b[i]<16) Serial.print('0'); Serial.print(b[i], HEX); Serial.print(' '); }
    Serial.println();
    a += take; n -= take;
  }
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

  uint8_t m,t,c; readJEDEC(m,t,c);
  Serial.print("JEDEC "); Serial.print(m,HEX); Serial.print(' ');
  Serial.print(t,HEX); Serial.print(' '); Serial.println(c,HEX);

  uint32_t size=0, hash=0;
  if (!readFooter(size, hash)) {
    if (g_footerError) Serial.println(g_footerError);
    else Serial.println("FOOTER INVALID");
    return;
  }

  Serial.print("SIZE="); Serial.print(size); Serial.print(" bytes (");
  Serial.print(size/1024.0, 2); Serial.println(" KB)");
  Serial.print("HASH=0x"); Serial.println(hash, HEX);

  // Bütünlük doğrulaması
  uint32_t calc = fnv1a_flash(size);
  Serial.print("VERIFY: ");
  if (calc == hash) Serial.println("OK");
  else {
    Serial.print("FAIL (calc=0x"); Serial.print(calc, HEX); Serial.println(")");
  }

  // Opsiyonel: ilk 64 baytı göster
  hexdumpHead(min((uint32_t)64, size));
}

void loop() { /* boş */ }
