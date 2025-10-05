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
static const char*    g_parserError  = nullptr;

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

// ---- Intel HEX helpers ----
static int hex2nib(uint8_t c) {
  if (c>='0' && c<='9') return c-'0';
  if (c>='A' && c<='F') return c-'A'+10;
  if (c>='a' && c<='f') return c-'a'+10;
  return -1;
}

static int get_hex_byte(uint8_t hi, uint8_t lo) {
  int a = hex2nib(hi);
  int b = hex2nib(lo);
  if (a < 0 || b < 0) return -1;
  return (a << 4) | b;
}

static bool verifyIntelHexPayload(uint32_t total_size) {
  uint8_t buf[256];
  uint32_t off = 0;
  uint32_t ext_base = 0;
  uint8_t line[128];
  uint8_t data[64];
  uint16_t li = 0;
  bool in_line = false;
  bool seen_eof = false;

  g_parserError = "HEX PARSER: EOF RECORD NOT FOUND";

  while (off < total_size) {
    uint16_t n = (uint16_t)((total_size - off > sizeof(buf)) ? sizeof(buf) : (total_size - off));
    flashRead(off, buf, n);
    off += n;

    for (uint16_t i = 0; i < n; i++) {
      uint8_t c = buf[i];

      if (!in_line) {
        if (c == ':') { in_line = true; li = 0; }
        continue;
      }

      if (c == '\n' || c == '\r') {
        if (li < 10) { in_line = false; continue; }

        int LL  = get_hex_byte(line[0], line[1]);
        int AHi = get_hex_byte(line[2], line[3]);
        int ALo = get_hex_byte(line[4], line[5]);
        int TT  = get_hex_byte(line[6], line[7]);
        if (LL < 0 || AHi < 0 || ALo < 0 || TT < 0) {
          g_parserError = "HEX PARSER: INVALID RECORD HEADER";
          in_line = false;
          return false;
        }

        uint16_t addr16 = ((uint16_t)AHi << 8) | (uint16_t)ALo;
        uint8_t len = (uint8_t)LL;

        uint16_t need = 8 + (uint16_t)len * 2 + 2;
        if (li < need) {
          g_parserError = "HEX PARSER: TRUNCATED RECORD";
          in_line = false;
          return false;
        }

        uint8_t sum = len + AHi + ALo + (uint8_t)TT;

        if (len > sizeof(data)) {
          g_parserError = "HEX PARSER: DATA FIELD TOO LONG";
          in_line = false;
          return false;
        }

        uint16_t p = 8;
        for (uint8_t k = 0; k < len; k++) {
          int b = get_hex_byte(line[p], line[p+1]);
          if (b < 0) {
            g_parserError = "HEX PARSER: INVALID DATA BYTE";
            in_line = false;
            return false;
          }
          data[k] = (uint8_t)b;
          sum += (uint8_t)b;
          p += 2;
        }

        int CC = get_hex_byte(line[p], line[p+1]);
        if (CC < 0) {
          g_parserError = "HEX PARSER: INVALID CHECKSUM";
          in_line = false;
          return false;
        }
        sum += (uint8_t)CC;
        if ((sum & 0xFF) != 0) {
          g_parserError = "HEX PARSER: CHECKSUM MISMATCH";
          in_line = false;
          return false;
        }

        if (TT == 0x00) {
          (void)addr16;
          for (uint8_t k = 0; k < len; k++) {
            (void)data[k];
          }
        } else if (TT == 0x01) {
          seen_eof = true;
          g_parserError = nullptr;
          return true;
        } else if (TT == 0x04) {
          if (len != 2) {
            g_parserError = "HEX PARSER: INVALID EXTENDED ADDRESS";
            in_line = false;
            return false;
          }
          uint16_t upper = ((uint16_t)data[0] << 8) | data[1];
          ext_base = ((uint32_t)upper) << 16;
          (void)ext_base;
        }

        in_line = false;
        continue;
      }

      if (li < sizeof(line)) line[li++] = c;
      else {
        g_parserError = "HEX PARSER: LINE OVERFLOW";
        in_line = false;
        return false;
      }
    }
  }

  if (seen_eof) {
    g_parserError = nullptr;
    return true;
  }
  g_parserError = "HEX PARSER: EOF RECORD NOT FOUND";
  return false;
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
    Serial.println("REJECT: FOOTER MISSING OR INVALID");
    if (g_footerError) Serial.println(g_footerError);
    return;
  }

  Serial.print("SIZE="); Serial.print(size); Serial.print(" bytes (");
  Serial.print(size/1024.0, 2); Serial.println(" KB)");
  Serial.print("HASH=0x"); Serial.println(hash, HEX);

  // Bütünlük doğrulaması
  uint32_t calc = fnv1a_flash(size);
  if (calc != hash) {
    Serial.print("REJECT: FNV MISMATCH (calc=0x");
    Serial.print(calc, HEX);
    Serial.println(")");
    return;
  }

  Serial.println("FNV MATCH: OK");

  if (!verifyIntelHexPayload(size)) {
    Serial.println("REJECT: INTEL HEX PARSE FAILURE");
    if (g_parserError) Serial.println(g_parserError);
    return;
  }

  Serial.println("HEX PARSE: OK");

  // Opsiyonel: ilk 64 baytı göster
  hexdumpHead(min((uint32_t)64, size));
}

void loop() { /* boş */ }
