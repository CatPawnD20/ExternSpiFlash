#include <SPI.h>
#include <string.h>

// Mega2560 + TXB0106 + SST25VF016B (2MB SPI NOR)
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t  FLASH_CS   = 44; // CE#
static const uint8_t  FLASH_WP   = 46; // WP#
static const uint8_t  FLASH_HOLD = 48; // HOLD#
static const uint32_t FLASH_SIZE = 2UL * 1024UL * 1024UL; // 2MB
static const uint16_t SECTOR_SZ  = 4096;

static const uint32_t FOOTER_BASE    = FLASH_SIZE - SECTOR_SZ;
static const uint32_t FOOTER_ADDR    = FOOTER_BASE;
static const uint8_t  FOOTER_LEN     = 16;
static const char     FOOTER_MAGIC[] = "EXUPv1"; // 6B magic

// FNV-1a constants (mirrors bootloader)
#define FNV_OFFSET 0x811C9DC5UL
#define FNV_PRIME  0x01000193UL

SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0); // güvenli hız

// Komutlar
#define CMD_READ    0x03
#define CMD_RDSR    0x05
#define CMD_JEDECID 0x9F

inline void fsel()   { digitalWrite(FLASH_CS, LOW); }
inline void fdesel() { digitalWrite(FLASH_CS, HIGH); }

uint8_t flashSR() {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_RDSR);
  uint8_t s = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
  return s;
}

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

void flashReadJEDEC(uint8_t &m, uint8_t &t, uint8_t &c) {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_JEDECID);
  m = SPI.transfer(0xFF);
  t = SPI.transfer(0xFF);
  c = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
}

static int hex2nib(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int get_hex_byte(uint8_t hi, uint8_t lo) {
  int a = hex2nib(hi);
  int b = hex2nib(lo);
  if (a < 0 || b < 0) return -1;
  return (a << 4) | b;
}

uint32_t fnv1a_over_flash(uint32_t size) {
  uint8_t tmp[256];
  uint32_t hash = FNV_OFFSET;
  uint32_t addr = 0;
  while (size) {
    uint16_t n = (size > sizeof(tmp)) ? sizeof(tmp) : (uint16_t)size;
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

bool readFooter(uint32_t &size, uint32_t &fnv) {
  uint8_t footer[FOOTER_LEN];
  flashRead(FOOTER_ADDR, footer, FOOTER_LEN);
  if (memcmp(footer, FOOTER_MAGIC, 6) != 0) return false;

  size = (uint32_t)footer[8]
       | ((uint32_t)footer[9]  << 8)
       | ((uint32_t)footer[10] << 16)
       | ((uint32_t)footer[11] << 24);

  fnv = (uint32_t)footer[12]
      | ((uint32_t)footer[13] << 8)
      | ((uint32_t)footer[14] << 16)
      | ((uint32_t)footer[15] << 24);

  if (size == 0 || size > FOOTER_BASE) return false; // footer rezerv alanın sonrasında olamaz
  return true;
}

bool verifyIntelHexImage(uint32_t totalSize, String &reason) {
  uint8_t buf[256];
  uint32_t off = 0;
  uint32_t extBase = 0;
  uint8_t line[128];
  uint8_t data[64];
  uint16_t li = 0;
  bool inLine = false;
  bool seenEOF = false;

  while (off < totalSize) {
    uint16_t n = (uint16_t)((totalSize - off > sizeof(buf)) ? sizeof(buf) : (totalSize - off));
    flashRead(off, buf, n);
    off += n;

    for (uint16_t i = 0; i < n; i++) {
      uint8_t c = buf[i];

      if (!inLine) {
        if (c == ':') {
          inLine = true;
          li = 0;
        }
        continue;
      }

      if (c == '\n' || c == '\r') {
        if (li < 10) {
          inLine = false;
          continue;
        }

        int LL  = get_hex_byte(line[0], line[1]);
        int AHi = get_hex_byte(line[2], line[3]);
        int ALo = get_hex_byte(line[4], line[5]);
        int TT  = get_hex_byte(line[6], line[7]);
        if (LL < 0 || AHi < 0 || ALo < 0 || TT < 0) {
          inLine = false;
          continue;
        }

        uint8_t len = (uint8_t)LL;
        uint16_t need = 8 + (uint16_t)len * 2 + 2;
        if (li < need) {
          reason = F("Record truncated before checksum");
          inLine = false;
          return false;
        }

        uint8_t sum = len + (uint8_t)AHi + (uint8_t)ALo + (uint8_t)TT;

        if (len > sizeof(data)) {
          reason = F("Record data length exceeds buffer");
          inLine = false;
          return false;
        }

        uint16_t p = 8;
        for (uint8_t k = 0; k < len; k++) {
          int b = get_hex_byte(line[p], line[p + 1]);
          if (b < 0) {
            reason = F("Invalid hex digit in data field");
            inLine = false;
            return false;
          }
          data[k] = (uint8_t)b;
          sum += (uint8_t)b;
          p += 2;
        }

        int CC = get_hex_byte(line[p], line[p + 1]);
        if (CC < 0) {
          reason = F("Invalid checksum field");
          inLine = false;
          return false;
        }
        sum += (uint8_t)CC;
        if ((sum & 0xFF) != 0) {
          reason = F("Checksum mismatch");
          inLine = false;
          return false;
        }

        if (TT == 0x00) {
          // data, nothing to do besides validation
        } else if (TT == 0x01) {
          seenEOF = true;
          return true;
        } else if (TT == 0x04) {
          if (len != 2) {
            reason = F("Invalid extended linear address length");
            inLine = false;
            return false;
          }
          uint16_t upper = ((uint16_t)data[0] << 8) | data[1];
          extBase = ((uint32_t)upper) << 16;
          (void)extBase;
        } else {
          // ignore other record types
        }

        inLine = false;
        continue;
      }

      if (li < sizeof(line)) {
        line[li++] = c;
      } else {
        reason = F("Line buffer overflow");
        inLine = false;
        return false;
      }
    }
  }

  reason = seenEOF ? String() : String(F("EOF record missing"));
  return seenEOF;
}

uint32_t detectStoredSize() {
  const uint16_t STEP = 256;         // blok okuma
  uint8_t buf[STEP];
  // Baştan varsayılan: boş
  int64_t last_non_ff = -1;

  // Sondan geriye doğru tara ama footer sektöründen önce dur
  int64_t addr = (int64_t)FOOTER_BASE;
  while (addr > 0) {
    uint16_t chunk = (addr >= STEP) ? STEP : (uint16_t)addr;
    addr -= chunk;
    flashRead((uint32_t)addr, buf, chunk);
    // Bu blokta 0xFF olmayan var mı?
    for (int i = chunk - 1; i >= 0; --i) {
      if (buf[i] != 0xFF) {
        last_non_ff = addr + i;
        break;
      }
    }
    if (last_non_ff >= 0) break;
  }

  if (last_non_ff < 0) return 0;           // tamamen boş (hepsi 0xFF)
  return (uint32_t)(last_non_ff + 1);      // indeks -> toplam byte
}

void dumpHead(uint32_t n) {
  // ilk n baytı hexdump
  const uint16_t CH = 16;
  uint8_t b[CH];
  uint32_t addr = 0;
  while (n) {
    uint16_t take = (n > CH) ? CH : n;
    flashRead(addr, b, take);
    Serial.print("0x"); Serial.print(addr, HEX); Serial.print(": ");
    for (uint16_t i=0;i<take;i++) {
      if (b[i] < 16) Serial.print('0');
      Serial.print(b[i], HEX); Serial.print(' ');
    }
    Serial.println();
    addr += take;
    n -= take;
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

  uint8_t m,t,c; flashReadJEDEC(m,t,c);
  Serial.print("JEDEC "); Serial.print(m,HEX); Serial.print(' ');
  Serial.print(t,HEX); Serial.print(' '); Serial.println(c,HEX);

  uint32_t footerSize = 0;
  uint32_t footerHash = 0;
  if (!readFooter(footerSize, footerHash)) {
    Serial.println("Footer not found or invalid. No verified image present.");
    return;
  }

  Serial.println("Footer found. Verifying hash and Intel HEX records...");

  uint32_t computedHash = fnv1a_over_flash(footerSize);
  if (computedHash != footerHash) {
    Serial.print("FNV mismatch. Expected 0x");
    Serial.print(footerHash, HEX);
    Serial.print(", got 0x");
    Serial.println(computedHash, HEX);
    return;
  }

  String reason;
  if (!verifyIntelHexImage(footerSize, reason)) {
    Serial.print("Intel HEX verification failed: ");
    Serial.println(reason);
    return;
  }

  Serial.println("Image accepted by bootloader rules.");
  Serial.print("Size: ");
  Serial.print(footerSize);
  Serial.print(" bytes (");
  Serial.print(footerSize / 1024.0f, 2);
  Serial.println(" KB)");
  Serial.print("FNV-1a: 0x");
  Serial.println(computedHash, HEX);

  // Opsiyonel: ilk 64 baytı göster (Intel-HEX ise ':' ile başlar = 0x3A)
  dumpHead(64);
}

void loop() {
  // boş
}
