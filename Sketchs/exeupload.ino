#include <SPI.h>

// Mega2560 + TXB0106 + SST25VF016B (2MB) Loader (AAI Word Program + FOOTER)
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t  FLASH_CS   = 44; // CE#
static const uint8_t  FLASH_WP   = 46; // WP#
static const uint8_t  FLASH_HOLD = 48; // HOLD#
static const uint32_t FLASH_SIZE = 2UL * 1024UL * 1024UL; // 2MB
static const uint16_t PAGE_SIZE  = 256;   // host chunk boyutu
static const uint16_t SECTOR_SZ  = 4096;

// Footer yerleşimi (son sektör)
static const uint32_t FOOTER_SECTOR_BASE = FLASH_SIZE - SECTOR_SZ;
static const uint32_t FOOTER_ADDR        = FOOTER_SECTOR_BASE; // sektör başı
static const char     FOOTER_MAGIC[]     = "EXUPv1";           // 6B
static const uint8_t  FOOTER_TOTAL_LEN   = 16;                 // 6 + 2 + 4 + 4

// TXB0106 için güvenli hız: 500 kHz, MODE0
SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0);

// Komutlar
#define CMD_WREN      0x06
#define CMD_WRDI      0x04
#define CMD_RDSR      0x05
#define CMD_WRSR      0x01
#define CMD_EWSR      0x50
#define CMD_READ      0x03
#define CMD_PP_BYTE   0x02      // Byte-Program
#define CMD_AAI       0xAD      // AAI Word Program
#define CMD_SE        0x20
#define CMD_JEDECID   0x9F

inline void fsel()   { digitalWrite(FLASH_CS, LOW); }
inline void fdesel() { digitalWrite(FLASH_CS, HIGH); }

// ---- SR / WIP / WEL ----
uint8_t flashSR() {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_RDSR);
  uint8_t s = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
  return s;
}
bool flashWaitBusyTimeout(uint32_t ms) {
  unsigned long t0 = millis();
  while (flashSR() & 0x01) { if (millis() - t0 > ms) return false; }
  return true;
}
bool flashWREN_and_check() {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_WREN); fdesel();
  SPI.endTransaction();
  for (int i=0;i<8;i++) { if (flashSR() & 0x02) return true; delayMicroseconds(50); }
  return false;
}

// ---- Protect / Unprotect ----
bool flashWriteSR(uint8_t value) {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_EWSR); fdesel();
  SPI.endTransaction();

  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_WRSR); SPI.transfer(value); fdesel();
  SPI.endTransaction();

  delayMicroseconds(50);
  return (flashSR() == value);
}
bool srBpClear(uint8_t sr) { return ((sr & 0b00111100) == 0) && ((sr & 0b10000000) == 0); }

// ---- JEDEC ----
void flashReadJEDEC(uint8_t &m, uint8_t &t, uint8_t &c) {
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_JEDECID);
  m = SPI.transfer(0xFF); t = SPI.transfer(0xFF); c = SPI.transfer(0xFF);
  fdesel();
  SPI.endTransaction();
}

// ---- Erase / Program / Read ----
bool flashEraseSector_safe(uint32_t addr) {
  if (!flashWREN_and_check()) return false;
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

// AAI Word Program (tek/adres hizalı + çiftler + sondaki tek bayt)
bool flashProgram_AAI(uint32_t addr, const uint8_t* data, uint16_t len) {
  if (len == 0) return true;

  // addr tek ise ilk baytı 0x02
  if (addr & 1) {
    if (!flashWREN_and_check()) return false;
    SPI.beginTransaction(flashSPI);
    fsel();
    SPI.transfer(CMD_PP_BYTE);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(data[0]);
    fdesel();
    SPI.endTransaction();
    if (!flashWaitBusyTimeout(10UL)) return false;
    addr += 1; data += 1; len -= 1;
    if (len == 0) return true;
  }

  // AAI başlat (adres + ilk 2 bayt)
  if (len >= 2) {
    if (!flashWREN_and_check()) return false;
    SPI.beginTransaction(flashSPI);
    fsel();
    SPI.transfer(CMD_AAI);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(data[0]);
    SPI.transfer(data[1]);
    fdesel();
    SPI.endTransaction();
    if (!flashWaitBusyTimeout(10UL)) return false;
    addr += 2; data += 2; len  -= 2;

    // AAI devam (adres yok)
    while (len >= 2) {
      SPI.beginTransaction(flashSPI);
      fsel();
      SPI.transfer(CMD_AAI);
      SPI.transfer(data[0]);
      SPI.transfer(data[1]);
      fdesel();
      SPI.endTransaction();
      if (!flashWaitBusyTimeout(10UL)) return false;
      addr += 2; data += 2; len  -= 2;
    }

    // AAI çıkış (WRDI)
    SPI.beginTransaction(flashSPI);
    fsel(); SPI.transfer(CMD_WRDI); fdesel();
    SPI.endTransaction();
    if (!flashWaitBusyTimeout(10UL)) return false;
  }

  // Kalan tek bayt → 0x02
  if (len == 1) {
    if (!flashWREN_and_check()) return false;
    SPI.beginTransaction(flashSPI);
    fsel();
    SPI.transfer(CMD_PP_BYTE);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(data[0]);
    fdesel();
    SPI.endTransaction();
    if (!flashWaitBusyTimeout(10UL)) return false;
  }

  return true;
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

// ---- FNV-1a ----
uint32_t fnv1a_update(uint32_t h, const uint8_t* d, size_t n) {
  for (size_t i=0;i<n;i++) { h ^= d[i]; h *= 16777619UL; }
  return h;
}
uint32_t fnv1a_flash(uint32_t size) {
  const uint16_t BUF=256; uint8_t tmp[BUF];
  uint32_t h=2166136261UL, addr=0;
  while (size) {
    uint16_t n = (size > BUF) ? BUF : size;
    flashRead(addr, tmp, n);
    h = fnv1a_update(h, tmp, n);
    addr += n; size -= n;
  }
  return h;
}

// ---- Intel HEX Yardımcıları ----
int hex2nib(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}
int get_hex_byte(uint8_t hi, uint8_t lo) {
  int a = hex2nib(hi), b = hex2nib(lo);
  if (a < 0 || b < 0) return -1;
  return (a << 4) | b;
}

struct HexParserState {
  bool     inLine;
  bool     seenEOF;
  uint16_t lineLen;
  uint8_t  line[128];
  uint32_t extBase;
};

void hexParserInit(HexParserState &st) {
  st.inLine  = false;
  st.seenEOF = false;
  st.lineLen = 0;
  st.extBase = 0;
}

bool hexParserProcess(HexParserState &st, const uint8_t *buf, uint16_t len, const char* &err) {
  for (uint16_t i = 0; i < len; i++) {
    uint8_t c = buf[i];

    if (st.seenEOF) {
      if (c == '\r' || c == '\n' || c == 0x1A) continue;
      err = "HEX_AFTER_EOF";
      return false;
    }

    if (!st.inLine) {
      if (c == ':') {
        st.inLine = true;
        st.lineLen = 0;
      } else if (c == '\r' || c == '\n') {
        // boş satırlar kabul
      } else {
        err = "HEX_SYNC";
        return false;
      }
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (st.lineLen < 10) {
        st.inLine = false;
        err = "HEX_LINE_SHORT";
        return false;
      }

      int LL  = get_hex_byte(st.line[0], st.line[1]);
      int AHi = get_hex_byte(st.line[2], st.line[3]);
      int ALo = get_hex_byte(st.line[4], st.line[5]);
      int TT  = get_hex_byte(st.line[6], st.line[7]);
      if (LL < 0 || AHi < 0 || ALo < 0 || TT < 0) {
        st.inLine = false;
        err = "HEX_FIELD";
        return false;
      }

      uint16_t need = 8 + (uint16_t)((uint8_t)LL) * 2 + 2;
      if (st.lineLen != need) {
        st.inLine = false;
        err = (st.lineLen < need) ? "HEX_LENGTH" : "HEX_LINE_LONG";
        return false;
      }

      uint8_t sum = (uint8_t)LL + (uint8_t)AHi + (uint8_t)ALo + (uint8_t)TT;
      uint8_t data[64];
      uint8_t lenBytes = (uint8_t)LL;
      if (lenBytes > sizeof(data)) {
        st.inLine = false;
        err = "HEX_DATA_TOO_LONG";
        return false;
      }

      uint16_t p = 8;
      for (uint8_t k = 0; k < lenBytes; k++) {
        int b = get_hex_byte(st.line[p], st.line[p + 1]);
        if (b < 0) {
          st.inLine = false;
          err = "HEX_FIELD";
          return false;
        }
        data[k] = (uint8_t)b;
        sum += (uint8_t)b;
        p += 2;
      }

      int CC = get_hex_byte(st.line[p], st.line[p + 1]);
      if (CC < 0) {
        st.inLine = false;
        err = "HEX_FIELD";
        return false;
      }
      sum += (uint8_t)CC;
      if ((sum & 0xFF) != 0) {
        st.inLine = false;
        err = "HEX_CHECKSUM";
        return false;
      }

      if (TT == 0x00) {
        // data record → adres alanı sadece doğrulama için, dış flash yazımı adres gözetmiyor
        // Üst adres kayıtları sadece sıfırlanır
      } else if (TT == 0x01) {
        if (lenBytes != 0) {
          st.inLine = false;
          err = "HEX_EOF_LEN";
          return false;
        }
        st.seenEOF = true;
      } else if (TT == 0x04) {
        if (lenBytes != 2) {
          st.inLine = false;
          err = "HEX_EXT_LEN";
          return false;
        }
        st.extBase = ((uint32_t)data[0] << 8 | data[1]) << 16;
      } else {
        // diğer kayıt tipleri yoksayılır
      }

      st.inLine = false;
      continue;
    }

    if (st.lineLen >= sizeof(st.line)) {
      st.inLine = false;
      err = "HEX_LINE_LONG";
      return false;
    }
    st.line[st.lineLen++] = c;
  }

  return true;
}

bool hexParserFinalize(const HexParserState &st, const char* &err) {
  if (st.inLine) {
    err = "HEX_PARTIAL";
    return false;
  }
  if (!st.seenEOF) {
    err = "HEX_NO_EOF";
    return false;
  }
  return true;
}

// ---- Host IO ----
bool readExact(uint8_t* buf, size_t n, unsigned long to=15000) {
  size_t got=0; unsigned long t0=millis();
  while (got<n) {
    if (Serial.available()) buf[got++]=Serial.read();
    if (millis()-t0>to) return false;
  }
  return true;
}
uint32_t rdU32LE(const uint8_t* p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

// ---- FOOTER Yaz/Doğrula ----
// footer: [MAGIC 6][RES 2 *0][SIZE LE 4][FNV LE 4]  => toplam 16B
bool writeFooter(uint32_t size, uint32_t hash) {
  // Son sektör temiz değilse sil
  if (!flashEraseSector_safe(FOOTER_SECTOR_BASE)) return false;

  // Footer verisini hazırla
  uint8_t f[FOOTER_TOTAL_LEN];
  memset(f, 0, sizeof(f));
  memcpy(f, FOOTER_MAGIC, 6);           // MAGIC
  // f[6], f[7] = 0x00 (reserved)
  f[8]  = (uint8_t)( size        & 0xFF);
  f[9]  = (uint8_t)((size >> 8)  & 0xFF);
  f[10] = (uint8_t)((size >> 16) & 0xFF);
  f[11] = (uint8_t)((size >> 24) & 0xFF);
  f[12] = (uint8_t)( hash        & 0xFF);
  f[13] = (uint8_t)((hash >> 8)  & 0xFF);
  f[14] = (uint8_t)((hash >> 16) & 0xFF);
  f[15] = (uint8_t)((hash >> 24) & 0xFF);

  // Küçük olduğu için en basit yol: BYTE PROGRAM ile tek tek yaz
  for (uint8_t i=0; i<FOOTER_TOTAL_LEN; i++) {
    if (!flashWREN_and_check()) return false;
    SPI.beginTransaction(flashSPI);
    fsel();
    SPI.transfer(CMD_PP_BYTE);
    SPI.transfer((FOOTER_ADDR + i) >> 16);
    SPI.transfer((FOOTER_ADDR + i) >> 8);
    SPI.transfer((FOOTER_ADDR + i) & 0xFF);
    SPI.transfer(f[i]);
    fdesel();
    SPI.endTransaction();
    if (!flashWaitBusyTimeout(10UL)) return false;
  }
  // Doğrula
  uint8_t r[FOOTER_TOTAL_LEN];
  flashRead(FOOTER_ADDR, r, FOOTER_TOTAL_LEN);
  for (uint8_t i=0;i<FOOTER_TOTAL_LEN;i++) {
    if (r[i]!=f[i]) return false;
  }
  return true;
}

bool readFooter(uint32_t &size, uint32_t &hash) {
  uint8_t r[FOOTER_TOTAL_LEN];
  flashRead(FOOTER_ADDR, r, FOOTER_TOTAL_LEN);
  if (memcmp(r, FOOTER_MAGIC, 6)!=0) return false;
  // reserved geç
  size = (uint32_t)r[8] | ((uint32_t)r[9]<<8) | ((uint32_t)r[10]<<16) | ((uint32_t)r[11]<<24);
  hash = (uint32_t)r[12] | ((uint32_t)r[13]<<8) | ((uint32_t)r[14]<<16) | ((uint32_t)r[15]<<24);
  return true;
}

// ---- Arduino ----
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
  Serial.print("JEDEC "); Serial.print(m,HEX); Serial.print(" ");
  Serial.print(t,HEX); Serial.print(" "); Serial.println(c,HEX);

  if (!flashWriteSR(0x00)) Serial.println("SR_UNPROTECT_FAIL");
  uint8_t sr = flashSR();
  Serial.print("SR="); Serial.println(sr, HEX);
  if (!srBpClear(sr)) { Serial.println("BP_SET"); return; }

  Serial.println("READY");
}

void loop() {
  // Header: "EXUP"(4) + size(LE 4) + fnv(LE 4)
  uint8_t hdr[12];
  if (!readExact(hdr, 12, 30000)) return;
  if (!(hdr[0]=='E'&&hdr[1]=='X'&&hdr[2]=='U'&&hdr[3]=='P')) { Serial.println("BAD"); return; }

  uint32_t fsize = rdU32LE(&hdr[4]);
  uint32_t fhash = rdU32LE(&hdr[8]);
  // Last 4KB sector reserved for footer metadata.
  if (fsize==0 || fsize>FOOTER_SECTOR_BASE) { Serial.println("SIZEERR"); return; }

  // Yalnızca dosyanın kapsadığı sektörleri sil
  uint32_t last = fsize - 1;
  uint32_t s0 = 0 / SECTOR_SZ;
  uint32_t s1 = last / SECTOR_SZ;
  for (uint32_t s=s0; s<=s1; s++) {
    if (!flashEraseSector_safe(s*SECTOR_SZ)) {
      Serial.write(0x15); Serial.println("TIMEOUT_SE"); Serial.println("ERR"); return;
    }
  }

  Serial.println("OK");

  // Host'tan 256B bloklar halinde al; AAI ile yaz ve doğrula
  uint8_t page[PAGE_SIZE], back[PAGE_SIZE];
  uint32_t addr=0, remain=fsize;
  uint32_t runningFNV = 2166136261UL;
  HexParserState parser;
  hexParserInit(parser);
  bool parserFinalized = false;
  while (remain) {
    uint16_t n = (remain > PAGE_SIZE) ? PAGE_SIZE : remain;
    if (!readExact(page, n, 20000)) { Serial.write(0x15); Serial.println("RXERR"); Serial.println("ERR"); return; }

    const char* perr = NULL;
    if (!hexParserProcess(parser, page, n, perr)) {
      Serial.write(0x15);
      Serial.println(perr ? perr : "HEXERR");
      Serial.println("ERR");
      return;
    }

    bool lastChunk = (remain == n);
    if (lastChunk) {
      parserFinalized = true;
      if (!hexParserFinalize(parser, perr)) {
        Serial.write(0x15);
        Serial.println(perr ? perr : "HEXERR");
        Serial.println("ERR");
        return;
      }
    }

    runningFNV = fnv1a_update(runningFNV, page, n);

    if (!flashProgram_AAI(addr, page, n)) { Serial.write(0x15); Serial.println("TIMEOUT_PP"); Serial.println("ERR"); return; }

    // Doğrulama
    flashRead(addr, back, n);
    for (uint16_t i=0;i<n;i++) {
      if (page[i]!=back[i]) {
        Serial.write(0x15);
        Serial.print("MISMATCH @0x"); Serial.println(addr+i, HEX);
        Serial.println("ERR"); return;
      }
    }

    Serial.write(0x06); // ACK
    addr   += n;
    remain -= n;
  }

  if (!parserFinalized) {
    const char* perr = NULL;
    if (!hexParserFinalize(parser, perr)) {
      Serial.write(0x15);
      Serial.println(perr ? perr : "HEXERR");
      Serial.println("ERR");
      return;
    }
  }

  if (runningFNV != fhash) {
    Serial.write(0x15);
    Serial.println("HASH_MISMATCH");
    Serial.println("ERR");
    return;
  }

  // Tüm-dosya hash doğrulaması
  uint32_t calc = fnv1a_flash(fsize);
  if (calc!=fhash) { Serial.println("ERR"); return; }

  // ---- FOOTER yaz ----
  if (!writeFooter(fsize, fhash)) { Serial.println("FOOTER_FAIL"); return; }

  // test için geri oku (opsiyonel)
  uint32_t sz2=0, hs2=0;
  if (readFooter(sz2, hs2) && sz2==fsize && hs2==fhash) Serial.println("FOOTER_OK");

  Serial.println("DONE");
}
