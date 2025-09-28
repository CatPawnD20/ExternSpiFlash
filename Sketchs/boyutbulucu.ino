#include <SPI.h>

// Mega2560 + TXB0106 + SST25VF016B (2MB SPI NOR)
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t  FLASH_CS   = 44; // CE#
static const uint8_t  FLASH_WP   = 46; // WP#
static const uint8_t  FLASH_HOLD = 48; // HOLD#
static const uint32_t FLASH_SIZE = 2UL * 1024UL * 1024UL; // 2MB

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

uint32_t detectStoredSize() {
  const uint16_t STEP = 256;         // blok okuma
  uint8_t buf[STEP];
  // Baştan varsayılan: boş
  int64_t last_non_ff = -1;

  // Sondan geriye doğru tara: [FLASH_SIZE-STEP ... 0]
  for (int64_t addr = (int64_t)FLASH_SIZE - STEP; addr >= 0; addr -= STEP) {
    flashRead((uint32_t)addr, buf, STEP);
    // Bu blokta 0xFF olmayan var mı?
    int last = -1;
    for (int i = STEP-1; i >= 0; --i) {
      if (buf[i] != 0xFF) { last = i; break; }
    }
    if (last >= 0) {
      last_non_ff = addr + last;
      break;
    }
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

  uint32_t size = detectStoredSize();
  Serial.print("Detected size: ");
  Serial.print(size);
  Serial.print(" bytes (");
  float kb = size / 1024.0f;
  Serial.print(kb, 2);
  Serial.println(" KB)");

  // Opsiyonel: ilk 64 baytı göster (Intel-HEX ise ':' ile başlar = 0x3A)
  dumpHead(64);
}

void loop() {
  // boş
}
