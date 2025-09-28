#include <SPI.h>

// Mega2560 + TXB0106 + SST25VF016B (2MB)
// Pins: CS(CE#)=D44, WP#=D46, HOLD#=D48, SPI: MISO=50 MOSI=51 SCK=52

static const uint8_t FLASH_CS   = 44; // CE#
static const uint8_t FLASH_WP   = 46; // WP#
static const uint8_t FLASH_HOLD = 48; // HOLD#

SPISettings flashSPI(500000, MSBFIRST, SPI_MODE0);

// Komutlar
#define CMD_WREN    0x06
#define CMD_RDSR    0x05
#define CMD_EWSR    0x50
#define CMD_WRSR    0x01
#define CMD_CE      0x60
#define CMD_JEDECID 0x9F

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

bool flashChipErase() {
  if (!flashWREN()) return false;
  SPI.beginTransaction(flashSPI);
  fsel(); SPI.transfer(CMD_CE); fdesel();
  SPI.endTransaction();
  // 2MB için tipik 10–30s, max 60s
  return flashWaitBusyTimeout(60000UL);
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

  Serial.println("Chip Erase...");
  if (flashChipErase()) Serial.println("DONE (flash empty)");
  else Serial.println("ERASE TIMEOUT/FAIL");
}

void loop() {
  // boş
}
