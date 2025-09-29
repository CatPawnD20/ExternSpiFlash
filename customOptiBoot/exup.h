#pragma once
#include <stdint.h>
#include <stdbool.h>

/* SPI flash pinleri (Arduino Mega pinleri → PORT/bit)
   D44 = PL5 (CS#), D46 = PL3 (WP#), D48 = PL1 (HOLD#)
*/
#define FLASH_CS_PORT   PORTL
#define FLASH_CS_DDR    DDRL
#define FLASH_CS_BIT    5

#define FLASH_WP_PORT   PORTL
#define FLASH_WP_DDR    DDRL
#define FLASH_WP_BIT    3

#define FLASH_HOLD_PORT PORTL
#define FLASH_HOLD_DDR  DDRL
#define FLASH_HOLD_BIT  1

// SPI hız: fosc/32 ≈ 500 kHz (TXB0106 güvenli)
void exup_init_spi(void);

// EXUP akışı: footer kontrol + FNV doğrulama + HEX parse + flash programlama.
// true dönerse güncelleme yapılmıştır (bitti → uygulamaya atlanabilir).
bool exup_check_and_update(void);
