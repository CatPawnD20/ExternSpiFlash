#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>  // cli(), sei()
#include "exup.h"
#include <stdbool.h>
#include <avr/wdt.h>

/* --- SPI low-level --- */
static inline uint8_t spi_txrx(uint8_t v){
    SPDR = v; while(!(SPSR & _BV(SPIF))); return SPDR;
}
void exup_init_spi(void){
    // MOSI(PB2), SCK(PB1), SS(PB0) out; MISO(PB3) in
    DDRB |= _BV(PB2) | _BV(PB1) | _BV(PB0);
    // CS/WP/HOLD out ve HIGH
    FLASH_CS_DDR   |= _BV(FLASH_CS_BIT);
    FLASH_WP_DDR   |= _BV(FLASH_WP_BIT);
    FLASH_HOLD_DDR |= _BV(FLASH_HOLD_BIT);
    FLASH_CS_PORT   |= _BV(FLASH_CS_BIT);
    FLASH_WP_PORT   |= _BV(FLASH_WP_BIT);
    FLASH_HOLD_PORT |= _BV(FLASH_HOLD_BIT);

    // Master, fosc/32 → SPR1=1, SPI2X=1
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1);
    SPSR = _BV(SPI2X);
}
static inline void cs_low(void){  FLASH_CS_PORT &= ~_BV(FLASH_CS_BIT); }
static inline void cs_high(void){ FLASH_CS_PORT |=  _BV(FLASH_CS_BIT); }

/* --- Flash READ (0x03) --- */
static void flash_read(uint32_t addr, uint8_t *buf, uint16_t len){
    cs_low();
    spi_txrx(0x03);
    spi_txrx((addr >> 16) & 0xFF);
    spi_txrx((addr >> 8)  & 0xFF);
    spi_txrx(addr & 0xFF);
    for (uint16_t i=0;i<len;i++) buf[i]=spi_txrx(0xFF);
    cs_high();
}

/* --- Footer: [MAGIC "EXUPv1"(6)][RES2][SIZE LE4][FNV LE4] son 4KB sektör başında --- */
#define FLASH_SIZE     (2UL*1024UL*1024UL)
#define SECTOR_SZ      4096UL
#define FOOTER_ADDR    (FLASH_SIZE - SECTOR_SZ)
static const uint8_t MAGIC[6] = {'E','X','U','P','v','1'};

static bool read_footer(uint32_t *size_out, uint32_t *fnv_out){
    uint8_t f[16];
    flash_read(FOOTER_ADDR, f, 16);
    if (memcmp(f, MAGIC, 6) != 0) return false;
    uint32_t sz = (uint32_t)f[8] | ((uint32_t)f[9]<<8) | ((uint32_t)f[10]<<16) | ((uint32_t)f[11]<<24);
    uint32_t hv = (uint32_t)f[12]| ((uint32_t)f[13]<<8)| ((uint32_t)f[14]<<16)| ((uint32_t)f[15]<<24);
    if (sz == 0 || sz > FLASH_SIZE - SECTOR_SZ) return false;
    *size_out = sz; *fnv_out = hv; return true;
}

/* --- FNV-1a (SPI üstünden) --- */
#define FNV_OFFSET 0x811C9DC5UL
#define FNV_PRIME  0x01000193UL
static uint32_t fnv1a_over_spi(uint32_t size){
    uint8_t tmp[256]; uint32_t h=FNV_OFFSET, a=0;
    while(size){
        uint16_t n = (size>sizeof(tmp)) ? sizeof(tmp) : (uint16_t)size;
        flash_read(a, tmp, n);
        for(uint16_t i=0;i<n;i++){ h ^= tmp[i]; h *= FNV_PRIME; }
        a += n; size -= n;
    }
    return h;
}

/* --- Uygulamaya yazım güvenliği --- */
/* 4KB boot (LDSECTIONS .text=0x3F000) → uygulama alanı 0x00000–0x3EFFF */
#define APP_FLASH_END  (0x3F000UL)
#ifndef SPM_PAGESIZE
#  define SPM_PAGESIZE 512           // ATmega2560 için 512 byte
#endif

static uint8_t pagebuf[SPM_PAGESIZE];
static uint32_t cur_page_base = 0xFFFFFFFF;
static bool page_dirty = false;

static void page_erase_write(uint32_t addr_base){
    // Sadece uygulama alanı
    if (addr_base >= APP_FLASH_END) return;

    // Erase
    boot_page_erase(addr_base);
    boot_spm_busy_wait();
    // Fill
    for (uint16_t i=0;i<SPM_PAGESIZE;i+=2){
        uint16_t w = pagebuf[i] | ((uint16_t)pagebuf[i+1] << 8);
        boot_page_fill(addr_base + i, w);
    }
    // Write
    boot_page_write(addr_base);
    boot_spm_busy_wait();
    boot_rww_enable();
}

static void page_commit_if_dirty(void){
    if (page_dirty){
        page_erase_write(cur_page_base);
        page_dirty = false;
    }
}
static void page_switch(uint32_t new_base){
    if (new_base == cur_page_base) return;
    page_commit_if_dirty();
    cur_page_base = new_base;
    // Sayfa tamponunu 0xFF ile temizle
    for (uint16_t i=0;i<SPM_PAGESIZE;i++) pagebuf[i]=0xFF;
}

/* --- Hex yardımcıları --- */
static int hex2nib(uint8_t c){
    if (c>='0' && c<='9') return c-'0';
    if (c>='A' && c<='F') return c-'A'+10;
    if (c>='a' && c<='f') return c-'a'+10;
    return -1;
}
static int get_hex_byte(uint8_t hi, uint8_t lo){
    int a=hex2nib(hi), b=hex2nib(lo);
    if (a<0 || b<0) return -1;
    return (a<<4)|b;
}

/* --- SPI üzerinden Intel HEX parse & program --- */
static bool apply_hex_from_spi(uint32_t total_size){
    // 256B blok blok oku; satır satır çöz
    uint8_t buf[256];
    uint32_t off = 0;
    uint32_t ext_base = 0;      // 0x04 (ELA) ile gelen üst 16 bit << 16
    uint8_t  line[128];         // uzun satırlar için rahat tampon
    uint8_t  data[64];          // satır başına veri (64B yeterli)
    uint16_t li = 0;
    bool in_line = false;
    bool seen_eof = false;

    // sayfa durumunu temizle
    cur_page_base = 0xFFFFFFFF;
    page_dirty = false;

    while (off < total_size){
        uint16_t n = (uint16_t)((total_size - off > sizeof(buf)) ? sizeof(buf) : (total_size - off));
        flash_read(off, buf, n);
        off += n;

        for (uint16_t i = 0; i < n; i++){
            uint8_t c = buf[i];

            if (!in_line){
                if (c == ':'){ in_line = true; li = 0; }
                continue;
            }

            if (c == '\n' || c == '\r'){
                // en az :LLAAAATTCC (':' yok, bu yüzden 10 yerine 8 kontrol ediyoruz)
                if (li < 10){ in_line = false; continue; }

                int LL  = get_hex_byte(line[0], line[1]);
                int AHi = get_hex_byte(line[2], line[3]);
                int ALo = get_hex_byte(line[4], line[5]);
                int TT  = get_hex_byte(line[6], line[7]);
                if (LL < 0 || AHi < 0 || ALo < 0 || TT < 0){ in_line = false; continue; }

                uint16_t addr16 = ((uint16_t)AHi << 8) | (uint16_t)ALo;
                uint8_t  len    = (uint8_t)LL;

                // veri + checksum için yeterli uzunluk var mı?
                // baştan 8 hex char (LL AAAA TT) + 2*len + 2(CC)
                uint16_t need = 8 + (uint16_t)len * 2 + 2;
                if (li < need){ in_line = false; return false; }

                // checksum topla
                uint8_t sum = len + AHi + ALo + (uint8_t)TT;

                // veri parse
                if (len > sizeof(data)){ in_line = false; return false; }
                uint16_t p = 8;
                for (uint8_t k = 0; k < len; k++){
                    int b = get_hex_byte(line[p], line[p+1]);
                    if (b < 0){ in_line = false; return false; }
                    data[k] = (uint8_t)b;
                    sum += (uint8_t)b;
                    p += 2;
                }
                // CC
                int CC = get_hex_byte(line[p], line[p+1]);
                if (CC < 0){ in_line = false; return false; }
                sum += (uint8_t)CC;
                if ((sum & 0xFF) != 0){ in_line = false; return false; }

                // record tipi
                if (TT == 0x00){ // data
                    uint32_t abs = ext_base + addr16;
                    for (uint8_t k = 0; k < len; k++){
                        uint32_t a = abs + k;
                        if (a >= APP_FLASH_END) continue; // boot’u koru
                        uint32_t pg = (a / SPM_PAGESIZE) * SPM_PAGESIZE;
                        if (pg != cur_page_base) page_switch(pg);
                        uint16_t off_in = (uint16_t)(a - cur_page_base);
                        pagebuf[off_in] = data[k];
                        page_dirty = true;
                    }
                } else if (TT == 0x01){ // EOF
                    page_commit_if_dirty();
                    seen_eof = true;
                    return true;
                } else if (TT == 0x04){ // ELA (upper 16 bits)
                    if (len != 2){ in_line = false; return false; }
                    uint16_t upper = ((uint16_t)data[0] << 8) | data[1];
                    ext_base = ((uint32_t)upper) << 16;
                } else {
                    // 02, 03, 05 vs. → yoksay
                }

                in_line = false;
                continue;
            }

            // satır gövdesi
            if (li < sizeof(line)) line[li++] = c;
            else { in_line = false; return false; } // aşım
        }
    }

    // Dosya bitti ama EOF görmedik → başarısız
    page_commit_if_dirty();
    return seen_eof;
}

/* --- Dış arayüz --- */
bool exup_check_and_update(void){
    uint32_t size = 0, hv = 0;
    exup_init_spi();
    if (!read_footer(&size, &hv)) return false;

    if (fnv1a_over_spi(size) != hv) return false;

    // Güvenli yazım: kesmeleri kapat
    uint8_t sreg = SREG;
    cli();
    bool ok = apply_hex_from_spi(size);
    SREG = sreg;

    if (ok){
        // İstersen burada footer’ı silmek için SPI-erase ekleyebilirsin.
        // Başarılı güncellemeden sonra hızlı reset:
        wdt_enable(WDTO_15MS);
        for(;;){}  // reseti bekle
    }
    return ok;
}
