# pip install pyserial
import argparse
import sys
import struct
import serial
import time

FNV_OFFSET = 0x811C9DC5
FNV_PRIME  = 0x01000193
FLASH_SIZE = 2 * 1024 * 1024
FOOTER_RESERVATION = 4096

def fnv1a(data: bytes) -> int:
    h = FNV_OFFSET
    for b in data:
        h ^= b
        h = (h * FNV_PRIME) & 0xFFFFFFFF
    return h

def drain_device_lines(ser, duration_s=2.0):
    """Belirli süre cihazdan gelen satırları 'DEVICE>' diyerek yazdırır."""
    t0 = time.time()
    while time.time() - t0 < duration_s:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            break
        print("DEVICE>", line)

def print_any_pending_lines(ser):
    """Bekleyen satırları tüketmeden, varsa hızlıca gösterir."""
    # timeout düşükse readline zaten kısa bekler; 2-3 satırdan fazlasını bekleme
    for _ in range(3):
        if ser.in_waiting <= 0:
            return
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("DEVICE>", line)

def ensure_intel_hex(data: bytes, path: str) -> bytes:
    """Intel HEX beklenirken yanlışlıkla .bin göndermeyi engelle."""
    stripped = data.lstrip()
    if not stripped.startswith(b":"):
        raise ValueError(
            f"{path} dosyası Intel HEX gibi görünmüyor; ilk anlamlı karakter ':' değil. "
            "Bootloader yalnızca HEX akışını kabul eder."
        )
    return data

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="EXUP loader'a Intel HEX dosyası gönderir"
    )
    parser.add_argument(
        "image",
        help="SPI flash'a yazılacak Intel HEX dosyasının yolu"
    )
    parser.add_argument(
        "--port",
        default="COM12",
        help="Seri port (varsayılan: COM12)"
    )
    return parser.parse_args()

def main():
    args = parse_args()
    port = args.port
    path = args.image

    # Dosyayı oku
    with open(path, "rb") as f:
        raw = f.read()

    if len(raw) == 0:
        print("Empty file"); sys.exit(1)

    data = ensure_intel_hex(raw, path)
    size = len(data)
    max_payload = FLASH_SIZE - FOOTER_RESERVATION
    if size > max_payload:
        raise ValueError(
            f"Payload of {size} bytes exceeds usable flash capacity of {max_payload} bytes; the last 4KB are reserved for the footer sector."
        )

    h = fnv1a(data)

    # Seri portu aç
    ser = serial.Serial(port, 115200, timeout=0.5)
    try:
        time.sleep(1.5)         # port stabilize
        ser.reset_input_buffer()

        # Açılış loglarını yakala (JEDEC / READY)
        drain_device_lines(ser, duration_s=2.0)

        # Header: "EXUP" + size(LE) + fnv1a(LE)
        header = b"EXUP" + struct.pack("<I", size) + struct.pack("<I", h)
        ser.write(header)

        # Handshake: "OK" bekle (diğer satırları da göster)
        t0 = time.time()
        got_ok = False
        while time.time() - t0 < 5.0:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print("DEVICE>", line)
            if line == "OK":
                got_ok = True
                break
            # "READY" vb. başka satırlar da gelebilir; hepsini gösteriyoruz
        if not got_ok:
            print("Handshake failed: no OK")
            sys.exit(2)

        # 256B bloklarla gönderim + ACK/NAK kontrolü
        CHUNK = 256
        sent = 0
        for i in range(0, size, CHUNK):
            chunk = data[i:i+CHUNK]
            ser.write(chunk)
            ack = ser.read(1)

            if ack == b'\x06':
                # ACK — bazen cihaz log atabilir, gösterebilelim
                print_any_pending_lines(ser)
            elif ack == b'\x15':
                # NAK — cihaz teşhis mesajları yollar; birkaç satırını göster
                print("DEVICE> NAK")
                # Genellikle 1-2 satır oluyor (ör: TIMEOUT_PP / MISMATCH ...)
                for _ in range(4):
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        break
                    print("DEVICE>", line)
                sys.exit(3)
            else:
                print(f"No ACK at {i}, resp={repr(ack)}")
                # Belki cihaz metin döktü; göster
                print_any_pending_lines(ser)
                sys.exit(3)

            sent += len(chunk)
            if (i // CHUNK) % 64 == 0:
                print(f"Progress: {sent}/{size}")

        # Final sonuç satırını bekle ve göster
        t0 = time.time()
        final = None
        while time.time() - t0 < 5.0:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print("DEVICE>", line)
            final = line
            if line in ("DONE", "ERR"):
                break

        if final == "DONE":
            sys.exit(0)
        else:
            sys.exit(4)

    finally:
        # Portu garanti kapat (Windows’ta kilit kalmasın)
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
