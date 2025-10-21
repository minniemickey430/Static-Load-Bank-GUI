from __future__ import annotations

# ------------------ Protokol ------------------
# Çerçeve: [STX][ADDR][CMD][LEN][DATA...][CRC][ETX]
STX = 0xAA  # Start of frame (başlangıç baytı)
ETX = 0x55  # End of frame (bitiş baytı)
BAUD = 115200  # UART baud rate

# Komut (CMD) alanı; cihaz tarafında bu koda göre alt-komutlar yorumlanır
CMD_RLY  = 0x10  # Röle kontrol/geri-bildirim komutları
CMD_FAN  = 0x20  # Fan modu/hızı ve eşik ile ilgili komutlar/ACK
CMD_TMP  = 0x30  # Sıcaklık (NTC) okuma cevabı/isteği
CMD_ADDR = 0x40  # Cihaz adresi sorgu/ayar
CMD_EVT  = 0x50  # Sistem olayları (reset, hata kodları vb.)
CMD_OLED = 0x60  # OLED ekran uyku/uyandırma komutları

# Röle alt-komutları (SUB) — DATA[0] alanı
SUB_RLY_ON  = 0x01  # Belirtilen röleyi aç
SUB_RLY_OFF = 0x02  # Belirtilen röleyi kapa
SUB_RLY_QRY = 0x03  # Röle durum sorgusu (opsiyonel kullanım)
SUB_RLY_ERR = 0xEE  # Cihazdan hata bildirimi (ör. Holt denetimi başarısız)

# Sıcaklık alt-komutları — GUI kendi min/max hesaplar; cihaz tarafı indeksli veri dönebilir
SUB_TMP_ALL = 0x00  # Tüm NTC kanallarını iste/raporla
SUB_TMP_MAX = 0x01  # (kullanılmıyor)
SUB_TMP_MIN = 0x02  # (kullanılmıyor)
SUB_TMP_ONE = 0x03  # Tek bir NTC kanalı (index verilirse)

# Fan kontrolü
FAN_LABELS = ["%0", "%25", "%50", "%75", "%100", "AUTO"]  # GUI tarafındaki gösterim etiketleri
FAN_SUBMAP = {"%0":0, "%25":1, "%50":2, "%75":3, "%100":4, "AUTO":5}  # Etiket→alt-komut kodu
FAN_STATUS_MANUAL_OK = 0xF1  # Cihazdan gelen durum: MANUAL moda geçiş onayı
FAN_STATUS_AUTO_OK   = 0xF2  # Cihazdan gelen durum: AUTO moda geçiş onayı

# OLED ekran kontrolü
SUB_OLED_SLEEP = 0x00  # Ekranı uykuya al
SUB_OLED_WAKE  = 0x01  # Ekranı uyandır
OLED_DEBOUNCE_MS = 300  # OLED komutları için yazılım debouncing (ms)

# Sistem olayları (CMD=0x50 altında a1 alanı ile raporlanır)
# Bu harita yalnızca GUI tarafında okunabilir metin üretmek içindir.
SYS_STATUS_MAP = {
    0x01: "Invalid length",                 # LEN alanı/çerçeve uzunluğu geçersiz
    0x02: "Unknown subcmd",                 # Bilinmeyen alt-komut
    0x03: "Missing subcmd",                 # Alt-komut eksik
    0x04: "Buffer overflow",                # Dahili buffer taşması
    0x05: "Timeout",                        # Zaman aşımı
    0x06: "Sensor not found",               # Sensör/kanal bulunamadı
    0x07: "Relay fault (Holt check failed)",# Röle/Holt bütünlük kontrol hatası
}
