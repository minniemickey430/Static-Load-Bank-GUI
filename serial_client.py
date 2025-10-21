from __future__ import annotations
import time, threading
from typing import Optional

# PySerial modülü; bazı ortamlarda yoksa hata vermesin diye try/except içinde import edilir
try:
    import serial
except Exception as e:
    serial = None  # import hatası durumunda kodun tamamı çökmesin

from .protocol import crc8_atm, hex_dump
from .constants import BAUD, STX, ETX

# ------------------ Serial Client ------------------
class SerialClient:
    """
    Tek bir seri portu yöneten istemci sınıfı.
    - UART bağlantısını açar/kapatır.
    - Gelen veriyi (binary + metin) parse eder.
    - Gönderim/alımlarda log callback'lerini tetikler.
    """

    def __init__(self, port:str, baud:int, device_addr:int, rx_cb, rx_line_cb, log_cb, hex_cb):
        # Kullanıcıdan alınan parametreler
        self.port = port                # COM port adı (ör: 'COM5')
        self.baud = baud                # Baud hızı
        self.device_addr = device_addr  # Hedef cihaz adresi (frame içinde kullanılır)
        # Callback fonksiyonları
        self.rx_cb = rx_cb              # Binary frame geldiğinde çağrılır
        self.rx_line_cb = rx_line_cb    # Text satırı geldiğinde çağrılır
        self.log_cb = log_cb            # Genel log callback
        self.hex_cb = hex_cb            # HEX dump callback

        # Dahili üyeler
        self.ser: Optional[serial.Serial] = None   # Seri port nesnesi
        self._stop = threading.Event()             # Thread durdurma bayrağı
        self._th = None                            # Reader thread
        self._buf = bytearray()                    # Binary buffer (frame parse)
        self._text = bytearray()                   # Text buffer ('\n' bazlı)

    # --- yardımcı log metodları ---
    def log(self, s): 
        if self.log_cb:
            self.log_cb(s)

    def hexlog(self, s):
        if self.hex_cb:
            self.hex_cb(s)

    # --- Port açma işlemi ---
    def open(self):
        """
        Port zaten açıksa önce kapatır, sonra yeniden açar.
        Ardından RX thread'i başlatır.
        """
        # Önceden çalışan thread varsa düzgün kapat
        if self._th and self._th.is_alive():
            self._stop.set()
            try:
                if self.ser:
                    # Buffer'ları temizle
                    try:
                        self.ser.reset_input_buffer()
                        self.ser.reset_output_buffer()
                    except:
                        pass
                    # IO işlemlerini iptal et (varsa)
                    try:
                        if hasattr(self.ser, "cancel_read"):
                            self.ser.cancel_read()
                        if hasattr(self.ser, "cancel_write"):
                            self.ser.cancel_write()
                    except:
                        pass
                    # Port açıksa kapat
                    if self.ser.is_open:
                        self.ser.close()
            except:
                pass
            self._th.join(timeout=0.5)

        if not self.port:
            raise RuntimeError("Select a port first")

        # Portu aç
        self.ser = serial.Serial(
            self.port, self.baud, timeout=0.05, write_timeout=0.2
        )

        # Thread kontrol bayraklarını sıfırla ve başlat
        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

        self.log(f"[OPEN] {self.port}@{self.baud} (Addr={self.device_addr})")

    # --- Port kapatma işlemi ---
    def close(self):
        """
        RX thread'ini ve portu güvenli şekilde kapatır.
        """
        self._stop.set()
        try:
            if self.ser:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except:
                    pass
                try:
                    if hasattr(self.ser, "cancel_read"):
                        self.ser.cancel_read()
                    if hasattr(self.ser, "cancel_write"):
                        self.ser.cancel_write()
                except:
                    pass
                if self.ser.is_open:
                    self.ser.close()
        finally:
            # Thread temizliği
            th = self._th
            self._th = None
            if th and th.is_alive():
                th.join(timeout=0.8)
            self.ser = None

        self.log("[CLOSE]")

    # --- Port durumu sorgulama ---
    def is_open(self) -> bool:
        """Port açık mı?"""
        return bool(self.ser and self.ser.is_open)

    # --- Veri gönderimi ---
    def send(self, blob: bytes):
        """
        Verilen byte dizisini seri porttan gönderir.
        Port açık değilse log’a “NOT SENT” olarak yazar.
        """
        if not self.is_open():
            self.hexlog(f"TX[{self.port or 'DISCONNECTED'}] {hex_dump(blob)}  ;; NOT SENT")
            return
        try:
            self.ser.write(blob)
            self.hexlog(f"TX[{self.port}] {hex_dump(blob)}")
        except Exception as e:
            self.log(f"[TX ERR] {e}")

    # ============================================================
    #                ALIM (RECEIVE) TARAFLARI
    # ============================================================

    def _emit_text(self):
        """
        _text buffer’ında biriken ASCII satırları '\n' bazlı ayırır
        ve rx_line_cb callback’ine iletir.
        """
        while True:
            nl = self._text.find(b'\n')
            if nl < 0:
                break
            line = self._text[:nl].rstrip(b'\r').decode('utf-8', 'ignore')
            del self._text[:nl + 1]
            try:
                self.rx_line_cb(line)
            except Exception as e:
                self.log(f"[TEXT CB ERR] {e}")

    def _consume_one(self) -> bool:
        """
        _buf içinde bir frame tamamlanmışsa işler.
        CRC kontrolü yapar, doğruysa rx_cb callback’ini tetikler.
        """
        buf = self._buf
        s = buf.find(bytes([STX]))  # Başlangıç baytını bul
        if s < 0:
            # STX yoksa tüm buffer text kabul edilir
            if buf:
                self._text.extend(buf)
                buf.clear()
                self._emit_text()
            return False

        # STX öncesi varsa text’e aktar
        if s > 0:
            self._text.extend(buf[:s])
            del buf[:s]
            self._emit_text()

        # Minimum frame uzunluğu kontrolü
        if len(buf) < 6:
            return False

        ln = buf[3]         # Uzunluk alanı (LEN)
        need = 6 + ln       # STX..ETX dahil toplam frame uzunluğu
        if len(buf) < need:
            return False     # henüz tamamlanmamış frame

        frame = bytes(buf[:need])

        # Son byte ETX değilse kaydırarak hatayı temizle
        if frame[-1] != ETX:
            del buf[0]
            return True

        # CRC kontrolü
        body = frame[1:-2]
        crc = frame[-2]
        exp = crc8_atm(body)
        if crc != exp:
            self.hexlog(f"RX[{self.port}] {hex_dump(frame)}")
            self.log(f"[CRC ERR] got=0x{crc:02X} exp=0x{exp:02X}; drop")
            del buf[0]
            return True

        # Geçerli frame — hex dump ve callback
        self.hexlog(f"RX[{self.port}] {hex_dump(frame)}")
        addr = frame[1]
        cmd = frame[2]
        data = frame[4:4 + ln]
        sub = data[0] if ln >= 1 else 0
        a1 = data[1] if ln >= 2 else 0
        a2 = data[2] if ln >= 3 else 0

        try:
            self.rx_cb(addr, cmd, sub, a1, a2, data)
        except Exception as e:
            self.log(f"[RX BIN CB ERR] {e}")

        # İşlenen frame’i buffer’dan çıkar
        del buf[:need]
        return True

    # --- Okuma thread’i ---
    def _loop(self):
        """
        RX thread fonksiyonu:
        Porttan sürekli veri okur, parse eder ve callback'leri tetikler.
        """
        while not self._stop.is_set():
            try:
                if self.ser and self.ser.is_open:
                    # 512 byte’a kadar oku
                    chunk = self.ser.read(512)
                    if chunk:
                        self._buf.extend(chunk)
                        # Frame tamamlanmış mı kontrol et
                        progressed = True
                        while progressed:
                            progressed = self._consume_one()
                        # Artan text satırlarını işle
                        self._emit_text()
                # CPU yükünü azaltmak için kısa bekleme
                time.sleep(0.01)
            except Exception as e:
                self.log(f"[RX ERR] {e}")
                time.sleep(0.05)
