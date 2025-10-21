from __future__ import annotations
from typing import Optional, Dict
from .serial_client import SerialClient
from .protocol import pack, hex_dump

# ------------------ Shared Serial Hub (Same-COM modu) ------------------
class SerialHub:
    """
    Tek bir seri port üzerinden birden fazla 'Drawer' panelinin
    haberleşmesini sağlayan merkez sınıf.

    - Tüm GUI panelleri (DevicePanel) aynı COM portu paylaşabilir.
    - Gelen frame’leri doğru panele yönlendirir.
    - TX/RX loglarını panellere iletir.
    """

    def __init__(self, app: 'App'):
        # Ana GUI (App) referansı
        self.app = app
        # Ortak kullanılan SerialClient örneği
        self.client: Optional[SerialClient] = None
        # Seçili COM port ismi
        self.shared_port: str = ""
        # Cihaz adresi → ilgili DevicePanel eşlemesi
        self.addr_to_panel: Dict[int, DevicePanel] = {}

    # --- Ortak portu aç ---
    def open(self, port: str):
        """
        Paylaşılan COM portu açar ve SerialClient başlatır.
        Önceden açık bağlantı varsa önce kapatır.
        """
        self.shared_port = port
        self.close()
        # Tek port, tüm drawer'lar için kullanılacak
        self.client = SerialClient(
            port, 115200, 0,             # addr=0 → broadcast/ortak kanal
            self._rx_bin, self._rx_line, # callback fonksiyonları
            self._log, self._hex
        )
        self.client.open()
        self._log("[HUB] Shared serial opened.")

    # --- Ortak portu kapat ---
    def close(self):
        """Açık olan shared COM bağlantısını güvenli şekilde kapatır."""
        if self.client:
            try:
                self.client.close()
            except:
                pass
            self.client = None
            self._log("[HUB] Shared serial closed.")

    # --- Port açık mı? ---
    def is_open(self) -> bool:
        """Shared port aktif mi kontrol eder."""
        return bool(self.client and self.client.is_open())

    # --- Adres → Panel eşlemesini güncelle ---
    def refresh_addr_map(self):
        """
        Her panelde girili olan cihaz adreslerini okur
        ve addr_to_panel sözlüğünü günceller.
        """
        mapping: Dict[int, DevicePanel] = {}
        for p in self.app.panels:
            try:
                a = int(p.addr_var.get()) & 0xFF
                mapping[a] = p
            except:
                pass
        self.addr_to_panel = mapping

    # --- Bir panelden gelen frame'i port üzerinden gönder ---
    def send_from_panel(self, panel: 'DevicePanel', frame: bytes, note: str):
        """
        Bir panel (örneğin Drawer 1) bir frame göndermek isterse,
        bu fonksiyon ortak client üzerinden gönderimi yapar.
        """
        port = self.shared_port or (self.client.port if self.client else 'DISCONNECTED')
        # HEX log GUI tarafında gösterilir
        panel.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; {note}")
        if self.client:
            self.client.send(frame)

    # --- Genel log (metin tabanlı) ---
    def _log(self, s: str):
        """
        Hub log mesajlarını ilk panele (Drawer 1) yazar.
        Bu panel genel sistem log penceresi olarak kullanılır.
        """
        if self.app.panels:
            self.app.panels[0].log(s)

    # --- HEX log ---
    def _hex(self, s: str):
        """
        Hub HEX dump çıktısını ilk panelde gösterir.
        (TX/RX veri trafiği için)
        """
        if self.app.panels:
            self.app.panels[0].hex_log(s)

    # --- Binary frame alımı ---
    def _rx_bin(self, addr: int, cmd: int, sub: int, a1: int, a2: int, data: bytes):
        """
        SerialClient tarafından çağrılır.
        Gelen frame'in adresine göre doğru DevicePanel'e yönlendirir.
        """
        self.refresh_addr_map()
        target = self.addr_to_panel.get(addr)
        port = self.shared_port or (self.client.port if self.client else 'DISCONNECTED')

        if target:
            # Frame doğru panele aitse ilgili panelin handler'ını çağır
            target.hex_log(f"RX[{port}] {hex_dump(pack(addr, cmd, data))}")
            try:
                target.on_frame_bin(addr, cmd, sub, a1, a2, data)
            except Exception as e:
                target.log(f"[HUB RX ERR] {e}")
        else:
            # Adres eşleşmiyorsa genel log'a bilgi yazar
            if self.app.panels:
                self.app.panels[0].log(
                    f"[HUB] Unmapped ADDR=0x{addr:02X} cmd=0x{cmd:02X}"
                )

    # --- Text satırı alımı ---
    def _rx_line(self, line: str):
        """
        Text (ASCII) satırları her panele dağıtır.
        Bu sayede NTC[x] = ... gibi string mesajlar tüm panellerde işlenebilir.
        """
        for p in self.app.panels:
            p.on_line_text(line)
