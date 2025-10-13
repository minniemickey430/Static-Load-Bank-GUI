

from __future__ import annotations
import argparse
from typing import Optional, List, Tuple

# ---- Protokol sabitleri (relay_manager.h ile uyumlu) ----
STX = 0xAA
ETX = 0x55
BAUD = 115200
CMD_RELAY_CONTROL   = 0x10
CMD_FAN_CONTROL     = 0x20
CMD_TEMPERATURE     = 0x30
CMD_DEVICE_ADDRESS  = 0x40
CMD_SYSTEM_STATUS   = 0x50
CMD_OLED_CONTROL    = 0x60

SUBCMD_RELAY_ON     = 0x01
SUBCMD_RELAY_OFF    = 0x02
SUBCMD_RELAY_READ   = 0x03

SUBCMD_FAN_MANUAL   = 0x01  # arg: 0..4  -> %0,%25,%50,%75,%100
SUBCMD_FAN_AUTO     = 0x05

SUBCMD_TEMP_ALL     = 0x00
SUBCMD_TEMP_ONE     = 0x03  # arg: index

SUBCMD_OLED_OFF     = 0x00
SUBCMD_OLED_ON      = 0x01

STATUS_OK              = 0x00
STATUS_FAN_MANUAL_OK   = 0xF1
STATUS_FAN_AUTO_OK     = 0xF2

ADDR_OLED_DEVICE       = 0x60  # OLED cihaz adresi

# ---- CRC-8 ATM ----
def crc8_atm(data_bytes: bytes) -> int:
    crc = 0x00
    for b in data_bytes:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_frame(addr: int, cmd: int, sub: Optional[int]=None, args: bytes=b"") -> bytes:
    if sub is None:
        body = bytes([addr, cmd, 0x00])
    else:
        payload = bytes([sub]) + (args or b"")
        body = bytes([addr, cmd, len(payload)]) + payload
    c = crc8_atm(body)
    return bytes([STX]) + body + bytes([c, ETX])

class FrameParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, chunk: bytes) -> List[Tuple[int,int,int,Optional[int],bytes,bytes]]:
        out = []
        if not chunk: return out
        self.buf.extend(chunk)
        while True:
            i = self.buf.find(bytes([STX]))
            if i < 0:
                if len(self.buf) > 8192: self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            addr = self.buf[1] if len(self.buf) > 1 else None
            cmd  = self.buf[2] if len(self.buf) > 2 else None
            ln   = self.buf[3] if len(self.buf) > 3 else None
            if addr is None or cmd is None or ln is None: break
            total = 1 + 3 + ln + 1 + 1
            if len(self.buf) < total: break
            frame = bytes(self.buf[:total]); del self.buf[:total]
            if frame[-1] != ETX:
                continue
            body = frame[1:-2]
            crc  = frame[-2]
            if crc8_atm(body) != crc:
                continue
            data = body[3:]
            sub  = data[0] if len(data) >= 1 else None
            args = data[1:] if len(data) >= 2 else b""
            out.append((body[0], body[1], body[2], sub, args, frame))
        return out

# ---- GUI (yalnız gerçek cihaza bağlanır) ----
def run_gui(port: str, baud: int):
    import serial
    import tkinter as tk
    from tkinter import ttk, messagebox

    ser: Optional[serial.Serial] = None
    parser = FrameParser()

    temps = [0]*20  # NTC0..NTC19
    ntc_count = 20

    # --- yardımcılar ---
    def log(msg: str):
        txt.configure(state="normal"); txt.insert("end", msg + "\n"); txt.configure(state="disabled"); txt.see("end")

    def scope_push(line: str):
        scope.configure(state="normal"); scope.insert("end", line + "\n"); scope.configure(state="disabled"); scope.see("end")

    def safe_close():
        nonlocal ser
        try:
            if ser and ser.is_open:
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass
                ser.close()
        except Exception:
            pass

    def open_port():
        nonlocal ser
        try:
            safe_close()
            ser = serial.Serial(port, baud, timeout=0.03)
            # bağlanma sonrası küçük bir flush
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            log(f"[OPEN] {port}@{baud}")
        except Exception as e:
            messagebox.showerror("Open Error", str(e))

    def close_port():
        safe_close()
        log("[CLOSE]")

    def send(addr: int, cmd: int, sub: Optional[int], args: bytes=b""):
        if not ser or not ser.is_open:
            messagebox.showwarning("Port", "Seri port açık değil.")
            return
        frm = build_frame(addr, cmd, sub, args)
        try:
            ser.write(frm)
            scope_push(frm.hex(" ").upper())
        except Exception as e:
            log(f"[TX ERR] {e}")

    # RX poll — tek timer, UI flicker olmaması için sadece metin güncelle
    def poll_rx():
        if ser and ser.is_open:
            try:
                data = ser.read(8192)
                if data:
                    for addr, cmd, ln, sub, args, raw in parser.feed(data):
                        scope_push(raw.hex(" ").upper())
                        handle_frame(addr, cmd, sub, args)
            except Exception as e:
                log(f"[RX ERR] {e}")
        root.after(30, poll_rx)

    # --- RX handler ---
    def handle_frame(addr: int, cmd: int, sub: Optional[int], args: bytes):
        # DEVICE ADDRESS bildirimi
        if cmd == CMD_DEVICE_ADDRESS and sub == 0x00 and len(args) >= 1:
            log(f"[ADDR] Device Address = 0x{args[0]:02X}")
            return
        # SYSTEM STATUS (ör: reset)
        if cmd == CMD_SYSTEM_STATUS and sub == 0x00 and len(args) >= 1:
            log("[SYS] Röleler init’te sıfırlandı")
            return

        # Röle read cevabı: AA ADDR 10 02 03 YY
        if cmd == CMD_RELAY_CONTROL and sub == SUBCMD_RELAY_READ and len(args) >= 1:
            val = 1 if args[0] == 1 else 0
            rid = last_relay_read_id.get()
            if 1 <= rid <= 20:
                relay_states[rid-1].set("AÇIK" if val else "KAPALI")
            log(f"[RELAY] Sorgu -> {('AÇIK' if val==1 else 'KAPALI')} (rid={rid})")
            return

        # Röle on/off echo: AA ADDR 10 02 <sub> <rid>
        if cmd == CMD_RELAY_CONTROL and sub in (SUBCMD_RELAY_ON, SUBCMD_RELAY_OFF) and len(args) >= 1:
            rid = int(args[0])
            if 1 <= rid <= 20:
                relay_states[rid-1].set("AÇIK" if sub==SUBCMD_RELAY_ON else "KAPALI")
            log(f"[RELAY] {'ON' if sub==SUBCMD_RELAY_ON else 'OFF'} OK (rid={rid})")
            return

        # Fan: AA ADDR 20 02 SUB STATUS
        if cmd == CMD_FAN_CONTROL and sub in (SUBCMD_FAN_MANUAL, SUBCMD_FAN_AUTO) and len(args) >= 1:
            st = args[0]
            if st == STATUS_FAN_MANUAL_OK: log("[FAN] MANUAL OK")
            elif st == STATUS_FAN_AUTO_OK: log("[FAN] AUTO OK")
            else: log(f"[FAN] Status=0x{st:02X}")
            return

        # Sıcaklık tek paket: AA ADDR 30 02 II TT
        if cmd == CMD_TEMPERATURE and sub == 0x02 and len(args) >= 2:
            ii = args[0] & 0xFF
            tt = args[1] & 0xFF
            if ii == 6:  # NTC6 ofset +2°C (sadece ekranda)
                tt = min(255, (tt + 2) & 0xFF)
            if 0 <= ii < len(temps):
                temps[ii] = tt
                # UI metnini güncelle (yeniden çizim yapma -> flicker yok)
                if ii < 10:
                    ntc_vars_row1[ii].set(f"NTC{ii}: {tt} °C")
                else:
                    ntc_vars_row2[ii-10].set(f"NTC{ii}: {tt} °C")
                # min/max güncelle
                cur = temps[:ntc_count]
                mx, mn = max(cur), min(cur)
                idx_mx, idx_mn = cur.index(mx), cur.index(mn)
                max_var.set(f"Max: NTC{idx_mx} = {mx} °C")
                min_var.set(f"Min: NTC{idx_mn} = {mn} °C")
            return

        # OLED: AA 60 60 02 <sub> <status>
        if cmd == CMD_OLED_CONTROL and sub in (SUBCMD_OLED_ON, SUBCMD_OLED_OFF) and len(args) >= 1:
            st = args[0]
            if st == STATUS_OK:
                log(f"[OLED] {'ON' if sub==SUBCMD_OLED_ON else 'OFF'} OK")
            else:
                log(f"[OLED] Status=0x{st:02X}")
            return

        # genel
        log(f"[RX] addr=0x{addr:02X} cmd=0x{cmd:02X} sub={None if sub is None else f'0x{sub:02X}'} len(args)={len(args)}")

    # ---- UI ----
    root = tk.Tk()
    root.title("UART GUI — REAL DEVICE")

    # Port
    top = ttk.Frame(root); top.pack(fill="x", padx=8, pady=6)
    ttk.Button(top, text="Open", command=open_port).pack(side="left", padx=4)
    ttk.Button(top, text="Close", command=close_port).pack(side="left", padx=4)

    # Röle alanı: 4x5 (1..20)
    rf = ttk.LabelFrame(root, text="Röleler (1–20)"); rf.pack(fill="x", padx=8, pady=6)
    ttk.Label(rf, text="ADDR(hex):").grid(row=0, column=0, sticky="w", padx=4, pady=2)
    addr_relay_sv = tk.StringVar(value="01")
    ttk.Entry(rf, width=5, textvariable=addr_relay_sv).grid(row=0, column=1, sticky="w", padx=4, pady=2)

    # 20 röle: her hücrede "Rxx", durum etiketi, ON, OFF
    relay_states = [tk.StringVar(value="-") for _ in range(20)]
    last_relay_read_id = tk.IntVar(value=1)

    def mk_relay_cell(parent, idx: int, r: int, c_start: int):
        # idx: 0..19 -> relay_id = idx+1
        rid = idx + 1
        frm = ttk.Frame(parent); frm.grid(row=r, column=c_start, columnspan=4, padx=4, pady=4, sticky="w")
        ttk.Label(frm, text=f"R{rid:02d}").grid(row=0, column=0, sticky="w")
        lab = ttk.Label(frm, textvariable=relay_states[idx], width=6)
        lab.grid(row=0, column=1, padx=4)

        def do_on():
            a = int(addr_relay_sv.get(), 16)
            send(a, CMD_RELAY_CONTROL, SUBCMD_RELAY_ON, bytes([rid]))
        def do_off():
            a = int(addr_relay_sv.get(), 16)
            send(a, CMD_RELAY_CONTROL, SUBCMD_RELAY_OFF, bytes([rid]))
        def do_read():
            a = int(addr_relay_sv.get(), 16)
            last_relay_read_id.set(rid)
            send(a, CMD_RELAY_CONTROL, SUBCMD_RELAY_READ, bytes([rid]))

        ttk.Button(frm, text="ON",  width=4, command=do_on).grid(row=0, column=2, padx=2)
        ttk.Button(frm, text="OFF", width=4, command=do_off).grid(row=0, column=3, padx=2)
        ttk.Button(frm, text="?",   width=2, command=do_read).grid(row=0, column=4, padx=2)

    # 4 satır × 5 kolon
    idx = 0
    for rr in range(1, 5):            # satırlar 1..4; 0. satırda addr alanı var
        for cc in range(0, 5):        # kolonlar 0..4
            mk_relay_cell(rf, idx, rr, cc*5)
            idx += 1

    # Fan
    ff = ttk.LabelFrame(root, text="Fan"); ff.pack(fill="x", padx=8, pady=6)
    ttk.Label(ff, text="ADDR(hex):").pack(side="left"); addr_fan_sv = tk.StringVar(value="00")
    ttk.Entry(ff, width=5, textvariable=addr_fan_sv).pack(side="left", padx=4)

    # Seviyeler 1..5 görünsün; MCU'ya (seviye-1) gitsin
    def fan_set(level_1_to_5: int):
        lvl = max(1, min(5, level_1_to_5))
        a = int(addr_fan_sv.get(), 16)
        send(a, CMD_FAN_CONTROL, SUBCMD_FAN_MANUAL, bytes([(lvl-1) & 0xFF]))
    ttk.Button(ff, text="1", command=lambda: fan_set(1)).pack(side="left", padx=2)
    ttk.Button(ff, text="2", command=lambda: fan_set(2)).pack(side="left", padx=2)
    ttk.Button(ff, text="3", command=lambda: fan_set(3)).pack(side="left", padx=2)
    ttk.Button(ff, text="4", command=lambda: fan_set(4)).pack(side="left", padx=2)
    ttk.Button(ff, text="5", command=lambda: fan_set(5)).pack(side="left", padx=2)
    def fan_auto():
        a = int(addr_fan_sv.get(), 16)
        send(a, CMD_FAN_CONTROL, SUBCMD_FAN_AUTO, b"")
    ttk.Button(ff, text="AUTO", command=fan_auto).pack(side="left", padx=8)

    # Sıcaklık
    tf = ttk.LabelFrame(root, text="Sıcaklıklar (NTC0..NTC19)"); tf.pack(fill="x", padx=8, pady=6)
    ttk.Label(tf, text="ADDR(hex):").pack(side="left"); addr_temp_sv = tk.StringVar(value="01")
    ttk.Entry(tf, width=5, textvariable=addr_temp_sv).pack(side="left", padx=4)

    def temp_all():
        a = int(addr_temp_sv.get(), 16)
        # Sadece TEK İSTEK gönder; UI'da yalnızca metin güncellemeleri yapılır (flicker yok)
        send(a, CMD_TEMPERATURE, SUBCMD_TEMP_ALL, b"")
    ttk.Button(tf, text="Read Temps", command=temp_all).pack(side="left", padx=8)

    ttk.Label(tf, text="Tek NTC:").pack(side="left"); ntc_idx_sv = tk.StringVar(value="0")
    ttk.Entry(tf, width=5, textvariable=ntc_idx_sv).pack(side="left", padx=4)
    def temp_one():
        a = int(addr_temp_sv.get(), 16); ii = int(ntc_idx_sv.get()) & 0xFF
        send(a, CMD_TEMPERATURE, SUBCMD_TEMP_ONE, bytes([ii]))
    ttk.Button(tf, text="Oku", command=temp_one).pack(side="left", padx=4)

    # OLED
    of = ttk.LabelFrame(root, text="OLED (ADDR=0x60)"); of.pack(fill="x", padx=8, pady=6)
    ttk.Label(of, text="ADDR(hex):").pack(side="left")
    addr_oled_sv = tk.StringVar(value="60")
    ttk.Entry(of, width=5, textvariable=addr_oled_sv).pack(side="left", padx=4)
    def oled_on():
        a = int(addr_oled_sv.get(), 16)  # 0x60
        send(a, CMD_OLED_CONTROL, SUBCMD_OLED_ON, b"")
    def oled_off():
        a = int(addr_oled_sv.get(), 16)
        send(a, CMD_OLED_CONTROL, SUBCMD_OLED_OFF, b"")
    ttk.Button(of, text="OLED ON", command=oled_on).pack(side="left", padx=4)
    ttk.Button(of, text="OLED OFF", command=oled_off).pack(side="left", padx=4)

    # Sıcaklık grid (0..19) — sadece metin güncellenir (widget yeniden yaratılmaz)
    grid1 = ttk.Frame(root); grid1.pack(fill="x", padx=8, pady=(4,0))
    grid2 = ttk.Frame(root); grid2.pack(fill="x", padx=8, pady=(0,6))
    global ntc_vars_row1, ntc_vars_row2
    ntc_vars_row1 = [tk.StringVar(value=f"NTC{i}: - °C") for i in range(10)]
    ntc_vars_row2 = [tk.StringVar(value=f"NTC{i+10}: - °C") for i in range(10)]
    for i in range(10):
        ttk.Label(grid1, textvariable=ntc_vars_row1[i], width=14).pack(side="left", padx=2)
    for i in range(10):
        ttk.Label(grid2, textvariable=ntc_vars_row2[i], width=14).pack(side="left", padx=2)

    # Min/Max
    mm = ttk.Frame(root); mm.pack(fill="x", padx=8, pady=4)
    global max_var, min_var
    max_var = tk.StringVar(value="Max: -")
    min_var = tk.StringVar(value="Min: -")
    ttk.Label(mm, textvariable=max_var).pack(side="left", padx=8)
    ttk.Label(mm, textvariable=min_var).pack(side="left", padx=8)

    # Log & Scope
    lf = ttk.LabelFrame(root, text="Log"); lf.pack(fill="both", expand=True, padx=8, pady=6)
    txt = tk.Text(lf, height=10, state="disabled"); txt.pack(fill="both", expand=True)

    sf = ttk.LabelFrame(root, text="Virtual Scope (HEX)"); sf.pack(fill="both", expand=True, padx=8, pady=6)
    scope = tk.Text(sf, height=8, state="disabled"); scope.pack(fill="both", expand=True)

    root.after(50, poll_rx)
    root.mainloop()

# ---- CLI ----
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="UART GUI — Real Device Only")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    run_gui(args.port, args.baud)
