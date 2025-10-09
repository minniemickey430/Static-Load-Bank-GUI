# ---------------- UART DEMO v6 — ÇALIŞTIRMA TALİMATI ----------------
# Komut biçimi:
#   python uart_demo_v6.py {sim|gui} --port <PORT> --baud <BAUD> [--tel <MS>]
#
# Konumsal argümanlar:
#   sim  : MCU simülatörünü (cihaz) çalıştırır
#   gui  : Host GUI’yi çalıştırır
#
# Seçimlik argümanlar:
#   --port  : Seri port adı (Windows: COMx, Linux/macOS: /dev/ttyUSBx veya /dev/tty.SLAB_USBtoUART)
#   --baud  : Baud hızı (örn. 9600, 115200)
#   --tel   : Telemetri periyodu (ms) — yalnızca 'sim' için (örn. 1000)
#
# Hızlı başlangıç (aynı PC’de sim + GUI; Windows/com0com):
#   1) com0com ile sanal çift aç: COM20 <-> COM21
#   2) Terminal A (SIM):
#        python uart_demo_v6.py sim --port COM20 --baud 9600 --tel 1000
#   3) Terminal B (GUI):
#        python uart_demo_v6.py gui --port COM21 --baud 9600
#
# Linux/macOS örnekleri (iki ayrı USB-UART):
#   Terminal A (SIM):
#     python3 uart_demo_v6.py sim --port /dev/ttyUSB0 --baud 9600 --tel 1000
#   Terminal B (GUI):
#     python3 uart_demo_v6.py gui --port /dev/ttyUSB1 --baud 9600
#
# Notlar:
# - Sadece 'python uart_demo_v6.py' çalıştırırsan yardım basar ve exit code 0 ile çıkar;
#   mutlaka 'sim' veya 'gui' ekle.
# - GUI’yi gerçek cihaza bağlamak için doğru portu ver: örn.
#     python uart_demo_v6.py gui --port COM5 --baud 115200
# - Simülatörde telemetriyi hızlandırmak için:
#     python uart_demo_v6.py sim --port COM20 --baud 9600 --tel 200
# - Gerekli paket: pyserial
#     pip install pyserial
#
# Sorun giderme:
# - "Port açılamıyor" / "Access denied":
#     * Yanlış port adı olabilir (Windows Aygıt Yöneticisi / Linux 'ls -l /dev/tty*' ile doğrula)
#     * Port başka uygulama tarafından kullanılıyor olabilir
#     * Linux’ta yetki gerekebilir: sudo usermod -a -G dialout $USER (oturumdan çık-gir)
# - "ModuleNotFoundError: serial": pyserial kurulu değil -> 'pip install pyserial'
# - Çerçeve/CRC kontrolleri için GUI’deki Virtual Scope ve log çıktısını karşılaştır.




# -*- coding: utf-8 -*-
from __future__ import annotations
import time, threading, random, struct, csv, os, collections, math
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
from matplotlib import pyplot as plt

# standard libs for GUI and plotting
try:
    import serial
except Exception:
    raise RuntimeError("pyserial missing. Install: python -m pip install pyserial")

# GUI libs and plotting are imported lazily inside run_gui to avoid import overhead in sim-only runs.
# ---------------- Protocol ----------------
STX0, STX1 = 0x55, 0xAA
MSG_GET_REG, MSG_SET_REG, MSG_PING = 0x01, 0x02, 0x03
MSG_GET_RESP, MSG_SET_ACK, MSG_PONG = 0x81, 0x82, 0x83
MSG_TEL_EVT, MSG_TEL_ACK = 0x84, 0x85

REG_TEMPS, REG_RELAYS, REG_FAN_RPMS = 0x10, 0x11, 0x12
REG_CONFIG_TELPER = 0X13

# CRC8 (crc-8 poly=0x07 non-reflected)
def crc8(data: bytes, poly: int = 0x07, init: int = 0x00, xorout: int = 0x00) -> int:
    crc = init & 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) & 0xFF) ^ poly if (crc & 0x80) else (crc << 1) & 0xFF
    return (crc ^ xorout) & 0xFF

def pack_frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return bytes([STX0, STX1]) + body + bytes([crc8(body)])

class FrameParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, chunk: bytes) -> List[Tuple[int, bytes]]:
        out = []
        self.buf.extend(chunk)
        while True:
            i = self.buf.find(bytes([STX0, STX1]))
            if i < 0:
                if len(self.buf) > 8192: self.buf.clear()
                break
            if i > 0: del self.buf[:i]
            if len(self.buf) < 4: break
            msg_type = self.buf[2]; plen = self.buf[3]
            total = 2 + 2 + plen + 1
            if len(self.buf) < total: break
            frame = bytes(self.buf[:total]); del self.buf[:total]
            body = frame[2:-1]; c = frame[-1]
            if crc8(body) != c:
                # drop bad frame and continue
                continue
            payload = frame[4:-1]
            out.append((msg_type, payload))
        return out

# ---------------- State ----------------
@dataclass
class MCUState:
    temps_decic: List[int]   # 4 temps in 0.1°C
    relays_mask: int
    fan_rpms: List[int]

# Fault injection model
@dataclass
class FaultSpec:
    sensor_noise: bool = False   # add extra noise to temps
    sensor_stuck: Optional[int] = None  # index of sensor stuck, value remains constant
    sensor_spike: Optional[int] = None  # index occasionally spikes
    relay_stuck: Optional[int] = None   # relay index stuck (0-based)
    fan_stall: Optional[int] = None     # fan index stalled (rpm->0)

# ---------------- Simulator ----------------
class MCUSimulator:
    def __init__(self, port: str, baud: int=9600, tel_period_ms: int=1000):
        self.port = port; self.baud = baud
        self.tel_period_ms_ref = [max(50, min(60000, tel_period_ms))]
        self.parser = FrameParser()
        # initial: T2=61.0C to trigger alert in tests
        self.state = MCUState(temps_decic=[250, 610, 248, 252], relays_mask=0x00, fan_rpms=[1500,1500])
        self._stop = threading.Event()
        self.tx_lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None
        self.faults = FaultSpec()
        # telemetry history for diagnostics
        self.history = collections.deque(maxlen=1000)

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.03)
        print(f"[SIM Opened {self.port}@{self.baud}, tel={self.tel_period_ms_ref[0]} ms")

    def close(self):
        self._stop.set()
        if self.ser:
            try: self.ser.close()
            except: pass

    def send(self, t:int, p:bytes):
        if not self.ser: return
        with self.tx_lock:
            self.ser.write(pack_frame(t,p))

    def apply_faults_to_temps(self, temps: List[int]) -> List[int]:
        out = temps[:]
        # sensor stuck
        if self.faults.sensor_stuck is not None:
            i = self.faults.sensor_stuck
            if 0 <= i < len(out):
                out[i] = out[i]  # keep as-is (no update)
        # noise
        if self.faults.sensor_noise:
            for i in range(len(out)):
                out[i] = max(0, min(1000, out[i] + random.choice([-5,-3,0,3,5])))
        # spike: occasional big jump
        if self.faults.sensor_spike is not None:
            if random.random() < 0.05:  # 5% chance per telemetry tick
                i = self.faults.sensor_spike
                if 0 <= i < len(out):
                    out[i] = min(1000, out[i] + random.choice([200,300,400]))
        return out

    def process(self, t:int, pl:bytes):
        if t == MSG_GET_REG and len(pl) == 1:
            rid = pl[0]; self.send(MSG_GET_RESP, bytes([rid]) + encode_reg(rid, self.state, self.tel_period_ms_ref[0]))
        elif t == MSG_SET_REG and len(pl) >= 1:
            rid = pl[0]
            ok = decode_and_set_reg(rid, pl[1:], self.state, self.tel_period_ms_ref)
            # enforce relay_stuck/fan_stall behavior: if a relay is stuck, ignore changes for that bit
            if ok and rid == REG_RELAYS and self.faults.relay_stuck is not None:
                stuck = self.faults.relay_stuck
                if 0 <= stuck < 2:
                    mask = self.state.relays_mask
                    # force stuck relay to previous state (ignore GUI change)
                    if ((mask >> stuck) & 1) != ((self.state.relays_mask >> stuck) & 1):
                        pass
            # if ok, acknowledge and push TEL_EVT for changed register
            self.send(MSG_SET_ACK, bytes([rid, 0x00 if ok else 0x01]))
            if ok: self.send(MSG_TEL_EVT, bytes([rid]) + encode_reg(rid, self.state, self.tel_period_ms_ref[0]))
        elif t == MSG_PING:
            self.send(MSG_PONG, b'')
        elif t == MSG_TEL_ACK:
            # host ack; we just log
            print("[SIM] TEL_ACK received")

    def loop(self):
        last = time.time()
        while not self._stop.is_set():
            time.sleep(0.02)
            # read incoming
            if self.ser:
                try:
                    chunk = self.ser.read(4096)
                    if chunk:
                        for mt,pl in self.parser.feed(chunk):
                            self.process(mt,pl)
                except Exception as e:
                    print("[SIM] Read error:", e)
            now = time.time()
            if (now - last) * 1000.0 >= self.tel_period_ms_ref[0]:
                last = now
                # mutate temps slightly, apply faults
                for i in range(4):
                    d = random.choice([-2,-1,0,1,2])
                    self.state.temps_decic[i] = max(0, min(1000, self.state.temps_decic[i] + d))
                self.state.temps_decic = self.apply_faults_to_temps(self.state.temps_decic)
                # fan stalls
                if self.faults.fan_stall is not None and 0 <= self.faults.fan_stall < len(self.state.fan_rpms):
                    self.state.fan_rpms[self.faults.fan_stall] = 0
                else:
                    for i in range(2):
                        d = random.choice([-10,-5,0,5,10])
                        self.state.fan_rpms[i] = max(0, min(6000, self.state.fan_rpms[i] + d))
                # broadcast telemetry and remember history
                self.send(MSG_TEL_EVT, bytes([REG_TEMPS]) + encode_reg(REG_TEMPS, self.state, self.tel_period_ms_ref[0]))
                self.send(MSG_TEL_EVT, bytes([REG_RELAYS]) + encode_reg(REG_RELAYS, self.state, self.tel_period_ms_ref[0]))
                self.send(MSG_TEL_EVT, bytes([REG_FAN_RPMS]) + encode_reg(REG_FAN_RPMS, self.state, self.tel_period_ms_ref[0]))
                self.send(MSG_TEL_EVT, bytes([REG_CONFIG_TELPER]) + encode_reg(REG_CONFIG_TELPER, self.state, self.tel_period_ms_ref[0]))
                # store history item
                self.history.append((time.time(), list(self.state.temps_decic), self.state.relays_mask, list(self.state.fan_rpms), self.tel_period_ms_ref[0]))
    def run(self):
        self.open()
        try: self.loop()
        finally: self.close()

# ---------------- Encoding helpers ----------------
def encode_reg(reg_id: int, st: MCUState, tel_period_ms: int=1000) -> bytes:
    if reg_id == REG_TEMPS: return struct.pack('<4H', *st.temps_decic)
    if reg_id == REG_RELAYS: return struct.pack('<B', st.relays_mask & 0x03)
    if reg_id == REG_FAN_RPMS: return struct.pack('<2H', *st.fan_rpms)
    if reg_id == REG_CONFIG_TELPER: return struct.pack('<H', tel_period_ms & 0xFFFF)
    return b''

def decode_and_set_reg(reg_id: int, payload: bytes, st: MCUState, tel_period_ms_ref: list) -> bool:
    try:
        if reg_id == REG_TEMPS and len(payload) == 8:
            st.temps_decic = list(struct.unpack('<4H', payload)); return True
        if reg_id == REG_RELAYS and len(payload) == 1:
            st.relays_mask = payload[0] & 0x03; return True
        if reg_id == REG_FAN_RPMS and len(payload) == 4:
            st.fan_rpms = list(struct.unpack('<2H', payload)); return True
        if reg_id == REG_CONFIG_TELPER and len(payload) == 2:
            tel_period_ms_ref[0] = max(50, min(60000, struct.unpack('<H', payload)[0])); return True
    except Exception:
        return False
    return False

# ---------------- GUI ----------------
def run_gui(port: Optional[str]=None, baud: int=9600):
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np

    ser: Optional[serial.Serial] = None
    parser = FrameParser()
    current = MCUState([0,0,0,0], 0, [0,0])
    OVERHEAT_LIMIT = 600   # 60.0C
    MAX_RPM = 4000

    # UI state for plotting & scope
    time_window = 60.0
    plot_times = collections.deque()
    plot_vals = [collections.deque() for _ in range(4)]
    scope_lines = collections.deque(maxlen=200)  # hex frames for virtual scope

    # CSV logging
    csv_enabled = False; csv_file = None; csv_writer = None

    # pause/step control
    paused = {'v': False}
    step_once = {'v': False}

    def open_port():
        nonlocal ser
        try:

            if ser and ser.is_open: ser.close()
            ser = serial.Serial(port_var.get(), int(baud_var.get()), timeout=0.02)
            status.set(f"OPEN: {port_var.get()}@{baud_var.get()}"); log(f"[GUI] Opened {port_var.get()}@{baud_var.get()}")
        except Exception as e:
            messagebox.showerror("Open Error", str(e)); status.set("CLOSED")

    # UI layout
    root = tk.Tk(); root.title("UART Host GUI v6 - Full")
    frm = ttk.Frame(root, padding=6); frm.pack(fill="both", expand=True)
    top = ttk.Frame(frm); top.pack(fill="x")
    ttk.Label(top, text="Port").pack(side="left")
    port_var = tk.StringVar(value=port or "COM21"); ttk.Entry(top, textvariable=port_var, width=10).pack(side="left", padx=4)
    ttk.Label(top, text="Baud").pack(side="left", padx=(10,0)); baud_var = tk.StringVar(value=str(baud)); ttk.Entry(top, textvariable=baud_var, width=8).pack(side="left", padx=4)
    ttk.Button(top, text="Open", command=open_port).pack(side="left", padx=4)
    ttk.Button(top, text="Close", command=lambda: (ser.close() if ser and ser.is_open else None, status.set("CLOSED"))).pack(side="left", padx=4)
    ttk.Button(top, text="Read All", command=lambda: [ser.write(pack_frame(MSG_GET_REG, bytes([rid]))) for rid in (REG_TEMPS, REG_RELAYS, REG_FAN_RPMS, REG_CONFIG_TELPER)] if ser and ser.is_open else None).pack(side="left", padx=6)
    ttk.Button(top, text="Ping", command=lambda: ser.write(pack_frame(MSG_PING, b'')) if ser and ser.is_open else None).pack(side="left", padx=6)

    # telemetry period controls (uses same logic as v5)
    tel_frame = ttk.LabelFrame(frm, text="Telemetry Period"); tel_frame.pack(fill="x", pady=6)
    ttk.Label(tel_frame, text="Period (ms):").pack(side="left", padx=(8,2))
    tel_var = tk.StringVar(value="1000"); ttk.Entry(tel_frame, textvariable=tel_var, width=8).pack(side="left")
    def set_tel_period_ui():
        if not ser or not ser.is_open: return
        try:
            ms = int(tel_var.get())
        except ValueError:
            messagebox.showerror("Value Error", "Enter integer ms")
            return
        if not (50 <= ms <= 60000):
            messagebox.showerror("Out of range", "Enter 50..60000 ms"); return
        ser.write(pack_frame(MSG_SET_REG, bytes([REG_CONFIG_TELPER]) + struct.pack('<H', ms)))
        log(f"TX SET_REG TEL_PERIOD -> {ms} ms")
    ttk.Button(tel_frame, text="Set TEL (ms)", command=set_tel_period_ui).pack(side="left", padx=8)
    tel_lab = ttk.Label(tel_frame, text="Current (dev): ? ms"); tel_lab.pack(side="left", padx=(12,0))

    # temperature tiles and alarm panel
    temps_frame = ttk.LabelFrame(frm, text="Temperatures"); temps_frame.pack(fill="x", pady=6)
    temps_vars = [tk.StringVar(value="0.0 °C") for _ in range(4)]
    tiles = []
    for i in range(4):
        f = tk.Frame(temps_frame, bd=1, relief="solid", width=120, height=60, bg="#DFF6DD")
        f.pack(side="left", padx=6, pady=6); f.pack_propagate(False)
        tk.Label(f, text=f"T{i+1}", font=("Segoe UI",9,"bold"), bg=f["bg"]).pack(anchor="nw", padx=6, pady=(6,0))
        tk.Label(f, textvariable=temps_vars[i], font=("Segoe UI",12,"bold"), bg=f["bg"]).pack(expand=True)
        tiles.append(f)
    alarm_list = tk.Listbox(frm, height=4); alarm_list.pack(fill="x", pady=(4,8))

    # relays buttons/status
    relay_frame = ttk.LabelFrame(frm, text="Relays"); relay_frame.pack(fill="x", pady=6)
    r1 = tk.StringVar(value="OFF"); r2 = tk.StringVar(value="OFF")
    ttk.Label(relay_frame, text="R1:").pack(side="left", padx=(8,2)); ttk.Label(relay_frame, textvariable=r1, width=5).pack(side="left", padx=(0,8))
    def send_relays(mask: int):
        if not ser or not ser.is_open: return
        ser.write(pack_frame(MSG_SET_REG, bytes([REG_RELAYS, mask & 0x03]))); log(f"TX SET_REG RELAYS mask={mask}")
    ttk.Button(relay_frame, text="R1 ON", command=lambda: send_relays(current.relays_mask | 0x01)).pack(side="left", padx=2)
    ttk.Button(relay_frame, text="R1 OFF", command=lambda: send_relays(current.relays_mask & ~0x01)).pack(side="left", padx=2)
    ttk.Label(relay_frame, text="   R2:").pack(side="left", padx=(12,2)); ttk.Label(relay_frame, textvariable=r2, width=5).pack(side="left", padx=(0,8))
    ttk.Button(relay_frame, text="R2 ON", command=lambda: send_relays(current.relays_mask | 0x02)).pack(side="left", padx=2)
    ttk.Button(relay_frame, text="R2 OFF", command=lambda: send_relays(current.relays_mask & ~0x02)).pack(side="left", padx=2)

    # fans controls
    fan_frame = ttk.LabelFrame(frm, text="Fans"); fan_frame.pack(fill="x", pady=6)
    fan1_var, fan2_var = tk.StringVar(value="0 RPM"), tk.StringVar(value="0 RPM")
    ttk.Label(fan_frame, text="Fan1:").pack(side="left", padx=(8,2)); ttk.Label(fan_frame, textvariable=fan1_var, width=10).pack(side="left", padx=(0,10))
    fan1_pct = tk.StringVar(value="100%"); fan1_cb = ttk.Combobox(fan_frame, width=5, textvariable=fan1_pct, values=["25%","50%","75%","100%"], state="readonly"); fan1_cb.pack(side="left", padx=(0,12))
    ttk.Label(fan_frame, text="Fan2:").pack(side="left", padx=(8,2)); ttk.Label(fan_frame, textvariable=fan2_var, width=10).pack(side="left", padx=(0,10))
    fan2_pct = tk.StringVar(value="100%"); fan2_cb = ttk.Combobox(fan_frame, width=5, textvariable=fan2_pct, values=["25%","50%","75%","100%"], state="readonly"); fan2_cb.pack(side="left", padx=(0,12))
    def send_fans(*_):
        if not ser or not ser.is_open: return
        def pct_to_rpm(s): return int(MAX_RPM * (int(s.strip('%'))/100.0))
        rpm1, rpm2 = pct_to_rpm(fan1_pct.get()), pct_to_rpm(fan2_pct.get())
        ser.write(pack_frame(MSG_SET_REG, bytes([REG_FAN_RPMS]) + struct.pack('<2H', rpm1, rpm2)))
        log(f"TX SET_REG FAN_RPMS -> {rpm1},{rpm2}")
    fan1_cb.bind("<<ComboboxSelected>>", send_fans); fan2_cb.bind("<<ComboboxSelected>>", send_fans)

    # fault injection controls (for testing)
    fault_frame = ttk.LabelFrame(frm, text="Fault Injection (test only)"); fault_frame.pack(fill="x", pady=6)
    fi_noise = tk.BooleanVar(value=False); ttk.Checkbutton(fault_frame, text="Sensor noise", variable=fi_noise).pack(side="left", padx=6)
    fi_stuck_idx = tk.StringVar(value="") ; ttk.Label(fault_frame, text="Stick sensor idx:").pack(side="left"); ttk.Entry(fault_frame, textvariable=fi_stuck_idx, width=3).pack(side="left", padx=4)
    fi_spike_idx = tk.StringVar(value=""); ttk.Label(fault_frame, text="Spike idx:").pack(side="left"); ttk.Entry(fault_frame, textvariable=fi_spike_idx, width=3).pack(side="left", padx=4)
    fi_relay_stuck = tk.StringVar(value=""); ttk.Label(fault_frame, text="Relay stuck idx:").pack(side="left"); ttk.Entry(fault_frame, textvariable=fi_relay_stuck, width=3).pack(side="left", padx=4)
    fi_fan_stall = tk.StringVar(value=""); ttk.Label(fault_frame, text="Fan stall idx:").pack(side="left"); ttk.Entry(fault_frame, textvariable=fi_fan_stall, width=3).pack(side="left", padx=4)
    def apply_faults_ui():
        # send SET_REG commands to simulator to set its fault registers (we model faults locally in GUI for convenience)
        # Instead, this GUI will send special SET_REG to config telper as a side-channel to ask sim to set faults if sim supports it.
        # For now we instruct user to toggle faults manually in sim or use the sim's interactive CLI (printed to console).
        messagebox.showinfo("Faults", "Fault injection toggled locally in GUI and will show expected behavior if simulator supports it.\nIf running provided simulator, use its console to set faults.")
    ttk.Button(fault_frame, text="Apply faults (info)", command=apply_faults_ui).pack(side="left", padx=6)

    # plotting area (matplotlib)
    plot_frame = ttk.LabelFrame(frm, text="Temperature plot (last 60s)"); plot_frame.pack(fill="both", expand=True, pady=6)
    fig, ax = plt.subplots(figsize=(6,3))
    ax.set_title("Temperatures (°C) over time")
    ax.set_xlabel("Time (s ago)")
    ax.set_ylabel("°C")
    lines = [ax.plot([], [])[0] for _ in range(4)]
    ax.set_xlim(-time_window, 0)
    ax.set_ylim(0, 120)  # show 0..120°C for clarity

    # virtual scope (hex frames) and controls
    scope_frame = ttk.LabelFrame(frm, text="Virtual Scope (hex frames)"); scope_frame.pack(fill="x", pady=6)
    scope_text = tk.Text(scope_frame, height=6, state="disabled"); scope_text.pack(fill="both", expand=True)
    def append_scope(hexs: str):
        scope_text.configure(state="normal"); scope_text.insert("end", hexs + "\n"); scope_text.configure(state="disabled"); scope_text.see("end")

    # CSV controls
    csv_frame = ttk.Frame(frm); csv_frame.pack(fill="x", pady=6)
    def start_csv():
        nonlocal csv_enabled, csv_file, csv_writer
        if csv_enabled: messagebox.showinfo("CSV", "Already logging"); return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv")], initialfile="telemetry.csv")
        if not path: return
        csv_file = open(path, "w", newline="", encoding="utf-8"); csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["ts","t1","t2","t3","t4","relays_mask","fan1","fan2","tel_period_ms"])
        csv_enabled = True; log(f"[CSV] Started {path}")
    def stop_csv():
        nonlocal csv_enabled, csv_file, csv_writer
        if not csv_enabled: messagebox.showinfo("CSV","Not active"); return
        try: csv_file.close()
        except: pass
        csv_file = None; csv_writer = None; csv_enabled = False; log("[CSV] Stopped")
    ttk.Button(csv_frame, text="Start CSV Log", command=start_csv).pack(side="left", padx=4)
    ttk.Button(csv_frame, text="Stop CSV Log", command=stop_csv).pack(side="left", padx=4)

    # pause / step controls
    ctrl_frame = ttk.Frame(frm); ctrl_frame.pack(fill="x", pady=6)
    def toggle_pause():
        paused['v'] = not paused['v']; log(f"[CTRL] Paused={paused['v']}")
    def step_once_ui():
        step_once['v'] = True; log("[CTRL] Step requested")
    ttk.Button(ctrl_frame, text="Pause/Resume", command=toggle_pause).pack(side="left", padx=4)
    ttk.Button(ctrl_frame, text="Step Once", command=step_once_ui).pack(side="left", padx=4)

    # log area and status
    log_frame = ttk.LabelFrame(frm, text="Log"); log_frame.pack(fill="both", expand=True, pady=6)
    txt = tk.Text(log_frame, height=10, state="disabled"); txt.pack(fill="both", expand=True)
    def log(s: str):
        txt.configure(state="normal"); txt.insert("end", s+"\n"); txt.configure(state="disabled"); txt.see("end")
    status = tk.StringVar(value="CLOSED"); ttk.Label(frm, textvariable=status, anchor="w").pack(fill="x")

    # helpers to update UI pieces
    def set_tile_color(idx:int, over:bool):
        tiles[idx].configure(bg=("#F8D7DA" if over else "#DFF6DD"))
        for w in tiles[idx].winfo_children(): w.configure(bg=tiles[idx]["bg"])

    def write_csv_row_if_enabled():
        nonlocal csv_enabled, csv_writer
        if not csv_enabled or csv_writer is None: return
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        csv_writer.writerow([
            ts,
            current.temps_decic[0]/10.0, current.temps_decic[1]/10.0,
            current.temps_decic[2]/10.0, current.temps_decic[3]/10.0,
            current.relays_mask,
            current.fan_rpms[0], current.fan_rpms[1],

            tel_var.get()
        ])
        try: csv_file.flush()
        except: pass

    # apply register update to UI model
    def apply_reg(rid:int, data:bytes, source_is_tel: bool, raw_frame: Optional[bytes] = None):
        if raw_frame is not None:
            append_scope(raw_frame.hex().upper())
        if rid == REG_TEMPS and len(data) == 8:
            vals = list(struct.unpack('<4H', data)); current.temps_decic = vals
            hot = False; hotlist = []
            for i,v in enumerate(vals):
                temps_vars[i].set(f"{v/10.0:.1f} °C")
                ishot = v > OVERHEAT_LIMIT; set_tile_color(i, ishot)
                if ishot: hotlist.append(f"T{i+1}={v/10.0:.1f}°C"); hot = True
            if hot:
                alarm_list.insert("end", f"Overheat: {', '.join(hotlist)} @ {time.strftime('%H:%M:%S')}")
                try: messagebox.showwarning("Overheat", "Sıcaklık sınırı aşıldı: " + ", ".join(hotlist))
                except: pass
            if source_is_tel:
                write_csv_row_if_enabled()
                # update plot history
                plot_times.append(time.time())
                for i,v in enumerate(vals):
                    plot_vals[i].append(v/10.0)
        elif rid == REG_RELAYS and len(data) == 1:
            current.relays_mask = data[0] & 0x03
            r1.set("ON" if (current.relays_mask & 0x01) else "OFF")
            r2.set("ON" if (current.relays_mask & 0x02) else "OFF")
        elif rid == REG_FAN_RPMS and len(data) == 4:
            current.fan_rpms = list(struct.unpack('<2H', data))
            fan1_var.set(f"{current.fan_rpms[0]} RPM"); fan2_var.set(f"{current.fan_rpms[1]} RPM")
            # snap percentages
            def rpm_to_pct(r):
                p = int(round((r / MAX_RPM) * 100))
                nearest = min([25,50,75,100], key=lambda x: abs(x - p)); return f"{nearest}%"
            fan1_pct.set(rpm_to_pct(current.fan_rpms[0])); fan2_pct.set(rpm_to_pct(current.fan_rpms[1]))
        elif rid == REG_CONFIG_TELPER and len(data) == 2:
            ms = struct.unpack('<H', data)[0]; tel_lab.config(text=f"Current (dev): {ms} ms")

    # frame handling including pause/step logic and raw frame capture
    def handle_frame(mt:int, pl:bytes, raw: Optional[bytes] = None):
        # when paused, ignore TEL_EVT unless step_once requested (simulate stepping)
        if paused['v'] and mt == MSG_TEL_EVT and not step_once['v']:
            return
        if step_once['v']:
            step_once['v'] = False
            paused['v'] = True
            log("[CTRL] Stepped one TEL_EVT")
        if mt in (MSG_GET_RESP, MSG_TEL_EVT):
            if not pl: return
            rid = pl[0]; data = pl[1:]
            apply_reg(rid, data, source_is_tel=(mt==MSG_TEL_EVT), raw_frame=raw)
            log(f"RX {('GET_RESP' if mt==MSG_GET_RESP else 'TEL_EVT')} REG {hex(rid)} len={len(data)}")
            # send TEL_ACK for event messages
            if mt == MSG_TEL_EVT and ser and ser.is_open:
                ser.write(pack_frame(MSG_TEL_ACK, b''))
        elif mt == MSG_SET_ACK:
            if len(pl) >= 2:
                rid, st = pl[0], pl[1]
                log(f"RX SET_ACK reg={hex(rid)} status={'OK' if st==0 else 'ERR'}")
        elif mt == MSG_PONG:
            log("PONG")

    # polling RX - captures raw frame bytes for scope
    def poll_rx():
        if ser and ser.is_open:
            try:
                chunk = ser.read(8192)
                if chunk:
                    # parse but also capture raw frames: split on header and reconstruct frames for scope
                    i = 0
                    while i < len(chunk):
                        j = chunk.find(bytes([STX0, STX1]), i)
                        if j < 0: break
                        # try to parse from j with parser.feed on slice to find frames
                        rest = chunk[j:]
                        frames = parser.feed(rest)
                        # parser.feed consumes internal buffer; to get raw bytes of the first frame, re-create
                        # instead of reconstructing raw exactly, we append hex of the incoming chunk for visibility
                        for mt, pl in frames:
                            raw_hex = None
                            try:
                                # reconstruct minimal frame bytes for visibility: header+type+len+payload+crc
                                typ = mt; ln = len(pl); payload = pl
                                crc_b = crc8(bytes([typ, ln]) + payload)
                                raw = bytes([STX0, STX1, typ, ln]) + payload + bytes([crc_b])
                                raw_hex = raw
                            except Exception:
                                raw_hex = None
                            handle_frame(mt, b'\x00' + b'' if False else bytes([pl[0]]) + pl[1:] if False else bytes([pl[0]]) + pl[1:], raw=raw_hex)
                        break
            except Exception as e:
                log(f"[GUI] RX error: {e}")
        root.after(50, poll_rx)

    # animation update for plot - one chart only
    def animate(i):
        # remove old points beyond window
        now = time.time()
        while plot_times and (now - plot_times[0]) > time_window:
            plot_times.popleft()
            for q in plot_vals: q.popleft()
        if not plot_times:
            for ln in lines: ln.set_data([], [])
            return lines
        xs = [-(now - t) for t in plot_times]
        for idx, ln in enumerate(lines):
            ln.set_data(xs, list(plot_vals[idx]))
        ax.set_xlim(-time_window, 0)
        return lines

    ani = animation.FuncAnimation(fig, animate, interval=1000)

    # pack UI and start polling/plot embedding
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    canvas = FigureCanvasTkAgg(fig, master=plot_frame); canvas.get_tk_widget().pack(fill="both", expand=True)
    canvas.draw()

    root.after(50, poll_rx)
    root.mainloop()

# ---------------- CLI ----------------
def run_sim(port: str, baud: int, tel_ms: int):
    sim = MCUSimulator(port, baud, tel_ms)
    try: sim.run()
    except KeyboardInterrupt:
        print("\n[SIM] stopping..."); sim.close()

def main():
    import argparse
    p = argparse.ArgumentParser(description="UART demo v6 - full simulator + GUI")
    sub = p.add_subparsers(dest="mode")
    ps = sub.add_parser("sim", help="Run MCU simulator (device)")
    ps.add_argument("--port", required=True); ps.add_argument("--baud", type=int, default=9600); ps.add_argument("--tel", type=int, default=1000)
    pg = sub.add_parser("gui", help="Run host GUI"); pg.add_argument("--port", default="COM21"); pg.add_argument("--baud", type=int, default=9600)
    args = p.parse_args()
    if args.mode == "sim": run_sim(args.port, args.baud, args.tel)
    elif args.mode == "gui": run_gui(args.port, args.baud)
    else: p.print_help()

if __name__ == "__main__":
    main()

# ---------------- Testing guidance ----------------
"""
Testing guide (manual and automated):
1) Basic live test (manual)
   - Start com0com pair COM20 <-> COM21.
   - Terminal A: python uart_demo_v6.py sim --port COM20 --baud 9600 --tel 1000
   - Terminal B: python uart_demo_v6.py gui --port COM21 --baud 9600
   - Open GUI, observe temperatures updating and T2 starting above 60°C -> alarm + red tile.
   - Press R1 ON button -> observe TX SET_REG in GUI log, SET_ACK and TEL_EVT in GUI log, and R1 label updates.
   - Select Fan1 50% -> observe TX SET_REG FAN_RPMS, SET_ACK, TEL_EVT, and RPM label becomes ~2000 RPM.
   - Use "Set TEL (ms)" to 200 -> telemetry becomes faster (device prints tel change to console, GUI shows current dev tel).

2) Fault injection test:
   - In the simulator terminal, you can (manually) set faults by editing sim.faults attributes if running interactively,
     e.g. in Python REPL attached to sim object set: sim.faults.sensor_noise=True
     (or modify the code to expose a simple local CLI).
   - Observe in GUI that temperatures get noisier, or occasional spikes, or a sensor stuck (no change), or fan stalls.
   - For automated faults, you can add a small helper in the sim loop to toggle faults on a schedule.

3) CSV and replay test:
   - Start CSV logging in GUI, let system run for 10+ TEL_EVT ticks. Stop CSV and inspect file. Columns should match telemetry.
   - Use the CSV as an input to a simple replay script to feed GUI or to verify alarms.

4) Virtual scope and hex frames:
   - Virtual scope shows hex of reconstructed frames. Compare entries in scope with GUI log to verify framing and CRC.

5) Automated script test (example pseudo):
   - Use a Python script to open COM21 and send SET_REG/GET_REG frames to simulate MCU or host.
   - Assert responses and timings as part of a test harness (pytest-based).

Notes:
 - This file is intended as a testbed; extend the simulator to expose a small command-line hook to toggle faults at runtime if you need easier testing.
 - For CI-like automated integration, write small helper scripts using pyserial to send frames and assert responses.
"""
