

from __future__ import annotations
import time, threading, csv, re
from typing import Optional, Dict, List, Set

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    raise RuntimeError("pyserial is required. Install: python -m pip install pyserial") from e

# ------------------ Protokol ------------------
STX = 0xAA
ETX = 0x55
BAUD = 115200

CMD_RLY  = 0x10
CMD_FAN  = 0x20
CMD_TMP  = 0x30
CMD_ADDR = 0x40
CMD_EVT  = 0x50
CMD_OLED = 0x60

# Röle
SUB_RLY_ON  = 0x01
SUB_RLY_OFF = 0x02
SUB_RLY_QRY = 0x03
SUB_RLY_ERR = 0xEE

# Sıcaklık
SUB_TMP_ALL = 0x00
SUB_TMP_MAX = 0x01  # (kullanılmıyor; GUI kendi min/max'ını hesaplar)
SUB_TMP_MIN = 0x02  # (kullanılmıyor; GUI kendi min/max'ını hesaplar)
SUB_TMP_ONE = 0x03

# Fan
FAN_LABELS = ["%0", "%25", "%50", "%75", "%100", "AUTO"]
FAN_SUBMAP = {"%0":0, "%25":1, "%50":2, "%75":3, "%100":4, "AUTO":5}
FAN_STATUS_MANUAL_OK = 0xF1
FAN_STATUS_AUTO_OK   = 0xF2

# OLED
SUB_OLED_SLEEP = 0x00
SUB_OLED_WAKE  = 0x01
OLED_DEBOUNCE_MS = 300

# Sistem hata/durum (CMD=0x50 → a1)
SYS_STATUS_MAP = {
    0x01: "Invalid length",
    0x02: "Unknown subcmd",
    0x03: "Missing subcmd",
    0x04: "Buffer overflow",
    0x05: "Timeout",
    0x06: "Sensor not found",
    0x07: "Relay fault (Holt check failed)",
}

# ------------------ CRC8-ATM ------------------
def crc8_atm(buf: bytes, poly=0x07, init=0x00, xorout=0x00) -> int:
    c = init & 0xFF
    for b in buf:
        c ^= b
        for _ in range(8):
            c = (((c << 1) & 0xFF) ^ poly) if (c & 0x80) else ((c << 1) & 0xFF)
    return (c ^ xorout) & 0xFF

# ------------------ Paketleme ------------------
def pack(addr:int, cmd:int, data:bytes) -> bytes:
    addr &= 0xFF; cmd &= 0xFF
    ln = len(data) & 0xFF
    body = bytes([addr, cmd, ln]) + data
    crc  = crc8_atm(body)
    return bytes([STX]) + body + bytes([crc, ETX])

def hex_dump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

# ------------------ Serial Client ------------------
class SerialClient:
    def __init__(self, port:str, baud:int, device_addr:int, rx_cb, rx_line_cb, log_cb, hex_cb):
        self.port=port; self.baud=baud; self.device_addr=device_addr
        self.rx_cb=rx_cb; self.rx_line_cb=rx_line_cb
        self.log_cb=log_cb; self.hex_cb=hex_cb
        self.ser: Optional[serial.Serial]=None
        self._stop=threading.Event(); self._th=None
        self._buf=bytearray(); self._text=bytearray()

    def log(self,s): self.log_cb and self.log_cb(s)
    def hexlog(self,s): self.hex_cb and self.hex_cb(s)

    def open(self):
        # Eski thread/port varsa temizle
        if self._th and self._th.is_alive():
            self._stop.set()
            try:
                if self.ser:
                    try:
                        self.ser.reset_input_buffer()
                        self.ser.reset_output_buffer()
                    except:
                        pass
                    try:
                        if hasattr(self.ser, "cancel_read"):  self.ser.cancel_read()
                        if hasattr(self.ser, "cancel_write"): self.ser.cancel_write()
                    except:
                        pass
                    if self.ser.is_open:
                        self.ser.close()
            except:
                pass
            self._th.join(timeout=0.5)

        if not self.port:
            raise RuntimeError("Select a port first")

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        self.log(f"[OPEN] {self.port}@{self.baud} (Addr={self.device_addr})")

    def close(self):
        self._stop.set()
        try:
            if self.ser:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except:
                    pass
                try:
                    if hasattr(self.ser, "cancel_read"):  self.ser.cancel_read()
                    if hasattr(self.ser, "cancel_write"): self.ser.cancel_write()
                except:
                    pass
                if self.ser.is_open:
                    self.ser.close()
        finally:
            th = self._th
            self._th = None
            if th and th.is_alive():
                th.join(timeout=0.8)
            self.ser = None
        self.log("[CLOSE]")

    def is_open(self)->bool: return bool(self.ser and self.ser.is_open)

    def send(self,blob:bytes):
        if not self.is_open():
            self.hexlog(f"TX[{self.port or 'DISCONNECTED'}] {hex_dump(blob)}  ;; NOT SENT")
            return
        try:
            self.ser.write(blob)
            self.hexlog(f"TX[{self.port}] {hex_dump(blob)}")
        except Exception as e:
            self.log(f"[TX ERR] {e}")

    # ---- RX parse ----
    def _emit_text(self):
        while True:
            nl=self._text.find(b'\n')
            if nl<0: break
            line=self._text[:nl].rstrip(b'\r').decode('utf-8','ignore')
            del self._text[:nl+1]
            try: self.rx_line_cb(line)
            except Exception as e: self.log(f"[TEXT CB ERR] {e}")

    def _consume_one(self)->bool:
        buf=self._buf
        s=buf.find(bytes([STX]))
        if s<0:
            if buf:
                self._text.extend(buf); buf.clear(); self._emit_text()
            return False
        if s>0:
            self._text.extend(buf[:s]); del buf[:s]; self._emit_text()

        if len(buf)<6: return False
        ln = buf[3]
        need = 6 + ln
        if len(buf) < need: return False

        frame = bytes(buf[:need])
        if frame[-1] != ETX:
            del buf[0]; return True
        body = frame[1:-2]
        crc  = frame[-2]
        exp  = crc8_atm(body)
        if crc != exp:
            self.hexlog(f"RX[{self.port}] {hex_dump(frame)}")
            self.log(f"[CRC ERR] got=0x{crc:02X} exp=0x{exp:02X}; drop")
            del buf[0]
            return True
        self.hexlog(f"RX[{self.port}] {hex_dump(frame)}")
        addr = frame[1]; cmd = frame[2]
        data = frame[4:4+ln]
        sub  = data[0] if ln>=1 else 0
        a1   = data[1] if ln>=2 else 0
        a2   = data[2] if ln>=3 else 0
        try:
            self.rx_cb(addr, cmd, sub, a1, a2, data)
        except Exception as e:
            self.log(f"[RX BIN CB ERR] {e}")
        del buf[:need]
        return True

    def _loop(self):
        while not self._stop.is_set():
            try:
                if self.ser and self.ser.is_open:
                    chunk=self.ser.read(512)
                    if chunk:
                        self._buf.extend(chunk)
                        progressed=True
                        while progressed: progressed=self._consume_one()
                        self._emit_text()
                time.sleep(0.01)
            except Exception as e:
                self.log(f"[RX ERR] {e}"); time.sleep(0.05)

# ------------------ Shared Serial Hub (Same-COM modu) ------------------
class SerialHub:
    def __init__(self, app:'App'):
        self.app = app
        self.client: Optional[SerialClient] = None
        self.shared_port: str = ""
        self.addr_to_panel: Dict[int, DevicePanel] = {}

    def open(self, port:str):
        self.shared_port = port
        self.close()
        self.client = SerialClient(
            port, BAUD, 0,
            self._rx_bin, self._rx_line,
            self._log, self._hex
        )
        self.client.open()
        self._log("[HUB] Shared serial opened.")
        self.refresh_addr_map()

    def close(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
            self.client = None
            self._log("[HUB] Shared serial closed.")

    def is_open(self)->bool:
        return bool(self.client and self.client.is_open())

    def refresh_addr_map(self):
        mapping: Dict[int, DevicePanel] = {}
        for p in self.app.panels:
            try:
                a = int(p.addr_var.get()) & 0xFF
                mapping[a] = p
            except:
                pass
        self.addr_to_panel = mapping

    def send_from_panel(self, panel:'DevicePanel', frame:bytes, note:str):
        port = self.shared_port or (self.client.port if self.client else 'DISCONNECTED')
        if not self.is_open():
            panel.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; NOT SENT (shared closed)")
            return
        panel.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; {note}")
        self.client.send(frame)

    def _log(self, s:str):
        if self.app.panels:
            self.app.panels[0].log(s)

    def _hex(self, s:str):
        if self.app.panels:
            self.app.panels[0].hex_log(s)

    def _rx_bin(self, addr:int, cmd:int, sub:int, a1:int, a2:int, data:bytes):
        self.refresh_addr_map()
        target = self.addr_to_panel.get(addr)
        port = self.shared_port or (self.client.port if self.client else 'DISCONNECTED')
        if target:
            target.hex_log(f"RX[{port}] {hex_dump(pack(addr, cmd, data))}")
            try:
                target.on_frame_bin(addr, cmd, sub, a1, a2, data)
            except Exception as e:
                target.log(f"[HUB RX ERR] {e}")
        else:
            if self.app.panels:
                self.app.panels[0].log(f"[HUB] Unmapped ADDR=0x{addr:02X} cmd=0x{cmd:02X}")

    def _rx_line(self, line:str):
        for p in self.app.panels:
            p.on_line_text(line)

# ------------------ GUI ------------------
class DevicePanel(ttk.Frame):
    MAX_HEX_LINES = 1800
    KEEP_HEX_LINES = 1200
    MAX_LOG_LINES = 1500
    KEEP_LOG_LINES = 900

    def __init__(self, master, idx:int, app:'App'):
        super().__init__(master)
        self.app = app
        self.idx=idx
        self.default_addr=idx+1          # Drawer adresleri 1..5
        self.client: Optional[SerialClient]=None

        self.port_var=tk.StringVar(value="")
        self.addr_var=tk.IntVar(value=self.default_addr)
        self.addr_var.trace_add('write', lambda *args: self.app.hub.refresh_addr_map())

        self.last_tx=tk.StringVar(value="Last TX: -")
        self.last_rx=tk.StringVar(value="Last RX: -")

        self.status_text=tk.StringVar(value="Status: Idle")
        self.fan_mode=tk.StringVar(value="Fan Mode: MANUAL")
        self.oled_state=tk.StringVar(value="OLED: WAKE")

        self.relays=[0]*40
        self.ntc_vals=[0]*20           # 0.1°C *10 => decicelsius
        self.ntc_seen=[False]*20
        self.ntc_min=0; self.ntc_max=0

        self.fans=[0,0,0]

        # NTC threshold — varsayılan 40.0°C
        self.ntc_threshold_var = tk.DoubleVar(value=40.0)
        self.NTC_LIMIT = 400  # 40.0°C

        self._oled_last_ms = 0

        # CSV (periyodik)
        self.csv_enabled=False
        self.csv_file=None
        self.csv_writer=None
        self.csv_interval_var = tk.IntVar(value=1)  # saniye
        self._csv_job = None

        # NTC UI coalesce
        self._ntc_dirty: Set[int] = set()
        self._ntc_flush_job = None

        self._init_style(); self._build_ui()

    # ---------- style ----------
    def _init_style(self):
        st=ttk.Style(self)
        try: st.theme_use('clam')
        except: pass

        bg = "#f4f5f7"
        card_bg = "#ffffff"
        card_bd = "#d1d5db"
        txt = "#111827"
        acc = "#2563eb"
        acc_act = "#1d4ed8"
        danger = "#dc2626"
        danger_act = "#b91c1c"

        self.configure(style="Root.TFrame")
        st.configure("Root.TFrame", background=bg)
        st.configure("Header.TFrame", background=bg)
        st.configure("Header.TLabel", background=bg, foreground=txt, font=('Segoe UI', 12, 'bold'))

        st.configure("Card.TLabelframe", background=card_bg, foreground=txt, bordercolor=card_bd, relief="solid")
        st.configure("Card.TLabelframe.Label", background=card_bg, foreground="#374151", font=('Segoe UI', 10, 'bold'))
        st.configure("TLabel", background=card_bg, foreground=txt)
        st.configure("TFrame", background=card_bg)
        st.configure("TNotebook", background=bg, tabmargins=[8,6,8,0])
        st.configure("TNotebook.Tab", padding=[10,6], font=('Segoe UI', 10, 'bold'))
        st.map("TNotebook.Tab", background=[("selected", card_bg)])

        st.configure("Accent.TButton", foreground="white", background=acc, padding=(10,4))
        st.map("Accent.TButton", background=[('active',acc_act)])
        st.configure("Danger.TButton", foreground="white", background=danger, padding=(10,4))
        st.map("Danger.TButton", background=[('active',danger_act)])
        st.configure("Tool.TButton", padding=(6,2))

        st.configure("Status.TFrame", background="#eef2f7")
        st.configure("Status.TLabel", background="#eef2f7", foreground="#374151", font=('Segoe UI', 9))

    # ---------- utils ----------
    def _trim_text_lines(self, widget: tk.Text, max_lines:int, keep_lines:int):
        try:
            end_index = widget.index('end-1c')
            total_lines = int(end_index.split('.')[0])
            if total_lines > max_lines:
                first_keep_line = total_lines - keep_lines
                widget.delete('1.0', f'{first_keep_line}.0')
        except Exception:
            pass

    def hex_log(self,s:str):
        self.hextext.insert('end', s+"\n")
        self.hextext.see('end')
        self._trim_text_lines(self.hextext, self.MAX_HEX_LINES, self.KEEP_HEX_LINES)
        if s.startswith("TX"): self.last_tx.set("Last TX: "+s.split(" ",1)[1])
        elif s.startswith("RX"): self.last_rx.set("Last RX: "+s.split(" ",1)[1])

    def log(self,s:str, tag:str=None):
        self.logtext.insert('end', s+"\n", (tag,) if tag else ())
        self.logtext.see('end')
        self._trim_text_lines(self.logtext, self.MAX_LOG_LINES, self.KEEP_LOG_LINES)

    def log_error(self, s:str):
        self.set_status("Error: " + s)
        self.log(s, tag="error")

    def set_status(self, text:str):
        self.status_text.set("Status: " + text)

    # ---------- shared-mode UI helpers ----------
    def set_shared_mode(self, on: bool):
        if on:
            self.port_cb.configure(state='disabled')
            self.btn_connect.configure(state='disabled')
            self.btn_disconnect.configure(state='disabled')
            # bireysel bağlantı açıksa kapat (portu boşa çıkar)
            try: 
                if self.client and self.client.is_open(): 
                    self.client.close()
            except: 
                pass
            self.client = None
        else:
            self.port_cb.configure(state='readonly')
            self.btn_connect.configure(state='normal')

    # ---------- UI ----------
    def _build_ui(self):
        head = ttk.Frame(self, style="Header.TFrame")
        head.grid(row=0, column=0, sticky="ew", padx=8, pady=(8,4))
        head.grid_columnconfigure(12, weight=1)

        ttk.Label(head,text=f"Drawer {self.idx+1}",style='Header.TLabel').grid(row=0, column=0, padx=(6,12))
        ttk.Label(head,text="Port:",style='Header.TLabel').grid(row=0, column=1, sticky="e")
        self.port_cb=ttk.Combobox(head,textvariable=self.port_var,width=18,state='readonly')
        self.port_cb.grid(row=0, column=2, padx=(6,10))
        self.port_cb.bind('<<ComboboxSelected>>', lambda e: self.app.on_panel_port_changed(self))

        ttk.Label(head,text="Addr:",style='Header.TLabel').grid(row=0, column=3, sticky="e")
        self.addr_sp=ttk.Spinbox(head,from_=1,to=5,textvariable=self.addr_var,width=6)
        self.addr_sp.grid(row=0, column=4, padx=(6,10))
        ttk.Button(head,text="Scan",command=self.refresh_ports,style="Tool.TButton").grid(row=0, column=5)
        self.btn_connect = ttk.Button(head,text="Connect",command=self.connect,style="Accent.TButton")
        self.btn_connect.grid(row=0, column=6, padx=(8,4))
        self.btn_disconnect = ttk.Button(head,text="Disconnect",command=self.disconnect, state='disabled',style="Tool.TButton")
        self.btn_disconnect.grid(row=0, column=7)

        # CSV interval + start/stop
        ttk.Label(head,text="Interval (s):",style='Header.TLabel').grid(row=0, column=8, sticky="e")
        self.csv_sp = ttk.Spinbox(head, from_=1, to=3600, increment=1, textvariable=self.csv_interval_var, width=6)
        self.csv_sp.grid(row=0, column=9, padx=(6,10))
        ttk.Button(head,text="Start CSV",command=self.start_csv,style="Tool.TButton").grid(row=0, column=10, padx=(12,2))
        ttk.Button(head,text="Stop CSV",command=self.stop_csv,style="Tool.TButton").grid(row=0, column=11)

        # durum barı
        bar = ttk.Frame(self, style="Card.TLabelframe")
        bar.grid(row=1, column=0, sticky="ew", padx=8, pady=(4,6))
        bar.grid_columnconfigure(1, weight=1)
        ttk.Label(bar, textvariable=self.status_text).grid(row=0, column=0, padx=(10,4), pady=6, sticky="w")
        rightwrap = ttk.Frame(bar)
        rightwrap.grid(row=0, column=1, padx=10, sticky="e")
        ttk.Label(rightwrap, textvariable=self.oled_state).pack(side="right", padx=(10,0))
        ttk.Label(rightwrap, textvariable=self.fan_mode).pack(side="right")

        # NTC threshold
        thrf=ttk.Frame(self, style="Card.TLabelframe")
        thrf.grid(row=2, column=0, sticky="ew", padx=8, pady=(0,6))
        ttk.Label(thrf, text="NTC Threshold (°C):").pack(side='left', padx=(12,6), pady=6)
        thr_sp = ttk.Spinbox(thrf, from_=20.0, to=120.0, increment=0.5,
                             textvariable=self.ntc_threshold_var, width=6,
                             command=self._on_threshold_change)
        thr_sp.pack(side='left')
        ttk.Label(thrf, text="Soft green ≤ threshold < soft red").pack(side='left', padx=10)

        # ana grid
        main = ttk.Frame(self, style="Root.TFrame")
        main.grid(row=3, column=0, sticky="nsew", padx=8, pady=(0,8))
        self.grid_rowconfigure(3, weight=1)
        main.grid_columnconfigure(0, weight=1)
        main.grid_columnconfigure(1, weight=1)
        main.grid_columnconfigure(2, weight=1)

        # NTC kartı
        ntcf=ttk.LabelFrame(main,text="NTC Temperatures (20)",style='Card.TLabelframe')
        ntcf.grid(row=0, column=0, sticky="nsew", padx=(0,6), pady=0)
        ntcf.grid_columnconfigure(tuple(range(5)), weight=1)
        self.ntc_tiles=[]; cols=5
        for i in range(20):
            f=tk.Frame(ntcf,bd=1,relief='solid',width=180,height=100,bg="#ffffff")
            r=i//cols; c=i%cols
            f.grid(row=r,column=c,padx=4,pady=4,sticky="nsew")
            f.grid_propagate(False)
            l1=tk.Label(f,text=f"N{i}",font=("Segoe UI",9,"bold"),bg=f["bg"],fg="#2563eb", anchor="w")
            l1.pack(anchor='nw',padx=6,pady=(6,0))
            l2=tk.Label(f,text="0.0 °C",font=("Segoe UI",12,"bold"),bg=f["bg"],fg="#111827",
                        width=8, anchor="center")
            l2.pack(expand=True)
            self.ntc_tiles.append((f,l1,l2))
        mm=ttk.Frame(ntcf)
        mm.grid(row=4, column=0, columnspan=5, sticky="ew", pady=(2,6))
        self.min_var=tk.StringVar(value="Min: -")
        self.max_var=tk.StringVar(value="Max: -")
        ttk.Label(mm,textvariable=self.min_var).pack(side='left',padx=8)
        ttk.Label(mm,textvariable=self.max_var).pack(side='left',padx=8)
        readbtn = ttk.Button(mm,text="Read Temps",command=self.read_temps,style="Tool.TButton")
        readbtn.pack(side='right', padx=8)

        # Röle kartı
        rfrm=ttk.LabelFrame(main,text="Relays (Channel View)",style='Card.TLabelframe')
        rfrm.grid(row=0, column=1, sticky="nsew", padx=6, pady=0)
        rfrm.grid_columnconfigure(0, weight=1)
        rfrm.grid_columnconfigure(1, weight=1)

        left =ttk.LabelFrame(rfrm,text="Static (ch 1..20 → rid=2*k)",style='Card.TLabelframe')
        left.grid(row=0, column=0, sticky="nsew", padx=(8,4), pady=8)
        for c in range(5): left.grid_columnconfigure(c, weight=1)

        right=ttk.LabelFrame(rfrm,text="Electronic (ch 1..20 → rid=2*k-1) — single-select",style='Card.TLabelframe')
        right.grid(row=0, column=1, sticky="nsew", padx=(4,8), pady=8)
        for c in range(5): right.grid_columnconfigure(c, weight=1)

        opt=ttk.Frame(rfrm)
        opt.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=(0,10))
        self.relay_btns=[None]*40

        for kch in range(1,21):
            rid = 2 * kch
            idx = rid - 1
            b=tk.Button(left,text=str(kch),width=4,height=2,bg='red',
                        activebackground='#14532d', fg='white',
                        command=lambda ch=kch:self.on_click_static_channel(ch))
            b.grid(row=(kch-1)//5, column=(kch-1)%5, padx=3, pady=3)
            self.relay_btns[idx]=b

        for kch in range(1,21):
            rid = 2 * kch - 1
            idx = rid - 1
            b=tk.Button(right,text=str(kch),width=4,height=2,bg='red',
                        activebackground='#14532d', fg='white',
                        command=lambda ch=kch:self.on_click_electronic_channel(ch))
            b.grid(row=(kch-1)//5, column=(kch-1)%5, padx=3, pady=3)
            self.relay_btns[idx]=b

        ttk.Button(opt,text="All OFF",style='Danger.TButton',command=self.all_off).pack(side='left', padx=(4,6))
        ttk.Button(opt,text="All ON (static)",style='Accent.TButton',command=self.all_on).pack(side='left')

        rule_lbl = ttk.Label(opt, text="Note: In electronic–static pairs, only one electronic channel may be ON at a time.")
        rule_lbl.pack(side='right', padx=8)

        # Sağ kolon: Fan/OLED + Monitor
        rightcol = ttk.Frame(main, style="Root.TFrame")
        rightcol.grid(row=0, column=2, sticky="nsew", padx=(6,0))
        rightcol.grid_rowconfigure(1, weight=1)

        fanf=ttk.LabelFrame(rightcol,text="Fans (synced)",style='Card.TLabelframe')
        fanf.grid(row=0, column=0, sticky="ew", pady=(0,6))
        self.fan_vars=[]; self.fan_cbs=[]
        for i in range(3):
            row=ttk.Frame(fanf); row.pack(fill='x',padx=10,pady=4)
            ttk.Label(row,text=f"Fan {i+1}:",width=10).pack(side='left')
            var=tk.StringVar(value="AUTO")
            cb=ttk.Combobox(row,textvariable=var,state='readonly',width=7,values=FAN_LABELS)
            cb.pack(side='left'); cb.bind('<<ComboboxSelected>>', lambda e,ii=i:self.on_fan_select(ii))
            self.fan_vars.append(var); self.fan_cbs.append(cb)
        ttk.Button(fanf,text="Set AUTO",command=self.set_fans_auto_startup,style="Tool.TButton").pack(padx=10, pady=(0,8), anchor="w")

        oledf=ttk.LabelFrame(rightcol,text="OLED Screen",style='Card.TLabelframe')
        oledf.grid(row=1, column=0, sticky="ew", pady=(0,6))
        btnrow = ttk.Frame(oledf)
        btnrow.pack(fill='x', padx=10, pady=6)
        b_sleep = ttk.Button(btnrow,text="Sleep",command=self.oled_sleep,style="Tool.TButton")
        b_sleep.pack(side='left', padx=(0,8))
        b_wake  = ttk.Button(btnrow,text="Wake", command=self.oled_wake, style="Tool.TButton")
        b_wake.pack(side='left')

        mon=ttk.LabelFrame(rightcol,text="HEX & Logs",style='Card.TLabelframe')
        mon.grid(row=2, column=0, sticky="nsew")
        lab=ttk.Frame(mon); lab.pack(fill='x', padx=8, pady=(6,0))
        ttk.Label(lab,textvariable=self.last_tx).pack(side='left',padx=(2,8))
        ttk.Label(lab,textvariable=self.last_rx).pack(side='left',padx=(2,8))
        self.hextext=tk.Text(mon,height=10,bg="#f9fafb",fg="#111827",insertbackground="#111827",bd=1, highlightthickness=0)
        self.hextext.pack(fill='both',expand=True,padx=8,pady=(4,6))
        self.logtext=tk.Text(mon,height=8,bg="#f9fafb",fg="#111827",insertbackground="#111827",bd=1, highlightthickness=0)
        self.logtext.pack(fill='both',expand=True,padx=8,pady=(0,8))
        self.logtext.tag_configure("error", foreground="#b91c1c")

        foot = ttk.Frame(self, style="Status.TFrame")
        foot.grid(row=4, column=0, sticky="ew")
        ttk.Label(foot, text="Electronic–static pairs: only one electronic channel may be ON.", style="Status.TLabel").pack(side='right', padx=10, pady=4)

        self.grid_columnconfigure(0, weight=1)

    # helpers
    def _addr(self)->int:
        try: return int(self.addr_var.get()) & 0xFF
        except: return self.default_addr & 0xFF

    def _paint(self, idx:int):
        self.relay_btns[idx].configure(bg=('green' if self.relays[idx] else 'red'))

    def _on_threshold_change(self):
        try:
            thr = float(self.ntc_threshold_var.get())
        except Exception:
            thr = 40.0
            self.ntc_threshold_var.set(thr)
        self.NTC_LIMIT = int(round(thr * 10.0))
        self._ntc_dirty = set(range(20))
        self._schedule_ntc_flush()

    def _repaint_all_ntc_tiles(self):
        for i, val in enumerate(self.ntc_vals):
            self._set_ntc_tile(i, val)

    def refresh_ports(self):
        ports=[p.device for p in list_ports.comports()]
        if ports:
            self.port_cb['values']=ports
            if not self.app.same_port_var.get():
                self.btn_connect.configure(state='normal')
            self.log(f"[PORT] {len(ports)} port: {', '.join(ports)}")
        else:
            self.port_cb['values'] = ["(no ports)"]
            self.port_cb.set("(no ports)")
            if not self.app.same_port_var.get():
                self.btn_connect.configure(state='disabled')
            self.log("[PORT] 0 port bulundu")

    def connect(self):
        if self.app.same_port_var.get():
            return  # Same-COM modunda bireysel bağlanma yok

        port=self.port_var.get().strip()
        if not port or port=="(no ports)":
            messagebox.showerror('Port','Select a port first')
            return

        try:
            if self.client:
                self.client.close()
        except:
            pass

        self.btn_connect.configure(state='disabled')
        self.btn_disconnect.configure(state='normal')

        def worker():
            try:
                self.client=SerialClient(
                    port, BAUD, self._addr(),
                    self._rx_bin_thread, self._rx_line_thread,
                    self._log_thread, self._hex_thread
                )
                self.client.open()
                self.after(0, self.set_fans_auto_startup)
                self.after(0, self.oled_wake)
                self._log_thread("[INFO] Connected.")
                self.set_status("OK")
            except Exception as e:
                self.after(0, lambda: (self.btn_connect.configure(state='normal'),
                                        self.btn_disconnect.configure(state='disabled'),
                                        messagebox.showerror('Connect', f'{e}')))
                self.set_status(f"Error: {e}")
        threading.Thread(target=worker,daemon=True).start()

    def disconnect(self):
        if self.app.same_port_var.get():
            return  # Same-COM modunda bireysel disconnect yok
        try:
            if self.client:
                self.client.close()
        except:
            pass
        self.client = None
        self.btn_disconnect.configure(state='disabled')
        self.btn_connect.configure(state='normal')
        self.set_status("Disconnected")

    def send_and_log(self, frame:bytes, note:str):
        if self.app.same_port_var.get():
            self.app.hub.send_from_panel(self, frame, note)
        else:
            port=self.client.port if (self.client and self.client.port) else 'DISCONNECTED'
            if not (self.client and self.client.is_open()):
                self.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; NOT SENT")
                return
            self.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; {note}")
            self.client.send(frame)

    # ---- FAN: startup AUTO helper ----
    def set_fans_auto_startup(self):
        for v in self.fan_vars:
            v.set("AUTO")
        frame = pack(self._addr(), CMD_FAN, bytes([FAN_SUBMAP["AUTO"]]))
        self.send_and_log(frame, "FAN -> AUTO (startup)")

    # ---- OLED controls (debounce) ----
    def _oled_can_fire(self)->bool:
        now_ms = int(time.time()*1000)
        if now_ms - self._oled_last_ms < OLED_DEBOUNCE_MS:
            self.set_status("Warning: OLED command ignored (debounce)")
            return False
        self._oled_last_ms = now_ms
        return True

    def oled_sleep(self):
        if not self._oled_can_fire(): return
        frame = pack(self._addr(), CMD_OLED, bytes([SUB_OLED_SLEEP]))
        self.send_and_log(frame, "OLED -> SLEEP")

    def oled_wake(self):
        if not self._oled_can_fire(): return
        frame = pack(self._addr(), CMD_OLED, bytes([SUB_OLED_WAKE]))
        self.send_and_log(frame, "OLED -> WAKE")

    # ---- Röle Actions ----
    def on_click_static_channel(self, ch:int):
        rid = 2 * ch
        idx = rid - 1
        want_on = (self.relays[idx] == 0)
        self.relays[idx] = 1 if want_on else 0
        self._paint(idx)
        data=bytes([SUB_RLY_ON if want_on else SUB_RLY_OFF, rid])
        frame=pack(self._addr(), CMD_RLY, data)
        self.send_and_log(frame, f"Static ch={ch} (rid={rid}) -> {'ON' if want_on else 'OFF'}")

    def on_click_electronic_channel(self, ch:int):
        rid = 2 * ch - 1
        idx = rid - 1
        want_on = (self.relays[idx] == 0)
        if want_on:
            for r in range(1,41,2):
                j = r - 1
                if j != idx and self.relays[j] != 0:
                    self.relays[j] = 0
                    self._paint(j)
        self.relays[idx] = 1 if want_on else 0
        self._paint(idx)
        data=bytes([SUB_RLY_ON if want_on else SUB_RLY_OFF, rid])
        frame=pack(self._addr(), CMD_RLY, data)
        self.send_and_log(frame, f"Electronic ch={ch} (rid={rid}) -> {'ON' if want_on else 'OFF'}")

    def all_off(self):
        frame=pack(self._addr(), CMD_RLY, bytes([SUB_RLY_OFF, 0x00]))
        for k in range(40):
            if self.relays[k]!=0:
                self.relays[k]=0
                self._paint(k)
        self.send_and_log(frame, "ALL OFF")

    def _schedule_frames_spaced(self, frames_notes: List[tuple], gap_ms:int=15):
        """Komutları gap_ms aralıklarla sırayla gönder."""
        def send_next(i:int):
            if i>=len(frames_notes): 
                return
            frame, note = frames_notes[i]
            self.send_and_log(frame, note)
            self.after(gap_ms, lambda: send_next(i+1))
        send_next(0)

    def all_on(self):
        # 1) Tüm STATIC (rid=2*k) ON — aralıklı gönder
        frames=[]
        for ch in range(1,21):
            rid = 2 * ch
            idx = rid - 1
            if self.relays[idx] != 1:
                self.relays[idx] = 1
                self._paint(idx)
            frames.append( (pack(self._addr(), CMD_RLY, bytes([SUB_RLY_ON, rid])),
                           f"ALL ON (static) -> ch={ch} (rid={rid})") )

        # 2) Tüm ELECTRONIC (rid tek) OFF — aralıklı gönder (statikleri etkilemesin)
        for r in range(1,41,2):
            j=r-1
            if self.relays[j]!=0:
                self.relays[j]=0
                self._paint(j)
            frames.append( (pack(self._addr(), CMD_RLY, bytes([SUB_RLY_OFF, r])),
                           f"electronic rid={r} -> OFF") )

        self._schedule_frames_spaced(frames, gap_ms=15)

    # ---- Fan (dropdown; senkron)
    def on_fan_select(self, _idx:int):
        label=self.fan_vars[_idx].get()
        sub=FAN_SUBMAP.get(label, 0)
        for v in self.fan_vars: v.set(label)
        pct = 0 if sub==0 else 25 if sub==1 else 50 if sub==2 else 75 if sub==3 else 100 if sub==4 else self.fans[0]
        self.fans=[pct,pct,pct]
        frame=pack(self._addr(), CMD_FAN, bytes([sub]))
        self.send_and_log(frame, f"FAN -> {label}")

    # ---- Sıcaklık
    def read_temps(self):
        self.send_and_log(pack(self._addr(), CMD_TMP, bytes([SUB_TMP_ALL])), "TEMP ALL")

    def _update_ntc_value(self, idx:int, val_decic:int):
        self.ntc_vals[idx] = int(val_decic) * 10
        self.ntc_seen[idx] = True
        self._ntc_dirty.add(idx)
        self._schedule_ntc_flush()

    def _schedule_ntc_flush(self):
        if self._ntc_flush_job is None:
            self._ntc_flush_job = self.after(50, self._flush_ntc_ui)

    def _flush_ntc_ui(self):
        self._ntc_flush_job = None
        if not self._ntc_dirty:
            return
        dirty = list(self._ntc_dirty)
        self._ntc_dirty.clear()
        for i in dirty:
            self._set_ntc_tile(i, self.ntc_vals[i], only_if_changed=True)
        self._recompute_minmax()

    def _recompute_minmax(self):
        indices = [i for i, s in enumerate(self.ntc_seen) if s]
        if not indices:
            self.min_var.set("Min: -")
            self.max_var.set("Max: -")
            return
        min_idx = min(indices, key=lambda i: self.ntc_vals[i])
        max_idx = max(indices, key=lambda i: self.ntc_vals[i])
        self.ntc_min = self.ntc_vals[min_idx]
        self.ntc_max = self.ntc_vals[max_idx]
        self.min_var.set(f"Min: {self.ntc_min/10.0:.1f} °C (N{min_idx})")
        self.max_var.set(f"Max: {self.ntc_max/10.0:.1f} °C (N{max_idx})")

    # ---- CSV (periyodik) ----
    def start_csv(self):
        if self.csv_enabled:
            messagebox.showinfo('CSV','Already logging'); 
            return
        path=filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV','*.csv')], initialfile=f"drawer{self.idx+1}_log.csv")
        if not path: return
        try:
            self.csv_file=open(path,'w',newline='',encoding='utf-8')
        except Exception as e:
            messagebox.showerror("CSV", f"File open error: {e}")
            return
        self.csv_writer=csv.writer(self.csv_file)
        header=['ts']+[f"R{i+1}" for i in range(40)]+[f"N{i}" for i in range(20)]+['NTC_MIN','NTC_MAX']+['FAN1','FAN2','FAN3','FAN_MODE','OLED']
        self.csv_writer.writerow(header)
        self.csv_enabled=True
        self.log(f"[CSV] start {path} (interval={self.csv_interval_var.get()} s)")
        self._schedule_csv_next()

    def _schedule_csv_next(self):
        if self._csv_job is not None:
            try: self.after_cancel(self._csv_job)
            except: pass
            self._csv_job = None
        if not self.csv_enabled: 
            return
        self._csv_snapshot_write()
        interval_ms = max(1, int(self.csv_interval_var.get())) * 1000
        self._csv_job = self.after(interval_ms, self._schedule_csv_next)

    def _csv_snapshot_write(self):
        if not (self.csv_enabled and self.csv_writer): 
            return
        ts=time.strftime('%Y-%m-%d %H:%M:%S')
        fan_mode = self.fan_mode.get().split(':',1)[-1].strip()
        oled_mode = self.oled_state.get().split(':',1)[-1].strip()
        row=[ts]+self.relays[:]+[v/10.0 for v in self.ntc_vals]+[self.ntc_min/10.0,self.ntc_max/10.0]+self.fans[:]+[fan_mode, oled_mode]
        try:
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            self.log_error(f"[CSV] write error: {e}")

    def stop_csv(self):
        if self._csv_job is not None:
            try: self.after_cancel(self._csv_job)
            except: pass
            self._csv_job = None
        if self.csv_enabled and self.csv_file:
            try: self.csv_file.close()
            except: pass
        self.csv_file=None; self.csv_writer=None; self.csv_enabled=False
        self.log("[CSV] stop")

    # ---- RX handlers ----
    def _rx_bin_thread(self, addr:int, cmd:int, sub:int, a1:int, a2:int, data:bytes):
        self.after(0, lambda: self.on_frame_bin(addr, cmd, sub, a1, a2, data))

    def _rx_line_thread(self, line:str):
        self.after(0, lambda: self.on_line_text(line))

    def _log_thread(self, s:str):
        self.after(0, lambda: self.log(s))

    def _hex_thread(self, s:str):
        self.after(0, lambda: self.hex_log(s))

    def on_frame_bin(self, addr:int, cmd:int, sub:int, a1:int, a2:int, data:bytes):
        if cmd==CMD_ADDR and len(data)>=2:
            val=data[1]
            self.addr_var.set(val)
            self.log(f"[ADDR] device reports address=0x{val:02X}")
            return

        if cmd==CMD_EVT:
            # System Reset: AA .. 50 02 00 E8 .. 55 → sub=0x00, a1=0xE8
            if sub == 0x00 and a1 == 0xE8:
                self.log("[EVT] System Reset")
                for i in range(40):
                    if self.relays[i]!=0:
                        self.relays[i]=0
                        self._paint(i)
                self.set_status("Warning: system reset")
                return

            msg = SYS_STATUS_MAP.get(a1)
            if msg:
                self.log_error(f"[EVT] 0x{a1:02X} → {msg}")
            else:
                for i in range(40):
                    if self.relays[i]!=0:
                        self.relays[i]=0
                        self._paint(i)
                self.set_status("Warning: system event (init/reset?)")
                self.log("[EVT] System event received (init/reset?)")
            return

        if addr != (self._addr() & 0xFF):
            return

        if cmd==CMD_RLY:
            if sub==SUB_RLY_ERR:
                self.log_error(f"[RLY][ERR] Possible Holt/relay fault (arg1=0x{a1:02X})")
                return
            if len(data)>=2 and 1<=data[0]<=40 and data[1] in (0,1):
                rid, state=data[0], data[1]
                self.relays[rid-1]=state; self._paint(rid-1)
                ch = (rid//2) if (rid%2==0) else ((rid+1)//2)
                side = "Static" if (rid%2==0) else "Electronic"
                self.set_status(f"OK: Relay {side} ch={ch} -> {state}")
                self.log(f"[RLY] {side} ch={ch} (rid={rid}) -> {state}")
            else:
                self.log(f"[RLY] ack sub=0x{sub:02X} a1=0x{a1:02X} a2=0x{a2:02X}")

        elif cmd==CMD_FAN:
            st=a1
            if st==FAN_STATUS_MANUAL_OK:
                self.fan_mode.set("Fan Mode: MANUAL"); self.set_status("OK: Fan Manual"); self.log("[FAN] Manual set OK")
            elif st==FAN_STATUS_AUTO_OK:
                self.fan_mode.set("Fan Mode: AUTO");   self.set_status("OK: Fan Auto");   self.log("[FAN] Auto set OK (25% start; >20→50, >30→75, >40→100)")
            else:
                self.set_status(f"Warning: Fan status 0x{st:02X}")
                self.log(f"[FAN] status=Unknown (0x{st:02X})")

        elif cmd==CMD_TMP:
            if sub in (SUB_TMP_MIN, SUB_TMP_MAX):
                return
            idx = sub
            val_decic = a1
            if 0 <= idx < 20:
                self._update_ntc_value(idx, val_decic)

        elif cmd==CMD_OLED:
            if a1==0x00:
                if sub==SUB_OLED_SLEEP:
                    self.oled_state.set("OLED: SLEEP"); self.set_status("OK: OLED Sleep"); self.log("[OLED] Sleep OK")
                elif sub==SUB_OLED_WAKE:
                    self.oled_state.set("OLED: WAKE");  self.set_status("OK: OLED Wake");  self.log("[OLED] Wake OK")
                else:
                    self.set_status("OK: OLED Ack (unknown sub)"); self.log("[OLED] Ack OK (unknown sub)")
            else:
                self.log_error(f"[OLED] NACK status=0x{a1:02X}")

        else:
            self.log(f"[UNK] cmd=0x{cmd:02X} sub=0x{sub:02X} a1=0x{a1:02X} a2=0x{a2:02X}")

    # ASCII yedek (opsiyonel)
    _re_ntc=re.compile(r"NTC\[(\d+)\]\s*=\s*([+-]?\d+(?:\.\d+)?)\s*C", re.I)

    def on_line_text(self,line:str):
        self.log(line)
        m=self._re_ntc.search(line)
        if m:
            i=int(m.group(1)); val=float(m.group(2))
            if 0<=i<20:
                self.ntc_seen[i]=True
                self.ntc_vals[i]=int(round(val*10))
                self._ntc_dirty.add(i)
                self._schedule_ntc_flush()

    def _set_ntc_tile(self,i:int, val:int, only_if_changed:bool=False):
        f,l1,l2=self.ntc_tiles[i]
        deg_text = f"{val/10.0:.1f} °C"
        if (not only_if_changed) or (l2.cget("text") != deg_text):
            l2.configure(text=deg_text)
        hot=(val>self.NTC_LIMIT)
        want_bg = "#fde2e2" if hot else "#dcfce7"
        if (not only_if_changed) or (f.cget("bg") != want_bg):
            f.configure(bg=want_bg)
            if l1.cget("bg") != want_bg: l1.configure(bg=want_bg)
            if l2.cget("bg") != want_bg: l2.configure(bg=want_bg)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('STM Relay/NTC/FAN/OLED GUI — CRC8')
        self.geometry('1320x940')
        self.minsize(1100, 800)
        self['background']="#f4f5f7"

        self.hub = SerialHub(self)
        self.same_port_var = tk.BooleanVar(value=False)
        self.closing = False

        top=ttk.Frame(self, style="Header.TFrame"); top.pack(fill='x',pady=(6,4))
        ttk.Button(top,text='Global Scan Ports',command=self.scan_ports,style="Tool.TButton").pack(side='left',padx=8)

        self.same_chk = ttk.Checkbutton(top, text="Same COM for all drawers", variable=self.same_port_var,
                                        command=self.on_same_toggle)
        self.same_chk.pack(side='left', padx=8)

        self.btn_shared_connect = ttk.Button(top, text='Connect (Shared)', command=self.shared_connect, style="Accent.TButton", state='disabled')
        self.btn_shared_disconnect = ttk.Button(top, text='Disconnect (Shared)', command=self.shared_disconnect, style="Tool.TButton", state='disabled')
        self.btn_shared_connect.pack(side='left', padx=(12,4))
        self.btn_shared_disconnect.pack(side='left')

        ttk.Label(top,text="MEHA",style='Header.TLabel').pack(side='right',padx=10,pady=4)

        self.nb=ttk.Notebook(self); self.nb.pack(fill='both',expand=True, padx=6, pady=(0,6))
        self.panels: List[DevicePanel] = []
        for i in range(5):
            container = ttk.Frame(self, style="Root.TFrame")
            p=DevicePanel(container,i,self)
            p.pack(fill='both', expand=True)
            self.nb.add(container,text=f"Drawer {i+1}")
            self.panels.append(p)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(200,self.scan_ports)

    # ---- Same COM toggling / port syncing ----
    def on_same_toggle(self):
        on = self.same_port_var.get()
        if on:
            # bireysel bağlantıları kapat, portu boşa çıkar
            for p in self.panels:
                try:
                    if p.client and p.client.is_open():
                        p.client.close()
                except: pass
                p.client = None
                p.set_shared_mode(True)
            shared_port = self.panels[0].port_var.get()
            if shared_port:
                for p in self.panels:
                    p.port_var.set(shared_port)
            self.btn_shared_connect.configure(state='normal')
            self.btn_shared_disconnect.configure(state='normal')
        else:
            self.shared_disconnect()
            for p in self.panels:
                p.set_shared_mode(False)
            self.btn_shared_connect.configure(state='disabled')
            self.btn_shared_disconnect.configure(state='disabled')

    def on_panel_port_changed(self, panel:DevicePanel):
        if self.same_port_var.get():
            val = panel.port_var.get()
            for p in self.panels:
                p.port_var.set(val)

    def shared_connect(self):
        if not self.same_port_var.get():
            messagebox.showinfo("Info", "Enable 'Same COM for all drawers' first.")
            return
        # tüm bireysel bağlantıları garanti kapat
        for p in self.panels:
            try:
                if p.client and p.client.is_open():
                    p.client.close()
            except: pass
            p.client = None

        port = self.panels[0].port_var.get().strip()
        if not port or port=="(no ports)":
            messagebox.showerror('Port','Select a port first')
            return
        try:
            self.hub.open(port)
            for p in self.panels:
                try:
                    p.set_fans_auto_startup(); p.oled_wake()
                    p.log("[INFO] Shared Connected."); p.set_status("OK (shared)")
                except:
                    pass
        except Exception as e:
            messagebox.showerror('Connect (Shared)', f'{e}')

    def shared_disconnect(self):
        try:
            self.hub.close()
        except:
            pass
        for p in self.panels:
            p.set_status("Disconnected (shared)")
            p.log("[INFO] Shared Disconnected.")

    # ---- Window close ----
    def _on_close(self):
        self.closing = True
        try:
            self.hub.close()
        except:
            pass
        for p in getattr(self, "panels", []):
            try:
                p.stop_csv()
                if p.client and p.client.is_open():
                    p.client.close()
            except:
                pass
        try:
            self.update_idletasks()
        except:
            pass
        self.destroy()

    # ---- Scanning ----
    def scan_ports(self):
        ports=[p.device for p in list_ports.comports()]
        for p in self.panels:
            if ports:
                p.port_cb['values']=ports
                if not self.same_port_var.get():
                    p.btn_connect.configure(state='normal')
            else:
                p.port_cb['values'] = ["(no ports)"]
                p.port_cb.set("(no ports)")
                if not self.same_port_var.get():
                    p.btn_connect.configure(state='disabled')
        if ports:
            self.panels[0].log(f"[PORT] global scan: {len(ports)} port: {', '.join(ports)}")
        else:
            self.panels[0].log("[PORT] global scan: 0 port")

if __name__=='__main__':
    App().mainloop()