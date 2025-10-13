from __future__ import annotations
import time, threading, csv, re
from typing import Optional, Set

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
SUB_TMP_ONE = 0x03

# Fan (GUI'de 1–5 görünür, MCU'ya 0–4 gönderilir; AUTO opsiyonel)
FAN_LABELS = ["1", "2", "3", "4", "5", "AUTO"]
# GUI etiketi -> MCU sub değer eşlemesi (manuel: 0..4, AUTO: 5)
FAN_SUBMAP = {"1":0, "2":1, "3":2, "4":3, "5":4, "AUTO":5}
FAN_STATUS_MANUAL_OK = 0xF1
FAN_STATUS_AUTO_OK   = 0xF2

# OLED
SUB_OLED_SLEEP = 0x00
SUB_OLED_WAKE  = 0x01
OLED_ADDR = 0x60
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

    def log(self,s): 
        if self.log_cb: self.log_cb(s)
    def hexlog(self,s): 
        if self.hex_cb: self.hex_cb(s)

    def open(self):
        if not self.port: raise RuntimeError("Select a port first")
        # eski bağlantıyı temizle
        try:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer(); self.ser.reset_output_buffer(); self.ser.close()
        except Exception:
            pass
        self.ser=serial.Serial(self.port,self.baud,timeout=0.05,write_timeout=0.2)
        self._stop.clear()
        self._th=threading.Thread(target=self._loop,daemon=True); self._th.start()
        self.log(f"[OPEN] {self.port}@{self.baud} (Addr={self.device_addr})")

    def close(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                self.ser.close()
        except Exception:
            pass
        self.log("[CLOSE]")

    def is_open(self)->bool: 
        return bool(self.ser and self.ser.is_open)

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
            try: 
                if self.rx_line_cb: self.rx_line_cb(line)
            except Exception as e: 
                self.log(f"[TEXT CB ERR] {e}")

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
            if self.rx_cb: self.rx_cb(addr, cmd, sub, a1, a2, data)
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

# ------------------ GUI ------------------
class DevicePanel(ttk.Frame):
    MAX_HEX_LINES = 1800
    KEEP_HEX_LINES = 1200
    MAX_LOG_LINES = 1500
    KEEP_LOG_LINES = 900

    def __init__(self, master, idx:int):
        super().__init__(master)
        self.idx=idx; self.default_addr=1  # 1..5 aralığı
        self.client: Optional[SerialClient]=None

        self.port_var=tk.StringVar(value="")
        self.addr_var=tk.IntVar(value=self.default_addr)

        self.last_tx=tk.StringVar(value="Last TX: -")
        self.last_rx=tk.StringVar(value="Last RX: -")

        self.status_text=tk.StringVar(value="Status: Idle")
        self.fan_mode=tk.StringVar(value="Fan Mode: MANUAL")
        self.oled_state=tk.StringVar(value="OLED: WAKE")

        # --- Durumlar ---
        self.relays=[0]*20            # 20 röle (R1..R20)
        self.ntc_vals=[0]*20          # °C (tamsayı)
        self.ntc_min=None; self.ntc_max=None
        self.fans=[0,0,0]             # görsel yüzde

        # GUI-otomatik fan seviyesi (1..5), flooding önlemek için son gönderilen
        self._last_gui_fan_level = None

        self._oled_last_ms = 0

        self.csv_enabled=False; self.csv_file=None; self.csv_writer=None

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
        if tag:
            self.logtext.insert('end', s+"\n", (tag,))
        else:
            self.logtext.insert('end', s+"\n")
        self.logtext.see('end')
        self._trim_text_lines(self.logtext, self.MAX_LOG_LINES, self.KEEP_LOG_LINES)

    def log_error(self, s:str):
        self.set_status("Error: " + s)
        self.log(s, tag="error")

    def set_status(self, text:str):
        self.status_text.set("Status: " + text)

    # ---------- UI ----------
    def _build_ui(self):
        # üst çubuk
        head = ttk.Frame(self, style="Header.TFrame")
        head.grid(row=0, column=0, sticky="ew", padx=8, pady=(8,4))
        head.grid_columnconfigure(10, weight=1)

        ttk.Label(head,text=f"Drawer {self.idx+1}",style='Header.TLabel').grid(row=0, column=0, padx=(6,12))
        ttk.Label(head,text="Port:",style='Header.TLabel').grid(row=0, column=1, sticky="e")
        self.port_cb=ttk.Combobox(head,textvariable=self.port_var,width=18,state='readonly')
        self.port_cb.grid(row=0, column=2, padx=(6,10))
        ttk.Label(head,text="Addr:",style='Header.TLabel').grid(row=0, column=3, sticky="e")
        # 1..5 aralığı
        self.addr_sp=ttk.Spinbox(head,from_=1,to=5,increment=1,textvariable=self.addr_var,width=6)
        self.addr_sp.grid(row=0, column=4, padx=(6,10))
        ttk.Button(head,text="Scan",command=self.refresh_ports,style="Tool.TButton").grid(row=0, column=5)
        self.btn_connect = ttk.Button(head,text="Connect",command=self.connect,style="Accent.TButton")
        self.btn_connect.grid(row=0, column=6, padx=(8,4))
        self.btn_disconnect = ttk.Button(head,text="Disconnect",command=self.disconnect, state='disabled',style="Tool.TButton")
        self.btn_disconnect.grid(row=0, column=7)
        ttk.Button(head,text="Start CSV",command=self.start_csv,style="Tool.TButton").grid(row=0, column=8, padx=(12,2))
        ttk.Button(head,text="Stop CSV",command=self.stop_csv,style="Tool.TButton").grid(row=0, column=9)

        # durum barı
        bar = ttk.Frame(self, style="Card.TLabelframe")
        bar.grid(row=1, column=0, sticky="ew", padx=8, pady=(4,6))
        bar.grid_columnconfigure(1, weight=1)
        ttk.Label(bar, textvariable=self.status_text).grid(row=0, column=0, padx=(10,4), pady=6, sticky="w")
        rightwrap = ttk.Frame(bar)
        rightwrap.grid(row=0, column=1, padx=10, sticky="e")
        ttk.Label(rightwrap, textvariable=self.oled_state).pack(side="right", padx=(10,0))
        ttk.Label(rightwrap, textvariable=self.fan_mode).pack(side="right")

        # ana grid
        main = ttk.Frame(self, style="Root.TFrame")
        main.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0,8))
        self.grid_rowconfigure(2, weight=1)
        main.grid_columnconfigure(0, weight=1)
        main.grid_columnconfigure(1, weight=1)
        main.grid_columnconfigure(2, weight=1)

        # NTC kartı
        ntcf=ttk.LabelFrame(main,text="NTC Temperatures (N0..N19)",style='Card.TLabelframe')
        ntcf.grid(row=0, column=0, sticky="nsew", padx=(0,6), pady=0)
        ntcf.grid_columnconfigure(tuple(range(5)), weight=1)
        self.ntc_tiles=[]; cols=5
        for i in range(20):
            f=tk.Frame(ntcf,bd=1,relief='solid',width=180,height=100,bg="#ffffff")
            r=i//cols; c=i%cols
            f.grid(row=r,column=c,padx=4,pady=4,sticky="nsew")
            f.grid_propagate(False)
            l1=tk.Label(f,text=f"N{i}",font=("Segoe UI",9,"bold"),bg=f["bg"],fg="#2563eb")
            l1.pack(anchor='nw',padx=6,pady=(6,0))
            l2=tk.Label(f,text="0 °C",font=("Segoe UI",12,"bold"),bg=f["bg"],fg="#111827")
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

        # Röle kartı — 4x5 grid (R1..R20)
        rfrm=ttk.LabelFrame(main,text="Relays (R1..R20)",style='Card.TLabelframe')
        rfrm.grid(row=0, column=1, sticky="nsew", padx=6, pady=0)
        rfrm.grid_columnconfigure(tuple(range(5)), weight=1)
        self.relay_btns=[None]*20
        for i in range(20):
            rid = i+1
            b=tk.Button(rfrm,text=f"R{rid}",width=6,height=2,bg='red',
                        activebackground='#14532d', fg='white',
                        command=lambda rid=rid:self.on_click_relay(rid))
            r=i//5; c=i%5
            b.grid(row=r, column=c, padx=6, pady=6, sticky="nsew")
            self.relay_btns[i]=b
        opt=ttk.Frame(rfrm)
        opt.grid(row=4, column=0, columnspan=5, sticky="ew", padx=8, pady=(6,8))
        ttk.Button(opt,text="All OFF",style='Danger.TButton',command=self.all_off).pack(side='left', padx=(4,6))

        # Sağ kolon: Fan/OLED + Monitor
        rightcol = ttk.Frame(main, style="Root.TFrame")
        rightcol.grid(row=0, column=2, sticky="nsew", padx=(6,0))
        rightcol.grid_rowconfigure(2, weight=1)

        fanf=ttk.LabelFrame(rightcol,text="Fan (GUI auto by Max NTC)",style='Card.TLabelframe')
        fanf.grid(row=0, column=0, sticky="ew", pady=(0,6))
        self.fan_vars=[]; self.fan_cbs=[]
        row=ttk.Frame(fanf); row.pack(fill='x',padx=10,pady=4)
        ttk.Label(row,text=f"Level:",width=10).pack(side='left')
        var=tk.StringVar(value="2")
        cb=ttk.Combobox(row,textvariable=var,state='readonly',width=7,values=FAN_LABELS)
        cb.pack(side='left'); cb.bind('<<ComboboxSelected>>', lambda e:self.on_fan_select())
        self.fan_vars.append(var); self.fan_cbs.append(cb)
        ttk.Label(fanf, text=">40→5, >30→4, >20→3, aksi→2").pack(anchor='w', padx=10, pady=(0,6))

        oledf=ttk.LabelFrame(rightcol,text="OLED Screen (ADDR=0x60)",style='Card.TLabelframe')
        oledf.grid(row=1, column=0, sticky="ew", pady=(0,6))
        btnrow = ttk.Frame(oledf); btnrow.pack(fill='x', padx=10, pady=6)
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

        # alt durum barı
        foot = ttk.Frame(self, style="Status.TFrame")
        foot.grid(row=3, column=0, sticky="ew")
        ttk.Label(foot, text="Röle grid: 4×5 (R1..R20).", style="Status.TLabel").pack(side='right', padx=10, pady=4)

        self.grid_columnconfigure(0, weight=1)

    # helpers
    def _addr(self)->int:
        try: return int(self.addr_var.get()) & 0xFF
        except: return 1

    def _paint_relay(self, idx:int):
        self.relay_btns[idx].configure(bg=('green' if self.relays[idx] else 'red'))

    def refresh_ports(self):
        ports=[p.device for p in list_ports.comports()]
        if ports:
            self.port_cb['values']=ports
            self.btn_connect.configure(state='normal')
            self.log(f"[PORT] {len(ports)} port: {', '.join(ports)}")
        else:
            self.port_cb['values'] = ["(no ports)"]
            self.port_cb.set("(no ports)")
            self.btn_connect.configure(state='disabled')
            self.log("[PORT] 0 port bulundu")

    def connect(self):
        # önce varsa mevcut bağlantıyı kapat
        try:
            if self.client: self.client.close()
        except Exception:
            pass
        port=self.port_var.get().strip()
        if not port or port=="(no ports)":
            messagebox.showerror('Port','Select a port first'); return
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
                self._log_thread("[INFO] Connected.")
                self.set_status("OK")
            except Exception as e:
                self.after(0, lambda: (self.btn_connect.configure(state='normal'),
                                        self.btn_disconnect.configure(state='disabled'),
                                        messagebox.showerror('Connect', f'{e}')))
                self.set_status(f"Error: {e}")
        threading.Thread(target=worker,daemon=True).start()

    def disconnect(self):
        try:
            if self.client: self.client.close()
        except Exception:
            pass
        self.btn_disconnect.configure(state='disabled')
        self.btn_connect.configure(state='normal')
        self.set_status("Disconnected")

    def send_and_log(self, frame:bytes, note:str):
        port=self.client.port if (self.client and self.client.port) else 'DISCONNECTED'
        self.hex_log(f"TX[{port}] {hex_dump(frame)}  ;; {note}")
        if self.client: self.client.send(frame)

    # ---- FAN manuel seçim (combobox) ----
    def on_fan_select(self):
        label=self.fan_vars[0].get()
        sub=FAN_SUBMAP.get(label, 1)  # default 2→sub=1
        if label == "AUTO":
            frame=pack(self._addr(), CMD_FAN, bytes([5]))
            self.send_and_log(frame, "FAN -> AUTO (MCU)")
            self.fan_mode.set("Fan Mode: AUTO")
            return
        # manuel 1..5
        level = int(label)                       # 1..5
        self._last_gui_fan_level = level
        frame=pack(self._addr(), CMD_FAN, bytes([(level-1) & 0xFF]))
        self.send_and_log(frame, f"FAN -> level {level} (manual)")
        self.fan_mode.set("Fan Mode: MANUAL")

    # ---- Sıcaklık iste ----
    def read_temps(self):
        # Sadece ALL iste (flicker yok); min/max'ı GUI hesaplayacak
        self.send_and_log(pack(self._addr(), CMD_TMP, bytes([SUB_TMP_ALL])), "TEMP ALL")

    # ---- CSV
    def start_csv(self):
        if self.csv_enabled: messagebox.showinfo('CSV','Already logging'); return
        path=filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV','*.csv')], initialfile=f"drawer{self.idx+1}_log.csv")
        if not path: return
        self.csv_file=open(path,'w',newline='',encoding='utf-8')
        self.csv_writer=csv.writer(self.csv_file)
        header=['ts']+[f"R{i+1}" for i in range(20)]+[f"N{i}" for i in range(20)]+['NTC_MIN','NTC_MAX']+['FAN_LEVEL','FAN_MODE','OLED']
        self.csv_writer.writerow(header); self.csv_enabled=True; self.log(f"[CSV] start {path}")

    def stop_csv(self):
        if self.csv_enabled and self.csv_file:
            try: self.csv_file.close()
            except: pass
        self.csv_file=None; self.csv_writer=None; self.csv_enabled=False; self.log("[CSV] stop")

    def _csv_tick(self):
        if not self.csv_enabled or not self.csv_writer: return
        ts=time.strftime('%Y-%m-%d %H:%M:%S')
        fan_mode = self.fan_mode.get().split(':',1)[-1].strip()
        oled_mode = self.oled_state.get().split(':',1)[-1].strip()
        fan_lvl = self._last_gui_fan_level if self._last_gui_fan_level is not None else 0
        row=[ts]+self.relays[:]+self.ntc_vals[:]+[
            (self.ntc_min if self.ntc_min is not None else -1),
            (self.ntc_max if self.ntc_max is not None else -1),
            fan_lvl, fan_mode, oled_mode
        ]
        self.csv_writer.writerow(row)
        try: self.csv_file.flush()
        except: pass

    # ---- RX thread → UI thread köprüleri ----
    def _rx_bin_thread(self, addr:int, cmd:int, sub:int, a1:int, a2:int, data:bytes):
        self.after(0, lambda: self.on_frame_bin(addr, cmd, sub, a1, a2, data))

    def _rx_line_thread(self, line:str):
        self.after(0, lambda: self.on_line_text(line))

    def _log_thread(self, s:str):
        self.after(0, lambda: self.log(s))

    def _hex_thread(self, s:str):
        self.after(0, lambda: self.hex_log(s))

    # ---- FRAME handler ----
    def on_frame_bin(self, addr:int, cmd:int, sub:int, a1:int, a2:int, data:bytes):
        # Adres / event
        if cmd==CMD_ADDR and len(data)>=2:
            val=data[1]
            self.addr_var.set(val)
            self.log(f"[ADDR] device reports address=0x{val:02X}")
            return

        if cmd==CMD_EVT:
            msg = SYS_STATUS_MAP.get(a1)
            if msg:
                self.log_error(f"[EVT] 0x{a1:02X} → {msg}")
            else:
                for i in range(20):
                    if self.relays[i]!=0:
                        self.relays[i]=0
                        self._paint_relay(i)
                self.set_status("Warning: system event (reset?)")
                self.log("[EVT] System event received (init/reset?)")
            self._csv_tick()
            return

        # hedef adres değilse yoksay
        if addr != (self._addr() & 0xFF):
            return

        if cmd==CMD_RLY:
            # ack formatını basitçe işle
            if len(data)>=2 and 1<=data[0]<=20 and data[1] in (0,1):
                rid, state=data[0], data[1]
                self.relays[rid-1]=state; self._paint_relay(rid-1)
                self.set_status(f"OK: Relay R{rid} -> {state}")
                self.log(f"[RLY] R{rid} -> {state}")
            elif sub==SUB_RLY_ERR:
                self.log_error(f"[RLY][ERR] Possible Holt/relay fault (arg1=0x{a1:02X})")
            else:
                self.log(f"[RLY] ack sub=0x{sub:02X} a1=0x{a1:02X} a2=0x{a2:02X}")
            self._csv_tick()

        elif cmd==CMD_FAN:
            st=a1
            if st==FAN_STATUS_MANUAL_OK:
                self.fan_mode.set("Fan Mode: MANUAL"); self.set_status("OK: Fan Manual"); self.log("[FAN] Manual set OK")
            elif st==FAN_STATUS_AUTO_OK:
                self.fan_mode.set("Fan Mode: AUTO");   self.set_status("OK: Fan Auto");   self.log("[FAN] Auto set OK")
            else:
                self.set_status(f"Warning: Fan status 0x{st:02X}")
                self.log(f"[FAN] status=Unknown (0x{st:02X})")
            self._csv_tick()

        elif cmd==CMD_TMP:
            # RX: AA ADDR 30 02 II TT   (II: index 0..19, TT: °C uint8)
            idx=sub; val=a1
            if 0<=idx<20:
                # NTC6 ekranda +2 °C
                if idx == 6:
                    val = min(255, (val + 2) & 0xFF)
                self.ntc_vals[idx]=int(val)
                self._set_ntc_tile(idx, self.ntc_vals[idx])

                # Min/Max GUI'de hesapla
                seen = [v for v in self.ntc_vals if isinstance(v, int)]
                if seen:
                    self.ntc_min = min(seen)
                    self.ntc_max = max(seen)
                    self.min_var.set(f"Min: {self.ntc_min} °C")
                    self.max_var.set(f"Max: {self.ntc_max} °C")

                # GUI otomatik fan: max'a göre seviye
                self._gui_auto_fan_from_max()

            self._csv_tick()

        elif cmd==CMD_OLED:
            if a1==0x00:
                if sub==SUB_OLED_SLEEP:
                    self.oled_state.set("OLED: SLEEP"); self.set_status("OK: OLED Sleep"); self.log("[OLED] Sleep OK")
                elif sub==SUB_OLED_WAKE:
                    self.oled_state.set("OLED: WAKE");  self.set_status("OK: OLED Wake");  self.log("[OLED] Wake OK")
                else:
                    self.set_status("OK: OLED Ack"); self.log("[OLED] Ack OK")
            else:
                self.log_error(f"[OLED] NACK status=0x{a1:02X}")
            self._csv_tick()

        else:
            self.log(f"[UNK] cmd=0x{cmd:02X} sub=0x{sub:02X} a1=0x{a1:02X} a2=0x{a2:02X}")

    # ASCII yedek (opsiyonel)
    _re_ntc=re.compile(r"NTC\[(\d+)\]\s*=\s*([+-]?\d+(?:\.\d+)?)\s*C", re.I)
    _re_max=re.compile(r"Max\s*Temp\s*=\s*([+-]?\d+(?:\.\d+)?)\s*C", re.I)
    _re_min=re.compile(r"Min\s*Temp\s*=\s*([+-]?\d+(?:\.\d+)?)\s*C", re.I)

    def on_line_text(self,line:str):
        self.log(line)
        m=self._re_ntc.search(line)
        if m:
            i=int(m.group(1)); val=float(m.group(2))
            if 0<=i<20:
                v=int(round(val))
                if i == 6: v=min(255, v+2)
                self.ntc_vals[i]=v
                self._set_ntc_tile(i, v); 
                self._refresh_minmax_and_fan()
                self._csv_tick(); 
                return
        m=self._re_max.search(line)
        if m:
            self.ntc_max=int(round(float(m.group(1))))
            self.max_var.set(f"Max: {self.ntc_max} °C"); self._csv_tick(); return
        m=self._re_min.search(line)
        if m:
            self.ntc_min=int(round(float(m.group(1))))
            self.min_var.set(f"Min: {self.ntc_min} °C"); self._csv_tick(); return

    # ---- GUI yardımcıları ----
    def _set_ntc_tile(self,i:int, val:int):
        f,l1,l2=self.ntc_tiles[i]
        l2.configure(text=f"{int(val)} °C")   # threshold/renk yok → flicker yok

    def _refresh_minmax_and_fan(self):
        seen = [v for v in self.ntc_vals if isinstance(v, int)]
        if seen:
            self.ntc_min = min(seen); self.ntc_max = max(seen)
            self.min_var.set(f"Min: {self.ntc_min} °C")
            self.max_var.set(f"Max: {self.ntc_max} °C")
            self._gui_auto_fan_from_max()

    def _gui_auto_fan_from_max(self):
        if self.ntc_max is None: return
        mx = self.ntc_max
        # >40→5, >30→4, >20→3, aksi→2
        if   mx > 40: level = 5
        elif mx > 30: level = 4
        elif mx > 20: level = 3
        else:         level = 2
        if self._last_gui_fan_level == level: 
            return
        self._last_gui_fan_level = level
        try:
            self.fan_vars[0].set(str(level))
        except Exception:
            pass
        # MCU'ya (level-1) gönder
        frame=pack(self._addr(), CMD_FAN, bytes([(level-1) & 0xFF]))
        self.send_and_log(frame, f"FAN[GUI-AUTO] max={mx}°C -> level {level}")
        self.fan_mode.set("Fan Mode: MANUAL")

    # ---- Röle Actions (20 adet) ----
    def on_click_relay(self, rid:int):
        idx = rid - 1
        want_on = (self.relays[idx] == 0)
        self.relays[idx] = 1 if want_on else 0
        self._paint_relay(idx)
        data=bytes([SUB_RLY_ON if want_on else SUB_RLY_OFF, rid])
        frame=pack(self._addr(), CMD_RLY, data)
        self.send_and_log(frame, f"Relay R{rid} -> {'ON' if want_on else 'OFF'}")

    def all_off(self):
        for k in range(20):
            if self.relays[k]!=0:
                self.relays[k]=0
                self._paint_relay(k)
        # Opsiyonel toplu komut yoksa hepsini tek tek OFF göndermek yerine logla:
        self.send_and_log(pack(self._addr(), CMD_RLY, bytes([SUB_RLY_OFF, 0x00])), "ALL OFF (broadcast/impl)")

    # ---- OLED controls (debounce, sabit addr=0x60) ----
    def _oled_can_fire(self)->bool:
        now_ms = int(time.time()*1000)
        if now_ms - self._oled_last_ms < OLED_DEBOUNCE_MS:
            self.set_status("Warning: OLED command ignored (debounce)")
            return False
        self._oled_last_ms = now_ms
        return True

    def oled_sleep(self):
        if not self._oled_can_fire(): return
        frame = pack(OLED_ADDR, CMD_OLED, bytes([SUB_OLED_SLEEP]))
        self.send_and_log(frame, "OLED -> SLEEP")

    def oled_wake(self):
        if not self._oled_can_fire(): return
        frame = pack(OLED_ADDR, CMD_OLED, bytes([SUB_OLED_WAKE]))
        self.send_and_log(frame, "OLED -> WAKE")

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('STM Relay/NTC/FAN/OLED GUI — CRC8')
        self.geometry('1280x860')
        self.minsize(1100, 760)
        self['background']="#f4f5f7"

        top=ttk.Frame(self, style="Header.TFrame"); top.pack(fill='x',pady=(6,4))
        ttk.Button(top,text='Global Scan Ports',command=self.scan_ports,style="Tool.TButton").pack(side='left',padx=8)
        ttk.Label(top,text="MEHA",style='Header.TLabel').pack(side='right',padx=10,pady=4)

        self.nb=ttk.Notebook(self); self.nb.pack(fill='both',expand=True, padx=6, pady=(0,6))
        self.panels=[]
        for i in range(5):
            container = ttk.Frame(self, style="Root.TFrame")
            p=DevicePanel(container,i)
            p.pack(fill='both', expand=True)
            self.nb.add(container,text=f"Drawer {i+1}")
            self.panels.append(p)
        self.after(200,self.scan_ports)

    def scan_ports(self):
        ports=[p.device for p in list_ports.comports()]
        for p in self.panels:
            if ports:
                p.port_cb['values']=ports
                p.btn_connect.configure(state='normal')
            else:
                p.port_cb['values'] = ["(no ports)"]
                p.port_cb.set("(no ports)")
                p.btn_connect.configure(state='disabled')
        if ports:
            self.panels[0].log(f"[PORT] global scan: {len(ports)} port: {', '.join(ports)}")
        else:
            self.panels[0].log("[PORT] global scan: 0 port")

if __name__=='__main__':
    App().mainloop()