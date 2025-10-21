from __future__ import annotations
import tkinter as tk
from tkinter import ttk, messagebox
from typing import List

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

from .hub import SerialHub
from .device_panel import DevicePanel

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Static Load Bank GUI')
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

        ascii_art = "S2350\n ♡（„• ֊ •„)♡"
        ttk.Label(top, text=ascii_art, style='Header.TLabel', justify='right').pack(side='right', padx=10, pady=4)

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
    # (Orijinal App sınıfındaki TÜM metodları aynen devam ettir)
    # on_same_toggle, on_panel_port_changed, shared_connect, shared_disconnect,
    # _on_close, scan_ports ... hepsi eksiksiz buraya.
    ...
