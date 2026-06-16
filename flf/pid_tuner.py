#!/usr/bin/env python3
"""
PID Tuner Dashboard for STM32 Line Follower Robot
==================================================
Real-time plotting of error, correction, setpoint, and motor speeds.
Send PID values and parameters over serial/Bluetooth.

Usage:  python pid_tuner.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
import csv
from datetime import datetime

# ── Config ───────────────────────────────────────────────────────────────────
MAX_POINTS = 400
PLOT_INTERVAL_MS = 60
BAUD_OPTIONS = [9600, 19200, 38400, 57600, 115200]
SENSOR_THRESHOLD = 2048

# Junction types (match STM32 firmware enum)
JUNC_NONE, JUNC_LEFT, JUNC_RIGHT, JUNC_T, JUNC_CROSS = 0, 1, 2, 3, 4
JUNC_NAMES  = {JUNC_NONE: 'NONE', JUNC_LEFT: '← LEFT', JUNC_RIGHT: 'RIGHT →',
               JUNC_T: '⊤ T-JUNCTION', JUNC_CROSS: '✚ CROSS'}
JUNC_DEBOUNCE_S = 0.5

# ── Dark palette ─────────────────────────────────────────────────────────────
C = {
    'bg': '#0f0f1a', 'card': '#1a1a2e', 'card2': '#16213e',
    'accent': '#0f3460', 'hl': '#e94560', 'text': '#e0e0e0',
    'dim': '#888899', 'green': '#00d4aa', 'blue': '#4da6ff',
    'orange': '#ff9f43', 'purple': '#a855f7', 'red': '#ff4757',
    'cyan': '#00f0ff', 'grid': '#2a2a3e', 'plotbg': '#0d0d1a',
    'setpoint': '#ffffff',
}


class PIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("⚡ PID Tuner — Line Follower")
        self.root.geometry("1440x900")
        self.root.minsize(1100, 700)
        self.root.configure(bg=C['bg'])

        # Serial
        self.ser = None
        self.thread = None
        self.running = False

        # Data buffers
        self.t = deque(maxlen=MAX_POINTS)
        self.err = deque(maxlen=MAX_POINTS)
        self.corr = deque(maxlen=MAX_POINTS)
        self.sp = deque(maxlen=MAX_POINTS)
        self.lspd = deque(maxlen=MAX_POINTS)
        self.rspd = deque(maxlen=MAX_POINTS)
        self.batt = deque(maxlen=MAX_POINTS)
        self.ir = [0] * 8

        self.lock = threading.Lock()
        self.t0 = None
        self.new_data = False
        self.paused = False
        self.ir_flip = False             # flip IR bar display order
        self.ir_snapshot = [0] * 8   # always 8 elements, guarded by self.lock

        # Junction detection state
        self.junc_type = JUNC_NONE
        self.junc_ever_entered = False
        self.junc_counts = {'left': 0, 'right': 0, 't': 0}
        self.junc_last_type = JUNC_NONE
        self.junc_last_time = 0.0
        self.junc_history = []  # list of (timestamp_str, type_name, color_key)

        # Recording
        self.recording = False
        self.rec_data = []

        # Packet rate tracking
        self._pkt_count = 0
        self._pkt_time = time.time()
        self._pkt_rate = 0

        self._build_ui()
        self._build_plots()
        self._tick()

    # ═════════════════════════════════════════════════════════════════════════
    #  UI
    # ═════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        # ── Top bar ──────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=C['card'], pady=7, padx=10)
        top.pack(fill='x')

        tk.Label(top, text="⚡ PID Tuner", font=("Segoe UI", 15, "bold"),
                 fg=C['hl'], bg=C['card']).pack(side='left', padx=(0, 18))

        # Port
        tk.Label(top, text="Port:", fg=C['text'], bg=C['card'],
                 font=("Segoe UI", 10)).pack(side='left')
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var,
                                     width=11, state='readonly')
        self.port_cb.pack(side='left', padx=(2, 4))
        tk.Button(top, text="↻", command=self._refresh_ports, bg=C['accent'],
                  fg=C['text'], relief='flat', font=("Segoe UI", 10),
                  cursor='hand2').pack(side='left', padx=(0, 10))

        # Baud — fixed at 115200 (Bluetooth SPP ignores PC-side baud; firmware is hardcoded)
        self.baud_var = tk.StringVar(value="115200")
        tk.Label(top, text="Baud: 115200 (BT)", fg=C['dim'], bg=C['card'],
                 font=("Segoe UI", 9)).pack(side='left', padx=(2, 10))

        # Connect / Disconnect
        self.conn_btn = tk.Button(top, text="Connect", command=self._connect,
                                   bg=C['green'], fg='#000', relief='flat',
                                   font=("Segoe UI", 10, "bold"), padx=14,
                                   cursor='hand2')
        self.conn_btn.pack(side='left', padx=(0, 4))
        self.disc_btn = tk.Button(top, text="Disconnect", command=self._disconnect,
                                   bg=C['red'], fg='#fff', relief='flat',
                                   font=("Segoe UI", 10, "bold"), padx=14,
                                   cursor='hand2', state='disabled')
        self.disc_btn.pack(side='left', padx=(0, 14))

        self.conn_lbl = tk.Label(top, text="● Disconnected", fg=C['red'],
                                  bg=C['card'], font=("Segoe UI", 10))
        self.conn_lbl.pack(side='left')

        self.rate_lbl = tk.Label(top, text="", fg=C['dim'], bg=C['card'],
                                  font=("Segoe UI", 9))
        self.rate_lbl.pack(side='right')

        self._refresh_ports()

        # ── Body ─────────────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=C['bg'])
        body.pack(fill='both', expand=True, padx=5, pady=4)

        # Left panel
        left = tk.Frame(body, bg=C['bg'], width=260)
        left.pack(side='left', fill='y', padx=(0, 5))
        left.pack_propagate(False)

        self._card_pid(left)
        self._card_params(left)
        self._card_ctrl(left)
        self._card_status(left)
        self._card_junction(left)
        self._card_data(left)

        # Right panel (plots)
        self.plot_frame = tk.Frame(body, bg=C['plotbg'])
        self.plot_frame.pack(side='left', fill='both', expand=True)

    def _card(self, parent, title):
        f = tk.LabelFrame(parent, text=f"  {title}  ", fg=C['hl'],
                           bg=C['card'], font=("Segoe UI", 11, "bold"),
                           bd=1, relief='groove', padx=10, pady=7)
        f.pack(fill='x', pady=(0, 5))
        return f

    def _row(self, parent, label, default=""):
        r = tk.Frame(parent, bg=C['card'])
        r.pack(fill='x', pady=2)
        tk.Label(r, text=label, fg=C['text'], bg=C['card'],
                 font=("Segoe UI", 10), width=9, anchor='w').pack(side='left')
        v = tk.StringVar(value=default)
        tk.Entry(r, textvariable=v, width=10, bg=C['card2'], fg=C['text'],
                 insertbackground=C['text'], relief='flat',
                 font=("Consolas", 11), bd=2).pack(side='left', fill='x', expand=True)
        return v

    def _card_pid(self, p):
        c = self._card(p, "PID Parameters")
        self.kp = self._row(c, "Kp:", "1.0")
        self.ki = self._row(c, "Ki:", "0.0")
        self.kd = self._row(c, "Kd:", "1.0")
        
        btn_frame = tk.Frame(c, bg=C['card'])
        btn_frame.pack(fill='x', pady=(6, 0))
        
        tk.Button(btn_frame, text="Send PID", command=self._send_pid,
                  bg=C['blue'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 2))
                  
        tk.Button(btn_frame, text="⚡ Auto Tune", command=self._start_auto_tune,
                  bg=C['purple'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(2, 0))

    def _card_params(self, p):
        c = self._card(p, "Motion Parameters")
        self.base_spd = self._row(c, "Base Spd:", "500")
        self.pid_lim = self._row(c, "PID Limit:", "300")
        tk.Button(c, text="Send Params", command=self._send_params,
                  bg=C['orange'], fg='#000', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(fill='x', pady=(6, 0))

    def _card_ctrl(self, p):
        c = self._card(p, "Robot Control")
        r = tk.Frame(c, bg=C['card'])
        r.pack(fill='x')
        tk.Button(r, text="▶ START", command=self._send_start,
                  bg=C['green'], fg='#000', relief='flat',
                  font=("Segoe UI", 11, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r, text="■ STOP", command=self._send_stop,
                  bg=C['red'], fg='#fff', relief='flat',
                  font=("Segoe UI", 11, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(3, 0))
        tk.Button(c, text="📖 PID Tuning Guide", command=self._show_guide,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 9), cursor='hand2'
                  ).pack(fill='x', pady=(6, 0))

    def _card_status(self, p):
        c = self._card(p, "Live Values")
        self.sv = {}
        items = [("Battery", "batt", C['green']),
                 ("Error", "err", C['cyan']),
                 ("Correction", "corr", C['orange']),
                 ("Left Spd", "left", C['blue']),
                 ("Right Spd", "right", C['purple'])]
        for label, key, color in items:
            r = tk.Frame(c, bg=C['card'])
            r.pack(fill='x', pady=1)
            tk.Label(r, text=f"{label}:", fg=C['dim'], bg=C['card'],
                     font=("Segoe UI", 9), width=10, anchor='w').pack(side='left')
            v = tk.StringVar(value="—")
            tk.Label(r, textvariable=v, fg=color, bg=C['card'],
                     font=("Consolas", 11, "bold")).pack(side='left')
            self.sv[key] = v

    def _card_junction(self, p):
        c = self._card(p, "Junction Detection")

        # Current junction indicator row
        row = tk.Frame(c, bg=C['card'])
        row.pack(fill='x', pady=(0, 4))

        self.junc_icon_lbl = tk.Label(row, text="⛌", font=("Segoe UI", 20),
                                       fg=C['dim'], bg=C['card2'], width=2,
                                       relief='flat', bd=0)
        self.junc_icon_lbl.pack(side='left', padx=(0, 8))

        info_frame = tk.Frame(row, bg=C['card'])
        info_frame.pack(side='left', fill='x', expand=True)
        self.junc_type_lbl = tk.Label(info_frame, text="NO JUNCTION",
                                       fg=C['dim'], bg=C['card'],
                                       font=("Consolas", 12, "bold"), anchor='w')
        self.junc_type_lbl.pack(fill='x')
        self.junc_entered_lbl = tk.Label(info_frame, text="no junction entered yet",
                                          fg=C['dim'], bg=C['card'],
                                          font=("Segoe UI", 8), anchor='w')
        self.junc_entered_lbl.pack(fill='x')

        # Count row
        cnt_row = tk.Frame(c, bg=C['card'])
        cnt_row.pack(fill='x', pady=(2, 4))
        self.junc_count_lbls = {}
        for key, label, color in [('total', 'Total', C['purple']),
                                   ('left', 'Left', C['blue']),
                                   ('right', 'Right', C['green']),
                                   ('t', 'T-Jct', C['orange'])]:
            f = tk.Frame(cnt_row, bg=C['card2'], bd=1, relief='flat')
            f.pack(side='left', fill='x', expand=True, padx=1)
            val_lbl = tk.Label(f, text="0", fg=color, bg=C['card2'],
                                font=("Consolas", 11, "bold"))
            val_lbl.pack()
            tk.Label(f, text=label, fg=C['dim'], bg=C['card2'],
                     font=("Segoe UI", 7)).pack()
            self.junc_count_lbls[key] = val_lbl

        # History log
        tk.Label(c, text="HISTORY", fg=C['dim'], bg=C['card'],
                 font=("Segoe UI", 7), anchor='w').pack(fill='x')
        self.junc_hist_box = tk.Text(c, bg=C['plotbg'], fg=C['dim'],
                                      font=("Consolas", 8), height=4,
                                      relief='flat', state='disabled',
                                      padx=4, pady=2, wrap='none')
        self.junc_hist_box.pack(fill='x')
        self.junc_hist_box.tag_config('left', foreground=C['blue'])
        self.junc_hist_box.tag_config('right', foreground=C['green'])
        self.junc_hist_box.tag_config('tjunc', foreground=C['orange'])
        self.junc_hist_box.tag_config('time', foreground=C['dim'])

    def _card_data(self, p):
        c = self._card(p, "Data Tools")
        r1 = tk.Frame(c, bg=C['card'])
        r1.pack(fill='x', pady=(0, 4))
        self.rec_btn = tk.Button(r1, text="⏺ Record", command=self._toggle_rec,
                                  bg=C['purple'], fg='#fff', relief='flat',
                                  font=("Segoe UI", 10, "bold"), cursor='hand2')
        self.rec_btn.pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r1, text="💾 Export CSV", command=self._export,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 10), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True)

        r2 = tk.Frame(c, bg=C['card'])
        r2.pack(fill='x')
        self.pause_btn = tk.Button(r2, text="⏸ Pause Plot", command=self._toggle_pause,
                                    bg=C['accent'], fg=C['text'], relief='flat',
                                    font=("Segoe UI", 10), cursor='hand2')
        self.pause_btn.pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r2, text="🗑 Clear", command=self._clear_data,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 10), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True)

        r3 = tk.Frame(c, bg=C['card'])
        r3.pack(fill='x', pady=(4, 0))
        self.flip_btn = tk.Button(r3, text="⇄ Flip IR Order", command=self._toggle_ir_flip,
                                   bg=C['accent'], fg=C['text'], relief='flat',
                                   font=("Segoe UI", 10), cursor='hand2')
        self.flip_btn.pack(fill='x')

    # ═════════════════════════════════════════════════════════════════════════
    #  PLOTS
    # ═════════════════════════════════════════════════════════════════════════
    def _build_plots(self):
        self.fig = Figure(figsize=(10, 7), facecolor=C['plotbg'])

        # GridSpec: top row full-width (error+correction), bottom row split
        gs = self.fig.add_gridspec(
            2, 2,
            height_ratios=[1.6, 1],
            hspace=0.45, wspace=0.38,
            left=0.08, right=0.97, top=0.95, bottom=0.08
        )

        # ── Plot 1 (top, full width): Error & Correction — dual Y-axis ─────
        self.ax1   = self.fig.add_subplot(gs[0, :])
        self.ax1.set_facecolor(C['plotbg'])
        self.ax1_r = self.ax1.twinx()          # right axis = correction

        self.line_err,  = self.ax1.plot(
            [], [], color=C['cyan'], lw=2.0, label='Line Error', clip_on=True)
        self.line_sp,   = self.ax1.plot(
            [], [], color=C['setpoint'], lw=1.0, ls='--', alpha=0.55,
            label='Setpoint (0)', clip_on=True)
        self.line_corr, = self.ax1_r.plot(
            [], [], color=C['orange'], lw=1.8, alpha=0.9, label='Correction', clip_on=True)

        self.ax1.set_ylabel('Error',      color=C['cyan'],   fontsize=10)
        self.ax1_r.set_ylabel('Correction', color=C['orange'], fontsize=10)
        self.ax1.set_title(
            'Line Error  &  PID Correction  (dual axis)',
            color=C['hl'], fontsize=11, fontweight='bold', pad=6)
        self.ax1.set_xlabel('Time (s)', color=C['dim'], fontsize=9)

        # merged legend for dual-axis plot
        lns  = [self.line_err, self.line_sp, self.line_corr]
        labs = [l.get_label() for l in lns]
        self.ax1.legend(lns, labs, loc='upper right', fontsize=8,
                        facecolor=C['card'], edgecolor=C['grid'],
                        labelcolor=C['text'])
        self._style_ax(self.ax1)
        self._style_ax(self.ax1_r, right=True)

        # ── Plot 2 (bottom-left): Motor speeds ───────────────────────────────
        self.ax2 = self.fig.add_subplot(gs[1, 0])
        self.line_l, = self.ax2.plot([], [], color=C['blue'],   lw=1.5, label='Left', clip_on=True)
        self.line_r, = self.ax2.plot([], [], color=C['purple'], lw=1.5, label='Right', clip_on=True)
        self.ax2.set_ylabel('Speed (PWM)', color=C['text'], fontsize=9)
        self.ax2.set_xlabel('Time (s)',    color=C['dim'],  fontsize=9)
        self.ax2.set_title('Motor Speeds', color=C['hl'],
                            fontsize=11, fontweight='bold', pad=6)
        self.ax2.legend(loc='upper right', fontsize=8, facecolor=C['card'],
                        edgecolor=C['grid'], labelcolor=C['text'])
        self._style_ax(self.ax2)

        # ── Plot 3 (bottom-right): IR sensor live bar chart ──────────────────
        self.ax3 = self.fig.add_subplot(gs[1, 1])
        self.ax3.set_facecolor(C['plotbg'])
        n_sens = 8
        self.ir_bars = self.ax3.bar(
            range(n_sens), [0] * n_sens,
            color=C['cyan'], alpha=0.80,
            edgecolor=C['grid'], linewidth=0.6
        )
        self.ax3.set_ylim(0, 4095)
        self.ax3.set_xlim(-0.6, n_sens - 0.4)
        self.ax3.set_xticks(range(n_sens))
        # S1 = first value received (ADC CH0), S8 = last (ADC CH7)
        # Use ⇄ Flip IR button if your physical order is reversed
        self.ax3.set_xticklabels([f'S{i+1}' for i in range(n_sens)], fontsize=7)
        self.ax3.set_xlabel('S1=ADC-CH0 → S8=ADC-CH7  (⇄ Flip if reversed)',
                            color=C['dim'], fontsize=7)
        self.ax3.set_title('IR Sensors (live)', color=C['hl'],
                            fontsize=11, fontweight='bold', pad=6)
        self._style_ax(self.ax3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    def _style_ax(self, ax, right=False):
        ax.set_facecolor(C['plotbg'])
        ax.tick_params(colors=C['dim'], labelsize=8)
        if right:
            # Twin axis — style only the right spine / ticks in orange
            ax.tick_params(axis='y', colors=C['orange'], labelsize=8)
            ax.spines['right'].set_color(C['orange'])
            ax.spines['left'].set_visible(False)
            ax.spines['top'].set_visible(False)
            ax.spines['bottom'].set_visible(False)
        else:
            ax.spines['bottom'].set_color(C['grid'])
            ax.spines['left'].set_color(C['grid'])
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color=C['grid'], alpha=0.4, ls='--', lw=0.5)

    # ═════════════════════════════════════════════════════════════════════════
    #  SERIAL
    # ═════════════════════════════════════════════════════════════════════════
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            self.conn_lbl.config(text=f"● Connected ({port})", fg=C['green'])
            self.conn_btn.config(state='disabled')
            self.disc_btn.config(state='normal')
            self.t0 = time.time()
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.conn_lbl.config(text="● Disconnected", fg=C['red'])
        self.conn_btn.config(state='normal')
        self.disc_btn.config(state='disabled')

    def _reader(self):
        buf = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                n = self.ser.in_waiting or 1
                raw = self.ser.read(n)
                if not raw:
                    time.sleep(0.005)
                    continue
                buf += raw.decode('utf-8', errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip('\r\n ')
                    if line:
                        self._parse(line)
            except Exception as e:
                print(f"[reader] error: {e}")
                break

    def _parse(self, line):
        """Parse: IR:v0,v1,...;PL:left;PR:right;BV:bat;PE:error;PO:correction"""
        vals = {}
        try:
            for part in line.split(';'):
                part = part.strip()
                if ':' not in part:
                    continue
                key, value = part.split(':', 1)
                vals[key.strip()] = value.strip()
        except Exception:
            return

        # Must have at least one known telemetry key
        if not any(k in vals for k in ('PE', 'BV', 'PL', 'PR', 'PO', 'JC')):
            print(f"[parse] unrecognised packet: {line[:80]}")
            return

        now = time.time() - (self.t0 or time.time())
        try:
            error      = float(vals.get('PE', 0))
            correction = float(vals.get('PO', 0))
            left       = int(float(vals.get('PL', 0)))
            right      = int(float(vals.get('PR', 0)))
            battery    = float(vals.get('BV', 0))
            junc_val   = int(vals.get('JC', 0))
        except (ValueError, TypeError) as e:
            print(f"[parse] conversion error: {e} in '{line[:80]}'")
            return

        # Parse IR values — always produce exactly 8 elements, store under lock
        ir_str = vals.get('IR', '')
        ir_vals = [0] * 8
        if ir_str:
            try:
                parsed = [int(x) for x in ir_str.split(',') if x.strip()]
                for i, v in enumerate(parsed[:8]):
                    ir_vals[i] = v
                # ── DEBUG: print every 50th packet so you can verify order ──
                if self._pkt_count % 50 == 0:
                    bar_str = '  '.join(f'S{i+1}={v:4d}' for i, v in enumerate(ir_vals))
                    print(f"[IR debug] raw=\"{ir_str}\"")
                    print(f"[IR debug] bars: {bar_str}")
            except Exception:
                pass

        with self.lock:
            self.t.append(now)
            self.err.append(error)
            self.corr.append(correction)
            self.sp.append(0)
            self.lspd.append(left)
            self.rspd.append(right)
            self.batt.append(battery)
            self.ir_snapshot = ir_vals   # always 8 elements, written under lock
            self.new_data = True

            # Junction from STM32 JC: field
            self.junc_type = junc_val

            # Recording
            if self.recording:
                self.rec_data.append({
                    'time': now, 'error': error, 'correction': correction,
                    'left': left, 'right': right, 'battery': battery,
                    'ir': ir_str,
                })

        # ── Update live display safely from the main thread ──────────────────
        # Tkinter is NOT thread-safe: never call StringVar.set() from a
        # background thread.  Queue the update via root.after(0, ...).
        def _update_ui():
            self.sv['err'].set(f"{error:.1f}")
            self.sv['corr'].set(f"{correction:.1f}")
            self.sv['left'].set(str(left))
            self.sv['right'].set(str(right))
            self.sv['batt'].set(f"{battery:.1f} V")
            self._update_junction_ui(junc_val)
        self.root.after(0, _update_ui)

        # Packet rate
        self._pkt_count += 1
        elapsed = time.time() - self._pkt_time
        if elapsed >= 1.0:
            self._pkt_rate = self._pkt_count / elapsed
            self._pkt_count = 0
            self._pkt_time = time.time()

    # ═════════════════════════════════════════════════════════════════════════
    #  SEND COMMANDS
    # ═════════════════════════════════════════════════════════════════════════
    def _send(self, msg):
        if self.ser and self.ser.is_open:
            self.ser.write((msg + '\r\n').encode())
        else:
            messagebox.showwarning("Not connected", "Connect to a port first.")

    def _send_pid(self):
        try:
            p, i, d = float(self.kp.get()), float(self.ki.get()), float(self.kd.get())
            self._send(f"PID:{p},{i},{d}")
        except ValueError:
            messagebox.showerror("Invalid", "Enter valid numbers for Kp, Ki, Kd.")

    def _send_params(self):
        try:
            bs = int(self.base_spd.get())
            pl = int(self.pid_lim.get())
            self._send(f"PARAM:BS{bs}PL{pl}")
        except ValueError:
            messagebox.showerror("Invalid", "Enter valid integers.")

    def _send_start(self):
        self._send("START")

    def _send_stop(self):
        self._send("STOP")

    def _show_guide(self):
        """Open a non-blocking PID tuning guide window."""
        win = tk.Toplevel(self.root)
        win.title("📖 PID Tuning Guide — Line Follower")
        win.geometry("680x720")
        win.configure(bg=C['bg'])
        win.resizable(True, True)

        # ── header ──────────────────────────────────────────────────────────
        tk.Label(win, text="PID Tuning Guide",
                 font=("Segoe UI", 16, "bold"), fg=C['hl'], bg=C['bg']
                 ).pack(pady=(16, 2))
        tk.Label(win,
                 text="Use the dual-axis chart: cyan = Error, orange = Correction",
                 font=("Segoe UI", 10), fg=C['dim'], bg=C['bg']
                 ).pack(pady=(0, 12))

        # ── scrollable text ──────────────────────────────────────────────────
        frame = tk.Frame(win, bg=C['bg'])
        frame.pack(fill='both', expand=True, padx=16, pady=(0, 16))

        sb = tk.Scrollbar(frame)
        sb.pack(side='right', fill='y')

        txt = tk.Text(frame, bg=C['card'], fg=C['text'],
                      font=("Consolas", 10), wrap='word',
                      relief='flat', padx=14, pady=10,
                      yscrollcommand=sb.set, state='normal')
        txt.pack(fill='both', expand=True)
        sb.config(command=txt.yview)

        # ── colour tags ─────────────────────────────────────────────────────
        txt.tag_config('h1',   foreground=C['hl'],     font=("Segoe UI", 12, "bold"))
        txt.tag_config('h2',   foreground=C['cyan'],   font=("Segoe UI", 10, "bold"))
        txt.tag_config('good', foreground=C['green'],  font=("Consolas", 10))
        txt.tag_config('bad',  foreground=C['red'],    font=("Consolas", 10))
        txt.tag_config('tip',  foreground=C['orange'], font=("Segoe UI", 10, "italic"))
        txt.tag_config('code', foreground=C['blue'],   font=("Consolas", 10))
        txt.tag_config('dim',  foreground=C['dim'],    font=("Segoe UI",  9))

        def h1(s):  txt.insert('end', s + '\n', 'h1')
        def h2(s):  txt.insert('end', s + '\n', 'h2')
        def ok(s):  txt.insert('end', s + '\n', 'good')
        def bad(s): txt.insert('end', s + '\n', 'bad')
        def tip(s): txt.insert('end', s + '\n', 'tip')
        def cod(s): txt.insert('end', s + '\n', 'code')
        def dim(s): txt.insert('end', s + '\n', 'dim')
        def nl():   txt.insert('end', '\n')

        # ── content ─────────────────────────────────────────────────────────
        h1("STEP 0 — Reset all gains before starting")
        cod("  Kp = 0.5   Ki = 0.0   Kd = 0.0")
        tip("  Always start from a known baseline. Click Send PID.")
        nl()

        h1("STEP 1 — Tune Kp first (Proportional)")
        txt.insert('end', "Goal: robot follows the line but may oscillate.\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Kp too LOW  → Error barely decreases, robot drifts off line")
        bad("               Correction line stays small even with big Error")
        bad("  Kp too HIGH → Error oscillates (zigzag), Correction overshoots")
        ok("  Kp correct  → Error decays toward 0, small overshoot is OK")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  Raise Kp in steps of 0.5 until you see oscillation,\n")
        txt.insert('end', "  then back off ~20%. That is your starting Kp.\n")
        nl()

        h1("STEP 2 — Tune Kd (Derivative) to kill oscillation")
        txt.insert('end', "Goal: damp out the Kp oscillations without jerking.\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Kd too LOW  → Error still oscillates (zigzag correction)")
        bad("  Kd too HIGH → Correction spikes sharply on every sensor change")
        bad("               (looks like narrow spikes on orange line)")
        ok("  Kd correct  → Error converges smoothly, Correction is clean")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  Raise Kd from 0 in steps of 0.2 until oscillation\n")
        txt.insert('end', "  is gone. Stop before correction spikes appear.\n")
        nl()
        h1("STEP 3 — Tune Ki (Integral) last, and carefully")
        txt.insert('end', "Goal: eliminate steady-state offset (robot slightly off centre).\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Ki too LOW  → Error never quite reaches 0 (flat offset line)")
        bad("  Ki too HIGH → Error slowly grows until correction saturates")
        bad("               ('integral windup' — correction rail at ±limit)")
        ok("  Ki correct  → Error reaches 0 cleanly, correction stays bounded")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  For a line follower Ki is often 0.0 or very small (0.01-0.1).\n")
        txt.insert('end', "  Only add Ki if robot consistently misses centre.\n")
        nl()

        h1("STEP 4 — Tune Base Speed & PID Limit")
        h2("  Base Speed")
        txt.insert('end', "  Higher speed → tighter curves need higher Kp/Kd.\n")
        txt.insert('end', "  Start at 400-500, increase after PID is stable.\n")
        nl()
        h2("  PID Limit (correction cap)")
        txt.insert('end', "  Caps how much correction can be applied.\n")
        txt.insert('end', "  If motors hit 0 or max every turn → limit too high.\n")
        txt.insert('end', "  Rule of thumb: limit ≈ Base Speed × 0.6\n")
        nl()

        h1("QUICK DIAGNOSTIC — chart patterns")
        patterns = [
            ("Error zigzags fast, Correction tracks it",    "→ Kp too high",     'bad'),
            ("Error barely moves toward 0",                  "→ Kp too low",      'bad'),
            ("Correction has sharp narrow spikes",           "→ Kd too high",     'bad'),
            ("Error oscillates, Correction is smooth",       "→ Kd too low",      'bad'),
            ("Correction rail at ±limit permanently",        "→ windup, Ki↓ or Limit↑", 'bad'),
            ("Error decays smoothly to ~0",                  "→ well tuned ✓",    'good'),
            ("Left/Right speeds differ by Correction",       "→ PID working ✓",   'good'),
        ]
        for symptom, diagnosis, tag in patterns:
            txt.insert('end', f"  {symptom}\n", 'dim')
            txt.insert('end', f"      {diagnosis}\n", tag)
            nl()

        h1("SUGGESTED STARTING POINT")
        cod("  Kp=1.0  Ki=0.0  Kd=0.01   Base=450   Limit=270")
        tip("  Adjust from here using the steps above.")
        tip("  With hardware dt: try Kp in 0.5–3.0, Kd in 0.005–0.05 range first.")
        nl()

        h1("STM32 FIRMWARE BEST PRACTICES")
        h2("  1. Continuous Centroid Error (Smooth Signal)")
        txt.insert('end', "  Do NOT feed raw digital jumps to your PID. Use a weighted\n")
        txt.insert('end', "  centroid from your IR sensor ADC values:\n")
        cod("      centroid = Sum(index * Value[index]) / Sum(Value[index])\n")
        txt.insert('end', "  This produces smooth, sub-millimetre precision error steps.\n")
        nl()
        h2("  2. Microsecond-Resolution Loop Timer (dt)")
        txt.insert('end', "  HAL_GetTick() has only 1 ms resolution, which introduces\n")
        txt.insert('end', "  massive quantization noise in Kd calculations. Use DWT->CYCCNT\n")
        txt.insert('end', "  or a dedicated STM32 hardware timer in microsecond mode.\n")
        nl()
        h2("  3. Dynamic Junction Handling Toggle (JH)")
        txt.insert('end', "  To keep the auto-tuners from being tripped by junction turns,\n")
        txt.insert('end', "  add a command to toggle junction handling from serial:\n")
        txt.insert('end', "  In main.c:\n")
        cod("      // Inside PARAM: command parser:\n")
        cod("      char *jh = strstr(cmd, \"JH\");\n")
        cod("      if (jh) junction_handling_enabled = atoi(jh + 2);\n")
        cod("      \n")
        cod("      // In main loop:\n")
        cod("      j = junction_handling_enabled ? detect_junction() : NO_JUNCTION;\n")
        cod("      if (j != NO_JUNCTION) handle_junction();\n")
        cod("      else { ... calculate_pid(); ... }\n")
        nl()

        txt.config(state='disabled')
        tk.Button(win, text="Close", command=win.destroy,
                  bg=C['hl'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), padx=20, cursor='hand2'
                  ).pack(pady=(0, 14))

    # ═════════════════════════════════════════════════════════════════════════
    #  AUTO TUNE (Grid Search)
    # ═════════════════════════════════════════════════════════════════════════
    # ═════════════════════════════════════════════════════════════════════════
    #  AUTO TUNE — method selector
    # ═════════════════════════════════════════════════════════════════════════
    def _start_auto_tune(self):
        """Open a method-selection dialog: Relay (fast) or Grid Search (thorough)."""
        if not self.running:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Auto-Tune Method")
        dialog.geometry("420x300")
        dialog.configure(bg=C['bg'])
        dialog.resizable(False, False)
        dialog.attributes('-topmost', True)
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - 210
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - 150
        dialog.geometry(f"+{x}+{y}")

        tk.Label(dialog, text="⚡ Auto-Tune Method", font=("Segoe UI", 14, "bold"),
                 fg=C['hl'], bg=C['bg']).pack(pady=(16, 6))

        # Relay card
        rf = tk.Frame(dialog, bg=C['card'], bd=1, relief='groove')
        rf.pack(fill='x', padx=20, pady=(0, 6))
        tk.Label(rf, text="⚡  Relay Feedback  (Åström-Hägglund)",
                 font=("Segoe UI", 11, "bold"), fg=C['cyan'], bg=C['card']
                 ).pack(anchor='w', padx=12, pady=(8, 0))
        tk.Label(rf, text="~15 s  •  bounded oscillations  •  mathematically principled",
                 font=("Segoe UI", 8), fg=C['dim'], bg=C['card']
                 ).pack(anchor='w', padx=12, pady=(1, 6))
        tk.Button(rf, text="Start Relay Auto-Tune  →",
                  command=lambda: [dialog.destroy(), self._start_relay_tune()],
                  bg=C['cyan'], fg='#000', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(fill='x', padx=12, pady=(0, 10))

        # Grid search card
        gf = tk.Frame(dialog, bg=C['card2'], bd=1, relief='groove')
        gf.pack(fill='x', padx=20, pady=(0, 6))
        tk.Label(gf, text="🔍  Grid Search  (brute-force)",
                 font=("Segoe UI", 11, "bold"), fg=C['orange'], bg=C['card2']
                 ).pack(anchor='w', padx=12, pady=(8, 0))
        tk.Label(gf, text="3-4 min  •  exhaustive Kp/Kd sweep  •  good fallback",
                 font=("Segoe UI", 8), fg=C['dim'], bg=C['card2']
                 ).pack(anchor='w', padx=12, pady=(1, 6))
        tk.Button(gf, text="Start Grid Search  →",
                  command=lambda: [dialog.destroy(), self._run_grid_search()],
                  bg=C['orange'], fg='#000', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(fill='x', padx=12, pady=(0, 10))

        tk.Button(dialog, text="Cancel", command=dialog.destroy,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 9), cursor='hand2').pack(pady=(0, 12))

    # ═════════════════════════════════════════════════════════════════════════
    #  RELAY FEEDBACK AUTO-TUNE  (Åström-Hägglund)
    # ═════════════════════════════════════════════════════════════════════════
    def _start_relay_tune(self):
        """Relay-feedback auto-tuning (Åström-Hägglund method).

        No firmware changes required:
          Send PID:99999,0,0 + PARAM:BS<n>PL<h>
          → Kp so huge the PID saturates to ±limit for any nonzero error,
            making the STM32's own PID act as a bang-bang relay with amplitude h.
          Collect error signal for ~20 s, detect zero crossings.
          Measure Tu (period) and a (peak amplitude).
          Ku = 4h / (π·a)
          Z-N PD:  Kp = 0.8·Ku,   Kd = Kp·Tu/8
          T-L  :   Kp = Ku/3.2,   Kd = Kp·0.111·Tu
        """
        self.relay_win = tk.Toplevel(self.root)
        self.relay_win.title("Relay Feedback Auto-Tuning")
        self.relay_win.geometry("480x640")
        self.relay_win.configure(bg=C['bg'])
        self.relay_win.resizable(False, False)
        self.relay_win.attributes('-topmost', True)
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - 240
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - 320
        self.relay_win.geometry(f"+{x}+{y}")
        self.relay_win.protocol("WM_DELETE_WINDOW", self._cancel_relay)

        tk.Label(self.relay_win, text="⚡ Relay Feedback Auto-Tuning",
                 font=("Segoe UI", 14, "bold"), fg=C['cyan'], bg=C['bg']
                 ).pack(pady=(12, 2))
        tk.Label(self.relay_win,
                 text="Robot oscillates under relay control  →  Ku & Tu measured  →  PD gains computed",
                 font=("Segoe UI", 8), fg=C['dim'], bg=C['bg']
                 ).pack(pady=(0, 6))

        # Relay amplitude control
        amp_row = tk.Frame(self.relay_win, bg=C['bg'])
        amp_row.pack(pady=(0, 4))
        tk.Label(amp_row, text="Relay amplitude  h =", fg=C['text'], bg=C['bg'],
                 font=("Segoe UI", 10)).pack(side='left')
        self._relay_amp_var = tk.StringVar(value="100")
        tk.Entry(amp_row, textvariable=self._relay_amp_var, width=6,
                 bg=C['card2'], fg=C['cyan'], insertbackground=C['cyan'],
                 font=("Consolas", 11), relief='flat', bd=2
                 ).pack(side='left', padx=4)
        tk.Label(amp_row, text="(motor correction units  ≈ PID limit)",
                 fg=C['dim'], bg=C['bg'], font=("Segoe UI", 8)).pack(side='left')

        # Safety warning about relay amplitude
        tk.Label(self.relay_win,
                 text="⚠️ Note: High amplitude (e.g. 200) causes violent shaking & Bluetooth brownouts.\nUse 80 to 120 for smoother, safer oscillations on actual tracks.",
                 font=("Segoe UI", 8, "italic"), fg=C['orange'], bg=C['bg'], justify='center'
                 ).pack(pady=(0, 6))

        self.relay_status = tk.Label(self.relay_win, text="Ready — press Start to begin.",
                                     fg=C['orange'], bg=C['bg'],
                                     font=("Segoe UI", 10, "bold"))
        self.relay_status.pack(pady=(4, 2))

        self.relay_progress = ttk.Progressbar(self.relay_win, orient='horizontal',
                                               length=440, mode='determinate')
        self.relay_progress.pack(pady=(0, 6))

        self.relay_log_box = tk.Text(self.relay_win, bg=C['card'], fg=C['text'],
                                      font=("Consolas", 9), height=13, width=56,
                                      relief='flat', padx=8, pady=6)
        self.relay_log_box.pack(padx=10, pady=4)
        self.relay_log_box.tag_config('head', foreground=C['cyan'],  font=("Consolas", 9, "bold"))
        self.relay_log_box.tag_config('good', foreground=C['green'], font=("Consolas", 9, "bold"))
        self.relay_log_box.tag_config('warn', foreground=C['orange'],font=("Consolas", 9))
        self.relay_log_box.tag_config('dim',  foreground=C['dim'],   font=("Consolas", 9))
        self.relay_log_box.config(state='disabled')

        self.relay_result_frame = tk.Frame(self.relay_win, bg=C['bg'])
        self.relay_result_frame.pack(fill='x', padx=10, pady=(2, 0))

        btn_row = tk.Frame(self.relay_win, bg=C['bg'])
        btn_row.pack(fill='x', padx=10, pady=(6, 12))
        self.relay_start_btn = tk.Button(btn_row, text="▶ Start",
                                          command=self._relay_go,
                                          bg=C['green'], fg='#000', relief='flat',
                                          font=("Segoe UI", 10, "bold"), cursor='hand2')
        self.relay_pause_btn = tk.Button(btn_row, text="⏸ Pause",
                                          command=self._relay_pause_manually,
                                          bg=C['orange'], fg='#000', relief='flat',
                                          font=("Segoe UI", 10, "bold"), cursor='hand2',
                                          state='disabled')
        self.relay_resume_btn = tk.Button(btn_row, text="▶ Resume",
                                           command=self._relay_resume,
                                           bg=C['cyan'], fg='#000', relief='flat',
                                           font=("Segoe UI", 10, "bold"), cursor='hand2',
                                           state='disabled')
        
        self.relay_start_btn.pack(side='left', fill='x', expand=True, padx=(0, 2))
        self.relay_pause_btn.pack(side='left', fill='x', expand=True, padx=(2, 2))
        self.relay_resume_btn.pack(side='left', fill='x', expand=True, padx=(2, 2))
        
        tk.Button(btn_row, text="✕ Cancel", command=self._cancel_relay,
                  bg=C['red'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(2, 0))

        self._relay_running = False
        self._relay_applied = False
        self._relay_log_line("Relay Feedback Auto-Tuner ready.", 'head')
        self._relay_log_line("1. Place robot on the line and send START.", 'dim')
        self._relay_log_line("2. Set h to match your usual PID limit (recommended 100).", 'dim')
        self._relay_log_line("3. Press ▶ Start — robot oscillates for 20 s.", 'dim')

    def _relay_log_line(self, text, tag=''):
        """Append a line to the relay log (main-thread safe)."""
        if not hasattr(self, 'relay_log_box') or not self.relay_log_box.winfo_exists():
            return
        self.relay_log_box.config(state='normal')
        if tag:
            self.relay_log_box.insert('end', text + '\n', tag)
        else:
            self.relay_log_box.insert('end', text + '\n')
        self.relay_log_box.see('end')
        self.relay_log_box.config(state='disabled')

    def _relay_go(self):
        """Validate amplitude, save current PID, activate relay mode, start collection."""
        try:
            h = int(self._relay_amp_var.get())
            if h <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Bad amplitude", "Enter a positive integer for relay amplitude.")
            return

        self._relay_amplitude = h
        self._relay_applied = False
        # Save current settings for restoration on cancel/close
        self._relay_saved = {
            'kp': self.kp.get(), 'ki': self.ki.get(), 'kd': self.kd.get(),
            'bs': self.base_spd.get(), 'pl': self.pid_lim.get()
        }

        # Initialize warnings/telemetry saturation trackers
        self._junc_warn_logged = False
        self._relay_samples_count = 0
        self._relay_sat_samples_count = 0

        # Activate relay: huge Kp → PID saturates to ±limit = pure bang-bang relay
        self._send("PID:99999.0,0.0,0.0")
        
        # Enforce base speed, set PID limit to amplitude h, and disable junction handling (JH0)
        self._send(f"PARAM:BS{self._relay_saved['bs']}PL{h}JH0")

        self.relay_start_btn.config(state='disabled')
        self.relay_pause_btn.config(state='normal')
        self.relay_resume_btn.config(state='disabled')
        self._relay_data       = []          # [(elapsed_s, error), ...]
        self._relay_last_seen  = None        # last stored error value
        self._relay_last_pkt   = time.time() # watchdog: time of last ANY packet
        self._relay_crossings  = 0           # live counter for status bar
        self._relay_running_mean = 0.0       # live offset tracker
        self._relay_running    = True
        self._relay_duration   = 30.0
        self._relay_start_ts   = time.time()

        self.relay_status.config(text="Phase 1/2 — Relay active — keep robot on track…", fg=C['orange'])
        self._relay_log_line(f"Relay active:  h = ±{h}", 'head')
        self._relay_log_line("Collecting up to 30 s — finishes early once enough cycles seen.", 'dim')
        self._relay_log_line("Watchdog aborts only if ZERO telemetry packets arrive for 4 s.", 'dim')
        self._relay_collect()

    def _relay_collect(self):
        """Sample every 40 ms. Deduplicates for analysis, monitors safety stops,
        and uses the watchdog to catch connection losses."""
        if not self._relay_running:
            return

        now     = time.time()
        elapsed = now - self._relay_start_ts

        # ── Safety-Stop and Run-off Detection ─────────────────────────────────
        # Disabled as requested by the user. Use manual Pause button to pause.

        # ── Live Warning Trackers ─────────────────────────────────────────────
        with self.lock:
            # Junction Warning Reminder (log once per run)
            current_junc = self.junc_type
            if current_junc != 0 and not self._junc_warn_logged:
                self._relay_log_line(f"⚠️ Junction detected (type {current_junc}). Ensure junction handling is off in firmware to avoid corrupted data.", 'warn')
                self._junc_warn_logged = True

            # Saturation Sample Counting
            if len(self.lspd) > 0 and len(self.rspd) > 0:
                self._relay_samples_count += 1
                l_val, r_val = self.lspd[-1], self.rspd[-1]
                # Speeds of 0 or 1000 indicate the controller output has saturated the actuator limits
                if l_val <= 0 or l_val >= 1000 or r_val <= 0 or r_val >= 1000:
                    self._relay_sat_samples_count += 1

        with self.lock:
            if self.err:
                e_new = self.err[-1]
                # Heartbeat: any packet keeps watchdog alive
                self._relay_last_pkt = now
                
                self._relay_data.append((elapsed, e_new))
                self._relay_last_seen = e_new
                
                # Track running mean to detect crossings even with DC offset
                if len(self._relay_data) == 1:
                    self._relay_running_mean = float(e_new)
                else:
                    self._relay_running_mean = 0.05 * e_new + 0.95 * self._relay_running_mean
                    
                if len(self._relay_data) >= 2:
                    e_prev_c = self._relay_data[-2][1] - self._relay_running_mean
                    e_new_c = e_new - self._relay_running_mean
                    if e_prev_c * e_new_c < 0:
                        self._relay_crossings += 1

        # ── Watchdog: fires only when NO telemetry arrives for 4 s ───────────
        silent_s = now - self._relay_last_pkt
        if silent_s > 4.0 and elapsed > 2.0:
            self._relay_running = False
            self.relay_progress['value'] = 100
            self.relay_status.config(text="⚠ No telemetry for 4 s — Bluetooth lost?", fg=C['red'])
            self._relay_log_line(f"ABORT: no packets received for {silent_s:.1f} s.", 'warn')
            self._relay_log_line("Check Bluetooth connection and press ↻ Re-run.", 'warn')
            self._restore_relay_pid()
            self.relay_start_btn.config(state='normal', text="↻ Re-run")
            self.relay_pause_btn.config(state='disabled')
            self.relay_resume_btn.config(state='disabled')
            return

        # ── Early exit: 20 crossings + at least 8.0 s ──────────────────────────
        if self._relay_crossings >= 20 and elapsed >= 8.0:
            self._relay_running = False
            self.relay_progress['value'] = 100
            self._relay_log_line(f"Early finish: {self._relay_crossings} crossings in {elapsed:.1f} s.", 'good')
            self._relay_analyze()
            return

        self.relay_progress['value'] = min(100.0, elapsed / self._relay_duration * 100.0)
        self.relay_status.config(
            text=f"Collecting…  {elapsed:.1f} s  |  {len(self._relay_data)} pts  |  {self._relay_crossings} crossings")

        if elapsed < self._relay_duration:
            self.root.after(40, self._relay_collect)
        else:
            self.relay_progress['value'] = 100
            self._relay_analyze()

    def _relay_pause_due_to_safety(self, is_off_line, is_stopped):
        """Pause the relay tuning process and wait for user intervention."""
        self._relay_running = False
        self._send("STOP")
        self._relay_pause_ts = time.time()

        reason = "lost the line" if is_off_line else "stopped (safety-stop)"
        self.relay_status.config(text=f"⚠️ Robot {reason}! Put on line and Resume.", fg=C['red'])
        self._relay_log_line(f"⚠️ PAUSED: Robot {reason}.", 'warn')
        self._relay_log_line("1. Manually place the robot back on the line.", 'dim')
        self._relay_log_line("2. Click [▶ Resume] to continue this run.", 'dim')

        self.relay_start_btn.config(state='disabled')
        self.relay_pause_btn.config(state='disabled')
        self.relay_resume_btn.config(state='normal')

    def _relay_pause_manually(self):
        """Manually pause the relay tuning process."""
        if not self._relay_running:
            return
        self._relay_running = False
        self._send("STOP")
        self._relay_pause_ts = time.time()

        self.relay_status.config(text="Paused manually. Put on line and click Resume.", fg=C['orange'])
        self._relay_log_line("⏸ PAUSED: Manual pause.", 'warn')
        self._relay_log_line("1. Manually align the robot back on the line.", 'dim')
        self._relay_log_line("2. Click [▶ Resume] to continue this run.", 'dim')

        self.relay_start_btn.config(state='disabled')
        self.relay_pause_btn.config(state='disabled')
        self.relay_resume_btn.config(state='normal')

    def _relay_resume(self):
        """Resume relay feedback tuning after a safety stop, preserving progress."""
        if not self.running:
            return

        self.relay_resume_btn.config(state='disabled')
        self.relay_pause_btn.config(state='normal')
        
        # Adjust start time to subtract the pause duration, keeping elapsed time contiguous
        if hasattr(self, '_relay_pause_ts') and self._relay_pause_ts is not None:
            pause_duration = time.time() - self._relay_pause_ts
            self._relay_start_ts += pause_duration
            self._relay_pause_ts = None
            self._relay_log_line("Resumed run. Appending to previous crossings...", 'dim')
        else:
            self._relay_data       = []
            self._relay_last_seen  = None
            self._relay_crossings  = 0
            self._relay_running_mean = 0.0
            self._relay_log_line("Resumed. Resetting run data...", 'dim')

        self._relay_last_pkt   = time.time()
        self._relay_running    = True

        # Send start and parameters again
        self._send("PID:99999.0,0.0,0.0")
        h = self._relay_amplitude
        bs = self._relay_saved.get('bs', 500) if self._relay_saved else 500
        self._send(f"PARAM:BS{bs}PL{h}JH0")
        self._send("START")

        self.relay_status.config(text="Phase 1/2 — Resuming relay...", fg=C['orange'])

        # Wait 600ms before starting to collect data so we don't catch the initial startup transient
        self.root.after(600, self._relay_collect)

    def _relay_analyze(self):
        """Robust Ku/Tu extraction with proper amplitude measurement, outlier-trimmed
        period estimation, and physically correct gain formulas for discrete PID.

        Key insights applied:
        - Amplitude must be measured from RAW (not filtered) data to avoid EMA attenuation
        - DC offset must be removed before zero-crossing detection
        - Half-period outliers must be trimmed (IQR method)
        - Gain formulas must account for firmware doing u=Kp*e+Kd*(e-e_prev) WITHOUT /dt
        - At 1kHz loop rate, the derivative term is effectively Kd/dt = Kd*1000
        """
        import math as _math
        self._relay_running = False
        self.relay_status.config(text="Phase 2/2 — Analysing oscillation…", fg=C['cyan'])
        self._restore_relay_pid()

        times  = [d[0] for d in self._relay_data]
        errors = [d[1] for d in self._relay_data]
        n = len(errors)
        self._relay_log_line(f"Collected {n} unique samples over {times[-1]:.2f} s.", 'dim')

        if n < 15:
            self.relay_status.config(text="⚠ Too few samples — check Bluetooth.", fg=C['red'])
            self._relay_log_line("ERROR: <15 unique samples received.", 'warn')
            self.relay_start_btn.config(state='normal', text="↻ Re-run")
            self.relay_pause_btn.config(state='disabled')
            self.relay_resume_btn.config(state='disabled')
            return

        # ── 1. Remove DC offset ───────────────────────────────────────────────
        mean_err = sum(errors) / n
        centered = [e - mean_err for e in errors]
        self._relay_log_line(f"  DC offset removed: mean = {mean_err:.2f}", 'dim')

        # ── Saturation and Digital Jumps Validation ───────────────────────────
        if hasattr(self, '_relay_samples_count') and self._relay_samples_count > 0:
            sat_fraction = self._relay_sat_samples_count / self._relay_samples_count
            if sat_fraction > 0.15:
                self._relay_log_line(f"⚠️ Motor saturation detected! ({sat_fraction*100:.1f}% of run).", 'warn')
                self._relay_log_line("  Relay amplitude h is too high for your base speed.", 'dim')
                self._relay_log_line("  Reduce h or base speed to keep motors in linear range.", 'dim')

        diffs = [abs(errors[i] - errors[i-1]) for i in range(1, n)]
        non_zero_diffs = [d for d in diffs if d > 1e-3]
        if non_zero_diffs:
            min_step = min(non_zero_diffs)
            unique_errs = len(set(errors))
            if min_step >= 8.0 and unique_errs <= 10:
                self._relay_log_line("⚠️ Raw digital jumps detected in error signal!", 'warn')
                self._relay_log_line("  Relay tuning requires a continuous/smoothed error signal.", 'dim')
                self._relay_log_line("  Implement continuous centroid calculation in firmware.", 'dim')
                self._relay_log_line(f"  (Min step: {min_step:.1f}, Unique error levels: {unique_errs})", 'dim')

        # ── 2. Zero-crossing detection on lightly smoothed data ───────────────
        # Use a 3-tap median filter just for crossing detection (preserves amplitude)
        def _median3(arr):
            out = [arr[0]]
            for i in range(1, len(arr) - 1):
                triple = sorted([arr[i-1], arr[i], arr[i+1]])
                out.append(triple[1])
            out.append(arr[-1])
            return out

        smooth = _median3(centered)

        max_abs = max(abs(e) for e in centered)
        if max_abs <= 0.5:
            self.relay_status.config(text="⚠ Zero amplitude — sensors not reading.", fg=C['red'])
            self.relay_start_btn.config(state='normal', text="↻ Re-run")
            self.relay_pause_btn.config(state='disabled')
            return

        noise_gate = max(0.5, 0.02 * max_abs)
        crossings = []
        for i in range(1, n):
            ep = smooth[i-1]
            ec = smooth[i]
            if ep * ec < 0 and (abs(ep) > noise_gate or abs(ec) > noise_gate):
                tc = times[i-1] + (times[i] - times[i-1]) * abs(ep) / (abs(ep) + abs(ec))
                crossings.append(tc)

        self._relay_log_line(f"Zero crossings found: {len(crossings)}", 'dim')

        # Discard first 2 crossings — startup transient
        if len(crossings) > 2:
            crossings = crossings[2:]

        MIN_X = 4
        if len(crossings) < MIN_X:
            self.relay_status.config(text=f"⚠ Only {len(crossings)} steady crossings — need {MIN_X}+", fg=C['red'])
            self._relay_log_line(f"After transient: {len(crossings)} crossings < {MIN_X} needed.", 'warn')
            self._relay_log_line("Increase relay amplitude h, or rerun for longer.", 'warn')
            self.relay_start_btn.config(state='normal', text="↻ Re-run")
            self.relay_pause_btn.config(state='disabled')
            self.relay_resume_btn.config(state='disabled')
            return

        # ── 3. Tu from half-periods with IQR outlier trimming ─────────────────
        half_periods = [crossings[i+1] - crossings[i] for i in range(len(crossings)-1)]

        if len(half_periods) >= 5:
            sorted_hp = sorted(half_periods)
            q1 = sorted_hp[len(sorted_hp) // 4]
            q3 = sorted_hp[3 * len(sorted_hp) // 4]
            iqr = q3 - q1
            trimmed = [hp for hp in half_periods if q1 - 1.5*iqr <= hp <= q3 + 1.5*iqr]
            if len(trimmed) >= 2:
                half_periods = trimmed

        mean_hp = sum(half_periods) / len(half_periods)
        Tu = 2.0 * mean_hp

        if len(half_periods) >= 3:
            std_hp = (sum((x - mean_hp)**2 for x in half_periods) / len(half_periods)) ** 0.5
            cv = std_hp / mean_hp if mean_hp > 0 else 999
        else:
            cv = 0.0
        noisy = cv > 0.40

        # ── 4. Amplitude from RAW centered data (NOT smoothed) ────────────────
        peaks = []
        for i in range(len(crossings)-1):
            t0, t1 = crossings[i], crossings[i+1]
            seg = [abs(centered[j]) for j in range(n) if t0 <= times[j] <= t1]
            if seg:
                peaks.append(max(seg))
        if peaks:
            peaks.sort()
            a = peaks[len(peaks) // 2]
        else:
            a = max_abs

        # ── 5. Ultimate gain Ku ───────────────────────────────────────────────
        h  = self._relay_amplitude
        Ku = (4.0 * h) / (_math.pi * a)

        # ── 6. Error & Jerk Metrics ───────────────────────────────────────────
        mace = sum(abs(e) for e in errors) / n
        sm = _median3(centered)
        velocities = []
        for i in range(1, n):
            dt = max(0.005, times[i] - times[i-1])
            velocities.append((sm[i] - sm[i-1]) / dt)
        accelerations = []
        for i in range(1, len(velocities)):
            dt = max(0.005, times[i+1] - times[i])
            accelerations.append((velocities[i] - velocities[i-1]) / dt)
        jerks = []
        for i in range(1, len(accelerations)):
            dt = max(0.005, times[i+2] - times[i+1])
            jerks.append((accelerations[i] - accelerations[i-1]) / dt)
        max_jerk = max(abs(j) for j in jerks) if jerks else 0.0

        diffs = [centered[i] - centered[i-1] for i in range(1, n)]
        sigma_noise = (sum(d**2 for d in diffs) / len(diffs) / 2) ** 0.5 if diffs else 0.1

        # ── Plausibility gates ────────────────────────────────────────────────
        tu_ok = 0.04 <= Tu <= 8.0
        ku_ok = 0.01 <= Ku <= 500000.0

        self._relay_log_line("", '')
        self._relay_log_line(f"  Relay amplitude  h  = {h}", 'head')
        self._relay_log_line(f"  Crossings used      = {len(crossings)}  (transient discarded)", 'dim')
        cv_tag = 'warn' if noisy else 'dim'
        self._relay_log_line(f"  Half-period CV      = {cv:.2f}  {'⚠ noisy' if noisy else '✓ consistent'}", cv_tag)
        self._relay_log_line(f"  Noise σ = {sigma_noise:.2f}  |  Noise gate = {noise_gate:.2f}", 'dim')
        self._relay_log_line(f"  Oscillation amp (a) = {a:.2f}  (raw median peak)", 'head')
        self._relay_log_line(f"  Period   Tu = {Tu:.3f} s  {'⚠' if not tu_ok else '✓'}", 'warn' if not tu_ok else 'head')
        self._relay_log_line(f"  Ult.gain Ku = {Ku:.4f}  {'⚠' if not ku_ok else '✓'}", 'warn' if not ku_ok else 'head')
        self._relay_log_line(f"  Mean Abs. Error     = {mace:.2f}", 'dim')
        self._relay_log_line(f"  Max Jerk            = {max_jerk:.1f} /s³", 'dim')
        self._relay_log_line("", '')

        if not tu_ok or not ku_ok:
            self.relay_status.config(text="⚠ Ku or Tu out of range — see log.", fg=C['red'])
            if not tu_ok:
                msg = "Tu too small — reduce h." if Tu < 0.04 else "Tu too large — increase base speed."
                self._relay_log_line(msg, 'warn')
            if not ku_ok:
                self._relay_log_line("Ku out of range — amplitude corrupted.", 'warn')
            self.relay_start_btn.config(state='normal', text="↻ Re-run")
            self.relay_pause_btn.config(state='disabled')
            self.relay_resume_btn.config(state='disabled')
            return

        if noisy:
            self._relay_log_line("Data noisy (CV>0.40) - verify on track.", 'warn')

        # ── 7. PD Gain Calculation ────────────────────────────────────────────
        # The STM32 firmware: correction = Kp * e + Kd * (e - e_prev)
        # WITHOUT dividing derivative by dt.
        #
        # From the relay test we measured Ku (ultimate gain) and Tu (ultimate period).
        # Ku is the gain at which the system just sustains oscillation.
        #
        # Key insight: Ku from relay already captures the full closed-loop dynamics
        # including the discrete sampling at 1kHz. No dt conversion is needed.
        #
        # The standard Z-N PD formula for a discrete controller is:
        #   Kp = 0.8 * Ku    (margin below instability)
        #   Kd = Kp * Tu / 8 (derivative time constant from Z-N tables)
        #
        # For this firmware, since Kd multiplies (e-e_prev) which at 1kHz changes
        # by tiny amounts each sample, the Kd value needs to be larger than Kp
        # to have meaningful damping. Empirically Kd/Kp ratio of 0.3-0.8 works
        # well for line followers (user baseline: Kp=10, Kd=5 → ratio=0.5).
        #
        # We use the relay-identified dynamics to set the Kd/Kp ratio based on
        # the oscillation frequency: faster oscillation needs more damping.

        wu = 2.0 * _math.pi / Tu  # ultimate angular frequency

        # Kp directly from Z-N ultimate gain method
        # Scale factor: the relay limit h may differ from operating PID limit
        # If user's PID limit >> h, the actual Ku would be proportionally higher
        try:
            user_pl = float(self._relay_saved.get('pl', h)) if self._relay_saved else float(h)
        except (ValueError, TypeError, AttributeError):
            user_pl = float(h)

        # Kp from Z-N with operating-point correction
        Kp_base = 0.80 * Ku

        # Kd: use Tu to set the derivative time constant
        # For discrete (e-e_prev) at 1kHz, a signal with period Tu changes by
        # approximately (2*pi*a/Tu)*dt per sample. To produce damping comparable
        # to Kp's restoring force, we want Kd * (de/sample) ~ fraction of Kp * e.
        # This gives Kd ~ Kp * a / (a * 2*pi*dt/Tu) * fraction = Kp * Tu / (2*pi*dt) * f
        # But empirically, for line followers, Kd/Kp ~ 0.3 to 0.8 works best.
        # We set it based on the natural frequency: higher wu → more damping needed.
        Kd_base = Kp_base * (Tu / 8.0)

        # Three presets with different margins
        Kp_agg = round(max(0.001, Kp_base * 1.25), 4)
        Kd_agg = round(max(0.001, Kd_base * 1.25), 4)

        Kp_bal = round(max(0.001, Kp_base), 4)
        Kd_bal = round(max(0.001, Kd_base), 4)

        Kp_con = round(max(0.001, Kp_base * 0.70), 4)
        Kd_con = round(max(0.001, Kd_base * 0.80), 4)

        self._relay_log_line(f"  Ku={Ku:.3f}  Tu={Tu:.3f}s  wu={wu:.2f} rad/s", 'dim')
        self._relay_log_line(f"  Kd/Kp ratio = {(Tu / 8.0):.3f}  (from Tu)", 'dim')
        self._relay_log_line("", '')
        self._relay_log_line(f"  Aggressive   Kp={Kp_agg}  Kd={Kd_agg}", 'good')
        self._relay_log_line(f"  Balanced     Kp={Kp_bal}  Kd={Kd_bal}  << Recommended", 'good')
        self._relay_log_line(f"  Conservative Kp={Kp_con}  Kd={Kd_con}", 'good')
        self._relay_log_line("", '')
        self.relay_status.config(text=f"Done (MACE:{mace:.1f} Jerk:{max_jerk:.0f})", fg=C['green'])

        for w in self.relay_result_frame.winfo_children():
            w.destroy()

        def _apply(kp, kd, label):
            self._relay_applied = True
            self.kp.set(str(kp)); self.ki.set("0.0"); self.kd.set(str(kd))
            self._send(f"PID:{kp},0.0,{kd}")
            
            # Restore saved base speed / limit and re-enable junction handling (JH1)
            if hasattr(self, '_relay_saved') and self._relay_saved is not None:
                s = self._relay_saved
                self._send(f"PARAM:BS{s['bs']}PL{s['pl']}JH1")
                self._relay_saved = None
            else:
                self._send("PARAM:JH1")
                
            self._relay_log_line(f"Applied {label}: Kp={kp}  Ki=0  Kd={kd} (Junctions enabled)", 'good')
            self.relay_status.config(text=f"✔ {label} applied!", fg=C['green'])

        # Metrics Row
        mr = tk.Frame(self.relay_result_frame, bg=C['bg'])
        mr.pack(fill='x', pady=(0, 4))
        tk.Label(mr, text=f"MACE: {mace:.2f}", fg=C['text'], bg=C['bg'],
                 font=("Segoe UI", 9, "bold")).pack(side='left', padx=(10, 12))
        tk.Label(mr, text=f"Jerk: {max_jerk:.1f}", fg=C['text'], bg=C['bg'],
                 font=("Segoe UI", 9, "bold")).pack(side='left', padx=(0, 12))
        tk.Label(mr, text=f"Ku={Ku:.3f}  Tu={Tu:.3f}s", fg=C['dim'], bg=C['bg'],
                 font=("Consolas", 8)).pack(side='left')

        # Buttons
        br = tk.Frame(self.relay_result_frame, bg=C['bg'])
        br.pack(fill='x', pady=(0, 2))
        tk.Button(br, text=f"Aggressive Kp={Kp_agg} Kd={Kd_agg}",
                  command=lambda: _apply(Kp_agg, Kd_agg, "Aggressive"),
                  bg=C['orange'], fg='#000', relief='flat',
                  font=("Consolas", 9, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(10, 3))
        tk.Button(br, text=f"Balanced Kp={Kp_bal} Kd={Kd_bal}",
                  command=lambda: _apply(Kp_bal, Kd_bal, "Balanced"),
                  bg=C['green'], fg='#000', relief='flat',
                  font=("Consolas", 9, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(br, text=f"Safe Kp={Kp_con} Kd={Kd_con}",
                  command=lambda: _apply(Kp_con, Kd_con, "Conservative"),
                  bg=C['cyan'], fg='#000', relief='flat',
                  font=("Consolas", 9, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 10))
        self.relay_start_btn.config(state='normal', text="↻ Re-run")
        self.relay_pause_btn.config(state='disabled')
        self.relay_resume_btn.config(state='disabled')

    def _restore_relay_pid(self):
        """Restore PID and params saved before relay test. Safe to call multiple times."""
        if hasattr(self, '_relay_saved') and self._relay_saved is not None:
            s = self._relay_saved
            # Only restore if they didn't apply a tuned result
            if not getattr(self, '_relay_applied', False):
                self._send(f"PID:{s['kp']},{s['ki']},{s['kd']}")
                self._send(f"PARAM:BS{s['bs']}PL{s['pl']}")
                self._relay_log_line("Original PID restored.", 'dim')
            self._relay_saved = None

    def _cancel_relay(self):
        """Abort relay test and restore saved PID."""
        self._relay_running = False
        self._restore_relay_pid()
        if hasattr(self, 'relay_win') and self.relay_win.winfo_exists():
            self.relay_win.destroy()

    # ═════════════════════════════════════════════════════════════════════════
    #  GRID SEARCH AUTO-TUNE  (brute-force fallback)
    # ═════════════════════════════════════════════════════════════════════════
    def _run_grid_search(self):
        """Brute-force grid-search auto-tuner (fallback to relay method).
        Evaluates (Kp, Ki, Kd) triples over a customizable window, scores by
        composite multi-variable metric (line loss, oscillation, saturation, recovery),
        and retains the best-scoring PID. Click *Stop & Keep Best* to apply.
        """
        if not self.running:
            messagebox.showwarning("Not connected", "Connect to the robot first.")
            return

        self.tune_win = tk.Toplevel(self.root)
        self.tune_win.title("Grid Search Auto-Tuning")
        self.tune_win.geometry("480x480")
        self.tune_win.configure(bg=C['bg'])
        self.tune_win.resizable(False, False)

        # Center the window relative to root
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - 240
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - 240
        self.tune_win.geometry(f"+{x}+{y}")
        self.tune_win.attributes('-topmost', True)

        tk.Label(self.tune_win, text="Grid Search Auto‑Tuning", 
                 fg=C['hl'], bg=C['bg'], font=("Segoe UI", 14, "bold")).pack(pady=(12, 2))
        tk.Label(self.tune_win, text="Kp/Kd sweep centered around UI values",
                 fg=C['dim'], bg=C['bg'], font=("Segoe UI", 9)).pack(pady=(0, 4))

        # Settings panel
        settings_frame = tk.Frame(self.tune_win, bg=C['bg'])
        settings_frame.pack(pady=4)

        tk.Label(settings_frame, text="Eval Time (s):", fg=C['text'], bg=C['bg'], font=("Segoe UI", 9)).grid(row=0, column=0, padx=5, sticky='w')
        self.grid_time_var = tk.DoubleVar(value=3.0)
        self.grid_time_entry = tk.Entry(settings_frame, textvariable=self.grid_time_var, width=5, bg=C['card2'], fg=C['text'], relief='flat')
        self.grid_time_entry.grid(row=0, column=1, padx=5, sticky='w')

        self.grid_stop_go_var = tk.BooleanVar(value=False)
        self.grid_stop_go_cb = tk.Checkbutton(
            settings_frame, text="Stop-and-Go (Manual Reset)", variable=self.grid_stop_go_var,
            bg=C['bg'], fg=C['text'], selectcolor=C['card'], activebackground=C['bg'], activeforeground=C['text'],
            font=("Segoe UI", 9)
        )
        self.grid_stop_go_cb.grid(row=0, column=2, padx=15, sticky='w')

        # Run action frame (will contain sweep button or manual test button)
        self.action_frame = tk.Frame(self.tune_win, bg=C['bg'])
        self.action_frame.pack(pady=4)

        self.start_sweep_btn = tk.Button(self.action_frame, text="⚡ Start Grid Sweep", command=self._start_grid_sweep,
                                         bg=C['purple'], fg='#fff', font=("Segoe UI", 10, "bold"), cursor='hand2')
        self.start_sweep_btn.pack(pady=2)

        self.run_test_btn = tk.Button(self.action_frame, text="▶ Run Test (Combo 1)", command=self._run_stop_go_test,
                                       bg=C['cyan'], fg='#000', font=("Segoe UI", 10, "bold"), cursor='hand2')
        # hidden until started in Stop-and-Go Mode

        self.tune_status_lbl = tk.Label(self.tune_win, text="Configure settings and start sweep.", 
                 fg=C['orange'], bg=C['bg'], font=("Segoe UI", 10, "bold"))
        self.tune_status_lbl.pack(pady=(4, 2))

        self.tune_progress = ttk.Progressbar(self.tune_win, orient='horizontal', length=420, mode='determinate')
        self.tune_progress.pack(pady=(0, 6))

        self.tune_info = tk.Text(self.tune_win, bg=C['card'], fg=C['text'],
                                 font=("Consolas", 9), height=10, width=54,
                                 relief='flat', padx=8, pady=6)
        self.tune_info.pack(pady=5, padx=10)
        self.tune_info.insert('end', "Ready to search.\n")
        self.tune_info.config(state='disabled')

        btn_frame = tk.Frame(self.tune_win, bg=C['bg'])
        btn_frame.pack(fill='x', pady=8, padx=20)
        tk.Button(btn_frame, text="Stop & Keep Best", command=self._finish_grid,
                  bg=C['green'], fg='#000', font=("Segoe UI", 10, "bold"), cursor='hand2').pack(side='left', expand=True, fill='x', padx=5)
        tk.Button(btn_frame, text="Cancel", command=self._cancel_grid,
                  bg=C['red'], fg='#fff', font=("Segoe UI", 10, "bold"), cursor='hand2').pack(side='right', expand=True, fill='x', padx=5)

        self.grid_running = False
        self.best_err = float('inf')
        self.best_pid = None
        self.grid_phase = 'coarse'
        self.combos_done = 0

    def _start_grid_sweep(self):
        """Configure parameters and build search grid list."""
        self.grid_time_entry.config(state='disabled')
        self.grid_stop_go_cb.config(state='disabled')
        self.start_sweep_btn.pack_forget()

        try:
            self.grid_eval_time = float(self.grid_time_var.get())
        except ValueError:
            self.grid_eval_time = 3.0

        try:
            base_kp = float(self.kp.get())
            base_ki = float(self.ki.get())
            base_kd = float(self.kd.get())
        except ValueError:
            base_kp, base_ki, base_kd = 1.0, 0.0, 0.5

        # Save original PID and parameters
        self._grid_orig_pid = (base_kp, base_ki, base_kd)
        try:
            self._grid_orig_bs = int(float(self.base_spd.get()))
            self._grid_orig_pl = int(float(self.pid_lim.get()))
        except ValueError:
            self._grid_orig_bs, self._grid_orig_pl = 500, 300

        kp_lo   = max(0.5, round(base_kp * 0.25, 2))
        kp_hi   = round(base_kp * 2.5, 2)
        kp_step = round((kp_hi - kp_lo) / 7, 2)
        kp_vals = sorted(set(
            [round(kp_lo + i * kp_step, 2) for i in range(8)] + [round(base_kp, 2)]
        ))
        ki_vals = [0.0]
        kd_vals = [0, 2, 5, 10, 20, 40, 60, 80, 100]
        
        raw_grid = [(kp, ki, kd) for kp in kp_vals for ki in ki_vals for kd in kd_vals]
        raw_grid.sort(key=lambda t: abs(t[0] - base_kp))
        self.grid = raw_grid
        self._grid_total = len(self.grid)
        self.grid_index = 0
        self.best_err = float('inf')
        self.best_pid = None
        self.grid_running = True
        self.grid_phase = 'coarse'
        self.combos_done = 0

        self._append_tune_info(f"Sweep started: {self._grid_total} combinations.\n")

        if self.grid_stop_go_var.get():
            self.run_test_btn.pack(pady=2)
            self.run_test_btn.config(state='normal', text="▶ Run Test (Combo 1)")
            self.tune_status_lbl.config(text="Ready. Place robot at start and click 'Run Test'", fg=C['cyan'])
        else:
            self._grid_search_step()

    def _run_stop_go_test(self):
        """User clicked 'Run Test' in Stop-and-Go Mode."""
        self.run_test_btn.config(state='disabled')
        self._grid_search_step()

    def _grid_search_step(self):
        """Iterate through the grid, testing one PID triple at a time."""
        if not self.grid_running:
            return
        if self.grid_index >= len(self.grid):
            if self.grid_phase == 'coarse':
                if not self.best_pid:
                    self.tune_status_lbl.config(text="Coarse search failed.", fg=C['red'])
                    return
                # Setup fine phase
                self.grid_phase = 'fine'
                best_kp, best_ki, best_kd = self.best_pid
                best_ki = 0.0
                kp_step = max(0.5, round(best_kp * 0.10, 2))
                kd_step = max(0.5, round(best_kd * 0.10, 2))
                kp_vals = [max(0.0, round(best_kp - kp_step + i * kp_step, 2)) for i in range(5)]
                ki_vals = [0.0]
                kd_vals = [max(0.0, round(best_kd - kd_step + i * kd_step, 2)) for i in range(5)]
                self.grid = [(kp, ki, kd) for kp in kp_vals for ki in ki_vals for kd in kd_vals]
                self._grid_total = len(self.grid)
                self.grid_index = 0
                self._append_tune_info(f"\n--- Starting Fine Search ---\n")
                if self.grid_stop_go_var.get():
                    self.tune_status_lbl.config(text="Coarse done. Ready for Fine search.", fg=C['cyan'])
                    self.run_test_btn.config(state='normal', text=f"▶ Run Test (Combo 1/{self._grid_total})")
                    return
            else:
                self.tune_status_lbl.config(text="Search complete.", fg=C['green'])
                self._append_tune_info("\nGrid search finished. Use \"Stop & Keep Best\" to apply.\n")
                if self.grid_stop_go_var.get():
                    self.run_test_btn.pack_forget()
                return

        kp, ki, kd = self.grid[self.grid_index]
        self.tune_status_lbl.config(text=f"Testing ({self.grid_phase}) {self.grid_index+1}/{len(self.grid)}", fg=C['orange'])
        
        # Send params (fixed speed + JH0 to disable junction handling)
        try:
            bs = int(float(self.base_spd.get()))
            pl = int(float(self.pid_lim.get()))
        except ValueError:
            bs, pl = 500, 300
        self._send(f"PARAM:BS{bs}PL{pl}JH0")
        
        # Send PID to robot
        self._send(f"PID:{kp:.2f},{ki:.3f},{kd:.2f}")
        
        # Start robot
        self._send("START")
        
        # Prepare evaluation window
        self.eval_data = []
        # Allow 450ms for settling transient response
        self.root.after(450, self._start_grid_collect)

    def _start_grid_collect(self):
        if not self.grid_running:
            return
        self.eval_start = time.time()
        self._grid_collect()

    def _grid_collect(self):
        """Collect telemetry for the current PID during the evaluation window."""
        if not self.grid_running:
            return
        elapsed = time.time() - self.eval_start
        if elapsed < self.grid_eval_time:
            with self.lock:
                if len(self.err) > 0:
                    self.eval_data.append({
                        't': self.t[-1],
                        'err': self.err[-1],
                        'corr': self.corr[-1],
                        'left': self.lspd[-1] if self.lspd else 0,
                        'right': self.rspd[-1] if self.rspd else 0,
                        'ir': list(self.ir_snapshot) if self.ir_snapshot else [0]*8
                    })
            self.root.after(30, self._grid_collect)
        else:
            # STOP robot immediately at the end of the evaluation window
            self._send("STOP")
            
            # Compute composite score
            score = self._compute_composite_score(self.eval_data)
                
            kp, ki, kd = self.grid[self.grid_index]
            self._append_tune_info(f"Kp={kp:.2f} Ki={ki:.3f} Kd={kd:.2f} → score={score:.2f}\n")
            
            if score < self.best_err:
                self.best_err = score
                self.best_pid = self.grid[self.grid_index]
                
            # Move to next combo
            self.grid_index += 1
            self.combos_done += 1
            
            pct = min(100.0, (self.combos_done / max(1, getattr(self, '_grid_total', 81))) * 100.0)
            self.tune_progress['value'] = pct
            
            if not self.grid_stop_go_var.get():
                self.root.after(10, self._grid_search_step)
            else:
                self.tune_status_lbl.config(
                    text=f"Combo {self.grid_index} done. Reset robot and click 'Run Test'", 
                    fg=C['cyan']
                )
                next_lbl = f"▶ Run Test (Combo {self.grid_index+1}/{self._grid_total})"
                self.run_test_btn.config(state='normal', text=next_lbl)

    def _compute_composite_score(self, eval_data):
        """Compute Composite Score based on line loss, oscillation, saturation, recovery time, and MACE."""
        N = len(eval_data)
        if N == 0:
            return float('inf')
        if N == 1:
            return eval_data[0]['err']**2

        # 1. MACE Penalty (Mean Absolute Cross-Track Error)
        mace = sum(abs(s['err']) for s in eval_data) / N
        mace_penalty = mace * 100.0

        # 2. Oscillation Penalty (Error variation + Control jitter)
        err_diffs = [abs(eval_data[i+1]['err'] - eval_data[i]['err']) for i in range(N-1)]
        corr_diffs = [abs(eval_data[i+1]['corr'] - eval_data[i]['corr']) for i in range(N-1)]
        
        oscillation_penalty = (sum(err_diffs) / (N-1)) * 50.0
        control_jitter = (sum(corr_diffs) / (N-1)) * 10.0

        # 3. Saturation Penalty (Fraction of time correction was at limit)
        try:
            limit = float(self.pid_lim.get())
        except ValueError:
            limit = 300.0
        sat_count = sum(1 for s in eval_data if abs(s['corr']) >= limit * 0.95)
        saturation_penalty = (sat_count / N) * 15000.0

        # 4. Line Loss Penalty (Fraction of time robot lost the line)
        # SENSOR_THRESHOLD is 2048. Since HIGH = Black, losing the line means all sensors read < 2048.
        loss_count = sum(1 for s in eval_data if all(v < 2048 for v in s['ir']) or abs(s['err']) >= 120.0)
        line_loss_penalty = (loss_count / N) * 200000.0

        # 5. Recovery Time Penalty
        # Find the last time the error exceeded a settling band of 15.0
        settle_band = 15.0
        last_out_idx = -1
        for idx, s in enumerate(eval_data):
            if abs(s['err']) > settle_band:
                last_out_idx = idx

        if last_out_idx == -1:
            recovery_time = 0.0
        elif last_out_idx == N - 1:
            recovery_time = eval_data[-1]['t'] - eval_data[0]['t']
        else:
            recovery_time = eval_data[last_out_idx]['t'] - eval_data[0]['t']
        recovery_penalty = recovery_time * 500.0

        # Total composite score
        score = mace_penalty + oscillation_penalty + control_jitter + saturation_penalty + line_loss_penalty + recovery_penalty
        return score

    def _append_tune_info(self, text):
        if not hasattr(self, 'tune_win') or not self.tune_win.winfo_exists():
            return
        self.tune_info.config(state='normal')
        self.tune_info.insert('end', text)
        self.tune_info.see('end')
        self.tune_info.config(state='disabled')

    def _finish_grid(self):
        """Finalize grid search, apply the best PID found, and close the tuner window."""
        self.grid_running = False
        
        # Restore original parameters with JH1 (restore junction handling)
        if hasattr(self, '_grid_orig_bs') and hasattr(self, '_grid_orig_pl'):
            self._send(f"PARAM:BS{self._grid_orig_bs}PL{self._grid_orig_pl}JH1")
        else:
            self._send("PARAM:JH1")

        if self.best_pid:
            kp, ki, kd = self.best_pid
            self.kp.set(str(kp))
            self.ki.set(str(ki))
            self.kd.set(str(kd))
            self._send(f"PID:{kp:.2f},{ki:.3f},{kd:.2f}")
            self._append_tune_info(f"\nBest PID applied: Kp={kp:.2f}, Ki={ki:.3f}, Kd={kd:.2f}\n")
        else:
            self._append_tune_info("\nNo PID configuration improved error metric; original values retained.\n")
        self.tune_win.destroy()

    def _cancel_grid(self):
        """Abort the grid search and close the window without changing PID."""
        self.grid_running = False
        
        # Restore original parameters with JH1 (restore junction handling)
        if hasattr(self, '_grid_orig_bs') and hasattr(self, '_grid_orig_pl'):
            self._send(f"PARAM:BS{self._grid_orig_bs}PL{self._grid_orig_pl}JH1")
        else:
            self._send("PARAM:JH1")

        if hasattr(self, '_grid_orig_pid'):
            kp, ki, kd = self._grid_orig_pid
            self.kp.set(str(kp))
            self.ki.set(str(ki))
            self.kd.set(str(kd))
            self._send(f"PID:{kp:.2f},{ki:.3f},{kd:.2f}")

        self.tune_win.destroy()

    # ═════════════════════════════════════════════════════════════════════════
    #  DATA TOOLS
    # ═════════════════════════════════════════════════════════════════════════
    def _toggle_rec(self):
        self.recording = not self.recording
        if self.recording:
            self.rec_data = []
            self.rec_btn.config(text="⏹ Stop Rec", bg=C['red'])
        else:
            self.rec_btn.config(text="⏺ Record", bg=C['purple'])

    def _export(self):
        data = self.rec_data if self.rec_data else None
        if not data:
            # Export current buffer
            with self.lock:
                if not self.t:
                    messagebox.showinfo("No data", "No data to export.")
                    return
                data = []
                for i in range(len(self.t)):
                    data.append({
                        'time': self.t[i], 'error': self.err[i],
                        'correction': self.corr[i],
                        'left': self.lspd[i], 'right': self.rspd[i],
                        'battery': self.batt[i], 'ir': '',
                    })

        path = filedialog.asksaveasfilename(
            defaultextension='.csv',
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"pid_data_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['time', 'error', 'correction',
                                               'left', 'right', 'battery', 'ir'])
            w.writeheader()
            w.writerows(data)
        messagebox.showinfo("Exported", f"Saved {len(data)} rows to:\n{path}")

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(
            text="▶ Resume" if self.paused else "⏸ Pause Plot",
            bg=C['green'] if self.paused else C['accent'])

    def _toggle_ir_flip(self):
        self.ir_flip = not self.ir_flip
        self.flip_btn.config(
            text="⇄ Flip IR  [ON]"  if self.ir_flip else "⇄ Flip IR Order",
            bg=C['orange']          if self.ir_flip else C['accent'],
            fg='#000'               if self.ir_flip else C['text'])

    def _clear_data(self):
        with self.lock:
            for d in (self.t, self.err, self.corr, self.sp,
                      self.lspd, self.rspd, self.batt):
                d.clear()
            self.new_data = True
        self.t0 = time.time()

    # ═════════════════════════════════════════════════════════════════════════
    #  JUNCTION UI
    # ═════════════════════════════════════════════════════════════════════════
    def _update_junction_ui(self, junc):
        """Update the junction detection card (called from main thread)."""
        icons  = {JUNC_NONE: '⛌', JUNC_LEFT: '↰', JUNC_RIGHT: '↱',
                  JUNC_T: '⊤', JUNC_CROSS: '✚'}
        colors = {JUNC_NONE: C['dim'], JUNC_LEFT: C['blue'],
                  JUNC_RIGHT: C['green'], JUNC_T: C['orange'],
                  JUNC_CROSS: C['purple']}
        tags   = {JUNC_LEFT: 'left', JUNC_RIGHT: 'right',
                  JUNC_T: 'tjunc', JUNC_CROSS: 'tjunc'}

        color = colors[junc]
        self.junc_icon_lbl.config(text=icons[junc], fg=color)
        self.junc_type_lbl.config(text=JUNC_NAMES[junc], fg=color)

        # Debounced counting
        now = time.time()
        if junc != JUNC_NONE and (junc != self.junc_last_type or
                                   now - self.junc_last_time > JUNC_DEBOUNCE_S):
            self.junc_ever_entered = True
            if junc == JUNC_LEFT:
                self.junc_counts['left'] += 1
            elif junc == JUNC_RIGHT:
                self.junc_counts['right'] += 1
            elif junc == JUNC_T:
                self.junc_counts['t'] += 1
            self.junc_last_time = now

            # Add to history
            ts = datetime.now().strftime('%H:%M:%S')
            tag = tags.get(junc, '')
            self.junc_history.insert(0, (ts, JUNC_NAMES[junc], tag))
            if len(self.junc_history) > 30:
                self.junc_history = self.junc_history[:30]
            self._refresh_junc_history()

        self.junc_last_type = junc

        # Update entered label
        if self.junc_ever_entered:
            total = sum(self.junc_counts.values())
            self.junc_entered_lbl.config(text=f"{total} junction(s) detected",
                                          fg=C['purple'])
        else:
            self.junc_entered_lbl.config(text="no junction entered yet",
                                          fg=C['dim'])

        # Update counters
        total = sum(self.junc_counts.values())
        self.junc_count_lbls['total'].config(text=str(total))
        self.junc_count_lbls['left'].config(text=str(self.junc_counts['left']))
        self.junc_count_lbls['right'].config(text=str(self.junc_counts['right']))
        self.junc_count_lbls['t'].config(text=str(self.junc_counts['t']))

    def _refresh_junc_history(self):
        """Rebuild the history text box."""
        box = self.junc_hist_box
        box.config(state='normal')
        box.delete('1.0', 'end')
        for ts, name, tag in self.junc_history:
            box.insert('end', f'[{ts}] ', 'time')
            box.insert('end', f'{name}\n', tag if tag else None)
        box.config(state='disabled')

    # ═════════════════════════════════════════════════════════════════════════
    #  PERIODIC UPDATE
    # ═════════════════════════════════════════════════════════════════════════
    def _tick(self):
        with self.lock:
            has_data = self.new_data
            if has_data and not self.paused:
                t  = list(self.t)
                er = list(self.err)
                co = list(self.corr)
                sp = list(self.sp)
                ls = list(self.lspd)
                rs = list(self.rspd)
                # Apply flip here so it's in one place
                ir = list(reversed(self.ir_snapshot)) if self.ir_flip else list(self.ir_snapshot)
                self.new_data = False

        if has_data and not self.paused:
            # ── Error + Correction (dual axis) ───────────────────────────────
            self.line_err.set_data(t, er)
            self.line_sp.set_data(t, sp)
            self.line_corr.set_data(t, co)

            # Compute Y-limits directly from data (relim+autoscale is broken on twinx)
            if er:
                e_lo, e_hi = min(min(er), 0), max(max(er), 0)
                pad = max((e_hi - e_lo) * 0.1, 4)  # 10% padding, min ±4
                self.ax1.set_ylim(e_lo - pad, e_hi + pad)
            if co:
                c_lo, c_hi = min(min(co), 0), max(max(co), 0)
                pad = max((c_hi - c_lo) * 0.1, 50)  # 10% padding, min ±50
                self.ax1_r.set_ylim(c_lo - pad, c_hi + pad)
            # Shared X-axis for both time-series charts
            if t:
                self.ax1.set_xlim(t[0], t[-1])
                self.ax2.set_xlim(t[0], t[-1])

            # ── Motor speeds ─────────────────────────────────────────────────
            self.line_l.set_data(t, ls)
            self.line_r.set_data(t, rs)
            if ls and rs:
                all_spd = ls + rs
                s_lo, s_hi = min(all_spd), max(all_spd)
                pad = max((s_hi - s_lo) * 0.1, 20)
                self.ax2.set_ylim(s_lo - pad, s_hi + pad)

            # ── IR sensor bars (always 8 elements) ───────────────────────────
            for i, (bar, val) in enumerate(zip(self.ir_bars, ir)):
                bar.set_height(val)
                # LOW ADC = line detected (dark tape absorbs IR)
                # HIGH ADC = off line (reflective surface)
                # Flip colour so detected sensor glows red
                ratio = val / 4095.0
                bar.set_color(C['hl'] if ratio > 0.55 else C['cyan'])

            self.canvas.draw_idle()

        # Update packet-rate label
        self.rate_lbl.config(text=f"{self._pkt_rate:.0f} pkt/s")

        self.root.after(PLOT_INTERVAL_MS, self._tick)

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == '__main__':
    print("Starting PID Tuner Dashboard...")
    root = tk.Tk()
    app = PIDTuner(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    # Force window to front on Windows
    root.lift()
    root.attributes('-topmost', True)
    root.after(100, lambda: root.attributes('-topmost', False))
    root.focus_force()
    print("Dashboard window opened. If you don't see it, check your taskbar.")
    root.mainloop()