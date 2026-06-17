#!/usr/bin/env python3

"""

CNC Controller GUI v2.7.2 - GRBL v1.1 Hybrid + Tick Straight + M33 Pulse
(compatible with firmware grbl_cnc v6.4.7+M33)

Changes from v2.7.0:
- Tick Straight pulse now uses firmware-timed command: M33 D<ms>
- Parse custom status fields: Sp: (spindle/laser) and LP: (pulse active, remaining ms)
- Handle firmware async messages: [PULSE DONE] and [PULSE ABORTED]
- Emergency/Alarm paths now reset GUI laser runtime flags harder
- Tick Straight stage text now reflects Pulse -> Wait -> Move -> Settle -> Check flow

Changes from v2.5.3:
- GRBL v1.1 status format parsing
  <Idle|MPos:x,y,z|FS:f,s|Bf:b,r|Pn:XYZ|H:111>
- Realtime commands: ?, !, ~, 0x18 (Ctrl-X) sent as single bytes
- Config read uses OK-terminated stream (no === decoration scan)
- Settings renumbered to match firmware v6.4.2:
    * $26 = Homing Debounce (GRBL standard)
    * $29 = Homing Axis Mask (custom, was $26)
    * $40 = Report Interval (custom, was $32 - now laser mode)
- New settings: $0 step pulse, $1 step idle, $11 junction deviation
- Auto-detect Grbl 1.1f welcome string (request status on handshake)
- Periodic ? status polling every 250ms
- Emergency STOP uses soft-reset 0x18 + STOP text + M5

"""


import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import serial
import serial.tools.list_ports
import threading
import json
import re
import queue
from datetime import datetime
import time
import math


# ============================================================================
#                         TOOLTIP
# ============================================================================


class ToolTip:
    def __init__(self, widget, text, delay=500):
        self.widget = widget
        self.text = text
        self.delay = delay
        self.tip_window = None
        self.scheduled = None
        widget.bind('<Enter>', self._on_enter)
        widget.bind('<Leave>', self._on_leave)
        widget.bind('<Button>', self._on_leave)

    def _on_enter(self, event=None):
        self._cancel()
        self.scheduled = self.widget.after(self.delay, self._show)

    def _on_leave(self, event=None):
        self._cancel()
        self._hide()

    def _cancel(self):
        if self.scheduled:
            self.widget.after_cancel(self.scheduled)
            self.scheduled = None

    def _show(self):
        if self.tip_window:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify=tk.LEFT,
                        background="#ffffdd", foreground="#333",
                        relief=tk.SOLID, borderwidth=1,
                        font=("Segoe UI", 9), padx=6, pady=3)
        label.pack()

    def _hide(self):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None


def add_tooltip(widget, text):
    return ToolTip(widget, text)


# ============================================================================
#                         SERIAL MANAGER
# ============================================================================


class SerialManager:
    def __init__(self):
        self.serial_port = None
        self.is_connected = False
        self.rx_thread = None
        self.running = False
        self.rx_queue = queue.Queue()

    def get_available_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=0.1, write_timeout=1)
            self.is_connected = True
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            return True
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            return False

    def disconnect(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
        self.serial_port = None

    def send(self, data):
        if self.is_connected and self.serial_port:
            try:
                self.serial_port.write((data + '\n').encode('utf-8'))
                return True
            except:
                return False
        return False

    def send_realtime(self, byte_val):
        """Send a single byte WITHOUT newline (GRBL realtime: ?, !, ~, 0x18)."""
        if self.is_connected and self.serial_port:
            try:
                if isinstance(byte_val, str):
                    self.serial_port.write(byte_val.encode('utf-8'))
                else:
                    self.serial_port.write(bytes([byte_val]))
                return True
            except:
                return False
        return False

    def _rx_loop(self):
        buffer = ""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    raw = self.serial_port.read(self.serial_port.in_waiting)
                    decoded = raw.decode('utf-8', errors='ignore')
                    buffer += decoded
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        stripped = line.strip()
                        if stripped:
                            self.rx_queue.put(stripped)
                    # Handle leftover data without newline (partial line)
                    if len(buffer) > 1024:  # safety limit
                        if buffer.strip():
                            self.rx_queue.put(buffer.strip())
                        buffer = ""
                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)


# ============================================================================
#                         XY VISUALIZER (No Trail)
# ============================================================================


class XYVisualizer(tk.Canvas):
    """Simple XY position visualizer without trail"""

    def __init__(self, parent, size=180, max_travel=200):
        super().__init__(parent, width=size, height=size, bg='#1a1a2e', highlightthickness=1,
                        highlightbackground='#333')
        self.size = size
        self.max_travel = max_travel
        self.pos_x = 0
        self.pos_y = 0

        self._draw_grid()
        self.marker = None
        self.crosshair_h = None
        self.crosshair_v = None
        self.coord_text = None

    def _draw_grid(self):
        margin = 15
        inner = self.size - 2 * margin

        # Grid lines
        for i in range(5):
            x = margin + i * inner / 4
            y = margin + i * inner / 4
            self.create_line(x, margin, x, self.size - margin, fill='#2d2d44', width=1)
            self.create_line(margin, y, self.size - margin, y, fill='#2d2d44', width=1)

        # Axes
        self.create_line(margin, self.size - margin, self.size - margin, self.size - margin,
                        fill='#e53935', width=2)  # X
        self.create_line(margin, margin, margin, self.size - margin,
                        fill='#43a047', width=2)  # Y

        # Labels
        self.create_text(self.size - margin, self.size - 5, text="X", fill='#e53935', font=("Arial", 8, "bold"))
        self.create_text(5, margin, text="Y", fill='#43a047', font=("Arial", 8, "bold"))

        # Origin
        self.create_oval(margin-3, self.size-margin-3, margin+3, self.size-margin+3,
                        fill='#ffd700', outline='')

    def update_position(self, x, y):
        self.pos_x = x
        self.pos_y = y
        self._redraw()

    def _redraw(self):
        margin = 15
        inner = self.size - 2 * margin

        # Clear old elements
        if self.marker:
            self.delete(self.marker)
        if self.crosshair_h:
            self.delete(self.crosshair_h)
        if self.crosshair_v:
            self.delete(self.crosshair_v)
        if self.coord_text:
            self.delete(self.coord_text)

        # Calculate screen position
        screen_x = margin + (self.pos_x / self.max_travel) * inner
        screen_y = self.size - margin - (self.pos_y / self.max_travel) * inner

        # Clamp
        screen_x = max(margin, min(self.size - margin, screen_x))
        screen_y = max(margin, min(self.size - margin, screen_y))

        # Crosshairs
        self.crosshair_h = self.create_line(margin, screen_y, self.size - margin, screen_y,
                                            fill='#555', dash=(2, 2))
        self.crosshair_v = self.create_line(screen_x, margin, screen_x, self.size - margin,
                                            fill='#555', dash=(2, 2))

        # Marker
        r = 6
        self.marker = self.create_oval(screen_x - r, screen_y - r, screen_x + r, screen_y + r,
                                       fill='#00e676', outline='white', width=2)

        # Coordinates text
        self.coord_text = self.create_text(self.size / 2, 10,
                                           text=f"X:{self.pos_x:.1f} Y:{self.pos_y:.1f}",
                                           fill='#aaa', font=("Consolas", 8))


# ============================================================================
#                         ACCELERATION VISUALIZER (Multi-Axis)
# ============================================================================


class AccelVisualizer(tk.Canvas):
    """Trapezoidal acceleration profile visualizer - supports X/Y/Z"""

    def __init__(self, parent, width=300, height=120):
        super().__init__(parent, width=width, height=height, bg='#f5f5f5',
                        highlightthickness=1, highlightbackground='#ddd')
        self.width = width
        self.height = height

        # Current axis parameters
        self.axis_name = "X"
        self.accel = 100
        self.max_speed = 3000
        self.distance = 50

        self._draw()

    def update_params(self, axis_name="X", accel=100, max_speed=3000, distance=50):
        """Update with specific axis parameters"""
        self.axis_name = axis_name
        self.accel = max(1, accel)
        self.max_speed = max(1, max_speed)
        self.distance = max(0.1, distance)
        self._draw()

    def _draw(self):
        self.delete("all")

        margin_x = 40
        margin_y = 25
        w = self.width - 2 * margin_x
        h = self.height - 2 * margin_y

        # Axis colors
        axis_colors = {"X": "#e53935", "Y": "#43a047", "Z": "#1e88e5"}
        color = axis_colors.get(self.axis_name, "#2196f3")

        # Title
        self.create_text(self.width / 2, 10, text=f"{self.axis_name} Axis Profile",
                        font=("Segoe UI", 9, "bold"), fill=color)

        # Axes
        self.create_line(margin_x, margin_y, margin_x, self.height - margin_y, fill='#333', width=1)
        self.create_line(margin_x, self.height - margin_y, self.width - margin_x,
                        self.height - margin_y, fill='#333', width=1)

        # Labels
        self.create_text(margin_x - 20, margin_y + h/2, text="V", font=("Segoe UI", 8), fill='#666')
        self.create_text(margin_x + w/2, self.height - 8, text="Distance", font=("Segoe UI", 8), fill='#666')

        # Calculate profile
        v_max = self.max_speed / 60  # mm/s
        a = self.accel  # mm/s²
        d = self.distance  # mm

        if a <= 0 or v_max <= 0:
            return

        # Time and distance to reach max speed
        t_accel = v_max / a
        d_accel = 0.5 * a * t_accel ** 2

        # Check profile type
        if 2 * d_accel > d:
            # Triangle profile - can't reach max speed
            d_accel = d / 2
            v_peak = math.sqrt(2 * a * d_accel)
            d_cruise = 0
            profile_type = "Triangle"
        else:
            # Trapezoid profile
            v_peak = v_max
            d_cruise = d - 2 * d_accel
            profile_type = "Trapezoid"

        # Normalize for drawing
        x1 = margin_x  # Start
        x2 = margin_x + (d_accel / d) * w  # End accel
        x3 = margin_x + ((d_accel + d_cruise) / d) * w  # Start decel
        x4 = margin_x + w  # End

        y_bottom = self.height - margin_y
        y_top = margin_y + h * (1 - v_peak / v_max) if v_max > 0 else margin_y + h

        # Ensure y_top doesn't go above margin
        y_top = max(margin_y, y_top)

        # Draw filled profile
        points = [x1, y_bottom, x2, y_top, x3, y_top, x4, y_bottom]

        # Fill color based on axis
        fill_colors = {"X": "#ffebee", "Y": "#e8f5e9", "Z": "#e3f2fd"}
        fill_color = fill_colors.get(self.axis_name, "#e3f2fd")

        self.create_polygon(points, fill=fill_color, outline=color, width=2)

        # Peak speed annotation
        self.create_text((x2 + x3) / 2, y_top - 12, text=f"{v_peak*60:.0f} mm/min",
                        font=("Segoe UI", 8, "bold"), fill=color)

        # Profile type
        self.create_text(self.width - margin_x - 5, margin_y + 5, text=profile_type,
                        font=("Segoe UI", 8), fill='#888', anchor=tk.NE)

        # Phase labels
        if d_accel > 0:
            self.create_text((x1 + x2) / 2, y_bottom + 10, text="Accel",
                            font=("Segoe UI", 7), fill='#4caf50')
        if d_cruise > 0:
            self.create_text((x2 + x3) / 2, y_bottom + 10, text="Cruise",
                            font=("Segoe UI", 7), fill='#2196f3')
        self.create_text((x3 + x4) / 2, y_bottom + 10, text="Decel",
                        font=("Segoe UI", 7), fill='#f44336')

        # Accel value
        self.create_text(margin_x + 5, margin_y + 5, text=f"a={self.accel} mm/s\u00b2",
                        font=("Segoe UI", 7), fill='#666', anchor=tk.NW)


# ============================================================================
#                         RECT PATTERN PREVIEW
# ============================================================================


class RectPatternPreview(tk.Canvas):
    """GOWDELAYRECT pattern preview"""

    def __init__(self, parent, size=150):
        super().__init__(parent, width=size, height=size, bg='#fafafa',
                        highlightthickness=1, highlightbackground='#ddd')
        self.size = size
        self.width_mm = 50
        self.height_mm = 50
        self.step = 10

        self._draw()

    def update_params(self, width_mm, height_mm, step):
        self.width_mm = max(1, width_mm)
        self.height_mm = max(1, height_mm)
        self.step = max(0.1, step)
        self._draw()

    def _draw(self):
        self.delete("all")

        margin = 20
        inner = self.size - 2 * margin

        max_dim = max(self.width_mm, self.height_mm)
        scale = inner / max_dim

        rect_w = self.width_mm * scale
        rect_h = self.height_mm * scale

        offset_x = margin + (inner - rect_w) / 2
        offset_y = margin + (inner - rect_h) / 2

        # Boundary
        self.create_rectangle(offset_x, offset_y, offset_x + rect_w, offset_y + rect_h,
                             outline='#ccc', dash=(3, 3))

        # Serpentine pattern
        num_rows = int(self.height_mm / self.step) + 1
        row_height = rect_h / max(1, num_rows - 1) if num_rows > 1 else 0

        points = []
        for i in range(min(num_rows, 20)):
            y = offset_y + i * row_height
            if i % 2 == 0:
                points.extend([offset_x, y, offset_x + rect_w, y])
            else:
                points.extend([offset_x + rect_w, y, offset_x, y])

        if len(points) >= 4:
            self.create_line(points, fill='#2196f3', width=2, arrow=tk.LAST)

        # Start marker
        self.create_oval(offset_x - 4, offset_y - 4, offset_x + 4, offset_y + 4,
                        fill='#4caf50', outline='white')
        self.create_text(offset_x, offset_y - 12, text="Start", font=("Segoe UI", 7), fill='#4caf50')

        # Info
        self.create_text(self.size / 2, self.size - 8,
                        text=f"{self.width_mm:.0f}\u00d7{self.height_mm:.0f}mm",
                        font=("Segoe UI", 8), fill='#666')


# ============================================================================
#                         CONFIG WINDOW
# ============================================================================


class ConfigWindow(tk.Toplevel):
    # GRBL v1.1 standard + custom extensions (firmware v6.4.2)
    CONFIG_PARAMS = [
        # --- GRBL v1.1 standard ---
        ("$0",  "Step Pulse (us)",    "\u0110\u1ed9 r\u1ed9ng xung step (us)", 3),
        ("$1",  "Step Idle (ms)",     "Tr\u1ec5 t\u1eaft motor (ms, 255=lu\u00f4n on)", 25),
        ("$2",  "Step Invert Mask",   "Bit mask \u0111\u1ea3o step (0-7)", 0),
        ("$3",  "Dir Invert Mask",    "Bit mask \u0111\u1ea3o h\u01b0\u1edbng (0-7)", 0),
        ("$4",  "Enable Invert",      "\u0110\u1ea3o enable (0/1)", 1),
        ("$5",  "Limit Invert Mask",  "Bit mask \u0111\u1ea3o limit (0-7)", 3),
        ("$11", "Junction Deviation", "\u0110\u1ed9 l\u1ec7ch junction (mm)", 1.0),
        ("$20", "Soft Limit Enable",  "Gi\u1edbi h\u1ea1n m\u1ec1m (0/1)", 1),
        ("$21", "Hard Limit Enable",  "Gi\u1edbi h\u1ea1n c\u1ee9ng (firmware \u00e9p =1)", 1),
        ("$22", "Homing Enable",      "B\u1eadt home (0/1)", 1),
        ("$23", "Homing Dir Mask",    "H\u01b0\u1edbng home (0-7)", 3),
        ("$24", "Homing Feed Rate",   "T\u1ed1c home ch\u1eadm (mm/min)", 25.0),
        ("$25", "Homing Seek Rate",   "T\u1ed1c home nhanh (mm/min)", 2500.0),
        ("$26", "Homing Debounce ms", "Ch\u1ed1ng rung switch (ms)", 25),
        ("$27", "Homing Pulloff",     "L\u00f9i sau home (mm)", 10.0),
        ("$28", "Homing Cycle",       "0=any-order, 1=Z->X->Y", 1),
        # --- Custom extensions ---
        ("$29", "Homing Axis Mask",   "Tr\u1ee5c \u0111\u01b0\u1ee3c home (custom)", 3),
        ("$40", "Report Interval",    "Chu k\u1ef3 auto-report (ms, custom)", 300),
        # --- Per-axis (GRBL standard) ---
        ("$100", "X Steps/mm",   "B\u01b0\u1edbc/mm tr\u1ee5c X", 4.0),
        ("$101", "Y Steps/mm",   "B\u01b0\u1edbc/mm tr\u1ee5c Y", 4.0),
        ("$102", "Z Steps/mm",   "B\u01b0\u1edbc/mm tr\u1ee5c Z", 80.0),
        ("$110", "X Max Rate",   "T\u1ed1c max X (mm/min)", 10000.0),
        ("$111", "Y Max Rate",   "T\u1ed1c max Y (mm/min)", 10000.0),
        ("$112", "Z Max Rate",   "T\u1ed1c max Z (mm/min)", 500.0),
        ("$120", "X Accel",      "Gia t\u1ed1c X (mm/s\u00b2)", 100.0),
        ("$121", "Y Accel",      "Gia t\u1ed1c Y (mm/s\u00b2)", 100.0),
        ("$122", "Z Accel",      "Gia t\u1ed1c Z (mm/s\u00b2)", 50.0),
        ("$130", "X Max Travel", "H\u00e0nh tr\u00ecnh X (mm)", 10000.0),
        ("$131", "Y Max Travel", "H\u00e0nh tr\u00ecnh Y (mm)", 10000.0),
        ("$132", "Z Max Travel", "H\u00e0nh tr\u00ecnh Z (mm)", 20000.0),
    ]

    def __init__(self, parent, serial_manager, main_app):
        super().__init__(parent)
        self.serial_manager = serial_manager
        self.main_app = main_app
        self.title("\u2699\ufe0f CNC Configuration")
        self.geometry("950x720")
        self.configure(bg='#f5f5f5')

        self.transient(parent)
        parent.update_idletasks()
        self.geometry(f"+{parent.winfo_x() + 10}+{parent.winfo_y() + 10}")

        self.config_values = {}
        self.original_values = {}
        self.current_axis = "X"

        self._create_widgets()
        self._load_defaults()
        self.after(300, self._check_and_auto_read)

    def _create_widgets(self):
        header = tk.Frame(self, bg='#2196F3', height=45)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        tk.Label(header, text="\u2699\ufe0f CNC Configuration", font=("Segoe UI", 13, "bold"),
                fg='white', bg='#2196F3').pack(side=tk.LEFT, padx=15, pady=8)
        self.status_label = tk.Label(header, text="\u23f3", font=("Segoe UI", 10),
                                     fg='#E3F2FD', bg='#2196F3')
        self.status_label.pack(side=tk.RIGHT, padx=15)

        main_paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=2)

        toolbar = ttk.Frame(left_frame)
        toolbar.pack(fill=tk.X, pady=(0, 5))

        self.read_btn = ttk.Button(toolbar, text="\U0001f4d6 Read", command=self._read_config)
        self.read_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="\U0001f4dd Write", command=self._write_changed).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="\U0001f4be Save", command=self._save_json).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="\U0001f4c2 Load", command=self._load_json).pack(side=tk.LEFT, padx=2)

        self.progress_var = tk.DoubleVar()
        ttk.Progressbar(toolbar, variable=self.progress_var, maximum=100, length=100).pack(side=tk.RIGHT, padx=5)

        table_frame = ttk.Frame(left_frame)
        table_frame.pack(fill=tk.BOTH, expand=True)

        columns = ("code", "name", "value")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=22)
        self.tree.heading("code", text="Code")
        self.tree.heading("name", text="Parameter")
        self.tree.heading("value", text="Value")
        self.tree.column("code", width=60, anchor=tk.CENTER)
        self.tree.column("name", width=150, anchor=tk.W)
        self.tree.column("value", width=80, anchor=tk.CENTER)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        self.tree.bind("<Double-1>", self._on_double_click)

        edit_frame = ttk.Frame(left_frame)
        edit_frame.pack(fill=tk.X, pady=5)

        self.selected_label = ttk.Label(edit_frame, text="-", font=("Consolas", 10, "bold"), width=6)
        self.selected_label.pack(side=tk.LEFT, padx=5)
        ttk.Label(edit_frame, text="=").pack(side=tk.LEFT)
        self.value_entry = ttk.Entry(edit_frame, width=12, font=("Consolas", 10))
        self.value_entry.pack(side=tk.LEFT, padx=5)
        self.value_entry.bind("<Return>", lambda e: self._update_value())
        ttk.Button(edit_frame, text="\u2713 Apply", command=self._update_value).pack(side=tk.LEFT, padx=5)

        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=1)

        accel_frame = ttk.LabelFrame(right_frame, text="\U0001f4c8 Acceleration Profile", padding=10)
        accel_frame.pack(fill=tk.X, pady=5)

        axis_selector = ttk.Frame(accel_frame)
        axis_selector.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(axis_selector, text="Axis:").pack(side=tk.LEFT)

        self.axis_var = tk.StringVar(value="X")
        for axis, color in [("X", "#e53935"), ("Y", "#43a047"), ("Z", "#1e88e5")]:
            rb = ttk.Radiobutton(axis_selector, text=axis, value=axis,
                                variable=self.axis_var, command=self._on_axis_change)
            rb.pack(side=tk.LEFT, padx=5)

        self.accel_viz = AccelVisualizer(accel_frame, width=300, height=130)
        self.accel_viz.pack()

        test_frame = ttk.Frame(accel_frame)
        test_frame.pack(fill=tk.X, pady=5)

        ttk.Label(test_frame, text="Test Distance:").pack(side=tk.LEFT)
        self.test_dist = ttk.Entry(test_frame, width=8)
        self.test_dist.insert(0, "50")
        self.test_dist.pack(side=tk.LEFT, padx=5)
        self.test_dist.bind("<KeyRelease>", lambda e: self._update_accel_viz())
        ttk.Label(test_frame, text="mm").pack(side=tk.LEFT)

        for d in ["10", "50", "100", "200"]:
            btn = ttk.Button(test_frame, text=d, width=4,
                           command=lambda x=d: self._set_test_distance(x))
            btn.pack(side=tk.LEFT, padx=2)

        info_frame = ttk.LabelFrame(right_frame, text="\U0001f527 Axis Summary", padding=10)
        info_frame.pack(fill=tk.X, pady=10)

        self.axis_cards = {}
        cards_row = ttk.Frame(info_frame)
        cards_row.pack(fill=tk.X)

        for axis, color in [("X", "#e53935"), ("Y", "#43a047"), ("Z", "#1e88e5")]:
            card = tk.Frame(cards_row, bg='white', relief=tk.RIDGE, bd=1)
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)

            tk.Label(card, text=axis, font=("Segoe UI", 12, "bold"), fg=color, bg='white').pack()

            info_label = tk.Label(card, text="Steps: -\nRate: -\nAccel: -",
                                 font=("Consolas", 8), bg='white', justify=tk.LEFT)
            info_label.pack(padx=5, pady=5)
            self.axis_cards[axis] = info_label

        desc_frame = ttk.LabelFrame(right_frame, text="\U0001f4cb Description", padding=10)
        desc_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.desc_text = tk.Text(desc_frame, height=6, font=("Segoe UI", 9), wrap=tk.WORD,
                                 bg='#f9f9f9', state=tk.DISABLED)
        self.desc_text.pack(fill=tk.BOTH, expand=True)

    def _set_test_distance(self, d):
        self.test_dist.delete(0, tk.END)
        self.test_dist.insert(0, d)
        self._update_accel_viz()

    def _on_axis_change(self):
        self.current_axis = self.axis_var.get()
        self._update_accel_viz()

    def _load_defaults(self):
        for code, name, desc, default in self.CONFIG_PARAMS:
            self.tree.insert("", tk.END, values=(code, name, default), tags=(desc,))
            self.config_values[code] = default
            self.original_values[code] = default
        self._update_axis_cards()
        self._update_accel_viz()

    def _on_select(self, event):
        item = self.tree.selection()
        if item:
            values = self.tree.item(item[0])["values"]
            tags = self.tree.item(item[0])["tags"]
            code = values[0]
            self.selected_label.config(text=code)

            if code in ["$120", "$110", "$130", "$100"]:
                self.axis_var.set("X")
                self.current_axis = "X"
            elif code in ["$121", "$111", "$131", "$101"]:
                self.axis_var.set("Y")
                self.current_axis = "Y"
            elif code in ["$122", "$112", "$132", "$102"]:
                self.axis_var.set("Z")
                self.current_axis = "Z"

            self._update_accel_viz()

            self.desc_text.config(state=tk.NORMAL)
            self.desc_text.delete(1.0, tk.END)
            self.desc_text.insert(tk.END, f"{code}: {values[1]}\n\n", "bold")
            self.desc_text.insert(tk.END, f"{tags[0] if tags else ''}\n\n")
            self.desc_text.insert(tk.END, f"Current Value: {values[2]}")
            self.desc_text.config(state=tk.DISABLED)

    def _on_double_click(self, event):
        item = self.tree.selection()
        if item:
            values = self.tree.item(item[0])["values"]
            self.value_entry.delete(0, tk.END)
            self.value_entry.insert(0, str(values[2]))
            self.value_entry.focus()
            self.value_entry.select_range(0, tk.END)

    def _update_value(self):
        item = self.tree.selection()
        if not item:
            return
        try:
            new_value = float(self.value_entry.get())
            values = list(self.tree.item(item[0])["values"])
            values[2] = new_value
            self.tree.item(item[0], values=values)
            self.config_values[values[0]] = new_value
            self._update_axis_cards()
            self._update_accel_viz()
            self.status_label.config(text=f"\u2713 {values[0]}={new_value}")
        except ValueError:
            pass

    def _update_axis_cards(self):
        for axis, idx in [("X", 0), ("Y", 1), ("Z", 2)]:
            steps = self.config_values.get(f"$10{idx}", 0)
            rate = self.config_values.get(f"$11{idx}", 0)
            accel = self.config_values.get(f"$12{idx}", 0)
            travel = self.config_values.get(f"$13{idx}", 0)
            text = f"Steps: {steps}/mm\nRate: {rate}\nAccel: {accel}\nTravel: {travel}"
            self.axis_cards[axis].config(text=text)

    def _update_accel_viz(self):
        try:
            axis = self.current_axis
            axis_idx = {"X": 0, "Y": 1, "Z": 2}.get(axis, 0)
            accel = self.config_values.get(f"$12{axis_idx}", 100)
            max_speed = self.config_values.get(f"$11{axis_idx}", 3000)
            dist_str = self.test_dist.get()
            dist = float(dist_str) if dist_str else 50
            self.accel_viz.update_params(axis, accel, max_speed, dist)
        except:
            pass

    def _check_and_auto_read(self):
        if not self.serial_manager.is_connected:
            self.status_label.config(text="\u26a0\ufe0f Not connected")
            return
        state = self.main_app.state_var.get().lower()
        if state in ["idle", "i", "connected"]:
            self.status_label.config(text="\U0001f504 Reading...")
            self.after(200, self._read_config)
        else:
            self.after(500, self._check_and_auto_read)

    def _read_config(self):
        if not self.serial_manager.is_connected:
            return
        self.progress_var.set(0)
        self.read_btn.config(state=tk.DISABLED)
        self.main_app.config_read_callback = self._on_config_received
        self.main_app.config_read_buffer = []
        self.main_app.config_read_active = True
        self.main_app.config_read_timeout = time.time() + 5
        self.serial_manager.send("$$")
        self._check_config_read()

    def _check_config_read(self):
        if not self.winfo_exists():
            return
        if self.main_app.config_read_active:
            self.progress_var.set(min(90, self.progress_var.get() + 10))
            if time.time() > self.main_app.config_read_timeout:
                self.main_app.config_read_active = False
                self._on_config_received(self.main_app.config_read_buffer)
            else:
                self.after(100, self._check_config_read)

    def _on_config_received(self, lines):
        if not self.winfo_exists():
            return
        self.progress_var.set(100)
        self.read_btn.config(state=tk.NORMAL)
        count = 0
        for line in lines:
            for pattern in [r'\$(\d+)\s*=\s*([0-9.\-]+)', r'\$(\d+)\s+\w+=\s*([0-9.\-]+)']:
                match = re.search(pattern, line)
                if match:
                    code = f"${match.group(1)}"
                    try:
                        value = float(match.group(2))
                        if self._update_tree_value(code, value):
                            count += 1
                            break
                    except:
                        pass
        self._update_axis_cards()
        self._update_accel_viz()
        if count > 0:
            self.status_label.config(text=f"\u2705 Read {count}")
            for code, val in self.config_values.items():
                self.original_values[code] = val
        else:
            self.status_label.config(text="\u26a0\ufe0f No params")

    def _update_tree_value(self, code, value):
        for item in self.tree.get_children():
            values = list(self.tree.item(item)["values"])
            if values[0] == code:
                values[2] = value
                self.tree.item(item, values=values)
                self.config_values[code] = value
                return True
        return False

    def _write_changed(self):
        if not self.serial_manager.is_connected:
            return
        changed = [(c, v) for c, v in self.config_values.items()
                   if abs(v - self.original_values.get(c, v)) > 0.0001]
        if changed:
            for code, value in changed:
                self.serial_manager.send(f"{code}={value}")
                self.original_values[code] = value
                time.sleep(0.1)
            self.status_label.config(text=f"\u2705 Wrote {len(changed)}")

    def _save_json(self):
        filename = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if filename:
            data = {code: {"name": name, "value": self.config_values.get(code, 0)}
                    for code, name, *_ in self.CONFIG_PARAMS}
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.status_label.config(text="\u2705 Saved")

    def _load_json(self):
        filename = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if filename:
            with open(filename, 'r') as f:
                data = json.load(f)
            for code in self.config_values:
                if code in data:
                    self._update_tree_value(code, data[code]["value"])
            self._update_axis_cards()
            self._update_accel_viz()
            self.status_label.config(text="\u2705 Loaded")


# ============================================================================
#                         MAIN APPLICATION
# ============================================================================


class CNCControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🎛️ CNC Controller v2.7.2 (Grbl + M33)")
        self.root.geometry("1300x850")
        self.root.minsize(1100, 700)
        self.root.configure(bg='#fafafa')

        self.serial_manager = SerialManager()

        # Variables
        self.pos_x = tk.DoubleVar(value=0.0)
        self.pos_y = tk.DoubleVar(value=0.0)
        self.pos_z = tk.DoubleVar(value=0.0)
        self.state_var = tk.StringVar(value="Disconnected")
        self.feedrate_var = tk.StringVar(value="1000")
        self.distance_var = tk.StringVar(value="10")
        # Firmware already auto-reports status (default $40=300ms), so GUI
        # polling with realtime '?' is optional and disabled by default.
        self.gui_poll_status_var = tk.BooleanVar(value=False)

        self._prev_state = ""
        self._prev_pos = [None, None, None]

        # Laser tracking
        self.laser_on = False              # local/UI display intent
        self.laser_hw_on = False           # parsed from Sp:<0|1>
        self.laser_pulse_active = False    # parsed from LP:<active,remain>
        self.laser_pulse_remaining_ms = 0
        self._pending_laser_cmd = None
        self._ts_pulse_done = False
        self._ts_pulse_aborted = False
        self._ts_lp_seen_active = False     # True only after firmware reports LP:1 during current M33

        # Command sequence (for multi-step operations with laser)
        self._sequence_commands = []
        self._sequence_idx = 0
        self._sequence_running = False
        self._sequence_laser_before = False
        self._sequence_laser_after = False
        self._sequence_laser_delay_ms = 500

        # GOWDELAY
        self.gwd_x = tk.StringVar(value="")
        self.gwd_y = tk.StringVar(value="")
        self.gwd_z = tk.StringVar(value="")
        self.gwd_step = tk.StringVar(value="1")
        self.gwd_delay = tk.StringVar(value="100")
        self.gwd_feedrate = tk.StringVar(value="500")
        self.gwd_laser_before = tk.BooleanVar(value=False)
        self.gwd_laser_after = tk.BooleanVar(value=False)
        self.gwd_laser_delay = tk.StringVar(value="500")

        # Tick Straight (new tab v2.7)
        self.ts_target_x = tk.StringVar(value="")
        self.ts_target_y = tk.StringVar(value="")
        self.ts_target_z = tk.StringVar(value="")
        self.ts_interval = tk.StringVar(value="5")
        self.ts_delay    = tk.StringVar(value="500")    # ms
        self.ts_feedrate = tk.StringVar(value="500")
        self.ts_settle   = tk.StringVar(value="200")   # ms - delay sau khi toi Idle truoc khi ban M33
        self.ts_pos_tol  = tk.StringVar(value="0.25")  # mm - nguong WARN pos mismatch (nen >= nua buoc vat ly)
        self.ts_autocorrect = tk.BooleanVar(value=True)  # tu dong gui G1 bu khi drift > tol
        self._ts_state          = "idle"   # idle|tick_pulse_send|wait_pulse_done|move|wait_idle|settle|check_pos|done
        self._ts_step_index     = 0
        self._ts_total_steps    = 0
        self._ts_start_pos      = (0.0, 0.0, 0.0)
        self._ts_target_pos     = (0.0, 0.0, 0.0)
        self._ts_unit_vec       = (0.0, 0.0, 0.0)
        self._ts_total_distance = 0.0
        self._ts_distance_done  = 0.0
        self._ts_interval_mm    = 5.0
        self._ts_delay_ms       = 500
        self._ts_feed           = 500.0
        self._ts_settle_ms      = 100
        self._ts_pos_tol        = 0.25
        self._ts_running        = False
        self._ts_after_id       = None
        # Sync flags for wait_idle (fix random position bug)
        self._ts_saw_busy       = False        # set True khi thay state Run/Hold sau khi gui G1
        self._ts_wait_start_ms  = 0            # timestamp luc vao wait_idle (timeout 30s)
        self._ts_expected_pos   = (0.0, 0.0, 0.0)  # toa do tuyet doi mong doi sau move

        # GOWDELAYRECT
        self.rect_width = tk.StringVar(value="50")
        self.rect_height = tk.StringVar(value="50")
        self.rect_step = tk.StringVar(value="5")
        self.rect_delay = tk.StringVar(value="100")
        self.rect_feedrate = tk.StringVar(value="500")
        self.rect_laser_before = tk.BooleanVar(value=False)
        self.rect_laser_after = tk.BooleanVar(value=False)
        self.rect_laser_delay = tk.StringVar(value="500")

        # Config
        self.config_window = None
        self.config_loaded_from_device = False
        self.config_loaded_time = None
        self.config_read_callback = None
        self.config_read_buffer = []
        self.config_read_active = False
        self.config_read_timeout = 0

        # Commands
        self.pending_commands = []
        self.waiting_for_ok = False

        self._setup_styles()
        self._create_menu()
        self._create_widgets()
        self._start_polling()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')

        # ---- Palette ----
        BG       = '#fafafa'
        FG       = '#212121'
        MUTED    = '#5f6368'
        BORDER   = '#dadce0'
        PRIMARY  = '#1976D2'
        PRIMARY2 = '#1565C0'
        SUCCESS  = '#2E7D32'
        SUCCESS2 = '#1B5E20'
        DANGER   = '#C62828'
        DANGER2  = '#B71C1C'
        ACCENT   = '#F9A825'
        ACCENT2  = '#F57F17'

        # ---- Generic widgets ----
        style.configure(".", font=("Segoe UI", 9), background=BG, foreground=FG)
        style.configure("TFrame",      background=BG)
        style.configure("TLabel",      background=BG, foreground=FG)
        style.configure("TLabelframe", background=BG, bordercolor=BORDER,
                        relief="groove", borderwidth=1)
        style.configure("TLabelframe.Label",
                        font=("Segoe UI", 9, "bold"), foreground=PRIMARY2, background=BG)
        style.configure("TButton",
                        font=("Segoe UI", 9), padding=(8, 4),
                        background="#ffffff", foreground=FG,
                        bordercolor=BORDER, focusthickness=0, relief="flat")
        style.map("TButton",
                  background=[("active", "#e8f0fe"), ("pressed", "#d2e3fc")],
                  bordercolor=[("active", PRIMARY)])
        style.configure("TEntry",
                        fieldbackground="white", bordercolor=BORDER,
                        lightcolor=BORDER, darkcolor=BORDER, padding=3)
        style.configure("TCombobox",
                        fieldbackground="white", bordercolor=BORDER,
                        padding=3)
        style.configure("TCheckbutton", background=BG, foreground=FG)
        style.configure("TRadiobutton", background=BG, foreground=FG)
        style.configure("TSeparator",   background=BORDER)

        # ---- Notebook (tabs) ----
        style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(4, 4, 0, 0))
        style.configure("TNotebook.Tab",
                        padding=[16, 8], font=("Segoe UI", 10),
                        background="#eceff1", foreground=MUTED, borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", PRIMARY), ("active", "#cfd8dc")],
                  foreground=[("selected", "white"), ("active", FG)],
                  expand=[("selected", [1, 1, 1, 0])])

        # ---- Treeview ----
        style.configure("Treeview",
                        background="white", fieldbackground="white",
                        foreground=FG, rowheight=24, bordercolor=BORDER,
                        font=("Segoe UI", 9))
        style.configure("Treeview.Heading",
                        font=("Segoe UI", 9, "bold"),
                        background="#eceff1", foreground=FG, relief="flat")
        style.map("Treeview",
                  background=[("selected", "#bbdefb")],
                  foreground=[("selected", FG)])

        # ---- Progressbar ----
        style.configure("Horizontal.TProgressbar",
                        background=PRIMARY, troughcolor="#eceff1",
                        bordercolor=BORDER, lightcolor=PRIMARY, darkcolor=PRIMARY2)

        # ---- Custom action buttons ----
        def _btn(name, bg, bg2, fg="white"):
            style.configure(name,
                            font=("Segoe UI", 10, "bold"),
                            padding=(14, 8),
                            background=bg, foreground=fg,
                            bordercolor=bg2, focusthickness=0,
                            relief="flat")
            style.map(name,
                      background=[("active", bg2), ("pressed", bg2),
                                  ("disabled", "#cfd8dc")],
                      foreground=[("disabled", "#90a4ae")])

        _btn("Primary.TButton", PRIMARY, PRIMARY2)
        _btn("Success.TButton", SUCCESS, SUCCESS2)
        _btn("Danger.TButton",  DANGER,  DANGER2)
        _btn("Accent.TButton",  ACCENT,  ACCENT2, fg="#212121")

        # Small variant
        def _btn_sm(name, bg, bg2, fg="white"):
            style.configure(name,
                            font=("Segoe UI", 9, "bold"),
                            padding=(8, 4),
                            background=bg, foreground=fg,
                            bordercolor=bg2, focusthickness=0,
                            relief="flat")
            style.map(name,
                      background=[("active", bg2), ("pressed", bg2),
                                  ("disabled", "#cfd8dc")],
                      foreground=[("disabled", "#90a4ae")])
        _btn_sm("PrimarySm.TButton", PRIMARY, PRIMARY2)
        _btn_sm("DangerSm.TButton",  DANGER,  DANGER2)
        _btn_sm("SuccessSm.TButton", SUCCESS, SUCCESS2)

    def _create_menu(self):
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="\u2699\ufe0f Config", command=self._open_config)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

    def _create_widgets(self):
        header = tk.Frame(self.root, bg='#fafafa')
        header.pack(fill=tk.X, padx=10, pady=8)

        # Connection
        self.conn_frame = ttk.LabelFrame(header, text="\U0001f50c Connection", padding=6)
        self.conn_frame.pack(side=tk.LEFT)

        self.conn_expanded = ttk.Frame(self.conn_frame)
        self.conn_expanded.pack(fill=tk.X)

        ttk.Label(self.conn_expanded, text="Port:").pack(side=tk.LEFT, padx=3)
        self.port_combo = ttk.Combobox(self.conn_expanded, width=10, state="readonly")
        self.port_combo.pack(side=tk.LEFT)
        btn_refresh = ttk.Button(self.conn_expanded, text="\U0001f504", width=3, command=self._refresh_ports)
        btn_refresh.pack(side=tk.LEFT, padx=2)
        add_tooltip(btn_refresh, "Refresh ports")

        ttk.Label(self.conn_expanded, text="Baud:").pack(side=tk.LEFT, padx=3)
        self.baud_combo = ttk.Combobox(self.conn_expanded, width=7, state="readonly", values=["9600", "115200"])
        self.baud_combo.set("115200")
        self.baud_combo.pack(side=tk.LEFT)

        self.connect_btn = ttk.Button(self.conn_expanded, text="\U0001f517 Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=8)

        self.conn_status = ttk.Label(self.conn_expanded, text="\u25cf Disconnected", foreground="red")
        self.conn_status.pack(side=tk.LEFT, padx=5)

        self.conn_compact = ttk.Frame(self.conn_frame)
        self.compact_label = ttk.Label(self.conn_compact, text="", font=("Segoe UI", 9))
        self.compact_label.pack(side=tk.LEFT, padx=5)
        ttk.Button(self.conn_compact, text="\U0001f50c Disconnect", command=self._toggle_connection).pack(side=tk.LEFT)

        self._refresh_ports()

        # Status
        self.status_frame = ttk.LabelFrame(header, text="\U0001f4ca Status", padding=6)

        ttk.Label(self.status_frame, text="State:").pack(side=tk.LEFT, padx=3)
        self.state_label = ttk.Label(self.status_frame, textvariable=self.state_var,
                                      font=("Segoe UI", 11, "bold"), foreground="gray", width=10)
        self.state_label.pack(side=tk.LEFT, padx=5)

        ttk.Separator(self.status_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        colors = {"X": "#e53935", "Y": "#43a047", "Z": "#1e88e5"}
        self.pos_labels = {}
        for axis, var in [("X", self.pos_x), ("Y", self.pos_y), ("Z", self.pos_z)]:
            ttk.Label(self.status_frame, text=f"{axis}:", font=("Segoe UI", 10, "bold"),
                     foreground=colors[axis]).pack(side=tk.LEFT)
            lbl = tk.Label(self.status_frame, text="0.000", font=("Consolas", 12, "bold"),
                          width=14, anchor=tk.E, bg='#fafafa')
            lbl.pack(side=tk.LEFT, padx=3)
            self.pos_labels[axis] = lbl

        ttk.Separator(self.status_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        # Laser Indicator
        self.laser_frame = tk.Frame(self.status_frame, bg='#fafafa')
        self.laser_frame.pack(side=tk.LEFT, padx=5)
        self.laser_indicator = tk.Canvas(self.laser_frame, width=16, height=16,
                                         bg='#fafafa', highlightthickness=0)
        self.laser_indicator.pack(side=tk.LEFT)
        self.laser_dot = self.laser_indicator.create_oval(2, 2, 14, 14,
                                                          fill='#555', outline='#888')
        ttk.Label(self.laser_frame, text="LASER", font=("Segoe UI", 9, "bold"),
                  foreground="#555").pack(side=tk.LEFT, padx=2)

        ttk.Separator(self.status_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        quick = ttk.Frame(self.status_frame)
        quick.pack(side=tk.LEFT)
        # Compact toolbar: Home | STOP | Resume | Soft-Reset | Clear | Zero
        # STOP   = "!"  feed-hold (machine -> Hold, can be resumed)
        # Resume = "~"  resume from Hold (Hold -> Run -> Idle when motion ends)
        # Reset  = 0x18 soft-reset (any state -> Idle, motion is aborted)
        # Clear  = unlock alarm/limit ($X)
        quick_btns = [
            ("\U0001f3e0",  self._home,       "Home all axes",            None),
            ("\u23f8",      self._feed_hold,  "Feed-hold (! \u2192 Hold)", "Accent.TButton"),
            ("\u25b6",      self._resume,     "Resume from Hold (~)",      "Success.TButton"),
            ("\u23fc",      self._soft_reset, "Soft-reset (0x18) \u2192 Idle, abort motion", "Danger.TButton"),
            ("\U0001f513",  self._clear,      "Clear alarm ($X)",          None),
            ("\U0001f4cd",  self._zero_all,   "Zero XYZ (G92)",            None),
        ]
        for text, cmd, tip, style in quick_btns:
            kw = {"width": 3, "command": cmd}
            if style:
                kw["style"] = style
            btn = ttk.Button(quick, text=text, **kw)
            btn.pack(side=tk.LEFT, padx=1)
            add_tooltip(btn, tip)

        # Main
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        left = ttk.Frame(main_paned)
        main_paned.add(left, weight=3)

        self.notebook = ttk.Notebook(left)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        control_tab = ttk.Frame(self.notebook)
        self.notebook.add(control_tab, text="  \U0001f3ae Control  ")
        self._create_control_tab(control_tab)

        gwd_tab = ttk.Frame(self.notebook)
        self.notebook.add(gwd_tab, text="  \U0001f504 GoWDelay  ")
        self._create_gowdelay_tab(gwd_tab)

        rect_tab = ttk.Frame(self.notebook)
        self.notebook.add(rect_tab, text="  \U0001f532 GoWDelayRect  ")
        self._create_gowdelayrect_tab(rect_tab)

        ts_tab = ttk.Frame(self.notebook)
        self.notebook.add(ts_tab, text="  \U0001f4cd Tick Straight  ")
        self._create_tick_straight_tab(ts_tab)

        right = ttk.Frame(main_paned)
        main_paned.add(right, weight=2)
        self._create_right_panel(right)

    def _create_control_tab(self, parent):
        top_row = ttk.Frame(parent)
        top_row.pack(fill=tk.X, padx=10, pady=10)

        jog_frame = ttk.LabelFrame(top_row, text="\U0001f3ae Jog Control", padding=10)
        jog_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        settings = ttk.Frame(jog_frame)
        settings.pack(fill=tk.X, pady=5)

        ttk.Label(settings, text="Dist:").pack(side=tk.LEFT)
        entry_d = ttk.Entry(settings, textvariable=self.distance_var, width=6)
        entry_d.pack(side=tk.LEFT, padx=2)
        add_tooltip(entry_d, "Kho\u1ea3ng c\u00e1ch (mm)")

        ttk.Label(settings, text="Feed:").pack(side=tk.LEFT, padx=(10, 0))
        entry_f = ttk.Entry(settings, textvariable=self.feedrate_var, width=6)
        entry_f.pack(side=tk.LEFT, padx=2)
        add_tooltip(entry_f, "T\u1ed1c \u0111\u1ed9 (mm/min)")

        for d in ["0.1", "1", "10", "50", "100"]:
            btn = ttk.Button(settings, text=d, width=4, command=lambda x=d: self.distance_var.set(x))
            btn.pack(side=tk.LEFT, padx=1)

        btn_frame = ttk.Frame(jog_frame)
        btn_frame.pack(pady=10)

        xy = ttk.LabelFrame(btn_frame, text="XY", padding=5)
        xy.pack(side=tk.LEFT, padx=15)

        ttk.Button(xy, text="Y+", width=6, command=lambda: self._jog("Y", 1)).grid(row=0, column=1)
        ttk.Button(xy, text="X-", width=6, command=lambda: self._jog("X", -1)).grid(row=1, column=0)
        ttk.Button(xy, text="\u23f9", width=6, command=self._stop).grid(row=1, column=1)
        ttk.Button(xy, text="X+", width=6, command=lambda: self._jog("X", 1)).grid(row=1, column=2)
        ttk.Button(xy, text="Y-", width=6, command=lambda: self._jog("Y", -1)).grid(row=2, column=1)

        z = ttk.LabelFrame(btn_frame, text="Z", padding=5)
        z.pack(side=tk.LEFT, padx=15)
        ttk.Button(z, text="Z+", width=6, command=lambda: self._jog("Z", 1)).pack(pady=3)
        ttk.Button(z, text="Z-", width=6, command=lambda: self._jog("Z", -1)).pack(pady=3)

        self.jog_status = ttk.Label(btn_frame, text="", foreground="blue", width=15)
        self.jog_status.pack(side=tk.LEFT, padx=10)

        viz_frame = ttk.LabelFrame(top_row, text="\U0001f4cd XY Position", padding=10)
        viz_frame.pack(side=tk.LEFT, padx=(10, 0))

        self.xy_viz = XYVisualizer(viz_frame, size=160, max_travel=200)
        self.xy_viz.pack()

        actions = ttk.LabelFrame(parent, text="\u26a1 Quick Actions", padding=10)
        actions.pack(fill=tk.X, padx=10, pady=5)

        row1 = ttk.Frame(actions)
        row1.pack(fill=tk.X, pady=3)
        # Primary motion / safety controls
        row1_btns = [
            ("\U0001f3e0 Home All", self._home,       "Home X/Y/Z",            None),
            ("\u23f9 STOP",         self._stop,       "D\u1eebng kh\u1ea9n (0x18 + STOP + M5)", "Danger.TButton"),
            ("\u23f8 Hold",         self._feed_hold,  "Feed-hold (! \u2192 Hold)", "Accent.TButton"),
            ("\u25b6 Resume",       self._resume,     "Resume (~ \u2192 Run \u2192 Idle)", "Success.TButton"),
            ("\u23fc Reset",        self._soft_reset, "Soft-reset (0x18) \u2192 Idle, abort motion", None),
            ("\U0001f513 Clear",    self._clear,      "Clear alarm ($X)", None),
        ]
        for text, cmd, tip, style in row1_btns:
            kw = {"width": 8, "command": cmd}
            if style:
                kw["style"] = style
            btn = ttk.Button(row1, text=text, **kw)
            btn.pack(side=tk.LEFT, padx=1)
            add_tooltip(btn, tip)

        row2 = ttk.Frame(actions)
        row2.pack(fill=tk.X, pady=3)
        for text, cmd, tip in [("\U0001f4cd Zero", self._zero_all, "Zero XYZ (G92)"),
                               ("\U0001f50c Enable", lambda: self._send("M17"), "B\u1eadt motor"),
                               ("\U0001f4a4 Disable", lambda: self._send("M18"), "T\u1eaft motor"),
                               ("\u2699\ufe0f Config", self._open_config, "C\u1ea5u h\u00ecnh"),
                               ("\u2753 Status", lambda: self._send("?"), "L\u1ea5y status")]:
            btn = ttk.Button(row2, text=text, width=12, command=cmd)
            btn.pack(side=tk.LEFT, padx=3)
            add_tooltip(btn, tip)

        relay = ttk.LabelFrame(parent, text="\U0001f50c Relay", padding=10)
        relay.pack(fill=tk.X, padx=10, pady=5)

        relay_row = ttk.Frame(relay)
        relay_row.pack()

        ttk.Label(relay_row, text="Spindle:").pack(side=tk.LEFT)
        ttk.Button(relay_row, text="M3 ON", width=7, command=lambda: self._send_laser_cmd("M3")).pack(side=tk.LEFT, padx=2)
        ttk.Button(relay_row, text="M5 OFF", width=7, command=lambda: self._send_laser_cmd("M5")).pack(side=tk.LEFT, padx=2)

        ttk.Label(relay_row, text="  Coolant:").pack(side=tk.LEFT)
        ttk.Button(relay_row, text="M7", width=4, command=lambda: self._send("M7")).pack(side=tk.LEFT, padx=1)
        ttk.Button(relay_row, text="M8", width=4, command=lambda: self._send("M8")).pack(side=tk.LEFT, padx=1)
        ttk.Button(relay_row, text="M9", width=4, command=lambda: self._send("M9")).pack(side=tk.LEFT, padx=1)

    def _create_gowdelay_tab(self, parent):
        main = ttk.Frame(parent, padding=15)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text="\U0001f504 GOWDELAY - Di chuy\u1ec3n v\u1edbi delay", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W)

        target = ttk.LabelFrame(main, text="Target (\u0111\u1ec3 tr\u1ed1ng = b\u1ecf qua)", padding=10)
        target.pack(fill=tk.X, pady=10)

        pos_row = ttk.Frame(target)
        pos_row.pack(fill=tk.X)

        colors = {"X": "#e53935", "Y": "#43a047", "Z": "#1e88e5"}
        for axis, var in [("X", self.gwd_x), ("Y", self.gwd_y), ("Z", self.gwd_z)]:
            ttk.Label(pos_row, text=f"{axis}:", foreground=colors[axis], font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(8, 2))
            ttk.Entry(pos_row, textvariable=var, width=10).pack(side=tk.LEFT)

        ttk.Button(pos_row, text="\U0001f4cd Current", command=self._use_current_pos).pack(side=tk.LEFT, padx=10)
        ttk.Button(pos_row, text="\U0001f5d1\ufe0f", command=self._clear_gwd_pos, width=3).pack(side=tk.LEFT)

        params = ttk.LabelFrame(main, text="Parameters", padding=10)
        params.pack(fill=tk.X, pady=10)

        p_row = ttk.Frame(params)
        p_row.pack(fill=tk.X)

        ttk.Label(p_row, text="Step D:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.gwd_step, width=7).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(p_row, text="Delay W:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.gwd_delay, width=7).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(p_row, text="Feed F:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.gwd_feedrate, width=7).pack(side=tk.LEFT)

        laser_frame = ttk.LabelFrame(main, text="\U0001f526 Laser Options", padding=10)
        laser_frame.pack(fill=tk.X, pady=10)

        laser_row1 = ttk.Frame(laser_frame)
        laser_row1.pack(fill=tk.X)
        ttk.Checkbutton(laser_row1, text="B\u1eadt laser (M3) tr\u01b0\u1edbc khi ch\u1ea1y",
                        variable=self.gwd_laser_before).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(laser_row1, text="T\u1eaft laser (M5) sau khi ch\u1ea1y",
                        variable=self.gwd_laser_after).pack(side=tk.LEFT, padx=5)

        laser_row2 = ttk.Frame(laser_frame)
        laser_row2.pack(fill=tk.X, pady=5)
        ttk.Label(laser_row2, text="Delay sau M3 (ms):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(laser_row2, textvariable=self.gwd_laser_delay, width=7).pack(side=tk.LEFT)

        preview = ttk.LabelFrame(main, text="Preview", padding=10)
        preview.pack(fill=tk.X, pady=10)

        self.gwd_preview = ttk.Label(preview, text="", font=("Consolas", 10), foreground="#1976D2")
        self.gwd_preview.pack()

        btns = ttk.Frame(main)
        btns.pack(fill=tk.X, pady=10)

        ttk.Button(btns, text="\u25b6\ufe0f Execute GOWDELAY", command=self._execute_gowdelay).pack(side=tk.LEFT, padx=5)
        ttk.Button(btns, text="\u23f9\ufe0f Stop", command=self._stop).pack(side=tk.LEFT)

        for var in [self.gwd_x, self.gwd_y, self.gwd_z, self.gwd_step, self.gwd_delay, self.gwd_feedrate]:
            var.trace_add("write", self._update_gwd_preview)

    def _create_gowdelayrect_tab(self, parent):
        main = ttk.Frame(parent, padding=15)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(main)
        top.pack(fill=tk.X)

        info = ttk.Frame(top)
        info.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(info, text="\U0001f532 GOWDELAYRECT - Qu\u00e9t h\u00ecnh ch\u1eef nh\u1eadt", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W)

        size_frame = ttk.LabelFrame(info, text="Rectangle Size", padding=10)
        size_frame.pack(fill=tk.X, pady=10)

        size_row = ttk.Frame(size_frame)
        size_row.pack(fill=tk.X)

        ttk.Label(size_row, text="Width X:", foreground="#e53935", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        entry_w = ttk.Entry(size_row, textvariable=self.rect_width, width=8)
        entry_w.pack(side=tk.LEFT, padx=(2, 15))

        ttk.Label(size_row, text="Height Y:", foreground="#43a047", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        entry_h = ttk.Entry(size_row, textvariable=self.rect_height, width=8)
        entry_h.pack(side=tk.LEFT)

        params = ttk.LabelFrame(info, text="Parameters", padding=10)
        params.pack(fill=tk.X, pady=10)

        p_row = ttk.Frame(params)
        p_row.pack(fill=tk.X)

        ttk.Label(p_row, text="Step D:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.rect_step, width=7).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(p_row, text="Delay W:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.rect_delay, width=7).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(p_row, text="Feed F:").pack(side=tk.LEFT)
        ttk.Entry(p_row, textvariable=self.rect_feedrate, width=7).pack(side=tk.LEFT)

        laser_frame = ttk.LabelFrame(info, text="\U0001f526 Laser Options", padding=10)
        laser_frame.pack(fill=tk.X, pady=10)

        laser_row1 = ttk.Frame(laser_frame)
        laser_row1.pack(fill=tk.X)
        ttk.Checkbutton(laser_row1, text="B\u1eadt laser (M3) tr\u01b0\u1edbc khi ch\u1ea1y",
                        variable=self.rect_laser_before).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(laser_row1, text="T\u1eaft laser (M5) sau khi ch\u1ea1y",
                        variable=self.rect_laser_after).pack(side=tk.LEFT, padx=5)

        laser_row2 = ttk.Frame(laser_frame)
        laser_row2.pack(fill=tk.X, pady=5)
        ttk.Label(laser_row2, text="Delay sau M3 (ms):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(laser_row2, textvariable=self.rect_laser_delay, width=7).pack(side=tk.LEFT)

        preview_frame = ttk.LabelFrame(top, text="\U0001f4d0 Pattern Preview", padding=10)
        preview_frame.pack(side=tk.LEFT, padx=(15, 0))

        self.rect_pattern = RectPatternPreview(preview_frame, size=160)
        self.rect_pattern.pack()

        cmd_frame = ttk.LabelFrame(main, text="Command", padding=10)
        cmd_frame.pack(fill=tk.X, pady=10)

        self.rect_preview = ttk.Label(cmd_frame, text="", font=("Consolas", 10), foreground="#1976D2")
        self.rect_preview.pack()

        self.rect_stats = ttk.Label(cmd_frame, text="", foreground="gray")
        self.rect_stats.pack()

        btns = ttk.Frame(main)
        btns.pack(fill=tk.X, pady=10)

        ttk.Button(btns, text="\u25b6\ufe0f Execute GOWDELAYRECT", command=self._execute_gowdelayrect).pack(side=tk.LEFT, padx=5)
        ttk.Button(btns, text="\u23f9\ufe0f Stop", command=self._stop).pack(side=tk.LEFT)

        for var in [self.rect_width, self.rect_height, self.rect_step, self.rect_delay, self.rect_feedrate]:
            var.trace_add("write", self._update_rect_preview)

    # === Parameter descriptions (used by tooltip + side panel) ===
    _TS_PARAM_HELP = {
        "target": (
            "V\u1ecb tr\u00ed \u0111\u00edch (Target Position)",
            "T\u1ecda \u0111\u1ed9 TUY\u1ec6T \u0110\u1ed0I (mm) m\u00e0 \u0111\u1ea7u \u0111\u1ee5ng s\u1ebd di chuy\u1ec3n t\u1edbi.\n"
            "\n"
            "\u2022 \u0110\u1ec3 tr\u1ed1ng = gi\u1eef nguy\u00ean tr\u1ee5c \u0111\u00f3 (kh\u00f4ng di chuy\u1ec3n tr\u1ee5c \u0111\u00f3).\n"
            "\u2022 \u0110\u01a1n v\u1ecb: mm.\n"
            "\u2022 N\u00fat 'Use Current' \u0111\u1ec3 d\u00f9ng v\u1ecb tr\u00ed hi\u1ec7n t\u1ea1i l\u00e0m \u0111\u00edch (kh\u00f4ng di chuy\u1ec3n).\n"
            "\n"
            "V\u00ed d\u1ee5: X=100, Y=50, Z tr\u1ed1ng \u2192 di chuy\u1ec3n t\u1eeb (cx,cy,cz) "
            "\u0111\u1ebfn (100, 50, cz) theo \u0111\u01b0\u1eddng th\u1eb3ng."
        ),
        "interval": (
            "Kho\u1ea3ng b\u01b0\u1edbc (Interval / Step)",
            "Kho\u1ea3ng c\u00e1ch DI CHUY\u1ec2N m\u1ed7i chu k\u1ef3 (mm).\n"
            "\u0110o theo \u0111\u01b0\u1eddng th\u1eb3ng t\u1eeb \u0111i\u1ec3m hi\u1ec7n t\u1ea1i \u0111\u1ebfn \u0111\u00edch (vector kho\u1ea3ng c\u00e1ch).\n"
            "\n"
            "\u2022 Gi\u00e1 tr\u1ecb nh\u1ecf \u2192 nhi\u1ec1u ch\u1ea5m \u2192 d\u00e0y \u0111\u1eb7c (\u0111\u1eb9p, ch\u1eadm).\n"
            "\u2022 Gi\u00e1 tr\u1ecb l\u1edbn \u2192 \u00edt ch\u1ea5m \u2192 th\u01b0a (nhanh).\n"
            "\u2022 N\u1ebfu kho\u1ea3ng cu\u1ed1i nh\u1ecf h\u01a1n interval, b\u01b0\u1edbc cu\u1ed1i s\u1ebd ch\u1ec9 \u0111i ph\u1ea7n c\u00f2n l\u1ea1i.\n"
            "\n"
            "V\u00ed d\u1ee5: kho\u1ea3ng c\u00e1ch 23mm, interval 5mm \u2192 4 b\u01b0\u1edbc \u00d75mm + 1 b\u01b0\u1edbc 3mm = 5 b\u01b0\u1edbc."
        ),
        "delay": (
            "Th\u1eddi gian tr\u1ec5 (Delay / Dwell)",
            "Kho\u1ea3ng th\u1eddi gian pulse laser t\u1ea1i m\u1ed7i \u0111i\u1ec3m d\u1eebng.\n"
            "GUI g\u1eedi M33 D<ms>, firmware STM32 t\u1ef1 b\u1eadt \u2192 \u0111\u1ee3i \u2192 t\u1eaft.\n"
            "\n"
            "\u2022 \u0110\u01a1n v\u1ecb: mili-gi\u00e2y (ms). V\u00ed d\u1ee5: 500 = 0.5 gi\u00e2y.\n"
            "\u2022 Th\u1eddi gian n\u00e0y quy\u1ebft \u0111\u1ecbnh \u0111\u1ed9 \u0111\u1eadm c\u1ee7a m\u1ed7i ch\u1ea5m.\n"
            "\u2022 Tr\u1ec5 c\u00e0ng l\u1edbn \u2192 ch\u1ea5m c\u00e0ng s\u00e2u / \u0111\u1eadm.\n"
            "\u2022 Gi\u00e1 tr\u1ecb 0 \u2192 pulse ng\u1eafn nh\u1ea5t c\u00f3 th\u1ec3.\n"
            "\n"
            "Khuy\u1ebfn ngh\u1ecb:\n"
            "  \u2022 100-300ms cho ch\u1ea5m m\u1edd\n"
            "  \u2022 500-1000ms cho ch\u1ea5m \u0111\u1eadm\n"
            "  \u2022 >2000ms ch\u1ec9 d\u00f9ng cho v\u1eadt li\u1ec7u kh\u00f3 c\u1eaft"
        ),
        "feedrate": (
            "T\u1ed1c \u0111\u1ed9 di chuy\u1ec3n (Feedrate)",
            "T\u1ed1c \u0111\u1ed9 di chuy\u1ec3n GI\u1eeeA c\u00e1c \u0111i\u1ec3m tick (mm/min).\n"
            "\n"
            "\u2022 \u0110\u01a1n v\u1ecb: mm/ph\u00fat (chu\u1ea9n G-code).\n"
            "\u2022 KH\u00d4NG \u1ea3nh h\u01b0\u1edfng \u0111\u1ebfn th\u1eddi gian tick (delay).\n"
            "\u2022 Th\u1ea5p (200-500): an to\u00e0n, ch\u00ednh x\u00e1c.\n"
            "\u2022 V\u1eeba (500-2000): chu\u1ea9n cho h\u1ea7u h\u1ebft tr\u01b0\u1eddng h\u1ee3p.\n"
            "\u2022 Cao (>3000): ki\u1ec3m tra max_rate ($110-112) c\u1ee7a m\u00e1y."
        ),
    }

    def _create_tick_straight_tab(self, parent):
        """Tick Straight: tạo chấm laser theo đường thẳng.

        Layout: action bar (Run/Stop + status) ở TRÊN CÙNG, sticky.
        Tham số ở dưới, gọn 4 dòng. Preview ở cuối.
        """
        PRIMARY  = "#1976D2"
        SUCCESS  = "#2E7D32"
        DANGER   = "#C62828"
        MUTED    = "#5f6368"

        root = ttk.Frame(parent)
        root.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        # ============================================================
        # 1) ACTION BAR (sticky top) — luôn nhìn thấy
        # ============================================================
        action_card = tk.Frame(root, bg="white", highlightbackground="#dadce0",
                               highlightthickness=1)
        action_card.pack(fill=tk.X, pady=(0, 10))

        # Title + workflow hint
        title_row = tk.Frame(action_card, bg="white")
        title_row.pack(fill=tk.X, padx=12, pady=(10, 4))
        tk.Label(title_row,
                 text="\U0001f4cd  TICK STRAIGHT",
                 font=("Segoe UI", 13, "bold"),
                 fg=PRIMARY, bg="white").pack(side=tk.LEFT)
        tk.Label(title_row,
                 text="  —  Tick → Move → Tick → Move → … → Đích",
                 font=("Segoe UI", 9),
                 fg=MUTED, bg="white").pack(side=tk.LEFT, padx=(4, 0))

        # Big action buttons + progress
        btn_row = tk.Frame(action_card, bg="white")
        btn_row.pack(fill=tk.X, padx=12, pady=(0, 10))

        self.ts_run_btn = ttk.Button(btn_row,
                                     text="▶  CHẠY",
                                     style="Success.TButton",
                                     command=self._ts_start)
        self.ts_run_btn.pack(side=tk.LEFT, padx=(0, 6), ipadx=10)
        add_tooltip(self.ts_run_btn,
                    "Bắt đầu chuỗi Tick → Move → Tick …\n"
                    "Yêu cầu: đã Connect, tham số hợp lệ.")

        self.ts_stop_btn = ttk.Button(btn_row,
                                      text="⏹  DỪNG",
                                      style="Danger.TButton",
                                      command=self._ts_stop,
                                      state=tk.DISABLED)
        self.ts_stop_btn.pack(side=tk.LEFT, padx=6, ipadx=10)
        add_tooltip(self.ts_stop_btn,
                    "Dừng khẩn cấp:\n"
                    " • Hủy chuỗi tick ngay\n"
                    " • Gửi M5 (laser OFF)\n"
                    " • Gửi '!' (feed-hold)")

        # Spacer
        tk.Frame(btn_row, bg="white").pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Status badge
        self.ts_status = tk.Label(btn_row,
                                  text="⏸  Sẵn sàng",
                                  font=("Segoe UI", 10, "bold"),
                                  fg=MUTED, bg="white",
                                  padx=10, pady=4)
        self.ts_status.pack(side=tk.LEFT, padx=(8, 8))

        self.ts_progress = ttk.Progressbar(btn_row, length=180,
                                           mode="determinate")
        self.ts_progress.pack(side=tk.LEFT)

        # ============================================================
        # 2) PARAMETERS (compact, 4 hàng)
        # ============================================================
        params = ttk.LabelFrame(root, text="  ⚙  Tham số  ", padding=10)
        params.pack(fill=tk.X, pady=(0, 8))

        # --- Row 1: Target X Y Z + buttons ---
        r1 = ttk.Frame(params)
        r1.pack(fill=tk.X, pady=3)
        ttk.Label(r1, text="Đích:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        for label, var, color in [("X", self.ts_target_x, "#e53935"),
                                   ("Y", self.ts_target_y, "#43a047"),
                                   ("Z", self.ts_target_z, "#1e88e5")]:
            ttk.Label(r1, text=label, foreground=color,
                      font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(6, 2))
            ent = ttk.Entry(r1, textvariable=var, width=9,
                            font=("Consolas", 10))
            ent.pack(side=tk.LEFT)
            ent.bind("<KeyRelease>", lambda e: self._update_ts_preview())
            add_tooltip(ent, self._TS_PARAM_HELP["target"][1])
        ttk.Label(r1, text="mm", foreground=MUTED).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Button(r1, text="📍 Hiện tại", style="PrimarySm.TButton",
                   command=self._ts_use_current).pack(side=tk.LEFT, padx=2)
        ttk.Button(r1, text="✕", width=3,
                   command=self._ts_clear_target).pack(side=tk.LEFT, padx=2)

        # --- Row 2: Interval ---
        r2 = ttk.Frame(params)
        r2.pack(fill=tk.X, pady=3)
        ttk.Label(r2, text="Interval:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        ent_i = ttk.Entry(r2, textvariable=self.ts_interval, width=9,
                          font=("Consolas", 10))
        ent_i.pack(side=tk.LEFT, padx=(0, 4))
        ent_i.bind("<KeyRelease>", lambda e: self._update_ts_preview())
        add_tooltip(ent_i, self._TS_PARAM_HELP["interval"][1])
        ttk.Label(r2, text="mm", foreground=MUTED).pack(side=tk.LEFT, padx=(0, 10))
        for d in ["0.5", "1", "2", "5", "10"]:
            ttk.Button(r2, text=d, width=4,
                       command=lambda x=d: (self.ts_interval.set(x),
                                            self._update_ts_preview())
                       ).pack(side=tk.LEFT, padx=1)

        # --- Row 3: Delay ---
        r3 = ttk.Frame(params)
        r3.pack(fill=tk.X, pady=3)
        ttk.Label(r3, text="Delay:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        ent_d = ttk.Entry(r3, textvariable=self.ts_delay, width=9,
                          font=("Consolas", 10))
        ent_d.pack(side=tk.LEFT, padx=(0, 4))
        add_tooltip(ent_d, self._TS_PARAM_HELP["delay"][1])
        ttk.Label(r3, text="ms", foreground=MUTED).pack(side=tk.LEFT, padx=(0, 10))
        for d in ["100", "300", "500", "1000", "2000"]:
            ttk.Button(r3, text=d, width=5,
                       command=lambda x=d: self.ts_delay.set(x)
                       ).pack(side=tk.LEFT, padx=1)

        # --- Row 4: Feedrate ---
        r4 = ttk.Frame(params)
        r4.pack(fill=tk.X, pady=3)
        ttk.Label(r4, text="Feed:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        ent_f = ttk.Entry(r4, textvariable=self.ts_feedrate, width=9,
                          font=("Consolas", 10))
        ent_f.pack(side=tk.LEFT, padx=(0, 4))
        ent_f.bind("<KeyRelease>", lambda e: self._update_ts_preview())
        add_tooltip(ent_f, self._TS_PARAM_HELP["feedrate"][1])
        ttk.Label(r4, text="mm/min", foreground=MUTED).pack(side=tk.LEFT, padx=(0, 10))
        for d in ["200", "500", "1000", "2000", "3000"]:
            ttk.Button(r4, text=d, width=5,
                       command=lambda x=d: self.ts_feedrate.set(x)
                       ).pack(side=tk.LEFT, padx=1)

        # --- Row 5: Settle delay (cho co khi on dinh truoc khi ban M3) ---
        r5 = ttk.Frame(params)
        r5.pack(fill=tk.X, pady=3)
        ttk.Label(r5, text="Settle:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        ent_s = ttk.Entry(r5, textvariable=self.ts_settle, width=9,
                          font=("Consolas", 10))
        ent_s.pack(side=tk.LEFT, padx=(0, 4))
        add_tooltip(ent_s,
            "Tre sau khi may toi Idle truoc khi ban M33.\n"
            "Dung de co khi het rung (vibration) -> cham chinh xac hon.\n"
            "Khuyen nghi: 50-200ms voi may nho, 200-500ms voi may to.\n"
            "0 = ban pulse ngay khi thay Idle.")
        ttk.Label(r5, text="ms", foreground=MUTED).pack(side=tk.LEFT, padx=(0, 10))
        for d in ["0", "50", "100", "200", "500"]:
            ttk.Button(r5, text=d, width=5,
                       command=lambda x=d: self.ts_settle.set(x)
                       ).pack(side=tk.LEFT, padx=1)

        # --- Row 6: Position tolerance (nguong WARN khi sai vi tri) ---
        r6 = ttk.Frame(params)
        r6.pack(fill=tk.X, pady=3)
        ttk.Label(r6, text="Pos tol:", width=10,
                  font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        ent_t = ttk.Entry(r6, textvariable=self.ts_pos_tol, width=9,
                          font=("Consolas", 10))
        ent_t.pack(side=tk.LEFT, padx=(0, 4))
        add_tooltip(ent_t,
            "Nguong WARN khi vi tri thuc khac expected.\n"
            "Don vi: mm (vi tri MPos/WPos firmware gui ve).\n"
            "Nen dat >= nua buoc vat ly cua truc thap nhat.\n"
            "Vi du $100=4 steps/mm -> 1 step=0.25mm, nen tol khoang 0.25-0.5mm.\n"
            "May buoc min hon co the dung 0.01-0.05mm.")
        ttk.Label(r6, text="mm", foreground=MUTED).pack(side=tk.LEFT, padx=(0, 10))
        for d in ["0.01", "0.05", "0.1", "0.25", "0.5"]:
            ttk.Button(r6, text=d, width=5,
                       command=lambda x=d: self.ts_pos_tol.set(x)
                       ).pack(side=tk.LEFT, padx=1)

        # --- Row 7: Auto-correct checkbox ---
        r7 = ttk.Frame(params)
        r7.pack(fill=tk.X, pady=3)
        ttk.Label(r7, text="", width=10).pack(side=tk.LEFT)   # spacer
        cb = ttk.Checkbutton(r7,
            text="\U0001f3af  Auto-correct drift (gui G1 bu khi drift > tol)",
            variable=self.ts_autocorrect)
        cb.pack(side=tk.LEFT)
        add_tooltip(cb,
            "Khi BAT: neu vi tri thuc khac target > tol, gui them G1 de may bu chinh xac.\n"
            "Khi TAT: chi log warning, chap nhan vi tri thuc.\n"
            "Khuyen nghi: BAT khi can chinh xac cao (laser khac chi tiet).")

        # ============================================================
        # 3) PREVIEW (tính toán sơ bộ)
        # ============================================================
        preview_frame = ttk.LabelFrame(root,
            text="  📊  Tính toán sơ bộ  ", padding=10)
        preview_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))
        self.ts_preview = tk.Label(preview_frame, text="—",
                                   font=("Consolas", 9), fg="#212121",
                                   bg="#FFFDE7", relief=tk.SUNKEN, bd=1,
                                   anchor=tk.NW, justify=tk.LEFT,
                                   wraplength=620, padx=10, pady=8)
        self.ts_preview.pack(fill=tk.BOTH, expand=True)

        # ============================================================
        # 4) HELP HINT (collapsed: 1 dòng nhỏ phía dưới)
        # ============================================================
        hint = tk.Label(root,
            text="💡 Quy trình: Tick (M33 D<delay> trên firmware) → Move Interval mm → … → Đích.   "
                 "Để trống X/Y/Z = giữ nguyên trục đó.   "
                 "Hover vào ô để xem chi tiết.",
            font=("Segoe UI", 8), fg=MUTED, bg="#fafafa",
            anchor=tk.W, justify=tk.LEFT, wraplength=700)
        hint.pack(fill=tk.X, pady=(4, 0))

        # Initial preview
        self._update_ts_preview()

    def _ts_show_help(self, key):
        """Compatibility shim — help panel removed in compact layout.
        All field-specific help is now shown via tooltips on hover."""
        return

    def _ts_use_current(self):
        self.ts_target_x.set(f"{self.pos_x.get():.3f}")
        self.ts_target_y.set(f"{self.pos_y.get():.3f}")
        self.ts_target_z.set(f"{self.pos_z.get():.3f}")
        self._update_ts_preview()

    def _ts_clear_target(self):
        self.ts_target_x.set("")
        self.ts_target_y.set("")
        self.ts_target_z.set("")
        self._update_ts_preview()

    def _update_ts_preview(self):
        try:
            tx = float(self.ts_target_x.get()) if self.ts_target_x.get().strip() else self.pos_x.get()
            ty = float(self.ts_target_y.get()) if self.ts_target_y.get().strip() else self.pos_y.get()
            tz = float(self.ts_target_z.get()) if self.ts_target_z.get().strip() else self.pos_z.get()
            interval = float(self.ts_interval.get()) if self.ts_interval.get().strip() else 0
            delay    = float(self.ts_delay.get())    if self.ts_delay.get().strip()    else 0
            feed     = float(self.ts_feedrate.get()) if self.ts_feedrate.get().strip() else 0
            cx, cy, cz = self.pos_x.get(), self.pos_y.get(), self.pos_z.get()
            dx, dy, dz = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if interval > 0 and dist > 1e-6:
                n_full = int(dist / interval)
                rem    = dist - n_full * interval
                n_total = n_full + (1 if rem > 1e-4 else 0)
                last_step = rem if rem > 1e-4 else interval
            else:
                n_total = 0
                last_step = 0
            n_ticks = n_total + 1
            # Estimated time
            est_move_s = (dist / feed * 60.0) if feed > 0 else 0  # seconds
            est_tick_s = (n_ticks * delay / 1000.0)
            est_total_s = est_move_s + est_tick_s + (n_total * 0.2)  # +200ms overhead/step
            mins = int(est_total_s // 60); secs = int(est_total_s % 60)

            self.ts_preview.config(
                text=(f"\U0001f4cd  Hi\u1ec7n t\u1ea1i: ({cx:7.3f}, {cy:7.3f}, {cz:7.3f}) mm\n"
                      f"\U0001f3af  \u0110\u00edch:    ({tx:7.3f}, {ty:7.3f}, {tz:7.3f}) mm\n"
                      f"\U0001f4cf  Kho\u1ea3ng c\u00e1ch: {dist:8.3f} mm\n"
                      f"\U0001f50d  S\u1ed1 ch\u1ea5m (tick): {n_ticks}  "
                      f"(b\u01b0\u1edbc: {n_total}"
                      + (f", b\u01b0\u1edbc cu\u1ed1i {last_step:.3f}mm" if abs(last_step - interval) > 1e-4 and n_total > 0 else "")
                      + ")\n"
                      f"\u23f1  Th\u1eddi gian \u01b0\u1edbc t\u00ednh: ~ {mins}m {secs}s "
                      f"(tick {est_tick_s:.1f}s + move {est_move_s:.1f}s)")
            )
        except (ValueError, tk.TclError):
            self.ts_preview.config(text="\u26a0 Tham s\u1ed1 kh\u00f4ng h\u1ee3p l\u1ec7")

    def _ts_start(self):
        if self._ts_running:
            return
        if not self.serial_manager.is_connected:
            messagebox.showwarning("Tick Straight", "Ch\u01b0a k\u1ebft n\u1ed1i")
            return

        try:
            tx = float(self.ts_target_x.get()) if self.ts_target_x.get().strip() else self.pos_x.get()
            ty = float(self.ts_target_y.get()) if self.ts_target_y.get().strip() else self.pos_y.get()
            tz = float(self.ts_target_z.get()) if self.ts_target_z.get().strip() else self.pos_z.get()
            interval = float(self.ts_interval.get())
            delay_ms = int(float(self.ts_delay.get()))
            feed     = float(self.ts_feedrate.get())
            settle_ms = int(float(self.ts_settle.get())) if self.ts_settle.get().strip() else 200
            pos_tol   = float(self.ts_pos_tol.get())     if self.ts_pos_tol.get().strip()  else 0.25
            autocorrect = self.ts_autocorrect.get()
            if interval <= 0:
                raise ValueError("Interval must be > 0")
            if delay_ms < 0:
                raise ValueError("Delay must be >= 0")
            if feed <= 0:
                raise ValueError("Feedrate must be > 0")
            if settle_ms < 0:
                raise ValueError("Settle must be >= 0")
            if pos_tol < 0:
                raise ValueError("Pos tolerance must be >= 0")
        except (ValueError, tk.TclError) as e:
            messagebox.showerror("Tick Straight", f"Tham s\u1ed1 kh\u00f4ng h\u1ee3p l\u1ec7: {e}")
            return

        cx, cy, cz = self.pos_x.get(), self.pos_y.get(), self.pos_z.get()
        dx, dy, dz = tx - cx, ty - cy, tz - cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1e-4:
            messagebox.showinfo("Tick Straight",
                                "V\u1ecb tr\u00ed \u0111\u00edch tr\u00f9ng v\u1edbi v\u1ecb tr\u00ed hi\u1ec7n t\u1ea1i")
            return

        # Save engine state
        self._ts_start_pos      = (cx, cy, cz)
        self._ts_target_pos     = (tx, ty, tz)
        self._ts_unit_vec       = (dx / dist, dy / dist, dz / dist)
        self._ts_total_distance = dist
        self._ts_distance_done  = 0.0
        self._ts_interval_mm    = interval
        self._ts_delay_ms       = delay_ms
        self._ts_feed           = feed
        self._ts_settle_ms      = settle_ms
        self._ts_pos_tol        = pos_tol
        self._ts_autocorrect    = autocorrect
        self._ts_correct_count  = 0       # so lan auto-correct (de log)
        self._ts_step_index     = 0
        n_full = int(dist / interval)
        rem    = dist - n_full * interval
        self._ts_total_steps = n_full + (1 if rem > 1e-4 else 0)
        self.ts_progress["maximum"] = self._ts_total_steps
        self.ts_progress["value"]   = 0

        self._ts_running       = True
        self._ts_saw_busy      = False
        self._ts_wait_start_ms = 0
        self._ts_expected_pos  = (cx, cy, cz)   # ban dau = vi tri hien tai
        self.ts_run_btn.config(state=tk.DISABLED)
        self.ts_stop_btn.config(state=tk.NORMAL)

        # Switch to absolute mode for clean tracking, then start with initial tick
        # KHONG dung G91 trong loop -> tranh drift (FIX B4)
        self._send("G90")
        self.ts_status.config(text="\u25b6 B\u1eaft \u0111\u1ea7u: Tick #1...", fg="#2196F3")
        self._log(f"[TS] Start abs-mode: ({cx:.2f},{cy:.2f},{cz:.2f}) -> "
                  f"({tx:.2f},{ty:.2f},{tz:.2f}) dist={dist:.3f} "
                  f"interval={interval} steps={self._ts_total_steps} "
                  f"delay={delay_ms}ms feed={feed} settle={settle_ms}ms tol={pos_tol} "
                  f"autocorrect={'ON' if autocorrect else 'OFF'}", "info")
        # Begin: tick at current position
        self._ts_pulse_done = False
        self._ts_pulse_aborted = False
        self._ts_lp_seen_active = False
        self._ts_state = "tick_pulse_send"
        self._ts_after_id = self.root.after(150, self._ts_tick_step)

    def _ts_stop(self):
        """Tick Straight stop now delegates to the global emergency-stop path.
        This is required because M33 pulse timing lives in firmware, not GUI.
        """
        self._log("[TS] User stop requested", "info")
        self._stop()

    def _ts_finish(self, msg="\u2714 Ho\u00e0n t\u1ea5t"):
        self._ts_running = False
        self._ts_state = "done"
        self._ts_after_id = None
        self._ts_pulse_done = False
        self._ts_pulse_aborted = False
        self._ts_lp_seen_active = False
        # Defensive: always send M5 at completion, never leave laser on.
        # Use raw _send() instead of _send_laser_cmd() to avoid stealing the
        # delayed OK that belongs to the just-finished M33 pulse.
        if self.serial_manager.is_connected:
            self._send("M5")
        self.laser_on = False
        self.laser_hw_on = False
        self.laser_pulse_active = False
        self.laser_pulse_remaining_ms = 0
        self._update_laser_indicator()
        self.ts_run_btn.config(state=tk.NORMAL)
        self.ts_stop_btn.config(state=tk.DISABLED)
        self.ts_status.config(text=msg, fg="#4caf50")
        self._log(f"[TS] {msg}", "info")

    def _ts_tick_step(self):
        """Execute one phase of the Tick Straight state machine.

        New pulse flow:
            tick_pulse_send -> wait_pulse_done -> move -> wait_idle -> settle -> check_pos

        Laser pulse timing is now performed by firmware via:
            M33 D<ms>
        so GUI no longer uses Tkinter after() as the pulse-width source.
        """
        if not self._ts_running:
            return

        if self._ts_state == "tick_pulse_send":
            tick_no = self._ts_step_index + 1
            self._ts_pulse_done = False
            self._ts_pulse_aborted = False
            self._ts_lp_seen_active = False
            self.laser_pulse_active = True
            self.laser_on = True
            self._update_laser_indicator()
            self._send_laser_cmd(f"M33 D{self._ts_delay_ms}")
            self.ts_status.config(
                text=f"⚡ Tick {tick_no}/{self._ts_total_steps + 1}: Pulse {self._ts_delay_ms}ms",
                fg="#ff9800")
            self._ts_wait_start_ms = int(time.time() * 1000)
            self._ts_state = "wait_pulse_done"
            self._ts_after_id = self.root.after(10, self._ts_tick_step)
            return

        if self._ts_state == "wait_pulse_done":
            elapsed_ms = int(time.time() * 1000) - self._ts_wait_start_ms
            if self._ts_pulse_aborted:
                self._ts_finish("⚠ Pulse aborted")
                return

            if self._ts_pulse_done:
                self._ts_pulse_done = False
                self.laser_pulse_active = False
                self.laser_pulse_remaining_ms = 0
                self.laser_on = False
                self._update_laser_indicator()

                remaining = self._ts_total_distance - self._ts_distance_done
                if remaining < 1e-4:
                    self._ts_finish(f"✔ Hoàn tất {self._ts_total_steps + 1} ticks")
                    return

                self._ts_state = "move"
                self._ts_after_id = self.root.after(20, self._ts_tick_step)
                return

            timeout_ms = max(2000, self._ts_delay_ms + 1000)
            remain = self.laser_pulse_remaining_ms
            if remain > 0:
                tick_no = self._ts_step_index + 1
                self.ts_status.config(
                    text=f"⚡ Tick {tick_no}/{self._ts_total_steps + 1}: waiting done ({remain}ms left)",
                    fg="#ff9800")
            if elapsed_ms > timeout_ms:
                self._log(f"[TS] pulse timeout (> {timeout_ms}ms)", "error")
                self._ts_finish("⚠ Timeout chờ pulse done")
                return

            self._ts_after_id = self.root.after(10, self._ts_tick_step)
            return

        if self._ts_state == "move":
            remaining = self._ts_total_distance - self._ts_distance_done
            step = min(self._ts_interval_mm, remaining)
            ux, uy, uz = self._ts_unit_vec
            sx, sy, sz = self._ts_start_pos
            done_after = self._ts_distance_done + step
            nx = sx + ux * done_after
            ny = sy + uy * done_after
            nz = sz + uz * done_after
            self._ts_expected_pos   = (nx, ny, nz)
            self._ts_distance_done  = done_after
            self._ts_step_index    += 1

            parts = ["G1"]
            cx, cy, cz = self.pos_x.get(), self.pos_y.get(), self.pos_z.get()
            if abs(nx - cx) > 1e-5: parts.append(f"X{nx:.4f}")
            if abs(ny - cy) > 1e-5: parts.append(f"Y{ny:.4f}")
            if abs(nz - cz) > 1e-5: parts.append(f"Z{nz:.4f}")
            parts.append(f"F{self._ts_feed:.0f}")
            self._send(" ".join(parts))

            self.ts_status.config(
                text=f"➡ Move {self._ts_step_index}/{self._ts_total_steps}: {step:.3f}mm → ({nx:.2f},{ny:.2f},{nz:.2f})",
                fg="#2196F3")
            self.ts_progress["value"] = self._ts_step_index

            self._ts_saw_busy      = False
            self._ts_wait_start_ms = int(time.time() * 1000)
            self._ts_state         = "wait_idle"
            try:
                self.serial_manager.send_realtime("?")
            except Exception:
                pass
            self._ts_after_id = self.root.after(80, self._ts_tick_step)
            return

        if self._ts_state == "wait_idle":
            cur_state = (self.state_var.get() or "").lower()
            elapsed_ms = int(time.time() * 1000) - self._ts_wait_start_ms

            if elapsed_ms > 30000:
                self._log(f"[TS] wait_idle TIMEOUT (>30s, state={cur_state}) - aborting", "error")
                self._ts_finish("⚠ Timeout chờ Idle - dừng")
                return

            # Mark busy if GUI catches Run/Hold/Jog. For very short moves the
            # 250ms status polling may miss Run completely, so do not require it.
            if cur_state in ("run", "hold", "jog"):
                if not self._ts_saw_busy:
                    self._ts_saw_busy = True
                    self._log(f"[TS] saw busy state={cur_state} after {elapsed_ms}ms", "debug")
                self._ts_after_id = self.root.after(100, self._ts_tick_step)
                return

            # Accept Idle when either we have seen busy, or after a grace time.
            # This prevents short-grid moves from hanging forever in wait_idle.
            if cur_state == "idle" and (self._ts_saw_busy or elapsed_ms >= 500):
                try:
                    self.serial_manager.send_realtime("?")
                except Exception:
                    pass

                if self._ts_settle_ms > 0:
                    self._ts_state = "settle"
                    self.ts_status.config(
                        text=f"⏳ Settle {self._ts_settle_ms}ms...", fg="#9c27b0")
                    self._ts_after_id = self.root.after(self._ts_settle_ms,
                                                        self._ts_tick_step)
                else:
                    self._ts_state = "check_pos"
                    self._ts_after_id = self.root.after(80, self._ts_tick_step)
                return

            if elapsed_ms < 2000:
                try:
                    self.serial_manager.send_realtime("?")
                except Exception:
                    pass
            self._ts_after_id = self.root.after(50, self._ts_tick_step)
            return

        if self._ts_state == "settle":
            try:
                self.serial_manager.send_realtime("?")
            except Exception:
                pass
            self._ts_state = "check_pos"
            self._ts_after_id = self.root.after(80, self._ts_tick_step)
            return

        if self._ts_state == "check_pos":
            ex, ey, ez = self._ts_expected_pos
            cx, cy, cz = self.pos_x.get(), self.pos_y.get(), self.pos_z.get()
            pos_err = math.sqrt((ex-cx)**2 + (ey-cy)**2 + (ez-cz)**2)

            self.ts_status.config(text=f"🔍 Check pos drift={pos_err:.3f}", fg="#607d8b")

            if pos_err > self._ts_pos_tol:
                self._log(f"[TS] WARN: pos mismatch err={pos_err:.3f} > tol={self._ts_pos_tol} "
                          f"expected=({ex:.2f},{ey:.2f},{ez:.2f}) "
                          f"actual=({cx:.2f},{cy:.2f},{cz:.2f})", "error")

                if self._ts_autocorrect and self._ts_correct_count < 3:
                    self._ts_correct_count += 1
                    parts = ["G1"]
                    if abs(ex - cx) > 1e-5: parts.append(f"X{ex:.4f}")
                    if abs(ey - cy) > 1e-5: parts.append(f"Y{ey:.4f}")
                    if abs(ez - cz) > 1e-5: parts.append(f"Z{ez:.4f}")
                    parts.append(f"F{self._ts_feed * 0.5:.0f}")
                    correct_cmd = " ".join(parts)
                    self._log(f"[TS] AUTO-CORRECT #{self._ts_correct_count}: {correct_cmd}", "info")
                    self._send(correct_cmd)
                    self.ts_status.config(
                        text=f"🎯 Auto-correct drift={pos_err:.2f}",
                        fg="#9c27b0")
                    self._ts_saw_busy = False
                    self._ts_wait_start_ms = int(time.time() * 1000)
                    self._ts_state = "wait_idle"
                    self._ts_after_id = self.root.after(80, self._ts_tick_step)
                    return
            elif pos_err > 0.01:
                self._log(f"[TS] pos ok (drift={pos_err:.3f} within tol={self._ts_pos_tol})",
                          "debug")

            self._ts_correct_count = 0
            self._ts_state = "tick_pulse_send"
            self._ts_after_id = self.root.after(20, self._ts_tick_step)
            return

    def _create_right_panel(self, parent):
        log_frame = ttk.LabelFrame(parent, text="\U0001f4dc UART Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = scrolledtext.ScrolledText(log_frame, font=("Consolas", 9), height=15,
                                                   bg='#1e1e1e', fg='#d4d4d4', insertbackground='white')
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state=tk.DISABLED)

        self.log_text.tag_configure("sent", foreground="#569cd6")
        self.log_text.tag_configure("recv", foreground="#6a9955")
        self.log_text.tag_configure("error", foreground="#f14c4c")
        self.log_text.tag_configure("info", foreground="#808080")
        self.log_text.tag_configure("debug", foreground="#aaaa00")

        ctrl = ttk.Frame(log_frame)
        ctrl.pack(fill=tk.X, pady=3)

        ttk.Button(ctrl, text="\U0001f5d1\ufe0f", width=3, command=self._clear_log).pack(side=tk.LEFT)
        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl, text="Auto", variable=self.autoscroll_var).pack(side=tk.LEFT, padx=5)
        self.show_status_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctrl, text="Status", variable=self.show_status_var).pack(side=tk.LEFT)
        ttk.Checkbutton(ctrl, text="Poll ?", variable=self.gui_poll_status_var).pack(side=tk.LEFT, padx=5)

        cmd_frame = ttk.Frame(log_frame)
        cmd_frame.pack(fill=tk.X, pady=5)

        self.cmd_entry = ttk.Entry(cmd_frame, font=("Consolas", 10))
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.cmd_entry.bind("<Return>", self._send_command)

        ttk.Button(cmd_frame, text="\U0001f4e4", width=3, command=self._send_command).pack(side=tk.LEFT)

        quick = ttk.Frame(log_frame)
        quick.pack(fill=tk.X, pady=3)

        for cmd in ["?", "$$", "HELP", "LIMITS", "G90", "G91", "MCURESET"]:
            ttk.Button(quick, text=cmd, width=(8 if cmd == "MCURESET" else 5),
                       command=lambda c=cmd: self._quick_send(c)).pack(side=tk.LEFT, padx=1)

    # ==================== Laser Indicator ====================

    def _update_laser_indicator(self):
        display_on = self.laser_on or self.laser_hw_on or self.laser_pulse_active
        if display_on:
            self.laser_indicator.itemconfig(self.laser_dot, fill='#ff1744', outline='#d50000')
            self.laser_indicator.delete("glow")
            self.laser_indicator.create_oval(0, 0, 16, 16,
                                             fill='', outline='#ff1744', width=2,
                                             tags="glow")
            for child in self.laser_frame.winfo_children():
                if isinstance(child, tk.Label) and child.cget('text') == 'LASER':
                    child.config(foreground='#d50000')
        else:
            self.laser_indicator.itemconfig(self.laser_dot, fill='#555', outline='#888')
            self.laser_indicator.delete("glow")
            for child in self.laser_frame.winfo_children():
                if isinstance(child, tk.Label) and child.cget('text') == 'LASER':
                    child.config(foreground='#555')

    def _send_laser_cmd(self, cmd):
        cmd_up = cmd.upper().strip()
        if cmd_up in ("M3", "M5") or cmd_up.startswith("M33"):
            self._pending_laser_cmd = cmd_up
        if cmd_up.startswith("M33"):
            self.laser_pulse_active = True
            self.laser_on = True
            self._update_laser_indicator()
        self._send(cmd)

    # ==================== Connection ====================

    def _refresh_ports(self):
        ports = self.serial_manager.get_available_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])

    def _toggle_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
            self.conn_compact.pack_forget()
            self.status_frame.pack_forget()
            self.conn_expanded.pack(fill=tk.X)
            self.conn_status.config(text="\u25cf Disconnected", foreground="red")
            self._update_state("Disconnected", True)
            self.laser_on = False
            self.laser_hw_on = False
            self.laser_pulse_active = False
            self.laser_pulse_remaining_ms = 0
            self._pending_laser_cmd = None
            self._update_laser_indicator()
        else:
            if self.serial_manager.connect(self.port_combo.get(), int(self.baud_combo.get())):
                self.conn_expanded.pack_forget()
                self.compact_label.config(text=f"\U0001f4e1 {self.port_combo.get()}")
                self.conn_compact.pack(fill=tk.X)
                self.status_frame.pack(side=tk.LEFT, fill=tk.X, padx=(10, 0))
                self._update_state("Connected", True)
                self._log(f"Connected to {self.port_combo.get()}", "info")

    # ==================== Actions ====================

    def _jog(self, axis, direction):
        try:
            dist = float(self.distance_var.get()) * direction
            feed = float(self.feedrate_var.get())
            self._send(f"G91 G1 {axis}{dist:.3f} F{feed:.0f}")
            self.pending_commands.append("G90")
            self.waiting_for_ok = True
            self.jog_status.config(text=f"{axis}{dist:+.1f}")
        except:
            pass

    def _home(self):
        self._send("HOME")

    def _feed_hold(self):
        """Feed-hold: send '!' realtime. Machine: Run -> Hold (can RESUME)."""
        if not self.serial_manager.is_connected:
            return
        self.serial_manager.send_realtime('!')
        self._log(">>> ! (feed-hold)", "sent")

    def _resume(self):
        """Resume from Hold: send '~' realtime. Machine: Hold -> Run -> Idle."""
        if not self.serial_manager.is_connected:
            return
        self.serial_manager.send_realtime('~')
        self._log(">>> ~ (resume)", "sent")

    def _soft_reset(self):
        """Soft-reset (Ctrl-X 0x18). Any state -> Idle. Motion is aborted.
        Also clears pending command queue and any running sequences (no laser change)."""
        # Cancel local pending state
        self.pending_commands.clear()
        self.waiting_for_ok = False
        self._sequence_running = False
        self._sequence_commands = []
        self._sequence_idx = 0
        self._sequence_laser_after = False
        if getattr(self, "_ts_running", False):
            self._ts_running = False
            self._ts_state = "idle"
            if self._ts_after_id:
                try: self.root.after_cancel(self._ts_after_id)
                except Exception: pass
                self._ts_after_id = None
            self.ts_run_btn.config(state=tk.NORMAL)
            self.ts_stop_btn.config(state=tk.DISABLED)
            self.ts_status.config(text="\u23fc Soft-reset", fg="#f44336")
        if self.serial_manager.is_connected:
            self.serial_manager.send_realtime(0x18)
            self._log(">>> 0x18 (soft-reset)", "sent")
        self.laser_on = False
        self.laser_hw_on = False
        self.laser_pulse_active = False
        self.laser_pulse_remaining_ms = 0
        self._pending_laser_cmd = None
        self._update_laser_indicator()
        self.jog_status.config(text="RESET")

    def _stop(self):
        """Emergency stop: GRBL soft-reset (Ctrl-X 0x18) + STOP text + M5.
        Also aborts Tick Straight sequence if running."""
        self.pending_commands.clear()
        self.waiting_for_ok = False
        self._sequence_running = False
        self._sequence_commands = []
        self._sequence_idx = 0
        self._sequence_laser_after = False
        # Abort Tick Straight if active - always force M5
        if getattr(self, "_ts_running", False):
            self._ts_running = False
            self._ts_state = "idle"
            if self._ts_after_id:
                try: self.root.after_cancel(self._ts_after_id)
                except Exception: pass
                self._ts_after_id = None
            # Force M5 regardless of laser_on flag (may be stale)
            if self.serial_manager.is_connected:
                self._send_laser_cmd("M5")
            self.laser_on = False
            self._update_laser_indicator()
            self.ts_run_btn.config(state=tk.NORMAL)
            self.ts_stop_btn.config(state=tk.DISABLED)
            self.ts_status.config(text="\u23f9 Emergency - Laser OFF",
                                  fg="#f44336")
        if self.serial_manager.is_connected:
            # Realtime soft reset (single byte, NO newline)
            self.serial_manager.send_realtime(0x18)
            # Belt-and-suspenders: also send STOP text command (firmware accepts both)
            self._send("STOP")
            if self.laser_on or self.laser_hw_on or self.laser_pulse_active:
                self._send_laser_cmd("M5")
        self.laser_on = False
        self.laser_hw_on = False
        self.laser_pulse_active = False
        self.laser_pulse_remaining_ms = 0
        self._pending_laser_cmd = None
        self._update_laser_indicator()
        self.jog_status.config(text="STOP")

    def _clear(self):
        self._send("CLEAR")

    def _zero_all(self):
        self._send("G92 X0 Y0 Z0")

    def _use_current_pos(self):
        self.gwd_x.set(f"{self.pos_x.get():.3f}")
        self.gwd_y.set(f"{self.pos_y.get():.3f}")
        self.gwd_z.set(f"{self.pos_z.get():.3f}")

    def _clear_gwd_pos(self):
        self.gwd_x.set("")
        self.gwd_y.set("")
        self.gwd_z.set("")

    def _update_gwd_preview(self, *args):
        try:
            parts = ["GOWDELAY"]
            for axis, var in [("X", self.gwd_x), ("Y", self.gwd_y), ("Z", self.gwd_z)]:
                if var.get().strip():
                    parts.append(f"{axis}{var.get()}")
            parts.extend([f"D{self.gwd_step.get()}", f"W{self.gwd_delay.get()}", f"F{self.gwd_feedrate.get()}"])
            self.gwd_preview.config(text=" ".join(parts))
        except:
            pass

    def _update_rect_preview(self, *args):
        try:
            w, h, d = float(self.rect_width.get()), float(self.rect_height.get()), float(self.rect_step.get())
            cmd = f"GOWDELAYRECT X{w:.1f} Y{h:.1f} D{d:.1f} W{self.rect_delay.get()} F{self.rect_feedrate.get()}"
            self.rect_preview.config(text=cmd)
            rows = int(h / d) + 1
            steps = int(w / d) * rows
            self.rect_stats.config(text=f"\u2248 {rows} rows \u00d7 {int(w/d)} = {steps} steps")
            self.rect_pattern.update_params(w, h, d)
        except:
            pass

    # ==================== Command Sequence Engine ====================

    def _start_sequence(self, commands, laser_before=False, laser_after=False, laser_delay_ms=500):
        """Start a command sequence with optional laser control.
        
        M5 (laser off) is NOT added to the sequential command list.
        Instead, it is sent AFTER receiving a GOWDELAY COMPLETE or
        GOWDELAYRECT COMPLETE message from the firmware.
        """
        full_sequence = []
        self._sequence_laser_before = laser_before
        self._sequence_laser_after = laser_after
        self._sequence_laser_delay_ms = laser_delay_ms

        if laser_before:
            full_sequence.append("M3")
        full_sequence.extend(commands)

        if not full_sequence:
            return

        self._sequence_commands = full_sequence
        self._sequence_idx = 0
        self._sequence_running = True

        self._log(f"[DBG] Sequence started: {full_sequence}, laser_after={laser_after}", "debug")

        cmd = self._sequence_commands[0]
        if cmd in ("M3", "M5"):
            self._pending_laser_cmd = cmd
        self._log(f">>> [SEQ] {cmd}", "sent")
        self.serial_manager.send(cmd)

    def _advance_sequence(self):
        if not self._sequence_running:
            return

        self._sequence_idx += 1

        if self._sequence_idx >= len(self._sequence_commands):
            self._log("[SEQ] All commands sent, waiting for COMPLETE...", "info")
            return

        cmd = self._sequence_commands[self._sequence_idx]
        prev_cmd = self._sequence_commands[self._sequence_idx - 1]

        if prev_cmd == "M3" and self._sequence_laser_delay_ms > 0:
            delay_ms = self._sequence_laser_delay_ms
            self._log(f"[SEQ] Waiting {delay_ms}ms for laser warm-up...", "info")
            self.root.after(delay_ms, lambda: self._send_next_sequence_cmd(cmd))
        else:
            self._send_next_sequence_cmd(cmd)

    def _send_next_sequence_cmd(self, cmd):
        if not self._sequence_running:
            return
        if cmd in ("M3", "M5"):
            self._pending_laser_cmd = cmd
        self._log(f">>> [SEQ] {cmd}", "sent")
        self.serial_manager.send(cmd)

    # ==================== Execute GOWDELAY / GOWDELAYRECT ====================

    def _execute_gowdelay(self):
        try:
            x, y, z = self.gwd_x.get().strip(), self.gwd_y.get().strip(), self.gwd_z.get().strip()
            if not any([x, y, z]):
                messagebox.showerror("Error", "Specify axis")
                return
            d, w, f = float(self.gwd_step.get()), int(self.gwd_delay.get()), float(self.gwd_feedrate.get())
            parts = ["GOWDELAY"]
            if x: parts.append(f"X{float(x):.3f}")
            if y: parts.append(f"Y{float(y):.3f}")
            if z: parts.append(f"Z{float(z):.3f}")
            parts.extend([f"D{d:.3f}", f"W{w}", f"F{f:.0f}"])
            gwd_cmd = " ".join(parts)

            try:
                laser_delay = int(self.gwd_laser_delay.get())
            except ValueError:
                laser_delay = 500

            self._start_sequence(
                commands=[gwd_cmd],
                laser_before=self.gwd_laser_before.get(),
                laser_after=self.gwd_laser_after.get(),
                laser_delay_ms=laser_delay
            )
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _execute_gowdelayrect(self):
        try:
            w, h, d = float(self.rect_width.get()), float(self.rect_height.get()), float(self.rect_step.get())
            delay, f = int(self.rect_delay.get()), float(self.rect_feedrate.get())
            rect_cmd = f"GOWDELAYRECT X{w:.3f} Y{h:.3f} D{d:.3f} W{delay} F{f:.0f}"

            try:
                laser_delay = int(self.rect_laser_delay.get())
            except ValueError:
                laser_delay = 500

            self._start_sequence(
                commands=[rect_cmd],
                laser_before=self.rect_laser_before.get(),
                laser_after=self.rect_laser_after.get(),
                laser_delay_ms=laser_delay
            )
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _send(self, cmd):
        if self.serial_manager.is_connected:
            self.serial_manager.send(cmd)
            self._log(f">>> {cmd}", "sent")
        else:
            messagebox.showwarning("Warning", "Not connected")

    def _send_command(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            up = cmd.upper().strip()
            if up in ("M3", "M5") or up.startswith("M33"):
                self._pending_laser_cmd = up
            self._send(cmd)
            self.cmd_entry.delete(0, tk.END)

    def _quick_send(self, cmd):
        up = cmd.upper().strip()
        if up in ("M3", "M5") or up.startswith("M33"):
            self._pending_laser_cmd = up
        self._send(cmd)

    def _log(self, msg, tag=None):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n", tag)
        if int(self.log_text.index('end-1c').split('.')[0]) > 500:
            self.log_text.delete('1.0', '100.0')
        if self.autoscroll_var.get():
            self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete('1.0', tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _open_config(self):
        if not self.config_window or not self.config_window.winfo_exists():
            self.config_window = ConfigWindow(self.root, self.serial_manager, self)
        else:
            self.config_window.lift()

    # ==================== Polling ====================

    def _start_polling(self):
        self._poll_serial()
        self._poll_status()

    def _poll_status(self):
        """Optional GRBL realtime '?' polling.

        Firmware grbl_cnc auto-reports compact status by default ($40=300ms),
        so periodic GUI polling is disabled unless the user ticks "Poll ?".
        Manual '?' buttons still work.
        """
        try:
            if self.serial_manager.is_connected and self.gui_poll_status_var.get():
                self.serial_manager.send_realtime('?')
        except Exception:
            pass
        self.root.after(250, self._poll_status)

    def _poll_serial(self):
        while not self.serial_manager.rx_queue.empty():
            try:
                self._process_received(self.serial_manager.rx_queue.get_nowait())
            except:
                break
        self.root.after(20, self._poll_serial)

    def _process_received(self, line):
        # GRBL v1.1 welcome string: "Grbl 1.1f ['$' for help]"
        if line.startswith("Grbl ") and "for help" in line:
            self._log(f"<<< {line}", "info")
            self._update_state("Idle", True)
            self.root.after(100, lambda: self.serial_manager.send_realtime('?'))
            return

        if self.config_read_active:
            if re.match(r'^\$\d+\s*=\s*[-+]?\d', line):
                self.config_read_buffer.append(line)
                return
            if line.strip().lower() == 'ok' and len(self.config_read_buffer) > 0:
                self.config_read_active = False
                if self.config_read_callback:
                    self.config_read_callback(self.config_read_buffer)
                return
            return

        line_upper = line.upper().strip()

        # 1) Pulse async messages - must be handled before generic OK
        if "PULSE DONE" in line_upper:
            self._log(f"<<< {line}", "recv")
            self.laser_on = False
            self.laser_hw_on = False
            self.laser_pulse_active = False
            self.laser_pulse_remaining_ms = 0
            self._ts_pulse_done = True
            self._ts_lp_seen_active = False
            self._pending_laser_cmd = None
            self._update_laser_indicator()
            return

        if "PULSE ABORTED" in line_upper:
            self._log(f"<<< {line}", "error")
            self.laser_on = False
            self.laser_hw_on = False
            self.laser_pulse_active = False
            self.laser_pulse_remaining_ms = 0
            self._ts_pulse_aborted = True
            self._ts_lp_seen_active = False
            self._pending_laser_cmd = None
            self._update_laser_indicator()
            return

        # 2) GOWDELAY / GOWDELAYRECT COMPLETE
        if "GOWDELAY COMPLETE" in line_upper or "GOWDELAYRECT COMPLETE" in line_upper:
            self._log(f"<<< {line}", "recv")
            debug_info = (
                f"[DBG] COMPLETE received! line_upper='{line_upper}', "
                f"seq_running={self._sequence_running}, "
                f"laser_after={self._sequence_laser_after}, "
                f"laser_on={self.laser_on}"
            )
            self._log(debug_info, "debug")

            if self._sequence_laser_after:
                self._log("[SEQ] laser_after=True, sending M5 to turn off laser", "info")
                self._send_laser_cmd("M5")
            else:
                self._log("[SEQ] laser_after=False, NOT sending M5", "debug")

            self._sequence_running = False
            self._sequence_commands = []
            self._sequence_idx = 0
            self._sequence_laser_after = False
            self._log("[SEQ] Sequence complete", "info")
            self.jog_status.config(text="")
            return

        # 3) OK
        if line_upper == "OK":
            self._log(f"<<< {line}", "recv")
            self.jog_status.config(text="")

            if self._pending_laser_cmd == "M3":
                self.laser_on = True
                self.laser_hw_on = True
                self._update_laser_indicator()
                self._log("[LASER] ON", "info")
                self._pending_laser_cmd = None
            elif self._pending_laser_cmd == "M5":
                self.laser_on = False
                self.laser_hw_on = False
                self.laser_pulse_active = False
                self.laser_pulse_remaining_ms = 0
                self._update_laser_indicator()
                self._log("[LASER] OFF", "info")
                self._pending_laser_cmd = None
            elif self._pending_laser_cmd and self._pending_laser_cmd.startswith("M33"):
                self._pending_laser_cmd = None

            if self._sequence_running:
                self._advance_sequence()
                return

            if self.waiting_for_ok and self.pending_commands:
                self._send(self.pending_commands.pop(0))
                if not self.pending_commands:
                    self.waiting_for_ok = False
            else:
                self.waiting_for_ok = False
            return

        # 4) ERROR
        if line_upper.startswith("ERROR"):
            self._log(f"<<< {line}", "error")
            if self._pending_laser_cmd and self._pending_laser_cmd.startswith("M33"):
                self.laser_on = False
                self.laser_hw_on = False
                self.laser_pulse_active = False
                self.laser_pulse_remaining_ms = 0
                self._ts_pulse_aborted = True
                self._ts_lp_seen_active = False
                self._pending_laser_cmd = None
                self._update_laser_indicator()
            if self._ts_running:
                self._ts_finish("⚠ Firmware error - dừng Tick Straight")
            self.pending_commands.clear()
            self.waiting_for_ok = False
            return

        # 5) Status reports <...>
        if line.startswith('<') and line.endswith('>'):
            self._parse_status(line)
            if self.show_status_var.get():
                self._log(f"<<< {line}", "info")
            return

        # 6) Emergency Stop
        if "EMERGENCY" in line_upper or "EM STOP" in line_upper:
            self._log(f"<<< {line}", "error")
            if self.laser_on or self.laser_hw_on or self.laser_pulse_active:
                self._send_laser_cmd("M5")
                self._log("[EMERGENCY] Laser turned OFF", "error")
            self.laser_on = False
            self.laser_hw_on = False
            self.laser_pulse_active = False
            self.laser_pulse_remaining_ms = 0
            self._update_laser_indicator()
            self._sequence_running = False
            self._sequence_commands = []
            self._sequence_idx = 0
            self._sequence_laser_after = False
            self.pending_commands.clear()
            self.waiting_for_ok = False
            if self._ts_running:
                self._ts_running = False
                self._ts_state = "idle"
                if self._ts_after_id:
                    try:
                        self.root.after_cancel(self._ts_after_id)
                    except Exception:
                        pass
                    self._ts_after_id = None
                self.ts_run_btn.config(state=tk.NORMAL)
                self.ts_stop_btn.config(state=tk.DISABLED)
                self.ts_status.config(text="⚠ Emergency / Pulse stopped", fg="#f44336")
            return

        # 7) ALARM
        if line_upper.startswith("ALARM"):
            self._log(f"<<< {line}", "error")
            if self.laser_on or self.laser_hw_on or self.laser_pulse_active:
                self._send_laser_cmd("M5")
            self.laser_on = False
            self.laser_hw_on = False
            self.laser_pulse_active = False
            self.laser_pulse_remaining_ms = 0
            self._update_laser_indicator()
            self._sequence_running = False
            if self._ts_running:
                self._ts_running = False
                self._ts_state = "idle"
                if self._ts_after_id:
                    try:
                        self.root.after_cancel(self._ts_after_id)
                    except Exception:
                        pass
                    self._ts_after_id = None
                self.ts_run_btn.config(state=tk.NORMAL)
                self.ts_stop_btn.config(state=tk.DISABLED)
                self.ts_status.config(text="⚠ Alarm - laser off", fg="#f44336")
            return

        # 8) Everything else
        self._log(f"<<< {line}", "recv")

    def _update_state(self, state, force=False):
        if force or state != self._prev_state:
            self._prev_state = state
            self.state_var.set(state)
            colors = {"idle": "#4caf50", "run": "#2196f3", "hold": "#ff9800", "homing": "#9c27b0",
                     "alarm": "#f44336", "limit": "#f44336", "gowdelay": "#00bcd4", "gowdelayrect": "#00bcd4",
                     "disconnected": "#9e9e9e", "connected": "#9e9e9e"}
            self.state_label.config(foreground=colors.get(state.lower(), "#9e9e9e"))

    def _update_position(self, x, y, z):
        for i, (val, axis) in enumerate([(x, "X"), (y, "Y"), (z, "Z")]):
            if self._prev_pos[i] is None or abs(val - self._prev_pos[i]) > 0.0005:
                self._prev_pos[i] = val
                [self.pos_x, self.pos_y, self.pos_z][i].set(val)
                self.pos_labels[axis].config(text=f"{val:.3f}")
        self.xy_viz.update_position(x, y)

    def _parse_status(self, line):
        """GRBL v1.1 + custom status parser.
        Supports extra firmware fields:
            Sp:<0|1>      current spindle/laser relay state
            LP:<a,remain> pulse active flag + remaining ms
        """
        try:
            content = line[1:-1]
            parts = content.split('|')
            if not parts:
                return

            state_main = parts[0].split(':')[0]
            state_map = {
                "Idle": "Idle", "Run": "Run", "Hold": "Hold",
                "Home": "Homing", "Alarm": "Alarm", "Door": "Alarm",
                "Check": "Idle", "Jog": "Run", "Sleep": "Idle",
                "I": "Idle", "R": "Run", "H": "Hold", "A": "Alarm",
                "L": "Limit", "G": "Homing",
                "D": "GoWDelay", "E": "GoWDelayRect",
            }
            self._update_state(state_map.get(state_main, state_main))

            laser_changed = False
            for field in parts[1:]:
                if field.startswith("MPos:") or field.startswith("WPos:"):
                    pos = field.split(':', 1)[1].split(',')
                    if len(pos) >= 3:
                        self._update_position(float(pos[0]), float(pos[1]), float(pos[2]))
                elif ':' not in field and ',' in field:
                    # Firmware auto-report compact format:
                    #   <I|x,y,z|Sp:0|LP:0,0|L000|R000>
                    # GRBL request format remains:
                    #   <Idle|MPos:x,y,z|FS:f,s|...>
                    try:
                        pos = field.split(',')
                        if len(pos) >= 3:
                            self._update_position(float(pos[0]), float(pos[1]), float(pos[2]))
                    except Exception:
                        pass
                elif field.startswith("Sp:"):
                    try:
                        self.laser_hw_on = bool(int(field.split(':', 1)[1]))
                        laser_changed = True
                    except Exception:
                        pass
                elif field.startswith("LP:"):
                    try:
                        vals = field.split(':', 1)[1].split(',')
                        prev_active = self.laser_pulse_active
                        new_active = bool(int(vals[0]))
                        self.laser_pulse_active = new_active
                        self.laser_pulse_remaining_ms = int(vals[1]) if len(vals) > 1 else 0

                        # Fallback for lost async "[PULSE DONE]".
                        # Important race guard: do NOT use GUI's local pre-set
                        # laser_pulse_active as proof that firmware started the pulse,
                        # because an old LP:0,0 status can arrive after M33 is sent.
                        # Only accept LP:0 after this same pulse has reported LP:1.
                        if self._ts_running and self._ts_state == "wait_pulse_done":
                            if new_active:
                                self._ts_lp_seen_active = True
                            elif self._ts_lp_seen_active:
                                self._ts_pulse_done = True
                                self._ts_lp_seen_active = False
                        laser_changed = True
                    except Exception:
                        pass

            if laser_changed:
                if not self.laser_hw_on and not self.laser_pulse_active and self._pending_laser_cmd != "M3":
                    self.laser_on = False
                self._update_laser_indicator()
        except Exception:
            pass

    def on_closing(self):
        if (self.laser_on or self.laser_hw_on or self.laser_pulse_active) and self.serial_manager.is_connected:
            self.serial_manager.send("M5")
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CNCControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
