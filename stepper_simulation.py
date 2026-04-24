"""
Stepper Motor Open-Loop Simulation
===================================
Interactive simulation demonstrating step loss under load
using a pulley-weight mechanism.

- Lifting (Forward) → Motor rotates Anti-Clockwise (CCW)
- Lowering (Reverse) → Motor rotates Clockwise (CW)

Author: NANDHAKUMAR J
"""

import tkinter as tk
from tkinter import ttk, font as tkfont
import math
import time

# ============================================================
# CONSTANTS
# ============================================================
PPR = 200                   # Pulses per revolution
STEP_ANGLE_DEG = 1.8        # degrees per step
PULLEY_RADIUS_M = 0.02      # meters (20 mm)
PULLEY_RADIUS_MM = 20       # mm
T_AVAILABLE = 1.0           # Nm  (motor holding torque)
GRAVITY = 9.81              # m/s²
MAX_LOAD_KG = 15            # kg
MAX_RPM = 300

# Colors — Dark engineering theme
C_BG_DARK = "#0a0e17"
C_BG_PANEL = "#111827"
C_BG_CARD = "#1a2336"
C_BG_INPUT = "#0d1321"
C_BORDER = "#1e2d45"
C_BORDER_ACCENT = "#2a3f63"
C_TEXT = "#e8ecf4"
C_TEXT_SEC = "#8b95a8"
C_TEXT_MUTED = "#5a6478"
C_CYAN = "#00d4ff"
C_PURPLE = "#7b2ff7"
C_GREEN = "#00e676"
C_RED = "#ff4060"
C_ORANGE = "#ff9800"
C_YELLOW = "#ffd600"
C_MOTOR_BODY = "#1e2d45"
C_MOTOR_BORDER = "#3a5280"
C_SHAFT = "#4a5568"


# ============================================================
# PHYSICS ENGINE
# ============================================================
class PhysicsEngine:
    """Calculates step loss, displacement, and torque."""

    @staticmethod
    def expected_displacement_mm(steps):
        """D = (steps / PPR) × 2πr  in mm"""
        return (steps / PPR) * (2 * math.pi * PULLEY_RADIUS_M) * 1000

    @staticmethod
    def torque_required(mass_kg):
        """T_req = m × g × r"""
        return mass_kg * GRAVITY * PULLEY_RADIUS_M

    @staticmethod
    def actual_steps(commanded, mass_kg):
        t_req = PhysicsEngine.torque_required(mass_kg)
        if t_req <= T_AVAILABLE:
            return commanded
        ratio = T_AVAILABLE / t_req
        return int(commanded * ratio)

    @staticmethod
    def step_rate(rpm):
        """Steps per second at given RPM."""
        if rpm <= 0:
            return 0
        return (rpm / 60.0) * PPR


# ============================================================
# MAIN APPLICATION
# ============================================================
class StepperSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Stepper Motor Open-Loop Simulation — Step Loss Demo")
        self.configure(bg=C_BG_DARK)
        self.state("zoomed")  # start maximized on Windows
        self.minsize(1100, 650)

        # ---- State ----
        self.running = False
        self.direction = 1          # 1 = Forward (Lift / CCW), -1 = Reverse (Lower / CW)
        self.rpm_var = tk.IntVar(value=60)
        self.load_var = tk.DoubleVar(value=0.0)
        self.steps_str_var = tk.StringVar(value="1000")  # StringVar to avoid IntVar crash
        self._steps_value = 1000  # last valid parsed value
        self._steps_debounce_id = None  # debounce timer for step entry

        # Animation state
        self.anim_step = 0.0        # current commanded step (float, animated)
        self.anim_actual = 0.0      # current actual step (float, animated)
        self.pulley_angle = 0.0     # radians (visual rotation)
        self.progress = 0.0
        self.last_time = 0.0

        # Computed results (snapshot at start)
        self.commanded = 0
        self.actual = 0
        self.expected_disp = 0.0
        self.actual_disp = 0.0
        self.missed = 0
        self.has_step_loss = False
        self.has_stall = False

        # Log history
        self.log_lines = []

        # Graph data (load → missed steps for current commanded)
        self.graph_data = []

        # ---- Build UI ----
        self._build_fonts()
        self._build_layout()
        self._compute_static()     # initial calculation
        self._update_data_panel()
        self._update_graph_data()
        self._draw_simulation()

        # Bind resize
        self.sim_canvas.bind("<Configure>", lambda e: self._draw_simulation())

    # ----------------------------------------------------------
    # Fonts
    # ----------------------------------------------------------
    def _build_fonts(self):
        self.font_title = tkfont.Font(family="Segoe UI", size=11, weight="bold")
        self.font_section = tkfont.Font(family="Segoe UI", size=9, weight="bold")
        self.font_label = tkfont.Font(family="Segoe UI", size=9)
        self.font_value = tkfont.Font(family="Consolas", size=10, weight="bold")
        self.font_small = tkfont.Font(family="Consolas", size=8)
        self.font_header = tkfont.Font(family="Segoe UI", size=14, weight="bold")
        self.font_sub = tkfont.Font(family="Segoe UI", size=8)
        self.font_btn = tkfont.Font(family="Segoe UI", size=9, weight="bold")
        self.font_warn = tkfont.Font(family="Consolas", size=11, weight="bold")

    # ----------------------------------------------------------
    # Layout
    # ----------------------------------------------------------
    def _build_layout(self):
        # Header
        hdr = tk.Frame(self, bg=C_BG_PANEL, height=50)
        hdr.pack(fill="x", side="top")
        hdr.pack_propagate(False)

        tk.Label(hdr, text="⚙  Stepper Motor Simulation", font=self.font_header,
                 bg=C_BG_PANEL, fg=C_CYAN).pack(side="left", padx=16, pady=8)
        tk.Label(hdr, text="OPEN-LOOP  ·  STEP LOSS DEMONSTRATOR", font=self.font_sub,
                 bg=C_BG_PANEL, fg=C_TEXT_MUTED).pack(side="left", padx=4)

        # Status badge
        self.status_frame = tk.Frame(hdr, bg=C_BG_CARD, highlightbackground=C_BORDER,
                                     highlightthickness=1, padx=10, pady=2)
        self.status_frame.pack(side="right", padx=16, pady=10)
        self.status_dot = tk.Label(self.status_frame, text="●", font=self.font_small,
                                   bg=C_BG_CARD, fg=C_TEXT_MUTED)
        self.status_dot.pack(side="left")
        self.status_label = tk.Label(self.status_frame, text="IDLE", font=self.font_small,
                                     bg=C_BG_CARD, fg=C_TEXT_SEC)
        self.status_label.pack(side="left", padx=(4, 0))

        # Separator
        tk.Frame(self, bg=C_BORDER, height=1).pack(fill="x")

        # Main 3-column area
        main = tk.Frame(self, bg=C_BG_DARK)
        main.pack(fill="both", expand=True)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # LEFT  — Control Panel
        left = tk.Frame(main, bg=C_BG_PANEL, width=270)
        left.grid(row=0, column=0, sticky="nsew")
        left.grid_propagate(False)
        self._build_control_panel(left)

        # Vertical separator
        tk.Frame(main, bg=C_BORDER, width=1).grid(row=0, column=1, sticky="ns", padx=0)

        # CENTER — Simulation Canvas
        center = tk.Frame(main, bg=C_BG_DARK)
        center.grid(row=0, column=2, sticky="nsew")
        main.columnconfigure(2, weight=1)
        self._build_simulation_panel(center)

        # Vertical separator
        tk.Frame(main, bg=C_BORDER, width=1).grid(row=0, column=3, sticky="ns", padx=0)

        # RIGHT — Data Display
        right = tk.Frame(main, bg=C_BG_PANEL, width=300)
        right.grid(row=0, column=4, sticky="nsew")
        right.grid_propagate(False)
        self._build_data_panel(right)

    # ----------------------------------------------------------
    # CONTROL PANEL (Left)
    # ----------------------------------------------------------
    def _build_control_panel(self, parent):
        # Title
        thdr = tk.Frame(parent, bg=C_BG_PANEL)
        thdr.pack(fill="x")
        tk.Label(thdr, text="CONTROL PANEL", font=self.font_section,
                 bg=C_BG_PANEL, fg=C_TEXT_SEC).pack(padx=14, pady=(10, 4), anchor="w")
        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x")

        # Scrollable area
        canvas_scroll = tk.Canvas(parent, bg=C_BG_PANEL, highlightthickness=0)
        scrollbar = tk.Scrollbar(parent, orient="vertical", command=canvas_scroll.yview,
                                 bg=C_BG_PANEL, troughcolor=C_BG_DARK)
        scroll_frame = tk.Frame(canvas_scroll, bg=C_BG_PANEL)
        scroll_frame.bind("<Configure>",
                          lambda e: canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all")))
        canvas_scroll.create_window((0, 0), window=scroll_frame, anchor="nw", width=268)
        canvas_scroll.configure(yscrollcommand=scrollbar.set)
        canvas_scroll.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        pad = dict(padx=14, pady=(0, 0))
        sep_pad = dict(padx=14, pady=8)

        # --- Motor Control ---
        self._section_label(scroll_frame, "MOTOR CONTROL")

        btn_frame = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        btn_frame.pack(fill="x", **pad)

        self.btn_start = tk.Button(btn_frame, text="▶  Start", font=self.font_btn,
                                   bg="#0d2818", fg=C_GREEN, activebackground="#1a4030",
                                   activeforeground=C_GREEN, relief="flat", bd=0,
                                   cursor="hand2", command=self.start_motor)
        self.btn_start.pack(side="left", fill="x", expand=True, ipady=5, padx=(0, 4))

        self.btn_stop = tk.Button(btn_frame, text="■  Stop", font=self.font_btn,
                                  bg="#280d14", fg=C_RED, activebackground="#401a24",
                                  activeforeground=C_RED, relief="flat", bd=0,
                                  cursor="hand2", command=self.stop_motor)
        self.btn_stop.pack(side="left", fill="x", expand=True, ipady=5, padx=(4, 0))

        # Direction
        tk.Label(scroll_frame, text="Direction", font=self.font_label,
                 bg=C_BG_PANEL, fg=C_TEXT_MUTED).pack(anchor="w", padx=14, pady=(8, 2))

        dir_frame = tk.Frame(scroll_frame, bg=C_BG_INPUT, highlightbackground=C_BORDER,
                             highlightthickness=1)
        dir_frame.pack(fill="x", **pad)

        self.btn_fwd = tk.Button(dir_frame, text="▲  LIFT  (CCW)", font=self.font_btn,
                                 bg=C_CYAN, fg=C_BG_DARK, activebackground="#33ddff",
                                 relief="flat", bd=0, cursor="hand2",
                                 command=lambda: self.set_direction(1))
        self.btn_fwd.pack(side="left", fill="x", expand=True, ipady=4)

        self.btn_rev = tk.Button(dir_frame, text="▼  LOWER  (CW)", font=self.font_btn,
                                 bg=C_BG_INPUT, fg=C_TEXT_MUTED, activebackground=C_BG_CARD,
                                 relief="flat", bd=0, cursor="hand2",
                                 command=lambda: self.set_direction(-1))
        self.btn_rev.pack(side="left", fill="x", expand=True, ipady=4)

        self._separator(scroll_frame)

        # --- Speed Control ---
        self._section_label(scroll_frame, "SPEED CONTROL")

        spd_hdr = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        spd_hdr.pack(fill="x", **pad)
        tk.Label(spd_hdr, text="Speed", font=self.font_label,
                 bg=C_BG_PANEL, fg=C_TEXT_SEC).pack(side="left")
        self.speed_disp = tk.Label(spd_hdr, text="60 RPM", font=self.font_value,
                                   bg=C_BG_PANEL, fg=C_CYAN)
        self.speed_disp.pack(side="right")

        self.speed_slider = tk.Scale(scroll_frame, from_=0, to=MAX_RPM, orient="horizontal",
                                     variable=self.rpm_var, showvalue=False,
                                     bg=C_BG_PANEL, fg=C_CYAN, troughcolor=C_BG_INPUT,
                                     highlightthickness=0, bd=0, sliderrelief="flat",
                                     activebackground=C_CYAN, length=230,
                                     command=self._on_speed_change)
        self.speed_slider.pack(padx=14)

        marks = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        marks.pack(fill="x", padx=14)
        for v in ["0", "100", "200", "300"]:
            tk.Label(marks, text=v, font=self.font_small, bg=C_BG_PANEL,
                     fg=C_TEXT_MUTED).pack(side="left", expand=True)

        self._separator(scroll_frame)

        # --- Load Control ---
        self._section_label(scroll_frame, "LOAD CONTROL")

        load_hdr = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        load_hdr.pack(fill="x", **pad)
        tk.Label(load_hdr, text="Load Mass", font=self.font_label,
                 bg=C_BG_PANEL, fg=C_TEXT_SEC).pack(side="left")
        self.load_disp = tk.Label(load_hdr, text="0.00 kg", font=self.font_value,
                                  bg=C_BG_PANEL, fg=C_ORANGE)
        self.load_disp.pack(side="right")

        self.load_slider = tk.Scale(scroll_frame, from_=0, to=MAX_LOAD_KG,
                                    orient="horizontal", resolution=0.1,
                                    variable=self.load_var, showvalue=False,
                                    bg=C_BG_PANEL, fg=C_ORANGE, troughcolor=C_BG_INPUT,
                                    highlightthickness=0, bd=0, sliderrelief="flat",
                                    activebackground=C_ORANGE, length=230,
                                    command=self._on_load_change)
        self.load_slider.pack(padx=14)

        lmarks = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        lmarks.pack(fill="x", padx=14)
        for v in ["0", "5", "10", "15"]:
            tk.Label(lmarks, text=v + " kg", font=self.font_small, bg=C_BG_PANEL,
                     fg=C_TEXT_MUTED).pack(side="left", expand=True)

        # Torque bar
        torque_frame = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        torque_frame.pack(fill="x", padx=14, pady=(6, 0))

        self.torque_canvas = tk.Canvas(torque_frame, height=14, bg=C_BG_INPUT,
                                       highlightthickness=1, highlightbackground=C_BORDER)
        self.torque_canvas.pack(fill="x")

        tlabels = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        tlabels.pack(fill="x", padx=14, pady=(2, 0))
        self.torque_req_label = tk.Label(tlabels, text="T_req: 0.000 Nm", font=self.font_small,
                                         bg=C_BG_PANEL, fg=C_TEXT_MUTED)
        self.torque_req_label.pack(side="left")
        tk.Label(tlabels, text=f"T_avail: {T_AVAILABLE:.3f} Nm", font=self.font_small,
                 bg=C_BG_PANEL, fg=C_TEXT_MUTED).pack(side="right")

        self._separator(scroll_frame)

        # --- Step Input ---
        self._section_label(scroll_frame, "STEP INPUT")

        tk.Label(scroll_frame, text="Commanded Steps", font=self.font_label,
                 bg=C_BG_PANEL, fg=C_TEXT_MUTED).pack(anchor="w", padx=14, pady=(0, 2))

        inp_frame = tk.Frame(scroll_frame, bg=C_BG_INPUT, highlightbackground=C_BORDER,
                             highlightthickness=1)
        inp_frame.pack(fill="x", padx=14)

        self.step_entry = tk.Entry(inp_frame, textvariable=self.steps_str_var,
                                   font=self.font_value, bg=C_BG_INPUT, fg=C_TEXT,
                                   insertbackground=C_CYAN, relief="flat", bd=4, width=12)
        self.step_entry.pack(side="left", fill="x", expand=True)
        tk.Label(inp_frame, text="steps", font=self.font_small, bg=C_BG_INPUT,
                 fg=C_TEXT_MUTED).pack(side="right", padx=6)

        self.step_entry.bind("<Return>", lambda e: self._on_steps_change())
        self.step_entry.bind("<FocusOut>", lambda e: self._on_steps_change())
        self.step_entry.bind("<KeyRelease>", lambda e: self._on_steps_change())

        # Param chips
        chips = tk.Frame(scroll_frame, bg=C_BG_PANEL)
        chips.pack(fill="x", padx=14, pady=(6, 0))
        for lbl, val in [("PPR", "200"), ("Radius", "20mm"), ("Step∠", "1.8°")]:
            cf = tk.Frame(chips, bg=C_BG_CARD, highlightbackground=C_BORDER,
                          highlightthickness=1, padx=4, pady=1)
            cf.pack(side="left", padx=(0, 4))
            tk.Label(cf, text=lbl, font=self.font_small, bg=C_BG_CARD,
                     fg=C_TEXT_MUTED).pack(side="left")
            tk.Label(cf, text=val, font=self.font_small, bg=C_BG_CARD,
                     fg=C_TEXT_SEC).pack(side="left", padx=(3, 0))

        self._separator(scroll_frame)

        # --- Reset Button ---
        self.btn_reset = tk.Button(scroll_frame, text="↺  Reset Simulation", font=self.font_btn,
                                   bg=C_BG_CARD, fg=C_TEXT_SEC, activebackground=C_BG_INPUT,
                                   activeforeground=C_TEXT, relief="flat", bd=0,
                                   cursor="hand2", command=self.reset_simulation)
        self.btn_reset.pack(fill="x", padx=14, ipady=6, pady=(0, 14))

    def _section_label(self, parent, text):
        tk.Label(parent, text=text, font=self.font_section,
                 bg=C_BG_PANEL, fg=C_TEXT_MUTED).pack(anchor="w", padx=14, pady=(10, 6))

    def _separator(self, parent):
        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x", padx=14, pady=10)

    # ----------------------------------------------------------
    # SIMULATION PANEL (Center)
    # ----------------------------------------------------------
    def _build_simulation_panel(self, parent):
        thdr = tk.Frame(parent, bg=C_BG_PANEL)
        thdr.pack(fill="x")
        tk.Label(thdr, text="MECHANICAL SIMULATION", font=self.font_section,
                 bg=C_BG_PANEL, fg=C_TEXT_SEC).pack(side="left", padx=14, pady=(10, 4))

        badge_f = tk.Frame(thdr, bg=C_BG_PANEL)
        badge_f.pack(side="right", padx=14, pady=(10, 4))
        tk.Label(badge_f, text="● Expected", font=self.font_small,
                 bg=C_BG_PANEL, fg=C_GREEN).pack(side="left", padx=(0, 8))
        tk.Label(badge_f, text="● Actual", font=self.font_small,
                 bg=C_BG_PANEL, fg=C_RED).pack(side="left")

        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x")

        self.sim_canvas = tk.Canvas(parent, bg=C_BG_DARK, highlightthickness=0)
        self.sim_canvas.pack(fill="both", expand=True)

        # Warning label (hidden by default)
        self.warning_var = tk.StringVar(value="")
        self.warning_label = tk.Label(self.sim_canvas, textvariable=self.warning_var,
                                      font=self.font_warn, bg=C_BG_DARK, fg=C_ORANGE,
                                      padx=16, pady=6)
        # Will be placed via canvas window

    # ----------------------------------------------------------
    # DATA PANEL (Right)
    # ----------------------------------------------------------
    def _build_data_panel(self, parent):
        thdr = tk.Frame(parent, bg=C_BG_PANEL)
        thdr.pack(fill="x")
        tk.Label(thdr, text="DATA DISPLAY", font=self.font_section,
                 bg=C_BG_PANEL, fg=C_TEXT_SEC).pack(padx=14, pady=(10, 4), anchor="w")
        tk.Frame(parent, bg=C_BORDER, height=1).pack(fill="x")

        # Scrollable
        canvas_scroll = tk.Canvas(parent, bg=C_BG_PANEL, highlightthickness=0)
        scroll_frame = tk.Frame(canvas_scroll, bg=C_BG_PANEL)
        scroll_frame.bind("<Configure>",
                          lambda e: canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all")))
        canvas_scroll.create_window((0, 0), window=scroll_frame, anchor="nw", width=298)
        canvas_scroll.pack(fill="both", expand=True)

        pad = dict(padx=10, pady=(0, 0))

        # --- Displacement ---
        self._section_label(scroll_frame, "DISPLACEMENT")

        disp_card = tk.Frame(scroll_frame, bg=C_BG_CARD, highlightbackground=C_BORDER,
                             highlightthickness=1)
        disp_card.pack(fill="x", **pad)

        self.disp_expected = self._data_row(disp_card, "Expected", "0.000 mm", fg=C_GREEN)
        self.disp_actual = self._data_row(disp_card, "Actual", "0.000 mm", fg=C_RED)
        self.disp_error = self._data_row(disp_card, "Error", "0.000 mm", fg=C_ORANGE)

        # --- Step Analysis ---
        self._section_label(scroll_frame, "STEP ANALYSIS")

        step_card = tk.Frame(scroll_frame, bg=C_BG_CARD, highlightbackground=C_BORDER,
                             highlightthickness=1)
        step_card.pack(fill="x", **pad)

        self.step_cmd_lbl = self._data_row(step_card, "Commanded", "0")
        self.step_ach_lbl = self._data_row(step_card, "Achieved", "0")
        self.step_miss_lbl = self._data_row(step_card, "Missed", "0", fg=C_RED)

        # --- Motor Status ---
        self._section_label(scroll_frame, "MOTOR STATUS")

        mot_card = tk.Frame(scroll_frame, bg=C_BG_CARD, highlightbackground=C_BORDER,
                            highlightthickness=1)
        mot_card.pack(fill="x", **pad)

        self.mot_rpm_lbl = self._data_row(mot_card, "Current RPM", "0")
        self.mot_dir_lbl = self._data_row(mot_card, "Direction", "Lift (CCW)")
        self.mot_prog_lbl = self._data_row(mot_card, "Progress", "0 %")

        # --- Graph ---
        self._section_label(scroll_frame, "LOAD vs STEP ERROR")

        self.graph_canvas = tk.Canvas(scroll_frame, bg=C_BG_CARD, height=150,
                                      highlightthickness=1, highlightbackground=C_BORDER)
        self.graph_canvas.pack(fill="x", **pad)

        # --- Log ---
        self._section_label(scroll_frame, "DATA LOG")

        self.log_text = tk.Text(scroll_frame, height=7, bg=C_BG_INPUT, fg=C_TEXT_MUTED,
                                font=self.font_small, relief="flat", bd=4, wrap="word",
                                insertbackground=C_CYAN, state="disabled",
                                highlightthickness=1, highlightbackground=C_BORDER)
        self.log_text.pack(fill="x", padx=10, pady=(0, 10))
        self.log_text.tag_configure("info", foreground=C_TEXT_MUTED)
        self.log_text.tag_configure("success", foreground=C_GREEN)
        self.log_text.tag_configure("warning", foreground=C_ORANGE)
        self.log_text.tag_configure("error", foreground=C_RED)

        self._add_log("System initialized. Ready.", "info")
        self._add_log(f"PPR={PPR}  r={PULLEY_RADIUS_MM}mm  T_avail={T_AVAILABLE}Nm", "info")

    def _data_row(self, parent, label, value, fg=C_TEXT):
        row = tk.Frame(parent, bg=C_BG_CARD)
        row.pack(fill="x", padx=8, pady=3)
        tk.Label(row, text=label, font=self.font_label, bg=C_BG_CARD,
                 fg=C_TEXT_SEC).pack(side="left")
        val_lbl = tk.Label(row, text=value, font=self.font_value, bg=C_BG_CARD, fg=fg)
        val_lbl.pack(side="right")
        return val_lbl

    # ----------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------
    def _on_speed_change(self, val=None):
        try:
            rpm = self.rpm_var.get()
        except tk.TclError:
            rpm = 0
        self.speed_disp.config(text=f"{rpm} RPM")
        self._compute_static()
        self._update_data_panel()
        self._draw_simulation()

    def _on_load_change(self, val=None):
        try:
            load = self.load_var.get()
        except tk.TclError:
            load = 0.0
        self.load_disp.config(text=f"{load:.2f} kg")
        self._compute_static()
        self._update_data_panel()
        self._update_torque_bar()
        self._update_graph_data()
        self._draw_simulation()

    def _on_steps_change(self):
        # Debounce: cancel previous pending update
        if self._steps_debounce_id is not None:
            self.after_cancel(self._steps_debounce_id)
        self._steps_debounce_id = self.after(150, self._do_steps_update)

    def _do_steps_update(self):
        """Actual update after debounce delay."""
        self._steps_debounce_id = None
        try:
            raw = self.step_entry.get().strip()
            if raw == "":
                return  # user is still typing, don't update yet
            v = int(raw)
            if v < 1:
                v = 1
            self._steps_value = v
        except (ValueError, tk.TclError):
            pass  # keep last valid _steps_value
        self._compute_static()
        self._update_data_panel()
        self._update_graph_data()
        self._draw_simulation()

    def set_direction(self, d):
        self.direction = d
        if d == 1:
            self.btn_fwd.config(bg=C_CYAN, fg=C_BG_DARK)
            self.btn_rev.config(bg=C_BG_INPUT, fg=C_TEXT_MUTED)
            self.mot_dir_lbl.config(text="Lift (CCW)")
        else:
            self.btn_rev.config(bg=C_CYAN, fg=C_BG_DARK)
            self.btn_fwd.config(bg=C_BG_INPUT, fg=C_TEXT_MUTED)
            self.mot_dir_lbl.config(text="Lower (CW)")
        self._add_log(f"Direction → {'Lift (CCW)' if d == 1 else 'Lower (CW)'}", "info")

    def start_motor(self):
        if self.running:
            return
        rpm = self.rpm_var.get()
        if rpm == 0:
            self._add_log("Cannot start: RPM is 0", "warning")
            return

        self._compute_static()

        # Reset animation state
        self.anim_step = 0.0
        self.anim_actual = 0.0
        self.pulley_angle = 0.0
        self.progress = 0.0

        self.running = True
        self.last_time = time.time()

        self.btn_start.config(bg=C_GREEN, fg=C_BG_DARK)
        self._update_status("RUNNING" if not self.has_step_loss else
                            ("STALL" if self.has_stall else "STEP LOSS"))

        self._add_log(
            f"Motor started — {self.commanded} steps @ {rpm} RPM, "
            f"Load: {self.load_var.get():.2f} kg", "success"
        )
        if self.has_stall:
            self._add_log("⚠ Motor STALL — Torque insufficient", "error")
        elif self.has_step_loss:
            self._add_log(f"⚠ Step loss expected — {self.missed} steps will be missed", "warning")

        self._animate()

    def stop_motor(self):
        if not self.running:
            return
        self.running = False
        self.btn_start.config(bg="#0d2818", fg=C_GREEN)
        self._update_status("IDLE")
        self._add_log("Motor stopped", "info")
        self.warning_var.set("")

    def reset_simulation(self):
        self.running = False
        self.anim_step = 0.0
        self.anim_actual = 0.0
        self.pulley_angle = 0.0
        self.progress = 0.0
        self.btn_start.config(bg="#0d2818", fg=C_GREEN)
        self._update_status("IDLE")
        self.warning_var.set("")
        self._compute_static()
        self._update_data_panel()
        self._draw_simulation()
        self._add_log("Simulation reset", "info")

    # ----------------------------------------------------------
    # PHYSICS (static snapshot)
    # ----------------------------------------------------------
    def _compute_static(self):
        self.commanded = max(1, self._steps_value)
        try:
            mass = self.load_var.get()
        except tk.TclError:
            mass = 0.0

        self.expected_disp = PhysicsEngine.expected_displacement_mm(self.commanded)
        self.actual = PhysicsEngine.actual_steps(self.commanded, mass)
        self.actual_disp = PhysicsEngine.expected_displacement_mm(self.actual)
        self.missed = self.commanded - self.actual

        t_req = PhysicsEngine.torque_required(mass)
        self.has_step_loss = t_req > T_AVAILABLE
        self.has_stall = t_req > T_AVAILABLE and (T_AVAILABLE / t_req) < 0.1 if t_req > 0 else False

    # ----------------------------------------------------------
    # DATA PANEL UPDATE
    # ----------------------------------------------------------
    def _update_data_panel(self):
        error = abs(self.expected_disp - self.actual_disp)

        self.disp_expected.config(text=f"{self.expected_disp:.3f} mm")
        self.disp_actual.config(text=f"{self.actual_disp:.3f} mm")
        self.disp_error.config(text=f"{error:.3f} mm")

        self.step_cmd_lbl.config(text=str(self.commanded))
        self.step_ach_lbl.config(text=str(self.actual))
        self.step_miss_lbl.config(text=str(self.missed),
                                   fg=C_RED if self.missed > 0 else C_TEXT)

        self.mot_rpm_lbl.config(text=str(self.rpm_var.get()) if self.running else "0")
        self.mot_prog_lbl.config(text=f"{int(self.progress * 100)} %")

    def _update_torque_bar(self):
        self.torque_canvas.delete("all")
        w = self.torque_canvas.winfo_width() or 230
        h = 14
        t_req = PhysicsEngine.torque_required(self.load_var.get())
        pct = min(t_req / T_AVAILABLE, 1.0)

        color = C_GREEN if pct < 0.75 else (C_ORANGE if pct < 1.0 else C_RED)
        self.torque_canvas.create_rectangle(0, 0, int(w * pct), h, fill=color, outline="")

        # Threshold line
        self.torque_canvas.create_line(w - 1, 0, w - 1, h, fill=C_RED, width=2)

        self.torque_req_label.config(text=f"T_req: {t_req:.3f} Nm")

    def _update_status(self, text):
        color_map = {
            "IDLE": (C_TEXT_MUTED, C_BG_CARD, C_BORDER),
            "RUNNING": (C_GREEN, "#0d2818", "#1a4030"),
            "STEP LOSS": (C_ORANGE, "#281e0d", "#403420"),
            "STALL": (C_RED, "#280d14", "#401a24"),
        }
        fg, bg, brd = color_map.get(text, color_map["IDLE"])
        self.status_dot.config(fg=fg, bg=bg)
        self.status_label.config(text=text, fg=fg, bg=bg)
        self.status_frame.config(bg=bg, highlightbackground=brd)

    # ----------------------------------------------------------
    # GRAPH  — Load vs Step Error
    # ----------------------------------------------------------
    def _update_graph_data(self):
        self.graph_data = []
        cmd = max(1, self._steps_value)
        for i in range(0, MAX_LOAD_KG * 10 + 1):
            m = i / 10.0
            actual = PhysicsEngine.actual_steps(cmd, m)
            missed = cmd - actual
            self.graph_data.append((m, missed))
        self._draw_graph()

    def _draw_graph(self):
        c = self.graph_canvas
        c.delete("all")
        w = c.winfo_width() or 280
        h = c.winfo_height() or 150
        if w < 40 or h < 40:
            return

        pad_l, pad_r, pad_t, pad_b = 40, 12, 16, 26
        pw = w - pad_l - pad_r
        ph = h - pad_t - pad_b

        max_err = max(10, max(d[1] for d in self.graph_data) if self.graph_data else 10)

        # Grid lines
        for i in range(5):
            y = pad_t + int(ph * i / 4)
            c.create_line(pad_l, y, pad_l + pw, y, fill=C_BORDER, width=1)

        # Threshold line
        threshold_kg = T_AVAILABLE / (GRAVITY * PULLEY_RADIUS_M)
        if threshold_kg <= MAX_LOAD_KG:
            tx = pad_l + int((threshold_kg / MAX_LOAD_KG) * pw)
            c.create_line(tx, pad_t, tx, pad_t + ph, fill=C_RED, width=1, dash=(4, 4))
            c.create_text(tx, pad_t - 6, text="Overload", font=self.font_small,
                          fill=C_RED, anchor="s")

        # Plot
        if len(self.graph_data) > 1:
            points = []
            for load, err in self.graph_data:
                x = pad_l + int((load / MAX_LOAD_KG) * pw)
                y = pad_t + ph - int((err / max_err) * ph)
                points.extend([x, y])
            if len(points) >= 4:
                c.create_line(points, fill=C_CYAN, width=2, smooth=True)

        # Current load marker
        cur_load = self.load_var.get()
        if cur_load > 0:
            cx = pad_l + int((cur_load / MAX_LOAD_KG) * pw)
            cur_missed = self.missed
            cy = pad_t + ph - int((cur_missed / max_err) * ph)
            dot_color = C_RED if self.has_step_loss else C_GREEN
            c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=dot_color,
                          outline=C_BG_DARK, width=2)

        # Axes labels
        c.create_text(pad_l + pw // 2, h - 4, text="Load (kg)",
                      font=self.font_small, fill=C_TEXT_MUTED)
        for i in range(4):
            val = int(MAX_LOAD_KG * i / 3)
            x = pad_l + int(pw * i / 3)
            c.create_text(x, pad_t + ph + 12, text=str(val), font=self.font_small,
                          fill=C_TEXT_MUTED)

        for i in range(5):
            val = int(max_err * (4 - i) / 4)
            y = pad_t + int(ph * i / 4)
            c.create_text(pad_l - 6, y, text=str(val), font=self.font_small,
                          fill=C_TEXT_MUTED, anchor="e")

    # ----------------------------------------------------------
    # LOG
    # ----------------------------------------------------------
    def _add_log(self, msg, tag="info"):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"
        self.log_text.config(state="normal")
        self.log_text.insert("end", line, tag)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ----------------------------------------------------------
    # ANIMATION LOOP
    # ----------------------------------------------------------
    def _animate(self):
        if not self.running:
            self._draw_simulation()
            return

        now = time.time()
        dt = min(now - self.last_time, 0.05)
        self.last_time = now

        step_rate = PhysicsEngine.step_rate(self.rpm_var.get())
        increment = step_rate * dt

        if self.anim_step < self.commanded:
            self.anim_step = min(self.anim_step + increment, self.commanded)

        # Actual proportional
        if self.commanded > 0:
            ratio = self.anim_step / self.commanded
            self.anim_actual = ratio * self.actual

        # Pulley visual angle based on actual steps
        # Lifting (direction=1) → CCW (negative angle visually)
        # Lowering (direction=-1) → CW (positive angle visually)
        self.pulley_angle = -(self.anim_actual / PPR) * 2 * math.pi * self.direction

        self.progress = self.anim_step / self.commanded if self.commanded > 0 else 0

        # Warning
        if self.has_stall:
            self.warning_var.set("⚠  MOTOR STALL — Torque Overloaded")
        elif self.has_step_loss:
            self.warning_var.set("⚠  Step Loss Detected")
        else:
            self.warning_var.set("")

        self._update_data_panel()
        self._draw_simulation()

        # Check complete
        if self.anim_step >= self.commanded:
            self.running = False
            self.progress = 1.0
            self.btn_start.config(bg="#0d2818", fg=C_GREEN)
            self._update_status("IDLE")
            self._update_data_panel()

            result_tag = "warning" if self.has_step_loss else "success"
            self._add_log(
                f"Done: {self.actual}/{self.commanded} steps"
                + (f" ({self.missed} missed)" if self.missed > 0 else ""),
                result_tag
            )
            self._draw_simulation()
            return

        self.after(16, self._animate)   # ~60 fps

    # ----------------------------------------------------------
    # CANVAS DRAWING
    # ----------------------------------------------------------
    def _draw_simulation(self):
        c = self.sim_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 100 or h < 100:
            return

        # --- Grid ---
        for gx in range(0, w, 40):
            c.create_line(gx, 0, gx, h, fill="#111e33", width=1)
        for gy in range(0, h, 40):
            c.create_line(0, gy, w, gy, fill="#111e33", width=1)

        # Layout references
        motor_cx = int(w * 0.30)
        motor_cy = int(h * 0.48)
        motor_w, motor_h = 120, 85
        pulley_r = 50
        shaft_len = 40

        # ===== RULER / LINEAR SCALE (top) =====
        scale_y = motor_cy - 110
        scale_x1 = int(w * 0.06)
        scale_x2 = int(w * 0.94)
        scale_mid = (scale_x1 + scale_x2) // 2
        scale_w = scale_x2 - scale_x1
        total_mm = 500  # display range ±250 mm

        # scale bg
        c.create_rectangle(scale_x1 - 4, scale_y - 18, scale_x2 + 4, scale_y + 22,
                           fill=C_BG_CARD, outline=C_BORDER)

        # ruler line
        c.create_line(scale_x1, scale_y, scale_x2, scale_y, fill=C_SHAFT, width=2)

        # ticks
        for mm in range(-250, 251, 5):
            x = scale_mid + int(mm * (scale_w / total_mm))
            if x < scale_x1 or x > scale_x2:
                continue
            tick_h = 10 if mm % 50 == 0 else (6 if mm % 10 == 0 else 3)
            color = C_TEXT_MUTED if mm % 50 == 0 else "#2a3f63"
            c.create_line(x, scale_y - tick_h, x, scale_y + tick_h, fill=color, width=1)
            if mm % 50 == 0:
                c.create_text(x, scale_y + 15, text=str(mm), font=self.font_small,
                              fill=C_TEXT_MUTED)

        # Zero mark
        c.create_line(scale_mid, scale_y - 14, scale_mid, scale_y + 14,
                      fill=C_CYAN, width=2)

        # ===== MOTOR BLOCK =====
        mx = motor_cx - motor_w // 2
        my = motor_cy - motor_h // 2

        # shadow
        c.create_rectangle(mx + 3, my + 3, mx + motor_w + 3, my + motor_h + 3,
                           fill="#050810", outline="")
        # body
        c.create_rectangle(mx, my, mx + motor_w, my + motor_h,
                           fill=C_MOTOR_BODY, outline=C_MOTOR_BORDER, width=2)
        # internal winding lines
        for i in range(1, 4):
            ly = my + int(motor_h * i / 4)
            c.create_line(mx + 10, ly, mx + motor_w - 10, ly,
                          fill="#162a48", width=1)
        # bolts
        for bx, by in [(mx + 8, my + 8), (mx + motor_w - 8, my + 8),
                        (mx + 8, my + motor_h - 8), (mx + motor_w - 8, my + motor_h - 8)]:
            c.create_oval(bx - 3, by - 3, bx + 3, by + 3, fill=C_SHAFT, outline=C_TEXT_MUTED)

        # LED
        led_color = C_GREEN if (self.running and not self.has_stall) else \
                    (C_RED if (self.running and self.has_stall) else C_TEXT_MUTED)
        c.create_oval(mx + motor_w - 18, motor_cy - 4, mx + motor_w - 10, motor_cy + 4,
                      fill=led_color, outline="")

        # Labels
        c.create_text(motor_cx, my + motor_h + 12, text="STEPPER MOTOR",
                      font=self.font_section, fill=C_TEXT_SEC)
        c.create_text(motor_cx, my + motor_h + 26, text=f"NEMA-23  /  {PPR} PPR",
                      font=self.font_small, fill=C_TEXT_MUTED)

        # ===== SHAFT =====
        shaft_x1 = motor_cx + motor_w // 2
        shaft_x2 = shaft_x1 + shaft_len
        c.create_line(shaft_x1, motor_cy, shaft_x2, motor_cy, fill=C_SHAFT, width=6)
        c.create_line(shaft_x1, motor_cy, shaft_x2, motor_cy,
                      fill="#2d3748", width=1, dash=(4, 3))

        # ===== PULLEY =====
        pulley_cx = shaft_x2 + pulley_r
        pulley_cy = motor_cy
        self._draw_pulley(c, pulley_cx, pulley_cy, pulley_r)

        # Pulley label
        c.create_text(pulley_cx, pulley_cy + pulley_r + 12, text="PULLEY",
                      font=self.font_section, fill=C_TEXT_SEC)
        c.create_text(pulley_cx, pulley_cy + pulley_r + 26,
                      text=f"r = {PULLEY_RADIUS_MM}mm", font=self.font_small, fill=C_TEXT_MUTED)

        # ===== STRING & WEIGHT =====
        self._draw_string_weight(c, pulley_cx, pulley_cy, pulley_r, h)

        # ===== POINTERS ON SCALE =====
        self._draw_pointers(c, scale_mid, scale_y, scale_w, total_mm, motor_cy)

        # ===== FORMULA BOX =====
        self._draw_formula_box(c, w, h)

        # ===== WARNING OVERLAY =====
        warn_text = self.warning_var.get()
        if warn_text:
            tw = c.winfo_width() // 2
            bg_c = "#33200a" if "Step Loss" in warn_text else "#330a14"
            fg_c = C_ORANGE if "Step Loss" in warn_text else C_RED
            c.create_rectangle(tw - 180, 10, tw + 180, 40, fill=bg_c,
                               outline=fg_c, width=1)
            c.create_text(tw, 25, text=warn_text, font=self.font_warn, fill=fg_c)

        # ===== ROTATION DIRECTION INDICATOR =====
        if self.running:
            arc_r = pulley_r + 18
            # CCW arrow for lifting, CW arrow for lowering
            if self.direction == 1:
                # CCW: arc from 30° to 150°
                c.create_arc(pulley_cx - arc_r, pulley_cy - arc_r,
                             pulley_cx + arc_r, pulley_cy + arc_r,
                             start=30, extent=120, style="arc",
                             outline=C_CYAN, width=1.5)
                # arrowhead at 150° = CCW tip
                ax = pulley_cx + int(arc_r * math.cos(math.radians(150)))
                ay = pulley_cy - int(arc_r * math.sin(math.radians(150)))
                c.create_text(ax - 8, ay - 2, text="↺", font=self.font_label, fill=C_CYAN)
            else:
                # CW: arc from 30° to 150° drawn clockwise
                c.create_arc(pulley_cx - arc_r, pulley_cy - arc_r,
                             pulley_cx + arc_r, pulley_cy + arc_r,
                             start=30, extent=-120, style="arc",
                             outline=C_CYAN, width=1.5)
                ax = pulley_cx + int(arc_r * math.cos(math.radians(30)))
                ay = pulley_cy - int(arc_r * math.sin(math.radians(30)))
                c.create_text(ax + 8, ay - 2, text="↻", font=self.font_label, fill=C_CYAN)

    def _draw_pulley(self, c, cx, cy, r):
        angle = self.pulley_angle

        # Outer rim shadow
        c.create_oval(cx - r + 2, cy - r + 2, cx + r + 2, cy + r + 2,
                      fill="#050810", outline="")
        # Outer rim
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill=C_MOTOR_BODY, outline=C_MOTOR_BORDER, width=2)
        # Groove
        c.create_oval(cx - r + 6, cy - r + 6, cx + r - 6, cy + r - 6,
                      fill="", outline="#1a3050", width=3)

        # Spokes (rotate with angle)
        for i in range(4):
            a = angle + (math.pi / 2) * i
            x1 = cx + int(10 * math.cos(a))
            y1 = cy + int(10 * math.sin(a))
            x2 = cx + int((r - 10) * math.cos(a))
            y2 = cy + int((r - 10) * math.sin(a))
            c.create_line(x1, y1, x2, y2, fill=C_MOTOR_BORDER, width=2)

        # Hub
        c.create_oval(cx - 10, cy - 10, cx + 10, cy + 10,
                      fill=C_MOTOR_BORDER, outline="#4a6590")
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                      fill="#5a7aa0", outline="")

        # Direction marker dot (rotates)
        dx = cx + int((r - 14) * math.cos(angle - math.pi / 2))
        dy = cy + int((r - 14) * math.sin(angle - math.pi / 2))
        c.create_oval(dx - 4, dy - 4, dx + 4, dy + 4, fill=C_CYAN, outline="")

    def _draw_string_weight(self, c, pcx, pcy, pr, canvas_h):
        # Weight displacement based on animated actual steps
        actual_revs = self.anim_actual / PPR
        linear_mm = actual_revs * 2 * math.pi * PULLEY_RADIUS_MM
        # Lifting → weight goes UP (negative Y), Lowering → weight goes DOWN
        weight_disp_px = linear_mm * 1.2 * self.direction  # scale for visibility

        string_x = pcx + pr - 5
        weight_base_y = pcy + pr + 65
        weight_y = weight_base_y - weight_disp_px  # minus because up = negative screen Y when lifting
        weight_w, weight_h = 55, 44

        # String arc around pulley
        c.create_arc(pcx - pr + 5, pcy - pr + 5, pcx + pr - 5, pcy + pr - 5,
                     start=270, extent=90, style="arc", outline=C_TEXT_SEC, width=1.5)

        # Vertical string
        c.create_line(string_x, pcy, string_x, weight_y, fill=C_TEXT_SEC, width=1.5)

        # Weight
        mass = self.load_var.get()
        if mass > 0:
            wx = string_x - weight_w // 2
            wy = weight_y

            # shadow
            c.create_rectangle(wx + 2, wy + 2, wx + weight_w + 2, wy + weight_h + 2,
                               fill="#050810", outline="")
            # body
            c.create_rectangle(wx, wy, wx + weight_w, wy + weight_h,
                               fill="#3a2525", outline="#7a5050", width=1)
            # label
            c.create_text(string_x, wy + weight_h // 2,
                          text=f"{mass:.1f} kg", font=self.font_value,
                          fill=C_TEXT)

            # Hook
            c.create_oval(string_x - 3, wy - 3, string_x + 3, wy + 3,
                          fill="#7a5050", outline="")

            # Gravity arrow
            arr_x = wx + weight_w + 14
            arr_y1 = wy + 5
            arr_y2 = wy + weight_h + 18
            c.create_line(arr_x, arr_y1, arr_x, arr_y2, fill=C_ORANGE, width=1.5,
                          arrow="last", arrowshape=(6, 8, 3))
            c.create_text(arr_x + 12, (arr_y1 + arr_y2) // 2, text="mg",
                          font=self.font_small, fill=C_ORANGE)

    def _draw_pointers(self, c, scale_mid, scale_y, scale_w, total_mm, motor_cy):
        # Expected displacement (mm)
        expected_revs = self.anim_step / PPR
        expected_mm = expected_revs * 2 * math.pi * PULLEY_RADIUS_MM * self.direction

        actual_revs = self.anim_actual / PPR
        actual_mm = actual_revs * 2 * math.pi * PULLEY_RADIUS_MM * self.direction

        expected_x = scale_mid + int(expected_mm * (scale_w / total_mm))
        actual_x = scale_mid + int(actual_mm * (scale_w / total_mm))

        pointer_top = scale_y + 24
        pointer_bot = motor_cy - 25

        # Expected pointer — green dashed
        c.create_line(expected_x, pointer_top, expected_x, pointer_bot,
                      fill=C_GREEN, width=2, dash=(4, 4))
        # Triangle marker
        c.create_polygon(expected_x - 6, scale_y + 26, expected_x + 6, scale_y + 26,
                         expected_x, scale_y + 18, fill=C_GREEN, outline="")
        c.create_text(expected_x, scale_y + 36, text="EXP", font=self.font_small, fill=C_GREEN)

        # Actual pointer — red solid
        ptr_color = C_RED if self.has_step_loss else C_GREEN
        c.create_line(actual_x, pointer_top + 4, actual_x, pointer_bot,
                      fill=ptr_color, width=3)
        c.create_polygon(actual_x - 6, scale_y + 30, actual_x + 6, scale_y + 30,
                         actual_x, scale_y + 22, fill=ptr_color, outline="")
        c.create_text(actual_x, scale_y + 47, text="ACT", font=self.font_small, fill=ptr_color)

        # Error bracket
        if self.has_step_loss and abs(expected_x - actual_x) > 3:
            bracket_y = scale_y + 54
            c.create_line(expected_x, bracket_y, actual_x, bracket_y,
                          fill=C_ORANGE, width=1, dash=(2, 2))
            err_mm = abs(expected_mm - actual_mm)
            c.create_text((expected_x + actual_x) // 2, bracket_y + 10,
                          text=f"Δ{err_mm:.1f}mm", font=self.font_small, fill=C_ORANGE)

    def _draw_formula_box(self, c, w, h):
        bx, by = 12, h - 100
        bw, bh = 250, 90

        c.create_rectangle(bx, by, bx + bw, by + bh, fill="#0d1321",
                           outline=C_BORDER, width=1)

        c.create_text(bx + 10, by + 10, text="FORMULAS", font=self.font_section,
                      fill=C_CYAN, anchor="w")

        lines = [
            "D = (steps/PPR) × 2πr",
            "T_req = m × g × r",
            f"T_avail = {T_AVAILABLE:.2f} Nm",
        ]
        y = by + 28
        for line in lines:
            c.create_text(bx + 10, y, text=line, font=self.font_small,
                          fill=C_TEXT_SEC, anchor="w")
            y += 14

        # Status
        t_req = PhysicsEngine.torque_required(self.load_var.get())
        if t_req <= T_AVAILABLE:
            c.create_text(bx + 10, y, text="✓ No step loss", font=self.font_small,
                          fill=C_GREEN, anchor="w")
        elif t_req > 0 and (T_AVAILABLE / t_req) >= 0.1:
            loss_pct = (1 - T_AVAILABLE / t_req) * 100
            c.create_text(bx + 10, y, text=f"⚠ Step loss: {loss_pct:.1f}%",
                          font=self.font_small, fill=C_ORANGE, anchor="w")
        else:
            c.create_text(bx + 10, y, text="✕ Motor stall", font=self.font_small,
                          fill=C_RED, anchor="w")


# ============================================================
# ENTRY POINT
# ============================================================
if __name__ == "__main__":
    app = StepperSimApp()
    app.mainloop()
