"""
gui.py - Modern Desktop GUI for SmartSweep AI
Built with CustomTkinter and Tkinter Canvas.
Features an interactive room grid, custom-drawn animated vacuum robot,
live AI statistics cards, comprehensive controls, and modern dark/light themes.
"""

import math
import tkinter as tk
from tkinter import filedialog, messagebox
from typing import Optional, Set, Tuple

import customtkinter as ctk

from astar import Coord, a_star_search, a_star_step_generator
from robot_simulation import CleaningMode, RobotSimulation, SimState
from room import Room, RoomError

# Color palettes for Dark and Light themes
THEME_COLORS = {
    "dark": {
        "canvas_bg": "#0b1329",
        "grid_line": "#1e293b",
        "cell_clean": "#0f172a",
        "cell_obstacle": "#334155",
        "cell_obstacle_border": "#475569",
        "cell_start": "#10b981",
        "cell_goal": "#ef4444",
        "cell_dirt": "#f59e0b",
        "cell_dirt_speck": "#d97706",
        "cell_cleaned": "#059669",
        "cell_cleaned_halo": "#065f46",
        "cell_explored": "#3b0764",
        "cell_explored_border": "#6b21a8",
        "cell_frontier": "#134e4a",
        "cell_frontier_border": "#0d9488",
        "path_line": "#38bdf8",
        "path_traveled": "#0284c7",
        "path_dot": "#7dd3fc",
        "robot_body": "#0284c7",
        "robot_ring": "#38bdf8",
        "robot_turret": "#0f172a",
        "robot_heading": "#34d399",
        "robot_glow": "#38bdf8",
        "card_bg": "#1e293b",
        "card_border": "#334155",
        "text_primary": "#f8fafc",
        "text_secondary": "#94a3b8",
        "text_accent": "#38bdf8",
    },
    "light": {
        "canvas_bg": "#f8fafc",
        "grid_line": "#e2e8f0",
        "cell_clean": "#ffffff",
        "cell_obstacle": "#475569",
        "cell_obstacle_border": "#334155",
        "cell_start": "#10b981",
        "cell_goal": "#ef4444",
        "cell_dirt": "#f59e0b",
        "cell_dirt_speck": "#b45309",
        "cell_cleaned": "#34d399",
        "cell_cleaned_halo": "#a7f3d0",
        "cell_explored": "#f3e8ff",
        "cell_explored_border": "#d8b4fe",
        "cell_frontier": "#ccfbf1",
        "cell_frontier_border": "#5eead4",
        "path_line": "#0284c7",
        "path_traveled": "#94a3b8",
        "path_dot": "#0369a1",
        "robot_body": "#0369a1",
        "robot_ring": "#0284c7",
        "robot_turret": "#ffffff",
        "robot_heading": "#10b981",
        "robot_glow": "#38bdf8",
        "card_bg": "#ffffff",
        "card_border": "#cbd5e1",
        "text_primary": "#0f172a",
        "text_secondary": "#64748b",
        "text_accent": "#0284c7",
    }
}


class SmartSweepGUI:
    """Main Application Controller and User Interface."""

    def __init__(self, root: ctk.CTk):
        self.root = root
        self.root.title("SmartSweep AI – Intelligent Robot Vacuum Cleaner Path Planner")
        self.root.geometry("1400x900")
        self.root.minsize(1100, 750)

        # Apply default theme
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.current_theme = "dark"

        # Core logic models
        self.room = Room(rows=15, cols=15)
        self.sim = RobotSimulation(self.room)

        # UI state variables
        self.interaction_mode = tk.StringVar(value="obstacle")  # "start", "goal", "obstacle", "dirt", "erase"
        self.cleaning_mode_var = tk.StringVar(value=CleaningMode.SMART_CLEANING.value)
        self.step_by_step_var = tk.BooleanVar(value=False)
        self.show_explored_var = tk.BooleanVar(value=True)
        self.show_path_var = tk.BooleanVar(value=True)
        self.diagonal_var = tk.BooleanVar(value=False)
        self.speed_ms = tk.IntVar(value=100)  # ms between movement steps
        self.hover_coord: Optional[Coord] = None

        # Animation callback IDs
        self.anim_timer_id: Optional[str] = None
        self.sweep_angle = 0.0  # Rotating vacuum brush animation effect
        self.is_mouse_down = False
        self.last_drag_cell: Optional[Coord] = None

        # Generator for step-by-step search
        self.search_generator = None
        self.search_active = False

        # Load demo room by default
        self.room.load_demo_room()
        self.sim.reset()

        # Build UI Components
        self._build_header()
        self._build_main_layout()

        # Canvas bind events
        self.canvas.bind("<Configure>", lambda e: self.render_grid())
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<Motion>", self._on_canvas_hover)
        self.canvas.bind("<Leave>", self._on_canvas_leave)

        # Initial render and stats refresh
        self.root.after(100, self._initial_setup)

    def _initial_setup(self):
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    # -------------------------------------------------------------------------
    # HEADER SECTION
    # -------------------------------------------------------------------------
    def _build_header(self):
        header_frame = ctk.CTkFrame(self.root, height=75, corner_radius=0, fg_color=("#f1f5f9", "#0f172a"))
        header_frame.pack(side="top", fill="x", padx=0, pady=0)
        header_frame.pack_propagate(False)

        # Left: App Logo & Title
        title_box = ctk.CTkFrame(header_frame, fg_color="transparent")
        title_box.pack(side="left", padx=20, pady=10)

        # App Title with Tagline
        titles_inner = ctk.CTkFrame(title_box, fg_color="transparent")
        titles_inner.pack(side="left", padx=10)

        title_row = ctk.CTkFrame(titles_inner, fg_color="transparent")
        title_row.pack(anchor="w")

        title_lbl = ctk.CTkLabel(
            title_row,
            text="SmartSweep AI",
            font=ctk.CTkFont(family="Segoe UI", size=22, weight="bold"),
            text_color=("#0284c7", "#38bdf8")
        )
        title_lbl.pack(side="left")

        subtitle_lbl = ctk.CTkLabel(
            title_row,
            text=" • Intelligent Robot Vacuum Path Planner",
            font=ctk.CTkFont(family="Segoe UI", size=14, weight="normal"),
            text_color=("#64748b", "#94a3b8")
        )
        subtitle_lbl.pack(side="left", padx=5)

        tagline_lbl = ctk.CTkLabel(
            titles_inner,
            text="Smarter Navigation. Efficient Cleaning. Powered by AI (A* Algorithm).",
            font=ctk.CTkFont(family="Segoe UI", size=11, slant="italic"),
            text_color=("#475569", "#64748b")
        )
        tagline_lbl.pack(anchor="w")

        # Right: Status Badge & Theme Toggle & Help
        right_box = ctk.CTkFrame(header_frame, fg_color="transparent")
        right_box.pack(side="right", padx=20, pady=12)

        # Status badge pill
        self.status_badge = ctk.CTkLabel(
            right_box,
            text="● READY",
            font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"),
            corner_radius=12,
            fg_color=("#e2e8f0", "#1e293b"),
            text_color="#10b981",
            padx=14,
            pady=5
        )
        self.status_badge.pack(side="left", padx=10)

        # Theme toggle switch
        self.theme_switch = ctk.CTkSwitch(
            right_box,
            text="Dark Mode",
            command=self._toggle_theme,
            font=ctk.CTkFont(family="Segoe UI", size=11),
            onvalue=1,
            offvalue=0
        )
        self.theme_switch.select()
        self.theme_switch.pack(side="left", padx=10)

        # Info / About button
        info_btn = ctk.CTkButton(
            right_box,
            text="ℹ About",
            width=80,
            height=30,
            command=self._show_about_dialog,
            fg_color=("#cbd5e1", "#334155"),
            hover_color=("#94a3b8", "#475569"),
            font=ctk.CTkFont(family="Segoe UI", size=11)
        )
        info_btn.pack(side="left", padx=5)

    # -------------------------------------------------------------------------
    # MAIN LAYOUT: Sidebar Left, Canvas Center, Sidebar Right
    # -------------------------------------------------------------------------
    def _build_main_layout(self):
        container = ctk.CTkFrame(self.root, fg_color="transparent")
        container.pack(fill="both", expand=True, padx=12, pady=12)

        # Left Sidebar (Controls)
        self._build_left_sidebar(container)

        # Center Area (Canvas & Legend)
        self._build_center_area(container)

        # Right Sidebar (AI Insights & Stats)
        self._build_right_sidebar(container)

    # -------------------------------------------------------------------------
    # LEFT SIDEBAR: CONTROLS
    # -------------------------------------------------------------------------
    def _build_left_sidebar(self, parent):
        left_frame = ctk.CTkScrollableFrame(
            parent,
            width=290,
            corner_radius=12,
            fg_color=("#f8fafc", "#1e293b"),
            border_width=1,
            border_color=("#e2e8f0", "#334155")
        )
        left_frame.pack(side="left", fill="y", padx=(0, 10))

        # --- Section 1: Room Presets & File Actions ---
        sec1_lbl = ctk.CTkLabel(
            left_frame,
            text="ROOM ACTIONS",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        )
        sec1_lbl.pack(anchor="w", padx=10, pady=(5, 5))

        btn_demo = ctk.CTkButton(
            left_frame, text="🏠 Load Demo Room", height=32,
            fg_color=("#0284c7", "#0369a1"), hover_color=("#0369a1", "#0284c7"),
            command=self.load_demo_room
        )
        btn_demo.pack(fill="x", padx=10, pady=3)

        btn_random = ctk.CTkButton(
            left_frame, text="🎲 Generate Random Room", height=32,
            fg_color=("#475569", "#334155"), hover_color=("#334155", "#475569"),
            command=self.generate_random_room
        )
        btn_random.pack(fill="x", padx=10, pady=3)

        row_io = ctk.CTkFrame(left_frame, fg_color="transparent")
        row_io.pack(fill="x", padx=10, pady=3)

        btn_save = ctk.CTkButton(
            row_io, text="💾 Save", width=125, height=30,
            fg_color=("#64748b", "#475569"), hover_color=("#475569", "#64748b"),
            command=self.save_room
        )
        btn_save.pack(side="left", padx=(0, 5))

        btn_load = ctk.CTkButton(
            row_io, text="📂 Load", width=125, height=30,
            fg_color=("#64748b", "#475569"), hover_color=("#475569", "#64748b"),
            command=self.load_room
        )
        btn_load.pack(side="left")

        btn_clear = ctk.CTkButton(
            left_frame, text="🗑 Clear Room", height=30,
            fg_color=("#ef4444", "#dc2626"), hover_color=("#dc2626", "#b91c1c"),
            command=self.clear_room
        )
        btn_clear.pack(fill="x", padx=10, pady=3)

        # --- Section 2: Interactive Cell Mode ---
        ctk.CTkLabel(
            left_frame, text="INTERACTION MODE",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        ).pack(anchor="w", padx=10, pady=(15, 5))

        modes = [
            ("⬛ Add / Remove Obstacle", "obstacle"),
            ("🧹 Add Dirt Spot", "dirt"),
            ("🏁 Set Start Position", "start"),
            ("🎯 Set Final Goal", "goal"),
            ("⬜ Erase Cell", "erase"),
        ]
        for label, val in modes:
            rb = ctk.CTkRadioButton(
                left_frame, text=label, variable=self.interaction_mode, value=val,
                font=ctk.CTkFont(family="Segoe UI", size=12)
            )
            rb.pack(anchor="w", padx=15, pady=3)

        # --- Section 3: Room & Grid Configuration ---
        ctk.CTkLabel(
            left_frame, text="GRID SETTINGS",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        ).pack(anchor="w", padx=10, pady=(15, 5))

        # Grid size selector
        grid_row = ctk.CTkFrame(left_frame, fg_color="transparent")
        grid_row.pack(fill="x", padx=10, pady=2)
        ctk.CTkLabel(grid_row, text="Grid Size:").pack(side="left")
        self.grid_size_menu = ctk.CTkOptionMenu(
            grid_row, values=["10 × 10", "15 × 15", "20 × 20", "25 × 25"],
            width=140, command=self._on_grid_size_change
        )
        self.grid_size_menu.set("15 × 15")
        self.grid_size_menu.pack(side="right")

        # Diagonal Movement toggle
        diag_switch = ctk.CTkSwitch(
            left_frame, text="Diagonal Moves (8-Dir)",
            variable=self.diagonal_var, command=self._on_diagonal_toggle
        )
        diag_switch.pack(anchor="w", padx=15, pady=6)

        # --- Section 4: AI & Operating Mode ---
        ctk.CTkLabel(
            left_frame, text="AI OPERATING MODE",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        ).pack(anchor="w", padx=10, pady=(15, 5))

        mode_menu = ctk.CTkSegmentedButton(
            left_frame,
            values=[CleaningMode.SMART_CLEANING.value, CleaningMode.SINGLE_DESTINATION.value],
            variable=self.cleaning_mode_var,
            command=self._on_cleaning_mode_change
        )
        mode_menu.pack(fill="x", padx=10, pady=3)

        # Visual toggles
        chk_explored = ctk.CTkCheckBox(
            left_frame, text="Show Explored Nodes",
            variable=self.show_explored_var, command=self.render_grid
        )
        chk_explored.pack(anchor="w", padx=15, pady=4)

        chk_path = ctk.CTkCheckBox(
            left_frame, text="Show Planned Path",
            variable=self.show_path_var, command=self.render_grid
        )
        chk_path.pack(anchor="w", padx=15, pady=4)

        chk_step_by_step = ctk.CTkCheckBox(
            left_frame, text="Step-by-Step Exploration",
            variable=self.step_by_step_var
        )
        chk_step_by_step.pack(anchor="w", padx=15, pady=4)

        # --- Section 5: Pathfinding & Execution Controls ---
        ctk.CTkLabel(
            left_frame, text="SIMULATION CONTROLS",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        ).pack(anchor="w", padx=10, pady=(15, 5))

        self.btn_find_path = ctk.CTkButton(
            left_frame, text="⚡ Find Optimal Path (A*)", height=38,
            font=ctk.CTkFont(family="Segoe UI", size=13, weight="bold"),
            fg_color=("#0284c7", "#06b6d4"), hover_color=("#0369a1", "#0891b2"),
            command=self.find_optimal_path
        )
        self.btn_find_path.pack(fill="x", padx=10, pady=5)

        self.btn_start = ctk.CTkButton(
            left_frame, text="▶ Start Cleaning", height=36,
            font=ctk.CTkFont(family="Segoe UI", size=13, weight="bold"),
            fg_color=("#10b981", "#059669"), hover_color=("#059669", "#047857"),
            command=self.start_cleaning
        )
        self.btn_start.pack(fill="x", padx=10, pady=4)

        row_play = ctk.CTkFrame(left_frame, fg_color="transparent")
        row_play.pack(fill="x", padx=10, pady=3)

        self.btn_pause = ctk.CTkButton(
            row_play, text="⏸ Pause", width=125, height=32,
            fg_color=("#f59e0b", "#d97706"), hover_color=("#d97706", "#b45309"),
            command=self.pause_resume
        )
        self.btn_pause.pack(side="left", padx=(0, 5))

        self.btn_step = ctk.CTkButton(
            row_play, text="⏭ Step Forward", width=125, height=32,
            fg_color=("#475569", "#334155"), hover_color=("#334155", "#475569"),
            command=self.step_forward
        )
        self.btn_step.pack(side="left")

        self.btn_reset = ctk.CTkButton(
            left_frame, text="🔄 Reset Simulation", height=32,
            fg_color=("#475569", "#334155"), hover_color=("#334155", "#475569"),
            command=self.reset_simulation
        )
        self.btn_reset.pack(fill="x", padx=10, pady=4)

        # Speed slider
        ctk.CTkLabel(
            left_frame, text="Movement Speed:",
            font=ctk.CTkFont(family="Segoe UI", size=11)
        ).pack(anchor="w", padx=15, pady=(10, 0))
        self.speed_slider = ctk.CTkSlider(
            left_frame, from_=20, to=300,
            command=self._on_speed_change
        )
        self.speed_slider.set(100)
        self.speed_slider.pack(fill="x", padx=15, pady=5)

    # -------------------------------------------------------------------------
    # CENTER AREA: CANVAS & LEGEND
    # -------------------------------------------------------------------------
    def _build_center_area(self, parent):
        center_frame = ctk.CTkFrame(
            parent,
            corner_radius=12,
            fg_color=("#ffffff", "#0f172a"),
            border_width=1,
            border_color=("#e2e8f0", "#334155")
        )
        center_frame.pack(side="left", fill="both", expand=True, padx=0)

        # Grid Header with hover coordinates
        grid_top = ctk.CTkFrame(center_frame, height=35, fg_color="transparent")
        grid_top.pack(side="top", fill="x", padx=15, pady=(10, 0))

        grid_title = ctk.CTkLabel(
            grid_top, text="ROOM NAVIGATION GRID",
            font=ctk.CTkFont(family="Segoe UI", size=13, weight="bold"),
            text_color=("#334155", "#cbd5e1")
        )
        grid_title.pack(side="left")

        self.coord_label = ctk.CTkLabel(
            grid_top, text="Hover: (Row -, Col -)",
            font=ctk.CTkFont(family="Segoe UI", size=12),
            text_color=("#64748b", "#94a3b8")
        )
        self.coord_label.pack(side="right")

        # Tkinter Canvas
        self.canvas_container = ctk.CTkFrame(center_frame, fg_color="transparent")
        self.canvas_container.pack(fill="both", expand=True, padx=15, pady=10)

        colors = THEME_COLORS[self.current_theme]
        self.canvas = tk.Canvas(
            self.canvas_container,
            bg=colors["canvas_bg"],
            highlightthickness=0,
            cursor="crosshair"
        )
        self.canvas.pack(fill="both", expand=True)

        # Bottom Legend Bar
        self._build_legend_bar(center_frame)

    def _build_legend_bar(self, parent):
        legend_frame = ctk.CTkFrame(parent, height=45, fg_color=("#f1f5f9", "#1e293b"), corner_radius=8)
        legend_frame.pack(side="bottom", fill="x", padx=15, pady=(0, 10))

        chips = [
            ("Start", "#10b981"),
            ("Goal", "#ef4444"),
            ("Obstacle", "#475569"),
            ("Dirt", "#f59e0b"),
            ("Cleaned", "#059669"),
            ("Path", "#38bdf8"),
            ("Explored", "#a855f7"),
            ("Frontier", "#14b8a6"),
            ("Robot", "#0284c7"),
        ]

        for text, col in chips:
            chip_box = ctk.CTkFrame(legend_frame, fg_color="transparent")
            chip_box.pack(side="left", padx=8, pady=8)

            dot = ctk.CTkFrame(chip_box, width=12, height=12, corner_radius=6, fg_color=col)
            dot.pack(side="left", padx=(0, 4))
            dot.pack_propagate(False)

            lbl = ctk.CTkLabel(chip_box, text=text, font=ctk.CTkFont(family="Segoe UI", size=11))
            lbl.pack(side="left")

    # -------------------------------------------------------------------------
    # RIGHT SIDEBAR: AI INSIGHTS & STATISTICS
    # -------------------------------------------------------------------------
    def _build_right_sidebar(self, parent):
        right_frame = ctk.CTkScrollableFrame(
            parent,
            width=290,
            corner_radius=12,
            fg_color=("#f8fafc", "#1e293b"),
            border_width=1,
            border_color=("#e2e8f0", "#334155")
        )
        right_frame.pack(side="right", fill="y", padx=(10, 0))

        # Title
        ctk.CTkLabel(
            right_frame, text="AI INSIGHTS & METRICS",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=("#64748b", "#94a3b8")
        ).pack(anchor="w", padx=10, pady=(5, 10))

        # Stat cards
        self.stat_widgets = {}

        self._create_stat_card(right_frame, "STATUS", "Ready", "status_card", col="#10b981")
        self._create_stat_card(right_frame, "PLANNED PATH COST", "0.0", "path_cost", col="#38bdf8")
        self._create_stat_card(right_frame, "DISTANCE TRAVELED", "0.0", "dist_traveled", col="#38bdf8")
        self._create_stat_card(right_frame, "STEPS (COMPLETED / TOTAL)", "0 / 0", "steps_card")
        self._create_stat_card(right_frame, "NODES EXPLORED (A*)", "0", "nodes_explored", col="#c084fc")
        self._create_stat_card(right_frame, "PATHFINDING TIME", "0.00 ms", "exec_time", col="#2dd4bf")

        # Dirt progress card
        dirt_card = ctk.CTkFrame(right_frame, corner_radius=8, fg_color=("#ffffff", "#0f172a"), border_width=1, border_color=("#e2e8f0", "#334155"))
        dirt_card.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(dirt_card, text="DIRT CLEANING PROGRESS", font=ctk.CTkFont(family="Segoe UI", size=10, weight="bold"), text_color=("#64748b", "#94a3b8")).pack(anchor="w", padx=12, pady=(8, 2))

        self.dirt_label = ctk.CTkLabel(dirt_card, text="0 Cleaned • 0 Remaining", font=ctk.CTkFont(family="Segoe UI", size=13, weight="bold"))
        self.dirt_label.pack(anchor="w", padx=12, pady=2)

        self.progress_bar = ctk.CTkProgressBar(dirt_card, height=10, progress_color="#10b981")
        self.progress_bar.set(0.0)
        self.progress_bar.pack(fill="x", padx=12, pady=(4, 6))

        self.progress_pct_lbl = ctk.CTkLabel(dirt_card, text="Progress: 0.0%", font=ctk.CTkFont(family="Segoe UI", size=11), text_color=("#10b981", "#34d399"))
        self.progress_pct_lbl.pack(anchor="w", padx=12, pady=(0, 8))

        self._create_stat_card(right_frame, "CURRENT ROBOT POSITION", "(1, 1)", "robot_pos")

        # Algorithmic notes card
        notes_card = ctk.CTkFrame(right_frame, corner_radius=8, fg_color=("#ffffff", "#0f172a"), border_width=1, border_color=("#e2e8f0", "#334155"))
        notes_card.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(notes_card, text="ALGORITHMIC SPECIFICATIONS", font=ctk.CTkFont(family="Segoe UI", size=10, weight="bold"), text_color=("#64748b", "#94a3b8")).pack(anchor="w", padx=12, pady=(8, 4))

        spec_text = (
            "• Evaluation: f(n) = g(n) + h(n)\n"
            "• Heuristic: Manhattan (4-Dir) / Octile (8-Dir)\n"
            "• Step Cost: 1.0 (Orthogonal), √2 (Diagonal)\n"
            "• Corner Cutting: Prevented strictly\n"
            "• Multi-Dirt: Greedy nearest-target sequence"
        )
        ctk.CTkLabel(
            notes_card, text=spec_text, justify="left",
            font=ctk.CTkFont(family="Segoe UI", size=10),
            text_color=("#475569", "#94a3b8")
        ).pack(anchor="w", padx=12, pady=(0, 10))

    def _create_stat_card(self, parent, title: str, default_val: str, key: str, col: Optional[str] = None):
        card = ctk.CTkFrame(parent, corner_radius=8, fg_color=("#ffffff", "#0f172a"), border_width=1, border_color=("#e2e8f0", "#334155"))
        card.pack(fill="x", padx=10, pady=5)

        lbl_title = ctk.CTkLabel(card, text=title, font=ctk.CTkFont(family="Segoe UI", size=10, weight="bold"), text_color=("#64748b", "#94a3b8"))
        lbl_title.pack(anchor="w", padx=12, pady=(8, 2))

        val_col = col if col else ("#0f172a", "#f8fafc")
        lbl_val = ctk.CTkLabel(card, text=default_val, font=ctk.CTkFont(family="Segoe UI", size=15, weight="bold"), text_color=val_col)
        lbl_val.pack(anchor="w", padx=12, pady=(0, 8))

        self.stat_widgets[key] = lbl_val

    # -------------------------------------------------------------------------
    # CANVAS RENDERING
    # -------------------------------------------------------------------------
    def _get_cell_geometry(self) -> Tuple[float, float, float, float]:
        """Calculates cell size and canvas offsets for centered, square rendering."""
        cw = max(100, self.canvas.winfo_width())
        ch = max(100, self.canvas.winfo_height())

        margin = 15.0
        avail_w = cw - 2 * margin
        avail_h = ch - 2 * margin

        cell_w = avail_w / self.room.cols
        cell_h = avail_h / self.room.rows
        cell_size = max(12.0, min(cell_w, cell_h))

        grid_w = cell_size * self.room.cols
        grid_h = cell_size * self.room.rows

        offset_x = margin + (avail_w - grid_w) / 2.0
        offset_y = margin + (avail_h - grid_h) / 2.0

        return cell_size, offset_x, offset_y, (grid_w, grid_h)

    def render_grid(self):
        """Draws room grid, obstacles, dirt, explored nodes, path, and the robot."""
        self.canvas.delete("all")
        theme = THEME_COLORS[self.current_theme]
        self.canvas.configure(bg=theme["canvas_bg"])

        cell_size, ox, oy, _ = self._get_cell_geometry()

        # 1. Draw Cells
        for r in range(self.room.rows):
            for c in range(self.room.cols):
                x1 = ox + c * cell_size
                y1 = oy + r * cell_size
                x2 = x1 + cell_size
                y2 = y1 + cell_size
                coord = (r, c)

                # Base floor color
                fill_color = theme["cell_clean"]
                border_color = theme["grid_line"]

                # A* Explored & Frontier visualization (if enabled)
                if self.show_explored_var.get():
                    if coord in self.sim.frontier_nodes and coord not in self.room.obstacles:
                        fill_color = theme["cell_frontier"]
                        border_color = theme["cell_frontier_border"]
                    elif coord in self.sim.explored_nodes and coord not in self.room.obstacles:
                        fill_color = theme["cell_explored"]
                        border_color = theme["cell_explored_border"]

                # Obstacle
                if coord in self.room.obstacles:
                    fill_color = theme["cell_obstacle"]
                    border_color = theme["cell_obstacle_border"]

                # Start cell
                elif coord == self.room.start:
                    fill_color = theme["cell_start"]

                # Goal cell
                elif coord == self.room.goal:
                    fill_color = theme["cell_goal"]

                # Draw cell rectangle
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=fill_color,
                    outline=border_color,
                    width=1
                )

                # Draw Cell Labels & Icons
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                if coord == self.room.start:
                    self.canvas.create_text(
                        cx, cy, text="S",
                        fill="#ffffff", font=("Segoe UI", int(max(9, cell_size * 0.45)), "bold")
                    )
                elif coord == self.room.goal:
                    self.canvas.create_text(
                        cx, cy, text="G",
                        fill="#ffffff", font=("Segoe UI", int(max(9, cell_size * 0.45)), "bold")
                    )

                # Dirt spot (uncleaned)
                elif coord in self.room.dirt_locations:
                    rad = cell_size * 0.32
                    self.canvas.create_oval(
                        cx - rad, cy - rad, cx + rad, cy + rad,
                        fill=theme["cell_dirt"], outline=theme["cell_dirt_speck"], width=1.5
                    )
                    # Center dust texture speck
                    self.canvas.create_oval(
                        cx - rad * 0.3, cy - rad * 0.3, cx + rad * 0.3, cy + rad * 0.3,
                        fill=theme["cell_dirt_speck"], outline=""
                    )

                # Cleaned dirt spot
                elif coord in self.room.cleaned_dirt:
                    rad = cell_size * 0.28
                    self.canvas.create_oval(
                        cx - rad, cy - rad, cx + rad, cy + rad,
                        fill=theme["cell_cleaned_halo"], outline=theme["cell_cleaned"], width=1.5
                    )
                    self.canvas.create_text(
                        cx, cy, text="✓",
                        fill=theme["cell_cleaned"], font=("Segoe UI", int(max(8, cell_size * 0.35)), "bold")
                    )

        # 2. Draw Planned Path (if enabled)
        if self.show_path_var.get() and len(self.sim.planned_path) > 1:
            self._render_path(cell_size, ox, oy, theme)

        # 3. Draw Robot Vacuum
        self._render_robot(cell_size, ox, oy, theme)

    def _render_path(self, cell_size: float, ox: float, oy: float, theme: dict):
        """Draws the route line and traversed trail on the canvas."""
        path = self.sim.planned_path
        cur_idx = self.sim.current_path_index

        # Traveled segment (darker/solid trail)
        if cur_idx > 0:
            traveled_pts = []
            for i in range(cur_idx + 1):
                r, c = path[i]
                traveled_pts.extend([ox + (c + 0.5) * cell_size, oy + (r + 0.5) * cell_size])
            if len(traveled_pts) >= 4:
                self.canvas.create_line(
                    traveled_pts, fill=theme["path_traveled"],
                    width=max(2.0, cell_size * 0.10), capstyle=tk.ROUND, joinstyle=tk.ROUND
                )

        # Remaining planned path (glowing line with waypoint dots)
        if cur_idx < len(path) - 1:
            remaining_pts = []
            for i in range(cur_idx, len(path)):
                r, c = path[i]
                remaining_pts.extend([ox + (c + 0.5) * cell_size, oy + (r + 0.5) * cell_size])
            if len(remaining_pts) >= 4:
                self.canvas.create_line(
                    remaining_pts, fill=theme["path_line"],
                    width=max(2.5, cell_size * 0.12), dash=(4, 2), capstyle=tk.ROUND, joinstyle=tk.ROUND
                )

        # Waypoint nodes on remaining path
        for i in range(cur_idx + 1, len(path)):
            r, c = path[i]
            px = ox + (c + 0.5) * cell_size
            py = oy + (r + 0.5) * cell_size
            dot_rad = max(2.0, cell_size * 0.08)
            self.canvas.create_oval(
                px - dot_rad, py - dot_rad, px + dot_rad, py + dot_rad,
                fill=theme["path_dot"], outline=""
            )

    def _render_robot(self, cell_size: float, ox: float, oy: float, theme: dict):
        """Renders the custom circular robot vacuum cleaner with lidar sensor and heading."""
        r, c = self.sim.current_pos
        cx = ox + (c + 0.5) * cell_size
        cy = oy + (r + 0.5) * cell_size

        rad = cell_size * 0.42

        # Outer chassis ring
        self.canvas.create_oval(
            cx - rad, cy - rad, cx + rad, cy + rad,
            fill=theme["robot_body"], outline=theme["robot_ring"], width=2.5
        )

        # Inner bumper / accent ring
        inner_rad = rad * 0.75
        self.canvas.create_oval(
            cx - inner_rad, cy - inner_rad, cx + inner_rad, cy + inner_rad,
            fill=theme["card_bg"], outline=theme["robot_ring"], width=1.5
        )

        # Lidar Sensor Turret in center
        turret_rad = rad * 0.35
        self.canvas.create_oval(
            cx - turret_rad, cy - turret_rad, cx + turret_rad, cy + turret_rad,
            fill=theme["robot_turret"], outline=theme["robot_ring"], width=1.5
        )

        # Heading direction chevron / indicator
        dr, dc = self.sim.current_heading
        angle = math.atan2(dr, dc)  # angle in radians

        tip_x = cx + math.cos(angle) * (rad * 0.85)
        tip_y = cy + math.sin(angle) * (rad * 0.85)
        base1_x = cx + math.cos(angle + 2.5) * (rad * 0.45)
        base1_y = cy + math.sin(angle + 2.5) * (rad * 0.45)
        base2_x = cx + math.cos(angle - 2.5) * (rad * 0.45)
        base2_y = cy + math.sin(angle - 2.5) * (rad * 0.45)

        self.canvas.create_polygon(
            [tip_x, tip_y, base1_x, base1_y, cx, cy, base2_x, base2_y],
            fill=theme["robot_heading"], outline=""
        )

        # Sweep animation brush effect when running
        if self.sim.state == SimState.RUNNING:
            sweep_rad = rad * 1.15
            arc_start = (self.sweep_angle * 180 / math.pi) % 360
            self.canvas.create_arc(
                cx - sweep_rad, cy - sweep_rad, cx + sweep_rad, cy + sweep_rad,
                start=arc_start, extent=60, style=tk.ARC,
                outline=theme["robot_glow"], width=2
            )

    # -------------------------------------------------------------------------
    # MOUSE INTERACTION ON GRID
    # -------------------------------------------------------------------------
    def _canvas_to_grid(self, event_x: float, event_y: float) -> Optional[Coord]:
        cell_size, ox, oy, (gw, gh) = self._get_cell_geometry()
        if not (ox <= event_x < ox + gw and oy <= event_y < oy + gh):
            return None
        c = int((event_x - ox) // cell_size)
        r = int((event_y - oy) // cell_size)
        if self.room.is_within_bounds(r, c):
            return (r, c)
        return None

    def _on_canvas_click(self, event):
        if self.sim.state == SimState.RUNNING:
            return  # Prevent editing while actively cleaning

        self.is_mouse_down = True
        coord = self._canvas_to_grid(event.x, event.y)
        if coord is not None:
            self.last_drag_cell = coord
            self._apply_interaction(coord)

    def _on_canvas_drag(self, event):
        if self.sim.state == SimState.RUNNING:
            return

        coord = self._canvas_to_grid(event.x, event.y)
        if coord is not None and coord != self.last_drag_cell:
            self.last_drag_cell = coord
            # For dragging, allow quick painting of obstacles or erasing
            mode = self.interaction_mode.get()
            if mode in ("obstacle", "erase", "dirt"):
                self._apply_interaction(coord)

    def _on_canvas_release(self, event):
        self.is_mouse_down = False
        self.last_drag_cell = None

    def _on_canvas_hover(self, event):
        coord = self._canvas_to_grid(event.x, event.y)
        if coord != self.hover_coord:
            self.hover_coord = coord
            if coord is not None:
                self.coord_label.configure(text=f"Hover: (Row {coord[0]}, Col {coord[1]})")
            else:
                self.coord_label.configure(text="Hover: (Row -, Col -)")

    def _on_canvas_leave(self, event):
        self.hover_coord = None
        self.coord_label.configure(text="Hover: (Row -, Col -)")

    def _apply_interaction(self, coord: Coord):
        """Applies modification according to currently selected interaction mode."""
        mode = self.interaction_mode.get()
        r, c = coord

        # Cancel any active simulation path since layout modified
        self.sim.invalidate_path()

        if mode == "obstacle":
            self.room.toggle_obstacle(r, c)
        elif mode == "dirt":
            if (r, c) in self.room.dirt_locations:
                self.room.remove_dirt(r, c)
            else:
                self.room.add_dirt(r, c)
        elif mode == "start":
            if (r, c) in self.room.obstacles:
                messagebox.showwarning("Invalid Start", "Start cannot be placed on an obstacle cell.")
            else:
                self.room.set_start(r, c)
                self.sim.current_pos = (r, c)
        elif mode == "goal":
            if (r, c) in self.room.obstacles:
                messagebox.showwarning("Invalid Goal", "Goal cannot be placed on an obstacle cell.")
            else:
                self.room.set_goal(r, c)
        elif mode == "erase":
            self.room.clear_cell(r, c)

        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    # -------------------------------------------------------------------------
    # PATHFINDING & SIMULATION
    # -------------------------------------------------------------------------
    def find_optimal_path(self):
        """Calculates path using A*. Handles fast or step-by-step exploration."""
        if self.sim.state == SimState.RUNNING:
            return

        self._stop_animation()

        # Step-by-step exploration mode
        if self.step_by_step_var.get() and self.cleaning_mode_var.get() == CleaningMode.SINGLE_DESTINATION.value:
            self._start_step_by_step_search()
            return

        # Fast calculation mode
        mode = CleaningMode(self.cleaning_mode_var.get())
        success = self.sim.calculate_plan(mode=mode)

        self.render_grid()
        self.update_stats()
        self.update_status_badge()

        if not success:
            messagebox.showwarning("Path Calculation", self.sim.status_message)

    def _start_step_by_step_search(self):
        """Initializes generator-based step-by-step A* exploration animation."""
        self.sim.reset()
        self.sim.state = SimState.EXPLORING
        self.search_generator = a_star_step_generator(
            rows=self.room.rows,
            cols=self.room.cols,
            start=self.room.start,
            goal=self.room.goal,
            obstacles=self.room.obstacles,
            allow_diagonal=self.room.allow_diagonal,
        )
        self.search_active = True
        self.update_status_badge()
        self._step_search_iteration()

    def _step_search_iteration(self):
        """Pumps one step of the A* search generator."""
        if not self.search_active or self.search_generator is None:
            return

        try:
            current, open_set, closed_set, is_done, final_res = next(self.search_generator)
            self.sim.explored_nodes = closed_set
            self.sim.frontier_nodes = open_set

            if is_done and final_res is not None:
                self.search_active = False
                self.search_generator = None
                if final_res.success:
                    self.sim.planned_path = final_res.path
                    self.sim._calculate_step_costs()
                    self.sim.total_path_cost = final_res.cost
                    self.sim.state = SimState.PATH_READY
                    self.sim.status_message = final_res.message
                else:
                    self.sim.state = SimState.NO_PATH
                    self.sim.status_message = final_res.message
                    messagebox.showwarning("Search Completed", final_res.message)

                self.render_grid()
                self.update_stats()
                self.update_status_badge()
                return

            self.render_grid()
            self.update_stats()
            self.anim_timer_id = self.root.after(30, self._step_search_iteration)

        except StopIteration:
            self.search_active = False
            self.search_generator = None
            self.update_status_badge()

    def start_cleaning(self):
        """Begins robot vacuum movement along the planned route."""
        if self.sim.state == SimState.RUNNING:
            return

        # If no path calculated yet, calculate now
        if not self.sim.planned_path or self.sim.state in (SimState.READY, SimState.NO_PATH):
            mode = CleaningMode(self.cleaning_mode_var.get())
            if not self.sim.calculate_plan(mode=mode):
                messagebox.showwarning("Cannot Start", self.sim.status_message)
                return

        self.sim.resume()
        self.update_status_badge()
        self._run_animation_step()

    def _run_animation_step(self):
        """Single tick in the animation event loop."""
        if self.sim.state != SimState.RUNNING:
            return

        self.sweep_angle += 0.4
        moved = self.sim.step_forward()

        self.render_grid()
        self.update_stats()
        self.update_status_badge()

        if moved:
            delay = self.speed_ms.get()
            self.anim_timer_id = self.root.after(delay, self._run_animation_step)
        else:
            # Reached destination / completed
            self._stop_animation()
            messagebox.showinfo("Mission Completed", self.sim.status_message)

    def pause_resume(self):
        """Toggles between paused and running state."""
        if self.sim.state == SimState.RUNNING:
            self._stop_animation()
            self.sim.pause()
            self.btn_pause.configure(text="▶ Resume")
        elif self.sim.state == SimState.PAUSED:
            self.sim.resume()
            self.btn_pause.configure(text="⏸ Pause")
            self._run_animation_step()
        self.update_status_badge()

    def step_forward(self):
        """Advances robot vacuum by exactly one step."""
        if self.sim.state == SimState.RUNNING:
            self.pause_resume()

        if not self.sim.planned_path:
            mode = CleaningMode(self.cleaning_mode_var.get())
            if not self.sim.calculate_plan(mode=mode):
                messagebox.showwarning("No Path", self.sim.status_message)
                return

        moved = self.sim.step_forward()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

        if not moved and self.sim.state == SimState.COMPLETED:
            messagebox.showinfo("Mission Completed", self.sim.status_message)

    def reset_simulation(self):
        """Stops active timers and restores robot to start position."""
        self._stop_animation()
        self.sim.reset()
        self.btn_pause.configure(text="⏸ Pause")
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def _stop_animation(self):
        """Cancels any pending after() callbacks safely."""
        if self.anim_timer_id is not None:
            self.root.after_cancel(self.anim_timer_id)
            self.anim_timer_id = None
        self.search_active = False
        self.search_generator = None

    # -------------------------------------------------------------------------
    # ROOM MANAGEMENT ACTIONS
    # -------------------------------------------------------------------------
    def load_demo_room(self):
        self._stop_animation()
        self.room.load_demo_room()
        self.sim.reset()
        self.sim.invalidate_path()
        self.grid_size_menu.set("15 × 15")
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def generate_random_room(self):
        self._stop_animation()
        self.room.generate_random(obstacle_density=0.20, dirt_count=5)
        self.sim.reset()
        self.sim.invalidate_path()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def clear_room(self):
        self._stop_animation()
        self.room.clear_room()
        self.sim.reset()
        self.sim.invalidate_path()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def save_room(self):
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Room Files", "*.json"), ("All Files", "*.*")],
            title="Save SmartSweep Room Layout"
        )
        if not filepath:
            return
        try:
            self.room.save_to_json(filepath)
            messagebox.showinfo("Room Saved", f"Successfully saved room layout to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save room:\n{e}")

    def load_room(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("JSON Room Files", "*.json"), ("All Files", "*.*")],
            title="Load SmartSweep Room Layout"
        )
        if not filepath:
            return
        try:
            self._stop_animation()
            self.room.load_from_json(filepath)
            self.sim.reset()
            self.sim.invalidate_path()
            size_str = f"{self.room.rows} × {self.room.cols}"
            if size_str in ["10 × 10", "15 × 15", "20 × 20", "25 × 25"]:
                self.grid_size_menu.set(size_str)
            self.diagonal_var.set(self.room.allow_diagonal)
            self.render_grid()
            self.update_stats()
            self.update_status_badge()
            messagebox.showinfo("Room Loaded", f"Successfully loaded room ({self.room.rows}×{self.room.cols}) from:\n{filepath}")
        except RoomError as e:
            messagebox.showerror("Load Error", f"Room format error:\n{e}")
        except Exception as e:
            messagebox.showerror("Load Error", f"Unexpected error reading file:\n{e}")

    # -------------------------------------------------------------------------
    # SETTINGS & CONTROLS HANDLERS
    # -------------------------------------------------------------------------
    def _on_grid_size_change(self, choice: str):
        self._stop_animation()
        parts = choice.split("×")
        dim = int(parts[0].strip())
        self.room.set_dimensions(dim, dim)
        self.room.start = (1, 1)
        self.room.goal = (dim - 2, dim - 2)
        self.room.clear_room()
        self.sim.reset()
        self.sim.invalidate_path()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def _on_diagonal_toggle(self):
        self._stop_animation()
        self.room.allow_diagonal = self.diagonal_var.get()
        self.sim.invalidate_path()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def _on_cleaning_mode_change(self, val: str):
        self._stop_animation()
        self.sim.invalidate_path()
        self.render_grid()
        self.update_stats()
        self.update_status_badge()

    def _on_speed_change(self, val: float):
        # Invert so right = faster (lower delay)
        # slider from 20 to 300
        delay = int(320 - val)
        self.speed_ms.set(max(15, delay))

    def _toggle_theme(self):
        if self.theme_switch.get() == 1:
            ctk.set_appearance_mode("dark")
            self.current_theme = "dark"
        else:
            ctk.set_appearance_mode("light")
            self.current_theme = "light"
        self.render_grid()
        self.update_status_badge()

    # -------------------------------------------------------------------------
    # STATISTICS & STATUS BADGE UPDATE
    # -------------------------------------------------------------------------
    def update_stats(self):
        stats = self.sim.get_stats()

        if "status_card" in self.stat_widgets:
            self.stat_widgets["status_card"].configure(text=self.sim.state.value)
        if "path_cost" in self.stat_widgets:
            self.stat_widgets["path_cost"].configure(text=f"{stats.total_path_cost:.2f} units")
        if "dist_traveled" in self.stat_widgets:
            self.stat_widgets["dist_traveled"].configure(text=f"{stats.distance_traveled:.2f} units")
        if "steps_card" in self.stat_widgets:
            self.stat_widgets["steps_card"].configure(text=f"{stats.steps_completed} / {stats.total_steps}")
        if "nodes_explored" in self.stat_widgets:
            self.stat_widgets["nodes_explored"].configure(text=str(stats.nodes_explored_count))
        if "exec_time" in self.stat_widgets:
            self.stat_widgets["exec_time"].configure(text=f"{stats.execution_time_ms:.2f} ms")
        if "robot_pos" in self.stat_widgets:
            self.stat_widgets["robot_pos"].configure(text=f"({stats.current_position[0]}, {stats.current_position[1]})")

        # Dirt counts and progress
        self.dirt_label.configure(
            text=f"{stats.dirt_cleaned_count} Cleaned • {stats.dirt_remaining_count} Remaining"
        )
        self.progress_bar.set(stats.progress_percentage / 100.0)
        self.progress_pct_lbl.configure(text=f"Progress: {stats.progress_percentage:.1f}%")

    def update_status_badge(self):
        state = self.sim.state
        color_map = {
            SimState.READY: ("#10b981", "● READY"),
            SimState.CALCULATING: ("#38bdf8", "⏳ CALCULATING"),
            SimState.EXPLORING: ("#a855f7", "🔍 EXPLORING"),
            SimState.PATH_READY: ("#06b6d4", "✓ PATH READY"),
            SimState.RUNNING: ("#3b82f6", "▶ CLEANING"),
            SimState.PAUSED: ("#f59e0b", "⏸ PAUSED"),
            SimState.COMPLETED: ("#10b981", "★ MISSION COMPLETE"),
            SimState.NO_PATH: ("#ef4444", "✖ NO PATH FOUND"),
        }
        col, txt = color_map.get(state, ("#94a3b8", state.value))
        self.status_badge.configure(text=txt, text_color=col)

    def _show_about_dialog(self):
        info_text = (
            "SmartSweep AI – Intelligent Robot Vacuum Path Planner\n\n"
            "Academic PBL Project: Computer Science / AI & Robotics\n\n"
            "Features:\n"
            "• Pure A* Pathfinding from scratch using Python heapq\n"
            "• Evaluation function: f(n) = g(n) + h(n)\n"
            "• 4-Directional Manhattan & 8-Directional Octile heuristics\n"
            "• Diagonal corner-cutting prevention\n"
            "• Multi-dirt cleaning with greedy nearest-target routing\n"
            "• Interactive room grid, demo presets, and JSON persistence\n"
            "• Real-time animation with live telemetry metrics\n\n"
            "Developed for College Project-Based Learning (PBL)."
        )
        messagebox.showinfo("About SmartSweep AI", info_text)
