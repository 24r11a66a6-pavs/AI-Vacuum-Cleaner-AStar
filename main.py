"""
main.py - Entry point for SmartSweep AI Desktop Application.
Initializes CustomTkinter root window and launches the SmartSweepGUI.
"""

import sys
import customtkinter as ctk
from gui import SmartSweepGUI


def main():
    # Set high DPI awareness and appearance
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    root = ctk.CTk()
    app = SmartSweepGUI(root)

    # Center window on screen
    root.update_idletasks()
    width = 1400
    height = 900
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry(f"{width}x{height}+{max(0, x)}+{max(0, y)}")

    try:
        root.mainloop()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
