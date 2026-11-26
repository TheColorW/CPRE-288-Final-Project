"""
I had to figure A LOT of stuff out but hopefully with some tweaking it'll be compatible with our cybot's data
For now it has the basic layout and some commands as well as a basic map of the testing field!

I need to actually have the cybot in order to see how plausible it is to output a basic map lol

Author: Joseph Rial
"""

# ---------------------------------------------------------------------------
# IMPORTS
# ---------------------------------------------------------------------------

import socket
import tkinter as tk
from tkinter import ttk

# ---------------------------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------------------------

# IP Address and Port
HOST = "127.0.0.1"  # TODO "192.168.1.1"
PORT = 65432        # TODO 288

# Width and height of testing area in cm
ARENA_W_CM = 462.0
ARENA_H_CM = 262.0

# How many px on the canvas represent 1 cm
PX_PER_CM = 1.0

# Pixels left around the arena inside the canvas so the outline isn't cut off
CANVAS_MARGIN = 10

# Rack numbers
# BL - 0
# TL - 1
# BR - 2
# TR - 3
RACKS = {
    "Bottom-Left (0)": 0,
    "Top-Left (1)": 1,
    "Bottom-Right (2)": 2,
    "Top-Right (3)": 3,
}

# Side letters
SIDES = {
    "LEFT": "L",
    "RIGHT": "R",
    "NORTH": "N",
    "SOUTH": "S",
}


# ---------------------------------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------------------------------

# Used to send messages to log
def log(msg):
    # Enable textbox so insert can be used
    log_output.configure(state="normal")
    # Insert msg
    log_output.insert("end", msg + "\n")
    # Autoscroll to bottom
    log_output.see("end")
    # Disable textbox
    log_output.configure(state="disabled")


# Polling/Idle handler
# Disables buttons when running a command
busy_buttons = []
def set_busy(is_busy):
    for button in busy_buttons:
        if is_busy:
            button.state(["disabled"])
        else:
            button.state(["!disabled"])


# ---------------------------------------------------------------------------
# MAIN WINDOW
# ---------------------------------------------------------------------------

root = tk.Tk() # Main window
root.title("Galumph's Robo Delivery v0.2") # Title of window
root.geometry("1100x760") # Width x Height of window in px
root.minsize(900, 600) # How small the user can make the window

# ---------------------------------------------------------------------------
# SOCKET COMMUNICATION
# ---------------------------------------------------------------------------

cybot = None
cybot_socket = None

def connect_to_cybot():
    global cybot
    global cybot_socket

    try:
        # Create a socket object
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Avoids freezing if invalid IP
        cybot_socket.settimeout(3)
        # Connect to the socket  (Note: Server must first be running)
        cybot_socket.connect((HOST, PORT))
        # Set back to normal blocking
        cybot_socket.settimeout(None)
        # Makefile creates a file object out of a socket
        cybot = cybot_socket.makefile("rbw", buffering=0)

        # Log success
        log("Connected")
        connect_button.configure(text="Disconnect", command=disconnect_from_cybot)

    except Exception as e: 
        log(f"Connection failed: {e}")
        cybot = None
        cybot_socket = None

def disconnect_from_cybot():
    global cybot
    global cybot_socket

    try: 
        if cybot_socket is not None:
            # Wakes any blocked read
            cybot_socket.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass

    try:
        if cybot is not None:
            cybot.close()
    except Exception as e: 
            log(str(e))

    # Reset State
    cybot = None
    connect_button.configure(text="Connect", command=connect_to_cybot)
    log("Disconnected")

def send_command(text):
    global cybot
    if cybot is None:
        log("ERROR: Not Connected")
        return
    try:
        cybot.write((text + "\n").encode())
        log(f"Command sent: {text}")
    except Exception as e: 
            log(f"Send Failed: {e}")
            disconnect_from_cybot()

# ---------------------------------------------------------------------------
# TOP LEVEL LAYOUT
# ---------------------------------------------------------------------------
#TODO REMOVE THE DEBUG STUFF BEFORE COMMITING BROSKI

connect_button = ttk.Button(root, text="Connect", command=connect_to_cybot)
connect_button.pack()

# Left frame with controls for the user to configure the cybot
control_frame = ttk.Frame(root, padding=10, relief="solid", borderwidth=1)
control_frame.pack(side="left", fill="y")

# Right frame with output
output_frame = ttk.Frame(root, padding=10, relief="solid", borderwidth=1)
output_frame.pack(side="left", fill="both", expand=True)


# ---------------------------------------------------------------------------
# OUTPUT LOG PANE (child of output_frame)
# ---------------------------------------------------------------------------

# Output log widget
log_frame = ttk.Frame(output_frame)
log_frame.pack(side="top", fill="both", expand=True)

# Read only textbox for log output
log_output = tk.Text(log_frame, width=60, height=12, state="disabled")
log_output.pack(side="left", fill="both", expand=True)

# Log scrollbar
log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=log_output.yview)
log_output.configure(yscrollcommand=log_scroll.set)
log_scroll.pack(side="right", fill="y")

# ---------------------------------------------------------------------------
# CONTROL PANE (child of control_frame)
# ---------------------------------------------------------------------------

# Label
ttk.Label(control_frame, text="Controls", font=("TkDefaultFont", 12, "bold")).pack(pady=(0, 8))

# --- Form 1: go to a rack -> "r <rack> <side>" ---
rack_form = ttk.LabelFrame(control_frame, text="Go to rack", padding=8)
rack_form.pack(fill="x", pady=5)

rack_choice = tk.StringVar(value="Bottom-Left (0)")
side_choice = tk.StringVar(value="LEFT")

# Rack selection box
ttk.Label(rack_form, text="Rack:").grid(row=0, column=0, sticky="w", pady=2)
rack_box = ttk.Combobox(rack_form,
                        textvariable=rack_choice,
                        values=list(RACKS.keys()),
                        state="readonly",
                        width=18)
rack_box.grid(row=0, column=1, pady=2)

# Side selection box
ttk.Label(rack_form, text="Side:").grid(row=1, column=0, sticky="w", pady=2)
side_box = ttk.Combobox(rack_form,
                        textvariable=side_choice,
                        values=list(SIDES.keys()),
                        state="readonly",
                        width=18)
side_box.grid(row=1, column=1, pady=2)

def on_go_rack():
    rack_number = RACKS[rack_choice.get()]
    side_letter = SIDES[side_choice.get()]
    send_command(f"r {rack_number} {side_letter}")

# Button
go_rack_button = ttk.Button(rack_form,
                            text="Go",
                            command=on_go_rack)
go_rack_button.grid(row=2, column=0, columnspan=2, pady=(6, 0))
busy_buttons.append(go_rack_button)

# --- Form 2: Full Mission -> "m <r1> <s1> <r2> <s2>" ---
mission_form = ttk.LabelFrame(control_frame, text="Full Mission", padding=8)
mission_form.pack(fill="x", pady=5)

# Location drop down options
# Defaults
m_rack1 = tk.StringVar(value="Bottom-Left (0)")
m_side1 = tk.StringVar(value="LEFT")
m_rack2 = tk.StringVar(value="Top-Right (3)")
m_side2 = tk.StringVar(value="RIGHT")

# First rack and side
ttk.Label(mission_form, text="Pickup:").grid(row=0, column=0, sticky="w", pady=2)
ttk.Combobox(mission_form,
             textvariable=m_rack1,
             values=list(RACKS.keys()),
             state="readonly",
             width=14).grid(row=0, column=1, pady=2)
ttk.Combobox(mission_form,
             textvariable=m_side1,
             values=list(SIDES.keys()),
             state="readonly",
             width=6).grid(row=0, column=2, pady=2)

# Second rack and side
ttk.Label(mission_form, text="Second:").grid(row=1, column=0, sticky="w", pady=2)
ttk.Combobox(mission_form,
             textvariable=m_rack2,
             values=list(RACKS.keys()),
             state="readonly",
             width=14).grid(row=1, column=1, pady=2)
ttk.Combobox(mission_form,
             textvariable=m_side2,
             values=list(SIDES.keys()),
             state="readonly",
             width=6).grid(row=1, column=2, pady=2)

def on_start_mission():
    r1 = RACKS[m_rack1.get()]
    s1 = SIDES[m_side1.get()]
    r2 = RACKS[m_rack2.get()]
    s2 = SIDES[m_side2.get()]
    send_command(f"m {r1} {s1} {r2} {s2}")

# Start mission button
start_mission_button = ttk.Button(mission_form, text="Start Mission", command=on_start_mission)
start_mission_button.grid(row=2, column=0, columnspan=3, pady=(6, 0))
busy_buttons.append(start_mission_button)

# --- Form 3: Drive to coordinate -> "g <x> <y>" ---
coord_form = ttk.LabelFrame(control_frame, text="Go to Coordinates", padding=8)

# X and Y input boxes
coord_x = tk.StringVar(value="0.0")
coord_y = tk.StringVar(value="0.0")

ttk.Label(coord_form, text=f"x (0-{ARENA_W_CM})").grid(row=0, column=0, sticky="w", pady=2)
ttk.Spinbox(coord_form, from_=0, to=ARENA_W_CM, textvariable=coord_x, width=8).grid(row=0, column=1, pady=2)

ttk.Label(coord_form, text=f"y (0-{ARENA_H_CM})").grid(row=1, column=0, sticky="w", pady=2)
ttk.Spinbox(coord_form, from_=0, to=ARENA_H_CM, textvariable=coord_y, width=8).grid(row=1, column=1, pady=2)
coord_form.pack(fill="x", pady=5)


# Drive to coord function
def on_go_coord():
    try:
        x = float(coord_x.get())
        y = float(coord_y.get())
    except ValueError:
        log("ERROR: X and Y must be numbers")
        return
    if not (0 <= x <= ARENA_W_CM and 0 <= y <= ARENA_H_CM):
        log("ERROR: coordinate is outside the arena")
        return
    send_command(f"g {x} {y}")

# --- Other functions ---
misc_form = ttk.LabelFrame(control_frame, text="Other", padding=8)
misc_form.pack(fill="x", pady=5)

scan_button = ttk.Button(misc_form, text="Scan (s)", command=lambda: send_command("s"))
scan_button.pack(fill="x", pady=2)
reset_button = ttk.Button(misc_form, text="Reset pose (c)", command=lambda: send_command("c"))
reset_button.pack(fill="x", pady=2)
cliff_button = ttk.Button(misc_form, text="Cliff sensors (p)", command=lambda: send_command("p"))
cliff_button.pack(fill="x", pady=2)
busy_buttons.extend([scan_button, reset_button, cliff_button])
    

# Drive button
go_coord_button = ttk.Button(coord_form, text="Go", command=on_go_coord)
go_coord_button.grid(row=2, column=0, columnspan=2, pady=(6, 0))
busy_buttons.append(go_coord_button)

# ---------------------------------------------------------------------------
# TESTFIELD CANVAS
# ---------------------------------------------------------------------------

# Canvas
canvas_width = int(ARENA_W_CM * PX_PER_CM) + 2 * CANVAS_MARGIN
canvas_height = int(ARENA_H_CM * PX_PER_CM) + 2 * CANVAS_MARGIN

canvas_title = ttk.Label(output_frame, text="Test Field Visualization")
canvas_title.pack(side="top", anchor="w", pady=(10, 2))

field_canvas = tk.Canvas(output_frame, width=canvas_width, height=canvas_height,
                         background="white", highlightthickness=1,
                         highlightbackground="gray")
field_canvas.pack(side="top")

# Convert cm to px
def cm_to_px(x_cm, y_cm):
    px = CANVAS_MARGIN + x_cm * PX_PER_CM
    py = CANVAS_MARGIN + (ARENA_H_CM - y_cm) * PX_PER_CM
    return px, py

# --- Canvas Drawing Logic ---

# TODO

# ---------------------------------------------------------------------------
# START
# ---------------------------------------------------------------------------
log("Click \"Connect\" to try to connect to the server/cybot. START SERVER FIRST!")
root.mainloop()
