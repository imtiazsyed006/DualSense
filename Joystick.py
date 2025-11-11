import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import threading
import socket
import struct
import time
import tkinter as tk
from tkinter import ttk
import pygame
import queue
import sys

# =================== CONFIG ===================
# Axes (typical DualSense via SDL/pygame; change if needed)
LEFT_X_AXIS   = 0  # Left stick X  -> roll  (-1..+1)
LEFT_Y_AXIS   = 1  # Left stick Y  -> pitch (-1..+1, up is -1 so we invert)
RIGHT_X_AXIS  = 2  # Right stick X -> sway  (-1..+1)
RIGHT_Y_AXIS  = 3  # Right stick Y -> surge (-1..+1, forward is -1 so we invert)
L2_AXIS       = 4  # Left trigger  -> yaw (0..+1 magnitude, sign via L1)
R2_AXIS       = 5  # Right trigger -> heave (0..+1 magnitude, sign via R1)

# Buttons (indices may differ per OS/driver)
L1_BUTTON           = 9      # set to your L1 index
R1_BUTTON           = 10     # R1
LIGHT_TOGGLE_BUTTON = 4     # <-- set this to the button you want for LIGHT toggle

# D-pad as BUTTONS (fallback for platforms where D-pad is not a hat)
# If your log shows e.g. "[BTN] Btn#13 pressed" for Up, set UP=13.
DPAD_UP_BUTTON    = 11  # set to None if unknown; adjust after seeing logs
DPAD_DOWN_BUTTON  = 12  # set to None if unknown; adjust after seeing logs
# Optionally LEFT/RIGHT if you need them:
DPAD_LEFT_BUTTON  = None
DPAD_RIGHT_BUTTON = None

# Optional button name map (helps logs). Extend as you discover indices.
BTN_NAMES = {
    0: "✖ Cross", 1: "○ Circle", 2: "□ Square", 3: "△ Triangle",
    4: "Share", 5: "PS", 6: "Options", 7: "L3", 8: "R3",
    9: "L1", 10: "R1", 11: "LightBtn", 15: "TouchPad",
    13: "DPadLeft", 12: "DPadDown", 16: "DPadLeft", 14: "DPadRight"
}
def btn_name(i): return BTN_NAMES.get(i, f"Btn#{i}")

# Deadzones
STICK_DEADZONE   = 0.08
TRIGGER_DEADZONE = 0.02

# Trigger rest (SDL often reports -1.0 released -> +1.0 pressed)
TRIG_REST  = -1.0

# Joystick/sample rate
LOOP_HZ = 30

# Light intensity bounds (CCR-style numbers, e.g., 6000..7600)
LIGHT_MIN = 6000
LIGHT_MAX = 7600
LIGHT_STEP = 25           # increment per repeat
REPEAT_FIRST_MS = 220     # initial delay before auto-repeat
REPEAT_NEXT_MS  = 70      # repeat rate when held

# Network defaults
DEFAULT_IP   = "192.168.0.25"
SEND_PORT    = 2001  # commands -> STM32
RECV_PORT    = 2000  # replies  <- STM32

# Log history limit (lines)
LOG_MAX_LINES = 2000
# =============================================

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def deadzone(v, dz):   return 0.0 if abs(v) < dz else v

def trigger_to_01(raw, rest=-1.0, dz=0.02):
    """Convert trigger raw [-1..+1] with rest≈-1 to 0..1."""
    if abs(raw - rest) < dz or raw <= rest:
        return 0.0
    t = (raw - rest) / (1.0 - rest)   # [rest..+1] -> [0..1]
    return clamp(t, 0.0, 1.0)

class ButtonDebouncer:
    def __init__(self, debounce_ms=100):
        self.debounce_ms = debounce_ms
        self._debounced_state = False
        self._last_raw_state = False
        self._last_change_ms = 0
    def update(self, raw_state: bool, now_ms: int):
        if raw_state != self._last_raw_state:
            self._last_raw_state = raw_state
            self._last_change_ms = now_ms
        else:
            if (now_ms - self._last_change_ms) >= self.debounce_ms:
                self._debounced_state = raw_state
        return self._debounced_state

class HoldRepeater:
    """Edge+hold auto-repeat for hat/button up/down."""
    def __init__(self, first_ms=220, next_ms=70):
        self.first_ms = first_ms
        self.next_ms  = next_ms
        self.state    = 0     # -1 down, 0 none, +1 up
        self.started  = False
        self.t0_ms    = 0
        self.last_ms  = 0
    def feed(self, dir_state: int, now_ms: int):
        """dir_state in {-1,0,+1}. Returns True when we should step."""
        step = False
        if dir_state != self.state:
            self.state   = dir_state
            self.started = dir_state != 0
            self.t0_ms   = now_ms
            self.last_ms = now_ms
            if self.started:
                step = True  # initial step on press
        else:
            if self.state != 0 and self.started:
                elapsed = now_ms - self.t0_ms
                if elapsed >= self.first_ms:
                    if (now_ms - self.last_ms) >= self.next_ms:
                        self.last_ms = now_ms
                        step = True
        return step

class NetClient:
    """
    Two TCP sockets:
      - tx_sock -> STM32: SEND on port 2001
      - rx_sock <- STM32: RECV on port 2000
    """
    def __init__(self, log_fn, set_led_fn):
        self.tx_sock = None
        self.rx_sock = None
        self._rx_thread = None
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self.log = log_fn
        self.set_led = set_led_fn

    def connect(self, ip: str):
        self.disconnect()
        try:
            self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tx_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.tx_sock.connect((ip, SEND_PORT))

            self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rx_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.rx_sock.connect((ip, RECV_PORT))

            self._stop.clear()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

            self.set_led(True)
            self.log(f"Connected to {ip}: TX->{SEND_PORT}, RX<-{RECV_PORT}", "sys")
            return True
        except Exception as e:
            self.log(f"Connect failed: {e}", "err")
            self.disconnect()
            return False

    def disconnect(self):
        self._stop.set()
        try:
            if self.rx_sock:
                try: self.rx_sock.shutdown(socket.SHUT_RDWR)
                except: pass
                self.rx_sock.close()
        finally:
            self.rx_sock = None
        try:
            if self.tx_sock:
                try: self.tx_sock.shutdown(socket.SHUT_RDWR)
                except: pass
                self.tx_sock.close()
        finally:
            self.tx_sock = None
        self.set_led(False)

    def is_connected(self):
        return (self.tx_sock is not None) and (self.rx_sock is not None)

    def _rx_loop(self):
        try:
            self.rx_sock.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    data = self.rx_sock.recv(4096)
                    if not data:
                        self.log("RX: connection closed by peer", "err")
                        break
                    try:
                        s = data.decode("utf-8", errors="replace")
                        self.log(s.rstrip("\r\n"), "rx")
                    except Exception:
                        self.log(data.hex(), "rx")
                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"RX error: {e}", "err")
                    break
        finally:
            self.disconnect()

    def send_input_packet_f32l(self, roll, pitch, yaw, surge, sway, heave,
                               light_intensity, light_enabled):
        """
        Packet: b"inputF32L:" + 6*float32 + uint32 (intensity) + uint8 (enabled)
        Layout: <6f I ?   (little-endian)
        """
        if not self.tx_sock:
            return
        try:
            payload = struct.pack("<6fI?",
                                  float(roll), float(pitch), float(yaw),
                                  float(surge), float(sway), float(heave),
                                  int(light_intensity),
                                  bool(light_enabled))
            buf = b"inputF32L:" + payload
            with self._tx_lock:
                self.tx_sock.sendall(buf)
            # quiet: no per-frame TX log
        except Exception as e:
            self.log(f"TX error: {e}", "err")
            self.disconnect()

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PS5 → STM32 (TCP) Controller (float32 + Light)")
        self.geometry("1000x620")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # ====== UI ======
        top = ttk.Frame(self); top.pack(fill="x", padx=8, pady=8)
        ttk.Label(top, text="STM32 IP:").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(top, textvariable=self.ip_var, width=18).pack(side="left", padx=(4,10))
        self.btn_connect = ttk.Button(top, text="Connect", command=self.on_connect); self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self.on_disconnect, state="disabled"); self.btn_disconnect.pack(side="left", padx=4)
        self.led_canvas = tk.Canvas(top, width=18, height=18, highlightthickness=0); self.led_canvas.pack(side="left", padx=10)
        self.led_id = self.led_canvas.create_oval(2,2,16,16, fill="#aa2222", outline="black")
        ttk.Label(top, text=f"Loop: {LOOP_HZ} Hz").pack(side="right")

        middle = ttk.Frame(self); middle.pack(fill="x", padx=8, pady=(0,8))

        # ---- Light widget ----
        light_frame = ttk.LabelFrame(middle, text="Light")
        light_frame.pack(side="left", padx=8, pady=4, ipadx=8, ipady=8)
        self.light_canvas = tk.Canvas(light_frame, width=140, height=140, bg="white", highlightthickness=0)
        self.light_canvas.grid(row=0, column=0, rowspan=3, padx=8, pady=8)
        self.light_outer = self.light_canvas.create_oval(10, 10, 130, 130, fill="#303030", outline="#202020", width=2)
        self.light_inner = self.light_canvas.create_oval(30, 30, 110, 110, fill="#202020", outline="")
        self.light_lbl    = ttk.Label(light_frame, text="Intensity: 6000")
        self.light_state  = ttk.Label(light_frame, text="State: Disabled")
        self.light_hint   = ttk.Label(light_frame, text="Toggle: LIGHT btn\nAdjust: D-pad Up/Down", foreground="gray")
        self.light_lbl.grid(row=0, column=1, sticky="w", padx=(6,4))
        self.light_state.grid(row=1, column=1, sticky="w", padx=(6,4))
        self.light_hint.grid(row=2, column=1, sticky="w", padx=(6,4))

        # Log
        log_frame = ttk.Frame(self); log_frame.pack(fill="both", expand=True, padx=8, pady=(0,8))
        self.log_text = tk.Text(log_frame, wrap="none", height=18); self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview); scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)
        self.log_text.tag_config("tx",  foreground="blue")
        self.log_text.tag_config("rx",  foreground="green")
        self.log_text.tag_config("sys", foreground="gray")
        self.log_text.tag_config("err", foreground="red")

        # ====== State ======
        self.msg_q = queue.Queue()
        self.net = NetClient(self._log_enqueue, self._set_led)
        self.running = True

        # toggles (determine sign for trigger channels)
        self.heave_invert = False
        self.yaw_invert   = False
        self.r1_deb = ButtonDebouncer(100)
        self.l1_deb = ButtonDebouncer(100)

        # Light control state
        self.light_enabled   = False
        self.light_intensity = LIGHT_MIN
        self.light_btn_deb   = ButtonDebouncer(120)

        self._prev_light_raw = False  # track raw edge for light toggle
        # D-pad repeat (via HAT 0)
        self.rep = HoldRepeater(REPEAT_FIRST_MS, REPEAT_NEXT_MS)

        # Track last-sent values for change-driven TX
        self._last_sent = {
            "roll": None, "pitch": None, "yaw": None,
            "surge": None, "sway": None, "heave": None,
            "light_intensity": None, "light_enabled": None
        }
        self._eps_axis = 0.01   # send only if |delta| > 0.01

        # init pygame joystick in background
        self._joy_thread = threading.Thread(target=self._joystick_loop, daemon=True)
        self._joy_thread.start()

        # drain log queue and animate light
        self.after(30, self._drain_log_queue)
        self.after(50, self._animate_light)

    # ------------- GUI helpers -------------
    def _set_led(self, on: bool):
        color = "#22aa22" if on else "#aa2222"
        def do():
            self.led_canvas.itemconfig(self.led_id, fill=color)
            self.btn_connect.config(state=("disabled" if on else "normal"))
            self.btn_disconnect.config(state=("normal" if on else "disabled"))
        self.after(0, do)

    def _log_enqueue(self, text, tag): self.msg_q.put((text, tag))

    def _drain_log_queue(self):
        try:
            while True:
                text, tag = self.msg_q.get_nowait()
                self._log_text(text, tag)
        except queue.Empty:
            pass
        finally:
            self.after(50, self._drain_log_queue)

    def _log_text(self, text, tag):
        self.log_text.insert("end", text + "\n", tag)
        self.log_text.see("end")
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > LOG_MAX_LINES:
            cut = max(1, LOG_MAX_LINES // 10)
            self.log_text.delete("1.0", f"{cut}.0")

    def _animate_light(self):
        # Color depends on enabled + intensity
        en = self.light_enabled
        I  = self.light_intensity
        bright = 0.15 if not en else (0.15 + 0.85 * ((I - LIGHT_MIN) / (LIGHT_MAX - LIGHT_MIN)))
        bright = clamp(bright, 0.0, 1.0)
        gamma  = 2.2
        g = pow(bright, 1/gamma)
        r = int(40 + 215 * g); gcol = int(35 + 200 * g); b = int(10 + 30 * g)
        fill_inner = f"#{r:02x}{gcol:02x}{b:02x}"
        fill_outer = f"#{max(0,r-40):02x}{max(0,gcol-40):02x}{max(0,b-40):02x}"
        self.light_canvas.itemconfig(self.light_inner, fill=fill_inner)
        self.light_canvas.itemconfig(self.light_outer, fill=fill_outer)
        self.light_lbl.config(text=f"Intensity: {self.light_intensity}")
        self.light_state.config(text=f"State: {'Enabled' if self.light_enabled else 'Disabled'}")
        self.after(80, self._animate_light)

    # ------------- Connect / Disconnect -------------
    def on_connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            self._log_enqueue("No IP entered", "err"); return
        self._log_enqueue(f"Connecting to {ip}...", "sys")
        threading.Thread(target=lambda: self.net.connect(ip), daemon=True).start()

    def on_disconnect(self):
        self._log_enqueue("Disconnecting…", "sys")
        threading.Thread(target=self.net.disconnect, daemon=True).start()

    # ------------- Joystick loop -------------
    def _joystick_loop(self):
        pygame.joystick.init()
        pygame.display.init()

        if pygame.joystick.get_count() == 0:
            self._log_enqueue("No joystick detected. Plug in your controller and restart.", "err")
            return

        js = pygame.joystick.Joystick(0); js.init()
        self._log_enqueue(f"Controller: {js.get_name()} (axes={js.get_numaxes()}, buttons={js.get_numbuttons()}, hats={js.get_numhats()})", "sys")

        # Track prior button/hat states for edge detection
        nbtn = js.get_numbuttons()
        prev_btns = [False] * nbtn
        prev_hat  = (0, 0)

        dt = 1.0 / LOOP_HZ
        try:
            while self.running:
                pygame.event.pump()
                now_ms = pygame.time.get_ticks()

                # -------- Buttons (edge-detected, debounced) --------
                r1_raw    = bool(js.get_button(R1_BUTTON)) if R1_BUTTON < nbtn else False
                l1_raw    = bool(js.get_button(L1_BUTTON)) if L1_BUTTON < nbtn else False
                light_raw = bool(js.get_button(LIGHT_TOGGLE_BUTTON)) if LIGHT_TOGGLE_BUTTON < nbtn else False

                r1 = self.r1_deb.update(r1_raw, now_ms)
                l1 = self.l1_deb.update(l1_raw, now_ms)
                light_btn = self.light_btn_deb.update(light_raw, now_ms)

                # Which button was pressed (generic print)
                for i in range(nbtn):
                    pressed = bool(js.get_button(i))
                    if pressed and not prev_btns[i]:
                        self._log_enqueue(f"[BTN] {btn_name(i)} pressed", "sys")
                    prev_btns[i] = pressed

                # R1/L1 toggle modes
                if not hasattr(self, "_prev_r1"):
                    self._prev_r1 = r1; self._prev_l1 = l1; self._prev_light = light_btn
                if r1 and not self._prev_r1:
                    self.heave_invert = not self.heave_invert
                    self._log_enqueue(f"[BTN] R1 pressed → Heave mode = {'NEG(−)' if self.heave_invert else 'POS(+)' }", "sys")
                if l1 and not self._prev_l1:
                    self.yaw_invert = not self.yaw_invert
                    self._log_enqueue(f"[BTN] L1 pressed → Yaw mode = {'NEG(−)' if self.yaw_invert else 'POS(+)' }", "sys")

                # Light toggle behavior (print ON/OFF + intensity)
# --- LIGHT TOGGLE: use RAW rising edge so quick taps work ---
                if light_raw and not self._prev_light_raw:
                    if self.light_intensity > LIGHT_MIN and self.light_enabled:
                        self.light_intensity = LIGHT_MIN
                        self.light_enabled = False
                        self._log_enqueue(f"[LIGHT] OFF, intensity→{self.light_intensity}", "sys")
                    elif not self.light_enabled:
                        self.light_enabled = True
                        self._log_enqueue(f"[LIGHT] ON,  intensity={self.light_intensity}", "sys")
                    else:
                        self.light_enabled = False
                        self._log_enqueue(f"[LIGHT] OFF, intensity={self.light_intensity}", "sys")

                # latch previous states
                self._prev_light_raw = light_raw
                self._prev_r1 = r1
                self._prev_l1 = l1

                # -------- D-pad via HAT (primary) --------
                hat_x, hat_y = (0, 0)
                use_hat = js.get_numhats() > 0
                if use_hat:
                    hat_x, hat_y = js.get_hat(0)  # (x,y) in {-1,0,1}
                    if hat_y == +1 and prev_hat[1] != +1:
                        self._log_enqueue("[HAT] Up pressed", "sys")
                    if hat_y == -1 and prev_hat[1] != -1:
                        self._log_enqueue("[HAT] Down pressed", "sys")
                    prev_hat = (hat_x, hat_y)

                # -------- D-pad via BUTTONS (fallback) --------
                dpad_up_pressed = (DPAD_UP_BUTTON   is not None and DPAD_UP_BUTTON   < nbtn and bool(js.get_button(DPAD_UP_BUTTON)))
                dpad_dn_pressed = (DPAD_DOWN_BUTTON is not None and DPAD_DOWN_BUTTON < nbtn and bool(js.get_button(DPAD_DOWN_BUTTON)))

                # Convert to light_dir: up=+1, down=-1
                light_dir = 0
                if use_hat and hat_y != 0:
                    light_dir = 1 if hat_y > 0 else -1
                else:
                    # fallback to buttons
                    if dpad_up_pressed and not dpad_dn_pressed:
                        light_dir = +1
                        self._log_enqueue("[DPAD] Up pressed", "sys")
                    elif dpad_dn_pressed and not dpad_up_pressed:
                        light_dir = -1
                        self._log_enqueue("[DPAD] Down pressed", "sys")

                # Change intensity with hold-repeater (only when light is enabled)
                if self.rep.feed(light_dir, now_ms) and self.light_enabled:
                    old = self.light_intensity
                    if light_dir > 0:
                        self.light_intensity = clamp(self.light_intensity + LIGHT_STEP, LIGHT_MIN, LIGHT_MAX)
                    elif light_dir < 0:
                        self.light_intensity = clamp(self.light_intensity - LIGHT_STEP, LIGHT_MIN, LIGHT_MAX)
                    if self.light_intensity != old:
                        self._log_enqueue(f"[LIGHT] intensity→{self.light_intensity}", "sys")

                # -------- Axes / Triggers (with deadzones + sign toggles) --------
                lx = clamp(js.get_axis(LEFT_X_AXIS),  -1.0, 1.0)
                ly = clamp(js.get_axis(LEFT_Y_AXIS),  -1.0, 1.0)
                rx = clamp(js.get_axis(RIGHT_X_AXIS), -1.0, 1.0)
                ry = clamp(js.get_axis(RIGHT_Y_AXIS), -1.0, 1.0)
                l2 = clamp(js.get_axis(L2_AXIS),      -1.0, 1.0)
                r2 = clamp(js.get_axis(R2_AXIS),      -1.0, 1.0)

                roll  = deadzone(lx, STICK_DEADZONE)         # -1..+1
                pitch = deadzone(-ly, STICK_DEADZONE)        # invert Y → + up
                sway  = deadzone(rx, STICK_DEADZONE)
                surge = deadzone(-ry, STICK_DEADZONE)

                heave_mag = trigger_to_01(r2, rest=TRIG_REST, dz=TRIGGER_DEADZONE)
                yaw_mag   = trigger_to_01(l2, rest=TRIG_REST, dz=TRIGGER_DEADZONE)
                heave = (-heave_mag) if self.heave_invert else (+heave_mag)
                yaw   = (-yaw_mag)   if self.yaw_invert   else (+yaw_mag)

                # -------- Send only when something changed --------
                def changed(name, new, eps=None):
                    old = self._last_sent[name]
                    if old is None:
                        return True
                    if eps is None:
                        return old != new
                    return abs(new - old) > eps

                need_send = (
                    changed("roll",  roll,  0.01) or
                    changed("pitch", pitch, 0.01) or
                    changed("yaw",   yaw,   0.01) or
                    changed("surge", surge, 0.01) or
                    changed("sway",  sway,  0.01) or
                    changed("heave", heave, 0.01) or
                    changed("light_intensity", float(self.light_intensity)) or
                    changed("light_enabled",  bool(self.light_enabled))
                )

                if self.net.is_connected() and need_send:
                    self.net.send_input_packet_f32l(
                        roll, pitch, yaw, surge, sway, heave,
                        self.light_intensity, self.light_enabled
                    )
                    # latch last-sent values
                    self._last_sent.update(dict(
                        roll=roll, pitch=pitch, yaw=yaw,
                        surge=surge, sway=sway, heave=heave,
                        light_intensity=float(self.light_intensity),
                        light_enabled=bool(self.light_enabled),
                    ))

                time.sleep(dt)
        except Exception as e:
            self._log_enqueue(f"Joystick loop error: {e}", "err")
        finally:
            try: js.quit()
            except: pass
            pygame.quit()

    def on_close(self):
        self.running = False
        self.on_disconnect()
        try: time.sleep(0.2)
        except: pass
        self.destroy()

if __name__ == "__main__":
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    App().mainloop()
