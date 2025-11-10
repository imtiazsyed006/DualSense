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
LEFT_X_AXIS   = 0  # Left stick X  -> roll
LEFT_Y_AXIS   = 1  # Left stick Y  -> pitch
RIGHT_X_AXIS  = 2  # Right stick X -> sway
RIGHT_Y_AXIS  = 3  # Right stick Y -> surge
L2_AXIS       = 4  # Left trigger  -> yaw (with L1 toggle)
R2_AXIS       = 5  # Right trigger -> heave (with R1 toggle)

# Buttons (indices may differ per OS/driver; adjust if needed)
L1_BUTTON     = 9   # guess; set to your actual L1 index
R1_BUTTON     = 10  # you said R1 works at index 10 on your setup

# Output scaling
OUT_MIN     = 44
OUT_CENTER  = 60
OUT_MAX     = 76

# Deadzones
STICK_DEADZONE   = 0.08
TRIGGER_DEADZONE = 0.02

# Axis defaults
STICK_REST = 0.0
TRIG_REST  = -1.0   # many SDL mappings report -1.0 at rest for triggers

# Joystick/sample rate
LOOP_HZ = 30

# Network defaults (you can change in GUI)
DEFAULT_IP   = "192.168.0.25"
SEND_PORT    = 2001  # commands -> STM32 (your C handles inputData: here)
RECV_PORT    = 2000  # replies  <- STM32

# Log history limit (lines)
LOG_MAX_LINES = 2000
# =============================================


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def map_bidirectional(raw, deadzone, rest=0.0, out_min=44, out_center=60, out_max=76):
    if abs(raw - rest) < deadzone:
        return out_center

    span_neg = rest - (-1.0)
    span_pos = 1.0 - rest

    if raw < rest:
        t = (raw - (-1.0)) / span_neg  # 0 at -1, 1 at rest
        val = out_min + (out_center - out_min) * t
    else:
        t = (raw - rest) / span_pos    # 0 at rest, 1 at +1
        val = out_center + (out_max - out_center) * t

    return clamp(val, out_min, out_max)

def map_one_direction(raw, deadzone, rest=-1.0, out_center=60, out_max=76):
    if abs(raw - rest) < deadzone:
        return out_center
    if raw <= rest:
        return out_center
    t = (raw - rest) / (1.0 - rest)  # rest..+1 -> 0..1
    val = out_center + (out_max - out_center) * t
    return clamp(val, out_center, out_max)

class ButtonDebouncer:
    """Rising-edge debouncer for digital buttons."""
    def __init__(self, debounce_ms=100):
        self.debounce_ms = debounce_ms
        self._debounced_state = False
        self._last_raw_state = False
        self._last_change_ms = 0

    def update(self, raw_state: bool, now_ms: int):
        if raw_state != self._last_raw_state:
            # raw edge -> start window
            self._last_raw_state = raw_state
            self._last_change_ms = now_ms
        else:
            # raw state stable; after window commit debounced state
            if (now_ms - self._last_change_ms) >= self.debounce_ms:
                self._debounced_state = raw_state
        return self._debounced_state

    @property
    def state(self):
        return self._debounced_state

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
        # simple text-oriented read (your C sends "PORT2000 READY\r\n" and other small strings)
        try:
            self.rx_sock.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    data = self.rx_sock.recv(4096)
                    if not data:
                        self.log("RX: connection closed by peer", "err")
                        break
                    # try decode as ASCII-ish; fallback to hex
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

    def send_input_packet(self, roll, pitch, yaw, surge, sway, heave):
        """
        Packet format expected by your firmware:
          "inputData:" (ASCII, 10 bytes) +
          6 * uint32 little-endian: roll, pitch, yaw, surge, sway, heave
        """
        if not self.tx_sock:
            return
        try:
            payload = struct.pack("<6I",
                                  int(roll), int(pitch), int(yaw),
                                  int(surge), int(sway), int(heave))
            buf = b"inputData:" + payload
            with self._tx_lock:
                self.tx_sock.sendall(buf)
            # For the GUI log (blue)
            self.log(f"TX inputData: roll={roll} pitch={pitch} yaw={yaw} "
                     f"surge={surge} sway={sway} heave={heave}", "tx")
        except Exception as e:
            self.log(f"TX error: {e}", "err")
            self.disconnect()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PS5 → STM32 (TCP) Controller")
        self.geometry("920x560")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # ====== UI ======
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=8)

        ttk.Label(top, text="STM32 IP:").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ip_entry = ttk.Entry(top, textvariable=self.ip_var, width=18)
        ip_entry.pack(side="left", padx=(4, 10))

        self.btn_connect = ttk.Button(top, text="Connect", command=self.on_connect)
        self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)

        # LED
        self.led_canvas = tk.Canvas(top, width=18, height=18, highlightthickness=0)
        self.led_canvas.pack(side="left", padx=10)
        self.led_id = self.led_canvas.create_oval(2, 2, 16, 16, fill="#aa2222", outline="black")

        # Rate label
        self.rate_lbl = ttk.Label(top, text=f"Loop: {LOOP_HZ} Hz")
        self.rate_lbl.pack(side="right")

        # Log
        log_frame = ttk.Frame(self)
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0,8))

        self.log_text = tk.Text(log_frame, wrap="none", height=20)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        # colors/tags
        self.log_text.tag_config("tx", foreground="blue")
        self.log_text.tag_config("rx", foreground="green")
        self.log_text.tag_config("sys", foreground="gray")
        self.log_text.tag_config("err", foreground="red")

        # ====== State ======
        self.msg_q = queue.Queue()
        self.net = NetClient(self._log_enqueue, self._set_led)
        self.running = True

        # joystick values (ints in [44..76])
        self.roll = OUT_CENTER
        self.pitch = OUT_CENTER
        self.yaw = OUT_CENTER
        self.surge = OUT_CENTER
        self.sway = OUT_CENTER
        self.heave = OUT_CENTER

        # toggles & debouncers
        self.heave_invert = False
        self.yaw_invert   = False
        self.r1_deb = ButtonDebouncer(100)
        self.l1_deb = ButtonDebouncer(100)

        # init pygame joystick in background
        self._joy_thread = threading.Thread(target=self._joystick_loop, daemon=True)
        self._joy_thread.start()

        # poll the log queue into the Text widget
        self.after(30, self._drain_log_queue)

    # ---------------- Logging & LED ----------------
    def _set_led(self, on: bool):
        # Called from NetClient threads -> schedule into Tk
        color = "#22aa22" if on else "#aa2222"
        def do():
            self.led_canvas.itemconfig(self.led_id, fill=color)
            self.btn_connect.config(state=("disabled" if on else "normal"))
            self.btn_disconnect.config(state=("normal" if on else "disabled"))
        self.after(0, do)

    def _log_enqueue(self, text, tag):
        self.msg_q.put((text, tag))

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
        # append, color by tag, autoscroll, trim history
        self.log_text.insert("end", text + "\n", tag)
        self.log_text.see("end")
        # trim lines to prevent memory leak
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > LOG_MAX_LINES:
            # delete oldest 10% lines
            cut = max(1, LOG_MAX_LINES // 10)
            self.log_text.delete("1.0", f"{cut}.0")

    # ---------------- Connect / Disconnect ----------------
    def on_connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            self._log_enqueue("No IP entered", "err")
            return
        self._log_enqueue(f"Connecting to {ip}...", "sys")
        threading.Thread(target=lambda: self.net.connect(ip), daemon=True).start()

    def on_disconnect(self):
        self._log_enqueue("Disconnecting…", "sys")
        threading.Thread(target=self.net.disconnect, daemon=True).start()

    # ---------------- Joystick ----------------
    def _joystick_loop(self):
        # Start pygame
        # pygame.init()
        pygame.joystick.init()
        pygame.display.init()
        # Wait for joystick
        if pygame.joystick.get_count() == 0:
            self._log_enqueue("No joystick detected. Plug in your controller and restart.", "err")
            return

        js = pygame.joystick.Joystick(0)
        js.init()
        self._log_enqueue(f"Using controller: {js.get_name()} (axes={js.get_numaxes()}, buttons={js.get_numbuttons()})", "sys")

        dt = 1.0 / LOOP_HZ
        try:
            while self.running:
                pygame.event.pump()

                # axes
                lx = clamp(js.get_axis(LEFT_X_AXIS), -1.0, 1.0)
                ly = clamp(js.get_axis(LEFT_Y_AXIS), -1.0, 1.0)
                rx = clamp(js.get_axis(RIGHT_X_AXIS), -1.0, 1.0)
                ry = clamp(js.get_axis(RIGHT_Y_AXIS), -1.0, 1.0)
                l2 = clamp(js.get_axis(L2_AXIS), -1.0, 1.0)
                r2 = clamp(js.get_axis(R2_AXIS), -1.0, 1.0)

                # buttons + debounce (guard if idx out of range)
                now_ms = pygame.time.get_ticks()
                num_buttons = js.get_numbuttons()
                r1_raw = bool(js.get_button(R1_BUTTON)) if R1_BUTTON < num_buttons else False
                l1_raw = bool(js.get_button(L1_BUTTON)) if L1_BUTTON < num_buttons else False

                r1_state = self.r1_deb.update(r1_raw, now_ms)
                l1_state = self.l1_deb.update(l1_raw, now_ms)

                # Rising edges -> toggle direction
                # Compare debounced vs previous via attributes we store on first call
                if not hasattr(self, "_prev_r1"):
                    self._prev_r1 = r1_state
                    self._prev_l1 = l1_state
                if r1_state and not self._prev_r1:
                    self.heave_invert = not self.heave_invert
                    self._log_enqueue(f"Heave mode: {'INV (60→44)' if self.heave_invert else 'NOR (60→76)'}", "sys")
                if l1_state and not self._prev_l1:
                    self.yaw_invert = not self.yaw_invert
                    self._log_enqueue(f"Yaw mode: {'INV (60→44)' if self.yaw_invert else 'NOR (60→76)'}", "sys")
                self._prev_r1 = r1_state
                self._prev_l1 = l1_state

                # map sticks
                sway  = map_bidirectional(rx, STICK_DEADZONE, rest=STICK_REST,
                                          out_min=OUT_MIN, out_center=OUT_CENTER, out_max=OUT_MAX)
                surge = map_bidirectional(-ry, STICK_DEADZONE, rest=STICK_REST,
                                          out_min=OUT_MIN, out_center=OUT_CENTER, out_max=OUT_MAX)
                roll  = map_bidirectional(lx, STICK_DEADZONE, rest=STICK_REST,
                                          out_min=OUT_MIN, out_center=OUT_CENTER, out_max=OUT_MAX)
                pitch = map_bidirectional(-ly, STICK_DEADZONE, rest=STICK_REST,
                                          out_min=OUT_MIN, out_center=OUT_CENTER, out_max=OUT_MAX)

                # heave from R2 (one-direction, toggle around center)
                heave_normal = map_one_direction(r2, TRIGGER_DEADZONE, rest=TRIG_REST,
                                                 out_center=OUT_CENTER, out_max=OUT_MAX)
                heave = OUT_CENTER - (heave_normal - OUT_CENTER) if self.heave_invert else heave_normal

                # yaw from L2 (one-direction, toggle around center)
                yaw_normal = map_one_direction(l2, TRIGGER_DEADZONE, rest=TRIG_REST,
                                               out_center=OUT_CENTER, out_max=OUT_MAX)
                yaw = OUT_CENTER - (yaw_normal - OUT_CENTER) if self.yaw_invert else yaw_normal

                # publish
                self.roll  = int(round(roll))
                self.pitch = int(round(pitch))
                self.yaw   = int(round(yaw))
                self.surge = int(round(surge))
                self.sway  = int(round(sway))
                self.heave = int(round(heave))

                # send over TCP if connected
                if self.net.is_connected():
                    self.net.send_input_packet(self.roll, self.pitch, self.yaw,
                                               self.surge, self.sway, self.heave)

                time.sleep(dt)
        except Exception as e:
            self._log_enqueue(f"Joystick loop error: {e}", "err")
        finally:
            try:
                js.quit()
            except Exception:
                pass
            pygame.quit()

    # ---------------- Close ----------------
    def on_close(self):
        self.running = False
        self.on_disconnect()
        # Give threads a moment to stop
        try:
            time.sleep(0.2)
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    # High-DPI fix on Windows (optional)
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    App().mainloop()
