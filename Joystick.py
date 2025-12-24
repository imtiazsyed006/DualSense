#ESC1 RR_Top
#ESC2 FR_Top
#ESC3 FR_Bot
#ESC4 RL_Bot
#ESC5 RR_Bot
#ESC6 RL_Top
#ESC7 FL_Top
#ESC8 FL_Bot
# packages that need to be installed are ffmpeg openCV etc. For ffmpeg use chocolatey
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
import subprocess
from datetime import datetime

import numpy as np
import cv2
from PIL import Image, ImageTk

# =================== CONFIG ===================
LEFT_X_AXIS   = 0  # Left stick X  -> sway (-1..+1)
LEFT_Y_AXIS   = 1  # Left stick Y  -> heave (-1..+1, up is -1 so we invert)

RIGHT_X_AXIS  = 2  # Right stick X -> yaw  (-1..+1)
RIGHT_Y_AXIS  = 3  # Right stick Y -> pitch (-1..+1, forward is -1 so we invert)

L2_AXIS       = 4  # Left trigger  -> roll (0..+1 magnitude, sign via L1 toggle)
R2_AXIS       = 5  # Right trigger -> surge (0..+1 magnitude, sign via R1 toggle)

L1_BUTTON           = 9      # L1: flip roll direction (toggle)
R1_BUTTON           = 10     # R1: flip surge direction (toggle)
LIGHT_TOGGLE_BUTTON = 4

DPAD_UP_BUTTON    = 11
DPAD_DOWN_BUTTON  = 12

DEBUG_MAP = False

STICK_DEADZONE   = 0.08
TRIGGER_DEADZONE = 0.02
TRIG_REST  = -1.0

LOOP_HZ = 30

LIGHT_MIN = 4000
LIGHT_MAX = 7600
LIGHT_STEP_PER_FRAME = 1

DEFAULT_IP   = "192.168.0.25"
SEND_PORT    = 2001
RECV_PORT    = 2000

LOG_MAX_LINES = 2000

# -------- Camera receiver settings (Windows) --------
CAM_W, CAM_H = 640, 480
FFMPEG_PATH  = r"C:\ProgramData\chocolatey\bin\ffmpeg.exe"

# SDP relative to script folder
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAM_SDP_PATH = os.path.join(SCRIPT_DIR, "stream.sdp")

# No-frame detection
CAM_NO_FRAME_TIMEOUT_S = 1.5

# Rotate camera frame counter-clockwise
ROTATE_CCW = True

# Recording settings
RECORD_FPS = 30.0          # best to match sender framerate
RECORD_FOURCC = "mp4v"     # works well with .mp4 on Windows
# =============================================


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def deadzone(v, dz):
    return 0.0 if abs(v) < dz else v

def trigger_to_01(raw, rest=-1.0, dz=0.02):
    if abs(raw - rest) < dz or raw <= rest:
        return 0.0
    t = (raw - rest) / (1.0 - rest)
    return clamp(t, 0.0, 1.0)


class NetClient:
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
        if pygame.joystick.get_count() == 0:
            self.log("No joystick detected.", "err")
            return False
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
                        s = data.decode("utf-8", errors="replace").rstrip("\r\n")
                        self.log(f"RX: {s}", "rx")
                    except Exception:
                        self.log(f"RX (bin): {data.hex()}", "rx")
                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"RX error: {e}", "err")
                    break
        finally:
            self.disconnect()

    def send_input_packet_f32l(self, roll, pitch, yaw, surge, sway, heave,
                               light_intensity, light_enabled):
        if not self.tx_sock:
            return
        try:
            payload = struct.pack(
                "<6fI?",
                float(roll), float(pitch), float(yaw),
                float(surge), float(sway), float(heave),
                int(light_intensity),
                bool(light_enabled)
            )
            buf = b"inputF32L:" + payload
            with self._tx_lock:
                self.tx_sock.sendall(buf)

            self.log(
                (f"TX: roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f} "
                 f"surge={surge:+.3f} sway={sway:+.3f} heave={heave:+.3f} | "
                 f"light={light_intensity} ({'EN' if light_enabled else 'DIS'})"),
                "tx"
            )
        except Exception as e:
            self.log(f"TX error: {e}", "err")
            self.disconnect()


class FFmpegCameraReceiver:
    def __init__(self, log_fn):
        self.log = log_fn
        self.proc = None
        self.thread = None
        self.stop_evt = threading.Event()

        self._lock = threading.Lock()
        self._last_frame = None
        self._last_frame_time = 0.0

    @staticmethod
    def _read_exact(pipe, n: int):
        buf = bytearray()
        while len(buf) < n:
            chunk = pipe.read(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def start(self):
        if self.proc is not None:
            return

        if not os.path.exists(FFMPEG_PATH):
            self.log(f"[CAM] ffmpeg not found: {FFMPEG_PATH}", "err")
            return
        if not os.path.exists(CAM_SDP_PATH):
            self.log(f"[CAM] stream.sdp not found: {CAM_SDP_PATH}", "err")
            self.log("[CAM] Put stream.sdp next to this script OR change CAM_SDP_PATH.", "err")
            return

        cmd = [
            FFMPEG_PATH,
            "-hide_banner",
            "-loglevel", "warning",
            "-protocol_whitelist", "file,udp,rtp",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-probesize", "32",
            "-analyzeduration", "0",
            "-i", CAM_SDP_PATH,
            "-an",
            "-vsync", "0",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-"
        ]

        self.log(f"[CAM] Starting ffmpeg with SDP: {CAM_SDP_PATH}", "sys")

        try:
            self.stop_evt.clear()
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
        except Exception as e:
            self.proc = None
            self.log(f"[CAM] Failed to start FFmpeg: {e}", "err")
            return

        def _stderr_logger():
            try:
                for line in iter(self.proc.stderr.readline, b""):
                    if self.stop_evt.is_set():
                        break
                    s = line.decode(errors="ignore").strip()
                    if s:
                        self.log(f"[CAM][ffmpeg] {s}", "sys")
            except Exception:
                pass

        threading.Thread(target=_stderr_logger, daemon=True).start()

        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()
        self.log("[CAM] Receiver started", "sys")

    def stop(self):
        self.stop_evt.set()
        if self.proc:
            try:
                self.proc.terminate()
            except Exception:
                pass
            self.proc = None
        self.thread = None
        self.log("[CAM] Receiver stopped", "sys")

    def _rx_loop(self):
        frame_size = CAM_W * CAM_H * 3
        got_any = False
        try:
            while not self.stop_evt.is_set() and self.proc and self.proc.stdout:
                raw = self._read_exact(self.proc.stdout, frame_size)
                if raw is None:
                    break
                got_any = True
                frame = np.frombuffer(raw, np.uint8).reshape((CAM_H, CAM_W, 3))
                with self._lock:
                    self._last_frame = frame
                    self._last_frame_time = time.time()
        finally:
            rc = None
            try:
                if self.proc:
                    rc = self.proc.poll()
            except Exception:
                pass

            if not got_any:
                self.log(f"[CAM] No frames received. ffmpeg returncode={rc}", "err")
                self.log("[CAM] Check stream is running + port in SDP matches sender.", "err")

            if not self.stop_evt.is_set():
                self.stop()

    def get_latest_frame(self):
        with self._lock:
            if self._last_frame is None:
                return None, 0.0
            return self._last_frame.copy(), self._last_frame_time


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PS5 → STM32 (TCP) Controller (float32 + Light)")
        self.geometry("1000x620")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=8)

        ttk.Label(top, text="STM32 IP:").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(top, textvariable=self.ip_var, width=18).pack(side="left", padx=(4, 10))

        self.btn_connect = ttk.Button(top, text="Connect", command=self.on_connect)
        self.btn_connect.pack(side="left", padx=4)

        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)

        self.led_canvas = tk.Canvas(top, width=18, height=18, highlightthickness=0)
        self.led_canvas.pack(side="left", padx=10)
        self.led_id = self.led_canvas.create_oval(2, 2, 16, 16, fill="#aa2222", outline="black")

        ttk.Label(top, text=f"Loop: {LOOP_HZ} Hz").pack(side="right")

        self.cam_status_var = tk.StringVar(value="CAM: STOPPED")
        self.cam_status_lbl = ttk.Label(top, textvariable=self.cam_status_var)
        self.cam_status_lbl.pack(side="right", padx=(0, 10))
        self._set_cam_status(False, stopped=True)

        self.rec_status_var = tk.StringVar(value="REC: OFF")
        self.rec_status_lbl = ttk.Label(top, textvariable=self.rec_status_var)
        self.rec_status_lbl.pack(side="right", padx=(0, 12))
        self._set_rec_status(False)

        middle = ttk.Frame(self)
        middle.pack(fill="x", padx=8, pady=(0, 8))

        light_frame = ttk.LabelFrame(middle, text="Light")
        light_frame.pack(side="left", padx=8, pady=4, ipadx=8, ipady=8)

        self.light_canvas = tk.Canvas(light_frame, width=140, height=140, bg="white", highlightthickness=0)
        self.light_canvas.grid(row=0, column=0, rowspan=3, padx=8, pady=8)

        self.light_outer = self.light_canvas.create_oval(10, 10, 130, 130, fill="#303030", outline="#202020", width=2)
        self.light_inner = self.light_canvas.create_oval(30, 30, 110, 110, fill="#202020", outline="")

        self.light_lbl = ttk.Label(light_frame, text="Intensity: 6000")
        self.light_state = ttk.Label(light_frame, text="State: Disabled")
        self.light_hint = ttk.Label(light_frame, text="Toggle: LIGHT btn\nAdjust: D-pad Up/Down", foreground="gray")

        self.light_lbl.grid(row=0, column=1, sticky="w", padx=(6, 4))
        self.light_state.grid(row=1, column=1, sticky="w", padx=(6, 4))
        self.light_hint.grid(row=2, column=1, sticky="w", padx=(6, 4))

        tabs_frame = ttk.Frame(self)
        tabs_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self.notebook = ttk.Notebook(tabs_frame)
        self.notebook.pack(fill="both", expand=True)

        # Log tab
        self.tab_log = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_log, text="Log")

        log_frame = ttk.Frame(self.tab_log)
        log_frame.pack(fill="both", expand=True)

        self.log_text = tk.Text(log_frame, wrap="none", height=18)
        self.log_text.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        self.log_text.tag_config("tx", foreground="blue")
        self.log_text.tag_config("rx", foreground="green")
        self.log_text.tag_config("sys", foreground="gray")
        self.log_text.tag_config("err", foreground="red")

        # Camera tab
        self.tab_cam = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_cam, text="Camera")

        cam_controls = ttk.Frame(self.tab_cam)
        cam_controls.pack(fill="x", padx=6, pady=6)

        self.btn_cam_start = ttk.Button(cam_controls, text="Start Camera", command=self.on_cam_start)
        self.btn_cam_start.pack(side="left", padx=(0, 6))

        self.btn_cam_stop = ttk.Button(cam_controls, text="Stop Camera", command=self.on_cam_stop, state="disabled")
        self.btn_cam_stop.pack(side="left", padx=(0, 10))

        self.btn_rec = ttk.Button(cam_controls, text="● Record", command=self.on_rec_toggle, state="disabled")
        self.btn_rec.pack(side="left")

        self.cam_canvas = tk.Label(self.tab_cam, text="Camera stopped", anchor="center")
        self.cam_canvas.pack(fill="both", expand=True, padx=6, pady=6)

        self._cam_imgtk = None

        # State
        self.msg_q = queue.Queue()
        self.net = NetClient(self._log_enqueue, self._set_led)
        self.running = True

        self.surge_invert = False
        self.roll_invert = False
        self._prev_r1_raw = False
        self._prev_l1_raw = False

        self.light_enabled = False
        self.light_intensity = LIGHT_MIN
        self._prev_light_raw = False

        self._prev_btn_raw = []
        self._last_sent = {
            "roll": None, "pitch": None, "yaw": None,
            "surge": None, "sway": None, "heave": None,
            "light_intensity": None, "light_enabled": None
        }

        self.cam = FFmpegCameraReceiver(self._log_enqueue)
        self._cam_running = False

        # Recording
        self._recording = False
        self._writer = None
        self._record_path = None

        self._joy_thread = threading.Thread(target=self._joystick_loop, daemon=True)
        self._joy_thread.start()

        self.after(30, self._drain_log_queue)
        self.after(50, self._animate_light)
        self.after(50, self._camera_ui_tick)

    # ---------- UI helpers ----------
    def _set_led(self, on: bool):
        color = "#22aa22" if on else "#aa2222"
        def do():
            self.led_canvas.itemconfig(self.led_id, fill=color)
            self.btn_connect.config(state=("disabled" if on else "normal"))
            self.btn_disconnect.config(state=("normal" if on else "disabled"))
        self.after(0, do)

    def _set_cam_status(self, ok: bool, stopped: bool = False):
        if stopped:
            self.cam_status_var.set("CAM: STOPPED")
            self.cam_status_lbl.configure(foreground="gray")
        else:
            if ok:
                self.cam_status_var.set("CAM: OK")
                self.cam_status_lbl.configure(foreground="green")
            else:
                self.cam_status_var.set("CAM: NO FEED")
                self.cam_status_lbl.configure(foreground="red")

    def _set_rec_status(self, on: bool):
        if on:
            self.rec_status_var.set("REC: ON")
            self.rec_status_lbl.configure(foreground="red")
        else:
            self.rec_status_var.set("REC: OFF")
            self.rec_status_lbl.configure(foreground="gray")

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
        self.log_text.insert("end", text + "\n", tag)
        self.log_text.see("end")
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            cut = max(1, LOG_MAX_LINES // 10)
            self.log_text.delete("1.0", f"{cut}.0")

    def _animate_light(self):
        en = self.light_enabled
        I = self.light_intensity
        bright = 0.15 if not en else (0.15 + 0.85 * ((I - LIGHT_MIN) / (LIGHT_MAX - LIGHT_MIN)))
        bright = clamp(bright, 0.0, 1.0)

        gamma = 2.2
        gg = pow(bright, 1 / gamma)
        r = int(40 + 215 * gg)
        gcol = int(35 + 200 * gg)
        b = int(10 + 30 * gg)

        fill_inner = f"#{r:02x}{gcol:02x}{b:02x}"
        fill_outer = f"#{max(0, r-40):02x}{max(0, gcol-40):02x}{max(0, b-40):02x}"

        self.light_canvas.itemconfig(self.light_inner, fill=fill_inner)
        self.light_canvas.itemconfig(self.light_outer, fill=fill_outer)
        self.light_lbl.config(text=f"Intensity: {self.light_intensity}")
        self.light_state.config(text=f"State: {'Enabled' if self.light_enabled else 'Disabled'}")
        self.after(80, self._animate_light)

    # ---------- Camera & Recording ----------
    def on_cam_start(self):
        if self._cam_running:
            return
        self._log_enqueue("[CAM] Start requested", "sys")
        self.cam.start()
        self._cam_running = True
        self.btn_cam_start.config(state="disabled")
        self.btn_cam_stop.config(state="normal")
        self.btn_rec.config(state="normal")
        self._set_cam_status(False, stopped=False)
        self.cam_canvas.config(text="Waiting for video...", image="")
        self._cam_imgtk = None

    def on_cam_stop(self):
        if not self._cam_running:
            return
        # Stop recording if running
        if self._recording:
            self._stop_recording()

        self._log_enqueue("[CAM] Stop requested", "sys")
        self.cam.stop()
        self._cam_running = False
        self.btn_cam_start.config(state="normal")
        self.btn_cam_stop.config(state="disabled")
        self.btn_rec.config(state="disabled")
        self.cam_canvas.config(image="", text="Camera stopped")
        self._cam_imgtk = None
        self._set_cam_status(False, stopped=True)

    def _start_recording(self):
        # Save in current working directory with date/time
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        fname = f"recording_{ts}.mp4"
        out_path = os.path.join(os.getcwd(), fname)

        # After rotation CCW, dimensions swap
        out_w, out_h = (CAM_H, CAM_W) if ROTATE_CCW else (CAM_W, CAM_H)

        fourcc = cv2.VideoWriter_fourcc(*RECORD_FOURCC)
        writer = cv2.VideoWriter(out_path, fourcc, RECORD_FPS, (out_w, out_h))

        if not writer.isOpened():
            self._log_enqueue(f"[REC] Failed to open writer for: {out_path}", "err")
            return

        self._writer = writer
        self._record_path = out_path
        self._recording = True
        self._set_rec_status(True)
        self.btn_rec.config(text="■ Stop", state="normal")
        self._log_enqueue(f"[REC] Recording started: {out_path}", "sys")

    def _stop_recording(self):
        try:
            if self._writer:
                self._writer.release()
        except Exception:
            pass
        path = self._record_path
        self._writer = None
        self._record_path = None
        self._recording = False
        self._set_rec_status(False)
        self.btn_rec.config(text="● Record")
        if path:
            self._log_enqueue(f"[REC] Recording saved: {path}", "sys")

    def on_rec_toggle(self):
        if not self._cam_running:
            self._log_enqueue("[REC] Camera not running", "err")
            return
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _camera_ui_tick(self):
        try:
            if self._cam_running:
                frame, t_frame = self.cam.get_latest_frame()
                now = time.time()

                ok = (frame is not None) and ((now - t_frame) < CAM_NO_FRAME_TIMEOUT_S)
                self._set_cam_status(ok, stopped=False)

                if frame is not None:
                    # Rotate CCW if requested
                    if ROTATE_CCW:
                        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                    # Write to file if recording
                    if self._recording and self._writer is not None:
                        try:
                            self._writer.write(frame)
                        except Exception as e:
                            self._log_enqueue(f"[REC] Write error: {e}", "err")
                            self._stop_recording()

                    # Display in Tkinter
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(rgb)

                    w = self.cam_canvas.winfo_width()
                    h = self.cam_canvas.winfo_height()
                    if w >= 50 and h >= 50:
                        img = img.resize((w, h))

                    self._cam_imgtk = ImageTk.PhotoImage(img)
                    self.cam_canvas.config(image=self._cam_imgtk, text="")
                else:
                    self.cam_canvas.config(image="", text="Waiting for video...")
                    self._cam_imgtk = None

                if not ok:
                    self.cam_canvas.config(text="NO FEED (check Log tab for ffmpeg errors)")
        finally:
            self.after(50, self._camera_ui_tick)

    # ---------- Connect / Disconnect ----------
    def on_connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            self._log_enqueue("No IP entered", "err")
            return
        if pygame.joystick.get_count() != 0:
            self._log_enqueue(f"Connecting to {ip}...", "sys")
        else:
            self._log_enqueue("Please connect joystick first.", "sys")
        threading.Thread(target=lambda: self.net.connect(ip), daemon=True).start()

    def on_disconnect(self):
        self._log_enqueue("Disconnecting…", "sys")
        threading.Thread(target=self.net.disconnect, daemon=True).start()

    # ---------- Joystick loop ----------
    def _joystick_loop(self):
        pygame.init()
        if pygame.joystick.get_count() == 0:
            self._log_enqueue("No joystick detected. Plug in your controller and restart.", "err")
            return

        js = pygame.joystick.Joystick(0)
        js.init()

        self._log_enqueue(
            f"Controller: {js.get_name()} (axes={js.get_numaxes()}, buttons={js.get_numbuttons()}, hats={js.get_numhats()})",
            "sys"
        )

        nbtn = js.get_numbuttons()
        self._prev_btn_raw = [False] * nbtn
        dt = 1.0 / LOOP_HZ

        try:
            while self.running:
                pygame.event.pump()

                nbtn = js.get_numbuttons()
                l1_raw = bool(js.get_button(L1_BUTTON)) if L1_BUTTON < nbtn else False
                r1_raw = bool(js.get_button(R1_BUTTON)) if R1_BUTTON < nbtn else False
                light_raw = bool(js.get_button(LIGHT_TOGGLE_BUTTON)) if LIGHT_TOGGLE_BUTTON < nbtn else False

                if DEBUG_MAP:
                    for i in range(nbtn):
                        cur = bool(js.get_button(i))
                        if cur != self._prev_btn_raw[i]:
                            self._log_enqueue(f"[MAP] Btn#{i} -> {'DOWN' if cur else 'UP'}", "sys")
                        self._prev_btn_raw[i] = cur

                if l1_raw and not self._prev_l1_raw:
                    self.roll_invert = not self.roll_invert
                    self._log_enqueue(f"[BTN] L1 → Roll dir = {'NEG(−)' if self.roll_invert else 'POS(+)' }", "sys")
                self._prev_l1_raw = l1_raw

                if r1_raw and not self._prev_r1_raw:
                    self.surge_invert = not self.surge_invert
                    self._log_enqueue(f"[BTN] R1 → Surge dir = {'NEG(−)' if self.surge_invert else 'POS(+)' }", "sys")
                self._prev_r1_raw = r1_raw

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
                self._prev_light_raw = light_raw

                light_dir = 0
                hat_y = 0
                if js.get_numhats() > 0:
                    _, hat_y = js.get_hat(0)
                if hat_y != 0:
                    light_dir = 1 if hat_y > 0 else -1
                else:
                    up_raw = (DPAD_UP_BUTTON is not None and DPAD_UP_BUTTON < nbtn and bool(js.get_button(DPAD_UP_BUTTON)))
                    dn_raw = (DPAD_DOWN_BUTTON is not None and DPAD_DOWN_BUTTON < nbtn and bool(js.get_button(DPAD_DOWN_BUTTON)))
                    if up_raw and not dn_raw:
                        light_dir = +10
                    elif dn_raw and not up_raw:
                        light_dir = -10

                lx = clamp(js.get_axis(LEFT_X_AXIS),  -1.0, 1.0)
                ly = clamp(js.get_axis(LEFT_Y_AXIS),  -1.0, 1.0)
                rx = clamp(js.get_axis(RIGHT_X_AXIS), -1.0, 1.0)
                ry = clamp(js.get_axis(RIGHT_Y_AXIS), -1.0, 1.0)
                l2 = clamp(js.get_axis(L2_AXIS),      -1.0, 1.0)
                r2 = clamp(js.get_axis(R2_AXIS),      -1.0, 1.0)

                pitch = deadzone(-ry, STICK_DEADZONE)
                yaw   = deadzone(rx,  STICK_DEADZONE)

                heave = deadzone(-ly, STICK_DEADZONE)
                sway  = deadzone(lx,  STICK_DEADZONE)

                surge_mag = trigger_to_01(r2, rest=TRIG_REST, dz=TRIGGER_DEADZONE)
                surge = (-surge_mag) if self.surge_invert else (+surge_mag)

                roll_mag = trigger_to_01(l2, rest=TRIG_REST, dz=TRIGGER_DEADZONE)
                roll = (-roll_mag) if self.roll_invert else (+roll_mag)

                if light_dir != 0:
                    old = self.light_intensity
                    self.light_intensity = clamp(self.light_intensity + (LIGHT_STEP_PER_FRAME * light_dir),
                                                 LIGHT_MIN, LIGHT_MAX)
                    if self.light_intensity != old:
                        self._log_enqueue(f"[LIGHT] intensity→{self.light_intensity}", "sys")

                def changed(name, new, eps=None):
                    oldv = self._last_sent[name]
                    if oldv is None:
                        return True
                    if eps is None:
                        return oldv != new
                    return abs(new - oldv) > eps

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
        try:
            if self._recording:
                self._stop_recording()
        except Exception:
            pass
        try:
            self.on_cam_stop()
        except Exception:
            pass
        self.on_disconnect()
        try:
            time.sleep(0.2)
        except:
            pass
        self.destroy()


if __name__ == "__main__":
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    App().mainloop()
