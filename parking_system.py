"""Smart parking GUI: single camera + YOLO character detector + ESP32 RFID serial protocol.

ESP32 protocol expected (115200 baud):
  ESP32 -> PC: CARD,<UID>,<free_slots>
  ESP32 -> PC: GATE_CLOSED,<UID>,<ENTRY|EXIT>
  PC -> ESP32: OPEN,ENTRY | OPEN,EXIT | DENY,<reason>

Install dependencies:
  pip install opencv-python pillow pyserial ultralytics

Run example:
  python parking_system_gui.py --port COM3 --source 1
"""

from __future__ import annotations

import argparse
import json
import queue
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk, ImageDraw, ImageFont

try:
    import serial
except ImportError:
    serial = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


APP_DIR = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = APP_DIR / "runs" / "detect" / "bien_so_yolov8n" / "weights" / "best.pt"
DB_PATH = APP_DIR / "parking_records.json"
CARD_REGISTRY_PATH = APP_DIR / "card_plate_registry.json"
CAPTURE_DIR = APP_DIR / "captures"

# Palette inspired by the reference screenshot
BLUE = "#087FE8"
BLUE_DARK = "#073A78"
BLUE_DEEP = "#102C50"
BG = "#EAF4FF"
PANEL = "#F5FAFF"
BORDER = "#A9D2FF"
GREEN = "#10A638"
GREEN_DARK = "#07852B"
RED = "#F4434C"
ORANGE = "#FFA500"
TEXT = "#0B3268"
MUTED = "#55708E"
WHITE = "#FFFFFF"

VALID_CHARS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")


def normalize_plate(value: str) -> str:
    return re.sub(r"[^A-Z0-9]", "", value.upper())


def character_from_class(name: str) -> str | None:
    """Map a YOLO class name to one alphanumeric character."""
    label = str(name).strip().upper()
    if label in VALID_CHARS and len(label) == 1:
        return label
    # Common custom dataset styles: char_A, character-5, class_8
    match = re.search(r"(?:CHARACTER|CHAR|CLASS)[ _-]*([A-Z0-9])$", label)
    if match:
        return match.group(1)
    return None


def sort_character_detections(detections: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    """Group detected characters into text rows, then sort each row from left to right."""
    if not detections:
        return []
    detections = sorted(detections, key=lambda d: (d["cy"], d["cx"]))
    heights = sorted(max(1.0, d["y2"] - d["y1"]) for d in detections)
    median_h = heights[len(heights) // 2]
    row_tolerance = max(8.0, median_h * 0.62)
    rows: list[list[dict[str, Any]]] = []
    row_centers: list[float] = []
    for det in detections:
        best_idx = None
        best_dist = None
        for idx, center_y in enumerate(row_centers):
            dist = abs(det["cy"] - center_y)
            if dist <= row_tolerance and (best_dist is None or dist < best_dist):
                best_idx, best_dist = idx, dist
        if best_idx is None:
            rows.append([det])
            row_centers.append(det["cy"])
        else:
            rows[best_idx].append(det)
            row_centers[best_idx] = sum(d["cy"] for d in rows[best_idx]) / len(rows[best_idx])
    rows = sorted(rows, key=lambda row: sum(d["cy"] for d in row) / len(row))
    for row in rows:
        row.sort(key=lambda d: d["cx"])
    return rows


def format_plate(raw: str) -> str:
    """Format common Vietnamese plate pattern 12B28434 as 12B 284.34."""
    value = normalize_plate(raw)
    match = re.fullmatch(r"(\d{2}[A-Z]{1,2})(\d{5})", value)
    if match:
        return f"{match.group(1)} {match.group(2)[:3]}.{match.group(2)[3:]}"
    return value


class ParkingApp:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.root = tk.Tk()
        self.root.title("Hệ Thống Quản Lý Bãi Đỗ Xe")
        # Open maximized so the table and controls are not clipped by the Windows taskbar.
        self.root.geometry("1280x800")
        self.root.minsize(1000, 620)
        try:
            self.root.state("zoomed")
        except tk.TclError:
            pass
        self.root.configure(bg=BG)

        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.stop_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.model_lock = threading.Lock()
        self.latest_frame = None
        self.live_annotated_frame = None
        self.live_plate_norm = ""
        self.live_plate_display = ""
        self.live_detection_busy = False
        self.scan_in_progress = False
        self.cap = None
        self.ser = None
        self.model = None
        self.model_error = ""
        self.camera_error = ""
        self.serial_error = ""
        # Chỉ giữ liên kết UID-biển số của xe đang ở trong bãi.
        # Khi xe ra thành công, liên kết được xóa để thẻ có thể ghép với xe khác.
        self.parked: dict[str, dict[str, str]] = self.load_records()
        saved_registry = self.load_card_registry()
        self.card_registry: dict[str, dict[str, str]] = {
            uid: saved_registry.get(uid, {}) for uid in self.parked
        }
        for uid, record in self.parked.items():
            self.card_registry[uid] = {
                "plate": record.get("plate", ""),
                "display": record.get("display", format_plate(record.get("plate", ""))),
                "last_entry_time": record.get("entry_time", "---"),
            }
        self.save_card_registry()
        self.pending_actions: dict[str, dict[str, str]] = {}
        self.pending_uids: set[str] = set()
        self.total_slots = 3
        # Until the first SLOTS packet arrives, use saved records only as a temporary estimate.
        # After that, physical occupancy reported by the ESP32 sensors is authoritative.
        self.sensor_status_received = False
        self.initial_sensor_sync_done = False
        self.free_slots = max(0, self.total_slots - len(self.parked))
        self.occupied_slots = self.total_slots - self.free_slots
        self.current_uid = "---"
        self.current_plate = "---"
        self.current_action = "---"
        self.entry_time = "---"
        self.exit_time = "---"
        self.status_text = "SẴN SÀNG QUÉT THẺ"
        self.status_color = GREEN_DARK
        self.selected_mode = "AUTO"
        self.auto_mode = True
        self.current_annotated = None
        self.last_detected_norm = ""
        self.last_detected_display = ""
        self.last_recognition_at = 0.0

        self._build_ui()
        self._load_model()
        self._open_camera()
        self._open_serial()
        self._update_clock()
        self._update_camera()
        self.root.after(120, self._schedule_live_detection)
        self.root.after(100, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ---------- Persistence and device setup ----------
    def load_records(self) -> dict[str, dict[str, str]]:
        try:
            if DB_PATH.exists():
                data = json.loads(DB_PATH.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    return data
        except Exception as exc:
            print(f"Cannot load parking records: {exc}")
        return {}

    def save_records(self) -> None:
        try:
            tmp = DB_PATH.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.parked, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(DB_PATH)
        except Exception as exc:
            print(f"Cannot save parking records: {exc}")

    def load_card_registry(self) -> dict[str, dict[str, str]]:
        try:
            if CARD_REGISTRY_PATH.exists():
                data = json.loads(CARD_REGISTRY_PATH.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    return data
        except Exception as exc:
            print(f"Cannot load card/plate registry: {exc}")
        return {}

    def save_card_registry(self) -> None:
        try:
            tmp = CARD_REGISTRY_PATH.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.card_registry, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(CARD_REGISTRY_PATH)
        except Exception as exc:
            print(f"Cannot save card/plate registry: {exc}")

    def _load_model(self) -> None:
        if YOLO is None:
            self.model_error = "Chưa cài ultralytics"
            self._set_status("THIẾU THƯ VIỆN YOLO", RED)
            return
        weights = Path(self.args.weights).expanduser()
        if not weights.is_absolute():
            weights = (Path.cwd() / weights).resolve()
            # Prefer script-relative default if the command-line default was used.
            if self.args.weights == str(DEFAULT_WEIGHTS) and not weights.exists() and DEFAULT_WEIGHTS.exists():
                weights = DEFAULT_WEIGHTS
        if not weights.exists():
            self.model_error = f"Không tìm thấy model: {weights}"
            self._set_status("CHƯA TÌM THẤY best.pt", RED)
            print(self.model_error)
            return
        try:
            self.model = YOLO(str(weights))
            self._set_status("YOLO ĐÃ SẴN SÀNG", GREEN_DARK)
        except Exception as exc:
            self.model_error = str(exc)
            self._set_status("LỖI NẠP MODEL", RED)
            print(f"Model load error: {exc}")

    def _open_camera(self) -> None:
        source: Any = self.args.source
        try:
            if str(source).isdigit():
                source = int(source)
            if self.args.backend == "dshow" and hasattr(cv2, "CAP_DSHOW"):
                self.cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
            elif self.args.backend == "msmf" and hasattr(cv2, "CAP_MSMF"):
                self.cap = cv2.VideoCapture(source, cv2.CAP_MSMF)
            else:
                self.cap = cv2.VideoCapture(source)
            if not self.cap or not self.cap.isOpened():
                self.camera_error = f"Không mở được camera {source}"
                if self.cap:
                    self.cap.release()
                self.cap = None
                print(self.camera_error)
        except Exception as exc:
            self.camera_error = str(exc)
            self.cap = None
            print(f"Camera error: {exc}")

    def _open_serial(self) -> None:
        if serial is None:
            self.serial_error = "Chưa cài pyserial"
            self._set_status("THIẾU THƯ VIỆN SERIAL", RED)
            return
        try:
            self.ser = serial.Serial(self.args.port, self.args.baud, timeout=0.25)
            # ESP32 boards often reset when the port opens.
            time.sleep(1.8)
            # Do NOT clear the input buffer here: ESP32 sends its first SLOTS,<free>,<occupied>
            # packet during boot. Clearing the buffer would discard the only startup status
            # because the ESP32 sketch emits SLOTS only when its sensor count changes.
            threading.Thread(target=self._serial_reader, daemon=True).start()
            self._set_status(f"ĐÃ KẾT NỐI ESP32 • {self.args.port}", GREEN_DARK)
        except Exception as exc:
            self.serial_error = str(exc)
            self.ser = None
            self._set_status("CHƯA KẾT NỐI ESP32", RED)
            print(f"Serial connection error: {exc}")

    def _serial_reader(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.ser is None:
                    time.sleep(0.2)
                    continue
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.events.put(("serial", line))
            except Exception as exc:
                self.events.put(("serial_error", str(exc)))
                time.sleep(0.4)

    def _send(self, text: str) -> bool:
        try:
            if self.ser is None or not self.ser.is_open:
                self.events.put(("log", f"Không gửi được '{text}': chưa kết nối ESP32"))
                return False
            with self.serial_lock:
                self.ser.write((text + "\n").encode("utf-8"))
                self.ser.flush()
            self.events.put(("log", f"PC → ESP32: {text}"))
            return True
        except Exception as exc:
            self.events.put(("log", f"Lỗi gửi Serial: {exc}"))
            return False

    # ---------- UI ----------
    def _build_ui(self) -> None:
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

        titlebar = tk.Frame(self.root, bg=BLUE, height=38)
        titlebar.grid(row=0, column=0, sticky="ew")
        titlebar.grid_propagate(False)
        tk.Label(titlebar, text="▣  Hệ Thống Quản Lý Bãi Đỗ Xe", font=("Segoe UI", 11, "bold"),
                 bg=BLUE, fg=WHITE).pack(side="left", padx=16, pady=5)
        self.top_status = tk.Label(titlebar, text="ĐANG KHỞI ĐỘNG", font=("Segoe UI", 9, "bold"),
                                   bg=BLUE, fg=WHITE)
        self.top_status.pack(side="right", padx=18)

        content = tk.Frame(self.root, bg=BG, padx=12, pady=10)
        content.grid(row=1, column=0, sticky="nsew")
        content.grid_columnconfigure(0, weight=11, uniform="main")
        content.grid_columnconfigure(1, weight=9, uniform="main")
        content.grid_rowconfigure(1, weight=1)

        header = tk.Frame(content, bg=BLUE, height=66, highlightbackground=BORDER, highlightthickness=1)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        header.grid_propagate(False)
        tk.Label(header, text="BÃI ĐỖ XE THÔNG MINH", font=("Segoe UI", 24, "bold"),
                 bg=BLUE, fg=WHITE).pack(expand=True)

        left = tk.Frame(content, bg=PANEL, highlightbackground=BORDER, highlightthickness=1, padx=10, pady=10)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        left.grid_rowconfigure(1, weight=1)
        left.grid_columnconfigure(0, weight=1)
        tk.Label(left, text="📷  CAMERA NHẬN DIỆN", anchor="w", font=("Segoe UI", 15, "bold"),
                 bg=PANEL, fg=TEXT).grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.camera_label = tk.Label(left, bg="#13253E", fg=WHITE, text="Đang mở camera...",
                                     font=("Segoe UI", 15), relief="flat", anchor="center")
        self.camera_label.grid(row=1, column=0, sticky="nsew")
        result_panel = tk.Frame(left, bg="#DCEEFF", highlightbackground=BORDER, highlightthickness=1, padx=8, pady=6)
        result_panel.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        tk.Label(result_panel, text="KẾT QUẢ NHẬN DIỆN BIỂN SỐ", font=("Segoe UI", 11, "bold"),
                 bg="#DCEEFF", fg=TEXT).pack(fill="x", pady=(0, 5))
        self.plate_result = tk.Label(result_panel, text="---", font=("Segoe UI", 27, "bold"),
                                     bg=BLUE_DEEP, fg=WHITE, padx=8, pady=5, anchor="center")
        self.plate_result.pack(fill="x")

        right = tk.Frame(content, bg=PANEL, highlightbackground=BORDER, highlightthickness=1, padx=9, pady=9)
        right.grid(row=1, column=1, sticky="nsew", padx=(8, 0))
        right.grid_columnconfigure((0, 1, 2), weight=1, uniform="metrics")
        # The vehicle table receives most of the spare height in the right panel.
        right.grid_rowconfigure(5, weight=4, minsize=165)
        self.metric_values: dict[str, tk.Label] = {}
        metrics = [("CÒN LẠI", "free", GREEN_DARK), ("ĐÃ ĐỖ", "occupied", TEXT), ("TỔNG", "total", TEXT)]
        for col, (label, key, color) in enumerate(metrics):
            card = tk.Frame(right, bg="#E9F3FF", highlightbackground=BORDER, highlightthickness=1, padx=4, pady=3)
            card.grid(row=0, column=col, sticky="nsew", padx=3, pady=(0, 8))
            tk.Label(card, text=label, font=("Segoe UI", 10, "bold"), bg="#E9F3FF", fg=TEXT).pack()
            value = tk.Label(card, text="0", font=("Segoe UI", 22, "bold"), bg="#E9F3FF", fg=color)
            value.pack()
            self.metric_values[key] = value

        slots = tk.Frame(right, bg="#E9F3FF", highlightbackground=BORDER, highlightthickness=1, padx=5, pady=3)
        slots.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self.slot_lamps: list[tk.Label] = []
        for idx in range(3):
            cell = tk.Frame(slots, bg="#E9F3FF")
            cell.grid(row=0, column=idx, sticky="ew", padx=5)
            slots.grid_columnconfigure(idx, weight=1)
            tk.Label(cell, text=f"BÃI {idx + 1}", font=("Segoe UI", 10, "bold"), bg="#E9F3FF", fg=TEXT).pack()
            lamp = tk.Label(cell, text="TRỐNG", font=("Segoe UI", 9, "bold"), bg=GREEN, fg=WHITE,
                            padx=10, pady=3, relief="flat")
            lamp.pack(fill="x", pady=(2, 0))
            self.slot_lamps.append(lamp)

        self.availability_banner = tk.Label(right, text="BÃI ĐỖ CÒN TRỐNG", font=("Segoe UI", 12, "bold"),
                                            bg=GREEN, fg=WHITE, pady=5)
        self.availability_banner.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(0, 8))

        clockrow = tk.Frame(right, bg="#E9F3FF", highlightbackground=BORDER, highlightthickness=1, padx=6, pady=3)
        clockrow.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self.date_label = tk.Label(clockrow, text="📅 --/--/----", font=("Segoe UI", 9, "bold"), bg="#E9F3FF", fg=TEXT)
        self.date_label.pack(side="left", padx=6)
        self.time_label = tk.Label(clockrow, text="◷ --:--:--", font=("Segoe UI", 9, "bold"), bg="#E9F3FF", fg=TEXT)
        self.time_label.pack(side="right", padx=6)

        info = tk.Frame(right, bg="#E9F3FF", highlightbackground=BORDER, highlightthickness=1, padx=6, pady=3)
        info.grid(row=4, column=0, columnspan=3, sticky="nsew")
        info.grid_columnconfigure(1, weight=1)
        fields = [
            ("UID THẺ RFID", "uid"),
            ("BIỂN SỐ XE", "plate"),
            ("LOẠI XE / THAO TÁC", "action"),
            ("THỜI GIAN VÀO", "entry"),
            ("THỜI GIAN RA", "exit"),
            ("TRẠNG THÁI", "status"),
        ]
        self.info_values: dict[str, tk.Label] = {}
        for row, (label, key) in enumerate(fields):
            tk.Label(info, text=label, anchor="w", font=("Segoe UI", 9, "bold"), bg="#E9F3FF", fg=TEXT).grid(
                row=row, column=0, sticky="w", padx=(2, 6), pady=1)
            value = tk.Label(info, text="---", anchor="w", font=("Segoe UI", 9, "bold"), bg="#D7EAFE", fg=TEXT,
                             padx=7, pady=2, relief="flat")
            value.grid(row=row, column=1, sticky="ew", pady=1)
            self.info_values[key] = value
        self.info_values["status"].configure(fg=GREEN_DARK)

        # Danh sách liên kết thẻ RFID với biển số đã nhận diện.
        list_panel = tk.Frame(right, bg="#E9F3FF", highlightbackground=BORDER, highlightthickness=1, padx=6, pady=4)
        list_panel.grid(row=5, column=0, columnspan=3, sticky="nsew", pady=(5, 0))
        # Luôn cấp khoảng trống thực cho bảng; bảng nở theo chiều cao cửa sổ.
        list_panel.grid_rowconfigure(1, weight=1, minsize=140)
        list_panel.grid_columnconfigure(0, weight=1)
        list_header = tk.Frame(list_panel, bg="#E9F3FF")
        list_header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        tk.Label(list_header, text="DANH SÁCH THẺ ↔ BIỂN SỐ ĐANG ĐỖ",
                 font=("Segoe UI", 10, "bold"), bg="#E9F3FF", fg=TEXT).pack(side="left")
        self.registry_count = tk.Label(list_header, text="0 thẻ", font=("Segoe UI", 9, "bold"),
                                       bg="#E9F3FF", fg=MUTED)
        self.registry_count.pack(side="right")

        table_frame = tk.Frame(list_panel, bg="#E9F3FF")
        table_frame.grid(row=1, column=0, sticky="nsew")
        table_frame.grid_rowconfigure(0, weight=1, minsize=115)
        table_frame.grid_columnconfigure(0, weight=1)
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Parking.Treeview", font=("Segoe UI", 9), rowheight=22,
                        background=WHITE, fieldbackground=WHITE, foreground=TEXT)
        style.configure("Parking.Treeview.Heading", font=("Segoe UI", 9, "bold"),
                        background="#D7EAFE", foreground=TEXT)
        self.vehicle_tree = ttk.Treeview(
            table_frame, columns=("uid", "plate", "status"), show="headings",
            height=5, style="Parking.Treeview"
        )
        self.vehicle_tree.heading("uid", text="ID THẺ RFID")
        self.vehicle_tree.heading("plate", text="BIỂN SỐ XE")
        self.vehicle_tree.heading("status", text="TRẠNG THÁI")
        self.vehicle_tree.column("uid", width=145, minwidth=110, anchor="center")
        self.vehicle_tree.column("plate", width=120, minwidth=100, anchor="center")
        self.vehicle_tree.column("status", width=100, minwidth=85, anchor="center")
        tree_scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.vehicle_tree.yview)
        self.vehicle_tree.configure(yscrollcommand=tree_scroll.set)
        self.vehicle_tree.grid(row=0, column=0, sticky="nsew")
        tree_scroll.grid(row=0, column=1, sticky="ns")
        # Lăn chuột trực tiếp trên bảng để xem các dòng ngoài khung.
        self.vehicle_tree.bind("<MouseWheel>", self._on_vehicle_list_mousewheel)  # Windows
        self.vehicle_tree.bind("<Button-4>", lambda event: self.vehicle_tree.yview_scroll(-1, "units"))  # Linux up
        self.vehicle_tree.bind("<Button-5>", lambda event: self.vehicle_tree.yview_scroll(1, "units"))   # Linux down

        toolbar = tk.Frame(content, bg="#E1F0FF", highlightbackground=BORDER, highlightthickness=1, padx=8, pady=8)
        toolbar.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for col in range(2):
            toolbar.grid_columnconfigure(col, weight=1, uniform="buttons")
        self.clear_button = self._make_button(toolbar, "▤  XÓA TOÀN BỘ DỮ LIỆU", ORANGE, self.clear_data, 0)
        self.quit_button = self._make_button(toolbar, "⏻  THOÁT", RED, self.close, 1)

        bottom = tk.Frame(content, bg=BG)
        bottom.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(5, 0))
        self.footer_label = tk.Label(bottom, text="YOLO REALTIME • CHỤP ẢNH KHI QUẸT THẺ • RFID • ESP32", anchor="w",
                                     font=("Segoe UI", 8), bg=BG, fg=MUTED)
        self.footer_label.pack(side="left")
        self.log_label = tk.Label(bottom, text="", anchor="e", font=("Segoe UI", 8), bg=BG, fg=MUTED)
        self.log_label.pack(side="right")
        self._update_stats()
        self._refresh_info()
        self._update_vehicle_list()

    def _update_vehicle_list(self) -> None:
        """Show only active UID ↔ plate pairings for vehicles currently parked."""
        if not hasattr(self, "vehicle_tree"):
            return
        for item in self.vehicle_tree.get_children():
            self.vehicle_tree.delete(item)

        # Registry luôn đồng bộ với danh sách xe đang đỗ; không giữ liên kết xe đã ra.
        active_registry: dict[str, dict[str, str]] = {}
        for uid, record in self.parked.items():
            active_registry[uid] = {
                "plate": record.get("plate", ""),
                "display": record.get("display", format_plate(record.get("plate", ""))),
                "last_entry_time": record.get("entry_time", "---"),
            }
        self.card_registry = active_registry
        self.save_card_registry()

        items = sorted(
            self.card_registry.items(),
            key=lambda item: (item[1].get("display", item[1].get("plate", "")), item[0])
        )
        for uid, record in items:
            plate = record.get("display") or format_plate(record.get("plate", "")) or "---"
            self.vehicle_tree.insert("", "end", values=(uid, plate, "ĐANG ĐỖ"))
        self.registry_count.configure(text=f"{len(items)} xe đang đỗ")

    def _on_vehicle_list_mousewheel(self, event) -> str:
        """Allow scrolling the UID/plate list with the mouse wheel on Windows."""
        if event.delta:
            units = int(-event.delta / 120)
            if units == 0:
                units = -1 if event.delta > 0 else 1
            self.vehicle_tree.yview_scroll(units, "units")
        return "break"

    def _make_button(self, parent: tk.Widget, text: str, color: str, command, column: int) -> tk.Button:
        btn = tk.Button(parent, text=text, command=command, bg=color, fg=WHITE,
                        activebackground=color, activeforeground=WHITE, relief="flat", bd=0,
                        font=("Segoe UI", 10, "bold"), padx=7, pady=12, cursor="hand2")
        btn.grid(row=0, column=column, sticky="ew", padx=4, ipady=2)
        return btn

    # ---------- Rendering ----------
    def _update_clock(self) -> None:
        now = datetime.now()
        self.date_label.configure(text="📅  " + now.strftime("%A, %d/%m/%Y"))
        self.time_label.configure(text="◷  " + now.strftime("%H:%M:%S"))
        self.root.after(500, self._update_clock)

    def _update_camera(self) -> None:
        # Đọc frame mới liên tục để vừa hiển thị camera vừa làm nguồn cho YOLO.
        if self.cap is not None:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame.copy()

        with self.frame_lock:
            if self.live_annotated_frame is not None:
                display = self.live_annotated_frame.copy()
            else:
                display = self.latest_frame.copy() if self.latest_frame is not None else None

        if display is not None:
            try:
                max_w = max(400, self.camera_label.winfo_width())
                max_h = max(260, self.camera_label.winfo_height())
                h, w = display.shape[:2]
                scale = min(max_w / w, max_h / h)
                resized = cv2.resize(display, (max(1, int(w * scale)), max(1, int(h * scale))))
                resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(resized)
                photo = ImageTk.PhotoImage(image=image)
                self.camera_label.configure(image=photo, text="")
                self.camera_label.image = photo
            except Exception as exc:
                self.log_label.configure(text=f"Hiển thị camera: {exc}")
        elif self.camera_error:
            self.camera_label.configure(image="", text=f"KHÔNG MỞ ĐƯỢC CAMERA\n{self.camera_error}")
        else:
            self.camera_label.configure(image="", text="Đang chờ hình ảnh camera...")
        self.root.after(30, self._update_camera)

    def _schedule_live_detection(self) -> None:
        """Run YOLO on recent frames repeatedly without blocking Tkinter."""
        if self.stop_event.is_set():
            return
        if self.model is not None and not self.live_detection_busy:
            with self.frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
            if frame is not None:
                self.live_detection_busy = True
                threading.Thread(target=self._live_detection_worker, args=(frame,), daemon=True).start()
        # Limit YOLO frequency to avoid saturating CPU; camera preview itself remains smooth.
        self.root.after(180, self._schedule_live_detection)

    def _live_detection_worker(self, frame) -> None:
        try:
            with self.model_lock:
                plate_norm, plate_display, annotated = self._recognize_plate(frame)
            self.events.put(("live_detection_done", {
                "frame": annotated,
                "plate_norm": plate_norm,
                "plate_display": plate_display,
                "timestamp": time.time(),
            }))
        except Exception as exc:
            self.events.put(("live_detection_error", str(exc)))
            self.events.put(("live_detection_done", {
                "frame": frame,
                "plate_norm": "",
                "plate_display": "",
                "timestamp": time.time(),
            }))

    def _update_stats(self) -> None:
        self.free_slots = max(0, min(self.total_slots, int(self.free_slots)))
        self.occupied_slots = max(0, self.total_slots - self.free_slots)
        self.metric_values["free"].configure(text=str(self.free_slots))
        self.metric_values["occupied"].configure(text=str(self.occupied_slots))
        self.metric_values["total"].configure(text=str(self.total_slots))
        for idx, lamp in enumerate(self.slot_lamps):
            is_free = idx < self.free_slots
            lamp.configure(text="TRỐNG" if is_free else "ĐÃ ĐỖ", bg=GREEN if is_free else RED)
        if self.free_slots > 0:
            self.availability_banner.configure(text="BÃI ĐỖ CÒN TRỐNG", bg=GREEN)
        else:
            self.availability_banner.configure(text="BÃI ĐỖ ĐÃ ĐẦY", bg=RED)

    def _refresh_info(self) -> None:
        values = {
            "uid": self.current_uid,
            "plate": self.current_plate,
            "action": self.current_action,
            "entry": self.entry_time,
            "exit": self.exit_time,
            "status": self.status_text,
        }
        for key, value in values.items():
            self.info_values[key].configure(text=value)
        self.info_values["status"].configure(fg=self.status_color)
        self.top_status.configure(text=self.status_text)

    def _set_status(self, text: str, color: str = GREEN_DARK) -> None:
        self.status_text = text
        self.status_color = color
        if hasattr(self, "info_values"):
            self.info_values["status"].configure(text=text, fg=color)
        if hasattr(self, "top_status"):
            self.top_status.configure(text=text)

    # ---------- Serial protocol and parking logic ----------
    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "serial":
                    self._handle_serial_line(payload)
                elif kind == "serial_error":
                    self.serial_error = payload
                    self._set_status("LỖI SERIAL", RED)
                    self.log_label.configure(text="Lỗi kết nối Serial")
                elif kind == "recognition_done":
                    self._finish_recognition(payload)
                elif kind == "live_detection_done":
                    self.live_detection_busy = False
                    frame = payload.get("frame")
                    if frame is not None:
                        with self.frame_lock:
                            self.live_annotated_frame = frame.copy()
                    self.live_plate_norm = payload.get("plate_norm", "")
                    self.live_plate_display = payload.get("plate_display", "")
                    # Cập nhật kết quả nhận diện trực tiếp, nhưng không ghi đè thông tin
                    # trong lúc đang xử lý lần quẹt thẻ.
                    if not self.scan_in_progress:
                        if self.live_plate_norm:
                            self.plate_result.configure(text=self.live_plate_display, font=("Segoe UI", 27, "bold"))
                        else:
                            self.plate_result.configure(text="ĐANG TÌM BIỂN SỐ...", font=("Segoe UI", 18, "bold"))
                elif kind == "live_detection_error":
                    self.live_detection_busy = False
                    self.log_label.configure(text=f"Lỗi YOLO realtime: {str(payload)[:70]}")
                elif kind == "log":
                    self.log_label.configure(text=str(payload)[:90])
                elif kind == "camera_refresh_done":
                    self._set_status(str(payload), GREEN_DARK)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_events)

    def _handle_serial_line(self, line: str) -> None:
        print(f"ESP32: {line}")
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 3 and parts[0].upper() == "CARD":
            uid = parts[1].upper().replace(" ", "")
            try:
                free = int(parts[2])
                if 0 <= free <= self.total_slots:
                    self.free_slots = free
                    self._update_stats()
            except ValueError:
                pass
            with self.data_lock:
                if uid in self.pending_uids:
                    self._send("DENY,DUPLICATE_REQUEST")
                    return
                self.pending_uids.add(uid)
            self.current_uid = uid
            self.scan_in_progress = True
            self.current_action = "ĐANG NHẬN DIỆN"
            self.entry_time = "---"
            self.exit_time = "---"
            self.plate_result.configure(text="ĐANG NHẬN DIỆN...", font=("Segoe UI", 20, "bold"))
            self._set_status("ĐANG NHẬN DIỆN BIỂN SỐ...", BLUE)
            self._refresh_info()
            threading.Thread(target=self._recognition_worker, args=(uid, free, self.selected_mode), daemon=True).start()
        elif len(parts) >= 3 and parts[0].upper() == "GATE_CLOSED":
            uid = parts[1].upper().replace(" ", "")
            action = parts[2].upper()
            self._commit_transaction(uid, action)
        elif len(parts) >= 3 and parts[0].upper() == "SLOTS":
            # ESP32 protocol is exactly: SLOTS,<free_slots>,<occupied_slots>
            try:
                reported_free = int(parts[1])
                reported_occupied = int(parts[2])
                if not (0 <= reported_free <= self.total_slots and
                        0 <= reported_occupied <= self.total_slots):
                    raise ValueError("slot count out of range")
                if reported_free + reported_occupied != self.total_slots:
                    # Prefer the free count from ESP32; calculate the complementary count
                    # if packet values briefly disagree during sensor transitions.
                    reported_occupied = self.total_slots - reported_free
                self.free_slots = reported_free
                self.sensor_status_received = True
                self._update_stats()

                # One-time boot reconciliation: if ESP32 says every bay is empty but JSON
                # contains cars from a previous run, those records are stale. Clear them so
                # UI/list/counts agree with the sensors and stale cards cannot match old cars.
                if not self.initial_sensor_sync_done:
                    self.initial_sensor_sync_done = True
                    if reported_occupied == 0 and self.parked:
                        stale_count = len(self.parked)
                        self.parked.clear()
                        self.card_registry.clear()
                        self.save_records()
                        self.save_card_registry()
                        self._update_vehicle_list()
                        self.log_label.configure(
                            text=f"Đồng bộ ESP32: bãi trống, đã xóa {stale_count} bản ghi cũ"
                        )
                        self._set_status("ĐÃ ĐỒNG BỘ: BÃI XE ĐANG TRỐNG", GREEN_DARK)
                    elif reported_occupied != len(self.parked):
                        self.log_label.configure(
                            text=f"Cảm biến báo {reported_occupied} xe; dữ liệu đang đỗ có {len(self.parked)} xe"
                        )
            except (ValueError, IndexError) as exc:
                self.log_label.configure(text=f"Gói SLOTS không hợp lệ: {exc}")

    def _recognition_worker(self, uid: str, free_slots: int, mode: str) -> None:
        # Chụp frame hiện tại ngay khi nhận sự kiện CARD từ ESP32.
        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None
        if frame is None:
            self.events.put(("recognition_done", {"uid": uid, "free": free_slots, "mode": mode,
                                                    "error": "NO_CAMERA_FRAME", "reason": "NO_PLATE"}))
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe_uid = re.sub(r"[^A-Z0-9_-]", "", uid.upper()) or "UNKNOWN"
        try:
            CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
            raw_path = CAPTURE_DIR / f"{stamp}_{safe_uid}_capture.jpg"
            cv2.imwrite(str(raw_path), frame)
        except Exception as exc:
            print(f"Cannot save card-scan snapshot: {exc}")

        if self.model is None:
            self.events.put(("recognition_done", {"uid": uid, "free": free_slots, "mode": mode,
                                                    "error": "MODEL_NOT_LOADED", "reason": "NO_PLATE",
                                                    "frame": frame}))
            return
        try:
            # Dùng lock để YOLO real-time và YOLO lúc quẹt thẻ không chạy đồng thời.
            with self.model_lock:
                plate_norm, plate_display, annotated = self._recognize_plate(frame.copy())
            try:
                CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
                detected_path = CAPTURE_DIR / f"{stamp}_{safe_uid}_detected.jpg"
                cv2.imwrite(str(detected_path), annotated)
            except Exception as exc:
                print(f"Cannot save annotated snapshot: {exc}")
            self.events.put(("recognition_done", {"uid": uid, "free": free_slots, "mode": mode,
                                                    "plate_norm": plate_norm, "plate_display": plate_display,
                                                    "frame": annotated, "snapshot": str(raw_path),
                                                    "reason": "" if plate_norm else "NO_PLATE"}))
        except Exception as exc:
            self.events.put(("recognition_done", {"uid": uid, "free": free_slots, "mode": mode,
                                                    "error": str(exc), "reason": "NO_PLATE", "frame": frame}))

    def _recognize_plate(self, frame):
        result = self.model.predict(frame, conf=self.args.conf, imgsz=self.args.imgsz, verbose=False)[0]
        detections: list[dict[str, Any]] = []
        if result.boxes is not None:
            names = result.names
            for box in result.boxes:
                cls_id = int(box.cls.item())
                class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else names[cls_id]
                char = character_from_class(str(class_name))
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf.item())
                color = (45, 220, 70) if char else (80, 160, 255)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), max(18, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
                if char:
                    detections.append({"char": char, "cx": (x1 + x2) / 2, "cy": (y1 + y2) / 2,
                                       "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": conf})
        rows = sort_character_detections(detections)
        raw = "".join("".join(d["char"] for d in row) for row in rows)
        norm = normalize_plate(raw)
        # Avoid accepting a couple of isolated false detections as a plate.
        if len(norm) < 5:
            norm = ""
        display = format_plate(norm) if norm else "KHÔNG TÌM THẤY BIỂN SỐ"
        return norm, display, frame

    def _finish_recognition(self, data: dict[str, Any]) -> None:
        uid = data["uid"]
        plate_norm = data.get("plate_norm", "")
        plate_display = data.get("plate_display", "")
        frame = data.get("frame")
        self.scan_in_progress = False
        if frame is not None:
            self.current_annotated = frame
        if data.get("error"):
            print(f"Recognition issue: {data['error']}")
        if not plate_norm:
            self.plate_result.configure(text="KHÔNG NHẬN DIỆN ĐƯỢC", font=("Segoe UI", 17, "bold"))
            self.current_plate = "---"
            self.current_action = "TỪ CHỐI"
            self._send("DENY,NO_PLATE")
            self._set_status("TỪ CHỐI: KHÔNG THẤY BIỂN SỐ", RED)
            self._release_uid(uid)
            self._refresh_info()
            return

        self.current_plate = plate_display
        self.last_detected_norm = plate_norm
        self.last_detected_display = plate_display
        self.last_recognition_at = time.time()
        self.plate_result.configure(text=plate_display, font=("Segoe UI", 27, "bold"))
        mode = data.get("mode", "AUTO")
        free_slots = int(data.get("free", self.free_slots))

        with self.data_lock:
            known = self.parked.get(uid)
            other_uid = next((parked_uid for parked_uid, record in self.parked.items()
                              if parked_uid != uid and normalize_plate(record.get("plate", "")) == plate_norm), None)

        # AUTO: UID already parked => exit, otherwise entry. Buttons VÀO/RA can choose one scan's mode.
        if mode == "ENTRY":
            action = "ENTRY"
            if known:
                self._deny(uid, "THẺ NÀY ĐANG Ở TRONG BÃI", "CARD_ALREADY_PARKED")
                return
        elif mode == "EXIT":
            action = "EXIT"
            if not known:
                self._deny(uid, "THẺ CHƯA CÓ XE TRONG BÃI", "UID_NOT_PARKED")
                return
        else:
            action = "EXIT" if known else "ENTRY"

        if action == "ENTRY":
            if free_slots <= 0:
                self._deny(uid, "BÃI ĐỖ ĐÃ ĐẦY", "FULL")
                return
            if other_uid:
                self._deny(uid, "BIỂN SỐ ĐANG ĐƯỢC GẮN VỚI THẺ KHÁC", "DUPLICATE_PLATE")
                return
            self.pending_actions[uid] = {"action": "ENTRY", "plate": plate_norm,
                                         "display": plate_display, "time": datetime.now().isoformat(timespec="seconds")}
            self.current_action = "XE VÀO"
            self.entry_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            self.exit_time = "---"
            if self._send("OPEN,ENTRY"):
                self._set_status("ĐƯỢC PHÉP VÀO • ĐANG MỞ CỔNG", GREEN_DARK)
            else:
                self.pending_actions.pop(uid, None)
                self._deny(uid, "CHƯA KẾT NỐI ESP32", "SERIAL_ERROR", send=False)
        else:
            stored = normalize_plate(known.get("plate", "")) if known else ""
            if not known:
                self._deny(uid, "KHÔNG CÓ LỊCH SỬ XE ĐANG ĐỖ", "UID_NOT_PARKED")
                return
            if stored != plate_norm:
                self._deny(uid, "BIỂN SỐ KHÔNG KHỚP LÚC VÀO", "PLATE_MISMATCH")
                return
            self.pending_actions[uid] = {"action": "EXIT", "plate": plate_norm,
                                         "display": plate_display, "time": datetime.now().isoformat(timespec="seconds")}
            self.current_action = "XE RA"
            self.entry_time = known.get("entry_time", "---")
            self.exit_time = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            if self._send("OPEN,EXIT"):
                self._set_status("ĐƯỢC PHÉP RA • ĐANG MỞ CỔNG", GREEN_DARK)
            else:
                self.pending_actions.pop(uid, None)
                self._deny(uid, "CHƯA KẾT NỐI ESP32", "SERIAL_ERROR", send=False)
        self._refresh_info()
        # Chế độ tự động: UID đang có trong bãi thì xử lý xe ra, ngược lại xử lý xe vào.

    def _deny(self, uid: str, message: str, reason: str, send: bool = True) -> None:
        if send:
            self._send(f"DENY,{reason}")
        self.current_action = "TỪ CHỐI"
        self._set_status(message, RED)
        self._refresh_info()
        self._release_uid(uid)

    def _release_uid(self, uid: str) -> None:
        with self.data_lock:
            self.pending_uids.discard(uid)

    def _commit_transaction(self, uid: str, action: str) -> None:
        pending = self.pending_actions.get(uid)
        if not pending:
            self._set_status("CỔNG ĐÃ ĐÓNG", MUTED)
            self._release_uid(uid)
            return
        if action != pending["action"]:
            self.log_label.configure(text=f"Cảnh báo: action lệch {action}/{pending['action']}")
            return
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        if action == "ENTRY":
            self.parked[uid] = {
                "plate": pending["plate"],
                "display": pending["display"],
                "entry_time": now,
            }
            self.card_registry[uid] = {
                "plate": pending["plate"],
                "display": pending["display"],
                "last_entry_time": now,
            }
            self.save_card_registry()
            self.entry_time = now
            self.exit_time = "---"
            self.current_action = "ĐÃ VÀO BÃI"
            self._set_status("ĐÃ GHI NHẬN XE VÀO BÃI", GREEN_DARK)
        elif action == "EXIT":
            record = self.parked.pop(uid, None)
            # Xe ra thành công: xóa liên kết UID ↔ biển số để thẻ được phép ghép
            # với xe khác trong lượt vào tiếp theo.
            self.card_registry.pop(uid, None)
            self.save_card_registry()
            self.entry_time = record.get("entry_time", "---") if record else self.entry_time
            self.exit_time = now
            self.current_action = "ĐÃ RA KHỎI BÃI"
            self._set_status("ĐÃ GHI NHẬN XE RA BÃI", GREEN_DARK)
        self.pending_actions.pop(uid, None)
        self.save_records()
        self._release_uid(uid)
        # Use physical ESP32 sensor counts whenever available; otherwise estimate from records.
        if not self.sensor_status_received:
            self.free_slots = max(0, self.total_slots - len(self.parked))
        self._update_stats()
        self._refresh_info()
        self._update_vehicle_list()

    # ---------- Toolbar actions ----------
    def clear_data(self) -> None:
        """Delete all app-managed parking records and saved scan photos."""
        if self.scan_in_progress or self.pending_actions or self.pending_uids:
            messagebox.showwarning(
                "Chưa thể xóa dữ liệu",
                "Đang xử lý lượt quẹt thẻ hoặc cổng chưa hoàn tất. Hãy đợi giao dịch kết thúc rồi thử lại.",
                parent=self.root,
            )
            return

        confirmed = messagebox.askyesno(
            "Xác nhận xóa TOÀN BỘ dữ liệu",
            "Thao tác này sẽ xóa vĩnh viễn:\n"
            "• Danh sách xe đang đỗ và liên kết UID thẻ ↔ biển số\n"
            "• Dữ liệu lưu trong parking_records.json và card_plate_registry.json\n"
            "• Tất cả ảnh quẹt thẻ trong thư mục captures\n\n"
            "Không xóa model YOLO best.pt. Bạn có chắc chắn muốn tiếp tục?",
            icon="warning",
            parent=self.root,
        )
        if not confirmed:
            return

        deleted_images = 0
        errors: list[str] = []
        self.parked.clear()
        self.card_registry.clear()
        self.pending_actions.clear()
        self.pending_uids.clear()

        # Delete images generated by card scans. Keep the folder itself and the YOLO model.
        try:
            if CAPTURE_DIR.exists():
                for path in CAPTURE_DIR.iterdir():
                    if path.is_file():
                        try:
                            path.unlink()
                            deleted_images += 1
                        except OSError as exc:
                            errors.append(f"{path.name}: {exc}")
                    elif path.is_dir():
                        import shutil
                        try:
                            shutil.rmtree(path)
                        except OSError as exc:
                            errors.append(f"{path.name}: {exc}")
        except OSError as exc:
            errors.append(f"captures: {exc}")

        # Preserve real ESP32 occupancy; deleting records must not fake an empty physical lot.
        if not self.sensor_status_received:
            self.free_slots = self.total_slots
        self.current_uid = "---"
        self.current_plate = "---"
        self.current_action = "---"
        self.entry_time = "---"
        self.exit_time = "---"
        self.plate_result.configure(text="---", font=("Segoe UI", 27, "bold"))
        self._update_stats()
        self._refresh_info()
        self._update_vehicle_list()

        # Delete the persistent JSON files after refreshing the table (refresh writes an empty registry).
        # This prevents the application from reloading any previous UID/plate records next launch.
        for data_file in (DB_PATH, CARD_REGISTRY_PATH):
            try:
                data_file.unlink(missing_ok=True)
            except OSError as exc:
                errors.append(f"{data_file.name}: {exc}")

        if errors:
            self._set_status("ĐÃ XÓA, NHƯNG CÓ FILE KHÔNG XÓA ĐƯỢC", ORANGE)
            self.log_label.configure(text="; ".join(errors)[:120])
            messagebox.showwarning(
                "Xóa chưa hoàn tất",
                "Đã xóa dữ liệu trong bộ nhớ nhưng một số file không xóa được. Hãy đóng ứng dụng khác đang dùng file và thử lại:\n\n"
                + "\n".join(errors),
                parent=self.root,
            )
        else:
            self._set_status("ĐÃ XÓA TOÀN BỘ DỮ LIỆU ỨNG DỤNG", GREEN_DARK)
            self.log_label.configure(text=f"Đã xóa dữ liệu xe/thẻ và {deleted_images} ảnh chụp")

    def close(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Giao diện bãi đỗ xe thông minh một camera")
    parser.add_argument("--source", default="1", help="Chỉ số camera hoặc URL, mặc định 1")
    parser.add_argument("--backend", choices=("dshow", "msmf", "default"), default="dshow",
                        help="Backend camera trên Windows")
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS), help="Đường dẫn model YOLO best.pt")
    parser.add_argument("--port", default="COM18", help="Cổng Serial của ESP32, ví dụ COM3")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate Serial")
    parser.add_argument("--conf", type=float, default=0.5, help="Ngưỡng confidence YOLO")
    parser.add_argument("--imgsz", type=int, default=640, help="Kích thước inference YOLO")
    return parser.parse_args()


if __name__ == "__main__":
    app = ParkingApp(parse_args())
    app.run()
