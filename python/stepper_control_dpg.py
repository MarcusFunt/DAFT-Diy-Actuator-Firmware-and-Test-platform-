from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

import dearpygui.dearpygui as dpg
import serial
from serial.tools import list_ports

import daft_protocol as proto


BAUD_RATE = 115200
DEFAULT_TELEM_INTERVAL_MS = 100
COMMAND_QUEUE_DEPTH = 4
MAX_HISTORY_POINTS = 240

DRIVER_MODES = {
    "StealthChop": proto.DRIVER_MODE_STEALTHCHOP,
    "SpreadCycle": proto.DRIVER_MODE_SPREADCYCLE,
}


class SerialWorker:
    def __init__(self, on_event):
        self.on_event = on_event
        self.port = None
        self.thread = None
        self.stop_event = threading.Event()
        self.write_queue = queue.Queue()
        self.rx_buffer = bytearray()
        self.seq_lock = threading.Lock()
        self.next_seq = 1

    def connect(self, port_name):
        self.disconnect()
        self._clear_write_queue()
        self.rx_buffer.clear()
        self.stop_event.clear()
        self.port = serial.Serial(
            port_name,
            BAUD_RATE,
            timeout=0.02,
            write_timeout=0.2,
            inter_byte_timeout=0.02,
        )
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None

        if self.port:
            try:
                self.port.close()
            except serial.SerialException:
                pass
        self.port = None

    def send_packet(self, msg_id, payload=b"", flags=0):
        if not self.is_connected:
            raise RuntimeError("Serial port is not connected")

        with self.seq_lock:
            seq = self.next_seq
            self.next_seq = (self.next_seq + 1) & 0xFFFF
            if self.next_seq == 0:
                self.next_seq = 1

        self.write_queue.put(proto.encode_packet(msg_id, seq, payload, flags))
        return seq

    @property
    def is_connected(self):
        return self.port is not None and self.port.is_open

    def _clear_write_queue(self):
        while True:
            try:
                self.write_queue.get_nowait()
            except queue.Empty:
                return

    def _run(self):
        while not self.stop_event.is_set():
            try:
                while True:
                    try:
                        frame = self.write_queue.get_nowait()
                    except queue.Empty:
                        break
                    self.port.write(frame)

                chunk = self.port.read(128)
                if chunk:
                    self._consume_rx(chunk)
            except (serial.SerialException, OSError) as exc:
                self.on_event(("disconnected", str(exc)))
                self.stop_event.set()

            time.sleep(0.002)

    def _consume_rx(self, chunk):
        self.rx_buffer.extend(chunk)

        while True:
            try:
                delimiter_index = self.rx_buffer.index(0)
            except ValueError:
                if len(self.rx_buffer) > 96:
                    self.rx_buffer.clear()
                    self.on_event(("error", "RX buffer overflow; dropped partial frame"))
                return

            frame = bytes(self.rx_buffer[:delimiter_index])
            del self.rx_buffer[: delimiter_index + 1]
            if not frame:
                continue

            try:
                packet = proto.decode_packet(frame)
            except proto.ProtocolError as exc:
                self.on_event(("error", f"Dropped RX frame: {exc}"))
                continue

            self.on_event(("packet", packet))


@dataclass(frozen=True)
class Tags:
    MAIN: str = "daft.main"
    APP_THEME: str = "daft.theme.app"
    PRIMARY_BUTTON_THEME: str = "daft.theme.primary_button"
    SUCCESS_BUTTON_THEME: str = "daft.theme.success_button"
    DANGER_BUTTON_THEME: str = "daft.theme.danger_button"
    WARNING_BUTTON_THEME: str = "daft.theme.warning_button"
    ACCENT_TEXT_THEME: str = "daft.theme.accent_text"
    MUTED_TEXT_THEME: str = "daft.theme.muted_text"
    GOOD_TEXT_THEME: str = "daft.theme.good_text"
    WARN_TEXT_THEME: str = "daft.theme.warn_text"
    BAD_TEXT_THEME: str = "daft.theme.bad_text"

    PORT: str = "daft.port"
    CONNECT: str = "daft.connect"
    DISCONNECT: str = "daft.disconnect"
    REFRESH_PORTS: str = "daft.refresh_ports"
    CONNECTION_TEXT: str = "daft.connection_text"
    PING_BUTTON: str = "daft.ping"

    ENABLE_DRIVER: str = "daft.enable_driver"
    ZERO_BUTTON: str = "daft.zero"
    CLEAR_FAULTS_BUTTON: str = "daft.clear_faults"
    STATUS_BUTTON: str = "daft.status"
    ESTOP_BUTTON: str = "daft.estop"

    STATE_VALUE: str = "daft.state"
    DRIVER_VALUE: str = "daft.driver"
    POSITION_VALUE: str = "daft.position"
    TARGET_VALUE: str = "daft.target"
    SPEED_VALUE: str = "daft.speed"
    FAULT_VALUE: str = "daft.fault"
    TICK_VALUE: str = "daft.tick"
    QUEUE_VALUE: str = "daft.queue_value"
    QUEUE_BAR: str = "daft.queue_bar"
    TX_COUNTER: str = "daft.tx_counter"
    RX_COUNTER: str = "daft.rx_counter"

    ABS_TARGET: str = "daft.abs_target"
    REL_DELTA: str = "daft.rel_delta"
    MOVE_SPEED: str = "daft.move_speed"
    MOVE_ACCEL: str = "daft.move_accel"
    MOVE_LIMIT_FLAG: str = "daft.move_limit_flag"
    MOVE_ABS_BUTTON: str = "daft.move_abs"
    MOVE_REL_BUTTON: str = "daft.move_rel"

    VEL_SPEED: str = "daft.vel_speed"
    VEL_ACCEL: str = "daft.vel_accel"
    STOP_DECEL: str = "daft.stop_decel"
    RUN_VEL_BUTTON: str = "daft.run_vel"
    RAMP_STOP_BUTTON: str = "daft.ramp_stop"

    CURRENT: str = "daft.current"
    MICROSTEPS: str = "daft.microsteps"
    DRIVER_MODE: str = "daft.driver_mode"
    APPLY_DRIVER_BUTTON: str = "daft.apply_driver"

    SOFT_ENABLE: str = "daft.soft_enable"
    SOFT_MIN: str = "daft.soft_min"
    SOFT_MAX: str = "daft.soft_max"
    APPLY_SOFT_BUTTON: str = "daft.apply_soft"

    TELEM_RATE: str = "daft.telem_rate"
    APPLY_TELEM_BUTTON: str = "daft.apply_telem"

    POS_PLOT_X: str = "daft.pos_plot_x"
    POS_PLOT_Y: str = "daft.pos_plot_y"
    POS_SERIES: str = "daft.pos_series"
    TARGET_SERIES: str = "daft.target_series"
    LOG: str = "daft.log"
    ERROR_MODAL: str = "daft.error_modal"


T = Tags()


COMMAND_WIDGETS = (
    T.PING_BUTTON,
    T.ENABLE_DRIVER,
    T.ZERO_BUTTON,
    T.CLEAR_FAULTS_BUTTON,
    T.STATUS_BUTTON,
    T.ESTOP_BUTTON,
    T.ABS_TARGET,
    T.REL_DELTA,
    T.MOVE_SPEED,
    T.MOVE_ACCEL,
    T.MOVE_LIMIT_FLAG,
    T.MOVE_ABS_BUTTON,
    T.MOVE_REL_BUTTON,
    T.VEL_SPEED,
    T.VEL_ACCEL,
    T.STOP_DECEL,
    T.RUN_VEL_BUTTON,
    T.RAMP_STOP_BUTTON,
    T.CURRENT,
    T.MICROSTEPS,
    T.DRIVER_MODE,
    T.APPLY_DRIVER_BUTTON,
    T.SOFT_ENABLE,
    T.SOFT_MIN,
    T.SOFT_MAX,
    T.APPLY_SOFT_BUTTON,
    T.TELEM_RATE,
    T.APPLY_TELEM_BUTTON,
)


class DearStepperApp:
    def __init__(self):
        self.event_queue = queue.Queue()
        self.worker = SerialWorker(self.event_queue.put)
        self.log_lines = deque(maxlen=250)
        self.history = deque(maxlen=MAX_HISTORY_POINTS)
        self.history_start = None
        self.connected_port = ""
        self.tx_count = 0
        self.rx_count = 0

    def run(self):
        dpg.create_context()
        dpg.set_global_font_scale(1.04)
        self._create_themes()
        self._build_ui()
        self.refresh_ports()
        self._set_connected(False)

        dpg.create_viewport(title="DAFT Motion Console", width=1260, height=780, min_width=980, min_height=640)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window(T.MAIN, True)
        dpg.set_exit_callback(self._on_exit)

        while dpg.is_dearpygui_running():
            self._process_events()
            dpg.render_dearpygui_frame()

        self.worker.disconnect()
        dpg.destroy_context()

    def _create_themes(self):
        with dpg.theme(tag=T.APP_THEME):
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (18, 22, 28, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (25, 30, 38, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (33, 40, 50, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (45, 55, 68, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (53, 66, 82, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (48, 67, 88, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (63, 88, 116, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (77, 109, 144, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (226, 234, 242, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (105, 116, 128, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Border, (62, 73, 87, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (42, 56, 72, 255))
                dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (56, 76, 98, 255))
                dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (66, 91, 118, 255))
                dpg.add_theme_color(dpg.mvThemeCol_CheckMark, (95, 196, 157, 255))
                dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (95, 196, 157, 255))
                dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (112, 220, 177, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Tab, (32, 39, 49, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (56, 75, 96, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TabActive, (47, 61, 76, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg, (33, 41, 52, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TableRowBg, (25, 30, 38, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TableRowBgAlt, (29, 35, 44, 255))
                dpg.add_theme_color(dpg.mvThemeCol_PlotLines, (96, 190, 246, 255))
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 0)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 6)
                dpg.add_theme_style(dpg.mvStyleVar_GrabRounding, 6)
                dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 6)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 9, 6)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 14, 14)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 9, 8)
                dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 8, 7)

        self._button_theme(T.PRIMARY_BUTTON_THEME, (58, 116, 150), (75, 145, 185), (84, 165, 208))
        self._button_theme(T.SUCCESS_BUTTON_THEME, (39, 124, 91), (48, 154, 112), (57, 178, 129))
        self._button_theme(T.WARNING_BUTTON_THEME, (154, 116, 43), (181, 137, 54), (207, 158, 65))
        self._button_theme(T.DANGER_BUTTON_THEME, (160, 55, 61), (192, 68, 76), (221, 83, 91))
        self._text_theme(T.ACCENT_TEXT_THEME, (104, 197, 235, 255))
        self._text_theme(T.MUTED_TEXT_THEME, (143, 155, 168, 255))
        self._text_theme(T.GOOD_TEXT_THEME, (112, 220, 177, 255))
        self._text_theme(T.WARN_TEXT_THEME, (237, 188, 94, 255))
        self._text_theme(T.BAD_TEXT_THEME, (255, 111, 118, 255))
        dpg.bind_theme(T.APP_THEME)

    def _button_theme(self, tag, normal, hovered, active):
        with dpg.theme(tag=tag):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (*normal, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (*hovered, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (*active, 255))

    def _text_theme(self, tag, color):
        with dpg.theme(tag=tag):
            with dpg.theme_component(dpg.mvText):
                dpg.add_theme_color(dpg.mvThemeCol_Text, color)

    def _build_ui(self):
        with dpg.window(tag=T.MAIN, no_title_bar=True, no_collapse=True):
            with dpg.group(horizontal=True):
                self._build_left_panel()
                self._build_work_area()

    def _build_left_panel(self):
        with dpg.child_window(width=340, height=-1, border=True):
            dpg.add_text("DAFT Motion Console")
            dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
            dpg.add_text("ESP32-C6 / TMC2209")
            dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)
            dpg.add_separator()

            dpg.add_text("Connection")
            with dpg.group(horizontal=True):
                dpg.add_combo([], tag=T.PORT, width=205)
                dpg.add_button(label="Refresh", tag=T.REFRESH_PORTS, width=92, callback=self.refresh_ports)
            with dpg.group(horizontal=True):
                dpg.add_button(label="Connect", tag=T.CONNECT, width=145, callback=self.connect)
                dpg.add_button(label="Disconnect", tag=T.DISCONNECT, width=145, callback=self.disconnect)
            dpg.add_text("Disconnected", tag=T.CONNECTION_TEXT, wrap=300)
            dpg.bind_item_theme(T.CONNECTION_TEXT, T.MUTED_TEXT_THEME)
            dpg.add_button(label="Ping", tag=T.PING_BUTTON, width=-1, callback=self.ping)
            dpg.bind_item_theme(T.CONNECT, T.PRIMARY_BUTTON_THEME)

            dpg.add_separator()
            dpg.add_text("Live Status")
            with dpg.table(header_row=False, row_background=True, borders_innerH=True, borders_outerH=True, width=-1):
                dpg.add_table_column(width_fixed=True, init_width_or_weight=118)
                dpg.add_table_column()
                self._status_row("Motion", T.STATE_VALUE)
                self._status_row("Driver", T.DRIVER_VALUE)
                self._status_row("Position", T.POSITION_VALUE)
                self._status_row("Target", T.TARGET_VALUE)
                self._status_row("Speed", T.SPEED_VALUE)
                self._status_row("Faults", T.FAULT_VALUE)
                self._status_row("ESP tick", T.TICK_VALUE)
                self._status_row("Packets TX", T.TX_COUNTER)
                self._status_row("Packets RX", T.RX_COUNTER)

            dpg.add_spacer(height=4)
            dpg.add_text("Command Queue")
            dpg.add_progress_bar(tag=T.QUEUE_BAR, width=-1, height=20, default_value=0.0, overlay="--")
            dpg.add_text("--", tag=T.QUEUE_VALUE)
            dpg.bind_item_theme(T.QUEUE_VALUE, T.MUTED_TEXT_THEME)

    def _status_row(self, label, value_tag):
        with dpg.table_row():
            dpg.add_text(label)
            dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)
            dpg.add_text("--", tag=value_tag)

    def _build_work_area(self):
        with dpg.group(width=-1):
            self._build_quick_controls()
            with dpg.group(horizontal=True):
                with dpg.child_window(width=535, height=-1, border=True):
                    self._build_control_tabs()
                with dpg.child_window(width=-1, height=-1, border=True):
                    self._build_telemetry_panel()

    def _build_quick_controls(self):
        with dpg.child_window(height=86, width=-1, border=True):
            with dpg.group(horizontal=True):
                dpg.add_text("Manual Control")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                dpg.add_checkbox(label="Driver enabled", tag=T.ENABLE_DRIVER, callback=self.toggle_driver)
                dpg.add_button(label="Zero", tag=T.ZERO_BUTTON, width=76, callback=self.zero_position)
                dpg.add_button(label="Clear faults", tag=T.CLEAR_FAULTS_BUTTON, width=118, callback=self.clear_faults)
                dpg.add_button(label="Read status", tag=T.STATUS_BUTTON, width=112, callback=self.request_status)
                dpg.add_button(label="E-STOP", tag=T.ESTOP_BUTTON, width=118, height=34, callback=self.estop)
            dpg.add_text("Motion commands require the firmware driver-enable state to be on.")
            dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)
            dpg.bind_item_theme(T.ESTOP_BUTTON, T.DANGER_BUTTON_THEME)

    def _build_control_tabs(self):
        with dpg.tab_bar():
            with dpg.tab(label="Motion"):
                dpg.add_text("Position Moves")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                self._input_int("Absolute target", T.ABS_TARGET, 3200)
                self._input_int("Relative delta", T.REL_DELTA, 800)
                self._input_int("Max speed", T.MOVE_SPEED, 2000, min_value=1)
                self._input_int("Acceleration", T.MOVE_ACCEL, 2000, min_value=1)
                dpg.add_checkbox(label="Enforce configured soft limits on position moves", tag=T.MOVE_LIMIT_FLAG, default_value=True)
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Move absolute", tag=T.MOVE_ABS_BUTTON, width=175, callback=self.move_absolute)
                    dpg.add_button(label="Move relative", tag=T.MOVE_REL_BUTTON, width=175, callback=self.move_relative)
                dpg.bind_item_theme(T.MOVE_ABS_BUTTON, T.PRIMARY_BUTTON_THEME)
                dpg.bind_item_theme(T.MOVE_REL_BUTTON, T.PRIMARY_BUTTON_THEME)

                dpg.add_separator()
                dpg.add_text("Velocity")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                self._input_int("Target speed", T.VEL_SPEED, 1000)
                self._input_int("Acceleration", T.VEL_ACCEL, 1500, min_value=1)
                self._input_int("Ramp-stop decel", T.STOP_DECEL, 1500, min_value=1)
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Run velocity", tag=T.RUN_VEL_BUTTON, width=175, callback=self.run_velocity)
                    dpg.add_button(label="Ramp stop", tag=T.RAMP_STOP_BUTTON, width=175, callback=self.ramp_stop)
                dpg.bind_item_theme(T.RUN_VEL_BUTTON, T.SUCCESS_BUTTON_THEME)
                dpg.bind_item_theme(T.RAMP_STOP_BUTTON, T.WARNING_BUTTON_THEME)

            with dpg.tab(label="Driver"):
                dpg.add_text("Driver Configuration")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                self._input_int("Run current mA RMS", T.CURRENT, 800, min_value=1, max_value=65535)
                self._combo("Microsteps", T.MICROSTEPS, [str(value) for value in proto.SUPPORTED_MICROSTEPS], "16")
                self._combo("Mode", T.DRIVER_MODE, list(DRIVER_MODES), "StealthChop")
                dpg.add_button(label="Apply driver config", tag=T.APPLY_DRIVER_BUTTON, width=-1, callback=self.apply_driver_config)
                dpg.bind_item_theme(T.APPLY_DRIVER_BUTTON, T.PRIMARY_BUTTON_THEME)

                dpg.add_separator()
                dpg.add_text("Soft Limits")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                dpg.add_checkbox(label="Soft limits enabled", tag=T.SOFT_ENABLE, default_value=False)
                self._input_int("Minimum steps", T.SOFT_MIN, -100000)
                self._input_int("Maximum steps", T.SOFT_MAX, 100000)
                dpg.add_button(label="Apply soft limits", tag=T.APPLY_SOFT_BUTTON, width=-1, callback=self.apply_soft_limits)
                dpg.bind_item_theme(T.APPLY_SOFT_BUTTON, T.PRIMARY_BUTTON_THEME)

            with dpg.tab(label="Telemetry"):
                dpg.add_text("Telemetry Rate")
                dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
                self._input_int("Interval ms", T.TELEM_RATE, DEFAULT_TELEM_INTERVAL_MS, min_value=0, max_value=65535)
                dpg.add_button(label="Apply telemetry rate", tag=T.APPLY_TELEM_BUTTON, width=-1, callback=self.apply_telemetry_rate)
                dpg.bind_item_theme(T.APPLY_TELEM_BUTTON, T.PRIMARY_BUTTON_THEME)
                dpg.add_text("0 ms disables periodic STATUS packets.")
                dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)

    def _build_telemetry_panel(self):
        dpg.add_text("Position History")
        dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
        with dpg.plot(label="", height=255, width=-1, no_title=True):
            dpg.add_plot_axis(dpg.mvXAxis, label="seconds", tag=T.POS_PLOT_X)
            dpg.add_plot_axis(dpg.mvYAxis, label="steps", tag=T.POS_PLOT_Y)
            dpg.add_line_series([], [], label="Position", parent=T.POS_PLOT_Y, tag=T.POS_SERIES)
            dpg.add_line_series([], [], label="Target", parent=T.POS_PLOT_Y, tag=T.TARGET_SERIES)

        dpg.add_separator()
        dpg.add_text("Command Log")
        dpg.bind_item_theme(dpg.last_item(), T.ACCENT_TEXT_THEME)
        dpg.add_input_text(tag=T.LOG, multiline=True, readonly=True, width=-1, height=-1, default_value="")

    def _input_int(self, label, tag, default_value, min_value=-(2**31), max_value=2**31 - 1):
        with dpg.group(horizontal=True):
            dpg.add_text(label, bullet=False)
            dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)
            dpg.add_input_int(
                tag=tag,
                width=190,
                default_value=default_value,
                min_value=min_value,
                max_value=max_value,
                min_clamped=True,
                max_clamped=True,
            )

    def _combo(self, label, tag, items, default_value):
        with dpg.group(horizontal=True):
            dpg.add_text(label)
            dpg.bind_item_theme(dpg.last_item(), T.MUTED_TEXT_THEME)
            dpg.add_combo(items, tag=tag, width=190, default_value=default_value)

    def refresh_ports(self, sender=None, app_data=None, user_data=None):
        ports = [port.device for port in list_ports.comports()]
        dpg.configure_item(T.PORT, items=ports)
        current = dpg.get_value(T.PORT)
        if ports and current not in ports:
            dpg.set_value(T.PORT, ports[0])
        elif not ports:
            dpg.set_value(T.PORT, "")

    def connect(self, sender=None, app_data=None, user_data=None):
        port_name = dpg.get_value(T.PORT)
        if not port_name:
            self._show_error("No port selected", "Select the ESP32 serial port first.")
            return

        try:
            self.worker.connect(port_name)
        except (serial.SerialException, OSError) as exc:
            self._show_error("Connection failed", str(exc))
            return

        self.connected_port = port_name
        self._set_connected(True)
        self._append_log(f"Connected to {port_name} at {BAUD_RATE}")
        self.apply_telemetry_rate()
        self.request_status()

    def disconnect(self, sender=None, app_data=None, user_data=None):
        self.worker.disconnect()
        self.connected_port = ""
        self._set_connected(False)
        self._append_log("Disconnected")

    def ping(self, sender=None, app_data=None, user_data=None):
        self._send_packet(proto.MSG_PING)

    def request_status(self, sender=None, app_data=None, user_data=None):
        self._send_packet(proto.MSG_GET_STATUS)

    def toggle_driver(self, sender=None, app_data=None, user_data=None):
        enabled = bool(dpg.get_value(T.ENABLE_DRIVER))
        self._send_packet(proto.MSG_ENABLE if enabled else proto.MSG_DISABLE)

    def zero_position(self, sender=None, app_data=None, user_data=None):
        self._send_packet(proto.MSG_ZERO_POSITION)

    def clear_faults(self, sender=None, app_data=None, user_data=None):
        self._send_packet(proto.MSG_CLEAR_FAULTS)

    def estop(self, sender=None, app_data=None, user_data=None):
        self._send_packet(proto.MSG_ESTOP)

    def move_absolute(self, sender=None, app_data=None, user_data=None):
        self._send_move(proto.MSG_MOVE_ABS, int(dpg.get_value(T.ABS_TARGET)), "MOVE_ABS")

    def move_relative(self, sender=None, app_data=None, user_data=None):
        self._send_move(proto.MSG_MOVE_REL, int(dpg.get_value(T.REL_DELTA)), "MOVE_REL")

    def _send_move(self, msg_id, target_steps, command_name):
        speed = int(dpg.get_value(T.MOVE_SPEED))
        accel = int(dpg.get_value(T.MOVE_ACCEL))
        flags = 0x01 if dpg.get_value(T.MOVE_LIMIT_FLAG) else 0x00
        try:
            payload = proto.payload_move(target_steps, speed, accel, move_flags=flags)
        except ValueError as exc:
            self._show_error("Invalid move", str(exc))
            return
        self._send_packet(msg_id, payload, f"{command_name} target={target_steps} speed={speed} accel={accel}")

    def run_velocity(self, sender=None, app_data=None, user_data=None):
        speed = int(dpg.get_value(T.VEL_SPEED))
        accel = int(dpg.get_value(T.VEL_ACCEL))
        try:
            payload = proto.payload_run_velocity(speed, accel)
        except ValueError as exc:
            self._show_error("Invalid velocity", str(exc))
            return
        self._send_packet(proto.MSG_RUN_VEL, payload, f"RUN_VEL speed={speed} accel={accel}")

    def ramp_stop(self, sender=None, app_data=None, user_data=None):
        decel = int(dpg.get_value(T.STOP_DECEL))
        try:
            payload = proto.payload_stop_ramp(decel)
        except ValueError as exc:
            self._show_error("Invalid stop", str(exc))
            return
        self._send_packet(proto.MSG_STOP_RAMP, payload, f"STOP_RAMP decel={decel}")

    def apply_driver_config(self, sender=None, app_data=None, user_data=None):
        current = int(dpg.get_value(T.CURRENT))
        microsteps = int(dpg.get_value(T.MICROSTEPS))
        driver_mode_name = dpg.get_value(T.DRIVER_MODE)
        driver_mode = DRIVER_MODES[driver_mode_name]

        if microsteps not in proto.SUPPORTED_MICROSTEPS:
            self._show_error("Invalid microsteps", "Firmware supports 1, 2, 4, 8, 16, 32, 64, 128, or 256 microsteps.")
            return

        try:
            payload = proto.payload_driver_config(
                run_current_ma=current,
                microsteps=microsteps,
                driver_mode=driver_mode,
            )
        except ValueError as exc:
            self._show_error("Invalid driver config", str(exc))
            return

        self._send_packet(
            proto.MSG_DRIVER_CONFIG,
            payload,
            f"DRIVER_CONFIG current={current} microsteps={microsteps} mode={driver_mode_name}",
        )

    def apply_soft_limits(self, sender=None, app_data=None, user_data=None):
        enabled = bool(dpg.get_value(T.SOFT_ENABLE))
        minimum = int(dpg.get_value(T.SOFT_MIN))
        maximum = int(dpg.get_value(T.SOFT_MAX))
        try:
            payload = proto.payload_set_soft_limits(enabled, minimum, maximum)
        except ValueError as exc:
            self._show_error("Invalid soft limits", str(exc))
            return

        state = "enabled" if enabled else "disabled"
        self._send_packet(proto.MSG_SET_SOFT_LIMITS, payload, f"SET_SOFT_LIMITS {state} min={minimum} max={maximum}")

    def apply_telemetry_rate(self, sender=None, app_data=None, user_data=None):
        interval_ms = int(dpg.get_value(T.TELEM_RATE))
        try:
            payload = proto.payload_telem_rate(interval_ms)
        except ValueError as exc:
            self._show_error("Invalid telemetry rate", str(exc))
            return

        self._send_packet(proto.MSG_SET_TELEM_RATE, payload, f"SET_TELEM_RATE {interval_ms} ms")

    def _send_packet(self, msg_id, payload=b"", label=None):
        try:
            seq = self.worker.send_packet(msg_id, payload)
        except RuntimeError as exc:
            self._show_error("Not connected", str(exc))
            return None
        except ValueError as exc:
            self._show_error("Invalid command", str(exc))
            return None

        self.tx_count += 1
        dpg.set_value(T.TX_COUNTER, str(self.tx_count))
        self._append_log(f"> #{seq:05d} {label or proto.command_name(msg_id)}")
        return seq

    def _process_events(self):
        while not self.event_queue.empty():
            kind, payload = self.event_queue.get_nowait()
            if kind == "packet":
                self._handle_packet(payload)
            elif kind == "error":
                self._append_log(f"! {payload}")
            elif kind == "disconnected":
                self.connected_port = ""
                self._set_connected(False)
                self._append_log(f"Disconnected: {payload}")

    def _handle_packet(self, packet):
        self.rx_count += 1
        dpg.set_value(T.RX_COUNTER, str(self.rx_count))

        try:
            if packet.msg_id == proto.MSG_ACK:
                acked_id, _ = proto.parse_ack(packet)
                self._append_log(f"< ACK #{packet.seq:05d} {proto.command_name(acked_id)}")
            elif packet.msg_id == proto.MSG_NACK:
                rejected_id, reason = proto.parse_nack(packet)
                self._append_log(f"< NACK #{packet.seq:05d} {proto.command_name(rejected_id)}: {proto.reason_name(reason)}")
            elif packet.msg_id == proto.MSG_STATUS:
                self._update_status(proto.parse_status(packet))
            elif packet.msg_id == proto.MSG_EVENT:
                event_id, detail = proto.parse_event(packet)
                event_name = proto.EVENT_NAMES.get(event_id, f"event {event_id}")
                self._append_log(f"< EVENT {event_name} detail={detail}")
            else:
                self._append_log(f"< {proto.command_name(packet.msg_id)} #{packet.seq:05d} len={len(packet.payload)}")
        except proto.ProtocolError as exc:
            self._append_log(f"! Bad packet {proto.command_name(packet.msg_id)}: {exc}")

    def _update_status(self, fields):
        enabled = fields["driver_enabled"] == 1
        fault_flags = int(fields["fault_flags"])
        queue_free = int(fields["rx_queue_free"])
        motion_state = str(fields["motion_state_name"])

        dpg.set_value(T.ENABLE_DRIVER, enabled)
        dpg.set_value(T.STATE_VALUE, motion_state)
        dpg.set_value(T.DRIVER_VALUE, "enabled" if enabled else "disabled")
        dpg.set_value(T.POSITION_VALUE, str(fields["position_steps"]))
        dpg.set_value(T.TARGET_VALUE, str(fields["target_steps"]))
        dpg.set_value(T.SPEED_VALUE, str(fields["speed_sps"]))
        dpg.set_value(T.FAULT_VALUE, f"0x{fault_flags:04X}")
        dpg.set_value(T.TICK_VALUE, str(fields["esp_tick_ms"]))
        dpg.set_value(T.QUEUE_VALUE, f"{queue_free} of {COMMAND_QUEUE_DEPTH} slots free")
        dpg.set_value(T.QUEUE_BAR, max(0.0, min(1.0, queue_free / COMMAND_QUEUE_DEPTH)))
        dpg.configure_item(T.QUEUE_BAR, overlay=f"{queue_free}/{COMMAND_QUEUE_DEPTH}")

        if fault_flags:
            dpg.bind_item_theme(T.STATE_VALUE, T.BAD_TEXT_THEME)
            dpg.bind_item_theme(T.FAULT_VALUE, T.BAD_TEXT_THEME)
        elif motion_state in {"moving", "velocity", "stopping"}:
            dpg.bind_item_theme(T.STATE_VALUE, T.WARN_TEXT_THEME)
            dpg.bind_item_theme(T.FAULT_VALUE, T.GOOD_TEXT_THEME)
        else:
            dpg.bind_item_theme(T.STATE_VALUE, T.GOOD_TEXT_THEME)
            dpg.bind_item_theme(T.FAULT_VALUE, T.GOOD_TEXT_THEME)

        dpg.bind_item_theme(T.DRIVER_VALUE, T.GOOD_TEXT_THEME if enabled else T.MUTED_TEXT_THEME)
        self._append_history(int(fields["position_steps"]), int(fields["target_steps"]))

    def _append_history(self, position, target):
        now = time.monotonic()
        if self.history_start is None:
            self.history_start = now

        elapsed = now - self.history_start
        self.history.append((elapsed, position, target))
        xs = [point[0] for point in self.history]
        positions = [point[1] for point in self.history]
        targets = [point[2] for point in self.history]
        dpg.set_value(T.POS_SERIES, [xs, positions])
        dpg.set_value(T.TARGET_SERIES, [xs, targets])

        if len(xs) >= 2:
            dpg.set_axis_limits(T.POS_PLOT_X, max(0.0, xs[-1] - 30.0), max(30.0, xs[-1] + 1.0))
            low = min(positions + targets)
            high = max(positions + targets)
            padding = max(100, int((high - low) * 0.12))
            dpg.set_axis_limits(T.POS_PLOT_Y, low - padding, high + padding)

    def _set_connected(self, connected):
        if connected:
            dpg.set_value(T.CONNECTION_TEXT, f"Connected to {self.connected_port} at {BAUD_RATE}")
            dpg.bind_item_theme(T.CONNECTION_TEXT, T.GOOD_TEXT_THEME)
        else:
            dpg.set_value(T.CONNECTION_TEXT, "Disconnected")
            dpg.bind_item_theme(T.CONNECTION_TEXT, T.MUTED_TEXT_THEME)
            dpg.set_value(T.ENABLE_DRIVER, False)
            dpg.set_value(T.STATE_VALUE, "--")
            dpg.set_value(T.DRIVER_VALUE, "--")
            dpg.set_value(T.POSITION_VALUE, "--")
            dpg.set_value(T.TARGET_VALUE, "--")
            dpg.set_value(T.SPEED_VALUE, "--")
            dpg.set_value(T.FAULT_VALUE, "--")
            dpg.set_value(T.TICK_VALUE, "--")
            dpg.set_value(T.QUEUE_VALUE, "--")
            dpg.set_value(T.QUEUE_BAR, 0.0)
            dpg.configure_item(T.QUEUE_BAR, overlay="--")

        dpg.configure_item(T.PORT, enabled=not connected)
        dpg.configure_item(T.CONNECT, enabled=not connected)
        dpg.configure_item(T.DISCONNECT, enabled=connected)
        for tag in COMMAND_WIDGETS:
            if dpg.does_item_exist(tag):
                dpg.configure_item(tag, enabled=connected)

    def _append_log(self, line):
        timestamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"{timestamp}  {line}")
        if dpg.does_item_exist(T.LOG):
            dpg.set_value(T.LOG, "\n".join(self.log_lines))

    def _show_error(self, title, body):
        self._append_log(f"! {title}: {body}")
        if not dpg.is_dearpygui_running():
            return
        if dpg.does_item_exist(T.ERROR_MODAL):
            dpg.delete_item(T.ERROR_MODAL)

        viewport_width = dpg.get_viewport_client_width()
        viewport_height = dpg.get_viewport_client_height()
        width = 460
        height = 150
        pos = (max(0, (viewport_width - width) // 2), max(0, (viewport_height - height) // 2))
        with dpg.window(label=title, tag=T.ERROR_MODAL, modal=True, no_resize=True, width=width, height=height, pos=pos):
            dpg.add_text(body, wrap=420)
            dpg.add_button(label="OK", width=90, callback=lambda sender, app_data, user_data: dpg.delete_item(T.ERROR_MODAL))

    def _on_exit(self, sender=None, app_data=None, user_data=None):
        self.worker.disconnect()


if __name__ == "__main__":
    DearStepperApp().run()
