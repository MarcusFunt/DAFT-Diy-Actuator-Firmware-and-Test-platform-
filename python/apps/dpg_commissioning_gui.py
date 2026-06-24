from __future__ import annotations

import queue
import threading
import time
from pathlib import Path

import dearpygui.dearpygui as dpg

from daft_protocol import CONFIG_FIELDS, ControlMode, DaftClient, SerialTransport, field_by_name, list_serial_ports


VIEWPORT_WIDTH = 1180
VIEWPORT_HEIGHT = 760
TOP_PANEL_HEIGHT = 280
MOTION_PANEL_HEIGHT = 276


class Worker:
    def __init__(self, events):
        self.events = events
        self.transport: SerialTransport | None = None
        self.client: DaftClient | None = None
        self.thread: threading.Thread | None = None
        self.stop = threading.Event()

    def connect(self, port: str) -> None:
        self.disconnect()
        self.transport = SerialTransport(port, timeout=0.08)
        self.transport.open()
        self.client = DaftClient(self.transport)
        self.stop.clear()
        self.thread = threading.Thread(target=self._poll, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.transport:
            self.transport.close()
        self.transport = None
        self.client = None

    def call(self, name: str, *args):
        if self.client is None:
            self.events.put(("log", "not connected"))
            return
        try:
            result = getattr(self.client, name)(*args)
            self.events.put(("result", name, result))
        except Exception as exc:
            self.events.put(("log", f"{name}: {exc}"))

    def _poll(self):
        while not self.stop.is_set():
            if self.client is None:
                return
            try:
                self.call("status")
                time.sleep(0.25)
            except Exception as exc:
                self.events.put(("log", f"poll: {exc}"))
                time.sleep(0.5)


class App:
    def __init__(self):
        self.events = queue.Queue()
        self.worker = Worker(self.events)
        self.log: list[str] = []
        self.use_system_fonts = False
        self.connected_only_items: list[int | str] = []

    def run(self) -> None:
        dpg.create_context()
        self._configure_style()
        with dpg.window(tag="main", label="DAFT Commissioning", width=VIEWPORT_WIDTH, height=VIEWPORT_HEIGHT):
            self._build()
        dpg.create_viewport(title="DAFT Commissioning", width=VIEWPORT_WIDTH, height=VIEWPORT_HEIGHT)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)
        dpg.set_exit_callback(lambda: self.worker.disconnect())
        self.refresh_ports()
        while dpg.is_dearpygui_running():
            self._drain_events()
            dpg.render_dearpygui_frame()
        self.worker.disconnect()
        dpg.destroy_context()

    def _configure_style(self) -> None:
        font_path = Path("C:/Windows/Fonts/segoeui.ttf")
        bold_font_path = Path("C:/Windows/Fonts/segoeuib.ttf")
        if font_path.exists():
            with dpg.font_registry():
                dpg.add_font(str(font_path), 17, tag="ui_font")
                dpg.add_font(str(bold_font_path if bold_font_path.exists() else font_path), 22, tag="title_font")
                dpg.add_font(str(bold_font_path if bold_font_path.exists() else font_path), 18, tag="section_font")
            dpg.bind_font("ui_font")
            self.use_system_fonts = True

        with dpg.theme(tag="app_theme"):
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (21, 24, 29, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (28, 32, 38, 255))
                dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (28, 32, 38, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Border, (66, 76, 89, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (37, 43, 51, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (47, 55, 66, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (56, 66, 79, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (45, 54, 66, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (58, 70, 86, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (72, 86, 104, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (34, 111, 159, 255))
                dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (45, 132, 186, 255))
                dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (50, 147, 207, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Tab, (39, 47, 57, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (45, 132, 186, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TabActive, (34, 111, 159, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (233, 238, 244, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (152, 163, 176, 255))
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 14, 14)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 7)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 8)
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 8)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
                dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 5)
                dpg.add_theme_style(dpg.mvStyleVar_DisabledAlpha, 0.42)

        with dpg.theme(tag="danger_button_theme"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (145, 42, 49, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (181, 51, 59, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (111, 31, 37, 255))

        with dpg.theme(tag="primary_button_theme"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (34, 111, 159, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (45, 132, 186, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (24, 85, 126, 255))

        dpg.bind_theme("app_theme")

    def _build(self):
        with dpg.group(horizontal=True):
            with dpg.group(width=360):
                title = dpg.add_text("DAFT Commissioning")
                self._bind_font(title, "title_font")
                dpg.add_text("Single-axis actuator setup and motion test", color=(157, 169, 184))
            with dpg.group(horizontal=True):
                dpg.add_combo([], tag="port", width=180)
                dpg.add_button(label="Refresh", width=86, callback=self.refresh_ports)
                connect_button = dpg.add_button(label="Connect", width=92, callback=self.connect)
                dpg.bind_item_theme(connect_button, "primary_button_theme")
                self._connected_button(label="Disconnect", width=110, callback=self.disconnect)
                dpg.add_text("Disconnected", tag="header_connection_status", color=(238, 194, 117))

        dpg.add_spacer(height=4)

        with dpg.group(horizontal=True):
            with dpg.child_window(label="Telemetry", width=520, height=TOP_PANEL_HEIGHT, border=True, no_scrollbar=True):
                heading = dpg.add_text("Live Status", color=(201, 211, 223))
                self._bind_font(heading, "section_font")
                with dpg.table(header_row=False, width=-1, borders_innerH=True, borders_outerH=False):
                    dpg.add_table_column(width_fixed=True, init_width_or_weight=108)
                    dpg.add_table_column(width_fixed=True, init_width_or_weight=142)
                    dpg.add_table_column(width_fixed=True, init_width_or_weight=84)
                    dpg.add_table_column()
                    self._status_grid_row("Connection", "connection_status", "Enabled", "enabled")
                    self._status_grid_row("State", "state", "Mode", "mode")
                    self._status_grid_row("Position", "position", "Target", "target")
                    self._status_grid_row("Speed", "speed", "Faults", "faults")

            with dpg.child_window(label="Commands", width=-1, height=TOP_PANEL_HEIGHT, border=True, no_scrollbar=True):
                heading = dpg.add_text("Device Commands", color=(201, 211, 223))
                self._bind_font(heading, "section_font")
                dpg.add_text("Requests are sent to the selected serial port when connected.", color=(157, 169, 184))
                dpg.add_text("Disconnected. Connect to enable device actions.", tag="action_hint", color=(238, 194, 117))
                dpg.add_spacer(height=6)
                with dpg.group(horizontal=True):
                    self._connected_button(label="Identity", width=120, callback=lambda: self.worker.call("identity"))
                    self._connected_button(label="Faults", width=120, callback=lambda: self.worker.call("faults"))
                    self._connected_button(label="Counters", width=120, callback=lambda: self.worker.call("counters"))
                    self._connected_button(label="Driver Status", width=136, callback=lambda: self.worker.call("driver_status"))
                dpg.add_spacer(height=14)
                heading = dpg.add_text("Safety", color=(201, 211, 223))
                self._bind_font(heading, "section_font")
                with dpg.group(horizontal=True):
                    self._connected_button(label="Ramp Stop", width=120, callback=lambda: self.worker.call("ramp_stop", 1500))
                    self._connected_button(label="E-STOP", width=120, callback=lambda: self.worker.call("estop"), theme="danger_button_theme")

        with dpg.tab_bar():
            with dpg.tab(label="Motion"):
                with dpg.group(horizontal=True):
                    with dpg.child_window(width=520, height=MOTION_PANEL_HEIGHT, border=True):
                        heading = dpg.add_text("Position Move", color=(201, 211, 223))
                        self._bind_font(heading, "section_font")
                        self._form_input("Target / delta steps", "move_steps", 1000)
                        self._form_input("Velocity steps/s", "move_speed", 2000)
                        self._form_input("Acceleration steps/s^2", "move_accel", 2000)
                        dpg.add_spacer(height=6)
                        with dpg.group(horizontal=True):
                            self._connected_button(label="Move Absolute", width=138, callback=self.move_abs)
                            self._connected_button(label="Move Relative", width=138, callback=self.move_rel)

                    with dpg.child_window(width=-1, height=MOTION_PANEL_HEIGHT, border=True):
                        heading = dpg.add_text("Drive Mode", color=(201, 211, 223))
                        self._bind_font(heading, "section_font")
                        with dpg.group(horizontal=True):
                            self._connected_button(label="Enable Position", width=142, callback=lambda: self.worker.call("set_mode", ControlMode.POSITION))
                            self._connected_button(label="Velocity Mode", width=142, callback=lambda: self.worker.call("set_mode", ControlMode.VELOCITY))
                            self._connected_button(label="Disable Driver", width=142, callback=lambda: self.worker.call("set_mode", ControlMode.DISABLED))
                        dpg.add_spacer(height=14)
                        self._form_input("Run velocity steps/s", "run_speed", 1000)
                        dpg.add_spacer(height=6)
                        with dpg.group(horizontal=True):
                            self._connected_button(label="Run Velocity", width=138, callback=self.run_velocity)
                            self._connected_button(label="Ramp Stop", width=120, callback=lambda: self.worker.call("ramp_stop", 1500))
                            self._connected_button(label="E-STOP", width=120, callback=lambda: self.worker.call("estop"), theme="danger_button_theme")

            with dpg.tab(label="Configuration"):
                with dpg.child_window(width=620, height=220, border=True):
                    heading = dpg.add_text("Configuration Field", color=(201, 211, 223))
                    self._bind_font(heading, "section_font")
                    with dpg.table(header_row=False, width=-1):
                        dpg.add_table_column(width_fixed=True, init_width_or_weight=128)
                        dpg.add_table_column()
                        with dpg.table_row():
                            dpg.add_text("Field")
                            dpg.add_combo([info.name for info in CONFIG_FIELDS], tag="cfg_field", width=300, default_value="run_current_ma")
                        with dpg.table_row():
                            dpg.add_text("Value")
                            dpg.add_input_int(tag="cfg_value", width=180, default_value=800)
                    dpg.add_spacer(height=8)
                    with dpg.group(horizontal=True):
                        self._connected_button(label="Read Field", width=116, callback=self.config_read)
                        self._connected_button(label="Stage Field", width=116, callback=self.config_stage)
                        self._connected_button(label="Apply", width=92, callback=lambda: self.worker.call("config_apply"))
                        self._connected_button(label="Save", width=92, callback=lambda: self.worker.call("config_save"))
                        self._connected_button(label="Reset Defaults", width=142, callback=lambda: self.worker.call("config_reset"))

            with dpg.tab(label="Log"):
                dpg.add_input_text(tag="log", multiline=True, readonly=True, width=-1, height=390)

        self._set_connected(False)

    def _status_grid_row(self, label_a: str, tag_a: str, label_b: str, tag_b: str) -> None:
        with dpg.table_row():
            dpg.add_text(label_a, color=(157, 169, 184))
            dpg.add_text("Disconnected" if tag_a == "connection_status" else "--", tag=tag_a)
            dpg.add_text(label_b, color=(157, 169, 184))
            dpg.add_text("--", tag=tag_b)

    def _form_input(self, label: str, tag: str, default_value: int) -> None:
        with dpg.table(header_row=False, width=-1):
            dpg.add_table_column(width_fixed=True, init_width_or_weight=184)
            dpg.add_table_column(width_fixed=True, init_width_or_weight=200)
            with dpg.table_row():
                dpg.add_text(label, color=(157, 169, 184))
                dpg.add_input_int(tag=tag, width=180, default_value=default_value, step=100, step_fast=1000)

    def _bind_font(self, item: int | str, font_tag: str) -> None:
        if self.use_system_fonts and dpg.does_item_exist(font_tag):
            dpg.bind_item_font(item, font_tag)

    def _connected_button(self, *, label: str, width: int, callback, theme: str | None = None) -> int | str:
        button = dpg.add_button(label=label, width=width, callback=callback)
        if theme:
            dpg.bind_item_theme(button, theme)
        self.connected_only_items.append(button)
        return button

    def _set_connected(self, connected: bool) -> None:
        for item in self.connected_only_items:
            if connected:
                dpg.enable_item(item)
            else:
                dpg.disable_item(item)
        if dpg.does_item_exist("action_hint"):
            dpg.set_value(
                "action_hint",
                "Connected. Device commands are enabled." if connected else "Disconnected. Connect to enable device actions.",
            )
            dpg.configure_item("action_hint", color=(134, 197, 141) if connected else (238, 194, 117))
        if dpg.does_item_exist("header_connection_status"):
            dpg.set_value("header_connection_status", "Connected" if connected else "Disconnected")
            dpg.configure_item("header_connection_status", color=(134, 197, 141) if connected else (238, 194, 117))

    def refresh_ports(self):
        ports = list_serial_ports()
        dpg.configure_item("port", items=ports)
        if ports:
            dpg.set_value("port", ports[0])

    def connect(self):
        port = dpg.get_value("port")
        if not port:
            self._log("no port selected")
            return
        try:
            self.worker.connect(port)
            self._log(f"connected {port}")
            dpg.set_value("connection_status", f"Connected: {port}")
            self._set_connected(True)
        except Exception as exc:
            self._log(f"connect: {exc}")
            dpg.set_value("connection_status", "Disconnected")
            self._set_connected(False)

    def disconnect(self):
        self.worker.disconnect()
        dpg.set_value("connection_status", "Disconnected")
        self._set_connected(False)
        self._log("disconnected")

    def move_abs(self):
        self.worker.call("set_mode", ControlMode.POSITION)
        self.worker.call("move_abs", dpg.get_value("move_steps"), dpg.get_value("move_speed"), dpg.get_value("move_accel"))

    def move_rel(self):
        self.worker.call("set_mode", ControlMode.POSITION)
        self.worker.call("move_rel", dpg.get_value("move_steps"), dpg.get_value("move_speed"), dpg.get_value("move_accel"))

    def run_velocity(self):
        self.worker.call("set_mode", ControlMode.VELOCITY)
        self.worker.call("run_velocity", dpg.get_value("run_speed"), dpg.get_value("move_accel"))

    def config_read(self):
        self.worker.call("config_get", int(field_by_name(dpg.get_value("cfg_field"))))

    def config_stage(self):
        self.worker.call("config_stage", int(field_by_name(dpg.get_value("cfg_field"))), dpg.get_value("cfg_value"))

    def _drain_events(self):
        while not self.events.empty():
            item = self.events.get_nowait()
            if item[0] == "log":
                self._log(item[1])
            elif item[0] == "result":
                self._handle_result(item[1], item[2])

    def _handle_result(self, name: str, result):
        if name == "status":
            dpg.set_value("state", result["state"])
            dpg.set_value("mode", result["control_mode"])
            dpg.set_value("enabled", str(result["driver_enabled"]))
            dpg.set_value("position", str(result["position_steps"]))
            dpg.set_value("target", str(result["target_steps"]))
            dpg.set_value("speed", str(result["speed_sps"]))
            dpg.set_value("faults", f"0x{result['faults']:08X}")
        else:
            self._log(f"{name}: {result}")

    def _log(self, line: str):
        self.log.append(f"{time.strftime('%H:%M:%S')}  {line}")
        self.log = self.log[-250:]
        if dpg.does_item_exist("log"):
            dpg.set_value("log", "\n".join(self.log))


if __name__ == "__main__":
    App().run()
