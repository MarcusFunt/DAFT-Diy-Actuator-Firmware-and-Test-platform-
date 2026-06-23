from __future__ import annotations

import queue
import threading
import time

import dearpygui.dearpygui as dpg

from daft_protocol import CONFIG_FIELDS, ControlMode, DaftClient, SerialTransport, field_by_name, list_serial_ports


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

    def run(self) -> None:
        dpg.create_context()
        with dpg.window(tag="main", label="DAFT Commissioning", width=1120, height=720):
            self._build()
        dpg.create_viewport(title="DAFT Commissioning", width=1120, height=720)
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

    def _build(self):
        with dpg.group(horizontal=True):
            dpg.add_combo([], tag="port", width=180)
            dpg.add_button(label="Refresh", callback=self.refresh_ports)
            dpg.add_button(label="Connect", callback=self.connect)
            dpg.add_button(label="Disconnect", callback=lambda: self.worker.disconnect())
            dpg.add_button(label="Identity", callback=lambda: self.worker.call("identity"))
            dpg.add_button(label="Faults", callback=lambda: self.worker.call("faults"))
            dpg.add_button(label="Counters", callback=lambda: self.worker.call("counters"))

        with dpg.table(header_row=False, width=-1, borders_innerH=True):
            dpg.add_table_column(width_fixed=True, init_width_or_weight=160)
            dpg.add_table_column()
            for label, tag in (
                ("State", "state"),
                ("Mode", "mode"),
                ("Enabled", "enabled"),
                ("Position", "position"),
                ("Target", "target"),
                ("Speed", "speed"),
                ("Faults", "faults"),
            ):
                with dpg.table_row():
                    dpg.add_text(label)
                    dpg.add_text("--", tag=tag)

        with dpg.tab_bar():
            with dpg.tab(label="Motion"):
                dpg.add_button(label="Position Mode", callback=lambda: self.worker.call("set_mode", ControlMode.POSITION))
                dpg.add_same_line()
                dpg.add_button(label="Velocity Mode", callback=lambda: self.worker.call("set_mode", ControlMode.VELOCITY))
                dpg.add_same_line()
                dpg.add_button(label="Disable", callback=lambda: self.worker.call("set_mode", ControlMode.DISABLED))
                dpg.add_input_int(label="Target / delta steps", tag="move_steps", default_value=1000)
                dpg.add_input_int(label="Velocity steps/s", tag="move_speed", default_value=2000)
                dpg.add_input_int(label="Acceleration steps/s^2", tag="move_accel", default_value=2000)
                dpg.add_button(label="Move absolute", callback=self.move_abs)
                dpg.add_same_line()
                dpg.add_button(label="Move relative", callback=self.move_rel)
                dpg.add_input_int(label="Run velocity steps/s", tag="run_speed", default_value=1000)
                dpg.add_button(label="Run velocity", callback=self.run_velocity)
                dpg.add_same_line()
                dpg.add_button(label="Ramp stop", callback=lambda: self.worker.call("ramp_stop", 1500))
                dpg.add_same_line()
                dpg.add_button(label="E-STOP", callback=lambda: self.worker.call("estop"))

            with dpg.tab(label="Configuration"):
                dpg.add_combo([info.name for info in CONFIG_FIELDS], tag="cfg_field", width=300, default_value="run_current_ma")
                dpg.add_input_int(label="Value", tag="cfg_value", default_value=800)
                dpg.add_button(label="Read field", callback=self.config_read)
                dpg.add_same_line()
                dpg.add_button(label="Stage field", callback=self.config_stage)
                dpg.add_same_line()
                dpg.add_button(label="Apply", callback=lambda: self.worker.call("config_apply"))
                dpg.add_same_line()
                dpg.add_button(label="Save", callback=lambda: self.worker.call("config_save"))
                dpg.add_same_line()
                dpg.add_button(label="Reset defaults", callback=lambda: self.worker.call("config_reset"))
                dpg.add_button(label="Driver status", callback=lambda: self.worker.call("driver_status"))

            with dpg.tab(label="Log"):
                dpg.add_input_text(tag="log", multiline=True, readonly=True, width=-1, height=360)

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
        except Exception as exc:
            self._log(f"connect: {exc}")

    def move_abs(self):
        self.worker.call("move_abs", dpg.get_value("move_steps"), dpg.get_value("move_speed"), dpg.get_value("move_accel"))

    def move_rel(self):
        self.worker.call("move_rel", dpg.get_value("move_steps"), dpg.get_value("move_speed"), dpg.get_value("move_accel"))

    def run_velocity(self):
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
