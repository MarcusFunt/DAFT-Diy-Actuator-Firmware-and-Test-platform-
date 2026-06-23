import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

import daft_protocol as proto


BAUD_RATE = 115200
DEFAULT_TELEM_INTERVAL_MS = 100


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


class StepperUi(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32-C6 TMC2209 Stepper Control")
        self.geometry("760x520")
        self.minsize(680, 460)

        self.event_queue = queue.Queue()
        self.worker = SerialWorker(self.event_queue.put)

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="No status yet")
        self.enabled_var = tk.BooleanVar(value=False)

        self.position_var = tk.StringVar(value="3200")
        self.position_speed_var = tk.StringVar(value="2000")
        self.position_accel_var = tk.StringVar(value="2000")

        self.velocity_speed_var = tk.StringVar(value="1000")
        self.velocity_accel_var = tk.StringVar(value="1500")

        self.current_var = tk.StringVar(value="800")
        self.microsteps_var = tk.StringVar(value="16")

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._process_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        connection = ttk.Frame(self, padding=12)
        connection.grid(row=0, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(connection, text="Connect", command=self.connect).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(connection, text="Disconnect", command=self.disconnect).grid(row=0, column=4)
        ttk.Label(connection, textvariable=self.connection_var).grid(row=1, column=1, sticky="w", padx=8, pady=(8, 0))

        controls = ttk.Frame(self, padding=(12, 0, 12, 12))
        controls.grid(row=1, column=0, sticky="ew")
        controls.columnconfigure((0, 1, 2), weight=1, uniform="controls")

        position = ttk.LabelFrame(controls, text="Position Control", padding=12)
        position.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._entry(position, "Target position", self.position_var, 0)
        self._entry(position, "Max speed", self.position_speed_var, 1)
        self._entry(position, "Acceleration", self.position_accel_var, 2)
        ttk.Button(position, text="Go Position", command=self.go_position).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )

        velocity = ttk.LabelFrame(controls, text="Velocity Control", padding=12)
        velocity.grid(row=0, column=1, sticky="nsew", padx=8)
        self._entry(velocity, "Speed", self.velocity_speed_var, 0)
        self._entry(velocity, "Acceleration", self.velocity_accel_var, 1)
        ttk.Button(velocity, text="Start Velocity", command=self.start_velocity).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )
        ttk.Button(velocity, text="Ramp Stop", command=self.ramp_stop).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        ttk.Button(velocity, text="Emergency Stop", command=self.estop).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )

        config = ttk.LabelFrame(controls, text="Driver", padding=12)
        config.grid(row=0, column=2, sticky="nsew", padx=(8, 0))
        self._entry(config, "RMS current mA", self.current_var, 0)
        self._entry(config, "Microsteps", self.microsteps_var, 1)
        ttk.Button(config, text="Apply Config", command=self.apply_config).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )
        ttk.Checkbutton(config, text="Enable driver", variable=self.enabled_var, command=self.toggle_enable).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )
        ttk.Button(config, text="Zero Position", command=self.zero_position).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        ttk.Button(config, text="Read Status", command=self.request_status).grid(
            row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )

        status = ttk.Frame(self, padding=(12, 0, 12, 12))
        status.grid(row=2, column=0, sticky="nsew")
        status.rowconfigure(1, weight=1)
        status.columnconfigure(0, weight=1)

        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.log = tk.Text(status, height=12, wrap="word", state="disabled")
        self.log.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(status, orient="vertical", command=self.log.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)

    def _entry(self, parent, label, variable, row):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=14).grid(row=row, column=1, sticky="ew", pady=4, padx=(8, 0))

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect(self):
        port_name = self.port_var.get()
        if not port_name:
            messagebox.showerror("No port selected", "Select the ESP32 serial port first.")
            return

        try:
            self.worker.connect(port_name)
            self.connection_var.set(f"Connected to {port_name} at {BAUD_RATE}")
            self._append_log(f"Connected to {port_name}")
            self.send_packet(
                proto.MSG_SET_TELEM_RATE,
                proto.payload_telem_rate(DEFAULT_TELEM_INTERVAL_MS),
                f"SET_TELEM_RATE {DEFAULT_TELEM_INTERVAL_MS} ms",
            )
            self.request_status()
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", str(exc))

    def disconnect(self):
        self.worker.disconnect()
        self.connection_var.set("Disconnected")
        self._append_log("Disconnected")

    def send_packet(self, msg_id, payload=b"", label=None):
        try:
            seq = self.worker.send_packet(msg_id, payload)
        except RuntimeError as exc:
            messagebox.showerror("Not connected", str(exc))
            return None
        except ValueError as exc:
            messagebox.showerror("Invalid command", str(exc))
            return None

        self._append_log(f"> #{seq:05d} {label or proto.command_name(msg_id)}")
        return seq

    def go_position(self):
        position = self._as_int(self.position_var, "target position")
        speed = self._as_int(self.position_speed_var, "position max speed")
        accel = self._as_int(self.position_accel_var, "position acceleration")
        if position is None or speed is None or accel is None:
            return

        try:
            payload = proto.payload_move(position, speed, accel)
        except ValueError as exc:
            messagebox.showerror("Invalid move", str(exc))
            return

        self.send_packet(proto.MSG_MOVE_ABS, payload, f"MOVE_ABS target={position} speed={speed} accel={accel}")

    def start_velocity(self):
        speed = self._as_int(self.velocity_speed_var, "velocity speed")
        accel = self._as_int(self.velocity_accel_var, "velocity acceleration")
        if speed is None or accel is None:
            return

        try:
            payload = proto.payload_run_velocity(speed, accel)
        except ValueError as exc:
            messagebox.showerror("Invalid velocity", str(exc))
            return

        self.send_packet(proto.MSG_RUN_VEL, payload, f"RUN_VEL speed={speed} accel={accel}")

    def ramp_stop(self):
        decel = self._as_int(self.velocity_accel_var, "stop deceleration")
        if decel is None:
            return

        try:
            payload = proto.payload_stop_ramp(decel)
        except ValueError as exc:
            messagebox.showerror("Invalid stop", str(exc))
            return

        self.send_packet(proto.MSG_STOP_RAMP, payload, f"STOP_RAMP decel={decel}")

    def estop(self):
        self.send_packet(proto.MSG_ESTOP)

    def apply_config(self):
        current = self._as_int(self.current_var, "RMS current")
        microsteps = self._as_int(self.microsteps_var, "microsteps")
        if current is None or microsteps is None:
            return

        try:
            payload = proto.payload_driver_config(run_current_ma=current, microsteps=microsteps)
        except ValueError as exc:
            messagebox.showerror("Invalid driver config", str(exc))
            return

        self.send_packet(proto.MSG_DRIVER_CONFIG, payload, f"DRIVER_CONFIG current={current} microsteps={microsteps}")

    def toggle_enable(self):
        self.send_packet(proto.MSG_ENABLE if self.enabled_var.get() else proto.MSG_DISABLE)

    def zero_position(self):
        self.send_packet(proto.MSG_ZERO_POSITION)

    def request_status(self):
        self.send_packet(proto.MSG_GET_STATUS)

    def _as_int(self, variable, label):
        try:
            return int(variable.get())
        except ValueError:
            messagebox.showerror("Invalid value", f"Enter a whole number for {label}.")
            return None

    def _process_events(self):
        while not self.event_queue.empty():
            kind, payload = self.event_queue.get_nowait()
            if kind == "packet":
                self._handle_packet(payload)
            elif kind == "error":
                self._append_log(f"! {payload}")
            elif kind == "disconnected":
                self.connection_var.set("Disconnected")
                self._append_log(f"Disconnected: {payload}")
        self.after(50, self._process_events)

    def _handle_packet(self, packet):
        try:
            if packet.msg_id == proto.MSG_ACK:
                acked_id, _ = proto.parse_ack(packet)
                self._append_log(f"< ACK #{packet.seq:05d} {proto.command_name(acked_id)}")
            elif packet.msg_id == proto.MSG_NACK:
                rejected_id, reason = proto.parse_nack(packet)
                self._append_log(
                    f"< NACK #{packet.seq:05d} {proto.command_name(rejected_id)}: {proto.reason_name(reason)}"
                )
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
        enabled = fields["driver_enabled"]
        self.enabled_var.set(enabled == 1)
        self.status_var.set(
            f"Mode: {fields['motion_state_name']}    Enabled: {enabled}    "
            f"Position: {fields['position_steps']}    Target: {fields['target_steps']}    "
            f"Speed: {fields['speed_sps']}    Faults: 0x{fields['fault_flags']:04X}    "
            f"Queue free: {fields['rx_queue_free']}"
        )

    def _append_log(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _on_close(self):
        self.worker.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = StepperUi()
    app.mainloop()
