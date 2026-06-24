from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

from daft_protocol import CONFIG_FIELDS, ConfigField, ControlMode, DaftClient, MsgId, SerialTransport, field_by_name, list_serial_ports


HOST_TIMEOUT_FAULT = 1 << 9


def print_json(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True))


def with_client(port: str, fn):
    with SerialTransport(port) as transport:
        client = DaftClient(transport)
        return fn(client)


def _bench_default_log() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware_logs") / f"bench_{stamp}.jsonl"


def _bench_default_report(log_path: Path) -> Path:
    return log_path.with_suffix(".report.json")


def _parse_int_list(value: str | None, fallback: int) -> list[int]:
    if value is None or value.strip() == "":
        return [fallback]
    values = [int(part.strip(), 0) for part in value.split(",") if part.strip()]
    if not values:
        raise ValueError("list argument must contain at least one integer")
    return values


def _write_jsonl(handle, event: str, **fields) -> None:
    payload = {"t": time.time(), "event": event, **fields}
    handle.write(json.dumps(payload, sort_keys=True) + "\n")
    handle.flush()


def _bench_open(port: str) -> tuple[SerialTransport, DaftClient]:
    transport = SerialTransport(port, packet_timeout=3.0)
    transport.open()
    return transport, DaftClient(transport)


def _bench_reopen(transport: SerialTransport, port: str, delay_s: float = 2.0) -> tuple[SerialTransport, DaftClient]:
    transport.close()
    time.sleep(delay_s)
    deadline = time.monotonic() + 10.0
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return _bench_open(port)
        except Exception as exc:
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f"could not reopen {port} after reboot: {last_error}")


def _bench_wait_idle(client: DaftClient, log_handle, label: str, timeout_s: float) -> dict[str, int | str | bool]:
    deadline = time.monotonic() + timeout_s
    last_status: dict[str, int | str | bool] | None = None
    while time.monotonic() < deadline:
        last_status = client.status()
        _write_jsonl(log_handle, "status", label=label, status=last_status)
        if last_status["state"] in ("IDLE", "DISABLED", "FAULTED"):
            return last_status
        time.sleep(0.05)
    raise TimeoutError(f"{label} did not settle; last status={last_status}")


def run_bench(args) -> dict[str, object]:
    log_path = Path(args.log_jsonl) if args.log_jsonl else _bench_default_log()
    report_path = Path(args.report_json) if args.report_json else _bench_default_report(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    transport: SerialTransport | None = None
    client: DaftClient | None = None
    original_host_timeout: int | None = None
    original_direction_inverted: int | None = None
    final_status: dict[str, object] | None = None
    initial_counters: dict[str, int] | None = None
    initial_faults: int | None = None
    observed_faults_or = 0
    event_cursor = 0
    start_monotonic = time.monotonic()
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    report: dict[str, object] = {
        "ok": False,
        "started_at": started_at,
        "ended_at": None,
        "duration_s": 0.0,
        "log_jsonl": str(log_path),
        "report_json": str(report_path),
        "identity": None,
        "movements_completed": {"position": 0, "velocity_runs": 0, "total": 0},
        "max_counter_deltas": {},
        "observed_faults_or": 0,
        "fault_delta_or": 0,
        "uart_status": {"initial": None, "final": None, "ever_failed": False},
        "final_state": None,
        "final_status": None,
        "config_restored": False,
        "config_restore": {"host_timeout_ms": False, "direction_inverted": False},
        "telemetry_packets_seen": 0,
        "events_seen": 0,
        "event_counts": {},
        "transport_stats": {},
        "error": None,
    }

    speeds = _parse_int_list(args.speeds, args.speed)
    accels = _parse_int_list(args.accels, args.accel)
    telemetry_intervals = _parse_int_list(args.telemetry_intervals_ms, args.telemetry_interval_ms)

    def observe_faults(faults_value: int) -> None:
        nonlocal initial_faults, observed_faults_or
        if initial_faults is None:
            initial_faults = faults_value
        observed_faults_or |= faults_value
        report["observed_faults_or"] = observed_faults_or
        report["fault_delta_or"] = observed_faults_or & ~initial_faults

    def observe_status(status: dict[str, object]) -> None:
        observe_faults(int(status.get("faults", 0)))

    def observe_counters(counters: dict[str, int]) -> None:
        nonlocal initial_counters
        if initial_counters is None:
            initial_counters = dict(counters)
        max_deltas = dict(report["max_counter_deltas"])
        for key, value in counters.items():
            if key == "uptime_ms":
                continue
            delta = max(0, int(value) - int(initial_counters.get(key, 0)))
            max_deltas[key] = max(int(max_deltas.get(key, 0)), delta)
        report["max_counter_deltas"] = max_deltas

    def observe_driver(driver: dict[str, object], label: str) -> None:
        uart_status = dict(report["uart_status"])
        summary = {"configured": bool(driver.get("configured")), "uart_ok": bool(driver.get("uart_ok"))}
        if label == "initial":
            uart_status["initial"] = summary
        if label == "final":
            uart_status["final"] = summary
        if not summary["uart_ok"]:
            uart_status["ever_failed"] = True
        report["uart_status"] = uart_status

    def note_position_move() -> None:
        movements = dict(report["movements_completed"])
        movements["position"] = int(movements["position"]) + 1
        movements["total"] = int(movements["total"]) + 1
        report["movements_completed"] = movements

    def note_velocity_run() -> None:
        movements = dict(report["movements_completed"])
        movements["velocity_runs"] = int(movements["velocity_runs"]) + 1
        movements["total"] = int(movements["total"]) + 1
        report["movements_completed"] = movements

    def flush_events(log_handle) -> None:
        nonlocal event_cursor
        if client is None:
            return
        counts = dict(report["event_counts"])
        processed = 0
        while event_cursor < len(client.events):
            event = client.events[event_cursor]
            _write_jsonl(log_handle, "event_packet", event_packet=event)
            name = str(event.get("event", "UNKNOWN"))
            counts[name] = int(counts.get(name, 0)) + 1
            event_cursor += 1
            processed += 1
        report["events_seen"] = int(report["events_seen"]) + processed
        report["event_counts"] = counts

    def write_report() -> None:
        report["ended_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        report["duration_s"] = round(time.monotonic() - start_monotonic, 3)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    with log_path.open("w", encoding="utf-8") as log_handle:
        try:
            transport, client = _bench_open(args.port)
            _write_jsonl(log_handle, "start", port=args.port, args=vars(args))

            identity = client.identity()
            status = client.status()
            driver = client.driver_status()
            counters = client.counters()
            faults = client.faults()
            report["identity"] = identity
            observe_status(status)
            observe_faults(int(faults["faults"]))
            observe_counters(counters)
            observe_driver(driver, "initial")
            flush_events(log_handle)
            _write_jsonl(log_handle, "snapshot", label="initial", identity=identity, status=status, driver=driver, counters=counters, faults=faults)
            if not driver["configured"] or not driver["uart_ok"]:
                raise RuntimeError(f"driver is not healthy: {driver}")
            if status["faults"]:
                client.clear_faults()
                flush_events(log_handle)

            original_host_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            original_direction_inverted = int(client.config_get(int(ConfigField.DIRECTION_INVERTED))["value"])
            client.set_telemetry_rate(telemetry_intervals[0])
            client.set_mode(ControlMode.POSITION)
            flush_events(log_handle)

            duration_deadline = (
                time.monotonic() + (args.duration_min * 60.0) if args.duration_min > 0 else None
            )
            cycle = 0
            while cycle < args.cycles or (duration_deadline is not None and time.monotonic() < duration_deadline):
                speed = speeds[cycle % len(speeds)]
                accel = accels[cycle % len(accels)]
                telemetry_interval = telemetry_intervals[cycle % len(telemetry_intervals)]
                client.set_telemetry_rate(telemetry_interval)
                if args.exercise_direction_inversion:
                    client.config_stage(int(ConfigField.DIRECTION_INVERTED), cycle % 2)
                    client.config_apply()
                flush_events(log_handle)

                settle_timeout = max(5.0, abs(args.steps) / max(speed, 1) + 3.0)
                client.move_rel(args.steps, speed, accel)
                status = _bench_wait_idle(client, log_handle, f"move_positive_{cycle + 1}", settle_timeout)
                observe_status(status)
                note_position_move()
                flush_events(log_handle)
                client.move_rel(-args.steps, speed, accel)
                status = _bench_wait_idle(client, log_handle, f"move_negative_{cycle + 1}", settle_timeout)
                observe_status(status)
                note_position_move()
                flush_events(log_handle)
                cycle += 1

            client.set_mode(ControlMode.VELOCITY)
            client.run_velocity(speeds[0], accels[0])
            time.sleep(args.velocity_run_s)
            client.ramp_stop(accels[0])
            _bench_wait_idle(client, log_handle, "velocity_positive_stop", max(3.0, args.velocity_run_s + 2.0))
            note_velocity_run()
            client.run_velocity(-speeds[0], accels[0])
            time.sleep(args.velocity_run_s)
            client.ramp_stop(accels[0])
            status = _bench_wait_idle(client, log_handle, "velocity_negative_stop", max(3.0, args.velocity_run_s + 2.0))
            observe_status(status)
            note_velocity_run()
            flush_events(log_handle)

            for reboot_cycle in range(args.config_save_reboot_cycles):
                if args.exercise_direction_inversion:
                    desired_direction = (reboot_cycle + 1) % 2
                    client.config_stage(int(ConfigField.DIRECTION_INVERTED), desired_direction)
                    client.config_apply()
                client.config_save()
                client.acked(MsgId.REBOOT)
                transport, client = _bench_reopen(transport, args.port)
                event_cursor = 0
                if args.exercise_direction_inversion:
                    persisted_direction = int(client.config_get(int(ConfigField.DIRECTION_INVERTED))["value"])
                    _write_jsonl(log_handle, "config_persisted", field="direction_inverted", value=persisted_direction)
                    if persisted_direction != desired_direction:
                        raise RuntimeError(
                            f"direction inversion did not persist: {persisted_direction} != {desired_direction}"
                        )
                flush_events(log_handle)

            test_timeout_ms = max(100, min(args.host_timeout_ms, 1000))
            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), test_timeout_ms)
            client.config_apply()
            client.config_save()
            client.acked(MsgId.REBOOT)
            transport, client = _bench_reopen(transport, args.port)
            event_cursor = 0
            persisted_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            _write_jsonl(log_handle, "config_persisted", host_timeout_ms=persisted_timeout)
            if persisted_timeout != test_timeout_ms:
                raise RuntimeError(f"host timeout did not persist: {persisted_timeout} != {test_timeout_ms}")
            flush_events(log_handle)

            client.clear_faults()
            client.set_mode(ControlMode.VELOCITY)
            client.run_velocity(speeds[0], accels[0])
            time.sleep((test_timeout_ms / 1000.0) + 0.4)
            timeout_status = client.status()
            observe_status(timeout_status)
            _write_jsonl(log_handle, "status", label="after_host_timeout", status=timeout_status)
            if not (int(timeout_status["faults"]) & HOST_TIMEOUT_FAULT):
                raise RuntimeError(f"host timeout fault did not set: {timeout_status}")
            client.ramp_stop(accels[0])
            _bench_wait_idle(client, log_handle, "timeout_stop", 3.0)
            client.clear_faults()
            flush_events(log_handle)

            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), original_host_timeout)
            client.config_apply()
            client.config_save()
            client.acked(MsgId.REBOOT)
            transport, client = _bench_reopen(transport, args.port)
            event_cursor = 0
            restored_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            _write_jsonl(log_handle, "config_restored", host_timeout_ms=restored_timeout)
            if restored_timeout != original_host_timeout:
                raise RuntimeError(f"host timeout restore failed: {restored_timeout} != {original_host_timeout}")
            restore = dict(report["config_restore"])
            restore["host_timeout_ms"] = True

            if original_direction_inverted is not None:
                restored_direction = int(client.config_get(int(ConfigField.DIRECTION_INVERTED))["value"])
                if restored_direction != original_direction_inverted:
                    client.config_stage(int(ConfigField.DIRECTION_INVERTED), original_direction_inverted)
                    client.config_apply()
                    client.config_save()
                    client.acked(MsgId.REBOOT)
                    transport, client = _bench_reopen(transport, args.port)
                    event_cursor = 0
                    restored_direction = int(client.config_get(int(ConfigField.DIRECTION_INVERTED))["value"])
                restore["direction_inverted"] = restored_direction == original_direction_inverted
                _write_jsonl(log_handle, "config_restored", direction_inverted=restored_direction)
                if restored_direction != original_direction_inverted:
                    raise RuntimeError(
                        f"direction inversion restore failed: {restored_direction} != {original_direction_inverted}"
                    )
            report["config_restore"] = restore
            report["config_restored"] = all(bool(value) for value in restore.values())
            flush_events(log_handle)

            client.clear_faults()
            client.set_mode(ControlMode.DISABLED)
            final_status = client.status()
            final_faults = client.faults()
            final_counters = client.counters()
            final_driver = client.driver_status()
            observe_status(final_status)
            observe_faults(int(final_faults["faults"]))
            observe_counters(final_counters)
            observe_driver(final_driver, "final")
            flush_events(log_handle)
            report["final_state"] = final_status["state"]
            report["final_status"] = final_status
            report["transport_stats"] = transport.stats.__dict__
            report["telemetry_packets_seen"] = len(client.telemetry)
            _write_jsonl(log_handle, "snapshot", label="final", status=final_status, faults=final_faults, counters=final_counters, driver=final_driver, transport_stats=transport.stats.__dict__)
            if final_status["state"] != "DISABLED" or final_status["faults"] != 0:
                raise RuntimeError(f"final state is not clean: {final_status}")
            report["ok"] = bool(report["config_restored"]) and not bool(dict(report["uart_status"])["ever_failed"])
        except Exception as exc:
            report["ok"] = False
            report["error"] = str(exc)
            _write_jsonl(log_handle, "failure", error=str(exc))
        finally:
            if client is not None:
                try:
                    client.ramp_stop(args.accel)
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="ramp_stop", error=str(exc))
                try:
                    client.clear_faults()
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="clear_faults", error=str(exc))
                if original_host_timeout is not None:
                    try:
                        current_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
                        if current_timeout != original_host_timeout:
                            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), original_host_timeout)
                            client.config_apply()
                            client.config_save()
                    except Exception as exc:
                        _write_jsonl(log_handle, "cleanup_error", action="restore_host_timeout", error=str(exc))
                if original_direction_inverted is not None:
                    try:
                        current_direction = int(client.config_get(int(ConfigField.DIRECTION_INVERTED))["value"])
                        if current_direction != original_direction_inverted:
                            client.config_stage(int(ConfigField.DIRECTION_INVERTED), original_direction_inverted)
                            client.config_apply()
                            client.config_save()
                    except Exception as exc:
                        _write_jsonl(log_handle, "cleanup_error", action="restore_direction_inverted", error=str(exc))
                try:
                    client.set_mode(ControlMode.DISABLED)
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="disable", error=str(exc))
            if transport is not None:
                report["transport_stats"] = transport.stats.__dict__
                _write_jsonl(log_handle, "transport_stats", stats=transport.stats.__dict__)
                transport.close()
            if client is not None:
                report["telemetry_packets_seen"] = len(client.telemetry)
                flush_events(log_handle)
            write_report()
            _write_jsonl(log_handle, "report", report=report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="daft", description="DAFT Binary v2 CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")

    for name in ("ping", "identity", "status", "faults", "counters", "driver-status", "stop", "estop", "clear-faults"):
        p = sub.add_parser(name)
        p.add_argument("port")

    p = sub.add_parser("ramp-stop")
    p.add_argument("port")
    p.add_argument("--accel", type=int, default=1500)

    p = sub.add_parser("mode")
    p.add_argument("port")
    p.add_argument("mode", choices=("disabled", "position", "velocity", "commissioning"))

    p = sub.add_parser("move-rel")
    p.add_argument("port")
    p.add_argument("--steps", type=int, required=True)
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("move-abs")
    p.add_argument("port")
    p.add_argument("--target", type=int, required=True)
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("run-vel")
    p.add_argument("port")
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("telemetry")
    p.add_argument("port")
    p.add_argument("--interval-ms", type=int, required=True)

    p = sub.add_parser("bench")
    p.add_argument("port")
    p.add_argument("--cycles", type=int, default=2)
    p.add_argument("--steps", type=int, default=1000)
    p.add_argument("--speed", type=int, default=1000)
    p.add_argument("--accel", type=int, default=2000)
    p.add_argument("--telemetry-interval-ms", type=int, default=100)
    p.add_argument("--duration-min", type=float, default=0.0)
    p.add_argument("--speeds", help="Comma-separated speed sweep in steps/s")
    p.add_argument("--accels", help="Comma-separated acceleration sweep in steps/s^2")
    p.add_argument("--telemetry-intervals-ms", help="Comma-separated telemetry interval sweep in ms")
    p.add_argument("--exercise-direction-inversion", action="store_true")
    p.add_argument("--config-save-reboot-cycles", type=int, default=0)
    p.add_argument("--velocity-run-s", type=float, default=0.8)
    p.add_argument("--host-timeout-ms", type=int, default=250)
    p.add_argument("--log-jsonl")
    p.add_argument("--report-json")

    config = sub.add_parser("config")
    config_sub = config.add_subparsers(dest="config_cmd", required=True)
    p = config_sub.add_parser("fields")
    p = config_sub.add_parser("get")
    p.add_argument("port")
    p.add_argument("--field", default="all")
    p = config_sub.add_parser("stage")
    p.add_argument("port")
    p.add_argument("field")
    p.add_argument("value", type=int)
    for name in ("apply", "save", "reset"):
        p = config_sub.add_parser(name)
        p.add_argument("port")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.cmd == "scan":
        print_json({"ports": list_serial_ports()})
        return 0

    if args.cmd == "ping":
        print_json(with_client(args.port, lambda c: {"ok": c.ping()}))
    elif args.cmd == "identity":
        print_json(with_client(args.port, lambda c: c.identity()))
    elif args.cmd == "status":
        print_json(with_client(args.port, lambda c: c.status()))
    elif args.cmd == "faults":
        print_json(with_client(args.port, lambda c: c.faults()))
    elif args.cmd == "counters":
        print_json(with_client(args.port, lambda c: c.counters()))
    elif args.cmd == "driver-status":
        print_json(with_client(args.port, lambda c: c.driver_status()))
    elif args.cmd == "stop":
        print_json(with_client(args.port, lambda c: c.ramp_stop(1500)))
    elif args.cmd == "ramp-stop":
        print_json(with_client(args.port, lambda c: c.ramp_stop(args.accel)))
    elif args.cmd == "estop":
        print_json(with_client(args.port, lambda c: c.estop()))
    elif args.cmd == "clear-faults":
        print_json(with_client(args.port, lambda c: c.clear_faults()))
    elif args.cmd == "mode":
        modes = {
            "disabled": ControlMode.DISABLED,
            "position": ControlMode.POSITION,
            "velocity": ControlMode.VELOCITY,
            "commissioning": ControlMode.COMMISSIONING,
        }
        print_json(with_client(args.port, lambda c: c.set_mode(modes[args.mode])))
    elif args.cmd == "move-rel":
        print_json(with_client(args.port, lambda c: c.move_rel(args.steps, args.speed, args.accel)))
    elif args.cmd == "move-abs":
        print_json(with_client(args.port, lambda c: c.move_abs(args.target, args.speed, args.accel)))
    elif args.cmd == "run-vel":
        print_json(with_client(args.port, lambda c: c.run_velocity(args.speed, args.accel)))
    elif args.cmd == "telemetry":
        print_json(with_client(args.port, lambda c: c.set_telemetry_rate(args.interval_ms)))
    elif args.cmd == "bench":
        report = run_bench(args)
        print_json(report)
        return 0 if report.get("ok") else 1
    elif args.cmd == "config":
        if args.config_cmd == "fields":
            print_json([info.__dict__ | {"field": int(info.field)} for info in CONFIG_FIELDS])
        elif args.config_cmd == "get":
            def get_config(client: DaftClient):
                if args.field == "all":
                    return [client.config_get(int(info.field)) for info in CONFIG_FIELDS]
                return client.config_get(int(field_by_name(args.field)))
            print_json(with_client(args.port, get_config))
        elif args.config_cmd == "stage":
            print_json(with_client(args.port, lambda c: c.config_stage(int(field_by_name(args.field)), args.value)))
        elif args.config_cmd == "apply":
            print_json(with_client(args.port, lambda c: c.config_apply()))
        elif args.config_cmd == "save":
            print_json(with_client(args.port, lambda c: c.config_save()))
        elif args.config_cmd == "reset":
            print_json(with_client(args.port, lambda c: c.config_reset()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
