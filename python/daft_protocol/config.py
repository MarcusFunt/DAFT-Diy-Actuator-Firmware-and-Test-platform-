from __future__ import annotations

from dataclasses import dataclass

from .messages import ConfigField


@dataclass(frozen=True)
class ConfigFieldInfo:
    field: ConfigField
    name: str
    description: str


CONFIG_FIELDS: tuple[ConfigFieldInfo, ...] = (
    ConfigFieldInfo(ConfigField.TELEMETRY_INTERVAL_MS, "telemetry_interval_ms", "STATUS telemetry interval in ms"),
    ConfigFieldInfo(ConfigField.SOFT_LIMITS_ENABLED, "soft_limits_enabled", "1 enables soft limits, 0 disables them"),
    ConfigFieldInfo(ConfigField.SOFT_LIMIT_MIN_STEPS, "soft_limit_min_steps", "Minimum allowed position in steps"),
    ConfigFieldInfo(ConfigField.SOFT_LIMIT_MAX_STEPS, "soft_limit_max_steps", "Maximum allowed position in steps"),
    ConfigFieldInfo(ConfigField.DEFAULT_MAX_VELOCITY_SPS, "default_max_velocity_sps", "Default/max configured velocity"),
    ConfigFieldInfo(ConfigField.DEFAULT_MAX_ACCEL_SPS2, "default_max_accel_sps2", "Default/max configured acceleration"),
    ConfigFieldInfo(ConfigField.DEFAULT_STOP_DECEL_SPS2, "default_stop_decel_sps2", "Default controlled stop decel"),
    ConfigFieldInfo(ConfigField.RUN_CURRENT_MA, "run_current_ma", "TMC2209 run current in mA RMS"),
    ConfigFieldInfo(ConfigField.HOLD_CURRENT_MA, "hold_current_ma", "TMC2209 hold current in mA RMS"),
    ConfigFieldInfo(ConfigField.MICROSTEPS, "microsteps", "TMC2209 microstep setting"),
    ConfigFieldInfo(ConfigField.DRIVER_MODE, "driver_mode", "0 StealthChop, 1 SpreadCycle"),
    ConfigFieldInfo(ConfigField.POSITION_OFFSET_STEPS, "position_offset_steps", "Commissioned position offset"),
    ConfigFieldInfo(ConfigField.HOME_POSITION_STEPS, "home_position_steps", "Configured home position placeholder"),
    ConfigFieldInfo(ConfigField.DIRECTION_INVERTED, "direction_inverted", "1 inverts positive direction"),
    ConfigFieldInfo(ConfigField.HOST_TIMEOUT_MS, "host_timeout_ms", "Host-command timeout for velocity mode"),
    ConfigFieldInfo(ConfigField.VELOCITY_TIMEOUT_ACTION, "velocity_timeout_action", "0 ramp stop, 1 disable"),
    ConfigFieldInfo(ConfigField.ENABLE_TIMEOUT_MS, "enable_timeout_ms", "Reserved enable timeout"),
    ConfigFieldInfo(ConfigField.MAX_ALLOWED_CURRENT_MA, "max_allowed_current_ma", "Safety max current clamp"),
    ConfigFieldInfo(ConfigField.MAX_ALLOWED_VELOCITY_SPS, "max_allowed_velocity_sps", "Safety max velocity clamp"),
    ConfigFieldInfo(ConfigField.MAX_ALLOWED_ACCEL_SPS2, "max_allowed_accel_sps2", "Safety max accel clamp"),
)

_BY_NAME = {info.name: info.field for info in CONFIG_FIELDS}
_BY_FIELD = {info.field: info.name for info in CONFIG_FIELDS}


def field_by_name(name: str) -> ConfigField:
    normalized = name.strip().lower().replace("-", "_")
    if normalized in _BY_NAME:
        return _BY_NAME[normalized]
    try:
        return ConfigField(int(name, 0))
    except ValueError as exc:
        raise KeyError(f"unknown config field {name!r}") from exc


def field_name(field: ConfigField | int) -> str:
    try:
        enum_field = ConfigField(int(field))
    except ValueError:
        return f"field_{int(field)}"
    return _BY_FIELD.get(enum_field, enum_field.name.lower())
