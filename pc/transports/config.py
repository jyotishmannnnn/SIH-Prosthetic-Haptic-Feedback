"""
Transport configuration: which transport to use, and per-transport
connection parameters (Wi-Fi host/port, BLE device name, etc).

Precedence (highest wins): explicit kwargs passed to get_transport()
> environment variables > pc/.env (git-ignored, see .env.example)
> pc/local_config.json (git-ignored) > built-in defaults below.

No secrets/credentials are hard-coded anywhere in this repo. Wi-Fi
passwords, if a wireless transport ever needs one, belong in .env or
local_config.json only — both are git-ignored (see .gitignore).
"""

import json
import os

DEFAULT_TRANSPORT = "usb"

_DEFAULTS = {
    "usb": {
        "baud": 115200,
    },
    "wifi": {
        "host": "192.168.4.1",  # typical ESP32 SoftAP gateway address, NOT verified against real firmware
        "port": 4210,
        "timeout_s": 0.5,
    },
    "ble": {
        "device_name": "HapticESP32",
        "service_uuid": "6e400001-b5a3-f393-e0a9-e50e24dcca9e",   # placeholder Nordic-UART-style UUID, see docs/wireless-setup.md
        "characteristic_uuid": "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
    },
    "espnow": {},
}

_ENV_FILE_CACHE = None


def _read_env_file(path):
    """Minimal .env parser (KEY=VALUE per line, '#' comments) — no extra
    dependency (python-dotenv) needed for this small a format."""
    values = {}
    if not os.path.exists(path):
        return values
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def _env_file_values():
    global _ENV_FILE_CACHE
    if _ENV_FILE_CACHE is None:
        env_path = os.path.join(os.path.dirname(__file__), "..", ".env")
        _ENV_FILE_CACHE = _read_env_file(env_path)
    return _ENV_FILE_CACHE


def _local_config_values():
    path = os.path.join(os.path.dirname(__file__), "..", "local_config.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _lookup(key, fallback=None):
    """os.environ > .env file > fallback."""
    if key in os.environ:
        return os.environ[key]
    envfile = _env_file_values()
    if key in envfile:
        return envfile[key]
    return fallback


def load_transport_config():
    """Returns {"usb": {...}, "wifi": {...}, "ble": {...}, "espnow": {...}}
    merged from defaults -> local_config.json -> environment/.env."""
    cfg = {name: dict(params) for name, params in _DEFAULTS.items()}

    local = _local_config_values()
    for name, params in local.items():
        if name in cfg and isinstance(params, dict):
            cfg[name].update(params)

    # A handful of env-var overrides for the common wireless knobs, so a
    # demo can flip host/port without editing JSON:
    if _lookup("HAPTIC_WIFI_HOST"):
        cfg["wifi"]["host"] = _lookup("HAPTIC_WIFI_HOST")
    if _lookup("HAPTIC_WIFI_PORT"):
        cfg["wifi"]["port"] = int(_lookup("HAPTIC_WIFI_PORT"))
    if _lookup("HAPTIC_BLE_DEVICE_NAME"):
        cfg["ble"]["device_name"] = _lookup("HAPTIC_BLE_DEVICE_NAME")

    return cfg


def get_default_transport_name():
    return _lookup("TRANSPORT", DEFAULT_TRANSPORT)
