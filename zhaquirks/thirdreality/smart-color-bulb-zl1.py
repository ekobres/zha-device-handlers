"""Third Reality Smart Color Bulb ZL1 Quirks."""

# This quirk fixes a few problems with the ZL1 Bulb:
#   1.  Correct logical color temperature presentation (device reports 142-454
#       mireds but is actually 154-370). Map between logical and device ranges.
#   2.  Provide perceptual brightness mapping so very low but non-off levels
#       (raw 1..) are reachable and high end is slightly compressed.

import logging

from zigpy.quirks.v2 import QuirkBuilder
from zigpy.zcl.clusters.general import LevelControl as ZigpyLevelControl
from zigpy.zcl.clusters.lighting import Color as ZigpyColor

from zhaquirks import CustomCluster

_LOGGER = logging.getLogger(__name__)
_LOGGER.warning("Loading Third Reality ZL1 smart color bulb quirk module")

# Target fingerprint (gating)
TARGET_MANUFACTURER = "Third Reality, Inc"
TARGET_MODEL = "3RCB01057Z"
# Set to e.g. "1.00.66" to enforce firmware, or leave as None to disable FW gating.
TARGET_SW_BUILD_ID = None  # "1.00.66"

# ZCL constants
CMD_MOVE_TO_LEVEL = 0x00
CMD_MOVE_TO_LEVEL_WITH_ON_OFF = 0x04
CMD_MOVE_WITH_ON_OFF = 0x05
CMD_STEP = 0x02
CMD_STEP_WITH_ON_OFF = 0x06
CMD_MOVE_TO_COLOR_TEMP = 0x0A

ATTR_CURRENT_LEVEL = 0x0000
ATTR_COLOR_TEMP = 0x0007
ATTR_COLOR_CAPS = 0x400A  # left for reference; no longer overridden
ATTR_CT_MIN = 0x400B
ATTR_CT_MAX = 0x400C

# LevelControl.StepMode (ZCL 0x0008 Level Control): Up=0x00, Down=0x01
STEP_MODE_UP = 0x00
STEP_MODE_DOWN = 0x01

# Logical color temperature range (shown to HA)
LOGICAL_MIN_MIRED = 154  # ~6500 K (cool)
LOGICAL_MAX_MIRED = 370  # ~2700 K (warm)

# Device command range (actual on-wire)
DEVICE_MIN_MIRED = 142
DEVICE_MAX_MIRED = 454

## No FIXED_COLOR_CAPABILITIES needed (firmware now advertises correctly)

# Brightness perceptual mapping
# HA typically uses 3..254
HA_MIN_LEVEL_INPUT = 3
HA_MAX_LEVEL_INPUT = 254
DEVICE_LEVEL_MIN = 0
DEVICE_LEVEL_MAX = 254

MIN_LEVEL = 0
MAX_LEVEL = 254
DEFAULT_LEVEL = 128
DEFAULT_TRANSITION_TIME = 0


GAMMA_LOW = 2.2  # growth below knee (higher = more low-end resolution)
GAMMA_HIGH = 1.4  # growth above knee (>=1 compresses highs)
KNEE_IN = 25  # HA brightness at knee (0..255 scale)
KNEE_OUT = 0.18  # output fraction at knee (0..1), puts extra codes below knee
LOW_END_PIN_MAX = 50
EPSILON = 1e-6


def _clamp_value(x, lo, hi):
    if x < lo:
        return lo
    elif x > hi:
        return hi
    else:
        return x


def _linear_map(x, a, b, c, d):
    if b == a:
        return c
    return c + (x - a) * (d - c) / (b - a)


def _map_ha_brightness_to_device(req: int, *, log: bool = True) -> int:
    """Map HA brightness (3..254) to device level (1..254) with a perceptual curve.

    log: set False to suppress warning log (used during LUT construction).
    """
    try:
        v = int(req)
    except Exception:
        return req

    # Allow explicit 0 to pass through (true OFF intent).
    if v == 0:
        return 0

    # Clamp positive requests at/under HA_MIN_IN to device 1.
    if 0 < v <= HA_MIN_LEVEL_INPUT:
        return 1

    # Low-end linear pinning
    if v <= LOW_END_PIN_MAX:
        # Ensure 1..PIN_RAW_MAX never yields 0
        return max(1, v)

    # Normalize to 0..1 over [HA_MIN_IN..HA_MAX_IN]
    n = (v - HA_MIN_LEVEL_INPUT) / (HA_MAX_LEVEL_INPUT - HA_MIN_LEVEL_INPUT)
    n = _clamp_value(n, 0.0, 1.0)

    # Knee/gamma
    knee_n = _clamp_value(
        (KNEE_IN - HA_MIN_LEVEL_INPUT) / (HA_MAX_LEVEL_INPUT - HA_MIN_LEVEL_INPUT),
        0.0,
        1.0,
    )
    knee_out = _clamp_value(KNEE_OUT, 0.0, 1.0)

    if abs(knee_n - 0.0) < EPSILON:
        y = n**GAMMA_HIGH
    elif n <= knee_n:
        y = 0.0 if abs(knee_n - 0.0) < EPSILON else (n / knee_n) ** GAMMA_LOW * knee_out
    elif abs(knee_n - 1.0) < EPSILON:
        y = knee_out
    else:
        t = (n - knee_n) / (1.0 - knee_n)
        y = knee_out + (t**GAMMA_HIGH) * (1.0 - knee_out)

    # Scale to device domain
    dev = int(round(_linear_map(y, 0.0, 1.0, DEVICE_LEVEL_MIN, DEVICE_LEVEL_MAX)))

    # Avoid 0 for a positive request
    if dev == 0 and v > 0:
        dev = 1

    mapped = _clamp_value(dev, 0, DEVICE_LEVEL_MAX)
    if v != mapped:
        dev = mapped
    if log:
        _LOGGER.warning(
            "_map_ha_brightness_to_device: ha=%s device=%s (req=%s)",
            v,
            dev,
            req,
        )
    return dev


def _map_device_brightness_to_ha(dev: int, *, log: bool = True) -> int:
    """Inverse mapping for stable sliders (device level -> HA level).

    log: set False to suppress warning log (not currently called in LUT build).
    """
    original = dev
    dev = _clamp_value(int(dev), DEVICE_LEVEL_MIN, DEVICE_LEVEL_MAX)

    # Low-end linear pinning reflection
    if dev <= LOW_END_PIN_MAX:
        return _clamp_value(
            max(HA_MIN_LEVEL_INPUT, dev), HA_MIN_LEVEL_INPUT, HA_MAX_LEVEL_INPUT
        )

    y = (dev - DEVICE_LEVEL_MIN) / (DEVICE_LEVEL_MAX - DEVICE_LEVEL_MIN)
    knee_n = _clamp_value(
        (KNEE_IN - HA_MIN_LEVEL_INPUT) / (HA_MAX_LEVEL_INPUT - HA_MIN_LEVEL_INPUT),
        0.0,
        1.0,
    )
    knee_out = _clamp_value(KNEE_OUT, 0.0, 1.0)

    if abs(knee_n - 0.0) < EPSILON:
        n = y ** (1.0 / GAMMA_HIGH)
    elif y <= knee_out:
        n = (
            0.0
            if abs(knee_out - 0.0) < EPSILON
            else (y / knee_out) ** (1.0 / GAMMA_LOW) * knee_n
        )
    elif abs(1.0 - knee_out) < EPSILON:
        n = knee_n
    else:
        t = (y - knee_out) / (1.0 - knee_out)
        n = knee_n + (t ** (1.0 / GAMMA_HIGH)) * (1.0 - knee_n)

    ha = int(round(_linear_map(n, 0.0, 1.0, HA_MIN_LEVEL_INPUT, HA_MAX_LEVEL_INPUT)))
    ha_mapped = _clamp_value(ha, HA_MIN_LEVEL_INPUT, HA_MAX_LEVEL_INPUT)
    if log:
        _LOGGER.warning(
            "_map_device_brightness_to_ha: device=%s clamped_device=%s ha=%s mapped_ha=%s",
            original,
            dev,
            ha,
            ha_mapped,
        )
    return ha_mapped


# Color cluster


class Color(CustomCluster, ZigpyColor):
    """Custom Color cluster with CT mapping and XY capability fixes."""

    cluster_id = ZigpyColor.cluster_id

    LOG_MIN = LOGICAL_MIN_MIRED  # 154
    LOG_MAX = LOGICAL_MAX_MIRED  # 370
    DEV_MIN = DEVICE_MIN_MIRED  # 142
    DEV_MAX = DEVICE_MAX_MIRED  # 454

    # Gating
    def _active(self) -> bool:
        dev = getattr(self.endpoint, "device", None)
        if not dev:
            return False
        if getattr(dev, "manufacturer", "") != TARGET_MANUFACTURER:
            return False
        if getattr(dev, "model", "") != TARGET_MODEL:
            return False
        if TARGET_SW_BUILD_ID:
            sw = getattr(dev, "sw_build_id", "") or getattr(
                dev, "software_build_id", ""
            )
            if sw != TARGET_SW_BUILD_ID:
                return False
        return True

    def __init__(self, *args, **kwargs):
        """Initialize cluster and seed cache with corrected limits/capabilities."""
        super().__init__(*args, **kwargs)
        if self._active():
            # Seed cache for corrected CT limits only
            super()._update_attribute(ATTR_CT_MIN, self.LOG_MIN)
            super()._update_attribute(ATTR_CT_MAX, self.LOG_MAX)
            _LOGGER.warning(
                "Color.__init__: active=True logical_ct_limits=%s-%s device_ct_limits=%s-%s",
                self.LOG_MIN,
                self.LOG_MAX,
                self.DEV_MIN,
                self.DEV_MAX,
            )
        else:
            _LOGGER.warning("Color.__init__: active=False (not target device)")

    async def bind(self):
        """Bind cluster and push corrected attributes to cache."""
        res = await super().bind()
        if self._active():
            super()._update_attribute(ATTR_CT_MIN, self.LOG_MIN)
            super()._update_attribute(ATTR_CT_MAX, self.LOG_MAX)
            _LOGGER.warning(
                "Color.bind: enforced logical_ct_limits=%s-%s",
                self.LOG_MIN,
                self.LOG_MAX,
            )
        return res

    # Helpers
    def _convert_device_mireds_to_logical(self, dev_mired: int) -> int:
        dev_mired = _clamp_value(int(dev_mired), self.DEV_MIN, self.DEV_MAX)
        logical = int(
            round(
                _linear_map(
                    dev_mired, self.DEV_MIN, self.DEV_MAX, self.LOG_MIN, self.LOG_MAX
                )
            )
        )
        _LOGGER.warning(
            "Color._convert_device_mireds_to_logical: device_ct=%s logical_ct=%s",
            dev_mired,
            logical,
        )
        return logical

    def _convert_logical_mireds_to_device(self, log_mired: int) -> int:
        log_mired = _clamp_value(int(log_mired), self.LOG_MIN, self.LOG_MAX)
        device = int(
            round(
                _linear_map(
                    log_mired, self.LOG_MIN, self.LOG_MAX, self.DEV_MIN, self.DEV_MAX
                )
            )
        )
        _LOGGER.warning(
            "Color._convert_logical_mireds_to_device: logical_ct=%s device_ct=%s",
            log_mired,
            device,
        )
        return device

    # Commands
    async def command(
        self,
        command_id,
        *args,
        manufacturer=None,
        expect_reply=True,
        tsn=None,
        **kwargs,
    ):
        """Intercept CT command to map logical mireds to device range when active."""
        if not self._active():
            return await super().command(
                command_id,
                *args,
                manufacturer=manufacturer,
                expect_reply=expect_reply,
                tsn=tsn,
                **kwargs,
            )

        # Move to Color Temperature (0x0A)
        if command_id == CMD_MOVE_TO_COLOR_TEMP:
            try:
                if kwargs:
                    kw = dict(kwargs)
                    for key in ("color_temp_mireds", "color_temperature", "color_temp"):
                        if key in kw:
                            before = kw[key]
                            kw[key] = self._convert_logical_mireds_to_device(kw[key])
                            _LOGGER.warning(
                                "Color.command: cmd=0x%02X logical_ct=%s -> device_ct=%s",
                                command_id,
                                before,
                                kw[key],
                            )
                            kwargs = kw
                            break
                elif args:
                    before = args[0]
                    mapped = self._convert_logical_mireds_to_device(args[0])
                    args = (mapped,) + tuple(args[1:])
                    _LOGGER.warning(
                        "Color.command: cmd=0x%02X logical_ct=%s -> device_ct=%s",
                        command_id,
                        before,
                        mapped,
                    )
            except Exception as ex:  # noqa: BLE001
                _LOGGER.error("Color.command: exception mapping CT: %s", ex)

        return await super().command(
            command_id,
            *args,
            manufacturer=manufacturer,
            expect_reply=expect_reply,
            tsn=tsn,
            **kwargs,
        )

    # NOTE: We intentionally do NOT override move_to_color_temp / move_to_color_temperature.
    # Mapping is handled centrally in command() for CMD_MOVE_TO_COLOR_TEMP so that
    # upstream ZHA cluster handlers receive the native zigpy return shape without
    # any wrapper-induced signature issues.

    # Attribute writes / reads
    async def write_attributes(self, attrs, manufacturer=None):
        """Rewrite CT attributes to device range on write when active."""
        if not self._active():
            return await super().write_attributes(attrs, manufacturer=manufacturer)

        try:
            for k, v in attrs.items():
                if k in (ATTR_COLOR_TEMP, "color_temperature"):
                    before = v
                    attrs[k] = self._convert_logical_mireds_to_device(v)
                    _LOGGER.warning(
                        "Color.write_attributes: logical_ct=%s -> device_ct=%s",
                        before,
                        attrs[k],
                    )
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("Color.write_attributes: exception: %s", ex)
        return await super().write_attributes(attrs, manufacturer=manufacturer)

    async def read_attributes(
        self, attributes, allow_cache=True, only_cache=False, manufacturer=None
    ):
        """Normalize CT attributes to logical range when active."""
        _LOGGER.warning(
            "Color.read_attributes(entry): attrs=%s type=%s allow_cache=%s only_cache=%s active=%s",
            attributes,
            type(attributes).__name__,
            allow_cache,
            only_cache,
            self._active(),
        )
        # Record which attributes were explicitly requested so we can inject
        # logical defaults if the base read omits them (some stacks drop
        # PhysicalMin/Max on read failures or manufacturer quirkiness).
        requested_ids = set()
        requested_names = set()
        try:
            for a in attributes:
                if isinstance(a, int):
                    requested_ids.add(a)
                else:
                    requested_names.add(str(a))
        except Exception:  # noqa: BLE001
            pass
        result_tuple = await super().read_attributes(
            attributes,
            allow_cache=allow_cache,
            only_cache=only_cache,
            manufacturer=manufacturer,
        )
        # zigpy may return (success, failure); older path treated dict only.
        if isinstance(result_tuple, tuple) and len(result_tuple) == 2:
            result, failure = result_tuple
            if not isinstance(result, dict):
                result = dict(result or {})
            else:
                # Make a shallow copy to avoid mutating zigpy internal mapping
                result = dict(result)
        else:
            result, failure = (
                result_tuple if isinstance(result_tuple, dict) else {},
                {},
            )
            # Ensure plain dict copy
            result = dict(result)
        if not self._active():
            return (result, failure)

        # If requested but missing, inject logical values so caller always
        # observes normalized physical min/max. Use constants rather than
        # cache lookup to avoid chasing prior state.
        if (
            (
                ATTR_CT_MIN in requested_ids
                or "color_temp_physical_min_mireds" in requested_names
            )
            and ATTR_CT_MIN not in result
            and "color_temp_physical_min_mireds" not in result
        ):
            result[ATTR_CT_MIN] = self.LOG_MIN
            _LOGGER.warning(
                "Color.read_attributes: injecting missing CT_MIN=%s (logical)",
                self.LOG_MIN,
            )
        if (
            (
                ATTR_CT_MAX in requested_ids
                or "color_temp_physical_max_mireds" in requested_names
            )
            and ATTR_CT_MAX not in result
            and "color_temp_physical_max_mireds" not in result
        ):
            result[ATTR_CT_MAX] = self.LOG_MAX
            _LOGGER.warning(
                "Color.read_attributes: injecting missing CT_MAX=%s (logical)",
                self.LOG_MAX,
            )

        def set_val(key, name, value):
            if key in result:
                result[key] = value
            elif name in result:
                result[name] = value

        set_val(ATTR_CT_MIN, "color_temp_physical_min_mireds", self.LOG_MIN)
        set_val(ATTR_CT_MAX, "color_temp_physical_max_mireds", self.LOG_MAX)

        # Ensure physical min/max are always reported as logical values even if
        # super() returned device-range values (cache mutation via
        # _update_attribute doesn't rewrite the already-built result dict).
        if ATTR_CT_MIN in result:
            raw = result[ATTR_CT_MIN]
            if raw != self.LOG_MIN:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MIN raw=%s -> %s",
                    raw,
                    self.LOG_MIN,
                )
                result[ATTR_CT_MIN] = self.LOG_MIN
            else:
                _LOGGER.warning("Color.read_attributes: CT_MIN already logical=%s", raw)
        if "color_temp_physical_min_mireds" in result:
            raw = result["color_temp_physical_min_mireds"]
            if raw != self.LOG_MIN:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MIN(name) raw=%s -> %s",
                    raw,
                    self.LOG_MIN,
                )
                result["color_temp_physical_min_mireds"] = self.LOG_MIN
            else:
                _LOGGER.warning(
                    "Color.read_attributes: CT_MIN(name) already logical=%s", raw
                )

        if ATTR_CT_MAX in result:
            raw = result[ATTR_CT_MAX]
            if raw != self.LOG_MAX:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MAX raw=%s -> %s",
                    raw,
                    self.LOG_MAX,
                )
                result[ATTR_CT_MAX] = self.LOG_MAX
            else:
                _LOGGER.warning("Color.read_attributes: CT_MAX already logical=%s", raw)
        if "color_temp_physical_max_mireds" in result:
            raw = result["color_temp_physical_max_mireds"]
            if raw != self.LOG_MAX:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MAX(name) raw=%s -> %s",
                    raw,
                    self.LOG_MAX,
                )
                result["color_temp_physical_max_mireds"] = self.LOG_MAX
            else:
                _LOGGER.warning(
                    "Color.read_attributes: CT_MAX(name) already logical=%s", raw
                )

        if ATTR_COLOR_TEMP in result or "color_temperature" in result:
            raw = result.get(ATTR_COLOR_TEMP, result.get("color_temperature"))
            if isinstance(raw, int):
                try:
                    mapped = self._convert_device_mireds_to_logical(raw)
                    set_val(ATTR_COLOR_TEMP, "color_temperature", mapped)
                    _LOGGER.warning(
                        "Color.read_attributes: device_ct=%s -> logical_ct=%s",
                        raw,
                        mapped,
                    )
                except Exception as ex:  # noqa: BLE001
                    _LOGGER.error("Color.read_attributes: exception mapping CT: %s", ex)
            else:
                _LOGGER.warning(
                    "Color.read_attributes: skipping CT map (raw not int) raw=%s type=%s",
                    raw,
                    type(raw).__name__,
                )

        # ColorCapabilities (no override, just visibility for debugging)
        if ATTR_COLOR_CAPS in result or "color_capabilities" in result:
            caps_val = result.get(ATTR_COLOR_CAPS, result.get("color_capabilities"))
            if caps_val is None:
                _LOGGER.warning(
                    "Color.read_attributes: ColorCapabilities=None (no value)"
                )
            else:
                try:
                    val_int = int(caps_val)
                    flags = []
                    if val_int & 0x01:
                        flags.append("HueSat")
                    if val_int & 0x02:
                        flags.append("EnhancedHue")
                    if val_int & 0x04:
                        flags.append("ColorLoop")
                    if val_int & 0x08:
                        flags.append("XY")
                    if val_int & 0x10:
                        flags.append("CT")
                    _LOGGER.warning(
                        "Color.read_attributes: ColorCapabilities=0x%02X (%s)",
                        val_int,
                        ",".join(flags) or "none",
                    )
                except Exception as ex:  # noqa: BLE001
                    _LOGGER.error(
                        "Color.read_attributes: exception decoding ColorCapabilities: %s",
                        ex,
                    )

        # Exit summary (only log the keys actually returned)
        try:
            interesting = {}
            for key in (ATTR_CT_MIN, ATTR_CT_MAX, ATTR_COLOR_TEMP):
                if key in result:
                    interesting[f"0x{key:04X}"] = result[key]
            for name in (
                "color_temp_physical_min_mireds",
                "color_temp_physical_max_mireds",
                "color_temperature",
            ):
                if name in result:
                    interesting[name] = result[name]
            _LOGGER.warning("Color.read_attributes(exit): normalized=%s", interesting)
        except Exception:  # noqa: BLE001
            pass

        return (result, failure)

    # Cache normalization
    def _update_attribute(self, attrid, value):
        if not self._active():
            return super()._update_attribute(attrid, value)
        try:
            original = value
            if attrid == ATTR_CT_MIN:  # PhysicalMin
                value = self.LOG_MIN
            elif attrid == ATTR_CT_MAX:  # PhysicalMax
                value = self.LOG_MAX
            elif attrid == ATTR_COLOR_TEMP:  # CurrentColorTemperatureMireds
                value = self._convert_device_mireds_to_logical(value)
            if original != value:
                _LOGGER.warning(
                    "Color._update_attribute: attr=0x%04X raw=%s -> logical=%s",
                    attrid,
                    original,
                    value,
                )
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("Color._update_attribute: exception: %s", ex)
        return super()._update_attribute(attrid, value)


# Level cluster


class LevelControl(CustomCluster, ZigpyLevelControl):
    """Perceptual mapping (+ anti-OFF guards) for the 3R ZL1.

    Uses LUTs for idempotent round-trip and hysteresis to stop slider bounce.
    """

    # Gating
    def _active(self) -> bool:
        dev = getattr(self.endpoint, "device", None)
        if not dev:
            return False
        if getattr(dev, "manufacturer", "") != TARGET_MANUFACTURER:
            return False
        if getattr(dev, "model", "") != TARGET_MODEL:
            return False
        if TARGET_SW_BUILD_ID:
            sw = getattr(dev, "sw_build_id", "") or getattr(
                dev, "software_build_id", ""
            )
            if sw != TARGET_SW_BUILD_ID:
                return False
        return True

    # Init: build LUTs once
    def __init__(self, *args, **kwargs):
        """Build LUTs and initialize hysteresis when active."""
        super().__init__(*args, **kwargs)
        self._ha2dev = None
        self._dev2ha = None
        self._last_dev = None
        self._last_ha = None
        self._hysteresis = 2  # device-level counts considered "close enough"
        if self._active():
            self._build_brightness_lookup_tables()
            _LOGGER.warning(
                "LevelControl.__init__: active=True hysteresis=%s", self._hysteresis
            )
        else:
            _LOGGER.warning("LevelControl.__init__: active=False (not target device)")

    def _build_brightness_lookup_tables(self):
        """Build monotonic LUTs so inverse(forward(h)) == h for reachable values."""
        # Forward: HA(0..254) -> device(0..254)
        ha2dev = [0] * (MAX_LEVEL + 1)
        for h in range(MIN_LEVEL, MAX_LEVEL + 1):
            ha2dev[h] = _map_ha_brightness_to_device(h, log=False)

        # Inverse: device(0..254) -> nearest HA that produced it
        dev2ha = [HA_MIN_LEVEL_INPUT] * (MAX_LEVEL + 1)
        # Map each device value to the HA that yields the closest device output
        for d in range(MIN_LEVEL, MAX_LEVEL + 1):
            best_h = HA_MIN_LEVEL_INPUT
            best_err = 9999
            # Search HA domain that HA actually uses (3..254); include 0 for explicit off
            for h in range(MIN_LEVEL, MAX_LEVEL + 1):
                dv = ha2dev[h]
                err = abs(dv - d)
                if err < best_err or (err == best_err and h < best_h):
                    best_err = err
                    best_h = h
                    if best_err == 0:
                        break
            dev2ha[d] = best_h

        # Make both monotonic
        for i in range(1, MAX_LEVEL + 1):
            ha2dev[i] = max(ha2dev[i], ha2dev[i - 1])
        for i in range(1, MAX_LEVEL + 1):
            dev2ha[i] = max(dev2ha[i], dev2ha[i - 1])

        self._ha2dev = ha2dev
        self._dev2ha = dev2ha
        _LOGGER.warning("LevelControl._build_brightness_lookup_tables: tables_built")

    # Mapping helpers (LUT-backed)
    def _map_brightness_level(self, v: int) -> int:
        if not self._active() or self._ha2dev is None:
            return _map_ha_brightness_to_device(v)
        v = _clamp_value(int(v), MIN_LEVEL, MAX_LEVEL)
        mapped = int(self._ha2dev[v])
        _LOGGER.warning(
            "LevelControl._map_brightness_level: ha=%s device=%s", v, mapped
        )
        return mapped

    def _convert_device_level_to_ha(self, dev: int) -> int:
        if not self._active() or self._dev2ha is None:
            return _map_device_brightness_to_ha(dev)
        d = _clamp_value(int(dev), MIN_LEVEL, MAX_LEVEL)
        # Stickiness: echo last HA if near last device level
        if self._last_dev is not None and abs(d - self._last_dev) <= self._hysteresis:
            return int(self._last_ha if self._last_ha is not None else self._dev2ha[d])
        mapped = int(self._dev2ha[d])
        _LOGGER.warning(
            "LevelControl._convert_device_level_to_ha: device=%s ha=%s", d, mapped
        )
        return mapped

    def _remember_set(self, ha_level: int):
        """Call after sending a level to make inbound reports sticky."""
        self._last_ha = _clamp_value(int(ha_level), MIN_LEVEL, MAX_LEVEL)
        self._last_dev = self._map_brightness_level(self._last_ha)
        _LOGGER.warning(
            "LevelControl._remember_set: ha=%s last_dev=%s",
            self._last_ha,
            self._last_dev,
        )

    def _avoid_zero_result(self, cmd_id: int, dev_level: int) -> int:
        # Avoid OFF semantics on *with_on_off* when dev_level would be 0
        if (
            cmd_id
            in (
                CMD_MOVE_TO_LEVEL_WITH_ON_OFF,
                CMD_STEP_WITH_ON_OFF,
            )
            and dev_level == 0
        ):
            _LOGGER.warning(
                "LevelControl._avoid_zero_result: cmd=0x%02X forcing 1 from 0", cmd_id
            )
            return 1
        _LOGGER.warning(
            "LevelControl._avoid_zero_result: cmd=0x%02X dev_level=%s",
            cmd_id,
            dev_level,
        )
        return dev_level

    # Commands
    async def command(
        self,
        command_id,
        *args,
        manufacturer=None,
        expect_reply=True,
        tsn=None,
        **kwargs,
    ):
        """Map HA level to device level and apply anti-OFF guards when active."""
        if not self._active():
            return await super().command(
                command_id,
                *args,
                manufacturer=manufacturer,
                expect_reply=expect_reply,
                tsn=tsn,
                **kwargs,
            )
        _LOGGER.warning(
            "LevelControl.command: cmd=0x%02X args=%s kwargs=%s",
            command_id,
            args,
            kwargs,
        )

        # Handle move-like commands
        if command_id in (CMD_MOVE_TO_LEVEL, CMD_MOVE_TO_LEVEL_WITH_ON_OFF):
            return await self._handle_move_command(command_id, *args)

        # Handle step-like commands
        elif command_id in (CMD_STEP, CMD_STEP_WITH_ON_OFF):
            return await self._handle_step_command(command_id, *args)

        return await super().command(command_id, *args, **kwargs)

    async def _handle_move_command(self, command_id, *args):
        level = args[0] if args else None
        transition_time = args[1] if len(args) >= 2 else DEFAULT_TRANSITION_TIME
        if level is not None:
            # Remember for stickiness; map via LUT
            self._remember_set(int(level))
            mapped_level = self._map_brightness_level(level)
            _LOGGER.warning(
                "LevelControl._handle_move_command: cmd=0x%02X ha=%s mapped=%s transition=%s",
                command_id,
                level,
                mapped_level,
                transition_time,
            )
            if command_id == CMD_MOVE_TO_LEVEL_WITH_ON_OFF and level == 0:
                return await super().command(
                    CMD_MOVE_TO_LEVEL_WITH_ON_OFF, 0, transition_time
                )
            elif (
                command_id == CMD_MOVE_TO_LEVEL_WITH_ON_OFF
                and mapped_level == 0
                and level > 0
            ):
                return await super().command(CMD_MOVE_TO_LEVEL, 1, transition_time)
            else:
                return await super().command(command_id, mapped_level, transition_time)
        return await super().command(command_id, *args)

    async def _handle_step_command(self, command_id, *args):
        step_mode = args[0] if args else None
        step_size = args[1] if len(args) > 1 else 0
        transition_time = args[2] if len(args) > 2 else DEFAULT_TRANSITION_TIME
        if step_mode is not None and step_size is not None:
            # Read HA-normalized current level
            cur = await self.read_attributes([ATTR_CURRENT_LEVEL])
            current_level = DEFAULT_LEVEL
            if isinstance(cur, dict):
                current_level = int(
                    cur.get(ATTR_CURRENT_LEVEL, cur.get("current_level", DEFAULT_LEVEL))
                )

            if step_mode == STEP_MODE_UP:
                projected = min(current_level + int(step_size), MAX_LEVEL)
            elif step_mode == STEP_MODE_DOWN:
                projected = max(current_level - int(step_size), MIN_LEVEL)
            else:
                projected = current_level

            # Avoid OFF semantics when stepping to/below 0 with WITH_ON_OFF
            if command_id == CMD_STEP_WITH_ON_OFF and projected <= 0:
                return await super().command(CMD_MOVE_TO_LEVEL, 1, transition_time)

            _LOGGER.warning(
                "LevelControl._handle_step_command: cmd=0x%02X step_mode=%s step_size=%s projected=%s transition=%s",
                command_id,
                step_mode,
                step_size,
                projected,
                transition_time,
            )
            return await super().command(
                command_id, step_mode, step_size, transition_time
            )
        _LOGGER.warning(
            "LevelControl._handle_step_command: cmd=0x%02X step_mode=%s step_size=%s (no action taken)",
            command_id,
            step_mode,
            step_size,
        )
        return await super().command(command_id, *args)

    # Convenience wrappers route via command IDs
    async def move_to_level(self, level, transition_time):
        """Route to command 0x00 with mapping and stickiness when active."""
        if not self._active():
            return await super().command(CMD_MOVE_TO_LEVEL, level, transition_time)
        self._remember_set(int(level))
        mapped = self._map_brightness_level(level)
        if int(level) > 0 and mapped == 0:
            mapped = 1
        _LOGGER.warning(
            "LevelControl.move_to_level: ha=%s mapped=%s transition=%s",
            level,
            mapped,
            transition_time,
        )
        # Call base to avoid re-mapping
        return await super().command(CMD_MOVE_TO_LEVEL, mapped, transition_time)

    async def move_to_level_with_on_off(self, level, transition_time):
        """Route to command 0x04 with mapping and anti-OFF rewrite when active."""
        if not self._active():
            return await super().command(
                CMD_MOVE_TO_LEVEL_WITH_ON_OFF, level, transition_time
            )
        self._remember_set(int(level))
        mapped = self._map_brightness_level(level)
        mapped = self._avoid_zero_result(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, mapped)
        if mapped <= 1 and int(level) > 0:
            # Rewrite to plain move_to_level at 1
            return await super().command(CMD_MOVE_TO_LEVEL, 1, transition_time)
        _LOGGER.warning(
            "LevelControl.move_to_level_with_on_off: ha=%s mapped=%s transition=%s",
            level,
            mapped,
            transition_time,
        )
        # Call base to avoid re-mapping
        return await super().command(
            CMD_MOVE_TO_LEVEL_WITH_ON_OFF, mapped, transition_time
        )

    async def write_attributes(self, attributes, manufacturer=None):
        """Rewrite current_level writes to mapped device values when active."""
        if not self._active():
            return await super().write_attributes(attributes, manufacturer=manufacturer)
        try:
            attrs = dict(attributes)
            for k, v in attrs.items():
                if k in (ATTR_CURRENT_LEVEL, "current_level", "level"):
                    self._remember_set(int(v))
                    attrs[k] = self._map_brightness_level(v)
                    _LOGGER.warning(
                        "LevelControl.write_attributes: ha=%s -> device=%s", v, attrs[k]
                    )
            attributes = attrs
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("LevelControl.write_attributes: exception: %s", ex)
        return await super().write_attributes(attributes, manufacturer=manufacturer)

    # Normalize inbound/cache to avoid slider bounce
    def _update_attribute(self, attrid, value):
        if not self._active():
            return super()._update_attribute(attrid, value)
        try:
            if attrid == ATTR_CURRENT_LEVEL:  # CurrentLevel
                raw = int(value)
                value = self._convert_device_level_to_ha(int(value))
                _LOGGER.warning(
                    "LevelControl._update_attribute: device=%s -> ha=%s", raw, value
                )
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("LevelControl._update_attribute: exception: %s", ex)
        return super()._update_attribute(attrid, value)

    async def read_attributes(
        self, attributes, allow_cache=True, only_cache=False, manufacturer=None
    ):
        """Normalize current_level results to HA levels when active."""
        _LOGGER.warning(
            "LevelControl.read_attributes(entry): attrs=%s type=%s allow_cache=%s only_cache=%s active=%s",
            attributes,
            type(attributes).__name__,
            allow_cache,
            only_cache,
            self._active(),
        )
        result = await super().read_attributes(
            attributes,
            allow_cache=allow_cache,
            only_cache=only_cache,
            manufacturer=manufacturer,
        )
        if not self._active():
            return result
        if ATTR_CURRENT_LEVEL in result or "current_level" in result:
            raw = result.get(ATTR_CURRENT_LEVEL, result.get("current_level"))
            try:
                val = self._convert_device_level_to_ha(int(raw))
                if ATTR_CURRENT_LEVEL in result:
                    result[ATTR_CURRENT_LEVEL] = val
                else:
                    result["current_level"] = val
                _LOGGER.warning(
                    "LevelControl.read_attributes: device=%s -> ha=%s", raw, val
                )
            except Exception as ex:  # noqa: BLE001
                _LOGGER.error("LevelControl.read_attributes: exception: %s", ex)
        return result


# ===================== Device quirk =====================

(
    QuirkBuilder(TARGET_MANUFACTURER, TARGET_MODEL)
    .replaces(Color)
    .replaces(LevelControl)
    .add_to_registry()
)
