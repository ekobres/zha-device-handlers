"""Third Reality Smart Color Bulb ZL1 Quirks."""

# This quirk fixes a few problems with the ZL1 Bulb:
#   1.  Correct logical color temperature presentation (device reports 142-454
#       mireds but is actually 154-370). Map between logical and device ranges.
#   2.  Provide perceptual brightness mapping so very low but non-off levels
#       (raw 1..) are reachable and high end is slightly compressed.

import contextlib
import logging
import math
import time

from zigpy.quirks.v2 import QuirkBuilder
from zigpy.zcl.clusters.general import LevelControl as ZigpyLevelControl
from zigpy.zcl.clusters.lighting import Color as ZigpyColor

from zhaquirks import CustomCluster

_LOGGER = logging.getLogger(__name__)
_LOGGER.warning("Loading Third Reality ZL1 smart color bulb quirk module")

# Target fingerprint
TARGET_MANUFACTURER = "Third Reality, Inc"
TARGET_MODEL = "3RCB01057Z"

# ZCL constants
CMD_MOVE_TO_LEVEL = 0x00
CMD_MOVE_TO_LEVEL_WITH_ON_OFF = 0x04
CMD_STEP = 0x02
CMD_STEP_WITH_ON_OFF = 0x06
CMD_MOVE_TO_COLOR_TEMP = 0x0A
CMD_MOVE_TO_COLOR = 0x07
CMD_MOVE_TO_HUE_SAT = 0x06
CMD_ENHANCED_MOVE_TO_HUE_SAT = 0x43  # ZCL enhanced command

ATTR_CURRENT_LEVEL = 0x0000
ATTR_COLOR_TEMP = 0x0007
ATTR_CT_MIN = 0x400B
ATTR_CT_MAX = 0x400C
ATTR_CURRENT_X = 0x0003
ATTR_CURRENT_Y = 0x0004
ATTR_COLOR_MODE = 0x0008

STEP_MODE_UP = 0x00
STEP_MODE_DOWN = 0x01

LOGICAL_MIN_MIRED = 154  # ~6500 K (cool)
LOGICAL_MAX_MIRED = 370  # ~2700 K (warm)

DEVICE_MIN_MIRED = 142
DEVICE_MAX_MIRED = 454

HA_MIN_LEVEL_INPUT = 3  # HA normally emits 3..254 for brightness writes
HA_MAX_LEVEL_INPUT = 254
DEVICE_LEVEL_MIN = 0  # Allow 0 only for explicit OFF
DEVICE_LEVEL_MAX = 254

MIN_LEVEL = 0  # HA domain minimum (OFF semantic)
MAX_LEVEL = 254  # HA domain maximum
DEFAULT_LEVEL = 128
DEFAULT_TRANSITION_TIME = 0
DEFAULT_COLOR_TRANSITION_TENTHS = 4
STICKY_WINDOW_S = 2.8  # echo last HA value for this many seconds after a set (cover group refresh debounce)
HYSTERESIS_COUNTS = 3  # device-level tolerance for stickiness

BRIGHTNESS_THRESHOLD_PERCENT = (
    50  # Percent threshold between linear percent zone and curve
)
START_SLOPE_NORM = 1.0  # Hermite start derivative at threshold
END_SLOPE_NORM = 0.35  # Hermite end derivative (tunable)

UV_CT_SNAP_EPSILON = 0.015  # u'v' distance threshold to snap to CT (preferred)


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


def _ha_raw_to_percent(ha: int) -> int:
    """Approximate HA percent (1..100) from HA raw brightness (1..254).

    Home Assistant typically derives raw from percent with rounding: raw ≈ round(p/100 * 255).
    We invert: percent ≈ round(raw * 100 / 255). Guarantee minimum 1 for any non-zero raw.
    """
    if ha <= 0:
        return 0
    p = int(round(ha * 100.0 / 255.0))
    if p == 0:
        p = 1
    p = 100 if p > 100 else p
    return p


def _hermite_ease(x: float, d0: float, d1: float) -> float:
    """Cubic Hermite interpolation on [0,1] with endpoints (0,0),(1,1)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    x2 = x * x
    x3 = x2 * x
    h10 = x3 - 2 * x2 + x  # *d0
    h01 = -2 * x3 + 3 * x2  # *1
    h11 = x3 - x2  # *d1
    return h10 * d0 + h01 + h11 * d1


def _build_new_brightness_tables():  # noqa: C901
    """Construct forward/inverse brightness LUTs using percent-based low segment.

    Forward domain: HA raw 0..254 -> device raw 0..254
      * Convert HA raw -> percent p (0..100)
      * If p <= threshold: device_raw = p
      * Else apply Hermite easing over p in (threshold..100] mapping to
        [threshold .. DEVICE_LEVEL_MAX].
    Inverse maps device raw back to representative HA raw (first producing HA).
    """
    ha2dev = [0] * (MAX_LEVEL + 1)

    # Precompute anchor and auto-fit start slope for C1 continuity (dr/dp = 1)
    anchor_raw = BRIGHTNESS_THRESHOLD_PERCENT  # device raw at threshold percent
    curve_range = DEVICE_LEVEL_MAX - anchor_raw
    percent_span = 100 - BRIGHTNESS_THRESHOLD_PERCENT
    # dy/dx at x=0 for Hermite so that dr/dp = 1 at threshold:
    # dr/dp = curve_range * (dy/dx) * (1/percent_span) => dy/dx = percent_span/curve_range
    d0 = (percent_span / curve_range) if curve_range > 0 else 0.0
    for ha in range(0, MAX_LEVEL + 1):
        if ha == 0:
            ha2dev[ha] = 0
            continue
        p = _ha_raw_to_percent(ha)
        if p <= BRIGHTNESS_THRESHOLD_PERCENT:
            mapped = 1 if p <= 2 else p
        else:
            # Normalized progress across percent domain
            x = (p - BRIGHTNESS_THRESHOLD_PERCENT) / percent_span
            y = _hermite_ease(x, d0, END_SLOPE_NORM)
            mapped = anchor_raw + y * curve_range
        ha2dev[ha] = _clamp_value(
            int(round(mapped)), DEVICE_LEVEL_MIN, DEVICE_LEVEL_MAX
        )
        # Ensure monotonic (allow plateaus for equal percents; no forced +1)
        if ha > 0 and ha2dev[ha] < ha2dev[ha - 1]:
            ha2dev[ha] = ha2dev[ha - 1]

    # Guarantee final convergence
    ha2dev[MAX_LEVEL] = DEVICE_LEVEL_MAX

    # Inverse: first HA producing raw assigned
    dev2ha = [0] * (DEVICE_LEVEL_MAX + 1)
    for ha in range(0, MAX_LEVEL + 1):
        raw = ha2dev[ha]
        if raw <= DEVICE_LEVEL_MAX and dev2ha[raw] == 0:
            dev2ha[raw] = ha
    # Forward fill gaps
    last = 0
    for d in range(DEVICE_LEVEL_MAX + 1):
        if dev2ha[d] == 0 and d != 0:
            dev2ha[d] = last
        else:
            last = dev2ha[d]
    return ha2dev, dev2ha


# Precompute global tables once at import
BRIGHTNESS_HA2DEV, BRIGHTNESS_DEV2HA = _build_new_brightness_tables()
_LOGGER.warning(
    "Brightness tables (percent-based) built: threshold_percent=%s anchor_raw=%s",
    BRIGHTNESS_THRESHOLD_PERCENT,
    BRIGHTNESS_HA2DEV[int(round(BRIGHTNESS_THRESHOLD_PERCENT / 100 * 255))],
)


class Color(CustomCluster, ZigpyColor):
    """Custom Color cluster with CT mapping and CT snapping."""

    LOG_MIN = LOGICAL_MIN_MIRED  # 154
    LOG_MAX = LOGICAL_MAX_MIRED  # 370
    DEV_MIN = DEVICE_MIN_MIRED  # 142
    DEV_MAX = DEVICE_MAX_MIRED  # 454

    def __init__(self, *args, **kwargs):
        """Initialize cluster and seed cache with corrected limits/capabilities."""
        super().__init__(*args, **kwargs)
        # Seed cache for corrected CT limits only
        super()._update_attribute(ATTR_CT_MIN, self.LOG_MIN)
        super()._update_attribute(ATTR_CT_MAX, self.LOG_MAX)
        # Default color_mode unknown initially; leave as-is until first set
        _LOGGER.warning(
            "Color.__init__: logical_ct_limits=%s-%s device_ct_limits=%s-%s",
            self.LOG_MIN,
            self.LOG_MAX,
            self.DEV_MIN,
            self.DEV_MAX,
        )

    def _map_ct_logical_to_device(self, mired: int) -> int:
        return self._convert_logical_mireds_to_device(mired)

    def _map_ct_device_to_logical(self, dev_mired: int) -> int:
        return self._convert_device_mireds_to_logical(dev_mired)

    def _nearest_ct_uv(self, x: float, y: float):
        mired, dist_uv, _ = self._nearest_ct_in_uv(x, y)
        m = mired if mired is not None else self.LOG_MIN
        d = dist_uv if dist_uv is not None else float("inf")
        return int(m), float(d)

    def _should_snap_to_ct(self, x: float, y: float):
        """Pure decision helper: decide if xy should snap to CT.

        Returns a dict: { snap: bool, mired: int, uv_dist: float, epsilon: float, reason: str }
        No logging and no state changes.
        """
        mired, dist_uv = self._nearest_ct_uv(x, y)
        snapped = bool(dist_uv <= UV_CT_SNAP_EPSILON)
        return {
            "snap": snapped,
            "mired": int(mired),
            "uv_dist": float(dist_uv),
            "epsilon": float(UV_CT_SNAP_EPSILON),
            "reason": "uv<=epsilon" if snapped else "uv>epsilon",
        }

    async def _emit_move_to_ct(self, dev_mired: int, transition: int):
        return await super().command(
            CMD_MOVE_TO_COLOR_TEMP, int(dev_mired), int(transition)
        )

    async def _emit_move_to_xy(self, x16: int, y16: int, transition: int):
        return await super().command(
            CMD_MOVE_TO_COLOR, int(x16), int(y16), int(transition)
        )

    def _log_ct_cmd(self, command_id, logical_mired, dev_mired):
        _LOGGER.warning(
            "Color.command: cmd=0x%02X logical_ct=%s -> device_ct=%s",
            command_id,
            logical_mired,
            dev_mired,
        )

    def _log_xy_ct_snap(
        self, x_in, y_in, xc, yc, uv_dist, mired, dev_mired, epsilon, trans
    ):
        _LOGGER.warning(
            "Color.command: XY->CT snap: in=(%s,%s) xy=(%.4f,%.4f) locus_uv_dist=%.4f mired=%s dev_mired=%s eps=%.4f trans=%s",
            x_in,
            y_in,
            xc,
            yc,
            uv_dist,
            mired,
            dev_mired,
            epsilon,
            trans,
        )

    def _log_xy_no_snap(self, uv_dist, epsilon, xc, yc):
        _LOGGER.warning(
            "Color.command: XY no-snap: locus_uv_dist=%.4f > eps=%.4f (x=%.4f,y=%.4f)",
            uv_dist,
            epsilon,
            xc,
            yc,
        )

    def _log_hs_ct_snap(
        self, h_deg, s_raw, xc, yc, uv_dist, mired, dev_mired, epsilon, trans
    ):
        _LOGGER.warning(
            "Color.command: HS->XY->CT snap: h_deg=%s s_raw=%s -> xy=(%.4f,%.4f) locus_uv_dist=%.4f mired=%s dev_mired=%s eps=%.4f trans=%s",
            int(round(h_deg)),
            int(round(s_raw)),
            xc,
            yc,
            uv_dist,
            mired,
            dev_mired,
            epsilon,
            trans,
        )

    def _log_hs_no_snap(self, uv_dist, epsilon, xc, yc):
        _LOGGER.warning(
            "Color.command: HS->XY no-snap: locus_uv_dist=%.4f > eps=%.4f (x=%.4f,y=%.4f)",
            uv_dist,
            epsilon,
            xc,
            yc,
        )

    def _set_mode_ct(self):
        with contextlib.suppress(Exception):
            super()._update_attribute(ATTR_COLOR_MODE, 0x02)

    def _set_mode_xy(self):
        with contextlib.suppress(Exception):
            super()._update_attribute(ATTR_COLOR_MODE, 0x01)

    async def _handle_move_to_color_temp(self, mired: int, trans: int):
        dev_mired = self._map_ct_logical_to_device(mired)
        self._log_ct_cmd(CMD_MOVE_TO_COLOR_TEMP, mired, dev_mired)
        self._set_mode_ct()
        return await self._emit_move_to_ct(dev_mired, trans)

    async def _handle_xy_common(
        self,
        xf: float,
        yf: float,
        trans: int,
        *,
        kind: str,
        x_in: int | None = None,
        y_in: int | None = None,
        h_deg: float | None = None,
        s_raw: float | None = None,
    ):
        """Shared flow: decide CT snap, set mode, and emit."""
        xc, yc = float(xf), float(yf)
        decision = self._should_snap_to_ct(xc, yc)
        if decision["snap"]:
            dev_mired = self._map_ct_logical_to_device(int(decision["mired"]))
            if kind == "xy":
                self._log_xy_ct_snap(
                    x_in,
                    y_in,
                    xc,
                    yc,
                    decision["uv_dist"],
                    decision["mired"],
                    dev_mired,
                    decision["epsilon"],
                    trans,
                )
            else:
                self._log_hs_ct_snap(
                    float(h_deg or 0.0),
                    float(s_raw or 0.0),
                    xc,
                    yc,
                    decision["uv_dist"],
                    decision["mired"],
                    dev_mired,
                    decision["epsilon"],
                    trans,
                )
            self._set_mode_ct()
            return await self._emit_move_to_ct(dev_mired, trans)

        if kind == "xy":
            self._log_xy_no_snap(decision["uv_dist"], decision["epsilon"], xc, yc)
        else:
            self._log_hs_no_snap(decision["uv_dist"], decision["epsilon"], xc, yc)
        xi, yi = self._float_to_xy16(xc), self._float_to_xy16(yc)
        self._set_mode_xy()
        return await self._emit_move_to_xy(xi, yi, trans)

    async def _handle_move_to_color(self, x_in: int, y_in: int, trans: int):
        xf, yf = self._xy16_to_float(x_in), self._xy16_to_float(y_in)
        return await self._handle_xy_common(
            xf,
            yf,
            trans,
            kind="xy",
            x_in=int(x_in),
            y_in=int(y_in),
        )

    async def _handle_move_to_hs(
        self, h_deg: float, s01: float, trans: int, enhanced: bool
    ):
        xf, yf = self._hsv_to_xy(h_deg, s01)
        s_raw = s01 * 254.0
        return await self._handle_xy_common(
            xf,
            yf,
            trans,
            kind="hs",
            h_deg=float(h_deg),
            s_raw=float(s_raw),
        )

    def _hsv_to_xy(self, h_deg: float, s01: float):
        C = max(0.0, min(1.0, float(s01)))
        hp = (float(h_deg) / 60.0) % 6.0
        x_comp = C * (1 - abs((hp % 2.0) - 1.0))
        if 0.0 <= hp < 1.0:
            r1, g1, b1 = C, x_comp, 0.0
        elif 1.0 <= hp < 2.0:
            r1, g1, b1 = x_comp, C, 0.0
        elif 2.0 <= hp < 3.0:
            r1, g1, b1 = 0.0, C, x_comp
        elif 3.0 <= hp < 4.0:
            r1, g1, b1 = 0.0, x_comp, C
        elif 4.0 <= hp < 5.0:
            r1, g1, b1 = x_comp, 0.0, C
        else:
            r1, g1, b1 = C, 0.0, x_comp
        # Inverse gamma to linear (use static helper to avoid scoping confusion)
        rl, gl, bl = self._inv_gamma(r1), self._inv_gamma(g1), self._inv_gamma(b1)
        x_val = 0.4124 * rl + 0.3576 * gl + 0.1805 * bl
        y_val = 0.2126 * rl + 0.7152 * gl + 0.0722 * bl
        z_val = 0.0193 * rl + 0.1192 * gl + 0.9505 * bl
        denom = x_val + y_val + z_val
        if denom <= 0:
            return 0.3127, 0.3290
        return x_val / denom, y_val / denom

    @staticmethod
    def _inv_gamma(u: float) -> float:
        return (u / 12.92) if u <= 0.04045 else (((u + 0.055) / 1.055) ** 2.4)

    def _parse_move_to_color_temp(self, args, kwargs):
        """Return (mired, transition). Do not map/log here."""
        mired = None
        transition = None
        if kwargs:
            if "color_temp_mireds" in kwargs:
                mired = kwargs.get("color_temp_mireds")
            elif "color_temperature" in kwargs:
                mired = kwargs.get("color_temperature")
            elif "color_temp" in kwargs:
                mired = kwargs.get("color_temp")
            transition = kwargs.get("transition_time")
        if mired is None and args:
            mired = args[0]
            transition = args[1] if len(args) > 1 else transition
        if transition is None:
            transition = DEFAULT_COLOR_TRANSITION_TENTHS
        return int(mired if mired is not None else 0), int(transition)

    def _parse_move_to_color(self, args, kwargs):
        """Return (x16, y16, transition). Do not clamp/log here."""
        x16 = None
        y16 = None
        transition = None
        if kwargs:
            x16 = kwargs.get("color_x", kwargs.get("current_x"))
            y16 = kwargs.get("color_y", kwargs.get("current_y"))
            transition = kwargs.get("transition_time")
        if x16 is None and args:
            x16 = args[0]
        if y16 is None and args and len(args) > 1:
            y16 = args[1]
        if transition is None:
            transition = (
                args[2] if args and len(args) > 2 else DEFAULT_COLOR_TRANSITION_TENTHS
            )
        return int(x16 or 0), int(y16 or 0), int(transition)

    def _parse_move_to_hs(self, args, kwargs, enhanced: bool):  # noqa: C901
        """Return (hue_deg, sat01, transition). Input hue/sat untouched otherwise."""
        hue = None
        sat = None
        transition = None
        if kwargs:
            hue = kwargs.get("enhanced_hue" if enhanced else "hue")
            sat = kwargs.get("saturation")
            transition = kwargs.get("transition_time")
        if hue is None and args:
            hue = args[0]
        if sat is None and args and len(args) > 1:
            sat = args[1]
        if transition is None:
            transition = (
                args[2] if args and len(args) > 2 else DEFAULT_COLOR_TRANSITION_TENTHS
            )
        # Normalize
        if enhanced:
            h_deg = (int(hue or 0) % 65536) * (360.0 / 65535.0)
        else:
            h_deg = (int(hue or 0) % 255) * (360.0 / 254.0)
        s01 = _clamp_value(int(sat or 0), 0, 254) / 254.0
        return float(h_deg), float(s01), int(transition)

    @staticmethod
    def _xy16_to_float(v: int) -> float:
        try:
            iv = int(v)
        except Exception:  # noqa: BLE001
            return 0.0
        iv = max(iv, 0)
        iv = min(iv, 65535)
        return iv / 65535.0

    @staticmethod
    def _float_to_xy16(f: float) -> int:
        try:
            val = float(f)
        except Exception:  # noqa: BLE001
            val = 0.0
        val = max(val, 0.0)
        val = min(val, 1.0)
        return int(round(val * 65535.0))

    @staticmethod
    def _xy_from_cct(cct: float):
        T = float(cct)
        T = max(T, 1667)
        T = min(T, 25000)
        if T <= 4000:
            x = (
                -0.2661239 * (1e9 / (T**3))
                - 0.2343589 * (1e6 / (T**2))
                + 0.8776956 * (1e3 / T)
                + 0.179910
            )
        else:
            x = (
                -3.0258469 * (1e9 / (T**3))
                + 2.1070379 * (1e6 / (T**2))
                + 0.2226347 * (1e3 / T)
                + 0.240390
            )
        if T <= 2222:
            y = -1.1063814 * (x**3) - 1.34811020 * (x**2) + 2.18555832 * x - 0.20219683
        elif T <= 4000:
            y = -0.9549476 * (x**3) - 1.37418593 * (x**2) + 2.09137015 * x - 0.16748867
        else:
            y = 3.0817580 * (x**3) - 5.87338670 * (x**2) + 3.75112997 * x - 0.37001483
        return x, y

    @staticmethod
    def _cct_from_xy(x: float, y: float) -> float:
        n = (x - 0.3320) / (y - 0.1858) if (y - 0.1858) != 0 else 0.0
        cct = 449 * (n**3) + 3525 * (n**2) + 6823.3 * n + 5520.33
        # Bound for sanity
        cct = max(cct, 1000)
        cct = min(cct, 25000)
        return cct

    @staticmethod
    def _xy_to_uv(x: float, y: float):
        denom = -2 * x + 12 * y + 3
        if denom == 0:
            return 0.0, 0.0
        u = (4 * x) / denom
        v = (9 * y) / denom
        return u, v

    def _nearest_ct_in_uv(self, x: float, y: float):
        # Search logical CT range and find closest point on locus in u'v'
        u_in, v_in = self._xy_to_uv(x, y)
        best_mired = None
        best_dist = float("inf")
        best_xy = None
        for mired in range(self.LOG_MIN, self.LOG_MAX + 1):
            T = 1_000_000.0 / float(mired)
            cx, cy = self._xy_from_cct(T)
            u, v = self._xy_to_uv(cx, cy)
            d = math.hypot(u - u_in, v - v_in)
            if d < best_dist:
                best_dist = d
                best_mired = mired
                best_xy = (cx, cy)
        return best_mired, best_dist, best_xy

    async def bind(self):
        """Bind cluster and push corrected attributes to cache."""
        res = await super().bind()
        super()._update_attribute(ATTR_CT_MIN, self.LOG_MIN)
        super()._update_attribute(ATTR_CT_MAX, self.LOG_MAX)
        _LOGGER.warning(
            "Color.bind: enforced logical_ct_limits=%s-%s",
            self.LOG_MIN,
            self.LOG_MAX,
        )
        return res

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
        """Intercept Color commands to apply CT mapping and CT snapping."""
        # Dispatcher per command
        if command_id == CMD_MOVE_TO_COLOR_TEMP:
            # Narrow: only guard parsing issues; let handler errors surface
            try:
                mired, trans = self._parse_move_to_color_temp(args, kwargs)
            except (TypeError, ValueError, KeyError) as ex:
                _LOGGER.error("Color.command: parse error for MoveToColorTemp: %s", ex)
                return await super().command(
                    command_id,
                    *args,
                    manufacturer=manufacturer,
                    expect_reply=expect_reply,
                    tsn=tsn,
                    **kwargs,
                )
            return await self._handle_move_to_color_temp(mired, trans)
        elif command_id == CMD_MOVE_TO_COLOR:
            try:
                x_in, y_in, trans = self._parse_move_to_color(args, kwargs)
            except (TypeError, ValueError, KeyError) as ex:
                _LOGGER.error("Color.command: parse error for MoveToColor: %s", ex)
                return await super().command(
                    command_id,
                    *args,
                    manufacturer=manufacturer,
                    expect_reply=expect_reply,
                    tsn=tsn,
                    **kwargs,
                )
            return await self._handle_move_to_color(x_in, y_in, trans)
        elif command_id in (CMD_MOVE_TO_HUE_SAT, CMD_ENHANCED_MOVE_TO_HUE_SAT):
            enhanced = command_id == CMD_ENHANCED_MOVE_TO_HUE_SAT
            try:
                h_deg, s, trans = self._parse_move_to_hs(args, kwargs, enhanced)
            except (TypeError, ValueError, KeyError) as ex:
                _LOGGER.error(
                    "Color.command: parse error for %s: %s",
                    "EnhancedMoveToHueSat" if enhanced else "MoveToHueSat",
                    ex,
                )
                return await super().command(
                    command_id,
                    *args,
                    manufacturer=manufacturer,
                    expect_reply=expect_reply,
                    tsn=tsn,
                    **kwargs,
                )
            return await self._handle_move_to_hs(h_deg, s, trans, enhanced)

        return await super().command(
            command_id,
            *args,
            manufacturer=manufacturer,
            expect_reply=expect_reply,
            tsn=tsn,
            **kwargs,
        )

    async def write_attributes(self, attributes, manufacturer=None):
        """Rewrite CT attributes to device range on write when active."""
        try:
            attrs = dict(attributes)
            for k, v in attrs.items():
                if k in (ATTR_COLOR_TEMP, "color_temperature"):
                    before = v
                    attrs[k] = self._map_ct_logical_to_device(v)
                    _LOGGER.warning(
                        "Color.write_attributes: logical_ct=%s -> device_ct=%s",
                        before,
                        attrs[k],
                    )

        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("Color.write_attributes: exception: %s", ex)
        return await super().write_attributes(attributes, manufacturer=manufacturer)

    async def read_attributes(  # noqa: C901
        self, attributes, allow_cache=True, only_cache=False, manufacturer=None
    ):
        """Normalize CT attributes to logical range."""
        _LOGGER.warning(
            "Color.read_attributes(entry): attrs=%s type=%s allow_cache=%s only_cache=%s",
            attributes,
            type(attributes).__name__,
            allow_cache,
            only_cache,
        )

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

        original_is_tuple = isinstance(result_tuple, tuple) and len(result_tuple) == 2
        if original_is_tuple:
            result, failure = result_tuple
            if not isinstance(result, dict):
                result = dict(result or {})
            else:
                result = dict(result)
        else:
            failure = {}
            result = result_tuple if isinstance(result_tuple, dict) else {}
            result = dict(result)

        if (
            (
                ATTR_CT_MIN in requested_ids
                or "color_temp_physical_min_mireds" in requested_names
                or "color_temp_physical_min" in requested_names
            )
            and ATTR_CT_MIN not in result
            and "color_temp_physical_min_mireds" not in result
            and "color_temp_physical_min" not in result
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
                or "color_temp_physical_max" in requested_names
            )
            and ATTR_CT_MAX not in result
            and "color_temp_physical_max_mireds" not in result
            and "color_temp_physical_max" not in result
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
        set_val(ATTR_CT_MIN, "color_temp_physical_min", self.LOG_MIN)
        set_val(ATTR_CT_MAX, "color_temp_physical_max_mireds", self.LOG_MAX)
        set_val(ATTR_CT_MAX, "color_temp_physical_max", self.LOG_MAX)

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
        if "color_temp_physical_min" in result:
            raw = result["color_temp_physical_min"]
            if raw != self.LOG_MIN:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MIN(short) raw=%s -> %s",
                    raw,
                    self.LOG_MIN,
                )
                result["color_temp_physical_min"] = self.LOG_MIN
            else:
                _LOGGER.warning(
                    "Color.read_attributes: CT_MIN(short) already logical=%s", raw
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
        if "color_temp_physical_max" in result:
            raw = result["color_temp_physical_max"]
            if raw != self.LOG_MAX:
                _LOGGER.warning(
                    "Color.read_attributes: correcting CT_MAX(short) raw=%s -> %s",
                    raw,
                    self.LOG_MAX,
                )
                result["color_temp_physical_max"] = self.LOG_MAX
            else:
                _LOGGER.warning(
                    "Color.read_attributes: CT_MAX(short) already logical=%s", raw
                )

        if ATTR_COLOR_TEMP in result or "color_temperature" in result:
            raw = result.get(ATTR_COLOR_TEMP, result.get("color_temperature"))
            if isinstance(raw, int):
                try:
                    mapped = self._map_ct_device_to_logical(raw)
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

        try:
            interesting = {}
            for key in (ATTR_CT_MIN, ATTR_CT_MAX, ATTR_COLOR_TEMP):
                if key in result:
                    interesting[f"0x{key:04X}"] = result[key]
            for name in (
                "color_temp_physical_min_mireds",
                "color_temp_physical_min",
                "color_temp_physical_max_mireds",
                "color_temp_physical_max",
                "color_temperature",
            ):
                if name in result:
                    interesting[name] = result[name]
            _LOGGER.warning("Color.read_attributes(exit): normalized=%s", interesting)
        except Exception:  # noqa: BLE001
            pass

        return (result, failure) if original_is_tuple else result

    def _update_attribute(self, attrid, value):
        try:
            original = value
            if attrid == ATTR_CT_MIN:
                value = self.LOG_MIN
            elif attrid == ATTR_CT_MAX:
                value = self.LOG_MAX
            elif attrid == ATTR_COLOR_TEMP:
                value = self._map_ct_device_to_logical(value)
                with contextlib.suppress(Exception):
                    super()._update_attribute(ATTR_COLOR_MODE, 0x02)
            elif attrid in (ATTR_CURRENT_X, ATTR_CURRENT_Y):
                with contextlib.suppress(Exception):
                    super()._update_attribute(ATTR_COLOR_MODE, 0x01)
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


class LevelControl(CustomCluster, ZigpyLevelControl):
    """Perceptual mapping (+ anti-OFF guards) for the 3R ZL1.

    Uses LUTs for idempotent round-trip and hysteresis to stop slider bounce.
    """

    def __init__(self, *args, **kwargs):
        """Build LUTs and initialize hysteresis."""
        super().__init__(*args, **kwargs)
        self._ha2dev = None
        self._dev2ha = None
        self._last_dev = None
        self._last_ha = None
        self._hysteresis = HYSTERESIS_COUNTS
        self._sticky_until = 0.0
        self._build_brightness_lookup_tables()
        _LOGGER.warning("LevelControl.__init__: hysteresis=%s", self._hysteresis)

    def _build_brightness_lookup_tables(self):
        """Attach precomputed brightness LUTs (deep copies to allow per-instance tweaks)."""
        self._ha2dev = list(BRIGHTNESS_HA2DEV)
        self._dev2ha = list(BRIGHTNESS_DEV2HA)
        _LOGGER.warning(
            "LevelControl._build_brightness_lookup_tables: tables_attached threshold=%s start_slope=%s end_slope=%s",
            BRIGHTNESS_THRESHOLD_PERCENT,
            START_SLOPE_NORM,
            END_SLOPE_NORM,
        )

    def _now(self) -> float:
        return time.monotonic()

    def _map_brightness_level(self, v: int) -> int:
        if self._ha2dev is None:
            v = _clamp_value(int(v), MIN_LEVEL, MAX_LEVEL)
            p = _ha_raw_to_percent(v)
            if p <= BRIGHTNESS_THRESHOLD_PERCENT:
                mapped = 1 if p <= 2 else p
            else:
                frac = (p - BRIGHTNESS_THRESHOLD_PERCENT) / (
                    100 - BRIGHTNESS_THRESHOLD_PERCENT
                )
                mapped = _clamp_value(
                    int(
                        round(
                            BRIGHTNESS_THRESHOLD_PERCENT
                            + frac * (DEVICE_LEVEL_MAX - BRIGHTNESS_THRESHOLD_PERCENT)
                        )
                    ),
                    DEVICE_LEVEL_MIN,
                    DEVICE_LEVEL_MAX,
                )
            _LOGGER.warning(
                "LevelControl._map_brightness_level(fallback): ha=%s p=%s device=%s",
                v,
                p,
                mapped,
            )
            return mapped
        v = _clamp_value(int(v), MIN_LEVEL, MAX_LEVEL)
        p = _ha_raw_to_percent(v)
        mapped = int(self._ha2dev[v])
        _LOGGER.warning(
            "LevelControl._map_brightness_level: ha=%s p=%s device=%s (LUT)",
            v,
            p,
            mapped,
        )
        return mapped

    def _convert_device_level_to_ha(self, dev: int) -> int:
        if self._dev2ha is None:
            d = _clamp_value(int(dev), DEVICE_LEVEL_MIN, DEVICE_LEVEL_MAX)
            if d == 0:
                return 0
            if d <= BRIGHTNESS_THRESHOLD_PERCENT:
                return int(round(d * 255 / 100))
            high_frac = (d - BRIGHTNESS_THRESHOLD_PERCENT) / (
                DEVICE_LEVEL_MAX - BRIGHTNESS_THRESHOLD_PERCENT
            )
            p = BRIGHTNESS_THRESHOLD_PERCENT + high_frac * (
                100 - BRIGHTNESS_THRESHOLD_PERCENT
            )
            return int(round(p * 255 / 100))
        d = _clamp_value(int(dev), MIN_LEVEL, MAX_LEVEL)
        if self._last_ha is None:
            _LOGGER.warning(
                "LevelControl._convert_device_level_to_ha: device=%s -> ha=%s (group-compat identity)",
                d,
                d,
            )
            return int(d)
        now = self._now()
        if self._last_ha is not None and now <= (self._sticky_until or 0):
            _LOGGER.warning(
                "LevelControl._convert_device_level_to_ha: device=%s -> ha=%s (sticky-window remain=%.3fs)",
                d,
                self._last_ha,
                (self._sticky_until - now),
            )
            return int(self._last_ha)
        if self._last_dev is not None and abs(d - self._last_dev) <= self._hysteresis:
            chosen = int(
                self._last_ha if self._last_ha is not None else self._dev2ha[d]
            )
            _LOGGER.warning(
                "LevelControl._convert_device_level_to_ha: device=%s -> ha=%s (hysteresis<=%s last_dev=%s)",
                d,
                chosen,
                self._hysteresis,
                self._last_dev,
            )
            return chosen
        mapped = int(self._dev2ha[d])
        _LOGGER.warning(
            "LevelControl._convert_device_level_to_ha: device=%s -> ha=%s (normal)",
            d,
            mapped,
        )
        return mapped

    def _remember_set(self, ha_level: int):
        """Call after sending a level to make inbound reports sticky."""
        self._last_ha = _clamp_value(int(ha_level), MIN_LEVEL, MAX_LEVEL)
        self._last_dev = self._map_brightness_level(self._last_ha)
        self._sticky_until = self._now() + STICKY_WINDOW_S
        _LOGGER.warning(
            "LevelControl._remember_set: ha=%s last_dev=%s sticky_window=%.3fs",
            self._last_ha,
            self._last_dev,
            STICKY_WINDOW_S,
        )

    def _avoid_zero_result(self, cmd_id: int, dev_level: int) -> int:
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

    async def command(
        self,
        command_id,
        *args,
        manufacturer=None,
        expect_reply=True,
        tsn=None,
        **kwargs,
    ):
        """Map HA level to device level and apply anti-OFF guards."""

        _LOGGER.warning(
            "LevelControl.command: cmd=0x%02X args=%s kwargs=%s",
            command_id,
            args,
            kwargs,
        )

        if command_id in (CMD_MOVE_TO_LEVEL, CMD_MOVE_TO_LEVEL_WITH_ON_OFF):
            return await self._handle_move_command(command_id, *args)

        elif command_id in (CMD_STEP, CMD_STEP_WITH_ON_OFF):
            return await self._handle_step_command(command_id, *args)

        return await super().command(command_id, *args, **kwargs)

    async def _handle_move_command(self, command_id, *args):
        level = args[0] if args else None
        transition_time = args[1] if len(args) >= 2 else DEFAULT_TRANSITION_TIME
        if level is not None:
            self._remember_set(int(level))
            p = _ha_raw_to_percent(int(level))
            mapped_level = self._map_brightness_level(level)
            _LOGGER.warning(
                "LevelControl._handle_move_command: cmd=0x%02X ha=%s p=%s mapped=%s transition=%s",
                command_id,
                level,
                p,
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

    async def move_to_level(self, level, transition_time):
        """Route to command 0x00 with mapping and stickiness."""

        self._remember_set(int(level))
        p = _ha_raw_to_percent(int(level))
        mapped = self._map_brightness_level(level)
        if int(level) > 0 and mapped == 0:
            mapped = 1
        _LOGGER.warning(
            "LevelControl.move_to_level: ha=%s p=%s mapped=%s transition=%s",
            level,
            p,
            mapped,
            transition_time,
        )
        return await super().command(CMD_MOVE_TO_LEVEL, mapped, transition_time)

    async def move_to_level_with_on_off(self, level, transition_time):
        """Route to command 0x04 with mapping and anti-OFF rewrite."""

        self._remember_set(int(level))
        p = _ha_raw_to_percent(int(level))
        mapped = self._map_brightness_level(level)
        mapped = self._avoid_zero_result(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, mapped)
        if mapped <= 1 and int(level) > 0:
            return await super().command(CMD_MOVE_TO_LEVEL, 1, transition_time)
        _LOGGER.warning(
            "LevelControl.move_to_level_with_on_off: ha=%s p=%s mapped=%s transition=%s",
            level,
            p,
            mapped,
            transition_time,
        )
        return await super().command(
            CMD_MOVE_TO_LEVEL_WITH_ON_OFF, mapped, transition_time
        )

    async def write_attributes(self, attributes, manufacturer=None):
        """Rewrite current_level writes to mapped device values."""

        try:
            attrs = dict(attributes)
            for k, v in attrs.items():
                if k in (ATTR_CURRENT_LEVEL, "current_level", "level"):
                    iv = int(v)
                    self._remember_set(iv)
                    p = _ha_raw_to_percent(iv)
                    attrs[k] = self._map_brightness_level(iv)
                    _LOGGER.warning(
                        "LevelControl.write_attributes: ha=%s p=%s -> device=%s",
                        iv,
                        p,
                        attrs[k],
                    )
            attributes = attrs
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("LevelControl.write_attributes: exception: %s", ex)
        return await super().write_attributes(attributes, manufacturer=manufacturer)

    def _update_attribute(self, attrid, value):
        try:
            if attrid == ATTR_CURRENT_LEVEL:  # CurrentLevel
                raw = int(value)
                value = self._convert_device_level_to_ha(int(value))
                _LOGGER.warning(
                    "LevelControl._update_attribute: device=%s -> ha=%s (sticky_until=%.3f now=%.3f last_dev=%s last_ha=%s)",
                    raw,
                    value,
                    self._sticky_until,
                    self._now(),
                    self._last_dev,
                    self._last_ha,
                )
        except Exception as ex:  # noqa: BLE001
            _LOGGER.error("LevelControl._update_attribute: exception: %s", ex)
        return super()._update_attribute(attrid, value)

    async def read_attributes(  # noqa: C901
        self, attributes, allow_cache=True, only_cache=False, manufacturer=None
    ):
        """Normalize current_level results to HA levels."""
        _LOGGER.warning(
            "LevelControl.read_attributes(entry): attrs=%s type=%s allow_cache=%s only_cache=%s",
            attributes,
            type(attributes).__name__,
            allow_cache,
            only_cache,
        )
        result = await super().read_attributes(
            attributes,
            allow_cache=allow_cache,
            only_cache=only_cache,
            manufacturer=manufacturer,
        )
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
