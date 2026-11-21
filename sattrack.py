#!/usr/bin/env python3
"""Sat Tracker - satellite pass predictor for M5Stack CardputerZero (+ Cap LoRa-1262 GPS).

Predicts when the ISS and other satellites pass over you, where to look, and
whether you can see them with your own eyes. Orbit data (TLEs) comes from
CelesTrak over Wi-Fi and is cached; the orbit model (SGP4) is built in, so no
extra packages are needed.

Pages
  1 Passes   the next 24 hours of passes: time, satellite, highest point,
             direction; a star marks passes you can see with the naked eye
  2 Pass     one pass in detail: sky chart of its path, countdown, rise /
             peak / set times and directions, live position while overhead
  3 Sky      every tracked satellite above the horizon right now
  4 Setup    satellite group, update orbit data, location, lowest pass,
             units, clock check

Keys (the red arrows printed on the keys)
  Z / C  previous / next page        F / X  move up / down
  Enter  open pass / change setting  V      naked-eye passes only
  U      update orbit data           L      type your location
  ESC    quit

Run with --preview out.png PAGE --mock to render without hardware.
"""

import fcntl
import glob
import json
import math
import mmap
import os
import select
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from datetime import datetime, timezone

import numpy as np
HAS_NUMPY = True
from PIL import Image, ImageDraw, ImageFont

VERSION = "1.0.3"
W, H = 320, 170
FB_DEVICE = os.environ.get("APPLAUNCH_LINUX_FBDEV_DEVICE", "/dev/fb0")
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
EXT5V = "/sys/class/leds/ext_5v_out/brightness"

HOME = os.path.expanduser("~")
DATA_DIR = os.path.join(HOME, ".local", "share", "pocket-sattrack")
CONF_PATH = os.path.join(HOME, ".config", "pocket-sattrack", "config.json")
CELESTRAK = "https://celestrak.org/NORAD/elements/gp.php?GROUP={}&FORMAT=tle"
MIN_FETCH_INTERVAL = 2 * 3600       # CelesTrak asks clients not to re-download within 2 hours
AUTO_FETCH_AGE = 24 * 3600
MAX_SATS = 200

GROUPS = [("stations", "Space stations"), ("visual", "Brightest (naked eye)"),
          ("amateur", "Amateur radio"), ("weather", "Weather")]
MIN_ELS = [10, 20, 30, 0]

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602
EV_KEY = 0x01
KEY_ESC, KEY_1, KEY_2, KEY_3, KEY_4 = 1, 2, 3, 4, 5
KEY_0, KEY_MINUS, KEY_BACKSPACE, KEY_TAB, KEY_Q = 11, 12, 14, 15, 16
KEY_W, KEY_E, KEY_U, KEY_ENTER = 17, 18, 22, 28
KEY_S, KEY_F, KEY_L, KEY_SEMICOLON = 31, 33, 38, 39
KEY_Z, KEY_X, KEY_C, KEY_V, KEY_N = 44, 45, 46, 47, 49
KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_SPACE = 51, 52, 53, 57
KEY_UP, KEY_LEFT, KEY_RIGHT, KEY_DOWN = 103, 105, 106, 108
# Cardputer Zero symbol layer: sym+B sends KEY_HIRAGANA (91), sym+H sends
# KEY_KPDOT (83) and sym+R sends KEY_KP7 (71) instead of the PC codes.
KEY_KP7, KEY_KPDOT, KEY_HIRAGANA = 71, 83, 91
NUM_KEYS = {KEY_1: 0, KEY_2: 1, KEY_3: 2, KEY_4: 3}
DIGITS = {2: "1", 3: "2", 4: "3", 5: "4", 6: "5", 7: "6", 8: "7", 9: "8", 10: "9", 11: "0"}
TEXT_KEYS = dict(DIGITS)
TEXT_KEYS.update({KEY_DOT: ".", KEY_KPDOT: ".", KEY_HIRAGANA: ".",
                  KEY_COMMA: ",", KEY_MINUS: "-", KEY_KP7: "-", KEY_SPACE: " ",
                  KEY_N: "N", KEY_S: "S", KEY_E: "E", KEY_W: "W"})

PAGES = ["Passes", "Pass", "Sky", "Setup"]
P_LIST, P_PASS, P_SKY, P_SETUP = range(4)

BG = (10, 12, 16)
PANEL = (24, 28, 36)
GRID = (40, 46, 58)
SEL = (34, 52, 74)
TEXT = (230, 234, 240)
DIM = (130, 138, 152)
GREEN = (60, 220, 130)
CYAN = (60, 190, 255)
YELLOW = (255, 205, 60)
ORANGE = (255, 140, 50)
RED = (255, 75, 75)
KEY_RED = (235, 70, 70)   # the red arrow markings on the keyboard
PURPLE = (175, 125, 255)
STAR = "★"

_running = True


def _stop(_s, _f):
    global _running
    _running = False


def load_font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for path in (os.path.join(FONT_DIR, name), "/usr/share/fonts/TTF/" + name):
        try:
            return ImageFont.truetype(path, size)
        except (OSError, IOError):
            pass
    return ImageFont.load_default()


def compass(az):
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int((az % 360) / 45.0 + 0.5) % 8]


def fmt_in(sec):
    sec = int(sec)
    if sec < 60:
        return "{} s".format(max(0, sec))
    if sec < 3600:
        return "{} min".format(sec // 60)
    return "{} h {:02d} m".format(sec // 3600, (sec % 3600) // 60)


def fmt_latlon(lat, lon):
    return "{:.2f}° {} {:.2f}° {}".format(abs(lat), "N" if lat >= 0 else "S", abs(lon), "E" if lon >= 0 else "W")


def parse_latlon(text):
    """'40.71, -74.01' or '40.71 N 74.01 W' -> (lat, lon) or None."""
    t = text.upper().replace(",", " ")
    for h in "NSEW":
        t = t.replace(h, " " + h + " ")
    toks = t.split()
    nums, i = [], 0
    while i < len(toks):
        try:
            v = float(toks[i])
        except ValueError:
            return None
        if i + 1 < len(toks) and toks[i + 1] in "NSEW":
            if toks[i + 1] in "SW":
                v = -abs(v)
            i += 1
        nums.append(v)
        i += 1
    if len(nums) != 2 or not (-90 <= nums[0] <= 90 and -180 <= nums[1] <= 180):
        return None
    return nums[0], nums[1]


def load_config():
    conf = {"group": "stations", "min_el": 10, "units": "mi", "visible_only": False,
            "lat": None, "lon": None, "alt": 0.0, "loc_source": None}
    try:
        with open(CONF_PATH) as f:
            conf.update(json.load(f))
    except (OSError, ValueError):
        pass
    return conf


def save_config(conf):
    try:
        os.makedirs(os.path.dirname(CONF_PATH), exist_ok=True)
        tmp = CONF_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(conf, f, indent=1)
        os.replace(tmp, CONF_PATH)
    except OSError:
        pass


def cap_power_on():
    """Switch on the Cap's 5 V rail (powers its GPS). Returns the previous value or None."""
    try:
        with open(EXT5V) as f:
            prev = int(f.read().strip() or 0)
        if prev <= 0:
            with open(EXT5V, "w") as f:
                f.write("1\n")
        return prev
    except (OSError, ValueError):
        return None


# --------------------------------------------------------------------------
# Orbit math: SGP4 (near-Earth, WGS-72 constants as used by NORAD/CelesTrak),
# vectorised over time with NumPy. Follows Vallado et al., "Revisiting
# Spacetrack Report #3" (AIAA 2006-6753). Deep-space orbits (period >= 225 min,
# e.g. geostationary weather satellites) are skipped.
# --------------------------------------------------------------------------
MU = 398600.8
RE = 6378.135
XKE = 60.0 / math.sqrt(RE ** 3 / MU)
J2, J3, J4 = 0.001082616, -0.00000253881, -0.00000165597
J3OJ2 = J3 / J2
X2O3 = 2.0 / 3.0
VKMPS = RE * XKE / 60.0
TWOPI = 2.0 * math.pi
DEG = math.pi / 180.0
MIN_PER_DAY = 1440.0


class TleError(ValueError):
    pass


def _tle_checksum_ok(line):
    s = 0
    for ch in line[:68]:
        if ch.isdigit():
            s += int(ch)
        elif ch == "-":
            s += 1
    return line[68:69].isdigit() and s % 10 == int(line[68])


def _tle_float(field):
    """Parse TLE 'assumed decimal point' fields like ' 66816-4' -> 0.66816e-4."""
    f = field.strip()
    if not f:
        return 0.0
    sign = 1.0
    if f[0] in "+-":
        sign = -1.0 if f[0] == "-" else 1.0
        f = f[1:]
    k = max(f.rfind("-"), f.rfind("+"))
    mant, exp = (f[:k], int(f[k:])) if k > 0 else (f, 0)
    return sign * float("0." + mant) * 10 ** exp


def jday(year, mon, day, hr, minute, sec):
    return (367.0 * year - int(7 * (year + int((mon + 9) / 12.0)) * 0.25) + int(275 * mon / 9.0) + day
            + 1721013.5 + ((sec / 60.0 + minute) / 60.0 + hr) / 24.0)


def unix_to_jd(t):
    return np.asarray(t, dtype=float) / 86400.0 + 2440587.5


def gmst(jd):
    tut1 = (jd - 2451545.0) / 36525.0
    temp = (-6.2e-6 * tut1 ** 3 + 0.093104 * tut1 ** 2
            + (876600.0 * 3600 + 8640184.812866) * tut1 + 67310.54841)
    return np.mod(temp * DEG / 240.0, TWOPI)


class Satellite:
    def __init__(self, name, line1, line2, check=True):
        line1, line2 = line1.rstrip().ljust(69), line2.rstrip().ljust(69)
        if not line1.startswith("1 ") or not line2.startswith("2 "):
            raise TleError("not a TLE")
        if check and not (_tle_checksum_ok(line1) and _tle_checksum_ok(line2)):
            raise TleError("bad checksum")
        self.name = name.strip() or line2[2:7].strip()
        self.norad = int(line1[2:7])
        yy = int(line1[18:20])
        year = 2000 + yy if yy < 57 else 1900 + yy
        doy = float(line1[20:32])
        self.epoch_jd = jday(year, 1, 0, 0, 0, 0) + doy
        self.bstar = _tle_float(line1[53:61])
        self.inclo = float(line2[8:16]) * DEG
        self.nodeo = float(line2[17:25]) * DEG
        self.ecco = float("0." + line2[26:33].strip())
        self.argpo = float(line2[34:42]) * DEG
        self.mo = float(line2[43:51]) * DEG
        self.no_kozai = float(line2[52:63]) * TWOPI / MIN_PER_DAY
        self.period_min = TWOPI / self.no_kozai
        self.deep = self.period_min >= 225.0
        if not self.deep:
            self._init()

    def _init(self):
        ecco, inclo, argpo, mo, bstar = self.ecco, self.inclo, self.argpo, self.mo, self.bstar
        ss = 78.0 / RE + 1.0
        qzms2t = ((120.0 - 78.0) / RE) ** 4
        cosio = math.cos(inclo)
        cosio2 = cosio * cosio
        eccsq = ecco * ecco
        omeosq = 1.0 - eccsq
        rteosq = math.sqrt(omeosq)
        ak = (XKE / self.no_kozai) ** X2O3
        d1 = 0.75 * J2 * (3.0 * cosio2 - 1.0) / (rteosq * omeosq)
        dl = d1 / (ak * ak)
        adel = ak * (1.0 - dl * dl - dl * (1.0 / 3.0 + 134.0 * dl * dl / 81.0))
        dl = d1 / (adel * adel)
        self.no = no = self.no_kozai / (1.0 + dl)
        ao = (XKE / no) ** X2O3
        sinio = math.sin(inclo)
        po = ao * omeosq
        con42 = 1.0 - 5.0 * cosio2
        self.con41 = con41 = -con42 - cosio2 - cosio2
        posq = po * po
        rp = ao * (1.0 - ecco)
        self.isimp = rp < (220.0 / RE + 1.0)
        sfour, qzms24 = ss, qzms2t
        perige = (rp - 1.0) * RE
        if perige < 156.0:
            sfour = perige - 78.0
            if perige < 98.0:
                sfour = 20.0
            qzms24 = ((120.0 - sfour) / RE) ** 4
            sfour = sfour / RE + 1.0
        pinvsq = 1.0 / posq
        tsi = 1.0 / (ao - sfour)
        self.eta = eta = ao * ecco * tsi
        etasq = eta * eta
        eeta = ecco * eta
        psisq = abs(1.0 - etasq)
        coef = qzms24 * tsi ** 4
        coef1 = coef / psisq ** 3.5
        cc2 = coef1 * no * (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq))
                            + 0.375 * J2 * tsi / psisq * con41 * (8.0 + 3.0 * etasq * (8.0 + etasq)))
        self.cc1 = cc1 = bstar * cc2
        cc3 = -2.0 * coef * tsi * J3OJ2 * no * sinio / ecco if ecco > 1.0e-4 else 0.0
        self.x1mth2 = x1mth2 = 1.0 - cosio2
        self.cc4 = 2.0 * no * coef1 * ao * omeosq * (
            eta * (2.0 + 0.5 * etasq) + ecco * (0.5 + 2.0 * etasq)
            - J2 * tsi / (ao * psisq) * (-3.0 * con41 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta))
                                         + 0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * math.cos(2.0 * argpo)))
        self.cc5 = 2.0 * coef1 * ao * omeosq * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq)
        cosio4 = cosio2 * cosio2
        temp1 = 1.5 * J2 * pinvsq * no
        temp2 = 0.5 * temp1 * J2 * pinvsq
        temp3 = -0.46875 * J4 * pinvsq * pinvsq * no
        self.mdot = no + 0.5 * temp1 * rteosq * con41 + 0.0625 * temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4)
        self.argpdot = (-0.5 * temp1 * con42 + 0.0625 * temp2 * (7.0 - 114.0 * cosio2 + 395.0 * cosio4)
                        + temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4))
        xhdot1 = -temp1 * cosio
        self.nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) + 2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio
        self.omgcof = bstar * cc3 * math.cos(argpo)
        self.xmcof = -X2O3 * coef * bstar / eeta if ecco > 1.0e-4 else 0.0
        self.nodecf = 3.5 * omeosq * xhdot1 * cc1
        self.t2cof = 1.5 * cc1
        den = 1.0 + cosio if abs(cosio + 1.0) > 1.5e-12 else 1.5e-12
        self.xlcof = -0.25 * J3OJ2 * sinio * (3.0 + 5.0 * cosio) / den
        self.aycof = -0.5 * J3OJ2 * sinio
        self.delmo = (1.0 + eta * math.cos(mo)) ** 3
        self.sinmao = math.sin(mo)
        self.x7thm1 = 7.0 * cosio2 - 1.0
        if not self.isimp:
            cc1sq = cc1 * cc1
            self.d2 = d2 = 4.0 * ao * tsi * cc1sq
            temp = d2 * tsi * cc1 / 3.0
            self.d3 = d3 = (17.0 * ao + sfour) * temp
            self.d4 = d4 = 0.5 * temp * ao * tsi * (221.0 * ao + 31.0 * sfour) * cc1
            self.t3cof = d2 + 2.0 * cc1sq
            self.t4cof = 0.25 * (3.0 * d3 + cc1 * (12.0 * d2 + 10.0 * cc1sq))
            self.t5cof = 0.2 * (3.0 * d4 + 12.0 * cc1 * d3 + 6.0 * d2 * d2 + 15.0 * cc1sq * (2.0 * d2 + cc1sq))

    def propagate(self, tsince):
        """tsince: minutes from epoch (scalar or array). Returns r, v in km and km/s (TEME), shape (3, n)."""
        if self.deep:
            raise TleError("deep-space orbit not supported")
        t = np.atleast_1d(np.asarray(tsince, dtype=float))
        xmdf = self.mo + self.mdot * t
        argpdf = self.argpo + self.argpdot * t
        nodedf = self.nodeo + self.nodedot * t
        argpm, mm = argpdf, xmdf
        t2 = t * t
        nodem = nodedf + self.nodecf * t2
        tempa = 1.0 - self.cc1 * t
        tempe = self.bstar * self.cc4 * t
        templ = self.t2cof * t2
        if not self.isimp:
            delomg = self.omgcof * t
            delm = self.xmcof * ((1.0 + self.eta * np.cos(xmdf)) ** 3 - self.delmo)
            temp = delomg + delm
            mm = xmdf + temp
            argpm = argpdf - temp
            t3 = t2 * t
            t4 = t3 * t
            tempa = tempa - self.d2 * t2 - self.d3 * t3 - self.d4 * t4
            tempe = tempe + self.bstar * self.cc5 * (np.sin(mm) - self.sinmao)
            templ = templ + self.t3cof * t3 + t4 * (self.t4cof + t * self.t5cof)
        am = (XKE / self.no) ** X2O3 * tempa * tempa
        nm = XKE / am ** 1.5
        em = np.maximum(self.ecco - tempe, 1.0e-6)
        mm = mm + self.no * templ
        xlm = mm + argpm + nodem
        nodem = np.mod(nodem, TWOPI)
        argpm = np.mod(argpm, TWOPI)
        xlm = np.mod(xlm, TWOPI)
        mm = np.mod(xlm - argpm - nodem, TWOPI)
        sinim, cosim = math.sin(self.inclo), math.cos(self.inclo)
        # long-period periodics
        axnl = em * np.cos(argpm)
        temp = 1.0 / (am * (1.0 - em * em))
        aynl = em * np.sin(argpm) + temp * self.aycof
        xl = mm + argpm + nodem + temp * self.xlcof * axnl
        # Kepler's equation
        u = np.mod(xl - nodem, TWOPI)
        eo1 = u.copy()
        for _ in range(10):
            sineo1, coseo1 = np.sin(eo1), np.cos(eo1)
            tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl
            tem5 = np.clip((u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5, -0.95, 0.95)
            eo1 = eo1 + tem5
            if np.all(np.abs(tem5) < 1.0e-12):
                break
        sineo1, coseo1 = np.sin(eo1), np.cos(eo1)
        # short-period preliminary quantities
        ecose = axnl * coseo1 + aynl * sineo1
        esine = axnl * sineo1 - aynl * coseo1
        el2 = axnl * axnl + aynl * aynl
        pl = am * (1.0 - el2)
        rl = am * (1.0 - ecose)
        rdotl = np.sqrt(am) * esine / rl
        rvdotl = np.sqrt(pl) / rl
        betal = np.sqrt(1.0 - el2)
        temp = esine / (1.0 + betal)
        sinu = am / rl * (sineo1 - aynl - axnl * temp)
        cosu = am / rl * (coseo1 - axnl + aynl * temp)
        su = np.arctan2(sinu, cosu)
        sin2u = (cosu + cosu) * sinu
        cos2u = 1.0 - 2.0 * sinu * sinu
        temp = 1.0 / pl
        temp1 = 0.5 * J2 * temp
        temp2 = temp1 * temp
        # update for short-period periodics
        mrt = rl * (1.0 - 1.5 * temp2 * betal * self.con41) + 0.5 * temp1 * self.x1mth2 * cos2u
        su = su - 0.25 * temp2 * self.x7thm1 * sin2u
        xnode = nodem + 1.5 * temp2 * cosim * sin2u
        xinc = self.inclo + 1.5 * temp2 * cosim * sinim * cos2u
        mvt = rdotl - nm * temp1 * self.x1mth2 * sin2u / XKE
        rvdot = rvdotl + nm * temp1 * (self.x1mth2 * cos2u + 1.5 * self.con41) / XKE
        sinsu, cossu = np.sin(su), np.cos(su)
        snod, cnod = np.sin(xnode), np.cos(xnode)
        sini, cosi = np.sin(xinc), np.cos(xinc)
        xmx = -snod * cosi
        xmy = cnod * cosi
        ux = xmx * sinsu + cnod * cossu
        uy = xmy * sinsu + snod * cossu
        uz = sini * sinsu
        vx = xmx * cossu - cnod * sinsu
        vy = xmy * cossu - snod * sinsu
        vz = sini * cossu
        r = np.array([ux, uy, uz]) * (mrt * RE)
        v = (np.array([ux, uy, uz]) * mvt + np.array([vx, vy, vz]) * rvdot) * VKMPS
        r[:, mrt < 1.0] = np.nan        # decayed
        return r, v

    def tsince(self, jd):
        return (np.asarray(jd) - self.epoch_jd) * MIN_PER_DAY

    def age_days(self, now_unix):
        return float(unix_to_jd(now_unix)) - self.epoch_jd


# ---- observer geometry --------------------------------------------------------
WGS84_A = 6378.137
WGS84_F = 1.0 / 298.257223563


def observer_ecef(lat, lon, alt_km=0.0):
    phi, lam = lat * DEG, lon * DEG
    e2 = WGS84_F * (2.0 - WGS84_F)
    n = WGS84_A / math.sqrt(1.0 - e2 * math.sin(phi) ** 2)
    return np.array([(n + alt_km) * math.cos(phi) * math.cos(lam),
                     (n + alt_km) * math.cos(phi) * math.sin(lam),
                     (n * (1.0 - e2) + alt_km) * math.sin(phi)])


def teme_to_ecef(r, jd):
    g = gmst(jd)
    c, s = np.cos(g), np.sin(g)
    return np.array([c * r[0] + s * r[1], -s * r[0] + c * r[1], r[2]])


def look_angles(r_ecef, lat, lon, obs):
    """Azimuth / elevation (degrees) and range (km) of ECEF positions from the observer."""
    phi, lam = lat * DEG, lon * DEG
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    d = r_ecef - obs[:, None]
    e = -sl * d[0] + cl * d[1]
    n = -sp * cl * d[0] - sp * sl * d[1] + cp * d[2]
    u = cp * cl * d[0] + cp * sl * d[1] + sp * d[2]
    rng = np.sqrt(e * e + n * n + u * u)
    az = np.mod(np.degrees(np.arctan2(e, n)), 360.0)
    el = np.degrees(np.arcsin(np.clip(u / rng, -1.0, 1.0)))
    return az, el, rng


def subpoint(r_ecef):
    x, y, z = r_ecef
    lon = np.degrees(np.arctan2(y, x))
    p = np.sqrt(x * x + y * y)
    e2 = WGS84_F * (2.0 - WGS84_F)
    lat = np.arctan2(z, p * (1.0 - e2))
    for _ in range(4):
        n = WGS84_A / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1.0 - e2 * n / (n + alt)))
    n = WGS84_A / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - n
    return np.degrees(lat), lon, alt


def sun_eci(jd):
    """Unit vector towards the Sun (equatorial, of date; ~0.01 deg)."""
    n = np.asarray(jd) - 2451545.0
    L = np.radians(np.mod(280.460 + 0.9856474 * n, 360.0))
    g = np.radians(np.mod(357.528 + 0.9856003 * n, 360.0))
    lam = L + np.radians(1.915) * np.sin(g) + np.radians(0.020) * np.sin(2 * g)
    eps = np.radians(23.439 - 0.0000004 * n)
    return np.array([np.cos(lam), np.cos(eps) * np.sin(lam), np.sin(eps) * np.sin(lam)])


def sun_elevation(jd, lat, lon, obs):
    s = teme_to_ecef(sun_eci(jd) * 1.496e8, jd)
    if s.ndim == 1:
        s = s[:, None]
    return look_angles(s, lat, lon, obs)[1]


def sunlit(r_teme, jd):
    """True where the satellite is in sunlight (cylindrical Earth-shadow model)."""
    s = sun_eci(jd)
    if s.ndim == 1:
        s = s[:, None]
    dot = np.sum(r_teme * s, axis=0)
    perp = np.sqrt(np.maximum(0.0, np.sum(r_teme * r_teme, axis=0) - dot * dot))
    return (dot > 0) | (perp > RE)


# ---- pass prediction -----------------------------------------------------------
def find_passes(sat, lat, lon, obs, start_unix, hours=24.0, step_s=30.0, min_el=10.0):
    """Passes rising above the horizon in the window. Each pass: dict with aos/tca/los (unix),
    az at each, max elevation, and whether it is visible to the eye at its peak."""
    n = int(hours * 3600 / step_s) + 1
    tu = start_unix + np.arange(n) * step_s
    jd = unix_to_jd(tu)
    r, _ = sat.propagate(sat.tsince(jd))
    if np.all(np.isnan(r[0])):
        return []
    az, el, _ = look_angles(teme_to_ecef(r, jd), lat, lon, obs)
    el = np.nan_to_num(el, nan=-90.0)
    up = el > 0
    passes = []
    i = 0
    while i < n:
        if not up[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and up[j + 1]:
            j += 1
        k = i + int(np.argmax(el[i:j + 1]))
        if el[k] >= min_el - 1.0:
            # interpolate horizon crossings between samples
            if i > 0:
                f = el[i - 1] / (el[i - 1] - el[i])
                aos = tu[i - 1] + f * step_s
            else:
                aos = tu[0]
            if j < n - 1:
                f = el[j] / (el[j] - el[j + 1])
                los = tu[j] + f * step_s
            else:
                los = tu[-1]
            # refine the peak at 1-second resolution around the best sample
            tf = tu[k] + np.arange(-step_s, step_s + 0.5, 1.0)
            jdf = unix_to_jd(tf)
            rf, _ = sat.propagate(sat.tsince(jdf))
            elf = np.nan_to_num(look_angles(teme_to_ecef(rf, jdf), lat, lon, obs)[1], nan=-90.0)
            kf = int(np.argmax(elf))
            tca, mx = float(tf[kf]), float(elf[kf])
            if mx >= min_el:
                p = sat_position(sat, np.array([aos, tca, los]), lat, lon, obs)
                # visible wherever it is sunlit, at least 10 degrees up, while the sky is dark;
                # check the whole pass (satellites can leave or enter Earth's shadow part-way)
                tv = np.linspace(aos, los, max(8, int((los - aos) / 10.0)))
                pv = sat_position(sat, tv, lat, lon, obs)
                dark = sun_elevation(unix_to_jd(tv), lat, lon, obs) < -6.0
                vmask = pv["sunlit"] & dark & (pv["el"] >= 10.0)
                vis = bool(vmask.any())
                vis_from = float(tv[np.argmax(vmask)]) if vis else None
                vis_to = float(tv[len(vmask) - 1 - np.argmax(vmask[::-1])]) if vis else None
                passes.append({"sat": sat, "aos": float(aos), "tca": tca, "los": float(los),
                               "aos_az": float(p["az"][0]), "tca_az": float(p["az"][1]), "los_az": float(p["az"][2]),
                               "max_el": min(90.0, mx), "visible": vis, "vis_from": vis_from, "vis_to": vis_to,
                               "partial": i == 0 or j == n - 1})
        i = j + 1
    return passes


def sat_position(sat, unix_times, lat, lon, obs):
    tu = np.atleast_1d(np.asarray(unix_times, dtype=float))
    jd = unix_to_jd(tu)
    r, v = sat.propagate(sat.tsince(jd))
    ecef = teme_to_ecef(r, jd)
    az, el, rng = look_angles(ecef, lat, lon, obs)
    sla, slo, alt = subpoint(ecef)
    return {"az": az, "el": el, "range": rng, "lat": sla, "lon": slo, "alt": alt,
            "speed": np.sqrt(np.sum(v * v, axis=0)), "sunlit": sunlit(r, jd)}


def parse_tle_text(text, check=True):
    """Satellites from a 3-line (name + 2 lines) or 2-line TLE file. Bad entries are skipped."""
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    sats, skipped = [], 0
    i = 0
    while i < len(lines):
        if lines[i].startswith("1 ") and i + 1 < len(lines) and lines[i + 1].startswith("2 "):
            name, l1, l2 = "", lines[i], lines[i + 1]
            i += 2
        elif i + 2 < len(lines) and lines[i + 1].startswith("1 ") and lines[i + 2].startswith("2 "):
            name, l1, l2 = lines[i], lines[i + 1], lines[i + 2]
            i += 3
        else:
            i += 1
            continue
        try:
            sats.append(Satellite(name, l1, l2, check=check))
        except (TleError, ValueError):
            skipped += 1
    return sats, skipped


# --------------------------------------------------------------------------
# Orbit data from CelesTrak (cached; refreshed at most every 2 hours)
# --------------------------------------------------------------------------
class Catalog:
    def __init__(self, data_dir=DATA_DIR, fetch=None):
        self.data_dir = data_dir
        self.lock = threading.Lock()
        self.group = None
        self.sats = []
        self.skipped_deep = 0
        self.fetched_at = None          # unix time of the cached download
        self.status = ""
        self.busy = False
        self.fetch = fetch or self._http_fetch
        self.version = 0                # bumps whenever the satellite list changes

    def path(self, group):
        return os.path.join(self.data_dir, group + ".tle")

    def load(self, group):
        sats, deep, fetched = [], 0, None
        p = self.path(group)
        try:
            with open(p) as f:
                txt = f.read()
            fetched = os.path.getmtime(p)
            parsed, _ = parse_tle_text(txt)
            seen = set()
            for s in parsed:
                if s.norad in seen:
                    continue
                seen.add(s.norad)
                if s.deep:
                    deep += 1
                else:
                    sats.append(s)
        except OSError:
            pass
        with self.lock:
            self.group, self.sats, self.skipped_deep, self.fetched_at = group, sats[:MAX_SATS], deep, fetched
            self.version += 1
        return len(sats)

    def age(self, now):
        return None if self.fetched_at is None else now - self.fetched_at

    def can_update(self, now):
        a = self.age(now)
        return a is None or a >= MIN_FETCH_INTERVAL

    def _http_fetch(self, group):
        req = urllib.request.Request(CELESTRAK.format(group),
                                     headers={"User-Agent": "pocket-sattrack/{} (CardputerZero)".format(VERSION)})
        with urllib.request.urlopen(req, timeout=25) as r:
            return r.read().decode("ascii", "replace")

    def update(self, group, now, force=False):
        """Download in the background. Returns False if a refresh isn't allowed yet."""
        if self.busy:
            return False
        if not force and not self.can_update(now):
            return False
        self.busy = True
        self.status = "Downloading orbit data…"
        threading.Thread(target=self._update, args=(group,), daemon=True).start()
        return True

    def _update(self, group):
        try:
            txt = self.fetch(group)
            sats, _ = parse_tle_text(txt)
            if not sats:
                raise ValueError("no satellites in the reply")
            os.makedirs(self.data_dir, exist_ok=True)
            tmp = self.path(group) + ".tmp"
            with open(tmp, "w") as f:
                f.write(txt)
            os.replace(tmp, self.path(group))
            n = self.load(group)
            self.status = "Updated: {} satellites".format(n)
        except Exception as e:
            reason = getattr(e, "reason", None) or e
            self.status = "Update failed: {}".format(str(reason)[:60])
        finally:
            self.busy = False


# --------------------------------------------------------------------------
# Pass prediction (background thread)
# --------------------------------------------------------------------------
class Predictor:
    def __init__(self, catalog, clock):
        self.catalog = catalog
        self.clock = clock
        self.lock = threading.Lock()
        self.passes = []
        self.progress = None            # (done, total) while computing
        self.computed_for = None
        self.wake = threading.Event()
        self.stop_flag = False
        self.site = None                # (lat, lon, alt_m)
        self.min_el = 10
        self.t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.t.start()

    def request(self, site, min_el):
        with self.lock:
            self.site, self.min_el = site, min_el
        self.wake.set()

    def _key(self):
        return (self.site, self.min_el, self.catalog.version)

    def _run(self):
        last = 0
        while not self.stop_flag:
            self.wake.wait(5.0)
            self.wake.clear()
            if self.site is None:
                continue
            key = self._key()
            if key == self.computed_for and time.monotonic() - last < 900:
                continue
            self._compute(key)
            last = time.monotonic()

    def _compute(self, key):
        site, min_el, _ = key
        lat, lon, alt = site
        obs = observer_ecef(lat, lon, alt / 1000.0)
        with self.catalog.lock:
            sats = list(self.catalog.sats)
        now = self.clock()
        out = []
        self.progress = (0, len(sats))
        for i, s in enumerate(sats):
            if self.stop_flag or self._key() != key:
                self.progress = None
                return
            try:
                out += find_passes(s, lat, lon, obs, now - 900, hours=24.25, min_el=min_el)
            except Exception:
                pass
            self.progress = (i + 1, len(sats))
        out.sort(key=lambda p: p["aos"])
        out = merge_docked(out)
        with self.lock:
            self.passes = out
            self.computed_for = key
        self.progress = None


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def merge_docked(passes):
    """Docked modules and visiting spacecraft (e.g. ISS + POISK + NAUKA + PROGRESS) make the
    same pass; keep the main object (lowest catalogue number) and note how many are with it."""
    out = []
    for p in passes:
        twin = None
        for q in reversed(out[-12:]):
            if abs(q["aos"] - p["aos"]) > 30:
                continue
            if (abs(q["tca"] - p["tca"]) < 15 and abs(q["max_el"] - p["max_el"]) < 1.5
                    and _angle_diff(q["aos_az"], p["aos_az"]) < 5 and _angle_diff(q["los_az"], p["los_az"]) < 5):
                twin = q
                break
        if twin is None:
            p = dict(p, docked=[])
            out.append(p)
        elif p["sat"].norad < twin["sat"].norad:
            i = out.index(twin)
            out[i] = dict(p, docked=twin["docked"] + [twin["sat"].name])
        else:
            twin["docked"].append(p["sat"].name)
    return out


def sky_now(sats, lat, lon, alt, when):
    """Every satellite above the horizon right now."""
    obs = observer_ecef(lat, lon, alt / 1000.0)
    jd = unix_to_jd(np.array([when]))
    sun_el = float(sun_elevation(jd, lat, lon, obs)[0])
    up = []
    for s in sats:
        try:
            r, _ = s.propagate(s.tsince(jd))
            if np.isnan(r[0, 0]):
                continue
            az, el, rng = look_angles(teme_to_ecef(r, jd), lat, lon, obs)
            if el[0] > 0:
                up.append({"sat": s, "az": float(az[0]), "el": float(el[0]), "range": float(rng[0]),
                           "visible": bool(sunlit(r, jd)[0]) and sun_el < -6.0})
        except Exception:
            pass
    up.sort(key=lambda u: u["sat"].norad)
    merged = []
    for u in up:
        twin = next((m for m in merged if abs(m["el"] - u["el"]) < 0.5 and _angle_diff(m["az"], u["az"]) < 1.0), None)
        if twin:
            twin["docked"] += 1
        else:
            merged.append(dict(u, docked=0))
    merged.sort(key=lambda u: -u["el"])
    return merged, sun_el


# --------------------------------------------------------------------------
# GPS (the Cap's GNSS receiver on /dev/ttyS0, as in Trail Recorder)
# --------------------------------------------------------------------------
GPS_DEVICE = os.environ.get("LORASCAN_GPS_DEV", "/dev/ttyS0")


def _nmea_ok(line):
    if "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _nmea_latlon(v, hemi):
    if not v or "." not in v:
        return None
    try:
        dot = v.index(".")
        val = float(v[:dot - 2]) + float(v[dot - 2:]) / 60.0
    except ValueError:
        return None
    return -val if hemi in ("S", "W") else val


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * 6371008.8 * math.asin(min(1.0, math.sqrt(a)))


class GpsReader:
    def __init__(self, path=GPS_DEVICE):
        self.path = path
        self.lat = self.lon = None
        self.quality = 0
        self.valid = False
        self.sats = 0
        self.hdop = None
        self.last_fix = 0.0
        self.last_data = 0.0
        self.in_view = {}        # constellation talker -> (satellites in view, time)
        self.error = None
        self.stop_flag = False
        self.track = []          # (t, lat, lon) every ~10 s / 20 m while listening
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._run, daemon=True)

    @property
    def has_fix(self):
        return self.valid and self.quality > 0 and self.lat is not None and time.monotonic() - self.last_fix < 5

    def position(self):
        return (self.lat, self.lon) if self.has_fix else None

    def sats_in_view(self):
        now = time.monotonic()
        return sum(n for n, t in self.in_view.values() if now - t < 5)

    def status(self):
        view = self.sats_in_view()
        if self.has_fix:
            return "GPS fix · using {} of {} satellites".format(self.sats, max(view, self.sats)), GREEN
        if self.error:
            return "GPS: " + self.error, ORANGE
        if time.monotonic() - self.last_data < 3:
            if view:
                return "GPS searching… {} satellites in view".format(view), YELLOW
            return "GPS searching… no satellites yet (needs sky view)", YELLOW
        return "GPS: no data yet", DIM

    def start(self):
        self.t.start()

    def feed(self, line):
        line = line.strip()
        if not line.startswith("$") or not _nmea_ok(line):
            return
        self.last_data = time.monotonic()
        p = line.split("*")[0].split(",")
        kind = p[0][3:]
        try:
            if kind == "GSV" and len(p) > 3 and p[3]:
                self.in_view[p[0][1:3]] = (int(p[3]), time.monotonic())
            elif kind == "GGA" and len(p) > 9:
                self.quality = int(p[6] or 0)
                self.sats = int(p[7] or 0)
                self.hdop = float(p[8]) if p[8] else self.hdop
                if self.quality:
                    la, lo = _nmea_latlon(p[2], p[3]), _nmea_latlon(p[4], p[5])
                    if la is not None and lo is not None:
                        self.lat, self.lon, self.last_fix = la, lo, time.monotonic()
            elif kind == "RMC" and len(p) > 6:
                self.valid = p[2] == "A"
                if self.valid:
                    la, lo = _nmea_latlon(p[3], p[4]), _nmea_latlon(p[5], p[6])
                    if la is not None and lo is not None:
                        self.lat, self.lon, self.last_fix = la, lo, time.monotonic()
        except ValueError:
            return
        if self.has_fix:
            with self.lock:
                if not self.track or (time.time() - self.track[-1][0] > 10 and
                                      haversine(self.track[-1][1], self.track[-1][2], self.lat, self.lon) > 20):
                    self.track.append((time.time(), self.lat, self.lon))

    def _run(self):
        try:
            fd = os.open(self.path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self.error = "{} ({})".format(e.strerror.lower(), self.path) if e.errno != 13 else \
                "no permission (reboot once after installing)"
            return
        saved = None
        try:
            try:
                import termios
                attrs = termios.tcgetattr(fd)
                saved = list(attrs)
                cc = list(attrs[6])
                cc[termios.VMIN], cc[termios.VTIME] = 0, 0
                termios.tcsetattr(fd, termios.TCSANOW, [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL,
                                                        0, termios.B115200, termios.B115200, cc])
            except Exception:
                saved = None
            buf = b""
            while not self.stop_flag:
                r, _, _ = select.select([fd], [], [], 0.5)
                if not r:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except (BlockingIOError, InterruptedError):
                    continue
                if not chunk:
                    time.sleep(0.2)
                    continue
                buf = (buf + chunk)[-8192:]
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    self.feed(ln.decode("ascii", "replace"))
        finally:
            if saved:
                try:
                    import termios
                    termios.tcsetattr(fd, termios.TCSANOW, saved)
                except Exception:
                    pass
            os.close(fd)


class NoGps:
    error = "off"
    track = []
    lock = threading.Lock()
    has_fix = False

    def position(self):
        return None

    def status(self):
        return "GPS off", DIM

    def start(self):
        pass


# --------------------------------------------------------------------------
# GPS: position plus a time check (the system clock must be right for predictions)
# --------------------------------------------------------------------------
class SatGps(GpsReader):
    def __init__(self, path=GPS_DEVICE):
        super().__init__(path)
        self.alt = None
        self.clock_offset = None
        self.clock_t = 0.0

    def feed(self, line):
        super().feed(line)
        line = line.strip()
        if not line.startswith("$") or not _nmea_ok(line):
            return
        p = line.split("*")[0].split(",")
        kind = p[0][3:]
        try:
            if kind == "GGA" and len(p) > 9 and p[6] not in ("", "0") and p[9]:
                self.alt = float(p[9])
            elif kind == "RMC" and len(p) > 9 and p[2] == "A" and len(p[1]) >= 6 and len(p[9]) == 6:
                d, t = p[9], p[1]
                sec = float(t[4:])
                dt = datetime(2000 + int(d[4:6]), int(d[2:4]), int(d[0:2]), int(t[0:2]), int(t[2:4]),
                              int(sec), tzinfo=timezone.utc)
                self.clock_offset = dt.timestamp() + (sec - int(sec)) - time.time()
                self.clock_t = time.monotonic()
        except ValueError:
            pass

    def offset(self):
        if self.clock_offset is not None and time.monotonic() - self.clock_t < 600:
            return self.clock_offset
        return None


def ntp_synced():
    try:
        r = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                           capture_output=True, text=True, timeout=3)
        return r.stdout.strip() == "yes"
    except Exception:
        return None


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------
class UI:
    def __init__(self):
        self.page = P_LIST
        self.sel = 0
        self.top = 0
        self.pass_ref = None        # (norad, aos) of the pass shown on the Pass page
        self.setup_sel = 0
        self.edit = None            # text being typed for the location, or None
        self.toast = None
        self.toast_until = 0.0
        self.sky = ([], None)
        self.sky_t = 0.0


class Renderer:
    def __init__(self):
        self.f_title = load_font(13, bold=True)
        self.f_big = load_font(16, bold=True)
        self.f_med = load_font(12, bold=True)
        self.f_reg = load_font(11)
        self.f_sm = load_font(10)
        self.f_smb = load_font(10, bold=True)
        self.f_xs = load_font(9)

    # -- helpers --
    def tr(self, d, xr, y, s, font, fill):
        d.text((xr - d.textlength(s, font=font), y), s, font=font, fill=fill)

    def tc(self, d, xc, y, s, font, fill):
        d.text((xc - d.textlength(s, font=font) / 2, y), s, font=font, fill=fill)

    def fit(self, d, s, font, width):
        if d.textlength(s, font=font) <= width:
            return s
        while s and d.textlength(s + "…", font=font) > width:
            s = s[:-1]
        return s + "…"

    def hint_text(self, d, x, y, text, font, col):
        """Key hint with the arrow triangles drawn red, like the markings on the keyboard."""
        run, red = "", False
        for ch in text + "\0":
            is_red = ch in "▲▼◀▶"
            if (is_red != red or ch == "\0") and run:
                d.text((x, y), run, font=font, fill=KEY_RED if red else col)
                x += d.textlength(run, font=font)
                run = ""
            red = is_red
            if ch != "\0":
                run += ch

    def footer(self, d, text, right=None, rcol=DIM):
        self.hint_text(d, 6, H - 13, text, self.f_sm, DIM)
        if right:
            self.tr(d, 314, H - 13, right, self.f_sm, rcol)

    def panel(self, d, box):
        d.rounded_rectangle(box, radius=6, fill=PANEL)

    def header(self, d, ui, now):
        d.rectangle((0, 0, W, 18), fill=PANEL)
        d.text((6, 2), "Sat Tracker", font=self.f_title, fill=CYAN)
        for i in range(len(PAGES)):
            cx = 96 + i * 7
            d.ellipse((cx, 7, cx + 4, 11), fill=CYAN if i == ui.page else GRID)
        d.text((128, 4), PAGES[ui.page], font=self.f_smb, fill=TEXT)
        self.tr(d, 314, 3, time.strftime("%H:%M:%S", time.localtime(now)), self.f_reg, TEXT)

    def polar(self, d, cx, cy, r):
        for el in (0, 30, 60):
            rr = r * (90 - el) / 90.0
            d.ellipse((cx - rr, cy - rr, cx + rr, cy + rr), outline=GRID if el else (70, 78, 92))
        d.line((cx - r, cy, cx + r, cy), fill=GRID)
        d.line((cx, cy - r, cx, cy + r), fill=GRID)
        self.tc(d, cx, cy - r - 11, "N", self.f_smb, TEXT)
        self.tc(d, cx, cy + r + 1, "S", self.f_xs, DIM)
        d.text((cx + r + 3, cy - 6), "E", font=self.f_xs, fill=DIM)
        self.tr(d, cx - r - 3, cy - 6, "W", self.f_xs, DIM)

    def pxy(self, cx, cy, r, az, el):
        rr = r * (90.0 - max(0.0, min(90.0, el))) / 90.0
        a = math.radians(az)
        return cx + rr * math.sin(a), cy - rr * math.cos(a)

    def need_msg(self, d, lines):
        self.panel(d, (4, 22, 316, 152))
        y = 87 - 7 * len(lines)
        for i, (txt, col) in enumerate(lines):
            self.tc(d, 160, y, txt, self.f_med if i == 0 else self.f_sm, col)
            y += 16 if i == 0 else 13

    # -- pages --
    def page_list(self, d, app, now):
        ui = app.ui
        if app.missing():
            self.need_msg(d, app.missing())
            self.footer(d, "◀▶ page   ESC quit")
            return
        passes = app.visible_passes(now)
        prog = app.pred.progress
        if not passes:
            if prog or app.pred.computed_for is None:
                done = "{} of {} satellites".format(*prog) if prog else "starting"
                self.need_msg(d, [("Calculating passes…", YELLOW), (done, DIM)])
            else:
                why = "No naked-eye passes in the next 24 h" if app.conf["visible_only"] else \
                    "No passes above {}° in the next 24 h".format(app.conf["min_el"])
                self.need_msg(d, [(why, DIM), ("Try another satellite group on the Setup page", DIM)])
            self.footer(d, "V naked-eye only   ◀▶ page")
            return
        d.text((8, 21), "WHEN", font=self.f_xs, fill=DIM)
        d.text((76, 21), "SATELLITE", font=self.f_xs, fill=DIM)
        self.tr(d, 246, 21, "HIGH", self.f_xs, DIM)
        d.text((256, 21), "PATH", font=self.f_xs, fill=DIM)
        ui.sel = max(0, min(ui.sel, len(passes) - 1))
        rows = 9
        if ui.sel < ui.top:
            ui.top = ui.sel
        elif ui.sel >= ui.top + rows:
            ui.top = ui.sel - rows + 1
        ui.top = max(0, min(ui.top, max(0, len(passes) - rows)))
        y = 33
        for k in range(ui.top, min(len(passes), ui.top + rows)):
            p = passes[k]
            if k == ui.sel:
                d.rectangle((4, y - 1, 316, y + 12), fill=SEL)
            live = p["aos"] <= now <= p["los"]
            when = "NOW" if live else time.strftime("%a %H:%M", time.localtime(p["aos"]))
            d.text((8, y), when, font=self.f_smb if live else self.f_sm, fill=GREEN if live else TEXT)
            if p["visible"]:
                d.text((64, y - 1), STAR, font=self.f_sm, fill=YELLOW)
            d.text((76, y), self.fit(d, p["sat"].name, self.f_sm, 140), font=self.f_sm, fill=TEXT)
            mx = p["max_el"]
            self.tr(d, 246, y, "{:.0f}°".format(mx), self.f_smb,
                    GREEN if mx >= 45 else (TEXT if mx >= 20 else DIM))
            d.text((256, y), "{}→{}".format(compass(p["aos_az"]), compass(p["los_az"])), font=self.f_sm, fill=DIM)
            y += 13
        p = passes[ui.sel]
        right = "up now" if p["aos"] <= now <= p["los"] else "in " + fmt_in(p["aos"] - now)
        if prog:
            right = "updating {}/{}".format(*prog)
        self.footer(d, "▲▼ Enter details  V {}".format("all" if app.conf["visible_only"] else STAR + " only"),
                    right, GREEN)

    def page_pass(self, d, app, now):
        if app.missing():
            self.need_msg(d, app.missing())
            self.footer(d, "◀▶ page   ESC quit")
            return
        p = app.current_pass(now)
        if not p:
            self.need_msg(d, [("No pass selected", DIM), ("Pick one on the Passes page", DIM)])
            self.footer(d, "◀▶ page")
            return
        sat = p["sat"]
        lat, lon, alt = app.site()
        obs = observer_ecef(lat, lon, alt / 1000.0)
        cx, cy, r = 66, 90, 54
        self.polar(d, cx, cy, r)
        tt = np.linspace(p["aos"], p["los"], 60)
        track = sat_position(sat, tt, lat, lon, obs)
        pts = [self.pxy(cx, cy, r, a, e) for a, e in zip(track["az"], track["el"])]
        col = YELLOW if p["visible"] else CYAN
        if len(pts) > 1:
            d.line(pts, fill=col, width=2)
        ax, ay = pts[0]
        d.ellipse((ax - 3, ay - 3, ax + 3, ay + 3), fill=GREEN)
        bx, by = pts[-1]
        d.ellipse((bx - 3, by - 3, bx + 3, by + 3), outline=RED, width=1)
        live = p["aos"] <= now <= p["los"]
        q = sat_position(sat, [now], lat, lon, obs) if live else None
        if live:
            sx, sy = self.pxy(cx, cy, r, q["az"][0], q["el"][0])
            d.ellipse((sx - 5, sy - 5, sx + 5, sy + 5), fill=TEXT, outline=BG)
        x0 = 130
        name = sat.name
        if p.get("docked"):
            name += " +{}".format(len(p["docked"]))
        d.text((x0, 21), self.fit(d, name, self.f_med, 186), font=self.f_med, fill=CYAN)
        if live:
            d.text((x0, 37), "UP NOW", font=self.f_big, fill=GREEN)
            self.tr(d, 314, 41, "sets in " + fmt_in(p["los"] - now), self.f_sm, TEXT)
        elif now < p["aos"]:
            d.text((x0, 37), "in " + fmt_in(p["aos"] - now), font=self.f_big, fill=YELLOW)
        else:
            d.text((x0, 37), "passed", font=self.f_big, fill=DIM)
        y = 60

        def row(label, t, az, el=None, colr=TEXT):
            nonlocal y
            d.text((x0, y), label, font=self.f_sm, fill=DIM)
            d.text((x0 + 34, y), time.strftime("%H:%M:%S", time.localtime(t)), font=self.f_sm, fill=colr)
            where = "{} {:.0f}°".format(compass(az), az) if el is None else \
                "{}° up {}".format(int(round(el)), compass(az))
            self.tr(d, 314, y, where, self.f_smb, colr)
            y += 13
        row("Rise", p["aos"], p["aos_az"], colr=GREEN)
        row("Peak", p["tca"], p["tca_az"], p["max_el"], colr=YELLOW if p["visible"] else TEXT)
        row("Set", p["los"], p["los_az"])
        y += 2
        if live:
            look = "Look {} {:.0f}°, {:.0f}° up".format(compass(q["az"][0]), q["az"][0], q["el"][0])
            d.text((x0, y), look, font=self.f_smb, fill=TEXT)
            y += 13
            dist = q["range"][0] * (0.621371 if app.conf["units"] == "mi" else 1.0)
            d.text((x0, y), "{:,.0f} {} away · {}".format(
                dist, app.conf["units"], "lit" if q["sunlit"][0] else "in shadow"), font=self.f_sm, fill=DIM)
            y += 13
        else:
            dur = (p["los"] - p["aos"]) / 60.0
            d.text((x0, y), "{:.0f} min overhead · {}".format(dur, time.strftime("%a %b %d", time.localtime(p["aos"]))),
                   font=self.f_sm, fill=DIM)
            y += 13
        if p["visible"]:
            vf, vt = p.get("vis_from"), p.get("vis_to")
            if vf and vt and (vf - p["aos"] > 45 or p["los"] - vt > 45):
                msg = STAR + " Visible {}–{}".format(time.strftime("%H:%M", time.localtime(vf)),
                                                         time.strftime("%H:%M", time.localtime(vt)))
            else:
                msg = STAR + " Visible: a bright moving star"
            d.text((x0, y), msg, font=self.f_sm, fill=YELLOW)
        else:
            q2 = sat_position(sat, [p["tca"]], lat, lon, obs)
            why = "Not visible: daylight" if q2["sunlit"][0] else "Not visible: in Earth's shadow"
            d.text((x0, y), why, font=self.f_sm, fill=DIM)
        self.footer(d, "▲▼ prev/next pass  ◀▶ page")

    def page_sky(self, d, app, now):
        if app.missing():
            self.need_msg(d, app.missing())
            self.footer(d, "◀▶ page   ESC quit")
            return
        up, sun_el = app.ui.sky
        cx, cy, r = 68, 90, 54
        self.polar(d, cx, cy, r)
        for k in range(len(up) - 1, -1, -1):
            u = up[k]
            x, y = self.pxy(cx, cy, r, u["az"], u["el"])
            col = YELLOW if u["visible"] else CYAN
            d.ellipse((x - 3, y - 3, x + 3, y + 3), fill=col)
            if k < 8:
                d.text((x + 4, y - 5), str(k + 1), font=self.f_xs, fill=TEXT)
        x0 = 140
        d.text((x0, 21), "{} overhead now".format(len(up)), font=self.f_med, fill=TEXT)
        if sun_el is not None:
            sky = "daylight" if sun_el > -0.8 else ("twilight" if sun_el > -6 else "dark sky")
            self.tr(d, 314, 23, sky, self.f_sm, DIM)
        y = 38
        for k, u in enumerate(up[:8]):
            d.text((x0, y), str(k + 1), font=self.f_xs, fill=DIM)
            d.text((x0 + 10, y), self.fit(d, u["sat"].name, self.f_sm, 104), font=self.f_sm,
                   fill=YELLOW if u["visible"] else TEXT)
            self.tr(d, 314, y, "{:.0f}° {}".format(u["el"], compass(u["az"])), self.f_smb, TEXT)
            y += 13
        if not up:
            d.text((x0, 44), "Nothing from this group", font=self.f_sm, fill=DIM)
            d.text((x0, 57), "is above the horizon.", font=self.f_sm, fill=DIM)
        self.footer(d, "yellow = sunlit in a dark sky   ◀▶ page")

    def page_setup(self, d, app, now):
        ui = app.ui
        cat = app.cat
        rows = []
        gname = dict(GROUPS).get(app.conf["group"], app.conf["group"])
        rows.append(("Satellites", gname, CYAN))
        age = cat.age(now)
        if cat.busy:
            data = ("downloading…", YELLOW)
        elif age is None:
            data = ("none — Enter to download", ORANGE)
        else:
            data = ("{} sats · updated {} ago".format(len(cat.sats), fmt_in(age)),
                    TEXT if age < 72 * 3600 else ORANGE)
        rows.append(("Orbit data", data[0], data[1]))
        if ui.edit is not None:
            rows.append(("Location", ui.edit + "_", YELLOW))
        elif app.conf["lat"] is None:
            rows.append(("Location", "not set — GPS fix or Enter", ORANGE))
        else:
            src = {"gps": "GPS", "manual": "typed"}.get(app.conf["loc_source"], "")
            rows.append(("Location", "{}  ({})".format(fmt_latlon(app.conf["lat"], app.conf["lon"]), src), TEXT))
        rows.append(("Lowest pass", "{}° above horizon".format(app.conf["min_el"]), TEXT))
        rows.append(("Distance", "miles" if app.conf["units"] == "mi" else "kilometres", TEXT))
        off = app.gps.offset() if app.gps else None
        if off is not None and abs(off) > 2:
            clk = ("corrected by GPS ({:+.0f} s)".format(off), YELLOW)
        elif off is not None:
            clk = ("checked by GPS", GREEN)
        elif app.ntp is True:
            clk = ("set from the internet", GREEN)
        else:
            clk = ("not checked — Wi-Fi or GPS fix", ORANGE)
        rows.append(("Clock", clk[0], clk[1]))
        ui.setup_sel = max(0, min(ui.setup_sel, 4))
        y = 23
        for i, (k, v, col) in enumerate(rows):
            if i == ui.setup_sel:
                d.rectangle((4, y - 2, 316, y + 13), fill=SEL)
            d.text((10, y), k, font=self.f_sm, fill=DIM)
            d.text((86, y - 1), self.fit(d, v, self.f_reg, 226), font=self.f_reg, fill=col)
            y += 17
        st, col = app.gps.status() if app.gps else ("GPS off", DIM)
        d.text((10, y), self.fit(d, st, self.f_xs, 300), font=self.f_xs, fill=col)
        if cat.status:
            d.text((10, y + 11), self.fit(d, cat.status, self.f_xs, 300), font=self.f_xs,
                   fill=RED if "failed" in cat.status else DIM)
        if ui.edit is not None:
            self.footer(d, "e.g. 40.71 N 74.01 W   Enter save   ESC cancel")
        else:
            self.footer(d, "▲▼ choose  Enter change  U update  ◀▶ page")

    def render(self, app, now):
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        self.header(d, app.ui, now)
        page = app.ui.page
        if page == P_LIST:
            self.page_list(d, app, now)
        elif page == P_PASS:
            self.page_pass(d, app, now)
        elif page == P_SKY:
            self.page_sky(d, app, now)
        else:
            self.page_setup(d, app, now)
        ui = app.ui
        if ui.toast and time.monotonic() < ui.toast_until:
            msg, col = ui.toast
            w = d.textlength(msg, font=self.f_reg) + 20
            d.rounded_rectangle((160 - w / 2, 126, 160 + w / 2, 148), radius=6, fill=(14, 16, 22), outline=col)
            self.tc(d, 160, 130, msg, self.f_reg, col)
        return img


# --------------------------------------------------------------------------
# Framebuffer + keyboard (as in SysMon / Trail Recorder)
# --------------------------------------------------------------------------
class Framebuffer:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR)
        vbuf = bytearray(160)
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vbuf)
        self.xres, self.yres = struct.unpack_from("2I", vbuf, 0)
        self.bpp = struct.unpack_from("I", vbuf, 24)[0]
        fix_fmt = "16sL" + "I" * 6 + "HHHI"
        fbuf = bytearray(struct.calcsize(fix_fmt))
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, fbuf)
        fields = struct.unpack(fix_fmt, fbuf)
        self.smem_len, self.line_length = fields[2], fields[8]
        if self.bpp not in (16, 32):
            os.close(self.fd)
            raise RuntimeError("unsupported framebuffer depth: {} bpp".format(self.bpp))
        self.mm = mmap.mmap(self.fd, self.smem_len, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

    def _convert(self, img):
        if self.bpp == 32:
            return img.convert("RGBA").tobytes("raw", "BGRA")
        if HAS_NUMPY:
            a = np.asarray(img, dtype=np.uint16)
            v = ((a[..., 0] >> 3) << 11) | ((a[..., 1] >> 2) << 5) | (a[..., 2] >> 3)
            return v.astype("<u2").tobytes()
        out = bytearray()
        for r, g, b in img.getdata():
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            out += bytes((v & 0xFF, v >> 8))
        return bytes(out)

    def show(self, img):
        if (self.xres, self.yres) != img.size:
            img = img.rotate(90, expand=True) if (self.yres, self.xres) == img.size else img.resize((self.xres, self.yres))
        data = self._convert(img)
        row = self.xres * self.bpp // 8
        if row == self.line_length:
            self.mm.seek(0)
            self.mm.write(data)
        else:
            for y in range(self.yres):
                self.mm.seek(y * self.line_length)
                self.mm.write(data[y * row:(y + 1) * row])

    def clear(self):
        self.mm.seek(0)
        self.mm.write(b"\x00" * min(self.smem_len, self.line_length * self.yres))

    def close(self):
        try:
            self.mm.close()
        finally:
            os.close(self.fd)


EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)


class Keys:
    def __init__(self):
        self.fds = []
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                self.fds.append(os.open(path, os.O_RDONLY | os.O_NONBLOCK))
            except OSError:
                pass

    def poll(self, timeout):
        if not self.fds:
            time.sleep(timeout)
            return []
        try:
            r, _, _ = select.select(self.fds, [], [], timeout)
        except (OSError, ValueError):
            return []
        out = []
        for fd in r:
            try:
                data = os.read(fd, EVENT_SIZE * 32)
            except (BlockingIOError, OSError):
                continue
            for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, _, et, code, val = struct.unpack_from(EVENT_FMT, data, off)
                if et == EV_KEY and val in (1, 2):
                    out.append((code, val))
        return out

    def close(self):
        for fd in self.fds:
            try:
                os.close(fd)
            except OSError:
                pass


# --------------------------------------------------------------------------
# App
# --------------------------------------------------------------------------
class App:
    def __init__(self, conf, catalog, gps, clock_offset=None):
        self.conf = conf
        self.cat = catalog
        self.gps = gps
        self.ui = UI()
        self.fixed_offset = clock_offset
        self.ntp = None
        self.pred = Predictor(catalog, self.now)

    # -- time and place --
    def now(self):
        if self.fixed_offset is not None:
            return time.time() + self.fixed_offset
        off = self.gps.offset() if self.gps else None
        return time.time() + (off if off is not None and abs(off) > 2 else 0.0)

    def site(self):
        if self.conf["lat"] is None:
            return None
        return (round(self.conf["lat"], 3), round(self.conf["lon"], 3), round(self.conf.get("alt") or 0.0))

    def missing(self):
        if self.cat.busy and not self.cat.sats:
            return [("Downloading orbit data…", YELLOW), ("from CelesTrak over Wi-Fi", DIM)]
        if not self.cat.sats:
            msg = [("No orbit data yet", ORANGE), ("Connect to Wi-Fi, then press U", TEXT)]
            if self.cat.status:
                msg.append((self.cat.status, RED if "failed" in self.cat.status else DIM))
            return msg
        if self.site() is None:
            return [("Where are you?", ORANGE), ("Get a GPS fix outdoors, or press L", TEXT),
                    ("and type it, e.g. 40.71 N 74.01 W", DIM)]
        return None

    def check_gps(self):
        pos = self.gps.position() if self.gps else None
        if not pos:
            return
        lat, lon = pos
        old = self.site()
        alt = self.gps.alt or 0.0
        if old is None or self.conf["loc_source"] != "gps" or haversine(old[0], old[1], lat, lon) > 1000:
            self.conf.update(lat=round(lat, 3), lon=round(lon, 3), alt=round(alt), loc_source="gps")
            save_config(self.conf)
            self.refresh()

    def refresh(self):
        site = self.site()
        if site and self.cat.sats:
            self.pred.request(site, self.conf["min_el"])

    def visible_passes(self, now):
        with self.pred.lock:
            ps = [p for p in self.pred.passes if p["los"] > now]
        if self.conf["visible_only"]:
            ps = [p for p in ps if p["visible"]]
        return ps

    def current_pass(self, now):
        ps = self.visible_passes(now)
        if not ps:
            return None
        if self.ui.pass_ref:
            for p in ps:
                if (p["sat"].norad, round(p["aos"])) == self.ui.pass_ref:
                    return p
        return ps[min(self.ui.sel, len(ps) - 1)]

    def toast(self, msg, col=CYAN, secs=2.5):
        self.ui.toast = (msg, col)
        self.ui.toast_until = time.monotonic() + secs

    def set_group(self, group):
        self.conf["group"] = group
        save_config(self.conf)
        n = self.cat.load(group)
        self.ui.sel = self.ui.top = 0
        self.ui.pass_ref = None
        if n == 0 or (self.cat.age(self.now()) or 0) > AUTO_FETCH_AGE:
            self.update_data(quiet=n > 0)
        self.refresh()

    def update_data(self, quiet=False):
        now = self.now()
        if self.cat.busy:
            return
        if not self.cat.update(self.conf["group"], now):
            if not quiet:
                wait = MIN_FETCH_INTERVAL - (self.cat.age(now) or 0)
                self.toast("Up to date — next update allowed in {}".format(fmt_in(wait)), DIM, 3.0)

    def update_sky(self, now):
        site = self.site()
        if site is None:
            self.ui.sky = ([], None)
            return
        with self.cat.lock:
            sats = list(self.cat.sats)
        self.ui.sky = sky_now(sats, site[0], site[1], site[2], now)
        self.ui.sky_t = time.monotonic()

    # -- keys --
    def handle(self, code, repeat):
        """Returns False to quit."""
        ui = self.ui
        if ui.edit is not None:
            return self.handle_edit(code, repeat)
        if repeat and code not in (KEY_UP, KEY_DOWN, KEY_SEMICOLON, KEY_DOT, KEY_F, KEY_X):
            return True
        up = code in (KEY_UP, KEY_SEMICOLON, KEY_F)
        down = code in (KEY_DOWN, KEY_DOT, KEY_X)
        if code in (KEY_ESC, KEY_Q):
            return False
        if code in (KEY_RIGHT, KEY_TAB, KEY_SLASH, KEY_C):
            ui.page = (ui.page + 1) % len(PAGES)
        elif code in (KEY_LEFT, KEY_COMMA, KEY_Z):
            ui.page = (ui.page - 1) % len(PAGES)
        elif code in NUM_KEYS:
            ui.page = NUM_KEYS[code]
        elif code == KEY_U:
            self.update_data()
        elif code == KEY_L:
            ui.page, ui.setup_sel, ui.edit = P_SETUP, 2, ""
        elif code == KEY_V and ui.page in (P_LIST, P_PASS):
            self.conf["visible_only"] = not self.conf["visible_only"]
            save_config(self.conf)
            ui.sel = ui.top = 0
            ui.pass_ref = None
            self.toast("Showing naked-eye passes only" if self.conf["visible_only"] else "Showing all passes")
        elif ui.page == P_LIST:
            ps = self.visible_passes(self.now())
            if up:
                ui.sel = max(0, ui.sel - 1)
            elif down:
                ui.sel = min(max(0, len(ps) - 1), ui.sel + 1)
            elif code in (KEY_ENTER, KEY_SPACE) and ps:
                p = ps[min(ui.sel, len(ps) - 1)]
                ui.pass_ref = (p["sat"].norad, round(p["aos"]))
                ui.page = P_PASS
        elif ui.page == P_PASS:
            ps = self.visible_passes(self.now())
            if ps and (up or down):
                cur = self.current_pass(self.now())
                i = ps.index(cur) if cur in ps else 0
                i = max(0, min(len(ps) - 1, i + (-1 if up else 1)))
                ui.pass_ref = (ps[i]["sat"].norad, round(ps[i]["aos"]))
                ui.sel = i
        elif ui.page == P_SETUP:
            if up:
                ui.setup_sel = max(0, ui.setup_sel - 1)
            elif down:
                ui.setup_sel = min(4, ui.setup_sel + 1)
            elif code in (KEY_ENTER, KEY_SPACE):
                self.setup_change(ui.setup_sel)
        return True

    def setup_change(self, row):
        if row == 0:
            keys = [g for g, _ in GROUPS]
            i = keys.index(self.conf["group"]) if self.conf["group"] in keys else -1
            self.set_group(keys[(i + 1) % len(keys)])
            self.toast(dict(GROUPS)[self.conf["group"]])
        elif row == 1:
            self.update_data()
        elif row == 2:
            self.ui.edit = ""
        elif row == 3:
            i = MIN_ELS.index(self.conf["min_el"]) if self.conf["min_el"] in MIN_ELS else -1
            self.conf["min_el"] = MIN_ELS[(i + 1) % len(MIN_ELS)]
            save_config(self.conf)
            self.refresh()
        elif row == 4:
            self.conf["units"] = "km" if self.conf["units"] == "mi" else "mi"
            save_config(self.conf)

    def handle_edit(self, code, repeat):
        ui = self.ui
        if code == KEY_ESC:
            ui.edit = None
        elif code == KEY_BACKSPACE:
            ui.edit = ui.edit[:-1]
        elif code == KEY_ENTER and not repeat:
            ll = parse_latlon(ui.edit)
            if ll is None:
                self.toast("Didn't understand — e.g. 40.71 N 74.01 W", ORANGE, 3.0)
                return True
            self.conf.update(lat=round(ll[0], 3), lon=round(ll[1], 3), alt=0.0, loc_source="manual")
            save_config(self.conf)
            ui.edit = None
            self.refresh()
            self.toast("Location saved", GREEN)
        elif code in TEXT_KEYS and not repeat and len(ui.edit) < 28:
            ui.edit += TEXT_KEYS[code]
        return True


# --------------------------------------------------------------------------
# Previews with sample data (no network or hardware needed)
# --------------------------------------------------------------------------
def _tle_line(body):
    s = sum(int(c) if c.isdigit() else (1 if c == "-" else 0) for c in body[:68])
    return body[:68] + str(s % 10)


def sample_tles(epoch_unix):
    """Plausible current-epoch elements for well-known satellites (preview/screenshot data only)."""
    dt = datetime.fromtimestamp(epoch_unix, timezone.utc)
    doy = dt.timetuple().tm_yday + (dt.hour * 3600 + dt.minute * 60 + dt.second) / 86400.0
    sats = [("ISS (ZARYA)", 25544, 51.6390, 210.0, 15.50103472, 40.0),
            ("CSS (TIANHE)", 48274, 41.4660, 120.0, 15.61028462, 200.0),
            ("HST", 20580, 28.4690, 300.0, 15.27931542, 80.0),
            ("NOAA 15", 25338, 98.5600, 30.0, 14.26700000, 300.0),
            ("METEOR-M2 3", 57166, 98.7800, 160.0, 14.23900000, 10.0),
            ("SO-50", 27607, 64.5550, 250.0, 14.80000000, 120.0),
            ("STARLINK-1007", 44713, 53.0540, 75.0, 15.06400000, 160.0),
            ("SL-16 R/B", 22566, 71.0000, 340.0, 14.15000000, 250.0)]
    out = []
    for name, num, inc, raan, mm, ma in sats:
        l1 = _tle_line("1 {:05d}U 98067A   {:02d}{:012.8f}  .00010000  00000-0  18000-3 0  9990".format(
            num, dt.year % 100, doy))
        l2 = _tle_line("2 {:05d} {:8.4f} {:8.4f} 0004000 {:8.4f} {:8.4f} {:11.8f}100000".format(
            num, inc, raan, 90.0, ma, mm))
        out += [name, l1, l2]
    return "\n".join(out) + "\n"


def preview(path, page, mock):
    tmp = "/tmp/sattrack-preview"
    os.makedirs(tmp, exist_ok=True)
    now = time.time()
    conf = {"group": "visual", "min_el": 10, "units": "mi", "visible_only": False,
            "lat": 41.88, "lon": -87.63, "alt": 180.0, "loc_source": "gps"}
    cat = Catalog(tmp, fetch=lambda g: sample_tles(now - 6 * 3600))
    with open(cat.path("visual"), "w") as f:
        f.write(sample_tles(now - 6 * 3600))
    os.utime(cat.path("visual"), (now - 5 * 3600, now - 5 * 3600))
    cat.load("visual")
    app = App(conf, cat, None, clock_offset=0.0)
    app.ntp = True
    site = app.site()
    key = (site, conf["min_el"], cat.version)
    app.pred.site, app.pred.min_el = site, conf["min_el"]
    app.pred._compute(key)
    ps = app.visible_passes(now)
    if page == P_PASS and ps:
        best = max(ps[:12], key=lambda p: p["max_el"] + (30 if p["visible"] else 0))
        app.ui.pass_ref = (best["sat"].norad, round(best["aos"]))
    if page == P_SKY:
        # find a moment with several satellites up for the screenshot
        best_t, best_n = now, -1
        for k in range(0, 24 * 60, 4):
            t = now + k * 60
            n = len(sky_now(cat.sats, site[0], site[1], site[2], t)[0])
            if n > best_n:
                best_t, best_n = t, n
        app.update_sky(best_t)
        now = best_t
    app.ui.page = page
    app.ui.sel = 1
    Renderer().render(app, now).save(path)
    print("wrote", path, "({} passes)".format(len(ps)))
    return 0


def main():
    args = sys.argv[1:]
    if args and args[0] == "--preview":
        return preview(args[1] if len(args) > 1 else "preview.png",
                       int(args[2]) if len(args) > 2 and args[2].isdigit() else 0, "--mock" in args)
    if args and args[0] in ("--version", "-V"):
        print("Sat Tracker", VERSION)
        return 0

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGHUP, _stop)
    try:
        fb = Framebuffer(FB_DEVICE)
    except Exception as e:
        print("Sat Tracker: cannot open framebuffer {}: {}".format(FB_DEVICE, e), file=sys.stderr)
        return 3

    conf = load_config()
    prev_5v = cap_power_on()
    gps = SatGps()
    gps.start()
    cat = Catalog()
    app = App(conf, cat, gps)
    app.ntp = ntp_synced()
    ntp_t = time.monotonic()
    n = cat.load(conf["group"])
    age = cat.age(app.now())
    if n == 0 or (age is not None and age > AUTO_FETCH_AGE):
        app.update_data(quiet=True)
    app.pred.start()
    app.refresh()
    keys = Keys()
    renderer = Renderer()
    last_version = cat.version
    try:
        while _running:
            now = app.now()
            app.check_gps()
            if cat.version != last_version:
                last_version = cat.version
                app.refresh()
            if app.ui.page == P_SKY and time.monotonic() - app.ui.sky_t > 2.0:
                app.update_sky(now)
            if time.monotonic() - ntp_t > 60:
                app.ntp, ntp_t = ntp_synced(), time.monotonic()
            fb.show(renderer.render(app, now))
            for code, val in keys.poll(0.25):
                if not app.handle(code, val == 2):
                    return 0
    finally:
        app.pred.stop_flag = True
        app.pred.wake.set()
        gps.stop_flag = True
        save_config(conf)
        if prev_5v == 0:
            try:
                with open(EXT5V, "w") as f:
                    f.write("0\n")
            except OSError:
                pass
        keys.close()
        try:
            fb.clear()
        except Exception:
            pass
        fb.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
