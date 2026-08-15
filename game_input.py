#!/usr/bin/env python3
"""
game_input.py — единый игровой мост (RetroArch + Mario).
Работает ТОЛЬКО на время игры (m5hub остановлен), чтобы не было конфликта на I2C-шине.
- Джойстик V2 (I2C/PaHub канал 0, адрес 0x63) → стрелки (D-pad)
- GPIO 96 → A (прыжок, клавиша "0");  GPIO 131 → B (бег, Backspace)
- CardKB (I2C/PaHub канал 2, 0x5F) → Enter=Start, Space=Select, Esc=выход
Эмуляция через XTest (X11). Без зависимостей от m5hub.
"""
import os, fcntl, time, ctypes, collections, statistics
import gpiod
from Xlib import display, X
from Xlib.ext import xtest

I2C_BUS = 0
I2C_RDWR = 0x0707
I2C_M_RD = 1


class m(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('len', ctypes.c_uint16), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class d(ctypes.Structure):
    _fields_ = [('msgs', ctypes.POINTER(m)), ('nmsgs', ctypes.c_uint32)]


_g = []


def i2c_wr(fd, ad, da):
    b = (ctypes.c_uint8 * len(da))(*da)
    mg = m(ad, 0, len(da), b)
    wd = d(ctypes.pointer(mg), 1)
    _g.extend([b, mg, wd])
    fcntl.ioctl(fd, I2C_RDWR, wd)
    _g.clear()
    time.sleep(0.002)


def i2c_rd(fd, ad, re, n):
    wb = (ctypes.c_uint8 * 1)(re)
    rb = (ctypes.c_uint8 * n)()
    m0 = m(ad, 0, 1, wb)
    m1 = m(ad, I2C_M_RD, n, rb)
    ms = (m * 2)(m0, m1)
    wd = d(ms, 2)
    _g.extend([wb, rb, m0, m1, ms, wd])
    fcntl.ioctl(fd, I2C_RDWR, wd)
    _g.clear()
    return bytes(rb)


def i2c_rr(fd, ad, n):
    rb = (ctypes.c_uint8 * n)()
    mg = m(ad, I2C_M_RD, n, rb)
    wd = d(ctypes.pointer(mg), 1)
    _g.extend([rb, mg, wd])
    fcntl.ioctl(fd, I2C_RDWR, wd)
    _g.clear()
    return bytes(rb)


class GameInput:
    def __init__(self):
        self.fd = os.open('/dev/i2c-%d' % I2C_BUS, os.O_RDWR)
        _g.append(self.fd)
        self.disp = display.Display(':0')
        self._rst()
        self.cx = self.cy = 32768
        self._cal()
        self.kc = {}
        self.state = {}
        self._kl = 0
        self._jhist = collections.deque(maxlen=4)
        # GPIO кнопки (pull-up, 0 = нажата)
        self.chip = gpiod.Chip('gpiochip0')
        self.lines = self.chip.get_lines([96, 131])
        self.lines.request(consumer='game-input', type=gpiod.LINE_REQ_DIR_IN,
                           flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        self._gpio_state = {96: 1, 131: 1}

    # ── PaHub ──
    def _rst(self):
        try:
            i2c_wr(self.fd, 0x70, [0x00])
            time.sleep(0.01)
        except Exception:
            pass

    def sel(self, c):
        try:
            i2c_wr(self.fd, 0x70, [0x00])
            time.sleep(0.001)
        except Exception:
            pass
        i2c_wr(self.fd, 0x70, [1 << c])
        time.sleep(0.005)

    def rd(self, c, a, re, n):
        self.sel(c)
        return i2c_rd(self.fd, a, re, n)

    def rr(self, c, a, n):
        self.sel(c)
        return i2c_rr(self.fd, a, n)

    def _cal(self):
        sx = sy = n = 0
        for _ in range(50):
            try:
                dd = self.rd(0, 0x63, 0x00, 4)
                sx += dd[0] | (dd[1] << 8)
                sy += dd[2] | (dd[3] << 8)
                n += 1
            except Exception:
                pass
            time.sleep(0.01)
        if n:
            self.cx, self.cy = sx // n, sy // n
            print('[game] center X=%d Y=%d' % (self.cx, self.cy))

    # ── XTest ──
    def _keycode(self, keysym):
        if keysym not in self.kc:
            kc = self.disp.keysym_to_keycode(keysym)
            if not kc:
                kc = self.disp.keysym_to_keycode(0x20)
            self.kc[keysym] = kc
        return self.kc[keysym]

    def key(self, keysym, press):
        was = self.state.get(keysym, False)
        if press == was:
            return
        kc = self._keycode(keysym)
        xtest.fake_input(self.disp, X.KeyPress if press else X.KeyRelease, kc)
        self.disp.flush()
        self.state[keysym] = press

    def release_all(self):
        for ks in list(self.state):
            if self.state[ks]:
                self.key(ks, False)

    # ── Джойстик → стрелки ──
    def _joy(self):
        try:
            dd = self.rd(0, 0x63, 0x00, 4)
        except Exception:
            return
        x = dd[0] | (dd[1] << 8)
        y = dd[2] | (dd[3] << 8)
        dx, dy = x - self.cx, y - self.cy

        if abs(dx) < 6000 and abs(dy) < 6000:
            self._jhist.clear()
        else:
            self._jhist.append((dx, dy))
        if self._jhist:
            sdx = statistics.median([p[0] for p in self._jhist])
            sdy = statistics.median([p[1] for p in self._jhist])
        else:
            sdx = sdy = 0

        DEAD = 5000

        # Snap к доминирующей оси: диагональный наклон даёт только одну стрелку.
        # Если одна ось в 2+ раза сильнее другой — слабую зануляем.
        # Убирает ложные диагонали и «залипание» между направлениями (танчики).
        if abs(sdx) > abs(sdy) * 2:
            sdy = 0
        elif abs(sdy) > abs(sdx) * 2:
            sdx = 0

        LEFT, RIGHT, UP, DOWN = 0xFF51, 0xFF53, 0xFF52, 0xFF54
        self.key(LEFT, sdx < -DEAD)
        self.key(RIGHT, sdx > DEAD)
        self.key(UP, sdy < -DEAD)
        self.key(DOWN, sdy > DEAD)

    # ── CardKB → Start/Select/Esc ──
    def _cardkb(self):
        try:
            dd = self.rr(2, 0x5F, 1)
            k = dd[0] if dd else 0
        except Exception:
            return
        if k == self._kl:
            return
        if self._kl:
            ks = self._map(self._kl)
            if ks:
                self.key(ks, False)
        self._kl = k
        if k:
            ks = self._map(k)
            if ks:
                self.key(ks, True)

    def _map(self, code):
        return {
            0x0D: 0xFF0D,    # Enter → Start
            0x20: 0x0020,    # Space → Select
            0x1B: 0xFF1B,    # Esc → выход
        }.get(code)

    # ── GPIO → A/B ──
    def _gpio(self):
        vals = self.lines.get_values()
        mapping = [(96, 0xFF08), (131, 0x30)]
        for (line, keysym), v in zip(mapping, vals):
            pressed = (v == 0)  # pull-up: 0 = нажата
            if pressed != (self._gpio_state[line] == 0):
                self.key(keysym, pressed)
                self._gpio_state[line] = v

    def run(self):
        print('[game] bridge started (joystick=arrows, GPIO=A/B, CardKB=Start/Select/Esc)')
        while True:
            self._joy()
            self._cardkb()
            self._gpio()
            time.sleep(0.010)

    def cleanup(self):
        self.release_all()
        try:
            os.close(self.fd)
        except Exception:
            pass
        self.disp.close()


if __name__ == '__main__':
    import signal
    os.environ.setdefault('DISPLAY', ':0')
    b = GameInput()

    def _term(sig, frm):
        print('[game] SIGTERM — clean exit')
        try:
            b.cleanup()
        except Exception as e:
            print('[game] cleanup err:', e)
        os._exit(0)

    signal.signal(signal.SIGTERM, _term)
    try:
        b.run()
    except KeyboardInterrupt:
        b.cleanup()
