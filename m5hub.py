#!/usr/bin/env python3
"""
m5hub.py v9 — стабильная версия
Исправления на основе отладки и документации PaHub:
1. Сброс каналов PaHub (0x00) перед выбором — убирает пачки ошибок I2C
2. Медианный фильтр (6 семплов) — отсекает перекрёстные помехи
3. Виртуальная позиция курсора — без race condition
4. Фиксированный scale /32768 (без авто-калибровки)
5. Увеличенная dead zone 6000
"""

import os, fcntl, time, ctypes, subprocess, collections, statistics
from Xlib import display, X
from Xlib.ext import xtest

I2C_BUS=0; I2C_RDWR=0x0707; I2C_M_RD=1

class m(ctypes.Structure):
    _fields_=[('addr',ctypes.c_uint16),('flags',ctypes.c_uint16),
              ('len',ctypes.c_uint16),('buf',ctypes.POINTER(ctypes.c_uint8))]
class d(ctypes.Structure):
    _fields_=[('msgs',ctypes.POINTER(m)),('nmsgs',ctypes.c_uint32)]

_g = []

def i2c_wr(fd, ad, da):
    b=(ctypes.c_uint8*len(da))(*da)
    mg=m(ad,0,len(da),b)
    wd=d(ctypes.pointer(mg),1)
    _g.extend([b,mg,wd])
    fcntl.ioctl(fd,I2C_RDWR,wd)
    _g.clear()  # освобождаем буферы сразу — иначе утечка памяти
    time.sleep(0.002)

def i2c_rd(fd, ad, re, n):
    wb=(ctypes.c_uint8*1)(re); rb=(ctypes.c_uint8*n)()
    m0=m(ad,0,1,wb); m1=m(ad,I2C_M_RD,n,rb)
    ms=(m*2)(m0,m1); wd=d(ms,2)
    _g.extend([wb,rb,m0,m1,ms,wd])
    fcntl.ioctl(fd,I2C_RDWR,wd)
    out=bytes(rb)
    _g.clear()  # освобождаем буферы сразу — иначе утечка памяти
    return out

def i2c_rr(fd, ad, n):
    rb=(ctypes.c_uint8*n)()
    mg=m(ad,I2C_M_RD,n,rb)
    wd=d(ctypes.pointer(mg),1)
    _g.extend([rb,mg,wd])
    fcntl.ioctl(fd,I2C_RDWR,wd)
    out=bytes(rb)
    _g.clear()  # освобождаем буферы сразу — иначе утечка памяти
    return out


class Hub:
    def __init__(self):
        self.fd=os.open(f'/dev/i2c-{I2C_BUS}',os.O_RDWR)
        _g.append(self.fd)
        self._rst()

        self.d=display.Display(':0')
        self.ro=self.d.screen().root
        self.sw=self.d.screen().width_in_pixels
        self.sh=self.d.screen().height_in_pixels
        print(f"[m5hub] Экран: {self.sw}x{self.sh}")

        self.go=True
        self._t={'j':0,'s':0,'k':0}
        self._sb=False; self._sf=False; self._sp=0
        self._kl=0
        self._bs_last=0.0      # last Backspace timestamp
        self._bs_repeat=False  # auto-repeat active
        self._bs_next=0.0      # next repeat timestamp
        self._layout='us'        # current keyboard layout
        self._cx=self._cy=32768

        # Виртуальная позиция (вместо query_pointer)
        qp=self.ro.query_pointer()
        self._vx=qp.root_x
        self._vy=qp.root_y

        # Медианный фильтр: окно 6 семплов
        self._dx_hist=collections.deque(maxlen=6)
        self._dy_hist=collections.deque(maxlen=6)

        # Фиксированный масштаб (никакой авто-калибровки!)
        self._scale=32768.0

        self._err_count=0
        self._jb=False  # кнопка джойстика
        self._jt=0     # debounce таймер
        self._cal()

    def _rst(self):
        """Сброс PaHub — все каналы выключены"""
        try:
            i2c_wr(self.fd,0x70,[0x00])
            time.sleep(0.01)
        except: pass

    def sel(self,c):
        """Выбор канала PaHub с предварительным сбросом"""
        # Сначала сбрасываем ВСЕ каналы (документация PaHub рекомендует)
        try:
            i2c_wr(self.fd,0x70,[0x00])
            time.sleep(0.001)
        except: pass
        # Затем выбираем нужный
        i2c_wr(self.fd,0x70,[1<<c])
        time.sleep(0.005)

    def rd(self,c,a,re,n):
        self.sel(c)
        return i2c_rd(self.fd,a,re,n)

    def wr(self,c,a,re,da):
        self.sel(c)
        if isinstance(da,int): da=[da]
        i2c_wr(self.fd,a,[re]+list(da))

    def rr(self,c,a,n):
        self.sel(c)
        return i2c_rr(self.fd,a,n)

    def _cal(self):
        """Калибровка центра джойстика — устойчивая к выбросам (I2C-мусор после жёсткого kill)"""
        samples=[]
        for _ in range(50):
            try:
                d=self.rd(0,0x63,0x00,4)
                samples.append((d[0]|(d[1]<<8), d[2]|(d[3]<<8)))
            except: pass
            time.sleep(0.01)
        if samples:
            xs=sorted(s[0] for s in samples); ys=sorted(s[1] for s in samples)
            mx=xs[len(xs)//2]; my=ys[len(ys)//2]
            # Отбрасываем выбросы (дальше 4000 от медианы)
            good=[(x,y) for x,y in samples if abs(x-mx)<4000 and abs(y-my)<4000]
            if good:
                self._cx=sum(g[0] for g in good)//len(good)
                self._cy=sum(g[1] for g in good)//len(good)
                print(f"[m5hub] ⚙️ Центр: X={self._cx} Y={self._cy} (n={len(good)}/{len(samples)})")
            else:
                self._cx,self._cy=mx,my
                print(f"[m5hub] ⚙️ Центр(медиана): X={self._cx} Y={self._cy}")

    def _j(self):
        try:
            d=self.rd(0,0x63,0x00,4)
        except:
            self._err_count+=1
            return
        x_raw=d[0]|(d[1]<<8)
        y_raw=d[2]|(d[3]<<8)
        dx=x_raw-self._cx
        dy=y_raw-self._cy

        # Кнопка джойстика (центральный щелчок) = левый клик
        # Регистр 0x20, инвертирована: 0=нажата, !=0=отпущена
        try:
            bd=self.rd(0,0x63,0x20,1)
            bp=(bd[0]==0) if bd else False
            now=time.time()
            if bp and not self._jb and (now-self._jt)>0.3:
                self._jb=True; self._jt=now
                self._led_j(0,0,50)  # синий
                self._cl(1)
                time.sleep(0.05)
                self._led_j(0,0,0)
            elif not bp and self._jb:
                self._jb=False
        except:
            pass

        # Dead zone + сброс фильтра при остановке
        if abs(dx)<6000 and abs(dy)<6000:
            self._dx_hist.clear()
            self._dy_hist.clear()
            return

        # Сброс фильтра при смене направления (убирает "эффект памяти")
        if self._dx_hist:
            prev=statistics.median(self._dx_hist)
            if (prev>0)!=(dx>0) and abs(dx)>4000:
                self._dx_hist.clear()
        if self._dy_hist:
            prev=statistics.median(self._dy_hist)
            if (prev>0)!=(dy>0) and abs(dy)>4000:
                self._dy_hist.clear()

        # Медианный фильтр
        self._dx_hist.append(dx)
        self._dy_hist.append(dy)
        sdx=statistics.median(self._dx_hist)
        sdy=statistics.median(self._dy_hist)

        # Простой линейный scale
        sx=int(sdx/self._scale*110)
        sy=int(sdy/self._scale*110)

        if sx==0 and sy==0: return

        # Виртуальная позиция
        # Целевая позиция
        tx=max(0,min(self.sw-1,self._vx+sx))
        ty=max(0,min(self.sh-1,self._vy+sy))
        # Экспоненциальное сглаживание (0.30 = 30% к цели за тик)
        self._vx+=(tx-self._vx)*0.30
        self._vy+=(ty-self._vy)*0.30
        self.ro.warp_pointer(int(self._vx),int(self._vy))
        self.d.flush()

    def _s(self):
        try:
            d=self.rd(1,0x40,0x50,4)
            v=d[0]|(d[1]<<8)|(d[2]<<16)|(d[3]<<24)
            if v&0x80000000: v-=0x100000000
            if v and abs(v)<100:
                self._wh(v); self._led(0,50,50); time.sleep(0.03); self._led(0,0,0)
        except: pass
        try:
            d=self.rd(1,0x40,0x20,1)
            b=d[0]==0; t=time.time()
            if b and not self._sb:
                self._sb=1; self._sp=t; self._sf=0
            elif b and not self._sf and (t-self._sp)>=0.5:
                self._cl(3); self._led(50,0,0); self._sf=1
            elif not b and self._sb:
                self._sb=0
                if not self._sf:
                    self._led(0,50,0); self._cl(1); time.sleep(0.05)
                self._led(0,0,0)
        except: pass

    def _wh(self,c):
        b=4 if c>0 else 5
        for _ in range(abs(c)):
            xtest.fake_input(self.d,X.ButtonPress,b)
            xtest.fake_input(self.d,X.ButtonRelease,b)
        self.d.flush()

    def _cl(self,btn):
        xtest.fake_input(self.d,X.ButtonPress,btn); self.d.flush()
        time.sleep(0.015)
        xtest.fake_input(self.d,X.ButtonRelease,btn); self.d.flush()

    def _led(self,r,g,b):
        """RGB LED на Scroll (канал 1, адрес 0x40)"""
        try: self.wr(1,0x40,0x30,[0,g,r,b])
        except: pass

    def _led_j(self,r,g,b):
        """RGB LED на джойстике V2 — регистры 0x30-0x32 (B,G,R)"""
        try:
            # Протокол STM32: [0x30, B, G, R] (Blue=0x30, Green=0x31, Red=0x32)
            da=[0x30, b, g, r]
            ba=(ctypes.c_uint8*len(da))(*da)
            mg=m(0x63,0,len(da),ba)
            wd=d(ctypes.pointer(mg),1)
            _g.extend([ba,mg,wd])
            fcntl.ioctl(self.fd,I2C_RDWR,wd)
            _g.clear()  # освобождаем буферы сразу — иначе утечка памяти
        except:
            pass

    def _k(self):
        try:
            d=self.rr(2,0x5F,1); k=d[0] if d else 0
            if k!=self._kl:
                # DEBUG: log raw codes to /tmp/cardkb.log
                with open('/tmp/cardkb.log','a') as f:
                    f.write(f'{time.time():.3f} raw=0x{k:02X} prev=0x{self._kl:02X}\n')
                # Double-tap Backspace detection
                now=time.time()
                if k==0x08:
                    if self._bs_last>0 and now-self._bs_last<0.8:
                        self._bs_repeat=True
                        self._bs_next=now+0.2
                        print("[m5hub] BS auto-repeat (double-tap)")
                    self._bs_last=now
                elif k and k!=0:
                    if self._bs_repeat:
                        print("[m5hub] BS auto-repeat off")
                    self._bs_repeat=False
                    self._bs_last=0
                # Fn+Space (0xAF) — переключение раскладки US/RU
                if k==0xAF:
                    self._layout='ru' if self._layout=='us' else 'us'
                    subprocess.run(['setxkbmap',self._layout], capture_output=True,
                                   env={'DISPLAY':os.environ.get('DISPLAY',':0')})
                    print(f'[m5hub] Раскладка: {self._layout.upper()}')
                # Fn+Backspace (0x8B) — гашение экрана (экономия зарядки)
                if k==0x8B:
                    subprocess.run(['xset','dpms','force','off'], capture_output=True,
                                   env={'DISPLAY':os.environ.get('DISPLAY',':0')})
                    print('[m5hub] 🌙 Экран погашен (Fn+Backspace)')
                    self._kl=0
                    return
                if self._kl and self._kl in CKM: self._kv(CKM[self._kl],0,self._kl)
                if k and k in CKM: self._kv(CKM[k],1,k)
                self._kl=k
        except: pass

    def _kv(self,s,p,raw=0):
        if not p:
            if 0x41 <= raw <= 0x5A:
                kc = self.d.keysym_to_keycode(s + 0x20)
                if kc:
                    xtest.fake_input(self.d, X.KeyRelease, kc)
                    self.d.flush()
                    xtest.fake_input(self.d, X.KeyRelease, 50)
                    self.d.flush()
            elif raw in _XT_SYMS:
                kc = self.d.keysym_to_keycode(s)
                if kc:
                    xtest.fake_input(self.d, X.KeyRelease, kc)
                    self.d.flush()
            return
        if raw in _XT_SYMS:
            is_upper = 0x41 <= raw <= 0x5A
            lookup = s + 0x20 if is_upper else s
            kc = self.d.keysym_to_keycode(lookup)
            if not kc:
                kc = self.d.keysym_to_keycode(s)
            if kc:
                if is_upper:
                    xtest.fake_input(self.d, X.KeyPress, 50)
                    self.d.flush()
                xtest.fake_input(self.d, X.KeyPress, kc)
                self.d.flush()
            return
        try:
            subprocess.run(
                ['xdotool','type','--clearmodifiers','--delay','0',chr(s)],
                capture_output=True, timeout=1,
                env={'DISPLAY': os.environ.get('DISPLAY',':0')})
        except Exception:
            pass
    def run(self):
        print("[m5hub] 🚀 J0 S1 K2 (v9 — median filter + PaHub reset)")
        self.ro.warp_pointer(self.sw//2,self.sh//2); self.d.flush()
        # Switch to US layout — CardKB is a US QWERTY keyboard
        subprocess.run(['setxkbmap','us'], capture_output=True,
                       env={'DISPLAY':os.environ.get('DISPLAY',':0')})
        print('[m5hub] 🇺🇸 Раскладка: US')
        subprocess.run(['xdotool','mousemove',str(self.sw//2),str(self.sh//2)],
                       capture_output=True, env={'DISPLAY':os.environ.get('DISPLAY',':0')})
        for r,g,b in [(50,0,0),(0,50,0),(0,0,50),(0,0,0)]:
            self._led(r,g,b); time.sleep(0.08)
        # Гасим LED джойстика при старте
        self._led_j(0,0,0)
        print("[m5hub] 🟢 Готов")
        while self.go:
            try:
                t=time.time()
                if t-self._t['j']>=0.030: self._j(); self._t['j']=t
                if t-self._t['s']>=0.020: self._s(); self._t['s']=t
                if t-self._t['k']>=0.060: self._k(); self._t['k']=t
                # Backspace auto-repeat
                if self._bs_repeat and t>=self._bs_next:
                    self._kv(0xFF08,1,0x08); self._kv(0xFF08,0,0x08)
                    self._bs_next=t+0.05
                time.sleep(0.002)
            except KeyboardInterrupt: break
        print(f"[m5hub] Off (ошибок I2C: {self._err_count})")

    def cleanup(self):
        self._led(0,0,0)
        try: self._rst()  # сброс PaHub — чтобы шина не осталась залипшей
        except: pass
        os.close(self.fd); self.d.close()
        # Restore original layout
        subprocess.run(['setxkbmap','us,ru,ru'], capture_output=True,
                       env={'DISPLAY':os.environ.get('DISPLAY',':0')})
        print('[m5hub] 🇷🇺 Раскладка восстановлена')


# Symbols handled by XTest (letters, digits, space, control keys)
# Everything else goes through xdotool type
_XT_SYMS = frozenset(
    list(range(0x30, 0x3A))   # 0-9
    + list(range(0x41, 0x5B)) # A-Z
    + list(range(0x61, 0x7B)) # a-z
    + [0x20,   # Space
       0x0D,   # Enter
       0x09,   # Tab
       0x1B,   # Esc
       0x08,   # Backspace
       0xB4, 0xB5, 0xB6, 0xB7,  # Arrows (our firmware)
    ]
)

CKM = {
    # ── Control keys ──
    0x1B: 0xFF1B,  # Esc
    0x08: 0xFF08,  # Backspace (Del key in normal mode)
    0x7F: 0xFFFF,  # Delete (Shift+Del)
    0x09: 0xFF09,  # Tab
    0x0D: 0xFF0D,  # Enter
    0x20: 0x0020,  # Space

    # ── Arrow keys (CardKB custom codes 180-183) ──
    0xB4: 0xFF51,  # Left
    0xB5: 0xFF52,  # Up
    0xB6: 0xFF54,  # Down
    0xB7: 0xFF53,  # Right

    # ── ASCII printables 1:1 with X11 keysyms ──
    **{c: c for c in range(0x21, 0x5C)},  # ! through Z
    **{c: c for c in range(0x5C, 0x7F)},  # \ through ~
    **{c: c for c in range(0x61, 0x7B)},  # a-z
}


if __name__=='__main__':
    import sys, signal; os.environ.setdefault('DISPLAY',':0')
    h=Hub()
    def _term(sig,frm):
        print('[m5hub] SIGTERM — чистое завершение')
        try: h.cleanup()
        except Exception as e: print('[m5hub] cleanup err:', e)
        os._exit(0)
    signal.signal(signal.SIGTERM,_term)
    try: h.run()
    except KeyboardInterrupt: h.cleanup()
