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

import os, fcntl, time, ctypes, subprocess, collections, statistics, math, json, queue, threading
from Xlib import display, X
from Xlib.ext import xtest

try:
    import gi
except ImportError:
    gi = None

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
        self._layout='us'        # current keyboard layout
        self._t9=None            # T9Engine (лениво, при первом включении)
        self._t9_active=False    # Т9-режим (Fn+Tab)
        self._t9_osd=None        # T9OSD-окно (лениво)
        self._t9_led_t=0         # таймер поддержания LED джойстика
        self._t9_caps=False      # следующее Т9-слово — с заглавной (Shift+буква или Sym+цифра)
        self._t9_prev_layout='us'  # раскладка до включения Т9
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

        # ── Калибровка джойстика ──
        self.DZ_ON = 8000     # вход в движение (гистерезис): mag >= 8000
        self.DZ_OFF = 5000    # выход из движения: mag < 5000
        self.SNAP = True      # snap к направлениям (эмуляция D-pad)
        self.SNAP_DIRS = 8    # система/курсор: 8 направлений (диагонали нужны); в игре свой snap в game_input.py
        self.DIR_HYST = 10    # угловой гистерезис (градусы): насколько нужно отойти от текущего направления, чтобы оно сменилось
        self._moving = False  # флаг гистерезиса (магнитуда)
        self._dir = None      # текущее направление (для углового гистерезиса)

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
            if bp and not self._jb and (now-self._jt)>0.15:
                self._jb=True; self._jt=now
                self._led_j(0,0,50)  # синий
                self._cl(1)
                time.sleep(0.05)
                self._led_j(0,0,0)
            elif not bp and self._jb:
                self._jb=False
        except:
            pass

        # Гистерезис по магнитуде: вход в движение при mag>=DZ_ON,
        # выход при mag<DZ_OFF. Убирает дребезг на границе и случайные
        # срабатывания от малых отклонений (серая зона с хаотичным углом).
        mag = math.hypot(dx, dy)
        if self._moving:
            if mag < self.DZ_OFF:
                self._moving = False
                self._dir = None
                self._dx_hist.clear()
                self._dy_hist.clear()
                return
        else:
            if mag < self.DZ_ON:
                return
            self._moving = True

        # Сброс фильтра при смене направления (убирает "эффект памяти")
        if self._dx_hist:
            prev=statistics.median(self._dx_hist)
            if (prev>0)!=(dx>0) and abs(dx)>4000:
                self._dx_hist.clear()
        if self._dy_hist:
            prev=statistics.median(self._dy_hist)
            if (prev>0)!=(dy>0) and abs(dy)>4000:
                self._dy_hist.clear()

        # Медианный фильтр ПЕРЕД snap — сглаживает резкие переходы между
        # осями, чтобы snap не выдал ложную диагональ в момент смены направления
        self._dx_hist.append(dx)
        self._dy_hist.append(dy)
        sdx=statistics.median(self._dx_hist)
        sdy=statistics.median(self._dy_hist)

        # Snap к ближайшему направлению (эмуляция D-pad) по сглаженным
        # значениям. Угловой гистерезис: текущее направление «липкое» — чтобы
        # сменить его, нужно уйти чётко в сторону соседнего (иначе малый наклон
        # дёргает направление туда-сюда).
        if self.SNAP:
            smag = math.hypot(sdx, sdy)
            if smag > 0:
                deg = (math.degrees(math.atan2(sdy, sdx)) + 360.0) % 360.0
                step = 360.0 / self.SNAP_DIRS
                cand = int(round(deg / step)) % self.SNAP_DIRS
                if self._dir is None:
                    self._dir = cand
                else:
                    def _ad(a, b):
                        d = abs(a - b) % 360.0
                        return min(d, 360.0 - d)
                    if _ad(deg, cand * step) < _ad(deg, self._dir * step) - self.DIR_HYST:
                        self._dir = cand
                a = math.radians(self._dir * step)
                sdx = int(round(math.cos(a) * smag))
                sdy = int(round(math.sin(a) * smag))

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
                # Fn+Space (0xAF) — переключение раскладки US/RU
                if k==0xAF:
                    self._layout='ru' if self._layout=='us' else 'us'
                    subprocess.run(['setxkbmap',self._layout], capture_output=True,
                                   env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                    time.sleep(0.1)
                    self._layout=self._real_layout()  # GNOME мог не дать переключить — честная синхронизация
                    print(f'[m5hub] Раскладка: {self._layout.upper()}')
                # Fn+Backspace (0x8B) — гашение экрана (экономия зарядки)
                if k==0x8B:
                    subprocess.run(['xset','dpms','force','off'], capture_output=True,
                                   env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                    print('[m5hub] 🌙 Экран погашен (Fn+Backspace)')
                    self._kl=0
                    return
                # Fn+Enter (0xA3) — открыть «Обзор» (GNOME Overview / список программ)
                if k==0xA3:
                    subprocess.run(['xdotool','key','Super_L'], capture_output=True,
                                   env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                    print('[m5hub] 🗂️ Обзор (GNOME Overview)')
                    self._kl=0
                    return
                # Fn+Tab (0x8C) — включить/выключить Т9-режим (русский ввод)
                if k==0x8C:
                    self._t9_toggle()
                    self._kl=0
                    return
                # Т9-режим: перехват цифр/стрелок/подтверждения, остальное — нативно
                if self._t9_active and self._t9_handle(k):
                    self._kl=k
                    return
                if self._kl and self._kl in CKM: self._kv(CKM[self._kl],0,self._kl)
                if k and k in CKM: self._kv(CKM[k],1,k)
                self._kl=k
        except: pass

    # ── Т9-режим (русский набор цифрами) ──────────────────────────────
    def _t9_toggle(self):
        if not self._t9_active:
            if self._t9 is None:
                self._t9=T9Engine()
            self._t9_prev_layout=self._real_layout()  # запомнить, что было до Т9
            self._t9_active=True
            # Т9 = русский ввод: принудительно ru — никакого смешения рус/лат
            if self._real_layout()!='ru':
                subprocess.run(['setxkbmap','ru'], capture_output=True,
                               env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                time.sleep(0.15)
            self._layout='ru'
            self._led_j(0,80,0)  # зелёный LED — Т9 включён (STM32G0 может гаснуть по таймауту — цикл будет обновлять)
            self._t9_led_t=0
            print('[m5hub] ⌨️ Т9 ВКЛ: 2-9 буквы, ←/→ выбор, Space/Enter подтвердить, Esc сброс, Fn+Tab выкл')
            self._t9_osd_show()
        else:
            self._t9_active=False
            self._t9.reset()
            self._t9_caps=False
            # вернуть раскладку, которая была до включения Т9
            if self._real_layout()!=self._t9_prev_layout:
                subprocess.run(['setxkbmap',self._t9_prev_layout], capture_output=True,
                               env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                time.sleep(0.15)
            self._layout=self._t9_prev_layout
            self._led_j(0,0,0)   # LED погашен — Т9 выключен
            print('[m5hub] ⌨️ Т9 ВЫКЛ')
            self._t9_osd_hide()

    def _t9_handle(self,k):
        """Обработка клавиш в Т9-режиме. True — клавиша съедена."""
        t=self._t9
        # ── multi-tap (ручной ввод слова) ──
        if t.mt:
            if 0x32<=k<=0x39:            # 2-9 — буква (повтор <0.5с — следующая)
                t.mt_digit(chr(k),time.time()); self._t9_osd_show(); return True
            if k==0x08:                  # Backspace — стереть букву
                t.mt_backspace(); self._t9_osd_show(); return True
            if k in (0x0D,0x20):         # Enter/Space — слово готово
                w=t.mt_finish()
                if w:
                    t.add_word(w)
                    self._t9_save()
                    self._t9_type(w,True)
                self._t9_osd_hide(); return True
            if k==0x1B:                  # Esc — отмена ручного ввода
                t.mt_cancel(); t.reset(); self._t9_osd_show(); return True
            return False                 # остальное — нативно
        # Shift+буква (0x41-0x5A) — следующее слово с заглавной (буква не вводится)
        if 0x41<=k<=0x5A:
            self._t9_caps=True
            self._t9_osd_show(); return True
        # Sym+цифра (!@#$%^&*() — уникальные коды) — цифра с заглавной
        SYM_D={0x21:'1',0x40:'2',0x23:'3',0x24:'4',0x25:'5',0x5E:'6',0x26:'7',0x2A:'8',0x28:'9',0x29:'0'}
        if k in SYM_D:
            self._t9_caps=True
            t.add_digit(SYM_D[k]); self._t9_osd_show(); return True
        if k==0x30:                  # 0 — тоже пробел (подтвердить + пробел, или просто пробел)
            if t.seq:
                w=t.confirm()
                if w: self._t9_type(w,True)
                self._t9_osd_hide(); return True
            self._kv(0x0020,1,0x20); self._kv(0x0020,0,0x20)
            self._kl=k; return True
        if 0x32<=k<=0x39:            # 2-9 — цифры набора
            t.add_digit(chr(k)); self._t9_osd_show(); return True
        if k==0x08 and t.seq:        # Backspace — стереть цифру
            t.backspace(); self._t9_osd_show(); return True
        if k==0xB4:                  # ← — предыдущий кандидат
            t.prev_cand(); self._t9_osd_show(); return True
        if k==0xB7:                  # → — следующий кандидат
            t.next_cand(); self._t9_osd_show(); return True
        if k==0x20 and t.seq:        # Space — подтвердить + пробел
            w=t.confirm()
            if w: self._t9_type(w,True)
            self._t9_osd_hide(); return True
        if k==0x0D and t.seq:        # Enter — подтвердить без пробела, при пустом словаре — ручной ввод
            w=t.confirm()
            if w:
                self._t9_type(w,False)
            else:
                t.mt_start()          # слова нет — переходим в multi-tap
            self._t9_osd_show(); return True
        if k==0x1B and t.seq:        # Esc — только сброс набора (выход — только Fn+Tab)
            t.reset(); self._t9_osd_show(); return True
        return False

    def _t9_save(self):
        """Сохранить словарь (после добавления нового слова)."""
        try:
            with open(_T9_BASE,'w',encoding='utf-8') as f:
                json.dump(self._t9.base,f,ensure_ascii=False,separators=(',',':'))
            print(f'[m5hub] 📖 Словарь обновлён ({len(self._t9.base)} цепочек)')
        except Exception as e:
            print('[m5hub] Словарь не сохранился:',e)

    def _real_layout(self):
        """Реальная раскладка X-сервера (setxkbmap -query) — не полагаемся на self._layout,
        потому что GNOME/mutter переключает раскладку сам (input-sources)."""
        try:
            r=subprocess.run(['setxkbmap','-query'],capture_output=True,text=True,
                             env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
            for ln in r.stdout.splitlines():
                if ln.strip().startswith('layout:'):
                    return ln.split()[-1]
        except Exception:
            pass
        return self._layout

    def _t9_type(self,word,space):
        """Ввод русского слова через XTest: временно ru-раскладка, затем вернуть как было.
        Используем физические keycode ЙЦУКЕН — они не зависят от кэша раскладки python-xlib."""
        # физические keycode русской ЙЦУКЕН (PC-клавиатура, проверено на сервере 33/33)
        RU_KC={'й':24,'ц':25,'у':26,'к':27,'е':28,'н':29,'г':30,'ш':31,'щ':32,'з':33,'х':34,'ъ':35,
               'ф':38,'ы':39,'в':40,'а':41,'п':42,'р':43,'о':44,'л':45,'д':46,'ж':47,'э':48,'ё':49,
               'я':52,'ч':53,'с':54,'м':55,'и':56,'т':57,'ь':58,'б':59,'ю':60}
        real=self._real_layout()
        try:
            if real!='ru':
                subprocess.run(['setxkbmap','ru'], capture_output=True,
                               env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                time.sleep(0.15)  # дать X-серверу обновить раскладку
                # ВАЖНО: НЕ обновляем кэш python-xlib! _kv() полагается на us-кэш
                # (латинские keysym'ы) и вводит физические keycode — реальная раскладка
                # превращает их в русские буквы. _update_keymap сломал бы латинский ввод.
            for i,ch in enumerate(word):
                kc=RU_KC.get(ch,0)
                if kc:
                    if self._t9_caps and i==0:
                        xtest.fake_input(self.d,X.KeyPress,50); self.d.flush()  # Shift
                    xtest.fake_input(self.d,X.KeyPress,kc); self.d.flush()
                    xtest.fake_input(self.d,X.KeyRelease,kc); self.d.flush()
                    if self._t9_caps and i==0:
                        xtest.fake_input(self.d,X.KeyRelease,50); self.d.flush()
            self._t9_caps=False  # заглавная — только для первого слова после Shift
            if space:
                xtest.fake_input(self.d,X.KeyPress,65); self.d.flush()
                xtest.fake_input(self.d,X.KeyRelease,65); self.d.flush()
        finally:
            # в Т9-режиме всегда возвращаемся на ru (Т9 = русский ввод), иначе — как было
            target='ru' if self._t9_active else real
            if self._real_layout()!=target:
                subprocess.run(['setxkbmap',target], capture_output=True,
                               env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
                time.sleep(0.2)  # дать X-серверу применить раскладку — иначе XTest-события теряются
        self._layout=target  # синхронизация с реальной раскладкой

    def _t9_osd_show(self):
        if self._t9_osd is None:
            self._t9_osd=T9OSD()
        text=self._t9.osd_text()
        if self._t9_caps:
            text='<span color="#6f6" size="large">🔠 С ЗАГЛАВНОЙ</span>  '+text
        self._t9_osd.update(text)

    def _t9_osd_hide(self):
        if self._t9_osd is not None:
            self._t9_osd.hide()

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
                env={'DISPLAY': os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
        except Exception:
            pass
    def run(self):
        print("[m5hub] 🚀 J0 S1 K2 (v9 — median filter + PaHub reset)")
        self.ro.warp_pointer(self.sw//2,self.sh//2); self.d.flush()
        # Switch to US layout — CardKB is a US QWERTY keyboard
        subprocess.run(['setxkbmap','us'], capture_output=True,
                       env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
        print('[m5hub] 🇺🇸 Раскладка: US')
        subprocess.run(['xdotool','mousemove',str(self.sw//2),str(self.sh//2)],
                       capture_output=True, env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
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
                # Поддержание зелёного LED джойстика, пока Т9 активен (STM32G0 гаснет по таймауту)
                if self._t9_active and t-self._t9_led_t>=2.0:
                    self._led_j(0,80,0)
                    self._t9_led_t=t
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
                       env={'DISPLAY':os.environ.get('DISPLAY',':0'),'XAUTHORITY':os.environ.get('XAUTHORITY','/home/orangepi/.Xauthority')})
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


# ── Т9-движок и OSD-окно (русский набор цифрами) ───────────────────
_T9_BASE='/home/orangepi/.openclaw/workspace/t9/ru_t9.json'
_T9_LAYOUT={'2':'абвг','3':'деёжз','4':'ийкл','5':'мноп','6':'рсту','7':'фхцч','8':'шщъы','9':'ьэюя'}
_T9_KEY={}
for _k,_v in _T9_LAYOUT.items():
    for _ch in _v: _T9_KEY[_ch]=_k
_T9_KEY['ё']='3'

class T9Engine:
    """Последовательность цифр -> кандидаты по частоте (база: ru_t9.json)."""
    def __init__(self,base_path=_T9_BASE):
        with open(base_path,encoding='utf-8') as f:
            self.base=json.load(f)
        self.seq=''; self.cands=[]; self.idx=0
        # multi-tap (ручной ввод слова, которого нет в словаре)
        self.mt=False; self.mt_word=''; self.mt_cur=''; self.mt_last_d=None; self.mt_last_t=0.0
    def add_digit(self,d):
        self.seq+=d; self._recalc()
    def backspace(self):
        if self.seq:
            self.seq=self.seq[:-1]; self._recalc()
    def next_cand(self):
        if self.cands: self.idx=(self.idx+1)%len(self.cands)
    def prev_cand(self):
        if self.cands: self.idx=(self.idx-1)%len(self.cands)
    def confirm(self):
        w=self.cands[self.idx] if self.cands else None
        self.reset(); return w
    def reset(self):
        self.seq=''; self.cands=[]; self.idx=0
        self.mt=False; self.mt_word=''; self.mt_cur=''; self.mt_last_d=None
    def _recalc(self):
        self.cands=self.base.get(self.seq,[]); self.idx=0

    # ── multi-tap: ручной ввод слова ──
    def mt_start(self):
        self.mt=True; self.mt_word=''; self.mt_cur=''; self.mt_last_d=None; self.mt_last_t=0.0
    def mt_digit(self,d,now):
        """Нажатие цифры в multi-tap: повтор <0.5с — следующая буква группы."""
        grp=_T9_LAYOUT[d]
        if self.mt_last_d==d and self.mt_cur and now-self.mt_last_t<0.5:
            self.mt_cur=grp[(grp.index(self.mt_cur)+1)%len(grp)]
        else:
            if self.mt_cur: self.mt_word+=self.mt_cur   # фиксация предыдущей буквы
            self.mt_cur=grp[0]
        self.mt_last_d=d; self.mt_last_t=now
    def mt_backspace(self):
        if self.mt_word: self.mt_word=self.mt_word[:-1]
        self.mt_cur=''; self.mt_last_d=None
    def mt_finish(self):
        w=self.mt_word+self.mt_cur
        self.mt=False; self.mt_word=''; self.mt_cur=''; self.mt_last_d=None
        return w
    def mt_cancel(self):
        self.mt=False; self.mt_word=''; self.mt_cur=''; self.mt_last_d=None
    def mt_text(self):
        return self.mt_word+self.mt_cur
    def add_word(self,word):
        """Добавить слово в словарь (в начало кандидатов). Возвращает Т9-код."""
        seq=''.join(_T9_KEY.get(c,'') for c in word.lower())
        if not seq: return None
        lst=self.base.get(seq,[])
        if word in lst: lst.remove(word)
        self.base[seq]=[word]+lst
        return seq

    def osd_text(self):
        if self.mt:
            # multi-tap: слово красным (режим добавления нового слова)
            return (f'<span color="#f55" size="large">✍️</span> '
                    f'<b><span color="#f55" size="large">{self.mt_text() or "_"}</span></b> '
                    f'<span color="#aaa">· цифра — буква (быстрый повтор — след.) · Enter — готово</span>')
        if not self.seq:
            return '<span color="#777">Т9: 2-9 буквы · Backspace стереть · ←/→ выбор · Space/0 подтвердить · Fn+Tab выкл</span>'
        if not self.cands:
            return f'<span color="#f66">нет слова: {self.seq}</span> <span color="#aaa">— Enter: ручной ввод</span>'
        parts=[]
        for i,w in enumerate(self.cands[:8]):
            if i==self.idx:
                parts.append(f'<b><span color="#fff" background="#264" size="large">{w}</span></b>')
            else:
                parts.append(f'<span color="#aaa" size="large">{w}</span>')
        more=f' <span color="#666">+{len(self.cands)-8}</span>' if len(self.cands)>8 else ''
        return '  '.join(parts)+more

class T9OSD:
    """GTK-подсказка внизу по центру экрана. Не берёт фокус."""
    def __init__(self):
        self.q=queue.Queue()
        self.visible=False; self.win=None; self.label=None
        threading.Thread(target=self._run,daemon=True).start()
    def _run(self):
        if gi is None:
            return
        gi.require_version('Gtk','3.0')
        from gi.repository import Gtk, GLib
        Gtk.init(None)
        self.win=Gtk.Window(type=Gtk.WindowType.POPUP)
        self.win.set_decorated(False); self.win.set_keep_above(True)
        self.win.set_accept_focus(False); self.win.set_can_focus(False)
        self.win.set_skip_taskbar_hint(True); self.win.set_skip_pager_hint(True)
        # Полупрозрачность 50%: rgba-визуал + draw с альфой (текст остаётся непрозрачным)
        screen=self.win.get_screen()
        vis=screen.get_rgba_visual()
        if vis is not None:
            self.win.set_visual(vis)
        self.win.set_app_paintable(True)
        def _draw(w,cr):
            cr.set_source_rgba(0.12,0.12,0.12,0.5)  # тёмный фон, альфа 50%
            cr.paint()
            return False
        self.win.connect('draw',_draw)
        self.win.set_size_request(520,48)
        sw,sh=screen.get_width(),screen.get_height()
        self.win.move((sw-520)//2,sh-60)
        self.label=Gtk.Label(); self.label.set_use_markup(True)
        self.label.set_halign(Gtk.Align.CENTER)
        self.win.add(self.label); self.win.show_all(); self.win.hide()
        GLib.timeout_add(40,self._poll)
        Gtk.main()
    def _poll(self):
        try:
            while True:
                text,show=self.q.get_nowait()
                self.label.set_markup(text or '')
                if show and not self.visible:
                    self.win.show(); self.visible=True
                elif not show and self.visible:
                    self.win.hide(); self.visible=False
        except queue.Empty:
            pass
        return True
    def update(self,markup):
        self.q.put((markup,True))
    def hide(self):
        self.q.put(('',False))


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
