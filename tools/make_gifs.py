"""
デモの GIF（media/*.gif）を作り直すスクリプト（Windows 専用。依存: numpy, Pillow）

    python tools/make_gifs.py build/Release/demo.exe media [名前の一部]

demo.exe を起動し、H で HUD を消して P で一時停止したあと、N キー（1/120 s 進める）を
送りながらウィンドウを 1 フレームずつ撮る。シーン 9, 0, -（実時間で動くシーン）と、電気のシーン
F5〜F8 は、一時停止を解いて実時間で動かしながら約 15 fps で撮る。
キーは PostMessage で送るのでフォーカスは奪わない。撮影中はデモのウィンドウが画面に出る。
3 つ目の引数（カンマ区切り）を付けると、ファイル名にそのどれかを含む GIF だけを作る。
"""
import ctypes
import ctypes.wintypes as wt
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image

user32 = ctypes.windll.user32
gdi32 = ctypes.windll.gdi32
user32.SetProcessDPIAware()
user32.PostMessageW.argtypes = [wt.HWND, wt.UINT, wt.WPARAM, wt.LPARAM]
user32.PrintWindow.argtypes = [wt.HWND, wt.HDC, wt.UINT]


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [("biSize", wt.DWORD), ("biWidth", wt.LONG), ("biHeight", wt.LONG),
                ("biPlanes", wt.WORD), ("biBitCount", wt.WORD), ("biCompression", wt.DWORD),
                ("biSizeImage", wt.DWORD), ("biXPelsPerMeter", wt.LONG), ("biYPelsPerMeter", wt.LONG),
                ("biClrUsed", wt.DWORD), ("biClrImportant", wt.DWORD)]


def find_window(pid, timeout=10.0):
    found = []
    EnumProc = ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)

    def cb(hwnd, _):
        p = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(p))
        if p.value == pid and user32.IsWindowVisible(hwnd) and user32.GetWindowTextLengthW(hwnd) > 0:
            found.append(hwnd)
        return True

    t0 = time.time()
    while not found and time.time() - t0 < timeout:
        user32.EnumWindows(EnumProc(cb), 0)
        time.sleep(0.1)
    return found[0] if found else None


VK = {'-': 0xBD, '=': 0xBB, 'enter': 0x0D, 'bksp': 0x08, 'up': 0x26, 'down': 0x28, 'left': 0x25, 'right': 0x27}
VK.update({'f%d' % (k + 1): 0x70 + k for k in range(12)})   # F1〜F12（電気のシーンは F5〜F8）
EXTENDED = {'up', 'down', 'left', 'right'}
# raylib（GLFW）はスキャンコードでキーを決める。'=' は US 配列の位置（JIS 配列では ^）の 0x0D を直接送る
SCAN = {'=': 0x0D}


def _key_msg(hwnd, key, down):
    vk = VK[key] if key in VK else ord(key.upper())
    sc = SCAN[key] if key in SCAN else user32.MapVirtualKeyW(vk, 0)
    lp = 1 | (sc << 16) | ((1 << 24) if key in EXTENDED else 0)
    if not down:
        lp |= (1 << 30) | (1 << 31)
    user32.PostMessageW(hwnd, 0x100 if down else 0x101, vk, lp)                        # WM_KEYDOWN / UP


def key_down(hwnd, key):
    _key_msg(hwnd, key, True)


def key_up(hwnd, key):
    _key_msg(hwnd, key, False)


def send_key(hwnd, key, hold=0.03, gap=0.03):
    key_down(hwnd, key)
    time.sleep(hold)
    key_up(hwnd, key)
    time.sleep(gap)


def capture(hwnd):
    wr, cr, pt = wt.RECT(), wt.RECT(), wt.POINT(0, 0)
    user32.GetWindowRect(hwnd, ctypes.byref(wr))
    user32.GetClientRect(hwnd, ctypes.byref(cr))
    user32.ClientToScreen(hwnd, ctypes.byref(pt))
    w, h = wr.right - wr.left, wr.bottom - wr.top

    hdc_win = user32.GetWindowDC(hwnd)
    hdc_mem = gdi32.CreateCompatibleDC(hdc_win)
    hbmp = gdi32.CreateCompatibleBitmap(hdc_win, w, h)
    old = gdi32.SelectObject(hdc_mem, hbmp)
    user32.PrintWindow(hwnd, hdc_mem, 2)                                                # PW_RENDERFULLCONTENT

    bmi = BITMAPINFOHEADER()
    bmi.biSize, bmi.biWidth, bmi.biHeight = ctypes.sizeof(BITMAPINFOHEADER), w, -h    # 上から下
    bmi.biPlanes, bmi.biBitCount = 1, 32
    buf = ctypes.create_string_buffer(w * h * 4)
    gdi32.GetDIBits(hdc_mem, hbmp, 0, h, buf, ctypes.byref(bmi), 0)

    gdi32.SelectObject(hdc_mem, old)
    gdi32.DeleteObject(hbmp)
    gdi32.DeleteDC(hdc_mem)
    user32.ReleaseDC(hwnd, hdc_win)

    img = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)[:, :, 2::-1]            # BGRA → RGB
    ox, oy = pt.x - wr.left, pt.y - wr.top
    return Image.fromarray(img[oy:oy + cr.bottom, ox:ox + cr.right].copy())


def record(hwnd, frames, steps_per_frame, settle=0.05):
    out = [capture(hwnd)]
    for _ in range(frames):
        for _ in range(steps_per_frame):
            send_key(hwnd, 'n')
        time.sleep(settle)
        out.append(capture(hwnd))
    return out


def record_realtime(hwnd, seconds, fps=15, holds=()):
    """実時間で動いているウィンドウを撮る。holds = [(キー, 押す秒, 離す秒), ...] の間キーを押し続ける"""
    frames, t0 = [], time.time()
    state = [0] * len(holds)              # 0: まだ 1: 押している 2: 離した
    while time.time() - t0 < seconds:
        t = time.time()
        for i, (key, start, end) in enumerate(holds):
            if state[i] == 0 and t - t0 >= start:
                key_down(hwnd, key)
                state[i] = 1
            elif state[i] == 1 and t - t0 >= end:
                key_up(hwnd, key)
                state[i] = 2
        frames.append(capture(hwnd))
        time.sleep(max(0.0, 1.0 / fps - (time.time() - t)))
    for i, (key, _, _) in enumerate(holds):
        if state[i] == 1:
            key_up(hwnd, key)
    return frames, int(1000 * (time.time() - t0) / len(frames))


def save_gif(frames, path, width, delay_ms, hold_first_ms, hold_last_ms, colors=128):
    h = round(frames[0].height * width / frames[0].width)
    small = [f.resize((width, h), Image.LANCZOS) for f in frames]
    # 一部のフレームを並べて共通パレットを作る（フレームごとの色のちらつきを防ぐ）
    sample = small[::max(1, len(small) // 8)]
    sheet = Image.new("RGB", (width, h * len(sample)))
    for i, f in enumerate(sample):
        sheet.paste(f, (0, i * h))
    pal = sheet.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in small]
    durations = [delay_ms] * len(q)
    durations[0], durations[-1] = hold_first_ms, hold_last_ms
    q[0].save(path, save_all=True, append_images=q[1:], duration=durations, loop=0, disposal=1)
    print(f"saved {path}  {len(q)} frames  {os.path.getsize(path) / 1e6:.2f} MB")


def main():
    exe, outdir = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    only = sys.argv[3].split(",") if len(sys.argv) > 3 else [""]   # カンマ区切り
    os.makedirs(outdir, exist_ok=True)
    proc = subprocess.Popen([exe], cwd=os.path.dirname(exe))
    try:
        hwnd = find_window(proc.pid)
        if not hwnd:
            sys.exit("window not found")
        time.sleep(1.5)
        send_key(hwnd, 'h')          # HUD を消す
        send_key(hwnd, 'p')          # 一時停止（以降は N で 1/120 s ずつ進める）

        jobs = [
            # (キー, ファイル名, フレーム数, 1 フレームのステップ数, 撮り始める前のステップ数, 表示時間 ms)
            ('5',  'scene5_magnet_balls.gif',          90, 4, 0, 40),
            ('6',  'scene6_chain_or_ring.gif',         60, 1, 1, 50),   # 1 ステップ進めて接触を検出してから
            ('7',  'scene7_iron_sand.gif',             60, 1, 0, 50),
            ('m',  'scene7_iron_sand_ns.gif',          60, 1, 0, 50),   # 配置を N-S に切り替え
            ('mm', 'scene7_sand_uniform_field.gif',    60, 2, 0, 50),   # 磁石なし、水平な一様磁場だけ
            ('e',  'scene7_sand_vertical_field.gif',   60, 2, 0, 50),   # 続けて磁場を垂直に
            ('8',  'scene8_cube_magnets.gif',          90, 4, 0, 40),
            # シーン D: 3 段にして Enter で転がし、1 段目に当たる少し前から撮る（約 x0.1 のスロー）
            (['=', 'left', 'left', 'enter'], 'sceneD_gauss_accelerator.gif', 80, 4, 150, 40),
            # シーン E: 引き合う 2 個を放すと、くっつくまで磁力線がついていく
            (['bksp', 'm', 'k'], 'sceneE_field_lines.gif', 90, 2, 0, 40),
        ]
        for keys, name, n, spf, pre, delay in jobs:
            if not any(o in name for o in only):
                continue
            for key in keys:
                send_key(hwnd, key)
            for _ in range(pre):
                send_key(hwnd, 'n')
            time.sleep(0.3)
            frames = record(hwnd, n, spf)
            save_gif(frames, os.path.join(outdir, name), width=800, delay_ms=delay,
                     hold_first_ms=1000, hold_last_ms=2500)

        # 実時間で動くシーン（キー 9, 0, -）は一時停止を解いて撮る
        send_key(hwnd, 'p')
        realtime_jobs = [
            # (キー, ファイル名, 秒数, 押し続けるキー [(キー, 押す秒, 離す秒)])
            (['9'],      'scene9_magnetic_rollers.gif',  8.0, []),   # 1 個ずつ転がり、くっついて鎖になる
            (['m'],      'scene9_rolling_chains.gif',    6.0, []),   # 長さの違う鎖
            (['m'],      'scene9_iron_sand_swarm.gif',   6.0, []),   # 砂鉄の鎖が回りながら進む
            (['0', 'f'], 'scene0_levitating_tower.gif',  6.0, []),   # 縮めて放した塔が伸びて浮く（x0.1 のスロー）
            (['-'],      'scene_compass_array.gif',      6.0,        # 探り磁石を四角く動かす
             [('left', 0.3, 1.7), ('up', 1.9, 3.3), ('right', 3.5, 4.9)]),
            # 電気のシーン（electric_scenes.cpp。M で並べ方を切り替える）
            (['f5'],     'sceneF5_electric_bell.gif',    6.0, []),   # 電極の間で跳ね回る導体の小球
            (['m'],      'sceneF5_charge_sharing.gif',   6.0, []),   # 帯電した金属球が触れて電荷を分ける
            (['f6'],     'sceneF6_helix.gif',            8.0, []),   # 一様な磁場の中のらせん
            (['m'],      'sceneF6_magnetic_bottle.gif',  8.0, []),   # 2 つの磁石の間で跳ね返る
            (['m'],      'sceneF6_exb_drift.gif',        9.0, []),   # E x B ドリフト（上から）
            (['f7'],     'sceneF7_copper_tube.gif',      9.0, []),   # 銅の管とプラスチックの管
            (['m'],      'sceneF7_arago_disc.gif',       6.0, []),   # アラゴの円板
            (['m'],      'sceneF7_eddy_ramp.gif',        5.0, []),   # 磁石の上で遅くなる銅の板
            (['f8'],     'sceneF8_field_lines.gif',      8.0,        # 電気力線と等電位線（2 s ごとに並べ方を変える）
             [('m', 2.0, 2.1), ('m', 4.0, 4.1), ('m', 6.0, 6.1)]),
            # 電流のシーン（current_scenes.cpp。X で電流、V で向き、M で並べ方）
            (['f9'],     'sceneF9_oersted.gif',          7.0,        # 電流を入れると方位磁針が回る。4 s で向きを逆に
             [('x', 1.0, 1.1), ('v', 4.0, 4.1)]),
            (['m'],      'sceneF9_ring_tower.gif',       8.0,        # 電流の輪が浮く。5 s で切ると落ちる
             [('x', 5.0, 5.1)]),
            (['f10'],    'sceneF10_electromagnet.gif',   8.0,        # 電磁石が鉄球と釘を持ち上げ、切ると落とす
             [('x', 1.0, 1.1), ('x', 5.5, 5.6)]),
            (['m'],      'sceneF10_coil_gun.gif',        6.0,        # コイルガン（Enter で発射）
             [('enter', 0.6, 0.7)]),
            (['f11'],    'sceneF11_helmholtz.gif',      12.0,        # ヘルムホルツ → 磁気鏡 → カスプ
             [('m', 4.0, 4.1), ('m', 8.0, 8.1)]),
        ]
        for keys, name, seconds, hold in realtime_jobs:
            for key in keys:
                send_key(hwnd, key, hold=0.15)
            if not any(o in name for o in only):
                time.sleep(0.5)
                continue
            frames, interval = record_realtime(hwnd, seconds, holds=hold)
            save_gif(frames, os.path.join(outdir, name), width=800, delay_ms=interval,
                     hold_first_ms=interval, hold_last_ms=2000)
    finally:
        proc.terminate()


if __name__ == "__main__":
    main()
