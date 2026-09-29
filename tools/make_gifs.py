"""
デモの GIF（media/*.gif）を作り直すスクリプト（Windows 専用。依存: numpy, Pillow）

    python tools/make_gifs.py build/Release/demo.exe media

demo.exe を起動し、H で HUD を消して P で一時停止したあと、N キー（1/120 s 進める）を
送りながらウィンドウを 1 フレームずつ撮る。キーは PostMessage で送るのでフォーカスは奪わない。
撮影中はデモのウィンドウが画面に出る。
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


def send_key(hwnd, ch, hold=0.03, gap=0.03):
    vk = ord(ch.upper())
    sc = user32.MapVirtualKeyW(vk, 0)
    user32.PostMessageW(hwnd, 0x100, vk, 1 | (sc << 16))                               # WM_KEYDOWN
    time.sleep(hold)
    user32.PostMessageW(hwnd, 0x101, vk, 1 | (sc << 16) | (1 << 30) | (1 << 31))       # WM_KEYUP
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
            ('5', 'scene5_magnet_balls.gif',   90, 4, 0, 40),
            ('6', 'scene6_chain_or_ring.gif',  60, 1, 1, 50),   # 1 ステップ進めて接触を検出してから
            ('7', 'scene7_iron_sand.gif',      60, 1, 0, 50),
            ('m', 'scene7_iron_sand_ns.gif',   60, 1, 0, 50),   # 配置を N-S に切り替え
        ]
        for key, name, n, spf, pre, delay in jobs:
            send_key(hwnd, key)
            for _ in range(pre):
                send_key(hwnd, 'n')
            time.sleep(0.3)
            frames = record(hwnd, n, spf)
            save_gif(frames, os.path.join(outdir, name), width=800, delay_ms=delay,
                     hold_first_ms=1000, hold_last_ms=2500)
    finally:
        proc.terminate()


if __name__ == "__main__":
    main()
