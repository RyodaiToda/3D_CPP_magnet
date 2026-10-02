"""
MAGNET FLOW の GIF（media/flow_*.gif）を作るスクリプト（Windows 専用。依存: numpy, Pillow）

    python tools/make_flow_gifs.py build/Release/magnet_flow.exe media [grand|lab]
        grand: GRAND TOUR だけ、lab: 新しいギミックのステージ L1〜L4 だけ

magnet_flow.exe を起動し、決めた時刻に Space（開始・極性切替）を送りながら、
実時間で約 15 fps で撮る。時刻は flow_test のボットで成功を確かめた値。

01・02・04（magnetic_flow_gimmick_stage_spec.md）は、切替が多く実時間ではずれやすいので、
flow_test --record でボットの入力を記録のファイルに書き、ゲームの P（最高記録を流し直す）で撮る。
GRAND TOUR（全部のギミックを入れた大きなマップ）は flow_test --grandrun でボットの通しの走りを記録にし、
リプレイを 1 回流しながら撮って、区間ごとの GIF と、全体の早回しの GIF を作る。
L1〜L4（サイクロトロン、加速リング、共振ブランコ、鉄球のロープ）は flow_test --labrecord で、状態を見て押す
ボットの走りを記録にして、リプレイで撮る。
遊んだ人の記録のファイルは、撮る前に退避して、撮り終えたら戻す。
"""
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_gifs import capture, find_window, save_gif, send_key, key_down, key_up  # noqa: E402

# (ステージのキー, ファイル名, 切り替える時刻 [s]（最初は開始）, 撮る秒数)
JOBS = [
    ('2', 'flow_P1_switch.gif', [0.0, 1.30], 3.6),
    ('3', 'flow_P2_timing.gif', [0.0, 1.58], 3.4),
    ('4', 'flow_P3_chain.gif',  [0.0, 1.00, 5.00], 7.2),
]

# 流し直して撮る: (ステージの番号, キー, ファイル名, 切り替える時刻（開始を除く）, 撮る秒数, fps, 幅)
REPLAY_JOBS = [
    (4, '5', 'flow_G01_canyon.gif',  [2.0, 2.57], 5.0, 15, 560),
    (5, '6', 'flow_G02_orbit.gif',   [4.74, 7.48, 10.9], 12.6, 10, 480),
    (6, '7', 'flow_G04_factory.gif', [1.0, 4.0, 4.55], 5.6, 15, 560),
]
RECORDS = 'magnet_flow_records.txt'

# GRAND TOUR: リプレイの時刻（区間に着いた時刻は flow_test --grandrun が表示する）で切り出す
GRAND_KEY = '8'
GRAND_SECONDS = 82.0
GRAND_FPS = 12
GRAND_CLIPS = [   # (ファイル名, 始め s, 終わり s)
    ('flow_grand_2_pendulum.gif', 4.5, 12.2),
    ('flow_grand_4_lens.gif', 25.0, 29.5),
    ('flow_grand_6_steel.gif', 38.3, 50.0),
    ('flow_grand_7_storm.gif', 49.7, 57.6),
    ('flow_grand_8_compass.gif', 57.2, 66.6),
    ('flow_grand_9_core.gif', 66.3, 81.0),
]
GRAND_TOUR = ('flow_grand_tour.gif', 3, 400)   # (ファイル名, 何倍速, 幅)

# 新しいギミックのステージ: (キー, ファイル名, 撮る秒数, fps, 幅)
LAB_JOBS = [
    ('9', 'flow_L1_cyclotron.gif', 14.6, 9, 440),
    ('0', 'flow_L2_gauss_rings.gif', 5.4, 15, 520),
    ('-', 'flow_L3_resonance.gif', 10.6, 12, 480),
    ('=', 'flow_L4_ball_chain.gif', 6.4, 15, 520),
]


def play_and_record(hwnd, presses, seconds, fps=15):
    frames, t0, k = [], time.time(), 0
    while time.time() - t0 < seconds:
        t = time.time()
        while k < len(presses) and t - t0 >= presses[k]:
            key_down(hwnd, ' ')   # VK_SPACE = ord(' ')。同じフレームで離すと押したと見なされないので少し待つ
            time.sleep(0.035)
            key_up(hwnd, ' ')
            k += 1
        frames.append(capture(hwnd))
        time.sleep(max(0.0, 1.0 / fps - (time.time() - t)))
    return frames, int(1000 * (time.time() - t0) / len(frames))


def record_frames(hwnd, seconds, fps=15):
    frames, t0 = [], time.time()
    while time.time() - t0 < seconds:
        t = time.time()
        frames.append(capture(hwnd))
        time.sleep(max(0.0, 1.0 / fps - (time.time() - t)))
    return frames, int(1000 * (time.time() - t0) / len(frames))


def record_grand(hwnd, outdir):
    """GRAND TOUR のリプレイを撮り、区間ごとの GIF と全体の早回しの GIF を作る"""
    from PIL import Image
    send_key(hwnd, GRAND_KEY, hold=0.1)
    time.sleep(1.0)
    send_key(hwnd, 'p', hold=0.05)
    frames, stamps, t0 = [], [], time.time()
    while time.time() - t0 < GRAND_SECONDS:
        t = time.time()
        img = capture(hwnd)
        frames.append(img.resize((560, round(img.height * 560 / img.width)), Image.LANCZOS))
        stamps.append(t - t0)
        time.sleep(max(0.0, 1.0 / GRAND_FPS - (time.time() - t)))
    interval = int(1000 * stamps[-1] / len(frames))
    for name, a, b in GRAND_CLIPS:
        clip = [f for f, s in zip(frames, stamps) if a <= s <= b]
        save_gif(clip, os.path.join(outdir, name), width=480, delay_ms=interval, colors=48,
                 hold_first_ms=600, hold_last_ms=1200)
    name, speed, width = GRAND_TOUR
    save_gif(frames[::speed], os.path.join(outdir, name), width=width, delay_ms=interval, colors=40,
             hold_first_ms=1000, hold_last_ms=2500)


def record_lab(hwnd, outdir):
    """L1〜L4 のボットの走りをリプレイで撮る"""
    for stage_key, name, seconds, fps, width in LAB_JOBS:
        send_key(hwnd, stage_key, hold=0.1)
        time.sleep(1.0)
        send_key(hwnd, 'p', hold=0.05)
        frames, interval = record_frames(hwnd, seconds, fps)
        save_gif(frames, os.path.join(outdir, name), width=width, delay_ms=interval, colors=48,
                 hold_first_ms=800, hold_last_ms=1500)


def main():
    exe, outdir = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    grand_only = len(sys.argv) > 3 and sys.argv[3] == 'grand'   # GRAND TOUR だけ撮る
    lab_only = len(sys.argv) > 3 and sys.argv[3] == 'lab'       # L1〜L4 だけ撮る
    os.makedirs(outdir, exist_ok=True)
    exedir = os.path.dirname(exe)
    records, backup = os.path.join(exedir, RECORDS), os.path.join(exedir, RECORDS + '.bak')
    had_records = os.path.exists(records)
    if had_records:
        shutil.move(records, backup)
    for stage, _, _, presses, _, _, _ in REPLAY_JOBS:   # ボットの入力を記録にする
        subprocess.run([os.path.join(exedir, 'flow_test.exe'), '--record', records, str(stage)] +
                       [str(t) for t in presses], check=True)
    subprocess.run([os.path.join(exedir, 'flow_test.exe'), '--grandrun', records], check=True)
    subprocess.run([os.path.join(exedir, 'flow_test.exe'), '--labrecord', records], check=True)
    proc = subprocess.Popen([exe], cwd=exedir)
    try:
        hwnd = find_window(proc.pid)
        if not hwnd:
            sys.exit("window not found")
        time.sleep(2.0)
        send_key(hwnd, 'h')   # HUD を消す（N/S の表示は残る）
        send_key(hwnd, 'g')   # ゴーストを消す（流し直す走りと重なる）
        if grand_only:
            record_grand(hwnd, outdir)
            return
        if lab_only:
            record_lab(hwnd, outdir)
            return
        for stage_key, name, presses, seconds in JOBS:
            send_key(hwnd, stage_key, hold=0.1)
            time.sleep(0.8)
            frames, interval = play_and_record(hwnd, presses, seconds)
            save_gif(frames, os.path.join(outdir, name), width=560, delay_ms=interval, colors=48,
                     hold_first_ms=800, hold_last_ms=1500)
        for _, stage_key, name, _, seconds, fps, width in REPLAY_JOBS:
            send_key(hwnd, stage_key, hold=0.1)
            time.sleep(0.8)
            send_key(hwnd, 'p', hold=0.05)
            frames, interval = record_frames(hwnd, seconds, fps)
            save_gif(frames, os.path.join(outdir, name), width=width, delay_ms=interval, colors=48,
                     hold_first_ms=800, hold_last_ms=1500)
        record_grand(hwnd, outdir)
        record_lab(hwnd, outdir)
    finally:
        proc.terminate()
        time.sleep(0.5)
        if os.path.exists(records):
            os.remove(records)
        if had_records:
            shutil.move(backup, records)


if __name__ == "__main__":
    main()
