"""
MAGNET_SPEC.md の「付録 A 参照値」を再生成するスクリプト。
依存: numpy, scipy

    python tools/reference_values.py

単位系は仕様書 §3.2 のとおり K = mu0/(4 pi) = 1、接触距離 d = 1。
"""
import numpy as np
from scipy.integrate import quad

MU0 = 4e-7 * np.pi
G = 9.81


# ---------------------------------------------------------------------------
# 1. 双極子の場・力・トルク・エネルギー（K = 1）
# ---------------------------------------------------------------------------
def field(r, m):
    R = np.linalg.norm(r)
    n = r / R
    return (3 * np.dot(m, n) * n - m) / R**3


def energy_pair(r, m1, m2):
    return -np.dot(m2, field(r, m1))


def force_on_2(r, m1, m2):
    R = np.linalg.norm(r)
    n = r / R
    return 3 / R**4 * (np.dot(m1, n) * m2 + np.dot(m2, n) * m1
                       + np.dot(m1, m2) * n - 5 * np.dot(m1, n) * np.dot(m2, n) * n)


def self_check():
    rng = np.random.default_rng(1)
    m1, m2 = rng.normal(size=3), rng.normal(size=3)
    r = rng.normal(size=3) + np.array([2.0, 0, 0])
    h = 1e-6
    fd = np.array([-(energy_pair(r + h * e, m1, m2) - energy_pair(r - h * e, m1, m2)) / (2 * h)
                   for e in np.eye(3)])
    F2 = force_on_2(r, m1, m2)
    t1 = np.cross(m1, field(-r, m2))
    t2 = np.cross(m2, field(r, m1))
    L = t1 + t2 + np.cross(r, F2)          # r1 = 0, F1 = -F2
    print("[check] F vs -grad U  max diff =", np.max(np.abs(F2 - fd)))
    print("[check] dL/dt (spin + orbit)     =", np.max(np.abs(L)))


# ---------------------------------------------------------------------------
# 2. 鎖と輪のエネルギー（単位 u0 = K m^2 / d^3）
# ---------------------------------------------------------------------------
def total_energy(pos, mom):
    E = 0.0
    for i in range(len(pos)):
        for j in range(i + 1, len(pos)):
            E += energy_pair(pos[j] - pos[i], mom[i], mom[j])
    return E


def chain_ring_table(nmax=12):
    print("\n N    chain      ring    ring-chain")
    for N in range(2, nmax + 1):
        pc = np.array([[i, 0, 0] for i in range(N)], float)
        mc = np.array([[1, 0, 0]] * N, float)
        Ec = total_energy(pc, mc)
        if N >= 3:
            Rr = 0.5 / np.sin(np.pi / N)
            th = 2 * np.pi * np.arange(N) / N
            pr = np.stack([Rr * np.cos(th), Rr * np.sin(th), 0 * th], 1)
            mr = np.stack([-np.sin(th), np.cos(th), 0 * th], 1)
            Er = total_energy(pr, mr)
            print(f"{N:2d} {Ec:9.4f} {Er:9.4f} {Er - Ec:9.4f}")
        else:
            print(f"{N:2d} {Ec:9.4f}         -         -")


# ---------------------------------------------------------------------------
# 3. 磁石球の強さ Γ と相似則
# ---------------------------------------------------------------------------
def gamma_sphere(Br, rho, R):
    """接触して一直線に並んだ 2 球の引力 / 1 球の重さ"""
    return Br**2 / (8 * MU0 * rho * G * R)


# ---------------------------------------------------------------------------
# 4. 導体パイプの渦電流ブレーキ（準静的・軸上・点双極子モデル）
# ---------------------------------------------------------------------------
def pipe_k_thick(sigma, m, a_in, a_out):
    """無限に長い厚肉パイプの減衰係数 k [N s/m]  (F = -k v)"""
    return 45 * sigma * MU0**2 * m**2 / 1024 * (a_in**-3 - a_out**-3) / 3


def end_profile(zeta):
    """長いパイプの上端から深さ zeta*a の位置にいるときの k / k_inf（薄肉）"""
    f = lambda z: z**2 / (1 + z**2) ** 5
    tot = quad(f, -np.inf, np.inf)[0]
    return quad(f, -np.inf, zeta)[0] / tot


def pipe_table():
    Br, rho, R = 1.3, 7500.0, 6.35e-3      # 直径 12.7 mm, N42 相当
    V = 4 / 3 * np.pi * R**3
    m = Br / MU0 * V
    mass = rho * V
    print(f"\nmagnet: m = {m:.4f} A m^2, mass = {mass * 1e3:.2f} g")
    for name, sigma in [("Cu", 5.8e7), ("Al", 3.5e7)]:
        k = pipe_k_thick(sigma, m, 6.5e-3, 7.5e-3)
        print(f"{name}: k = {k:.3f} N s/m, gamma = {k / mass:.1f} 1/s, "
              f"v_t = {mass * G / k * 100:.2f} cm/s")
    print("\n depth/a   k/k_inf")
    for z in [-2, -1, -0.5, 0, 0.5, 1, 2, 3]:
        print(f"{z:7.1f}   {end_profile(z):.4f}")


if __name__ == "__main__":
    self_check()
    chain_ring_table()
    R = 2.5e-3
    Gm = gamma_sphere(1.2, 7500.0, R)
    print(f"\nNeocube (d = 5 mm, N35): Gamma = {Gm:.0f}, "
          f"t_c(real) = {1e3 * np.sqrt(R / (Gm * G)):.2f} ms")
    lam = 0.25 / R
    print(f"  R_sim = 0.25 -> lambda = {lam:.0f}, time scale = {np.sqrt(lam):.1f}x, "
          f"t_c(sim) = {1e3 * np.sqrt(0.25 / (Gm * G)):.2f} ms")
    pipe_table()
