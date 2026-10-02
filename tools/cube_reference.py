"""
一様に磁化した立方体どうしの力の参照値（magnet_test の X2）と、箱の点の位置 beta の決め方。

    python tools/cube_reference.py

単位は K = mu0 / 4pi = 1。辺 1、磁化 M = 1（モーメント 1）の立方体を 2 個置き、
2 個目が受ける力を 2 通りで計算する。
  厳密: 表面磁荷モデル。磁化が z 向きなら上面に +M、下面に -M の面電荷がある。
        長方形の面電荷が作る場は閉じた式で書けるので、受ける側の面だけ
        ガウス・ルジャンドル積分する（接している面どうしは 1e-7 だけ離して極限をとる）
  点:   各立方体を 2x2x2 の点の双極子にする。点はローカルで ±beta（辺に対する割合）
beta = 0.25 がセルの中心。beta = 0.2 で接触したときの引力が厳密値に 1% で一致する。
"""
import numpy as np


def gauss(n, a, b):
    x, w = np.polynomial.legendre.leggauss(n)
    return 0.5 * (b - a) * x + 0.5 * (a + b), 0.5 * (b - a) * w


def rect_field(X, Y, Z, x1, x2, y1, y2, s):
    """平面 z = 0 の長方形 [x1,x2]x[y1,y2]（面電荷密度 s）が点 (X, Y, Z) に作る場"""
    def lx(u):   # int dy / R を x 方向の端で評価したもの
        return (np.log((y2 - Y) + np.sqrt(u * u + (y2 - Y) ** 2 + Z * Z))
                - np.log((y1 - Y) + np.sqrt(u * u + (y1 - Y) ** 2 + Z * Z)))

    def ly(v):
        return (np.log((x2 - X) + np.sqrt(v * v + (x2 - X) ** 2 + Z * Z))
                - np.log((x1 - X) + np.sqrt(v * v + (x1 - X) ** 2 + Z * Z)))

    Bx = s * (lx(X - x2) - lx(X - x1))
    By = s * (ly(Y - y2) - ly(Y - y1))
    Bz = 0.0
    for xi, sx in ((x1, -1), (x2, 1)):
        for yj, sy in ((y1, -1), (y2, 1)):
            u, v = xi - X, yj - Y
            Bz = Bz + sx * sy * np.arctan2(u * v, Z * np.sqrt(u * u + v * v + Z * Z))
    return Bx, By, s * Bz


def exact_force(p, sign=1.0, n=200, eps=1e-7):
    """原点の立方体（磁化 +z）が、中心 p・磁化 sign*z の立方体に及ぼす力"""
    xs, wx = gauss(n, p[0] - 0.5, p[0] + 0.5)
    ys, wy = gauss(n, p[1] - 0.5, p[1] + 0.5)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    W = np.outer(wx, wy)
    F = np.zeros(3)
    for z2, s2 in ((p[2] - 0.5, -sign), (p[2] + 0.5, sign)):
        for z1, s1 in ((-0.5, -1.0), (0.5, 1.0)):
            Z = z2 - z1
            if abs(Z) < 1e-12:
                Z = eps
            B = rect_field(X, Y, Z, -0.5, 0.5, -0.5, 0.5, s1)
            F += s2 * np.array([np.sum(W * b) for b in B])
    return F


def dipole_force(r, m1, m2):
    d = np.linalg.norm(r)
    n = r / d
    a, b, c = m1 @ n, m2 @ n, m1 @ m2
    return 3 / d ** 4 * (m2 * a + m1 * b + n * (c - 5 * a * b))


def point_force(p, sign=1.0, beta=0.2):
    c = np.array([-beta, beta])
    pts = np.array([(x, y, z) for x in c for y in c for z in c])
    m1, m2 = np.array([0, 0, 1 / 8]), np.array([0, 0, sign / 8])
    F = np.zeros(3)
    for a in pts:
        for b in pts + np.asarray(p, dtype=float):
            F += dipole_force(b - a, m1, m2)
    return F


CASES = {
    "coaxial touch": ((0, 0, 1.0), 1.0),
    "side parallel": ((1.0, 0, 0), 1.0),
    "side antiparallel": ((1.0, 0, 0), -1.0),
    "slide 1/4": ((0.25, 0, 1.0), 1.0),
    "slide 1/2": ((0.5, 0, 1.0), 1.0),
    "coaxial gap 1/4": ((0, 0, 1.25), 1.0),
    "diagonal edge": ((1.0, 0, 1.0), 1.0),
}

if __name__ == "__main__":
    print(f"{'case':20s} {'exact (Fx, Fz)':>20s}  " + "  ".join(f"beta={b:<4}" for b in (0.2, 0.25)) + "  center")
    for name, (p, s) in CASES.items():
        e = exact_force(p, s)
        errs = []
        for beta in (0.2, 0.25):
            f = point_force(p, s, beta)
            errs.append(np.linalg.norm(f - e) / np.linalg.norm(e))
        center = dipole_force(np.asarray(p, dtype=float), np.array([0, 0, 1.0]), np.array([0, 0, s]))
        ec = np.linalg.norm(center - e) / np.linalg.norm(e)
        print(f"{name:20s} ({e[0]:+8.4f}, {e[2]:+8.4f})  " + "  ".join(f"{x:9.1%}" for x in errs) + f"  {ec:6.1%}")
