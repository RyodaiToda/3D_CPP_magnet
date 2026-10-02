#pragma once
// ---------------------------------------------------------------------------
// levitation_theory.h : 縦に並んだ磁石（同軸の点双極子）の塔のつり合い
//
//   k 番目の磁石（下から、モーメントの y 成分 mu_k、符号つき）が j 番目から受ける力:
//     F_k = -6 K mu_j mu_k sign(y_k - y_j) / (y_k - y_j)^4
//   （向きが逆なら反発、同じなら引力）。一番下は床の上 y_0 に置き、残りの
//   y_1 .. y_{N-1} を「全ペアの磁力 = 重さ」になるようニュートン法で解く。
//   塔のシーン（fun_scenes.cpp）と magnet_test の L1 で使う。
// ---------------------------------------------------------------------------
#include <cmath>
#include <vector>

namespace demo {

inline std::vector<float> dipoleStackEquilibrium(const std::vector<float>& mu, float mass, float g, float y0) {
    const int N = (int)mu.size();
    std::vector<double> y(N);
    if (N == 0) return {};
    y[0] = y0;
    // 初期値: 隣どうしの反発だけで、上に載っている k 個の重さを支える間隔
    for (int k = 1; k < N; ++k) {
        double load = (N - k) * mass * g;
        double gap  = std::pow(6.0 * std::fabs(mu[k] * mu[k - 1]) / load, 0.25);
        y[k] = y[k - 1] + gap;
    }

    auto residual = [&](const std::vector<double>& yy, std::vector<double>& r) {
        r.assign(N, 0.0);
        for (int k = 1; k < N; ++k) {
            double F = -mass * g;
            for (int j = 0; j < N; ++j) {
                if (j == k) continue;
                double d = yy[k] - yy[j];
                double s = (d > 0) ? 1.0 : -1.0;
                F += -6.0 * mu[j] * mu[k] * s / (d * d * d * d);
            }
            r[k] = F;
        }
    };

    std::vector<double> r, r2, dy(N);
    for (int it = 0; it < 100; ++it) {
        residual(y, r);
        double norm = 0;
        for (int k = 1; k < N; ++k) norm = std::max(norm, std::fabs(r[k]));
        if (norm < 1e-9 * mass * g) break;

        // 数値ヤコビアン（N は 10 程度なので密行列で十分）
        const int M = N - 1;
        std::vector<double> J(M * M), b(M);
        for (int c = 0; c < M; ++c) {
            std::vector<double> yp = y;
            double h = 1e-6 * std::max(1.0, std::fabs(y[c + 1]));
            yp[c + 1] += h;
            residual(yp, r2);
            for (int rr = 0; rr < M; ++rr) J[rr * M + c] = (r2[rr + 1] - r[rr + 1]) / h;
        }
        for (int rr = 0; rr < M; ++rr) b[rr] = -r[rr + 1];
        // ガウスの消去法（部分ピボット）
        for (int c = 0; c < M; ++c) {
            int p = c;
            for (int rr = c + 1; rr < M; ++rr) if (std::fabs(J[rr * M + c]) > std::fabs(J[p * M + c])) p = rr;
            for (int k = 0; k < M; ++k) std::swap(J[c * M + k], J[p * M + k]);
            std::swap(b[c], b[p]);
            for (int rr = c + 1; rr < M; ++rr) {
                double f = J[rr * M + c] / J[c * M + c];
                for (int k = c; k < M; ++k) J[rr * M + k] -= f * J[c * M + k];
                b[rr] -= f * b[c];
            }
        }
        for (int c = M - 1; c >= 0; --c) {
            double s = b[c];
            for (int k = c + 1; k < M; ++k) s -= J[c * M + k] * dy[k + 1];
            dy[c + 1] = s / J[c * M + c];
        }
        // 順番が入れ替わらないように歩幅を縮める
        double step = 1.0;
        for (;;) {
            bool ok = true;
            for (int k = 1; k < N; ++k)
                if (y[k] + step * dy[k] <= y[k - 1] + step * (k > 1 ? dy[k - 1] : 0.0) + 1e-3) ok = false;
            if (ok || step < 1e-4) break;
            step *= 0.5;
        }
        for (int k = 1; k < N; ++k) y[k] += step * dy[k];
    }
    std::vector<float> out(N);
    for (int k = 0; k < N; ++k) out[k] = (float)y[k];
    return out;
}

} // namespace demo
