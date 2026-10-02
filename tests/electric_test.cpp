// 電気の力の検証（ElectricSystem。描画なし）
//   E1  クーロン力（作用・反作用、-dU/dr と一致）
//   E2  導体の球の誘導（遠くでは 2 R^3 q^2 / d^5）
//   E3  接触で電荷を分け合う（容量の比、保存、絶縁体、コンデンサの板）
//   E4  電気ベル（平行板の間を往復し、触れるたびに電荷の符号が変わる）
//   E5  一様な磁場の中の円運動（速さが変わらない、周期と半径）
//   E6  E x B ドリフト
//   E7  磁気瓶（2 つの磁石の間で跳ね返る、磁気モーメントがほぼ保たれる）
//   E8  銅の管を落ちる磁石（終端速度が理論式と合う、sigma に反比例、エネルギーが減り続ける）
//   E9  渦電流の運動量（自由な磁石と自由な銅の板）
//   E10 アラゴの円板（磁石を回すと、銅の板が同じ向きに回り出す）
//   E11 回る銅の板のブレーキ（磁石があると速く止まる）
#include "phys/world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace phys;

static int failures = 0;
static void check(const char* name, bool ok, const char* detail = "") {
    std::printf("[%s] %-52s %s\n", ok ? " OK " : "FAIL", name, detail);
    if (!ok) ++failures;
}

constexpr float G_ACC = 9.81f;
constexpr float DT    = 1.0f / 120.0f;

static void clearForces(World& w) {
    for (const auto& b : w.getBodies()) { b->force = Vec3{0, 0, 0}; b->torque = Vec3{0, 0, 0}; }
}

static RigidBody* chargedBall(World& w, const Vec3& pos, float r, float m, float q, bool conductor) {
    RigidBody* b = w.createChargedSphere(pos, r, m, q, conductor);
    b->linearDamping = 0.0f;
    b->angularDamping = 0.0f;
    return b;
}

// 銅の管: 中心 c、軸 +y、壁の中心の半径 a、厚さ t、高さ 2 hh、周りの分割 n
static void copperTube(World& w, const Vec3& c, float a, float t, float hh, int n, float sigma, float spacing) {
    const float half = a * std::tan(PHYS_PI / n);   // 正多角形の辺（重ならないように。周の長さは円の 1.003 倍）
    for (int i = 0; i < n; ++i) {
        const float phi = 2.0f * PHYS_PI * (float)i / n;
        const Vec3  pos = c + Vec3{a * std::cos(phi), 0, a * std::sin(phi)};
        // 箱のローカル x が半径の向き（厚さ）、z が周りの向き
        w.createConductorBox(pos, {0.5f * t, hh, half}, 0.0f, sigma, Quat::fromAxisAngle({0, 1, 0}, -phi), spacing);
    }
}

int main() {
    char buf[512];

    // ---------------- E1. クーロン力 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        RigidBody* a = chargedBall(w, {0, 0, 0}, 0.2f, 1.0f, 1.5f, false);
        RigidBody* b = chargedBall(w, {1.2f, 1.6f, 0}, 0.2f, 1.0f, -2.0f, false);
        w.electric().applyForces(w.magnets(), DT);
        const Vec3  r = b->position - a->position;
        const float d = length(r);
        const Vec3  want = r * (1.5f * -2.0f / (d * d * d));
        const float err = length(b->force - want) / length(want);
        const float sum = length(a->force + b->force);
        // -dU/dr（b を r の向きに動かして数値微分）
        const Vec3 n = r / d, p0 = b->position;
        const float eps = 1e-3f;
        b->position = p0 + n * eps;
        const float Up = w.electric().potentialEnergy();
        b->position = p0 - n * eps;
        const float Um = w.electric().potentialEnergy();
        b->position = p0;
        const float dUdr = (Up - Um) / (2.0f * eps);
        const float errU = std::fabs(-dUdr - dot(want, n)) / length(want);
        std::snprintf(buf, sizeof buf, "|F - q1 q2 / r^2| / |F| %.1e, |F_a + F_b| %.1e, |-dU/dr - F| / |F| %.1e", err, sum, errU);
        check("E1 Coulomb force, action-reaction, -dU/dr", err < 1e-5f && sum < 1e-6f && errU < 2e-3f, buf);
    }

    // ---------------- E2. 導体の球の誘導 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float q = 2.0f, R = 0.3f, d = 4.0f;
        chargedBall(w, {0, 0, 0}, 0.2f, 0.0f, q, false);          // 静的な点電荷
        RigidBody* c = chargedBall(w, {d, 0, 0}, R, 1.0f, 0.0f, true);   // 帯電していない導体
        w.electric().updateDipoles(20);
        w.electric().applyForces(w.magnets(), DT);
        const float want = -2.0f * R * R * R * q * q / std::pow(d, 5.0f);   // 引かれる（-x）
        const float err  = std::fabs(c->force.x - want) / std::fabs(want);
        // 近づけるほど強く引かれる
        World w2;
        w2.gravity = Vec3{0, 0, 0};
        chargedBall(w2, {0, 0, 0}, 0.2f, 0.0f, q, false);
        RigidBody* c2 = chargedBall(w2, {1.0f, 0, 0}, R, 1.0f, 0.0f, true);
        w2.electric().updateDipoles(20);
        w2.electric().applyForces(w2.magnets(), DT);
        std::snprintf(buf, sizeof buf, "F at d = 4: %.4e (theory %.4e, err %.2f%%); at d = 1: %.3e", c->force.x, want, 100.0f * err,
                      c2->force.x);
        check("E2 neutral conductor is attracted (2 R^3 q^2 / d^5)", err < 0.01f && c2->force.x < 50.0f * want, buf);
    }

    // ---------------- E3. 接触で電荷を分け合う ----------------
    {
        auto touch = [](float r1, float q1, bool c1, float r2, float q2, bool c2, float& o1, float& o2) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            RigidBody* a = chargedBall(w, {0, 0, 0}, r1, 1.0f, q1, c1);
            RigidBody* b = chargedBall(w, {r1 + r2 - 0.001f, 0, 0}, r2, 1.0f, q2, c2);
            w.step(DT);
            o1 = w.electric().bodies()[w.electric().indexOf(a)].charge;
            o2 = w.electric().bodies()[w.electric().indexOf(b)].charge;
        };
        float a1, b1, a2, b2, a3, b3;
        touch(0.2f, 3.0f, true, 0.2f, 0.0f, true, a1, b1);    // 同じ大きさ: 半分ずつ
        touch(0.2f, 3.0f, true, 0.4f, 0.0f, true, a2, b2);    // 半径 1 : 2 → 1 : 2
        touch(0.2f, 3.0f, false, 0.2f, 0.0f, true, a3, b3);   // 絶縁体は分けない
        // 導体の球がコンデンサの板に触れる: R V になる
        World w;
        w.gravity = Vec3{0, -G_ACC, 0};
        w.createCapacitor({0, 1.0f, 0}, Quat{}, 2.0f, 2.0f, 2.0f, 0.2f, -5.0f, 5.0f);   // 下の板 A（y = -0.1..0）が -5 V
        RigidBody* ball = chargedBall(w, {0, 0.199f, 0}, 0.2f, 0.01f, 0.0f, true);
        w.step(DT);
        const float qb = w.electric().bodies()[w.electric().indexOf(ball)].charge;
        const float Q0 = w.electric().totalCharge() - w.electric().plateExchange();
        std::snprintf(buf, sizeof buf, "equal: %.3f %.3f; 1:2 -> %.3f %.3f; insulator %.3f %.3f; plate -5 V: %.3f (R V = %.3f), "
                      "charge - plate supply %.1e", a1, b1, a2, b2, a3, b3, qb, -0.2f * 5.0f, Q0);
        check("E3 charge sharing on contact (by capacitance)", std::fabs(a1 - 1.5f) < 1e-5f && std::fabs(b1 - 1.5f) < 1e-5f &&
                                                               std::fabs(a2 - 1.0f) < 1e-5f && std::fabs(b2 - 2.0f) < 1e-5f &&
                                                               a3 == 3.0f && b3 == 0.0f && std::fabs(qb + 1.0f) < 1e-5f &&
                                                               std::fabs(Q0) < 1e-5f, buf);
    }

    // ---------------- E4. 電気ベル ----------------
    {
        World w;
        w.substeps = 8;
        const float gap = 1.0f, R = 0.1f, m = 0.01f;
        // 下の板 A が -V、上の板 B が +V。下で R V の負の電荷をもらうと、上向きに重さの 3 倍の力を受ける
        const float V = std::sqrt(3.0f * m * G_ACC * gap / (2.0f * R));
        w.createCapacitor({0, 0.5f * gap, 0}, Quat{}, 1.0f, 1.0f, gap, 0.1f, -V, V);
        RigidBody* ball = chargedBall(w, {0, R + 0.001f, 0}, R, m, 0.0f, true);
        ball->restitution = 0.3f;
        const int bi = w.electric().indexOf(ball);
        int flips = 0, topHits = 0;
        float prevQ = 0.0f, maxY = 0.0f;
        for (int i = 0; i < 600; ++i) {
            w.step(DT);
            const float q = w.electric().bodies()[bi].charge;
            if (q * prevQ < 0.0f) ++flips;
            if (q > 0.0f && prevQ <= 0.0f) ++topHits;
            if (q != 0.0f) prevQ = q;
            maxY = std::max(maxY, ball->position.y);
        }
        const float Q0 = w.electric().totalCharge() - w.electric().plateExchange();
        std::snprintf(buf, sizeof buf, "V %.3f; in 5 s: charge sign flipped %d times, reached the top plate %d times, max y %.3f; "
                      "charge - plate supply %.1e", V, flips, topHits, maxY, Q0);
        check("E4 electric bell: shuttles and swaps charge", flips >= 8 && topHits >= 4 && maxY <= gap - R + 0.01f &&
                                                             std::fabs(Q0) < 1e-4f, buf);
    }

    // ---------------- E5. 一様な磁場の中の円運動 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.substeps = 8;
        const float q = 1.0f, m = 1.0f, B = 1.0f, kappa = 2.0f, v0 = 3.0f;
        w.magnets().externalField = Vec3{0, B, 0};
        w.electric().kappa = kappa;
        RigidBody* b = chargedBall(w, {0, 0, 0}, 0.1f, m, q, false);
        b->velocity = Vec3{v0, 0, 0};
        const float omega = kappa * q * B / m, rTheory = m * v0 / (kappa * q * B);
        const Vec3  center{0, 0, rTheory};   // v x B = x x y = +z の向きに曲がる
        float vErr = 0.0f, rMin = 1e9f, rMax = 0.0f, prevAng = 0.0f, turns = 0.0f;
        float firstT = -1.0f, lastT = -1.0f;
        int   laps = 0;
        float t = 0.0f;
        for (int i = 0; i < 2400; ++i) {
            w.step(DT);
            t += DT;
            vErr = std::max(vErr, std::fabs(length(b->velocity) - v0) / v0);
            const Vec3 r = b->position - center;
            rMin = std::min(rMin, length(r));
            rMax = std::max(rMax, length(r));
            const float ang = std::atan2(r.x, -r.z);
            float da = ang - prevAng;
            if (da > PHYS_PI) da -= 2.0f * PHYS_PI;
            if (da < -PHYS_PI) da += 2.0f * PHYS_PI;
            prevAng = ang;
            const float before = turns;
            turns += std::fabs(da) / (2.0f * PHYS_PI);
            if (std::floor(turns) > std::floor(before)) {
                ++laps;
                if (firstT < 0.0f) firstT = t;
                lastT = t;
            }
        }
        const float period = laps > 1 ? (lastT - firstT) / (float)(laps - 1) : 0.0f;
        const float pErr = std::fabs(period - 2.0f * PHYS_PI / omega) / (2.0f * PHYS_PI / omega);
        const float rErr = std::max(std::fabs(rMin - rTheory), std::fabs(rMax - rTheory)) / rTheory;
        // 陽解法（v += (kappa q / m) v x B h）なら、1 刻みで速さの 2 乗が (omega h)^2 ずつ増える
        const float hs = DT / (float)w.substeps;
        const float euler = std::sqrt(std::pow(1.0f + omega * hs * omega * hs, 20.0f / hs)) - 1.0f;
        std::snprintf(buf, sizeof buf, "20 s: speed change %.1e (explicit Euler would give %.1f%%); period %.4f (theory %.4f, %.3f%%); "
                      "radius %.4f..%.4f (theory %.4f)", vErr, 100.0f * euler, period, 2.0f * PHYS_PI / omega, 100.0f * pErr, rMin, rMax,
                      rTheory);
        check("E5 circle in a uniform B: speed kept, period, radius", vErr < 1e-4f && pErr < 0.005f && rErr < 0.005f, buf);
    }

    // ---------------- E6. E x B ドリフト ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.substeps = 8;
        const float q = 1.0f, m = 1.0f, B = 2.0f, kappa = 1.5f, E = 3.0f;
        w.magnets().externalField = Vec3{0, B, 0};
        w.electric().externalField = Vec3{0, 0, E};
        w.electric().kappa = kappa;
        RigidBody* b = chargedBall(w, {0, 0, 0}, 0.1f, m, q, false);
        const float omega = kappa * q * B / m, T = 2.0f * PHYS_PI / omega;
        const Vec3  want = cross(Vec3{0, 0, E}, Vec3{0, B, 0}) / (kappa * B * B);   // E x B / (kappa B^2)
        const int   steps = (int)std::lround(10.0f * T / DT);   // ちょうど 10 周期（から 1 刻み以内）
        for (int i = 0; i < steps; ++i) w.step(DT);
        const Vec3  vAvg = b->position / ((float)steps * DT);
        const float err  = length(vAvg - want) / length(want);
        std::snprintf(buf, sizeof buf, "drift (%.4f %.4f %.4f), theory (%.4f %.4f %.4f), err %.2f%%", vAvg.x, vAvg.y, vAvg.z, want.x,
                      want.y, want.z, 100.0f * err);
        check("E6 E x B drift", err < 0.01f, buf);
    }

    // ---------------- E7. 磁気瓶 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.substeps = 8;
        const float L = 3.0f, m0 = 50.0f;
        w.createMagnet({0, -L, 0}, 0.5f, 0.0f, {0, m0, 0});   // 同じ向きの 2 つの磁石（静的）
        w.createMagnet({0, L, 0}, 0.5f, 0.0f, {0, m0, 0});
        w.magnets().updateMoments(10);
        w.electric().kappa = 3.0f;
        RigidBody* b = chargedBall(w, {0.0f, 0, 0}, 0.05f, 1.0f, 1.0f, false);
        b->velocity = Vec3{2.0f, 1.2f, 0};
        auto mu = [&]() {
            const Vec3  Bv = ElectricSystem::magneticFieldAt(w.magnets(), b->position, b->id);
            const float Bl = length(Bv);
            const Vec3  vperp = b->velocity - Bv * (dot(b->velocity, Bv) / (Bl * Bl));
            return lengthSq(vperp) / (2.0f * Bl);
        };
        const float mu0 = mu();
        float muMin = mu0, muMax = mu0, yMax = 0.0f;
        int   bounces = 0;
        float prevVy = b->velocity.y;
        for (int i = 0; i < 2400; ++i) {
            w.step(DT);
            const float m = mu();
            muMin = std::min(muMin, m);
            muMax = std::max(muMax, m);
            yMax  = std::max(yMax, std::fabs(b->position.y));
            if (prevVy * b->velocity.y < 0.0f && std::fabs(b->position.y) > 0.3f) ++bounces;
            prevVy = b->velocity.y;
        }
        std::snprintf(buf, sizeof buf, "20 s: %d bounces, |y| max %.2f (magnets at %.1f), mu %.3f..%.3f (start %.3f)", bounces, yMax, L,
                      muMin, muMax, mu0);
        check("E7 magnetic bottle: bounces, mu nearly kept", bounces >= 4 && yMax < L - 0.6f && muMax / muMin < 1.2f, buf);
    }

    // ---------------- E8. 銅の管を落ちる磁石 ----------------
    {
        auto drop = [](float sigma, bool* monotone, float* vMid) {
            World w;
            w.substeps = 8;
            const float a = 0.5f, t = 0.08f, rm = 0.3f, mass = 1.0f, m0 = 5.0f;
            copperTube(w, {0, 0, 0}, a, t, 6.0f, 32, sigma, 0.08f);
            RigidBody* mag = w.createMagnet({0, 5.0f, 0}, rm, mass, {0, m0, 0});
            mag->linearDamping = 0.0f;
            float prevE = 1e30f;
            bool  mono = true;
            float vSum = 0.0f;
            int   nv = 0;
            for (int i = 0; i < 6000 && mag->position.y > -5.0f; ++i) {
                w.step(DT);
                const float e = 0.5f * mass * lengthSq(mag->velocity) + mass * G_ACC * mag->position.y;
                if (e > prevE + 1e-4f) mono = false;
                prevE = e;
                if (std::fabs(mag->position.y) < 2.0f) { vSum += -mag->velocity.y; ++nv; }   // 管の中ほど
            }
            if (monotone) *monotone = mono;
            *vMid = nv ? vSum / nv : 0.0f;
            return tubeDragCoefficient(m0, a, t, sigma);
        };
        bool  mono = false;
        float v1 = 0.0f, v2 = 0.0f;
        const float C1 = drop(0.05f, &mono, &v1);
        drop(0.10f, nullptr, &v2);
        const float vTheory = 1.0f * G_ACC / C1;
        const float err = std::fabs(v1 - vTheory) / vTheory;
        std::snprintf(buf, sizeof buf, "terminal speed %.4f (thin-tube theory %.4f, %.1f%%); sigma x2 -> %.4f (ratio %.3f); "
                      "energy falls %s", v1, vTheory, 100.0f * err, v2, v2 / v1, mono ? "yes" : "NO");
        check("E8 magnet in a copper tube: terminal speed", err < 0.15f && std::fabs(v2 / v1 - 0.5f) < 0.025f && mono, buf);
    }

    // ---------------- E9. 渦電流の運動量 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.substeps = 8;
        RigidBody* mag = w.createMagnet({-3.0f, 0.6f, 0}, 0.3f, 1.0f, {0, 5.0f, 0});
        RigidBody* cu  = w.createConductorBox({0, 0, 0}, {1.0f, 0.1f, 1.0f}, 2.0f, 0.5f);
        mag->velocity = Vec3{3.0f, 0, 0};
        mag->linearDamping = cu->linearDamping = 0.0f;
        mag->angularDamping = cu->angularDamping = 0.0f;
        const Vec3 P0 = mag->velocity * 1.0f + cu->velocity * 2.0f;
        float drift = 0.0f;
        for (int i = 0; i < 240; ++i) {
            w.step(DT);
            const Vec3 P = mag->velocity * 1.0f + cu->velocity * 2.0f;
            drift = std::max(drift, length(P - P0));
        }
        std::snprintf(buf, sizeof buf, "after 2 s: magnet vx %.3f, copper vx %.3f, |P - P0| max %.1e (P0 = 3)", mag->velocity.x,
                      cu->velocity.x, drift);
        check("E9 eddy currents conserve momentum", drift < 1e-4f * 3.0f && cu->velocity.x > 0.01f && mag->velocity.x < 2.99f, buf);
    }

    // ---------------- E10. アラゴの円板 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.substeps = 8;
        RigidBody* plate = w.createConductorBox({0, 0, 0}, {1.0f, 0.05f, 1.0f}, 1.0f, 1.0f);
        plate->angularDamping = 0.0f;
        w.setPinned(plate, true);
        RigidBody* mag = w.createMagnet({0.7f, 0.5f, 0}, 0.2f, 0.0f, {0, 3.0f, 0});   // 静的に動かす
        const float Wm = 2.0f, rad = 0.7f;
        float t = 0.0f;
        for (int i = 0; i < 360; ++i) {
            const float a = Wm * t;   // 上から見て反時計回り（+y まわり）
            mag->position          = Vec3{rad * std::cos(a), 0.5f, -rad * std::sin(a)};
            mag->kinematicVelocity = Vec3{-rad * Wm * std::sin(a), 0, -rad * Wm * std::cos(a)};
            w.step(DT);
            t += DT;
        }
        const float wy = plate->angularVelocity.y;
        std::snprintf(buf, sizeof buf, "magnet circles at +%.1f rad/s; after 3 s the copper plate spins at %+.3f rad/s", Wm, wy);
        check("E10 Arago's disc follows the rotating magnet", wy > 0.05f * Wm, buf);
    }

    // ---------------- E11. 回る銅の板のブレーキ ----------------
    {
        auto spin = [](bool magnets) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            w.substeps = 8;
            RigidBody* plate = w.createConductorBox({0, 0, 0}, {1.0f, 0.05f, 1.0f}, 1.0f, 1.0f);
            plate->angularDamping = 0.0f;
            w.setPinned(plate, true);
            plate->angularVelocity = Vec3{0, 5.0f, 0};
            if (magnets) {
                w.createMagnet({0.7f, 0.4f, 0}, 0.2f, 0.0f, {0, 3.0f, 0});
                w.createMagnet({0.7f, -0.4f, 0}, 0.2f, 0.0f, {0, 3.0f, 0});
            }
            for (int i = 0; i < 240; ++i) w.step(DT);
            return plate->angularVelocity.y;
        };
        const float with = spin(true), without = spin(false);
        std::snprintf(buf, sizeof buf, "spin 5.0 rad/s, after 2 s: with magnets %.3f, without %.3f", with, without);
        check("E11 eddy brake slows a spinning copper plate", with < 0.5f * 5.0f && with > 0.0f && without > 4.99f, buf);
    }

    std::printf("\n%s (%d failures)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}
