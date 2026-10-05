// 電流の導線とコイル（CurrentSystem）の検証（描画なし）
//   C1  直線の場（長い直線 2I/d と向き、有限の線分の cos t1 - cos t2 の式）
//   C2  正 64 角形の輪の場（中心、軸上、遠くの双極子）
//   C3  輪の軸上の磁石の力 m dB/dz、傾けた磁石のトルク m x B、作用・反作用
//   C4  同軸の 2 つの輪: 運動量・角運動量の保存、力（遠くで双極子、近くで二重積分）
//   C5  一様な外部磁場の中の輪: 合力 0、トルク I A x B0
//   C6  コイルの軸上の鉄球: m = alpha B、力 = -dU/dz
//   C7  コイルガン 1 段: 中心で切ると sqrt(2 dU / M) で出る。切らないと往復する
//   C8  ヘルムホルツコイル: 中心の場、一様さ、荷電粒子の円の半径
//   C9  エルステッド: 直線の導線のそばの方位磁針が atan(B_wire / B0) に向く
//   C10 磁力線: コイルの線が自分で閉じる（field_lines.h）
//   C12 速さ: 144 線分 x 磁性体の点 50 で 1 サブステップの時間
#include "phys/world.h"
#include "demo_common.h"
#include "field_lines.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace phys;

static int failures = 0;
static void check(const char* name, bool ok, const char* detail = "") {
    std::printf("[%s] %-56s %s\n", ok ? " OK " : "FAIL", name, detail);
    if (!ok) ++failures;
}

constexpr float G_ACC = 9.81f;
constexpr float DT    = 1.0f / 120.0f;

static void clearForces(World& w) {
    for (const auto& b : w.getBodies()) { b->force = Vec3{0, 0, 0}; b->torque = Vec3{0, 0, 0}; }
}

// 正 n 角形（半径 a）の面積
static float polygonArea(float a, int n) { return 0.5f * n * a * a * std::sin(2.0f * PHYS_PI / n); }

// ローカル +y を +x に向ける姿勢
static Quat axisX() { return Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI); }

// 全運動量と、原点まわりの全角運動量
static Vec3 totalMomentum(const World& w) {
    Vec3 P{0, 0, 0};
    for (const auto& b : w.getBodies()) if (!b->isStatic()) P += b->velocity * (1.0f / b->invMass);
    return P;
}
static Vec3 totalAngularMomentum(const World& w) {
    Vec3 L{0, 0, 0};
    for (const auto& b : w.getBodies()) {
        if (b->isStatic()) continue;
        L += cross(b->position, b->velocity * (1.0f / b->invMass));
        const Mat3 R = b->orientation.toMat3();
        const Vec3 wl = R.transposed() * b->angularVelocity;
        L += R * Vec3{wl.x / b->invInertiaLocal.x, wl.y / b->invInertiaLocal.y, wl.z / b->invInertiaLocal.z};
    }
    return L;
}

// 半径 a の円い輪（電流 I1、z = 0、軸 z）が、同軸で距離 d の半径 a の輪（電流 I2）に加える軸方向の力（double の数値積分）
static double coaxialLoopForce(double a, double d, double I1, double I2, int n) {
    double Bx = 0.0;
    for (int i = 0; i < n; ++i) {
        const double phi = 2.0 * 3.14159265358979323846 * (i + 0.5) / n, dphi = 2.0 * 3.14159265358979323846 / n;
        const double rx = a - a * std::cos(phi), ry = -a * std::sin(phi), rz = d;
        const double r3 = std::pow(rx * rx + ry * ry + rz * rz, 1.5);
        Bx += a * dphi * d * std::cos(phi) / r3;   // (dl x r)_x / r^3
    }
    Bx *= I1;
    return -2.0 * 3.14159265358979323846 * a * I2 * Bx;   // 平行な電流は引き合う（負 = 近づく向き）
}

int main() {
    char buf[512];
    const double PI_D = 3.14159265358979323846;

    // ---------------- C1. 直線の場 ----------------
    {
        const float I = 1.3f, d = 0.7f;
        const Vec3  B = segmentField({0, 0, -500.0f}, {0, 0, 500.0f}, {d, 0, 0}, I);
        const float err = std::fabs(length(B) - 2.0f * I / d) / (2.0f * I / d);
        const float dir = dot(normalize(B), Vec3{0, 1, 0});
        // 有限の線分: B = I / rho (cos t1 - cos t2)、向きは t x rho
        const Vec3 a{0.2f, -0.1f, 0.0f}, b{0.5f, 0.3f, 2.0f};
        const Vec3 t = normalize(b - a);
        float maxErr = 0.0f;
        unsigned seed = 12345;
        auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return (float)(seed >> 8) / 16777216.0f * 2.0f - 1.0f; };
        for (int k = 0; k < 50; ++k) {
            const Vec3  p{rnd() * 3.0f, rnd() * 3.0f, rnd() * 4.0f};
            const Vec3  ra = p - a, rb = p - b;
            const Vec3  rho = ra - t * dot(ra, t);
            const float rl  = length(rho);
            if (rl < 0.2f) continue;
            const float c1 = dot(ra, t) / length(ra), c2 = dot(rb, t) / length(rb);
            const Vec3  want = cross(t, rho / rl) * (I / rl * (c1 - c2));
            const Vec3  got  = segmentField(a, b, p, I);
            maxErr = std::max(maxErr, length(got - want) / length(want));
        }
        std::snprintf(buf, sizeof buf, "long wire: |B| vs 2I/d %.1e, direction %.6f;  finite segment vs cos formula %.1e", err, dir, maxErr);
        check("C1 straight wire field and direction", err < 1e-4f && dir > 0.9999f && maxErr < 1e-5f, buf);
    }

    // ---------------- C2. 正 64 角形の輪 ----------------
    {
        World w;
        const float a = 1.0f, I = 1.0f;
        const int   n = 64;
        w.currents().addCoil({0, 0, 0}, Quat{}, a, n, I);
        const CurrentSystem& cs = w.currents();
        const float Bc   = length(cs.fieldAt({0, 0, 0}));
        const float errC = std::fabs(Bc - 2.0f * PHYS_PI * I / a) / (2.0f * PHYS_PI * I / a);
        float errAxis = 0.0f;
        for (float y : {0.5f, 1.0f, 2.0f}) {
            const float want = 2.0f * PHYS_PI * I * a * a / std::pow(a * a + y * y, 1.5f);
            errAxis = std::max(errAxis, std::fabs(cs.fieldAt({0, y, 0}).y - want) / want);
        }
        const float m    = I * PHYS_PI * a * a;   // コイルの場は円の厳密な式（楕円積分）で求める
        const float yFar = 100.0f;   // 輪と双極子の差は (a/r)^2 なので、遠くで比べる
        const Vec3  Bfar = cs.fieldAt({0, yFar, 0});
        const float errFar = std::fabs(Bfar.y - 2.0f * m / (yFar * yFar * yFar)) / (2.0f * m / (yFar * yFar * yFar));
        const Vec3  Boff = cs.fieldAt({3.0f, 1.5f, 0.0f});
        const Vec3  Bdip = dipoleField({3.0f, 1.5f, 0.0f}, {0, m, 0});
        const float errOff = length(Boff - Bdip) / length(Bdip);
        // 正多角形の線分の和（直線の式）と、円の式を比べる
        float errPoly = 0.0f;
        for (const Vec3& p : {Vec3{0.3f, 0.4f, 0.1f}, Vec3{1.5f, -0.2f, 0.6f}, Vec3{0.9f, 0.3f, 0.0f}}) {
            Vec3 Bs{0, 0, 0};
            const Wire& wr = cs.wires()[0];
            for (int s = 0; s < wr.segmentCount(); ++s) { Vec3 pa, pb; wr.segment(s, pa, pb); Bs += segmentField(pa, pb, p, I); }
            errPoly = std::max(errPoly, length(Bs - cs.fieldAt(p)) / length(cs.fieldAt(p)));
        }
        // 軸のすぐそば: 軸と垂直な成分は B_rho = 3 pi I a^2 rho z / (a^2 + z^2)^(5/2) に近づき、軸上では 0
        //（a^2 を float で丸めると K と E の差が桁落ちして、1e-3 a 以内で符号まで狂った。コイルガンの弾が蹴られて見つかった）
        float errNear = 0.0f, onAxis = 0.0f;
        for (float rho : {1e-6f, 1e-4f, 1e-3f, 1e-2f}) {
            const float z = -1.0f, want = 3.0f * PHYS_PI * I * a * a * rho * z / std::pow(a * a + z * z, 2.5f);
            const Vec3  B = cs.fieldAt({rho, z, 0});
            errNear = std::max(errNear, std::fabs(B.x - want) / std::fabs(3.0f * PHYS_PI * I * a * a * 1e-2f / std::pow(a * a + z * z, 2.5f)));
        }
        for (float z : {-1.0f, 0.3f, 2.0f}) { const Vec3 B = cs.fieldAt({0, z, 0}); onAxis = std::max(onAxis, std::sqrt(B.x * B.x + B.z * B.z) / std::fabs(B.y)); }
        std::snprintf(buf, sizeof buf, "center %.1e (2 pi I / a), on axis %.1e, dipole at 100 a %.1e, 64-gon segments vs circle %.2f%% (at 3.4 a the loop differs from a dipole by %.1f%%); near the axis B_rho vs 3 pi I a^2 rho z / r^5 %.1e, on-axis side component %.1e",
                      errC, errAxis, errFar, 100 * errPoly, 100 * errOff, errNear, onAxis);
        check("C2 loop field (center, axis, far dipole, polygon, near axis)", errC < 1e-4f && errAxis < 1e-4f && errFar < 1e-3f && errPoly < 1e-2f && errNear < 1e-3f && onAxis < 1e-6f, buf);
    }

    // ---------------- C3. 輪の軸上の磁石 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float a = 1.0f, I = 2.0f, m0 = 1.0f;
        RigidBody* plate = w.createBox({0, 0, 0}, {a + 0.1f, 0.02f, a + 0.1f}, 1.0f);
        w.currents().addCoil({0, 0, 0}, Quat{}, a, 64, I, 1, plate);
        RigidBody* mag = w.createMagnet({0, 0.6f, 0}, 0.1f, 1.0f, {0, m0, 0});
        w.currents().prepare();
        clearForces(w);
        w.magnets().applyForces();
        w.currents().applyForces(w.magnets());
        const float eps = 1e-3f;
        const float dBdy = (w.currents().fieldAt({0, 0.6f + eps, 0}).y - w.currents().fieldAt({0, 0.6f - eps, 0}).y) / (2 * eps);
        const float want = m0 * dBdy;
        const float errF = std::fabs(mag->force.y - want) / std::fabs(want);
        const float side = std::sqrt(mag->force.x * mag->force.x + mag->force.z * mag->force.z) / std::fabs(want);
        const float sum  = length(mag->force + plate->force) / std::fabs(want);
        // 傾けた磁石のトルク
        RigidBody* mag2 = w.createMagnet({0.3f, 0.6f, 0.2f}, 0.1f, 1.0f, {0.5f, 0.8660254f, 0});
        clearForces(w);
        w.magnets().applyForces();
        w.currents().applyForces(w.magnets());
        const Vec3  mvec = w.magnets().bodies()[w.magnets().indexOf(mag2)].m;
        const Vec3  tauWant = cross(mvec, w.currents().fieldAt(mag2->position));
        // mag と mag2 の間の磁力のトルクを除く: mag2 だけの世界で測り直す
        World w2;
        w2.gravity = Vec3{0, 0, 0};
        w2.currents().addCoil({0, 0, 0}, Quat{}, a, 64, I);
        RigidBody* mag3 = w2.createMagnet({0.3f, 0.6f, 0.2f}, 0.1f, 1.0f, {0.5f, 0.8660254f, 0});
        w2.currents().prepare();
        clearForces(w2);
        w2.magnets().applyForces();
        w2.currents().applyForces(w2.magnets());
        const Vec3  tau3 = cross(w2.magnets().bodies()[0].m, w2.currents().fieldAt(mag3->position));
        const float errT = length(mag3->torque - tau3) / length(tau3);
        (void)tauWant;
        std::snprintf(buf, sizeof buf, "F_y vs m dB/dy %.2f%% (side %.1e), |F_magnet + F_coil| %.1e, torque vs m x B %.1e",
                      100 * errF, side, sum, errT);
        check("C3 dipole on loop axis: force, action-reaction, torque", errF < 5e-3f && side < 1e-4f && sum < 1e-5f && errT < 1e-5f, buf);
        // 世界を進めても落ちない（磁石が輪に引き込まれる）
        for (int i = 0; i < 60; ++i) w.step(DT);
    }

    // ---------------- C4. 同軸の 2 つの輪 ----------------
    {
        const float a = 1.0f, I1 = 2.0f, I2 = 1.5f;
        auto measure = [&](float d, float* errRef, float* errDip, float* sumF) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            RigidBody* p1 = w.createBox({0, 0, 0}, {a + 0.1f, 0.02f, a + 0.1f}, 1.0f);
            RigidBody* p2 = w.createBox({0, d, 0}, {a + 0.1f, 0.02f, a + 0.1f}, 1.0f);
            w.currents().addCoil({0, 0, 0}, Quat{}, a, 64, I1, 1, p1);
            w.currents().addCoil({0, 0, 0}, Quat{}, a, 64, I2, 1, p2);
            w.currents().prepare();
            clearForces(w);
            w.currents().applyForces(w.magnets());
            const double ref = coaxialLoopForce(a, d, I1, I2, 20000);
            *errRef = (float)std::fabs((double)p2->force.y - ref) / (float)std::fabs(ref);
            const float m1 = I1 * PHYS_PI * a * a, m2 = I2 * polygonArea(a, 64);   // b の場は円の式、a の求積点は多角形
            const float dip = -6.0f * m1 * m2 / (d * d * d * d);
            *errDip = std::fabs(p2->force.y - dip) / std::fabs(dip);
            *sumF   = length(p1->force + p2->force) / std::fabs(p2->force.y);
            return p2->force.y;
        };
        float eRef1, eDip1, s1, eRef10, eDip10, s10, eRef40, eDip40, s40;
        const float F1  = measure(1.0f, &eRef1, &eDip1, &s1);
        const float F10 = measure(10.0f, &eRef10, &eDip10, &s10);
        measure(40.0f, &eRef40, &eDip40, &s40);   // 輪と双極子の差は (a/d)^2（10 a で約 5%）なので、双極子とは 40 a で比べる
        // 自由な 2 つの輪を 2 秒動かして、運動量と角運動量
        World w;
        w.gravity = Vec3{0, 0, 0};
        RigidBody* p1 = w.createBox({0, 0, 0}, {a + 0.1f, 0.02f, a + 0.1f}, 1.0f);
        RigidBody* p2 = w.createBox({0.3f, 1.2f, 0.2f}, {a + 0.1f, 0.02f, a + 0.1f}, 2.0f);
        p2->orientation = Quat::fromAxisAngle({1, 0, 0}, 0.4f);
        for (RigidBody* b : {p1, p2}) { b->linearDamping = 0.0f; b->angularDamping = 0.0f; }
        w.currents().addCoil({0, 0, 0}, Quat{}, a, 48, I1, 1, p1);
        w.currents().addCoil({0, 0, 0}, Quat{}, a, 48, -I2, 1, p2);
        w.substeps = 4;
        float maxP = 0.0f, maxL = 0.0f, scaleP = 0.0f, scaleL = 0.0f;
        for (int i = 0; i < 240; ++i) {
            w.step(DT);
            maxP = std::max(maxP, length(totalMomentum(w)));
            maxL = std::max(maxL, length(totalAngularMomentum(w)));
            scaleP = std::max(scaleP, length(p2->velocity) * 2.0f);
            scaleL = std::max(scaleL, length(cross(p2->position, p2->velocity * 2.0f)));
        }
        std::snprintf(buf, sizeof buf, "F(d=a) %.3f vs integral %.2f%%, F(d=10a) %.4f vs integral %.2f%% (dipole %.1f%%), F(d=40a) vs dipole %.2f%%; |F1+F2| %.1e; 2 s free: |P| %.1e (of %.2f), |L| %.1e (of %.2f)",
                      F1, 100 * eRef1, F10, 100 * eRef10, 100 * eDip10, 100 * eDip40, std::max(s1, s10), maxP, scaleP, maxL, scaleL);
        check("C4 two coaxial loops: force vs theory, momentum, angular momentum",
              F1 < 0.0f && eRef1 < 1e-2f && eRef10 < 1e-2f && eDip40 < 1e-2f && s1 < 1e-5f && maxP < 1e-5f * scaleP && maxL < 1e-3f * scaleL, buf);
        (void)eDip1; (void)eRef40; (void)s40;
    }

    // ---------------- C5. 一様な外部磁場の中の輪 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float a = 0.8f, I = 1.5f;
        RigidBody* plate = w.createBox({0.5f, 1.0f, -0.2f}, {a + 0.1f, 0.02f, a + 0.1f}, 1.0f);
        plate->orientation = Quat::fromAxisAngle({0, 0, 1}, 0.6f);
        w.currents().addCoil({0, 0, 0}, Quat{}, a, 40, I, 1, plate);
        w.magnets().externalField = Vec3{0.3f, 0.0f, 0.5f};
        w.currents().prepare();
        clearForces(w);
        w.currents().applyForces(w.magnets());
        const Vec3  A    = w.currents().wires()[0].areaVector();
        const Vec3  want = cross(A * I, w.magnets().externalField);
        const float errT = length(plate->torque - want) / length(want);
        const float errF = length(plate->force) / length(want) * a;
        std::snprintf(buf, sizeof buf, "|F| / (|tau| / a) %.1e, torque vs I A x B0 %.1e (|A| = %.3f = %.3f polygon)", errF, errT, length(A),
                      polygonArea(a, 40));
        check("C5 loop in uniform field: zero force, torque I A x B0", errF < 1e-5f && errT < 1e-5f, buf);
    }

    // ---------------- C6. コイルの軸上の鉄球 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float a = 1.0f, I = 3.0f, r = 0.2f;
        const float alpha = sphereSusceptibility(r, 1000.0f);
        w.currents().addCoil({0, 0, 0}, Quat{}, a, 64, I);
        RigidBody* ball = w.createSoftMagnet({0, 0.8f, 0}, r, 1.0f, alpha);
        w.magnets().extraField = &w.currents();   // World::step が毎回つなぐ。ここでは手で
        w.magnets().updateMoments(20);
        const Vec3  m    = w.magnets().bodies()[0].m;
        const Vec3  want = w.currents().fieldAt(ball->position) * alpha;
        const float errM = length(m - want) / length(want);
        clearForces(w);
        w.magnets().applyForces();
        w.currents().applyForces(w.magnets());
        const float F = ball->force.y;
        const float eps = 1e-3f;
        auto U = [&](float y) {
            ball->position.y = y;
            w.magnets().updateMoments(20);
            return w.magnets().potentialEnergy();
        };
        const float dU = (U(0.8f + eps) - U(0.8f - eps)) / (2 * eps);
        ball->position.y = 0.8f;
        w.magnets().updateMoments(20);
        const float errF = std::fabs(F + dU) / std::fabs(dU);
        std::snprintf(buf, sizeof buf, "m vs alpha B %.1e, F_y %.4f vs -dU/dy %.4f (%.2f%%)", errM, F, -dU, 100 * errF);
        check("C6 iron ball on coil axis: induced moment, force = -dU/dy", errM < 1e-3f && errF < 2e-3f, buf);
    }

    // ---------------- C7. コイルガン 1 段 ----------------
    {
        // stepDt: 電流を切る判定の刻み（サブステップの刻みは 1/960 s のまま）
        auto run = [&](bool switchOff, float* vOut, float* xMax, float* vTheory, float stepDt = DT / 8.0f) {
            World w;
            w.gravity  = Vec3{0, 0, 0};
            w.substeps = std::max(1, (int)std::lround(stepDt / (DT / 8.0f)));
            const float a = 0.5f, I = 0.5f;
            const int   ci = w.currents().addCoil({0, 0, 0}, axisX(), a, 24, I, 10);
            const float x0 = -2.0f * a;   // 遠すぎると引き込むのに時間がかかる（本物も球はコイルのすぐ手前に置く）
            RigidBody* ball = demo::makeIronBall(w, {x0, 0, 0});
            ball->linearDamping = 0.0f;
            // 理論: 始めの位置と中心のエネルギー差
            w.magnets().extraField = &w.currents();
            auto U = [&](float x) { ball->position.x = x; w.magnets().updateMoments(30); return w.magnets().potentialEnergy(); };
            const float dU = U(x0) - U(0.0f);
            ball->position.x = x0;
            w.magnets().updateMoments(30);
            *vTheory = std::sqrt(2.0f * dU * ball->invMass);
            *xMax = x0;
            float t = 0.0f;
            // 切らない場合は 3 s だけ見る（軸の上は横に不安定なので、長く置くと球は導線の側へ寄っていく）
            while (t < (switchOff ? 20.0f : 3.0f) && ball->position.x < 3.0f) {
                if (switchOff && ball->position.x >= 0.0f) w.currents().wires()[ci].current = 0.0f;
                w.step(stepDt);
                t += stepDt;
                *xMax = std::max(*xMax, ball->position.x);
            }
            *vOut = switchOff ? ball->velocity.x : ball->position.x;
        };
        float v, xm, vt, xEnd, xm2, vt2, vc, xmc, vtc;
        run(true, &v, &xm, &vt);
        run(true, &vc, &xmc, &vtc, DT);   // 1/120 s ごとにしか切らないと、中心を最大 v dt = 0.09 m 過ぎてから切れる（引き戻される）
        run(false, &xEnd, &xm2, &vt2);
        const float err = std::fabs(v - vt) / vt, errCoarse = std::fabs(vc - vtc) / vtc;
        std::snprintf(buf, sizeof buf, "switch off at center: exit %.3f m/s vs sqrt(2 dU/M) %.3f (%.2f%%; switching only every 1/120 s: %.1f%% low); no switch (3 s): max x %.2f, end x %.2f (oscillates)",
                      v, vt, 100 * err, 100 * errCoarse, xm2, xEnd);
        check("C7 coil gun stage: exit speed, oscillates without switching", err < 1e-2f && xm2 < 1.0f && std::fabs(xEnd) < 1.0f, buf);
    }

    // ---------------- C8. ヘルムホルツコイル ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float a = 1.5f, I = 1.0f;
        w.currents().addCoil({0, -0.5f * a, 0}, Quat{}, a, 48, I);
        w.currents().addCoil({0, 0.5f * a, 0}, Quat{}, a, 48, I);
        const float Bc   = w.currents().fieldAt({0, 0, 0}).y;
        const float want = std::pow(0.8f, 1.5f) * 4.0f * PHYS_PI * I / a;
        const float errC = std::fabs(Bc - want) / want;
        float nonUni = 0.0f;
        for (float y : {-0.25f * a, 0.0f, 0.25f * a})
            for (float x : {0.0f, 0.25f * a}) nonUni = std::max(nonUni, std::fabs(length(w.currents().fieldAt({x, y, 0})) / Bc - 1.0f));
        // 荷電粒子の円運動
        const float q = 0.1f, mass = 1.0f, v0 = 1.0f;
        w.electric().kappa = 10.0f;
        const float rTh = mass * v0 / (w.electric().kappa * q * Bc);
        RigidBody* p = w.createChargedSphere({rTh, 0, 0}, 0.02f, mass, q);
        p->velocity = Vec3{0, 0, -v0};
        p->linearDamping = 0.0f;
        w.substeps = 4;
        const float T = 2.0f * PHYS_PI * mass / (w.electric().kappa * q * Bc);
        const int   steps = (int)std::lround(5.0f * T / DT);
        Vec3 mean{0, 0, 0};
        std::vector<Vec3> pts;
        for (int i = 0; i < steps; ++i) {
            w.step(DT);
            pts.push_back(p->position);
            mean += p->position;
        }
        mean = mean * (1.0f / pts.size());
        float rMean = 0.0f, rMin = 1e9f, rMax = 0.0f, yDrift = 0.0f;
        for (const Vec3& x : pts) {
            const float r = std::sqrt((x.x - mean.x) * (x.x - mean.x) + (x.z - mean.z) * (x.z - mean.z));
            rMean += r; rMin = std::min(rMin, r); rMax = std::max(rMax, r);
            yDrift = std::max(yDrift, std::fabs(x.y));
        }
        rMean /= pts.size();
        const float errR = std::fabs(rMean - rTh) / rTh;
        std::snprintf(buf, sizeof buf, "B center %.3f vs (4/5)^1.5 4 pi I/a %.2f%%; non-uniformity within a/4 %.2f%%; orbit radius %.4f vs %.4f (%.2f%%, min %.4f max %.4f)",
                      Bc, 100 * errC, 100 * nonUni, rMean, rTh, 100 * errR, rMin, rMax);
        check("C8 Helmholtz coils: center field, uniformity, particle orbit", errC < 5e-3f && nonUni < 1e-2f && errR < 1e-2f && yDrift < 1e-3f, buf);
    }

    // ---------------- C9. エルステッド ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float I = 0.3f, h = 0.5f, B0 = 1.2f;
        w.magnets().externalField = Vec3{B0, 0, 0};   // 「地磁気」は +x。導線も x に沿う（針の真上）
        w.currents().addWire(nullptr, {{-50.0f, h, 0}, {50.0f, h, 0}}, false, I);
        RigidBody* needle = w.createMagnet({0, 0, 0}, 0.1f, 0.1f, {1.0f, 0, 0});
        needle->angularDamping = 3.0f;
        w.setPinned(needle, true);
        w.substeps = 4;
        for (int i = 0; i < 720; ++i) w.step(DT);
        const Vec3  m   = w.magnets().bodies()[0].m;
        const float ang = std::atan2(-m.z, m.x) * 180.0f / PHYS_PI;   // 導線の場は -z 向き（右ねじ）
        const float want = std::atan2(2.0f * I / h, B0) * 180.0f / PHYS_PI;
        std::snprintf(buf, sizeof buf, "needle %.2f deg vs atan(2I/h / B0) = %.2f deg, position moved %.1e", ang, want, length(needle->position));
        check("C9 Oersted: compass needle under a straight wire", std::fabs(ang - want) < 1.0f && length(needle->position) < 1e-5f, buf);
    }

    // ---------------- C10. 磁力線（コイルの線が閉じる） ----------------
    {
        World w;
        w.currents().addCoil({0, 0, 0}, Quat{}, 1.0f, 48, 1.0f);
        demo::FieldLineTracer tracer;
        tracer.fluxPerLine = 0.6f;
        tracer.traceAll(w);
        int closed = 0, total = 0, leaves = 0;
        for (const demo::FieldLine& l : tracer.lines()) {
            if (l.pts.size() < 4) continue;
            ++total;
            if (length(l.pts.front() - l.pts.back()) < 0.1f) ++closed;
            else if (length(l.pts.front()) > 3.0f && length(l.pts.back()) > 3.0f) ++leaves;   // 内側の線は範囲の外まで広がる（両端が外）
        }
        w.currents().wires()[0].current = 0.0f;
        tracer.traceAll(w);
        const int after = (int)tracer.lines().size();
        std::snprintf(buf, sizeof buf, "%d / %d lines close on themselves, %d leave the box at both ends; with the current off: %d lines", closed, total,
                      leaves, after);
        check("C10 field lines of a coil close, vanish when switched off", total > 0 && closed + leaves == total && closed > 0 && after == 0, buf);
    }

    // ---------------- C12. 速さ ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.currents().addSolenoid({0, 0, 0}, Quat{}, 0.5f, 1.6f, 6, 24, 1.0f, 10);
        demo::makeIronBox(w, {0, 0, 0}, {0.15f, 0.8f, 0.15f});
        for (int i = 0; i < 5; ++i) demo::makeIronBox(w, {1.5f + 0.5f * i, -1.0f, 0}, {0.1f, 0.1f, 0.1f});
        demo::makeIronBall(w, {0, -1.5f, 0});
        demo::makeIronBall(w, {0.6f, -1.5f, 0});
        int points = 0;
        for (const MagneticBody& mb : w.magnets().bodies()) points += mb.pointCount;
        w.step(DT);
        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();
        const int  N  = 200;
        for (int i = 0; i < N; ++i) {
            w.currents().prepare();
            w.magnets().applyForces();
            w.currents().applyForces(w.magnets());
        }
        const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count() / N;
        std::snprintf(buf, sizeof buf, "%d segments x %d magnetic points: %.3f ms per substep", 144, points, ms);
        check("C12 timing (record only)", true, buf);
    }

    std::printf("\n%s\n", failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
