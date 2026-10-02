// 磁石・砂鉄のヘッドレス検証（raylib 不要）
//   magnet_test [--csv <dir>]   --csv を付けると T3 のエネルギー時系列を CSV で出す
#include "phys/world.h"
#include "field_lines.h"
#include "gauss_rail.h"
#include "levitation_theory.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace phys;

static int failures = 0;
static void check(const char* name, bool ok, const char* detail = "") {
    std::printf("[%s] %-42s %s\n", ok ? " OK " : "FAIL", name, detail);
    if (!ok) ++failures;
}
static void info(const char* name, const char* detail) {
    std::printf("[INFO] %-42s %s\n", name, detail);
}

// ---------------------------------------------------------------------------
// 共通の設定（仕様書 §3.3: 半径 0.25、Gamma = 779）
// ---------------------------------------------------------------------------
constexpr float R_MAG  = 0.25f;
constexpr float M_MAG  = 0.5f;
constexpr float GAMMA  = 779.0f;
constexpr float G_ACC  = 9.81f;
constexpr float DT     = 1.0f / 120.0f;
constexpr int   SUBSTEPS = 8;

static const float M0 = momentFromGamma(GAMMA, M_MAG, 2.0f * R_MAG, G_ACC);

static RigidBody* addMagnet(World& w, const Vec3& pos, const Vec3& dir,
                            float moment, float radius = R_MAG, float mass = M_MAG) {
    RigidBody* b = w.createMagnet(pos, radius, mass, normalize(dir) * moment);
    b->restitution     = 0.3f;
    b->friction        = 0.3f;
    b->rollingFriction = 0.01f;
    b->linearDamping   = 0.0f;
    b->angularDamping  = 0.0f;
    return b;
}

static RigidBody* addIron(World& w, const Vec3& pos, float radius, float mass,
                          float alpha, float saturation = 0.0f) {
    RigidBody* b = w.createSoftMagnet(pos, radius, mass, alpha, saturation);
    b->linearDamping  = 0.0f;
    b->angularDamping = 0.0f;
    return b;
}

static Vec3 linearMomentum(const World& w) {
    Vec3 P{0, 0, 0};
    for (const auto& b : w.getBodies())
        if (!b->isStatic()) P += b->velocity * (1.0f / b->invMass);
    return P;
}

// 球だけを想定（慣性が等方的）
static Vec3 angularMomentum(const World& w) {
    Vec3 L{0, 0, 0};
    for (const auto& b : w.getBodies()) {
        if (b->isStatic()) continue;
        float M = 1.0f / b->invMass;
        L += cross(b->position, b->velocity * M) + b->angularVelocity * (1.0f / b->invInertiaLocal.x);
    }
    return L;
}

// 箱も含む一般の角運動量（自転は R I R^T w）
static Vec3 angularMomentumGeneral(const World& w) {
    Vec3 L{0, 0, 0};
    for (const auto& b : w.getBodies()) {
        if (b->isStatic()) continue;
        float M = 1.0f / b->invMass;
        Mat3 R  = b->orientation.toMat3();
        Vec3 wl = R.transposed() * b->angularVelocity;
        Vec3 Il{wl.x / b->invInertiaLocal.x, wl.y / b->invInertiaLocal.y, wl.z / b->invInertiaLocal.z};
        L += cross(b->position, b->velocity * M) + R * Il;
    }
    return L;
}

static RigidBody* addMagnetBox(World& w, const Vec3& pos, const Vec3& he, float mass,
                               const Vec3& momentLocal, const Quat& q = Quat{}) {
    RigidBody* b = w.createMagnetBox(pos, he, mass, momentLocal, q);
    b->linearDamping  = 0.0f;
    b->angularDamping = 0.0f;
    return b;
}

static void clearForces(World& w) {
    for (const auto& b : w.getBodies()) { b->force = Vec3{0, 0, 0}; b->torque = Vec3{0, 0, 0}; }
}

static void simulate(World& w, float seconds, int substeps = SUBSTEPS) {
    w.substeps = substeps;
    int steps = (int)std::lround(seconds / DT);
    for (int i = 0; i < steps; ++i) w.step(DT);
}

static const char* typeName(MagnetSystem::StructureType t) {
    switch (t) {
    case MagnetSystem::StructureType::Single: return "single";
    case MagnetSystem::StructureType::Chain:  return "chain";
    case MagnetSystem::StructureType::Ring:   return "ring";
    default:                                  return "other";
    }
}

// 付録 A.1（単位 u0 = K m^2 / d^3）
static const float CHAIN_U[13] = {0, 0, -2.0000f, -4.2500f, -6.5741f, -8.9294f, -11.3007f,
                                  -13.6813f, -16.0677f, -18.4580f, -20.8511f, -23.2462f, -25.6427f};
static const float RING_U[13]  = {0, 0, 0, -3.7500f, -6.7071f, -9.5656f, -12.3184f,
                                  -14.9913f, -17.6066f, -20.1800f, -22.7224f, -25.2414f, -27.7423f};

int main(int argc, char** argv) {
    std::string csvDir;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], "--csv") == 0) csvDir = argv[i + 1];

    char buf[256];
    std::mt19937 rng(20260929);
    std::normal_distribution<float>       gauss(0.0f, 1.0f);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    auto randomDir = [&]() { return normalize(Vec3{gauss(rng), gauss(rng), gauss(rng)}); };

    std::printf("m0 = %.4f (Gamma %.0f, R %.2f, M %.2f)\n\n", M0, GAMMA, R_MAG, M_MAG);

    // ---------------- T1. 力 = -grad U、G m2 = F、G は対称・トレース 0 ----------------
    {
        float worstF = 0, worstG = 0, worstSym = 0, worstTr = 0;
        for (int k = 0; k < 100; ++k) {
            Vec3 m1{gauss(rng), gauss(rng), gauss(rng)};
            Vec3 m2{gauss(rng), gauss(rng), gauss(rng)};
            Vec3 r = randomDir() * (0.8f + 2.2f * uni(rng));
            float rl = length(r);

            Vec3  F = dipoleForce(r, m1, m2);
            float h = 1e-3f * rl;
            Vec3  Fnum;
            for (int a = 0; a < 3; ++a) {
                Vec3 e{0, 0, 0}; e[a] = h;
                Fnum[a] = -(dipoleEnergy(r + e, m1, m2) - dipoleEnergy(r - e, m1, m2)) / (2 * h);
            }
            float scale = 3.0f * length(m1) * length(m2) / (rl * rl * rl * rl);
            worstF = std::max(worstF, length(F - Fnum) / scale);

            Mat3 G = dipoleGradient(r, m1);
            worstG = std::max(worstG, length(G * m2 - F) / scale);
            float gs = 3.0f * length(m1) / (rl * rl * rl * rl);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    worstSym = std::max(worstSym, std::fabs(G.col(j)[i] - G.col(i)[j]) / gs);
            worstTr = std::max(worstTr, std::fabs(G.c0.x + G.c1.y + G.c2.z) / gs);
        }
        std::snprintf(buf, sizeof buf, "F vs -gradU %.1e  Gm2 %.1e  sym %.1e  tr %.1e",
                      worstF, worstG, worstSym, worstTr);
        check("T1 force = -grad U, F = G m", worstF < 1e-3f && worstG < 1e-5f &&
                                             worstSym < 1e-5f && worstTr < 1e-5f, buf);
    }

    // ---------------- T2. 運動量・角運動量の保存（接触なし） ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        RigidBody* a = addMagnet(w, {-1.5f, 0.0f, 0.0f}, {0.3f, 1.0f, 0.2f}, 0.3f);
        RigidBody* b = addMagnet(w, { 1.5f, 0.2f, 0.0f}, {1.0f, 0.2f, -0.4f}, 0.3f);
        a->angularVelocity = Vec3{1.0f, 2.0f, 0.5f};
        b->angularVelocity = Vec3{-0.5f, 0.3f, 1.5f};

        Vec3 P0 = linearMomentum(w), L0 = angularMomentum(w);
        Vec3 Lorb0 = cross(a->position, a->velocity * M_MAG) + cross(b->position, b->velocity * M_MAG);
        int maxContacts = 0;
        int steps = (int)std::lround(10.0f / DT);
        w.substeps = SUBSTEPS;
        for (int i = 0; i < steps; ++i) {
            w.step(DT);
            maxContacts = std::max(maxContacts, w.contactCount());
        }
        Vec3 Lorb1 = cross(a->position, a->velocity * M_MAG) + cross(b->position, b->velocity * M_MAG);
        float errP = length(linearMomentum(w) - P0);
        float errL = length(angularMomentum(w) - L0) / length(L0);
        std::snprintf(buf, sizeof buf, "dP=%.1e dL/L=%.1e (orbital L moved %.3f) contacts=%d",
                      errP, errL, length(Lorb1 - Lorb0), maxContacts);
        check("T2 momentum & angular momentum", errP < 1e-4f && errL < 1e-3f && maxContacts == 0, buf);
    }

    // ---------------- T3a. 接触前のエネルギー誤差（サブステップ数と積分の次数） ----------------
    //   一直線に並べた 2 個を静止状態から放し、接触の少し前の共通の時刻 t* で
    //   |E - E0| / (運動エネルギー) を測る。接触ソルバの影響を受けない、磁力の積分だけの誤差。
    //   半陽的オイラー法の誤差は主に (h/2) F.v（蓄積しない 1 次の項）で、接触直前に最大になる
    {
        auto twoMagnets = [](World& w) {
            w.gravity = Vec3{0, 0, 0};
            addMagnet(w, {-0.75f, 0, 0}, {1, 0, 0}, M0);   // 中心間 1.5 = 接触距離の 3 倍
            addMagnet(w, { 0.75f, 0, 0}, {1, 0, 0}, M0);
        };
        // 細かい刻みで接触時刻を求め、その 1 ステップ手前（DT の整数倍）を t* にする
        int contactStep = 0;
        {
            World w; twoMagnets(w);
            w.substeps = 128;
            for (int i = 1; i < (int)(2.0f / DT) && contactStep == 0; ++i) {
                w.step(DT);
                if (w.contactCount() > 0) contactStep = i;
            }
        }
        const int tStar = std::max(1, contactStep - 1);

        const int subs[] = {1, 2, 4, 8, 16, 32};
        float err[6] = {}, sep = 0.0f;
        std::string table;
        for (int k = 0; k < 6; ++k) {
            World w; twoMagnets(w);
            float E0 = w.energy().total();
            w.substeps = subs[k];
            for (int i = 0; i < tStar; ++i) w.step(DT);
            EnergyReport e = w.energy();
            err[k] = std::fabs(e.total() - E0) / std::max(e.kinetic, 1e-9f);
            if (subs[k] == 8) sep = length(w.getBodies()[1]->position - w.getBodies()[0]->position);
            char row[48];
            std::snprintf(row, sizeof row, " %d:%.2f%%", subs[k], err[k] * 100.0f);
            table += row;
        }
        // 1 次の収束: サブステップ 4 以降、刻みを半分にすると誤差もほぼ半分（比 0.35〜0.65）
        bool firstOrder = true;
        for (int k = 3; k < 6; ++k) {
            float ratio = err[k] / err[k - 1];
            if (ratio < 0.35f || ratio > 0.65f) firstOrder = false;
        }
        std::snprintf(buf, sizeof buf, "t*=%.3f s (r=%.2f d):%s", tStar * DT, sep / (2 * R_MAG),
                      table.c_str());
        check("T3a energy error before contact ~ h", firstOrder, buf);
    }

    // ---------------- T3b. エネルギーが注入されない（爆発しない） ----------------
    //   16 個をばらまいて 20 秒。max(E - E0) / max|U| を測る。
    //   仕様書の「反発 1」は使わない。反発は磁力を積分した後の速度で判定されるので、
    //   反発 1 だと衝突のたびにそのサブステップの引き込み速度（約 16）まで跳ね返し、
    //   刻みに比例したエネルギーを注入してしまう。
    //   デモと同じ材質（反発 0.3、摩擦 0.3）で、衝突は散逸してもエネルギーは増えないことを確かめる。
    {
        FILE* csv = nullptr;
        if (!csvDir.empty()) {
            std::filesystem::create_directories(csvDir);
            csv = std::fopen((csvDir + "/t3_energy.csv").c_str(), "w");
            if (csv) std::fprintf(csv, "substeps,t,total,kinetic,rotational,magnetic\n");
        }

        // 同じ初期配置をすべてのサブステップ数で使う
        std::vector<Vec3> pos, dir;
        std::mt19937 rng3(3);
        std::uniform_real_distribution<float> box(-1.2f, 1.2f);
        while ((int)pos.size() < 16) {
            Vec3 p{box(rng3), box(rng3), box(rng3)};
            bool ok = true;
            for (const Vec3& q : pos) if (length(p - q) < 0.7f) { ok = false; break; }
            if (!ok) continue;
            pos.push_back(p);
            dir.push_back(normalize(Vec3{box(rng3), box(rng3), box(rng3)}));
        }

        const int subs[] = {1, 2, 4, 8, 16};
        std::string table;
        float err8 = 1e9f;
        for (int s : subs) {
            World w;
            w.gravity  = Vec3{0, 0, 0};
            w.substeps = s;
            for (int i = 0; i < 16; ++i) addMagnet(w, pos[i], dir[i], M0);

            float E0 = w.energy().total();
            float maxGain = 0.0f, Uscale = 1e-9f;
            bool finite = true;
            int steps = (int)std::lround(20.0f / DT);
            for (int i = 0; i <= steps; ++i) {
                if (i > 0) w.step(DT);
                EnergyReport e = w.energy();
                if (!std::isfinite(e.total())) { finite = false; break; }
                maxGain = std::max(maxGain, e.total() - E0);
                Uscale  = std::max(Uscale, std::fabs(e.magnetic));
                if (csv && i % 6 == 0)
                    std::fprintf(csv, "%d,%.4f,%.6f,%.6f,%.6f,%.6f\n", s, i * DT,
                                 e.total(), e.kinetic, e.rotational, e.magnetic);
            }
            float rel = finite ? maxGain / Uscale : INFINITY;
            if (s == 8) err8 = rel;
            char row[48];
            std::snprintf(row, sizeof row, " %d:%.2f%%", s, rel * 100.0f);
            table += row;
        }
        if (csv) std::fclose(csv);
        std::snprintf(buf, sizeof buf, "gain:%s", table.c_str());
        check("T3b no energy injection (8: <1%)", err8 < 0.01f, buf);
    }

    // ---------------- T4. 鎖と輪の静的エネルギー（付録 A.1） ----------------
    {
        float worst = 0.0f;
        for (int N = 2; N <= 12; ++N) {
            for (int ring = 0; ring < 2; ++ring) {
                if (ring && N < 3) continue;
                World w;
                for (int k = 0; k < N; ++k) {
                    if (!ring) {
                        addMagnet(w, {(float)k, 0, 0}, {1, 0, 0}, 1.0f, 0.5f, 1.0f);
                    } else {
                        float th = 2.0f * PHYS_PI * k / N;
                        float Rc = 0.5f / std::sin(PHYS_PI / N);   // 1 辺 = d = 1
                        addMagnet(w, {Rc * std::cos(th), Rc * std::sin(th), 0},
                                  {-std::sin(th), std::cos(th), 0}, 1.0f, 0.5f, 1.0f);
                    }
                }
                float ref = ring ? RING_U[N] : CHAIN_U[N];
                worst = std::max(worst, std::fabs(w.magnets().potentialEnergy() - ref) / std::fabs(ref));
            }
        }
        std::snprintf(buf, sizeof buf, "worst relative error %.1e", worst);
        check("T4 chain / ring energies (A.1)", worst < 1e-4f, buf);
    }

    // ---------------- T5. 接触ソルバが磁力を受け止める ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        RigidBody* a = addMagnet(w, {-R_MAG, 0, 0}, {1, 0, 0}, M0);
        RigidBody* b = addMagnet(w, { R_MAG, 0, 0}, {1, 0, 0}, M0);
        simulate(w, 1.0f);

        float impulse = 0.0f;
        for (const Manifold& mf : w.getManifolds())
            if ((mf.a == a && mf.b == b) || (mf.a == b && mf.b == a))
                for (int i = 0; i < mf.count; ++i) impulse += mf.contacts[i].normalImpulse;
        float h    = DT / SUBSTEPS;
        float r    = length(b->position - a->position);
        float Fsol = impulse / h;
        float Fexp = 6.0f * M0 * M0 / (r * r * r * r);
        float d    = 2.0f * R_MAG;
        float Fd   = 6.0f * M0 * M0 / (d * d * d * d);
        float err  = std::fabs(Fsol - Fexp) / Fexp;
        std::snprintf(buf, sizeof buf, "solver %.1f vs 6m^2/r^4 %.1f (r=%.4f, at d: %.1f) err %.2f%%",
                      Fsol, Fexp, r, Fd, err * 100.0f);
        check("T5 contact holds magnetic force", err < 0.02f, buf);
    }

    // ---------------- T6. 円弧は鎖になるか輪になるか（記録のみ） ----------------
    {
        for (int N = 3; N <= 10; ++N) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            const float d = 2.0f * R_MAG;
            const int   P = N + 1;                              // (N+1) 角形から 1 頂点を抜く
            const float Rc = 0.5f * d / std::sin(PHYS_PI / P);
            for (int k = 0; k < N; ++k) {
                float th = 2.0f * PHYS_PI * k / P;
                addMagnet(w, {Rc * std::cos(th), Rc * std::sin(th), 0},
                          {-std::sin(th), std::cos(th), 0}, M0);
            }
            simulate(w, 5.0f);

            const char* type = "?";
            int size = 0;
            for (const auto& st : w.magneticStructures())
                for (int idx : st.members)
                    if (idx == 0) { type = typeName(st.type); size = (int)st.members.size(); }
            float u0 = M0 * M0 / (d * d * d);
            EnergyReport e = w.energy();
            std::snprintf(buf, sizeof buf, "%s of %d   U=%.3f u0 (chain %.3f, ring %.3f)  K=%.1e u0  contacts=%d",
                          type, size, e.magnetic / u0, CHAIN_U[N], RING_U[N],
                          (e.kinetic + e.rotational) / u0, w.contactCount());
            char name[32];
            std::snprintf(name, sizeof name, "T6 arc N=%d", N);
            info(name, buf);
        }
    }

    // ---------------- I1. 磁石と鉄球（軸上）: F = -12 alpha m^2 / r^7 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float alpha = 0.7f * R_MAG * R_MAG * R_MAG;
        const float r = 2.0f;
        RigidBody* mag  = addMagnet(w, {0, 0, 0}, {1, 0, 0}, 1.0f);
        RigidBody* iron = addIron(w, {r, 0, 0}, R_MAG, M_MAG, alpha);
        w.magnets().updateMoments(3);
        w.magnets().applyForces();
        Vec3  Fexp{-12.0f * alpha / std::pow(r, 7.0f), 0, 0};
        float err  = length(iron->force - Fexp) / length(Fexp);
        float errR = length(iron->force + mag->force) / length(Fexp);
        // fieldAt: 鉄球の中心では自分の場を除いた磁石の場 2m/r^3 になる
        Vec3  Bexp{2.0f / (r * r * r), 0, 0};
        float errB = length(w.magnets().fieldAt(iron->position) - Bexp) / length(Bexp);
        std::snprintf(buf, sizeof buf, "F=%.4e (expect %.4e) err %.1e, action-reaction %.1e, B %.1e",
                      iron->force.x, Fexp.x, err, errR, errB);
        check("I1 induced force on axis", err < 1e-3f && errR < 1e-6f && errB < 1e-5f, buf);
    }

    // ---------------- I2. 誘導モーメントの反復が収束する ----------------
    {
        World w;
        const float rf = 0.03f;
        addMagnet(w, {0, 0, 0}, {1, 0, 0}, M0);
        for (int k = 0; k < 10; ++k)
            addIron(w, {R_MAG + rf + 2.0f * rf * k, 0, 0}, rf, 1e-3f, 0.7f * rf * rf * rf);

        float changes[20];
        for (int it = 0; it < 20; ++it) changes[it] = w.magnets().updateMoments(1);
        float maxM = 0.0f;
        for (const auto& mb : w.magnets().bodies())
            if (mb.chiLocal.x > 0) maxM = std::max(maxM, length(mb.m));
        bool monotone = true;
        for (int it = 1; it < 20; ++it)
            if (changes[it] > changes[it - 1] * 1.0001f && changes[it] > 1e-6f * maxM) monotone = false;
        float finalRel = changes[19] / maxM;
        std::snprintf(buf, sizeof buf, "change/|m|: it1 %.1e  it5 %.1e  it20 %.1e",
                      changes[0] / maxM, changes[4] / maxM, finalRel);
        check("I2 induced moments converge", monotone && finalRel < 1e-5f, buf);
    }

    // ---------------- I3. 誘導を含むエネルギー U = -1/2 sum p.B と力の整合 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const float alpha = 0.7f * R_MAG * R_MAG * R_MAG;
        addMagnet(w, {0, 0, 0}, {1.0f, 0.3f, 0.0f}, 1.0f);
        RigidBody* A = addIron(w, {1.0f, 0.3f, 0.0f}, R_MAG, M_MAG, alpha);
        addIron(w, {1.2f, -0.4f, 0.2f}, R_MAG, M_MAG, alpha);

        const Vec3 xA = A->position;
        const Vec3 e  = normalize(Vec3{1, 1, 1});
        auto energyAt = [&](const Vec3& p) {
            A->position = p;
            w.magnets().updateMoments(60);
            return w.magnets().potentialEnergy();
        };
        const float h = 1e-3f;
        float Fnum = -(energyAt(xA + e * h) - energyAt(xA - e * h)) / (2 * h);

        A->position = xA;
        w.magnets().updateMoments(60);
        for (const auto& b : w.getBodies()) { b->force = Vec3{0, 0, 0}; b->torque = Vec3{0, 0, 0}; }
        w.magnets().applyForces();
        float Fe  = dot(A->force, e);
        float err = std::fabs(Fe - Fnum) / length(A->force);
        std::snprintf(buf, sizeof buf, "F.e=%.5e  -dU/ds=%.5e  err %.1e", Fe, Fnum, err);
        check("I3 energy consistent with induced force", err < 1e-3f, buf);
    }

    // ---------------- I4. 砂鉄つきでも運動量が保存する ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        addMagnet(w, {0, 0, 0}, {1, 0, 0}, M0);

        const float rf    = 0.03f;
        const float ratio = std::pow(rf / R_MAG, 3.0f);
        const float mf    = M_MAG * ratio;
        const float alpha = 0.7f * rf * rf * rf;
        const float sat   = 0.5f * M0 * ratio;
        std::vector<Vec3> placed;
        while ((int)placed.size() < 50) {
            Vec3 p = randomDir() * (0.5f + 0.7f * uni(rng));
            bool ok = true;
            for (const Vec3& q : placed) if (length(p - q) < 0.08f) { ok = false; break; }
            if (!ok) continue;
            placed.push_back(p);
            addIron(w, p, rf, mf, alpha, sat);
        }
        w.magnets().updateMoments(5);

        Vec3  P0 = linearMomentum(w);
        float scale = 1e-9f;
        bool  finite = true;
        int steps = (int)std::lround(5.0f / DT);
        w.substeps = SUBSTEPS;
        for (int i = 0; i < steps; ++i) {
            w.step(DT);
            float s = 0.0f;
            for (const auto& b : w.getBodies()) {
                if (!std::isfinite(b->position.x)) finite = false;
                s += length(b->velocity) / b->invMass;
            }
            scale = std::max(scale, s);
        }
        float err = length(linearMomentum(w) - P0) / scale;
        std::snprintf(buf, sizeof buf, "|dP| / max sum|p| = %.1e", err);
        check("I4 momentum with induced grains", finite && err < 1e-4f, buf);
    }

    // ---------------- X1. 反磁場係数と誘導係数 ----------------
    {
        Vec3  Nc = demagFactors({0.5f, 0.5f, 0.5f});
        float worstSum = 0.0f;
        for (int k = 0; k < 20; ++k) {
            Vec3 he{0.05f + uni(rng), 0.05f + uni(rng), 0.05f + uni(rng)};
            Vec3 N = demagFactors(he);
            worstSum = std::max(worstSum, std::fabs(N.x + N.y + N.z - 1.0f));
        }
        // 細長い棒は長い向きに磁化しやすい。chi -> 無限大 で球は r^3、箱は V / (4 pi N)
        Vec3  aBar   = boxSusceptibility({0.05f, 0.4f, 0.05f}, 1e6f);
        float aSph   = sphereSusceptibility(0.5f, 1e6f);
        Vec3  Nbar   = demagFactors({0.05f, 0.4f, 0.05f});
        float Vbar   = 8.0f * 0.05f * 0.4f * 0.05f;
        float errBar = std::fabs(aBar.y - Vbar / (4.0f * PHYS_PI * Nbar.y)) / aBar.y;
        std::snprintf(buf, sizeof buf, "cube N=(%.4f,%.4f,%.4f) |sum-1|<%.1e  bar alpha long/short %.1f  sphere %.4f",
                      Nc.x, Nc.y, Nc.z, worstSum, aBar.y / aBar.x, aSph);
        check("X1 demag factors & susceptibility",
              std::fabs(Nc.x - 1.0f / 3) < 1e-4f && std::fabs(Nc.z - 1.0f / 3) < 1e-4f && worstSum < 1e-4f &&
              errBar < 1e-3f && aBar.y > 5.0f * aBar.x && std::fabs(aSph - 0.125f) < 1e-5f, buf);
    }

    // ---------------- X2. 立方体の磁石どうしの力（厳密な表面磁荷モデルと比較） ----------------
    //   辺 1、モーメント 1 の立方体 2 個。参照値は一様磁化の直方体の表面磁荷を
    //   数値積分したもの（tools/cube_reference.py）。単位 K m^2 / a^4
    {
        struct Case { const char* name; Vec3 pos; float sign; Vec3 ref; };
        const Case cases[] = {
            {"coaxial touch", {0, 0, 1.0f}, 1.0f, {0, 0, -5.113f}},
            {"side parallel", {1.0f, 0, 0}, 1.0f, {2.556f, 0, 0}},
            {"side antipar.", {1.0f, 0, 0}, -1.0f, {-2.556f, 0, 0}},
            {"slide 1/4", {0.25f, 0, 1.0f}, 1.0f, {-1.515f, 0, -3.582f}},
        };
        std::string table;
        float worstAxis = 0.0f, worstSlide = 0.0f, worstMom = 0.0f;
        for (const Case& c : cases) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            RigidBody* a = addMagnetBox(w, {0, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f, {0, 0, 1.0f});
            RigidBody* b = addMagnetBox(w, c.pos, {0.5f, 0.5f, 0.5f}, 1.0f, {0, 0, c.sign});
            w.magnets().applyForces();
            float err = length(b->force - c.ref) / length(c.ref);
            worstMom  = std::max(worstMom, length(a->force + b->force) / length(c.ref));
            if (c.pos.x == 0.25f) worstSlide = std::max(worstSlide, err);
            else                  worstAxis  = std::max(worstAxis, err);
            char row[64];
            std::snprintf(row, sizeof row, " %s %.1f%%", c.name, err * 100.0f);
            table += row;
        }
        std::snprintf(buf, sizeof buf, "%s", table.c_str());
        check("X2 cube-cube force vs exact (axis <3%)", worstAxis < 0.03f && worstSlide < 0.15f &&
                                                     worstMom < 1e-6f, buf);
    }

    // ---------------- X3. 近い・遠いの切り替えでの力の跳び ----------------
    //   立方体は中心の双極子とのずれが (a/r)^4 で小さいので跳びもほぼない。
    //   細長い箱は (L/r)^2 のずれが残る（記録のみ。接触力に比べれば 1e-4 以下）
    {
        auto jump = [&](const Vec3& he) {
            float worst = 0.0f;
            for (int k = 0; k < 20; ++k) {
                Vec3 dir = randomDir();
                Quat qa  = Quat::fromAxisAngle(randomDir(), uni(rng) * PHYS_PI);
                Quat qb  = Quat::fromAxisAngle(randomDir(), uni(rng) * PHYS_PI);
                Vec3  F[2];
                float scale = 1.0f;
                for (int side = 0; side < 2; ++side) {
                    World w;
                    w.gravity = Vec3{0, 0, 0};
                    addMagnetBox(w, {0, 0, 0}, he, 1.0f, {0, 1.0f, 0}, qa);
                    float rs = w.magnets().nearScale * 2.0f * w.magnets().bodies()[0].boundRadius;   // 切り替えの距離
                    RigidBody* b = addMagnetBox(w, dir * (rs * (side ? 1.0005f : 0.9995f)), he, 1.0f, {0, 1.0f, 0}, qb);
                    w.magnets().applyForces();
                    F[side] = b->force;
                    scale   = 3.0f / (rs * rs * rs * rs);   // 力の大きさの目安 3 m^2 / r^4（向きで打ち消すことがある）
                }
                worst = std::max(worst, length(F[1] - F[0]) / scale);
            }
            return worst;
        };
        float cube = jump({0.25f, 0.25f, 0.25f});
        float bar  = jump({0.5f, 0.3f, 0.2f});
        std::snprintf(buf, sizeof buf, "worst |F_far - F_near| / (3m^2/r^4): cube %.2f%%  (1.0x0.6x0.4 box %.1f%%)",
                      cube * 100.0f, bar * 100.0f);
        check("X3 near / far switch is smooth (cube <1%)", cube < 0.01f, buf);
    }

    // ---------------- X4. 箱を含む運動量・角運動量の保存（接触なし） ----------------
    //   非対称な箱の自由回転は、磁力がなくても積分器（ジャイロ項）の誤差で L が少しずれる。
    //   同じ初期状態で磁性を消した場合と比べ、磁力が加えたずれだけを見る
    {
        auto run = [&](bool magnetic, int substeps, float& errP, float& errL, int& contacts) {
            World w;
            w.gravity = Vec3{0, 0, 0};
            RigidBody* a = addMagnetBox(w, {-1.4f, 0, 0}, {0.3f, 0.2f, 0.15f}, 1.0f, {0, 0.8f, 0},
                                        Quat::fromAxisAngle(normalize(Vec3{1, 2, 3}), 0.7f));
            RigidBody* b = addMagnetBox(w, {1.4f, 0.3f, 0}, {0.25f, 0.25f, 0.25f}, 1.0f, {0.6f, 0, 0},
                                        Quat::fromAxisAngle(normalize(Vec3{-2, 1, 0}), 1.1f));
            const Vec3 heI{0.08f, 0.35f, 0.08f};
            RigidBody* c = w.createSoftMagnetBox({0, 1.3f, 0.4f}, heI, 0.3f, boxSusceptibility(heI, 1000.0f), 0.0f,
                                                 Quat::fromAxisAngle({0, 0, 1}, 0.9f));
            c->linearDamping = c->angularDamping = 0.0f;
            a->angularVelocity = Vec3{0.5f, 1.0f, -0.3f};
            b->angularVelocity = Vec3{-0.4f, 0.2f, 0.8f};
            if (!magnetic)
                for (MagneticBody& mb : w.magnets().bodies()) mb.permanentLocal = mb.chiLocal = Vec3{0, 0, 0};
            w.magnets().updateMoments(10);

            Vec3 P0 = linearMomentum(w), L0 = angularMomentumGeneral(w);
            contacts = 0;
            w.substeps = substeps;
            for (int i = 0; i < (int)std::lround(3.0f / DT); ++i) {
                w.step(DT);
                contacts = std::max(contacts, w.contactCount());
            }
            errP = length(linearMomentum(w) - P0);
            errL = length(angularMomentumGeneral(w) - L0) / length(L0);
        };
        float errP, errL, errP0, errL0, errP32, errL32;
        int contacts, contacts0, contacts32;
        run(true, SUBSTEPS, errP, errL, contacts);
        run(false, SUBSTEPS, errP0, errL0, contacts0);
        run(true, 4 * SUBSTEPS, errP32, errL32, contacts32);

        // 1 回の力の評価では、原点まわりの全トルク sum (x x F + tau) が丸め誤差の範囲で 0
        float netTorque, torqueScale;
        {
            World w;
            addMagnetBox(w, {-0.3f, 0, 0}, {0.3f, 0.2f, 0.15f}, 1.0f, {0, 0.8f, 0},
                         Quat::fromAxisAngle(normalize(Vec3{1, 2, 3}), 0.7f));
            addMagnetBox(w, {0.35f, 0.3f, 0}, {0.25f, 0.25f, 0.25f}, 1.0f, {0.6f, 0, 0},
                         Quat::fromAxisAngle(normalize(Vec3{-2, 1, 0}), 1.1f));
            w.magnets().applyForces();
            Vec3 T{0, 0, 0};
            torqueScale = 0.0f;
            for (const auto& b : w.getBodies()) {
                T += cross(b->position, b->force) + b->torque;
                torqueScale = std::max(torqueScale, length(b->torque));
            }
            netTorque = length(T) / torqueScale;
        }
        std::snprintf(buf, sizeof buf, "net torque %.1e  dP=%.1e  dL/L: %d sub %.1e, %d sub %.1e (no magnetism %.1e)",
                      netTorque, errP, SUBSTEPS, errL, 4 * SUBSTEPS, errL32, errL0);
        // 残る dL は積分の誤差（刻みを 1/4 にすると減る）
        check("X4 momentum & ang. momentum with boxes",
              netTorque < 1e-5f && errP < 1e-4f && errL < 5e-3f && errL32 < 0.5f * errL && contacts == 0, buf);
    }

    // ---------------- X5. 一様な外部磁場: 力 0、トルク m x B0、誘導 m = alpha B0 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const Vec3 B0{0.0f, 2.0f, 0.5f};
        w.magnets().externalField = B0;
        RigidBody* mag  = addMagnet(w, {-5.0f, 0, 0}, {1, 0, 0}, 1.0f);
        const float alpha = sphereSusceptibility(R_MAG, 7.0f);
        RigidBody* iron = addIron(w, {5.0f, 0, 0}, R_MAG, M_MAG, alpha);
        w.magnets().updateMoments(5);
        clearForces(w);
        w.magnets().applyForces();
        // 2 つは 10 離してあるので、互いの場は B0 の 1e-3 程度
        const Vec3& mi = w.magnets().bodies()[1].m;
        float errT = length(mag->torque - cross(Vec3{1, 0, 0}, B0)) / length(cross(Vec3{1, 0, 0}, B0));
        float errM = length(mi - B0 * alpha) / length(B0 * alpha);
        float Fsum = length(mag->force) + length(iron->force);
        std::snprintf(buf, sizeof buf, "torque vs m x B0 %.1e  induced m vs alpha B0 %.1e  |F| %.1e (mutual only)",
                      errT, errM, Fsum);
        check("X5 uniform field: torque & induction", errT < 1e-3f && errM < 2e-3f && Fsum < 1e-3f, buf);
    }

    // ---------------- X6. 外部磁場中の方位磁針（エネルギー保存） ----------------
    //   磁石球を磁場から 60 度傾けて放す。振り子と同じで、U = -m.B0 と回転エネルギーの和が一定
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.magnets().externalField = Vec3{0.5f, 0, 0};   // 周期 0.4 s（刻み 1/960 s で 1 周 400 ステップ）
        addMagnet(w, {0, 0, 0}, {std::cos(1.047f), std::sin(1.047f), 0}, M0);
        float E0 = w.energy().total(), maxDev = 0.0f, Urange = 0.0f;
        float Umin = 1e9f, Umax = -1e9f;
        w.substeps = SUBSTEPS;
        for (int i = 0; i < (int)std::lround(10.0f / DT); ++i) {
            w.step(DT);
            EnergyReport e = w.energy();
            maxDev = std::max(maxDev, std::fabs(e.total() - E0));
            Umin = std::min(Umin, e.magnetic);
            Umax = std::max(Umax, e.magnetic);
        }
        Urange = Umax - Umin;
        std::snprintf(buf, sizeof buf, "max |E - E0| / swing of U = %.2f%%  (U swings %.1f)",
                      maxDev / Urange * 100.0f, Urange);
        check("X6 compass in uniform field conserves E", maxDev / Urange < 0.01f && Urange > 0.5f, buf);
    }

    // ---------------- X7. 鉄の棒は長い向きを外部磁場にそろえる ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        const Vec3 B0{3.0f, 0, 0};
        w.magnets().externalField = B0;
        const Vec3 he{0.05f, 0.4f, 0.05f};
        RigidBody* bar = w.createSoftMagnetBox({0, 0, 0}, he, 0.1f, boxSusceptibility(he, 1000.0f), 0.0f,
                                               Quat::fromAxisAngle({0, 0, 1}, 0.785f));   // 長い軸 (+y) を 45 度
        bar->angularDamping = 2.0f;
        simulate(w, 10.0f);
        Vec3  axis  = bar->orientation.rotate(Vec3{0, 1, 0});
        float angle = std::acos(std::min(1.0f, std::fabs(dot(axis, normalize(B0))))) * 180.0f / PHYS_PI;
        std::snprintf(buf, sizeof buf, "angle between bar and field after 10 s: %.2f deg (start 45)", angle);
        check("X7 iron bar aligns with field", angle < 3.0f, buf);
    }

    // ---------------- X8. 外部磁場と誘導を含むエネルギーと力の整合 ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        w.magnets().externalField = Vec3{0.5f, -0.3f, 0.8f};
        addMagnet(w, {0, 0, 0}, {1.0f, 0.3f, 0.0f}, 1.0f);
        const Vec3 he{0.3f, 0.2f, 0.25f};
        RigidBody* A = w.createSoftMagnetBox({1.3f, 0.4f, 0.1f}, he, 1.0f, boxSusceptibility(he, 50.0f), 0.0f,
                                             Quat::fromAxisAngle(normalize(Vec3{1, 1, 0}), 0.4f));
        addIron(w, {1.2f, -0.9f, 0.3f}, R_MAG, M_MAG, sphereSusceptibility(R_MAG, 7.0f));

        const Vec3 xA = A->position;
        const Vec3 e  = normalize(Vec3{1, -1, 2});
        auto energyAt = [&](const Vec3& p) {
            A->position = p;
            w.magnets().updateMoments(80);
            return w.magnets().potentialEnergy();
        };
        const float h = 1e-3f;
        float Fnum = -(energyAt(xA + e * h) - energyAt(xA - e * h)) / (2 * h);
        A->position = xA;
        w.magnets().updateMoments(80);
        clearForces(w);
        w.magnets().applyForces();
        float Fe  = dot(A->force, e);
        float err = std::fabs(Fe - Fnum) / length(A->force);
        std::snprintf(buf, sizeof buf, "F.e=%.5e  -dU/ds=%.5e  err %.1e", Fe, Fnum, err);
        check("X8 energy vs force (box, field, induced)", err < 2e-3f, buf);
    }

    // ---------------- P1. 位置を固定した磁石（方位磁針）は動かずに磁場へ向く ----------------
    {
        World w;
        w.substeps = 4;
        w.magnets().externalField = Vec3{3.0f, 0, 0};
        RigidBody* a = addMagnet(w, {0, 1.0f, 0}, {0, 1, 0.2f}, M0);
        RigidBody* b = addMagnet(w, {0.6f, 1.0f, 0}, {0, -1, 0}, M0);   // 近くの磁石からも力を受ける
        a->angularDamping = b->angularDamping = 1.0f;
        w.setPinned(a, true);
        const Vec3 p0 = a->position;
        simulate(w, 5.0f, 4);
        Vec3  m     = w.magnets().bodies()[0].m;
        float moved = length(a->position - p0);
        float angle = std::acos(std::min(1.0f, dot(normalize(m), normalize(w.magnets().bodies()[0].B)))) * 180.0f / PHYS_PI;
        std::snprintf(buf, sizeof buf, "moved %.1e   angle to local field %.2f deg   free magnet moved %.2f",
                      moved, angle, length(b->position - Vec3{0.6f, 1.0f, 0}));
        check("P1 pinned magnet only rotates", moved < 1e-6f && angle < 2.0f, buf);
    }

    // ---------------- L1. 浮かぶ磁石の塔のつり合い（シーン 0 と同じ構成） ----------------
    //   摩擦 0 の筒に立方体の磁石 6 個を交互の向きで入れ、20 秒後の高さを点双極子の
    //   つり合い（levitation_theory.h、全ペア）と比べる
    {
        const int   N    = 6;
        const float half = 0.25f, V = 0.125f;
        const float sphereVol = 4.0f / 3.0f * PHYS_PI * R_MAG * R_MAG * R_MAG;
        const float m = M0 * V / sphereVol, mass = M_MAG * V / sphereVol;
        std::vector<float> mu(N);
        for (int k = 0; k < N; ++k) mu[k] = (k % 2 == 0) ? m : -m;
        std::vector<float> yt = demo::dipoleStackEquilibrium(mu, mass, G_ACC, half);

        World w;
        w.substeps = SUBSTEPS;
        w.createBox({0, -1.0f, 0}, {30, 1, 30}, 0.0f);
        const float in = half + 0.02f, t = 0.05f, h = 0.5f * (yt.back() + 3.0f);
        for (int s = -1; s <= 1; s += 2) {
            RigidBody* wx = w.createBox({s * (in + t), h, 0}, {t, h, in + 2 * t}, 0.0f);
            RigidBody* wz = w.createBox({0, h, s * (in + t)}, {in, h, t}, 0.0f);
            wx->friction = wz->friction = 0.0f;
        }
        std::vector<RigidBody*> cubes;
        for (int k = 0; k < N; ++k) {
            RigidBody* c = w.createMagnetBox({0, half + (yt[k] - half) * 0.95f, 0}, {half, half, half}, mass,
                                             {0, mu[k], 0});
            c->friction = 0.4f; c->restitution = 0.2f; c->linearDamping = 0.5f;   // 振動を止めてから比べる
            cubes.push_back(c);
        }
        simulate(w, 20.0f);
        float worst = 0.0f;
        std::string table;
        for (int k = 1; k < N; ++k) {
            float d = (cubes[k]->position.y - yt[k]) / (yt[k] - half);
            worst = std::max(worst, std::fabs(d));
            char row[32];
            std::snprintf(row, sizeof row, " %.2f/%.2f", cubes[k]->position.y, yt[k]);
            table += row;
        }
        std::snprintf(buf, sizeof buf, "sim/theory:%s  worst %.2f%%", table.c_str(), worst * 100.0f);
        check("L1 levitating tower matches theory (<1%)", worst < 0.01f, buf);
    }

    // ---------------- R1. 回転する一様な磁場で磁石が転がる ----------------
    //   磁場を x-y 面の中で x → y の向きに回すと、磁石は z 軸まわりに回り、-x へ転がる。
    //   滑らずに転がるなら速さは 2 pi f R
    {
        World w;
        w.substeps = SUBSTEPS;
        RigidBody* g = w.createBox({0, -1.0f, 0}, {30, 1, 30}, 0.0f);
        g->friction = 0.7f;
        RigidBody* a = addMagnet(w, {0, R_MAG, 0}, {1, 0, 0}, M0);
        a->friction = 0.3f; a->rollingFriction = 0.01f;
        const float f = 0.1f, B0 = 5.0f;   // 実時間の 1 Hz
        const int   steps = (int)std::lround(10.0f / DT);
        for (int i = 1; i <= steps; ++i) {
            float ph = 2.0f * PHYS_PI * f * i * DT;
            w.magnets().externalField = Vec3{std::cos(ph), std::sin(ph), 0} * B0;
            w.step(DT);
        }
        float ideal = 2.0f * PHYS_PI * f * R_MAG * 10.0f;
        float ratio = -a->position.x / ideal;
        std::snprintf(buf, sizeof buf, "moved x %.3f (ideal rolling %.3f, ratio %.3f)  z %.1e",
                      a->position.x, -ideal, ratio, a->position.z);
        check("R1 magnet rolls in a rotating field", ratio > 0.9f && ratio < 1.05f && std::fabs(a->position.z) < 1e-3f, buf);
    }

    // ---------------- G1. ガウス加速器（シーン D と同じ組み立て） ----------------
    //   シーンで Enter を押したときと同じく、組み立ててすぐ最初の球を転がし、段と段の中間を
    //   前向きに通った球の速さを測る。鉄球 3 個の段は、今のソルバで全段に球を送る。
    //   鉄球 1 個の段は、入る球と出る球が磁石から同じ距離なので、出るエネルギーが 0
    {
        auto runGauss = [](int balls, demo::GaussMeter& meter) {
            World w;
            RigidBody* g = w.createBox({0, -1.0f, 0}, {30.0f, 1.0f, 30.0f}, 0.0f);
            g->friction = 0.7f; g->restitution = 0.1f;
            w.substeps = demo::MAG_SUBSTEPS;
            demo::GaussParams p;
            p.balls = balls;
            demo::GaussRail r = demo::buildGaussRail(w, p);
            r.launch->velocity = Vec3{2.0f, 0, 0};
            meter.reset(p);
            const int steps = (int)std::lround(20.0f / DT);
            for (int i = 0; i < steps; ++i) {
                meter.update(w, p);
                w.step(DT);
            }
            return p.stages + 1;
        };
        std::string rows;
        bool ok3 = false;
        float e1 = demo::gaussStageEnergy(1), e3 = demo::gaussStageEnergy(3);
        for (int balls = 1; balls <= 4; ++balls) {
            demo::GaussMeter meter;
            const int points = runGauss(balls, meter);
            char row[128];
            int  n = std::snprintf(row, sizeof row, " n=%d:", balls);
            for (float v : meter.speeds) n += std::snprintf(row + n, sizeof row - n, " %.1f", v);
            rows += row;
            if (balls == 3) ok3 = meter.passed() == points && meter.speeds.back() > meter.speeds.front();
        }
        std::snprintf(buf, sizeof buf, "dE(n=1) %.3f  dE(n=3) %.2f  speeds at checkpoints%s", e1, e3, rows.c_str());
        check("G1 Gauss accelerator (3 balls: all stages)", ok3 && e3 > 0.0f && std::fabs(e1) < 0.05f * e3, buf);
    }

    // ---------------- F1. 磁力線（シーン E） ----------------
    //   磁石 1 個の磁力線は r = L sin^2(theta)（theta はモーメントからの角度）。たどった線の各点で
    //   L が一定かを見る。本数は表面を出る磁束 2 pi m / a に比例（ここでは 20 本になるように設定）。
    //   範囲の外に出なかった線は、磁石の S 極側に入って閉じる
    {
        World w;
        RigidBody* mag = addMagnet(w, {0, 0, 0}, {0, 0, 1}, M0);
        mag->setMass(0.0f);
        w.magnets().updateMoments(1);
        demo::FieldLineTracer tr;
        tr.fluxPerLine = 2.0f * PHYS_PI * M0 / (1.01f * R_MAG) / 20.0f;
        tr.traceAll(w);
        float worst = 0.0f;
        int   closed = 0, open = 0, wrongEnd = 0;
        for (const demo::FieldLine& l : tr.lines()) {
            auto Lof = [](const Vec3& p) {
                const float r = length(p), s2 = (p.x * p.x + p.y * p.y) / (r * r);
                return r / s2;
            };
            const float L0 = Lof(l.pts.front());
            for (size_t k = 0; k + 1 < l.pts.size(); ++k)   // 最後の点は磁石の中
                worst = std::max(worst, std::fabs(Lof(l.pts[k]) / L0 - 1.0f));
            const Vec3& e = l.pts.back();
            if (length(e) < R_MAG) { ++closed; if (e.z > 0.0f) ++wrongEnd; }
            else ++open;
        }
        std::snprintf(buf, sizeof buf, "lines %d (closed %d, leave region %d)  max |L/L0 - 1| %.2e  end on N side %d",
                      (int)tr.lines().size(), closed, open, worst, wrongEnd);
        check("F1 dipole field lines r = L sin^2(theta)", worst < 0.01f && wrongEnd == 0 &&
              std::abs((int)tr.lines().size() - 20) <= 1 && closed >= 10, buf);
    }

    // ---------------- S1. 床に落とした磁石が静止するか（記録のみ。デモのシーン 5 と 8） ----------------
    {
        const float sphereVol = 4.0f / 3.0f * PHYS_PI * R_MAG * R_MAG * R_MAG;
        for (int cubes = 0; cubes < 2; ++cubes) {
            World w;
            w.substeps = SUBSTEPS;
            RigidBody* ground = w.createBox({0, -1.0f, 0}, {30.0f, 1.0f, 30.0f}, 0.0f);
            ground->friction = 0.7f; ground->restitution = 0.1f;
            std::mt19937 rs(8);
            std::uniform_real_distribution<float> u01(0.0f, 1.0f);
            std::vector<Vec3> placed;
            while ((int)placed.size() < 27) {
                Vec3 p{(u01(rs) - 0.5f) * 5.0f, 1.2f + u01(rs) * 3.5f, (u01(rs) - 0.5f) * 5.0f};
                bool ok = true;
                for (const Vec3& q : placed) if (length(p - q) < 1.0f) { ok = false; break; }
                if (!ok) continue;
                placed.push_back(p);
                Vec3 d = normalize(Vec3{u01(rs) - 0.5f, u01(rs) - 0.5f, u01(rs) - 0.5f});
                RigidBody* b;
                if (cubes) {
                    const float V = 0.125f;
                    b = w.createMagnetBox(p, {0.25f, 0.25f, 0.25f}, M_MAG * V / sphereVol,
                                          {0, M0 * V / sphereVol, 0}, Quat::fromAxisAngle(d, u01(rs) * 6.28f));
                    b->restitution = 0.2f; b->friction = 0.4f;
                } else {
                    b = w.createMagnet(p, R_MAG, M_MAG, d * M0);
                    b->restitution = 0.3f; b->friction = 0.3f; b->rollingFriction = 0.01f;
                }
            }
            std::string table;
            float t = 0.0f;
            for (float mark : {2.0f, 5.0f, 10.0f, 15.0f}) {
                while (t < mark - 1e-4f) { w.step(DT); t += DT; }
                float K = 0.0f, Kmax = 0.0f;
                for (const auto& b : w.getBodies()) {
                    if (b->isStatic()) continue;
                    float M = 1.0f / b->invMass;
                    Vec3 wl = b->orientation.toMat3().transposed() * b->angularVelocity;
                    float k = 0.5f * M * lengthSq(b->velocity) +
                              0.5f * (wl.x * wl.x / b->invInertiaLocal.x + wl.y * wl.y / b->invInertiaLocal.y +
                                      wl.z * wl.z / b->invInertiaLocal.z);
                    K += k; Kmax = std::max(Kmax, k);
                }
                char row[64];
                std::snprintf(row, sizeof row, " t=%.0f K=%.3g(max %.2g)", mark, K, Kmax);
                table += row;
            }
            info(cubes ? "S1 27 cube magnets settle" : "S1 27 sphere magnets settle", table.c_str());
        }
    }

    // ---------------- B1. 総当たりのコスト（記録のみ） ----------------
    {
        const int Ns[] = {64, 202, 302, 502};
        for (int N : Ns) {
            World w;
            if (N == 64) {
                for (int k = 0; k < 64; ++k)
                    addMagnet(w, {(k % 4) * 0.6f, ((k / 4) % 4) * 0.6f, (k / 16) * 0.6f},
                              randomDir(), M0);
            } else {
                addMagnet(w, {-0.4f, 0, 0}, {1, 0, 0}, M0);
                addMagnet(w, { 0.4f, 0, 0}, {1, 0, 0}, M0);
                for (int k = 0; k < N - 2; ++k)
                    addIron(w, {(k % 20) * 0.07f - 0.7f, 0.4f + ((k / 20) % 5) * 0.07f,
                                    (k / 100) * 0.07f}, 0.03f, 1e-3f, 1e-5f, 1e-3f);
            }
            const double pairs = N * (N - 1) / 2.0;
            const int iters = std::max(20, std::min(2000, (int)(2e7 / pairs)));
            w.magnets().applyForces();
            auto t0 = std::chrono::steady_clock::now();
            for (int it = 0; it < iters; ++it) w.magnets().applyForces();
            auto t1 = std::chrono::steady_clock::now();
            double sec    = std::chrono::duration<double>(t1 - t0).count();
            double nsPair = sec * 1e9 / (iters * pairs);
            double msFrame = 16.0 * pairs * nsPair * 1e-6;   // 60 FPS: 2 step x 8 substeps
            std::snprintf(buf, sizeof buf, "%.0f pairs  %.1f ns/pair  -> %.2f ms/frame",
                          pairs, nsPair, msFrame);
            char name[32];
            std::snprintf(name, sizeof name, "B1 applyForces N=%d", N);
            info(name, buf);
        }
    }

    // ---------------- B2. 立方体の磁石が密集したときのコスト（記録のみ） ----------------
    {
        const int Ns[] = {27, 64};
        for (int N : Ns) {
            World w;
            const int side = (N == 27) ? 3 : 4;
            for (int k = 0; k < N; ++k)
                addMagnetBox(w, {(k % side) * 0.55f, ((k / side) % side) * 0.55f, (k / (side * side)) * 0.55f},
                             {0.25f, 0.25f, 0.25f}, M_MAG, randomDir() * M0);
            const int iters = 200;
            w.magnets().applyForces();
            auto t0 = std::chrono::steady_clock::now();
            for (int it = 0; it < iters; ++it) w.magnets().applyForces();
            auto t1 = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double>(t1 - t0).count() * 1e3 / iters;
            std::snprintf(buf, sizeof buf, "%.3f ms/call  -> %.2f ms/frame", ms, 16.0 * ms);
            char name[32];
            std::snprintf(name, sizeof name, "B2 cube cluster N=%d", N);
            info(name, buf);
        }
    }

    std::printf("\n%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
