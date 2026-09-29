// 磁石・砂鉄のヘッドレス検証（raylib 不要）
//   magnet_test [--csv <dir>]   --csv を付けると T3 のエネルギー時系列を CSV で出す
#include "phys/world.h"

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

    std::printf("\n%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
