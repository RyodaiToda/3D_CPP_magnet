// ---------------------------------------------------------------------------
// electric_scenes.cpp : 電気を見せるシーン（electric_scenes.h）
// ---------------------------------------------------------------------------
#include "electric_scenes.h"

#include "raymath.h"

#include <cmath>
#include <cstdio>

using namespace phys;

namespace {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }

constexpr float G_ACC = 9.81f;
const Color COL_POS    {255, 160, 40, 255};
const Color COL_NEG    {160, 95, 255, 255};
const Color COL_COPPER {200, 125, 80, 255};

// 軌跡（物体の位置を毎ステップ足す。古い点から捨てる）
struct Trail {
    RigidBody* body = nullptr;
    std::vector<Vec3> pts;
    float q = 0.0f;
    void push(size_t maxN) {
        pts.push_back(body->position);
        if (pts.size() > maxN) pts.erase(pts.begin(), pts.begin() + (pts.size() - maxN));
    }
    void draw() const {
        const Color c = q >= 0.0f ? COL_POS : COL_NEG;
        for (size_t i = 1; i < pts.size(); ++i)
            DrawLine3D(toRay(pts[i - 1]), toRay(pts[i]), Fade(c, 0.15f + 0.85f * (float)i / pts.size()));
    }
};

// 矢印（from → to）
void arrow(const Vec3& from, const Vec3& to, float r, Color c) {
    const Vec3  d = to - from;
    const float L = length(d);
    if (L < 1e-4f) return;
    const Vec3 u = d / L, mid = to - u * std::min(0.4f * L, 4.0f * r);
    DrawCylinderEx(toRay(from), toRay(mid), r, r, 5, c);
    DrawCylinderEx(toRay(mid), toRay(to), 2.4f * r, 0.0f, 6, c);
}

RigidBody* ball(World& w, const Vec3& pos, float r, float m, float q, bool conductor) {
    return w.createChargedSphere(pos, r, m, q, conductor);
}

} // namespace

Color chargeColor(float q, float ref, bool conductor) {
    const Color base = conductor ? COL_COPPER : Color{215, 215, 222, 255};
    if (q == 0.0f || ref <= 0.0f) return base;
    const float s = clampf(std::fabs(q) / ref, 0.0f, 1.0f);
    return ColorLerp(base, q > 0.0f ? COL_POS : COL_NEG, 0.35f + 0.65f * s);
}

// ===========================================================================
// F5 電気ベルと帯電球
// ===========================================================================
class BellScene : public FunScene {
public:
    BellScene() { timeScale = 1.0f; }
    const char* title() const override { return "F5: Electric bell (charges move on contact)"; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = 8;
        if (layout == 0) buildDance(host);
        else             buildShare(host);
        wantCameraReset = true;
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_M)) { layout = (layout + 1) % 2; build(host); }
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[200];
        const ElectricSystem& es = w.electric();
        if (layout == 0) {
            int up = 0;
            for (const ElectricBody& eb : es.bodies()) up += eb.body->position.y > plateTop + 0.3f ? 1 : 0;
            std::snprintf(buf, sizeof buf, "plates %+.2f / %+.2f (gap %.1f m): a ball picks up R V on contact, the field throws it across",
                          -V, V, GAP);
            out.add(buf, Color{255, 200, 120, 255});
            std::snprintf(buf, sizeof buf, "%d / %d balls in the air   charge carried across so far %.2f", up, (int)es.bodies().size(),
                          es.plateExchange());
            out.add(buf);
            out.add("upward push on a ball at the bottom plate = 3 x its weight", GRAY, 14);
        } else {
            std::snprintf(buf, sizeof buf, "charged metal ball q = %.2f rolls into neutral metal balls: induced pull, touch, share, repel",
                          q0);
            out.add(buf, Color{255, 200, 120, 255});
            std::snprintf(buf, sizeof buf, "total charge %.3f (kept)", es.totalCharge());
            out.add(buf);
        }
        out.gap(4);
        out.add("M layout (bell / sharing)   R rebuild", GRAY);
    }

private:
    static constexpr float GAP = 1.6f, R = 0.1f, MASS = 0.01f;
    int   layout = 0;
    float V = 0.0f, q0 = 0.0f, plateTop = 0.0f;

    // 上下 2 枚の電極（下 -V、上 +V）と、ガラスの壁。中に導体の小球
    void buildDance(SceneHost& host) {
        World& w = host.world;
        const Vec3 c{0, 0.15f + 0.5f * GAP + 0.1f, 0};
        V = std::sqrt(3.0f * MASS * G_ACC * GAP / (2.0f * R));   // 下の板で -R V をもらうと、上向きに重さの 3 倍
        const int ci = w.createCapacitor(c, Quat{}, 1.4f, 1.4f, GAP, 0.1f, -V, V);
        const Capacitor& cap = w.electric().capacitors()[ci];
        host.paint(cap.plateA, ColorLerp(Color{150, 150, 160, 255}, COL_NEG, 0.45f));
        host.paint(cap.plateB, Fade(ColorLerp(Color{150, 150, 160, 255}, COL_POS, 0.45f), 0.45f));
        plateTop = c.y - 0.5f * GAP;
        const Color glass{200, 220, 255, 40};
        for (int k = 0; k < 4; ++k) {
            const float s = (k % 2 == 0) ? 1.0f : -1.0f;
            const Vec3  pos = k < 2 ? Vec3{1.45f * s, c.y, 0} : Vec3{0, c.y, 1.45f * s};
            const Vec3  he  = k < 2 ? Vec3{0.05f, 0.5f * GAP, 1.5f} : Vec3{1.5f, 0.5f * GAP, 0.05f};
            host.addStatic(pos, he, glass, 0.1f, 0.5f);
        }
        for (int i = 0; i < 14; ++i) {
            const float x = -1.0f + 0.62f * (float)(i % 4) + 0.11f * (float)((i * 7) % 3);
            const float z = -1.0f + 0.62f * (float)(i / 4) + 0.09f * (float)((i * 5) % 3);
            RigidBody* b = ball(w, {x, plateTop + R + 0.001f, z}, R, MASS, 0.0f, true);
            b->restitution = 0.45f;
            b->friction    = 0.3f;
        }
        chargeRef = R * V;
        camTargetX = 0.0f; camTargetY = c.y; camTargetZ = 0.0f;
        camDistance = 6.5f; camPitch = 0.3f; camYaw = 0.6f;
    }

    // 机の上: 帯電した金属球が、帯電していない金属球の群れへ転がる
    void buildShare(SceneHost& host) {
        World& w = host.world;
        const float mBig = 0.3f, rBig = 0.25f, r = 0.2f, m = 0.15f;
        q0 = chargeFromGamma(60.0f, mBig, 2.0f * rBig, G_ACC);   // 同じ球どうしなら、接触でクーロン力が重さの 60 倍
        RigidBody* big = ball(w, {-3.0f, rBig, 0}, rBig, mBig, q0, true);
        big->velocity = Vec3{1.2f, 0, 0};
        const Vec3 spots[5] = {{0.0f, 0, 0.1f}, {0.6f, 0, -0.45f}, {0.7f, 0, 0.55f}, {1.3f, 0, 0.05f}, {1.9f, 0, -0.4f}};
        for (const Vec3& s : spots) ball(w, s + Vec3{0, r, 0}, r, m, 0.0f, true);
        chargeRef = 0.5f * q0;
        camTargetX = 0.0f; camTargetY = 0.3f; camTargetZ = 0.0f;
        camDistance = 8.0f; camPitch = 0.55f; camYaw = 0.2f;
    }
};

// ===========================================================================
// F6 磁場の中の荷電粒子（この世界の 1/c を大きくする）
// ===========================================================================
class LorentzScene : public FunScene {
public:
    LorentzScene() { timeScale = 1.0f; }
    const char* title() const override { return "F6: Charged particles in magnetic fields"; }
    bool ownsField() const override { return true; }

    void build(SceneHost& host) override {
        host.reset();
        World& w = host.world;
        w.gravity  = Vec3{0, 0, 0};
        w.substeps = 8;
        trails.clear();
        auto particle = [&](const Vec3& pos, float q, float m, const Vec3& v) {
            RigidBody* b = ball(w, pos, 0.1f, m, q, false);
            b->velocity = v;
            b->linearDamping = 0.0f;
            b->angularDamping = 0.0f;
            Trail t;
            t.body = b;
            t.q    = q;
            trails.push_back(t);
        };
        // 粒子の電荷は小さく、kappa を大きくする（回り方は同じで、粒子どうしのクーロン力が小さくなる）
        if (mode == 0) {   // 一様な磁場（+y）の中のらせん。電荷の符号で回る向きが逆、電荷が大きいほど小さく回る
            w.magnets().externalField = Vec3{0, B0, 0};
            w.electric().kappa = 15.0f;
            particle({-4.5f, -2.5f, 0}, 0.1f, 1.0f, {0, 0.35f, 3.0f});
            particle({0.0f, -2.5f, 0}, -0.1f, 1.0f, {0, 0.35f, 3.0f});
            particle({4.5f, -2.5f, 0}, 0.2f, 1.0f, {0, 0.35f, 3.0f});
            camTargetY = 0.0f; camDistance = 15.0f; camPitch = 0.25f;
        } else if (mode == 1) {   // 磁気瓶: 同じ向きの 2 つの磁石（静的）の間
            w.createMagnet({0, -3.0f, 0}, 0.5f, 0.0f, {0, 50.0f, 0});
            w.createMagnet({0, 3.0f, 0}, 0.5f, 0.0f, {0, 50.0f, 0});
            w.magnets().updateMoments(10);
            w.electric().kappa = 10.0f;
            particle({0, 0, 0}, 0.1f, 1.0f, {1.5f, 2.5f, 0});
            particle({0.4f, 0, 0.4f}, -0.1f, 1.0f, {-1.2f, -2.2f, 0.8f});
            camTargetY = 0.0f; camDistance = 10.0f; camPitch = 0.15f;
        } else {   // E x B: 板の電場（+z）と一様な磁場（+y）。どの粒子も同じ速さ E / (kappa B) で -x へ流れる
            w.magnets().externalField = Vec3{0, 1.0f, 0};
            w.electric().kappa = 10.0f;
            const Quat q = Quat::fromAxisAngle({1, 0, 0}, 0.5f * PHYS_PI);   // ローカル +y → +z
            const int ci = w.createCapacitor({0, 0, 0}, q, 8.0f, 2.0f, 4.6f, 0.1f, EXB_V, -EXB_V);
            host.paint(w.electric().capacitors()[ci].plateA, Fade(ColorLerp(Color{150, 150, 160, 255}, COL_POS, 0.45f), 0.25f));
            host.paint(w.electric().capacitors()[ci].plateB, Fade(ColorLerp(Color{150, 150, 160, 255}, COL_NEG, 0.45f), 0.25f));
            particle({6.5f, -1.5f, -2.0f}, 0.1f, 1.0f, {0, 0, 0});
            particle({6.5f, 0.0f, -2.0f}, 0.2f, 1.0f, {0, 0, 0});
            particle({6.5f, 1.5f, -2.0f}, 0.3f, 1.0f, {0, 0, 0});
            camTargetY = 0.0f; camDistance = 17.0f; camPitch = 1.15f;   // 動きは x-z の面の中なので、上から見る
        }
        camTargetX = 0.0f; camTargetZ = 0.0f; camYaw = 0.0f;
        chargeRef = 0.2f;
        wantCameraReset = true;
    }

    void beforeStep(World&, float) override {
        for (Trail& t : trails) t.push(mode == 0 ? 2400 : 3000);
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 3; build(host); }
        if (IsKeyPressed(KEY_ENTER)) build(host);
    }

    void draw3D(const World& w) const override {
        for (const Trail& t : trails) t.draw();
        if (mode == 0 || mode == 2) {   // 一様な磁場の向き
            const float B = length(w.magnets().externalField);
            for (int i = -2; i <= 2; ++i)
                for (int k = -1; k <= 1; ++k)
                    arrow(Vec3{2.5f * i, -3.5f, 2.5f * k}, Vec3{2.5f * i, -3.5f + 0.6f + 0.2f * B, 2.5f * k}, 0.02f,
                          Fade(Color{120, 200, 255, 255}, 0.5f));
        }
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[200];
        const float kappa = w.electric().kappa;
        std::snprintf(buf, sizeof buf, "F = q (E + kappa v x B), kappa = %.1f  (this world's 1/c: light at %.2f m/s)", kappa, 1.0f / kappa);
        out.add(buf, Color{255, 200, 120, 255});
        if (mode == 0) {
            out.add("uniform B (+y): helices. radius m v / (kappa q B), period 2 pi m / (kappa q B)");
            for (const Trail& t : trails) {
                const float om = kappa * std::fabs(t.q) * B0 / (1.0f / t.body->invMass);
                std::snprintf(buf, sizeof buf, "  q %+.1f: theory radius %.2f m, period %.2f s   speed now %.3f m/s", t.q,
                              3.0f / om, 2.0f * PHYS_PI / om, length(t.body->velocity));
                out.add(buf, LIGHTGRAY, 14);
            }
        } else if (mode == 1) {
            out.add("magnetic bottle: the field is stronger near each magnet, the particles bounce back");
            for (const Trail& t : trails) {
                const Vec3  B  = ElectricSystem::magneticFieldAt(w.magnets(), t.body->position, t.body->id);
                const float Bl = length(B);
                const Vec3  vp = t.body->velocity - B * (dot(t.body->velocity, B) / std::max(Bl * Bl, 1e-12f));
                std::snprintf(buf, sizeof buf, "  q %+.1f: y %+.2f  |B| %.2f  magnetic moment m v_perp^2 / 2B = %.3f", t.q,
                              t.body->position.y, Bl, lengthSq(vp) / (2.0f * std::max(Bl, 1e-6f)));
                out.add(buf, LIGHTGRAY, 14);
            }
        } else {
            const float E = 2.0f * EXB_V / 4.6f, B = length(w.magnets().externalField);
            std::snprintf(buf, sizeof buf, "E x B drift: every particle drifts at E / (kappa B) = %.2f m/s (-x), whatever q and m", E / (kappa * B));
            out.add(buf);
            for (const Trail& t : trails) {
                std::snprintf(buf, sizeof buf, "  q %+.1f: x %+.2f", t.q, t.body->position.x);
                out.add(buf, LIGHTGRAY, 14);
            }
        }
        out.gap(4);
        out.add("M mode (helix / bottle / E x B)   Enter restart   O magnetic field lines", GRAY);
    }

private:
    static constexpr float B0 = 2.0f, EXB_V = 34.5f;
    int mode = 0;
    std::vector<Trail> trails;
};

// ===========================================================================
// F7 渦電流
// ===========================================================================
class EddyScene : public FunScene {
public:
    EddyScene() { timeScale = 1.0f; }
    const char* title() const override { return "F7: Eddy currents (copper vs plastic)"; }
    bool ownsField() const override { return true; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = 8;
        mags.clear();
        blocks.clear();
        plate = nullptr;
        if (mode == 0)      buildTubes(host);
        else if (mode == 1) buildArago(host);
        else                buildRamp(host);
        wantCameraReset = true;
    }

    void beforeStep(World& w, float dt) override {
        if (mode != 1) return;
        t += dt;   // アラゴ: 磁石を上で回す（静的な物体を動かす）
        const float a = ARAGO_W * t;
        for (size_t i = 0; i < mags.size(); ++i) {
            const float ai = a + PHYS_PI * (float)i;
            mags[i]->position          = Vec3{ARAGO_R * std::cos(ai), 1.45f, -ARAGO_R * std::sin(ai)};
            mags[i]->kinematicVelocity = Vec3{-ARAGO_R * ARAGO_W * std::sin(ai), 0, -ARAGO_R * ARAGO_W * std::cos(ai)};
        }
        (void)w;
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 3; build(host); }
        if (IsKeyPressed(KEY_ENTER)) build(host);
    }

    void draw3D(const World& w) const override {
        // 渦電流（強いところだけ、向きの矢印）
        const auto& s = w.electric().eddySamples();
        float jMax = 0.0f;
        for (const auto& e : s) jMax = std::max(jMax, length(e.J));
        if (jMax <= 0.0f) return;
        const int stride = std::max(1, (int)s.size() / 500);
        for (size_t i = 0; i < s.size(); i += stride) {
            const float j = length(s[i].J);
            if (j < 0.15f * jMax) continue;
            const Vec3 d = s[i].J / j * (0.12f + 0.25f * j / jMax);
            arrow(s[i].x - d * 0.5f, s[i].x + d * 0.5f, 0.012f, Fade(Color{120, 230, 255, 255}, 0.4f + 0.6f * j / jMax));
        }
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[200];
        out.add("moving magnet + conductor: J = sigma (v_rel x B), force J x B brakes the motion (Lenz)", Color{255, 200, 120, 255});
        if (mode == 0) {
            std::snprintf(buf, sizeof buf, "copper tube (left): magnet speed %.2f m/s, thin-tube theory terminal speed %.2f m/s",
                          mags.size() > 0 ? length(mags[0]->velocity) : 0.0f, vTheory);
            out.add(buf);
            std::snprintf(buf, sizeof buf, "plastic tube (right): magnet speed %.2f m/s (free fall)", mags.size() > 1 ? length(mags[1]->velocity) : 0.0f);
            out.add(buf);
        } else if (mode == 1) {
            std::snprintf(buf, sizeof buf, "Arago's disc: magnets circle at %.1f rad/s, the copper plate follows at %.2f rad/s", ARAGO_W,
                          plate ? plate->angularVelocity.y : 0.0f);
            out.add(buf);
        } else {
            std::snprintf(buf, sizeof buf, "magnets under the ramp: copper block %.2f m/s, plastic block %.2f m/s",
                          blocks.size() > 0 ? length(blocks[0]->velocity) : 0.0f, blocks.size() > 1 ? length(blocks[1]->velocity) : 0.0f);
            out.add(buf);
        }
        out.add("cyan arrows: eddy current density (strongest only)", GRAY, 14);
        out.gap(4);
        out.add("M mode (tubes / Arago / ramp)   Enter restart   O magnetic field lines", GRAY);
        (void)w;
    }

private:
    static constexpr float ARAGO_W = 2.0f, ARAGO_R = 0.7f;
    int   mode = 0;
    float t = 0.0f, vTheory = 0.0f;
    std::vector<RigidBody*> mags, blocks;
    RigidBody* plate = nullptr;

    // 管: 中心 (x, 中ほどの高さ)、壁の中心の半径 a、厚さ th、高さ 2 hh、周り n 分割。sigma = 0 ならプラスチック
    void tube(SceneHost& host, float x, float yc, float a, float th, float hh, int n, float sigma, Color c) {
        World& w = host.world;
        const float half = a * std::tan(PHYS_PI / n);
        for (int i = 0; i < n; ++i) {
            const float phi = 2.0f * PHYS_PI * (float)i / n;
            const Vec3  pos{x + a * std::cos(phi), yc, a * std::sin(phi)};
            const Quat  q = Quat::fromAxisAngle({0, 1, 0}, -phi);
            RigidBody* b = sigma > 0.0f ? w.createConductorBox(pos, {0.5f * th, hh, half}, 0.0f, sigma, q, 0.16f)
                                        : w.createBox(pos, {0.5f * th, hh, half}, 0.0f);
            b->orientation = q;
            b->friction    = 0.2f;
            host.paint(b, c);
        }
    }

    void buildTubes(SceneHost& host) {
        World& w = host.world;
        const float a = 0.5f, th = 0.08f, hh = 4.0f, m0 = 5.0f, mass = 1.0f;
        const float sigma = sigmaForTubeTerminalSpeed(m0, mass, G_ACC, a, th, 0.8f);
        vTheory = mass * G_ACC / tubeDragCoefficient(m0, a, th, sigma);
        // 2 本の管は 6 m 離す（近いと 2 つの磁石が横に押し合い、銅の管の磁石が壁にこすれる）
        tube(host, -3.0f, hh + 0.2f, a, th, hh, 32, sigma, Fade(COL_COPPER, 0.55f));
        tube(host, 3.0f, hh + 0.2f, a, th, hh, 32, 0.0f, Color{200, 205, 215, 70});
        for (int k = 0; k < 2; ++k) {
            RigidBody* m = w.createMagnet({k == 0 ? -3.0f : 3.0f, 2.0f * hh + 0.6f, 0}, 0.3f, mass, {0, m0, 0});
            m->linearDamping = 0.0f;
            mags.push_back(m);
        }
        camTargetX = 0.0f; camTargetY = 5.5f; camTargetZ = 0.0f;
        camDistance = 19.0f; camPitch = 0.05f; camYaw = 0.0f;
    }

    void buildArago(SceneHost& host) {
        World& w = host.world;
        w.gravity = Vec3{0, 0, 0};   // 板は軸で支えるので、重力はなくてよい（見やすさのため）
        t = 0.0f;
        plate = w.createConductorBox({0, 1.0f, 0}, {1.2f, 0.05f, 1.2f}, 1.0f, 2.0f, Quat{}, 0.15f);
        plate->angularDamping = 0.02f;
        w.setPinned(plate, true);
        host.paint(plate, COL_COPPER);
        host.addStatic({0, 0.42f, 0}, {0.06f, 0.5f, 0.06f}, Color{150, 158, 172, 255});   // 軸（板とは触れない）
        for (int i = 0; i < 2; ++i) {
            RigidBody* m = w.createMagnet({ARAGO_R, 1.45f, 0}, 0.2f, 0.0f, {0, -3.0f, 0});
            mags.push_back(m);
        }
        camTargetX = 0.0f; camTargetY = 1.0f; camTargetZ = 0.0f;
        camDistance = 6.0f; camPitch = 0.6f; camYaw = 0.5f;
    }

    void buildRamp(SceneHost& host) {
        World& w = host.world;
        const float ang = 22.0f * PHYS_PI / 180.0f;
        const Quat  q   = Quat::fromAxisAngle({0, 0, 1}, -ang);   // +x へ下る坂
        const Vec3  down{std::cos(ang), -std::sin(ang), 0}, up = q.rotate(Vec3{0, 1, 0});
        const Vec3  c{0, 2.2f, 0};
        RigidBody* ramp = host.addStatic(c, {5.0f, 0.15f, 1.4f}, Color{150, 158, 172, 200}, 0.08f, 0.0f);
        ramp->orientation = q;
        // 坂の中ほどの下に、上向きの磁石を並べる（両方のレーン）
        for (int i = 0; i < 6; ++i)
            for (int k = -1; k <= 1; k += 2) {
                const Vec3 p = c + down * (-0.6f + 0.55f * (float)i) + up * 0.0f + Vec3{0, 0, 0.6f * (float)k};
                w.createMagnet(p, 0.12f, 0.0f, up * 2.0f);
            }
        for (int k = 0; k < 2; ++k) {
            const Vec3 p = c + down * -4.3f + up * (0.15f + 0.08f + 0.01f) + Vec3{0, 0, k == 0 ? -0.6f : 0.6f};
            RigidBody* b = k == 0 ? w.createConductorBox(p, {0.3f, 0.08f, 0.3f}, 0.5f, 0.001f, q, 0.1f)
                                  : w.createBox(p, {0.3f, 0.08f, 0.3f}, 0.5f);
            b->orientation = q;
            b->friction    = 0.08f;
            host.paint(b, k == 0 ? COL_COPPER : Color{235, 235, 240, 255});
            blocks.push_back(b);
        }
        camTargetX = 0.0f; camTargetY = 2.0f; camTargetZ = 0.0f;
        camDistance = 11.0f; camPitch = 0.35f; camYaw = 0.25f;
    }
};

// ===========================================================================
// F8 電場の見える化（z = 0 の面の電気力線と等電位線）
// ===========================================================================
class EFieldScene : public FunScene {
public:
    EFieldScene() { timeScale = 1.0f; }
    const char* title() const override { return "F8: Electric field lines and equipotentials"; }

    void build(SceneHost& host) override {
        host.reset();
        World& w = host.world;
        w.gravity = Vec3{0, 0, 0};
        const float r = 0.2f;
        auto charge = [&](const Vec3& p, float q) { ball(w, p, r, 0.0f, q, false); };
        if (mode == 0) {        // 異符号
            charge({-2.0f, 0, 0}, 1.0f);
            charge({2.0f, 0, 0}, -1.0f);
        } else if (mode == 1) { // 同符号
            charge({-2.0f, 0, 0}, 1.0f);
            charge({2.0f, 0, 0}, 1.0f);
        } else if (mode == 2) { // 平行板と、間の導体の球（分極して場を曲げる）
            const Quat q = Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);   // ローカル +y → +x
            const int ci = w.createCapacitor({0, 0, 0}, q, 3.0f, 2.0f, 5.0f, 0.1f, 2.5f, -2.5f);
            host.paint(w.electric().capacitors()[ci].plateA, ColorLerp(Color{150, 150, 160, 255}, COL_POS, 0.5f));
            host.paint(w.electric().capacitors()[ci].plateB, ColorLerp(Color{150, 150, 160, 255}, COL_NEG, 0.5f));
            ball(w, {0, 0, 0}, 0.7f, 0.0f, 0.0f, true);
        } else {                // 点電荷と、帯電していない導体の球（誘導）
            charge({-2.5f, 0, 0}, 1.0f);
            ball(w, {1.0f, 0, 0}, 0.8f, 0.0f, 0.0f, true);
        }
        w.electric().updateDipoles(30);
        chargeRef = 1.0f;
        traceLines(w);
        contour(w);
        camTargetX = 0.0f; camTargetY = 0.0f; camTargetZ = 0.0f;
        camDistance = 11.0f; camPitch = 0.05f; camYaw = 0.0f;
        wantCameraReset = true;
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 4; build(host); }
        if (IsKeyPressed(KEY_V)) showContours = !showContours;
    }

    void draw3D(const World&) const override {
        for (const auto& l : lines) {
            for (size_t i = 1; i < l.size(); ++i) DrawLine3D(toRay(l[i - 1]), toRay(l[i]), Color{255, 230, 140, 255});
            if (l.size() > 10) {   // 真ん中に向きの矢印（+ → -）
                const size_t m = l.size() / 2;
                arrow(l[m - 1], l[m + 1] + (l[m + 1] - l[m - 1]) * 1.5f, 0.02f, Color{255, 230, 140, 255});
            }
        }
        if (showContours)
            for (const auto& s : segs) DrawLine3D(toRay(s.a), toRay(s.b), s.c);
    }

    void hud(const World& w, HudLines& out) const override {
        static const char* names[] = {"+q and -q", "+q and +q", "parallel plates and a metal ball (polarized)",
                                      "+q and a neutral metal ball (induction)"};
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s: field lines leave + charges and end on - charges", names[mode]);
        out.add(buf, Color{255, 200, 120, 255});
        out.add("yellow: electric field lines in the z = 0 plane   colored: equipotentials (orange +, violet -)");
        std::snprintf(buf, sizeof buf, "electrostatic energy %.3f", w.electric().potentialEnergy());
        out.add(buf, LIGHTGRAY, 14);
        out.gap(4);
        out.add("M layout   V equipotentials", GRAY);
    }

private:
    struct Seg { Vec3 a, b; Color c; };
    int  mode = 0;
    bool showContours = true;
    std::vector<std::vector<Vec3>> lines;
    std::vector<Seg> segs;

    static Vec3 flat(Vec3 v) { v.z = 0.0f; return v; }

    // 電気力線: + の電荷（と + の板、導体の球の外向きの側）から、場に沿ってたどる（z = 0 の面の中）
    void traceLines(const World& w) {
        lines.clear();
        const ElectricSystem& es = w.electric();
        std::vector<Vec3> starts;
        for (const ElectricBody& eb : es.bodies()) {
            const Vec3 c = eb.body->position;
            if (eb.charge > 0.0f) {
                const int n = 16;
                for (int k = 0; k < n; ++k) {
                    const float a = 2.0f * PHYS_PI * ((float)k + 0.5f) / n;
                    starts.push_back(c + Vec3{std::cos(a), std::sin(a), 0} * (eb.radius + 0.02f));
                }
            } else if (eb.conductor) {   // 帯電していない導体: 場が外向きのところから
                for (int k = 0; k < 24; ++k) {
                    const float a = 2.0f * PHYS_PI * ((float)k + 0.5f) / 24;
                    const Vec3  d{std::cos(a), std::sin(a), 0};
                    const Vec3  p = c + d * (eb.radius + 0.02f);
                    if (dot(es.fieldAt(p), d) > 0.3f * length(es.fieldAt(p)) && k % 2 == 0) starts.push_back(p);
                }
            }
        }
        for (const Capacitor& cap : es.capacitors()) {   // + の板の内側の面から
            const Vec3 n = cap.orientation.rotate(Vec3{0, 1, 0});
            const Vec3 face = cap.center - n * (cap.halfExtents.y - 0.02f) * (cap.potentialA >= cap.potentialB ? 1.0f : -1.0f);
            const Vec3 side = cap.orientation.rotate(Vec3{1, 0, 0});
            for (int k = -6; k <= 6; ++k) starts.push_back(flat(face + side * (0.42f * (float)k)));
        }
        for (const Vec3& s : starts) {
            std::vector<Vec3> l{s};
            Vec3 p = s;
            for (int i = 0; i < 900; ++i) {
                const Vec3 E1 = flat(es.fieldAt(p));
                if (lengthSq(E1) < 1e-10f) break;
                const Vec3 mid = p + normalize(E1) * 0.02f;
                const Vec3 E2  = flat(es.fieldAt(mid));
                if (lengthSq(E2) < 1e-10f) break;
                p = p + normalize(E2) * 0.04f;
                l.push_back(p);
                if (std::fabs(p.x) > 7.0f || std::fabs(p.y) > 5.0f) break;
                bool stop = false;
                for (const ElectricBody& eb : es.bodies())
                    if ((eb.charge < 0.0f || eb.conductor) && length(p - eb.body->position) < eb.radius) stop = true;
                for (const Capacitor& cap : es.capacitors()) {
                    const Vec3 q = cap.orientation.toMat3().transposed() * (p - cap.center);
                    if (std::fabs(q.y) > cap.halfExtents.y && std::fabs(q.x) < cap.halfExtents.x) stop = true;
                }
                if (stop) break;
            }
            lines.push_back(l);
        }
    }

    // 等電位線: z = 0 の面の格子で電位を求め、マーチングスクエアで線にする
    void contour(const World& w) {
        segs.clear();
        const ElectricSystem& es = w.electric();
        const int   NX = 141, NY = 101;
        const float x0 = -7.0f, y0 = -5.0f, h = 0.1f;
        std::vector<float> phi(NX * NY);
        for (int j = 0; j < NY; ++j)
            for (int i = 0; i < NX; ++i) phi[j * NX + i] = es.potentialAt(Vec3{x0 + h * i, y0 + h * j, -0.01f});
        static const float levels[] = {-2.0f, -1.0f, -0.6f, -0.35f, -0.2f, -0.1f, 0.0f, 0.1f, 0.2f, 0.35f, 0.6f, 1.0f, 2.0f};
        for (float L : levels) {
            const Color c = L > 0.0f ? Fade(COL_POS, 0.75f) : L < 0.0f ? Fade(COL_NEG, 0.85f) : Fade(RAYWHITE, 0.6f);
            for (int j = 0; j + 1 < NY; ++j)
                for (int i = 0; i + 1 < NX; ++i) {
                    const float v[4] = {phi[j * NX + i], phi[j * NX + i + 1], phi[(j + 1) * NX + i + 1], phi[(j + 1) * NX + i]};
                    const Vec3  pc[4] = {{x0 + h * i, y0 + h * j, -0.01f}, {x0 + h * (i + 1), y0 + h * j, -0.01f},
                                         {x0 + h * (i + 1), y0 + h * (j + 1), -0.01f}, {x0 + h * i, y0 + h * (j + 1), -0.01f}};
                    Vec3 cut[4];
                    int  n = 0;
                    for (int e = 0; e < 4; ++e) {
                        const float a = v[e] - L, b = v[(e + 1) % 4] - L;
                        if ((a < 0.0f) == (b < 0.0f)) continue;
                        const float t = a / (a - b);
                        cut[n++] = pc[e] + (pc[(e + 1) % 4] - pc[e]) * t;
                    }
                    if (n >= 2) segs.push_back({cut[0], cut[1], c});
                    if (n == 4) segs.push_back({cut[2], cut[3], c});
                }
        }
    }
};

std::unique_ptr<FunScene> makeBellScene()    { return std::make_unique<BellScene>(); }
std::unique_ptr<FunScene> makeLorentzScene() { return std::make_unique<LorentzScene>(); }
std::unique_ptr<FunScene> makeEddyScene()    { return std::make_unique<EddyScene>(); }
std::unique_ptr<FunScene> makeEFieldScene()  { return std::make_unique<EFieldScene>(); }
