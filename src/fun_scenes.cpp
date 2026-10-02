// ---------------------------------------------------------------------------
// fun_scenes.cpp : 磁力を見せるシーン A・B・C・D・E（fun_scenes.h）
// ---------------------------------------------------------------------------
#include "fun_scenes.h"

#include "demo_common.h"
#include "field_lines.h"
#include "gauss_rail.h"
#include "levitation_theory.h"

#include "raymath.h"

#include <cmath>
#include <cstdio>

using namespace phys;
using namespace demo;

namespace {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }

// 決定論的な乱数
struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    float next() { s = s * 1664525u + 1013904223u; return ((s >> 8) & 0xFFFFFF) / 16777216.0f; }
    Vec3 dir() {
        for (;;) {
            Vec3 v{next() * 2 - 1, next() * 2 - 1, next() * 2 - 1};
            float l2 = lengthSq(v);
            if (l2 > 0.01f && l2 <= 1.0f) return v / std::sqrt(l2);
        }
    }
};

// 太さのある矢印
void drawArrow(const Vec3& from, const Vec3& to, float radius, Color c) {
    Vec3  d   = to - from;
    float len = length(d);
    if (len < 1e-4f) return;
    Vec3 head = to - d * (std::min(0.3f, 0.35f * len) / len);
    DrawCylinderEx(toRay(from), toRay(head), radius, radius, 8, c);
    DrawCylinderEx(toRay(head), toRay(to), radius * 2.5f, 0.0f, 8, c);
}

// 動く磁性体の重心（なければ原点）
Vec3 magnetCentroid(const World& w) {
    Vec3 c{0, 0, 0};
    int  n = 0;
    for (const MagneticBody& mb : w.magnets().bodies()) {
        if (mb.body->isStatic() || mb.body->position.y < -1.0f) continue;
        c += mb.body->position;
        ++n;
    }
    return n ? c / (float)n : Vec3{0, 0, 0};
}

// ===========================================================================
// A. 磁気ローラー（回転する外部磁場）
//   縦の面の中で回る一様な磁場 B(t) = B0 (cos wt u + sin wt y)。磁石はトルク m x B で
//   磁場について回り、床との摩擦で転がる（回転軸 u x y、進む向き -u）。
//   一様な磁場なので力は 0。動かしているのはトルクだけ。
// ===========================================================================
class RollerScene : public FunScene {
public:
    RollerScene() { camTargetY = 0.3f; camDistance = 14.0f; camPitch = 0.75f; }

    const char* title() const override { return "9: Magnetic rollers (rotating field)"; }
    bool ownsField() const override { return true; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = MAG_SUBSTEPS;
        phase = 0.0;
        Rng rng(7);

        // 囲い（透明、摩擦 0、高さ 3）。摩擦があると、回っている磁石が壁をよじ登って乗り越える。
        // 鎖は端から端へ宙返りしながら進むので、いちばん長い鎖より高くする
        const Color fence{170, 200, 240, 30};
        const float fh = 1.5f;
        host.addStatic({ ARENA + 0.1f, fh, 0}, {0.1f, fh, ARENA + 0.2f}, fence, 0.0f, 0.2f);
        host.addStatic({-ARENA - 0.1f, fh, 0}, {0.1f, fh, ARENA + 0.2f}, fence, 0.0f, 0.2f);
        host.addStatic({0, fh,  ARENA + 0.1f}, {ARENA, fh, 0.1f}, fence, 0.0f, 0.2f);
        host.addStatic({0, fh, -ARENA - 0.1f}, {ARENA, fh, 0.1f}, fence, 0.0f, 0.2f);

        if (layout == 0) {
            // 1 個ずつ。間隔 3.8 は、止まっている磁石が床の摩擦に勝って引き合う距離（約 3.6）より少し遠い
            for (int i = -1; i <= 1; ++i)
                for (int k = -1; k <= 1; ++k)
                    makeMagnetBall(w, {i * 3.8f, MAG_RADIUS, k * 3.8f}, rng.dir());
        } else if (layout == 1) {
            // 長さの違う鎖（向きは鎖の軸）。横に並んだ平行な鎖は反発するので、くっつかない
            const int   lens[4] = {2, 3, 4, 5};
            const float d = 2.0f * MAG_RADIUS;
            for (int c = 0; c < 4; ++c)
                for (int k = 0; k < lens[c]; ++k)
                    makeMagnetBall(w, {(k - 0.5f * (lens[c] - 1)) * d, MAG_RADIUS, -4.5f + 3.0f * c}, {1, 0, 0});
        } else {
            // 砂鉄だけ（磁石なし）。磁化した粒が鎖になり、磁場について回りながら転がる。
            // 粒どうしの引力は重さの約 20 倍（磁石の 1/40）なので、サブステップ 2 で足りる
            w.substeps = 2;
            for (int n = 0; n < 60;) {
                Vec3 p{(rng.next() - 0.5f) * 2.4f, GRAIN_RADIUS + rng.next() * 0.2f, (rng.next() - 0.5f) * 2.4f};
                if (p.x * p.x + p.z * p.z > 1.2f * 1.2f) continue;
                makeGrain(w, p);
                ++n;
            }
        }
        if (layout == 2) { B0 = 40.0f; freqHz = 0.3f; camDistance = 5.0f;  camTargetY = 0.0f; }
        else             { B0 = 5.0f;  freqHz = 1.0f; camDistance = 14.0f; camTargetY = 0.3f; }
        wantCameraReset = true;
        beforeStep(w, 0.0f);
        w.magnets().updateMoments(3);
    }

    void beforeStep(World& w, float dt) override {
        if (!fieldOn) {
            fieldNow = Vec3{0, 0, 0};
        } else {
            // 周波数は実時間での値。シミュレーションの 1 秒は実物の 1/TIME_SCALE_REAL 秒
            phase += (reverse ? -1.0 : 1.0) * 2.0 * PHYS_PI * (freqHz / TIME_SCALE_REAL) * dt;
            const Vec3 u = travelDir() * -1.0f;
            fieldNow = (u * (float)std::cos(phase) + Vec3{0, 1, 0} * (float)std::sin(phase)) * B0;
        }
        w.magnets().externalField = fieldNow;
    }

    void handleInput(SceneHost& host) override {
        const float dt = GetFrameTime();
        if (IsKeyDown(KEY_LEFT))  heading -= 1.5f * dt;
        if (IsKeyDown(KEY_RIGHT)) heading += 1.5f * dt;
        if (IsKeyPressed(KEY_UP))   freqHz = std::min(10.0f, freqHz * 1.25f);
        if (IsKeyPressed(KEY_DOWN)) freqHz = std::max(0.05f, freqHz / 1.25f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) B0 = std::min(200.0f, B0 * 1.25f);
        if (IsKeyPressed(KEY_LEFT_BRACKET))  B0 = std::max(0.5f, B0 / 1.25f);
        if (IsKeyPressed(KEY_X)) fieldOn = !fieldOn;
        if (IsKeyPressed(KEY_V)) reverse = !reverse;
        if (IsKeyPressed(KEY_M)) { layout = (layout + 1) % 3; build(host); }
    }

    void draw3D(const World& w) const override {
        const Vec3  c = magnetCentroid(w);
        const float s = camDistance / 14.0f;   // 矢印の大きさは見ている距離に合わせる
        // 進む向き（床の上の緑の矢印）
        const Vec3 t = travelDir() * (reverse ? -1.0f : 1.0f);
        drawArrow(Vec3{c.x, 0.03f, c.z}, Vec3{c.x, 0.03f, c.z} + t * (1.4f * s), 0.04f * s, Color{90, 220, 120, 255});
        if (!fieldOn) return;
        // 磁場の向き（黄）と、磁場が回る面（円）
        const Vec3 o = Vec3{c.x, 1.8f * s, c.z};
        drawArrow(o, o + fieldNow * (1.1f * s / B0), 0.035f * s, Color{250, 220, 80, 255});
        const Vec3  u     = travelDir() * -1.0f;
        const float angle = std::atan2(-u.z, u.x) * RAD2DEG;
        DrawCircle3D(toRay(o), 1.1f * s, Vector3{0, 1, 0}, angle, Fade(Color{250, 220, 80, 255}, 0.6f));
    }

    void hud(const World&, HudLines& out) const override {
        static const char* layouts[] = {"single magnets", "chains (2, 3, 4, 5)", "iron sand (60 grains)"};
        char buf[160];
        if (fieldOn)
            std::snprintf(buf, sizeof buf, "rotating field ON   %.2f Hz   |B0| %.1f mT   %s",
                          freqHz, B0 * FIELD_MT_PER_UNIT, reverse ? "(reversed)" : "");
        else
            std::snprintf(buf, sizeof buf, "rotating field OFF");
        out.add(buf, Color{250, 220, 80, 255});
        std::snprintf(buf, sizeof buf, "heading %.0f deg   layout: %s", heading * RAD2DEG, layouts[layout]);
        out.add(buf);
        out.add("torque m x B turns each magnet; friction with the floor makes it roll");
        out.add("(the field does work, so total energy is not conserved)", GRAY);
        out.gap(4);
        out.add("Left/Right heading   Up/Down frequency   [ ] strength", GRAY);
        out.add("X field on/off   V reverse   M layout", GRAY);
    }

private:
    static constexpr float ARENA = 6.0f;
    int    layout  = 0;
    bool   fieldOn = true;
    bool   reverse = false;
    float  heading = 0.0f;    // 進む向き（xz 平面の角度）
    float  freqHz  = 1.0f;    // 実時間での回転数
    float  B0      = 5.0f;    // 場の強さ（1 単位 ≒ 1 mT）
    double phase   = 0.0;
    Vec3   fieldNow{0, 0, 0};

    Vec3 travelDir() const { return Vec3{std::cos(heading), 0, std::sin(heading)}; }
};

// ===========================================================================
// B. 浮かぶ磁石の塔（磁気ばね）
//   細い筒の中に立方体の磁石を、隣どうしが反発する向き（交互）に入れる。
//   k 番目のすき間は上に載っている磁石の重さを支えるので、下ほど詰まる。
//   シミュレーションの高さを、点双極子のつり合い（全ペア）の理論値と並べて表示する。
// ===========================================================================
class TowerScene : public FunScene {
public:
    TowerScene() { camPitch = 0.12f; fitCamera(); }

    const char* title() const override { return "0: Levitating tower (magnetic spring)"; }

    void build(SceneHost& host) override { rebuild(host, true); }

    // compressed = true なら理論の高さから少し縮めて放す（ばねのように伸びて、振動しながら落ち着く）
    void rebuild(SceneHost& host, bool compressed) {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = MAG_SUBSTEPS;
        stack.clear();
        signs.clear();
        spheres.clear();
        alternating = true;
        tubeTop = theoryHeights(MAX_COUNT).back() + 2.5f;

        addTube(host, 0.0f, CUBE_HALF + CLEARANCE);
        theory = theoryHeights(count);
        fitCamera();
        for (int k = 0; k < count; ++k) {
            float y = CUBE_HALF + (theory[k] - CUBE_HALF) * (compressed ? 0.92f : 1.0f);
            addCube(w, y, (k % 2 == 0) ? 1.0f : -1.0f);
        }

        if (layout == 1) {
            // 比較用: 球の磁石は筒の中でも向きを変えられるので、ひっくり返ってくっつく
            addTube(host, SPHERE_TUBE_X, MAG_RADIUS + CLEARANCE);
            std::vector<float> mu(count);
            const float m0 = MAG_M0 * magnetMomentScale();
            for (int k = 0; k < count; ++k) mu[k] = (k % 2 == 0) ? m0 : -m0;
            std::vector<float> ys = dipoleStackEquilibrium(mu, MAG_MASS, 9.81f, MAG_RADIUS);
            for (int k = 0; k < count; ++k) {
                Vec3 dir{0.03f * ((k % 3) - 1.0f), (k % 2 == 0) ? 1.0f : -1.0f, 0.02f};   // ほんの少し傾ける
                RigidBody* b = makeMagnetBall(w, {SPHERE_TUBE_X, ys[k], 0}, dir);
                b->friction = 0.3f;
                spheres.push_back(b);
            }
        }
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        if (IsKeyPressed(KEY_UP) && count < MAX_COUNT) {         // 反発する向きで上から落とす
            ++count;
            addCube(w, tubeTop - 0.6f, -signs.back());
            theory = theoryHeights(count);
        }
        if (IsKeyPressed(KEY_DOWN) && count > 2) { --count; rebuild(host, false); }
        if (IsKeyPressed(KEY_X) && count < MAX_COUNT) {          // 引き合う向き（上の磁石と同じ）で落とす
            ++count;
            addCube(w, tubeTop - 0.6f, signs.back());
            alternating = false;
        }
        if (IsKeyPressed(KEY_K)) stack.back()->velocity.y -= 6.0f;   // 上から押す
        if (IsKeyPressed(KEY_M)) { layout = (layout + 1) % 2; rebuild(host, true); }
    }

    void draw3D(const World&) const override {
        if (!alternating) return;
        const float s = 2.0f * (CUBE_HALF + CLEARANCE + WALL) + 0.2f;
        for (int k = 1; k < count && k < (int)stack.size(); ++k)
            DrawCubeWires(Vector3{0, theory[k], 0}, s, 0.002f, s, Color{90, 230, 120, 255});
    }

    void hud(const World&, HudLines& out) const override {
        char buf[160];
        std::snprintf(buf, sizeof buf, "%d cube magnets in a tube, %s", (int)stack.size(),
                      alternating ? "neighbours repel (N-N, S-S)" : "one cube flipped");
        out.add(buf, Color{90, 230, 120, 255});
        if (alternating) {
            out.add("  k    y sim   y theory   diff      (green rings = theory)");
            float worst = 0.0f;
            for (int k = (int)stack.size() - 1; k >= 1; --k) {
                float ys = stack[k]->position.y, yt = theory[k];
                float d  = (ys - yt) / (yt - CUBE_HALF);
                worst = std::max(worst, std::fabs(d));
                std::snprintf(buf, sizeof buf, "%3d   %6.2f   %6.2f   %+6.1f%%", k, ys, yt, 100.0f * d);
                out.add(buf, LIGHTGRAY, 14);
            }
            out.add("theory: coaxial point dipoles, all pairs, weight balance", GRAY, 14);
        } else {
            out.add("theory applies only when neighbours repel (press Down or R)", GRAY);
        }
        out.gap(4);
        out.add("Up add cube   Down remove   X add flipped   K push top   M compare spheres", GRAY);
    }

private:
    static constexpr int   MAX_COUNT     = 10;
    static constexpr float CLEARANCE     = 0.02f;   // 筒と磁石のすき間
    static constexpr float WALL          = 0.05f;   // 筒の壁の半分の厚さ
    static constexpr float SPHERE_TUBE_X = 3.5f;

    int   count      = 6;
    int   layout     = 0;
    bool  alternating = true;
    float tubeTop    = 10.0f;
    std::vector<RigidBody*> stack, spheres;
    std::vector<float>      signs;      // 各立方体の磁化の向き（+1: N 極が上）
    std::vector<float>      theory;

    // 今の個数の塔が画面に収まるカメラ
    void fitCamera() {
        const float top = theoryHeights(count).back();
        camTargetY  = 0.5f * top;
        camDistance = 1.1f * top + 3.0f;
    }

    static std::vector<float> theoryHeights(int n) {
        std::vector<float> mu(n);
        const float m = CUBE_M * magnetMomentScale();
        for (int k = 0; k < n; ++k) mu[k] = (k % 2 == 0) ? m : -m;
        return dipoleStackEquilibrium(mu, CUBE_MASS, 9.81f, CUBE_HALF);
    }

    void addCube(World& w, float y, float sign) {
        RigidBody* b = makeMagnetCube(w, {0, y, 0}, Quat{}, sign);
        b->linearDamping = 0.15f;   // 空気抵抗など（振動が 10 秒ほどで収まるように）
        stack.push_back(b);
        signs.push_back(sign);
    }

    // 摩擦 0 の透明な四角い筒（内側の半幅 inner）
    void addTube(SceneHost& host, float x, float inner) const {
        const Color glass{170, 200, 240, 45};
        const float h = 0.5f * tubeTop;
        host.addStatic({x + inner + WALL, h, 0}, {WALL, h, inner + 2 * WALL}, glass, 0.0f, 0.0f);
        host.addStatic({x - inner - WALL, h, 0}, {WALL, h, inner + 2 * WALL}, glass, 0.0f, 0.0f);
        host.addStatic({x, h,  inner + WALL}, {inner, h, WALL}, glass, 0.0f, 0.0f);
        host.addStatic({x, h, -inner - WALL}, {inner, h, WALL}, glass, 0.0f, 0.0f);
    }
};

// ===========================================================================
// C. 方位磁針の群れ
//   磁石の球を格子に並べ、位置だけ固定する（回転は自由）。互いの場だけで向きが
//   そろい、渦や縞の模様（磁区）ができる。上から探り磁石を近づけると、向きが
//   次々に変わる。
// ===========================================================================
class CompassScene : public FunScene {
public:
    CompassScene() { camTargetY = 0.8f; camDistance = 9.5f; camPitch = 1.05f; }

    const char* title() const override { return "-: Compass array (pinned magnets)"; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = 4;   // 位置が動かないので、接近の速さで刻みが決まることはない
        needles.clear();
        Rng rng(11);

        const float half = 0.5f * (N - 1) * SPACING;
        host.addStatic({0, 0.15f, 0}, {half + 0.45f, 0.15f, half + 0.45f}, Color{92, 70, 52, 255});   // 板

        for (int i = 0; i < N; ++i)
            for (int k = 0; k < N; ++k) {
                RigidBody* b = makeMagnetBall(w, {i * SPACING - half, HEIGHT, k * SPACING - half}, rng.dir());
                b->angularDamping = lowDamping ? 0.02f : 0.3f;
                w.setPinned(b, true);
                needles.push_back(b);
            }

        // 探り磁石（静的。矢印キーで動かす）
        probe = makeMagnetBall(w, probeOn ? probePos : FAR_AWAY, {0, probeSign, 0}, true, 1.0f, PROBE_RADIUS);
        w.magnets().updateMoments(1);
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        const float dt = GetFrameTime(), v = 3.0f;
        if (IsKeyDown(KEY_LEFT))  probePos.x -= v * dt;
        if (IsKeyDown(KEY_RIGHT)) probePos.x += v * dt;
        if (IsKeyDown(KEY_UP))    probePos.z -= v * dt;
        if (IsKeyDown(KEY_DOWN))  probePos.z += v * dt;
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) probePos.y = std::min(5.0f, probePos.y + 0.3f);
        if (IsKeyPressed(KEY_LEFT_BRACKET))  probePos.y = std::max(HEIGHT + 0.7f, probePos.y - 0.3f);
        if (IsKeyPressed(KEY_V)) {
            probeSign = -probeSign;
            probe->orientation = (probeSign > 0) ? Quat{} : Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
        }
        if (IsKeyPressed(KEY_B)) probeOn = !probeOn;
        if (IsKeyPressed(KEY_X)) {                       // かき混ぜる
            Rng rng((unsigned)(GetTime() * 1000.0) | 1u);
            for (RigidBody* b : needles) {
                b->orientation     = Quat::fromAxisAngle(rng.dir(), rng.next() * 2.0f * PHYS_PI);
                b->angularVelocity = rng.dir() * 8.0f;
            }
        }
        if (IsKeyPressed(KEY_J)) {
            lowDamping = !lowDamping;
            for (RigidBody* b : needles) b->angularDamping = lowDamping ? 0.02f : 0.3f;
        }
        probe->position = probeOn ? probePos : FAR_AWAY;   // 静的な物体なので、位置を直接動かしてよい
        (void)w;
    }

    void draw3D(const World& w) const override {
        // 各磁石の N 極の向き（白い線）
        for (const MagneticBody& mb : w.magnets().bodies()) {
            if (mb.body == probe || !w.isPinned(mb.body)) continue;
            Vec3 d = normalize(mb.m) * 0.36f;
            DrawLine3D(toRay(mb.body->position), toRay(mb.body->position + d), RAYWHITE);
        }
        if (probeOn)
            DrawLine3D(toRay(probePos), Vector3{probePos.x, 0.3f, probePos.z}, Fade(RAYWHITE, 0.5f));
    }

    void hud(const World& w, HudLines& out) const override {
        Vec3 M{0, 0, 0};
        for (RigidBody* b : needles) M += b->orientation.rotate(Vec3{0, 1, 0});
        const float d  = 2.0f * MAG_RADIUS;
        const float m0 = MAG_M0 * magnetMomentScale();
        const float u0 = m0 * m0 / (d * d * d);
        char buf[160];
        std::snprintf(buf, sizeof buf, "%d x %d pinned magnets (spacing %.1f)   net magnetization %.2f",
                      N, N, SPACING, length(M) / needles.size());
        out.add(buf, Color{240, 200, 120, 255});
        std::snprintf(buf, sizeof buf, "magnetic energy %.2f u0 per magnet   damping %s",
                      w.magnets().potentialEnergy() / (u0 * needles.size()), lowDamping ? "low" : "high");
        out.add(buf);
        std::snprintf(buf, sizeof buf, "probe magnet %s   %s pole down   height %.1f",
                      probeOn ? "ON" : "OFF", probeSign < 0 ? "N" : "S", probePos.y);
        out.add(buf);
        out.gap(4);
        out.add("Arrows move probe   [ ] height   V flip   B probe on/off", GRAY);
        out.add("X shake   J damping   E uniform field", GRAY);
    }

private:
    static constexpr int   N            = 10;
    static constexpr float SPACING      = 0.6f;
    static constexpr float HEIGHT       = 0.8f;
    static constexpr float PROBE_RADIUS = 0.35f;
    inline static const Vec3 FAR_AWAY{0, 500.0f, 0};

    std::vector<RigidBody*> needles;
    RigidBody* probe      = nullptr;
    bool       probeOn    = true;
    float      probeSign  = 1.0f;          // +1: N 極が上（S 極が下）
    Vec3       probePos{2.0f, 2.0f, 2.0f};
    bool       lowDamping = false;
};

// ===========================================================================
// D. ガウス加速器（gauss_rail.h）
//   Enter で最初の鉄球を転がす。各段の手前と後ろ（段と段の中間）を球が通った
//   ときの速さを測り、損失がない場合の速さ（1 段ごとに gaussStageEnergy だけ
//   運動エネルギーが増える）と並べて表示する。
// ===========================================================================
class GaussScene : public FunScene {
public:
    GaussScene() { camTargetY = 0.0f; camPitch = 0.45f; camYaw = 0.3f; fitCamera(); }

    const char* title() const override { return "=: Gauss accelerator"; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = MAG_SUBSTEPS;
        rail = buildGaussRail(w, params);
        for (RigidBody* r : rail.rails) host.paint(r, Color{150, 158, 172, 255});
        stageEnergy = gaussStageEnergy(params.balls);
        launched = false;
        meter.reset(params);
        fitCamera();
        wantCameraReset = true;
    }

    void beforeStep(World& w, float) override {
        if (launched) meter.update(w, params);   // 測る点（段と段の中間）を通った球の速さ
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_ENTER)) {                     // 並べ直して、最初の球を転がす
            build(host);
            wantCameraReset = false;
            rail.launch->velocity = Vec3{LAUNCH_SPEED, 0, 0};
            launched = true;
        }
        int balls = params.balls, stages = params.stages;
        if (IsKeyPressed(KEY_UP))    balls  = std::min(4, balls + 1);
        if (IsKeyPressed(KEY_DOWN))  balls  = std::max(1, balls - 1);
        if (IsKeyPressed(KEY_RIGHT)) stages = std::min(8, stages + 1);
        if (IsKeyPressed(KEY_LEFT))  stages = std::max(1, stages - 1);
        if (balls != params.balls || stages != params.stages) {
            params.balls = balls; params.stages = stages;
            build(host);
        }
    }

    void draw3D(const World&) const override {
        // 測る点（床の上の線）。測れた点は緑
        const float z = GAUSS_RAIL_INNER + 2.0f * GAUSS_RAIL_WALL + 0.25f;
        for (int c = 0; c <= params.stages; ++c) {
            const float x = gaussCheckpointX(params, c);
            const Color col = meter.speeds[c] >= 0.0f ? Color{90, 230, 120, 255} : Color{200, 200, 210, 160};
            DrawLine3D(Vector3{x, 0.02f, -z}, Vector3{x, 0.02f, z}, col);
        }
    }

    void hud(const World&, HudLines& out) const override {
        char buf[160];
        std::snprintf(buf, sizeof buf, "%d stages x (cube magnet + %d steel balls)   magnetic energy per stage %.3f mJ",
                      params.stages, params.balls, stageEnergy * ENERGY_MJ_PER_UNIT);
        out.add(buf, Color{120, 200, 255, 255});
        if (!launched) {
            out.add("press Enter to roll the first ball");
        } else {
            out.add("  checkpoint   speed [m/s]   lossless [m/s]");
            // 損失がなければ、段 k を出た球の運動エネルギーは、段 k に入った球の運動エネルギー + 1 段ぶん
            const std::vector<float>& v = meter.speeds;
            for (int c = 0; c <= params.stages; ++c) {
                float ideal = -1.0f;
                if (c > 0 && v[c - 1] >= 0.0f) ideal = std::sqrt(v[c - 1] * v[c - 1] + 2.0f * stageEnergy / MAG_MASS);
                char name[16], vs[16] = "   -", is[16] = "";
                if (c == 0) std::snprintf(name, sizeof name, "in       ");
                else        std::snprintf(name, sizeof name, "after %d  ", c);
                if (v[c] >= 0.0f) std::snprintf(vs, sizeof vs, "%5.2f", v[c] * SPEED_MPS_PER_UNIT);
                if (ideal >= 0.0f) std::snprintf(is, sizeof is, "%5.2f", ideal * SPEED_MPS_PER_UNIT);
                std::snprintf(buf, sizeof buf, "  %s   %8s      %8s", name, vs, is);
                out.add(buf, LIGHTGRAY, 14);
            }
            out.add("losses: sliding until the ball rolls, the stage recoils, the impact is not ideal", GRAY, 14);
        }
        out.gap(4);
        out.add("Enter launch   Up/Down balls per stage   Left/Right stages   F slow motion", GRAY);
    }

private:
    static constexpr float LAUNCH_SPEED = 2.0f;   // 0.2 m/s でそっと転がす
    // エネルギーの換算: 質量 0.5 は実物の 0.49 g、速さ 1 は 0.1 m/s → 0.5 * 1 * 1 は 0.49e-3 * 0.01 / 0.5 J
    static constexpr float ENERGY_MJ_PER_UNIT = 1000.0f * (0.49e-3f / MAG_MASS) * 0.01f;

    GaussParams        params;
    GaussRail          rail;
    float              stageEnergy = 0.0f;
    bool               launched    = false;
    GaussMeter         meter;

    void fitCamera() { camDistance = 0.62f * params.stages * params.spacing + 4.0f; }
};

// ===========================================================================
// E. 磁力線（field_lines.h）
//   磁石と鉄を置いて、磁力線（3D の曲線。デモ本体が描く。O キーで ON/OFF）と、
//   面の上の砂鉄（その点の B の向きの短い線。明るさが |B|）を見る。
//   物は固定してあり、キーで動かせる。K で放すと、くっつく間も磁力線がついていく。
// ===========================================================================
class FieldScene : public FunScene {
public:
    // 注視点を左奥にずらして、物が HUD に隠れない画面の右下に来るようにする
    FieldScene() { camTargetX = -1.6f; camTargetY = 0.3f; camTargetZ = -1.2f; camDistance = 7.5f; camPitch = 0.95f; camYaw = 0.0f; }

    const char* title() const override { return "BkSp: Field lines"; }
    bool wantsFieldLines() const override { return true; }
    bool ownsField() const override { return layout == IRON_IN_FIELD; }

    void build(SceneHost& host) override {
        host.reset();
        // 半透明の台（台の下を通る磁力線も透けて見える）
        host.addStatic({0, -1.0f, 0}, {30.0f, 1.0f, 30.0f}, Color{70, 75, 85, 170});
        World& w = host.world;
        w.substeps = MAG_SUBSTEPS;
        items.clear();
        masses.clear();
        selected = 0;
        released = false;
        filings  = (layout == HORSESHOE) ? 2 : 1;
        planeY   = MAG_RADIUS;

        const float R = MAG_RADIUS, d = 2.0f * R;
        auto magnet = [&](const Vec3& p, const Vec3& dir) { fix(makeMagnetBall(w, p, dir)); };
        switch (layout) {
        case 0:   // 磁石 1 個
            magnet({0, R, 0}, {1, 0, 0});
            break;
        case 1:   // 引き合う向き（N と S が向き合う）
            magnet({-1.2f, R, 0}, {1, 0, 0});
            magnet({ 1.2f, R, 0}, {1, 0, 0});
            break;
        case 2:   // 反発する向き（N と N が向き合う）
            magnet({-1.2f, R, 0}, { 1, 0, 0});
            magnet({ 1.2f, R, 0}, {-1, 0, 0});
            break;
        case 3:   // 磁石と鉄球と鉄の棒
            magnet({-1.0f, R, 0}, {1, 0, 0});
            fix(makeIronBall(w, {0.05f, R, 0}));
            fix(makeIronBox(w, {1.0f, 0.12f, 0}, {0.12f, 0.12f, 0.9f}));
            break;
        case HORSESHOE:   // 鉄の板に立てた立方体の磁石 2 個（U 字磁石）
            planeY = 0.2f + CUBE_HALF;
            fix(makeIronBox(w, {0, 0.1f, 0}, {1.0f, 0.1f, 0.3f}));
            fix(makeMagnetCube(w, {-0.7f, planeY, 0}, Quat{},  1.0f));   // N 極が上
            fix(makeMagnetCube(w, { 0.7f, planeY, 0}, Quat{}, -1.0f));   // S 極が上
            break;
        case 5: { // 鎖と輪（6 個ずつ）
            for (int k = 0; k < 6; ++k) magnet({(k - 2.5f) * d, R, -1.3f}, {1, 0, 0});
            const float rc = 0.5f * d / std::sin(PHYS_PI / 6.0f);
            for (int k = 0; k < 6; ++k) {
                const float th = 2.0f * PHYS_PI * k / 6.0f;
                magnet({rc * std::cos(th), R, 1.3f + rc * std::sin(th)}, {-std::sin(th), 0, std::cos(th)});
            }
            break;
        }
        case IRON_IN_FIELD:   // 一様な外部磁場の中の鉄（球と、斜めの棒）
            planeY = 0.15f;
            w.magnets().externalField = Vec3{FIELD_MT / FIELD_MT_PER_UNIT, 0, 0};
            fix(makeIronBall(w, {-1.6f, 0.55f, 0}, 0.55f));
            fix(makeIronBox(w, {1.4f, 0.12f, 0}, {1.3f, 0.12f, 0.12f},
                            Quat::fromAxisAngle({0, 1, 0}, 0.25f * PHYS_PI)));
            break;
        }
        w.magnets().updateMoments(30);
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        if (IsKeyPressed(KEY_M)) { layout = (layout + 1) % LAYOUT_COUNT; build(host); wantCameraReset = true; return; }
        if (IsKeyPressed(KEY_I)) filings = (filings + 1) % 3;
        if (items.empty()) return;
        if (IsKeyPressed(KEY_X)) selected = (selected + 1) % (int)items.size();
        if (IsKeyPressed(KEY_K)) {   // 放す ⇄ 固定する
            released = !released;
            for (size_t i = 0; i < items.size(); ++i) {
                items[i]->velocity = items[i]->angularVelocity = Vec3{0, 0, 0};
                items[i]->setMass(released ? masses[i] : 0.0f);
            }
        }
        if (released) return;   // 動いている間はキーで動かさない

        // 選んだ物を動かす（静的な物体なので、位置と姿勢を直接書き換えてよい）
        RigidBody*  b  = items[selected];
        const float dt = GetFrameTime();
        if (IsKeyDown(KEY_LEFT))  b->position.x -= 2.0f * dt;
        if (IsKeyDown(KEY_RIGHT)) b->position.x += 2.0f * dt;
        if (IsKeyDown(KEY_UP))    b->position.z -= 2.0f * dt;
        if (IsKeyDown(KEY_DOWN))  b->position.z += 2.0f * dt;
        float turn = 0.0f;
        if (IsKeyDown(KEY_LEFT_BRACKET))  turn += 1.5f * dt;
        if (IsKeyDown(KEY_RIGHT_BRACKET)) turn -= 1.5f * dt;
        if (turn != 0.0f) {
            b->orientation = Quat::fromAxisAngle({0, 1, 0}, turn) * b->orientation;
            b->orientation.normalizeInPlace();
        }
        if (IsKeyPressed(KEY_V)) {   // 極を反転（モーメントに垂直な軸のまわりに半回転）
            const int  i = w.magnets().indexOf(b);
            const Vec3 m = i >= 0 ? w.magnets().bodies()[i].m : Vec3{0, 1, 0};
            Vec3 axis = cross(m, Vec3{0, 1, 0});
            if (lengthSq(axis) < 1e-6f * std::max(1e-6f, lengthSq(m))) axis = Vec3{1, 0, 0};
            b->orientation = Quat::fromAxisAngle(normalize(axis), PHYS_PI) * b->orientation;
            b->orientation.normalizeInPlace();
        }
    }

    // マウスの指す点（砂鉄の面の上）の場
    void updateView(const World& w, const Camera3D& cam, Vector2 screenPoint) override {
        const Ray ray = GetScreenToWorldRay(screenPoint, cam);
        probeOk = false;
        float t = -1.0f;
        if (filings == 2) { if (std::fabs(ray.direction.z) > 1e-4f) t = -ray.position.z / ray.direction.z; }
        else if (std::fabs(ray.direction.y) > 1e-4f) t = (planeY - ray.position.y) / ray.direction.y;
        if (t <= 0.0f) return;
        probe = Vec3{ray.position.x + ray.direction.x * t, ray.position.y + ray.direction.y * t,
                     ray.position.z + ray.direction.z * t};
        if (std::fabs(probe.x) > EXTENT || std::fabs(probe.z) > EXTENT || probe.y < 0.0f || probe.y > EXTENT) return;
        for (const MagneticBody& mb : w.magnets().bodies())
            if (length(probe - mb.body->position) < mb.boundRadius) return;   // 物の中
        probeB  = w.magnets().fieldAt(probe);
        probeOk = true;
    }

    void draw3D(const World& w) const override {
        if (filings) drawFilings(w);
        if (!released && !items.empty()) {
            const RigidBody* b = items[selected];
            const float r = (b->shape.type == ShapeType::Box ? std::max(b->shape.halfExtents.x, b->shape.halfExtents.z)
                                                             : b->shape.radius) + 0.12f;
            DrawCircle3D(Vector3{b->position.x, 0.02f, b->position.z}, r, Vector3{1, 0, 0}, 90.0f,
                         Color{255, 220, 80, 255});
        }
        if (probeOk) {
            const float l = length(probeB);
            DrawSphere(toRay(probe), 0.04f, RAYWHITE);
            if (l > 0.0f) drawArrow(probe, probe + probeB * (0.5f / l), 0.015f, RAYWHITE);
        }
    }

    void draw2D(const World&, const Camera3D& cam) const override {
        if (!probeOk) return;
        const Vector2 s = GetWorldToScreen(toRay(probe), cam);
        DrawText(TextFormat("%.1f mT", length(probeB) * FIELD_MT_PER_UNIT), (int)s.x + 12, (int)s.y - 8, 18, RAYWHITE);
    }

    void hud(const World&, HudLines& out) const override {
        static const char* names[LAYOUT_COUNT] = {
            "one magnet", "two magnets, attracting (N-S)", "two magnets, repelling (N-N)",
            "magnet + iron ball + iron bar", "horseshoe: 2 cube magnets on an iron plate",
            "chain vs ring (6 magnets each)", "iron in a uniform field (20 mT)"};
        static const char* notes[LAYOUT_COUNT] = {
            "lines leave the N pole (red) and return to the S pole (blue)",
            "lines run straight from one magnet into the other",
            "lines push each other aside; B = 0 at the midpoint (neutral point)",
            "iron pulls the lines in: the flux passes through it",
            "the plate carries the flux; lines arc from pole to pole",
            "ring: the flux stays inside the loop (few lines). chain: strong poles at the ends",
            "iron gathers the field lines; release (K) and the bar turns along the field"};
        static const char* filingNames[3] = {"off", "horizontal plane", "vertical plane"};
        char buf[160];
        std::snprintf(buf, sizeof buf, "layout %d/%d: %s", layout + 1, LAYOUT_COUNT, names[layout]);
        out.add(buf, Color{255, 220, 80, 255});
        out.add(notes[layout]);
        std::snprintf(buf, sizeof buf, "iron filings: %s   objects %s", filingNames[filings],
                      released ? "released (K to fix)" : "fixed (K to release)");
        out.add(buf);
        if (probeOk) {
            std::snprintf(buf, sizeof buf, "B at cursor: %.1f mT", length(probeB) * FIELD_MT_PER_UNIT);
            out.add(buf);
        }
        out.add("line density and colour show |B|; arrows point from N to S", GRAY, 14);
        out.gap(4);
        out.add("M layout   X select   Arrows move   [ ] turn   V flip   K release   I filings   O lines", GRAY);
    }

private:
    static constexpr int   LAYOUT_COUNT  = 7;
    static constexpr int   HORSESHOE     = 4;
    static constexpr int   IRON_IN_FIELD = 6;
    static constexpr float FIELD_MT      = 20.0f;
    static constexpr float EXTENT        = 4.5f;   // 砂鉄をまく範囲（±。鉛直な面では高さ 0〜EXTENT）

    int   layout   = 0;
    int   filings  = 1;       // 0: なし 1: 水平な面 2: 鉛直な面 (z = 0)
    float planeY   = MAG_RADIUS;
    bool  released = false;
    int   selected = 0;
    std::vector<RigidBody*> items;    // 動かせる物（磁石と鉄）
    std::vector<float>      masses;   // 放したときの質量
    bool  probeOk = false;
    Vec3  probe{0, 0, 0}, probeB{0, 0, 0};
    mutable FieldSnapshot snap;

    // 動く物体として作って、質量を覚えてから固定する
    void fix(RigidBody* b) {
        masses.push_back(1.0f / b->invMass);
        b->setMass(0.0f);
        items.push_back(b);
    }

    // 砂鉄: 面の上に散らした点ごとに、B の面内の向きの短い線。明るさは |B|（対数）
    void drawFilings(const World& w) const {
        snap.capture(w.magnets());
        const int   N    = 110;
        const float step = 2.0f * EXTENT / N, len = 0.7f * step;
        const auto& bodies = w.magnets().bodies();
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) {
                // 格子を少しずらして、模様が格子に見えないようにする（決定論的）
                unsigned h = (unsigned)(i * 73856093) ^ (unsigned)(j * 19349663);
                h = h * 1664525u + 1013904223u;
                const float jx = ((h >> 8) & 0xFF) / 255.0f - 0.5f, jy = ((h >> 16) & 0xFF) / 255.0f - 0.5f;
                const float a = -EXTENT + (i + 0.5f + 0.8f * jx) * step;
                const float c = (filings == 2 ? 0.0f : -EXTENT) + (j + 0.5f + 0.8f * jy) * step * (filings == 2 ? 0.5f : 1.0f);
                const Vec3  p = filings == 2 ? Vec3{a, c, 0.0f} : Vec3{a, planeY, c};
                bool in = false;
                for (const MagneticBody& mb : bodies)
                    if (lengthSq(p - mb.body->position) < mb.boundRadius * mb.boundRadius) { in = true; break; }
                if (in) continue;
                Vec3 B = snap.at(p);
                const float mT = length(B) * FIELD_MT_PER_UNIT;
                if (mT < 0.3f) continue;   // 弱すぎて砂鉄が向きをそろえない
                if (filings == 2) B.z = 0.0f; else B.y = 0.0f;   // 面の中の向き
                const float bl = length(B);
                if (bl < 1e-9f) continue;
                const float s = clampf((std::log10(mT) + 0.5f) / 3.0f, 0.0f, 1.0f);
                const unsigned char v = (unsigned char)(95 + 160 * s);
                const Vec3 e = B * (0.5f * len / bl);
                DrawLine3D(toRay(p - e), toRay(p + e), Color{v, v, (unsigned char)std::min(255, v + 10), 255});
            }
    }
};

} // namespace

std::unique_ptr<FunScene> makeRollerScene()  { return std::make_unique<RollerScene>(); }
std::unique_ptr<FunScene> makeTowerScene()   { return std::make_unique<TowerScene>(); }
std::unique_ptr<FunScene> makeCompassScene() { return std::make_unique<CompassScene>(); }
std::unique_ptr<FunScene> makeGaussScene()   { return std::make_unique<GaussScene>(); }
std::unique_ptr<FunScene> makeFieldScene()   { return std::make_unique<FieldScene>(); }
