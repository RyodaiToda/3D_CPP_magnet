// ---------------------------------------------------------------------------
// current_scenes.cpp : 電流の導線とコイルを見せるシーン（current_scenes.h）
// ---------------------------------------------------------------------------
#include "current_scenes.h"

#include "demo_common.h"
#include "electric_scenes.h"
#include "raymath.h"

#include <cmath>
#include <cstdio>

using namespace phys;
using namespace demo;

namespace {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }

constexpr float G_ACC = 9.81f;
const Color COL_COPPER{200, 125, 80, 255};
const Color COL_WIRE_OFF{95, 70, 55, 255};
const Color COL_ARROW{255, 225, 90, 255};
const Color COL_TITLE{255, 200, 120, 255};

void arrow(const Vec3& from, const Vec3& to, float r, Color c) {
    const Vec3  d = to - from;
    const float L = length(d);
    if (L < 1e-4f) return;
    const Vec3 u = d / L, mid = to - u * std::min(0.4f * L, 4.0f * r);
    DrawCylinderEx(toRay(from), toRay(mid), r, r, 5, c);
    DrawCylinderEx(toRay(mid), toRay(to), 2.4f * r, 0.0f, 6, c);
}

// 軌跡（electric_scenes.cpp と同じ）
struct Trail {
    RigidBody* body = nullptr;
    std::vector<Vec3> pts;
    float q = 0.0f;
    void push(size_t maxN) {
        pts.push_back(body->position);
        if (pts.size() > maxN) pts.erase(pts.begin(), pts.begin() + (pts.size() - maxN));
    }
    void draw() const {
        const Color c = chargeColor(q, std::fabs(q), false);
        for (size_t i = 1; i < pts.size(); ++i)
            DrawLine3D(toRay(pts[i - 1]), toRay(pts[i]), Fade(c, 0.15f + 0.85f * (float)i / pts.size()));
    }
};

// 摩擦 0 の透明な四角い筒（内側の半幅 inner、高さ top）
void addGlassTube(SceneHost& host, const Vec3& base, float inner, float top) {
    const Color glass{170, 200, 240, 45};
    const float wall = 0.05f, h = 0.5f * top;
    host.addStatic(base + Vec3{inner + wall, h, 0}, {wall, h, inner + 2 * wall}, glass, 0.0f, 0.0f);
    host.addStatic(base + Vec3{-inner - wall, h, 0}, {wall, h, inner + 2 * wall}, glass, 0.0f, 0.0f);
    host.addStatic(base + Vec3{0, h, inner + wall}, {inner, h, wall}, glass, 0.0f, 0.0f);
    host.addStatic(base + Vec3{0, h, -inner - wall}, {inner, h, wall}, glass, 0.0f, 0.0f);
}

} // namespace

// ---------------------------------------------------------------------------
// 導線の描画
// ---------------------------------------------------------------------------
void drawWires(const World& w, double time) {
    for (const Wire& wire : w.currents().wires()) {
        const float I  = wire.effectiveCurrent();
        const int   ns = wire.segmentCount();
        if (ns <= 0) continue;
        const Color c = I != 0.0f ? COL_COPPER : COL_WIRE_OFF;
        float total = 0.0f;
        for (int s = 0; s < ns; ++s) {
            Vec3 a, b;
            wire.segment(s, a, b);
            DrawCylinderEx(toRay(a), toRay(b), wire.radius, wire.radius, 8, c);
            total += length(b - a);
        }
        if (I == 0.0f || total < 0.3f) continue;
        // 電流の向きに流れる矢印（0.6 m おき。強さで速さが変わる）
        const float spacing = 0.6f, speed = 0.6f + 0.6f * std::min(1.0f, std::fabs(wire.current) * 2.0f);
        float phase = (float)std::fmod(time * speed, (double)spacing);
        if (I < 0.0f) phase = spacing - phase;
        float acc = 0.0f;
        for (int s = 0; s < ns; ++s) {
            Vec3 a, b;
            wire.segment(s, a, b);
            const float L = length(b - a);
            if (L < 1e-6f) { continue; }
            const Vec3 d = (b - a) / L * (I > 0.0f ? 1.0f : -1.0f);
            float m = std::ceil((acc - phase) / spacing) * spacing + phase;   // この線分の最初の矢印の位置（累積の長さ）
            for (; m < acc + L; m += spacing) {
                if (m < acc) continue;
                const Vec3 p = a + (b - a) * ((m - acc) / L);
                DrawCylinderEx(toRay(p - d * 0.09f), toRay(p + d * 0.09f), 2.6f * wire.radius, 0.0f, 6, COL_ARROW);
            }
            acc += L;
        }
    }
}

// ===========================================================================
// F9 エルステッドと輪の塔
// ===========================================================================
class OerstedScene : public FunScene {
public:
    OerstedScene() { timeScale = 1.0f; }
    const char* title() const override { return "F9: Oersted's compasses / tower of current rings"; }
    bool ownsField() const override { return true; }
    float fieldLineFlux() const override { return 0.8f; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        needles.clear();
        rings.clear();
        wireIndex.clear();
        if (mode == 0) buildOersted(host);
        else           buildTower(host);
        applyCurrent(host.world);
        wantCameraReset = true;
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 2; build(host); return; }
        if (IsKeyPressed(KEY_ENTER)) { build(host); return; }
        if (IsKeyPressed(KEY_X)) on = !on;
        if (IsKeyPressed(KEY_V)) { if (mode == 0) sign = -sign; else sameDir = !sameDir; }
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) scale = std::min(3.0f, scale * 1.25f);
        if (IsKeyPressed(KEY_LEFT_BRACKET))  scale = std::max(0.2f, scale / 1.25f);
        applyCurrent(w);
    }

    void draw3D(const World& w) const override {
        if (mode == 0) {   // 各方位磁針の N 極の向き（白い線）と、地磁気の向き
            for (const MagneticBody& mb : w.magnets().bodies()) {
                if (!w.isPinned(mb.body)) continue;
                const Vec3 d = normalize(mb.m) * 0.3f;
                DrawLine3D(toRay(mb.body->position), toRay(mb.body->position + d), RAYWHITE);
            }
            arrow(Vec3{-3.4f, TABLE_TOP + 0.05f, 3.0f}, Vec3{-2.6f, TABLE_TOP + 0.05f, 3.0f}, 0.02f, Fade(Color{120, 200, 255, 255}, 0.7f));
        }
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[220];
        const float I = scale * (mode == 0 ? I_WIRE : I_RING) * (on ? 1.0f : 0.0f);
        if (mode == 0) {
            const float Bw = 2.0f * std::fabs(I) * (float)sign / WIRE_H;   // 針の列の真上の直線の場（右ねじ: +x の電流は -z 向き）
            const float B0 = length(w.magnets().externalField);
            const float theory = std::atan2(-Bw, B0) * 180.0f / PHYS_PI;
            float measured = 0.0f;
            if (!needles.empty()) {
                const Vec3 m = w.magnets().bodies()[w.magnets().indexOf(needles[needles.size() / 2])].m;
                measured = std::atan2(m.z, m.x) * 180.0f / PHYS_PI;
            }
            out.add("Oersted (1820): a current above compass needles turns them (B circles the wire, right-hand rule)", COL_TITLE);
            std::snprintf(buf, sizeof buf, "wire current %s I = %.2f %s   B under the wire 2I/h = %.1f mT   'earth' field B0 = %.1f mT (+x)",
                          on ? "ON " : "OFF", std::fabs(I), I * (float)sign >= 0.0f ? "(+x)" : "(-x)", std::fabs(Bw) * FIELD_MT_PER_UNIT,
                          B0 * FIELD_MT_PER_UNIT);
            out.add(buf);
            std::snprintf(buf, sizeof buf, "needle under the wire: theory atan(B_wire / B0) = %+.1f deg   measured %+.1f deg   (farther rows turn less)",
                          theory, measured);
            out.add(buf);
        } else {
            out.add("rings of current: parallel currents attract, antiparallel repel (F ~ 4 pi a I^2 / d for close rings)", COL_TITLE);
            std::snprintf(buf, sizeof buf, "ring current %s %.3f x %d turns   direction %s", on ? "ON " : "OFF", std::fabs(I), RING_TURNS,
                          sameDir ? "all the same (attract -> stack)" : "alternating (repel -> float)");
            out.add(buf);
            std::string gaps = "gaps:";
            for (size_t i = 1; i < rings.size(); ++i) {
                std::snprintf(buf, sizeof buf, " %.2f", rings[i]->position.y - rings[i - 1]->position.y - 2.0f * RING_T);
                gaps += buf;
            }
            out.add(gaps.c_str(), LIGHTGRAY, 14);
        }
        out.gap(4);
        out.add("M layout   X current on/off   V direction   [ ] strength   Enter restart   O field lines", GRAY);
    }

private:
    static constexpr float TABLE_TOP = 0.5f, NEEDLE_Y = 0.65f, WIRE_H = 0.5f, I_WIRE = 1.1f, B_EARTH = 3.0f;
    static constexpr float RING_A = 0.6f, RING_T = 0.04f, RING_MASS = 0.3f, I_RING = 0.12f;
    static constexpr int   RING_TURNS = 10, RING_COUNT = 5;
    int   mode = 0, sign = 1;
    bool  on = false, sameDir = false;
    float scale = 1.0f;
    std::vector<RigidBody*> needles, rings;
    std::vector<int>        wireIndex;

    void applyCurrent(World& w) {
        const float I = scale * (on ? 1.0f : 0.0f);
        for (size_t i = 0; i < wireIndex.size(); ++i) {
            Wire& wire = w.currents().wires()[wireIndex[i]];
            if (mode == 0) wire.current = I * I_WIRE * (float)sign;
            else           wire.current = I * I_RING * ((sameDir || i % 2 == 0) ? 1.0f : -1.0f);
        }
    }

    // 机の上の 7 x 7 の方位磁針（位置を固定、弱い磁石）と、その上を通る直線の導線
    void buildOersted(SceneHost& host) {
        World& w = host.world;
        w.substeps = 4;
        w.magnets().externalField = Vec3{B_EARTH, 0, 0};   // 「地磁気」
        host.addStatic({0, 0.25f, 0}, {3.6f, 0.25f, 3.6f}, Color{92, 70, 52, 255});
        const int   n = 7;
        const float sp = 0.8f, half = 0.5f * (n - 1) * sp;
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < n; ++k) {
                // 弱い磁石（隣どうしの場が「地磁気」より十分弱くなるように）
                RigidBody* b = makeMagnetBall(w, {i * sp - half, NEEDLE_Y, k * sp - half}, {1, 0, 0}, false, 0.04f, 0.12f);
                b->angularDamping = 2.0f;
                w.setPinned(b, true);
                needles.push_back(b);
            }
        w.magnets().updateMoments(1);
        wireIndex.push_back(w.currents().addWire(nullptr, {{-6.0f, NEEDLE_Y + WIRE_H, 0}, {6.0f, NEEDLE_Y + WIRE_H, 0}}, false, 0.0f, 1, 0.035f));
        // 導線を支える柱
        host.addStatic({-5.0f, 0.5f * (NEEDLE_Y + WIRE_H), 0}, {0.05f, 0.5f * (NEEDLE_Y + WIRE_H), 0.05f}, Color{150, 158, 172, 255});
        host.addStatic({5.0f, 0.5f * (NEEDLE_Y + WIRE_H), 0}, {0.05f, 0.5f * (NEEDLE_Y + WIRE_H), 0.05f}, Color{150, 158, 172, 255});
        camTargetX = 0.4f; camTargetY = 0.6f; camTargetZ = 0.3f;
        camDistance = 8.5f; camPitch = 0.85f; camYaw = 0.0f;
    }

    // ガラスの筒の中の、電流の輪（薄い板の物体にコイルを付ける）
    void buildTower(SceneHost& host) {
        World& w = host.world;
        w.substeps = 4;
        const float inner = RING_A + 0.1f;
        addGlassTube(host, {0, 0, 0}, inner, 4.5f);
        for (int i = 0; i < RING_COUNT; ++i) {
            RigidBody* b = w.createBox({0, RING_T + 0.75f * (float)i, 0}, {RING_A + 0.08f, RING_T, RING_A + 0.08f}, RING_MASS);
            b->friction      = 0.0f;
            b->restitution   = 0.05f;
            b->linearDamping = 0.6f;    // 空気抵抗など（振動が数秒で収まるように）
            b->angularDamping = 1.0f;
            host.paint(b, Fade(Color{235, 225, 200, 255}, 0.35f));
            wireIndex.push_back(w.currents().addCoil({0, 0, 0}, Quat{}, RING_A, 32, 0.0f, RING_TURNS, b, 0.035f));
            rings.push_back(b);
        }
        on = true;
        camTargetX = 0.0f; camTargetY = 2.0f; camTargetZ = 0.0f;
        camDistance = 7.5f; camPitch = 0.15f; camYaw = 0.5f;
    }
};

// ===========================================================================
// F10 電磁石とコイルガン
// ===========================================================================
class ElectromagnetScene : public FunScene {
public:
    ElectromagnetScene() { timeScale = 1.0f; }
    const char* title() const override { return "F10: Electromagnet / coil gun"; }
    bool wantsFieldLines() const override { return mode == 0; }
    float fieldLineFlux() const override { return mode == 0 ? 3.0f : 1.0f; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        held.clear();
        coils.clear();
        stageSpeed.clear();
        stageTheory.clear();
        ball = nullptr;
        if (mode == 0) buildMagnet(host);
        else           buildGun(host);
        applyCurrent(w);
        wantCameraReset = true;
    }

    void beforeStep(World& w, float dt) override {
        if (mode != 1 || !ball) return;
        t += dt;
        // コイルガン: 球が近づくあいだだけ ON、球の中心が通ったら OFF（autoSwitch を切ると ON のまま）。
        // 切る判定は 1 ステップ（1/120 s）ごとなので、このステップの半分だけ先の位置で判定する（遅れて切ると引き戻される）
        for (size_t k = 0; k < coils.size(); ++k) {
            Wire& wire = w.currents().wires()[coils[k]];
            const bool approaching = ball->position.x + 0.5f * ball->velocity.x * dt < COIL_X[k];
            wire.current = (started && (approaching || !autoSwitch)) ? I_GUN * scale : 0.0f;
            if (started && !passed[k] && ball->position.x > COIL_X[k] + 1.0f) {   // 段の後ろで速さを測る
                passed[k]     = true;
                stageSpeed[k] = ball->velocity.x;
            }
        }
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 2; build(host); return; }
        if (IsKeyPressed(KEY_ENTER)) {
            if (mode == 1 && !started) started = true;
            else build(host);
            return;
        }
        if (IsKeyPressed(KEY_X)) on = !on;
        if (IsKeyPressed(KEY_V)) autoSwitch = !autoSwitch;
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) scale = std::min(3.0f, scale * 1.25f);
        if (IsKeyPressed(KEY_LEFT_BRACKET))  scale = std::max(0.2f, scale / 1.25f);
        if (mode == 0) applyCurrent(w);
    }

    void draw3D(const World& w) const override {
        if (mode == 1) {   // 段の後ろの測る位置（床の線）
            for (float x : COIL_X)
                DrawLine3D(Vector3{x + 1.0f, RAIL_TOP + 0.005f, -0.5f}, Vector3{x + 1.0f, RAIL_TOP + 0.005f, 0.5f}, Fade(RAYWHITE, 0.5f));
        }
        (void)w;
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[240];
        if (mode == 0) {
            const Vec3  Bapplied = w.currents().fieldAt(core ? core->position : Vec3{0, 0, 0});
            const MagneticBody* mb = core ? &w.magnets().bodies()[w.magnets().indexOf(core)] : nullptr;
            int lifted = 0;
            for (RigidBody* b : held) lifted += b->position.y > TABLE_TOP + 0.3f ? 1 : 0;
            out.add("electromagnet: the coil's field magnetizes the iron core (m = alpha B), the core pulls the iron below", COL_TITLE);
            std::snprintf(buf, sizeof buf, "current %s %.2f x %d turns   coil field at the core %.0f mT   core moment %.1f (%.0f%% of saturation)",
                          on ? "ON " : "OFF", I_MAG * scale, SOL_RINGS * SOL_TURNS, length(Bapplied) * FIELD_MT_PER_UNIT,
                          mb ? length(mb->m) : 0.0f, mb && mb->saturation > 0.0f ? 100.0f * length(mb->m) / mb->saturation : 0.0f);
            out.add(buf);
            std::snprintf(buf, sizeof buf, "iron pieces lifted %d / %d   (the ones under the pole jump up and stick; the others only tilt toward it)",
                          lifted, (int)held.size());
            out.add(buf);
            out.add("field lines: the flux crowds into the core and leaves through its end (the pole)", GRAY, 14);
        } else {
            out.add("coil gun: each coil is ON only while the ball approaches, OFF once the center passes", COL_TITLE);
            std::snprintf(buf, sizeof buf, "coil current %.2f x %d turns   switching %s   ball %.2f m/s   %s", I_GUN * scale, GUN_TURNS,
                          autoSwitch ? "automatic" : "OFF (stays on: the coil pulls the ball back)", ball ? ball->velocity.x : 0.0f,
                          started ? "" : "Enter to fire");
            out.add(buf);
            for (size_t k = 0; k < coils.size(); ++k) {
                std::snprintf(buf, sizeof buf, "  after coil %zu: %s%.2f m/s   theory %.1f (rolling) to %.1f (sliding)", k + 1, passed[k] ? "" : "-",
                              passed[k] ? stageSpeed[k] : 0.0f, stageTheory[k], stageTheory[k] * std::sqrt(1.4f));
                out.add(buf, LIGHTGRAY, 14);
            }
            out.add("theory: v^2 = v_prev^2 + 2 dU / M, dU = coil energy from the start to the center", GRAY, 14);
            out.add("(rolling puts 2/7 of it into spin; the pull is strong enough to make the ball slip, so it lands in between)", GRAY, 14);
        }
        out.gap(4);
        out.add(mode == 0 ? "M layout   X current on/off   [ ] strength   Enter restart   O field lines"
                          : "M layout   Enter fire / restart   V switching on/off   [ ] strength   O field lines", GRAY);
    }

private:
    static constexpr float TABLE_TOP = 0.97f, CORE_HX = 0.15f, CORE_HY = 0.7f, CORE_BOTTOM = 1.45f, BOBBIN_H = 0.3f;
    static constexpr float SOL_A = 0.42f, SOL_L = 1.2f, I_MAG = 0.10f;
    static constexpr int   SOL_RINGS = 6, SOL_SEG = 24, SOL_TURNS = 20;
    static constexpr float RAIL_TOP = 0.5f, GUN_A = 0.45f, I_GUN = 0.5f;
    static constexpr int   GUN_TURNS = 10;
    static constexpr float COIL_X[3] = {0.0f, 4.0f, 8.0f};
    int   mode = 0;
    bool  on = false, autoSwitch = true, started = false;
    bool  passed[3] = {false, false, false};
    float scale = 1.0f, t = 0.0f;
    RigidBody* core = nullptr;
    RigidBody* ball = nullptr;
    std::vector<RigidBody*> held;
    std::vector<int>   coils;
    std::vector<float> stageSpeed, stageTheory;

    void applyCurrent(World& w) {
        if (mode != 0) return;
        for (int ci : coils) w.currents().wires()[ci].current = on ? I_MAG * scale : 0.0f;
    }

    // 腕から下がる鉄の芯にソレノイドを巻く。下の机に鉄球と釘
    void buildMagnet(SceneHost& host) {
        World& w = host.world;
        w.substeps = 8;   // 芯のすぐ下では場の勾配が急で、軽い釘が強く加速する
        const Vec3  he{CORE_HX, CORE_HY, CORE_HX};
        const float V = 8.0f * he.x * he.y * he.z;
        const Vec3  cpos{0, CORE_BOTTOM + CORE_HY, 0};
        core = w.createSoftMagnetBox(cpos, he, 0.0f, boxSusceptibility(he, IRON_CHI), IRON_SAT * MAG_M0 * V / SPHERE_VOL);
        host.addStatic({0, cpos.y + CORE_HY + 0.4f, 0}, {0.1f, 0.4f, 0.1f}, Color{150, 158, 172, 255});       // 芯を支える柱
        host.addStatic({1.5f, cpos.y + CORE_HY + 0.8f, 0}, {1.6f, 0.06f, 0.2f}, Color{150, 158, 172, 255});   // 腕
        host.addStatic({3.0f, 0.5f * (cpos.y + CORE_HY + 0.8f), 0}, {0.1f, 0.5f * (cpos.y + CORE_HY + 0.8f), 0.1f}, Color{150, 158, 172, 255});
        const int first = w.currents().addSolenoid(cpos, Quat{}, SOL_A, SOL_L, SOL_RINGS, SOL_SEG, 0.0f, SOL_TURNS, nullptr, 0.035f);
        for (int k = 0; k < SOL_RINGS; ++k) coils.push_back(first + k);
        // 巻き枠（コイルと芯の間に物が入り込まないように。入ると芯の横腹に貼り付いて跳ね回る）
        host.addStatic(cpos, {BOBBIN_H, 0.5f * SOL_L, BOBBIN_H}, Color{60, 55, 60, 120}, 0.6f, 0.05f);
        // 机と、鉄球・鉄の塊（芯の近くに置く。離れていると持ち上がらない）
        host.addStatic({0, 0.5f * TABLE_TOP, 0}, {1.6f, 0.5f * TABLE_TOP, 1.2f}, Color{92, 70, 52, 255});
        const Vec3 balls[3] = {{0.0f, 0, 0.0f}, {0.42f, 0, 0.12f}, {-0.38f, 0, -0.2f}};
        for (const Vec3& s : balls) held.push_back(makeIronBall(w, s + Vec3{0, TABLE_TOP + 0.2f, 0}, 0.2f));
        const Vec3 cubes[3] = {{0.0f, 0, 0.42f}, {-0.3f, 0, 0.33f}, {0.3f, 0, -0.35f}};
        for (int k = 0; k < 3; ++k) {
            const Quat q = Quat::fromAxisAngle({0, 1, 0}, 0.4f + 1.1f * k);
            held.push_back(makeIronBox(w, cubes[k] + Vec3{0, TABLE_TOP + 0.1f, 0}, {0.1f, 0.1f, 0.1f}, q));
        }
        for (RigidBody* b : held) {   // 芯に強く引かれてぶつかるので、跳ね返さない（震えないように）
            b->linearDamping = 0.3f;
            b->restitution   = 0.05f;
            b->friction      = 0.6f;
        }
        camTargetX = 0.0f; camTargetY = 1.7f; camTargetZ = 0.0f;
        camDistance = 7.0f; camPitch = 0.2f; camYaw = 0.5f;
    }

    // レールの上の鉄球と、3 つのコイル
    void buildGun(SceneHost& host) {
        World& w = host.world;
        w.substeps = 8;
        started = false;
        t = 0.0f;
        for (bool& p : passed) p = false;
        RigidBody* rail = host.addStatic({4.5f, RAIL_TOP - 0.03f, 0}, {7.5f, 0.03f, 0.6f}, Color{150, 158, 172, 220}, 0.7f, 0.05f);
        (void)rail;
        const float by = RAIL_TOP + MAG_RADIUS;
        ball = makeIronBall(w, {-1.0f, by, 0});
        ball->linearDamping = 0.0f;
        const Quat q = Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);   // ローカル +y → +x
        for (float x : COIL_X) coils.push_back(w.currents().addCoil({x, by, 0}, q, GUN_A, 24, 0.0f, GUN_TURNS, nullptr, 0.035f));
        stageSpeed.assign(coils.size(), 0.0f);
        // 理論: 段ごとの dU（球は床を転がるので、運動エネルギーは 7/5 M v^2 / 2）
        stageTheory.assign(coils.size(), 0.0f);
        float v2 = 0.0f;
        for (size_t k = 0; k < coils.size(); ++k) {
            const float x0 = k == 0 ? -1.0f : COIL_X[k] - 4.0f;
            w.currents().wires()[coils[k]].current = I_GUN * scale;
            w.currents().prepare();
            w.magnets().extraField = &w.currents();
            auto U = [&](float x) { ball->position.x = x; w.magnets().updateMoments(30); return w.magnets().potentialEnergy(); };
            const float dU = U(x0) - U(COIL_X[k]);
            w.currents().wires()[coils[k]].current = 0.0f;
            v2 += 2.0f * dU * ball->invMass / 1.4f;
            stageTheory[k] = std::sqrt(std::max(0.0f, v2));
        }
        ball->position.x = -1.0f;
        w.magnets().updateMoments(30);
        camTargetX = 4.0f; camTargetY = 1.5f; camTargetZ = 0.0f;   // 注視点を上にして、レールを画面の下に出す（HUD にかくれない）
        camDistance = 9.5f; camPitch = 0.22f; camYaw = 0.0f;
    }
};

constexpr float ElectromagnetScene::COIL_X[3];

// ===========================================================================
// F11 ヘルムホルツコイル
// ===========================================================================
class HelmholtzScene : public FunScene {
public:
    HelmholtzScene() { timeScale = 1.0f; }
    const char* title() const override { return "F11: Helmholtz coils (charged particles in a uniform field)"; }
    bool ownsField() const override { return true; }
    float fieldLineFlux() const override { return 1.2f; }

    void build(SceneHost& host) override {
        host.reset();
        World& w = host.world;
        w.gravity  = Vec3{0, 0, 0};
        w.substeps = 4;
        w.electric().kappa = KAPPA;
        trails.clear();
        coils.clear();
        const float a = A;
        if (mode == 0) {          // ヘルムホルツ: 間隔 = 半径
            coils.push_back(w.currents().addCoil({0, -0.5f * a, 0}, Quat{}, a, 48, I0, 1, nullptr, 0.04f));
            coils.push_back(w.currents().addCoil({0, 0.5f * a, 0}, Quat{}, a, 48, I0, 1, nullptr, 0.04f));
        } else if (mode == 1) {   // 離した 2 つのコイル: 磁気瓶（ミラー装置）。回る半径がコイルより十分小さくなるように電流を強く
            coils.push_back(w.currents().addCoil({0, -BOTTLE_H, 0}, Quat{}, a, 48, 4.0f * I0, 1, nullptr, 0.04f));
            coils.push_back(w.currents().addCoil({0, BOTTLE_H, 0}, Quat{}, a, 48, 4.0f * I0, 1, nullptr, 0.04f));
        } else {                  // 逆向きの 2 つ（中心で B = 0。カスプ）
            coils.push_back(w.currents().addCoil({0, -0.5f * a, 0}, Quat{}, a, 48, I0, 1, nullptr, 0.04f));
            coils.push_back(w.currents().addCoil({0, 0.5f * a, 0}, Quat{}, a, 48, -I0, 1, nullptr, 0.04f));
        }
        Bc = w.currents().fieldAt({0, 0, 0}).y;
        auto particle = [&](const Vec3& pos, float q, const Vec3& v) {
            RigidBody* b = w.createChargedSphere(pos, 0.06f, 1.0f, q, false);
            b->velocity       = v;
            b->linearDamping  = 0.0f;
            b->angularDamping = 0.0f;
            Trail tr;
            tr.body = b;
            tr.q    = q;
            trails.push_back(tr);
        };
        if (mode == 0) {   // 中心の面（y = 0）から出す。軸から離れると、弱い勾配（ミラー力）で外へ押されるため
            particle({0.5f, 0.0f, 0}, 0.1f, {0, 0, 2.0f});                   // 円（半径 m v / (kappa q B)）
            particle({0.0f, 0.3f, 0.6f}, -0.1f, {0, 0, 2.0f});              // 逆向きに回る
            particle({-0.5f, 0.0f, 0}, 0.2f, {0, 0.06f, 2.0f});             // 電荷 2 倍: 半径半分。ゆっくり軸方向に進む（らせん）
        } else if (mode == 1) {   // 磁気瓶: v_perp が十分あれば、コイルの手前（場が強くなるところ）で跳ね返る。少ないと抜ける（ロスコーン）
            particle({0.3f, 0.0f, 0}, 0.1f, {0, 1.0f, 1.6f});
            particle({-0.3f, 0.0f, 0.3f}, -0.1f, {0, -1.0f, -1.6f});
            particle({0.0f, 0.0f, -0.4f}, 0.2f, {0, 1.6f, 1.0f});           // v_perp が小さい: 抜けていく
        } else {                  // カスプ: 上下で B の向きが逆なので、同じ電荷が逆向きに回る。中心は B = 0 でまっすぐ
            particle({0.3f, 0.6f, 0}, 0.1f, {0, 0, 2.0f});
            particle({0.3f, -0.6f, 0}, 0.1f, {0, 0, 2.0f});
            particle({-1.2f, 0.0f, 0.2f}, 0.2f, {1.5f, 0, 0});
        }
        chargeRef = 0.2f;
        camTargetX = 0.0f; camTargetY = -0.2f; camTargetZ = 0.0f;
        camDistance = mode == 1 ? 10.0f : 8.0f; camPitch = 0.25f; camYaw = 0.6f;
        wantCameraReset = true;
    }

    void beforeStep(World&, float) override {
        for (Trail& tr : trails) tr.push(2400);
    }

    void handleInput(SceneHost& host) override {
        if (IsKeyPressed(KEY_M)) { mode = (mode + 1) % 3; build(host); }
        if (IsKeyPressed(KEY_ENTER)) build(host);
    }

    void draw3D(const World&) const override {
        for (const Trail& tr : trails) tr.draw();
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[240];
        const float kappa = w.electric().kappa;
        static const char* names[] = {"Helmholtz pair (spacing = radius): nearly uniform field between the coils",
                                      "two coils far apart = magnetic bottle: particles with enough v_perp bounce between the coils",
                                      "anti-Helmholtz (opposite currents): B = 0 at the center (cusp). Above and below, the same charge turns the opposite way"};
        out.add(names[mode], COL_TITLE);
        if (mode == 0) {
            const float theory = std::pow(0.8f, 1.5f) * 4.0f * PHYS_PI * I0 / A;
            float nonUni = 0.0f;
            for (float y : {-0.25f * A, 0.0f, 0.25f * A})
                for (float x : {0.0f, 0.25f * A}) nonUni = std::max(nonUni, std::fabs(length(w.currents().fieldAt({x, y, 0})) / Bc - 1.0f));
            std::snprintf(buf, sizeof buf, "B at center %.2f mT   theory (4/5)^1.5 4 pi I / a = %.2f mT   within a/4 of the center: %.1f%% uniform",
                          Bc * FIELD_MT_PER_UNIT, theory * FIELD_MT_PER_UNIT, 100.0f * nonUni);
            out.add(buf);
        } else if (mode == 1) {
            const float Bm = w.currents().fieldAt({0, BOTTLE_H, 0}).y;
            std::snprintf(buf, sizeof buf, "B at center %.2f mT, at a coil %.2f mT (mirror ratio %.1f): reflected if sin^2(pitch angle) > %.2f, else lost",
                          Bc * FIELD_MT_PER_UNIT, Bm * FIELD_MT_PER_UNIT, Bm / Bc, Bc / Bm);
            out.add(buf);
        } else {
            std::snprintf(buf, sizeof buf, "B at center %.2f mT   at y = a/2 %.2f mT", Bc * FIELD_MT_PER_UNIT,
                          w.currents().fieldAt({0, 0.5f * A, 0}).y * FIELD_MT_PER_UNIT);
            out.add(buf);
        }
        std::snprintf(buf, sizeof buf, "F = q (E + kappa v x B), kappa = %.0f (this world's 1/c)", kappa);
        out.add(buf, LIGHTGRAY, 14);
        for (const Trail& tr : trails) {
            const Vec3  B  = ElectricSystem::magneticFieldAt(w.magnets(), tr.body->position, tr.body->id, &w.currents());
            const float Bl = length(B);
            const Vec3  vp = tr.body->velocity - B * (dot(tr.body->velocity, B) / std::max(Bl * Bl, 1e-12f));
            const float om = kappa * std::fabs(tr.q) * Bl / (1.0f / tr.body->invMass);
            std::snprintf(buf, sizeof buf, "  q %+.1f: |B| here %.2f mT   radius m v_perp / (kappa q B) = %.3f m   period %.2f s   speed %.3f", tr.q,
                          Bl * FIELD_MT_PER_UNIT, om > 1e-6f ? length(vp) / om : 0.0f, om > 1e-6f ? 2.0f * PHYS_PI / om : 0.0f,
                          length(tr.body->velocity));
            out.add(buf, LIGHTGRAY, 14);
        }
        out.gap(4);
        out.add("M layout (Helmholtz / bottle / cusp)   Enter restart   O field lines", GRAY);
    }

private:
    static constexpr float A = 1.5f, I0 = 1.0f, KAPPA = 10.0f, BOTTLE_H = 2.0f;
    int   mode = 0;
    float Bc = 0.0f;
    std::vector<Trail> trails;
    std::vector<int>   coils;
};

std::unique_ptr<FunScene> makeOerstedScene()       { return std::make_unique<OerstedScene>(); }
std::unique_ptr<FunScene> makeElectromagnetScene() { return std::make_unique<ElectromagnetScene>(); }
std::unique_ptr<FunScene> makeHelmholtzScene()     { return std::make_unique<HelmholtzScene>(); }
