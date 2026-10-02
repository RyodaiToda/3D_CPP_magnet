// ---------------------------------------------------------------------------
// flow_main.cpp : MAGNET FLOW（3D 磁気アクションパズル）のプロトタイプ
//
//   仕様: magnetic_flow_spec.md の §3（プロトタイプ版）と、
//         magnetic_flow_gimmick_stage_spec.md のプロトタイプ（01 磁気峡谷、02 軌道庭園、04 磁気工場）と
//         全部のギミックを入れた大きなマップ GRAND TOUR（9 区間）と、新しいギミックのステージ L1〜L4（§17:
//         サイクロトロン、ガウス加速リング、共振ブランコ、鉄球のロープ）
//   操作:
//     Space / 左クリック / パッド A : 開始、極性切替（N <-> S。押した瞬間に反応）。発射台の上では発射
//     R / パッド Y                  : すぐにやり直す（GRAND TOUR では最後のチェックポイントへ）
//     Shift+R / Backspace           : GRAND TOUR を始めからやり直す
//     [ / ]                         : GRAND TOUR の前・次のチェックポイントから練習する（記録しない）
//     M                             : マップ全体を見る
//     1 - 8                         : ステージ（S0、P1、P2、P3、01、02、04、GRAND TOUR）
//     9 0 - =                       : 新しいギミックのステージ（L1 サイクロトロン、L2 加速リング、L3 共振ブランコ、L4 鉄球のロープ）
//     Enter                         : クリアしたら次のステージへ
//     右ドラッグ / 右スティック     : カメラを一時的に回す（離すと戻る）
//     G ゴースト（最高記録の走り）  P 最高記録の入力を流し直す（リプレイ）
//     F1 調整パネル（↑↓ 項目、←→ 値）  F2 力の表示  F3 磁力線
//     Tab 自由カメラ  H HUD
//   最高記録（時間・切替・最大速度・連続磁気イベント・使った磁石・リトライ・入力・軌跡、
//   GRAND TOUR はチェックポイントの時刻と落ちた回数も）は実行ファイルの隣の magnet_flow_records.txt に保存する
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "field_lines.h"
#include "keymap.h"
#include "flow/flow_audio.h"
#include "flow/flow_records.h"
#include "flow/flow_stages.h"
#include "flow/flow_world.h"
#include "hud_lines.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace phys;
using namespace flow;

namespace {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }
inline Vec3    toPhys(const Vector3& v) { return Vec3{v.x, v.y, v.z}; }

const Color COL_N      {232, 72, 62, 255};
const Color COL_S      {62, 122, 240, 255};
const Color COL_SOLID  {78, 86, 102, 255};
const Color COL_IRON   {150, 152, 160, 255};
const Color COL_DOOR   {205, 150, 70, 255};
const Color COL_BG     {22, 24, 32, 255};

Color polColor(int p) { return p > 0 ? COL_N : COL_S; }

// 調整パネルの項目
struct Tunable { const char* name; float* value; float step; float lo, hi; };

// ---------------------------------------------------------------------------
// 描画の部品
// ---------------------------------------------------------------------------
struct Models { Model cube{}, sphere{}, hemi{}; };

void drawOrientedModel(const Model& m, const Vector3& pos, const Quat& q, const Vector3& scale, Color c) {
    Vector3 axis; float angle;
    QuaternionToAxisAngle(Quaternion{q.x, q.y, q.z, q.w}, &axis, &angle);
    if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};
    DrawModelEx(m, pos, axis, angle * RAD2DEG, scale, c);
}

void drawBox(const Models& md, const RigidBody* b, const Vec3& offset, const Vec3& he, Color c, bool wires = true) {
    const Vector3 pos = toRay(b->position + b->orientation.rotate(offset));
    drawOrientedModel(md.cube, pos, b->orientation, toRay(he * 2.0f), c);
    if (wires) {
        Vector3 axis; float angle;
        const Quat& q = b->orientation;
        QuaternionToAxisAngle(Quaternion{q.x, q.y, q.z, q.w}, &axis, &angle);
        if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};
        DrawModelWiresEx(md.cube, pos, axis, angle * RAD2DEG, toRay(he * 2.0f), Fade(BLACK, 0.35f));
    }
}

// 磁石: ローカル +y 側を N（赤）、-y 側を S（青）で塗り分ける
void drawMagnet(const Models& md, const RigidBody* b, float brighten) {
    auto lift = [&](Color c) {   // brighten > 0 で明るく、< 0 で暗く（点滅して消えている磁石）
        auto ch = [&](unsigned char v) { return (unsigned char)clampf(v + 120.0f * brighten, 0.0f, 255.0f); };
        return Color{ch(c.r), ch(c.g), ch(c.b), 255};
    };
    if (b->shape.type == ShapeType::Box) {
        Vec3 half = b->shape.halfExtents;
        half.y *= 0.5f;
        drawBox(md, b, Vec3{0,  half.y, 0}, half, lift(COL_N));
        drawBox(md, b, Vec3{0, -half.y, 0}, half, lift(COL_S));
    } else {
        const float r = b->shape.radius;
        drawOrientedModel(md.hemi, toRay(b->position), b->orientation, Vector3{r, r, r}, lift(COL_N));
        drawOrientedModel(md.hemi, toRay(b->position), b->orientation * Quat::fromAxisAngle({1, 0, 0}, PHYS_PI),
                          Vector3{r, r, r}, lift(COL_S));
    }
}

// 磁力線の色（|B| の対数）
Color fieldColor(float B) {
    const float t = clampf((std::log10(std::max(B, 1e-6f)) + 1.0f) / 3.5f, 0.0f, 1.0f);
    return ColorLerp(Color{60, 80, 170, 200}, Color{250, 230, 120, 230}, t);
}

// 光線と静的な箱（カメラのめり込み防止）。当たった距離（0〜1）、なければ 1
float rayToStatics(const World& w, const Vec3& from, const Vec3& to) {
    const Vec3 d = to - from;
    float best = 1.0f;
    for (const auto& bp : w.getBodies()) {
        const RigidBody* b = bp.get();
        if (!b->isStatic() || b->shape.type != ShapeType::Box) continue;
        const Mat3 Rt = b->orientation.toMat3().transposed();
        const Vec3 o = Rt * (from - b->position), dl = Rt * d;
        const Vec3 he = b->shape.halfExtents + Vec3{0.2f, 0.2f, 0.2f};
        float t0 = 0.0f, t1 = 1.0f;
        bool hit = true;
        for (int k = 0; k < 3 && hit; ++k) {
            if (std::fabs(dl[k]) < 1e-6f) { hit = std::fabs(o[k]) <= he[k]; continue; }
            float a = (-he[k] - o[k]) / dl[k], c = (he[k] - o[k]) / dl[k];
            if (a > c) std::swap(a, c);
            t0 = std::max(t0, a);
            t1 = std::min(t1, c);
            hit = t0 <= t1;
        }
        if (hit && t0 > 0.0f) best = std::min(best, t0);
    }
    return best;
}

// 軌道コア: 外を向いている極の色の球に、回転の向きに回る経線と、渦の届く範囲の輪を重ねる
void drawCore(const Models& md, const Core& c, float glow) {
    const RigidBody* b = c.body;
    const float r = b->shape.radius;
    const Color base = c.pole > 0 ? COL_N : COL_S;
    const Color col  = ColorLerp(base, RAYWHITE, 0.25f * glow);
    DrawModel(md.sphere, toRay(b->position), r, col);
    Vec3 e1, e2;
    buildTangents(c.axis, e1, e2);
    if (dot(cross(e1, e2), c.axis) < 0.0f) std::swap(e1, e2);   // e1 → e2 が回転の向き
    const Vec3 o = b->position;
    // 経線（回転の向きに回る）
    for (int k = 0; k < 6; ++k) {
        const float ph = c.spin + (float)k * PHYS_PI / 3.0f;
        const Vec3  d  = e1 * std::cos(ph) + e2 * std::sin(ph);
        Vec3 prev = o + (d * std::cos(-1.3f) + c.axis * std::sin(-1.3f)) * (r * 1.02f);
        for (int i = 1; i <= 12; ++i) {
            const float th = -1.3f + 2.6f * (float)i / 12.0f;
            const Vec3  q  = o + (d * std::cos(th) + c.axis * std::sin(th)) * (r * 1.02f);
            DrawLine3D(toRay(prev), toRay(q), Fade(RAYWHITE, 0.55f));
            prev = q;
        }
    }
    // 赤道と、渦の届く範囲（点線）と、回転の向きに流れる矢
    auto ring = [&](float rad, int n, float dash, Color cc) {
        for (int i = 0; i < n; ++i) {
            const float a0 = 2.0f * PHYS_PI * (float)i / n, a1 = a0 + 2.0f * PHYS_PI * dash / n;
            DrawLine3D(toRay(o + (e1 * std::cos(a0) + e2 * std::sin(a0)) * rad),
                       toRay(o + (e1 * std::cos(a1) + e2 * std::sin(a1)) * rad), cc);
        }
    };
    ring(r * 1.03f, 48, 1.0f, Fade(RAYWHITE, 0.8f));
    const float reach = r + 0.5f + c.reach;
    ring(reach, 64, 0.5f, Fade(base, 0.35f + 0.4f * glow));
    const float mid = r + 0.5f + 0.45f * c.reach;
    for (int k = 0; k < 8; ++k) {
        const float a = 1.5f * c.spin + (float)k * PHYS_PI / 4.0f;
        const Vec3  p = o + (e1 * std::cos(a) + e2 * std::sin(a)) * mid;
        const Vec3  t = e2 * std::cos(a) - e1 * std::sin(a);            // 回転の向き
        const Vec3  n = e1 * std::cos(a) + e2 * std::sin(a);
        const Vec3  tail = p - t * 0.9f;
        const Color cc = Fade(RAYWHITE, 0.25f + 0.5f * glow);
        DrawLine3D(toRay(tail), toRay(p), cc);
        DrawLine3D(toRay(p), toRay(p - t * 0.3f + n * 0.2f), cc);
        DrawLine3D(toRay(p), toRay(p - t * 0.3f - n * 0.2f), cc);
    }
}

// 円（中心 o、半径 rad、面の法線 n）を点線で描く。frac は描く割合（0〜1、残り時間の弧など）
void drawRing(const Vec3& o, const Vec3& n, float rad, int segs, float dash, Color c, float frac = 1.0f) {
    Vec3 e1, e2;
    buildTangents(normalize(n), e1, e2);
    const int m = std::max(1, (int)(segs * clampf(frac, 0.0f, 1.0f)));
    for (int i = 0; i < m; ++i) {
        const float a0 = 2.0f * PHYS_PI * (float)i / segs, a1 = a0 + 2.0f * PHYS_PI * dash / segs;
        DrawLine3D(toRay(o + (e1 * std::cos(a0) + e2 * std::sin(a0)) * rad),
                   toRay(o + (e1 * std::cos(a1) + e2 * std::sin(a1)) * rad), c);
    }
}

// 矢印（from → to。太さ r）
void drawArrow(const Vec3& from, const Vec3& to, float r, Color c) {
    const Vec3  d = to - from;
    const float L = length(d);
    if (L < 1e-4f) return;
    const Vec3 u = d / L, mid = to - u * std::min(0.4f * L, 4.0f * r);
    DrawCylinderEx(toRay(from), toRay(mid), r, r, 6, c);
    DrawCylinderEx(toRay(mid), toRay(to), 2.4f * r, 0.0f, 8, c);
}

// 両面の三角形（下からも見える）
void drawTri2(const Vec3& a, const Vec3& b, const Vec3& c, Color col) {
    DrawTriangle3D(toRay(a), toRay(b), toRay(c), col);
    DrawTriangle3D(toRay(a), toRay(c), toRay(b), col);
}

// 塗った円盤（中心 o、法線 n、半径 rad）
void fillDisc(const Vec3& o, const Vec3& n, float rad, Color c, int segs = 40) {
    Vec3 e1, e2;
    buildTangents(normalize(n), e1, e2);
    for (int i = 0; i < segs; ++i) {
        const float a0 = 2.0f * PHYS_PI * (float)i / segs, a1 = 2.0f * PHYS_PI * (float)(i + 1) / segs;
        drawTri2(o, o + (e1 * std::cos(a0) + e2 * std::sin(a0)) * rad, o + (e1 * std::cos(a1) + e2 * std::sin(a1)) * rad, c);
    }
}

const Color COL_PAD   {60, 190, 175, 255};
const Color COL_STEEL {92, 108, 132, 255};

// ---------------------------------------------------------------------------
// ゲーム
// ---------------------------------------------------------------------------
struct Game {
    FlowWorld  fw;
    FlowAudio  audio;
    Models     md;
    int        stage = GRAND_STAGE;
    RecordBook book;
    std::string recordPath;

    // カメラ
    Camera3D cam{};
    Vec3  camDir{1, -0.3f, 0};
    Vec3  camTarget{0, 0, 0};
    float fov = 60.0f;
    float shake = 0.0f;
    Vec3  focusPoint{0, 0, 0};   // 連鎖で動いている物の中心
    float focusHold = 0.0f, focusMix = 0.0f;
    float yawOff = 0.0f, pitchOff = 0.0f, offIdle = 10.0f;
    bool  freeCam = false;
    Vec3  freePos{0, 5, 10};
    float freeYaw = 0.0f, freePitch = -0.3f;
    // 連鎖カメラ（画面の隅の小窓）: 連鎖がプレイヤーから離れたところで起きているときに出す
    RenderTexture2D chainRT{};
    bool     chainOk = false;
    Camera3D chainCam{};
    Vec3     chainFocus{0, 0, 0};
    float    chainShow = 0.0f;
    const Camera3D* drawCam = &cam;   // 描いているカメラ（切替の輪の向き）

    // 演出
    float pulse = 0.0f;      // 切替の輪（1 → 0）
    float tint  = 0.0f;
    float chainMsg = 0.0f;
    float missMsg  = 0.0f;
    float lapMsg   = 0.0f;
    float clearTimer = 0.0f;
    bool  newBest  = false;
    StageRecord lastClear;   // 今回クリアしたときの記録（クリアの表示用）
    // GRAND TOUR
    bool  mapView    = false;   // マップ全体を見る（M）
    float zoneBanner = 0.0f;    // 区間に入ったときの名前の表示（残り時間）
    float cpMsg      = 0.0f;    // チェックポイントに着いたときの表示
    bool  hasSplit   = false;   // 最高記録とのチェックポイントの時刻の差
    float splitDelta = 0.0f;
    float magMsg     = 0.0f;    // 鋼が磁化した
    float boostMsg   = 0.0f;    // サイクロトロンの隙間・加速リングで押された

    bool showForces = true, showField = false, showTuning = false, showHud = true, showGhost = true;
    // 最高記録の入力をそのまま流し直す（P）。押した時刻はシミュレーションの時刻なので、同じ走りになる
    bool               replaying = false;
    std::vector<float> replayInputs, replayResets;   // 押した時刻と、手でチェックポイントに戻った時刻
    size_t             replayNext = 0, replayReset = 0;
    int  tuneSel = 0;
    std::vector<Tunable> tunables;
    demo::FieldLineTracer tracer;

    Game() {
        FlowParams& p = fw.params;
        tunables = {
            {"pull strength K",        &p.K,        25.0f, 25.0f, 2000.0f},
            {"iron pull K_iron",       &p.Kiron,    10.0f, 0.0f,  1000.0f},
            {"softening a [m]",        &p.soft,     0.1f,  0.2f,  4.0f},
            {"range Rc [m]",           &p.range,    0.5f,  2.0f,  40.0f},
            {"force cap [x weight]",   &p.forceCap, 0.5f,  0.5f,  30.0f},
            {"speed cap [m/s]",        &p.speedCap, 1.0f,  5.0f,  80.0f},
            {"gravity [x 9.81]",       &p.gravity,  0.05f, 0.0f,  2.0f},
            {"core push [x]",          &p.corePush, 0.1f,  0.0f,  4.0f},
            {"core lock [x]",          &p.coreLock, 0.1f,  0.0f,  4.0f},
            {"swing pump [x]",         &p.swingPump, 0.1f, 0.0f,  4.0f},
        };
        tracer.fluxPerLine = 30.0f;
        tracer.maxLines    = 200;
    }

    const StageRecord* bestRecord() const { return book.get(stageKey(stage)); }

    void load(int s) {
        stage = s;
        loadStage(fw, stage);
        tracer.reset();
        resetCamera();
        newBest = false;
        chainShow = 0.0f;
        hasSplit = false;
        zoneBanner = fw.bigMap() ? 3.0f : 0.0f;
    }
    // GRAND TOUR: チェックポイント cp の発射台から練習する
    void warp(int cp) {
        replaying = false;
        fw.warpTo(cp);
        tracer.reset();
        resetCamera();
        newBest = false;
        chainShow = 0.0f;
        hasSplit = false;
        zoneBanner = 3.0f;
    }
    void restart() {
        fw.restart();
        tracer.reset();
        missMsg = 0.0f;
        newBest = false;
        chainShow = 0.0f;
    }
    void startReplay() {
        const StageRecord* best = bestRecord();
        if (!best || best->inputs.empty()) return;
        restart();
        replayInputs = best->inputs;
        replayResets = best->resets;
        replayNext   = 0;
        replayReset  = 0;
        replaying    = true;
    }
    void feedReplay() {   // 固定刻みの前に呼ぶ
        while (replaying && replayReset < replayResets.size() && fw.state == State::Running &&
               fw.stats.time >= replayResets[replayReset] - 1e-4f) {
            fw.resetToCheckpoint();
            ++replayReset;
        }
        while (replaying && replayNext < replayInputs.size() &&
               (fw.state == State::Ready || (fw.state == State::Running && fw.stats.time >= replayInputs[replayNext] - 1e-4f))) {
            fw.press();
            ++replayNext;
        }
        // GRAND TOUR は落ちてもチェックポイントから続く
        if (replaying && fw.state != State::Running && fw.state != State::Ready && fw.state != State::Failed) replaying = false;
    }

    void resetCamera() {
        camDir = normalize(fw.viewDir);
        camTarget = fw.overview ? fw.viewCenter : fw.startPos;
        yawOff = pitchOff = 0.0f;
    }

    // --- 入力 ---
    void input(float dt) {
        const bool pad = IsGamepadAvailable(0);
        const bool pressSwitch = IsKeyPressed(KEY_SPACE) || (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !freeCam) ||
                                 (pad && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN));
        if (pressSwitch) { replaying = false; fw.press(); }
        const bool big = fw.bigMap();
        const bool full = IsKeyPressed(KEY_BACKSPACE) || (IsKeyPressed(KEY_R) && IsKeyDown(KEY_LEFT_SHIFT));
        if (full || IsKeyPressed(KEY_R) || (pad && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_UP))) {
            replaying = false;
            if (big && !full && fw.state == State::Running) { fw.resetToCheckpoint(); missMsg = 0.0f; }   // 最後のチェックポイントへ
            else if (!big || full || fw.state != State::Failed) { restart(); zoneBanner = big ? 3.0f : 0.0f; }
        }
        if (big && (IsKeyPressed(demo::keyLeftBracket()) || IsKeyPressed(demo::keyRightBracket()))) {   // 練習: 前・次のチェックポイント
            const int d  = IsKeyPressed(demo::keyRightBracket()) ? 1 : -1;
            const int cp = std::max(0, std::min(fw.checkpointTotal - 1, std::max(0, fw.checkpoint) + d));
            warp(cp);
        }
        if (IsKeyPressed(KEY_M)) mapView = !mapView;
        if (IsKeyPressed(KEY_P)) startReplay();
        static const int stageKeys[STAGE_COUNT] = {KEY_ONE, KEY_TWO,  KEY_THREE, KEY_FOUR,  KEY_FIVE,  KEY_SIX,
                                                   KEY_SEVEN, KEY_EIGHT, KEY_NINE, KEY_ZERO, KEY_MINUS, KEY_EQUAL};
        for (int k = 0; k < STAGE_COUNT; ++k)
            if (IsKeyPressed(stageKeys[k])) { replaying = false; load(k); }
        if (IsKeyPressed(KEY_ENTER) && fw.state == State::Cleared) { replaying = false; load((stage + 1) % STAGE_COUNT); }
        if (IsKeyPressed(KEY_F1)) showTuning = !showTuning;
        if (IsKeyPressed(KEY_F2)) showForces = !showForces;
        if (IsKeyPressed(KEY_F3)) { showField = !showField; tracer.reset(); }
        if (IsKeyPressed(KEY_H))  showHud = !showHud;
        if (IsKeyPressed(KEY_G))  showGhost = !showGhost;
        if (IsKeyPressed(KEY_TAB)) {
            freeCam = !freeCam;
            if (freeCam) {
                freePos = toPhys(cam.position);
                const Vec3 f = normalize(toPhys(cam.target) - freePos);
                freeYaw = std::atan2(f.x, f.z);
                freePitch = std::asin(clampf(f.y, -1.0f, 1.0f));
                DisableCursor();
            } else {
                EnableCursor();
            }
        }
        if (showTuning) {
            if (IsKeyPressed(KEY_UP))   tuneSel = (tuneSel + (int)tunables.size() - 1) % (int)tunables.size();
            if (IsKeyPressed(KEY_DOWN)) tuneSel = (tuneSel + 1) % (int)tunables.size();
            Tunable& t = tunables[tuneSel];
            const float dir = (IsKeyPressed(KEY_RIGHT) ? 1.0f : 0.0f) - (IsKeyPressed(KEY_LEFT) ? 1.0f : 0.0f);
            if (dir != 0.0f) *t.value = clampf(*t.value + dir * t.step, t.lo, t.hi);
        }
        // カメラを一時的に回す
        float dx = 0.0f, dy = 0.0f;
        if (!freeCam && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) { const Vector2 d = GetMouseDelta(); dx = d.x * 0.005f; dy = d.y * 0.004f; }
        if (pad) {
            dx += GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_X) * 2.0f * dt;
            dy += GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_Y) * 1.5f * dt;
        }
        if (std::fabs(dx) + std::fabs(dy) > 1e-4f) {
            yawOff -= dx;
            pitchOff = clampf(pitchOff - dy, -0.8f, 0.8f);
            offIdle = 0.0f;
        } else {
            offIdle += dt;
            if (offIdle > 1.5f) {   // 離して 1.5 s たったら戻す
                const float k = 1.0f - std::exp(-3.0f * dt);
                yawOff -= yawOff * k;
                pitchOff -= pitchOff * k;
            }
        }
    }

    // --- 出来事 → 音と演出 ---
    void react() {
        FlowEvents& e = fw.events;
        if (e.started)  audio.switched(fw.polarity);
        if (e.switched) { audio.switched(fw.polarity); pulse = 1.0f; tint = 1.0f; }
        if (e.kick)     { audio.kick(); shake = 1.0f; }
        if (e.hit)      audio.hit();
        if (e.capture && !e.kick) audio.capture();
        if (e.lap)      { audio.lap((int)fw.orbitAngle / 6); lapMsg = 1.0f; }
        if (e.chain)    { audio.chain(); chainMsg = 1.5f; }
        if (e.gateOpened) { audio.gate(); chainMsg = 2.0f; }
        if (e.failed)   { audio.fail(); missMsg = 1.0f; }
        if (e.launched) { audio.kick(); pulse = 1.0f; shake = 0.6f; }
        if (e.swing)    audio.capture();
        if (e.snapped)  { audio.hit(); shake = 0.8f; }
        if (e.magnetized) { audio.chain(); magMsg = 1.2f; }
        if (e.rotorDown)  { audio.gate(); chainMsg = 2.0f; shake = std::max(shake, 0.8f); }
        if (e.boost)      { audio.boost(length(fw.player->velocity)); boostMsg = 0.6f; }
        if (e.zoneEntered) zoneBanner = 3.0f;
        if (e.checkpoint) {
            audio.gate();
            cpMsg = 2.0f;
            const StageRecord* best = bestRecord();
            hasSplit = !fw.practice && best && fw.checkpoint < (int)best->splits.size() &&
                       fw.checkpoint < (int)fw.splits.size();
            if (hasSplit) splitDelta = fw.splits[fw.checkpoint] - best->splits[fw.checkpoint];
        }
        if (e.cleared) {
            audio.goal();
            lastClear = StageRecord::fromWorld(fw);
            newBest = !fw.practice && book.offer(stageKey(stage), lastClear);   // 練習（途中から）は記録しない
            if (newBest && !recordPath.empty()) book.save(recordPath);
        }
        e = FlowEvents{};
    }

    // --- カメラ ---
    void updateCamera(float dt) {
        if (freeCam) {
            const Vector2 d = GetMouseDelta();
            freeYaw -= d.x * 0.0025f;
            freePitch = clampf(freePitch - d.y * 0.0025f, -1.5f, 1.5f);
            const Vec3 fwd{std::cos(freePitch) * std::sin(freeYaw), std::sin(freePitch), std::cos(freePitch) * std::cos(freeYaw)};
            const Vec3 right = normalize(cross(fwd, Vec3{0, 1, 0}));
            const float v = (IsKeyDown(KEY_LEFT_SHIFT) ? 30.0f : 10.0f) * dt;
            if (IsKeyDown(KEY_W)) freePos += fwd * v;
            if (IsKeyDown(KEY_S)) freePos -= fwd * v;
            if (IsKeyDown(KEY_D)) freePos += right * v;
            if (IsKeyDown(KEY_A)) freePos -= right * v;
            if (IsKeyDown(KEY_E)) freePos.y += v;
            if (IsKeyDown(KEY_Q)) freePos.y -= v;
            cam.position = toRay(freePos);
            cam.target   = toRay(freePos + fwd);
            cam.fovy     = 60.0f;
            return;
        }
        if (mapView && fw.bigMap()) {   // マップ全体: 区間の箱をすべて入れる高さから斜めに見下ろす
            Vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
            for (const Zone& z : fw.zones) {
                if (z.lo.y < -1e5f) continue;
                for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], z.lo[k]); hi[k] = std::max(hi[k], z.hi[k]); }
            }
            const Vec3  c = (lo + hi) * 0.5f;
            const float R = 0.5f * length(hi - lo);
            const Vec3  d = normalize(Vec3{0.25f, -1.0f, 0.55f});
            const Vec3  want = c - d * (R / std::tan(25.0f * DEG2RAD) * 0.85f);   // 視野 50 度に収める
            cam.position = toRay(toPhys(cam.position) + (want - toPhys(cam.position)) * (1.0f - std::exp(-4.0f * dt)));
            cam.target   = toRay(toPhys(cam.target) + (c - toPhys(cam.target)) * (1.0f - std::exp(-4.0f * dt)));
            cam.fovy     = 50.0f;
            return;
        }
        const RigidBody* p = fw.player;
        const Vec3  v = p->velocity;
        const float speed = length(v);
        const float kSmooth = 1.0f - std::exp(-3.0f * dt);

        if (fw.overview) {   // 全体を見せる（軌道庭園）。プレイヤーの側へ少し寄せる
            Vec3 want = Quat::fromAxisAngle({0, 1, 0}, yawOff).rotate(normalize(fw.viewDir));
            const Vec3 side = normalize(cross(want, Vec3{0, 1, 0}));
            want = normalize(Quat::fromAxisAngle(side, pitchOff).rotate(want));
            camDir = normalize(camDir + (want - camDir) * kSmooth);
            const Vec3 target = fw.viewCenter + (p->position - fw.viewCenter) * 0.35f;
            camTarget = camTarget + (target - camTarget) * (1.0f - std::exp(-2.0f * dt));
            Vec3 pos = camTarget - camDir * fw.viewDist;
            if (shake > 0.0f) {
                const float a = 0.3f * shake * shake, tt = (float)GetTime();
                pos += Vec3{a * std::sin(tt * 71.0f), a * std::sin(tt * 53.0f + 1.0f), a * std::sin(tt * 61.0f + 2.0f)};
            }
            fov += (55.0f - fov) * (1.0f - std::exp(-4.0f * dt));
            cam.position = toRay(pos);
            cam.target   = toRay(camTarget);
            cam.fovy     = fov;
            return;
        }

        // 見る向き: 遅いときはステージの既定の向き、速いほど進む向きを混ぜる（軌道コアを回っている間は混ぜない）
        Vec3 want = normalize(fw.viewDir);
        if (speed > 2.0f && fw.state != State::Ready && fw.orbitCore < 0) {
            const float k = clampf((speed - 2.0f) / 10.0f, 0.0f, 0.65f);
            Vec3 vd = normalize(v);
            vd.y = clampf(vd.y, -0.6f, 0.4f);   // 真上・真下は見にくいので抑える
            want = normalize(want * (1.0f - k) + normalize(vd) * k);
        }
        // 手で回したぶん
        want = Quat::fromAxisAngle({0, 1, 0}, yawOff).rotate(want);
        const Vec3 side = normalize(cross(want, Vec3{0, 1, 0}));
        want = normalize(Quat::fromAxisAngle(side, pitchOff).rotate(want));
        camDir = normalize(camDir + (want - camDir) * kSmooth);

        Vec3  target = p->position + v * 0.12f;
        float dist   = 8.0f + 0.15f * speed;
        if (fw.orbitCore >= 0) {   // 回っているコアとプレイヤーの両方を入れる
            target = p->position + (fw.cores[fw.orbitCore].body->position - p->position) * 0.5f;
            dist  += 4.0f;
        }
        // 連鎖を見せる: プレイヤーが遅く、動いている物（開いているゲートを含む）があれば、両方が入るように引く
        // （連鎖カメラの小窓が使えるときは、そちらで見せる）
        if (!chainOk && speed < 3.0f && fw.state != State::Ready) {
            Vec3 focus{0, 0, 0};
            int  n = 0;
            for (const auto& bp : fw.world.getBodies()) {
                const Kind k = fw.kindOf(bp.get());
                if ((k == Kind::MovableMagnet || k == Kind::Iron) && length(bp->velocity) > 0.5f) { focus += bp->position; ++n; }
            }
            for (const Gate& g : fw.gates)
                if (g.opening && g.open < 1.0f) { focus += g.closedPos; ++n; }
            if (n > 0) { focusPoint = focus / (float)n; focusHold = 1.5f; }
        }
        if (focusHold > 0.0f) {
            focusMix = std::min(1.0f, focusMix + dt * 2.0f);
        } else {
            focusMix = std::max(0.0f, focusMix - dt * 1.5f);
        }
        focusHold = std::max(0.0f, focusHold - dt);
        if (focusMix > 0.0f) {
            const float m = focusMix * focusMix * (3.0f - 2.0f * focusMix);
            target = target + (focusPoint - target) * (0.5f * m);
            dist  += m * std::max(0.0f, 0.7f * length(focusPoint - p->position) - 3.0f);
        }
        camTarget = target;
        Vec3 pos = target - camDir * dist + Vec3{0, 2.0f, 0};
        // 壁や台にさえぎられたら、まず上へ逃がし、それでもだめなら手前に寄せる（3 m より近づけない）
        float t = rayToStatics(fw.world, target, pos);
        for (float lift = 2.0f; t < 1.0f && lift <= 8.0f; lift += 2.0f) {
            const Vec3 up = pos + Vec3{0, lift, 0};
            const float tu = rayToStatics(fw.world, target, up);
            if (tu >= 1.0f) { pos = up; t = 1.0f; }
        }
        if (t < 1.0f) pos = target + (pos - target) * std::max(std::min(1.0f, 3.0f / dist), t - 0.05f);
        if (shake > 0.0f) {
            const float a = 0.25f * shake * shake;
            const float tt = (float)GetTime();
            pos += Vec3{a * std::sin(tt * 71.0f), a * std::sin(tt * 53.0f + 1.0f), a * std::sin(tt * 61.0f + 2.0f)};
        }
        const float wantFov = 60.0f + 12.0f * clampf(speed / 20.0f, 0.0f, 1.0f);
        fov += (wantFov - fov) * (1.0f - std::exp(-4.0f * dt));
        cam.position = toRay(pos);
        cam.target   = toRay(target);
        cam.fovy     = fov;
    }

    // 連鎖カメラ: 動いている物（可動磁石・鉄・開いている扉）の中心を横から見る
    void updateChainCam(float dt) {
        chainShow = std::max(0.0f, chainShow - dt);
        if (!chainOk || fw.state == State::Ready || fw.overview) return;
        Vec3 focus{0, 0, 0};
        int  n = 0;
        for (const auto& bp : fw.world.getBodies()) {
            const Kind k = fw.kindOf(bp.get());
            if ((k == Kind::MovableMagnet || k == Kind::Iron) && length(bp->velocity) > 0.5f) { focus += bp->position; ++n; }
        }
        for (const Gate& g : fw.gates)
            if (g.opening && g.open < 1.0f) { focus += g.door->position; ++n; }
        if (n > 0) {
            focus = focus / (float)n;
            if (length(focus - fw.player->position) > 6.0f) {
                if (chainShow <= 0.0f) chainFocus = focus;
                chainShow = 1.2f;
            }
            chainFocus = chainFocus + (focus - chainFocus) * (1.0f - std::exp(-5.0f * dt));
        }
        if (chainShow <= 0.0f) return;
        const Vec3 d = normalize(Vec3{fw.viewDir.x, 0, fw.viewDir.z});
        const Vec3 s = normalize(cross(d, Vec3{0, 1, 0}));
        chainCam.position   = toRay(chainFocus + s * 4.5f - d * 2.5f + Vec3{0, 3.0f, 0});
        chainCam.target     = toRay(chainFocus);
        chainCam.up         = Vector3{0, 1, 0};
        chainCam.fovy       = 50.0f;
        chainCam.projection = CAMERA_PERSPECTIVE;
    }

    void renderChainCam() {
        if (!chainOk || chainShow <= 0.0f) return;
        BeginTextureMode(chainRT);
        ClearBackground(COL_BG);
        BeginMode3D(chainCam);
        drawCam = &chainCam;
        draw3D(false);
        drawCam = &cam;
        EndMode3D();
        EndTextureMode();
    }

    // 大きなマップのギミックの描画（発射台の矢印、振り子の綱、鋼の磁化、点滅の残り時間、磁針の軸、方位磁針）
    void drawGimmicks(bool main) {
        const Vec3 pp = fw.player->position;
        // 発射台: 発射の向きの矢印。台のないチェックポイントは光の輪
        for (int i = 0; i < (int)fw.pads.size(); ++i) {
            const Pad& pd = fw.pads[i];
            const float a = pd.cooldown > 0.0f ? 0.25f : (fw.seatedPad == i ? 1.0f : 0.6f);
            const bool  locked = pd.lockGate >= 0 && !fw.padActive(i);   // 扉が開くまでは発射しない
            if (lengthSq(pd.launch) > 0.0f) {
                const Vec3 d = normalize(pd.launch);
                drawArrow(pd.seat - Vec3{0, 0.4f, 0}, pd.seat - Vec3{0, 0.4f, 0} + d * 2.6f, 0.07f,
                          locked ? Fade(GRAY, 0.3f) : Fade(COL_PAD, a));
            }
            const Color rc = pd.checkpoint <= fw.checkpoint ? Color{120, 240, 170, 255} : Color{255, 220, 110, 255};
            drawRing(pd.seat - Vec3{0, 0.45f, 0}, {0, 1, 0}, pd.seatRadius, 40, 0.6f, Fade(rc, 0.5f));
            if (!pd.body) drawRing(pd.seat + Vec3{0, 1.0f, 0}, {0, 1, 0}, 0.9f, 24, 1.0f, Fade(rc, 0.8f));
        }
        // 磁気振り子: 綱がつながる距離（点線）と、つながっている綱
        for (const Swing& sw : fw.swings) {
            const Vec3  a = sw.body->position;
            const Vec3  n = lengthSq(sw.planeN) > 0.0f ? sw.planeN : Vec3{0, 0, 1};
            const Color pc2 = sw.pole > 0 ? COL_N : COL_S;
            const bool  near = length(pp - a) < sw.length + 8.0f;
            drawRing(a, n, sw.length, 72, 0.5f, Fade(pc2, near ? 0.45f : 0.2f));
            if (sw.engaged) {
                drawRing(a, n, sw.rod, 72, 1.0f, Fade(pc2, 0.8f));
                DrawCylinderEx(toRay(a), toRay(pp), 0.09f, 0.05f, 6, Fade(ColorLerp(pc2, RAYWHITE, 0.4f), 0.85f));
            }
        }
        // 磁化した鋼: 磁化の向きの矢印（N の先が赤）
        for (const Steel& st : fw.steels) {
            const Vec3  M = st.worldM();
            const float m = length(M);
            if (m < 0.05f) continue;
            const RigidBody* b = st.body;
            const float L = b->shape.type == ShapeType::Sphere ? b->shape.radius * 1.6f : 1.4f;
            const Vec3  u = M / m, c = b->position;
            DrawCylinderEx(toRay(c - u * L), toRay(c), 0.08f, 0.08f, 6, Fade(COL_S, 0.4f + 0.6f * m));
            drawArrow(c, c + u * L, 0.08f, Fade(COL_N, 0.4f + 0.6f * m));
            drawRing(c, u, L * 0.9f, 32, 0.6f, Fade(Color{255, 230, 120, 255}, 0.6f * m));
        }
        // 点滅する磁石: 次に切り替わるまでの残り時間の弧（点いているあいだ黄、消えているあいだ灰）
        for (const Animator& an : fw.animators) {
            if (an.pulsePeriod <= 0.0f) continue;
            const bool  on  = an.level > 0.5f;
            const float len = on ? an.pulseOn * an.pulsePeriod : (1.0f - an.pulseOn) * an.pulsePeriod;
            const float r   = an.body->shape.type == ShapeType::Sphere ? an.body->shape.radius + 0.5f : 2.0f;
            const Vec3  n   = normalize(toPhys(drawCam->position) - an.body->position);
            drawRing(an.body->position, n, r, 48, 1.0f, on ? Color{255, 220, 110, 255} : Fade(GRAY, 0.8f),
                     an.nextToggle / std::max(0.1f, len));
        }
        // 磁気トルクで回る板: 軸
        for (const Rotor& ro : fw.rotors)
            DrawCylinderEx(toRay(ro.pivot - ro.axis * 1.2f), toRay(ro.pivot + ro.axis * 1.2f), 0.25f, 0.25f, 10, COL_DOOR);
        // 方位磁針の並び: 今の極性のプレイヤーがそこで受ける力の向き（先が黄）。近いときだけ
        if (main) {
            for (const NeedleField& nf : fw.needles) {
                if (length(pp - nf.center) > 70.0f) continue;
                for (int i = 0; i < nf.nu; ++i) {
                    for (int j = 0; j < nf.nv; ++j) {
                        const Vec3 p = nf.center + nf.u * ((i - 0.5f * (nf.nu - 1)) * nf.step) + nf.v * ((j - 0.5f * (nf.nv - 1)) * nf.step);
                        Vec3 F = fw.forceAt(p, fw.polarity);
                        F = F - Vec3{0, 1, 0} * dot(F, Vec3{0, 1, 0}) * 0.7f;   // 水平の向きを見やすく
                        const float f = length(F);
                        if (f < 0.3f) continue;
                        const Vec3  u = F / f;
                        const float a = clampf(0.25f + f / 30.0f, 0.25f, 1.0f);
                        DrawCylinderEx(toRay(p - u * 0.55f), toRay(p), 0.05f, 0.05f, 4, Fade(GRAY, a));
                        DrawCylinderEx(toRay(p), toRay(p + u * 0.55f), 0.07f, 0.0f, 5, Fade(Color{255, 220, 110, 255}, a));
                    }
                }
            }
        }
    }

    // 新しいギミックの描画（サイクロトロン、加速リング、共振ブランコの振れ、鉄球のロープの先）
    void drawLab() {
        const Vec3  pp = fw.player->position;
        const float tt = (float)GetTime();
        // サイクロトロン: 2 つの半分を「そこで回るのに要る極と異極の色」（引き込む色）で塗る。隙間の縁、外の縁、出口の弦、
        // 回る向きに回る矢（回る速さ omega で回る = リズム）。極性が今いる半分に合わないと、その半分を白く点滅させる
        for (const Cyclotron& c : fw.cyclotrons) {
            const Vec3 e1 = c.gapN, e2 = normalize(cross(c.axis, c.gapN));
            const Vec3 o  = c.center - c.axis * 0.55f;   // 円盤はプレイヤーの下の面
            Vec3  rIn;
            float h;
            const bool  in = fw.cyclotronInside(c, pp, &rIn, &h);
            const float uP = dot(rIn, c.gapN);
            const float halfLen = std::sqrt(std::max(0.0f, c.radius * c.radius - c.gapHalf * c.gapHalf));
            for (int side = -1; side <= 1; side += 2) {
                Color col = side > 0 ? COL_S : COL_N;
                const bool wrong = in && fw.state == State::Running && std::fabs(uP) >= c.gapHalf &&
                                   (uP > 0.0f ? 1 : -1) == side && fw.polarity * side < 0;
                if (wrong) col = ColorLerp(col, RAYWHITE, 0.5f + 0.5f * std::sin(tt * 25.0f));
                const Vec3 n = e1 * (float)side;
                const int  N = 48;
                auto edge = [&](float th, float& r0, float& r1) {   // n から e2 の向きへ th。内は隙間の縁、外は縁か出口の弦
                    const Vec3  d  = n * std::cos(th) + e2 * std::sin(th);
                    const float cn = std::cos(th);
                    r0 = cn > 1e-3f ? c.gapHalf / cn : 1e9f;
                    r1 = c.radius;
                    const float cp = dot(d, c.portDir);
                    if (cp > 1e-3f) r1 = std::min(r1, c.portRadius / cp);
                    return d;
                };
                for (int i = 0; i < N; ++i) {
                    const float th0 = -0.5f * PHYS_PI + PHYS_PI * (float)i / N, th1 = th0 + PHYS_PI / N;
                    float a0, b0, a1, b1;
                    const Vec3 d0 = edge(th0, a0, b0), d1 = edge(th1, a1, b1);
                    if (a0 >= b0 || a1 >= b1) continue;
                    const Color fc = Fade(col, wrong ? 0.4f : 0.2f);
                    drawTri2(o + d0 * a0, o + d0 * b0, o + d1 * b1, fc);
                    drawTri2(o + d0 * a0, o + d1 * b1, o + d1 * a1, fc);
                }
            }
            // 隙間の縁（押されたら光る）と、曲げる力が全部になる縁（点線）
            const Color gc = ColorLerp(Color{255, 220, 110, 200}, RAYWHITE, c.flash);
            for (int side = -1; side <= 1; side += 2) {
                const Vec3 a = o + e1 * (c.gapHalf * side);
                DrawCylinderEx(toRay(a - e2 * halfLen), toRay(a + e2 * halfLen), 0.05f + 0.1f * c.flash, 0.05f + 0.1f * c.flash, 5, gc);
                const float uf = c.gapHalf + c.fringe;
                const float lf = std::sqrt(std::max(0.0f, c.radius * c.radius - uf * uf));
                for (int k = 0; k < 16; ++k) {
                    const float s0 = -lf + 2.0f * lf * (float)k / 16, s1 = s0 + lf / 16;
                    DrawLine3D(toRay(o + e1 * (uf * side) + e2 * s0), toRay(o + e1 * (uf * side) + e2 * s1), Fade(gc, 0.35f));
                }
            }
            if (c.flash > 0.0f) {   // 押された: 隙間の帯を塗る
                drawTri2(o - e1 * c.gapHalf - e2 * halfLen, o + e1 * c.gapHalf - e2 * halfLen, o + e1 * c.gapHalf + e2 * halfLen,
                         Fade(Color{255, 230, 140, 255}, 0.35f * c.flash));
                drawTri2(o - e1 * c.gapHalf - e2 * halfLen, o + e1 * c.gapHalf + e2 * halfLen, o - e1 * c.gapHalf + e2 * halfLen,
                         Fade(Color{255, 230, 140, 255}, 0.35f * c.flash));
            }
            // 外の縁と、出口の弦（緑）
            for (int i = 0; i < 96; ++i) {
                const float a0 = 2.0f * PHYS_PI * (float)i / 96, a1 = a0 + 2.0f * PHYS_PI / 96;
                const Vec3  d0 = e1 * std::cos(a0) + e2 * std::sin(a0), d1 = e1 * std::cos(a1) + e2 * std::sin(a1);
                if (dot(d0, c.portDir) * c.radius > c.portRadius || dot(d1, c.portDir) * c.radius > c.portRadius) continue;
                DrawLine3D(toRay(o + d0 * c.radius), toRay(o + d1 * c.radius), Fade(RAYWHITE, 0.6f));
            }
            const Vec3  pq = normalize(cross(c.axis, c.portDir));
            const float pl = std::sqrt(std::max(0.0f, c.radius * c.radius - c.portRadius * c.portRadius));
            DrawCylinderEx(toRay(o + c.portDir * c.portRadius - pq * pl), toRay(o + c.portDir * c.portRadius + pq * pl), 0.08f, 0.08f, 6,
                           Color{120, 240, 150, 255});
            // 回る向きの矢（omega で回る）
            for (int k = 0; k < 6; ++k) {
                const float a = c.omega * tt + (float)k * PHYS_PI / 3.0f;
                const Vec3  d = e1 * std::cos(a) + e2 * std::sin(a);
                const Vec3  t = normalize(cross(c.axis, d));
                const Vec3  q = o + d * (0.55f * c.radius) + c.axis * 0.02f;
                drawArrow(q - t * 0.9f, q + t * 0.5f, 0.05f, Fade(RAYWHITE, 0.35f));
            }
        }
        // 加速リング: 内側の極の色の太い輪。効く範囲にいる輪は明るく、面に薄い円盤（ここで切り替える）
        for (const GaussRing& g : fw.rings) {
            const Color pc2 = g.pole > 0 ? COL_N : COL_S;
            const Color col = ColorLerp(Fade(pc2, g.lit ? 1.0f : 0.55f), RAYWHITE, 0.7f * g.flash);
            for (int k = 0; k < 4; ++k) {
                const float dr = k < 2 ? -0.12f : 0.12f, dx = (k % 2) ? -0.12f : 0.12f;
                drawRing(g.center + g.axis * dx, g.axis, g.radius + 0.25f + dr, 48, 1.0f, col);
            }
            if (g.lit) fillDisc(g.center, g.axis, g.radius, Fade(pc2, 0.12f));
            if (g.flash > 0.0f) drawRing(g.center, g.axis, g.radius + 0.6f + 1.2f * (1.0f - g.flash), 48, 1.0f, Fade(RAYWHITE, g.flash));
        }
        // 共振ブランコ: おもりの通る弧（今の振れの大きさ）と、壁の角度の印
        for (const Rotor& ro : fw.rotors) {
            if (!ro.spendOnHi) continue;
            const Vec3 arm = ro.com - ro.pivot;
            auto at = [&](float a) { return ro.pivot + Quat::fromAxisAngle(ro.axis, a).rotate(arm); };
            const float pk = std::max(ro.peak, std::fabs(ro.angle));
            for (int i = 0; i < 48; ++i) {
                const float a0 = -pk + 2.0f * pk * (float)i / 48, a1 = a0 + 2.0f * pk / 48;
                DrawLine3D(toRay(at(a0)), toRay(at(a1)), Fade(Color{255, 190, 120, 255}, ro.reachedHi ? 0.25f : 0.6f));
            }
            const Vec3 tip = at(ro.hi), n = normalize(tip - ro.pivot);
            DrawCylinderEx(toRay(tip - n * 0.6f), toRay(tip + n * 0.6f), 0.07f, 0.07f, 6,
                           ro.reachedHi ? Color{120, 240, 150, 255} : Color{255, 120, 100, 255});
        }
        // 鉄球のロープ: つかむ先の球のまわりに、先の極の色の輪（カメラに向ける）
        for (const BallChain& ch : fw.chains) {
            if (ch.balls.empty()) continue;
            const RigidBody* tip = ch.balls.back();
            const int   pole = fw.uniformPole[tip->id];
            const Vec3  n    = normalize(toPhys(drawCam->position) - tip->position);
            const float near = length(pp - tip->position) < 1.0f ? 1.0f : 0.0f;
            drawRing(tip->position, n, tip->shape.radius + 0.25f + 0.05f * std::sin(tt * 6.0f), 32, 1.0f,
                     Fade(pole > 0 ? COL_N : COL_S, 0.5f + 0.5f * near));
        }
    }

    // --- 描画 ---
    void draw3D(bool main = true) {
        const World& w = fw.world;
        // 落ちたら失敗する高さに、遠くまで格子を敷く（奥行きの手がかり。遠いほど薄く）
        {
            const float cx0 = fw.player->position.x - std::fmod(fw.player->position.x, 2.0f);
            const float cz0 = fw.player->position.z - std::fmod(fw.player->position.z, 2.0f);
            const float y = fw.killY, L = 80.0f;
            for (int i = -40; i <= 40; ++i) {
                const float o = 2.0f * (float)i;
                const Color c = Fade(Color{120, 130, 150, 255}, 0.35f * (1.0f - std::fabs((float)i) / 41.0f));
                DrawLine3D(Vector3{cx0 + o, y, cz0 - L}, Vector3{cx0 + o, y, cz0 + L}, c);
                DrawLine3D(Vector3{cx0 - L, y, cz0 + o}, Vector3{cx0 + L, y, cz0 + o}, c);
            }
        }
        for (const auto& bp : w.getBodies()) {
            const RigidBody* b = bp.get();
            const Kind k = fw.kindOf(b);
            switch (k) {
            case Kind::Player: break;
            case Kind::FixedMagnet: {
                float lv = 1.0f;   // 点滅する磁石は、消えているあいだ暗く
                for (const Animator& a : fw.animators) if (a.body == b && a.pulsePeriod > 0.0f) lv = a.level;
                drawMagnet(md, b, -0.75f * (1.0f - lv));
                break;
            }
            case Kind::MovableMagnet: drawMagnet(md, b, 0.6f * fw.flash[b->id]); break;
            case Kind::Core: {
                const int ci = fw.coreOf[b->id];
                const Core& c = fw.cores[ci];
                if (c.plain) {   // 磁極（磁針の端、振り子のおもり。磁力が抜けたら灰色）
                    const float r = b->shape.radius;
                    DrawModel(md.sphere, toRay(b->position), r, fw.charge[b->id] > 0.0f ? (c.pole > 0 ? COL_N : COL_S) : GRAY);
                    DrawSphereWires(toRay(b->position), r * 1.05f, 6, 10, Fade(RAYWHITE, 0.35f));
                } else {
                    drawCore(md, c, ci == fw.orbitCore ? 1.0f : 0.0f);
                }
                break;
            }
            case Kind::Anchor: {   // 磁気振り子の錨
                const Swing& sw = fw.swings[fw.swingOf[b->id]];
                const float r = b->shape.radius;
                const Color pc2 = sw.pole > 0 ? COL_N : COL_S;
                DrawModel(md.sphere, toRay(b->position), r, ColorLerp(pc2, RAYWHITE, sw.engaged ? 0.35f : 0.0f));
                const Vec3 n = lengthSq(sw.planeN) > 0.0f ? sw.planeN : Vec3{0, 0, 1};
                for (int k = 0; k < 3; ++k) {   // まわる輪
                    const Quat q = Quat::fromAxisAngle(n, sw.spin * (1.0f + 0.4f * k) + (float)k);
                    drawRing(b->position, q.rotate(Vec3{0, 1, 0}), r * (1.25f + 0.2f * k), 32, 1.0f, Fade(pc2, 0.6f));
                }
                break;
            }
            case Kind::Pad: {   // 発射台（チェックポイント）
                const Pad& pd = fw.pads[fw.padOf[b->id]];
                const bool on = fw.seatedPad == fw.padOf[b->id];
                const Color c = ColorLerp(pd.cooldown > 0.0f ? Fade(COL_PAD, 0.6f) : COL_PAD, RAYWHITE, on ? 0.35f : 0.0f);
                drawBox(md, b, Vec3{0, 0, 0}, b->shape.halfExtents, c);
                break;
            }
            case Kind::Iron: {
                const float f = fw.flash[b->id];
                const Color c = ColorLerp(fw.isSteel(b) ? COL_STEEL : COL_IRON, RAYWHITE, f);
                if (b->shape.type == ShapeType::Sphere) {
                    const float r = b->shape.radius;
                    drawOrientedModel(md.sphere, toRay(b->position), b->orientation, Vector3{r, r, r}, c);
                } else {
                    drawBox(md, b, Vec3{0, 0, 0}, b->shape.halfExtents, c);
                }
                break;
            }
            case Kind::Door: drawBox(md, b, Vec3{0, 0, 0}, b->shape.halfExtents, COL_DOOR); break;
            default:
                if (b->shape.type == ShapeType::Box) drawBox(md, b, Vec3{0, 0, 0}, b->shape.halfExtents, COL_SOLID);
                break;
            }
        }
        // 連鎖で動き出した物の輪郭
        for (const auto& bp : w.getBodies()) {
            const RigidBody* b = bp.get();
            const float f = b->id < (int)fw.flash.size() ? fw.flash[b->id] : 0.0f;
            if (f <= 0.0f) continue;
            const float r = (b->shape.type == ShapeType::Sphere ? b->shape.radius : length(b->shape.halfExtents)) + 0.3f * (1.0f - f) + 0.1f;
            DrawSphereWires(toRay(b->position), r, 8, 12, Fade(Color{255, 230, 120, 255}, f));
        }

        // ゲートの判定の箱とゴール
        for (const Gate& g : fw.gates) {
            const Vec3 c = (g.lo + g.hi) * 0.5f, s = g.hi - g.lo;
            const Color col = g.opening ? Color{120, 240, 140, 255} : Color{255, 210, 90, 255};
            if (g.rotorTrigger < 0) DrawCubeWires(toRay(c), s.x, s.y, s.z, col);   // 振り子で開く扉には判定の箱がない
            if (g.held > 0.0f && !g.opening && g.rotorTrigger < 0) DrawCube(toRay(c), s.x, s.y, s.z, Fade(col, 0.25f));
            if (g.hinged) {   // 回る扉のヒンジ
                const Vec3 a = g.pivot - g.hingeAxis * 2.2f, b = g.pivot + g.hingeAxis * 2.2f;
                DrawCylinderEx(toRay(a), toRay(b), 0.18f, 0.18f, 8, Fade(COL_DOOR, 0.9f));
            }
        }
        drawGimmicks(main);
        drawLab();
        if (fw.hasGoal) {
            const Vec3 c = (fw.goalLo + fw.goalHi) * 0.5f, s = fw.goalHi - fw.goalLo;
            DrawCubeWires(toRay(c), s.x, s.y, s.z, Color{110, 240, 150, 255});
            DrawLine3D(toRay(Vec3{c.x, fw.goalLo.y, c.z}), toRay(Vec3{c.x, fw.goalLo.y + 30.0f, c.z}), Fade(Color{110, 240, 150, 255}, 0.5f));
        }

        // 最高記録のゴースト（今回と同じ時刻の位置）と、その軌跡
        const StageRecord* best = bestRecord();
        // 軌跡を描く（breaks の添字の手前で線を切る。チェックポイントに戻ったところ）
        auto drawTrail = [&](const std::vector<Vec3>& tr, const std::vector<int>& breaks, auto colorAt) {
            size_t bk = 0;
            for (size_t i = 1; i < tr.size(); ++i) {
                while (bk < breaks.size() && (size_t)breaks[bk] < i) ++bk;
                if (bk < breaks.size() && (size_t)breaks[bk] == i) continue;
                DrawLine3D(toRay(tr[i - 1]), toRay(tr[i]), colorAt(i));
            }
        };
        if (main && showGhost && best && best->trail.size() > 1) {
            drawTrail(best->trail, best->breaks, [](size_t) { return Fade(Color{120, 240, 170, 255}, 0.22f); });
            Vec3 gp;
            if (fw.state != State::Ready && best->ghostAt(fw.stats.time, gp)) {
                DrawSphere(toRay(gp), 0.5f, Fade(Color{120, 240, 170, 255}, 0.28f));
                DrawSphereWires(toRay(gp), 0.52f, 6, 10, Fade(Color{120, 240, 170, 255}, 0.6f));
            }
        }
        // 前回の軌跡（灰色）と、切り替えた点
        drawTrail(fw.prevTrail, fw.prevBreaks, [](size_t) { return Fade(LIGHTGRAY, 0.45f); });
        for (const SwitchMark& m : fw.prevMarks) DrawSphere(toRay(m.p), 0.12f, Fade(polColor(m.polarity), 0.6f));
        // 今回の軌跡
        {
            const size_t n = fw.trail.size(), from = fw.bigMap() && n > 900 ? n - 900 : 0;   // 長いマップでは最近の 30 s
            const std::vector<Vec3> recent(fw.trail.begin() + from, fw.trail.end());
            std::vector<int> br;
            for (int i : fw.trailBreaks) if ((size_t)i > from) br.push_back(i - (int)from);
            drawTrail(recent, br, [&](size_t i) { return Fade(RAYWHITE, 0.25f + 0.75f * (float)i / recent.size()); });
        }
        for (const SwitchMark& m : fw.marks) DrawSphere(toRay(m.p), 0.12f, polColor(m.polarity));

        // 磁力線（F3）
        if (showField) {
            for (const demo::FieldLine& l : tracer.lines())
                for (size_t k = 0; k + 1 < l.pts.size(); ++k)
                    DrawLine3D(toRay(l.pts[k]), toRay(l.pts[k + 1]), fieldColor(l.mag[k]));
        }

        // プレイヤー
        const RigidBody* p = fw.player;
        const Color pc = polColor(fw.polarity);
        drawOrientedModel(md.sphere, toRay(p->position), p->orientation, Vector3{0.5f, 0.5f, 0.5f}, pc);
        DrawModelWiresEx(md.sphere, toRay(p->position), Vector3{0, 1, 0}, 0.0f, Vector3{0.52f, 0.52f, 0.52f}, Fade(WHITE, 0.25f));
        if (pulse > 0.0f) {   // 切り替えた瞬間の広がる輪
            const float r = 0.6f + 2.5f * (1.0f - pulse);
            const Vec3  n = normalize(toPhys(drawCam->position) - p->position);   // DrawCircle3D の円は xy 面（法線 +z）
            const Vec3  ax = cross(Vec3{0, 0, 1}, n);
            const float an = std::acos(clampf(n.z, -1.0f, 1.0f)) * RAD2DEG;
            const Vector3 axis = lengthSq(ax) > 1e-8f ? toRay(normalize(ax)) : Vector3{1, 0, 0};
            for (int k = 0; k < 3; ++k)
                DrawCircle3D(toRay(p->position), r + 0.05f * k, axis, an, Fade(pc, pulse));
        }

        // 力（F2）: 効いている物への線（吸引は緑、反発は橙）と合力の矢印
        if (main && showForces && fw.state != State::Cleared) {
            const float cap = fw.params.forceCap * G_ACC;
            const auto& bodies = w.getBodies();
            for (const PlayerLink& l : fw.links) {
                const float f = length(l.force);
                if (f < 0.02f * cap) continue;
                const RigidBody* b = bodies[l.body].get();
                const float s = clampf(f / cap, 0.0f, 1.0f);
                const Color col = l.attract ? Color{110, 235, 140, 255} : Color{255, 150, 60, 255};
                DrawCylinderEx(toRay(p->position), toRay(b->position), 0.015f + 0.05f * s, 0.01f, 5, Fade(col, 0.25f + 0.6f * s));
            }
            const float nf = length(fw.netForce);
            if (nf > 0.02f * cap) {
                const Vec3 tip = p->position + fw.netForce * (2.0f / cap);
                DrawCylinderEx(toRay(p->position), toRay(tip), 0.05f, 0.05f, 6, Fade(RAYWHITE, 0.8f));
                DrawCylinderEx(toRay(tip), toRay(tip + normalize(fw.netForce) * 0.35f), 0.13f, 0.0f, 8, Fade(RAYWHITE, 0.8f));
            }
            const float vf = length(fw.vortexForce);   // 軌道コアの渦の力（水色）
            if (vf > 0.05f * cap) {
                const Vec3 tip = p->position + fw.vortexForce * (2.0f / cap);
                DrawCylinderEx(toRay(p->position), toRay(tip), 0.04f, 0.04f, 6, Fade(SKYBLUE, 0.8f));
                DrawCylinderEx(toRay(tip), toRay(tip + normalize(fw.vortexForce) * 0.3f), 0.1f, 0.0f, 8, Fade(SKYBLUE, 0.8f));
            }
        }
    }

    // 入力の時刻の帯: 最高記録（緑）、前回（灰）、今回（N 赤 / S 青）。早すぎ・遅すぎを見比べる
    // 入力の時刻の帯: 最高記録（緑）、前回（灰）、今回（N 赤 / S 青）。早すぎ・遅すぎを見比べる。
    // GRAND TOUR では今の区間だけ（最後のチェックポイントに着いた時刻を 0 にする）
    void drawTimeline(int x, int y, int w) {
        const StageRecord* best = bestRecord();
        const bool big = fw.bigMap();
        const int  cp  = std::max(0, fw.checkpoint);
        const float t0 = big && cp < (int)fw.splits.size() ? fw.splits[cp] : 0.0f;
        float b0 = 0.0f, b1 = best ? best->time : 0.0f;
        if (big && best) {
            b0 = cp < (int)best->splits.size() ? best->splits[cp] : 0.0f;
            b1 = cp + 1 < (int)best->splits.size() ? best->splits[cp + 1] : best->time;
        }
        const float now = fw.stats.time - t0;
        const float prevT = !big && fw.prevTrail.size() > 1 ? (float)(fw.prevTrail.size() - 1) / 30.0f : 0.0f;
        float T = std::max({now + 0.5f, prevT, best ? b1 - b0 : 0.0f, 3.0f});
        T = std::ceil(T);
        auto X = [&](float t) { return x + (int)(w * clampf(t / T, 0.0f, 1.0f)); };
        DrawRectangle(x - 6, y - 26, w + 12, 58, Fade(BLACK, 0.45f));
        for (int k = 0; k <= (int)T; ++k) DrawLine(X((float)k), y - 20, X((float)k), y + 28, Fade(GRAY, 0.35f));
        auto row = [&](int yy, const char* label, const std::vector<float>& in, float from, float to, float end, Color c, bool pol) {
            DrawText(label, x - 4 - MeasureText(label, 12) - 4, yy - 5, 12, c);
            if (end > 0.0f) DrawLine(x, yy, X(end), yy, Fade(c, 0.6f));
            for (size_t i = 0; i < in.size(); ++i) {
                if (in[i] < from - 1e-4f || in[i] > to + 1e-4f) continue;
                const Color cc = pol ? (i == 0 ? RAYWHITE : polColor((int)(i % 2 == 1 ? -fw.startPolarity : fw.startPolarity))) : c;
                DrawRectangle(X(in[i] - from) - 1, yy - 6, 3, 13, pol && big ? RAYWHITE : cc);
            }
        };
        if (best) row(y - 14, "best", best->inputs, b0, b1, b1 - b0, Color{120, 240, 170, 255}, false);
        if (!big && !fw.prevInputs.empty()) row(y, "last", fw.prevInputs, 0.0f, 1e9f, prevT, LIGHTGRAY, false);
        row(y + 14, "now", fw.inputs, t0, 1e9f, fw.state == State::Ready ? 0.0f : now, RAYWHITE, true);
        if (fw.state != State::Ready) DrawLine(X(now), y - 20, X(now), y + 28, RAYWHITE);
        DrawText(TextFormat(big ? "zone %.0f s" : "%.0f s", T), x + w - (big ? 54 : 24), y - 24, 12, GRAY);
    }

    // 点 p が画面のどこに見えるか（カメラの後ろなら false）
    bool onScreen(const Vec3& p, Vector2& out) const {
        const Vec3 f = normalize(toPhys(cam.target) - toPhys(cam.position));
        if (dot(p - toPhys(cam.position), f) < 0.5f) return false;
        out = GetWorldToScreen(toRay(p), cam);
        return true;
    }

    // GRAND TOUR の表示: 区間の名前、チェックポイント、次の発射台の印、マップ全体の名札
    void drawGrandOverlay(int W, int H) {
        const Color gold{255, 220, 110, 255}, green{120, 240, 150, 255};
        // 区間に入った: 名前とひとこと
        if (zoneBanner > 0.0f && fw.zone >= 0 && fw.zone < (int)fw.zones.size() && !mapView) {
            const float a = std::min(1.0f, zoneBanner / 0.8f);
            const Zone& z = fw.zones[fw.zone];
            const char* t = z.name.c_str();
            DrawText(t, W / 2 - MeasureText(t, 44) / 2, H / 3 - 20, 44, Fade(RAYWHITE, a));
            DrawText(z.hint.c_str(), W / 2 - MeasureText(z.hint.c_str(), 20) / 2, H / 3 + 32, 20, Fade(LIGHTGRAY, a));
        }
        // チェックポイントに着いた: 最高記録との差
        if (cpMsg > 0.0f && fw.checkpoint > 0) {
            const float a = std::min(1.0f, cpMsg);
            const char* t = TextFormat("CHECKPOINT %d / %d   %.2f s", fw.checkpoint + 1, fw.checkpointTotal, fw.stats.time);
            DrawText(t, W / 2 - MeasureText(t, 28) / 2, H / 3 + 70, 28, Fade(green, a));
            if (hasSplit) {
                const char* d = TextFormat("%+.2f s", splitDelta);
                DrawText(d, W / 2 - MeasureText(d, 28) / 2, H / 3 + 102, 28,
                         Fade(splitDelta <= 0.0f ? green : Color{255, 140, 120, 255}, a));
            }
        }
        if (magMsg > 0.0f) {
            const char* t = "MAGNETIZED";
            DrawText(t, W / 2 - MeasureText(t, 26) / 2, H / 2 + 40, 26, Fade(gold, std::min(1.0f, magMsg)));
        }
        // 次のチェックポイントの印
        for (const Pad& pd : fw.pads) {
            if (pd.checkpoint != fw.checkpoint + 1) continue;
            Vector2 sp;
            if (!onScreen(pd.seat + Vec3{0, 2.5f, 0}, sp)) continue;
            const char* t = pd.checkpoint >= fw.checkpointTotal - 1 && !fw.hasGoal ? "GOAL" : TextFormat("CP %d", pd.checkpoint + 1);
            DrawText(t, (int)sp.x - MeasureText(t, 20) / 2, (int)sp.y - 10, 20, Fade(gold, 0.9f));
        }
        if (fw.hasGoal) {
            Vector2 sp;
            if (onScreen((fw.goalLo + fw.goalHi) * 0.5f + Vec3{0, 4.0f, 0}, sp))
                DrawText("GOAL", (int)sp.x - MeasureText("GOAL", 22) / 2, (int)sp.y - 11, 22, green);
        }
        // マップ全体: 区間の名前と、プレイヤーの位置
        if (mapView) {
            for (int i = 0; i < (int)fw.zones.size(); ++i) {
                const Zone& z = fw.zones[i];
                if (z.lo.y < -1e5f) continue;
                Vector2 sp;
                if (!onScreen((z.lo + z.hi) * 0.5f, sp)) continue;
                const Color c = i < fw.zone ? green : i == fw.zone ? gold : RAYWHITE;
                DrawText(z.name.c_str(), (int)sp.x - MeasureText(z.name.c_str(), 18) / 2, (int)sp.y - 9, 18, c);
            }
            Vector2 pp;
            if (onScreen(fw.player->position, pp)) {
                DrawCircleLines((int)pp.x, (int)pp.y, 14, polColor(fw.polarity));
                DrawCircleLines((int)pp.x, (int)pp.y, 15, RAYWHITE);
                DrawText("YOU", (int)pp.x + 18, (int)pp.y - 8, 16, RAYWHITE);
            }
            const char* t = "MAP  (M to return)";
            DrawText(t, W / 2 - MeasureText(t, 24) / 2, 16, 24, gold);
        }
    }

    void draw2D() {
        const int W = GetScreenWidth(), H = GetScreenHeight();
        if (tint > 0.0f) DrawRectangle(0, 0, W, H, Fade(polColor(fw.polarity), 0.12f * tint));

        // 大きな N / S
        const Color pc = polColor(fw.polarity);
        const int cx = W / 2, cy = H - 70;
        DrawCircle(cx, cy, 44 + 10 * pulse, Fade(pc, 0.9f));
        DrawCircleLines(cx, cy, 48 + 10 * pulse, RAYWHITE);
        const char* pol = fw.polarity > 0 ? "N" : "S";
        DrawText(pol, cx - MeasureText(pol, 56) / 2, cy - 28, 56, RAYWHITE);

        // 軌道コアを回っている: 周回数と速さ
        if (fw.orbitCore >= 0) {
            const float laps = std::max(0.0f, fw.orbitAngle) / (2.0f * PHYS_PI);
            const char* t = TextFormat("ORBIT %.1f", laps);
            const int fs = 28 + (int)(8 * lapMsg);
            DrawText(t, cx + 70, cy - 30, fs, Color{150, 220, 255, 255});
            const float sp = length(fw.player->velocity);
            DrawRectangle(cx + 70, cy + 6, 160, 10, Fade(BLACK, 0.5f));
            DrawRectangle(cx + 70, cy + 6, (int)(160 * clampf(sp / fw.params.speedCap, 0.0f, 1.0f)), 10, Color{150, 220, 255, 255});
            DrawText(TextFormat("%.1f m/s", sp), cx + 236, cy + 2, 16, Color{150, 220, 255, 255});
        }
        // 磁気振り子: 回った回数と速さ
        if (fw.swingActive >= 0 && fw.orbitCore < 0) {
            const float loops = std::fabs(fw.swingAngle) / (2.0f * PHYS_PI);
            const char* t = loops >= 1.0f ? TextFormat("SWING  loop %d", (int)loops) : "SWING";
            DrawText(t, cx + 70, cy - 30, 28 + (int)(8 * lapMsg), Color{255, 190, 120, 255});
            const float sp = length(fw.player->velocity);
            DrawRectangle(cx + 70, cy + 6, 160, 10, Fade(BLACK, 0.5f));
            DrawRectangle(cx + 70, cy + 6, (int)(160 * clampf(sp / fw.params.speedCap, 0.0f, 1.0f)), 10, Color{255, 190, 120, 255});
            DrawText(TextFormat("%.1f m/s", sp), cx + 236, cy + 2, 16, Color{255, 190, 120, 255});
        }
        // 新しいギミック: 見出しと速さ（または振れ）のメーター
        auto meter = [&](const char* title, float frac, const char* value, Color col) {
            DrawText(title, cx + 70, cy - 30, 26 + (int)(8 * boostMsg), col);
            DrawRectangle(cx + 70, cy + 6, 160, 10, Fade(BLACK, 0.5f));
            DrawRectangle(cx + 70, cy + 6, (int)(160 * clampf(frac, 0.0f, 1.0f)), 10, col);
            DrawText(value, cx + 236, cy + 2, 16, col);
        };
        const float spd = length(fw.player->velocity);
        if (fw.state == State::Running && fw.orbitCore < 0 && fw.swingActive < 0) {
            for (const Cyclotron& c : fw.cyclotrons)   // サイクロトロン: 周回数と押された回数
                if (c.inside)
                    meter(TextFormat("CYCLOTRON  turn %.1f  boost x%d", c.turns, c.boosts), spd / fw.params.speedCap,
                          TextFormat("%.1f m/s", spd), Color{255, 220, 110, 255});
            if (!fw.rings.empty()) {   // 加速リング: くぐった輪の数
                int passed = 0;
                bool lit = false;
                for (const GaussRing& g : fw.rings) {
                    passed += dot(fw.player->position - g.center, g.axis) > g.reach ? 1 : 0;
                    lit = lit || g.lit;
                }
                if (lit || (passed > 0 && passed < (int)fw.rings.size()))
                    meter(TextFormat("RING %d / %d", passed, (int)fw.rings.size()), spd / fw.params.speedCap,
                          TextFormat("%.1f m/s", spd), Color{255, 220, 110, 255});
            }
            for (const Rotor& ro : fw.rotors) {   // 共振ブランコ: 振れ（壁の角度まで）。壁が壊れたら発射
                if (!ro.spendOnHi) continue;
                const float pk = std::max(ro.peak, std::fabs(ro.angle));
                if (ro.reachedHi)
                    meter("WALL DOWN  -  launch!", 1.0f, "", Color{120, 240, 150, 255});
                else
                    meter(TextFormat("SWING %.0f / %.0f deg", pk * RAD2DEG, ro.hi * RAD2DEG), pk / ro.hi, "",
                          Color{255, 190, 120, 255});
            }
        }
        // 連続磁気イベント
        if (fw.stats.combo >= 2 && fw.comboClock <= COMBO_GAP && fw.state == State::Running) {
            const char* t = TextFormat("FLOW x%d", fw.stats.combo);
            const int tw = MeasureText(t, 30);
            DrawText(t, cx - tw / 2, cy - 104, 30, Color{255, 220, 110, 255});
            const float left = 1.0f - fw.comboClock / COMBO_GAP;
            DrawRectangle(cx - tw / 2, cy - 70, (int)(tw * left), 5, Color{255, 220, 110, 255});
        }

        if (showHud) {
            const StageRecord* best = bestRecord();
            HudLines h;
            const bool big = fw.bigMap();
            const Zone* z = big && fw.zone >= 0 && fw.zone < (int)fw.zones.size() ? &fw.zones[fw.zone] : nullptr;
            h.add(z ? TextFormat("%s  -  %s", stageName(stage), z->name.c_str()) : stageName(stage), RAYWHITE, 22);
            h.add(z ? z->hint.c_str() : stageHint(stage), LIGHTGRAY, 16);
            h.gap(6);
            h.add(TextFormat("time %.2f s   speed %4.1f m/s   switches %d", fw.stats.time, length(fw.player->velocity),
                             fw.stats.switches), RAYWHITE, 18);
            h.add(TextFormat("magnets used %d/%d   best flow %d   retry %d", fw.stats.magnetsUsed, fw.magnetTotal,
                             fw.stats.bestCombo, fw.retries), LIGHTGRAY, 16);
            if (best)
                h.add(TextFormat("best %.2f s (%d switches, flow %d)%s", best->time, best->switches, best->bestCombo,
                                 showGhost ? "  ghost on" : ""), Color{120, 240, 150, 255});
            if (big) {
                h.add(TextFormat("checkpoint %d/%d   falls %d%s", std::max(0, fw.checkpoint) + 1, fw.checkpointTotal,
                                 fw.stats.falls, fw.practice ? "   PRACTICE (not recorded)" : ""),
                      fw.practice ? Color{255, 180, 90, 255} : Color{150, 220, 255, 255}, 16);
                if (hasSplit)
                    h.add(TextFormat("split vs best %+.2f s", splitDelta),
                          splitDelta <= 0.0f ? Color{120, 240, 150, 255} : Color{255, 140, 120, 255}, 16);
            }
            if (fw.stats.chain > 0) h.add(TextFormat("chain %d", fw.stats.chain), Color{255, 220, 110, 255});
            if (!mapView) h.draw(10, 10, 380);
            if (fw.state == State::Ready || fw.state == State::Cleared) {   // 操作の説明（画面の下）
                const char* keys = big ? "SPACE/click switch / launch   R checkpoint   Shift+R restart   [ ] practice zone   M map   "
                                         "1-0 - = stage   G ghost   P replay   F1 tuning   F2 forces   F3 field   Tab free cam   H hud"
                                       : "SPACE/click switch N<->S   R retry   1-0 - = stage   G ghost   P replay best   F1 tuning   "
                                         "F2 forces   F3 field   Tab free cam   H hud";
                DrawText(keys, W / 2 - MeasureText(keys, 14) / 2, H - 18, 14, LIGHTGRAY);
            }
            if (fw.state != State::Ready || !fw.prevInputs.empty() || best) drawTimeline(60, H - 50, std::min(420, W / 2 - 160));
        }
        if (replaying && showHud) {
            const char* t = "REPLAY (best)";
            DrawText(t, W - MeasureText(t, 24) - 20, 16, 24, Color{120, 240, 170, 255});
        }
        if (fw.state == State::Ready && !replaying) {
            const char* t = "SPACE / click  to start";
            DrawText(t, W / 2 - MeasureText(t, 30) / 2, H / 2 - 60, 30, RAYWHITE);
        }
        if (chainMsg > 0.0f && fw.stats.chain > 0 && !mapView) {
            const char* t = TextFormat("CHAIN x%d", fw.stats.chain);
            DrawText(t, W / 2 - MeasureText(t, 40) / 2, H / 4 - 30, 40, Fade(Color{255, 220, 110, 255}, std::min(1.0f, chainMsg)));
        }
        if (missMsg > 0.0f) {
            const char* t = fw.bigMap() ? "MISS  -  back to the checkpoint" : "MISS";
            DrawText(t, W / 2 - MeasureText(t, 48) / 2, H / 2 - 40, 48, Fade(Color{255, 120, 100, 255}, missMsg));
        }
        if (fw.bigMap()) drawGrandOverlay(W, H);
        // 連鎖カメラ（右下の小窓）
        if (chainOk && chainShow > 0.0f && fw.state != State::Cleared) {
            const float a  = std::min(1.0f, chainShow / 0.3f);
            const int   tw = chainRT.texture.width, th = chainRT.texture.height;
            const int   px = W - tw - 16, py = H - th - 16;
            DrawTextureRec(chainRT.texture, Rectangle{0, 0, (float)tw, (float)-th}, Vector2{(float)px, (float)py}, Fade(WHITE, a));
            DrawRectangleLines(px - 1, py - 1, tw + 2, th + 2, Fade(Color{255, 220, 110, 255}, a));
            DrawText("CHAIN CAM", px + 8, py + 6, 16, Fade(Color{255, 220, 110, 255}, a));
        }
        if (fw.state == State::Cleared) {
            const StageRecord& r = lastClear;
            const int pw = 440, ph = 300, px = W / 2 - pw / 2, py = H / 2 - ph / 2 - 30;
            DrawRectangle(px, py, pw, ph, Fade(BLACK, 0.7f));
            DrawRectangleLines(px, py, pw, ph, Color{110, 240, 150, 255});
            DrawText("CLEAR", px + 20, py + 16, 40, Color{110, 240, 150, 255});
            if (newBest) DrawText("NEW BEST", px + 270, py + 28, 22, Color{255, 220, 110, 255});
            int y = py + 70;
            auto line = [&](const char* t) { DrawText(t, px + 24, y, 20, RAYWHITE); y += 26; };
            line(TextFormat("time          %.2f s", r.time));
            line(TextFormat("max speed     %.1f m/s", r.maxSpeed));
            line(TextFormat("magnet flow   %d in a row", r.bestCombo));
            line(TextFormat("magnets used  %d / %d", r.magnetsUsed, fw.magnetTotal));
            line(TextFormat("switches      %d    chain %d", r.switches, fw.stats.chain));
            line(TextFormat("retries       %d", r.retries));
            if (fw.bigMap()) DrawText(TextFormat("falls %d", r.falls), px + 290, py + 96, 20, LIGHTGRAY);
            const StageRecord* best = bestRecord();
            if (best && !newBest) DrawText(TextFormat("best %.2f s", best->time), px + 290, py + 70, 20, Color{120, 240, 150, 255});
            DrawText(fw.bigMap() ? "R  restart      P  replay best      M  map" : "R  retry      Enter  next stage      P  replay best",
                     px + 24, py + ph - 32, 18, LIGHTGRAY);
        }
        if (showTuning) {
            HudLines t;
            t.add("tuning (Up/Down select, Left/Right change)", Color{255, 220, 110, 255}, 16);
            for (size_t i = 0; i < tunables.size(); ++i)
                t.add(TextFormat("%s %-22s %8.2f", (int)i == tuneSel ? ">" : " ", tunables[i].name, *tunables[i].value),
                      (int)i == tuneSel ? RAYWHITE : LIGHTGRAY, 16);
            t.draw(W - 380, 10, 360);
        }
    }

    void tick(float frameDt) {
        const float dt = std::min(frameDt, 0.05f);
        input(dt);
        // 物理（固定刻み）。クリアしたら 0.6 s かけて時間を止める（速いままゴールを通り抜けないように）
        static float acc = 0.0f;
        clearTimer = fw.state == State::Cleared ? clearTimer + dt : 0.0f;
        const float slow = fw.state == State::Cleared ? std::max(0.0f, 1.0f - clearTimer / 0.6f) : 1.0f;
        acc = std::min(acc + dt * slow, 0.1f);
        const float FIXED = 1.0f / 120.0f;
        while (acc >= FIXED) {
            feedReplay();
            fw.step(FIXED);
            acc -= FIXED;
            for (float& f : fw.flash) f = std::max(0.0f, f - FIXED / 0.8f);
        }
        react();
        if (!fw.bigMap() && fw.state == State::Failed && fw.failTimer > 0.4f) restart();   // 失敗したらすぐやり直す
        if (showField) tracer.update(fw.world, 2.0);
        audio.orbit(fw.orbitCore >= 0 && fw.state == State::Running, length(fw.player->velocity));
        pulse    = std::max(0.0f, pulse - dt / 0.35f);
        tint     = std::max(0.0f, tint - dt / 0.2f);
        shake    = std::max(0.0f, shake - dt / 0.25f);
        chainMsg = std::max(0.0f, chainMsg - dt);
        missMsg  = std::max(0.0f, missMsg - dt / 0.6f);
        lapMsg   = std::max(0.0f, lapMsg - dt / 0.4f);
        zoneBanner = std::max(0.0f, zoneBanner - dt);
        cpMsg    = std::max(0.0f, cpMsg - dt);
        magMsg   = std::max(0.0f, magMsg - dt);
        boostMsg = std::max(0.0f, boostMsg - dt / 0.6f);
        updateCamera(dt);
        updateChainCam(dt);
    }
};

} // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "MAGNET FLOW (prototype)");
    SetTargetFPS(60);

    Game g;
    g.audio.init();
    g.md.cube   = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    g.md.sphere = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 24));
    g.md.hemi   = LoadModelFromMesh(GenMeshHemiSphere(1.0f, 12, 24));
    g.chainRT   = LoadRenderTexture(400, 225);
    g.chainOk   = IsRenderTextureValid(g.chainRT);
    g.recordPath = std::string(GetApplicationDirectory()) + "magnet_flow_records.txt";
    g.book.load(g.recordPath);
    g.cam.up         = Vector3{0, 1, 0};
    g.cam.projection = CAMERA_PERSPECTIVE;
    g.load(GRAND_STAGE);   // 全部のギミックを入れた大きなマップから
    g.updateCamera(1.0f);

    while (!WindowShouldClose()) {
        g.tick(GetFrameTime());
        g.renderChainCam();
        BeginDrawing();
        ClearBackground(COL_BG);
        BeginMode3D(g.cam);
            g.draw3D();
        EndMode3D();
        g.draw2D();
        EndDrawing();
    }

    if (g.chainOk) UnloadRenderTexture(g.chainRT);
    UnloadModel(g.md.cube);
    UnloadModel(g.md.sphere);
    UnloadModel(g.md.hemi);
    g.audio.shutdown();
    CloseWindow();
    return 0;
}
