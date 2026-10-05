// ---------------------------------------------------------------------------
// coilgun_scene.cpp : コイル砲台のシューティング（coilgun_scene.h）
//
//   砲台（+x 向き、左右に振れる）のコイル 3 段で磁石のパックを撃ち、的（鉄の棒・磁石の棒）を倒す。
//   奥の敵の砲台も同じ装置でパックを撃ってくるので、こちらのコイルで受け止めて弾にする。
//   調整の数字は CURRENT_SPEC.md §4 F12（スクラッチの exp28 で振った）。
// ---------------------------------------------------------------------------
#include "coilgun_scene.h"

#include "demo_common.h"
#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace phys;
using namespace demo;

namespace {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }

const Color COL_TITLE{255, 200, 120, 255};
const Color COL_ARROW{255, 225, 90, 255};
const Color COL_COPPER{200, 125, 80, 255};
const Color COL_WALL{150, 158, 172, 150};
const Color COL_PIN_DOWN{70, 70, 75, 255};
const Color COL_ENEMY{230, 90, 90, 255};

// --- 砲台 --------------------------------------------------------------------
constexpr float COIL_A = 0.6f, I_GUN = 0.21f;       // 中心の場 2 pi I n / a = 22（a = 0.45, I = 0.16 と同じ。広い方が切る窓が長い）
constexpr int   COIL_SEG = 24, COIL_TURNS = 10;
constexpr float COIL_LX[3] = {1.0f, 3.0f, 5.0f};    // 支点（装填口）からの距離
constexpr float BARREL_END = 6.5f, STOP_LX = -0.9f;
constexpr float R_BALL = MAG_RADIUS, BULLET_S = 0.1f, PUSH = 1.0f;
constexpr float YAW_MAX = 35.0f * PHYS_PI / 180.0f, YAW_RATE = 40.0f * PHYS_PI / 180.0f;
// --- 進行 --------------------------------------------------------------------
constexpr float ROUND_TIME = 90.0f;
constexpr int   LIVES = 3, PLAYER_AMMO = 8, ENEMY_AMMO = 6;
constexpr int   PTS_IRON = 100, PTS_MAGNET = 150, PTS_CATCH = 150;
constexpr float ENEMY_FIRST = 6.0f, ENEMY_PERIOD = 7.0f;
constexpr float REST_SPEED = 0.3f, REST_TIME = 2.0f, CAPTURE_DIST = 0.7f, CATCH_SPEED = 0.8f;
constexpr float HALF_X = 15.0f;   // これより奥で止まったパックは敵の弾倉へ
// --- 的 ----------------------------------------------------------------------
const Vec3  PIN_HE{0.15f, 0.5f, 0.15f};
constexpr float MAGPIN_SCALE = 0.1f;   // 磁石の棒のモーメント（磁石球の磁化 x これ）。遅いパックはくっつき、速ければ倒れる
constexpr float COPPER_SIGMA = 0.08f, COPPER_SPACING = 0.3f;
// 的はこちらのレーン（z = 0）から 2.5 m 以上、敵の弾の道（(28, 6) → (0, 0)、z = 0.21 x）から 2 m 以上離す
const Vec3 IRON_PINS[6]   = {{9, 0, -3}, {11, 0, -5.5f}, {13, 0, -2.5f}, {16, 0, 5.8f}, {18, 0, -4.5f}, {22, 0, -2.5f}};
const Vec3 MAGNET_PINS[3] = {{14, 0, -6.5f}, {20, 0, -2.5f}, {21, 0, 6.5f}};
const Vec3 COPPER_MATS[2] = {{9, 0, -5.5f}, {15.5f, 0, -3.4f}};   // 的の手前（砲台から見た線の上）

struct Pin { RigidBody* body; bool magnet; Vec3 home; bool down = false; };

// 砲台。支点 pivot のまわりに yaw で回る（コイル・壁・止めは静的で、毎ステップ置き直す）
struct Turret {
    Vec3  pivot;
    float yawBase = 0.0f, yaw = 0.0f;   // yawBase = pi で敵（-x 向き）
    int   coil[3] = {-1, -1, -1};
    RigidBody* walls[2] = {nullptr, nullptr};
    RigidBody* stop = nullptr;
    Vec3  wallHome[2], stopHome;        // yaw 0 のときの位置
    std::vector<RigidBody*> magazine;
    Vec3  magazineBase, magazineStep;
    RigidBody* loaded = nullptr;        // 装填口に pin された弾
    RigidBody* shot   = nullptr;        // 発射して砲身の中にある弾
    int   stage = 0;                    // 0: 待ち、1..3: その段のコイルが ON、4: 全部 OFF（弾が出るのを待つ）
    int   plannedStages = 3;            // 敵: 使う段の数
    float switchErr[3] = {0, 0, 0};     // 段を切ったときの、中心からのずれ（+ は遅れ）
    float stageSpeed[3] = {0, 0, 0};    // 段の後ろ（中心 + 0.9 m）での速さ
    bool  stagePassed[3] = {false, false, false};
    bool  anyShot = false;
    // 受け止め
    RigidBody* incoming = nullptr;
    bool  catchOn[3] = {false, false, false};
    float catchErr[3] = {0, 0, 0};
    int   catchUsed = 0;

    Quat rot() const { return Quat::fromAxisAngle({0, 1, 0}, yawBase + yaw); }
    Vec3 fwd() const { return rot().rotate(Vec3{1, 0, 0}); }
    Vec3 toWorld(const Vec3& local) const { return pivot + rot().rotate(local); }
    Vec3 toLocal(const Vec3& p) const {
        const Vec3 d = p - pivot, f = fwd(), r = rot().rotate(Vec3{0, 0, 1});
        return Vec3{dot(d, f), d.y, dot(d, r)};
    }
    Vec3 coilCenter(int k) const { return toWorld({COIL_LX[k], 0, 0}); }

    void build(World& w, SceneHost& host, const Vec3& p, float base, bool enemy) {
        pivot = p;
        yawBase = base;
        yaw = 0.0f;
        const Quat axisQ = Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);   // 局所 +y → +x
        for (int k = 0; k < 3; ++k)
            coil[k] = w.currents().addCoil(pivot + Vec3{COIL_LX[k], 0, 0}, axisQ, COIL_A, COIL_SEG, 0.0f, COIL_TURNS, nullptr, 0.035f);
        const float wx = 0.5f * (STOP_LX + BARREL_END - 0.3f), wh = 0.5f * (BARREL_END - 0.3f - STOP_LX);
        const Color wc = enemy ? Color{COL_ENEMY.r, COL_ENEMY.g, COL_ENEMY.b, 150} : COL_WALL;
        for (int s = 0; s < 2; ++s) {
            wallHome[s] = pivot + Vec3{wx, 0.0f, (s == 0 ? 1.0f : -1.0f) * (R_BALL + 0.08f)};
            walls[s] = host.addStatic(wallHome[s], {wh, R_BALL + 0.05f, 0.04f}, wc, 0.0f, 0.1f);
        }
        stopHome = pivot + Vec3{STOP_LX, 0.0f, 0};
        stop     = host.addStatic(stopHome, {0.05f, R_BALL + 0.05f, R_BALL + 0.12f}, wc, 0.0f, 0.1f);
        place(w);
    }

    void place(World& w) {
        const Quat R = rot();
        for (int k = 0; k < 3; ++k) {
            Wire& wire = w.currents().wires()[coil[k]];
            wire.orientation = R;
            wire.origin      = pivot - R.rotate(pivot);   // 支点のまわりの回転（静的なコイルは世界座標を pointsLocal に持つ）
        }
        for (int s = 0; s < 2; ++s) { walls[s]->position = pivot + R.rotate(wallHome[s] - pivot); walls[s]->orientation = R; }
        stop->position    = pivot + R.rotate(stopHome - pivot);
        stop->orientation = R;
        if (loaded) {   // 装填口が弾を保持する（位置と向き。隣の弾倉の場で向きが変わらないように）
            loaded->position        = pivot;
            loaded->orientation     = R * Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);
            loaded->angularVelocity = Vec3{0, 0, 0};
        }
    }

    void addToMagazine(World& w, RigidBody* b) {
        magazine.push_back(b);
        b->velocity = Vec3{0, 0, 0};
        b->angularVelocity = Vec3{0, 0, 0};
        b->position = magazineBase + magazineStep * (float)(magazine.size() - 1);
        w.setPinned(b, true);
    }
    void loadNext(World& w) {
        if (loaded || shot || magazine.empty()) return;
        loaded = magazine.back();
        magazine.pop_back();
        loaded->position = pivot;
        loaded->orientation = rot() * Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);   // 双極子（ローカル +y）を砲身の向きに。弾倉で隣と揃って別を向いていると、コイルの中で向き直るあいだ力が抜ける
        loaded->angularVelocity = Vec3{0, 0, 0};
        w.setPinned(loaded, true);
    }
    void fire(World& w) {
        if (!loaded) return;
        shot   = loaded;
        loaded = nullptr;
        w.setPinned(shot, false);
        shot->velocity = fwd() * PUSH;
        stage   = 1;
        anyShot = true;
        for (int k = 0; k < 3; ++k) { switchErr[k] = 0; stageSpeed[k] = 0; stagePassed[k] = false; }
    }
    void nextStage(float lx) {   // 手で切る
        if (stage < 1 || stage > 3) return;
        switchErr[stage - 1] = lx - COIL_LX[stage - 1];
        ++stage;
    }
    void resetCatch() {
        incoming = nullptr;
        for (int k = 0; k < 3; ++k) { catchOn[k] = false; catchErr[k] = 0; }
        catchUsed = 0;
    }
};

class CoilGunScene : public FunScene {
public:
    CoilGunScene() { timeScale = 1.0f; }
    const char* title() const override { return "F12: Coil-gun battery (timing is the firepower; the same coils catch incoming pucks)"; }
    bool ownsSpace() const override { return true; }
    bool compactHud() const override { return true; }
    float fieldLineFlux() const override { return 1.0f; }

    void build(SceneHost& host) override {
        host.reset();
        host.addGround();
        World& w = host.world;
        w.substeps = 4;
        pins.clear();
        balls.clear();
        graveyard.clear();
        trails.clear();
        repaint.clear();
        player = Turret{};
        enemy  = Turret{};
        score = 0; lives = LIVES; time = 0.0f; over = false; captured = 0; coreHits = 0; catches = 0;
        enemyTimer = ENEMY_FIRST;
        enemyShotIndex = 0;

        // 的
        for (const Vec3& p : IRON_PINS) {
            RigidBody* b = makeIronBox(w, {p.x, PIN_HE.y, p.z}, PIN_HE);
            pins.push_back({b, false, b->position});
        }
        const float V = 8.0f * PIN_HE.x * PIN_HE.y * PIN_HE.z;
        for (const Vec3& p : MAGNET_PINS) {
            RigidBody* b = w.createMagnetBox({p.x, PIN_HE.y, p.z}, PIN_HE, MAG_MASS * V / SPHERE_VOL,
                                             {0, MAG_M0 * V / SPHERE_VOL * MAGPIN_SCALE * magnetMomentScale(), 0});
            b->restitution = 0.2f;
            b->friction    = 0.5f;
            pins.push_back({b, true, b->position});
        }
        for (const Vec3& p : COPPER_MATS) {
            RigidBody* plate = w.createConductorBox({p.x, -0.01f, p.z}, {1.5f, 0.02f, 1.5f}, 0.0f, COPPER_SIGMA, Quat{}, COPPER_SPACING);
            host.paint(plate, COL_COPPER);
        }
        // 砲台と弾倉。敵は横にずらして置き、こちらの装填口を狙う（受け止めるには砲台を敵に向ける）
        const Vec3 P{0, R_BALL, 0}, E{28.0f, R_BALL, 6.0f};
        const Vec3 f = normalize(P - E);
        player.build(w, host, P, 0.0f, false);
        player.magazineBase = Vec3{-1.0f, R_BALL, -3.0f};
        player.magazineStep = Vec3{-0.6f, 0, 0};
        enemy.build(w, host, E, std::atan2(-f.z, f.x), true);
        enemy.magazineBase = Vec3{30.0f, R_BALL, 9.0f};
        enemy.magazineStep = Vec3{0.6f, 0, 0};
        hostile.assign(PLAYER_AMMO + ENEMY_AMMO, 0);
        for (int i = 0; i < PLAYER_AMMO + ENEMY_AMMO; ++i) {
            RigidBody* b = makePuck(w, {0, R_BALL, 0});
            balls.push_back(b);
            if (i < PLAYER_AMMO) player.addToMagazine(w, b);
            else                 enemy.addToMagazine(w, b);
        }
        restTime.assign(balls.size(), 0.0f);
        player.loadNext(w);
        enemy.loadNext(w);
        // 段ごとの理論（押し出し 1 m/s + 各段の dU。滑るので回転は入れない）
        computeTheory(w);
        w.magnets().updateMoments(30);
        camTargetX = 11.0f; camTargetY = 0.0f; camTargetZ = 0.0f;
        camDistance = 26.0f; camPitch = 0.44f; camYaw = -0.5f * PHYS_PI;   // 砲台の後ろ上から +x を見る
        wantCameraReset = true;
    }

    void beforeStep(World& w, float dt) override {
        if (!over) {
            time += dt;
            if (time >= ROUND_TIME) { over = true; time = ROUND_TIME; }
        }
        player.place(w);
        enemy.place(w);
        if (!over) runEnemy(w, dt);
        updateShot(w, player, autoSwitch);
        updateShot(w, enemy, true);
        updateCatch(w);
        checkHits(w);
        // コイルの電流（撃つ段と受け止めの段の OR）
        for (int k = 0; k < 3; ++k) {
            const bool fireOn = player.shot && player.stage == k + 1;
            w.currents().wires()[player.coil[k]].current = (fireOn || player.catchOn[k]) ? I_GUN : 0.0f;
            const bool eFire = enemy.shot && enemy.stage == k + 1 && k < enemy.plannedStages;
            w.currents().wires()[enemy.coil[k]].current = eFire ? I_GUN : 0.0f;
        }
        recycle(w, dt);
        checkPins();
        player.loadNext(w);
        enemy.loadNext(w);
        // 軌跡
        for (size_t i = 0; i < balls.size(); ++i) {
            if (w.isPinned(balls[i])) { trails[balls[i]].clear(); continue; }
            auto& tr = trails[balls[i]];
            tr.push_back(balls[i]->position);
            if (tr.size() > 50) tr.erase(tr.begin());
        }
    }

    void handleInput(SceneHost& host) override {
        World& w = host.world;
        for (RigidBody* b : repaint) host.paint(b, COL_PIN_DOWN);
        repaint.clear();
        if (IsKeyPressed(KEY_ENTER)) { build(host); return; }
        if (IsKeyPressed(KEY_V)) autoSwitch = !autoSwitch;
        if (over) return;
        const float dt = GetFrameTime();
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  player.yaw = std::min(YAW_MAX, player.yaw + YAW_RATE * dt);
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) player.yaw = std::max(-YAW_MAX, player.yaw - YAW_RATE * dt);
        if (IsKeyPressed(KEY_SPACE)) {
            if (!player.shot) player.fire(w);
            else if (!autoSwitch) player.nextStage(player.toLocal(player.shot->position).x);
        }
        if (IsKeyPressed(KEY_C) && player.incoming && !autoSwitch && player.catchUsed < 3) {
            // いちばん近い、まだ使っていないコイルを ON（中心の前に押すと先に加速してしまう）
            const float lx = player.toLocal(player.incoming->position).x;
            int best = -1;
            for (int k = 0; k < 3; ++k)
                if (!player.catchOn[k] && (best < 0 || std::fabs(lx - COIL_LX[k]) < std::fabs(lx - COIL_LX[best]))) best = k;
            if (best >= 0) { player.catchOn[best] = true; player.catchErr[best] = COIL_LX[best] - lx; ++player.catchUsed; }
        }
    }

    void draw3D(const World& w) const override {
        // コイルの中心の印（切る目印）と、狙いの線
        for (const Turret* t : {&player, &enemy})
            for (int k = 0; k < 3; ++k) {
                const Vec3 c = t->coilCenter(k);
                DrawLine3D(toRay(c + Vec3{0, COIL_A + 0.05f, 0}), toRay(c + Vec3{0, COIL_A + 0.35f, 0}), Fade(COL_ARROW, 0.8f));
            }
        const Vec3 muzzle = player.toWorld({BARREL_END, 0, 0});
        DrawLine3D(toRay(muzzle), toRay(muzzle + player.fwd() * 30.0f), Fade(RAYWHITE, 0.25f));
        for (const auto& kv : trails)
            for (size_t i = 1; i < kv.second.size(); ++i)
                DrawLine3D(toRay(kv.second[i - 1]), toRay(kv.second[i]), Fade(COL_ARROW, 0.1f + 0.7f * (float)i / kv.second.size()));
        // 受け止める相手の上に印
        if (player.incoming) {
            const Vec3 p = player.incoming->position;
            DrawCylinderEx(toRay(p + Vec3{0, 1.1f, 0}), toRay(p + Vec3{0, 0.45f, 0}), 0.07f, 0.0f, 6, COL_ENEMY);
        }
        (void)w;
    }

    void draw2D(const World&, const Camera3D&) const override {
        // タイミングのバー: 砲身の中の弾（撃つ弾か、向かってくる弾）の位置と、コイルの中心
        RigidBody* b = player.shot ? player.shot : player.incoming;
        if (!b) return;
        const int W = GetScreenWidth(), H = GetScreenHeight();
        const int bw = 560, bh = 14, x0 = (W - bw) / 2, y0 = H - 70;
        auto px = [&](float lx) { return x0 + (int)((lx + 1.0f) / (BARREL_END + 1.0f) * bw); };
        DrawRectangle(x0 - 10, y0 - 26, bw + 20, 54, Fade(BLACK, 0.5f));
        DrawRectangle(x0, y0, bw, bh, Fade(GRAY, 0.5f));
        for (int k = 0; k < 3; ++k) {
            const bool on = player.shot ? player.stage == k + 1 : player.catchOn[k];
            DrawRectangle(px(COIL_LX[k] - COIL_A), y0, px(COIL_LX[k] + COIL_A) - px(COIL_LX[k] - COIL_A), bh, Fade(on ? COL_COPPER : DARKGRAY, 0.6f));
            DrawRectangle(px(COIL_LX[k]) - 1, y0 - 6, 3, bh + 12, on ? COL_ARROW : LIGHTGRAY);
        }
        const float lx = player.toLocal(b->position).x;
        DrawCircle(px(lx), y0 + bh / 2, 7, player.shot ? RAYWHITE : COL_ENEMY);
        DrawText(player.shot ? "Space: cut the coil as the puck passes its center" : "C: switch the coil on as the puck passes its center",
                 x0, y0 - 22, 14, LIGHTGRAY);
    }

    void hud(const World& w, HudLines& out) const override {
        char buf[260];
        std::snprintf(buf, sizeof buf, "ammo %d   score %d   lives %d   time %.0f s%s", (int)player.magazine.size() + (player.loaded ? 1 : 0),
                      score, lives, ROUND_TIME - time, over ? "   ROUND OVER (Enter to play again)" : "");
        out.add(buf, COL_TITLE);
        std::snprintf(buf, sizeof buf, "fire: perfect timing gives %.1f / %.1f / %.1f m/s after each coil   (cut 0.03 s late: 20%% less; 0.06 s: 40%% less; 0.1 s: half)",
                      theoryV[0], theoryV[1], theoryV[2]);
        out.add(buf);
        if (player.anyShot) {
            std::string s = "last shot:";
            for (int k = 0; k < 3; ++k) {
                if (player.stagePassed[k]) std::snprintf(buf, sizeof buf, "   coil %d: %s %.2f m -> %.1f m/s", k + 1, player.switchErr[k] >= 0 ? "late" : "early",
                                                         std::fabs(player.switchErr[k]), player.stageSpeed[k]);
                else std::snprintf(buf, sizeof buf, "   coil %d: -", k + 1);
                s += buf;
            }
            out.add(s.c_str(), LIGHTGRAY, 14);
        }
        if (player.incoming) {
            std::snprintf(buf, sizeof buf, "incoming puck %.1f m/s   catch: switch a coil ON as it passes the center; one stage removes up to %.1f m/s   (%d used)",
                          length(player.incoming->velocity), catchV, player.catchUsed);
            out.add(buf, COL_ENEMY);
        } else if (!over) {
            std::snprintf(buf, sizeof buf, "enemy battery (right) fires at you in %.1f s with %d stage%s (ammo %d): turn toward it to catch   pucks that stop in its half become its ammo", enemyTimer,
                          nextEnemyStages(), nextEnemyStages() == 1 ? "" : "s", (int)enemy.magazine.size() + (enemy.loaded ? 1 : 0));
            out.add(buf, LIGHTGRAY, 14);
        }
        int down = 0;
        for (const Pin& p : pins) down += p.down ? 1 : 0;
        std::snprintf(buf, sizeof buf, "pins down %d / %d (iron %d pts, needs > 5 m/s; magnet %d pts, > 3 m/s but a slow puck sticks to it)   captured %d   caught %d   hits on the battery %d",
                      down, (int)pins.size(), PTS_IRON, PTS_MAGNET, captured, catches, coreHits);
        out.add(buf, LIGHTGRAY, 14);
        out.gap(4);
        std::snprintf(buf, sizeof buf, "A/D aim   Space fire / cut the next coil   C catch stage   V auto timing (%s)   Enter restart   O field lines",
                      autoSwitch ? "ON: the machine times it" : "off");
        out.add(buf, GRAY);
        (void)w;
    }

private:
    Turret player, enemy;
    std::vector<Pin> pins;
    std::vector<RigidBody*> balls, graveyard;
    std::vector<float> restTime;
    std::vector<RigidBody*> repaint;
    std::map<RigidBody*, std::vector<Vec3>> trails;
    std::vector<char> hostile;   // 敵が撃った弾（砲台に当たると被弾）
    bool  autoSwitch = false, over = false;
    int   score = 0, lives = LIVES, captured = 0, coreHits = 0, catches = 0, enemyShotIndex = 0;
    float time = 0.0f, enemyTimer = ENEMY_FIRST;
    float theoryV[3] = {0, 0, 0}, catchV = 0.0f;

    static RigidBody* makePuck(World& w, const Vec3& p) {
        RigidBody* b = makeMagnetBall(w, p, {1, 0, 0}, false, BULLET_S);
        b->linearDamping   = 0.0f;
        b->friction        = 0.0f;   // 滑るパック（転がると双極子が回り、引きと反発が交互になる）
        b->rollingFriction = 0.0f;
        b->angularDamping  = 1.0f;   // 双極子が場に揃って落ち着く
        return b;
    }

    int nextEnemyStages() const { static const int pattern[4] = {1, 2, 1, 3}; return pattern[enemyShotIndex % 4]; }

    void computeTheory(World& w) {
        RigidBody* b = player.loaded;
        if (!b) return;
        const Vec3 keep = b->position;
        w.setPinned(b, false);   // pin 中（静的扱い）は点の位置が更新されない
        float v2 = PUSH * PUSH;
        for (int k = 0; k < 3; ++k) {
            w.currents().wires()[player.coil[k]].current = I_GUN;
            w.currents().prepare();
            w.magnets().extraField = &w.currents();
            auto U = [&](float lx) { b->position = player.toWorld({lx, 0, 0}); w.magnets().updateMoments(30); return w.magnets().potentialEnergy(); };
            const float dU = U(k == 0 ? 0.0f : COIL_LX[k - 1]) - U(COIL_LX[k]);
            w.currents().wires()[player.coil[k]].current = 0.0f;
            if (k == 0) catchV = std::sqrt(std::max(0.0f, 2.0f * dU / MAG_MASS));   // pin 中は invMass が 0 なので質量を直に使う
            v2 += 2.0f * dU / MAG_MASS;
            theoryV[k] = std::sqrt(std::max(0.0f, v2));
        }
        b->position = keep;
        w.setPinned(b, true);
    }

    // 撃った弾の段（autoMode なら半ステップ先の位置で中心を過ぎた瞬間に切る）
    void updateShot(World& w, Turret& t, bool autoMode) {
        if (!t.shot) return;
        if (w.isPinned(t.shot) || !inBarrel(t, t.shot)) { t.shot = nullptr; t.stage = 0; return; }   // 回収された／砲身を出た
        const Vec3  lp = t.toLocal(t.shot->position);
        const float lv = dot(t.shot->velocity, t.fwd());
        for (int k = 0; k < 3; ++k)
            if (!t.stagePassed[k] && lp.x > COIL_LX[k] + 0.9f) { t.stagePassed[k] = true; t.stageSpeed[k] = lv; }
        if (autoMode && t.stage >= 1 && t.stage <= 3) {
            const int   k  = t.stage - 1;
            const float xp = lp.x + 0.5f * lv * (1.0f / 120.0f);
            if (xp >= COIL_LX[k] || k >= t.plannedStages) { t.switchErr[k] = lp.x - COIL_LX[k]; ++t.stage; }
        }
    }

    bool inBarrel(const Turret& t, const RigidBody* b) const {
        const Vec3 lp = t.toLocal(b->position);
        return lp.x > STOP_LX - 0.2f && lp.x < BARREL_END && std::fabs(lp.z) < 0.6f && lp.y < 1.0f;
    }

    // 向かってくる弾（砲身の中で、こちらへ動いているもの）を受け止める
    void updateCatch(World& w) {
        Turret& t = player;
        if (t.incoming && (w.isPinned(t.incoming) || !inBarrel(t, t.incoming))) t.resetCatch();
        if (!t.incoming) {
            for (RigidBody* b : balls) {
                if (w.isPinned(b) || b == t.shot || b == t.loaded || !inBarrel(t, b)) continue;
                if (dot(b->velocity, t.fwd()) < -0.5f) { t.incoming = b; break; }
            }
            if (!t.incoming) return;
        }
        const Vec3  lp = t.toLocal(t.incoming->position);
        const float lv = dot(t.incoming->velocity, t.fwd());
        if (autoSwitch) {   // 中心を過ぎた瞬間に ON（撃つときの鏡像）
            const float xp = lp.x + 0.5f * lv * (1.0f / 120.0f);
            for (int k = 0; k < 3; ++k)
                if (!t.catchOn[k] && xp <= COIL_LX[k] && lp.x > COIL_LX[k] - 0.5f) { t.catchOn[k] = true; t.catchErr[k] = COIL_LX[k] - lp.x; ++t.catchUsed; }
        }
        if (length(t.incoming->velocity) < CATCH_SPEED) {   // 止まった: 弾になる
            score += PTS_CATCH;
            ++catches;
            hostile[indexOf(t.incoming)] = 0;
            t.addToMagazine(w, t.incoming);
            t.resetCatch();
        }
    }

    // 敵の弾が砲台（支点から 1 m）に届いたら被弾。受け止めそこねて抜けた弾も、向けていなくて横から当たった弾も同じ
    void checkHits(World& w) {
        for (size_t i = 0; i < balls.size(); ++i) {
            RigidBody* b = balls[i];
            if (!hostile[i] || w.isPinned(b)) continue;
            if (length(b->position - player.pivot) < 1.0f) {
                ++coreHits;
                if (--lives <= 0) { lives = 0; over = true; }
                if (player.incoming == b) player.resetCatch();
                player.addToMagazine(w, b);
                hostile[i] = 0;
            }
        }
    }

    int indexOf(const RigidBody* b) const {
        for (size_t i = 0; i < balls.size(); ++i) if (balls[i] == b) return (int)i;
        return 0;
    }

    void runEnemy(World& w, float dt) {
        enemyTimer -= dt;
        if (enemyTimer > 0.0f) return;
        if (!enemy.loaded || enemy.shot) { enemyTimer = 1.0f; return; }   // まだ前の弾が砲身にある／弾がない
        enemy.plannedStages = nextEnemyStages();
        ++enemyShotIndex;
        enemy.fire(w);
        hostile[indexOf(enemy.shot)] = 1;
        enemyTimer = ENEMY_PERIOD;
    }

    // 止まった弾・場の外の弾を弾倉へ。立っている磁石の棒のそばで止まった弾は捕まった（失う）
    void recycle(World& w, float dt) {
        for (size_t i = 0; i < balls.size(); ++i) {
            RigidBody* b = balls[i];
            if (w.isPinned(b)) { restTime[i] = 0.0f; continue; }
            const Vec3& p = b->position;
            const bool outside = p.x < -6.0f || p.x > 36.0f || std::fabs(p.z) > 12.0f || p.y < -2.0f;
            restTime[i] = length(b->velocity) < REST_SPEED ? restTime[i] + dt : 0.0f;
            if (!outside && restTime[i] < REST_TIME) continue;
            restTime[i] = 0.0f;
            hostile[i] = 0;
            bool stuck = false;
            for (const Pin& pin : pins)
                if (pin.magnet && !pin.down && length(pin.body->position - p) < CAPTURE_DIST + PIN_HE.y) stuck = true;
            if (stuck) {
                ++captured;
                graveyard.push_back(b);
                b->velocity = Vec3{0, 0, 0};
                b->position = Vec3{-20.0f, R_BALL, -10.0f + 0.6f * (float)graveyard.size()};
                w.setPinned(b, true);
            } else if (p.x < HALF_X) {
                player.addToMagazine(w, b);
            } else {
                enemy.addToMagazine(w, b);
            }
        }
    }

    void checkPins() {
        for (Pin& pin : pins) {
            if (pin.down) continue;
            const Vec3 up = pin.body->orientation.rotate(Vec3{0, 1, 0});
            if (up.y < 0.7f || length(pin.body->position - pin.home) > 0.5f) {
                pin.down = true;
                score += pin.magnet ? PTS_MAGNET : PTS_IRON;
                repaint.push_back(pin.body);
            }
        }
    }
};

} // namespace

std::unique_ptr<FunScene> makeCoilGunScene() { return std::make_unique<CoilGunScene>(); }
