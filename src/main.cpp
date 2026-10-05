// ---------------------------------------------------------------------------
// main.cpp : raylib による 3D 物理エンジンのデモ
//
//   操作:
//     左ドラッグ / 右ドラッグ : カメラ回転
//     マウスホイール          : ズーム
//     Tab                     : FPS 視点の切り替え（地面を歩く。マウスで見回し、WASD で歩く、
//                               Space でジャンプ、Ctrl でしゃがむ、Shift で走る、左クリックで発射、Esc で戻る）
//     Space                   : 視点方向に球を発射（シーン 5 以降では磁石球）
//     1 - 8, 9, 0, -, =, BkSp : シーン切り替え（9, 0, -, =, Backspace は fun_scenes.cpp の A〜E）
//     F5 - F8                 : 電気のシーン（electric_scenes.cpp。電気ベル、荷電粒子、渦電流、電気力線）
//     F9 - F11                : 電流のシーン（current_scenes.cpp。エルステッドと輪の塔、電磁石とコイルガン、ヘルムホルツ）
//     F12                     : コイル砲台のシューティング（coilgun_scene.cpp。段のタイミングが火力、同じコイルで受け止める）
//     R                       : シーンをリセット
//     P                       : 一時停止 / 再開
//     N                       : 1 ステップだけ進める（一時停止中）
//     F                       : 磁石のシーンの速さ  実時間 x1 ↔ x0.1（スロー）
//     C                       : 接触点の表示切り替え
//     W                       : ワイヤーフレーム表示切り替え（FPS 視点では移動）
//     G                       : ジャイロ効果の ON/OFF
//     Z                       : 重力の ON/OFF
//     T                       : 砂鉄を叩く（シーン 7）
//     M                       : 配置の切り替え（シーン 7、9、0）
//     E                       : 一様な外部磁場  なし → 水平 → 垂直
//     L                       : 磁石の材質  ネオジム → フェライト → ゴム磁石（磁力の強さ）
//     O                       : 磁力線の表示（磁石のシーン。シーン E では最初から表示）
//     H                       : HUD の表示切り替え
//     Esc                     : FPS 視点を抜ける / 終了
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "phys/world.h"

#include "demo_common.h"
#include "field_lines.h"
#include "coilgun_scene.h"
#include "current_scenes.h"
#include "electric_scenes.h"
#include "fun_scenes.h"
#include "walker.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace phys;
using namespace demo;

// ---------------------------------------------------------------------------
// 変換ヘルパ
// ---------------------------------------------------------------------------
static inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }
static inline Vec3    toPhys(const Vector3& v) { return Vec3{v.x, v.y, v.z}; }

// 磁石の設定は demo_common.h（MAGNET_SPEC.md §3.3）
static const int GRAIN_COUNT = 200;   // シーン 7 の砂鉄の粒の数

// 付録 A.1: 鎖と輪の磁気エネルギー（単位 K m^2 / d^3、添字 N）
static const float CHAIN_U[9] = {0, 0, -2.0000f, -4.2500f, -6.5741f, -8.9294f, -11.3007f, -13.6813f, -16.0677f};
static const float RING_U[9]  = {0, 0, 0, -3.7500f, -6.7071f, -9.5656f, -12.3184f, -14.9913f, -17.6066f};

static const char* structureName(MagnetSystem::StructureType t) {
    switch (t) {
    case MagnetSystem::StructureType::Single: return "single";
    case MagnetSystem::StructureType::Chain:  return "chain";
    case MagnetSystem::StructureType::Ring:   return "ring";
    default:                                  return "other";
    }
}

// ---------------------------------------------------------------------------
// デモ本体
// ---------------------------------------------------------------------------
struct Demo {
    World world;
    std::vector<Color> colors;   // body->id をそのまま添字に使う

    Model cubeModel{};
    Model sphereModel{};
    Model hemiModel{};           // 磁石の N 極・S 極（半球 2 つで 1 球）

    int   sceneIndex   = 0;
    float camTargetX   = 0.0f;   // シーンごとの推奨カメラ設定
    float camTargetY   = 3.0f;
    float camTargetZ   = 0.0f;
    float camDistance  = 24.0f;
    float camPitchPref = 0.28f;
    float camYawPref   = 0.9f;
    float timeScale    = 1.0f;   // 1 実秒あたりに進めるシミュレーション時間（磁石のシーンは 10 で実時間）
    bool  paused       = false;
    bool  showContacts = false;
    bool  wireframe    = false;
    bool  showHud      = true;
    double stepMs      = 0.0;

    // --- 磁石のシーン ---
    int      sandLayout = 0;    // シーン 7 の配置（M キー）
    int      fieldMode  = 0;    // 外部磁場（E キー） 0: なし 1: 水平 (+x) 2: 垂直 (+y)
    float    fieldStrength = 5.0f;   // シーンごとの外部磁場の強さ
    unsigned seed = 1;

    struct ArcGroup { int N; std::vector<RigidBody*> members; };   // シーン 6 の各群
    std::vector<ArcGroup> arcGroups;

    std::unique_ptr<FunScene> fun;   // シーン 9, 0, -, =, Backspace（fun_scenes.cpp）と F5〜F8（electric_scenes.cpp）

    // 磁力線（O キー）。1 本あたりの磁束は、磁石 1 個から LINES_PER_MAGNET 本出る値にする
    FieldLineTracer tracer;
    bool            fieldLinesOn = false;
    static constexpr float LINES_PER_MAGNET = 24.0f;

    SceneHost host() { return SceneHost{world, colors}; }

    float rnd() {                        // 決定論的な乱数 [0, 1)
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) & 0xFFFFFF) / 16777216.0f;
    }
    Vec3 rndDir() {
        for (;;) {
            Vec3 v{rnd() * 2 - 1, rnd() * 2 - 1, rnd() * 2 - 1};
            float l2 = lengthSq(v);
            if (l2 > 0.01f && l2 <= 1.0f) return v / std::sqrt(l2);
        }
    }

    // 磁石・砂鉄・立方体の磁石・鉄の塊（材質は demo_common.h）
    RigidBody* addMagnet(const Vec3& pos, const Vec3& dir, bool fixed = false) {
        return makeMagnetBall(world, pos, dir, fixed);
    }
    void addGrain(const Vec3& pos) { makeGrain(world, pos); }
    RigidBody* addMagnetCube(const Vec3& pos, const Quat& q) { return makeMagnetCube(world, pos, q); }
    RigidBody* addIron(const Vec3& pos, const Vec3& he, const Quat& q = Quat{}) {
        return makeIronBox(world, pos, he, q);
    }

    Quat rndQuat() { return Quat::fromAxisAngle(rndDir(), rnd() * 2.0f * PHYS_PI); }

    void applyField() {
        const Vec3 dir = (fieldMode == 1) ? Vec3{1, 0, 0} : (fieldMode == 2) ? Vec3{0, 1, 0} : Vec3{0, 0, 0};
        world.magnets().externalField = dir * fieldStrength;
    }

    // 砂鉄を叩く（紙を指で叩くのと同じ。静止摩擦から解放して磁力線に沿って並び直させる）
    void tapGrains() {
        for (const MagneticBody& mb : world.magnets().bodies()) {
            if (mb.body->isStatic() || lengthSq(mb.permanentLocal) > 0.0f) continue;
            if (mb.body->shape.type != ShapeType::Sphere) continue;   // 鉄の塊は叩かない
            mb.body->velocity += Vec3{(rnd() - 0.5f) * 0.4f, 0.6f + rnd() * 0.4f, (rnd() - 0.5f) * 0.4f};
        }
    }

    // --- 見た目 ---
    static Color palette(int i) {
        static const Color cols[] = {
            {230,  95,  90, 255}, {245, 170,  70, 255}, {245, 220,  95, 255},
            { 95, 200, 130, 255}, { 80, 165, 235, 255}, {150, 120, 225, 255},
            {235, 130, 185, 255}, {110, 210, 205, 255},
        };
        return cols[i % 8];
    }

    RigidBody* addBox(const Vec3& pos, const Vec3& he, float mass, Color c) {
        RigidBody* b = world.createBox(pos, he, mass);
        if ((int)colors.size() <= b->id) colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }
    RigidBody* addSphere(const Vec3& pos, float r, float mass, Color c) {
        RigidBody* b = world.createSphere(pos, r, mass);
        if ((int)colors.size() <= b->id) colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }

    void addGround() {
        RigidBody* g = addBox({0, -1.0f, 0}, {30.0f, 1.0f, 30.0f}, 0.0f, Color{70, 75, 85, 255});
        g->friction    = 0.7f;
        g->restitution = 0.1f;
    }

    // -----------------------------------------------------------------------
    // シーン
    // -----------------------------------------------------------------------
    // fresh = false ならシーン 9, 0, - の設定（周波数や個数）を保ったまま作り直す
    void buildScene(int index, bool fresh = true) {
        sceneIndex    = index;
        arcGroups.clear();
        fieldStrength = 5.0f;
        tracer.reset();
        tracer.fluxPerLine = 2.0f * PHYS_PI * MAG_M0 * magnetMomentScale() / (1.01f * MAG_RADIUS) / LINES_PER_MAGNET;
        if (index >= 8) {
            if (fresh || !fun)
                fun = (index == 8) ? makeRollerScene() : (index == 9) ? makeTowerScene()
                    : (index == 10) ? makeCompassScene() : (index == 11) ? makeGaussScene()
                    : (index == 12) ? makeFieldScene() : (index == 13) ? makeBellScene()
                    : (index == 14) ? makeLorentzScene() : (index == 15) ? makeEddyScene() : (index == 16) ? makeEFieldScene()
                    : (index == 17) ? makeOerstedScene() : (index == 18) ? makeElectromagnetScene() : (index == 19) ? makeHelmholtzScene() : makeCoilGunScene();
            if (fresh) fieldLinesOn = fun->wantsFieldLines();
            if (fun->fieldLineFlux() > 0.0f) tracer.fluxPerLine = fun->fieldLineFlux();
            SceneHost h = host();
            fun->build(h);
            camTargetX   = fun->camTargetX;
            camTargetY   = fun->camTargetY;
            camTargetZ   = fun->camTargetZ;
            camDistance  = fun->camDistance;
            camPitchPref = fun->camPitch;
            camYawPref   = fun->camYaw;
            timeScale    = fun->timeScale;
            return;
        }
        fun.reset();
        if (fresh) fieldLinesOn = false;
        timeScale    = 1.0f;
        camPitchPref = 0.28f;
        camYawPref   = 0.9f;
        camTargetX   = 0.0f;
        camTargetZ   = 0.0f;
        camTargetY   = 3.0f;
        camDistance  = 24.0f;
        world.clear();
        colors.clear();
        arcGroups.clear();
        seed           = 12345u + (unsigned)index;
        world.gravity  = Vec3{0, -9.81f, 0};
        world.substeps = 1;
        fieldStrength  = 5.0f;
        addGround();

        switch (index) {
        case 0: { // 箱のピラミッド
            const int   base = 6;
            const float s    = 0.5f;   // 半辺
            const float gap  = 0.02f;
            for (int row = 0; row < base; ++row) {
                int n = base - row;
                for (int i = 0; i < n; ++i) {
                    float x = (i - (n - 1) * 0.5f) * (2 * s + gap);
                    float y = s + row * (2 * s + gap);
                    RigidBody* b = addBox({x, y, 0}, {s, s, s}, 1.0f, palette(row));
                    b->friction    = 0.6f;
                    b->restitution = 0.0f;
                }
            }
            break;
        }
        case 1: { // ねじれた塔
            camTargetY = 8.0f; camDistance = 34.0f;
            const float s = 0.6f;
            for (int i = 0; i < 14; ++i) {
                RigidBody* b = addBox({0, s + i * (2 * s + 0.01f), 0}, {s, s, s}, 1.0f, palette(i));
                b->orientation = Quat::fromAxisAngle({0, 1, 0}, i * 0.12f);
                b->friction    = 0.7f;
                b->restitution = 0.0f;
            }
            break;
        }
        case 2: { // 箱の中に球を積む
            camTargetY = 2.2f; camDistance = 15.0f;

            // 静的な壁で囲いを作る（球は転がって散らばるため）
            const float W = 2.4f, H = 1.4f, T = 0.25f;
            Color wallCol{85, 92, 105, 255};
            addBox({ W, H, 0}, {T, H, W + T}, 0.0f, wallCol);
            addBox({-W, H, 0}, {T, H, W + T}, 0.0f, wallCol);
            addBox({0, H,  W}, {W + T, H, T}, 0.0f, wallCol);
            addBox({0, H, -W}, {W + T, H, T}, 0.0f, wallCol);

            int k = 0;
            unsigned seed = 12345;
            auto rnd = [&seed]() {           // 決定論的な微小ジッタ
                seed = seed * 1664525u + 1013904223u;
                return ((seed >> 16) & 0xFFFF) / 65535.0f - 0.5f;
            };
            for (int y = 0; y < 7; ++y)
                for (int x = 0; x < 4; ++x)
                    for (int z = 0; z < 4; ++z) {
                        float r = 0.4f;
                        RigidBody* b = addSphere({(x - 1.5f) * 0.95f + rnd() * 0.1f,
                                                  1.6f + y * 0.9f,
                                                  (z - 1.5f) * 0.95f + rnd() * 0.1f},
                                                 r, 0.8f, palette(k++));
                        b->friction    = 0.4f;
                        b->restitution = 0.05f;
                    }
            break;
        }
        case 3: { // 斜面・ドミノ・混在
            camTargetY = 2.0f; camDistance = 30.0f;
            RigidBody* ramp = addBox({-7, 2.0f, 0}, {5.0f, 0.25f, 3.0f}, 0.0f, Color{95, 105, 120, 255});
            ramp->orientation = Quat::fromAxisAngle({0, 0, 1}, -0.35f);
            ramp->friction    = 0.35f;

            for (int i = 0; i < 4; ++i) {
                RigidBody* b = addBox({-10.0f + i * 0.9f, 5.6f - i * 0.35f, (i - 1.5f) * 1.2f},
                                      {0.4f, 0.4f, 0.4f}, 1.0f, palette(i));
                b->friction = 0.3f;
            }
            for (int i = 0; i < 3; ++i)
                addSphere({-9.5f + i * 1.1f, 6.4f, -2.0f + i * 1.6f}, 0.4f, 1.2f, palette(i + 4))
                    ->friction = 0.3f;

            // ドミノ列
            for (int i = 0; i < 14; ++i) {
                RigidBody* d = addBox({2.0f + i * 0.85f, 0.9f, 0}, {0.12f, 0.9f, 0.5f}, 0.6f, palette(i));
                d->friction    = 0.6f;
                d->restitution = 0.0f;
            }
            // 先頭を倒すための球
            addSphere({0.2f, 2.6f, 0}, 0.35f, 2.5f, Color{255, 240, 240, 255})->velocity = Vec3{7, 0, 0};
            break;
        }
        case 4: { // 磁石の球をばらまく
            camTargetY = 1.0f; camDistance = 12.0f;
            world.substeps = MAG_SUBSTEPS;
            std::vector<Vec3> placed;
            while ((int)placed.size() < 64) {
                Vec3 p{(rnd() - 0.5f) * 7.0f, 1.0f + rnd() * 5.0f, (rnd() - 0.5f) * 7.0f};
                bool ok = true;
                for (const Vec3& q : placed) if (length(p - q) < 0.9f) { ok = false; break; }
                if (!ok) continue;
                placed.push_back(p);
                addMagnet(p, rndDir());
            }
            break;
        }
        case 5: { // 鎖か輪か: 隙間を 1 か所あけた N = 3〜8 の円弧を床に並べて同時に放す
            camTargetY = 0.0f; camDistance = 14.0f;
            world.substeps = MAG_SUBSTEPS;
            const float d = 2.0f * MAG_RADIUS;
            for (int g = 0; g < 6; ++g) {
                ArcGroup grp;
                grp.N = 3 + g;
                // 3 x 2 の格子。間隔 6 なら群どうしの引力は床の摩擦より十分弱い
                const float cx = (g % 3 - 1) * 6.0f;
                const float cz = (g / 3 - 0.5f) * 6.0f;
                const int   P  = grp.N + 1;                  // (N+1) 角形から 1 頂点を抜いた円弧
                const float Rc = 0.5f * d / std::sin(PHYS_PI / P);
                for (int k = 0; k < grp.N; ++k) {
                    float th = 2.0f * PHYS_PI * k / P;
                    grp.members.push_back(addMagnet({cx + Rc * std::cos(th), MAG_RADIUS, cz + Rc * std::sin(th)},
                                                    {-std::sin(th), 0, std::cos(th)}));   // 接線方向
                }
                arcGroups.push_back(grp);
            }
            break;
        }
        case 6: { // 砂鉄: 床に固定した磁石のまわりに粒をまく
            camTargetY = 0.0f; camDistance = 3.2f;
            world.substeps = MAG_SUBSTEPS;
            fieldStrength = 40.0f;   // 粒どうしの引力が重さの約 20 倍になる強さ
            std::vector<Vec3> fixedPos;
            if (sandLayout == 3) {                        // 3: 磁石なし、一様な外部磁場だけ
                if (fieldMode == 0) fieldMode = 1;
            } else if (sandLayout == 0) {
                fixedPos.push_back({0, MAG_RADIUS, 0});
                addMagnet(fixedPos[0], {1, 0, 0}, true);
            } else {                                      // 1: N-S（引き合う向き） 2: N-N（反発する向き）
                fixedPos.push_back({-0.9f, MAG_RADIUS, 0});
                fixedPos.push_back({ 0.9f, MAG_RADIUS, 0});
                addMagnet(fixedPos[0], {1, 0, 0}, true);
                addMagnet(fixedPos[1], {sandLayout == 1 ? 1.0f : -1.0f, 0, 0}, true);
            }
            for (MagneticBody& mb : world.magnets().bodies()) mb.body->restitution = 0.0f;   // 粒を跳ね返さない

            // 粒は磁石に引き寄せられる範囲（床の摩擦に勝てる範囲、磁石の中心から約 1）にまく
            int n = 0;
            while (n < GRAIN_COUNT) {
                Vec3 p{(rnd() - 0.5f) * 3.0f, GRAIN_RADIUS + rnd() * 0.3f, (rnd() - 0.5f) * 2.0f};
                float nearest = 1e9f;
                for (const Vec3& q : fixedPos) nearest = std::min(nearest, std::hypot(p.x - q.x, p.z - q.z));
                if (fixedPos.empty()) {
                    if (std::fabs(p.x) > 0.9f || std::fabs(p.z) > 0.9f) continue;
                } else if (nearest < MAG_RADIUS + 2.0f * GRAIN_RADIUS || nearest > 1.0f) continue;
                addGrain(p);
                ++n;
            }
            applyField();
            world.magnets().updateMoments(3);
            break;
        }
        case 7: { // 立方体の磁石と鉄の塊
            camTargetY = 0.5f; camDistance = 11.0f;
            world.substeps = MAG_SUBSTEPS;
            addIron({-2.2f, 0.12f, 0.0f}, {0.12f, 0.12f, 0.9f});     // 棒（釘）
            addIron({ 2.2f, 0.06f, 0.0f}, {0.7f, 0.06f, 0.7f});      // 板
            addIron({ 0.0f, 0.3f, -2.3f}, {0.3f, 0.3f, 0.3f});       // 立方体
            std::vector<Vec3> placed;
            while ((int)placed.size() < 27) {
                Vec3 p{(rnd() - 0.5f) * 5.0f, 1.2f + rnd() * 3.5f, (rnd() - 0.5f) * 5.0f};
                bool ok = true;
                for (const Vec3& q : placed) if (length(p - q) < 1.0f) { ok = false; break; }
                if (!ok) continue;
                placed.push_back(p);
                addMagnetCube(p, rndQuat());
            }
            applyField();
            world.magnets().updateMoments(3);
            break;
        }
        }
        applyField();
    }

    // 固定ステップ 1 回（シーン 9, 0, -, = は直前に外部磁場などを更新する）
    void stepPhysics(float dt) {
        if (fun) fun->beforeStep(world, dt);
        world.step(dt);
    }

    // -----------------------------------------------------------------------
    void shoot(const Camera3D& cam) {
        Vector3 fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        Vec3 origin = toPhys(Vector3Add(cam.position, Vector3Scale(fwd, 1.5f)));
        if (sceneIndex >= 4) {   // 磁石のシーンでは弾も磁石球（向きはランダム）
            addMagnet(origin, rndDir())->velocity = toPhys(Vector3Scale(fwd, 25.0f));
            return;
        }
        RigidBody* b = addSphere(origin, 0.45f, 6.0f, Color{255, 250, 235, 255});
        b->velocity    = toPhys(Vector3Scale(fwd, 40.0f));
        b->restitution = 0.35f;
        b->friction    = 0.4f;
    }

    // -----------------------------------------------------------------------
    // 姿勢 q のモデルを描く
    static void drawOriented(const Model& model, const Vector3& pos, const Quat& q, float scale, Color c) {
        Vector3 axis; float angle;
        QuaternionToAxisAngle(Quaternion{q.x, q.y, q.z, q.w}, &axis, &angle);
        if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};
        DrawModelEx(model, pos, axis, angle * RAD2DEG, Vector3{scale, scale, scale}, c);
    }

    // 物体 b のローカル座標で中心 offset、半辺 he の箱を描く
    void drawBoxPart(const RigidBody* b, const Vec3& offset, const Vec3& he, Color c) const {
        Vector3 axis; float angle;
        const Quat& q = b->orientation;
        QuaternionToAxisAngle(Quaternion{q.x, q.y, q.z, q.w}, &axis, &angle);
        if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};
        Vector3 pos   = toRay(b->position + q.rotate(offset));
        Vector3 scale = toRay(he * 2.0f);
        if (!wireframe) DrawModelEx(cubeModel, pos, axis, angle * RAD2DEG, scale, c);
        DrawModelWiresEx(cubeModel, pos, axis, angle * RAD2DEG, scale, wireframe ? c : Fade(BLACK, 0.35f));
    }

    // 箱の磁石: 磁化の向き（いちばん大きい成分の軸）で箱を 2 つに分け、N 側を赤、S 側を青で描く
    void drawMagnetBox(const MagneticBody& mb) const {
        const Vec3& p  = mb.permanentLocal;
        const Vec3& he = mb.body->shape.halfExtents;
        int k = 0;
        if (std::fabs(p.y) >= std::fabs(p.x) && std::fabs(p.y) >= std::fabs(p.z)) k = 1;
        else if (std::fabs(p.z) >= std::fabs(p.x)) k = 2;
        Vec3 half = he, off{0, 0, 0};
        half[k] *= 0.5f;
        off[k]   = (p[k] >= 0.0f ? 1.0f : -1.0f) * half[k];
        drawBoxPart(mb.body, off,         half, Color{220, 65, 60, 255});
        drawBoxPart(mb.body, off * -1.0f, half, Color{60, 100, 220, 255});
    }

    // 磁性体: 磁石は N 極（赤、ローカル +y 側）と S 極（青）、鉄は磁化の強さで明るさを変える
    void drawMagnetic() const {
        const Quat flip = Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
        for (const MagneticBody& mb : world.magnets().bodies()) {
            const RigidBody* b = mb.body;
            Vector3 pos = toRay(b->position);
            float   r   = b->shape.radius;
            const bool  permanent = lengthSq(mb.permanentLocal) > 0.0f;
            // 鉄の明るさは磁化の強さ（飽和に対する割合の平方根。一様な外部磁場で弱く磁化されても見えるように）
            const float s = (!permanent && mb.saturation > 0.0f)
                                ? std::sqrt(clampf(length(mb.m) / mb.saturation, 0.0f, 1.0f)) : 0.0f;
            if (b->shape.type == ShapeType::Box) {
                if (permanent) {
                    drawMagnetBox(mb);
                } else {
                    unsigned char v = (unsigned char)(95 + 130 * s);
                    drawBoxPart(b, Vec3{0, 0, 0}, b->shape.halfExtents, Color{v, v, (unsigned char)(v + 15), 255});
                }
                continue;
            }
            if (permanent) {
                drawOriented(hemiModel, pos, b->orientation,        r, Color{220, 65, 60, 255});
                drawOriented(hemiModel, pos, b->orientation * flip, r, Color{60, 100, 220, 255});
            } else {
                const float   base = r < 0.1f ? 50.0f : 105.0f;   // 砂鉄は暗く、鉄球は明るめに
                unsigned char v = (unsigned char)(base + (215.0f - base) * s);
                DrawModel(sphereModel, pos, r, Color{v, v, (unsigned char)(v + 12), 255});
            }
        }
    }

    // シーン 6: 各群の上に「鎖 / 輪」と群内のエネルギー（理論値と比較）を表示
    void drawArcLabels(const Camera3D& cam) const {
        if (arcGroups.empty()) return;
        const MagnetSystem& ms      = world.magnets();
        const auto          structs = world.magneticStructures();
        const float d  = 2.0f * MAG_RADIUS;
        const float m0 = MAG_M0 * magnetMomentScale();
        const float u0 = m0 * m0 / (d * d * d);
        for (const ArcGroup& g : arcGroups) {
            const int   first = ms.indexOf(g.members[0]);
            const char* type  = "?";
            int size = 0;
            for (const auto& st : structs)
                for (int idx : st.members)
                    if (idx == first) { type = structureName(st.type); size = (int)st.members.size(); }

            float U = 0.0f;
            Vec3  center{0, 0, 0};
            for (size_t a = 0; a < g.members.size(); ++a) {
                const MagneticBody& A = ms.bodies()[ms.indexOf(g.members[a])];
                center += A.body->position * (1.0f / g.members.size());
                for (size_t b = a + 1; b < g.members.size(); ++b) {
                    const MagneticBody& B = ms.bodies()[ms.indexOf(g.members[b])];
                    U += dipoleEnergy(B.body->position - A.body->position, A.m, B.m);
                }
            }
            Vector2 sp = GetWorldToScreen(toRay(center + Vec3{0, 1.6f, 0}), cam);
            int x = (int)sp.x - 70, y = (int)sp.y;
            DrawText(TextFormat("N=%d  %s%s", g.N, type, size == g.N ? "" : " (split)"), x, y, 20, RAYWHITE);
            DrawText(TextFormat("U %.2f u0", U / u0), x, y + 22, 16, LIGHTGRAY);
            DrawText(TextFormat("chain %.2f / ring %.2f", CHAIN_U[g.N], RING_U[g.N]), x, y + 40, 14, GRAY);
        }
    }

    // 磁石のシーンの HUD（エネルギーの内訳、構造の数、強さ、外部磁場）
    void magnetHud(HudLines& out) const {
        EnergyReport e = world.energy();
        if (world.electric().empty())
            out.add(TextFormat("E %.1f = kin %.1f + rot %.1f + grav %.1f + mag %.1f",
                               e.total(), e.kinetic, e.rotational, e.gravity, e.magnetic));
        else
            out.add(TextFormat("E %.2f = kin %.2f + rot %.2f + grav %.2f + mag %.2f + elec %.2f",
                               e.total(), e.kinetic, e.rotational, e.gravity, e.magnetic, e.electric));
        if (fun) {
            fun->hud(world, out);
        } else {
            if (sceneIndex == 6) {
                static const char* layouts[] = {"1 magnet", "N-S", "N-N", "field only"};
                out.add(TextFormat("grains %d   layout %s", GRAIN_COUNT, layouts[sandLayout]));
            } else {
                int count[4] = {0, 0, 0, 0};
                for (const auto& st : world.magneticStructures()) ++count[(int)st.type];
                out.add(TextFormat("single %d   chain %d   ring %d   other %d",
                                   count[0], count[1], count[2], count[3]));
            }
        }
        if (!fun || !fun->ownsField()) {
            static const char* fieldNames[] = {"off", "horizontal (+x)", "vertical (+y)"};
            if (fieldMode == 0) out.add("external field off   (E to change)");
            else out.add(TextFormat("external field %s  |B0| %.0f mT   (E to change)", fieldNames[fieldMode],
                                    fieldStrength * FIELD_MT_PER_UNIT));
        }
        if (sceneIndex < 13)   // 電気のシーンの磁石は材質を使わない
            out.add(TextFormat("magnet: %s  Br %.1f T  Gamma %.0f (pull at contact / weight)   L to change",
                               MAGNET_GRADES[magnetGrade].name, MAGNET_GRADES[magnetGrade].br, magnetGamma()),
                    magnetGrade == 0 ? GRAY : Color{255, 190, 120, 255});
        out.add(TextFormat("substeps %d   gravity %s", world.substeps, world.gravity.y != 0.0f ? "ON" : "OFF"), GRAY);
        if (!fun) out.add("T tap sand   M layout (scene 7)", GRAY);
    }

    // 磁力線の色（|B| の対数。0.1 mT で暗い青 → 1 T で白）
    static Color fieldColor(float B) {
        const float t = clampf((std::log10(std::max(B * FIELD_MT_PER_UNIT, 1e-6f)) + 1.0f) / 4.0f, 0.0f, 1.0f);
        static const Color stops[4] = {{50, 70, 170, 255}, {50, 180, 235, 255}, {250, 215, 70, 255}, {255, 255, 255, 255}};
        const float x = t * 3.0f;
        const int   i = std::min(2, (int)x);
        const float f = x - i;
        const Color a = stops[i], b = stops[i + 1];
        return Color{(unsigned char)(a.r + (b.r - a.r) * f), (unsigned char)(a.g + (b.g - a.g) * f),
                     (unsigned char)(a.b + (b.b - a.b) * f), 255};
    }

    void drawFieldLines() const {
        for (const FieldLine& l : tracer.lines()) {
            if (l.pts.size() < 2) continue;
            float total = 0.0f;
            for (size_t k = 0; k + 1 < l.pts.size(); ++k) {
                DrawLine3D(toRay(l.pts[k]), toRay(l.pts[k + 1]), fieldColor(l.mag[k]));
                total += length(l.pts[k + 1] - l.pts[k]);
            }
            // 線の真ん中に向きの矢印（N → S）
            float acc = 0.0f;
            for (size_t k = 0; k + 1 < l.pts.size(); ++k) {
                const Vec3  seg = l.pts[k + 1] - l.pts[k];
                const float sl  = length(seg);
                acc += sl;
                if (acc < 0.5f * total || sl < 1e-6f) continue;
                const Vec3 dir = seg / sl;
                DrawCylinderEx(toRay(l.pts[k] - dir * 0.06f), toRay(l.pts[k] + dir * 0.08f), 0.035f, 0.0f, 6,
                               fieldColor(l.mag[k]));
                break;
            }
        }
    }

    // 磁力線の色の凡例（画面の左下）
    static void drawFieldLegend() {
        const int x = 16, y = GetScreenHeight() - 44, w = 260, h = 10;
        DrawRectangle(x - 8, y - 22, w + 16, 58, Fade(BLACK, 0.55f));
        DrawText("field lines: |B|", x, y - 18, 14, LIGHTGRAY);
        for (int i = 0; i < w; ++i) {
            const float mT = std::pow(10.0f, -1.0f + 4.0f * i / (w - 1));
            DrawRectangle(x + i, y, 1, h, fieldColor(mT / FIELD_MT_PER_UNIT));
        }
        static const char* labels[] = {"0.1", "1", "10", "100", "1000 mT"};
        for (int k = 0; k < 5; ++k) DrawText(labels[k], x + k * (w - 1) / 4 - (k == 4 ? 20 : 4), y + h + 4, 12, GRAY);
    }

    void draw() const {
        drawMagnetic();
        drawBodies(false);
        if (!world.currents().empty()) drawWires(world, GetTime());
        if (fieldLinesOn) drawFieldLines();
        rlDisableDepthMask();   // 半透明（筒のガラスなど）は不透明な物体の後に、奥行きを書かずに描く
        drawBodies(true);
        rlEnableDepthMask();
        if (showContacts) {
            for (const auto& m : world.getManifolds()) {
                for (int i = 0; i < m.count; ++i) {
                    Vector3 p = toRay(m.contacts[i].position);
                    DrawSphere(p, 0.06f, RED);
                    DrawLine3D(p, toRay(m.contacts[i].position + m.normal * 0.5f), YELLOW);
                }
            }
        }
    }

    void drawBodies(bool translucent) const {
        for (const auto& b : world.getBodies()) {
            if (world.magnets().indexOf(b.get()) >= 0) continue;   // 磁性体は drawMagnetic で描く
            Color c = (b->id < (int)colors.size()) ? colors[b->id] : WHITE;
            if ((c.a < 255) != translucent) continue;
            const int ei = world.electric().indexOf(b.get());   // 帯電した球は電荷の色（+ 橙、- 紫、導体は銅色）
            if (ei >= 0 && b->shape.type == ShapeType::Sphere) {
                const ElectricBody& eb = world.electric().bodies()[ei];
                const Color cc = chargeColor(eb.charge, fun ? fun->chargeRef : 1.0f, eb.conductor);
                c = Color{cc.r, cc.g, cc.b, c.a};
            }

            Quaternion q{b->orientation.x, b->orientation.y, b->orientation.z, b->orientation.w};
            Vector3 axis; float angle;
            QuaternionToAxisAngle(q, &axis, &angle);
            if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};

            Vector3 pos = toRay(b->position);

            if (b->shape.type == ShapeType::Box) {
                Vector3 scale = toRay(b->shape.halfExtents * 2.0f);
                if (!wireframe) DrawModelEx(cubeModel, pos, axis, angle * RAD2DEG, scale, c);
                DrawModelWiresEx(cubeModel, pos, axis, angle * RAD2DEG, scale,
                                 wireframe ? c : Fade(BLACK, translucent ? 0.12f : 0.35f));
            } else {
                float r = b->shape.radius;
                Vector3 scale{r, r, r};
                if (!wireframe) DrawModelEx(sphereModel, pos, axis, angle * RAD2DEG, scale, c);
                DrawModelWiresEx(sphereModel, pos, axis, angle * RAD2DEG, scale,
                                 wireframe ? c : Fade(BLACK, 0.25f));
            }
        }
    }

    const char* sceneName() const {
        static const char* names[] = {
            "1: Box pyramid", "2: Twisted tower", "3: Spheres in a bin", "4: Ramp + dominoes",
            "5: Magnet balls", "6: Chain or ring?", "7: Iron sand", "8: Cube magnets + iron"};
        return fun ? fun->title() : names[sceneIndex];
    }
};

// ---------------------------------------------------------------------------
int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "raylib 3D Physics Engine");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);   // Esc は FPS 視点を抜けるのに使う（通常の視点では終了）

    Demo demo;
    demo.cubeModel   = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    demo.sphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 14, 20));
    demo.hemiModel   = LoadModelFromMesh(GenMeshHemiSphere(1.0f, 10, 20));
    demo.buildScene(0);

    // --- カメラ ---
    //   通常: 注視点のまわりを回る（球面座標。ドラッグで回転、ホイールでズーム）
    //   FPS : Tab で切り替え。マウスで見回し、WASD で移動する
    float   camYaw = 0.9f, camPitch = demo.camPitchPref;
    float   camDist   = demo.camDistance;
    Vector3 camTarget{0, demo.camTargetY, 0};
    bool    fps = false;
    Vector3 fpsPos{0, 0, 0};                 // FPS 視点の目の位置
    Walker  walker;
    float   fpsYaw = 0.0f, fpsPitch = 0.0f, fpsSpeed = 4.0f;

    auto resetCamera = [&]() {
        camTarget = Vector3{demo.camTargetX, demo.camTargetY, demo.camTargetZ};
        camDist   = demo.camDistance;
        camPitch  = demo.camPitchPref;
        camYaw    = demo.camYawPref;
    };
    auto fpsForward = [&]() {
        return Vector3{cosf(fpsPitch) * sinf(fpsYaw), sinf(fpsPitch), cosf(fpsPitch) * cosf(fpsYaw)};
    };
    auto leaveFps = [&]() {
        // 今見ている点のまわりを回る通常の視点に戻す
        fps = false;
        EnableCursor();
        Vector3 f = fpsForward();
        camTarget = Vector3Add(fpsPos, Vector3Scale(f, camDist));
        camYaw    = atan2f(-f.x, -f.z);
        camPitch  = asinf(Clamp(-f.y, -1.0f, 1.0f));
    };

    Camera3D camera{};
    camera.up         = Vector3{0, 1, 0};
    camera.fovy       = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    const float FIXED_DT = 1.0f / 120.0f;
    float  accumulator = 0.0f;
    double simClock = 0.0, realClock = 0.0;   // 実際に出た速さの測定
    float  achievedScale = 1.0f;
    bool   quit = false;

    while (!WindowShouldClose() && !quit) {
        const float frameDt = GetFrameTime();

        // ------------------------------------------------------------------
        // カメラ
        // ------------------------------------------------------------------
        if (IsKeyPressed(KEY_TAB)) {
            if (!fps) {
                // 注視点から水平に 3〜8 離れたところ（カメラの側）へ、カメラの高さから降りて、注視点を向く
                fps = true;
                const Vec3 tgt = toPhys(camera.target), cam = toPhys(camera.position);
                Vec3  h{cam.x - tgt.x, 0.0f, cam.z - tgt.z};
                float hl = length(h);
                if (hl < 1e-3f) { h = Vec3{0, 0, 1}; hl = 1.0f; }
                const float dist = clampf(hl, 3.0f, 8.0f);
                walker.feet     = Vec3{tgt.x + h.x / hl * dist, std::max(cam.y - Walker::EYE, 0.0f), tgt.z + h.z / hl * dist};
                walker.velY     = 0.0f;
                walker.onGround = false;
                walker.eye      = Walker::EYE;
                fpsPos   = toRay(walker.feet + Vec3{0, Walker::EYE, 0});
                fpsYaw   = atan2f(-h.x, -h.z);
                fpsPitch = Clamp(atan2f(tgt.y - Walker::EYE, dist), -0.6f, 0.3f);
                DisableCursor();
            } else {
                leaveFps();
            }
        }
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (fps) leaveFps();
            else     quit = true;
        }

        if (fps) {
            Vector2 d = GetMouseDelta();
            fpsYaw  -= d.x * 0.0025f;
            fpsPitch = Clamp(fpsPitch - d.y * 0.0025f, -1.5f, 1.5f);
            fpsSpeed = Clamp(fpsSpeed * powf(1.2f, GetMouseWheelMove()), 0.5f, 40.0f);

            // 地面を歩く（WASD は水平、Space でジャンプ、Ctrl でしゃがむ）
            const Vec3  flat{sinf(fpsYaw), 0.0f, cosf(fpsYaw)};
            const Vec3  right{-flat.z, 0.0f, flat.x};
            const bool  crouch = IsKeyDown(KEY_LEFT_CONTROL);
            const float dt     = std::min(frameDt, 0.05f);
            const float step   = fpsSpeed * (IsKeyDown(KEY_LEFT_SHIFT) ? 2.5f : 1.0f) * (crouch ? 0.5f : 1.0f) * dt;
            Vec3 move{0, 0, 0};
            if (IsKeyDown(KEY_W)) move += flat;
            if (IsKeyDown(KEY_S)) move -= flat;
            if (IsKeyDown(KEY_D)) move += right;
            if (IsKeyDown(KEY_A)) move -= right;
            if (lengthSq(move) > 0.0f) move = normalize(move) * step;
            walker.update(demo.world, move, IsKeyPressed(KEY_SPACE), crouch, dt);
            fpsPos = toRay(walker.feet + Vec3{0, walker.eye, 0});

            camera.position = fpsPos;
            camera.target   = Vector3Add(fpsPos, fpsForward());
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) demo.shoot(camera);
        } else {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d = GetMouseDelta();
                camYaw   -= d.x * 0.005f;
                camPitch += d.y * 0.005f;
                camPitch  = Clamp(camPitch, -1.45f, 1.45f);
            }
            camDist = Clamp(camDist - GetMouseWheelMove() * 1.8f, 2.0f, 90.0f);

            camera.target   = camTarget;
            camera.position = Vector3{
                camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
                camTarget.y + camDist * sinf(camPitch),
                camTarget.z + camDist * cosf(camPitch) * cosf(camYaw)};
            if (IsKeyPressed(KEY_SPACE) && !(demo.fun && demo.fun->ownsSpace())) demo.shoot(camera);
            if (IsKeyPressed(KEY_W))     demo.wireframe = !demo.wireframe;
        }

        // ------------------------------------------------------------------
        // キー操作
        // ------------------------------------------------------------------
        if (IsKeyPressed(KEY_R)) {
            demo.buildScene(demo.sceneIndex, false);
            if (!fps) resetCamera();
        }
        if (IsKeyPressed(KEY_P))     demo.paused = !demo.paused;
        if (IsKeyPressed(KEY_C))     demo.showContacts = !demo.showContacts;
        if (IsKeyPressed(KEY_G))     demo.world.enableGyroscopic = !demo.world.enableGyroscopic;
        if (IsKeyPressed(KEY_Z))
            demo.world.gravity = (demo.world.gravity.y != 0.0f) ? Vec3{0, 0, 0} : Vec3{0, -9.81f, 0};
        if (IsKeyPressed(KEY_H))     demo.showHud = !demo.showHud;
        if (IsKeyPressed(KEY_F) && demo.sceneIndex >= 4)
            demo.timeScale = (demo.timeScale > 1.5f) ? 1.0f : TIME_SCALE_REAL;
        if (IsKeyPressed(KEY_L)) {                       // 磁石の材質（磁力の強さ）を変えて作り直す
            magnetGrade = (magnetGrade + 1) % MAGNET_GRADE_COUNT;
            if (demo.sceneIndex >= 4) {
                const float keepScale = demo.timeScale;
                demo.buildScene(demo.sceneIndex, false);
                demo.timeScale = keepScale;
                if (!fps) resetCamera();
            }
        }
        if (!demo.fun) {
            if (IsKeyPressed(KEY_T)) demo.tapGrains();
            if (IsKeyPressed(KEY_M) && demo.sceneIndex == 6) {
                demo.sandLayout = (demo.sandLayout + 1) % 4;
                if (demo.sandLayout == 0) demo.fieldMode = 0;   // 一巡したら外部磁場も戻す
                demo.buildScene(6);
            }
        }
        if (IsKeyPressed(KEY_E) && !(demo.fun && demo.fun->ownsField())) {
            demo.fieldMode = (demo.fieldMode + 1) % 3;
            demo.applyField();
        }
        if (demo.fun) {
            SceneHost h = demo.host();
            demo.fun->handleInput(h);
            if (demo.fun->wantCameraReset) {
                demo.fun->wantCameraReset = false;
                demo.camTargetX  = demo.fun->camTargetX;
                demo.camTargetY  = demo.fun->camTargetY;
                demo.camTargetZ  = demo.fun->camTargetZ;
                demo.camDistance = demo.fun->camDistance;
                demo.camPitchPref = demo.fun->camPitch;
                demo.camYawPref   = demo.fun->camYaw;
                if (!fps) resetCamera();
            }
            if (demo.fun->wantFieldLinesReset) {
                demo.fun->wantFieldLinesReset = false;
                demo.fieldLinesOn = demo.fun->wantsFieldLines();
                demo.tracer.reset();
            }
        }

        int nextScene = -1;
        for (int k = 0; k < 8; ++k)
            if (IsKeyPressed(KEY_ONE + k)) nextScene = k;
        if (IsKeyPressed(KEY_NINE))  nextScene = 8;
        if (IsKeyPressed(KEY_ZERO))  nextScene = 9;
        if (IsKeyPressed(KEY_MINUS)) nextScene = 10;
        if (IsKeyPressed(KEY_EQUAL)) nextScene = 11;
        if (IsKeyPressed(KEY_BACKSPACE)) nextScene = 12;
        for (int k = 0; k < 8; ++k)   // 電気のシーン（F5〜F8）、電流のシーン（F9〜F11）、コイル砲台（F12）
            if (IsKeyPressed(KEY_F5 + k)) nextScene = 13 + k;
        if (IsKeyPressed(KEY_O) && demo.sceneIndex >= 4) {
            demo.fieldLinesOn = !demo.fieldLinesOn;
            demo.tracer.reset();
        }
        if (nextScene >= 0) {
            demo.fieldMode = 0;   // シーンを変えたら外部磁場は切る
            demo.buildScene(nextScene);
            if (!fps) resetCamera();
        }

        // ------------------------------------------------------------------
        // 物理更新（固定タイムステップ）
        //   描画フレームレートに関係なく、常に同じ dt で積分する。
        //   1 実秒あたり timeScale 秒ぶん進める。追いつけないぶんは捨てる（遅くなるだけで破綻しない）
        // ------------------------------------------------------------------
        bool   stepOnce = IsKeyPressed(KEY_N);
        double t0 = GetTime();
        int    steps = 0;
        if (!demo.paused) {
            accumulator += std::min(frameDt, 0.1f) * demo.timeScale;
            const float maxAcc = demo.timeScale * 0.05f + FIXED_DT;   // スパイラル・オブ・デス回避
            if (accumulator > maxAcc) accumulator = maxAcc;
            while (accumulator >= FIXED_DT) {
                demo.stepPhysics(FIXED_DT);
                accumulator -= FIXED_DT;
                ++steps;
            }
            simClock  += steps * FIXED_DT;
            realClock += frameDt;
            if (realClock > 0.5) {
                achievedScale = (float)(simClock / realClock);
                simClock = realClock = 0.0;
            }
        } else if (stepOnce) {
            demo.stepPhysics(FIXED_DT);
        }
        demo.stepMs = demo.stepMs * 0.9 + (GetTime() - t0) * 1000.0 * 0.1;

        // 磁力線: 1 フレームに 2.5 ms までたどる（物が動かなければ、たどり終えた後は何もしない）
        if (demo.fieldLinesOn && demo.sceneIndex >= 4) demo.tracer.update(demo.world, 2.5);
        if (demo.fun) {
            const Vector2 center{GetScreenWidth() * 0.5f, GetScreenHeight() * 0.5f};
            demo.fun->updateView(demo.world, camera, fps ? center : GetMousePosition());
        }

        // ------------------------------------------------------------------
        // 描画
        // ------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{28, 30, 38, 255});

        BeginMode3D(camera);
            demo.draw();
            if (demo.fun) demo.fun->draw3D(demo.world);
            // 地面の上面 (y=0) と重なると Z ファイティングするので少し持ち上げる
            rlPushMatrix();
            rlTranslatef(0.0f, 0.01f, 0.0f);
            DrawGrid(60, 1.0f);
            rlPopMatrix();
        EndMode3D();
        demo.drawArcLabels(camera);
        if (demo.fun) demo.fun->draw2D(demo.world, camera);
        if (demo.fieldLinesOn && demo.sceneIndex >= 4 && demo.showHud) Demo::drawFieldLegend();

        if (fps) {   // 照準
            const int cx = GetScreenWidth() / 2, cy = GetScreenHeight() / 2;
            DrawLine(cx - 8, cy, cx + 8, cy, Fade(RAYWHITE, 0.7f));
            DrawLine(cx, cy - 8, cx, cy + 8, Fade(RAYWHITE, 0.7f));
        }

        if (demo.showHud) {
            const bool magScene = demo.sceneIndex >= 4;
            const bool compact  = demo.fun && demo.fun->compactHud();   // ゲームのシーン: 汎用の案内を出さない
            HudLines hud;
            hud.add(demo.sceneName(), RAYWHITE, 20);
            if (compact) {
                hud.add(TextFormat("physics %.2f ms/frame   %d FPS   %s", demo.stepMs, GetFPS(), demo.paused ? "[PAUSED]" : ""), GRAY, 14);
                hud.gap(6);
                demo.fun->hud(demo.world, hud);
                hud.draw(10, 10, 330);
                EndDrawing();
                continue;
            }
            hud.add(TextFormat("bodies %d   contacts %d", demo.world.bodyCount(), demo.world.contactCount()));
            hud.add(TextFormat("physics %.2f ms/frame   %d FPS", demo.stepMs, GetFPS()));
            hud.add(TextFormat("gyroscopic %s   %s", demo.world.enableGyroscopic ? "ON" : "OFF",
                               demo.paused ? "[PAUSED]" : ""));
            if (demo.sceneIndex >= 13) {   // 電気のシーン: 長さは m、時間は実時間（lambda の換算なし）
                hud.add(TextFormat("time x%.2f of real time   (F: %s)", demo.paused ? 0.0f : achievedScale,
                                   demo.timeScale > 1.5f ? "back to x1" : "x10"), LIGHTGRAY);
            } else if (magScene) {
                const bool realTime = demo.timeScale > 1.5f;
                hud.add(TextFormat("real time x%.2f   (target x%.1f, F: %s)",
                                   demo.paused ? 0.0f : achievedScale / TIME_SCALE_REAL,
                                   demo.timeScale / TIME_SCALE_REAL, realTime ? "slow motion" : "real time"),
                        realTime ? Color{120, 220, 255, 255} : LIGHTGRAY);
            }
            hud.gap(6);
            hud.add("SPACE shoot   1-8 9 0 - = BkSp scene   F5-F8 electric   F9-F12 currents   R reset   P pause   N step", GRAY);
            hud.add(magScene ? "C contacts   G gyro   Z gravity   H hud   O field lines   L magnet material"
                             : "C contacts   G gyro   Z gravity   H hud", GRAY);
            hud.add(fps ? "FPS: mouse look  WASD walk  Space jump  Ctrl crouch  Shift run  click shoot  Tab/Esc exit"
                        : "drag/wheel camera   Tab FPS camera   W wireframe", GRAY);
            if (magScene) {
                hud.gap(8);
                demo.magnetHud(hud);
            }
            hud.draw(10, 10, 330);
        }

        EndDrawing();
    }

    UnloadModel(demo.cubeModel);
    UnloadModel(demo.sphereModel);
    UnloadModel(demo.hemiModel);
    CloseWindow();
    return 0;
}
