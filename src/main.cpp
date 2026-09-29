// ---------------------------------------------------------------------------
// main.cpp : raylib による 3D 物理エンジンのデモ
//
//   操作:
//     左ドラッグ / 右ドラッグ : カメラ回転
//     マウスホイール          : ズーム
//     Space                   : 視点方向に球を発射（シーン 5〜7 では磁石球）
//     1 - 7                   : シーン切り替え
//     R                       : シーンをリセット
//     P                       : 一時停止 / 再開
//     N                       : 1 ステップだけ進める（一時停止中）
//     C                       : 接触点の表示切り替え
//     W                       : ワイヤーフレーム表示切り替え
//     G                       : ジャイロ効果の ON/OFF
//     Z                       : 重力の ON/OFF
//     T                       : 砂鉄を叩く（シーン 7）
//     M                       : 磁石の配置を切り替え（シーン 7）
//     H                       : HUD の表示切り替え
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "phys/world.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace phys;

// ---------------------------------------------------------------------------
// 変換ヘルパ
// ---------------------------------------------------------------------------
static inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }
static inline Vec3    toPhys(const Vector3& v) { return Vec3{v.x, v.y, v.z}; }

// ---------------------------------------------------------------------------
// 磁石の設定（MAGNET_SPEC.md §3.3）
//   直径 5 mm のネオジム球（Gamma = 779）を半径 0.25 に拡大（lambda = 100）。
//   重力はそのままなので時間は sqrt(lambda) = 10 倍に伸びる（画面は実時間の 1/10）
// ---------------------------------------------------------------------------
static const float MAG_RADIUS   = 0.25f;
static const float MAG_MASS     = 0.5f;
static const float MAG_GAMMA    = 779.0f;
static const int   MAG_SUBSTEPS = 8;
static const float MAG_M0       = momentFromGamma(MAG_GAMMA, MAG_MASS, 2.0f * MAG_RADIUS, 9.81f);

// 砂鉄（磁鉄鉱の丸い粒）: 等方的な磁化率 0.7 r^3、飽和は磁石の単位体積あたりモーメントの 0.5 倍
static const float GRAIN_RADIUS = 0.04f;
static const int   GRAIN_COUNT  = 200;

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
    float camTargetY   = 3.0f;   // シーンごとの推奨カメラ設定
    float camDistance  = 24.0f;
    bool  paused       = false;
    bool  showContacts = false;
    bool  wireframe    = false;
    bool  showHud      = true;
    double stepMs      = 0.0;

    // --- 磁石のシーン ---
    int      sandLayout = 0;    // シーン 7 の配置（M キー）
    unsigned seed = 1;

    struct ArcGroup { int N; std::vector<RigidBody*> members; };   // シーン 6 の各群
    std::vector<ArcGroup> arcGroups;

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

    // 磁石球（向き dir、強さ MAG_M0）
    RigidBody* addMagnet(const Vec3& pos, const Vec3& dir, bool fixed = false) {
        RigidBody* b = world.createMagnet(pos, MAG_RADIUS, fixed ? 0.0f : MAG_MASS, normalize(dir) * MAG_M0);
        b->restitution     = 0.3f;   // ニッケルめっきの金属球
        b->friction        = 0.3f;
        b->rollingFriction = 0.01f;
        return b;
    }

    // 砂鉄の粒（磁鉄鉱の丸い粒。磁石と同じ密度）
    void addGrain(const Vec3& pos) {
        const float r     = GRAIN_RADIUS;
        const float ratio = (r / MAG_RADIUS) * (r / MAG_RADIUS) * (r / MAG_RADIUS);
        RigidBody* b = world.createSoftMagnet(pos, r, MAG_MASS * ratio, 0.7f * r * r * r, 0.5f * MAG_M0 * ratio);
        b->restitution     = 0.0f;
        b->friction        = 0.6f;
        b->rollingFriction = 0.1f;
        b->linearDamping   = 0.5f;
    }

    // 砂鉄を叩く（紙を指で叩くのと同じ。静止摩擦から解放して磁力線に沿って並び直させる）
    void tapGrains() {
        for (const MagneticBody& mb : world.magnets().bodies()) {
            if (mb.body->isStatic() || lengthSq(mb.permanentLocal) > 0.0f) continue;
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
    void buildScene(int index) {
        sceneIndex  = index;
        camTargetY  = 3.0f;
        camDistance = 24.0f;
        world.clear();
        colors.clear();
        arcGroups.clear();
        seed           = 12345u + (unsigned)index;
        world.gravity  = Vec3{0, -9.81f, 0};
        world.substeps = 1;
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
            std::vector<Vec3> fixedPos;
            if (sandLayout == 0) {
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
                if (nearest < MAG_RADIUS + 2.0f * GRAIN_RADIUS || nearest > 1.0f) continue;
                addGrain(p);
                ++n;
            }
            world.magnets().updateMoments(3);
            break;
        }
        }
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

    // 磁性体: 磁石は N 極（赤、ローカル +y 側）と S 極（青）の半球、砂鉄は磁化の強さで明るさを変える
    void drawMagnetic() const {
        const Quat flip = Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
        for (const MagneticBody& mb : world.magnets().bodies()) {
            const RigidBody* b = mb.body;
            Vector3 pos = toRay(b->position);
            float   r   = b->shape.radius;
            if (lengthSq(mb.permanentLocal) > 0.0f) {
                drawOriented(hemiModel, pos, b->orientation,        r, Color{220, 65, 60, 255});
                drawOriented(hemiModel, pos, b->orientation * flip, r, Color{60, 100, 220, 255});
            } else {
                float s = (mb.saturation > 0.0f) ? clampf(length(mb.m) / mb.saturation, 0.0f, 1.0f) : 0.0f;
                unsigned char v = (unsigned char)(50 + 160 * s);
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
        const float u0 = MAG_M0 * MAG_M0 / (d * d * d);
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

    // 磁石のシーンの HUD（エネルギーの内訳、構造の数、強さ、時間の倍率）
    void drawMagnetHud(int x, int y) const {
        EnergyReport e = world.energy();
        DrawText(TextFormat("E %.1f = kin %.1f + rot %.1f + grav %.1f + mag %.1f",
                            e.total(), e.kinetic, e.rotational, e.gravity, e.magnetic), x, y, 16, LIGHTGRAY);
        if (sceneIndex == 6) {
            DrawText(TextFormat("grains %d   layout %s", GRAIN_COUNT,
                                sandLayout == 0 ? "1 magnet" : sandLayout == 1 ? "N-S" : "N-N"),
                     x, y + 20, 16, LIGHTGRAY);
        } else {
            int count[4] = {0, 0, 0, 0};
            for (const auto& st : world.magneticStructures()) ++count[(int)st.type];
            DrawText(TextFormat("single %d   chain %d   ring %d   other %d",
                                count[0], count[1], count[2], count[3]), x, y + 20, 16, LIGHTGRAY);
        }
        DrawText(TextFormat("Gamma %.0f   substeps %d   gravity %s   real time x0.1",
                            MAG_GAMMA, world.substeps, world.gravity.y != 0.0f ? "ON" : "OFF"),
                 x, y + 40, 16, LIGHTGRAY);
        DrawText("Z gravity   T tap sand   M layout (scene 7)   H hud", x, y + 66, 16, GRAY);
    }

    void draw() const {
        drawMagnetic();
        for (const auto& b : world.getBodies()) {
            if (world.magnets().indexOf(b.get()) >= 0) continue;   // 磁性体は drawMagnetic で描く
            Color c = (b->id < (int)colors.size()) ? colors[b->id] : WHITE;

            Quaternion q{b->orientation.x, b->orientation.y, b->orientation.z, b->orientation.w};
            Vector3 axis; float angle;
            QuaternionToAxisAngle(q, &axis, &angle);
            if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};

            Vector3 pos = toRay(b->position);

            if (b->shape.type == ShapeType::Box) {
                Vector3 scale = toRay(b->shape.halfExtents * 2.0f);
                if (!wireframe) DrawModelEx(cubeModel, pos, axis, angle * RAD2DEG, scale, c);
                DrawModelWiresEx(cubeModel, pos, axis, angle * RAD2DEG, scale,
                                 wireframe ? c : Fade(BLACK, 0.35f));
            } else {
                float r = b->shape.radius;
                Vector3 scale{r, r, r};
                if (!wireframe) DrawModelEx(sphereModel, pos, axis, angle * RAD2DEG, scale, c);
                DrawModelWiresEx(sphereModel, pos, axis, angle * RAD2DEG, scale,
                                 wireframe ? c : Fade(BLACK, 0.25f));
            }
        }

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
};

// ---------------------------------------------------------------------------
int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "raylib 3D Physics Engine");
    SetTargetFPS(60);

    Demo demo;
    demo.cubeModel   = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    demo.sphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 14, 20));
    demo.hemiModel   = LoadModelFromMesh(GenMeshHemiSphere(1.0f, 10, 20));
    demo.buildScene(0);

    // --- カメラ（球面座標で手動制御） ---
    float   camYaw = 0.9f, camPitch = 0.28f;
    float   camDist   = demo.camDistance;
    Vector3 camTarget{0, demo.camTargetY, 0};

    Camera3D camera{};
    camera.up         = Vector3{0, 1, 0};
    camera.fovy       = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    const float FIXED_DT = 1.0f / 120.0f;
    float accumulator = 0.0f;

    while (!WindowShouldClose()) {
        // ------------------------------------------------------------------
        // 入力
        // ------------------------------------------------------------------
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            camYaw   -= d.x * 0.005f;
            camPitch += d.y * 0.005f;
            camPitch  = Clamp(camPitch, -1.45f, 1.45f);
        }
        camDist = Clamp(camDist - GetMouseWheelMove() * 1.8f, 4.0f, 90.0f);

        camera.target   = camTarget;
        camera.position = Vector3{
            camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
            camTarget.y + camDist * sinf(camPitch),
            camTarget.z + camDist * cosf(camPitch) * cosf(camYaw)};

        if (IsKeyPressed(KEY_SPACE)) demo.shoot(camera);
        if (IsKeyPressed(KEY_R)) {
            demo.buildScene(demo.sceneIndex);
            camTarget.y = demo.camTargetY; camDist = demo.camDistance;
        }
        if (IsKeyPressed(KEY_P))     demo.paused = !demo.paused;
        if (IsKeyPressed(KEY_C))     demo.showContacts = !demo.showContacts;
        if (IsKeyPressed(KEY_W))     demo.wireframe = !demo.wireframe;
        if (IsKeyPressed(KEY_G))     demo.world.enableGyroscopic = !demo.world.enableGyroscopic;
        if (IsKeyPressed(KEY_Z))
            demo.world.gravity = (demo.world.gravity.y != 0.0f) ? Vec3{0, 0, 0} : Vec3{0, -9.81f, 0};
        if (IsKeyPressed(KEY_T))     demo.tapGrains();
        if (IsKeyPressed(KEY_H))     demo.showHud = !demo.showHud;
        if (IsKeyPressed(KEY_M) && demo.sceneIndex == 6) {
            demo.sandLayout = (demo.sandLayout + 1) % 3;
            demo.buildScene(6);
        }
        for (int k = 0; k < 7; ++k)
            if (IsKeyPressed(KEY_ONE + k)) {
                demo.buildScene(k);
                camTarget.y = demo.camTargetY; camDist = demo.camDistance;
            }

        // ------------------------------------------------------------------
        // 物理更新（固定タイムステップ）
        //   描画フレームレートに関係なく、常に同じ dt で積分する。
        // ------------------------------------------------------------------
        bool stepOnce = IsKeyPressed(KEY_N);
        double t0 = GetTime();
        if (!demo.paused) {
            accumulator += GetFrameTime();
            if (accumulator > 0.25f) accumulator = 0.25f;   // スパイラル・オブ・デス回避
            while (accumulator >= FIXED_DT) {
                demo.world.step(FIXED_DT);
                accumulator -= FIXED_DT;
            }
        } else if (stepOnce) {
            demo.world.step(FIXED_DT);
        }
        demo.stepMs = demo.stepMs * 0.9 + (GetTime() - t0) * 1000.0 * 0.1;

        // ------------------------------------------------------------------
        // 描画
        // ------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{28, 30, 38, 255});

        BeginMode3D(camera);
            demo.draw();
            // 地面の上面 (y=0) と重なると Z ファイティングするので少し持ち上げる
            rlPushMatrix();
            rlTranslatef(0.0f, 0.01f, 0.0f);
            DrawGrid(60, 1.0f);
            rlPopMatrix();
        EndMode3D();
        demo.drawArcLabels(camera);

        static const char* sceneNames[] = {
            "1: Box pyramid", "2: Twisted tower", "3: Spheres in a bin", "4: Ramp + dominoes",
            "5: Magnet balls", "6: Chain or ring?", "7: Iron sand"};

        if (demo.showHud) {
            const bool magScene = !demo.world.magnets().bodies().empty();
            DrawRectangle(10, 10, magScene ? 520 : 330, magScene ? 262 : 172, Fade(BLACK, 0.55f));
            DrawText(TextFormat("%s", sceneNames[demo.sceneIndex]), 22, 20, 20, RAYWHITE);
            DrawText(TextFormat("bodies %d   contacts %d", demo.world.bodyCount(),
                                demo.world.contactCount()), 22, 46, 16, LIGHTGRAY);
            DrawText(TextFormat("physics %.2f ms   %d FPS", demo.stepMs, GetFPS()), 22, 66, 16, LIGHTGRAY);
            DrawText(TextFormat("gyroscopic %s   %s",
                                demo.world.enableGyroscopic ? "ON" : "OFF",
                                demo.paused ? "[PAUSED]" : ""), 22, 86, 16, LIGHTGRAY);
            DrawText("SPACE shoot   1-7 scene   R reset", 22, 112, 16, GRAY);
            DrawText("P pause   N step   C contacts", 22, 132, 16, GRAY);
            DrawText("W wireframe   G gyro   drag/wheel camera", 22, 152, 16, GRAY);
            if (magScene) demo.drawMagnetHud(22, 180);
        }

        EndDrawing();
    }

    UnloadModel(demo.cubeModel);
    UnloadModel(demo.sphereModel);
    UnloadModel(demo.hemiModel);
    CloseWindow();
    return 0;
}
