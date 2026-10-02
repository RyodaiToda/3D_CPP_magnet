// ---------------------------------------------------------------------------
// flow_stages.cpp : MAGNET FLOW のステージ（flow_stages.h）
//   座標は tests/flow_test.cpp のボット（切り替える時刻を振って、解けるか・成功する
//   時刻の幅を測る）で詰めた。
// ---------------------------------------------------------------------------
#include "flow/flow_stages.h"

#include <cmath>

namespace flow {

namespace {

// 01 磁気峡谷の M2
const Vec3  CANYON_M2{8.0f, 11.0f, 0};
const Vec3  CANYON_GOAL{30.0f, 1.0f, 0};   // ゴールの台（x と上面の高さ）
constexpr float CANYON_M2_Q    = 0.6f;

// 04 磁気工場
constexpr float FACTORY_PAD_Q    = 0.5f;   // 扉の裏の板の強さ
constexpr float FACTORY_PAD_H    = 5.0f;   // 板の高さ（倒れると橋の付け根からの距離）
constexpr float FACTORY_PAD_TILT = 0.0f;   // 板の上向きの傾き（倒れると前向きの傾き）
constexpr float FACTORY_GOAL_Y = 2.0f;   // ゴールの台の上面

// 軌道コア（02 軌道庭園）
constexpr float CORE_R     = 2.2f;
constexpr float CORE_PUSH  = 5.5f;
constexpr float CORE_LOCK  = 12.0f;
constexpr float CORE_REACH = 5.5f;
constexpr float CORE_REPEL = 0.35f;

// 半径 r の磁石の、物どうしの双極子の強さ（可動磁石と同じ密度として Gamma から決める）
float worldMoment(float r) {
    const float mass = 2.0f * (r / 0.6f) * (r / 0.6f) * (r / 0.6f);
    return momentForGamma(WORLD_GAMMA, mass, 2.0f * r);
}

Quat rotY(const Vec3& dir) {   // +y を dir に向ける
    const Vec3  d = normalize(dir);
    const float c = d.y;
    if (c > 0.9999f)  return Quat{};
    if (c < -0.9999f) return Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
    return Quat::fromAxisAngle(cross(Vec3{0, 1, 0}, d), std::acos(clampf(c, -1.0f, 1.0f)));
}

// 壁に埋めた磁石の板（面の向き normal、その面に出ている極 pole = +1: N / -1: S）。
// worldScale = 0 ならプレイヤーにだけ効く（可動磁石や鉄を引かない）
RigidBody* wallMagnet(FlowWorld& fw, const Vec3& pos, const Vec3& normal, float halfSize, int pole,
                      float worldScale = 1.0f, float q = MAGNET_Q) {
    const float t = 0.3f;
    return fw.addFixedMagnetBox(pos, {halfSize, t, halfSize}, rotY(normal * (float)pole), q,
                                worldScale * worldMoment(halfSize));
}

// ---------------------------------------------------------------------------
// S0: 練習部屋（ゴールなし）
// ---------------------------------------------------------------------------
void buildRoom(FlowWorld& fw) {
    const float W = 15.0f, H = 12.0f;
    fw.addSolid({0, -0.5f, 0}, {W, 0.5f, W});                  // 床
    fw.addSolid({0, H + 0.5f, 0}, {W, 0.5f, W});               // 天井
    fw.addSolid({ W + 0.5f, 0.5f * H, 0}, {0.5f, 0.5f * H, W});
    fw.addSolid({-W - 0.5f, 0.5f * H, 0}, {0.5f, 0.5f * H, W});
    fw.addSolid({0, 0.5f * H,  W + 0.5f}, {W, 0.5f * H, 0.5f});
    fw.addSolid({0, 0.5f * H, -W - 0.5f}, {W, 0.5f * H, 0.5f});

    wallMagnet(fw, {-W + 0.3f, 5.0f, 0}, {1, 0, 0}, 1.5f, +1);    // 左の壁: N
    wallMagnet(fw, { W - 0.3f, 5.0f, 0}, {-1, 0, 0}, 1.5f, -1);   // 右の壁: S
    wallMagnet(fw, {0, 5.0f, -W + 0.3f}, {0, 0, 1}, 1.5f, -1);    // 奥の壁: S
    wallMagnet(fw, {0, H - 0.3f, 0}, {0, -1, 0}, 1.5f, -1);       // 天井: S
    fw.addFixedMagnetSphere({-7, 6, -7}, 1.0f, {0, 1, 0}, MAGNET_Q, worldMoment(1.0f));   // 空中
    fw.addFixedMagnetSphere({ 7, 4,  7}, 1.0f, {0, -1, 0}, MAGNET_Q, worldMoment(1.0f));

    fw.addMovableMagnetBox({5, 0.4f, -5}, 0.4f, {0, 0, 1}, 1.5f, MAGNET_Q, worldMoment(0.5f));
    fw.addIronSphere({-4, 0.4f, 4}, 0.4f, 1.0f);
    fw.addIronSphere({-3, 0.4f, 5}, 0.4f, 1.0f);
    fw.addIronBox({0, 0.4f, 8}, {0.8f, 0.4f, 0.4f}, 2.0f);

    fw.startPos = {0, 3, 4};
    fw.viewDir  = {0, -0.35f, -1};
    fw.killY    = -5.0f;
}

// ---------------------------------------------------------------------------
// P1: Switch
//   発射台から A へ。A は近づくと S 極、通り過ぎると N 極がこちらを向くので、N のままなら
//   引かれてから押し出される（吸着して加速）。下の B は S 極が上を向いていて、N のままだと
//   引き込まれる。B の真上あたりで S に切り替えると、B に押し上げられてゴールの台へ届く。
//   成功する切替の時刻の幅は約 0.29 s（flow_test FL5）
// ---------------------------------------------------------------------------
void buildSwitch(FlowWorld& fw) {
    fw.addSolid({0, -0.5f, 0}, {1.5f, 0.5f, 1.5f});                       // 発射台
    fw.addFixedMagnetSphere({8, 4.5f, 0}, 1.0f, {0.8f, 0.6f, 0}, MAGNET_Q, worldMoment(1.0f));        // A
    fw.addFixedMagnetSphere({18, -4.0f, 0}, 1.0f, {-0.5f, -0.866f, 0}, MAGNET_Q, worldMoment(1.0f));  // B
    fw.addSolid({24, -0.5f, 0}, {3.0f, 0.5f, 3.0f});                      // ゴールの台
    fw.setGoal({21, 0.0f, -3}, {27, 4.0f, 3});
    fw.startPos       = {0, 0.5f, 0};
    fw.launchVelocity = {7, 5, 0};
    fw.viewDir        = {1, -0.35f, -0.65f};
    fw.killY          = -8.0f;
}

// ---------------------------------------------------------------------------
// P2: Timing
//   高い台から奥（-z）へ飛び出し、下の縦穴の磁石 C（S 極が上寄り）に引き込まれる。
//   待ってから S に切り替えると C に押し出されて、奥の高い台へ届く。早すぎると勢いが足りずに
//   落ち、遅すぎると C に当たるか、飛び出す向きがずれて上の張り出しに当たる。
//   成功する切替の時刻の幅は約 0.12 s（P1 より狭い）
// ---------------------------------------------------------------------------
void buildTiming(FlowWorld& fw) {
    fw.addSolid({0, 5.5f, 1.0f}, {1.5f, 0.5f, 1.5f});                     // 高い開始の台
    const float a = 150.0f * PHYS_PI / 180.0f;
    fw.addFixedMagnetSphere({0, -2.0f, -8.0f}, 1.2f, {0, std::cos(a), -std::sin(a)}, MAGNET_Q,
                            worldMoment(1.2f));                              // C
    fw.addSolid({0, 8.0f, -12.0f}, {3.0f, 0.5f, 4.0f});                   // 張り出し
    fw.addSolid({0, 2.5f, -14.0f}, {3.0f, 0.5f, 3.0f});                   // ゴールの台
    fw.setGoal({-3, 3.0f, -17.0f}, {3, 7.0f, -11.0f});
    fw.startPos       = {0, 6.5f, 1.0f};
    fw.launchVelocity = {0, 2, -4};
    fw.viewDir        = {0.7f, -0.35f, -1};
    fw.killY          = -12.0f;
}

// ---------------------------------------------------------------------------
// P3: Chain
//   1. プレイヤーは床の磁石 H1（S 極が上）の上から始まる。すぐ後ろ（-x）の壁の向こうの溝に
//      可動磁石 A（S 極がこちら）があり、N のままだと壁に引き寄せられる
//   2. S に切り替えると、A を後ろへ弾き飛ばし、自分は H1 と A に押されて前へ飛ぶ。
//      A は溝を滑って鉄球に吸い付き、2 つで B まで滑って B を引き寄せる。
//      鉄球か B が判定の箱（鉄球と B の間）に入るとゲートが開く
//   3. プレイヤーは斜めの磁石 H2（N 極がこちら）に引かれて吸い付き、待つ（ゲートは切り替えてから約 1 s で
//      開き始め、1.2 s かけて開く）
//   4. ゲートが開いたら N に切り替える。H2 に押し出され、ゲートの向こうの G（S 極がこちら）に
//      引かれて、穴を越えてゴールへ。開く前に切り替えると扉に当たって穴に落ちる
// ---------------------------------------------------------------------------
void buildChain(FlowWorld& fw) {
    fw.addSolid({-1.0f, -0.5f, 0}, {11.0f, 0.5f, 4.0f});                 // 床（x -12〜10）
    // H1（S 極が上。前に傾けて、弱め。プレイヤーにだけ効く）
    const Vec3 n1 = normalize(Vec3{0.6f, 1.0f, 0});
    wallMagnet(fw, {0, 0.3f, 0}, n1, 1.0f, -1, 0.0f, 0.15f);
    // 溝（z = 0、-x 向き）。床はすべりやすい。H1 側の端は止め
    fw.addSolid({-6.3f, 0.01f, 0}, {4.8f, 0.01f, 0.55f}, Quat{}, 0.1f, 0.0f);
    fw.addSolid({-6.3f, 0.3f, -0.75f}, {4.8f, 0.3f, 0.2f}, Quat{}, 0.0f, 0.0f);
    fw.addSolid({-6.3f, 0.3f,  0.75f}, {4.8f, 0.3f, 0.2f}, Quat{}, 0.0f, 0.0f);
    // H1 と溝の間の壁（プレイヤーは越えられない。磁力は壁を通る）
    fw.addSolid({-1.45f, 0.8f, 0}, {0.15f, 0.8f, 1.2f}, Quat{}, 0.3f, 0.0f);
    fw.addMovableMagnetBox({-2.2f, 0.42f, 0}, 0.4f, {-1, 0, 0}, 1.5f, MAGNET_Q, worldMoment(0.5f));   // A
    fw.addIronSphere({-6.0f, 0.42f, 0}, 0.4f, 1.0f, 0.3f);                                              // 鉄球
    fw.addMovableMagnetBox({-9.8f, 0.42f, 0}, 0.4f, {-1, 0, 0}, 1.5f, MAGNET_Q, worldMoment(0.5f));   // B
    wallMagnet(fw, {5.5f, 0.5f, 0}, normalize(Vec3{0.4f, 1.0f, 0}), 1.0f, +1, 0.0f);   // H2（N 極がこちら）
    // 穴（x 10〜16）の上の壁とゲート（x = 13）
    fw.addSolid({13, 3.0f,  5.0f}, {0.5f, 9.0f, 3.5f});
    fw.addSolid({13, 3.0f, -5.0f}, {0.5f, 9.0f, 3.5f});
    fw.addSolid({13, 9.0f, 0}, {0.5f, 3.0f, 1.5f});
    fw.addGate({13, 3.0f, 0}, {0.5f, 3.0f, 1.5f}, {0, -9.0f, 0}, {-9.3f, 0.0f, -0.6f}, {-7.0f, 1.0f, 0.6f})
        .openTime = 1.2f;
    // ゴールの台と、引き寄せる磁石 G（S 極がこちら）
    fw.addSolid({19, -0.5f, 0}, {3.0f, 0.5f, 3.0f});
    fw.addFixedMagnetSphere({19, 1.2f, 0}, 1.0f, {1, 0, 0}, MAGNET_Q, 0.0f);
    fw.setGoal({16.0f, 0.0f, -3}, {22, 5.0f, 3});
    fw.startPos       = Vec3{0, 0.3f, 0} + n1 * 0.81f;   // H1 の上
    fw.launchVelocity = {0, 0, 0};
    fw.viewDir        = {1, -0.4f, -0.35f};
    fw.killY          = -6.0f;
}


// ---------------------------------------------------------------------------
// 01 磁気峡谷（仕様 magnetic_flow_gimmick_stage_spec.md の Stage 01）: 吸着・反発・極性切替・上下の移動
//   1. 開始すると、峡谷の向こうの台の M1（S 極が上）に吸われて着地する（入力は開始だけ）
//   2. S に切り替えると、M1 に真上へ打ち上げられ、後ろの柱の M2（N 極が前を向く）に引き寄せられる
//   3. M2 に近づいていく途中で N に切り替えると、M2 に前へ押し出されて、ゴールの台の M3（S）へ。
//      早すぎると押し出す力が弱くて届かず、遅すぎると M2 に吸い付く
// ---------------------------------------------------------------------------
void buildCanyon(FlowWorld& fw) {
    // 峡谷の両側の崖（見た目の手がかり。z = ±8、上面 y = 2）
    fw.addSolid({15, -7.0f,  8.5f}, {19.0f, 9.0f, 1.0f});
    fw.addSolid({15, -7.0f, -8.5f}, {19.0f, 9.0f, 1.0f});
    fw.addSolid({0, -0.5f, 0}, {2.0f, 0.5f, 2.5f});                        // 開始の台
    fw.addSolid({10, -5.5f, 0}, {3.0f, 4.5f, 2.5f});                       // M1 の台（x 7〜13、上面 y = -1）
    wallMagnet(fw, {9.5f, -1.2f, 0}, {0, 1, 0}, 2.3f, -1, 0.0f, 0.35f);    // M1: S 極が上
    // M1 の後ろの高い柱と M2（N 極が前を向く）
    fw.addSolid(CANYON_M2 - Vec3{0.9f, 0, 0}, {0.6f, 3.0f, 2.5f});
    wallMagnet(fw, CANYON_M2, {1, 0, 0}, 1.8f, +1, 0.0f, CANYON_M2_Q);
    const float gx = CANYON_GOAL.x, gy = CANYON_GOAL.y;
    fw.addSolid({gx, 0.5f * (gy - 12.0f), 0}, {3.5f, 0.5f * (gy + 12.0f), 3.0f});   // ゴールの台（上面 y = gy）
    wallMagnet(fw, {gx, gy, 0}, {0, 1, 0}, 1.5f, -1, 0.0f, 0.6f);           // M3: S 極が上（N を引く）
    fw.setGoal({gx - 3.5f, gy, -3}, {gx + 3.5f, gy + 5.0f, 3});
    fw.startPos       = {0, 0.5f, 0};
    fw.launchVelocity = {6.5f, 4.5f, 0};
    fw.viewDir        = {1, -0.3f, -0.55f};
    fw.killY          = -12.0f;
}

// ---------------------------------------------------------------------------
// 02 軌道庭園（Stage 02）: 磁気軌道。大きな軌道コアを輪に並べる（上から見て時計回り、少しずつ低く）
//          M2            （北 = -z）
//        /    \
//      M1      M3
//        \    /
//          P
//          |
//         Goal           （南 = +z、いちばん低い）
//   P から M1 に吸われて周回し、速さを稼いで、M2 の向きで放す（S に切り替える）。
//   M2（N）→ M3（S）と続けてつなぎ、M3 から南のゴールへ。周回しすぎると振り飛ばされる。
//   コアの極は交互（S, N, S）なので、1 回の切替で「放す」と「次に吸われる」が同時に起きる
// ---------------------------------------------------------------------------
// 上から見て時計回りの軌道コア。回転面を next（次の目標の水平の向き）の側へ tiltDeg だけ上げる
// （その向きに放すと少し上向きに飛び出し、次のコアへ届きやすい）
Core& orbitCore(FlowWorld& fw, const Vec3& pos, int pole, const Vec3& next, float tiltDeg) {
    const Vec3  D = normalize(Vec3{next.x, 0, next.z});
    const float b = tiltDeg * PHYS_PI / 180.0f;
    Core& c = fw.addCore(pos, CORE_R, pole, D * std::sin(b) - Vec3{0, 1, 0} * std::cos(b));
    c.push  = CORE_PUSH;
    c.lock  = CORE_LOCK;
    c.reach = CORE_REACH;
    c.repel = CORE_REPEL;
    return c;
}

void buildOrbit(FlowWorld& fw) {
    const Vec3 m1{-10, 9, 0}, m2{2, 13, -14}, m3{10, 8, 0}, goal{9, -1, 12};
    orbitCore(fw, m1, -1, m2 - m1, 18.0f);    // M1（S）
    orbitCore(fw, m2, +1, m3 - m2, 10.0f);    // M2（N）。M1 より高いので、速さを稼がないと届かない
    orbitCore(fw, m3, -1, goal - m3, 0.0f);   // M3（S）
    fw.addSolid({-2, 8.5f, 5.5f}, {1.5f, 0.5f, 1.5f});                     // 開始の台 P
    fw.addSolid({0, -0.15f, 1.5f}, {2.0f, 9.85f, 2.0f});                     // 輪の中の柱（輪を横切る近道をふさぐ）
    // ゴール: M3 の南の低い台。西と南の壁（M1 の側からの近道を受け止めない）
    const float gx = goal.x, gy = goal.y, gz = goal.z, G = 5.0f;
    fw.addSolid({gx, gy - 0.5f, gz}, {G, 0.5f, G});                        // 床
    fw.addSolid({gx, gy + 2.0f, gz + G + 0.5f}, {G, 2.5f, 0.5f});          // 南
    fw.addSolid({gx - G - 0.5f, gy + 2.0f, gz}, {0.5f, 2.5f, G + 1.0f});   // 西
    wallMagnet(fw, {gx, gy, gz}, {0, 1, 0}, 2.0f, +1, 0.0f, 1.0f);         // N 極が上（S を引く）
    fw.setGoal({gx - G, gy, gz - G}, {gx + G, gy + 4.0f, gz + G});
    fw.startPos       = {-2, 9.5f, 5.5};
    fw.launchVelocity = {-6.0f, 2.0f, -4.0f};
    fw.viewDir        = {0, -1.0f, -0.85f};
    fw.overview       = true;
    fw.viewCenter     = {0, 4.0f, -1.0f};
    fw.viewDist       = 40.0f;
    fw.killY          = -10.0f;
}

// ---------------------------------------------------------------------------
// 04 磁気工場（Stage 04）: 磁気ドミノ。Player → 可動磁石 A → 鉄球 B → 可動磁石 C → 回る扉 → ゴール
//   1. プレイヤーは床の磁石 H1（S 極が上）の上から始まる。壁の向こうの溝に A（S 極がこちら）
//   2. S に切り替えると A を溝へ弾き飛ばし、自分は前の H2（N 極がこちら）へ飛んで吸い付く
//   3. A は鉄球 B に吸い付き（B は A に磁化される）、2 つで C を引き寄せる。B か C が判定の箱に
//      入ると、出口をふさぐ扉が前に倒れて、穴の上の橋になる
//   4. 扉の裏の磁石の板（S）は、閉じているときはこちらを向いて、H2 で待つプレイヤー（S）を押し返す。
//      倒れると上を向いて橋の上の台になる。倒れてから N に切り替えると、H2 に押し出されて橋の台に
//      吸い付く。もう一度 S に切り替えると、台に打ち上げられて、G（N）に引かれて高いゴールの台へ
// ---------------------------------------------------------------------------
void buildFactory(FlowWorld& fw) {
    fw.addSolid({-1.0f, -0.5f, 0}, {11.5f, 0.5f, 5.0f});                   // 床（x -12.5〜10.5）
    // H1（S 極が上。前に傾けて、弱め。プレイヤーにだけ効く）
    const Vec3 n1 = normalize(Vec3{0.6f, 1.0f, 0});
    wallMagnet(fw, {0, 0.3f, 0}, n1, 1.0f, -1, 0.0f, 0.15f);
    // 溝（-x 向き）。床はすべりやすい
    fw.addSolid({-6.3f, 0.01f, 0}, {4.8f, 0.01f, 0.55f}, Quat{}, 0.1f, 0.0f);
    fw.addSolid({-6.3f, 0.3f, -0.75f}, {4.8f, 0.3f, 0.2f}, Quat{}, 0.0f, 0.0f);
    fw.addSolid({-6.3f, 0.3f,  0.75f}, {4.8f, 0.3f, 0.2f}, Quat{}, 0.0f, 0.0f);
    fw.addSolid({-1.45f, 0.8f, 0}, {0.15f, 0.8f, 1.2f}, Quat{}, 0.3f, 0.0f);   // H1 と溝の間の壁
    fw.addMovableMagnetBox({-2.2f, 0.42f, 0}, 0.4f, {-1, 0, 0}, 1.5f, MAGNET_Q, worldMoment(0.5f));   // A
    fw.addIronSphere({-6.0f, 0.42f, 0}, 0.4f, 1.0f, 0.3f);                                              // 鉄球 B
    fw.addMovableMagnetBox({-9.8f, 0.42f, 0}, 0.4f, {-1, 0, 0}, 1.5f, MAGNET_Q, worldMoment(0.5f));   // C
    wallMagnet(fw, {5.5f, 0.5f, 0}, normalize(Vec3{0.4f, 1.0f, 0}), 1.0f, +1, 0.0f);   // H2（N 極がこちら）

    // 出口の壁（x 12〜12.5）と、戸口（z -1.8〜1.8、y 0〜7）をふさぐ回る扉
    const float WX = 12.25f;
    fw.addSolid({WX, 8.0f,  4.0f}, {0.25f, 8.0f, 2.2f});
    fw.addSolid({WX, 8.0f, -4.0f}, {0.25f, 8.0f, 2.2f});
    fw.addSolid({WX, 11.5f, 0}, {0.25f, 4.5f, 1.8f});
    // 扉: 下の辺（x = 12.5, y = 0）のまわりに -90 度（前へ倒れる）。判定の箱は鉄球 B と C の間
    Gate& g = fw.addHingedGate({WX, 3.5f, 0}, {0.25f, 3.5f, 1.8f}, Quat{}, {12.5f, 0.0f, 0}, {0, 0, 1},
                               -0.5f * PHYS_PI, {-9.3f, 0.0f, -0.6f}, {-7.0f, 1.0f, 0.6f});
    g.openTime = 1.2f;
    // 扉の裏の磁石の板（S がこちら = -x。H2 で待つ S のプレイヤーを押し返す。倒れると上を向いて橋の上の台になる）
    //   扉の上のほうに付け、少し上に傾ける（倒れると橋の先で前上を向く）
    fw.attachToGate(fw.gates.back(), wallMagnet(fw, {WX - 0.45f, FACTORY_PAD_H, 0}, normalize(Vec3{-1.0f, FACTORY_PAD_TILT, 0}),
                                                1.3f, -1, 0.0f, FACTORY_PAD_Q));
    // 穴（x 12.5〜20）の向こうの高いゴールの台と、引き寄せる磁石 G（N 極がこちら。S を引く）
    fw.addSolid({23.5f, FACTORY_GOAL_Y - 1.5f, 0}, {3.5f, 1.5f, 3.5f});
    fw.addFixedMagnetSphere({24.0f, FACTORY_GOAL_Y + 1.2f, 0}, 1.0f, {-1, 0, 0}, MAGNET_Q, 0.0f);
    fw.setGoal({20.0f, FACTORY_GOAL_Y, -3.5f}, {27.0f, FACTORY_GOAL_Y + 5.0f, 3.5f});
    fw.startPos       = Vec3{0, 0.3f, 0} + n1 * 0.81f;   // H1 の上
    fw.viewDir        = {1, -0.4f, -0.45f};
    fw.killY          = -6.0f;
}
} // namespace

const char* stageName(int index) {
    static const char* names[STAGE_COUNT] = {"S0  Practice room", "P1  Switch", "P2  Timing", "P3  Chain",
                                             "01  Magnetic Canyon", "02  Orbit Garden", "04  Magnetic Factory",
                                             "GRAND TOUR", "L1  Cyclotron", "L2  Gauss Rings", "L3  Resonance Swing",
                                             "L4  Ball Chain"};
    return names[index];
}

const char* stageKey(int index) {
    static const char* keys[STAGE_COUNT] = {"S0", "P1", "P2", "P3", "G01", "G02", "G04", "GRAND",
                                            "CYCLO", "GAUSS", "RESON", "ROPE"};
    return keys[index];
}

const char* stageHint(int index) {
    static const char* hints[STAGE_COUNT] = {
        "free play: feel the pull and the push",
        "get pulled past A, flip near B and get pushed up to the goal",
        "wait under C, then flip: too early falls, too late hits the ceiling",
        "move magnet A: its pull spreads to the ball and to B, and the gate opens",
        "get pulled to M1, flip to fly up, flip again right as you reach M2",
        "orbit a core to build speed, flip to let go toward the next one",
        "push magnet A down the line: A -> iron B -> magnet C turns the gate",
        "every gimmick in one big map: 9 zones, a checkpoint at each launch pad",
        "flip just before each gap so the half you enter pulls you in: same beat, wider spiral, out the port",
        "each ring pulls you in: flip right as you pass through it, and it shoves you on to the next",
        "pull the bob as it comes, push as it passes: grow the swing until it smashes the wall, then launch",
        "grab the tip of the chain, swing, flip to let go and fly to the next chain"};
    return hints[index];
}

void loadStage(FlowWorld& fw, int index) {
    switch (index) {
    case 0:  fw.builder = [](FlowWorld& w) { buildRoom(w);   w.finishBuild(); }; break;
    case 1:  fw.builder = [](FlowWorld& w) { buildSwitch(w); w.finishBuild(); }; break;
    case 2:  fw.builder = [](FlowWorld& w) { buildTiming(w); w.finishBuild(); }; break;
    case 3:  fw.builder = [](FlowWorld& w) { buildChain(w);  w.finishBuild(); }; break;
    case 4:  fw.builder = [](FlowWorld& w) { buildCanyon(w); w.finishBuild(); }; break;
    case 5:  fw.builder = [](FlowWorld& w) { buildOrbit(w);  w.finishBuild(); }; break;
    case 6:  fw.builder = [](FlowWorld& w) { buildFactory(w); w.finishBuild(); }; break;
    case 8:  fw.builder = [](FlowWorld& w) { buildCyclotron(w);  w.finishBuild(); }; break;
    case 9:  fw.builder = [](FlowWorld& w) { buildGaussRings(w); w.finishBuild(); }; break;
    case 10: fw.builder = [](FlowWorld& w) { buildResonance(w);  w.finishBuild(); }; break;
    case 11: fw.builder = [](FlowWorld& w) { buildBallChain(w);  w.finishBuild(); }; break;
    default: fw.builder = [](FlowWorld& w) { buildGrand(w); w.finishBuild(); }; break;
    }
    fw.retries = 0;
    fw.prevTrail.clear();
    fw.prevMarks.clear();
    fw.reset();
    fw.builder(fw);
}

} // namespace flow
