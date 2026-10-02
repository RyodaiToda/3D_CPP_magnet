// ---------------------------------------------------------------------------
// flow_grand.cpp : 大きなマップ「GRAND TOUR」（magnetic_flow_gimmick_stage_spec.md の全ギミック）
//
//   9 の区間をつなぐ。区間の始めは発射台（チェックポイント）で、前の区間はそこへ着けば抜けたことになる。
//     1 磁気峡谷     吸着・反発・極性切替・上下の移動（Stage 01）
//     2 振り子の谷   磁気振り子（G6）
//     3 軌道庭園     磁気軌道（G1。Stage 02）
//     4 レンズ回廊   磁気レンズ（G2。Stage 03）
//     5 磁気工場     磁気ドミノ（G3。Stage 04）
//     6 鋼の飛び石   磁化連鎖（G5）
//     7 磁気嵐       動的磁場（G4。Stage 05）
//     8 磁針の橋     磁気トルク
//     9 磁気コア     全部（Stage 06）
//   区間はそれぞれ自分の座標（始めの発射台の上面の中心が原点、+x が進む向き）で組み、Frame で
//   マップに置く。区間の出口（次の区間の原点）と曲がる向きを表に持ち、順に Frame をつなぐ。
//   座標と切替のタイミングは tests/flow_test.cpp のボット（--zprobe）で詰めた
// ---------------------------------------------------------------------------
#include "flow/flow_stages.h"

#include <algorithm>
#include <cmath>

namespace flow {

int grandOnlyZone = -1;

namespace {

// 区間の座標 → マップの座標（平行移動と、上から見て 90 度ずつの回転。回転は厳密）
struct Frame {
    Vec3 o{0, 0, 0};
    int  turn = 0;   // +y まわりに 90 度 x turn（+x → -z）
    Vec3 V(const Vec3& v) const {
        switch (turn & 3) {
        case 1:  return Vec3{v.z, v.y, -v.x};
        case 2:  return Vec3{-v.x, v.y, -v.z};
        case 3:  return Vec3{-v.z, v.y, v.x};
        default: return v;
        }
    }
    Vec3 P(const Vec3& p) const { return o + V(p); }
    Vec3 HE(const Vec3& he) const { return (turn & 1) ? Vec3{he.z, he.y, he.x} : he; }
    Quat Q(const Quat& q) const {
        return (turn & 3) ? Quat::fromAxisAngle({0, 1, 0}, 0.5f * PHYS_PI * (float)(turn & 3)) * q : q;
    }
    void box(const Vec3& lo, const Vec3& hi, Vec3& wlo, Vec3& whi) const {
        const Vec3 a = P(lo), b = P(hi);
        wlo = Vec3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
        whi = Vec3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
    }
};

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

// --- 区間の座標で置く部品 ---
RigidBody* solid(FlowWorld& fw, const Frame& F, const Vec3& pos, const Vec3& he, float friction = 0.6f,
                 float restitution = 0.1f) {
    return fw.addSolid(F.P(pos), F.HE(he), Quat{}, friction, restitution);
}

// 板の磁石（面の向き normal、その面に出ている極 pole）。worldScale = 0 ならプレイヤーにだけ効く
RigidBody* plate(FlowWorld& fw, const Frame& F, const Vec3& pos, const Vec3& normal, float halfSize, int pole,
                 float worldScale = 0.0f, float q = MAGNET_Q) {
    return fw.addFixedMagnetBox(F.P(pos), {halfSize, 0.3f, halfSize}, rotY(F.V(normal) * (float)pole), q,
                                worldScale * worldMoment(halfSize));
}

RigidBody* ball(FlowWorld& fw, const Frame& F, const Vec3& pos, float r, const Vec3& north, float q = MAGNET_Q,
                float worldScale = 0.0f) {
    return fw.addFixedMagnetSphere(F.P(pos), r, F.V(north), q, worldScale * worldMoment(r));
}

// 区間の始めの発射台（上面の中心が区間の原点）。deck > 0 なら、まわりに半幅 deck の着地台を置く
// （発射台は 5 m 先から引くので、着地台のどこに着いても発射台へ引き寄せられて止まる）
void startPad(FlowWorld& fw, const Frame& F, int cp, const Vec3& launch, int pol, float deck = 0.0f) {
    if (deck > 0.0f) solid(fw, F, {0, -0.6f, 0}, {deck, 0.5f, deck}, 0.8f, 0.0f);
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({2.0f, 0.5f, 2.0f}), F.V(launch), pol, cp);
    p.reach = 5.0f;
}

// 区間（カメラの向きと落ちる高さ）。箱と向きは区間の座標
Zone& zone(FlowWorld& fw, const Frame& F, const char* name, const char* hint, const Vec3& lo, const Vec3& hi,
           float killY, const Vec3& viewDir) {
    Vec3 wlo, whi;
    F.box(lo, hi, wlo, whi);
    Zone& z = fw.addZone(name, hint, wlo, whi, F.o.y + killY);
    z.viewDir = F.V(viewDir);
    return z;
}

// ---------------------------------------------------------------------------
// 1 磁気峡谷（Stage 01 と同じ配置）
// ---------------------------------------------------------------------------
void canyon(FlowWorld& fw, const Frame& F) {
    solid(fw, F, {15, -7.0f, 8.5f}, {19.0f, 9.0f, 1.0f});    // 峡谷の両側の崖
    solid(fw, F, {15, -7.0f, -8.5f}, {19.0f, 9.0f, 1.0f});
    solid(fw, F, {10, -5.5f, 0}, {3.0f, 4.5f, 2.5f});        // M1 の台（上面 y = -1）
    plate(fw, F, {9.5f, -1.2f, 0}, {0, 1, 0}, 2.3f, -1, 0.0f, 0.35f);   // M1: S 極が上
    const Vec3 m2{8.0f, 11.0f, 0};
    solid(fw, F, m2 - Vec3{0.9f, 0, 0}, {0.6f, 3.0f, 2.5f});              // M2 の柱
    plate(fw, F, m2, {1, 0, 0}, 1.8f, +1, 0.0f, 0.6f);                    // M2: N 極が前
    solid(fw, F, {30, -5.5f, 0}, {3.5f, 6.5f, 3.0f});                     // 出口の台（上面 y = 1）
    // Stage 01 の M3（ゴールの磁石）は置かない。次の区間の発射台が引き寄せる（M3 は次の発射を乱す）
}

constexpr float PENDULUM_LEN = 5.0f;
const Vec3 PENDULUM_A1{12.0f, 4.0f, 0};
const Vec3 PENDULUM_EXIT{46.00f, 3.00f, 0};

// ---------------------------------------------------------------------------
// 2 振り子の谷（G6）: 錨 A1（N）に S で吸われて回り、放して錨 A2（S）へ、そこから出口へ
// ---------------------------------------------------------------------------
void pendulum(FlowWorld& fw, const Frame& F) {
    Swing& a1 = fw.addSwing(F.P(PENDULUM_A1), 0.8f, +1, PENDULUM_LEN, F.V({0, 0, 1}));
    a1.pump    = 8.0f;
    a1.maxHold = 8.0f;
    const Vec3 ex = PENDULUM_EXIT;                       // 出口の柱（上に次の区間の発射台）
    solid(fw, F, {ex.x, 0.5f * (ex.y - 1.0f - 16.0f), 0}, {2.5f, 0.5f * (ex.y - 1.0f + 16.0f), 2.5f});
}

// ---------------------------------------------------------------------------
// 3 軌道庭園（Stage 02 と同じ配置。開始の台 P の上面の中心を原点に移した）
//   軌道コアは上から見て時計回りに M1（S）→ M2（N）→ M3（S）。M3 から南の低い出口へ
// ---------------------------------------------------------------------------
constexpr float CORE_R = 2.2f, CORE_PUSH = 5.5f, CORE_LOCK = 12.0f, CORE_REACH = 5.5f, CORE_REPEL = 0.35f;
constexpr float ORBIT_RAMP_DEG = 8.0f;
constexpr float ORBIT_RAMP_LEN = 16.0f;
const Vec3 ORBIT_SHIFT{2.0f, -9.0f, -5.5f};   // Stage 02 の座標 → 区間の座標

Core& orbitCore(FlowWorld& fw, const Frame& F, const Vec3& pos, int pole, const Vec3& next, float tiltDeg) {
    const Vec3  D = normalize(Vec3{next.x, 0, next.z});
    const float b = tiltDeg * PHYS_PI / 180.0f;
    Core& c = fw.addCore(F.P(pos), CORE_R, pole, F.V(D * std::sin(b) - Vec3{0, 1, 0} * std::cos(b)));
    c.push  = CORE_PUSH;
    c.lock  = CORE_LOCK;
    c.reach = CORE_REACH;
    c.repel = CORE_REPEL;
    return c;
}

// 発射台へ向かって下る着地台（発射台から side の向きに from 〜 to、傾き deg）。どこに着いても発射台へ転がる
//   side: 区間の +z を 0 として、上から見て 90 度ずつ（Frame の turn と同じ向き。3 なら -x）
void ramp(FlowWorld& fw, const Frame& F, float halfWidth, float from, float to, float deg, int side = 0) {
    Frame G;
    G.o    = F.o;
    G.turn = F.turn + side;
    const float a = deg * PHYS_PI / 180.0f, len = to - from, mid = 0.5f * (from + to);
    const Quat  q = G.Q(Quat::fromAxisAngle({1, 0, 0}, -a));   // 遠い端が上がる
    fw.addSolid(G.P({0, -0.5f + std::sin(a) * (mid - from), mid}), {halfWidth, 0.5f, 0.5f * len}, q, 0.5f, 0.0f);
}

void orbitPad(FlowWorld& fw, const Frame& F, int cp) {
    ramp(fw, F, 3.0f, 2.5f, ORBIT_RAMP_LEN, ORBIT_RAMP_DEG);   // 振り子の谷から来る側（区間の +z）
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({3.0f, 0.5f, 3.0f}), F.V({-6.0f, 2.0f, -4.0f}), +1, cp);
    p.reach = 2.5f;   // 軌道コアのまわりを回っているあいだに引かない
}

void orbit(FlowWorld& fw, const Frame& F) {
    const Vec3 S = ORBIT_SHIFT;
    const Vec3 m1 = Vec3{-10, 9, 0} + S, m2 = Vec3{2, 13, -14} + S, m3 = Vec3{10, 8, 0} + S, goal = Vec3{9, -1, 12} + S;
    orbitCore(fw, F, m1, -1, m2 - m1, 18.0f);   // M1（S）
    orbitCore(fw, F, m2, +1, m3 - m2, 10.0f);   // M2（N）。M1 より高いので、速さを稼がないと届かない
    orbitCore(fw, F, m3, -1, goal - m3, 0.0f);  // M3（S）
    solid(fw, F, Vec3{0, -0.15f, 1.5f} + S, {2.0f, 9.85f, 2.0f});   // 輪の中の柱（輪を横切る近道をふさぐ）
    // 出口: M3 の南の低い床（南と西の壁）。真ん中に次の区間の発射台
    const float G = 5.0f;
    solid(fw, F, goal + Vec3{0, -0.5f, 0}, {G, 0.5f, G});
    solid(fw, F, goal + Vec3{0, 2.0f, G + 0.5f}, {G, 2.5f, 0.5f});
    solid(fw, F, goal + Vec3{-G - 0.5f, 2.0f, 0}, {0.5f, 2.5f, G + 1.0f});
    // Stage 02 のゴールの磁石（N が上）は置かない。次の区間の発射台が引き寄せる（磁石は次の発射を乱す）
}

// ---------------------------------------------------------------------------
// 4 レンズ回廊（G2 磁気レンズ。Stage 03）: 出口は正面に見えるが、真ん中の壁がふさいでいる。
//   磁石の正面ではなく、磁場の端を通って曲げる。右の B1（N がこちら）は N を左へ押し、壁の左を抜けさせる。
//   左奥の B2（S がこちら）は S を右へ押し、出口へ戻す。B1 を過ぎてから B2 に着くまでに S に切り替える
//   （早すぎると曲がりきらずに壁へ、遅すぎると B2 に引き込まれる）
// ---------------------------------------------------------------------------
const Vec3 LENS_EXIT{34.0f, -8.0f, 0};
const Vec3 LENS_LAUNCH{14.0f, 9.0f, 0};
const Vec3 LENS_B1{5.0f, 1.0f, -3.5f};
const Vec3 LENS_B2{20.0f, 2.9f, 5.0f};
constexpr float LENS_Q1 = 0.6f;
constexpr float LENS_Q2 = 0.8f;
constexpr float LENS_WALL = 3.0f;    // 真ん中の壁の半幅

void lensPad(FlowWorld& fw, const Frame& F, int cp) {
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({2.0f, 0.5f, 2.0f}), F.V(LENS_LAUNCH), +1, cp);
    p.reach = 7.5f;   // 軌道庭園の出口の床（10 x 10）のどこに着いても引き寄せる
}

void lens(FlowWorld& fw, const Frame& F) {
    solid(fw, F, {15, -15.0f, 0}, {1.0f, 25.0f, LENS_WALL});                 // 真ん中の壁
    ball(fw, F, LENS_B1, 1.0f, {0, 0, 1}, LENS_Q1);     // B1: 道の側（+z）に N
    ball(fw, F, LENS_B2, 1.0f, {0, 0, 1}, LENS_Q2);     // B2: 道の側（-z）に S
    // 方位磁針の並び（描画だけ）: 道の下に、プレイヤーの今の極で受ける力の向きを見せる
    NeedleField nf;
    nf.center = F.P({13, -6.0f, 0});   // 壁の手前と横（奥の工場の床に重ならないように）
    nf.u = F.V({1, 0, 0});
    nf.v = F.V({0, 0, 1});
    nf.nu = 11; nf.nv = 7; nf.step = 2.0f;
    fw.needles.push_back(nf);
}

// ---------------------------------------------------------------------------
// 5 磁気工場（G3 磁気ドミノ。Stage 04 と同じ配置を、入口の発射台から z = -9 にずらした）
//   1. 入口の発射台から跳んで H1（S 極が上）に乗る（N）
//   2. S に切り替えると A を溝へ弾き、自分は H2（N 極がこちら）へ飛んで吸い付く
//   3. A → 鉄球 B（A に磁化される）→ C と連鎖して、扉が前に倒れて穴の上の橋になる
//   4. 倒れてから N に切り替えると H2 に押し出されて橋の板（S が上）へ。S に切り替えると打ち上がる
// ---------------------------------------------------------------------------
const Vec3 FACTORY_SHIFT{0, 0, -9.0f};
const Vec3 FACTORY_HOP{0, 6.5f, -8.5f};
constexpr float FACTORY_GOAL_Y = 2.0f;

void factoryPad(FlowWorld& fw, const Frame& F, int cp) {
    ramp(fw, F, 3.0f, 2.5f, 17.0f, 8.0f, 3);   // レンズ回廊から来る側（区間の -x）
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({3.0f, 0.5f, 3.0f}), F.V(FACTORY_HOP), +1, cp);
    p.reach = 5.0f;
    solid(fw, F, {0, -6.0f, 0}, {2.0f, 5.0f, 2.0f});   // 発射台の柱
    solid(fw, F, {3.5f, 1.0f, 0}, {0.5f, 2.0f, 3.0f});  // 行き過ぎを受け止める壁（+x）
}

void factory(FlowWorld& fw, const Frame& F) {
    const Vec3 S = FACTORY_SHIFT;
    solid(fw, F, Vec3{-1.0f, -0.5f, 0} + S, {11.5f, 0.5f, 5.0f});                 // 床（x -12.5〜10.5）
    const Vec3 n1 = normalize(Vec3{0.6f, 1.0f, 0});
    fw.addFixedMagnetBox(F.P(Vec3{0, 0.3f, 0} + S), {1.0f, 0.3f, 1.0f}, rotY(F.V(n1 * -1.0f)), 0.15f, 0.0f);   // H1
    solid(fw, F, Vec3{-6.3f, 0.01f, 0} + S, {4.8f, 0.01f, 0.55f}, 0.1f, 0.0f);   // 溝
    solid(fw, F, Vec3{-6.3f, 0.3f, -0.75f} + S, {4.8f, 0.3f, 0.2f}, 0.0f, 0.0f);
    solid(fw, F, Vec3{-6.3f, 0.3f, 0.75f} + S, {4.8f, 0.3f, 0.2f}, 0.0f, 0.0f);
    solid(fw, F, Vec3{-1.45f, 0.8f, 0} + S, {0.15f, 0.8f, 1.2f}, 0.3f, 0.0f);    // H1 と溝の間の壁
    // H1 のゆりかご（両脇の低い壁）: 入口の発射台から跳んできたプレイヤーを H1 の真ん中に収める
    solid(fw, F, Vec3{0.1f, 0.6f, 1.25f} + S, {1.4f, 0.6f, 0.15f}, 0.2f, 0.0f);
    solid(fw, F, Vec3{0.1f, 0.6f, -1.25f} + S, {1.4f, 0.6f, 0.15f}, 0.2f, 0.0f);
    fw.addMovableMagnetBox(F.P(Vec3{-2.2f, 0.42f, 0} + S), 0.4f, F.V({-1, 0, 0}), 1.5f, MAGNET_Q, worldMoment(0.5f));   // A
    fw.addIronSphere(F.P(Vec3{-6.0f, 0.42f, 0} + S), 0.4f, 1.0f, 0.3f);                                                 // 鉄球 B
    fw.addMovableMagnetBox(F.P(Vec3{-9.8f, 0.42f, 0} + S), 0.4f, F.V({-1, 0, 0}), 1.5f, MAGNET_Q, worldMoment(0.5f));   // C
    plate(fw, F, Vec3{5.5f, 0.5f, 0} + S, normalize(Vec3{0.4f, 1.0f, 0}), 1.0f, +1, 0.0f, MAGNET_Q);                   // H2
    // 出口の壁（x 12〜12.5）と、戸口をふさぐ回る扉
    const float WX = 12.25f;
    solid(fw, F, Vec3{WX, 8.0f, 4.0f} + S, {0.25f, 8.0f, 2.2f});
    solid(fw, F, Vec3{WX, 8.0f, -4.0f} + S, {0.25f, 8.0f, 2.2f});
    solid(fw, F, Vec3{WX, 11.5f, 0} + S, {0.25f, 4.5f, 1.8f});
    Vec3 tlo, thi;
    F.box(Vec3{-9.3f, 0.0f, -0.6f} + S, Vec3{-7.0f, 1.0f, 0.6f} + S, tlo, thi);
    Gate& g = fw.addHingedGate(F.P(Vec3{WX, 3.5f, 0} + S), F.HE({0.25f, 3.5f, 1.8f}), Quat{}, F.P(Vec3{12.5f, 0.0f, 0} + S),
                               F.V({0, 0, 1}), -0.5f * PHYS_PI, tlo, thi);
    g.openTime = 1.2f;
    fw.attachToGate(fw.gates.back(), plate(fw, F, Vec3{WX - 0.45f, 5.0f, 0} + S, {-1, 0, 0}, 1.3f, -1, 0.0f, 0.5f));
    // 穴（x 12.5〜20）の向こうの高い出口の台と、引き寄せる磁石 G（N 極がこちら。S を引く）
    solid(fw, F, Vec3{23.5f, FACTORY_GOAL_Y - 1.5f, 0} + S, {3.5f, 1.5f, 3.5f});
    // 行き過ぎを受け止める垂れ壁（高いところだけ。下は次の区間の発射が通る）
    solid(fw, F, Vec3{28.0f, FACTORY_GOAL_Y + 7.5f, 0} + S, {0.5f, 3.5f, 3.5f});
}

// ---------------------------------------------------------------------------
// 6 鋼の飛び石（G5 磁化連鎖）: 宙に浮かぶ鋼の板は、どちらの極でも引く（ふつうの鉄と同じ）。
//   触れているあいだに、触れた側がプレイヤーと異極になるように磁化される（0.4 s で満ちる）。
//   溜めてから切り替えると、残った磁化に弾かれて、板の面の向きに飛ぶ。最後の板 A は、
//   磁化すると近くの鋼の球 B を引き寄せる（双極子）。B が落ちて判定の箱に入ると出口の扉が開く
// ---------------------------------------------------------------------------
constexpr float STEEL_PAD_REACH = 6.0f;
const Vec3 STEEL_LAUNCH{9.0f, 7.0f, 0};
const Vec3 STEEL_S1{10.0f, -1.0f, 0};
const Vec3 STEEL_S2{25.0f, 3.0f, 0};
const Vec3 STEEL_A{40.0f, 7.0f, 0};
const Vec3 STEEL_EXIT{55.0f, 11.0f, 0};
constexpr float STEEL_TILT = 0.6f;
constexpr float STEEL_B_DIST = 4.0f;  // B と A の中心の距離（横）     // 板の面の前への傾き（法線 (tilt, 1, 0)）

void steelPad(FlowWorld& fw, const Frame& F, int cp) {
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({2.0f, 0.5f, 2.0f}), F.V(STEEL_LAUNCH), +1, cp);
    p.reach = STEEL_PAD_REACH;   // 工場の橋の上から打ち上がったプレイヤーを引き寄せる（Stage 04 の G の代わり）
}

Steel& steelStone(FlowWorld& fw, const Frame& F, const Vec3& pos, float moment) {
    const Vec3 n = normalize(Vec3{STEEL_TILT, 1.0f, 0});
    Steel& st = fw.addSteelBox(F.P(pos), {1.5f, 0.4f, 1.5f}, 0.0f, moment, F.Q(rotY(n)));
    st.body->friction        = 1.0f;   // 吸い付いたら転がり落ちない
    st.body->rollingFriction = 2.0f;
    return st;
}

void steel(FlowWorld& fw, const Frame& F) {
    steelStone(fw, F, STEEL_S1, 0.0f);
    steelStone(fw, F, STEEL_S2, 0.0f);
    steelStone(fw, F, STEEL_A, momentForGamma(WORLD_GAMMA, 9.26f, 2.0f));
    // B: A の横（+z）の棚の上の鋼の球。A が磁化すると引かれて棚を転がり、A の横に吸い付く。
    //    そこが判定の箱（B が 0.25 s 入っていると出口の扉が開く）
    const Vec3 b = STEEL_A + Vec3{0, 0.2f, STEEL_B_DIST};
    solid(fw, F, Vec3{b.x, b.y - 0.6f, 0.5f * (1.6f + b.z - STEEL_A.z) + STEEL_A.z},
          {0.8f, 0.1f, 0.5f * (b.z - STEEL_A.z - 1.6f) + 0.6f}, 0.2f, 0.0f);
    fw.addSteelSphere(F.P(b), 0.5f, 1.0f, momentForGamma(WORLD_GAMMA, 1.0f, 1.0f), 0.2f);   // プレイヤーへはほとんど効かない
    Vec3 tlo, thi;
    F.box(STEEL_A + Vec3{-1.2f, -1.0f, 1.4f}, STEEL_A + Vec3{1.2f, 1.6f, 2.8f}, tlo, thi);
    // 出口の扉（A と出口のあいだ、上へ開く）
    const Vec3 d = STEEL_A + Vec3{6.0f, 3.0f, 0};
    fw.addGate(F.P(d), F.HE({0.3f, 4.0f, 3.0f}), {0, 8.5f, 0}, tlo, thi).openTime = 0.8f;
    solid(fw, F, STEEL_EXIT - Vec3{0, 6.0f, 0}, {2.5f, 5.0f, 2.5f});   // 出口の柱
}

// ---------------------------------------------------------------------------
// 7 磁気嵐（G4 動的磁場。Stage 05）: 区間の時計（発射台に着いてから）で動く・点滅する・回る磁石
//   1. 往復する磁石の台（S が上。N を引く）が谷を往復している。台が手前に来るのを読んで跳び乗る
//   2. 台は谷を渡って向こう端で止まる。S に切り替えると台に押し上げられる
//   3. 上の点滅する磁石 P（N がこちら。S を引く）は、台が向こう端に着くころだけ点く。
//      点いているあいだに跳べば、引き上げられて出口へ。消えていると届かない（「0.5 秒後の正解」を読む）
//   まわりの回る磁石（半回転ずつ極が入れ替わる）は、嵐の見た目（プレイヤーにはほとんど効かない）
// ---------------------------------------------------------------------------
const Vec3 STORM_LAUNCH{7.0f, 5.0f, 0};
const Vec3 STORM_SHUTTLE{9.0f, -2.0f, 0};     // 台の手前の端
const Vec3 STORM_TRAVEL{18.0f, 0, 0};
constexpr float STORM_PERIOD = 5.0f;          // 台の往復の周期（P の点滅も同じ周期）
const Vec3 STORM_P{41.0f, 14.0f, 4.5f};           // 点滅する磁石（出口の奥）
constexpr float STORM_PQ = 1.5f;
constexpr float STORM_PON = 1.4f;             // P が点いている長さ [s]
constexpr float STORM_PLEAD = 0.4f;           // 台が向こう端に着く何秒前に P が点くか
const Vec3 STORM_EXIT{38.0f, 12.0f, 0};

void stormPad(FlowWorld& fw, const Frame& F, int cp) {
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({2.0f, 0.5f, 2.0f}), F.V(STORM_LAUNCH), +1, cp);
    p.reach = 5.0f;
}

void storm(FlowWorld& fw, const Frame& F) {
    // 往復する台（S が上）。区間の時計 0 のとき向こう端にいて、手前へ戻ってくる
    RigidBody* sh = fw.addFixedMagnetBox(F.P(STORM_SHUTTLE), {2.5f, 0.4f, 2.5f}, rotY(F.V({0, -1, 0})), 0.6f, 0.0f);
    sh->friction = 1.0f;
    Animator& a = fw.animate(sh);
    a.travel       = F.V(STORM_TRAVEL);
    a.travelPeriod = STORM_PERIOD;
    a.phase        = 0.5f * STORM_PERIOD;
    // 点滅する磁石 P（下向きの面に N。台が向こう端に着く時刻 = 周期の整数倍）
    RigidBody* pm = fw.addFixedMagnetSphere(F.P(STORM_P), 1.2f, F.V({0, -1, 0}), STORM_PQ, 0.0f);
    Animator& pa = fw.animate(pm);
    pa.pulsePeriod = STORM_PERIOD;
    pa.pulseOn     = STORM_PON / STORM_PERIOD;
    pa.phase       = STORM_PLEAD;   // 点く時刻 = 周期の整数倍 - STORM_PLEAD
    // 嵐の回る磁石（半回転ずつ。プレイヤーにはほとんど効かない）
    const Vec3 deco[] = {{14, 8, -7}, {22, 12, 7}, {30, 3, -8}, {18, -8, 6}};
    for (int k = 0; k < 4; ++k) {
        RigidBody* m = fw.addFixedMagnetSphere(F.P(deco[k]), 1.0f, F.V({1, 0, 0}), 0.05f, 0.0f);
        Animator& r = fw.animate(m);
        r.spinAxis = F.V(k % 2 ? Vec3{0, 0, 1} : Vec3{0, 1, 0});
        r.spinStep = 1.0f + 0.25f * (float)k;
    }
    solid(fw, F, STORM_EXIT - Vec3{0, 6.0f, 0}, {2.5f, 5.0f, 2.5f});   // 出口の柱
}

// ---------------------------------------------------------------------------
// 8 磁針の橋（磁気トルク）: 谷の真ん中に、縦の軸のまわりに回る長い棒磁石（方位磁針。両端が N と S の磁極）。
//   プレイヤーの磁力の反作用のトルクで回り、プレイヤーと異極の端がこちらを向く。
//   発射台に座っているあいだは S（台が S にする）なので、N 端がこちらを向いて谷に橋が架かる。
//   橋になったら N で跳ぶ。手前の N 端に押され、向こうの S 端に引かれて棒の上を渡り、S 端に吸い付く。
//   S に切り替えると S 端に押し出されて出口へ。早く跳ぶと、まだ橋になっていなくて谷に落ちる
// ---------------------------------------------------------------------------
const Vec3 COMPASS_PIVOT{14.0f, -1.0f, 0};
constexpr float COMPASS_HALF = 8.0f;          // 棒の半分の長さ
constexpr float COMPASS_START_DEG = 230.0f;    // 始めの向き（進む向きから、上から見て）
constexpr float COMPASS_INERTIA = 0.24f;
constexpr float COMPASS_DAMP = 2.5f;
constexpr float COMPASS_Q = 0.01f;
const Vec3 COMPASS_LAUNCH{8.0f, 3.5f, 0};
const Vec3 COMPASS_EXIT{26.0f, -1.5f, 0};

void compassPad(FlowWorld& fw, const Frame& F, int cp) {
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({2.0f, 0.5f, 2.0f}), F.V(COMPASS_LAUNCH), -1, cp);
    p.reach = 3.0f;
    p.hold  = -1;
}

void compass(FlowWorld& fw, const Frame& F) {
    // 棒（ローカル +y が N 端。磁力は両端の磁極が受け持つ）。始めは S 端が手前の右を向く
    const float a = COMPASS_START_DEG * PHYS_PI / 180.0f;
    const Vec3  dirN{std::cos(a), 0, std::sin(a)};    // N 端の向き（区間の座標）
    // ローカル +y を dirN へ、ローカル x を上下（厚さ）に向ける（上面が平らな橋）
    const Quat base = Quat::fromAxisAngle({0, 0, -1}, 0.5f * PHYS_PI);            // y → +x、x → -y
    const Quat yaw  = Quat::fromAxisAngle({0, 1, 0}, std::atan2(-dirN.z, dirN.x));  // +x → dirN
    RigidBody* bar = fw.addFixedMagnetBox(F.P(COMPASS_PIVOT), {0.5f, COMPASS_HALF, 1.5f}, F.Q(yaw * base), 0.0f, 0.0f);
    bar->friction = 0.05f;          // すべりやすい（着いた勢いのまま渡る）
    bar->rollingFriction = 0.0f;
    bar->restitution = 0.0f;
    Rotor& r = fw.addRotor(bar, F.P(COMPASS_PIVOT), {0, 1, 0}, -10.0f, 10.0f, COMPASS_INERTIA);
    r.damping = COMPASS_DAMP;
    r.bounce  = 0.0f;
    const float e = COMPASS_HALF + 0.6f;
    fw.attachToRotor(r, fw.addPole(F.P(COMPASS_PIVOT + dirN * e), 1.0f, +1, COMPASS_Q).body);    // N 端
    fw.attachToRotor(r, fw.addPole(F.P(COMPASS_PIVOT - dirN * e), 1.0f, -1, COMPASS_Q).body);    // S 端
    solid(fw, F, COMPASS_PIVOT - Vec3{0, 6.5f, 0}, {0.6f, 5.6f, 0.6f});   // 軸受けの柱（棒の下）
    solid(fw, F, COMPASS_EXIT - Vec3{0, 6.0f, 0}, {2.5f, 5.0f, 2.5f});    // 出口の柱
}

// ---------------------------------------------------------------------------
// 9 磁気コア（Stage 06。全部の要素）:
//   1. 軌道コア M1（S）で周回して速さを稼ぎ、S に切り替えて放す
//   2. 壁の M2（N がこちら）に吸い付き、N に切り替えて押し出されて M3（S がこちら）へ
//   3. M3 にいるあいだ、プレイヤー（N）の磁場が溝の可動磁石 M4（N がこちら）を押し出す。M4 は溝を滑って
//      鉄球を次々に引き寄せ（鉄の鎖）、鎖の先が溝の端の判定の箱に入るとコアの扉が開く
//   4. S に切り替えて M3 から跳び、扉の奥のコア C（N）に吸われて周回し、N に切り替えて上のゴールへ
// ---------------------------------------------------------------------------
const Vec3 CORE_LAUNCH{6.0f, 3.0f, -3.0f};
const Vec3 CORE_M1{9.0f, 1.0f, -7.0f};
const Vec3 CORE_M2{21.0f, -2.5f, -2.0f};
const Vec3 CORE_M2N{-0.3f, 1.0f, 0.8f};      // M2 の面の向き（M1 から来て吸い付き、押し出されて M3 へ）
const Vec3 CORE_M3{13.0f, 7.0f, 13.0f};
const Vec3 CORE_M3N{0.8f, 1.0f, 0};          // M3 の面の向き（C の側へ傾ける）
constexpr float CORE_RAIL = 11.0f;            // 鉄の鎖の溝の長さ
const Vec3 CORE_C{30.0f, 13.0f, 13.0f};
const Vec3 CORE_GOAL{41.0f, 6.0f, 19.0f};

void corePad(FlowWorld& fw, const Frame& F, int cp) {
    ramp(fw, F, 3.0f, 2.5f, 3.5f, 0.0f, 3);   // 磁針の橋から来る側（区間の -x）の短い縁（長いと橋に重なる）
    Pad& p = fw.addPad(F.P({0, -0.5f, 0}), F.HE({3.0f, 0.5f, 3.0f}), F.V(CORE_LAUNCH), +1, cp);
    p.reach = 4.0f;
}

void finale(FlowWorld& fw, const Frame& F) {
    // 1. 軌道コア M1（S。上から見て時計回り。M2 の側へ少し上げる）
    {
        const Vec3 D = normalize(Vec3{CORE_M2.x - CORE_M1.x, 0, CORE_M2.z - CORE_M1.z});
        const float b = 12.0f * PHYS_PI / 180.0f;
        Core& c = fw.addCore(F.P(CORE_M1), CORE_R, -1, F.V(D * std::sin(b) - Vec3{0, 1, 0} * std::cos(b)));
        c.push = CORE_PUSH; c.lock = CORE_LOCK; c.reach = CORE_REACH; c.repel = CORE_REPEL;
    }
    // 2. M2（柱の M1 の側に N の板）と M3（高い台の S の板）
    solid(fw, F, CORE_M2 - normalize(CORE_M2N) * 0.8f, {0.6f, 0.6f, 0.6f});   // M2 の台座
    RigidBody* m2 = plate(fw, F, CORE_M2, normalize(CORE_M2N), 2.2f, +1, 0.0f, 0.8f);
    m2->rollingFriction = 2.0f;   // 縦の板に吸い付いたまま転がり落ちない
    solid(fw, F, CORE_M3 + Vec3{0, -1.6f, 0}, {3.0f, 1.0f, 3.0f});
    plate(fw, F, CORE_M3 + Vec3{0, -0.35f, 0}, normalize(CORE_M3N), 1.8f, -1, 0.0f, 0.7f)->rollingFriction = 2.0f;
    // 3. M4 と鉄の鎖: M3 の台の奥（+z）へのびる平らな溝。M4 は溝の手前の端
    const Vec3 top = CORE_M3 + Vec3{0, -0.6f, 2.4f};
    const float slope = 0.0f, len = CORE_RAIL;   // 平らな溝（プレイヤーが押すまで動かない）
    const Vec3 dir{0, -std::sin(slope), std::cos(slope)};          // 溝の下る向き（+z）
    const Quat q = F.Q(Quat::fromAxisAngle({1, 0, 0}, slope));
    const Vec3 mid = top + dir * (0.5f * len);
    // 傾けた箱は、姿勢（q）に区間の向きが入っているので、半分の辺は区間の座標のまま渡す
    const Vec3 up = Quat::fromAxisAngle({1, 0, 0}, slope).rotate({0, 1, 0});
    fw.addSolid(F.P(mid - up * 0.45f), {0.6f, 0.1f, 0.5f * len}, q, 0.0f, 0.0f);    // 溝の底（すべる）
    fw.addSolid(F.P(mid + Vec3{0.75f, 0, 0} - up * 0.1f), {0.15f, 0.4f, 0.5f * len}, q, 0.0f, 0.0f);
    fw.addSolid(F.P(mid + Vec3{-0.75f, 0, 0} - up * 0.1f), {0.15f, 0.4f, 0.5f * len}, q, 0.0f, 0.0f);
    fw.addMovableMagnetBox(F.P(top + dir * 0.6f), 0.4f, F.V({0, 0, -1}), 1.0f, MAGNET_Q, worldMoment(0.5f));   // M4
    for (int k = 0; k < 3; ++k)
        fw.addIronSphere(F.P(top + dir * (4.5f + 3.2f * (float)k)), 0.4f, 1.0f, 0.3f);
    // 溝の下端の判定の箱と、コアの扉（C の手前の壁の戸口を上へ開く）
    Vec3 tlo, thi;
    const Vec3 end = top + dir * len;
    F.box(end + Vec3{-0.7f, -1.0f, -1.5f}, end + Vec3{0.7f, 1.0f, 1.0f}, tlo, thi);
    solid(fw, F, end + Vec3{0, -1.2f, 0.8f}, {0.8f, 0.2f, 1.2f});   // 鉄球の受け皿
    // コアの部屋（四方の高い壁と床）。入口は M3 の側の扉だけ
    const float X0 = CORE_C.x - 5.0f, X1 = CORE_C.x + 17.0f, Z0 = CORE_C.z - 11.0f, Z1 = CORE_C.z + 11.0f;
    const float Y0 = CORE_C.y - 11.0f, Y1 = CORE_C.y + 13.0f, XM = 0.5f * (X0 + X1), ZM = 0.5f * (Z0 + Z1);
    const float HX = 0.5f * (X1 - X0), HZ = 0.5f * (Z1 - Z0), HY = 0.5f * (Y1 - Y0), YM = 0.5f * (Y0 + Y1);
    // 屋根はない（上から中が見える。M3 から跳んでも壁の上は越えられない）
    solid(fw, F, {XM, Y0 - 0.5f, ZM}, {HX + 0.5f, 0.5f, HZ + 0.5f});         // 床
    solid(fw, F, {X1, YM, ZM}, {0.5f, HY, HZ});                              // 奥の壁
    solid(fw, F, {XM, YM, Z0}, {HX, HY, 0.5f});                              // 横の壁
    solid(fw, F, {XM, YM, Z1}, {HX, HY, 0.5f});
    // 入口の壁（x = X0）と扉（C の正面、6 x 8）
    const float DZ = 3.0f, DY = 4.0f;
    solid(fw, F, {X0, YM, 0.5f * (Z0 + CORE_C.z - DZ)}, {0.5f, HY, 0.5f * (CORE_C.z - DZ - Z0)});
    solid(fw, F, {X0, YM, 0.5f * (Z1 + CORE_C.z + DZ)}, {0.5f, HY, 0.5f * (Z1 - CORE_C.z - DZ)});
    solid(fw, F, {X0, 0.5f * (Y1 + CORE_C.y + DY), CORE_C.z}, {0.5f, 0.5f * (Y1 - CORE_C.y - DY), DZ});
    solid(fw, F, {X0, 0.5f * (Y0 + CORE_C.y - DY), CORE_C.z}, {0.5f, 0.5f * (CORE_C.y - DY - Y0), DZ});
    fw.addGate(F.P({X0, CORE_C.y, CORE_C.z}), F.HE({0.5f, DY, DZ}), F.V({0, 2.0f * DY + 0.5f, 0}), tlo, thi).openTime = 1.0f;
    // 4. コア C（N。扉の奥）とゴール（上の台）
    {
        const Vec3 D = normalize(Vec3{CORE_GOAL.x - CORE_C.x, 0, CORE_GOAL.z - CORE_C.z});
        const float b = 10.0f * PHYS_PI / 180.0f;
        Core& c = fw.addCore(F.P(CORE_C), CORE_R, +1, F.V(D * std::sin(b) - Vec3{0, 1, 0} * std::cos(b)));
        c.push = CORE_PUSH; c.lock = CORE_LOCK; c.reach = CORE_REACH; c.repel = CORE_REPEL;
    }
    solid(fw, F, CORE_GOAL + Vec3{0, -0.5f, 0}, {4.0f, 0.5f, 4.0f});
    plate(fw, F, CORE_GOAL + Vec3{0, -0.35f, 0}, {0, 1, 0}, 2.0f, -1, 0.0f, 0.8f);   // S が上（N を引く）
    Vec3 glo, ghi;
    F.box(CORE_GOAL + Vec3{-4, 0, -4}, CORE_GOAL + Vec3{4, 5, 4}, glo, ghi);
    fw.setGoal(glo, ghi);
}

// ---------------------------------------------------------------------------
// 区間の表
// ---------------------------------------------------------------------------
struct ZoneDef {
    const char* name;
    const char* hint;
    void (*pad)(FlowWorld&, const Frame&, int);   // 区間の始め（チェックポイント cp）
    void (*build)(FlowWorld&, const Frame&);
    Vec3 lo, hi;          // 区間の箱（区間の座標）
    float killY;
    Vec3 viewDir;
    Vec3 exit;            // 次の区間の原点（区間の座標）
    int  turn;            // 次の区間の向き（90 度 x turn）
    float viewDist = 0.0f;           // > 0: 全体を見せるカメラ（viewCenter を中心に、この距離から）
    Vec3  viewCenter{0, 0, 0};
};

const ZoneDef ZONES[] = {
    {"1  Magnetic Canyon", "get pulled to M1, flip to fly up, flip again as you near M2",
     [](FlowWorld& fw, const Frame& F, int cp) { startPad(fw, F, cp, {6.5f, 4.5f, 0}, +1); },
     canyon, {-4, -14, -10}, {33, 20, 10}, -12.0f, {1, -0.3f, -0.55f}, {30, 1.35f, 0}, 0},
    {"2  Pendulum Gorge", "S: the anchor catches you and swings you round - flip to let go",
     [](FlowWorld& fw, const Frame& F, int cp) { startPad(fw, F, cp, {6.0f, 7.0f, 0}, -1); },
     pendulum, {-3, -16, -10}, {56, 24, 10}, -15.0f, {0.1f, -0.15f, -1}, PENDULUM_EXIT, 3,
     34.0f, Vec3{22, 3, 0}},   // 振れる面を横から見る
    {"3  Orbit Garden", "orbit a core to build speed, flip to let go toward the next one",
     orbitPad, orbit, {-17, -20, -27}, {18, 12, 12}, -19.0f, {0, -1.0f, -0.85f}, Vec3{9, -0.9f, 12} + ORBIT_SHIFT, 0,
     40.0f, Vec3{0, 4.0f, -1.0f} + ORBIT_SHIFT},
    {"4  Lens Corridor", "the wall blocks the way: let the magnets bend your path around it",
     lensPad, lens, {-4, -32, -14}, {40, 14, 16}, -30.0f, {0.35f, -0.6f, -1}, LENS_EXIT, 0},
    {"5  Magnetic Factory", "push magnet A down the line: A -> iron B -> magnet C turns the gate",
     factoryPad, factory, Vec3{-14, -8, -5} + FACTORY_SHIFT, Vec3{28, 18, 10} + FACTORY_SHIFT, -6.0f, {1, -0.4f, -0.45f},
     Vec3{23.5f, FACTORY_GOAL_Y + 0.1f, 0} + FACTORY_SHIFT, 0},
    {"6  Steel Stepping Stones", "touch steel to magnetize it, then flip: it pushes you off. Magnetize A to drop ball B",
     steelPad, steel, {-4, -16, -10}, {44, 22, 10}, -15.0f, {0.2f, -0.25f, -1}, STEEL_EXIT, 0},
    {"7  Magnetic Storm", "the field moves: board the shuttle as it comes, climb the blinking ladder in rhythm",
     stormPad, storm, {-4, -20, -10}, {44, 26, 10}, -18.0f, {0.2f, -0.2f, -1}, STORM_EXIT, 0},
    {"8  Compass Bridge", "the giant needle turns to face your opposite pole: wait for it to point at you, then cross",
     compassPad, compass, {-4, -16, -12}, {34, 16, 12}, -14.0f, {0.4f, -0.55f, -1}, COMPASS_EXIT, 0},
    {"9  Magnetic Core", "orbit M1, M2 -> M3, let M3's field start the iron chain, then orbit the core to the goal",
     corePad, finale, {-6, -18, -14}, {52, 28, 26}, -16.0f, {0.35f, -0.75f, -1}, CORE_GOAL, 0},
};
constexpr int ZONE_COUNT = (int)(sizeof(ZONES) / sizeof(ZONES[0]));

} // namespace

int grandZoneCount() { return ZONE_COUNT; }

// 区間 k の座標 ↔ マップの座標（調整用）
static Frame zoneFrame(int k) {
    Frame F;
    for (int i = 0; i < k && i < ZONE_COUNT; ++i) {
        Frame n;
        n.o    = F.P(ZONES[i].exit);
        n.turn = F.turn + ZONES[i].turn;
        F = n;
    }
    return F;
}

Vec3 grandToLocal(int k, const Vec3& w) {
    const Frame F = zoneFrame(k);
    const Vec3  d = w - F.o;
    switch (F.turn & 3) {   // V の逆
    case 1:  return Vec3{-d.z, d.y, d.x};
    case 2:  return Vec3{-d.x, d.y, -d.z};
    case 3:  return Vec3{d.z, d.y, -d.x};
    default: return d;
    }
}

Vec3 grandToWorld(int k, const Vec3& l) { return zoneFrame(k).P(l); }

void buildGrand(FlowWorld& fw) {
    Frame F;
    for (int k = 0; k < ZONE_COUNT; ++k) {
        const ZoneDef& z = ZONES[k];
        // 調整用（grandOnlyZone）: その区間と、前後の区間を組む（となりの区間の磁石が出口や発射に効く）
        const bool on = grandOnlyZone < 0 || std::abs(grandOnlyZone - k) <= 1;
        const bool nextPad = grandOnlyZone >= 0 && grandOnlyZone + 2 == k;   // その次の区間は発射台だけ
        if (on || nextPad) {
            Zone& zz = zone(fw, F, z.name, z.hint, z.lo, z.hi, z.killY, z.viewDir);
            if (z.viewDist > 0.0f) {
                zz.overview   = true;
                zz.viewDist   = z.viewDist;
                zz.viewCenter = F.P(z.viewCenter);
            }
            z.pad(fw, F, k);
            if (on) z.build(fw, F);
        } else {
            fw.addZone(z.name, z.hint, Vec3{0, -1e6f, 0}, Vec3{0, -1e6f, 0}, -1e6f);   // 番号をそろえる
        }
        Frame next;
        next.o    = F.P(z.exit);
        next.turn = F.turn + z.turn;
        F = next;
    }
    // 始め: 最初の区間の発射台の上
    fw.startPos = Vec3{0, 0.5f, 0};
    fw.viewDir  = ZONES[0].viewDir;
    fw.killY    = -12.0f;
}

} // namespace flow
