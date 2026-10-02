// ---------------------------------------------------------------------------
// flow_lab.cpp : 新しいギミックの短いステージ（magnetic_flow_gimmick_stage_spec.md §17）
//   L1 Cyclotron        サイクロトロン: 隙間に入る手前で切り替え続けて、一定のリズムのまま渦を描いて速くなる
//   L2 Gauss Rings      ガウス加速リング: 輪をくぐる瞬間に切り替える。速くなるほどビートが詰まる
//   L3 Resonance Swing  共振ブランコ: 振り子の揺れに合わせて切り替え、揺れを育てて壁を叩き壊す
//   L4 Ball Chain       鉄球のロープ: 磁石球の鎖の先をつかんで振れ、切り替えて次の鎖へ渡る
//   値は tests/flow_test.cpp のボット（FL19〜FL22）と、調整用のスクラッチのプログラムで詰めた
// ---------------------------------------------------------------------------
#include "flow/flow_stages.h"

#include <cmath>

namespace flow {

namespace {

Quat rotY(const Vec3& dir) {   // +y を dir に向ける
    const Vec3  d = normalize(dir);
    const float c = d.y;
    if (c > 0.9999f)  return Quat{};
    if (c < -0.9999f) return Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
    return Quat::fromAxisAngle(cross(Vec3{0, 1, 0}, d), std::acos(clampf(c, -1.0f, 1.0f)));
}

// 板の磁石（面の向き normal に極 pole が出ている。プレイヤーにだけ効く）
RigidBody* plate(FlowWorld& fw, const Vec3& pos, const Vec3& normal, float halfSize, int pole, float q) {
    return fw.addFixedMagnetBox(pos, {halfSize, 0.3f, halfSize}, rotY(normal * (float)pole), q, 0.0f);
}

}   // namespace

// ---------------------------------------------------------------------------
// L1 サイクロトロン
//   宙に浮く円盤（半径 10 m）。隙間（x = 0 の帯）をはさんで、+x の半分では N、-x の半分では S のときに
//   上から見て反時計回りに回る（回る速さ 3 rad/s。速さによらない）。隙間では極性の向きに押される。
//   隙間に入る手前で、入っていく半分の極に切り替え続けると、約 1.2 s ごとのリズムのまま速くなり、
//   渦が広がる。+x の側だけ磁場が中心から 8 m の弦で終わっていて、渦がそこを越えると飛び出す
// ---------------------------------------------------------------------------
const Vec3  CYC_CENTER{0, 6, 0};
constexpr float CYC_OMEGA = 3.0f;
constexpr float CYC_KICK  = 1.8f;    // 1 回で増える速さ [m/s]
constexpr float CYC_GAP   = 0.6f;    // 隙間の半分の幅 [m]
constexpr float CYC_V0    = 6.0f;

void buildCyclotron(FlowWorld& fw) {
    Cyclotron& c = fw.addCyclotron(CYC_CENTER, {0, 1, 0}, {1, 0, 0});
    c.omega         = CYC_OMEGA;
    c.kick          = CYC_KICK;
    c.gapHalf       = CYC_GAP;
    c.fringe        = 1.0f;
    c.radius        = 10.0f;
    c.portRadius    = 8.0f;
    // 始めは隙間の上、最初の半円の半径 r1 のところ（中心から始めると渦の中心がずれて外へはみ出す）。
    // 最初の半分に入りきったところで押されるので、r1 = (v0 + kick) / omega
    fw.startPos       = CYC_CENTER + Vec3{0, 0, (CYC_V0 + CYC_KICK) / CYC_OMEGA};
    fw.launchVelocity = Vec3{CYC_V0, 0, 0};
    fw.startPolarity  = +1;
    // 外周の低い壁（出口の側だけ開いている）。逆に曲がって振り出されると、壁に当たって落ちる
    {
        constexpr int   SEG = 36;
        constexpr float RF  = 10.6f;
        const float open0 = -65.0f * PHYS_PI / 180.0f, open1 = 40.0f * PHYS_PI / 180.0f;   // 開いている向き（+x から）
        for (int i = 0; i < SEG; ++i) {
            float phi = 2.0f * PHYS_PI * ((float)i + 0.5f) / SEG;
            if (phi > PHYS_PI) phi -= 2.0f * PHYS_PI;
            if (phi > open0 && phi < open1) continue;
            const Vec3 pos = CYC_CENTER + Vec3{RF * std::cos(phi), 0, RF * std::sin(phi)};
            fw.addSolid(pos, {0.15f, 0.9f, RF * std::sin(PHYS_PI / SEG) + 0.05f}, Quat::fromAxisAngle({0, 1, 0}, -phi), 0.1f, 0.0f);
        }
    }
    // 出口の先のゴールの台と、引き寄せる磁石（N を引く S が上）
    //   （出口の弦 x = 8 を越えて -z の向きへ約 21 m/s で飛び出し、約 22 m 先に落ちる。弦を越えるところで、
    //     外向きに 0〜30 度ずれる）
    const Vec3 goal{13.5f, 0.0f, -21.5f};
    fw.addSolid(goal - Vec3{0, 0.5f, 0}, {9.0f, 0.5f, 5.0f});
    plate(fw, goal - Vec3{0, 0.2f, 0}, {0, 1, 0}, 2.0f, -1, 0.6f);
    fw.setGoal(goal + Vec3{-9.0f, 0.0f, -5.0f}, goal + Vec3{9.0f, 4.0f, 5.0f});
    fw.viewDir    = {0.35f, -1.0f, 0.8f};
    fw.overview   = true;
    fw.viewCenter = {4.0f, 3.0f, -8.0f};
    fw.viewDist   = 42.0f;
    fw.killY      = -6.0f;
}

// ---------------------------------------------------------------------------
// L2 ガウス加速リング
//   10 度上りの列に、内側の極が N, S, N, ... の輪を 8 m おきに 6 つ。輪は近づくプレイヤーを面へ引き込み、
//   くぐる瞬間に切り替えると押し出して、次の輪（逆の極）へ引き込ませる。最後の輪から斜め上へ飛び出し、
//   穴の向こうの台へ。くぐるたびに速くなるので、切り替える間隔が詰まっていく
// ---------------------------------------------------------------------------
const Vec3  GAUSS_ORIGIN{0, 3, 0};
constexpr float GAUSS_TILT    = 10.0f;
constexpr float GAUSS_SPACING = 8.0f;
constexpr int   GAUSS_RINGS   = 6;

Vec3 gaussAxis() {
    const float th = GAUSS_TILT * PHYS_PI / 180.0f;
    return Vec3{std::cos(th), std::sin(th), 0};
}

void buildGaussRings(FlowWorld& fw) {
    const Vec3 d = gaussAxis();
    for (int k = 0; k < GAUSS_RINGS; ++k) {
        const Vec3 c = GAUSS_ORIGIN + d * (GAUSS_SPACING * (float)(k + 1));
        GaussRing& g = fw.addGaussRing(c, d, 2.0f, (k % 2 == 0) ? +1 : -1);
        g.accel = 14.0f;
        g.reach = 3.0f;
        // 輪の支柱（見た目。輪の下の縁から地面まで）
        const float bottom = c.y - 2.45f, ground = -2.0f;
        fw.addSolid({c.x, 0.5f * (bottom + ground), c.z}, {0.15f, 0.5f * (bottom - ground), 0.15f});
    }
    // 開始の台
    const Vec3 start = GAUSS_ORIGIN + d * (GAUSS_SPACING - 3.0f - 1.0f);
    fw.addSolid(start - Vec3{0, 1.0f, 0}, {1.2f, 0.5f, 1.2f});
    // 穴の向こうのゴールの台（最後の輪の出口より 1 m 低い）。全部の輪で押されると約 22 m/s で x = 72 あたりに、
    // 最後の輪だけ外すと約 19 m/s で 68 あたりに落ちる（どちらも台の上）。途中の輪を外すと約 17 m/s で 64 の穴へ
    const Vec3 last = GAUSS_ORIGIN + d * (GAUSS_SPACING * (float)GAUSS_RINGS);
    const Vec3 goal = last + Vec3{24.7f, -1.0f, 0};
    fw.addSolid(goal - Vec3{0, 0.5f, 0}, {6.0f, 0.5f, 4.0f});
    plate(fw, goal - Vec3{0, 0.2f, 0}, {0, 1, 0}, 2.0f, +1, 0.6f);   // N が上（最後の輪で S になったプレイヤーを引く）
    fw.setGoal(goal + Vec3{-6.0f, 0.0f, -4.0f}, goal + Vec3{6.0f, 4.0f, 4.0f});
    fw.startPos       = start;
    fw.launchVelocity = d * 5.0f;
    fw.startPolarity  = -1;   // 輪 1（N）と異極: 引き込まれる
    fw.viewDir        = {1.0f, -0.35f, -0.9f};
    fw.killY          = -4.0f;
}

// ---------------------------------------------------------------------------
// L3 共振ブランコ
//   天井の軸から腕 5.5 m の振り子（おもりは N の磁極）。プレイヤーは最下点の横（z = +2.1）の窪みの台に座る。
//   「近づくあいだ引く（S）→ すれ違ったら押す（N）→ 向こうで折り返したら引く（S）」で揺れが育ち、
//   +x の側で 100 度まで振れると、おもりが壁を叩き壊す（壁は外へ倒れて橋になる。おもりは磁力が抜ける）。
//   壁が壊れると台の錠が外れ、押すと壊れた壁の穴へ飛んで、向こうの台へ
// ---------------------------------------------------------------------------
const Vec3  RESO_PIVOT{0, 10, 0};
constexpr float RESO_L     = 5.5f;
constexpr float RESO_BOB_R = 0.8f;
constexpr float RESO_HI    = 100.0f;   // 壁の角度 [deg]
constexpr float RESO_SEAT_Z = 2.1f;

void buildResonance(FlowWorld& fw) {
    const Vec3 piv = RESO_PIVOT;
    fw.addSolid({4.0f, -0.5f, 1.0f}, {12.0f, 0.5f, 6.0f});                       // 床
    fw.addSolid(piv + Vec3{0, 0.6f, 0}, {1.5f, 0.3f, 2.5f});                       // 軸の梁
    // 振り子: 腕（静的な箱）と、おもり（N の磁極）。Rotor が重力と、プレイヤーの力の反作用で回す
    RigidBody* rod = fw.addSolid(piv - Vec3{0, 0.5f * RESO_L, 0}, {0.12f, 0.5f * RESO_L, 0.12f});
    Core& bob = fw.addPole(piv - Vec3{0, RESO_L, 0}, RESO_BOB_R, +1, 0.3f);
    const float hi = RESO_HI * PHYS_PI / 180.0f;
    Rotor& r = fw.addRotor(rod, piv, {0, 0, 1}, -hi, hi, 0.0f);
    fw.attachToRotor(r, bob.body);
    r.mass      = 5.0f;
    r.inertia   = r.mass * RESO_L * RESO_L;
    r.com       = piv - Vec3{0, RESO_L, 0};
    r.damping   = 0.02f;
    r.bounce    = 0.3f;
    r.spendOnHi = true;
    r.angle     = -25.0f * PHYS_PI / 180.0f;   // 始めは少し振れている（真下で止まっていると横からの力では回らない）
    // -x の側の壊れない壁（おもりが当たって跳ね返る）
    const float bx = piv.x + RESO_L * std::sin(hi) + RESO_BOB_R + 0.05f;   // 壁の面（+x の側）
    const float by = piv.y - RESO_L * std::cos(hi);
    fw.addSolid({-bx - 0.25f, by, 0}, {0.25f, 2.5f, 1.5f});
    // 座る台（錠つきの発射台）と窪み。z の向きの壁は高く（引かれても押されても外れない）、x の向きの縁は低く（発射で越える）
    const float yb = piv.y - RESO_L;
    const Vec3  base{0, yb - 0.75f, RESO_SEAT_Z};
    Pad& pad = fw.addPad(base, {0.7f, 0.25f, 0.7f}, {6.5f, 10.5f, 0}, +1, -1);
    fw.addSolid({0, 0.5f * (base.y - 0.25f), RESO_SEAT_Z}, {0.7f, 0.5f * (base.y - 0.25f), 0.7f});   // 台の脚
    const float w = 0.8f, floorY = base.y + 0.25f, top = yb + 0.5f, low = floorY + 0.3f;
    fw.addSolid({ w + 0.1f, 0.5f * (floorY + low), RESO_SEAT_Z}, {0.1f, 0.5f * (low - floorY), w + 0.2f});
    fw.addSolid({-w - 0.1f, 0.5f * (floorY + low), RESO_SEAT_Z}, {0.1f, 0.5f * (low - floorY), w + 0.2f});
    fw.addSolid({0, 0.5f * (floorY + top), RESO_SEAT_Z - w - 0.1f}, {w, 0.5f * (top - floorY), 0.1f});
    fw.addSolid({0, 0.5f * (floorY + top), RESO_SEAT_Z + w + 0.1f}, {w, 0.5f * (top - floorY), 0.1f});
    // 壊れる壁: おもりが hi に届くと、下の辺のまわりに外へ倒れて橋になる。まわりの壁
    Gate& g = fw.addHingedGate({bx + 0.25f, by, 1.0f}, {0.25f, 2.5f, 2.5f}, Quat{}, {bx + 0.5f, by - 2.5f, 1.0f}, {0, 0, 1},
                               -0.5f * PHYS_PI, {0, 0, 0}, {0, 0, 0});
    g.rotorTrigger = (int)fw.rotors.size() - 1;
    g.openTime     = 0.5f;
    pad.lockGate   = (int)fw.gates.size() - 1;
    fw.addSolid({bx + 0.25f, 0.5f * (by - 2.5f), 1.0f}, {0.25f, 0.5f * (by - 2.5f), 4.0f});        // 穴の下
    fw.addSolid({bx + 0.25f, by + 3.75f, 1.0f}, {0.25f, 1.25f, 4.0f});                            // 穴の上
    fw.addSolid({bx + 0.25f, by, -2.75f}, {0.25f, 2.5f, 1.25f});                                  // 穴の横
    fw.addSolid({bx + 0.25f, by, 4.75f}, {0.25f, 2.5f, 1.25f});
    // 壁の向こうの台とゴール
    fw.addSolid({bx + 8.0f, by - 3.0f, 1.0f}, {4.0f, 0.5f, 3.0f});
    fw.setGoal({bx + 4.0f, by - 2.5f, -2.0f}, {bx + 12.0f, by + 1.5f, 4.0f});
    fw.startPos      = base + Vec3{0, 0.75f, 0};
    fw.startPolarity = -1;   // おもり（N）と異極: 近づくおもりを引く
    fw.viewDir       = {0.1f, -0.3f, -1.0f};
    fw.overview      = true;
    fw.viewCenter    = {2.5f, 7.0f, 0.5f};
    fw.viewDist      = 26.0f;
    fw.killY         = -3.0f;
}

// ---------------------------------------------------------------------------
// L4 鉄球のロープ
//   天井の磁石から、磁石球 8 個の鎖を 3 本垂らす（物どうしの双極子でつながっている。1 本ごとに 1 m 低い）。
//   鎖の先の極は N, S, N の順。高い開始の台から鎖 1 へ飛び込み、先の球に引かれてつかまり、振れる。
//   前へ振れたところで切り替えると、反発で離れて次の鎖（逆の極）に引かれる。鎖 3 から離れて、ゴールの台の磁石へ。
//   並んだ双極子の鎖は外から見ると両端に極がある棒磁石なので、プレイヤーには先の球だけが効く
// ---------------------------------------------------------------------------
constexpr float CHAIN_TOP_Y = 13.0f;
constexpr float CHAIN_X1    = 5.0f;
constexpr float CHAIN_GAP   = 8.0f;
constexpr float CHAIN_STEP  = -1.0f;   // 1 本ごとの天井の高さの段差
constexpr float CHAIN_START_Y = 10.0f;

void buildBallChain(FlowWorld& fw) {
    fw.addSolid({-1.0f, CHAIN_START_Y - 1.0f, 0}, {1.5f, 0.5f, 1.5f});   // 開始の台
    fw.addSolid({-1.0f, 0.5f * (CHAIN_START_Y - 1.5f) - 1.5f, 0}, {0.5f, 0.5f * (CHAIN_START_Y - 1.5f) + 1.5f, 0.5f});   // 台の柱
    for (int i = 0; i < 3; ++i) {
        const Vec3 top{CHAIN_X1 + CHAIN_GAP * (float)i, CHAIN_TOP_Y + CHAIN_STEP * (float)i, 0};
        fw.addSolid(top + Vec3{0, 0.8f, 0}, {1.2f, 0.3f, 1.2f});   // 天井の梁
        fw.addBallChain(top, 0.5f, 8, 0.3f, 0.3f, (i % 2 == 0) ? +1 : -1, 0.15f, 150.0f);
    }
    // ゴールの台と、引き寄せる磁石（鎖 3 を離れた N を引く S が上）
    const Vec3 goal{CHAIN_X1 + CHAIN_GAP * 3.0f, 4.0f, 0};
    fw.addSolid(goal - Vec3{0, 0.5f, 0}, {4.0f, 0.5f, 3.0f});
    plate(fw, goal - Vec3{0, 0.2f, 0}, {0, 1, 0}, 1.5f, -1, 0.5f);
    fw.setGoal(goal + Vec3{-4.0f, 0.0f, -3.0f}, goal + Vec3{4.0f, 4.0f, 3.0f});
    fw.startPos       = {-1.0f, CHAIN_START_Y, 0};
    fw.launchVelocity = {5.0f, 1.0f, 0};
    fw.startPolarity  = -1;   // 鎖 1 の先（N）と異極
    fw.viewDir        = {0.0f, -0.25f, -1.0f};
    fw.overview       = true;
    fw.viewCenter     = {13.0f, 8.0f, 0};
    fw.viewDist       = 30.0f;
    fw.killY          = -3.0f;
}

}   // namespace flow
