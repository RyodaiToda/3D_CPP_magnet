#pragma once
// ---------------------------------------------------------------------------
// flow_world.h : MAGNET FLOW の物理とルール（raylib に依存しない）
//
//   物どうし（固定磁石・可動磁石・鉄）の力は、今の双極子エンジン（MagnetSystem）で計算する。
//   プレイヤーは MagnetSystem に入れず、ゲーム用の「磁極」として働く:
//     磁性体 i の表面でいちばん近い点からプレイヤーまでの距離 d、向き r（i → プレイヤー）について
//       g(d) = 1 / (d^2 + a^2) * (1 - (d / Rc)^2)^2          （d < Rc のとき。届く距離 Rc）
//       磁石: F = s * c * q_i * K * g(d) * r    s: プレイヤーの極性（N = +1, S = -1）
//                                                c = n_i . r（n_i は N 極の向き。プレイヤー側を向いている極。N なら +）
//             → 同極なら反発（+r）、異極なら吸引（-r）
//       鉄  : F = -K_iron * q_i * g(d) * r      （極性に関係なく吸引）
//       軌道コア: 表面のどこでも同じ極が外を向いている（c = pole）。そのうえで「回転する磁場」が
//             プレイヤーを回転の向きに押し、回転面から外れる動きを抑え、回転面へ引き戻す（上限の外で別に加える）:
//               F_vortex = m * h(d) * ( a_push * t - k * (v - (v . t) t) - k_hold * (r . axis) axis )
//               t: 回転の向き（axis x r の単位ベクトル）、h(d) = (1 - d / D)^2（D: 渦の届く距離）
//             → 吸われて周回するほど速くなり、速すぎると引力が足りなくなって振り飛ばされる
//     合力が上限を超えたら全部のペアを同じ割合で縮め、動ける物には反作用 -F_i を加える
//     （運動量が保存する。プレイヤーが物を動かし、物どうしの双極子の力で連鎖が起きる）
//
//   大きなマップのためのギミック（magnetic_flow_gimmick_stage_spec.md §4 の残り）:
//     動的磁場（G4, Animator）   : 時間で回る・往復する・点滅する磁石
//     磁気振り子（G6, Swing）    : 異極で近づくと見えない綱がつながる錨。下を通るたびに押されて振れが大きくなる
//     磁化連鎖（G5, Steel）      : プレイヤーが触れると磁化される鋼。磁化はゆっくりしか変わらないので、
//                                  触れたまま切り替えると反発する。磁化は物どうしの双極子にもなる
//     磁気トルク（Rotor）        : ヒンジで回る板。プレイヤーの磁力の反作用のトルクと重力で回る
//     発射台（Pad）              : どちらの極でも引いて止める台。チェックポイントで、押すと決めた初速で飛ばす
//     区間（Zone）               : カメラの向きと落下の高さを、プレイヤーのいる区間で切り替える
//
//   新しいギミック（magnetic_flow_gimmick_stage_spec.md §17）:
//     サイクロトロン（Cyclotron） : 2 つの半分に分かれた円盤。半分の中では一定の速さで回り、隙間で切り替えると押される
//     ガウス加速リング（GaussRing）: 輪の面へ引き込み、くぐる瞬間に切り替えると押し出す
//     共振ブランコ                : Rotor（重力つき）の振り子を、プレイヤーの力の反作用で揺らす。振れると扉が開く
//     鉄球のロープ（BallChain）   : 物どうしの双極子でつながった可動の磁石球の鎖
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace flow {

using namespace phys;

constexpr float G_ACC      = 9.81f;
constexpr int   TRAIL_STEP = 4;                        // 軌跡は 4 ステップ（1/30 s）ごとに記録する
constexpr float COMBO_GAP  = 2.0f;                     // 磁気イベントがこの時間 [s] 以内に続けば連続とみなす
constexpr float RESPAWN_DELAY = 0.4f;                  // 落ちてからチェックポイントに戻るまで [s]

// 調整できる値（調整パネル F1）
struct FlowParams {
    float K        = 400.0f;  // 磁石 → プレイヤーの力の強さ
    float Kiron    = 150.0f;  // 鉄 → プレイヤー
    float soft     = 1.0f;    // なめらかにする長さ a [m]
    float range    = 12.0f;   // 届く距離 Rc [m]
    float forceCap = 6.0f;    // 合力の上限（プレイヤーの重さの何倍か）
    float speedCap = 25.0f;   // 速さの上限 [m/s]
    float gravity  = 1.0f;    // 重力の倍率
    float corePush = 1.0f;    // 軌道コアの渦が回転の向きに押す強さの倍率
    float coreLock = 1.0f;    // 軌道コアの渦が回転面から外れる動きを抑える強さの倍率
    float swingPump = 1.0f;   // 磁気振り子が押す強さの倍率
    int   substeps = 8;
};

enum class Kind : unsigned char { Other, Solid, FixedMagnet, MovableMagnet, Iron, Door, Player, Core, Anchor, Pad };

enum class State { Ready, Running, Cleared, Failed };

// 扉と一緒に動く物（扉に付けた磁石の板など）。閉じたときの位置・姿勢を覚えておく
struct Attached { RigidBody* body; Vec3 closedPos; Quat closedRot; };

struct Gate {
    RigidBody* door = nullptr;
    Vec3  closedPos, openPos;
    Quat  closedRot;
    // 回る扉: pivot を通る hingeAxis のまわりに openAngle [rad] だけ回る（false なら openPos へ平行移動）
    bool  hinged    = false;
    Vec3  pivot, hingeAxis{1, 0, 0};
    float openAngle = 0.0f;
    std::vector<Attached> attached;
    Vec3  lo, hi;            // 判定の箱（動く磁性体の中心が入ったら開く）
    float held     = 0.0f;   // 判定の箱に入っている時間
    float open     = 0.0f;   // 0: 閉 → 1: 開
    float openTime = 0.6f;   // 開くのにかかる時間 [s]
    bool  opening = false;
    int   zone = -1;         // 属する区間（チェックポイントから戻ったとき、前の区間の扉は開けておく）
    int   rotorTrigger = -1; // >= 0: 判定の箱の代わりに、この Rotor が止め hi に届いたら開く（振り子が壁を壊す）
};

// 軌道コア（G1 磁気軌道）: 回転する磁場を持つ、固定された大きな球
struct Core {
    RigidBody* body = nullptr;
    int   pole  = -1;          // 外を向いている極（+1: N / -1: S）
    Vec3  axis{0, 0, 1};       // 回転軸（右ねじの向きに回る）
    float push  = 5.0f;        // 回転の向きに押す加速度 [m/s^2]（表面で）
    float lock  = 2.5f;        // 回転面から外れる速度を抑える強さ [1/s]
    float hold  = 25.0f;       // 回転面（赤道の面）へ引き戻す強さ [1/s^2]（極の真下で止まらないように）
    float reach = 4.0f;        // 渦の届く距離（表面から）[m]
    float repel = 0.35f;       // 反発の強さ（吸引に対する割合）。弱くして、放した向きを進む向きに近づける
    float spin  = 0.0f;        // 描画用の回転角
    bool  plain = false;       // 渦のないただの磁極（棒磁石の端など。reach = 0）
};

// 動的磁場（G4）: 時間で回る・往復する・点滅する磁石。静的な物体の位置・姿勢・強さを毎サブステップ書き換える。
//   時刻は区間の時計（その区間のチェックポイントに着いてから。チェックポイントから戻ると 0 に戻る）
struct Animator {
    RigidBody* body = nullptr;
    Vec3  basePos;
    Quat  baseRot;
    Vec3  spinAxis{0, 1, 0};
    float spinRate     = 0.0f;      // 自転の速さ [rad/s]
    float spinStep     = 0.0f;      // > 0: 自転を spinStep [s] ごとに半回転ずつ進める（止まって・回る）
    Vec3  travel{0, 0, 0};          // 往復: basePos → basePos + travel → basePos
    float travelPeriod = 0.0f;      // 往復の周期 [s]（0: 動かない）
    float pulsePeriod  = 0.0f;      // 点滅の周期 [s]（0: 点滅しない）
    float pulseOn      = 0.5f;      // ON の割合
    float phase        = 0.0f;      // 時刻のずれ [s]
    float level        = 1.0f;      // 今の強さ（0: OFF, 1: ON）
    float nextToggle   = 0.0f;      // 次に ON / OFF が切り替わるまで [s]（描画用）
    float baseCharge   = 1.0f;
    Vec3  basePermanent{0, 0, 0};   // 物どうしの双極子（ローカル）
    int   magnetIndex = -1;         // MagnetSystem の添字（-1: 物どうしの双極子なし）
    int   zone = -1;
    std::vector<Attached> attached; // 一緒に動く物（closedPos / closedRot は basePos / baseRot のときの値）
};

// 磁気振り子（G6）: 錨（固定の球）。異極で length の内側を通ると、いちばん近づいた瞬間に見えない綱がつながり、
//   その距離に保たれる（速さを失わずに回り始める。近くをかすめるほど小さく速く回る）。
//   下を通るたびに進む向きへ押されて速くなる。回るのに要る力が maxHold を超えると切れて振り飛ばされる。
//   切り替えると綱が切れて、進んでいた向きに飛び出す
struct Swing {
    RigidBody* body = nullptr;
    int   pole   = -1;        // 外を向いている極（異極で綱がつながる）
    float length = 6.0f;      // 綱がつながる距離（錨の中心からプレイヤーの中心まで）
    float minLength = 2.0f;   // 綱のいちばん短い長さ
    float rod    = 0.0f;      // つながっている綱の長さ
    Vec3  planeN{0, 0, 1};    // 振れる面の法線（この面に保つ。0 なら自由）
    float lock   = 6.0f;      // 面から外れる速さを抑える強さ [1/s]
    float hold   = 20.0f;     // 面へ引き戻す強さ [1/s^2]
    float pump   = 4.0f;      // 下を通るときに進む向きへ押す加速度 [m/s^2]
    float cruise = 1e9f;      // 押す力はこの速さに近づくほど弱まる（同じ回り方にそろう）[m/s]
    float pull   = 0.3f;      // 綱がつながる前に引く強さ（磁石の吸引に対する割合）
    float repel  = 0.25f;     // 同極のときの反発の強さ（磁石の反発に対する割合）。弱くして、放した向きを進む向きに近づける
    float maxHold = 6.0f;     // 支えられる力（重さの何倍か）。回るのに要る力がこれを超えると綱が外へ滑って伸び、
                              // length を超えると切れて振り飛ばされる
    bool  engaged = false;
    bool  snapped = false;    // 切れた（切り替えるか、離れるまで、もうつながらない）
    float spin   = 0.0f;      // 描画用
};

// 磁化する鋼（G5）: プレイヤーが触れると、触れた側がプレイヤーと異極になるように磁化される
//   （ふつうの鉄と同じく、どちらの極でも引く）。磁化は rate の速さでしか変わらないので、
//   触れたまま切り替えると、しばらくは磁化が残って反発する。離れると decay 秒で消える。
//   磁化は物どうしの双極子にもなる（近くの鋼や磁石を引く）
struct Steel {
    RigidBody* body = nullptr;
    Vec3  mLocal{0, 0, 0};     // 磁化（ローカル。向き = N 極、大きさ 0..1）
    float rate   = 2.5f;       // 磁化が変わる速さ [1/s]（反転に 2 / rate 秒）
    float range  = 0.6f;       // 磁化する距離（表面どうし）[m]
    float decay  = 6.0f;       // 離れてから消えるまで [s]
    float moment = 0.0f;       // 磁化 1 のときの物どうしの双極子の強さ
    float strength = 1.0f;     // 磁化 1 のときのプレイヤーへの力（磁石の何倍か）
    int   magnetIndex = -1;
    bool  announced = false;   // 磁化したことを知らせた（大きさ 0.5 を超えた）
    Vec3  worldM() const { return body->orientation.rotate(mLocal); }
};

// 磁気トルクで回る板（ヒンジ）: プレイヤーの磁力の反作用によるトルクと、重力のトルクで回る
//   （ぶつかっても回らない。接触では動かない静的な物体として扱う）
struct Rotor {
    RigidBody* body = nullptr;
    Vec3  pivot, axis{0, 0, 1};
    Vec3  basePos;
    Quat  baseRot;
    std::vector<Attached> attached;
    float angle = 0.0f, omega = 0.0f;
    float lo = -PHYS_PI, hi = PHYS_PI;   // 止め [rad]
    float inertia = 40.0f;    // 慣性モーメント [kg m^2]
    float damping = 0.3f;     // [1/s]
    float mass = 0.0f;        // 重力のトルク: mass と重心 com（angle = 0 のとき、ワールド）
    Vec3  com{0, 0, 0};
    float bounce = 0.15f;     // 止めに当たったときの跳ね返り
    bool  reachedHi = false;
    bool  spendOnHi = false;  // 止め hi に届いたら、付けた磁石の磁力が抜ける（壁を叩き壊した振り子のおもり）
    float peak = 0.0f;        // 振れの大きさ（最後に折り返したときの |angle|）
    int   zone = -1;
};

// 発射台: どちらの極でも引いて止める台（チェックポイント）。座って押すと、決めた初速と極性で飛ばす
//   body = nullptr なら台のないチェックポイント（座った位置で記録するだけ）
struct Pad {
    RigidBody* body = nullptr;
    Vec3  seat;                // 座る位置（プレイヤーの中心）
    float seatRadius = 1.6f;   // この範囲に入ると座ったとみなす
    Vec3  launch{0, 0, 0};     // 発射の初速
    int   polarity = +1;       // 発射したときの極性
    int   hold = 0;            // 座っているあいだの極性（0: そのまま）。チェックポイントに戻ったときもこの極性
    int   checkpoint = -1;     // チェックポイントの番号
    float reach = 2.5f;        // 引く力の届く距離 [m]
    float cooldown = 0.0f;     // 発射してから引く力を止めている残り時間
    int   zone = -1;
    int   lockGate = -1;       // >= 0: この扉が開くまでは座らせるだけ（押すとふつうに切り替わる）。開くと発射台になる
};

// サイクロトロン: 宙に浮く円盤。隙間をはさんだ 2 つの半分（D 形）に分かれている。
//   半分の中では、進む向きに直角な力で円を描く（回る速さ omega は速さによらない。本物のサイクロトロンと同じ）:
//     F = m * omega * (s * side) * (axis x v_面内)      side: 隙間の法線 gapN の側が +1
//   → s * side = +1 で axis の右ねじの向きに回る。極性が半分に合わないと逆に曲がる
//   隙間（幅 2 gapHalf）の中はまっすぐ進む。隙間を抜けて次の半分に入りきったとき（隙間の縁から fringe）、
//   極性がその半分に合っていれば、進む向きに kick [m/s] だけ押される（半径が広がり、リズム pi / omega は変わらない）。
//   押されるかどうかは入りきった瞬間の極性で決まるので、それまでに切り替えればよい（遅れても押す力は減らない）。
//   早く切り替えすぎると、前の半分の終わりで逆に曲がる。隙間の縁の fringe の幅で、曲げる力を 0 から全部まで
//   なめらかに増やす（少し早く切り替えても乱れにくい）
//   円盤の面から height 以内では浮かせる（重力を打ち消し、面からのずれを戻す）。
//   出口の側（portDir）では、磁場が中心から portRadius の弦で終わる（渦がそこを越えると、外へ向かってまっすぐ飛び出す）
struct Cyclotron {
    Vec3  center;
    Vec3  axis{0, 1, 0};
    Vec3  gapN{1, 0, 0};
    float radius = 11.0f;
    float portRadius = 8.0f;   // 出口の弦の、中心からの距離
    Vec3  portDir{1, 0, 0};
    float gapHalf = 0.6f;      // 隙間の半分の幅 [m]
    float fringe  = 1.0f;      // 縁の幅 [m]
    float omega = 2.0f;        // 回る速さ [rad/s]
    float kick = 1.6f;         // 隙間を抜けて半分に入りきったときに増える速さ [m/s]
    float height = 3.0f;
    float lock = 6.0f;         // 面に直角な速さを抑える強さ [1/s]
    float hold = 30.0f;        // 面へ引き戻す強さ [1/s^2]
    // 状態（描画・数える）
    bool  inside = false;
    int   side = 0;            // プレイヤーのいる半分（-1 / +1。隙間の中では前の半分）
    int   kickedSide = 0;      // 最後に押されるかを決めた半分（同じ半分では 1 回だけ）
    float turns = 0.0f;        // 回った周回数（隙間を通るたびに 0.5）
    int   boosts = 0;          // 隙間で押された回数
    float flash = 0.0f;        // 隙間で押されたときに 1（描画で減らす）
};

// ガウス加速リング: 内側の極が pole の輪。面へ引き込み（異極）、面から押し出す（同極）:
//   F = m * accel * (s * pole) * sin(pi x / reach) * axis      x: 面からのずれ（axis の向きが +）
//   輪の中（軸から radius 以内、面から reach 以内）だけで働き、軸へ寄せる（重力で垂れないように）。
//   内側の極を交互に並べると、くぐる瞬間に切り替えるたびに、押し出されて次の輪に引き込まれる
struct GaussRing {
    Vec3  center;
    Vec3  axis{1, 0, 0};
    float radius = 2.0f;
    int   pole = +1;
    float accel = 25.0f;
    float reach = 1.6f;
    float guide = 40.0f;       // 軸へ寄せる強さ [1/s^2]
    float guideDamp = 6.0f;    // 軸から離れる速さを抑える [1/s]
    // 状態
    bool  lit = false;         // プレイヤーがこの輪の効く範囲にいる
    float flash = 0.0f;        // くぐって速くなったときに 1
    float entrySpeed = 0.0f;   // 効く範囲に入ったときの速さ
    bool  boosted = false;     // くぐって速くなった（最後に通ったとき）
};

// 鉄球のロープ: 天井の固定磁石 top から、可動の磁石球 balls が物どうしの双極子でつながって垂れている
struct BallChain {
    RigidBody* top = nullptr;
    std::vector<RigidBody*> balls;   // 上から順に
};

// 区間: プレイヤーのいる区間で、カメラの向きと落ちたら失敗する高さを切り替える
struct Zone {
    std::string name, hint;
    Vec3  lo, hi;
    float killY = -10.0f;
    Vec3  viewDir{1, -0.3f, 0};
    bool  overview = false;
    Vec3  viewCenter{0, 0, 0};
    float viewDist = 30.0f;
};

// 方位磁針の並び（描画だけ）: center のまわりに u, v 方向へ nu x nv 本
struct NeedleField { Vec3 center, u{1, 0, 0}, v{0, 0, 1}; int nu = 8, nv = 8; float step = 2.0f; };

struct SwitchMark { Vec3 p; int polarity; };

struct FlowStats {
    float time        = 0.0f;
    int   switches    = 0;
    float maxSpeed    = 0.0f;
    int   chain       = 0;     // 連鎖（動く物が初めて動いた・ゲートが開いた）
    int   events      = 0;     // 磁気イベントの数（吸着・射出・連鎖・ゲート）
    int   combo       = 0;     // 今続いている磁気イベントの数
    int   bestCombo   = 0;     // 連続磁気イベント数（いちばん長く続いたとき）
    int   magnetsUsed = 0;     // プレイヤーを重さ以上の力で引いた・押した磁石の数
    float laps        = 0.0f;  // 軌道コアのまわりを回った周回数（合計）
    int   falls       = 0;     // 落ちてチェックポイントに戻った回数（大きなマップ）
};

// 1 フレームのうちに起きたこと（描画・音が使ったら消す）
struct FlowEvents {
    bool switched = false, kick = false, hit = false, gateOpened = false, chain = false,
         cleared = false, failed = false, started = false, capture = false, lap = false,
         launched = false, checkpoint = false, respawned = false, swing = false, magnetized = false,
         rotorDown = false, zoneEntered = false, snapped = false,
         boost = false;   // サイクロトロンの隙間・加速リングで押されて速くなった
};

// 各物体からプレイヤーへの力（描画用）
struct PlayerLink { int body; Vec3 force; bool attract; };

class FlowWorld {
public:
    FlowParams params;
    World      world;
    RigidBody* player   = nullptr;
    int        polarity = +1;
    State      state    = State::Ready;
    FlowStats  stats;
    FlowEvents events;
    int        retries  = 0;    // やり直した回数（ステージを読み込むと 0）
    int        magnetTotal = 0; // ステージの磁石の数（固定・可動・コア・錨）

    // ステージの設定
    Vec3  startPos{0, 1, 0};
    int   startPolarity = +1;
    Vec3  launchVelocity{0, 0, 0};   // 開始したときの初速（発射台）
    Vec3  viewDir{0, -0.3f, -1};     // カメラの既定の向き
    bool  overview = false;          // true: カメラは viewCenter を中心に全体を見せる（速度の向きに回さない）
    Vec3  viewCenter{0, 0, 0};
    float viewDist = 30.0f;
    float killY = -10.0f;
    bool  hasGoal = false;
    Vec3  goalLo, goalHi;
    std::vector<Gate>     gates;
    std::vector<Core>     cores;
    std::vector<Animator> animators;
    std::vector<Swing>    swings;
    std::vector<Steel>    steels;
    std::vector<Rotor>    rotors;
    std::vector<Pad>      pads;
    std::vector<Zone>     zones;
    std::vector<NeedleField> needles;
    std::vector<Cyclotron> cyclotrons;
    std::vector<GaussRing> rings;
    std::vector<BallChain> chains;
    int buildZone = -1;              // 組み立て中の区間（ここで作った扉・磁石などに付ける）

    // 物体ごと（body->id を添字にする）
    std::vector<Kind>  kinds;
    std::vector<float> charge;   // q_i
    std::vector<Vec3>  poleAxis; // N 極の向き（ローカル。磁石はすべて +y）。物どうしの双極子の強さ（0 でもよい）とは別に持つ
    std::vector<float> flash;    // 連鎖で動き出したときに 1（描画で減らす）
    std::vector<char>  moved;
    std::vector<char>  used;     // プレイヤーを重さ以上の力で動かした
    std::vector<int>   coreOf;   // body->id → cores の添字（-1: コアでない）
    std::vector<int>   swingOf;  // body->id → swings の添字
    std::vector<int>   steelOf;  // body->id → steels の添字
    std::vector<int>   padOf;    // body->id → pads の添字
    std::vector<signed char> uniformPole;   // 0 以外: プレイヤーから見て、どの向きでもこの極（鎖の球。曲がっても引き・押しが変わらない）

    // 表示用
    std::vector<PlayerLink> links;
    Vec3  netForce{0, 0, 0};     // 磁力の合力（上限の後）
    Vec3  vortexForce{0, 0, 0};  // 軌道コアの渦・振り子の力（上限の外）
    int   orbitCore  = -1;       // 今回っているコア（-1: なし）
    float orbitAngle = 0.0f;     // そのコアのまわりを回った角度 [rad]
    int   swingActive = -1;      // 綱がつながっている錨（-1: なし）
    float swingAngle  = 0.0f;    // その錨のまわりを回った角度 [rad]
    float comboClock = 0.0f;     // 最後の磁気イベントからの時間
    std::vector<Vec3>       trail, prevTrail;   // 1/30 s ごとの位置（trail[i] は開始から i / 30 s）
    std::vector<int>        trailBreaks, prevBreaks;   // チェックポイントに戻ったところ（この添字の手前で線を切る）
    std::vector<SwitchMark> marks, prevMarks;
    std::vector<float>      inputs, prevInputs; // 押した時刻（開始 = 0 を含む）
    float failTimer = 0.0f;

    // 大きなマップ（チェックポイント）
    int   checkpoint = -1;            // 最後に着いたチェックポイント
    int   checkpointTotal = 0;
    int   zone = -1;                  // プレイヤーのいる区間
    int   seatedPad = -1;             // 座っている発射台
    bool  practice = false;           // 途中のチェックポイントから始めた（記録しない）
    std::vector<float> splits;        // チェックポイントに着いた時刻（splits[k] は k 番目）
    std::vector<float> resets;        // 手でチェックポイントに戻った時刻（リプレイで同じ時刻に戻す）
    std::vector<float> zoneStart;     // 区間の時計を始めた時刻（-1: まだ）

    std::function<void(FlowWorld&)> builder;   // やり直すときに同じステージを組み直す

    // --- ステージを組み立てる関数 ---
    void reset();                                            // 空にする（builder は残す）
    RigidBody* addSolid(const Vec3& pos, const Vec3& he, const Quat& q = Quat{}, float friction = 0.6f,
                        float restitution = 0.1f);
    // 固定磁石（静的）。worldMoment は物どうしの双極子の強さ（ローカル +y が N 極）。
    // 0 にすると、プレイヤーにだけ効き、可動磁石や鉄には効かない（連鎖を乱さない）
    RigidBody* addFixedMagnetSphere(const Vec3& pos, float radius, const Vec3& northDir, float q,
                                    float worldMoment);
    RigidBody* addFixedMagnetBox(const Vec3& pos, const Vec3& he, const Quat& q, float charge, float worldMoment);
    RigidBody* addMovableMagnetBox(const Vec3& pos, float half, const Vec3& northDir, float mass, float q,
                                   float worldMoment);
    RigidBody* addMovableMagnetSphere(const Vec3& pos, float radius, const Vec3& northDir, float mass, float q,
                                      float worldMoment);
    RigidBody* addIronSphere(const Vec3& pos, float radius, float mass, float q = 1.0f);
    RigidBody* addIronBox(const Vec3& pos, const Vec3& he, float mass, const Quat& q = Quat{}, float charge = 1.0f);
    // 軌道コア（プレイヤーにだけ効く。表面はすべりやすい）
    Core&      addCore(const Vec3& pos, float radius, int pole, const Vec3& axis, float q = 1.0f);
    // 磁極（表面のどこでも同じ極の球。渦なし）: 棒磁石の端など
    Core&      addPole(const Vec3& pos, float radius, int pole, float q = 1.0f);
    Gate&      addGate(const Vec3& pos, const Vec3& he, const Vec3& openOffset, const Vec3& triggerLo,
                       const Vec3& triggerHi);
    // 回る扉: pivot を通る axis のまわりに angle [rad] 回って開く
    Gate&      addHingedGate(const Vec3& pos, const Vec3& he, const Quat& q, const Vec3& pivot, const Vec3& axis,
                             float angle, const Vec3& triggerLo, const Vec3& triggerHi);
    void       attachToGate(Gate& g, RigidBody* b);           // 扉と一緒に動かす
    // 動的磁場: 磁石 b（固定磁石）を時間で動かす。設定は戻り値に書く
    Animator&  animate(RigidBody* b);
    void       attachToAnimator(Animator& a, RigidBody* b);
    // 磁気振り子の錨（プレイヤーにだけ効く）
    Swing&     addSwing(const Vec3& pos, float radius, int pole, float length, const Vec3& planeN, float q = 1.0f);
    // 磁化する鋼（mass = 0 で固定）。moment は磁化 1 のときの物どうしの双極子の強さ
    Steel&     addSteelSphere(const Vec3& pos, float radius, float mass, float moment, float q = 1.0f);
    Steel&     addSteelBox(const Vec3& pos, const Vec3& he, float mass, float moment, const Quat& q = Quat{},
                           float charge = 1.0f);
    // 磁気トルクで回る板: 静的な箱 b を pivot / axis のまわりに回す（angle は lo..hi）
    Rotor&     addRotor(RigidBody* b, const Vec3& pivot, const Vec3& axis, float lo, float hi, float inertia);
    void       attachToRotor(Rotor& r, RigidBody* b);
    // 発射台（チェックポイント checkpoint）。上面の中心の上に座る
    Pad&       addPad(const Vec3& pos, const Vec3& he, const Vec3& launch, int launchPolarity, int checkpoint);
    // 台のないチェックポイント（引かず、発射もしない。押すとふつうに切り替わる）。戻ると seat に polarity で置く
    Pad&       addCheckpoint(const Vec3& seat, float radius, int polarity, int checkpoint);
    Zone&      addZone(const std::string& name, const std::string& hint, const Vec3& lo, const Vec3& hi, float killY);
    Cyclotron& addCyclotron(const Vec3& center, const Vec3& axis, const Vec3& gapN);
    GaussRing& addGaussRing(const Vec3& center, const Vec3& axis, float radius, int pole);
    // 鉄球のロープ: topPos の固定磁石から、半径 r・質量 mass の磁石球を n 個、下へつなげる。
    // northDown = +1 なら N 極が下（いちばん下の球の下の面が N）。gamma は球どうしの強さ（接触した 2 個の引力 / 重さ）
    BallChain& addBallChain(const Vec3& topPos, float topRadius, int n, float r, float mass, int northDown, float q,
                            float gamma);
    void       setGoal(const Vec3& lo, const Vec3& hi) { hasGoal = true; goalLo = lo; goalHi = hi; }
    void       finishBuild();                                // プレイヤーを置く

    // --- 遊ぶ ---
    void restart();                  // builder で組み直す（前回の軌跡は残影として残す）
    void press();                    // 開始 / 極性切替 / 発射
    void begin();                    // 発射せずに開始する（発射台の上で時間を進める。調整・テスト用）
    void step(float dt);             // 固定刻み 1 回
    void resetToCheckpoint();        // 手でチェックポイントに戻る（大きなマップ。時刻を resets に残す）
    void warpTo(int checkpoint);     // 練習: そのチェックポイントの発射台から始める（記録しない）
    Vec3 computePlayerForce(std::vector<PlayerLink>* out, float* scale) const;   // 合力（上限の前）
    // 点 p に極性 pol のプレイヤーがいたときの磁力（上限の前。方位磁針の描画にも使う）
    Vec3 forceAt(const Vec3& p, int pol, std::vector<PlayerLink>* out = nullptr) const;
    Vec3 computeVortex() const;                                                   // 軌道コアの渦・振り子の力
    float zoneTime(int z) const;     // 区間 z の時計（動的磁場の時刻）

    Kind kindOf(const RigidBody* b) const { return b->id < (int)kinds.size() ? kinds[b->id] : Kind::Other; }
    bool isMagnet(Kind k) const {
        return k == Kind::FixedMagnet || k == Kind::MovableMagnet || k == Kind::Core || k == Kind::Anchor;
    }
    bool isSteel(const RigidBody* b) const { return b->id < (int)steelOf.size() && steelOf[b->id] >= 0; }
    bool bigMap() const { return checkpointTotal > 0; }   // チェックポイントのある大きなマップ
    bool padActive(int i) const;     // 座って押すと発射する（錠の扉が開いている）
    // サイクロトロンの磁場の中か（面内のずれ rIn と、面からの高さ h も返す）
    bool cyclotronInside(const Cyclotron& c, const Vec3& p, Vec3* rIn = nullptr, float* h = nullptr) const;

private:
    void tag(RigidBody* b, Kind k, float q, const Vec3& axisLocal = Vec3{0, 1, 0});
    void applyPlayerForce();
    void magneticEvent();            // 連続磁気イベントを数える
    void updateOrbit();
    void updateGates(float dt);
    void updateAnimators(float tOffset);
    void updateRotors(float h);
    void updateSteel(float dt);
    void updateSwings(float h);
    void updatePads(float dt);
    void updateZone();
    void updateCyclotrons(float dt);
    void updateRings(float dt);
    void respawn();                  // 最後のチェックポイントの発射台に戻す（時刻はそのまま進む）
    void placeAtCheckpoint(int cp);
    void openBehind(int zoneIndex);  // その区間より前の扉・板を開けておく
    int   stepCount  = 0;
    float prevSpeed  = 0.0f;
    int   lastCapture = -1;          // 最後に「吸着」とみなした物体
    float orbitPrev  = 0.0f;         // 前のステップの、コアのまわりの角度
    float swingPrev  = 0.0f;
    int   lastSeat   = -1;           // 前のステップで座っていた発射台
};

// 磁石の固定を作るときの便利関数: 強さ Gamma（接触した 2 個の引力 / 重さ）から双極子モーメント
float momentForGamma(float gamma, float mass, float contactDist);

} // namespace flow
