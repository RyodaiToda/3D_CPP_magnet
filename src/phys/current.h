#pragma once
// ---------------------------------------------------------------------------
// current.h : 決めた電流の導線とコイル（ビオ・サバール）
//
//   電流 I の値はこちらで決める（毎ステップ書き換えてよい。スイッチ）。誘導で変わることはない。
//   単位は磁気と同じ K = mu0 / 4pi = 1（輪の中心の場 = 2 pi I / a、長い直線のそばの場 = 2 I / d）。
//
//   導線は折れ線（頂点はローカル座標）。剛体に付けるか、body = nullptr で静的に置く
//   （見えない物体は作らない。コイルガンの球がコイルの中心を通れるように）。
//
//   場: 有限の直線 a -> b が点 x に作る場は厳密な式で求める（r_a = x - a、r_b = x - b）
//       B = I (|r_a| + |r_b|) (r_a x r_b) / ( |r_a| |r_b| (|r_a| |r_b| + r_a . r_b) )
//     （Hanson–Hirshman の形。cos t1 - cos t2 の式と同じ値で、軸の延長上でも安定）。
//     導線より近い点は、導線から radius の距離に置き直す（太さの中の場は扱わない）。
//
//   円いコイル（addCoil）の場は、正多角形の線分の和ではなく、円の厳密な式（完全楕円積分）で求める（速くて正確）。
//
//   力（applyForces。磁力の後、電気の前）。作用・反作用で返す:
//     - 磁性体の点 k（モーメント m_k）と円いコイル: F_k = grad(m_k . B) を円の場の中心差分で求め、x_k に加える。
//       コイルの物体には -F_k と、角運動量が保存するトルク -(m_k x B) - (x_k - c) x F_k（閉じた輪では厳密）
//     - 磁性体の点と折れ線の導線: 線分の求積点 x_q に f_q = I dl x B_dip(x_q; m_k)。
//       導線の物体に f_q を x_q で加え、磁性体には F_k = -sum f_q を x_k で加え、トルク m_k x B_wire(x_k) を足す。
//       （閉じた輪と厳密な場では F_dipole = grad(m . B) = -F_loop なので、この置き方が正しい。運動量は厳密に保存する）
//     - 導線 a と b: a の求積点に f = I_a dl x B_b。b には F_b = -F_a、tau_b = -tau_a - (c_a - c_b) x F_a
//       （閉じた輪なら角運動量も厳密に保存する。開いた導線には第三法則が成り立たないので、動く物体には閉じた輪だけを付ける）
//     - 一様な外部磁場 B0: 線分ごとに I (b - a) x B0 を中点に加える（一様な場では厳密）
//
//   鉄が電磁石の場で磁化するように、MagnetSystem::extraField（FieldSource）としてつなぐ。
//   MagnetSystem は各点の場に加えるだけで、トルクはここで加える（二重に数えない）。
//
//   World が CurrentSystem を 1 つ持ち、各サブステップの先頭で prepare()（ワールド座標の頂点）、
//   磁力の後で applyForces() を呼ぶ。
// ---------------------------------------------------------------------------
#include "phys/magnet.h"

#include <vector>

namespace phys {

// 強さの補助: 正 n 角形を円とみなしたときの、巻き数 turns のコイルの中心の場 2 pi I turns / a
float coilCenterField(float current, float radius, int turns = 1);
float currentForCoilCenterField(float field, float radius, int turns = 1);

// 有限の直線 a -> b（電流 current、太さ clampRadius）が点 p に作る場
Vec3 segmentField(const Vec3& a, const Vec3& b, const Vec3& p, float current, float clampRadius = 0.0f);
// 円い輪（中心 c、軸 n（単位ベクトル）、半径 a、電流 current、太さ clampRadius）が点 p に作る場（完全楕円積分で厳密）。
// distOut には輪（の線）までの距離を入れる
Vec3 loopField(const Vec3& c, const Vec3& n, float a, const Vec3& p, float current, float clampRadius = 0.0f, float* distOut = nullptr);

struct Wire {
    RigidBody* body = nullptr;         // 付ける物体（nullptr なら静的: origin / orientation の姿勢）
    Vec3  origin{0, 0, 0};
    Quat  orientation{};
    std::vector<Vec3> pointsLocal;     // 折れ線の頂点（body のローカル座標。body がなければ origin / orientation から）
    bool  closed  = true;
    float current = 0.0f;              // I（+ は頂点の順の向き）
    int   turns   = 1;                 // 巻き数（場と力には current * turns を使う）
    float radius  = 0.03f;             // 太さ（描画と近距離のクランプ）

    // 円いコイル（addCoil）: 場は正多角形の線分の和ではなく、円の厳密な式（楕円積分）で求める（速くて正確）。
    // 力の求積点には線分を使う
    bool  circle = false;
    Vec3  circleCenterLocal{0, 0, 0};
    Vec3  circleAxisLocal{0, 1, 0};
    float circleRadius = 0.0f;

    std::vector<Vec3> points;          // ワールド座標の頂点（prepare で更新）
    Vec3  circleCenter{0, 0, 0};       // ワールド座標の円の中心と軸（prepare で更新）
    Vec3  circleAxis{0, 1, 0};

    float effectiveCurrent() const { return current * (float)turns; }
    int   segmentCount() const { return closed ? (int)points.size() : (int)points.size() - 1; }
    void  segment(int s, Vec3& a, Vec3& b) const {
        a = points[s];
        b = points[(s + 1) % points.size()];
    }
    bool  dynamic() const { return body && !body->isStatic(); }
    Vec3  center() const { return body ? body->position : origin; }
    Vec3  velocity() const { return body ? body->velocity : Vec3{0, 0, 0}; }
    Vec3  areaVector() const;          // 閉じた輪の面積ベクトル（1/2 sum x_i x x_{i+1}、中心まわり）
};

class CurrentSystem : public FieldSource {
public:
    float minDistScale = 0.9f;   // 磁性体の点と求積点の近距離クランプ（磁石と同じ）

    // 導線を登録する。center / orientation は body のローカル座標（body がなければワールド）。戻り値は wires() の添字
    int addWire(RigidBody* body, const std::vector<Vec3>& pointsLocal, bool closed, float current, int turns = 1,
                float wireRadius = 0.03f, const Vec3& origin = Vec3{0, 0, 0}, const Quat& orientation = Quat{});
    // 正 segments 角形のコイル（ローカル +y が軸。+current で +y 向きの場）
    int addCoil(const Vec3& center, const Quat& orientation, float radius, int segments, float current, int turns = 1,
                RigidBody* body = nullptr, float wireRadius = 0.03f);
    // ソレノイド: 長さ length に rings 個の輪を等間隔に重ねる（らせんにはしない）。戻り値は最初の輪の添字
    int addSolenoid(const Vec3& center, const Quat& orientation, float radius, float length, int rings, int segments,
                    float current, int turnsPerRing = 1, RigidBody* body = nullptr, float wireRadius = 0.03f);
    void clear();
    bool empty() const { return wires_.empty(); }
    int  indexOf(const RigidBody* body) const;   // その物体に付いた最初の導線の添字（なければ -1）

    void prepare();                              // ワールド座標の頂点を更新する（各サブステップの先頭。手で物体を動かしたら呼ぶ）
    Vec3 fieldAt(const Vec3& p) const override;  // 全導線の場
    Vec3 fieldOfWire(int w, const Vec3& p) const;
    void applyForces(MagnetSystem& magnets);     // 磁性体・導線・外部磁場 <-> 導線（body->force / torque に加算）

    std::vector<Wire>&       wires()       { return wires_; }
    const std::vector<Wire>& wires() const { return wires_; }

private:
    std::vector<Wire> wires_;
    std::vector<int>  indexById_;   // body->id -> 最初の導線の添字（-1 = なし）
};

} // namespace phys
