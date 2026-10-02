#pragma once
// ---------------------------------------------------------------------------
// magnet.h : 磁気双極子（磁石・鉄の球と箱）の力とトルク
//
//   磁場は保存しない。源（各磁性体の位置とモーメント）のリストだけを持ち、
//   場が必要になった点でその都度、双極子の式を足し合わせて評価する。
//   単位系は K = mu0 / 4pi = 1。
//
//   すべての磁性体のモーメントは 1 本の式で決まる（磁石と鉄で分岐しない）:
//       m = R (p + chi * (R^T B))          ただし |m| <= saturation（0 なら上限なし）
//         p   : 永久モーメント（ローカル座標）      磁石 = m0 * y、鉄 = 0
//         chi : 磁化率（ローカル主軸の対角成分）    磁石 = 0、    鉄 = alpha（形で決まる）
//         B   : 他の磁性体と外部磁場がこの位置に作る場
//   力とトルクも共通:  F = (grad B) m,  tau = m x B
//
//   球: 一様に磁化した球の外の場は、中心の双極子 1 個と厳密に一致する。
//   箱: 2x2x2 の点に p, chi, saturation を 1/8 ずつ分けて持たせる。点はローカルで
//       ±boxPointScale * halfExtents に置く（0.4 = 辺の ±0.2 倍）。この位置は、一様に
//       磁化した立方体の厳密な場（表面磁荷モデル）に合わせて決めた。接触した 2 個の
//       引力は誤差 1%（中心 1 点なら +17%、セルの中心 ±0.25 に置くと +30%）。
//       点の組を使うのは近いペア（中心間 < nearScale * 外接球の半径の和）だけで、
//       離れていれば中心の双極子 1 個にまとめる（どちらでも誤差は 3% 未満）。
//   外部磁場: どこでも同じ B0（externalField）。磁石は向きがそろうだけで、鉄は磁化される
//
//   World が MagnetSystem を 1 つ持ち、World::step の各サブステップの先頭で
//   applyForces() を呼ぶ（磁石は World::createMagnet / createSoftMagnet で作る）。
// ---------------------------------------------------------------------------
#include "phys/collision.h"

#include <vector>

namespace phys {

// ---- 双極子の純粋関数（r: 双極子 → 観測点）--------------------------------
Vec3  dipoleField   (const Vec3& r, const Vec3& m);                   // B
Mat3  dipoleGradient(const Vec3& r, const Vec3& m);                   // G_ij = dB_i/dr_j（対称・トレース 0）
float dipoleEnergy  (const Vec3& r, const Vec3& m1, const Vec3& m2);  // r = x2 - x1
Vec3  dipoleForce   (const Vec3& r, const Vec3& m1, const Vec3& m2);  // 2 が受ける力 = G(r, m1) * m2

// 強さの無次元数 Gamma = (一直線に接触した 2 球の引力) / (1 球の重さ)
float momentFromGamma  (float gamma, float mass, float contactDist, float g);  // sqrt(Gamma M g d^4 / 6)
float gammaFromMaterial(float Br, float rho, float radius, float g = 9.81f);  // Br^2 / (8 mu0 rho g R)

// 誘導磁化の係数 alpha（m = alpha B）。chi は材料の体積磁化率（SI、鉄なら数百〜数千）
//   alpha_i = V / (4 pi) * chi / (1 + N_i chi)   N_i は反磁場係数（形だけで決まり、和は 1）
//   chi -> 無限大 でも V / (4 pi N_i) で頭打ちになる（球なら r^3）
Vec3  demagFactors(const Vec3& halfExtents);               // 直方体の反磁場係数（Aharoni 1998）
float sphereSusceptibility(float radius, float chi);       // 球: r^3 chi / (3 + chi)
Vec3  boxSusceptibility(const Vec3& halfExtents, float chi);

// ---- ほかの源の場（電流の導線など）。MagnetSystem::extraField につなぐと、各点の場に足される ----
struct FieldSource {
    virtual ~FieldSource() = default;
    virtual Vec3 fieldAt(const Vec3& p) const = 0;
};

// ---- 磁性体 ---------------------------------------------------------------
constexpr int MAX_MAGNET_POINTS = 8;

struct MagneticBody {
    RigidBody* body = nullptr;          // 球または箱
    Vec3  permanentLocal{0, 0, 0};      // p（物体全体）
    Vec3  chiLocal{0, 0, 0};            // chi（物体全体）
    float saturation = 0.0f;            // |m| の上限（物体全体。0 = 無制限）

    // 双極子を置く点（球は中心 1 点、箱は 2x2x2）。p, chi, saturation は点ごとに 1/count
    int   pointCount = 1;
    Vec3  pointLocal[MAX_MAGNET_POINTS] = {};
    float pointRadius = 0.0f;           // 点から表面までの最短距離（近距離クランプ用）
    float boundRadius = 0.0f;           // 外接球の半径（近い・遠いの判定用）

    Vec3  m{0, 0, 0};                   // 現在のモーメント（ワールド、点の和）
    Vec3  B{0, 0, 0};                   // この物体の位置の場（点の平均。外部磁場を含む）
    Vec3  pointM[MAX_MAGNET_POINTS] = {};   // 点ごとのモーメント
    Vec3  pointB[MAX_MAGNET_POINTS] = {};   // 点ごとの場（最後に評価した値）
};

class MagnetSystem {
public:
    float minDistScale  = 0.9f;  // 近距離クランプ: r_eff = max(r, minDistScale * (R1 + R2))
    float nearScale     = 2.0f;  // 中心間 < nearScale * (外接球の半径の和) なら箱を点の組で計算
    float boxPointScale = 0.4f;  // 箱の点の位置（ローカルで ±boxPointScale * halfExtents）
    Vec3  externalField{0, 0, 0};  // 一様な外部磁場 B0
    // ほかの源の場（電流の導線。World が毎サブステップ設定する）。各点の場に足し、エネルギーには
    // B0 と同じく -1/2 sum m . B_extra を数える。トルクと力は源の側（CurrentSystem）が加える
    const FieldSource* extraField = nullptr;

    // 磁石も砂鉄も同じ関数で登録する。戻り値は bodies() の添字
    int  add(RigidBody* body, const Vec3& permanentLocal,
             const Vec3& chiLocal = Vec3{0, 0, 0}, float saturation = 0.0f);
    void clear();
    int  indexOf(const RigidBody* body) const;   // bodies() の添字。磁性体でなければ -1

    // 各サブステップの先頭で呼ぶ。前回の B から m を更新し（Jacobi 1 回）、
    // 全ペアの力とトルクを body に加算して、次回用の B を求める
    void applyForces();

    // 現在位置で B を評価し直して m を解く。戻り値は最後の反復での |m| の最大変化量
    float updateMoments(int iterations);

    // U = -1/2 sum p . B - 1/2 sum m . B0（B は外部磁場を含む。現在位置で評価し直す）
    float potentialEnergy() const;
    Vec3  fieldAt(const Vec3& p) const;     // 任意の点の B（全磁性体の和 + 外部磁場）

    // 磁性体どうしの接触グラフを連結成分に分け、鎖・輪などに分類する
    enum class StructureType { Single, Chain, Ring, Other };
    struct Structure {
        StructureType    type = StructureType::Single;
        std::vector<int> members;           // bodies() の添字
    };
    std::vector<Structure> structures(const std::vector<Manifold>& manifolds) const;

    const std::vector<MagneticBody>& bodies() const { return bodies_; }
    std::vector<MagneticBody>&       bodies()       { return bodies_; }

private:
    // 点 k のモーメント（p, chi, saturation の 1/count と、点の場 B から）
    static Vec3 pointMoment(const MagneticBody& mb, const Mat3& R, const Vec3& B);
    // 各点の位置と、各点に保存した場から求めたモーメントを作業配列に写す
    void gather() const;
    // 作業配列の全ペアを評価して、各点の場（外部磁場を含む）と、withForces なら力・トルクを求める
    void evaluate(bool withForces) const;
    // 作業配列の場から各点のモーメントを求め直す。戻り値は |m| の最大変化量
    float remoment() const;
    // 作業配列のモーメントと場を bodies_ に書き戻す
    void store();

    std::vector<MagneticBody> bodies_;
    std::vector<int>          indexById_;   // body->id → bodies_ の添字（-1 = 磁性体でない）

    // 作業配列（毎回の確保を避ける）。物体ごと: 中心・合計モーメント・力・トルクなど。
    // 点ごと: 位置・モーメント・場（first_[i] から count 個が物体 i の点）
    mutable std::vector<Vec3>  xc_, mTot_, F_, T_, Bfar_;
    mutable std::vector<float> clampR_, boundR_, pointR_;
    mutable std::vector<int>   first_, count_;
    mutable std::vector<char>  fixed_;
    mutable std::vector<Vec3>  xp_, mp_, Bp_;
};

} // namespace phys
