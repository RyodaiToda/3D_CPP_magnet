#pragma once
// ---------------------------------------------------------------------------
// magnet.h : 磁気双極子（磁石球・砂鉄）の力とトルク
//
//   磁場は保存しない。源（各磁性体の位置とモーメント）のリストだけを持ち、
//   場が必要になった点でその都度、双極子の式を足し合わせて評価する。
//   単位系は K = mu0 / 4pi = 1。
//
//   すべての磁性体のモーメントは 1 本の式で決まる（磁石と砂鉄で分岐しない）:
//       m = R (p + chi * (R^T B))          ただし |m| <= saturation（0 なら上限なし）
//         p   : 永久モーメント（ローカル座標）      磁石 = m0 * y、砂鉄 = 0
//         chi : 磁化率（ローカル主軸の対角成分）    磁石 = 0、    砂鉄 = alpha * (1,1,1)
//         B   : 他の磁性体がこの位置に作る場
//   力とトルクも共通:  F = (grad B) m,  tau = m x B
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

// ---- 磁性体 ---------------------------------------------------------------
struct MagneticBody {
    RigidBody* body = nullptr;          // 球を想定（近距離クランプに半径を使う）
    Vec3  permanentLocal{0, 0, 0};      // p
    Vec3  chiLocal{0, 0, 0};            // chi
    float saturation = 0.0f;            // |m| の上限（0 = 無制限）

    Vec3  m{0, 0, 0};                   // 現在のモーメント（ワールド）
    Vec3  B{0, 0, 0};                   // 他の磁性体がこの位置に作る場（最後に評価した値）
};

class MagnetSystem {
public:
    float minDistScale = 0.9f;  // 近距離クランプ: r_eff = max(r, minDistScale * (R1 + R2))

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

    float potentialEnergy() const;          // U = -1/2 sum p . B（現在位置で評価し直す）
    Vec3  fieldAt(const Vec3& p) const;     // 任意の点の B（全磁性体の和）

    // 接触した 2 つの磁性体が引き合う相対加速度の上限 6|ma||mb|/(Ra+Rb)^4 * (1/Ma + 1/Mb)。
    // どちらかが磁性体でなければ 0（接触ソルバの反発しきい値に使う）
    float contactPull(const RigidBody* a, const RigidBody* b) const;

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
    Vec3 momentFor(const MagneticBody& mb, const Vec3& B) const;
    void gatherState(std::vector<Vec3>& x, std::vector<float>& rad, std::vector<Vec3>& m) const;

    std::vector<MagneticBody> bodies_;
    std::vector<int>          indexById_;   // body->id → bodies_ の添字（-1 = 磁性体でない）

    // applyForces の作業配列（毎回の確保を避ける）
    std::vector<Vec3>  x_, m_, F_, T_, B_;
    std::vector<float> rad_;
    std::vector<char>  fixed_;
};

} // namespace phys
