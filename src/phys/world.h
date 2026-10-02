#pragma once
// ---------------------------------------------------------------------------
// world.h : シミュレーション世界（Broadphase ＋ 逐次インパルス法ソルバ ＋ 磁力 ＋ 電気の力）
// ---------------------------------------------------------------------------
#include "phys/collision.h"
#include "phys/current.h"
#include "phys/electric.h"
#include "phys/magnet.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace phys {

// 全エネルギーの内訳（静的剛体は除く）
struct EnergyReport {
    float kinetic = 0.0f, rotational = 0.0f, gravity = 0.0f, magnetic = 0.0f, electric = 0.0f;
    float total() const { return kinetic + rotational + gravity + magnetic + electric; }
};

class World {
public:
    // --- チューニングパラメータ ---
    Vec3  gravity{0.0f, -9.81f, 0.0f};
    int   velocityIterations = 10;    // 逐次インパルスの反復回数
    int   positionIterations = 3;     // めり込み補正の反復回数
    float penetrationSlop    = 0.005f; // 許容するめり込み量（ジッタ防止）
    float positionCorrection = 0.4f;  // 1 反復で解消する割合
    float restitutionThreshold = 1.0f; // これ未満の接近速度では反発させない
    bool  warmStarting       = true;
    bool  enableGyroscopic   = true;  // 自由回転でのジャイロ効果（角運動量保存）
    int   substeps           = 1;     // step(dt) を dt/substeps の刻みで substeps 回まわす（強い磁力用）

    // --- 物体の生成 ---
    RigidBody* createBox(const Vec3& position, const Vec3& halfExtents, float mass);
    RigidBody* createSphere(const Vec3& position, float radius, float mass);
    // 磁石球: moment はワールド座標の永久モーメント（ローカル +y がこの向きになるよう姿勢を決める）
    RigidBody* createMagnet(const Vec3& position, float radius, float mass, const Vec3& moment);
    // 誘導磁化する球（砂鉄など）: 等方的な磁化率 alpha、|m| の上限 saturation（0 = 無制限）
    RigidBody* createSoftMagnet(const Vec3& position, float radius, float mass,
                                float alpha, float saturation = 0.0f);
    // 箱の磁石: momentLocal はローカル座標の永久モーメント（例: {0, m, 0} で +y 面が N 極）
    RigidBody* createMagnetBox(const Vec3& position, const Vec3& halfExtents, float mass,
                               const Vec3& momentLocal, const Quat& orientation = Quat{});
    // 誘導磁化する箱（鉄のブロック・釘など）: alphaLocal は boxSusceptibility で形から求める
    RigidBody* createSoftMagnetBox(const Vec3& position, const Vec3& halfExtents, float mass,
                                   const Vec3& alphaLocal, float saturation = 0.0f,
                                   const Quat& orientation = Quat{});
    // 帯電した球: charge は電荷（chargeFromGamma で決める）。conductor なら分極し、触れた導体・板と電荷をやり取りする
    RigidBody* createChargedSphere(const Vec3& position, float radius, float mass, float charge, bool conductor = false);
    // 渦電流が流れる導体（銅など）: sigma は導電率、spacing は電流を求める点の間隔（0 なら大きさから決める）
    RigidBody* createConductorSphere(const Vec3& position, float radius, float mass, float sigma, float spacing = 0.0f);
    RigidBody* createConductorBox(const Vec3& position, const Vec3& halfExtents, float mass, float sigma,
                                  const Quat& orientation = Quat{}, float spacing = 0.0f);
    // 平行板のコンデンサ: 中心 center、ローカル +y が板 A → 板 B の向き。板は halfX x halfZ、すき間 gap、厚さ thickness。
    // 静的な 2 枚の板を作り、間の箱の中に一様な電場 (V_A - V_B) / gap を置く。戻り値は capacitors() の添字
    int createCapacitor(const Vec3& center, const Quat& orientation, float halfX, float halfZ, float gap, float thickness,
                        float potentialA, float potentialB);
    void clear();

    // 位置を今の場所に固定し、回転だけ自由にする（方位磁針の軸受け）。false で外す。
    // 各サブステップで並進速度を 0 にし、位置を固定点に戻す（軸受けの力は仕事をしない）
    void setPinned(RigidBody* body, bool pinned);
    bool isPinned(const RigidBody* body) const;

    // --- 1 ステップ進める ---
    void step(float dt);

    // --- 参照用 ---
    const std::vector<std::unique_ptr<RigidBody>>& getBodies()    const { return bodies_; }
    const std::vector<Manifold>&                   getManifolds() const { return manifolds_; }
    int  bodyCount()    const { return (int)bodies_.size(); }
    int  contactCount() const;

    MagnetSystem&       magnets()       { return magnets_; }
    const MagnetSystem& magnets() const { return magnets_; }
    ElectricSystem&       electric()       { return electric_; }
    const ElectricSystem& electric() const { return electric_; }
    // 電流の導線・コイル（world.currents().addCoil(...) などで足す。電流は毎ステップ書き換えてよい）
    CurrentSystem&        currents()       { return currents_; }
    const CurrentSystem&  currents() const { return currents_; }
    std::vector<MagnetSystem::Structure> magneticStructures() const { return magnets_.structures(manifolds_); }
    EnergyReport energy() const;

private:
    void substep(float h);
    void integrateForces(float dt);
    void broadphase();
    void narrowphase();
    void warmStart();
    void prepareConstraints(float dt);
    void solveVelocities();
    void integratePositions(float dt);
    void correctPositions();
    void holdPinned(bool resetPosition);

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<std::pair<int, int>>        pairs_;
    std::vector<int>                        dynamic_;      // 動く物体の添字（broadphase の作業用）
    std::vector<AABB>                       aabbs_;
    std::vector<Manifold>                   manifolds_;
    std::unordered_map<uint64_t, Manifold>  prevManifolds_;
    MagnetSystem                            magnets_;
    ElectricSystem                          electric_;
    CurrentSystem                           currents_;
    std::vector<Vec3>                       preVelocity_, preAngularVelocity_;   // 力を積分する前の速度
    std::vector<char>                       pinned_;       // body->id → 位置を固定しているか
    std::vector<Vec3>                       pinAnchor_;    // body->id → 固定点

    int nextId_ = 0;
};

} // namespace phys
