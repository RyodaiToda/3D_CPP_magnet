#pragma once
// ---------------------------------------------------------------------------
// world.h : シミュレーション世界（Broadphase ＋ 逐次インパルス法ソルバ ＋ 磁力）
// ---------------------------------------------------------------------------
#include "phys/collision.h"
#include "phys/magnet.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace phys {

// 全エネルギーの内訳（静的剛体は除く）
struct EnergyReport {
    float kinetic = 0.0f, rotational = 0.0f, gravity = 0.0f, magnetic = 0.0f;
    float total() const { return kinetic + rotational + gravity + magnetic; }
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
    void clear();

    // --- 1 ステップ進める ---
    void step(float dt);

    // --- 参照用 ---
    const std::vector<std::unique_ptr<RigidBody>>& getBodies()    const { return bodies_; }
    const std::vector<Manifold>&                   getManifolds() const { return manifolds_; }
    int  bodyCount()    const { return (int)bodies_.size(); }
    int  contactCount() const;

    MagnetSystem&       magnets()       { return magnets_; }
    const MagnetSystem& magnets() const { return magnets_; }
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

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<std::pair<int, int>>        pairs_;
    std::vector<AABB>                       aabbs_;
    std::vector<Manifold>                   manifolds_;
    std::unordered_map<uint64_t, Manifold>  prevManifolds_;
    MagnetSystem                            magnets_;

    int nextId_ = 0;
};

} // namespace phys
