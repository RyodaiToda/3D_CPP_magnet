// ---------------------------------------------------------------------------
// world.cpp
//
// 1 ステップは dt / substeps の刻みで substeps 回のサブステップに分ける。
// 1 サブステップの流れ：
//   0. 磁力（磁性体の力・トルクを body->force / torque に加算）
//   1. 外力の積分（重力・力・トルク → 速度）
//   2. Broadphase（AABB の総当たり）
//   3. Narrowphase（接触マニフォールド生成）
//   4. ウォームスタート（前フレームのインパルスを再利用）
//   5. 制約の前計算（実効質量・反発バイアス）
//   6. 速度の反復解法（法線インパルス ＋ 摩擦インパルス）
//   7. 位置の積分
//   8. めり込みの位置補正
// substeps = 1 で磁性体がなければ、サブステップのない 1 ステップと同じになる。
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <cmath>

namespace phys {

// ---------------------------------------------------------------------------
// ジャイロスコープ項（オイラーの運動方程式 I*w' = -w x (I*w)）
//   陽解法だと発散しやすいので、Bullet と同じく物体座標系での陰的 1 段
//   ニュートン法で解く。これがないと細長い剛体の自由回転で角運動量が保存されない。
//     f(w) = I*(w_new - w) + dt * w_new x (I*w_new) = 0
//     J    = I + dt * ( skew(w) * I - skew(I*w) )
// ---------------------------------------------------------------------------
static Vec3 gyroscopicDelta(const RigidBody* b, float dt) {
    Mat3 R  = b->orientation.toMat3();
    Mat3 Rt = R.transposed();

    Vec3 wBody = Rt * b->angularVelocity;
    Vec3 Ilocal{ 1.0f / b->invInertiaLocal.x,
                 1.0f / b->invInertiaLocal.y,
                 1.0f / b->invInertiaLocal.z };
    Mat3 I  = Mat3::diagonal(Ilocal);
    Vec3 Iw = Vec3{ Ilocal.x * wBody.x, Ilocal.y * wBody.y, Ilocal.z * wBody.z };

    Vec3 f = cross(wBody, Iw) * dt;
    Mat3 J = I + (Mat3::skew(wBody) * I - Mat3::skew(Iw)) * dt;

    Vec3 delta = J.inverse() * f;   // w_body_new = w_body - delta
    return R * (-delta);            // ワールド座標での角速度変化
}

// ===========================================================================
// 物体の生成
// ===========================================================================
RigidBody* World::createBox(const Vec3& position, const Vec3& halfExtents, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id       = nextId_++;
    body->position = position;
    body->shape    = Shape::box(halfExtents);
    body->setMass(mass);
    RigidBody* raw = body.get();
    bodies_.push_back(std::move(body));
    return raw;
}

RigidBody* World::createSphere(const Vec3& position, float radius, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id       = nextId_++;
    body->position = position;
    body->shape    = Shape::sphere(radius);
    body->setMass(mass);
    RigidBody* raw = body.get();
    bodies_.push_back(std::move(body));
    return raw;
}

// ローカル +y を方向 d に向ける姿勢（d が 0 なら回さない）
static Quat orientationFromY(const Vec3& d) {
    if (lengthSq(d) < 1e-12f) return Quat{};
    Vec3  y{0, 1, 0};
    Vec3  n = normalize(d);
    float c = dot(y, n);
    if (c > 0.99999f)  return Quat{};
    if (c < -0.99999f) return Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
    return Quat::fromAxisAngle(cross(y, n), std::acos(c));
}

RigidBody* World::createMagnet(const Vec3& position, float radius, float mass, const Vec3& moment) {
    RigidBody* body = createSphere(position, radius, mass);
    body->orientation = orientationFromY(moment);
    magnets_.add(body, Vec3{0, length(moment), 0});
    return body;
}

RigidBody* World::createSoftMagnet(const Vec3& position, float radius, float mass,
                                   float alpha, float saturation) {
    RigidBody* body = createSphere(position, radius, mass);
    magnets_.add(body, Vec3{0, 0, 0}, Vec3{alpha, alpha, alpha}, saturation);
    return body;
}

void World::clear() {
    bodies_.clear();
    manifolds_.clear();
    prevManifolds_.clear();
    pairs_.clear();
    aabbs_.clear();
    magnets_.clear();
    nextId_ = 0;
}

EnergyReport World::energy() const {
    EnergyReport e;
    for (const auto& b : bodies_) {
        if (b->isStatic()) continue;
        float M = 1.0f / b->invMass;
        e.kinetic += 0.5f * M * lengthSq(b->velocity);

        Vec3 w = b->orientation.toMat3().transposed() * b->angularVelocity;  // 物体座標
        e.rotational += 0.5f * (w.x * w.x / b->invInertiaLocal.x +
                                w.y * w.y / b->invInertiaLocal.y +
                                w.z * w.z / b->invInertiaLocal.z);
        e.gravity -= M * dot(gravity, b->position);
    }
    e.magnetic = magnets_.potentialEnergy();
    return e;
}

int World::contactCount() const {
    int n = 0;
    for (const auto& m : manifolds_) n += m.count;
    return n;
}

// ===========================================================================
// 1. 外力の積分
// ===========================================================================
void World::integrateForces(float dt) {
    for (auto& b : bodies_) {
        b->updateInertiaWorld();
        if (b->isStatic()) {
            b->velocity = Vec3{0, 0, 0};
            b->angularVelocity = Vec3{0, 0, 0};
            b->force = Vec3{0, 0, 0};
            b->torque = Vec3{0, 0, 0};
            continue;
        }
        b->velocity        += (gravity + b->force * b->invMass) * dt;
        b->angularVelocity += b->invInertiaWorld * b->torque * dt;

        if (enableGyroscopic && lengthSq(b->angularVelocity) > 1e-8f)
            b->angularVelocity += gyroscopicDelta(b.get(), dt);

        // 指数的な減衰（時間刻みに依存しにくい形）
        b->velocity        *= 1.0f / (1.0f + dt * b->linearDamping);
        b->angularVelocity *= 1.0f / (1.0f + dt * b->angularDamping);
    }
}

// ===========================================================================
// 2. Broadphase : AABB の総当たり（数百体までなら十分実用的）
// ===========================================================================
void World::broadphase() {
    const int n = (int)bodies_.size();
    aabbs_.resize(n);
    for (int i = 0; i < n; ++i) {
        aabbs_[i] = bodies_[i]->computeAABB();
        aabbs_[i].expand(0.02f); // 少し膨らませて接触の取りこぼしを防ぐ
    }

    pairs_.clear();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (bodies_[i]->isStatic() && bodies_[j]->isStatic()) continue;
            if (!aabbs_[i].overlaps(aabbs_[j])) continue;
            pairs_.emplace_back(i, j);
        }
    }
}

// ===========================================================================
// 3. Narrowphase
// ===========================================================================
void World::narrowphase() {
    manifolds_.clear();
    Manifold m;
    for (const auto& p : pairs_) {
        m = Manifold{};
        if (collide(bodies_[p.first].get(), bodies_[p.second].get(), m))
            manifolds_.push_back(m);
    }
}

// ===========================================================================
// 4. ウォームスタート
//    前フレームの同じ接触点を局所座標の近さで探し、蓄積インパルスを引き継ぐ。
//    これがないと積み重ねがすぐ崩れる。
// ===========================================================================
static inline uint64_t pairKey(int a, int b) {
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

void World::warmStart() {
    if (!warmStarting) { prevManifolds_.clear(); return; }

    const float matchDistSq = 0.01f * 0.01f;

    for (auto& m : manifolds_) {
        auto it = prevManifolds_.find(pairKey(m.a->id, m.b->id));
        if (it == prevManifolds_.end()) continue;
        const Manifold& old = it->second;

        for (int i = 0; i < m.count; ++i) {
            for (int j = 0; j < old.count; ++j) {
                if (lengthSq(m.contacts[i].localA - old.contacts[j].localA) < matchDistSq &&
                    lengthSq(m.contacts[i].localB - old.contacts[j].localB) < matchDistSq) {
                    m.contacts[i].normalImpulse     = old.contacts[j].normalImpulse;
                    m.contacts[i].tangentImpulse[0] = old.contacts[j].tangentImpulse[0];
                    m.contacts[i].tangentImpulse[1] = old.contacts[j].tangentImpulse[1];
                    break;
                }
            }
        }
    }
}

// ===========================================================================
// 5. 制約の前計算
// ===========================================================================
// 方向 d に沿った接触の実効質量（の逆数）
//   K = 1/mA + 1/mB + (IA^-1 (rA x d)) x rA . d + (IB^-1 (rB x d)) x rB . d
static inline float effectiveMass(const RigidBody* A, const RigidBody* B,
                                  const Vec3& rA, const Vec3& rB, const Vec3& d) {
    Vec3 rAxd = cross(rA, d);
    Vec3 rBxd = cross(rB, d);
    float k = A->invMass + B->invMass
            + dot(A->invInertiaWorld * rAxd, rAxd)
            + dot(B->invInertiaWorld * rBxd, rBxd);
    return (k > 1e-9f) ? 1.0f / k : 0.0f;
}

void World::prepareConstraints(float dt) {
    for (auto& m : manifolds_) {
        RigidBody* A = m.a;
        RigidBody* B = m.b;

        for (int i = 0; i < m.count; ++i) {
            Contact& c = m.contacts[i];
            c.rA = c.position - A->position;
            c.rB = c.position - B->position;

            c.normalMass     = effectiveMass(A, B, c.rA, c.rB, m.normal);
            c.tangentMass[0] = effectiveMass(A, B, c.rA, c.rB, m.tangent[0]);
            c.tangentMass[1] = effectiveMass(A, B, c.rA, c.rB, m.tangent[1]);

            // 反発：接近速度が十分大きいときだけ跳ね返す（微振動を防ぐ）
            //   磁性体どうしの接触では、引き合う力が 1 刻みで与える接近速度の 2 倍を
            //   しきい値に足す。接触がわずかに離れた 1 刻みで得た引き込み速度を「衝突」と
            //   して跳ね返すと、くっついた磁石が毎刻み震え続けるため。
            //   磁性体どうしでなければ contactPull は 0 で、従来と同じ。
            Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
            float vn     = dot(relVel, m.normal);
            float threshold = restitutionThreshold + 2.0f * magnets_.contactPull(A, B) * dt;
            c.velocityBias = (vn < -threshold) ? m.restitution * vn : 0.0f;
        }

        // 前フレームから引き継いだインパルスを先に適用しておく
        if (warmStarting) {
            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];
                Vec3 P = m.normal     * c.normalImpulse
                       + m.tangent[0] * c.tangentImpulse[0]
                       + m.tangent[1] * c.tangentImpulse[1];
                A->applyImpulse(-P, c.rA);
                B->applyImpulse( P, c.rB);
            }
        }
    }
}

// ===========================================================================
// 6. 速度の反復解法
// ===========================================================================
void World::solveVelocities() {
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (auto& m : manifolds_) {
            RigidBody* A = m.a;
            RigidBody* B = m.b;

            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];

                // ---- 摩擦（先に解くと安定しやすい） ----
                float maxFriction = m.friction * c.normalImpulse;
                for (int k = 0; k < 2; ++k) {
                    Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                    float vt     = dot(relVel, m.tangent[k]);
                    float lambda = -vt * c.tangentMass[k];

                    float oldImpulse = c.tangentImpulse[k];
                    float newImpulse = clampf(oldImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - oldImpulse;
                    c.tangentImpulse[k] = newImpulse;

                    Vec3 P = m.tangent[k] * lambda;
                    A->applyImpulse(-P, c.rA);
                    B->applyImpulse( P, c.rB);
                }

                // ---- 法線方向（めり込みを押し返す） ----
                Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                float vn     = dot(relVel, m.normal);
                float lambda = -(vn + c.velocityBias) * c.normalMass;

                // 蓄積インパルスは常に非負（接触は引っ張れない）
                float oldImpulse = c.normalImpulse;
                float newImpulse = std::max(oldImpulse + lambda, 0.0f);
                lambda = newImpulse - oldImpulse;
                c.normalImpulse = newImpulse;

                Vec3 P = m.normal * lambda;
                A->applyImpulse(-P, c.rA);
                B->applyImpulse( P, c.rB);
            }

            // ---- 転がり摩擦 / スピン摩擦 --------------------------------
            //   滑り摩擦だけでは「転がる球」は永久に止まらない（接触点の
            //   相対速度が 0 なので仕事をしない）。そこで接触点まわりの
            //   相対角速度に対しても、法線力に比例した上限つきの角インパルス
            //   を掛けて抵抗させる。
            if (m.rollingFriction > 0.0f) {
                float totalPn = 0.0f;
                for (int i = 0; i < m.count; ++i) totalPn += m.contacts[i].normalImpulse;
                float maxRoll = m.rollingFriction * totalPn;

                if (maxRoll > 0.0f) {
                    Mat3 invISum = A->invInertiaWorld + B->invInertiaWorld;
                    for (int k = 0; k < 3; ++k) {
                        Vec3 axis = (k == 0) ? m.normal : m.tangent[k - 1];

                        float kAng = dot(axis, invISum * axis);
                        if (kAng < 1e-9f) continue;

                        Vec3  relW   = B->angularVelocity - A->angularVelocity;
                        float lambda = -dot(relW, axis) / kAng;

                        float oldImpulse = m.rollingImpulse[k];
                        float newImpulse = clampf(oldImpulse + lambda, -maxRoll, maxRoll);
                        lambda = newImpulse - oldImpulse;
                        m.rollingImpulse[k] = newImpulse;

                        Vec3 L = axis * lambda;
                        A->angularVelocity -= A->invInertiaWorld * L;
                        B->angularVelocity += B->invInertiaWorld * L;
                    }
                }
            }
        }
    }
}

// ===========================================================================
// 7. 位置の積分
// ===========================================================================
void World::integratePositions(float dt) {
    for (auto& b : bodies_) {
        if (b->isStatic()) continue;
        b->position += b->velocity * dt;
        b->orientation.integrate(b->angularVelocity, dt);
        b->force  = Vec3{0, 0, 0};
        b->torque = Vec3{0, 0, 0};
    }
}

// ===========================================================================
// 8. 位置補正（残っためり込みを直接押し戻す）
//    速度に手を入れないので、エネルギーを注入せず沈み込みだけ解消できる。
//
//    重要：接触点ごとに補正するので、そのたびに「現在の」めり込み量を
//    局所アンカーから計算し直す（真の Gauss-Seidel）。検出時の値を
//    そのまま使い回すと、接触点 4 個ぶん過剰に押し戻して積み上げが
//    上方向に吹き飛ぶ。
// ===========================================================================
void World::correctPositions() {
    const float maxCorrection = 0.2f; // 1 回の補正量の上限（暴れ防止）

    for (int iter = 0; iter < positionIterations; ++iter) {
        for (auto& m : manifolds_) {
            RigidBody* A = m.a;
            RigidBody* B = m.b;
            float invMassSum = A->invMass + B->invMass;
            if (invMassSum <= 0.0f) continue;

            Mat3 RA = A->orientation.toMat3();
            Mat3 RB = B->orientation.toMat3();

            for (int i = 0; i < m.count; ++i) {
                const Contact& c = m.contacts[i];

                // 検出時は同一点だったアンカーが、いま何処まで離れたかで
                // 現在のめり込み量を求める
                Vec3  pA  = A->position + RA * c.localA;
                Vec3  pB  = B->position + RB * c.localB;
                float pen = c.penetration - dot(pB - pA, m.normal);

                float depth = pen - penetrationSlop;
                if (depth <= 0.0f) continue;

                float amount = clampf(depth * positionCorrection, 0.0f, maxCorrection) / invMassSum;
                Vec3  corr   = m.normal * amount;
                A->position -= corr * A->invMass;
                B->position += corr * B->invMass;
            }
        }
    }
}

// ===========================================================================
// step
// ===========================================================================
void World::step(float dt) {
    if (dt <= 0.0f) return;
    const int n = std::max(1, substeps);
    for (int s = 0; s < n; ++s) substep(dt / (float)n);
}

void World::substep(float h) {
    magnets_.applyForces();
    integrateForces(h);
    broadphase();
    narrowphase();
    warmStart();
    prepareConstraints(h);
    solveVelocities();
    integratePositions(h);
    correctPositions();

    // 次フレームのウォームスタート用に保存
    prevManifolds_.clear();
    for (const auto& m : manifolds_)
        prevManifolds_[pairKey(m.a->id, m.b->id)] = m;
}

} // namespace phys
