// ---------------------------------------------------------------------------
// world.cpp
//
// 1 ステップは dt / substeps の刻みで substeps 回のサブステップに分ける。
// 1 サブステップの流れ：
//   0. 磁力（磁性体の力・トルクを body->force / torque に加算）、電流の導線の力、電気の力（静電気・渦電流）
//   1. 外力の積分（重力・力・トルク → 速度）。電荷を持つ物体の速度を磁場のまわりに回す（ローレンツ力）
//   2. Broadphase（AABB の総当たり）
//   3. Narrowphase（接触マニフォールド生成）。触れた導体どうし・導体と板で電荷をやり取りする
//   4. ウォームスタート（前フレームのインパルスを再利用）
//   5. 制約の前計算（実効質量・反発バイアス）
//   6. 速度の反復解法（法線インパルス ＋ 摩擦インパルス）
//   7. 位置の積分
//   8. めり込みの位置補正
// substeps = 1 で磁性体も電気の物体もなければ、サブステップのない 1 ステップと同じになる。
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <algorithm>
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

RigidBody* World::createMagnetBox(const Vec3& position, const Vec3& halfExtents, float mass,
                                  const Vec3& momentLocal, const Quat& orientation) {
    RigidBody* body = createBox(position, halfExtents, mass);
    body->orientation = orientation;
    magnets_.add(body, momentLocal);
    return body;
}

RigidBody* World::createSoftMagnetBox(const Vec3& position, const Vec3& halfExtents, float mass,
                                      const Vec3& alphaLocal, float saturation, const Quat& orientation) {
    RigidBody* body = createBox(position, halfExtents, mass);
    body->orientation = orientation;
    magnets_.add(body, Vec3{0, 0, 0}, alphaLocal, saturation);
    return body;
}

RigidBody* World::createChargedSphere(const Vec3& position, float radius, float mass, float charge, bool conductor) {
    RigidBody* body = createSphere(position, radius, mass);
    electric_.addBody(body, charge, conductor);
    return body;
}

RigidBody* World::createConductorSphere(const Vec3& position, float radius, float mass, float sigma, float spacing) {
    RigidBody* body = createSphere(position, radius, mass);
    electric_.addBody(body, 0.0f, true, sigma, spacing);
    return body;
}

RigidBody* World::createConductorBox(const Vec3& position, const Vec3& halfExtents, float mass, float sigma,
                                     const Quat& orientation, float spacing) {
    RigidBody* body = createBox(position, halfExtents, mass);
    body->orientation = orientation;
    electric_.addBody(body, 0.0f, false, sigma, spacing);
    return body;
}

int World::createCapacitor(const Vec3& center, const Quat& orientation, float halfX, float halfZ, float gap, float thickness,
                           float potentialA, float potentialB) {
    const Vec3 n   = orientation.rotate(Vec3{0, 1, 0});
    const Vec3 off = n * (0.5f * gap + 0.5f * thickness);
    RigidBody* a = createBox(center - off, Vec3{halfX, 0.5f * thickness, halfZ}, 0.0f);
    RigidBody* b = createBox(center + off, Vec3{halfX, 0.5f * thickness, halfZ}, 0.0f);
    a->orientation = orientation;
    b->orientation = orientation;
    return electric_.addCapacitor(a, b, potentialA, potentialB, center, orientation, Vec3{halfX, 0.5f * gap, halfZ});
}

void World::clear() {
    bodies_.clear();
    manifolds_.clear();
    prevManifolds_.clear();
    pairs_.clear();
    aabbs_.clear();
    magnets_.clear();
    magnets_.extraField = nullptr;
    electric_.clear();
    currents_.clear();
    pinned_.clear();
    pinAnchor_.clear();
    nextId_ = 0;
}

void World::setPinned(RigidBody* body, bool pinned) {
    if ((int)pinned_.size() <= body->id) {
        pinned_.resize(body->id + 1, 0);
        pinAnchor_.resize(body->id + 1);
    }
    pinned_[body->id]    = pinned ? 1 : 0;
    pinAnchor_[body->id] = body->position;
    if (pinned) body->velocity = Vec3{0, 0, 0};
}

bool World::isPinned(const RigidBody* body) const {
    return body->id < (int)pinned_.size() && pinned_[body->id];
}

// 固定した物体の並進を止める（回転はそのまま）
void World::holdPinned(bool resetPosition) {
    for (size_t id = 0; id < pinned_.size(); ++id) {
        if (!pinned_[id]) continue;
        RigidBody* b = bodies_[id].get();
        b->velocity = Vec3{0, 0, 0};
        if (resetPosition) b->position = pinAnchor_[id];
    }
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
    // 導線の場の中の磁性体のエネルギーは magnetic に入る（電源がする仕事は数えない）
    const_cast<MagnetSystem&>(magnets_).extraField = currents_.empty() ? nullptr : &currents_;
    e.magnetic = magnets_.potentialEnergy();
    if (!electric_.empty()) e.electric = electric_.potentialEnergy();
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
            b->velocity = b->kinematicVelocity;
            b->angularVelocity = b->kinematicAngularVelocity;
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
// 2. Broadphase : AABB の総当たり（動く物体 x 全物体。数百体までなら十分実用的）
// ===========================================================================
void World::broadphase() {
    const int n = (int)bodies_.size();
    aabbs_.resize(n);
    for (int i = 0; i < n; ++i) {
        aabbs_[i] = bodies_[i]->computeAABB();
        aabbs_[i].expand(0.02f); // 少し膨らませて接触の取りこぼしを防ぐ
    }

    // 静的どうしは調べない。動く物体ごとに全物体と比べ（動く物体どうしは i < j のときだけ）、
    // 総当たり（i < j の辞書順）と同じ並びに並べ直す（ソルバの結果を変えない）。
    // 静的な物体が多い場面（ゲームの大きなマップ）で、動く物体の数 x 全物体の数で済む
    dynamic_.clear();
    for (int i = 0; i < n; ++i)
        if (!bodies_[i]->isStatic()) dynamic_.push_back(i);
    pairs_.clear();
    for (int i : dynamic_) {
        for (int j = 0; j < n; ++j) {
            if (j == i || (j < i && !bodies_[j]->isStatic())) continue;
            if (!aabbs_[i].overlaps(aabbs_[j])) continue;
            pairs_.emplace_back(std::min(i, j), std::max(i, j));
        }
    }
    std::sort(pairs_.begin(), pairs_.end());
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

void World::prepareConstraints(float /*dt*/) {
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
            //   磁性体が関わる接触では、この刻みの力を積分する前の速度で判定する。
            //   積分後の速度には、この刻みで磁力が与えた引き込み速度（塊の中では 1 刻みで
            //   秒速十数）が含まれ、押し付けられているだけの接触を「衝突」として跳ね返して
            //   エネルギーを注入してしまうため。磁性体がなければ従来と同じ。
            Vec3 relVel;
            if (magnets_.indexOf(A) >= 0 || magnets_.indexOf(B) >= 0 || electric_.indexOf(A) >= 0 || electric_.indexOf(B) >= 0 ||
                currents_.indexOf(A) >= 0 || currents_.indexOf(B) >= 0) {
                relVel = (preVelocity_[B->id] + cross(preAngularVelocity_[B->id], c.rB))
                       - (preVelocity_[A->id] + cross(preAngularVelocity_[A->id], c.rA));
            } else {
                relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
            }
            float vn = dot(relVel, m.normal);
            c.velocityBias = (vn < -restitutionThreshold) ? m.restitution * vn : 0.0f;
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
    // 反発の判定用に、力を積分する前の速度を残す（body->id は bodies_ の添字と同じ）
    const bool electric = !electric_.empty();
    if (!magnets_.bodies().empty() || electric || !currents_.empty()) {
        preVelocity_.resize(bodies_.size());
        preAngularVelocity_.resize(bodies_.size());
        for (const auto& b : bodies_) {
            preVelocity_[b->id]        = b->velocity;
            preAngularVelocity_[b->id] = b->angularVelocity;
        }
    }
    // 導線: 頂点を更新し、磁性体の点の場に導線の場が入るようにつなぐ（World は値で持たれるので毎回設定する）
    const bool currents = !currents_.empty();
    if (currents) currents_.prepare();
    magnets_.extraField = currents ? &currents_ : nullptr;
    magnets_.applyForces();
    if (currents) currents_.applyForces(magnets_);
    if (electric) electric_.applyForces(magnets_, h, currents ? &currents_ : nullptr);
    integrateForces(h);
    if (electric) electric_.rotateLorentz(magnets_, h, currents ? &currents_ : nullptr);
    holdPinned(false);        // 固定した物体は力で並進しない
    broadphase();
    narrowphase();
    if (electric) electric_.exchangeCharges(manifolds_);
    warmStart();
    prepareConstraints(h);
    solveVelocities();
    holdPinned(false);        // 接触で得た並進速度も捨てる
    integratePositions(h);
    correctPositions();
    holdPinned(true);         // めり込み補正で動いた位置を固定点に戻す

    // 次フレームのウォームスタート用に保存
    prevManifolds_.clear();
    for (const auto& m : manifolds_)
        prevManifolds_[pairKey(m.a->id, m.b->id)] = m;
}

} // namespace phys
