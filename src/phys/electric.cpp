// ---------------------------------------------------------------------------
// electric.cpp : 電荷・電場・ローレンツ力・渦電流（electric.h）
// ---------------------------------------------------------------------------
#include "phys/electric.h"

#include "phys/current.h"

#include <algorithm>
#include <cmath>

namespace phys {

float chargeFromGamma(float gamma, float mass, float contactDist, float g) {
    return std::sqrt(gamma * mass * g) * contactDist;
}

float tubeDragCoefficient(float moment, float tubeRadius, float wall, float sigma) {
    const float a2 = tubeRadius * tubeRadius;
    return 45.0f * PHYS_PI * PHYS_PI / 64.0f * moment * moment * sigma * wall / (a2 * a2);
}

float sigmaForTubeTerminalSpeed(float moment, float mass, float g, float tubeRadius, float wall, float vTerminal) {
    return mass * g / (vTerminal * tubeDragCoefficient(moment, tubeRadius, wall, 1.0f));
}

// a b^T（magnet.cpp と同じ）
static Mat3 outer(const Vec3& a, const Vec3& b) { return {a * b.x, a * b.y, a * b.z}; }

// 点電荷 q の場と勾配（dE_i / dr_j = q (delta_ij - 3 n_i n_j) / r^3）
static Vec3 chargeField(const Vec3& r, float q) {
    const float invR = 1.0f / length(r);
    return r * (q * invR * invR * invR);
}

static Mat3 chargeGradient(const Vec3& r, float q) {
    const float invR = 1.0f / length(r);
    const Vec3  n    = r * invR;
    return (Mat3::identity() - outer(n, n) * 3.0f) * (q * invR * invR * invR);
}

// ---------------------------------------------------------------------------
// 登録
// ---------------------------------------------------------------------------
int ElectricSystem::addBody(RigidBody* body, float charge, bool conductor, float sigma, float spacing) {
    ElectricBody eb;
    eb.body   = body;
    eb.charge = charge;
    const bool sphere = body->shape.type == ShapeType::Sphere;
    eb.radius    = sphere ? body->shape.radius : 0.0f;
    eb.conductor = conductor && sphere;
    eb.alpha     = eb.conductor ? eb.radius * eb.radius * eb.radius : 0.0f;
    eb.sigma     = sigma;
    if (sigma > 0.0f) {   // 渦電流の点: 格子のセルの中心（球は中に入る点だけ）
        const Vec3 he = sphere ? Vec3{eb.radius, eb.radius, eb.radius} : body->shape.halfExtents;
        if (spacing <= 0.0f) spacing = std::max(std::min({he.x, he.y, he.z}), 0.25f * std::max({he.x, he.y, he.z}));
        int n[3];
        for (int a = 0; a < 3; ++a) n[a] = std::max(1, (int)std::ceil(2.0f * he[a] / spacing - 1e-4f));
        for (int i = 0; i < n[0]; ++i)
            for (int j = 0; j < n[1]; ++j)
                for (int k = 0; k < n[2]; ++k) {
                    const Vec3 q{he.x * (-1.0f + (2.0f * i + 1.0f) / n[0]), he.y * (-1.0f + (2.0f * j + 1.0f) / n[1]),
                                 he.z * (-1.0f + (2.0f * k + 1.0f) / n[2])};
                    if (sphere && length(q) > eb.radius) continue;
                    eb.eddyPoints.push_back(q);
                }
        const float volume = sphere ? 4.0f / 3.0f * PHYS_PI * eb.radius * eb.radius * eb.radius : 8.0f * he.x * he.y * he.z;
        eb.pointVolume = eb.eddyPoints.empty() ? 0.0f : volume / (float)eb.eddyPoints.size();
    }
    if ((int)indexById_.size() <= body->id) indexById_.resize(body->id + 1, -1);
    indexById_[body->id] = (int)bodies_.size();
    bodies_.push_back(eb);
    initialized_ = false;
    return (int)bodies_.size() - 1;
}

int ElectricSystem::addCapacitor(RigidBody* plateA, RigidBody* plateB, float potentialA, float potentialB, const Vec3& center,
                                 const Quat& orientation, const Vec3& halfExtents) {
    Capacitor c;
    c.plateA      = plateA;
    c.plateB      = plateB;
    c.potentialA  = potentialA;
    c.potentialB  = potentialB;
    c.center      = center;
    c.orientation = orientation;
    c.halfExtents = halfExtents;
    c.E = orientation.rotate(Vec3{0, 1, 0}) * ((potentialA - potentialB) / (2.0f * halfExtents.y));
    const int ci = (int)capacitors_.size();
    for (int k = 0; k < 2; ++k) {
        const RigidBody* b = k == 0 ? plateA : plateB;
        if ((int)capOfPlate_.size() <= b->id) capOfPlate_.resize(b->id + 1, -1);
        capOfPlate_[b->id] = ci * 2 + k;
    }
    capacitors_.push_back(c);
    return ci;
}

void ElectricSystem::clear() {
    bodies_.clear();
    capacitors_.clear();
    indexById_.clear();
    capOfPlate_.clear();
    eddySamples_.clear();
    initialized_ = false;
}

int ElectricSystem::indexOf(const RigidBody* body) const {
    return body->id < (int)indexById_.size() ? indexById_[body->id] : -1;
}

// ---------------------------------------------------------------------------
// 場
// ---------------------------------------------------------------------------
Vec3 ElectricSystem::capacitorField(const Vec3& p) const {
    Vec3 E{0, 0, 0};
    for (const Capacitor& c : capacitors_) {
        const Vec3 q = c.orientation.toMat3().transposed() * (p - c.center);
        if (std::fabs(q.x) <= c.halfExtents.x && std::fabs(q.y) <= c.halfExtents.y && std::fabs(q.z) <= c.halfExtents.z)
            E += c.E;
    }
    return E;
}

Vec3 ElectricSystem::fieldAt(const Vec3& p) const {
    Vec3 E = externalField + capacitorField(p);
    for (const ElectricBody& eb : bodies_) {
        const Vec3 r = p - eb.body->position;
        if (lengthSq(r) < 1e-12f) continue;
        if (eb.charge != 0.0f) E += chargeField(r, eb.charge);
        if (lengthSq(eb.p) > 0.0f) E += dipoleField(r, eb.p);
    }
    return E;
}

float ElectricSystem::potentialAt(const Vec3& p) const {
    float phi = -dot(externalField, p);
    for (const Capacitor& c : capacitors_) {   // 板の間の箱の中だけ: A の板の面で V_A から一様に下がる
        const Vec3 q = c.orientation.toMat3().transposed() * (p - c.center);
        if (std::fabs(q.x) <= c.halfExtents.x && std::fabs(q.y) <= c.halfExtents.y && std::fabs(q.z) <= c.halfExtents.z)
            phi += c.potentialA + (c.potentialB - c.potentialA) * (q.y + c.halfExtents.y) / (2.0f * c.halfExtents.y);
    }
    for (const ElectricBody& eb : bodies_) {
        const Vec3  r  = p - eb.body->position;
        const float r2 = lengthSq(r);
        if (r2 < 1e-12f) continue;
        const float invR = 1.0f / std::sqrt(r2);
        phi += eb.charge * invR + dot(eb.p, r) * invR * invR * invR;
    }
    return phi;
}

float ElectricSystem::potentialEnergy() const {
    float U = 0.0f;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        const ElectricBody& a = bodies_[i];
        if (a.charge == 0.0f) continue;
        U -= a.charge * dot(externalField, a.body->position);
        for (const Capacitor& c : capacitors_) {   // 板の間の電位（箱の中だけ）
            const Vec3 q = c.orientation.toMat3().transposed() * (a.body->position - c.center);
            if (std::fabs(q.x) <= c.halfExtents.x && std::fabs(q.y) <= c.halfExtents.y && std::fabs(q.z) <= c.halfExtents.z)
                U += a.charge * (c.potentialA + (c.potentialB - c.potentialA) * (q.y + c.halfExtents.y) / (2.0f * c.halfExtents.y));
        }
        for (size_t j = i + 1; j < bodies_.size(); ++j) {
            const ElectricBody& b = bodies_[j];
            if (b.charge == 0.0f) continue;
            const float r = std::max(length(b.body->position - a.body->position), minDistScale * (a.radius + b.radius));
            if (r > 1e-6f) U += a.charge * b.charge / r;
        }
    }
    for (const ElectricBody& eb : bodies_) U -= 0.5f * dot(eb.p, eb.E);   // 誘導双極子（分極のエネルギーを含む）
    return U;
}

float ElectricSystem::totalCharge() const {
    float Q = 0.0f;
    for (const ElectricBody& eb : bodies_) Q += eb.charge;
    return Q;
}

float ElectricSystem::plateExchange() const {
    float Q = 0.0f;
    for (const Capacitor& c : capacitors_) Q += c.exchanged;
    return Q;
}

Vec3 ElectricSystem::magneticFieldAt(const MagnetSystem& magnets, const Vec3& p, int skipBody, const CurrentSystem* currents) {
    Vec3 B = magnets.externalField;
    if (currents) B += currents->fieldAt(p);
    for (const MagneticBody& mb : magnets.bodies()) {
        if (mb.body->id == skipBody) continue;
        const Mat3 R = mb.body->orientation.toMat3();
        for (int k = 0; k < mb.pointCount; ++k) {
            const Vec3 r = p - (mb.body->position + R * mb.pointLocal[k]);
            if (lengthSq(r) < 1e-12f) continue;
            B += dipoleField(r, mb.pointM[k]);
        }
    }
    return B;
}

// ---------------------------------------------------------------------------
// 静電気: 全ペアの場（と力）。誘導双極子は前回の場から決める
// ---------------------------------------------------------------------------
void ElectricSystem::electrostatics(bool withForces) {
    const int n = (int)bodies_.size();
    for (ElectricBody& eb : bodies_) eb.E = externalField + capacitorField(eb.body->position);
    for (int i = 0; i < n; ++i) {
        ElectricBody& a = bodies_[i];
        const bool aSrc = a.charge != 0.0f || lengthSq(a.p) > 0.0f;
        for (int j = i + 1; j < n; ++j) {
            ElectricBody& b = bodies_[j];
            const bool bSrc = b.charge != 0.0f || lengthSq(b.p) > 0.0f;
            if (!aSrc && !bSrc) continue;
            if (a.alpha == 0.0f && b.alpha == 0.0f && (a.charge == 0.0f || b.charge == 0.0f)) continue;   // 互いに効かない
            Vec3        r = b.body->position - a.body->position;   // a → b
            const float d = length(r);
            if (d < 1e-6f) continue;
            const float dmin = minDistScale * (a.radius + b.radius);
            if (d < dmin) r = r * (dmin / d);
            // a が b の位置に作る場と勾配、b が a の位置に作る場
            Vec3 Eab{0, 0, 0}, Eba{0, 0, 0};
            Mat3 Gab = Mat3::zero();
            if (a.charge != 0.0f) { Eab += chargeField(r, a.charge); Gab = Gab + chargeGradient(r, a.charge); }
            if (lengthSq(a.p) > 0.0f) { Eab += dipoleField(r, a.p); Gab = Gab + dipoleGradient(r, a.p); }
            if (b.charge != 0.0f) Eba += chargeField(r * -1.0f, b.charge);
            if (lengthSq(b.p) > 0.0f) Eba += dipoleField(r * -1.0f, b.p);
            b.E += Eab;
            a.E += Eba;
            if (!withForces) continue;
            const Vec3 F = Eab * b.charge + Gab * b.p;   // b が受ける力（a は -F）
            if (!b.body->isStatic()) b.body->force += F;
            if (!a.body->isStatic()) a.body->force -= F;
        }
        if (withForces && !a.body->isStatic() && a.charge != 0.0f)   // 外部と板の一様な場（双極子には力が出ない）
            a.body->force += (externalField + capacitorField(a.body->position)) * a.charge;
    }
    for (ElectricBody& eb : bodies_) eb.p = eb.E * eb.alpha;
}

void ElectricSystem::updateDipoles(int iterations) {
    for (int k = 0; k < iterations; ++k) electrostatics(false);
    initialized_ = true;
}

// ---------------------------------------------------------------------------
// 渦電流（局所的なオームの法則）
// ---------------------------------------------------------------------------
void ElectricSystem::eddyCurrents(const MagnetSystem& magnets, float h, const CurrentSystem* currents) {
    eddySamples_.clear();
    const auto& mags = magnets.bodies();
    const int   nm   = (int)mags.size();
    const bool  ext  = lengthSq(magnets.externalField) > 0.0f;
    std::vector<int> wireIdx;   // 電流の流れている導線（源）
    if (currents)
        for (int i = 0; i < (int)currents->wires().size(); ++i)
            if (currents->wires()[i].effectiveCurrent() != 0.0f) wireIdx.push_back(i);
    const int   ns   = nm + (ext ? 1 : 0) + (int)wireIdx.size();   // 場の源（磁性体、外部磁場、導線）
    if (ns == 0) return;
    std::vector<Vec3> Bn(ns), Fn(ns), Tn(ns), un(ns);
    std::vector<float> dv2(ns);
    std::vector<RigidBody*> srcBody(ns, nullptr);   // 反作用を返す物体（外部磁場・静的な導線は nullptr）
    for (int s = 0; s < nm; ++s) { un[s] = mags[s].body->velocity; srcBody[s] = mags[s].body; }
    if (ext) un[nm] = Vec3{0, 0, 0};
    for (size_t i = 0; i < wireIdx.size(); ++i) {
        const Wire& w = currents->wires()[wireIdx[i]];
        const int   s = nm + (ext ? 1 : 0) + (int)i;
        un[s]      = w.velocity();
        srcBody[s] = w.dynamic() ? w.body : nullptr;
    }
    for (ElectricBody& eb : bodies_) {
        if (eb.sigma <= 0.0f || eb.eddyPoints.empty()) continue;
        RigidBody* c = eb.body;
        const Mat3 R  = c->orientation.toMat3();
        const Vec3 xc = c->position;
        std::fill(Fn.begin(), Fn.end(), Vec3{0, 0, 0});
        std::fill(Tn.begin(), Tn.end(), Vec3{0, 0, 0});
        std::fill(dv2.begin(), dv2.end(), 0.0f);
        float rr2 = 0.0f;   // 点の重心からの距離の 2 乗の平均（回転の慣性の目安）
        for (const Vec3& pl : eb.eddyPoints) {
            const Vec3 rel = R * pl, x = xc + rel;
            const Vec3 v   = c->velocity + cross(c->angularVelocity, rel);
            Vec3 Btot{0, 0, 0}, Ep{0, 0, 0};
            for (int s = 0; s < ns; ++s) {
                Vec3 B{0, 0, 0};
                if (s < nm) {
                    const MagneticBody& mb = mags[s];
                    if (mb.body != c) {
                        const Mat3 Rm = mb.body->orientation.toMat3();
                        for (int k = 0; k < mb.pointCount; ++k) {
                            Vec3        r  = x - (mb.body->position + Rm * mb.pointLocal[k]);
                            const float rl = length(r);
                            if (rl < 1e-6f) continue;
                            if (rl < mb.pointRadius) r = r * (mb.pointRadius / rl);   // 磁石の中の点（重なったとき）
                            B += dipoleField(r, mb.pointM[k]);
                        }
                    }
                } else if (ext && s == nm) {
                    B = magnets.externalField;
                } else {
                    B = currents->fieldOfWire(wireIdx[s - nm - (ext ? 1 : 0)], x);
                }
                Bn[s] = B;
                Btot += B;
                Ep += cross(v - un[s], B);
                dv2[s] += lengthSq(v - un[s]);
            }
            const Vec3 J = Ep * eb.sigma;
            for (int s = 0; s < ns; ++s) {
                const Vec3 f = cross(J, Bn[s]) * eb.pointVolume;
                Fn[s] += f;
                Tn[s] += cross(rel, f);
            }
            rr2 += lengthSq(rel);
            eddySamples_.push_back({x, J});
        }
        const float np = (float)eb.eddyPoints.size();
        rr2 = std::max(rr2 / np, 1e-6f);
        // 源ごとに加える（1 サブステップで相対的な動きを消しすぎないように縮める。作用と反作用は同じ割合）
        const float mc = c->isStatic() ? 0.0f : 1.0f / c->invMass;
        const float Ic = c->isStatic() ? 0.0f : 1.0f / std::max({c->invInertiaLocal.x, c->invInertiaLocal.y, c->invInertiaLocal.z});
        for (int s = 0; s < ns; ++s) {
            RigidBody* m  = srcBody[s];
            const bool mDyn = m && !m->isStatic() && m != c;
            const bool cDyn = !c->isStatic();
            if (!mDyn && !cDyn) continue;
            const float dvEff = std::sqrt(dv2[s] / np);
            if (dvEff < 1e-6f) continue;
            const float mm = mDyn ? 1.0f / m->invMass : 0.0f;
            const float mu = (mDyn && cDyn) ? mc * mm / (mc + mm) : (cDyn ? mc : mm);
            float scale = 1.0f;
            const float cLin = length(Fn[s]) / dvEff;   // 抵抗の係数の目安
            if (cLin * h > eddyStability * mu) scale = std::min(scale, eddyStability * mu / (cLin * h));
            if (cDyn) {
                const float cRot = length(Tn[s]) * std::sqrt(rr2) / dvEff;
                if (cRot * h > eddyStability * Ic) scale = std::min(scale, eddyStability * Ic / (cRot * h));
            }
            if (cDyn) {
                c->force  += Fn[s] * scale;
                c->torque += Tn[s] * scale;
            }
            if (mDyn) m->force -= Fn[s] * scale;
        }
    }
}

void ElectricSystem::applyForces(const MagnetSystem& magnets, float h, const CurrentSystem* currents) {
    if (!initialized_) updateDipoles(10);
    electrostatics(true);
    bool eddy = false;
    for (const ElectricBody& eb : bodies_) eddy = eddy || eb.sigma > 0.0f;
    if (eddy) eddyCurrents(magnets, h, currents && !currents->empty() ? currents : nullptr);
}

// ---------------------------------------------------------------------------
// ローレンツ力の磁場の部分: 速度を B のまわりに回す（dv/dt = (kappa q / m) v x B = Omega x v、Omega = -kappa q B / m）
// ---------------------------------------------------------------------------
void ElectricSystem::rotateLorentz(const MagnetSystem& magnets, float h, const CurrentSystem* currents) {
    if (currents && currents->empty()) currents = nullptr;
    for (ElectricBody& eb : bodies_) {
        RigidBody* b = eb.body;
        if (eb.charge == 0.0f || b->isStatic()) continue;
        const Vec3  B  = magneticFieldAt(magnets, b->position, b->id, currents);
        const float Bl = length(B);
        if (Bl < 1e-12f) continue;
        const Vec3  axis  = B / Bl;
        const float angle = -kappa * eb.charge * Bl * b->invMass * h;
        b->velocity = Quat::fromAxisAngle(axis, angle).rotate(b->velocity);
    }
}

// ---------------------------------------------------------------------------
// 接触での電荷の移動
// ---------------------------------------------------------------------------
void ElectricSystem::exchangeCharges(const std::vector<Manifold>& manifolds) {
    for (const Manifold& mf : manifolds) {
        const int ia = indexOf(mf.a), ib = indexOf(mf.b);
        ElectricBody* A = ia >= 0 ? &bodies_[ia] : nullptr;
        ElectricBody* B = ib >= 0 ? &bodies_[ib] : nullptr;
        if (A && B && A->conductor && B->conductor) {   // 導体の球どうし: 容量（半径）の比で分ける
            const float Q = A->charge + B->charge, Ct = A->radius + B->radius;
            A->charge = Q * A->radius / Ct;
            B->charge = Q * B->radius / Ct;
            continue;
        }
        // 導体の球とコンデンサの板: 球の電荷が R V になる
        for (int k = 0; k < 2; ++k) {
            ElectricBody*    ball  = k == 0 ? A : B;
            const RigidBody* other = k == 0 ? mf.b : mf.a;
            if (!ball || !ball->conductor || other->id >= (int)capOfPlate_.size() || capOfPlate_[other->id] < 0) continue;
            Capacitor&  cap = capacitors_[capOfPlate_[other->id] / 2];
            const float V   = (capOfPlate_[other->id] % 2 == 0) ? cap.potentialA : cap.potentialB;
            const float q   = ball->radius * V;
            cap.exchanged += q - ball->charge;
            ball->charge = q;
        }
    }
}

} // namespace phys
