// ---------------------------------------------------------------------------
// current.cpp : 決めた電流の導線とコイル（current.h）
// ---------------------------------------------------------------------------
#include "phys/current.h"

#include <algorithm>
#include <cmath>

namespace phys {

namespace {

// Gauss–Legendre の節点と重み（[-1, 1]）
struct Quadrature { int n; float x[4], w[4]; };
const Quadrature QUAD1{1, {0.0f}, {2.0f}};
const Quadrature QUAD2{2, {-0.57735027f, 0.57735027f}, {1.0f, 1.0f}};
const Quadrature QUAD4{4, {-0.86113631f, -0.33998104f, 0.33998104f, 0.86113631f},
                       {0.34785485f, 0.65214515f, 0.65214515f, 0.34785485f}};

// 点 x から線分（中点 mid、長さ len）までの距離で、求積点の数を決める
inline const Quadrature& quadratureFor(float dist, float len) {
    return dist > 4.0f * len ? QUAD1 : dist > 1.5f * len ? QUAD2 : QUAD4;
}

} // namespace

float coilCenterField(float current, float radius, int turns) {
    return 2.0f * PHYS_PI * current * (float)turns / radius;
}

float currentForCoilCenterField(float field, float radius, int turns) {
    return field * radius / (2.0f * PHYS_PI * (float)turns);
}

Vec3 segmentField(const Vec3& a, const Vec3& b, const Vec3& pIn, float current, float clampRadius) {
    Vec3 p = pIn;
    const Vec3  ab = b - a;
    const float L2 = lengthSq(ab);
    if (L2 < 1e-16f || current == 0.0f) return Vec3{0, 0, 0};
    if (clampRadius > 0.0f) {   // 導線より近い点は、導線から clampRadius の距離に置き直す
        const float t  = clampf(dot(p - a, ab) / L2, 0.0f, 1.0f);
        const Vec3  c  = a + ab * t;
        Vec3        d  = p - c;
        const float d2 = lengthSq(d);
        if (d2 < clampRadius * clampRadius) {
            if (d2 < 1e-12f) {
                Vec3 t1, t2;
                buildTangents(ab / std::sqrt(L2), t1, t2);
                d = t1;
            } else {
                d = d / std::sqrt(d2);
            }
            p = c + d * clampRadius;
        }
    }
    // double で計算する（長い直線のそばでは la lb + ra . rb が大きな数どうしの差になり、float では 1% ずれる）
    const double rax = p.x - a.x, ray = p.y - a.y, raz = p.z - a.z;
    const double rbx = p.x - b.x, rby = p.y - b.y, rbz = p.z - b.z;
    const double la  = std::sqrt(rax * rax + ray * ray + raz * raz);
    const double lb  = std::sqrt(rbx * rbx + rby * rby + rbz * rbz);
    const double den = la * lb * (la * lb + rax * rbx + ray * rby + raz * rbz);
    if (den < 1e-30) return Vec3{0, 0, 0};
    const double k = (double)current * (la + lb) / den;
    return Vec3{(float)((ray * rbz - raz * rby) * k), (float)((raz * rbx - rax * rbz) * k), (float)((rax * rby - ray * rbx) * k)};
}

// 第 1 種・第 2 種の完全楕円積分 K(k), E(k)（算術幾何平均。k2 = k^2）
static void ellipticKE(double k2, double& K, double& E) {
    double an = 1.0, bn = std::sqrt(std::max(0.0, 1.0 - k2)), sum = 0.5 * k2, pow2 = 0.5;
    for (int i = 0; i < 12; ++i) {
        const double c  = 0.5 * (an - bn);
        const double a1 = 0.5 * (an + bn);
        bn = std::sqrt(an * bn);
        an = a1;
        pow2 *= 2.0;
        sum += pow2 * c * c;
        if (c < 1e-15) break;
    }
    K = 3.14159265358979323846 / (2.0 * an);
    E = K * (1.0 - sum);
}

Vec3 loopField(const Vec3& c, const Vec3& n, float a, const Vec3& p, float current, float clampRadius, float* distOut) {
    const Vec3   d    = p - c;
    double       z    = dot(d, n);
    const Vec3   rv   = d - n * (float)z;
    double       rho  = length(rv);
    Vec3         rhoHat;
    if (rho > 1e-9) rhoHat = rv / (float)rho;
    else { Vec3 t2; buildTangents(n, rhoHat, t2); }
    // 輪の線までの距離（rho, z の面の中）。太さより近ければ、太さの距離に置き直す
    double drho = rho - a;
    double dist = std::sqrt(drho * drho + z * z);
    if (distOut) *distOut = (float)dist;
    if (current == 0.0f) return Vec3{0, 0, 0};
    if (dist < clampRadius) {
        if (dist < 1e-9) { drho = 0.0; z = clampRadius; }
        else { drho *= clampRadius / dist; z *= clampRadius / dist; }
        rho = a + drho;
    }
    const double alpha2 = (a + rho) * (a + rho) + z * z;
    const double beta2  = (a - rho) * (a - rho) + z * z;
    if (beta2 < 1e-30) return Vec3{0, 0, 0};
    double K, E;
    ellipticKE(4.0 * a * rho / alpha2, K, E);
    const double common = 2.0 * current / std::sqrt(alpha2);   // mu0 I / 2 pi = 2 K I
    const double Bz   = common * (K + (a * a - rho * rho - z * z) / beta2 * E);
    const double Brho = rho > 1e-9 ? common * z / rho * (-K + (a * a + rho * rho + z * z) / beta2 * E) : 0.0;
    return n * (float)Bz + rhoHat * (float)Brho;
}

Vec3 Wire::areaVector() const {
    if (!closed || points.size() < 3) return Vec3{0, 0, 0};
    const Vec3 c = center();
    Vec3 A{0, 0, 0};
    for (size_t i = 0; i < points.size(); ++i)
        A += cross(points[i] - c, points[(i + 1) % points.size()] - c);
    return A * 0.5f;
}

// ---------------------------------------------------------------------------
// 登録
// ---------------------------------------------------------------------------
int CurrentSystem::addWire(RigidBody* body, const std::vector<Vec3>& pointsLocal, bool closed, float current, int turns,
                           float wireRadius, const Vec3& origin, const Quat& orientation) {
    Wire w;
    w.body        = body;
    w.origin      = origin;
    w.orientation = orientation;
    w.pointsLocal = pointsLocal;
    w.closed      = closed;
    w.current     = current;
    w.turns       = turns;
    w.radius      = wireRadius;
    wires_.push_back(w);
    const int index = (int)wires_.size() - 1;
    if (body) {
        if ((int)indexById_.size() <= body->id) indexById_.resize(body->id + 1, -1);
        if (indexById_[body->id] < 0) indexById_[body->id] = index;
    }
    prepare();
    return index;
}

int CurrentSystem::addCoil(const Vec3& center, const Quat& orientation, float radius, int segments, float current, int turns,
                           RigidBody* body, float wireRadius) {
    std::vector<Vec3> pts;
    pts.reserve(segments);
    for (int i = 0; i < segments; ++i) {   // 上（+y）から見て反時計回り: +current で +y 向きの場
        const float a = 2.0f * PHYS_PI * (float)i / (float)segments;
        pts.push_back(center + orientation.rotate(Vec3{radius * std::cos(a), 0.0f, -radius * std::sin(a)}));
    }
    const int   index = addWire(body, pts, true, current, turns, wireRadius);
    Wire&       w     = wires_[index];
    w.circle            = true;
    w.circleCenterLocal = center;
    w.circleAxisLocal   = orientation.rotate(Vec3{0, 1, 0});
    w.circleRadius      = radius;
    prepare();
    return index;
}

int CurrentSystem::addSolenoid(const Vec3& center, const Quat& orientation, float radius, float length, int rings,
                               int segments, float current, int turnsPerRing, RigidBody* body, float wireRadius) {
    int first = -1;
    for (int k = 0; k < rings; ++k) {
        const float y = -0.5f * length + length * ((float)k + 0.5f) / (float)rings;
        const int   i = addCoil(center + orientation.rotate(Vec3{0, y, 0}), orientation, radius, segments, current,
                                turnsPerRing, body, wireRadius);
        if (first < 0) first = i;
    }
    return first;
}

void CurrentSystem::clear() {
    wires_.clear();
    indexById_.clear();
}

int CurrentSystem::indexOf(const RigidBody* body) const {
    return body->id < (int)indexById_.size() ? indexById_[body->id] : -1;
}

void CurrentSystem::prepare() {
    for (Wire& w : wires_) {
        const Vec3 o = w.body ? w.body->position : w.origin;
        const Quat q = w.body ? w.body->orientation : w.orientation;
        w.points.resize(w.pointsLocal.size());
        for (size_t i = 0; i < w.pointsLocal.size(); ++i) w.points[i] = o + q.rotate(w.pointsLocal[i]);
        if (w.circle) {
            w.circleCenter = o + q.rotate(w.circleCenterLocal);
            w.circleAxis   = normalize(q.rotate(w.circleAxisLocal));
        }
    }
}

// ---------------------------------------------------------------------------
// 場
// ---------------------------------------------------------------------------
Vec3 CurrentSystem::fieldOfWire(int wi, const Vec3& p) const {
    const Wire& w = wires_[wi];
    const float I = w.effectiveCurrent();
    Vec3 B{0, 0, 0};
    if (I == 0.0f || w.points.size() < 2) return B;
    if (w.circle) return loopField(w.circleCenter, w.circleAxis, w.circleRadius, p, I, w.radius);
    const int n = w.segmentCount();
    for (int s = 0; s < n; ++s) {
        Vec3 a, b;
        w.segment(s, a, b);
        B += segmentField(a, b, p, I, w.radius);
    }
    return B;
}

Vec3 CurrentSystem::fieldAt(const Vec3& p) const {
    Vec3 B{0, 0, 0};
    for (int i = 0; i < (int)wires_.size(); ++i) B += fieldOfWire(i, p);
    return B;
}

// ---------------------------------------------------------------------------
// 力
// ---------------------------------------------------------------------------
void CurrentSystem::applyForces(MagnetSystem& magnets) {
    const Vec3 B0    = magnets.externalField;
    const bool hasB0 = lengthSq(B0) > 0.0f;

    // 1. 磁性体の点 <-> 導線
    for (MagneticBody& mb : magnets.bodies()) {
        RigidBody* mbody  = mb.body;
        const bool magDyn = !mbody->isStatic();
        const Mat3 R      = mbody->orientation.toMat3();
        for (int k = 0; k < mb.pointCount; ++k) {
            const Vec3& mk = mb.pointM[k];
            if (lengthSq(mk) < 1e-24f) continue;
            const Vec3  xk   = mbody->position + R * mb.pointLocal[k];
            Vec3 Fk{0, 0, 0}, Bk{0, 0, 0};
            for (const Wire& w : wires_) {
                const bool  wDyn = w.dynamic();
                if (!magDyn && !wDyn) continue;
                const float I = w.effectiveCurrent();
                if (I == 0.0f || w.points.size() < 2) continue;
                const float rmin = minDistScale * (mb.pointRadius + w.radius);
                const int   ns   = w.segmentCount();
                if (w.circle) {
                    // 円いコイル: 力 F = grad(m . B) を円の厳密な場の中心差分で求める（場と同じ式なので、エネルギーと矛盾しない）。
                    // コイルの物体には -F と、角運動量が保存するトルク -(m x B) - (x_k - c) x F を返す
                    const Vec3  B   = loopField(w.circleCenter, w.circleAxis, w.circleRadius, xk, I, w.radius);
                    const float eps = 1e-3f * w.circleRadius;
                    Vec3 F;
                    for (int j = 0; j < 3; ++j) {
                        Vec3 e{0, 0, 0};
                        e[j] = eps;
                        const Vec3 dB = loopField(w.circleCenter, w.circleAxis, w.circleRadius, xk + e, I, w.radius) -
                                        loopField(w.circleCenter, w.circleAxis, w.circleRadius, xk - e, I, w.radius);
                        F[j] = dot(mk, dB) / (2.0f * eps);
                    }
                    Fk += F;
                    Bk += B;
                    if (wDyn) {
                        w.body->force  -= F;
                        w.body->torque -= cross(mk, B) + cross(xk - w.body->position, F);
                    }
                    continue;
                }
                for (int s = 0; s < ns; ++s) {
                    Vec3 a, b;
                    w.segment(s, a, b);
                    const Vec3  half = (b - a) * 0.5f, mid = (a + b) * 0.5f;
                    const float len  = 2.0f * length(half);
                    const Quadrature& Q = quadratureFor(length(xk - mid), len);
                    if (magDyn) Bk += segmentField(a, b, xk, I, w.radius);
                    for (int q = 0; q < Q.n; ++q) {
                        const Vec3 xq = mid + half * Q.x[q];
                        Vec3       r  = xq - xk;
                        const float r2 = lengthSq(r);
                        if (r2 < 1e-12f) continue;
                        if (r2 < rmin * rmin) r = r * (rmin / std::sqrt(r2));
                        const Vec3 f = cross(half, dipoleField(r, mk)) * (I * Q.w[q]);   // I dl x B
                        if (wDyn) w.body->applyForceAtPoint(f, xq);
                        Fk -= f;
                    }
                }
            }
            if (magDyn) {
                mbody->force  += Fk;
                mbody->torque += cross(xk - mbody->position, Fk) + cross(mk, Bk);
            }
        }
    }

    // 2. 導線どうし（a の求積点に b の場。b には作用・反作用）、3. 一様な外部磁場
    const int nw = (int)wires_.size();
    for (int ia = 0; ia < nw; ++ia) {
        const Wire& a = wires_[ia];
        const float Ia = a.effectiveCurrent();
        if (Ia == 0.0f || a.points.size() < 2) continue;
        const bool aDyn = a.dynamic();
        const int  nsa  = a.segmentCount();
        if (aDyn && hasB0) {
            for (int s = 0; s < nsa; ++s) {
                Vec3 p0, p1;
                a.segment(s, p0, p1);
                a.body->applyForceAtPoint(cross(p1 - p0, B0) * Ia, (p0 + p1) * 0.5f);
            }
        }
        for (int ib = ia + 1; ib < nw; ++ib) {
            const Wire& b = wires_[ib];
            const bool bDyn = b.dynamic();
            if (!aDyn && !bDyn) continue;
            if (b.effectiveCurrent() == 0.0f || b.points.size() < 2) continue;
            const Vec3 ca = a.center(), cb = b.center();
            Vec3 Fa{0, 0, 0}, Ta{0, 0, 0};
            for (int s = 0; s < nsa; ++s) {
                Vec3 p0, p1;
                a.segment(s, p0, p1);
                const Vec3 half = (p1 - p0) * 0.5f, mid = (p0 + p1) * 0.5f;
                for (int q = 0; q < QUAD2.n; ++q) {
                    const Vec3 xq = mid + half * QUAD2.x[q];
                    const Vec3 f  = cross(half, fieldOfWire(ib, xq)) * (Ia * QUAD2.w[q]);
                    Fa += f;
                    Ta += cross(xq - ca, f);
                }
            }
            if (aDyn) { a.body->force += Fa; a.body->torque += Ta; }
            if (bDyn) { b.body->force -= Fa; b.body->torque -= Ta + cross(ca - cb, Fa); }
        }
    }
}

} // namespace phys
