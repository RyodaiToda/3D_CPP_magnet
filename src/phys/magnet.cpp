// ---------------------------------------------------------------------------
// magnet.cpp
//
// 双極子 1 が点 2 に作る場（r = x2 - x1, n = r/|r|）:
//   B = [3 (m1.n) n - m1] / r^3
// 場の勾配（対称・トレース 0。真空中では div B = 0, rot B = 0 のため）:
//   G = 3/r^4 [ m1 n^T + n m1^T + (m1.n) I - 5 (m1.n) n n^T ]
// 双極子 2 が受ける力は F2 = G m2。展開すると
//   F2 = 3/r^4 [ (m1.n) m2 + (m2.n) m1 + (m1.m2) n - 5 (m1.n)(m2.n) n ]
// applyForces ではこの展開形を使い、3x3 行列は作らない（同じ値で安い）。
//
// 箱は 2x2x2 の点の双極子で表す（magnet.h）。点 a, b の組の力 F は点に働くので、
// 物体のトルクは (x_b - x_B) x F + m_b x B になる。点の組ごとに
//   r x F + m_a x B_b->a + m_b x B_a->b = 0
// が成り立つので、全角運動量は丸め誤差を除き厳密に保存する。
// ---------------------------------------------------------------------------
#include "phys/magnet.h"

#include <cmath>

namespace phys {

namespace {

constexpr float MU0 = 4.0e-7f * PHYS_PI;

// a b^T
inline Mat3 outer(const Vec3& a, const Vec3& b) {
    return { a * b.x, a * b.y, a * b.z };
}

inline bool isZero(const Vec3& v) { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; }

inline float bodyRadius(const RigidBody& b) {
    if (b.shape.type == ShapeType::Sphere) return b.shape.radius;
    const Vec3& he = b.shape.halfExtents;
    return std::min(he.x, std::min(he.y, he.z));
}

// 近距離クランプ: |r| < rMin なら向きを保ったまま rMin に伸ばす。r がほぼ 0 なら false
inline bool clampSeparation(Vec3& r, float rMin) {
    float r2 = lengthSq(r);
    if (r2 < 1e-12f) return false;
    if (r2 < rMin * rMin) r = r * (rMin / std::sqrt(r2));
    return true;
}

// 2a x 2b x 2c の直方体の、辺 c 方向の反磁場係数（A. Aharoni, J. Appl. Phys. 83, 3432 (1998)）
float demagZ(double a, double b, double c) {
    const double r  = std::sqrt(a * a + b * b + c * c);
    const double ab = std::sqrt(a * a + b * b), bc = std::sqrt(b * b + c * c), ac = std::sqrt(a * a + c * c);
    double t = (b * b - c * c) / (2 * b * c) * std::log((r - a) / (r + a))
             + (a * a - c * c) / (2 * a * c) * std::log((r - b) / (r + b))
             + b / (2 * c) * std::log((ab + a) / (ab - a))
             + a / (2 * c) * std::log((ab + b) / (ab - b))
             + c / (2 * a) * std::log((bc - b) / (bc + b))
             + c / (2 * b) * std::log((ac - a) / (ac + a))
             + 2 * std::atan(a * b / (c * r))
             + (a * a * a + b * b * b - 2 * c * c * c) / (3 * a * b * c)
             + (a * a + b * b - 2 * c * c) * r / (3 * a * b * c)
             + c / (a * b) * (ac + bc)
             - (ab * ab * ab + bc * bc * bc + ac * ac * ac) / (3 * a * b * c);
    return (float)(t / 3.14159265358979323846);
}

} // namespace

// ===========================================================================
// 純粋関数
// ===========================================================================
Vec3 dipoleField(const Vec3& r, const Vec3& m) {
    float invR  = 1.0f / length(r);
    Vec3  n     = r * invR;
    float invR3 = invR * invR * invR;
    return (n * (3.0f * dot(m, n)) - m) * invR3;
}

Mat3 dipoleGradient(const Vec3& r, const Vec3& m) {
    float invR  = 1.0f / length(r);
    Vec3  n     = r * invR;
    float invR4 = invR * invR * invR * invR;
    float mn    = dot(m, n);
    Mat3  G = outer(m, n) + outer(n, m) + Mat3::identity() * mn - outer(n, n) * (5.0f * mn);
    return G * (3.0f * invR4);
}

float dipoleEnergy(const Vec3& r, const Vec3& m1, const Vec3& m2) {
    float invR  = 1.0f / length(r);
    Vec3  n     = r * invR;
    float invR3 = invR * invR * invR;
    return (dot(m1, m2) - 3.0f * dot(m1, n) * dot(m2, n)) * invR3;
}

Vec3 dipoleForce(const Vec3& r, const Vec3& m1, const Vec3& m2) {
    float invR  = 1.0f / length(r);
    Vec3  n     = r * invR;
    float invR4 = invR * invR * invR * invR;
    float a = dot(m1, n), b = dot(m2, n), c = dot(m1, m2);
    return (m2 * a + m1 * b + n * (c - 5.0f * a * b)) * (3.0f * invR4);
}

float momentFromGamma(float gamma, float mass, float contactDist, float g) {
    float d2 = contactDist * contactDist;
    return std::sqrt(gamma * mass * g * d2 * d2 / 6.0f);
}

float gammaFromMaterial(float Br, float rho, float radius, float g) {
    return Br * Br / (8.0f * MU0 * rho * g * radius);
}

Vec3 demagFactors(const Vec3& he) {
    return { demagZ(he.y, he.z, he.x), demagZ(he.z, he.x, he.y), demagZ(he.x, he.y, he.z) };
}

float sphereSusceptibility(float radius, float chi) {
    return radius * radius * radius * chi / (3.0f + chi);
}

Vec3 boxSusceptibility(const Vec3& he, float chi) {
    const float V = 8.0f * he.x * he.y * he.z;
    const Vec3  N = demagFactors(he);
    const float k = V / (4.0f * PHYS_PI) * chi;
    return { k / (1.0f + N.x * chi), k / (1.0f + N.y * chi), k / (1.0f + N.z * chi) };
}

// ===========================================================================
// MagnetSystem
// ===========================================================================
int MagnetSystem::add(RigidBody* body, const Vec3& permanentLocal,
                      const Vec3& chiLocal, float saturation) {
    MagneticBody mb;
    mb.body           = body;
    mb.permanentLocal = permanentLocal;
    mb.chiLocal       = chiLocal;
    mb.saturation     = saturation;

    if (body->shape.type == ShapeType::Sphere) {
        mb.pointCount    = 1;
        mb.pointLocal[0] = Vec3{0, 0, 0};
        mb.pointRadius   = body->shape.radius;
        mb.boundRadius   = body->shape.radius;
    } else {
        const Vec3& he = body->shape.halfExtents;
        const Vec3  o  = he * boxPointScale;
        mb.pointCount = 8;
        for (int k = 0; k < 8; ++k)
            mb.pointLocal[k] = Vec3{(k & 1) ? o.x : -o.x, (k & 2) ? o.y : -o.y, (k & 4) ? o.z : -o.z};
        mb.pointRadius = (1.0f - boxPointScale) * std::min(he.x, std::min(he.y, he.z));
        mb.boundRadius = length(he);
    }

    const Mat3 R = body->orientation.toMat3();
    for (int k = 0; k < mb.pointCount; ++k) {
        mb.pointB[k] = externalField;
        mb.pointM[k] = pointMoment(mb, R, mb.pointB[k]);
        mb.m += mb.pointM[k];
    }
    mb.B = externalField;
    bodies_.push_back(mb);

    const int index = (int)bodies_.size() - 1;
    if ((int)indexById_.size() <= body->id) indexById_.resize(body->id + 1, -1);
    indexById_[body->id] = index;
    return index;
}

void MagnetSystem::clear() {
    bodies_.clear();
    indexById_.clear();
}

int MagnetSystem::indexOf(const RigidBody* body) const {
    return (body->id < (int)indexById_.size()) ? indexById_[body->id] : -1;
}

// 点のモーメント m = R (p + chi * (R^T B)) / count、|m| <= saturation / count
Vec3 MagnetSystem::pointMoment(const MagneticBody& mb, const Mat3& R, const Vec3& B) {
    const float w  = 1.0f / (float)mb.pointCount;
    const Vec3  Bl = R.transposed() * B;
    const Vec3& chi = mb.chiLocal;
    Vec3 m = R * (mb.permanentLocal + Vec3{chi.x * Bl.x, chi.y * Bl.y, chi.z * Bl.z}) * w;
    if (mb.saturation > 0.0f) {
        float len = length(m), sat = mb.saturation * w;
        if (len > sat) m *= sat / len;
    }
    return m;
}

// 各点の位置と、各点に保存した場（前回の評価）から求めたモーメントを作業配列に写す
void MagnetSystem::gather() const {
    const int n = (int)bodies_.size();
    xc_.resize(n); mTot_.resize(n); clampR_.resize(n); boundR_.resize(n);
    first_.resize(n); count_.resize(n); fixed_.resize(n);
    int np = 0;
    for (int i = 0; i < n; ++i) { first_[i] = np; count_[i] = bodies_[i].pointCount; np += count_[i]; }
    xp_.resize(np); mp_.resize(np); pointR_.resize(np);

    for (int i = 0; i < n; ++i) {
        const MagneticBody& mb = bodies_[i];
        const RigidBody&    b  = *mb.body;
        const Mat3 R = b.orientation.toMat3();
        xc_[i]     = b.position;
        clampR_[i] = bodyRadius(b);
        boundR_[i] = mb.boundRadius;
        // 動かず、場によってモーメントも変わらない物体（固定磁石）
        fixed_[i]  = b.isStatic() && isZero(mb.chiLocal);
        mTot_[i]   = Vec3{0, 0, 0};
        for (int k = 0; k < mb.pointCount; ++k) {
            const int p = first_[i] + k;
            xp_[p]     = (mb.pointCount == 1) ? b.position : b.position + R * mb.pointLocal[k];
            mp_[p]     = pointMoment(mb, R, mb.pointB[k]);
            pointR_[p] = mb.pointRadius;
            mTot_[i]  += mp_[p];
        }
    }
}

// 作業配列の場から各点のモーメントを求め直す（Jacobi 1 回）
float MagnetSystem::remoment() const {
    float maxChange = 0.0f;
    for (int i = 0; i < (int)bodies_.size(); ++i) {
        const MagneticBody& mb = bodies_[i];
        const Mat3 R = mb.body->orientation.toMat3();
        const Vec3 before = mTot_[i];
        mTot_[i] = Vec3{0, 0, 0};
        for (int k = 0; k < mb.pointCount; ++k) {
            const int p = first_[i] + k;
            mp_[p]    = pointMoment(mb, R, Bp_[p]);
            mTot_[i] += mp_[p];
        }
        maxChange = std::max(maxChange, length(mTot_[i] - before));
    }
    return maxChange;
}

// ---------------------------------------------------------------------------
// 全ペアの評価
//   ペア (i, j) ごとに「i が j に作る場」と「j が i に作る場」を評価し、
//   F_j = G_i(x_j) m_j、F_i = -F_j（作用・反作用。運動量は丸め誤差を除き厳密に保存）
//   tau_j = m_j x B_i(x_j)、tau_i = m_i x B_j(x_i)
//   箱を含む近いペアは点の組ごとに同じ計算をし、力の腕のぶんのトルクを足す。
//   遠いペアは中心の双極子 1 個どうしで、その場を相手の全点に同じ値で足す
// ---------------------------------------------------------------------------
void MagnetSystem::evaluate(bool withForces) const {
    const int n  = (int)bodies_.size();
    const int np = (int)xp_.size();
    Bp_.assign(np, Vec3{0, 0, 0});
    Bfar_.assign(n, Vec3{0, 0, 0});
    if (withForces) {
        F_.assign(n, Vec3{0, 0, 0});
        T_.assign(n, Vec3{0, 0, 0});
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (withForces && fixed_[i] && fixed_[j]) continue;   // broadphase と同じく静的どうしは飛ばす

            Vec3 d = xc_[j] - xc_[i];
            const float nearR = nearScale * (boundR_[i] + boundR_[j]);
            const bool  near  = (count_[i] > 1 || count_[j] > 1) && lengthSq(d) < nearR * nearR;

            if (!near) {
                if (!clampSeparation(d, minDistScale * (clampR_[i] + clampR_[j]))) continue;
                float invR  = 1.0f / std::sqrt(lengthSq(d));
                Vec3  nn    = d * invR;
                float invR3 = invR * invR * invR;
                const Vec3& mi = mTot_[i];
                const Vec3& mj = mTot_[j];
                float a = dot(mi, nn), b = dot(mj, nn);
                Vec3 Bij = (nn * (3.0f * a) - mi) * invR3;   // i が j の位置に作る場
                Vec3 Bji = (nn * (3.0f * b) - mj) * invR3;   // j が i の位置に作る場
                Bfar_[j] += Bij;  Bfar_[i] += Bji;
                if (withForces) {
                    float c  = dot(mi, mj);
                    Vec3  Fj = (mj * a + mi * b + nn * (c - 5.0f * a * b)) * (3.0f * invR3 * invR);
                    F_[j] += Fj;  F_[i] -= Fj;
                    T_[j] += cross(mj, Bij);
                    T_[i] += cross(mi, Bji);
                }
                continue;
            }

            for (int pa = first_[i]; pa < first_[i] + count_[i]; ++pa) {
                for (int pb = first_[j]; pb < first_[j] + count_[j]; ++pb) {
                    Vec3 r = xp_[pb] - xp_[pa];
                    if (!clampSeparation(r, minDistScale * (pointR_[pa] + pointR_[pb]))) continue;
                    float invR  = 1.0f / std::sqrt(lengthSq(r));
                    Vec3  nn    = r * invR;
                    float invR3 = invR * invR * invR;
                    const Vec3& ma = mp_[pa];
                    const Vec3& mb = mp_[pb];
                    float a = dot(ma, nn), b = dot(mb, nn);
                    Vec3 Bab = (nn * (3.0f * a) - ma) * invR3;   // 点 a が点 b に作る場
                    Vec3 Bba = (nn * (3.0f * b) - mb) * invR3;   // 点 b が点 a に作る場
                    Bp_[pb] += Bab;  Bp_[pa] += Bba;
                    if (withForces) {
                        float c  = dot(ma, mb);
                        Vec3  Fb = (mb * a + ma * b + nn * (c - 5.0f * a * b)) * (3.0f * invR3 * invR);
                        F_[j] += Fb;  F_[i] -= Fb;
                        T_[j] += cross(xp_[pb] - xc_[j], Fb) + cross(mb, Bab);
                        T_[i] += cross(ma, Bba) - cross(xp_[pa] - xc_[i], Fb);
                    }
                }
            }
        }
    }

    // 遠いペアの場と外部磁場を各点に足す。一様な外部磁場は力を生まず、トルク m x B0 だけ。
    // ほかの源の場（extraField）は各点の場に足すだけ（力とトルクは源の側が加える）
    for (int i = 0; i < n; ++i) {
        for (int p = first_[i]; p < first_[i] + count_[i]; ++p) {
            Bp_[p] += Bfar_[i] + externalField;
            if (extraField) Bp_[p] += extraField->fieldAt(xp_[p]);
        }
        if (withForces) T_[i] += cross(mTot_[i], externalField);
    }
}

// 作業配列の結果を bodies_ に書き戻す
void MagnetSystem::store() {
    for (int i = 0; i < (int)bodies_.size(); ++i) {
        MagneticBody& mb = bodies_[i];
        mb.m = mTot_[i];
        mb.B = Vec3{0, 0, 0};
        for (int k = 0; k < mb.pointCount; ++k) {
            mb.pointM[k] = mp_[first_[i] + k];
            mb.pointB[k] = Bp_[first_[i] + k];
            mb.B += mb.pointB[k] * (1.0f / mb.pointCount);
        }
    }
}

// ---------------------------------------------------------------------------
// 1 サブステップぶんの磁力
//   前回の場から m を更新し（Jacobi 1 回）、全ペアの力とトルクを body に加算して、
//   次回用の場を求める。力の計算と同じループで済むので、誘導のための追加ループはない
// ---------------------------------------------------------------------------
void MagnetSystem::applyForces() {
    if (bodies_.empty()) return;
    gather();
    evaluate(true);

    // 静的剛体に加えた力は World::integrateForces が捨てる
    for (int i = 0; i < (int)bodies_.size(); ++i) {
        bodies_[i].body->force  += F_[i];
        bodies_[i].body->torque += T_[i];
    }
    store();
}

float MagnetSystem::updateMoments(int iterations) {
    if (bodies_.empty() || iterations <= 0) return 0.0f;
    gather();
    float maxChange = 0.0f;
    for (int it = 0; it < iterations; ++it) {
        evaluate(false);
        maxChange = remoment();
    }
    store();   // 保存する場は、保存するモーメントを決めた場（applyForces と同じ関係）
    return maxChange;
}

// U = -1/2 sum p . B - 1/2 sum m . B0（B は外部磁場を含む、点ごとの和）
//   誘導モーメントが自己無撞着（m = p + chi B）なら、誘導に要するエネルギーまで
//   含めた線形系の全エネルギーに厳密に一致する。永久磁石だけなら通常の
//   ペアエネルギーの和 sum_{i<j} dipoleEnergy - sum p . B0 になる。
float MagnetSystem::potentialEnergy() const {
    if (bodies_.empty()) return 0.0f;
    gather();
    evaluate(false);

    float U = 0.0f;
    for (int i = 0; i < (int)bodies_.size(); ++i) {
        const MagneticBody& mb = bodies_[i];
        const Vec3 p = mb.body->orientation.rotate(mb.permanentLocal) * (1.0f / mb.pointCount);
        for (int k = 0; k < mb.pointCount; ++k) U -= 0.5f * dot(p, Bp_[first_[i] + k]);
        U -= 0.5f * dot(mTot_[i], externalField);
        if (extraField)   // ほかの源の場も B0 と同じ扱い（点ごと）
            for (int k = 0; k < mb.pointCount; ++k)
                U -= 0.5f * dot(mp_[first_[i] + k], extraField->fieldAt(xp_[first_[i] + k]));
    }
    return U;
}

Vec3 MagnetSystem::fieldAt(const Vec3& p) const {
    Vec3 B = externalField;
    if (extraField) B += extraField->fieldAt(p);
    for (const MagneticBody& mb : bodies_) {
        const Mat3 R = mb.body->orientation.toMat3();
        for (int k = 0; k < mb.pointCount; ++k) {
            Vec3 r = p - (mb.body->position + R * mb.pointLocal[k]);
            if (lengthSq(r) < 1e-12f) continue;
            B += dipoleField(r, pointMoment(mb, R, mb.pointB[k]));
        }
    }
    return B;
}

// ---------------------------------------------------------------------------
// 構造の判定（接触マニフォールドを辺とするグラフ）
//   頂点 1                          → 単体
//   全頂点の次数 2、辺数 = 頂点数    → 輪
//   次数 1 が 2 個、残りが次数 2     → 鎖
//   それ以外                        → その他（枝分かれ・塊）
// ---------------------------------------------------------------------------
std::vector<MagnetSystem::Structure> MagnetSystem::structures(const std::vector<Manifold>& manifolds) const {
    const int n = (int)bodies_.size();
    std::vector<std::vector<int>> adj(n);
    for (const Manifold& mf : manifolds) {
        int ia = indexOf(mf.a), ib = indexOf(mf.b);
        if (ia < 0 || ib < 0) continue;
        adj[ia].push_back(ib);
        adj[ib].push_back(ia);
    }

    std::vector<Structure> out;
    std::vector<char> seen(n, 0);
    for (int s = 0; s < n; ++s) {
        if (seen[s]) continue;
        Structure st;
        std::vector<int> stack{s};
        seen[s] = 1;
        int degreeSum = 0, deg1 = 0, deg2 = 0;
        while (!stack.empty()) {
            int v = stack.back(); stack.pop_back();
            st.members.push_back(v);
            int d = (int)adj[v].size();
            degreeSum += d;
            if (d == 1) ++deg1;
            if (d == 2) ++deg2;
            for (int w : adj[v])
                if (!seen[w]) { seen[w] = 1; stack.push_back(w); }
        }
        int V = (int)st.members.size();
        int E = degreeSum / 2;
        if (V == 1)                                   st.type = StructureType::Single;
        else if (deg2 == V && E == V)                 st.type = StructureType::Ring;
        else if (deg1 == 2 && deg2 == V - 2 && E == V - 1) st.type = StructureType::Chain;
        else                                          st.type = StructureType::Other;
        out.push_back(std::move(st));
    }
    return out;
}

} // namespace phys
