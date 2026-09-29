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

// 全ペアの場: B[i] = sum_{j != i} (j が i の位置に作る場)
void sumFields(const std::vector<Vec3>& x, const std::vector<float>& rad,
               const std::vector<Vec3>& m, float minDistScale, std::vector<Vec3>& B) {
    const int n = (int)x.size();
    B.assign(n, Vec3{0, 0, 0});
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            Vec3 r = x[j] - x[i];
            if (!clampSeparation(r, minDistScale * (rad[i] + rad[j]))) continue;
            B[j] += dipoleField(r, m[i]);
            B[i] += dipoleField(r, m[j]);   // 双極子の場は r の向きを反転しても同じ
        }
    }
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
    mb.m              = momentFor(mb, Vec3{0, 0, 0});
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

// m = R (p + chi * (R^T B))、|m| <= saturation
Vec3 MagnetSystem::momentFor(const MagneticBody& mb, const Vec3& B) const {
    Mat3 R  = mb.body->orientation.toMat3();
    Vec3 Bl = R.transposed() * B;
    const Vec3& chi = mb.chiLocal;
    Vec3 m = R * (mb.permanentLocal + Vec3{chi.x * Bl.x, chi.y * Bl.y, chi.z * Bl.z});
    if (mb.saturation > 0.0f) {
        float len = length(m);
        if (len > mb.saturation) m *= mb.saturation / len;
    }
    return m;
}

// 位置・半径と、前回の B から求めたモーメントを配列に写す
void MagnetSystem::gatherState(std::vector<Vec3>& x, std::vector<float>& rad,
                               std::vector<Vec3>& m) const {
    const int n = (int)bodies_.size();
    x.resize(n); rad.resize(n); m.resize(n);
    for (int i = 0; i < n; ++i) {
        const MagneticBody& mb = bodies_[i];
        x[i]   = mb.body->position;
        rad[i] = bodyRadius(*mb.body);
        m[i]   = momentFor(mb, mb.B);
    }
}

// ---------------------------------------------------------------------------
// 1 サブステップぶんの磁力
//   ペア (i, j) ごとに「i が j に作る場」と「j が i に作る場」を評価し、
//   F_j = G_i(x_j) m_j、F_i = -F_j（作用・反作用。運動量は丸め誤差を除き厳密に保存）
//   tau_j = m_j x B_i(x_j)、tau_i = m_i x B_j(x_i)
// ---------------------------------------------------------------------------
void MagnetSystem::applyForces() {
    const int n = (int)bodies_.size();
    if (n == 0) return;
    gatherState(x_, rad_, m_);
    F_.assign(n, Vec3{0, 0, 0});
    T_.assign(n, Vec3{0, 0, 0});
    B_.assign(n, Vec3{0, 0, 0});
    fixed_.resize(n);
    for (int i = 0; i < n; ++i) {
        bodies_[i].m = m_[i];
        // 動かず、場によってモーメントも変わらない物体（固定磁石）
        fixed_[i] = bodies_[i].body->isStatic() && isZero(bodies_[i].chiLocal);
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (fixed_[i] && fixed_[j]) continue;   // broadphase と同じく静的どうしは飛ばす

            Vec3 r = x_[j] - x_[i];
            if (!clampSeparation(r, minDistScale * (rad_[i] + rad_[j]))) continue;

            float invR  = 1.0f / std::sqrt(lengthSq(r));
            Vec3  nn    = r * invR;
            float invR3 = invR * invR * invR;
            const Vec3& mi = m_[i];
            const Vec3& mj = m_[j];
            float a = dot(mi, nn), b = dot(mj, nn), c = dot(mi, mj);

            Vec3 Bij = (nn * (3.0f * a) - mi) * invR3;   // i が j の位置に作る場
            Vec3 Bji = (nn * (3.0f * b) - mj) * invR3;   // j が i の位置に作る場
            Vec3 Fj  = (mj * a + mi * b + nn * (c - 5.0f * a * b)) * (3.0f * invR3 * invR);

            B_[j] += Bij;  B_[i] += Bji;
            F_[j] += Fj;   F_[i] -= Fj;
            T_[j] += cross(mj, Bij);
            T_[i] += cross(mi, Bji);
        }
    }

    // 静的剛体に加えた力は World::integrateForces が捨てる
    for (int i = 0; i < n; ++i) {
        RigidBody* b = bodies_[i].body;
        b->force  += F_[i];
        b->torque += T_[i];
        bodies_[i].B = B_[i];
    }
}

float MagnetSystem::updateMoments(int iterations) {
    std::vector<Vec3> x, m, B;
    std::vector<float> rad;
    gatherState(x, rad, m);

    float maxChange = 0.0f;
    for (int it = 0; it < iterations; ++it) {
        sumFields(x, rad, m, minDistScale, B);
        maxChange = 0.0f;
        for (int i = 0; i < (int)bodies_.size(); ++i) {
            Vec3 mNew = momentFor(bodies_[i], B[i]);
            maxChange = std::max(maxChange, length(mNew - m[i]));
            m[i] = mNew;
        }
    }
    for (int i = 0; i < (int)bodies_.size() && iterations > 0; ++i) {
        bodies_[i].m = m[i];
        bodies_[i].B = B[i];
    }
    return maxChange;
}

// U = -1/2 sum p_i . B_i
//   誘導モーメントが自己無撞着（m = p + chi B）なら、誘導に要するエネルギーまで
//   含めた線形系の全エネルギーに厳密に一致する。永久磁石だけなら通常の
//   ペアエネルギーの和 sum_{i<j} dipoleEnergy になる。
float MagnetSystem::potentialEnergy() const {
    std::vector<Vec3> x, m, B;
    std::vector<float> rad;
    gatherState(x, rad, m);
    sumFields(x, rad, m, minDistScale, B);

    float U = 0.0f;
    for (int i = 0; i < (int)bodies_.size(); ++i) {
        Vec3 p = bodies_[i].body->orientation.rotate(bodies_[i].permanentLocal);
        U -= 0.5f * dot(p, B[i]);
    }
    return U;
}

float MagnetSystem::contactPull(const RigidBody* a, const RigidBody* b) const {
    int ia = indexOf(a), ib = indexOf(b);
    if (ia < 0 || ib < 0) return 0.0f;
    float d  = bodyRadius(*a) + bodyRadius(*b);
    float d2 = d * d;
    return 6.0f * length(bodies_[ia].m) * length(bodies_[ib].m) / (d2 * d2) * (a->invMass + b->invMass);
}

Vec3 MagnetSystem::fieldAt(const Vec3& p) const {
    Vec3 B{0, 0, 0};
    for (const MagneticBody& mb : bodies_) {
        Vec3 r = p - mb.body->position;
        if (lengthSq(r) < 1e-12f) continue;
        B += dipoleField(r, momentFor(mb, mb.B));
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
