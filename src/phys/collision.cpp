// ---------------------------------------------------------------------------
// collision.cpp : Narrowphase（接触検出とマニフォールド生成）
//   - 球 vs 球
//   - 球 vs OBB
//   - OBB vs OBB   ... 15 軸の SAT ＋ 参照面/入射面クリッピング
// ---------------------------------------------------------------------------
#include "phys/collision.h"

#include <cfloat>
#include <cmath>

namespace phys {

// ===========================================================================
// 小さなヘルパ
// ===========================================================================

// 剛体ローカル座標へ変換（ウォームスタートの接触点マッチングに使う）
static inline Vec3 toLocal(const RigidBody* b, const Vec3& world) {
    Mat3 Rt = b->orientation.toMat3().transposed();
    return Rt * (world - b->position);
}

static void finalizeContacts(Manifold& m) {
    m.restitution = std::max(m.a->restitution, m.b->restitution);
    m.friction    = std::sqrt(m.a->friction * m.b->friction);
    m.rollingFriction = std::sqrt(m.a->rollingFriction * m.b->rollingFriction);
    buildTangents(m.normal, m.tangent[0], m.tangent[1]);
    for (int i = 0; i < m.count; ++i) {
        m.contacts[i].localA = toLocal(m.a, m.contacts[i].position);
        m.contacts[i].localB = toLocal(m.b, m.contacts[i].position);
    }
}

// ===========================================================================
// 球 vs 球
// ===========================================================================
static bool collideSphereSphere(RigidBody* a, RigidBody* b, Manifold& m) {
    Vec3  d  = b->position - a->position;
    float rr = a->shape.radius + b->shape.radius;
    float d2 = lengthSq(d);
    if (d2 > rr * rr) return false;

    float dist = std::sqrt(d2);
    Vec3  n    = (dist > 1e-6f) ? d / dist : Vec3{0, 1, 0}; // 完全に重なった場合の保険

    m.a = a; m.b = b;
    m.normal = n;
    m.count  = 1;
    m.contacts[0] = Contact{};
    m.contacts[0].penetration = rr - dist;
    m.contacts[0].position    = a->position + n * (a->shape.radius - (rr - dist) * 0.5f);
    return true;
}

// ===========================================================================
// 球 vs OBB
//   sphereIsA == true なら m.a が球、false なら m.a が箱
// ===========================================================================
static bool collideSphereBox(RigidBody* sph, RigidBody* box, Manifold& m, bool sphereIsA) {
    Mat3  R  = box->orientation.toMat3();
    Mat3  Rt = R.transposed();
    Vec3  he = box->shape.halfExtents;
    float r  = sph->shape.radius;

    // 球の中心を箱のローカル座標へ
    Vec3 local = Rt * (sph->position - box->position);

    Vec3 clamped{ clampf(local.x, -he.x, he.x),
                  clampf(local.y, -he.y, he.y),
                  clampf(local.z, -he.z, he.z) };

    bool centerInside = (clamped.x == local.x && clamped.y == local.y && clamped.z == local.z);

    Vec3  nBoxToSphere;  // 箱 → 球 の向き
    float penetration;
    Vec3  contactPoint;

    if (!centerInside) {
        Vec3  closestWorld = box->position + R * clamped;
        Vec3  delta        = sph->position - closestWorld;
        float d2           = lengthSq(delta);
        if (d2 > r * r) return false;

        float dist = std::sqrt(d2);
        nBoxToSphere = (dist > 1e-6f) ? delta / dist : Vec3{0, 1, 0};
        penetration  = r - dist;
        contactPoint = closestWorld;
    } else {
        // 中心が箱の内部：最も近い面へ押し出す
        int   bestAxis  = 0;
        float bestDepth = FLT_MAX;
        for (int k = 0; k < 3; ++k) {
            float depth = he[k] - std::fabs(local[k]);
            if (depth < bestDepth) { bestDepth = depth; bestAxis = k; }
        }
        float sign = (local[bestAxis] >= 0.0f) ? 1.0f : -1.0f;
        nBoxToSphere = R.col(bestAxis) * sign;

        Vec3 surf = local;
        surf[bestAxis] = sign * he[bestAxis];
        contactPoint = box->position + R * surf;
        penetration  = r + bestDepth;
    }

    if (sphereIsA) { m.a = sph; m.b = box; m.normal = -nBoxToSphere; }
    else           { m.a = box; m.b = sph; m.normal =  nBoxToSphere; }

    m.count = 1;
    m.contacts[0] = Contact{};
    m.contacts[0].penetration = penetration;
    m.contacts[0].position    = contactPoint;
    return true;
}

// ===========================================================================
// OBB vs OBB
// ===========================================================================

// 箱 b の「軸 axis・符号 sign」側の面を構成する 4 頂点（ワールド座標、順序は周回）
static void boxFaceVertices(const RigidBody* b, int axis, float sign, Vec3 out[4]) {
    Mat3 R  = b->orientation.toMat3();
    Vec3 he = b->shape.halfExtents;

    int u = (axis + 1) % 3;
    int v = (axis + 2) % 3;

    Vec3 center = b->position + R.col(axis) * (sign * he[axis]);
    Vec3 du = R.col(u) * he[u];
    Vec3 dv = R.col(v) * he[v];

    out[0] = center - du - dv;
    out[1] = center + du - dv;
    out[2] = center + du + dv;
    out[3] = center - du + dv;
}

// 平面 dot(n, p) <= d の内側で多角形をクリップ（Sutherland–Hodgman）
static int clipPolygonByPlane(const Vec3* in, int n, const Vec3& planeN, float planeD, Vec3* out) {
    int outCount = 0;
    for (int i = 0; i < n; ++i) {
        const Vec3& cur  = in[i];
        const Vec3& next = in[(i + 1) % n];

        float dCur  = dot(planeN, cur)  - planeD;
        float dNext = dot(planeN, next) - planeD;

        if (dCur <= 0.0f) out[outCount++] = cur;

        // 平面をまたぐ辺は交点を追加
        if ((dCur < 0.0f && dNext > 0.0f) || (dCur > 0.0f && dNext < 0.0f)) {
            float t = dCur / (dCur - dNext);
            out[outCount++] = cur + (next - cur) * t;
        }
    }
    return outCount;
}

// 接触点が 4 個を超えたら「最も深い点」＋「互いに離れた点」を選んで削る
static void reduceContacts(Vec3* pts, float* depths, int& count) {
    if (count <= MAX_CONTACT_POINTS) return;

    int  chosen[MAX_CONTACT_POINTS];
    bool used[32] = {false};

    // 1) 最も深くめり込んでいる点
    int best = 0;
    for (int i = 1; i < count; ++i) if (depths[i] > depths[best]) best = i;
    chosen[0] = best;
    used[best] = true;

    // 2) 既選択点からの最小距離が最大になる点を貪欲に追加（接触面を広く覆う）
    for (int k = 1; k < MAX_CONTACT_POINTS; ++k) {
        int   pick     = -1;
        float pickDist = -1.0f;
        for (int i = 0; i < count; ++i) {
            if (used[i]) continue;
            float minD = FLT_MAX;
            for (int j = 0; j < k; ++j)
                minD = std::min(minD, lengthSq(pts[i] - pts[chosen[j]]));
            if (minD > pickDist) { pickDist = minD; pick = i; }
        }
        chosen[k] = pick;
        used[pick] = true;
    }

    Vec3  tmpP[MAX_CONTACT_POINTS];
    float tmpD[MAX_CONTACT_POINTS];
    for (int k = 0; k < MAX_CONTACT_POINTS; ++k) { tmpP[k] = pts[chosen[k]]; tmpD[k] = depths[chosen[k]]; }
    for (int k = 0; k < MAX_CONTACT_POINTS; ++k) { pts[k] = tmpP[k]; depths[k] = tmpD[k]; }
    count = MAX_CONTACT_POINTS;
}

static bool collideBoxBox(RigidBody* A, RigidBody* B, Manifold& m) {
    Mat3 RA = A->orientation.toMat3();
    Mat3 RB = B->orientation.toMat3();
    Vec3 ea = A->shape.halfExtents;
    Vec3 eb = B->shape.halfExtents;

    Vec3 dWorld = B->position - A->position;

    // A のローカル系で相対姿勢と相対位置を表す
    float R[3][3], AbsR[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            R[i][j]    = dot(RA.col(i), RB.col(j));
            AbsR[i][j] = std::fabs(R[i][j]) + 1e-6f; // 平行軸での 0 割り回避
        }

    Vec3 t{ dot(RA.col(0), dWorld), dot(RA.col(1), dWorld), dot(RA.col(2), dWorld) };

    // ---- SAT ----------------------------------------------------------
    //  axisType: 0 = A の面, 1 = B の面, 2 = エッジ同士
    int   bestType = -1, bestI = 0, bestJ = 0;
    float bestOverlap = FLT_MAX;   // 実際のめり込み量
    float bestScore   = FLT_MAX;   // 比較用（面を優先するようバイアスを掛ける）

    // (1) A の 3 軸
    for (int i = 0; i < 3; ++i) {
        float ra = ea[i];
        float rb = eb[0] * AbsR[i][0] + eb[1] * AbsR[i][1] + eb[2] * AbsR[i][2];
        float overlap = ra + rb - std::fabs(t[i]);
        if (overlap < 0.0f) return false;                  // 分離軸を発見
        if (overlap < bestScore) { bestScore = overlap; bestOverlap = overlap; bestType = 0; bestI = i; }
    }

    // (2) B の 3 軸
    for (int j = 0; j < 3; ++j) {
        float ra = ea[0] * AbsR[0][j] + ea[1] * AbsR[1][j] + ea[2] * AbsR[2][j];
        float rb = eb[j];
        float proj = std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
        float overlap = ra + rb - proj;
        if (overlap < 0.0f) return false;
        float score = overlap * 1.005f + 1e-4f;            // わずかに A の面を優先
        if (score < bestScore) { bestScore = score; bestOverlap = overlap; bestType = 1; bestJ = j; }
    }

    // (3) エッジ x エッジ（9 通り）
    for (int i = 0; i < 3; ++i) {
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (int j = 0; j < 3; ++j) {
            int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

            float axisLen2 = 1.0f - R[i][j] * R[i][j];
            if (axisLen2 < 1e-6f) continue;                // 軸がほぼ平行 → 面テストで足りる

            float ra = ea[i1] * AbsR[i2][j] + ea[i2] * AbsR[i1][j];
            float rb = eb[j1] * AbsR[i][j2] + eb[j2] * AbsR[i][j1];
            float proj = std::fabs(t[i2] * R[i1][j] - t[i1] * R[i2][j]);
            float overlap = ra + rb - proj;
            if (overlap < 0.0f) return false;

            // 非正規化の軸で測っているので長さで割って正規化する
            float invLen = 1.0f / std::sqrt(axisLen2);
            float realOverlap = overlap * invLen;
            float score = realOverlap * 1.05f + 1e-3f;     // 面接触を優先（エッジは不安定になりやすい）
            if (score < bestScore) {
                bestScore = score; bestOverlap = realOverlap;
                bestType = 2; bestI = i; bestJ = j;
            }
        }
    }

    if (bestType < 0) return false;

    // ---- 衝突法線（A → B の向き）を決める ------------------------------
    Vec3 normal;
    if (bestType == 0) {
        normal = RA.col(bestI) * ((t[bestI] >= 0.0f) ? 1.0f : -1.0f);
    } else if (bestType == 1) {
        float proj = dot(dWorld, RB.col(bestJ));
        normal = RB.col(bestJ) * ((proj >= 0.0f) ? 1.0f : -1.0f);
    } else {
        normal = normalize(cross(RA.col(bestI), RB.col(bestJ)));
        if (dot(normal, dWorld) < 0.0f) normal = -normal;
    }

    m.a = A; m.b = B;
    m.normal = normal;
    m.count  = 0;

    // ---- エッジ x エッジ：接触点は 1 つ --------------------------------
    if (bestType == 2) {
        Vec3 eAxis = RA.col(bestI);
        Vec3 fAxis = RB.col(bestJ);

        // 法線方向の支持点（衝突しているエッジ上の点）を求める
        Vec3 pA = A->position;
        for (int k = 0; k < 3; ++k)
            if (k != bestI) pA += RA.col(k) * ((dot(RA.col(k), normal) > 0.0f) ? ea[k] : -ea[k]);

        Vec3 pB = B->position;
        for (int k = 0; k < 3; ++k)
            if (k != bestJ) pB += RB.col(k) * ((dot(RB.col(k), normal) < 0.0f) ? eb[k] : -eb[k]);

        // 2 直線の最近接点
        Vec3  r = pA - pB;
        float b = dot(eAxis, fAxis);
        float c = dot(eAxis, r);
        float f = dot(fAxis, r);
        float denom = 1.0f - b * b;

        Vec3 point;
        if (std::fabs(denom) < 1e-6f) {
            point = (pA + pB) * 0.5f;
        } else {
            float s = (b * f - c) / denom;
            float u = (f - b * c) / denom;
            point = ((pA + eAxis * s) + (pB + fAxis * u)) * 0.5f;
        }

        m.count = 1;
        m.contacts[0] = Contact{};
        m.contacts[0].position    = point;
        m.contacts[0].penetration = bestOverlap;
        finalizeContacts(m);
        return true;
    }

    // ---- 面接触：参照面に入射面をクリップする --------------------------
    RigidBody* ref;
    RigidBody* inc;
    Vec3       refNormal;   // 参照物体から相手へ向かう向き
    int        refAxis;

    if (bestType == 0) { ref = A; inc = B; refNormal =  normal; refAxis = bestI; }
    else               { ref = B; inc = A; refNormal = -normal; refAxis = bestJ; }

    Mat3 Rref = ref->orientation.toMat3();
    Mat3 Rinc = inc->orientation.toMat3();
    Vec3 heRef = ref->shape.halfExtents;

    float refSign = (dot(Rref.col(refAxis), refNormal) >= 0.0f) ? 1.0f : -1.0f;

    // 入射面 = refNormal と最も逆向きの面
    int   incAxis = 0;
    float incSign = 1.0f;
    float minDot  = FLT_MAX;
    for (int k = 0; k < 3; ++k) {
        for (float s : {1.0f, -1.0f}) {
            float d = dot(Rinc.col(k) * s, refNormal);
            if (d < minDot) { minDot = d; incAxis = k; incSign = s; }
        }
    }

    Vec3 poly[16], buf[16];
    boxFaceVertices(inc, incAxis, incSign, poly);
    int polyCount = 4;

    // 参照面の側面 4 平面でクリップ
    int u = (refAxis + 1) % 3;
    int v = (refAxis + 2) % 3;
    for (int axis : {u, v}) {
        Vec3  an = Rref.col(axis);
        float ac = dot(an, ref->position);

        polyCount = clipPolygonByPlane(poly, polyCount,  an,  ac + heRef[axis], buf);
        if (polyCount == 0) return false;
        for (int i = 0; i < polyCount; ++i) poly[i] = buf[i];

        polyCount = clipPolygonByPlane(poly, polyCount, -an, -ac + heRef[axis], buf);
        if (polyCount == 0) return false;
        for (int i = 0; i < polyCount; ++i) poly[i] = buf[i];
    }

    // 参照面より内側（めり込んでいる）点だけ残す
    Vec3  refFaceCenter = ref->position + Rref.col(refAxis) * (refSign * heRef[refAxis]);
    float refPlaneD     = dot(refNormal, refFaceCenter);

    Vec3  keptPts[16];
    float keptDepth[16];
    int   keptCount = 0;
    for (int i = 0; i < polyCount; ++i) {
        float sep = dot(refNormal, poly[i]) - refPlaneD;
        if (sep <= 0.0f) {
            // 2 面の中間に接触点を置く（描画上も自然になる）
            keptPts[keptCount]   = poly[i] - refNormal * (sep * 0.5f);
            keptDepth[keptCount] = -sep;
            ++keptCount;
        }
    }
    if (keptCount == 0) return false;

    reduceContacts(keptPts, keptDepth, keptCount);

    m.count = keptCount;
    for (int i = 0; i < keptCount; ++i) {
        m.contacts[i] = Contact{};
        m.contacts[i].position    = keptPts[i];
        m.contacts[i].penetration = keptDepth[i];
    }
    finalizeContacts(m);
    return true;
}

// ===========================================================================
// ディスパッチ
// ===========================================================================
bool collide(RigidBody* a, RigidBody* b, Manifold& m) {
    ShapeType ta = a->shape.type;
    ShapeType tb = b->shape.type;

    if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
        if (!collideSphereSphere(a, b, m)) return false;
        finalizeContacts(m);
        return true;
    }
    if (ta == ShapeType::Sphere && tb == ShapeType::Box) {
        if (!collideSphereBox(a, b, m, /*sphereIsA=*/true)) return false;
        finalizeContacts(m);
        return true;
    }
    if (ta == ShapeType::Box && tb == ShapeType::Sphere) {
        if (!collideSphereBox(b, a, m, /*sphereIsA=*/false)) return false;
        finalizeContacts(m);
        return true;
    }
    return collideBoxBox(a, b, m); // 内部で finalizeContacts 済み
}

} // namespace phys
