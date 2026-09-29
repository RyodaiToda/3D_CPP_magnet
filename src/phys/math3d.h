#pragma once
// ---------------------------------------------------------------------------
// math3d.h : 物理エンジン用の最小限の 3D 数学ライブラリ
//   raylib には依存しない（物理コアを描画から切り離すため）
// ---------------------------------------------------------------------------
#include <cmath>
#include <algorithm>

namespace phys {

constexpr float PHYS_PI = 3.14159265358979323846f;

// ===========================================================================
// Vec3
// ===========================================================================
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s)       const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s)       const { return {x / s, y / s, z / s}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s)       { x *= s; y *= s; z *= s; return *this; }

    float  operator[](int i) const { return (&x)[i]; }
    float& operator[](int i)       { return (&x)[i]; }
};

inline Vec3  operator*(float s, const Vec3& v) { return v * s; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}

inline float lengthSq(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v)   { return std::sqrt(dot(v, v)); }

inline Vec3 normalize(const Vec3& v) {
    float len = length(v);
    return (len > 1e-8f) ? v / len : Vec3{0, 0, 0};
}

inline Vec3 absVec(const Vec3& v) {
    return { std::fabs(v.x), std::fabs(v.y), std::fabs(v.z) };
}

// n に直交する正規直交な接線 2 本を作る（摩擦の 2 方向に使う）
inline void buildTangents(const Vec3& n, Vec3& t1, Vec3& t2) {
    // n の成分のうち最も小さい軸を選ぶと数値的に安定
    if (std::fabs(n.x) >= 0.57735f) t1 = normalize(Vec3{ n.y, -n.x, 0.0f });
    else                            t1 = normalize(Vec3{ 0.0f, n.z, -n.y });
    t2 = cross(n, t1);
}

// ===========================================================================
// Mat3 : 3x3 行列。「列」を 3 本の Vec3 として保持する。
//        M * v = c0*v.x + c1*v.y + c2*v.z
//        回転行列の場合、c0/c1/c2 はそのままローカル基底軸のワールド表現になる。
// ===========================================================================
struct Mat3 {
    Vec3 c0{1, 0, 0}, c1{0, 1, 0}, c2{0, 0, 1};

    Mat3() = default;
    Mat3(const Vec3& a, const Vec3& b, const Vec3& c) : c0(a), c1(b), c2(c) {}

    const Vec3& col(int i) const { return (i == 0) ? c0 : (i == 1) ? c1 : c2; }

    Vec3 operator*(const Vec3& v) const { return c0 * v.x + c1 * v.y + c2 * v.z; }
    Mat3 operator*(const Mat3& o) const { return { (*this) * o.c0, (*this) * o.c1, (*this) * o.c2 }; }

    Mat3 transposed() const {
        return { Vec3{c0.x, c1.x, c2.x},
                 Vec3{c0.y, c1.y, c2.y},
                 Vec3{c0.z, c1.z, c2.z} };
    }

    Mat3 operator+(const Mat3& o) const { return { c0 + o.c0, c1 + o.c1, c2 + o.c2 }; }
    Mat3 operator-(const Mat3& o) const { return { c0 - o.c0, c1 - o.c1, c2 - o.c2 }; }
    Mat3 operator*(float s)       const { return { c0 * s, c1 * s, c2 * s }; }

    // 余因子による逆行列（ジャイロスコープ項の陰的解法で使う）
    Mat3 inverse() const {
        Vec3 r0 = cross(c1, c2);
        Vec3 r1 = cross(c2, c0);
        Vec3 r2 = cross(c0, c1);
        float det = dot(c0, r0);
        if (std::fabs(det) < 1e-12f) return zero();
        float id = 1.0f / det;
        return { Vec3{r0.x, r1.x, r2.x} * id,
                 Vec3{r0.y, r1.y, r2.y} * id,
                 Vec3{r0.z, r1.z, r2.z} * id };
    }

    static Mat3 zero()     { return { Vec3{0,0,0}, Vec3{0,0,0}, Vec3{0,0,0} }; }
    static Mat3 identity() { return {}; }
    static Mat3 diagonal(const Vec3& d) {
        return { Vec3{d.x, 0, 0}, Vec3{0, d.y, 0}, Vec3{0, 0, d.z} };
    }
    // 外積行列： skew(v) * u == cross(v, u)
    static Mat3 skew(const Vec3& v) {
        return { Vec3{0, v.z, -v.y}, Vec3{-v.z, 0, v.x}, Vec3{v.y, -v.x, 0} };
    }
};

// ===========================================================================
// Quat : 姿勢を表す単位クォータニオン (x, y, z, w)
// ===========================================================================
struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat fromAxisAngle(const Vec3& axis, float radians) {
        Vec3 a = normalize(axis);
        float h = radians * 0.5f;
        float s = std::sin(h);
        return { a.x * s, a.y * s, a.z * s, std::cos(h) };
    }

    // ハミルトン積 (this の後に o ではなく、this * o = 「先に o、後に this」)
    Quat operator*(const Quat& o) const {
        return { w * o.x + x * o.w + y * o.z - z * o.y,
                 w * o.y - x * o.z + y * o.w + z * o.x,
                 w * o.z + x * o.y - y * o.x + z * o.w,
                 w * o.w - x * o.x - y * o.y - z * o.z };
    }

    Quat operator*(float s) const { return { x * s, y * s, z * s, w * s }; }
    Quat operator+(const Quat& o) const { return { x + o.x, y + o.y, z + o.z, w + o.w }; }

    void normalizeInPlace() {
        float len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len > 1e-8f) { float inv = 1.0f / len; x *= inv; y *= inv; z *= inv; w *= inv; }
        else { x = y = z = 0.0f; w = 1.0f; }
    }

    Vec3 rotate(const Vec3& v) const {
        // v' = v + 2w(q x v) + 2(q x (q x v))
        Vec3 q{x, y, z};
        Vec3 t = cross(q, v) * 2.0f;
        return v + t * w + cross(q, t);
    }

    Mat3 toMat3() const {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        return { Vec3{1 - 2 * (yy + zz),     2 * (xy + wz),     2 * (xz - wy)},   // 列 0
                 Vec3{    2 * (xy - wz), 1 - 2 * (xx + zz),     2 * (yz + wx)},   // 列 1
                 Vec3{    2 * (xz + wy),     2 * (yz - wx), 1 - 2 * (xx + yy)} }; // 列 2
    }

    // 角速度 w による 1 ステップ積分： q += 0.5 * (w_quat * q) * dt
    void integrate(const Vec3& omega, float dt) {
        Quat wq{omega.x, omega.y, omega.z, 0.0f};
        Quat dq = wq * (*this);
        x += dq.x * 0.5f * dt;
        y += dq.y * 0.5f * dt;
        z += dq.z * 0.5f * dt;
        w += dq.w * 0.5f * dt;
        normalizeInPlace();
    }
};

// ===========================================================================
// AABB : Broadphase 用の軸平行境界ボックス
// ===========================================================================
struct AABB {
    Vec3 min{0, 0, 0}, max{0, 0, 0};

    bool overlaps(const AABB& o) const {
        return (min.x <= o.max.x && max.x >= o.min.x) &&
               (min.y <= o.max.y && max.y >= o.min.y) &&
               (min.z <= o.max.z && max.z >= o.min.z);
    }
    void expand(float m) {
        min -= Vec3{m, m, m};
        max += Vec3{m, m, m};
    }
};

inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

} // namespace phys
