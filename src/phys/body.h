#pragma once
// ---------------------------------------------------------------------------
// body.h : 形状 (Shape) と剛体 (RigidBody)
// ---------------------------------------------------------------------------
#include "phys/math3d.h"

namespace phys {

enum class ShapeType { Sphere, Box };

struct Shape {
    ShapeType type = ShapeType::Box;
    float     radius = 0.5f;              // Sphere 用
    Vec3      halfExtents{0.5f, 0.5f, 0.5f}; // Box 用（半分の辺長）

    static Shape sphere(float r) {
        Shape s; s.type = ShapeType::Sphere; s.radius = r; return s;
    }
    static Shape box(const Vec3& he) {
        Shape s; s.type = ShapeType::Box; s.halfExtents = he; return s;
    }
};

// ---------------------------------------------------------------------------
// RigidBody
//   invMass == 0 の物体は静的（動かない・無限質量）とみなす。
// ---------------------------------------------------------------------------
struct RigidBody {
    int  id = 0;

    // --- 状態 ---
    Vec3 position{0, 0, 0};
    Quat orientation{};
    Vec3 velocity{0, 0, 0};        // 線速度
    Vec3 angularVelocity{0, 0, 0}; // 角速度（ワールド座標）

    // --- 蓄積される外力 ---
    Vec3 force{0, 0, 0};
    Vec3 torque{0, 0, 0};

    // --- 静的物体を動かすとき（動く台・回る扉など）の速度 ---
    //   静的物体の velocity は毎サブステップこの値になる（接触の摩擦・反発が動きを見る）。
    //   位置と姿勢は呼び出し側が書き換える
    Vec3 kinematicVelocity{0, 0, 0};
    Vec3 kinematicAngularVelocity{0, 0, 0};

    // --- 質量特性 ---
    float invMass = 1.0f;
    Vec3  invInertiaLocal{1, 1, 1}; // ローカル慣性テンソルの逆（対角のみ）
    Mat3  invInertiaWorld = Mat3::identity();

    // --- マテリアル ---
    float restitution    = 0.2f;  // 反発係数
    float friction       = 0.5f;  // 摩擦係数（滑り）
    float rollingFriction= 0.02f; // 転がり摩擦。0 にすると球は永久に転がり続ける
    float linearDamping  = 0.01f;
    float angularDamping = 0.05f;

    Shape shape;

    bool isStatic() const { return invMass == 0.0f; }

    // -----------------------------------------------------------------------
    // 質量を設定し、形状から慣性テンソルを計算する
    //   mass <= 0 を渡すと静的物体になる
    // -----------------------------------------------------------------------
    void setMass(float mass) {
        if (mass <= 0.0f) {
            invMass = 0.0f;
            invInertiaLocal = Vec3{0, 0, 0};
            invInertiaWorld = Mat3::zero();
            return;
        }
        invMass = 1.0f / mass;

        Vec3 I{1, 1, 1};
        if (shape.type == ShapeType::Sphere) {
            // 中身の詰まった球: I = 2/5 * m * r^2
            float v = 0.4f * mass * shape.radius * shape.radius;
            I = Vec3{v, v, v};
        } else {
            // 直方体: Ix = m/12 * (h^2 + d^2)  (h, d は全長)
            Vec3 s = shape.halfExtents * 2.0f;
            float k = mass / 12.0f;
            I = Vec3{ k * (s.y * s.y + s.z * s.z),
                      k * (s.x * s.x + s.z * s.z),
                      k * (s.x * s.x + s.y * s.y) };
        }
        invInertiaLocal = Vec3{ 1.0f / I.x, 1.0f / I.y, 1.0f / I.z };
        updateInertiaWorld();
    }

    // I_world^-1 = R * I_local^-1 * R^T
    void updateInertiaWorld() {
        if (isStatic()) { invInertiaWorld = Mat3::zero(); return; }
        Mat3 R = orientation.toMat3();
        Mat3 RD{ R.c0 * invInertiaLocal.x,
                 R.c1 * invInertiaLocal.y,
                 R.c2 * invInertiaLocal.z };
        invInertiaWorld = RD * R.transposed();
    }

    // -----------------------------------------------------------------------
    // 力・撃力の適用
    // -----------------------------------------------------------------------
    void applyForce(const Vec3& f) { force += f; }

    void applyForceAtPoint(const Vec3& f, const Vec3& worldPoint) {
        force  += f;
        torque += cross(worldPoint - position, f);
    }

    // r は重心からの相対ベクトル（ワールド座標）
    void applyImpulse(const Vec3& impulse, const Vec3& r) {
        if (isStatic()) return;
        velocity        += impulse * invMass;
        angularVelocity += invInertiaWorld * cross(r, impulse);
    }

    void applyImpulseAtCenter(const Vec3& impulse) {
        if (isStatic()) return;
        velocity += impulse * invMass;
    }

    // 接触点における速度（線速度 + 回転による速度）
    Vec3 velocityAt(const Vec3& r) const {
        return velocity + cross(angularVelocity, r);
    }

    // -----------------------------------------------------------------------
    // Broadphase 用 AABB
    // -----------------------------------------------------------------------
    AABB computeAABB() const {
        AABB box;
        if (shape.type == ShapeType::Sphere) {
            Vec3 r{shape.radius, shape.radius, shape.radius};
            box.min = position - r;
            box.max = position + r;
        } else {
            // 回転した OBB を包む AABB：|R| * halfExtents
            Mat3 R = orientation.toMat3();
            Vec3 he = shape.halfExtents;
            Vec3 ext{
                std::fabs(R.c0.x) * he.x + std::fabs(R.c1.x) * he.y + std::fabs(R.c2.x) * he.z,
                std::fabs(R.c0.y) * he.x + std::fabs(R.c1.y) * he.y + std::fabs(R.c2.y) * he.z,
                std::fabs(R.c0.z) * he.x + std::fabs(R.c1.z) * he.y + std::fabs(R.c2.z) * he.z };
            box.min = position - ext;
            box.max = position + ext;
        }
        return box;
    }
};

} // namespace phys
