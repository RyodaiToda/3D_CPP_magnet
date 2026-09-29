#pragma once
// ---------------------------------------------------------------------------
// collision.h : 接触点 / マニフォールド と Narrowphase の宣言
// ---------------------------------------------------------------------------
#include "phys/body.h"

namespace phys
{

    constexpr int MAX_CONTACT_POINTS = 4;

    // ---------------------------------------------------------------------------
    // 1 つの接触点
    // ---------------------------------------------------------------------------
    struct Contact
    {
        Vec3 position{0, 0, 0};   // ワールド座標の接触点
        float penetration = 0.0f; // めり込み量（正の値）

        // ウォームスタート時に「前フレームの同じ接触点」を探すためのローカル座標
        Vec3 localA{0, 0, 0};
        Vec3 localB{0, 0, 0};

        // --- ソルバが使う蓄積インパルス（ウォームスタートで引き継ぐ） ---
        float normalImpulse = 0.0f;
        float tangentImpulse[2] = {0.0f, 0.0f};

        // --- 毎フレーム再計算する一時データ ---
        Vec3 rA{0, 0, 0}, rB{0, 0, 0}; // 重心から接触点へのベクトル
        float normalMass = 0.0f;
        float tangentMass[2] = {0.0f, 0.0f};
        float velocityBias = 0.0f; // 反発によるバイアス
    };

    // ---------------------------------------------------------------------------
    // 2 物体間の接触マニフォールド
    //   normal は A から B へ向かう向きに正規化されている
    // ---------------------------------------------------------------------------
    struct Manifold
    {
        RigidBody *a = nullptr;
        RigidBody *b = nullptr;

        Vec3 normal{0, 1, 0};
        Vec3 tangent[2];

        Contact contacts[MAX_CONTACT_POINTS];
        int count = 0;

        float restitution = 0.0f;
        float friction = 0.0f;

        // 転がり摩擦（法線まわり = スピン、接線まわり = 転がり）
        float rollingFriction = 0.0f;
        float rollingImpulse[3] = {0.0f, 0.0f, 0.0f};
    };

    // Narrowphase: 接触があれば true を返し m を埋める
    bool collide(RigidBody *a, RigidBody *b, Manifold &m);

} // namespace phys
