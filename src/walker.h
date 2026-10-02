#pragma once
// ---------------------------------------------------------------------------
// walker.h : FPS 視点で地面を歩く人（raylib に依存しない）
//   静的な箱の上に立ち、静的な箱（壁）にぶつかる。動く物体とは当たらない。
//   動きは見やすさ優先の値（シミュレーションの時間の速さとは無関係）
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace demo {

using namespace phys;

struct Walker {
    Vec3  feet{0, 0, 0};
    float velY     = 0.0f;
    bool  onGround = false;
    float eye      = 1.6f;   // 足元から目までの高さ（しゃがむと下がる）

    static constexpr float EYE        = 1.6f;
    static constexpr float CROUCH_EYE = 0.6f;
    static constexpr float RADIUS     = 0.3f;
    static constexpr float STEP       = 0.35f;   // この高さまでの段は乗り越える
    static constexpr float GRAVITY    = 25.0f;
    static constexpr float JUMP       = 7.0f;

    // from から真下に向けた光線が最初に当たる静的な箱の面の高さ（なければ -1e9）
    static float groundBelow(const World& w, const Vec3& from) {
        float best = -1e9f;
        for (const auto& bp : w.getBodies()) {
            const RigidBody* b = bp.get();
            if (!b->isStatic() || b->shape.type != ShapeType::Box) continue;
            const Mat3  Rt = b->orientation.toMat3().transposed();
            const Vec3  o  = Rt * (from - b->position);
            const Vec3  d  = Rt * Vec3{0, -1, 0};
            const Vec3& he = b->shape.halfExtents;
            if (std::fabs(o.x) <= he.x && std::fabs(o.y) <= he.y && std::fabs(o.z) <= he.z)
                continue;   // 箱の中から（壁にめり込んだとき）。その上面に乗ると壁をよじ登ってしまう
            float t0 = 0.0f, t1 = 1e9f;
            bool  hit = true;
            for (int k = 0; k < 3 && hit; ++k) {
                if (std::fabs(d[k]) < 1e-6f) { hit = std::fabs(o[k]) <= he[k]; continue; }
                float a = (-he[k] - o[k]) / d[k], c = (he[k] - o[k]) / d[k];
                if (a > c) std::swap(a, c);
                t0 = std::max(t0, a);
                t1 = std::min(t1, c);
                hit = t0 <= t1;
            }
            if (hit) best = std::max(best, from.y - t0);
        }
        return best;
    }

    // 体（段の高さより上から目までの球 3 つ）が静的な箱にめり込んだら、水平に押し出す
    void pushOutOfWalls(const World& w) {
        for (int iter = 0; iter < 2; ++iter)
            for (const auto& bp : w.getBodies()) {
                const RigidBody* b = bp.get();
                if (!b->isStatic() || b->shape.type != ShapeType::Box) continue;
                const Mat3 R = b->orientation.toMat3();
                const float lo = STEP + RADIUS, hi = std::max(lo, eye);   // 段の高さより下は当てない
                const Vec3& he = b->shape.halfExtents;
                for (float h : {lo, 0.5f * (lo + hi), hi}) {
                    const Vec3 c{feet.x, feet.y + h, feet.z};
                    const Vec3 o = R.transposed() * (c - b->position);
                    if (std::fabs(o.x) <= he.x && std::fabs(o.y) <= he.y && std::fabs(o.z) <= he.z) {
                        // 中心が箱の中（薄い壁を抜けかけた）: 水平に近い軸のうち、いちばん浅い面から出す
                        int   best  = -1;
                        float depth = 1e9f;
                        for (int k = 0; k < 3; ++k) {
                            if (std::fabs(R.col(k).y) > 0.7f) continue;
                            const float dk = he[k] - std::fabs(o[k]);
                            if (dk < depth) { depth = dk; best = k; }
                        }
                        if (best < 0) continue;
                        Vec3 dir{R.col(best).x, 0.0f, R.col(best).z};
                        dir = normalize(dir) * (o[best] >= 0.0f ? 1.0f : -1.0f);
                        feet += dir * (depth + RADIUS);
                        continue;
                    }
                    Vec3 q = o;
                    for (int k = 0; k < 3; ++k) q[k] = clampf(q[k], -he[k], he[k]);
                    Vec3  diff = c - (b->position + R * q);
                    float dist = length(diff);
                    diff.y = 0.0f;
                    const float horiz = length(diff);
                    if (dist >= RADIUS || horiz < 1e-5f) continue;
                    feet += diff * ((RADIUS - dist) / horiz);
                }
            }
    }

    // move: 水平の移動量（このフレームぶん）
    void update(const World& w, const Vec3& move, bool jump, bool crouch, float dt) {
        eye += ((crouch ? CROUCH_EYE : EYE) - eye) * std::min(1.0f, 12.0f * dt);
        feet += move;
        pushOutOfWalls(w);
        const float ground = groundBelow(w, feet + Vec3{0, STEP, 0});
        if (onGround && jump) { velY = JUMP; onGround = false; }
        velY -= GRAVITY * dt;
        feet.y += velY * dt;
        if (feet.y <= ground) { feet.y = ground; velY = 0.0f; onGround = true; }
        else                  onGround = false;
        if (feet.y < -30.0f) { feet = Vec3{0, 10, 0}; velY = 0.0f; }   // 床の外に落ちたら戻す
    }
};

} // namespace demo
