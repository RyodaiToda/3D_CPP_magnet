// 物理コアのヘッドレス検証（raylib 不要）
#include "phys/world.h"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace phys;

static int failures = 0;
static void check(const char* name, bool ok, const char* detail = "") {
    std::printf("[%s] %-42s %s\n", ok ? " OK " : "FAIL", name, detail);
    if (!ok) ++failures;
}

static void simulate(World& w, float seconds, float dt = 1.0f / 120.0f) {
    int steps = (int)(seconds / dt);
    for (int i = 0; i < steps; ++i) w.step(dt);
}

int main() {
    char buf[256];

    // ---------------- 1. 自由落下 ----------------
    {
        World w;
        auto* s = w.createSphere({0, 100, 0}, 0.5f, 1.0f);
        s->linearDamping = 0.0f;
        simulate(w, 1.0f);
        float expectedV = -9.81f;
        float expectedY = 100.0f - 0.5f * 9.81f;
        std::snprintf(buf, sizeof buf, "y=%.3f (~%.3f) vy=%.3f (~%.3f)", s->position.y, expectedY, s->velocity.y, expectedV);
        check("free fall", std::fabs(s->velocity.y - expectedV) < 0.1f &&
                           std::fabs(s->position.y - expectedY) < 0.2f, buf);
    }

    // ---------------- 2. 球が地面で静止 ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);     // 静的地面（上面 y=0）
        auto* s = w.createSphere({0, 3, 0}, 0.5f, 1.0f);
        s->restitution = 0.0f;
        simulate(w, 4.0f);
        std::snprintf(buf, sizeof buf, "y=%.4f (~0.5) vy=%.4f", s->position.y, s->velocity.y);
        check("sphere rests on ground", std::fabs(s->position.y - 0.5f) < 0.02f &&
                                        std::fabs(s->velocity.y) < 0.05f, buf);
    }

    // ---------------- 3. 箱が地面で静止 ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        auto* b = w.createBox({0, 4, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        b->restitution = 0.0f;
        simulate(w, 4.0f);
        std::snprintf(buf, sizeof buf, "y=%.4f (~0.5) |w|=%.4f", b->position.y, length(b->angularVelocity));
        check("box rests on ground", std::fabs(b->position.y - 0.5f) < 0.02f &&
                                     length(b->angularVelocity) < 0.05f, buf);
    }

    // ---------------- 4. 5 段の積み重ねが崩れない ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        RigidBody* stack[5];
        for (int i = 0; i < 5; ++i) {
            stack[i] = w.createBox({0, 0.5f + i * 1.02f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
            stack[i]->restitution = 0.0f;
        }
        simulate(w, 6.0f);
        bool ok = true;
        float maxDrift = 0.0f;
        for (int i = 0; i < 5; ++i) {
            float horiz = std::sqrt(stack[i]->position.x * stack[i]->position.x +
                                    stack[i]->position.z * stack[i]->position.z);
            maxDrift = std::max(maxDrift, horiz);
            if (horiz > 0.15f) ok = false;
            float expectY = 0.5f + i * 1.0f;
            if (std::fabs(stack[i]->position.y - expectY) > 0.12f) ok = false;
        }
        std::snprintf(buf, sizeof buf, "top y=%.3f (~4.5) maxDrift=%.4f", stack[4]->position.y, maxDrift);
        check("5-box stack stays upright", ok, buf);
    }

    // ---------------- 5. 反発（跳ね返る） ----------------
    {
        World w;
        auto* g = w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        g->restitution  = 0.9f;
        auto* s = w.createSphere({0, 5, 0}, 0.5f, 1.0f);
        s->restitution  = 0.9f;
        s->linearDamping = 0.0f;
        // 落下高さ 4.5m、e=0.9 -> 1 回目の反発頂点は 0.5 + 4.5*0.81 = 4.15 付近
        // 連続する反発頂点が単調に減っていく（＝エネルギーが増えない）ことも確認する
        float peaks[3] = {0, 0, 0};
        int   peakIdx  = -1;
        bool  rising   = false;
        float prevY    = s->position.y;
        for (int i = 0; i < 1400; ++i) {
            w.step(1.0f / 120.0f);
            float y = s->position.y;
            if (y > prevY) { rising = true; }
            else if (rising) { // 頂点を通過
                rising = false;
                if (++peakIdx < 3) peaks[peakIdx] = prevY;
            }
            prevY = y;
        }
        bool decays = peaks[1] > 0.5f && peaks[1] < peaks[0] && peaks[2] < peaks[1];
        std::snprintf(buf, sizeof buf, "peaks: %.2f -> %.2f -> %.2f (1st expect ~4.15)",
                      peaks[0], peaks[1], peaks[2]);
        check("restitution bounces and decays", peaks[0] > 3.7f && peaks[0] < 4.4f && decays, buf);
    }

    // ---------------- 6. 摩擦で滑走が止まる ----------------
    {
        World w;
        auto* g = w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        g->friction = 0.6f;
        auto* b = w.createBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        b->friction    = 0.6f;
        b->restitution = 0.0f;
        b->velocity    = Vec3{6, 0, 0};
        simulate(w, 4.0f);
        std::snprintf(buf, sizeof buf, "x=%.2f vx=%.4f", b->position.x, b->velocity.x);
        check("friction stops sliding box", std::fabs(b->velocity.x) < 0.05f && b->position.x > 0.5f, buf);
    }

    // ---------------- 7. 摩擦 0 なら滑り続ける ----------------
    {
        World w;
        auto* g = w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        g->friction = 0.0f;
        auto* b = w.createBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        b->friction       = 0.0f;
        b->restitution    = 0.0f;
        b->linearDamping  = 0.0f;
        b->velocity       = Vec3{5, 0, 0};
        simulate(w, 2.0f);
        std::snprintf(buf, sizeof buf, "x=%.2f (~10) vx=%.3f (~5)", b->position.x, b->velocity.x);
        check("frictionless box keeps sliding", std::fabs(b->velocity.x - 5.0f) < 0.2f, buf);
    }

    // ---------------- 8. 傾いた箱が地面に落ちて安定する ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {50, 1, 50}, 0.0f);
        auto* b = w.createBox({0, 3, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        b->orientation = Quat::fromAxisAngle({1, 0.4f, 0.2f}, 0.6f);
        b->restitution = 0.0f;
        simulate(w, 8.0f);
        float speed = length(b->velocity) + length(b->angularVelocity);
        std::snprintf(buf, sizeof buf, "y=%.3f speed=%.4f", b->position.y, speed);
        check("tilted box settles", b->position.y > 0.4f && b->position.y < 0.95f && speed < 0.2f, buf);
    }

    // ---------------- 9. 角運動量：静止空間で自由回転が保存される ----------------
    {
        World w;
        w.gravity = Vec3{0, 0, 0};
        auto* b = w.createBox({0, 0, 0}, {0.5f, 1.0f, 1.5f}, 2.0f);
        b->angularDamping = 0.0f;
        b->angularVelocity = Vec3{1.0f, 3.0f, 0.5f};
        // L = I * w （ワールド）を初期値と比較
        auto angularMomentum = [](RigidBody* r) {
            Mat3 R  = r->orientation.toMat3();
            Mat3 Rt = R.transposed();
            // I_world = R * I_local * R^T,  I_local = 1/invInertiaLocal
            Vec3 Il{1.0f / r->invInertiaLocal.x, 1.0f / r->invInertiaLocal.y, 1.0f / r->invInertiaLocal.z};
            Mat3 RI{R.c0 * Il.x, R.c1 * Il.y, R.c2 * Il.z};
            return (RI * Rt) * r->angularVelocity;
        };
        Vec3 L0 = angularMomentum(b);
        simulate(w, 5.0f, 1.0f / 480.0f);
        Vec3 L1 = angularMomentum(b);
        float err = length(L1 - L0) / length(L0);
        std::snprintf(buf, sizeof buf, "relative drift=%.4f%%", err * 100.0f);
        check("angular momentum conserved", err < 0.02f, buf);
    }

    // ---------------- 10. 球の山が破綻しない（総当たりの安定性） ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {20, 1, 20}, 0.0f);
        for (int i = 0; i < 60; ++i) {
            float x = ((i * 37) % 11 - 5) * 0.35f;
            float z = ((i * 53) % 11 - 5) * 0.35f;
            auto* s = w.createSphere({x, 1.0f + i * 0.45f, z}, 0.2f, 0.5f);
            s->restitution = 0.1f;
        }
        simulate(w, 8.0f);
        bool ok = true;
        float maxY = 0.0f, maxSpeed = 0.0f;
        for (const auto& b : w.getBodies()) {
            if (b->isStatic()) continue;
            if (!std::isfinite(b->position.y) || b->position.y < -1.0f) ok = false;
            maxY = std::max(maxY, b->position.y);
            maxSpeed = std::max(maxSpeed, length(b->velocity));
        }
        std::snprintf(buf, sizeof buf, "maxY=%.2f maxSpeed=%.3f contacts=%d", maxY, maxSpeed, w.contactCount());
        check("60 spheres pile settles", ok && maxSpeed < 1.0f, buf);
    }

    // ---------------- 11. 転がり摩擦で球が止まる ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {60, 1, 60}, 0.0f);
        auto* s = w.createSphere({0, 0.5f, 0}, 0.5f, 1.0f);
        s->restitution     = 0.0f;
        s->rollingFriction = 0.1f;                 // 既定 (0.02) より強めに設定
        s->velocity        = Vec3{5, 0, 0};
        s->angularVelocity = Vec3{0, 0, -10.0f};   // v = w*r（滑りなしの転がり）
        simulate(w, 12.0f);
        std::snprintf(buf, sizeof buf, "x=%.2f v=%.4f w=%.4f",
                      s->position.x, length(s->velocity), length(s->angularVelocity));
        check("rolling friction stops a ball", length(s->velocity) < 0.15f &&
                                               length(s->angularVelocity) < 0.5f &&
                                               s->position.x > 1.0f, buf);
    }

    // ---------------- 12. 転がり摩擦 0 なら転がり続ける ----------------
    {
        World w;
        auto* g = w.createBox({0, -1, 0}, {200, 1, 200}, 0.0f);
        g->rollingFriction = 0.0f;
        auto* s = w.createSphere({0, 0.5f, 0}, 0.5f, 1.0f);
        s->rollingFriction = 0.0f;
        s->restitution     = 0.0f;
        s->linearDamping   = 0.0f;
        s->angularDamping  = 0.0f;
        s->velocity        = Vec3{5, 0, 0};
        s->angularVelocity = Vec3{0, 0, -10.0f};   // v = w*r で滑りなし
        simulate(w, 6.0f);
        std::snprintf(buf, sizeof buf, "v=%.3f (~5)", length(s->velocity));
        check("no rolling friction -> keeps rolling", std::fabs(length(s->velocity) - 5.0f) < 0.3f, buf);
    }

    // ---------------- 13. 14 段タワー（位置補正の回帰テスト） ----------------
    {
        World w;
        w.createBox({0, -1, 0}, {30, 1, 30}, 0.0f);
        std::vector<RigidBody*> tower;
        const float s = 0.6f;
        for (int i = 0; i < 14; ++i) {
            auto* b = w.createBox({0, s + i * (2 * s + 0.01f), 0}, {s, s, s}, 1.0f);
            b->friction    = 0.7f;
            b->restitution = 0.0f;
            b->orientation = Quat::fromAxisAngle({0, 1, 0}, i * 0.12f); // ねじれた塔
            tower.push_back(b);
        }
        float startTop = tower.back()->position.y;
        float peakTop  = startTop;
        for (int i = 0; i < 720; ++i) {
            w.step(1.0f / 120.0f);
            peakTop = std::max(peakTop, tower.back()->position.y);
        }
        float drift = 0.0f;
        for (auto* b : tower)
            drift = std::max(drift, std::sqrt(b->position.x * b->position.x + b->position.z * b->position.z));
        // 上向きに跳ね上がらない（＝位置補正がエネルギーを注入しない）ことも確認
        std::snprintf(buf, sizeof buf, "top %.2f -> %.2f (peak %.2f) drift=%.3f",
                      startTop, tower.back()->position.y, peakTop, drift);
        check("14-box twisted tower stands", drift < 0.2f &&
                                             peakTop < startTop + 0.02f &&
                                             tower.back()->position.y > startTop - 0.4f, buf);
    }

    std::printf("\n%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
