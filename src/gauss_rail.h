#pragma once
// ---------------------------------------------------------------------------
// gauss_rail.h : ガウス加速器（シーン D）の組み立てと、1 段で出る磁気エネルギー
//   raylib に依存しないので magnet_test からも使う
//
//   レールの上に「立方体の磁石 + 鉄球 n 個」の段を間隔 S で並べる（磁石の N 極は +x）。
//   鉄球を転がして段に当てると、球は磁石に引かれて加速してぶつかり、衝突が鉄球の列を
//   伝わって、磁石からいちばん遠い（いちばん弱く引かれている）球が飛び出す。
//   出ていく球は、入ってきた球より磁石から遠いところで離れるので、差の磁気エネルギー
//   だけ速くなる。
//
//   衝突が列を伝わる動き（ニュートンのゆりかご）は、逐次インパルス法が苦手とする
//   （解が収束すると、列が 1 つの塊として動く）。今のソルバ（速度の反復 10 回）では、
//   鉄球 3 個の段が安定して次の段へ球を送る（magnet_test G1）。
// ---------------------------------------------------------------------------
#include "demo_common.h"

#include <vector>

namespace demo {

struct GaussParams {
    int   stages  = 5;      // 段の数
    int   balls   = 3;      // 1 段あたりの鉄球の数
    float spacing = 5.0f;   // 段の間隔（隣の段の磁石に引かれない距離）
};

struct GaussRail {
    std::vector<RigidBody*> magnets;   // 段ごとの磁石
    std::vector<RigidBody*> balls;     // 段ごとに balls 個ずつ（磁石に近い順）
    std::vector<RigidBody*> rails;     // レールの壁と両端の止め（静的）
    std::vector<float>      stageX;    // 各段の磁石の x
    RigidBody* launch = nullptr;       // 最初に転がす球
};

inline const float GAUSS_RAIL_INNER = MAG_RADIUS + 0.02f;   // レールの内側の半幅
inline const float GAUSS_RAIL_WALL  = 0.05f;                // 壁の半分の厚さ
inline const float GAUSS_LAUNCH_GAP = 3.5f;                 // 最初の球と 1 段目の磁石の距離

// 磁石の姿勢（ローカル +y の N 極を +x に向ける）
inline Quat gaussMagnetOrientation() { return Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI); }

// レールと段を作る（段の並びの中心が x = 0）
inline GaussRail buildGaussRail(World& w, const GaussParams& p) {
    GaussRail r;
    const float d  = 2.0f * MAG_RADIUS;
    const float x0 = -0.5f * (p.stages - 1) * p.spacing;
    for (int k = 0; k < p.stages; ++k) r.stageX.push_back(x0 + k * p.spacing);

    // レール: 高さ 0.3 の低い壁 2 枚（摩擦 0）と、両端の止め
    const float xs = x0 - GAUSS_LAUNCH_GAP - 1.5f, xe = r.stageX.back() + 8.0f;
    const float zw = GAUSS_RAIL_INNER + GAUSS_RAIL_WALL;
    for (float s : {-1.0f, 1.0f}) {
        RigidBody* wall = w.createBox({0.5f * (xs + xe), 0.15f, s * zw},
                                      {0.5f * (xe - xs), 0.15f, GAUSS_RAIL_WALL}, 0.0f);
        wall->friction = 0.0f; wall->restitution = 0.0f;
        r.rails.push_back(wall);
    }
    for (float x : {xs, xe}) {
        RigidBody* stop = w.createBox({x, 0.3f, 0}, {0.1f, 0.3f, zw + GAUSS_RAIL_WALL}, 0.0f);
        stop->friction = 0.5f; stop->restitution = 0.1f;
        r.rails.push_back(stop);
    }

    for (float x : r.stageX) {
        RigidBody* m = makeMagnetCube(w, {x, CUBE_HALF, 0}, gaussMagnetOrientation());
        m->restitution = 0.9f;   // 鋼球とぶつかる硬い磁石
        m->friction    = 0.8f;   // 反動で下がらないように（実物はテープで留める）
        r.magnets.push_back(m);
        for (int j = 1; j <= p.balls; ++j) r.balls.push_back(makeIronBall(w, {x + j * d, MAG_RADIUS, 0}));
    }
    r.launch = makeIronBall(w, {x0 - GAUSS_LAUNCH_GAP, MAG_RADIUS, 0});
    w.magnets().updateMoments(10);
    return r;
}

// 測る点 c の x。0 は 1 段目の手前、c >= 1 は段 c の後ろ（次の段との中間）
inline float gaussCheckpointX(const GaussParams& p, int c) {
    return -0.5f * (p.stages - 1) * p.spacing + (c - 0.5f) * p.spacing;
}

// 測る点を前向きに通った最初の球の速さ（-1: まだ通っていない）
struct GaussMeter {
    std::vector<float> speeds;
    std::vector<float> prevX;   // 物体ごとの、前に見たときの x

    void reset(const GaussParams& p) { speeds.assign(p.stages + 1, -1.0f); prevX.clear(); }

    // 各ステップの前に呼ぶ（初めて見る物体は位置を覚えるだけ）
    void update(const World& w, const GaussParams& p) {
        const auto&  bodies = w.getBodies();
        const size_t seen   = prevX.size();
        prevX.resize(bodies.size());
        for (size_t i = 0; i < bodies.size(); ++i) {
            const RigidBody* b = bodies[i].get();
            if (i < seen && !b->isStatic() && b->shape.type == ShapeType::Sphere)
                for (int c = 0; c <= p.stages; ++c) {
                    const float xc = gaussCheckpointX(p, c);
                    if (speeds[c] < 0.0f && prevX[i] < xc && b->position.x >= xc) speeds[c] = b->velocity.x;
                }
            prevX[i] = b->position.x;
        }
    }

    // 最初から続けて通った測る点の数
    int passed() const {
        int n = 0;
        while (n < (int)speeds.size() && speeds[n] > 0.0f) ++n;
        return n;
    }
};

// 1 段で出る磁気エネルギー（損失がなければ、出ていく球の運動エネルギーがこれだけ増える）
//   前: 磁石 + 鉄球 n 個、入ってくる球は遠く    後: 入ってきた球が磁石に接し、いちばん外の球は遠く
inline float gaussStageEnergy(int balls) {
    World w;
    const float d = 2.0f * MAG_RADIUS, far = 1000.0f;
    makeMagnetCube(w, {0, 0, 0}, gaussMagnetOrientation());
    std::vector<RigidBody*> row;
    for (int j = 1; j <= balls; ++j) row.push_back(makeIronBall(w, {j * d, 0, 0}));
    RigidBody* in = makeIronBall(w, {-far, 0, 0});
    w.magnets().updateMoments(50);
    const float before = w.magnets().potentialEnergy();
    in->position         = Vec3{-d, 0, 0};
    row.back()->position = Vec3{far, 0, 0};
    w.magnets().updateMoments(50);
    const float after = w.magnets().potentialEnergy();
    return before - after;
}

} // namespace demo
