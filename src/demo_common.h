#pragma once
// ---------------------------------------------------------------------------
// demo_common.h : デモ（main.cpp と fun_scenes.cpp）で共有する磁石の設定
//
//   直径 5 mm のネオジム球（Gamma = 779）を半径 0.25 に拡大（lambda = 100）。
//   長さの単位は実物の 1 cm。重力はそのままなので、シミュレーションの 1 秒は
//   実物の 0.1 秒に当たる（MAGNET_SPEC.md §3.3）。実時間と同じ速さで見せるには、
//   1 実秒あたり TIME_SCALE_REAL = 10 秒ぶん進める。
// ---------------------------------------------------------------------------
#include "phys/world.h"

namespace demo {

using namespace phys;

inline const float MAG_RADIUS   = 0.25f;
inline const float MAG_MASS     = 0.5f;
inline const float MAG_GAMMA    = 779.0f;
inline const float MAG_BR       = 1.2f;    // 残留磁束密度 [T]（N35。Gamma = 779 に対応）
inline const int   MAG_SUBSTEPS = 8;
inline const float MAG_M0       = momentFromGamma(MAG_GAMMA, MAG_MASS, 2.0f * MAG_RADIUS, 9.81f);

// 1 実秒あたりに進めるシミュレーション時間（sqrt(lambda)）
inline const float TIME_SCALE_REAL = 10.0f;

// 速さの換算: 1 単位 = 1 cm、シミュレーションの 1 秒 = 実物の 0.1 秒
inline const float SPEED_MPS_PER_UNIT = 0.01f * TIME_SCALE_REAL;

// 磁石の材質（L キーで切り替え）。大きさと質量は同じまま、モーメントを残留磁束密度 Br に
// 比例させる。力は Br^2 に比例するので、Gamma（接触した 2 球の引力 / 重さ）も Br^2 に比例する
struct MagnetGrade { const char* name; float br; };
inline const MagnetGrade MAGNET_GRADES[] = {
    {"NdFeB (real 5 mm balls)", 1.2f},
    {"ferrite",                 0.4f},
    {"rubber magnet",           0.2f},
};
inline const int MAGNET_GRADE_COUNT = 3;
inline int       magnetGrade        = 0;
inline float magnetMomentScale() { return MAGNET_GRADES[magnetGrade].br / MAG_BR; }
inline float magnetGamma() { const float s = magnetMomentScale(); return MAG_GAMMA * s * s; }

// 場の単位の換算: 接触した隣の球の中心の場は、実物で Br/12、シミュレーションで 2 m0 / d^3
inline const float FIELD_MT_PER_UNIT =
    1000.0f * (MAG_BR / 12.0f) / (2.0f * MAG_M0 / (8.0f * MAG_RADIUS * MAG_RADIUS * MAG_RADIUS));

// 砂鉄（磁鉄鉱の丸い粒）: 等方的な磁化率 0.7 r^3、飽和は磁石の単位体積あたりモーメントの 0.5 倍
inline const float GRAIN_RADIUS = 0.04f;

// 立方体の磁石（辺 0.5 = 球の直径）と鉄。磁石球と同じ材料・密度なので、モーメントと質量は体積に比例
inline const float SPHERE_VOL = 4.0f / 3.0f * PHYS_PI * MAG_RADIUS * MAG_RADIUS * MAG_RADIUS;
inline const float CUBE_HALF  = 0.25f;
inline const float CUBE_VOL   = 8.0f * CUBE_HALF * CUBE_HALF * CUBE_HALF;
inline const float CUBE_M     = MAG_M0 * CUBE_VOL / SPHERE_VOL;
inline const float CUBE_MASS  = MAG_MASS * CUBE_VOL / SPHERE_VOL;
inline const float IRON_CHI   = 1000.0f;   // 軟鉄の磁化率（形の反磁場で実効値は頭打ちになる）
inline const float IRON_SAT   = 1.6f;      // 飽和磁化 / 磁石の磁化（鉄 2.1 T、ネオジム 1.3 T）

// 磁石球（向き dir、強さ MAG_M0 * strength * 材質の倍率。fixed なら静的）
inline RigidBody* makeMagnetBall(World& w, const Vec3& pos, const Vec3& dir, bool fixed = false,
                                 float strength = 1.0f, float radius = MAG_RADIUS) {
    const float scale = (radius / MAG_RADIUS) * (radius / MAG_RADIUS) * (radius / MAG_RADIUS);
    RigidBody* b = w.createMagnet(pos, radius, fixed ? 0.0f : MAG_MASS * scale,
                                  normalize(dir) * (MAG_M0 * scale * strength * magnetMomentScale()));
    b->restitution     = 0.3f;   // ニッケルめっきの金属球
    b->friction        = 0.3f;
    b->rollingFriction = 0.01f;
    return b;
}

// 砂鉄の粒（磁石と同じ密度）
inline RigidBody* makeGrain(World& w, const Vec3& pos) {
    const float r     = GRAIN_RADIUS;
    const float ratio = (r / MAG_RADIUS) * (r / MAG_RADIUS) * (r / MAG_RADIUS);
    RigidBody* b = w.createSoftMagnet(pos, r, MAG_MASS * ratio, 0.7f * r * r * r, 0.5f * MAG_M0 * ratio);
    b->restitution     = 0.0f;
    b->friction        = 0.6f;
    b->rollingFriction = 0.1f;
    b->linearDamping   = 0.5f;
    return b;
}

// 立方体の磁石（ローカル +y が N 極。sign = -1 で向きを逆に）
inline RigidBody* makeMagnetCube(World& w, const Vec3& pos, const Quat& q, float sign = 1.0f) {
    RigidBody* b = w.createMagnetBox(pos, {CUBE_HALF, CUBE_HALF, CUBE_HALF}, CUBE_MASS,
                                     {0, sign * CUBE_M * magnetMomentScale(), 0}, q);
    b->restitution = 0.2f;
    b->friction    = 0.4f;
    return b;
}

// 鉄の球（軸受けの鋼球。硬いので反発係数が大きい）
inline RigidBody* makeIronBall(World& w, const Vec3& pos, float radius = MAG_RADIUS) {
    const float ratio = (radius / MAG_RADIUS) * (radius / MAG_RADIUS) * (radius / MAG_RADIUS);
    RigidBody* b = w.createSoftMagnet(pos, radius, MAG_MASS * ratio, sphereSusceptibility(radius, IRON_CHI),
                                      IRON_SAT * MAG_M0 * ratio);
    b->restitution     = 0.9f;
    b->friction        = 0.3f;
    b->rollingFriction = 0.01f;
    return b;
}

// 鉄の塊（形から誘導係数を決める。磁石と同じ密度）
inline RigidBody* makeIronBox(World& w, const Vec3& pos, const Vec3& he, const Quat& q = Quat{}) {
    const float V = 8.0f * he.x * he.y * he.z;
    RigidBody* b = w.createSoftMagnetBox(pos, he, MAG_MASS * V / SPHERE_VOL,
                                         boxSusceptibility(he, IRON_CHI),
                                         IRON_SAT * MAG_M0 * V / SPHERE_VOL, q);
    b->restitution = 0.1f;
    b->friction    = 0.5f;
    return b;
}

} // namespace demo
