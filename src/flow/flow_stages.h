#pragma once
// ---------------------------------------------------------------------------
// flow_stages.h : MAGNET FLOW のステージ（raylib に依存しない）
//   0: S0 練習部屋（Prototype 0。ゴールなし）
//   1: P1 Switch   極性切替を理解する
//   2: P2 Timing   切替のタイミングで軌道が変わる
//   3: P3 Chain    自分の移動で磁石と鉄球が連鎖して、ゲートが開く
//   ここから magnetic_flow_gimmick_stage_spec.md のプロトタイプ 3 ステージ
//   4: 01 磁気峡谷   吸着・反発・極性切替・上下の移動
//   5: 02 軌道庭園   磁気軌道（軌道コアを周回して速さを稼ぎ、放す）
//   6: 04 磁気工場   磁気ドミノ（可動磁石 → 鉄球 → 可動磁石 → 回る扉）
//   7: GRAND TOUR    全部のギミックを入れた大きなマップ（9 区間、チェックポイントつき。flow_grand.cpp）
//   ここから新しいギミック（§17。flow_lab.cpp）
//   8: L1 Cyclotron        サイクロトロン
//   9: L2 Gauss Rings      ガウス加速リング
//  10: L3 Resonance Swing  共振ブランコ
//  11: L4 Ball Chain       鉄球のロープ
// ---------------------------------------------------------------------------
#include "flow/flow_world.h"

namespace flow {

constexpr int STAGE_COUNT = 12;
constexpr int GRAND_STAGE = 7;
constexpr int LAB_STAGE   = 8;   // L1〜L4 = 8〜11

const char* stageName(int index);
const char* stageKey(int index);       // 記録を保存するときの名前（並びを変えても変わらない）
const char* stageHint(int index);      // 画面に出す一言
void        loadStage(FlowWorld& fw, int index);   // builder を設定して組み立てる

// 大きなマップ（flow_grand.cpp）
void buildGrand(FlowWorld& fw);
int  grandZoneCount();
Vec3 grandToLocal(int zone, const Vec3& world);   // 区間の座標（区間の始めの発射台の上面の中心が原点、+x が進む向き）
Vec3 grandToWorld(int zone, const Vec3& local);
extern int grandOnlyZone;   // 調整用: >= 0 ならその区間と前後の区間だけを組む

// 新しいギミックのステージ（flow_lab.cpp）
void buildCyclotron(FlowWorld& fw);
void buildGaussRings(FlowWorld& fw);
void buildResonance(FlowWorld& fw);
void buildBallChain(FlowWorld& fw);

// ステージの磁石の既定の強さ
constexpr float MAGNET_Q      = 1.0f;    // プレイヤーへの力の強さ q
constexpr float WORLD_GAMMA   = 60.0f;   // 物どうしの双極子の強さ（接触した 2 個の引力 / 重さ）

} // namespace flow
