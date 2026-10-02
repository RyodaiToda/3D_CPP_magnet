#pragma once
// ---------------------------------------------------------------------------
// electric_scenes.h : 電気を見せるシーン（キー F5〜F8。ELECTRIC_SPEC.md）
//
//   F5 電気ベルと帯電球   : 平行板の間で導体の小球が跳ね回る（触れるたびに電荷が入れ替わる）／
//                          帯電した金属球が、帯電していない球を引き寄せ、触れると分け合って弾き合う
//   F6 磁場の中の荷電粒子 : 一様な磁場の中のらせん／2 つの磁石の間を行き来する磁気瓶／E x B ドリフト
//   F7 渦電流             : 銅の管とプラスチックの管の競争／アラゴの円板／磁石の上で遅くなる銅の板
//   F8 電場の見える化     : 電気力線と等電位線（点電荷、同符号、平行板と導体、誘導）
//
//   どのシーンも M で並べ方を切り替える。長さは m、時間は実時間（磁石のデモの lambda の換算はしない）。
// ---------------------------------------------------------------------------
#include "fun_scenes.h"

std::unique_ptr<FunScene> makeBellScene();      // F5
std::unique_ptr<FunScene> makeLorentzScene();   // F6
std::unique_ptr<FunScene> makeEddyScene();      // F7
std::unique_ptr<FunScene> makeEFieldScene();    // F8

// 電荷の色: + は橙、- は紫、0 は導体なら銅色（強さ ref で明るさが最大になる）
Color chargeColor(float q, float ref, bool conductor);
