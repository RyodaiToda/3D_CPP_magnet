#pragma once
// ---------------------------------------------------------------------------
// fun_scenes.h : 磁力を見せるシーン（キー 9, 0, -, =, Backspace）
//
//   A. 磁気ローラー   : 回転する外部磁場で磁石が転がる
//   B. 浮かぶ磁石の塔 : 筒の中で反発し合う立方体の磁石が宙に浮く（理論値と比較）
//   C. 方位磁針の群れ : 位置を固定した磁石の格子が、互いの場で模様を作る
//   D. ガウス加速器   : 磁石と鉄球の段に球を当てると、次々に速い球が飛び出す
//   E. 磁力線         : 磁石と鉄を置いて、磁力線と面の上の砂鉄を見る
//
//   どのシーンも既定で実時間と同じ速さ（real time x1）で動く。
//   シーンはデモ本体（main.cpp）から、組み立て・毎ステップの更新・キー入力・
//   追加の描画・HUD の行を呼ばれる。物体そのものの描画はデモ本体が行う。
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "phys/world.h"
#include "hud_lines.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// シーンがデモ本体に頼むこと（世界と、静的な物体の描画色）
struct SceneHost {
    phys::World&        world;
    std::vector<Color>& colors;   // body->id → 色。alpha < 255 は半透明で描く

    void reset() {
        world.clear();
        colors.clear();
        world.gravity  = phys::Vec3{0, -9.81f, 0};
        world.substeps = 1;
        world.magnets().externalField = phys::Vec3{0, 0, 0};
        world.electric().externalField = phys::Vec3{0, 0, 0};
        world.electric().kappa = 1.0f / phys::LIGHT_SPEED;
    }
    void paint(const phys::RigidBody* b, Color c) {
        if ((int)colors.size() <= b->id) colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
    }
    phys::RigidBody* addStatic(const phys::Vec3& pos, const phys::Vec3& he, Color c,
                               float friction = 0.7f, float restitution = 0.1f) {
        phys::RigidBody* b = world.createBox(pos, he, 0.0f);
        b->friction    = friction;
        b->restitution = restitution;
        paint(b, c);
        return b;
    }
    void addGround() { addStatic({0, -1.0f, 0}, {30.0f, 1.0f, 30.0f}, Color{70, 75, 85, 255}); }
};

class FunScene {
public:
    virtual ~FunScene() = default;
    virtual const char* title() const = 0;
    virtual void build(SceneHost& host) = 0;                      // シーンを作り直す
    virtual void beforeStep(phys::World& /*w*/, float /*dt*/) {}  // 各固定ステップの直前
    virtual void handleInput(SceneHost& /*host*/) {}              // 毎フレーム（キー操作）
    virtual void draw3D(const phys::World& /*w*/) const {}        // 追加の 3D 表示
    virtual void hud(const phys::World& /*w*/, HudLines& /*out*/) const {}
    virtual bool ownsField() const { return false; }  // 外部磁場をシーンが動かす（E キーは使わない）
    virtual bool wantsFieldLines() const { return false; }   // 磁力線（O キー）を最初から出す
    virtual float fieldLineFlux() const { return 0.0f; }
    virtual bool ownsSpace() const { return false; }   // Space をシーンが使う（main の「磁石を撃つ」を止める）
    virtual bool compactHud() const { return false; }  // 汎用の HUD（操作の案内・エネルギー）を出さず、シーンの行だけにする     // 磁力線 1 本あたりの磁束（0 なら磁石の既定値）
    // 毎フレーム、描く前に呼ぶ（screenPoint: マウスの位置。FPS 視点では画面の中心）
    virtual void updateView(const phys::World& /*w*/, const Camera3D& /*cam*/, Vector2 /*screenPoint*/) {}
    virtual void draw2D(const phys::World& /*w*/, const Camera3D& /*cam*/) const {}   // 3D の後の 2D 表示

    // カメラの推奨値と、既定の速さ（1 実秒あたりに進めるシミュレーション時間）
    float camTargetX  = 0.0f;
    float camTargetY  = 1.0f;
    float camTargetZ  = 0.0f;
    float camDistance = 12.0f;
    float camPitch    = 0.35f;
    float camYaw      = 0.9f;    // 0 なら +z 側から見る
    float timeScale   = 10.0f;   // 実時間と同じ速さ
    float chargeRef   = 1.0f;    // 電荷の色がいちばん濃くなる電荷（電気のシーン）
    bool  wantCameraReset = false;   // 配置を変えたときなど、推奨カメラに戻してほしい
    bool  wantFieldLinesReset = false;   // 配置を変えたので、磁力線の ON/OFF を wantsFieldLines() に戻してほしい
};

std::unique_ptr<FunScene> makeRollerScene();    // A
std::unique_ptr<FunScene> makeTowerScene();     // B
std::unique_ptr<FunScene> makeCompassScene();   // C
std::unique_ptr<FunScene> makeGaussScene();     // D
std::unique_ptr<FunScene> makeFieldScene();     // E
