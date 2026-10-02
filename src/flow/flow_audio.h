#pragma once
// ---------------------------------------------------------------------------
// flow_audio.h : MAGNET FLOW の効果音（音声ファイルは使わず、起動時にコードで合成する）
// ---------------------------------------------------------------------------
#include "raylib.h"

namespace flow {

class FlowAudio {
public:
    bool init();       // 音声デバイスを開いて音を作る（失敗しても遊べる）
    void shutdown();

    void switched(int polarity);   // 極性切替（N は高め、S は低め）
    void kick();                   // 反発に切り替わった
    void hit();                    // 強く当たった
    void chain();                  // 物が動き出した
    void gate();                   // ゲートが開く
    void goal();
    void fail();
    void capture();                // 別の磁石に吸われた
    void lap(int n);               // 軌道コアを 1 周した（周回が増えるほど高く）
    void boost(float speed);       // サイクロトロンの隙間・加速リングで押された（速いほど高く）
    // 軌道コアを回っている間のうなり（speed で高さを変える）。回っていなければ止める
    void orbit(bool on, float speed);

private:
    bool  ok_ = false;
    Sound switchN_{}, switchS_{}, kick_{}, hit_{}, chain_{}, gate_{}, goal_{}, fail_{}, capture_{}, lap_{}, hum_{}, boost_{};
    void play(const Sound& s) const { if (ok_) PlaySound(s); }
};

} // namespace flow
