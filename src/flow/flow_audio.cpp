// ---------------------------------------------------------------------------
// flow_audio.cpp : 効果音の合成（flow_audio.h）
//   44.1 kHz、16 bit、モノラルの波形を直接書いて Sound にする
// ---------------------------------------------------------------------------
#include "flow/flow_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <vector>

namespace flow {

namespace {

constexpr int   RATE = 44100;
constexpr float TAU  = 6.28318531f;

// f(t) を [-1, 1] で返す関数から音を作る
Sound synth(float seconds, const std::function<float(float)>& f, float volume = 0.5f) {
    const int n = (int)(seconds * RATE);
    std::vector<int16_t> pcm(n);
    for (int i = 0; i < n; ++i) {
        float v = f((float)i / RATE) * volume;
        v = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
        pcm[i] = (int16_t)(v * 32767.0f);
    }
    Wave w{};
    w.frameCount = (unsigned)n;
    w.sampleRate = RATE;
    w.sampleSize = 16;
    w.channels   = 1;
    w.data       = pcm.data();
    return LoadSoundFromWave(w);   // データはコピーされる
}

// 周波数が f0 → f1 に変わる正弦波の位相（指数的に変える）
float sweepPhase(float t, float f0, float f1, float T) {
    const float k = std::log(f1 / f0) / T;
    return std::fabs(k) < 1e-6f ? TAU * f0 * t : TAU * f0 * (std::exp(k * t) - 1.0f) / k;
}

float noise() { return (float)std::rand() / RAND_MAX * 2.0f - 1.0f; }

} // namespace

bool FlowAudio::init() {
    InitAudioDevice();
    if (!IsAudioDeviceReady()) return false;
    std::srand(12345);
    switchN_ = synth(0.09f, [](float t) { return std::sin(sweepPhase(t, 880, 1320, 0.09f)) * std::exp(-t * 30); });
    switchS_ = synth(0.09f, [](float t) { return std::sin(sweepPhase(t, 520, 350, 0.09f)) * std::exp(-t * 30); });
    kick_ = synth(0.30f, [](float t) {
        static float lp = 0.0f;
        lp += 0.25f * (noise() - lp);   // 軽く低域だけ残したノイズ
        return (0.7f * lp + 0.6f * std::sin(sweepPhase(t, 320, 90, 0.3f))) * std::exp(-t * 11);
    }, 0.55f);
    hit_ = synth(0.18f, [](float t) {
        return (std::sin(TAU * 85 * t) + 0.3f * noise() * std::exp(-t * 80)) * std::exp(-t * 22);
    }, 0.6f);
    chain_ = synth(0.18f, [](float t) {
        return (std::sin(TAU * 1250 * t) + 0.4f * std::sin(TAU * 2500 * t)) * std::exp(-t * 20);
    }, 0.3f);
    gate_ = synth(0.8f, [](float t) {
        return (std::sin(TAU * 523.3f * t) + std::sin(TAU * 659.3f * t) + std::sin(TAU * 784.0f * t)) / 3.0f *
               std::exp(-t * 3.5f) * std::fmin(1.0f, t * 60);
    }, 0.5f);
    goal_ = synth(0.9f, [](float t) {
        static const float notes[4] = {523.3f, 659.3f, 784.0f, 1046.5f};
        const int   k  = std::min(3, (int)(t / 0.12f));
        const float tk = t - k * 0.12f;
        return std::sin(TAU * notes[k] * t) * std::exp(-tk * (k == 3 ? 3.0f : 14.0f)) * std::fmin(1.0f, tk * 200);
    }, 0.45f);
    fail_ = synth(0.45f, [](float t) { return std::sin(sweepPhase(t, 440, 110, 0.45f)) * std::exp(-t * 4); }, 0.4f);
    capture_ = synth(0.16f, [](float t) {
        return std::sin(sweepPhase(t, 180, 420, 0.16f)) * std::exp(-t * 16) * std::fmin(1.0f, t * 300);
    }, 0.45f);
    lap_ = synth(0.12f, [](float t) {
        return (std::sin(TAU * 988 * t) + 0.5f * std::sin(TAU * 1976 * t)) * std::exp(-t * 28);
    }, 0.3f);
    // 加速: 短く上がる音（2 つの倍音）
    boost_ = synth(0.14f, [](float t) {
        return (std::sin(sweepPhase(t, 660, 990, 0.14f)) + 0.4f * std::sin(sweepPhase(t, 1320, 1980, 0.14f))) *
               std::exp(-t * 18) * std::fmin(1.0f, t * 400);
    }, 0.35f);
    // うなり: 0.5 s でちょうど周期がそろう周波数にして、つなげて鳴らしても継ぎ目が出ないようにする
    hum_ = synth(0.5f, [](float t) {
        return 0.6f * std::sin(TAU * 110 * t) + 0.3f * std::sin(TAU * 220 * t) + 0.15f * std::sin(TAU * 330 * t);
    }, 0.25f);
    ok_ = true;
    return true;
}

void FlowAudio::shutdown() {
    if (ok_) {
        for (Sound* s : {&switchN_, &switchS_, &kick_, &hit_, &chain_, &gate_, &goal_, &fail_, &capture_, &lap_, &hum_, &boost_})
            UnloadSound(*s);
        ok_ = false;
    }
    if (IsAudioDeviceReady()) CloseAudioDevice();
}

void FlowAudio::switched(int polarity) { play(polarity > 0 ? switchN_ : switchS_); }
void FlowAudio::kick()  { play(kick_); }
void FlowAudio::hit()   { play(hit_); }
void FlowAudio::chain() { play(chain_); }
void FlowAudio::gate()  { play(gate_); }
void FlowAudio::goal()  { play(goal_); }
void FlowAudio::fail()  { play(fail_); }
void FlowAudio::capture() { play(capture_); }

void FlowAudio::lap(int n) {
    if (!ok_) return;
    SetSoundPitch(lap_, 1.0f + 0.12f * (float)std::min(n, 8));
    PlaySound(lap_);
}

void FlowAudio::boost(float speed) {
    if (!ok_) return;
    SetSoundPitch(boost_, 0.8f + speed / 25.0f);
    PlaySound(boost_);
}

void FlowAudio::orbit(bool on, float speed) {
    if (!ok_) return;
    if (!on) {
        if (IsSoundPlaying(hum_)) StopSound(hum_);
        return;
    }
    SetSoundPitch(hum_, 0.6f + speed / 18.0f);
    SetSoundVolume(hum_, std::min(1.0f, 0.3f + speed / 20.0f));
    if (!IsSoundPlaying(hum_)) PlaySound(hum_);
}

} // namespace flow
