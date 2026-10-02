#pragma once
// ---------------------------------------------------------------------------
// field_lines.h : 磁力線（B の流線）をたどる（raylib に依存しない）
//
//   磁石の表面のうち B が外向きのところ（N 極側）から、B の向きに沿ってたどる。
//   線の本数は表面を出る磁束に比例させる（1 本 = fluxPerLine）。そのため線の混み具合が
//   |B| の大きさを表す。
//     - 磁石（永久モーメントあり）に入ったら止める（線は磁石の中を通って閉じる）
//     - 鉄（誘導磁化だけ）に入ったら、中を磁化の向きに抜けて続ける（磁束が鉄に吸い込まれて出てくる）
//     - 一様な外部磁場があれば、上流側の面からも同じ密度で線を出す
//     - 電流の導線（CurrentSystem）: 閉じた輪は、輪の面を通る磁束を等分した点から、開いた導線は
//       中点から垂直に 2 倍ずつ離れた点から、両向きにたどって 1 本につなぐ（線は導線のまわりで閉じる）
//   場は、たどり始めるときに写した源（各点の位置とモーメント、導線の線分）から計算する（World は変えない）。
//   毎フレーム決まった時間だけたどる。全部たどり終えたら、物が動いたとき（電流が変わったとき）だけやり直す。
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace demo {

using namespace phys;

// 磁性体の点の双極子・外部磁場・導線の線分を写したもの（場を速く何度も評価するため）
struct FieldSnapshot {
    struct Source { Vec3 x, m; };
    struct Seg { Vec3 a, b; float I, radius; };
    struct Loop { Vec3 c, n; float a, I, radius; };   // 円いコイル（厳密な式で速く評価する）
    std::vector<Source> sources;
    std::vector<Seg>    segs;
    std::vector<Loop>   loops;
    Vec3 B0{0, 0, 0};

    void capture(const MagnetSystem& ms, const CurrentSystem* cs = nullptr) {
        sources.clear();
        segs.clear();
        loops.clear();
        B0 = ms.externalField;
        for (const MagneticBody& mb : ms.bodies()) {
            const Mat3 R = mb.body->orientation.toMat3();
            for (int k = 0; k < mb.pointCount; ++k)
                if (lengthSq(mb.pointM[k]) > 1e-12f)
                    sources.push_back({mb.body->position + R * mb.pointLocal[k], mb.pointM[k]});
        }
        if (cs)
            for (const Wire& w : cs->wires()) {
                const float I = w.effectiveCurrent();
                if (I == 0.0f || w.points.size() < 2) continue;
                if (w.circle) {
                    loops.push_back({w.circleCenter, w.circleAxis, w.circleRadius, I, w.radius});
                    continue;
                }
                for (int s = 0; s < w.segmentCount(); ++s) {
                    Seg sg;
                    w.segment(s, sg.a, sg.b);
                    sg.I = I;
                    sg.radius = w.radius;
                    segs.push_back(sg);
                }
            }
    }

    static float segmentDistanceSq(const Seg& s, const Vec3& p) {
        const Vec3  ab = s.b - s.a;
        const float L2 = lengthSq(ab);
        const float t  = L2 > 1e-16f ? clampf(dot(p - s.a, ab) / L2, 0.0f, 1.0f) : 0.0f;
        return lengthSq(p - (s.a + ab * t));
    }

    // 点 p の B。nearest には最も近い源までの距離を入れる
    Vec3 at(const Vec3& p, float* nearest = nullptr) const {
        Vec3  B  = B0;
        float d2 = 1e18f;
        for (const Source& s : sources) {
            const Vec3  r  = p - s.x;
            const float r2 = lengthSq(r);
            d2 = std::min(d2, r2);
            if (r2 < 1e-10f) continue;
            const float invR = 1.0f / std::sqrt(r2);
            const Vec3  n    = r * invR;
            B += (n * (3.0f * dot(s.m, n)) - s.m) * (invR * invR * invR);
        }
        for (const Seg& s : segs) {
            d2 = std::min(d2, segmentDistanceSq(s, p));
            B += segmentField(s.a, s.b, p, s.I, s.radius);
        }
        for (const Loop& l : loops) {
            float dist;
            B += loopField(l.c, l.n, l.a, p, l.I, l.radius, &dist);
            d2 = std::min(d2, dist * dist);
        }
        if (nearest) *nearest = std::sqrt(d2);
        return B;
    }
};

struct FieldLine {
    std::vector<Vec3>  pts;
    std::vector<float> mag;   // 各点の |B|
};

class FieldLineTracer {
public:
    float  fluxPerLine = 8.0f;    // 線 1 本あたりの磁束
    int    maxLines    = 300;     // 本数の上限（超えるときは全体を同じ割合で減らす）
    float  maxLength   = 40.0f;   // 1 本の長さの上限
    int    maxSteps    = 500;

    const std::vector<FieldLine>& lines() const { return lines_; }
    bool busy() const { return active_; }
    void reset() { lines_.clear(); active_ = false; state_.clear(); }

    // budgetMs だけたどる（物が動いていなければ何もしない）
    void update(const World& w, double budgetMs) {
        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();
        if (!active_) {
            if (!changed(w.magnets(), w.currents())) return;
            start(w.magnets(), w.currents());
        }
        while (next_ < seeds_.size()) {
            traceSeed(next_);
            ++next_;
            if (std::chrono::duration<double, std::milli>(clock::now() - t0).count() > budgetMs) break;
        }
        if (next_ >= seeds_.size()) {
            lines_.resize(seeds_.size());
            active_ = false;
        }
    }

    // すべてたどり終えるまで続ける（テスト用）
    void traceAll(const World& w) {
        reset();
        start(w.magnets(), w.currents());
        while (next_ < seeds_.size()) { traceSeed(next_); ++next_; }
        lines_.resize(seeds_.size());
        active_ = false;
    }

private:
    struct Body {
        Vec3  c;
        Mat3  Rt;             // ワールド → ローカル
        bool  box = false;
        float r = 0.0f;       // 球の半径（箱は外接球の半径）
        Vec3  he{0, 0, 0};
        bool  permanent = false;
        Vec3  mdir{0, 0, 0};  // 磁化の向き（鉄を抜ける向き）
    };

    FieldSnapshot      snap_;
    std::vector<Body>  bodies_;
    std::vector<Vec3>  seeds_;
    std::vector<char>  both_;       // 両向きにたどる始点（導線の線）
    std::vector<int>   stop_;       // この物体に入ったら止める（-1: 磁石ならどれでも）
    std::vector<FieldLine> lines_;
    std::vector<float> state_;      // 前回たどり始めたときの位置・姿勢・モーメント・電流
    size_t next_   = 0;
    bool   active_ = false;
    Vec3   lo_{0, 0, 0}, hi_{0, 0, 0};   // この箱の外に出たら止める

    static void pushState(std::vector<float>& s, const MagnetSystem& ms, const CurrentSystem& cs) {
        s.clear();
        s.push_back(ms.externalField.x); s.push_back(ms.externalField.y); s.push_back(ms.externalField.z);
        for (const MagneticBody& mb : ms.bodies()) {
            const Vec3& p = mb.body->position;
            const Quat& q = mb.body->orientation;
            for (float v : {p.x, p.y, p.z, q.x, q.y, q.z, q.w, mb.m.x, mb.m.y, mb.m.z}) s.push_back(v);
        }
        for (const Wire& w : cs.wires()) {
            s.push_back(w.effectiveCurrent());
            for (const Vec3& p : w.points) { s.push_back(p.x); s.push_back(p.y); s.push_back(p.z); }
        }
    }

    bool changed(const MagnetSystem& ms, const CurrentSystem& cs) const {
        std::vector<float> now;
        pushState(now, ms, cs);
        if (now.size() != state_.size()) return true;
        for (size_t i = 0; i < now.size(); ++i) {
            const float tol = 1e-3f * std::max(1.0f, std::fabs(state_[i]));
            if (std::fabs(now[i] - state_[i]) > tol) return true;
        }
        return false;
    }

    int inside(const Vec3& p, int skip = -1) const {
        for (int i = 0; i < (int)bodies_.size(); ++i) {
            if (i == skip) continue;
            const Body& b = bodies_[i];
            const Vec3  d = p - b.c;
            if (lengthSq(d) > b.r * b.r) continue;
            if (!b.box) return i;
            const Vec3 o = b.Rt * d;
            if (std::fabs(o.x) <= b.he.x && std::fabs(o.y) <= b.he.y && std::fabs(o.z) <= b.he.z) return i;
        }
        return -1;
    }

    // 物体 b の中の点 p から向き d に進んで、表面を出る点
    static Vec3 exitPoint(const Body& b, const Vec3& p, const Vec3& d) {
        if (!b.box) {
            const Vec3  q  = p - b.c;
            const float bd = dot(q, d);
            const float t  = -bd + std::sqrt(std::max(0.0f, bd * bd - (lengthSq(q) - b.r * b.r)));
            return p + d * t;
        }
        const Vec3 o = b.Rt * (p - b.c), dl = b.Rt * d;
        float t = 1e9f;
        for (int k = 0; k < 3; ++k)
            if (std::fabs(dl[k]) > 1e-6f) t = std::min(t, ((dl[k] > 0 ? b.he[k] : -b.he[k]) - o[k]) / dl[k]);
        return p + d * t;
    }

    void extendBounds(const Vec3& p, bool& first) {
        if (first) { lo_ = hi_ = p; first = false; return; }
        lo_ = Vec3{std::min(lo_.x, p.x), std::min(lo_.y, p.y), std::min(lo_.z, p.z)};
        hi_ = Vec3{std::max(hi_.x, p.x), std::max(hi_.y, p.y), std::max(hi_.z, p.z)};
    }

    bool outside(const Vec3& p) const {
        return p.x < lo_.x || p.y < lo_.y || p.z < lo_.z || p.x > hi_.x || p.y > hi_.y || p.z > hi_.z;
    }

    void start(const MagnetSystem& ms, const CurrentSystem& cs) {
        pushState(state_, ms, cs);
        snap_.capture(ms, &cs);
        bodies_.clear();
        seeds_.clear();
        both_.clear();
        stop_.clear();
        next_   = 0;
        active_ = true;

        // 磁性体と導線（遠く離れた物は範囲の計算から外す）
        bool first = true;
        int  permanentCount = 0;
        bool hasCurrent = false;
        for (const Wire& w : cs.wires()) hasCurrent = hasCurrent || w.effectiveCurrent() != 0.0f;
        std::vector<char> seedFrom;   // この物体の表面から線を出す（磁石。電流があるときは、磁化した鉄の箱 = 電磁石の芯も）
        for (const MagneticBody& mb : ms.bodies()) {
            const RigidBody* rb = mb.body;
            Body b;
            b.c   = rb->position;
            b.Rt  = rb->orientation.toMat3().transposed();
            b.box = rb->shape.type == ShapeType::Box;
            b.he  = rb->shape.halfExtents;
            b.r   = b.box ? length(b.he) : rb->shape.radius;
            b.permanent = lengthSq(mb.permanentLocal) > 0.0f;
            b.mdir = lengthSq(mb.m) > 1e-12f ? normalize(mb.m) : Vec3{0, 0, 0};
            bodies_.push_back(b);
            const bool core = hasCurrent && b.box && !b.permanent && mb.saturation > 0.0f && length(mb.m) > 0.05f * mb.saturation;
            seedFrom.push_back(b.permanent || core);
            permanentCount += seedFrom.back();
            if (length(b.c) > 300.0f) continue;
            extendBounds(b.c, first);
        }
        for (const Wire& w : cs.wires()) {
            if (w.effectiveCurrent() == 0.0f) continue;
            for (const Vec3& p : w.points) if (length(p) <= 300.0f) extendBounds(p, first);
        }
        const float margin = 3.0f + 0.25f * length(hi_ - lo_);
        lo_ = lo_ - Vec3{margin, margin, margin};
        hi_ = hi_ + Vec3{margin, margin, margin};

        // 磁石の表面の点ごと（導線は輪の面の点ごと）に、外向きの磁束を求める
        struct Sample { Vec3 p; float flux; };
        std::vector<std::vector<Sample>> perBody;
        std::vector<float> totals;
        std::vector<char>  bothDirs;
        std::vector<int>   stopAt;   // この物体に入ったら止める（-1: 磁石ならどれでも）
        const int samples = std::clamp(permanentCount > 0 ? 6000 / permanentCount : 240, 48, 240);
        for (int i = 0; i < (int)bodies_.size(); ++i) {
            const Body& b = bodies_[i];
            if (!seedFrom[i] || length(b.c) > 300.0f) continue;
            std::vector<Sample> ss;
            auto add = [&](const Vec3& p, const Vec3& n, float area) {
                if (inside(p, i) >= 0) return;   // 接した隣の物体の中
                const float f = dot(snap_.at(p), n) * area;
                if (f > 0.0f) ss.push_back({p, f});
            };
            if (!b.box) {
                // 球: フィボナッチ格子
                const float a = b.r * 1.01f, area = 4.0f * PHYS_PI * a * a / samples;
                for (int k = 0; k < samples; ++k) {
                    const float z = 1.0f - 2.0f * (k + 0.5f) / samples, s = std::sqrt(1.0f - z * z);
                    const float ph = 2.39996323f * k;
                    const Vec3  n{s * std::cos(ph), s * std::sin(ph), z};
                    add(b.c + n * a, n, area);
                }
            } else {
                // 箱: 各面の格子
                const Mat3 R = b.Rt.transposed();
                const int  g = std::max(3, (int)std::sqrt(samples / 6.0f));
                for (int axis = 0; axis < 3; ++axis)
                    for (float sgn : {-1.0f, 1.0f}) {
                        const int   u = (axis + 1) % 3, v = (axis + 2) % 3;
                        const float area = 4.0f * b.he[u] * b.he[v] / (g * g);
                        for (int i1 = 0; i1 < g; ++i1)
                            for (int i2 = 0; i2 < g; ++i2) {
                                Vec3 o{0, 0, 0}, nl{0, 0, 0};
                                o[axis]  = sgn * (b.he[axis] + 0.01f);
                                o[u]     = ((i1 + 0.5f) / g * 2.0f - 1.0f) * b.he[u];
                                o[v]     = ((i2 + 0.5f) / g * 2.0f - 1.0f) * b.he[v];
                                nl[axis] = sgn;
                                add(b.c + R * o, R * nl, area);
                            }
                    }
            }
            float total = 0.0f;
            for (const Sample& s : ss) total += s.flux;
            perBody.push_back(std::move(ss));
            totals.push_back(total);
            bothDirs.push_back(0);
            stopAt.push_back(b.permanent ? -1 : i);   // 鉄の芯から出た線は、その芯に戻ったら止める
        }

        // 閉じた輪: 輪の面（中心からの扇形）の格子で、面を通る磁束を求める
        std::vector<Vec3> openSeeds;
        for (const Wire& w : cs.wires()) {
            if (w.effectiveCurrent() == 0.0f || w.points.size() < 2) continue;
            const int np = (int)w.points.size();
            Vec3 c{0, 0, 0};
            for (const Vec3& p : w.points) c += p;
            c = c * (1.0f / np);
            if (length(c) > 300.0f) continue;
            if (!w.closed || np < 3) {
                // 開いた導線: 中点から垂直に、2 倍ずつ離れた点
                const int  s0 = (np - 1) / 2;
                const Vec3 a = w.points[s0], b = w.points[s0 + 1];
                const Vec3 t = normalize(b - a), mid = (a + b) * 0.5f;
                Vec3 u, v;
                buildTangents(t, u, v);
                for (float r = std::max(2.0f * w.radius, 0.05f); r < 100.0f; r *= 2.0f) {
                    const Vec3 p = mid + u * r;
                    if (outside(p)) break;
                    if (inside(p) >= 0) continue;
                    openSeeds.push_back(p);
                }
                continue;
            }
            Vec3 n{0, 0, 0};
            for (int i = 0; i < np; ++i) n += cross(w.points[i] - c, w.points[(i + 1) % np] - c);
            if (lengthSq(n) < 1e-12f) continue;
            n = normalize(n);
            if (dot(snap_.at(c), n) < 0.0f) n = n * -1.0f;   // 線は B の向きに出す
            Vec3 u, v;
            buildTangents(n, u, v);
            float R = 0.0f;
            for (const Vec3& p : w.points) R = std::max(R, length(p - c));
            const int   g = 28;
            const float h = 2.0f * R / g, area = h * h;
            std::vector<Sample> ss;
            for (int i = 0; i < g; ++i)
                for (int j = 0; j < g; ++j) {
                    const Vec3 p = c + u * ((i + 0.5f) * h - R) + v * ((j + 0.5f) * h - R);
                    // 扇形の三角形のどれかの中にあるか
                    bool in = false;
                    for (int k = 0; k < np && !in; ++k) {
                        const Vec3 e0 = w.points[k] - c, e1 = w.points[(k + 1) % np] - c, d = p - c;
                        const float d00 = dot(e0, e0), d01 = dot(e0, e1), d11 = dot(e1, e1), d20 = dot(d, e0), d21 = dot(d, e1);
                        const float den = d00 * d11 - d01 * d01;
                        if (std::fabs(den) < 1e-12f) continue;
                        const float s = (d11 * d20 - d01 * d21) / den, t = (d00 * d21 - d01 * d20) / den;
                        in = s >= 0.0f && t >= 0.0f && s + t <= 1.0f;
                    }
                    if (!in || inside(p) >= 0) continue;
                    const float f = dot(snap_.at(p), n) * area;
                    if (f > 0.0f) ss.push_back({p, f});
                }
            float total = 0.0f;
            for (const Sample& s : ss) total += s.flux;
            perBody.push_back(std::move(ss));
            totals.push_back(total);
            bothDirs.push_back(1);
            stopAt.push_back(-1);
        }

        // 一様な外部磁場: 範囲の箱の上流側の端を通る、場に垂直な面の格子（間隔は同じ磁束密度になるように）
        std::vector<Vec3> fieldSeeds;
        const float b0 = length(snap_.B0);
        if (b0 > 1e-6f) {
            const Vec3  dir  = snap_.B0 / b0;
            const Vec3  c    = (lo_ + hi_) * 0.5f, half = (hi_ - lo_) * 0.5f;
            const float rad  = length(half);
            float back = 1e9f;   // 中心から上流側へ、箱の面までの距離
            for (int k = 0; k < 3; ++k)
                if (std::fabs(dir[k]) > 1e-6f) back = std::min(back, half[k] / std::fabs(dir[k]));
            const Vec3 o = c - dir * (0.98f * back);
            const Vec3 u = normalize(std::fabs(dir.y) < 0.9f ? cross(dir, Vec3{0, 1, 0}) : cross(dir, Vec3{1, 0, 0}));
            const Vec3 v = cross(dir, u);
            float spacing = std::sqrt(fluxPerLine / b0);
            spacing = std::max(spacing, rad / 8.0f);   // 多すぎないように
            const int n = (int)(rad / spacing);
            for (int i = -n; i <= n; ++i)
                for (int j = -n; j <= n; ++j) {
                    const Vec3 p = o + u * (i * spacing) + v * (j * spacing);
                    if (outside(p)) continue;
                    if (inside(p) >= 0) continue;
                    fieldSeeds.push_back(p);
                }
        }

        // 本数を決める（多すぎれば全体を同じ割合で減らす）
        float want = (float)fieldSeeds.size() + (float)openSeeds.size();
        for (float t : totals) want += t / fluxPerLine;
        const float scale = want > maxLines ? maxLines / want : 1.0f;
        for (size_t i = 0; i < perBody.size(); ++i) {
            const int k = (int)std::lround(totals[i] / fluxPerLine * scale);
            if (k <= 0) continue;
            // 磁束の累積を k 等分した位置の点を選ぶ（線の密度 ∝ 磁束密度）
            float acc = 0.0f;
            int   j   = 0;
            size_t last = (size_t)-1;
            for (size_t s = 0; s < perBody[i].size() && j < k; ++s) {
                acc += perBody[i][s].flux;
                while (j < k && acc >= (j + 0.5f) * totals[i] / k) {
                    if (s != last) { seeds_.push_back(perBody[i][s].p); both_.push_back(bothDirs[i]); stop_.push_back(stopAt[i]); }
                    last = s;
                    ++j;
                }
            }
        }
        const size_t stride = scale < 1.0f ? (size_t)std::ceil(1.0f / scale) : 1;
        for (size_t i = 0; i < fieldSeeds.size(); i += stride) { seeds_.push_back(fieldSeeds[i]); both_.push_back(0); stop_.push_back(-1); }
        for (size_t i = 0; i < openSeeds.size(); i += stride)  { seeds_.push_back(openSeeds[i]);  both_.push_back(1); stop_.push_back(-1); }

        if (lines_.size() < seeds_.size()) lines_.resize(seeds_.size());
    }

    Vec3 direction(const Vec3& p, float& nearest) const {
        const Vec3 B = snap_.at(p, &nearest);
        const float l = length(B);
        return l > 1e-12f ? B / l : Vec3{0, 0, 0};
    }

    void traceSeed(size_t i) {
        FieldLine& out = lines_[i];
        const bool closed = trace(seeds_[i], out, 1.0f, both_[i] != 0, stop_[i]);
        if (!both_[i] || closed) return;
        FieldLine back;
        trace(seeds_[i], back, -1.0f, false, stop_[i]);
        if (back.pts.size() <= 1) return;
        FieldLine joined;
        for (size_t k = back.pts.size(); k-- > 1;) { joined.pts.push_back(back.pts[k]); joined.mag.push_back(back.mag[k]); }
        joined.pts.insert(joined.pts.end(), out.pts.begin(), out.pts.end());
        joined.mag.insert(joined.mag.end(), out.mag.begin(), out.mag.end());
        out = std::move(joined);
    }

    // sign = -1 なら B と逆向きにたどる。closeOnReturn なら、始点に戻ったら閉じて true を返す。
    // stopBody >= 0 なら、その物体（鉄の芯）に入っても止める
    bool trace(Vec3 p, FieldLine& out, float sign, bool closeOnReturn, int stopBody) const {
        out.pts.clear();
        out.mag.clear();
        const Vec3 seed = p;
        float d  = 0.0f;
        Vec3  B  = snap_.at(p, &d);
        out.pts.push_back(p);
        out.mag.push_back(length(B));
        float travelled = 0.0f;
        int   passes = 0;
        for (int step = 0; step < maxSteps; ++step) {
            const float h = std::clamp(0.15f * d, 0.01f, 0.4f);
            float dd;
            const Vec3 k1 = direction(p, dd) * sign;
            const Vec3 k2 = direction(p + k1 * (0.5f * h), dd) * sign;
            const Vec3 k3 = direction(p + k2 * (0.5f * h), dd) * sign;
            const Vec3 k4 = direction(p + k3 * h, dd) * sign;
            const Vec3 step3 = (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (h / 6.0f);
            if (lengthSq(step3) < 1e-12f) break;   // 場が 0（中立点）
            Vec3 np = p + step3;
            travelled += h;

            const int hit = inside(np);
            if (hit >= 0) {
                const Body& b = bodies_[hit];
                out.pts.push_back(np);
                out.mag.push_back(out.mag.back());
                if (b.permanent || hit == stopBody || ++passes > 64) break;   // 磁石（か出てきた芯）に入った（閉じた）
                // 鉄: 中を磁化の向きに抜ける
                Vec3 dir = lengthSq(b.mdir) > 0.0f ? b.mdir : normalize(step3);
                if (dot(dir, step3) < 0.0f) dir = dir * -1.0f;
                np = exitPoint(b, np, dir) + dir * 0.01f;
                if (inside(np) >= 0) break;
            }
            p = np;
            B = snap_.at(p, &d);
            out.pts.push_back(p);
            out.mag.push_back(length(B));
            if (closeOnReturn && travelled > 6.0f * h && length(p - seed) < 1.5f * h) {   // 始点に戻った
                out.pts.push_back(seed);
                out.mag.push_back(out.mag.front());
                return true;
            }
            if (travelled > maxLength) break;
            if (outside(p)) break;
        }
        return false;
    }
};

} // namespace demo
