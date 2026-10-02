// MAGNET FLOW のヘッドレス検証（raylib 不要）
//   flow_test                 すべてのテスト（FL1〜FL22。約 5 分）
//   flow_test --sweep <stage> 切り替える時刻を振った結果を表示する（ステージの調整用）
//   flow_test --probe / --trace / --record   01・02・04 の調整と、GIF 用の記録
//   GRAND TOUR（大きなマップ）:
//     flow_test --grandrun [記録のファイル|-] [区間]   ボットが通しで走る（区間を渡すとその区間の軌跡を表示）
//     flow_test --gprobe <区間> <from> <to> <step> <window> [前の切替...]   通しの走りの中で最後の切替を振る
//     flow_test [--wait] [--full] --zprobe <区間-1> ... / --ztrace <区間-1> [切替...]   区間と前後だけを組んで調べる
//   新しいギミックのステージ L1〜L4（状態を見て押すボット）:
//     flow_test --lab <1..4> [delta] [onlyK] [skipK] [phase] [着地の高さ]   走らせて軌跡と出来事を表示する
//     flow_test --labwin <1..4> [from] [to] [step]   切替ごとに時刻をずらして、クリアできる範囲を測る
//     flow_test --labrecord <file>                   4 つのボットの走りを記録のファイルに足す（GIF 用）
#include "flow/flow_records.h"
#include "flow/flow_stages.h"
#include "flow/flow_world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace phys;
using namespace flow;

static int failures = 0;
static void check(const char* name, bool ok, const char* detail = "") {
    std::printf("[%s] %-44s %s\n", ok ? " OK " : "FAIL", name, detail);
    if (!ok) ++failures;
}

constexpr float DT = 1.0f / 120.0f;

// ---------------------------------------------------------------------------
// ボット: 開始してから switchTimes の時刻に極性を切り替える
// ---------------------------------------------------------------------------
struct RunResult { bool cleared = false, failed = false; float time = 0.0f; bool gateOpened = false; };

static RunResult runSchedule(int stage, const std::vector<float>& switchTimes, float maxT = 15.0f) {
    FlowWorld fw;
    loadStage(fw, stage);
    fw.press();   // 開始
    RunResult r;
    float  t = 0.0f;
    size_t k = 0;
    while (t < maxT) {
        while (k < switchTimes.size() && t >= switchTimes[k] - 1e-6f) { fw.press(); ++k; }
        fw.step(DT);
        t += DT;
        for (const Gate& g : fw.gates) r.gateOpened |= g.opening;
        if (fw.state == State::Cleared) { r.cleared = true; break; }
        if (fw.state == State::Failed)  { r.failed = true;  break; }
    }
    r.time = t;
    return r;
}

// runSchedule と同じだが、最後の世界を返す（記録や統計を見る）
static RunResult runWorld(FlowWorld& fw, int stage, const std::vector<float>& switchTimes, float maxT = 15.0f) {
    loadStage(fw, stage);
    fw.press();
    RunResult r;
    float  t = 0.0f;
    size_t k = 0;
    while (t < maxT) {
        while (k < switchTimes.size() && t >= switchTimes[k] - 1e-6f) { fw.press(); ++k; }
        fw.step(DT);
        t += DT;
        for (const Gate& g : fw.gates) r.gateOpened |= g.opening;
        if (fw.state == State::Cleared) { r.cleared = true; break; }
        if (fw.state == State::Failed)  { r.failed = true;  break; }
    }
    r.time = t;
    return r;
}

// 1 回だけ切り替える時刻を from から to まで step 刻みで振る。成功した時刻の範囲を返す
struct Window { float first = -1, last = -1; int count = 0; std::string map; };

static Window sweep(int stage, float from, float to, float step, const std::vector<float>& before = {}) {
    Window w;
    for (float T = from; T <= to + 1e-6f; T += step) {
        std::vector<float> s = before;
        s.push_back(T);
        RunResult r = runSchedule(stage, s);
        w.map += r.cleared ? '#' : (r.failed ? '.' : '-');
        if (r.cleared) {
            if (w.first < 0) w.first = T;
            w.last = T;
            ++w.count;
        }
    }
    return w;
}

static void printSweep(int stage, float from, float to, float step, const std::vector<float>& before = {}) {
    Window w = sweep(stage, from, to, step, before);
    std::printf("%s: switch time %.2f..%.2f step %.3f\n", stageName(stage), from, to, step);
    for (size_t i = 0; i < w.map.size(); i += 50)
        std::printf("  %5.2f  %s\n", from + i * step, w.map.substr(i, 50).c_str());
    std::printf("  clear at %d times, first %.2f last %.2f\n", w.count, w.first, w.last);
    RunResult none = runSchedule(stage, {});
    std::printf("  no switch: %s at %.2f s\n", none.cleared ? "clear" : none.failed ? "fail" : "timeout", none.time);
}

// 調整用: 最後の切替の時刻を振り、切り替えてから window 秒後の様子を 1 文字で表す
//   '#' クリア  '.' 失敗  数字: そのとき回っている軌道コアの番号  '-' それ以外
static std::string probeMap(int stage, float from, float to, float step, float window, const std::vector<float>& before) {
    std::string map;
    for (float T = from; T <= to + 1e-6f; T += step) {
        FlowWorld fw;
        loadStage(fw, stage);
        fw.press();
        std::vector<float> s = before;
        s.push_back(T);
        float  t = 0.0f;
        size_t k = 0;
        char   c = '-';
        while (t < T + window) {
            while (k < s.size() && t >= s[k] - 1e-6f) { fw.press(); ++k; }
            fw.step(DT);
            t += DT;
            if (fw.state == State::Cleared) { c = '#'; break; }
            if (fw.state == State::Failed)  { c = '.'; break; }
        }
        if (c == '-' && fw.orbitCore >= 0) c = (char)('0' + fw.orbitCore);
        map += c;
    }
    return map;
}

static void probe(int stage, float from, float to, float step, float window, const std::vector<float>& before) {
    const std::string map = probeMap(stage, from, to, step, window, before);
    std::printf("%s: last switch %.2f..%.2f step %.3f, +%.1f s\n", stageName(stage), from, to, step, window);
    for (size_t i = 0; i < map.size(); i += 50)
        std::printf("  %5.2f  %s\n", from + i * step, map.substr(i, 50).c_str());
}

// 文字 c が続く区間: {始まりの添字, 長さ}
struct Run { int start, len; };
static std::vector<Run> runsOf(const std::string& m, char c) {
    std::vector<Run> out;
    for (int i = 0; i < (int)m.size();) {
        if (m[i] != c) { ++i; continue; }
        int j = i;
        while (j < (int)m.size() && m[j] == c) ++j;
        out.push_back({i, j - i});
        i = j;
    }
    return out;
}
static Run longestRun(const std::string& m, char c) {
    Run best{-1, 0};
    for (const Run& r : runsOf(m, c)) if (r.len > best.len) best = r;
    return best;
}

// +y を dir に向ける回転
static Quat rotYTo(const Vec3& dir) {
    const Vec3  d = normalize(dir);
    if (d.y > 0.9999f)  return Quat{};
    if (d.y < -0.9999f) return Quat::fromAxisAngle({1, 0, 0}, PHYS_PI);
    return Quat::fromAxisAngle(cross(Vec3{0, 1, 0}, d), std::acos(clampf(d.y, -1.0f, 1.0f)));
}

static float angleDeg(const Vec3& a, const Vec3& b) {
    return std::acos(clampf(dot(normalize(a), normalize(b)), -1.0f, 1.0f)) * 180.0f / PHYS_PI;
}

// ---------------------------------------------------------------------------
// 大きなマップ: 区間 cp の発射台から走らせる（開始 = 発射、そのあと区間の時刻 sw で切り替える）。
//   次のチェックポイントに着く（最後はゴール）か、落ちるまで
// ---------------------------------------------------------------------------
struct ZoneRun { bool reached = false, failed = false; float time = 0.0f; };

static void startZone(FlowWorld& fw, int cp) {
    loadStage(fw, GRAND_STAGE);
    if (cp > 0) fw.warpTo(cp);
}

// 切替の時刻の列の先頭が負なら、発射せずに開始して、-sw[0] 秒後に発射する（発射台の上で待つ）
static bool g_waitFirst = false;

static ZoneRun runZone(FlowWorld& fw, int cp, const std::vector<float>& sw, float maxT = 15.0f) {
    startZone(fw, cp);
    if (g_waitFirst) fw.begin();
    else             fw.press();
    ZoneRun r;
    size_t k = 0;
    while (fw.stats.time < maxT) {
        while (k < sw.size() && fw.stats.time >= sw[k] - 1e-6f) { fw.press(); ++k; }
        fw.step(DT);
        if (fw.checkpoint > cp || fw.state == State::Cleared) { r.reached = true; break; }
        if (fw.state == State::Failed) { r.failed = true; break; }
    }
    r.time = fw.stats.time;
    return r;
}

// 区間 cp で、最後の切替の時刻を振り、切り替えてから window 秒後の様子を 1 文字で表す
//   '#' 次のチェックポイント  '.' 落ちた  数字: 回っている軌道コア  'A'..: 綱がつながっている錨
//   'a'..: 触れている固定の鋼  '-' それ以外
static std::string zoneMap(int cp, float from, float to, float step, float window, const std::vector<float>& before) {
    std::string map;
    for (float T = from; T <= to + 1e-6f; T += step) {
        FlowWorld fw;
        std::vector<float> s = before;
        s.push_back(T);
        const ZoneRun r = runZone(fw, cp, s, T + window);
        char c = r.reached ? '#' : r.failed ? '.' : '-';
        if (c == '-' && fw.orbitCore >= 0) c = (char)('0' + fw.orbitCore);
        if (c == '-' && fw.swingActive >= 0) c = (char)('A' + fw.swingActive);
        if (c == '-') {   // 触れている鋼（'a'..）
            for (int i = 0; i < (int)fw.steels.size(); ++i) {
                const RigidBody* b = fw.steels[i].body;
                if (!b->isStatic()) continue;
                const Mat3 R = b->orientation.toMat3();
                Vec3 q = R.transposed() * (fw.player->position - b->position);
                for (int a = 0; a < 3; ++a) q[a] = clampf(q[a], -b->shape.halfExtents[a], b->shape.halfExtents[a]);
                if (length(fw.player->position - (b->position + R * q)) < 0.8f) { c = (char)('a' + i); break; }
            }
        }
        map += c;
    }
    return map;
}

static void zoneTrace(int cp, const std::vector<float>& sw, float maxT) {
    FlowWorld fw;
    startZone(fw, cp);
    if (g_waitFirst) fw.begin();
    else             fw.press();
    size_t k = 0;
    int i = 0;
    while (fw.stats.time < maxT && fw.state == State::Running && fw.checkpoint <= cp) {
        while (k < sw.size() && fw.stats.time >= sw[k] - 1e-6f) { fw.press(); ++k; }
        if (i++ % 12 == 0) {   // 区間の座標で
            const Vec3 p = grandToLocal(cp, fw.player->position), v = grandToLocal(cp, fw.player->velocity) -
                                                                      grandToLocal(cp, Vec3{0, 0, 0});
            std::printf("t %5.2f %c  p (%7.2f %7.2f %7.2f)  v (%6.2f %6.2f %6.2f) |v| %5.2f  core %d swing %d", fw.stats.time,
                        fw.polarity > 0 ? 'N' : 'S', p.x, p.y, p.z, v.x, v.y, v.z, length(v), fw.orbitCore, fw.swingActive);
            for (const auto& bp : fw.world.getBodies()) {   // 動く物の位置
                const Kind kd = fw.kindOf(bp.get());
                if ((kd == Kind::MovableMagnet || kd == Kind::Iron) && !bp->isStatic()) {
                    const Vec3 q = grandToLocal(cp, bp->position);
                    std::printf("  [%.1f %.1f %.1f]", q.x, q.y, q.z);
                }
            }
            for (const Gate& g : fw.gates) if (g.zone == fw.zone) std::printf("  gate %.2f", g.open);
            for (const Rotor& r : fw.rotors) if (r.zone == fw.zone) std::printf("  rotor %.1f", r.angle * 57.3f);
            std::printf("\n");
        }
        fw.step(DT);
    }
    std::printf("end: %s at %.2f (checkpoint %d)\n", fw.checkpoint > cp || fw.state == State::Cleared ? "REACHED" :
                fw.state == State::Failed ? "fail" : "running", fw.stats.time, fw.checkpoint);
}

// ---------------------------------------------------------------------------
// 大きなマップを通しで走るボット: 区間ごとに、発射台に着いて（座って）から決めた時刻に押す。
//   fire: 着いてから発射するまで [s]。flips: 切り替える時刻（wait なら着いてから、そうでなければ発射から）
// ---------------------------------------------------------------------------
struct ZonePlan { float fire; std::vector<float> flips; bool wait; };

static std::vector<ZonePlan> grandPlan() {
    return {
        {0.0f, {2.0f, 2.57f}, false},                 // 1 磁気峡谷（開始 = 発射）
        {0.8f, {4.16f}, false},                       // 2 振り子の谷（2 周目で放す）
        {0.8f, {4.86f, 7.42f, 11.00f}, false},        // 3 軌道庭園（M1 を 3 周、M2、M3）
        {0.8f, {0.80f}, false},                       // 4 レンズ回廊
        {0.8f, {2.5f, 5.5f, 6.3f}, false},            // 5 磁気工場
        {0.8f, {1.6f, 4.0f, 8.6f}, false},            // 6 鋼の飛び石
        {1.2f, {4.74f}, true},                        // 7 磁気嵐（台が来るのを待つ）
        {5.0f, {}, true},                             // 8 磁針の橋（針がこちらを向くのを待つ）
        {0.8f, {2.76f, 4.6f, 9.0f, 11.92f}, false},   // 9 磁気コア
    };
}

struct GrandRun { bool cleared = false; float time = 0.0f; int falls = 0; std::vector<float> arrive; };
static int g_traceZone = -1;   // --grandrun <file|-> <区間>: その区間の軌跡を表示する

static GrandRun runGrand(FlowWorld& fw, const std::vector<ZonePlan>& plan, bool verbose = false) {
    loadStage(fw, GRAND_STAGE);
    GrandRun r;
    // 押す時刻（絶対）を、区間に着くたびに足していく
    std::vector<float> presses{0.0f};   // 最初は開始 = 発射
    int zoneSeen = 0;
    auto addZone = [&](int k, float t0) {
        const ZonePlan& p = plan[k];
        const float base = p.wait ? t0 : t0 + p.fire;
        if (k > 0) presses.push_back(t0 + p.fire);
        for (float f : p.flips) presses.push_back(base + f);
        std::sort(presses.begin(), presses.end());
    };
    r.arrive.push_back(0.0f);
    addZone(0, 0.0f);
    size_t next = 0;
    fw.press();
    ++next;
    while (fw.stats.time < 200.0f && fw.state != State::Cleared) {
        while (next < presses.size() && fw.state == State::Running && fw.stats.time >= presses[next] - 1e-6f) {
            fw.press();
            ++next;
        }
        fw.step(DT);
        if (verbose && g_traceZone == zoneSeen && ((int)(fw.stats.time * 120.0f + 0.5f)) % 24 == 0) {
            const Vec3 p = grandToLocal(zoneSeen, fw.player->position);
            std::printf("    t %6.2f (zone %5.2f) %c p (%6.2f %6.2f %6.2f) |v| %5.2f core %d\n", fw.stats.time,
                        fw.stats.time - r.arrive.back(), fw.polarity > 0 ? 'N' : 'S', p.x, p.y, p.z,
                        length(fw.player->velocity), fw.orbitCore);
            for (const Rotor& ro : fw.rotors) std::printf("    rotor %.1f\n", ro.angle * 57.2958f);
        }
        if (fw.checkpoint > zoneSeen && fw.checkpoint < (int)plan.size()) {
            zoneSeen = fw.checkpoint;
            r.arrive.push_back(fw.stats.time);
            if (verbose) std::printf("  zone %d reached at %.2f s (falls %d)\n", zoneSeen + 1, fw.stats.time, fw.stats.falls);
            addZone(zoneSeen, fw.stats.time);
        }
        if (fw.stats.falls > r.falls) {
            r.falls = fw.stats.falls;
            if (verbose) std::printf("  FELL in zone %d at %.2f s\n", zoneSeen + 1, fw.stats.time);
            break;
        }
    }
    r.cleared = fw.state == State::Cleared;
    r.time    = fw.stats.time;
    return r;
}

// 通しの走りの中で、区間 zone の最後の切替（区間の計画の flips の最後を from..to で振る）を測る。
//   それより前の区間は計画どおりに走る。文字は zoneMap と同じ（'#' は次のチェックポイント）
static std::string grandProbeMap(int zone, float from, float to, float step, float window, std::vector<float> flips) {
    std::string map;
    for (float T = from; T <= to + 1e-6f; T += step) {
        std::vector<ZonePlan> plan = grandPlan();
        std::vector<float> f = flips;
        f.push_back(T);
        plan[zone].flips = f;
        FlowWorld fw;
        loadStage(fw, GRAND_STAGE);
        std::vector<float> presses{0.0f};
        int seen = 0;
        float zoneT0 = 0.0f, lastAbs = 1e9f;
        auto add = [&](int k, float t0) {
            const ZonePlan& p = plan[k];
            const float base = p.wait ? t0 : t0 + p.fire;
            if (k > 0) presses.push_back(t0 + p.fire);
            for (float x : p.flips) presses.push_back(base + x);
            if (k == zone) lastAbs = base + T;
            std::sort(presses.begin(), presses.end());
        };
        add(0, 0.0f);
        size_t next = 1;
        fw.press();
        char c = '?';
        while (fw.stats.time < 300.0f) {
            while (next < presses.size() && fw.state == State::Running && fw.stats.time >= presses[next] - 1e-6f) {
                fw.press();
                ++next;
            }
            fw.step(DT);
            if (fw.state == State::Cleared) { c = '#'; break; }
            if (fw.stats.falls > 0) { c = seen < zone ? '?' : '.'; break; }
            if (fw.checkpoint > seen) {
                seen = fw.checkpoint;
                if (seen > zone) { c = '#'; break; }
                zoneT0 = fw.stats.time;
                add(seen, zoneT0);
            }
            if (fw.stats.time > lastAbs + window) {
                c = fw.orbitCore >= 0 ? (char)('0' + fw.orbitCore) : fw.swingActive >= 0 ? (char)('A' + fw.swingActive) : '-';
                break;
            }
        }
        map += c;
    }
    return map;
}

// ---------------------------------------------------------------------------
// 新しいギミックのステージ（L1〜L4）: 状態を見て押すボット
//   L1 サイクロトロン: 隙間に入る delta [s] 前に、入っていく半分の極へ切り替える（切替 k = 隙間を通った回数）
//   L2 加速リング    : 輪 k の面をくぐって delta 後に、その輪と同じ極へ切り替える（押し出される）
//   L3 共振ブランコ  : おもりが最下点を通って delta 後に押す極（N）へ、折り返したら引く極（S）へ。
//                      壁が壊れたら発射する。phase = -1 で逆の位相（近づくとき押し、離れるとき引く）
//   L4 鉄球のロープ  : 決めた時刻 LAB_CHAIN_FLIPS に切り替える
//   onlyK >= 0 なら k 番目の切替だけ delta ずらす（< 0 ならすべて）。skipK 番目の切替は抜かす
// ---------------------------------------------------------------------------
static std::vector<float> LAB_CHAIN_FLIPS = {1.68f, 3.03f, 4.83f};   // 調整: 環境変数 LAB_FLIPS="t1,t2,..." で置き換える

struct LabOpts { float delta = 0.0f; int onlyK = -1, skipK = -1, phase = +1; float maxT = 30.0f; bool trace = false; };
struct LabRun {
    bool  cleared = false, failed = false;
    float time = 0.0f;
    int   flips = 0;
    std::vector<float> evT, evV;   // 出来事の時刻と値（L1: 隙間を通った・速さ、L2: 輪を抜けた・速さ、L3: 折り返し・振れ [deg]、L4: つかんだ鎖）
    float breakT = -1.0f;          // L3: 壁が壊れた時刻
    bool  firedEarly = false;      // L3: 壁が壊れる前に押して発射した
    float dvDee = 0.0f;            // L1: 半分の中での速さの変化の最大（割合）
    float maxGap = 0.0f;           // L4: 鎖の球どうしのすき間の最大
    Vec3  exitP{0, 0, 0}, exitV{0, 0, 0};   // L1: 磁場から出たところと速度。L2: 最後の輪を抜けたところ
    bool  exited = false;
};

static float chainGapOf(const BallChain& ch) {
    float g = 0.0f;
    const RigidBody* prev = ch.top;
    for (const RigidBody* b : ch.balls) {
        g = std::max(g, length(b->position - prev->position) - prev->shape.radius - b->shape.radius);
        prev = b;
    }
    return g;
}

static LabRun runLab(FlowWorld& fw, int lab, const LabOpts& o = LabOpts{}) {
    loadStage(fw, LAB_STAGE + lab);
    fw.press();   // 開始
    LabRun r;
    int   k = 0, handled = -1;
    float prevA = 0.0f, prevW = 0.0f, pendingT = -1.0f, lastPeak = -1.0f, lastSpeed = 0.0f;
    int   want = 0, lastAtt = -1;
    bool  fired = false;
    if (lab == 2) { prevA = fw.rotors[0].angle; prevW = fw.rotors[0].omega; lastPeak = fw.rotors[0].peak; }
    while (fw.stats.time < o.maxT && fw.state == State::Running) {
        auto dOf = [&](int i) { return (o.onlyK < 0 || o.onlyK == i) ? o.delta : 0.0f; };
        auto press = [&](int i) {
            if (i == o.skipK || o.phase == 0) return;   // phase = 0: 何も押さない
            fw.press();
            ++r.flips;
        };
        if (lab == 0) {
            const Cyclotron& c = fw.cyclotrons[0];
            Vec3  rIn;
            float h;
            const int gi = (int)std::lround(c.turns * 2.0f);   // これから通る隙間
            if (gi != handled && fw.cyclotronInside(c, fw.player->position, &rIn, &h)) {
                const float u = dot(rIn, c.gapN), vn = dot(fw.player->velocity, c.gapN);
                const int   target = vn > 0.0f ? 1 : -1;
                const float s = -u * (float)target;   // 進む向きで、隙間の中心までの距離
                const float gh = c.gapHalf;
                const float trel = (s - gh) / std::max(0.3f, std::fabs(vn));
                const bool justMissed = o.skipK >= 0 && gi == o.skipK + 1 && s < 0.0f;   // 抜かした直後は次の隙間まで待つ
                if (fw.polarity == target && trel > 0.0f) handled = gi;   // もう合っている（始めの隙間）
                else if (fw.polarity != target && s > -(gh + 3.0f) && trel <= -dOf(gi) && !justMissed) { press(gi); handled = gi; }
            }
        } else if (lab == 1) {
            if (k < (int)fw.rings.size()) {
                const GaussRing& g = fw.rings[k];
                const float x = dot(fw.player->position - g.center, g.axis), va = dot(fw.player->velocity, g.axis);
                const float tPlane = va > 0.1f ? -x / va : 1e9f;   // 面までの時間（くぐった後は負）
                if (k == o.skipK) { if (x > g.reach) ++k; }
                else if (fw.polarity != g.pole && tPlane <= -dOf(k)) { press(k); ++k; }
                else if (fw.polarity == g.pole && x > g.reach) ++k;
            }
        } else if (lab == 2) {
            const Rotor& ro = fw.rotors[0];
            if (ro.reachedHi) {
                if (r.breakT < 0.0f) r.breakT = fw.stats.time;
                if (!fired && fw.stats.time > r.breakT + 0.6f) { fw.press(); fired = true; }
            } else {
                // 最下点を通った: 押す（おもりと同じ N）。折り返した: 引く（S）
                if (o.phase != 0 && prevA * ro.angle < 0.0f) { want = o.phase > 0 ? +1 : -1; pendingT = fw.stats.time + dOf(k); ++k; }
                if (o.phase != 0 && prevW * ro.omega < 0.0f) { want = o.phase > 0 ? -1 : +1; pendingT = fw.stats.time; }
                if (want != 0 && fw.stats.time >= pendingT - 1e-6f) {
                    if (fw.polarity != want) {
                        fw.press();
                        ++r.flips;
                        if (fw.events.launched) r.firedEarly = true;
                    }
                    want = 0;
                }
            }
            prevA = ro.angle;
            prevW = ro.omega;
        } else {
            if (k < (int)LAB_CHAIN_FLIPS.size() && fw.stats.time >= LAB_CHAIN_FLIPS[k] + dOf(k) - 1e-6f) { press(k); ++k; }
        }
        fw.step(DT);
        // 出来事
        if (lab == 0) {
            const float v = length(fw.player->velocity);
            const Cyclotron& c = fw.cyclotrons[0];
            if ((int)r.evT.size() < (int)std::lround(c.turns * 2.0f)) { r.evT.push_back(fw.stats.time); r.evV.push_back(v); }
            Vec3  rIn;
            float h;
            const bool in = fw.cyclotronInside(c, fw.player->position, &rIn, &h);
            if (in && std::fabs(dot(rIn, c.gapN)) > c.gapHalf + c.fringe + 0.5f && lastSpeed > 0.0f)
                r.dvDee = std::max(r.dvDee, std::fabs(v - lastSpeed) / lastSpeed);
            if (!in && !r.exited) { r.exited = true; r.exitP = fw.player->position; r.exitV = fw.player->velocity; }
            lastSpeed = v;
        } else if (lab == 1) {
            for (int i = (int)r.evT.size(); i < (int)fw.rings.size(); ++i) {
                const GaussRing& g = fw.rings[i];
                if (dot(fw.player->position - g.center, g.axis) <= g.reach) break;
                r.evT.push_back(fw.stats.time);
                r.evV.push_back(length(fw.player->velocity));
                if (i + 1 == (int)fw.rings.size()) { r.exited = true; r.exitP = fw.player->position; r.exitV = fw.player->velocity; }
            }
        } else if (lab == 2) {
            const Rotor& ro = fw.rotors[0];
            if (ro.peak != lastPeak) { lastPeak = ro.peak; r.evT.push_back(fw.stats.time); r.evV.push_back(ro.peak * 180.0f / PHYS_PI); }
        } else {
            for (int c = 0; c < (int)fw.chains.size(); ++c) {
                r.maxGap = std::max(r.maxGap, chainGapOf(fw.chains[c]));
                if (c != lastAtt && length(fw.player->position - fw.chains[c].balls.back()->position) < 0.86f) {
                    lastAtt = c;
                    r.evT.push_back(fw.stats.time);
                    r.evV.push_back((float)c);
                }
            }
        }
        if (o.trace && (int)std::lround(fw.stats.time * 120.0f) % 12 == 0) {
            const Vec3 p = fw.player->position, v = fw.player->velocity;
            std::printf("t %5.2f %c  p (%6.2f %6.2f %6.2f)  v (%6.2f %6.2f %6.2f) |v| %5.2f\n", fw.stats.time,
                        fw.polarity > 0 ? 'N' : 'S', p.x, p.y, p.z, v.x, v.y, v.z, length(v));
        }
    }
    r.cleared = fw.state == State::Cleared;
    r.failed  = fw.state == State::Failed;
    r.time    = fw.stats.time;
    return r;
}

// 切替 k だけを delta ずらして、ok(r) を満たす delta の範囲の地図（from..to、step 刻み）
template <class Ok>
static std::string labWindowMap(int lab, int k, float from, float to, float step, Ok ok) {
    std::string m;
    for (float d = from; d <= to + 1e-6f; d += step) {
        FlowWorld fw;
        LabOpts o;
        o.delta = d;
        o.onlyK = k;
        m += ok(runLab(fw, lab, o)) ? '#' : '.';
    }
    return m;
}

int main(int argc, char** argv) {
    if (const char* lf = std::getenv("LAB_FLIPS")) {
        LAB_CHAIN_FLIPS.clear();
        for (const char* q = lf; *q;) {
            LAB_CHAIN_FLIPS.push_back((float)std::atof(q));
            while (*q && *q != ',') ++q;
            if (*q == ',') ++q;
        }
    }
    if (argc >= 3 && std::strcmp(argv[1], "--lab") == 0) {   // flow_test --lab <1..4> [delta] [onlyK] [skipK] [phase]
        const int lab = std::atoi(argv[2]) - 1;
        LabOpts o;
        o.trace = true;
        if (argc > 3) o.delta = (float)std::atof(argv[3]);
        if (argc > 4) o.onlyK = std::atoi(argv[4]);
        if (argc > 5) o.skipK = std::atoi(argv[5]);
        if (argc > 6) o.phase = std::atoi(argv[6]);
        FlowWorld fw;
        const LabRun r = runLab(fw, lab, o);
        std::printf("events:");
        for (size_t i = 0; i < r.evT.size(); ++i) std::printf(" [%.2f %.2f]", r.evT[i], r.evV[i]);
        std::printf("\n%s at %.2f s, flips %d, break %.2f, dvDee %.4f, max gap %.3f, end p (%.2f %.2f %.2f)\n",
                    r.cleared ? "CLEAR" : r.failed ? "FAIL" : "timeout", r.time, r.flips, r.breakT, r.dvDee, r.maxGap,
                    fw.player->position.x, fw.player->position.y, fw.player->position.z);
        if (r.exited) {   // 出たところから、高さ landY に落ちる位置（重力だけ）
            const float landY = argc > 7 ? (float)std::atof(argv[7]) : 0.5f;
            const Vec3 p = r.exitP, v = r.exitV;
            const float a = 0.5f * G_ACC, b = -v.y, cc = landY - p.y;   // p.y + v.y t - a t^2 = landY
            const float t = (-b + std::sqrt(std::max(0.0f, b * b - 4.0f * a * cc))) / (2.0f * a);
            std::printf("exit p (%.2f %.2f %.2f) v (%.2f %.2f %.2f) |v| %.2f -> lands at y %.1f: (%.2f, %.2f) after %.2f s\n", p.x, p.y, p.z,
                        v.x, v.y, v.z, length(v), landY, p.x + v.x * t, p.z + v.z * t, t);
        }
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--labwin") == 0) {   // flow_test --labwin <1..4> [from] [to] [step]: 切替ごとの窓（クリアするか）
        const int   lab  = std::atoi(argv[2]) - 1;
        const float from = argc > 3 ? (float)std::atof(argv[3]) : -0.3f;
        const float to   = argc > 4 ? (float)std::atof(argv[4]) : 0.3f;
        const float step = argc > 5 ? (float)std::atof(argv[5]) : 0.02f;
        FlowWorld fw;
        const LabRun base = runLab(fw, lab);
        std::printf("base: %s at %.2f s, flips %d\n", base.cleared ? "CLEAR" : "not cleared", base.time, base.flips);
        const int K = lab == 0 ? (int)base.evT.size() + 1 : lab == 1 ? (int)fw.rings.size() : lab == 2 ? 8 : (int)LAB_CHAIN_FLIPS.size();
        for (int k = 0; k < K; ++k) {
            const std::string m = labWindowMap(lab, k, from, to, step, [](const LabRun& r) { return r.cleared; });
            std::printf("flip %2d  %s  %4d ms\n", k, m.c_str(), longestRun(m, '#').len * (int)std::lround(step * 1000.0f));
        }
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--labrecord") == 0) {   // flow_test --labrecord <file>: L1〜L4 のボットの走りを記録に足す（GIF 用）
        RecordBook book;
        book.load(argv[2]);
        for (int lab = 0; lab < 4; ++lab) {
            FlowWorld fw;
            const LabRun r = runLab(fw, lab);
            if (!r.cleared) { std::printf("%s: not cleared\n", stageKey(LAB_STAGE + lab)); return 1; }
            book.offer(stageKey(LAB_STAGE + lab), StageRecord::fromWorld(fw));
            std::printf("%s: clear %.2f s\n", stageKey(LAB_STAGE + lab), r.time);
        }
        book.save(argv[2]);
        return 0;
    }
    if (argc >= 7 && std::strcmp(argv[1], "--gprobe") == 0) {   // flow_test --gprobe <区間 1..9> <from> <to> <step> <window> [前の切替...]
        const int zone = std::atoi(argv[2]) - 1;
        std::vector<float> flips;
        for (int i = 7; i < argc; ++i) flips.push_back((float)std::atof(argv[i]));
        const float from = (float)std::atof(argv[3]), step = (float)std::atof(argv[5]);
        const std::string map = grandProbeMap(zone, from, (float)std::atof(argv[4]), step, (float)std::atof(argv[6]), flips);
        std::printf("zone %d (full run): last switch from %.2f step %.3f\n", zone + 1, from, step);
        for (size_t i = 0; i < map.size(); i += 50) std::printf("  %6.2f  %s\n", from + i * step, map.substr(i, 50).c_str());
        return 0;
    }
    if (argc >= 2 && std::strcmp(argv[1], "--grandrun") == 0) {   // 大きなマップを通しで（記録のファイルを渡すと保存）
        FlowWorld fw;
        if (argc >= 4) g_traceZone = std::atoi(argv[3]) - 1;
        const GrandRun r = runGrand(fw, grandPlan(), true);
        Vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};   // マップの大きさと、飛んだ道のり
        for (const Zone& z : fw.zones)
            for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], z.lo[a]); hi[a] = std::max(hi[a], z.hi[a]); }
        float path = 0.0f;
        for (size_t i = 1; i < fw.trail.size(); ++i) path += length(fw.trail[i] - fw.trail[i - 1]);
        std::printf("map %.0f x %.0f x %.0f m, %d bodies, path flown %.0f m\n", hi.x - lo.x, hi.y - lo.y, hi.z - lo.z,
                    fw.world.bodyCount(), path);
        std::printf("GRAND: %s at %.2f s, switches %d, max speed %.1f, flow %d, magnets %d/%d, falls %d\n",
                    r.cleared ? "CLEAR" : "not cleared", r.time, fw.stats.switches, fw.stats.maxSpeed, fw.stats.bestCombo,
                    fw.stats.magnetsUsed, fw.magnetTotal, fw.stats.falls);
        if (r.cleared && argc >= 3 && std::strcmp(argv[2], "-") != 0) {
            RecordBook book;
            book.load(argv[2]);
            book.offer(stageKey(GRAND_STAGE), StageRecord::fromWorld(fw));
            book.save(argv[2]);
            std::printf("saved to %s\n", argv[2]);
        }
        return r.cleared ? 0 : 1;
    }
    // 大きなマップの調整: 既定は区間だけを組む（速い）。--full でマップ全部
    //   --wait: 発射台の上で待ってから発射する（最初の切替の時刻が発射）
    bool full = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--full") == 0 || std::strcmp(argv[i], "--wait") == 0) {
            (argv[i][2] == 'f' ? full : g_waitFirst) = true;
            for (int j = i; j + 1 < argc; ++j) argv[j] = argv[j + 1];
            --argc;
            --i;
        }
    }
    if (argc >= 7 && std::strcmp(argv[1], "--zprobe") == 0) {   // flow_test --zprobe <cp> <from> <to> <step> <window> [前の切替...]
        const int cp = std::atoi(argv[2]);
        if (!full) grandOnlyZone = cp;
        std::vector<float> before;
        for (int i = 7; i < argc; ++i) before.push_back((float)std::atof(argv[i]));
        const float from = (float)std::atof(argv[3]), step = (float)std::atof(argv[5]);
        const std::string map = zoneMap(cp, from, (float)std::atof(argv[4]), step, (float)std::atof(argv[6]), before);
        std::printf("zone %d: last switch from %.2f step %.3f\n", cp + 1, from, step);
        for (size_t i = 0; i < map.size(); i += 50) std::printf("  %6.2f  %s\n", from + i * step, map.substr(i, 50).c_str());
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--ztrace") == 0) {   // flow_test --ztrace <cp> [切替時刻...]
        const int cp = std::atoi(argv[2]);
        if (!full) grandOnlyZone = cp;
        std::vector<float> sw;
        for (int i = 3; i < argc; ++i) sw.push_back((float)std::atof(argv[i]));
        zoneTrace(cp, sw, 20.0f);
        return 0;
    }
    if (argc >= 4 && std::strcmp(argv[1], "--record") == 0) {   // flow_test --record <file> <stage> [切替時刻...]
        // 決めた入力で走らせ、クリアしたら記録のファイルに足す（ゲームの P で流し直せる。GIF 用）
        RecordBook book;
        book.load(argv[2]);
        const int stage = std::atoi(argv[3]);
        std::vector<float> sw;
        for (int i = 4; i < argc; ++i) sw.push_back((float)std::atof(argv[i]));
        FlowWorld fw;
        const RunResult r = runWorld(fw, stage, sw);
        if (!r.cleared) {
            std::printf("%s: not cleared (%s at %.2f s)\n", stageKey(stage), r.failed ? "fail" : "timeout", r.time);
            return 1;
        }
        book.offer(stageKey(stage), StageRecord::fromWorld(fw));
        book.save(argv[2]);
        std::printf("%s: clear %.2f s, saved to %s\n", stageKey(stage), fw.stats.time, argv[2]);
        return 0;
    }
    if (argc >= 7 && std::strcmp(argv[1], "--probe") == 0) {   // flow_test --probe <stage> <from> <to> <step> <window> [前の切替...]
        std::vector<float> before;
        for (int i = 7; i < argc; ++i) before.push_back((float)std::atof(argv[i]));
        probe(std::atoi(argv[2]), (float)std::atof(argv[3]), (float)std::atof(argv[4]), (float)std::atof(argv[5]),
              (float)std::atof(argv[6]), before);
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--sweep") == 0) {
        const int   stage = std::atoi(argv[2]);
        const float from  = argc > 3 ? (float)std::atof(argv[3]) : 0.0f;
        const float to    = argc > 4 ? (float)std::atof(argv[4]) : 4.0f;
        const float step  = argc > 5 ? (float)std::atof(argv[5]) : 0.02f;
        std::vector<float> before;   // 振る切替の前に、決めた時刻で切り替える
        for (int i = 6; i < argc; ++i) before.push_back((float)std::atof(argv[i]));
        printSweep(stage, from, to, step, before);
        return 0;
    }

    if (argc >= 3 && std::strcmp(argv[1], "--trace") == 0) {   // 軌跡: flow_test --trace <stage> [切替時刻...]
        const int stage = std::atoi(argv[2]);
        std::vector<float> sw;
        for (int i = 3; i < argc; ++i) sw.push_back((float)std::atof(argv[i]));
        FlowWorld fw;
        loadStage(fw, stage);
        fw.press();
        float  t = 0.0f;
        size_t k = 0;
        for (int i = 0; i < 1800 && fw.state == State::Running; ++i) {
            while (k < sw.size() && t >= sw[k] - 1e-6f) { fw.press(); ++k; }
            if (i % 12 == 0) {
                const Vec3 p = fw.player->position, v = fw.player->velocity;
                std::printf("t %5.2f %c  p (%6.2f %6.2f %6.2f)  v (%6.2f %6.2f %6.2f)  F (%6.1f %6.1f %6.1f)\n", t,
                            fw.polarity > 0 ? 'N' : 'S', p.x, p.y, p.z, v.x, v.y, v.z, fw.netForce.x, fw.netForce.y,
                            fw.netForce.z);
                for (const auto& bp : fw.world.getBodies()) {   // 動く物の位置
                    const Kind kd = fw.kindOf(bp.get());
                    if (kd == Kind::MovableMagnet || kd == Kind::Iron)
                        std::printf("    [%.1f %.1f %.1f]", bp->position.x, bp->position.y, bp->position.z);
                }
                for (const Gate& g : fw.gates) std::printf("  gate %.2f", g.open);
                std::printf("\n");
            }
            fw.step(DT);
            t += DT;
        }
        std::printf("end: %s at %.2f\n", fw.state == State::Cleared ? "clear" : fw.state == State::Failed ? "fail" : "running", t);
        return 0;
    }

    char buf[256];

    // ---------------- FL1. 極性を変えると力が逆になる ----------------
    {
        FlowWorld fw;
        fw.reset();
        fw.addFixedMagnetSphere({0, 0, 0}, 1.0f, {0, 1, 0}, 1.0f, 10.0f);   // N 極が上
        fw.startPos = {0, 3, 0};
        fw.finishBuild();
        float s = 1.0f;
        const Vec3 Fn = fw.computePlayerForce(nullptr, &s);   // プレイヤー N、上に N 極 → 反発（+y）
        fw.polarity = -1;
        const Vec3 Fs = fw.computePlayerForce(nullptr, &s);   // S → 吸引（-y）
        const float err = length(Fn + Fs) / length(Fn);
        std::snprintf(buf, sizeof buf, "N: Fy %+.3f  S: Fy %+.3f  |Fn + Fs| / |Fn| %.1e", Fn.y, Fs.y, err);
        check("FL1 polarity flips the force exactly", err < 1e-6f && Fn.y > 0.0f && Fs.y < 0.0f, buf);
    }

    // ---------------- FL2. 運動量（上限で縮めるときも） ----------------
    {
        FlowWorld fw;
        fw.params.forceCap = 0.5f;      // 上限で縮める場合を通す
        fw.params.speedCap = 1e9f;
        fw.reset();
        fw.addMovableMagnetSphere({3, 0, 0}, 0.5f, {-1, 0, 0}, 2.0f, 1.0f, 5.0f);
        fw.addIronSphere({0, 0, 5}, 0.4f, 1.0f, 0.3f);
        fw.startPos = {0, 0, 0};
        fw.finishBuild();
        fw.params.gravity = 0.0f;
        fw.press();
        auto momentum = [&]() {
            Vec3 P{0, 0, 0};
            for (const auto& b : fw.world.getBodies())
                if (!b->isStatic()) P += b->velocity * (1.0f / b->invMass);
            return P;
        };
        float maxErr = 0.0f, scale = 0.0f, maxP = 0.0f;
        int   capped = 0, contacts = 0;
        for (int i = 0; i < 360; ++i) {
            if (i == 180) fw.press();   // 途中で極性を切り替える
            float s = 1.0f;
            fw.computePlayerForce(nullptr, &s);
            capped += s < 1.0f;
            fw.step(DT);
            contacts += fw.world.contactCount();
            maxErr = std::max(maxErr, length(momentum()));
            maxP   = std::max(maxP, length(fw.player->velocity) / fw.player->invMass);
        }
        scale = maxErr / std::max(maxP, 1e-9f);
        std::snprintf(buf, sizeof buf, "|P| max %.2e (relative %.1e)  capped steps %d  contacts %d", maxErr, scale,
                      capped, contacts);
        check("FL2 momentum with reaction (and cap)", scale < 1e-4f && capped > 0, buf);
    }

    // ---------------- FL3. 力と速さの上限 ----------------
    {
        FlowWorld fw;
        loadStage(fw, 0);
        fw.press();
        const float cap = fw.params.forceCap * G_ACC;
        float maxF = 0.0f, maxV = 0.0f;
        for (int i = 0; i < 600; ++i) {
            if (i % 90 == 0) fw.press();
            fw.step(DT);
            maxF = std::max(maxF, length(fw.netForce));
            maxV = std::max(maxV, length(fw.player->velocity));
        }
        std::snprintf(buf, sizeof buf, "max |F| %.2f (cap %.2f)  max speed %.2f (cap %.1f)", maxF, cap, maxV,
                      fw.params.speedCap);
        check("FL3 force and speed caps", maxF <= cap * 1.0001f && maxV <= fw.params.speedCap * 1.0001f, buf);
    }

    // ---------------- FL4. やり直しの再現性 ----------------
    {
        auto run = [](FlowWorld& fw) {
            fw.press();
            for (int i = 0; i < 400; ++i) {
                if (i == 70 || i == 200) fw.press();
                fw.step(DT);
            }
            return fw.player->position;
        };
        FlowWorld fw;
        loadStage(fw, 3);
        const Vec3 a = run(fw);
        fw.restart();
        const Vec3 b = run(fw);
        FlowWorld other;
        loadStage(other, 3);
        const Vec3 c = run(other);
        const bool same = std::memcmp(&a, &b, sizeof a) == 0 && std::memcmp(&a, &c, sizeof a) == 0;
        std::snprintf(buf, sizeof buf, "(%.6f, %.6f, %.6f) restart %s, new world %s", a.x, a.y, a.z,
                      std::memcmp(&a, &b, sizeof a) == 0 ? "same" : "DIFFERENT",
                      std::memcmp(&a, &c, sizeof a) == 0 ? "same" : "DIFFERENT");
        check("FL4 restart reproduces the run bit for bit", same, buf);
    }

    // ---------------- FL5. P1・P2: 切り替える時刻の幅 ----------------
    //   1 回だけ切り替える時刻を 10 ms 刻みで振る。いちばん長く続けて成功する区間の幅を測る。
    //   その前（早すぎる）と後（遅すぎる）に失敗があること、切り替えないと失敗することも見る
    {
        auto analyse = [](const Window& w, float from, float step, int& longest, float& start, bool& early, bool& late) {
            int run = 0, bestEnd = -1;
            longest = 0;
            for (int i = 0; i < (int)w.map.size(); ++i) {
                run = w.map[i] == '#' ? run + 1 : 0;
                if (run > longest) { longest = run; bestEnd = i; }
            }
            const int bestStart = bestEnd - longest + 1;
            start = from + bestStart * step;
            early = bestStart > 0 && w.map.find('.') < (size_t)bestStart;
            late  = w.map.find('.', (size_t)bestEnd + 1) != std::string::npos;
        };
        struct Case { int stage; float from, to; int minMs, maxMs; };
        for (const Case& c : {Case{1, 0.8f, 2.2f, 150, 2000}, Case{2, 0.3f, 2.8f, 40, 250}}) {
            const Window w = sweep(c.stage, c.from, c.to, 0.01f);
            int longest; float start; bool early, late;
            analyse(w, c.from, 0.01f, longest, start, early, late);
            const RunResult none = runSchedule(c.stage, {});
            const int ms = longest * 10;
            std::snprintf(buf, sizeof buf, "window %d ms from %.2f s (early fail %s, late fail %s, no switch %s)", ms,
                          start, early ? "yes" : "no", late ? "yes" : "no", none.cleared ? "clears" : "fails");
            char name[64];
            std::snprintf(name, sizeof name, "FL5 %s timing window %d-%d ms", stageName(c.stage), c.minMs, c.maxMs);
            check(name, ms >= c.minMs && ms <= c.maxMs && early && late && !none.cleared, buf);
        }
    }

    // ---------------- FL6. P3: 連鎖でゲートが開き、待ってから切り替えるとゴール ----------------
    {
        // 何もしない（開始だけ）と、10 秒たってもゲートは開かない
        const RunResult idle = runSchedule(3, {}, 10.0f);
        // 1.0 s で S（A を弾く）、T2 で N（H2 から飛ぶ）。T2 を振る
        const Window w = sweep(3, 1.5f, 6.0f, 0.05f, {1.0f});
        // いちばん長い成功区間の前に失敗があること（開く前に飛ぶと扉に当たる）。
        // 途中で切り替えて直接ゴールへ届く「近道」が区間の外に 1 つ 2 つあってもよい
        int run = 0, longest = 0, bestEnd = -1;
        for (int i = 0; i < (int)w.map.size(); ++i) {
            run = w.map[i] == '#' ? run + 1 : 0;
            if (run > longest) { longest = run; bestEnd = i; }
        }
        const int  bestStart = bestEnd - longest + 1;
        const bool early     = bestStart > 0 && w.map[bestStart - 1] == '.';
        // 最初の切替の時刻を振っても、連鎖が起きて解けるか（壁に寄って止まってから。2 回目は 4 s 後）
        int okT1 = 0, nT1 = 0;
        for (float T1 = 0.8f; T1 <= 2.0f + 1e-6f; T1 += 0.1f, ++nT1) okT1 += runSchedule(3, {T1, T1 + 4.0f}).cleared;
        std::snprintf(buf, sizeof buf, "idle: gate %s; T2 1.5..6: %s; first switch 0.8..2.0 s: %d/%d clear",
                      idle.gateOpened ? "OPENED" : "closed", w.map.c_str(), okT1, nT1);
        check("FL6 P3 chain opens the gate, waiting pays off",
              !idle.gateOpened && w.count > 0 && early && okT1 == nT1, buf);
    }


    // ======== ここから magnetic_flow_gimmick_stage_spec.md のプロトタイプ（01, 02, 04）========
    constexpr int G01 = 4, G02 = 5, G04 = 6;

    // ---------------- FL7. 軌道コア: 吸われて周回し、周回で速くなり、回りすぎると振り飛ばされる ----------------
    {
        FlowWorld fw;
        loadStage(fw, G02);
        fw.press();
        const float cap = fw.params.forceCap * G_ACC;
        float captureT = -1.0f, escapeT = -1.0f, maxVortex = 0.0f, maxNet = 0.0f;
        float lapSum[8] = {}, lapN[8] = {};
        float lapsAtEscape = 0.0f;
        for (int i = 0; i < 1800 && fw.state == State::Running; ++i) {
            fw.step(DT);
            const float t = (i + 1) * DT;
            maxNet    = std::max(maxNet, length(fw.netForce));
            maxVortex = std::max(maxVortex, length(fw.vortexForce));
            if (captureT < 0.0f && fw.orbitCore == 0) captureT = t;
            if (captureT >= 0.0f && escapeT < 0.0f) {
                if (fw.orbitCore == 0) {
                    const int lap = std::min(7, (int)(std::max(0.0f, fw.orbitAngle) / (2.0f * PHYS_PI)));
                    lapSum[lap] += length(fw.player->velocity);
                    lapN[lap]   += 1.0f;
                    lapsAtEscape = fw.orbitAngle / (2.0f * PHYS_PI);
                } else {
                    escapeT = t;
                }
            }
        }
        const float v1 = lapN[0] > 0 ? lapSum[0] / lapN[0] : 0.0f, v3 = lapN[2] > 0 ? lapSum[2] / lapN[2] : 0.0f;
        std::snprintf(buf, sizeof buf, "captured %.2f s, escaped %.2f s after %.1f laps, mean speed lap1 %.1f lap3 %.1f, %s",
                      captureT, escapeT, lapsAtEscape, v1, v3, fw.state == State::Failed ? "then fell" : "NOT FAILED");
        check("FL7 core captures, spins up, flings off", captureT > 0.0f && captureT < 1.5f && lapsAtEscape >= 3.0f &&
                                                        v3 >= v1 + 3.0f && fw.state == State::Failed, buf);
        std::snprintf(buf, sizeof buf, "max |F_magnet| %.2f (cap %.2f), max |F_vortex| %.2f (outside the cap)", maxNet, cap,
                      maxVortex);
        check("FL7 vortex is added outside the force cap", maxNet <= cap * 1.0001f && maxVortex > 1.0f, buf);

        // 放した向き: 切り替えてから 0.5 s 後の速度の向きが、切り替える前の向きと近い（接線に近く飛び出す）
        float worst = 0.0f;
        for (float T = 4.66f; T <= 4.82f + 1e-6f; T += 0.04f) {
            FlowWorld f2;
            loadStage(f2, G02);
            f2.press();
            Vec3  v0;
            float t  = 0.0f;
            bool  sw = false;
            while (t < T + 0.5f) {
                if (!sw && t >= T - 1e-6f) { v0 = f2.player->velocity; f2.press(); sw = true; }
                f2.step(DT);
                t += DT;
            }
            const Vec3 v1s = f2.player->velocity;
            worst = std::max(worst, angleDeg(Vec3{v0.x, 0, v0.z}, Vec3{v1s.x, 0, v1s.z}));
        }
        std::snprintf(buf, sizeof buf, "largest turn in the horizontal plane 0.5 s after letting go: %.1f deg", worst);
        check("FL7 letting go flies roughly where you were heading", worst <= 30.0f, buf);
    }

    // ---------------- FL8. 回る扉: ヒンジのまわりに回り、付けた磁石の板も一緒に回る ----------------
    {
        FlowWorld fw;
        fw.reset();
        fw.addSolid({0, -0.5f, 0}, {6, 0.5f, 6});
        const Vec3 pivot{0.25f, 0, 0};
        Gate& g = fw.addHingedGate({0, 2.0f, 0}, {0.25f, 2.0f, 1.0f}, Quat{}, pivot, {0, 0, 1}, -0.5f * PHYS_PI,
                                   {-4.0f, 0, -1}, {-2.0f, 1, 1});
        g.openTime = 0.5f;
        // 扉の -x の面に、S がこちら（-x）を向く板（N は +x）
        RigidBody* pad = fw.addFixedMagnetBox({-0.45f, 3.0f, 0}, {0.8f, 0.2f, 0.8f}, rotYTo({1, 0, 0}), 1.0f, 0.0f);
        fw.attachToGate(fw.gates.back(), pad);
        fw.addIronSphere({-3.0f, 0.4f, 0}, 0.4f, 1.0f);   // 判定の箱の中
        fw.startPos = {-5, 0.5f, 4};
        fw.finishBuild();
        fw.press();
        for (int i = 0; i < 240; ++i) fw.step(DT);
        const Gate& gg = fw.gates.back();
        const Quat  R  = Quat::fromAxisAngle({0, 0, 1}, -0.5f * PHYS_PI);
        const Vec3  doorUp  = gg.door->orientation.rotate({0, 1, 0});
        const Vec3  doorPos = gg.door->position, wantDoor = pivot + R.rotate(Vec3{0, 2.0f, 0} - pivot);
        const Vec3  padN    = pad->orientation.rotate({0, 1, 0});
        const Vec3  padPos  = pad->position, wantPad = pivot + R.rotate(Vec3{-0.45f, 3.0f, 0} - pivot);
        // 倒れた板の真上の N のプレイヤー: 上を向いた S に引かれる（-y）
        fw.player->position = padPos + Vec3{0, 1.5f, 0};
        fw.polarity = +1;
        float sc = 1.0f;
        const Vec3  F   = fw.computePlayerForce(nullptr, &sc);
        const float err = length(doorPos - wantDoor) + length(padPos - wantPad);
        std::snprintf(buf, sizeof buf, "open %.2f, door +y -> (%.2f %.2f %.2f), pad N -> (%.2f %.2f %.2f), pos err %.1e, "
                      "N above: Fy %+.1f", gg.open, doorUp.x, doorUp.y, doorUp.z, padN.x, padN.y, padN.z, err, F.y);
        check("FL8 hinged gate turns with its magnet plate", gg.open >= 1.0f && angleDeg(doorUp, {1, 0, 0}) < 0.5f &&
                                                             angleDeg(padN, {0, -1, 0}) < 0.5f && err < 1e-4f && F.y < 0.0f,
              buf);
    }

    // ---------------- FL9. 01 磁気峡谷: M2 での切替のタイミング ----------------
    {
        // M1 に吸い付いてから 2.0 s で打ち上げ、M2 のそばで切り替える時刻を 10 ms 刻みで振る
        const std::string m1 = probeMap(G01, 2.2f, 3.0f, 0.01f, 3.0f, {2.0f});
        const Run  r1 = longestRun(m1, '#');
        const bool early = r1.start > 0 && m1.find('.') < (size_t)r1.start;
        const bool lateNoClear = r1.start + r1.len < (int)m1.size() && m1.find('#', r1.start + r1.len) == std::string::npos;
        // 打ち上げる時刻を 1 s 遅らせても、窓は同じ幅で 1 s ずれる（M1 の上で止まって待てる）
        const std::string m2 = probeMap(G01, 3.2f, 4.0f, 0.01f, 3.0f, {3.0f});
        const Run  r2 = longestRun(m2, '#');
        const RunResult idle    = runSchedule(G01, {}, 10.0f);
        const RunResult oneFlip = runSchedule(G01, {2.0f}, 10.0f);
        std::snprintf(buf, sizeof buf, "window %d ms at +%.2f s after launch (early fail %s, late %s); launch 1 s later: %d ms "
                      "at +%.2f s; idle %s, no second flip %s", r1.len * 10, 0.2f + r1.start * 0.01f, early ? "yes" : "no",
                      lateNoClear ? "no clear" : "CLEARS", r2.len * 10, 0.2f + r2.start * 0.01f,
                      idle.cleared ? "CLEARS" : "no clear", oneFlip.cleared ? "CLEARS" : "no clear");
        check("FL9 01 Canyon: flip at M2, window 150-400 ms", r1.len >= 15 && r1.len <= 40 && early && lateNoClear &&
                                                              std::abs(r2.len - r1.len) <= 2 &&
                                                              std::abs(r2.start - r1.start) <= 1 && !idle.cleared &&
                                                              !oneFlip.cleared, buf);
    }

    // ---------------- FL10. 02 軌道庭園: 周回で速さを稼ぐ・つなぐ・近道 ----------------
    {
        // M1 から放す時刻を振る（'1' = M2 に吸われて回っている）。周回ごとに窓があり、1 周目は狭い
        const std::string m = probeMap(G02, 1.0f, 7.0f, 0.02f, 3.0f, {});
        const std::vector<Run> laps = runsOf(m, '1');
        const int first = laps.empty() ? 0 : laps[0].len;
        int widest = 0;
        std::string lapStr;
        for (const Run& r : laps) { widest = std::max(widest, r.len); lapStr += std::to_string(r.len * 20) + " "; }
        FlowWorld a, b;
        const RunResult normal   = runWorld(a, G02, {4.74f, 7.48f, 10.9f});
        // M1 → M3（M2 を飛ばす。途中でもう一度切り替えて M3 に吸われる）
        const RunResult shortcut = runWorld(b, G02, {4.94f, 5.6f, 7.30f});
        const RunResult idle     = runSchedule(G02, {}, 15.0f);
        std::snprintf(buf, sizeof buf, "M1 release windows (ms) %s; route M1-M2-M3 %s %.2f s (%d magnets), skip M2 %s %.2f s "
                      "(%d magnets); idle %s", lapStr.c_str(), normal.cleared ? "clear" : "FAIL", normal.time,
                      a.stats.magnetsUsed, shortcut.cleared ? "clear" : "FAIL", shortcut.time, b.stats.magnetsUsed,
                      idle.failed ? "flung off" : "NOT FAILED");
        check("FL10 02 Orbit: build speed, connect, skip one", laps.size() >= 3 && first * 20 <= 100 && widest * 20 >= 140 &&
                                                               normal.cleared && shortcut.cleared &&
                                                               shortcut.time < normal.time - 2.0f &&
                                                               b.stats.magnetsUsed < a.stats.magnetsUsed && idle.failed,
              buf);
    }

    // ---------------- FL11. 04 磁気工場: 連鎖で扉が倒れて橋になり、橋の上の板で打ち上がる ----------------
    {
        const RunResult idle   = runSchedule(G04, {}, 10.0f);
        const RunResult normal = runSchedule(G04, {1.0f, 4.0f, 4.55f});
        const RunResult early  = runSchedule(G04, {1.0f, 1.8f, 2.3f});    // 扉が倒れる前に飛ぶ
        const RunResult ramp   = runSchedule(G04, {1.0f, 2.40f});         // 倒れかけの扉を踏み台にする近道
        const std::string m = probeMap(G04, 4.0f, 5.5f, 0.01f, 3.0f, {1.0f, 4.0f});   // 橋の上で切り替える時刻
        const Run r = longestRun(m, '#');
        std::snprintf(buf, sizeof buf, "idle: gate %s; route %s %.2f s; jump before the gate falls %s; bridge flip window %d ms; "
                      "ramp trick %s", idle.gateOpened ? "OPENED" : "closed", normal.cleared ? "clear" : "FAIL", normal.time,
                      early.cleared ? "CLEARS" : "fails", r.len * 10, ramp.cleared ? "clear" : "no");
        check("FL11 04 Factory: chain -> gate -> bridge pad", !idle.gateOpened && normal.cleared && normal.gateOpened &&
                                                              !early.cleared && r.len >= 30 && ramp.cleared, buf);
    }

    // ---------------- FL12. 記録: 使った磁石・連続磁気イベント・入力・リトライ、保存と読み込み ----------------
    {
        FlowWorld fw;
        // 04 磁気工場: H1・A・H2・扉の板・G と、A・鉄球・C の連鎖とゲートが続けて起きる
        const RunResult rr = runWorld(fw, G04, {1.0f, 4.0f, 4.55f});
        const int  used = fw.stats.magnetsUsed, combo = fw.stats.bestCombo, total = fw.magnetTotal;
        const bool inOk = fw.inputs.size() == 4 && fw.inputs[0] == 0.0f && std::fabs(fw.inputs[1] - 1.0f) < 0.02f &&
                          std::fabs(fw.inputs[3] - 4.55f) < 0.02f;
        const StageRecord rec = StageRecord::fromWorld(fw);
        fw.restart();
        const int r1 = fw.retries;
        fw.restart();                       // 遊ばずにもう一度: 数えない
        const int r2 = fw.retries;
        RecordBook  book;
        StageRecord slow = rec;
        slow.time += 1.0f;
        const bool  first = book.offer("G04", rec), worse = book.offer("G04", slow);
        const char* path  = "flow_test_records.tmp";
        bool ok = book.save(path);
        RecordBook other;
        ok = ok && other.load(path);
        std::remove(path);
        const StageRecord* got = other.get("G04");
        bool same = ok && got && std::fabs(got->time - rec.time) < 1e-4f && got->inputs.size() == rec.inputs.size() &&
                    got->trail.size() == rec.trail.size() && got->magnetsUsed == used && got->bestCombo == combo;
        Vec3 gp;
        same = same && rec.trail.size() > 10 && got->ghostAt(10.0f / 30.0f, gp) && length(gp - rec.trail[10]) < 1e-3f;
        std::snprintf(buf, sizeof buf, "%s; magnets used %d/%d, flow %d, inputs %s, retries %d then %d; best only %s, "
                      "save/load %s", rr.cleared ? "clear" : "FAIL", used, total, combo, inOk ? "ok" : "WRONG", r1, r2,
                      first && !worse ? "yes" : "NO", same ? "same" : "DIFFERENT");
        check("FL12 records: magnets, flow, inputs, retries, file", rr.cleared && used >= 4 && combo >= 5 && inOk && r1 == 1 &&
                                                                    r2 == 1 && first && !worse && same, buf);
    }

    // ======== ここから GRAND TOUR（全部のギミックを入れた大きなマップ）========

    // ---------------- FL13. 動的磁場（G4）: 往復する台が運ぶ、点滅する、半回転ずつ回る ----------------
    {
        // 往復する台（S が上）に N のプレイヤーを乗せる。半周期（3 s）で台は 10 m 先、プレイヤーも
        FlowWorld fw;
        fw.reset();
        RigidBody* sh = fw.addFixedMagnetBox({0, 0, 0}, {2.5f, 0.4f, 2.5f}, rotYTo({0, -1, 0}), 0.6f, 0.0f);
        sh->friction = 1.0f;
        Animator& a = fw.animate(sh);
        a.travel = {10, 0, 0};
        a.travelPeriod = 6.0f;
        fw.startPos = {0, 0.91f, 0};
        fw.finishBuild();
        fw.press();
        for (int i = 0; i < 360; ++i) fw.step(DT);
        const float carried = fw.player->position.x, shuttle = sh->position.x;
        // 点滅: 点いているときは押し、消えているときは力 0（プレイヤーは止めておく）
        auto fieldAt = [&](FlowWorld& w, float t) {
            while (w.stats.time < t - 1e-6f) {
                w.player->position = Vec3{0, 4, 0};
                w.player->velocity = Vec3{0, 0, 0};
                w.step(DT);
            }
            w.player->position = Vec3{0, 4, 0};
            float sc = 1.0f;
            return w.computePlayerForce(nullptr, &sc).y;
        };
        FlowWorld fp;
        fp.reset();
        Animator& p = fp.animate(fp.addFixedMagnetSphere({0, 0, 0}, 1.0f, {0, 1, 0}, 1.0f, 0.0f));
        p.pulsePeriod = 2.0f;
        p.pulseOn = 0.5f;
        fp.params.gravity = 0.0f;
        fp.startPos = {0, 4, 0};
        fp.finishBuild();
        fp.press();
        const float on = fieldAt(fp, 0.5f), off = fieldAt(fp, 1.5f);
        // 半回転ずつ回る（1 s ごと）: N が上 → S が上
        FlowWorld fs;
        fs.reset();
        Animator& r = fs.animate(fs.addFixedMagnetSphere({0, 0, 0}, 1.0f, {0, 1, 0}, 1.0f, 0.0f));
        r.spinAxis = {0, 0, 1};
        r.spinStep = 1.0f;
        fs.params.gravity = 0.0f;
        fs.startPos = {0, 4, 0};
        fs.finishBuild();
        fs.press();
        const float before = fieldAt(fs, 0.4f), after = fieldAt(fs, 1.4f);
        std::snprintf(buf, sizeof buf, "shuttle at x %.2f carries the player to %.2f; pulse on Fy %+.1f off %.1e; "
                      "spin step Fy %+.1f -> %+.1f", shuttle, carried, on, off, before, after);
        check("FL13 dynamic field: shuttle, pulse, half turns", std::fabs(shuttle - 10.0f) < 0.05f &&
                                                                std::fabs(carried - shuttle) < 1.0f && on > 1.0f &&
                                                                std::fabs(off) < 1e-6f && before > 1.0f && after < -1.0f, buf);
    }

    // ---------------- FL14. 磁気振り子（G6）: 速さを失わずにつながり、回るほど速くなり、回りすぎると振り飛ばされる ----------------
    {
        FlowWorld fw;
        fw.reset();
        Swing& s = fw.addSwing({0, 10, 0}, 0.8f, -1, 5.0f, {0, 0, 1});
        s.pump = 8.0f;
        s.maxHold = 8.0f;
        fw.startPos = {-8, 8, 0};
        fw.launchVelocity = {6, 2, 0};
        fw.killY = -40.0f;
        fw.finishBuild();
        fw.press();
        float vBefore = 0.0f, vAfter = 0.0f, bottom[8] = {}, snapT = -1.0f;
        int loops = 0;
        bool was = false;
        for (int i = 0; i < 2400 && fw.state == State::Running; ++i) {
            const float v0 = length(fw.player->velocity);
            fw.step(DT);
            const Swing& sw = fw.swings[0];
            if (!was && sw.engaged) { vBefore = v0; vAfter = length(fw.player->velocity); }
            if (sw.engaged) {
                loops = (int)(std::fabs(fw.swingAngle) / (2.0f * PHYS_PI));
                const float rel = fw.player->position.y - 10.0f;
                if (rel < -sw.rod + 0.3f) bottom[std::min(loops, 7)] = std::max(bottom[std::min(loops, 7)], length(fw.player->velocity));
            }
            if (was && !sw.engaged && sw.snapped && snapT < 0.0f) snapT = (i + 1) * DT;
            was = sw.engaged;
        }
        // 放すと、進んでいた向きに飛び出す（0.3 s 後の向きのずれ）
        float worst = 0.0f;
        for (float T = 1.6f; T <= 2.4f + 1e-6f; T += 0.2f) {
            FlowWorld f2;
            f2.reset();
            Swing& s2 = f2.addSwing({0, 10, 0}, 0.8f, -1, 5.0f, {0, 0, 1});
            s2.pump = 8.0f;
            s2.maxHold = 8.0f;
            f2.startPos = {-8, 8, 0};
            f2.launchVelocity = {6, 2, 0};
            f2.killY = -40.0f;
            f2.finishBuild();
            f2.press();
            Vec3 v0;
            bool sw = false;
            while (f2.stats.time < T + 0.3f && f2.state == State::Running) {
                if (!sw && f2.stats.time >= T - 1e-6f) { v0 = f2.player->velocity; f2.press(); sw = true; }
                f2.step(DT);
            }
            // 0.3 s の重力の分を戻して比べる
            const Vec3 v1 = f2.player->velocity + Vec3{0, G_ACC * 0.3f, 0};
            worst = std::max(worst, angleDeg(v0, v1));
        }
        std::snprintf(buf, sizeof buf, "capture %.2f -> %.2f m/s; bottom speed loop0 %.1f loop1 %.1f loop2 %.1f; %d loops, "
                      "flung off at %.2f s; release turn %.1f deg", vBefore, vAfter, bottom[0], bottom[1], bottom[2], loops,
                      snapT, worst);
        // 周回で速くなる（伸びは、振り飛ばされる速さに近づくほど小さくなる）
        check("FL14 pendulum: catch, spin up, fling off, let go", vAfter >= 0.95f * vBefore && bottom[1] > bottom[0] + 1.0f &&
                                                                  bottom[2] >= bottom[1] && loops >= 2 && snapT > 0.0f &&
                                                                  worst < 35.0f, buf);
    }

    // ---------------- FL15. 磁化連鎖（G5）: 触れて溜めてから切り替えると弾かれる。磁化した鋼はほかの鋼を引く ----------------
    {
        auto kick = [](float T) {
            FlowWorld fw;
            fw.reset();
            fw.addSolid({0, -1.5f, 0}, {6, 0.5f, 6});
            fw.addSteelBox({0, 0, 0}, {1, 0.5f, 1}, 0.0f, 0.0f);
            fw.startPos = {0, 1.01f, 0};
            fw.finishBuild();
            fw.press();
            float vmax = 0.0f;
            bool  sw = false;
            while (fw.stats.time < T + 1.5f) {
                if (!sw && fw.stats.time >= T - 1e-6f) { fw.press(); sw = true; }
                fw.step(DT);
                vmax = std::max(vmax, fw.player->velocity.y);
            }
            return vmax;
        };
        const float k0 = kick(0.1f), k1 = kick(0.6f);
        auto chainRun = [](bool touch) {
            FlowWorld fw;
            fw.reset();
            fw.addSolid({0, -0.5f, 0}, {10, 0.5f, 10}, Quat{}, 0.3f);
            fw.addSteelBox({0, 1.0f, 0}, {1, 1, 1}, 0.0f, momentForGamma(WORLD_GAMMA, 9.26f, 2.0f));
            fw.addSteelSphere({3.5f, 0.4f, 0}, 0.4f, 1.0f, momentForGamma(WORLD_GAMMA, 1.0f, 0.8f));
            fw.startPos = touch ? Vec3{-1.6f, 1.0f, 0} : Vec3{-9.0f, 0.5f, 6.0f};
            fw.finishBuild();
            fw.press();
            for (int i = 0; i < 240; ++i) {
                if (touch) fw.player->position = Vec3{-1.6f, 1.0f, 0};   // A の -x の面に触れたまま
                fw.step(DT);
            }
            return 3.5f - fw.steels[1].body->position.x;
        };
        const float pulled = chainRun(true), idle = chainRun(false);
        std::snprintf(buf, sizeof buf, "flip after 0.1 s: up %.1f m/s, after 0.6 s: up %.1f m/s; magnetized A pulls ball B %.2f m "
                      "(untouched %.2f m)", k0, k1, pulled, idle);
        check("FL15 steel: charge then flip, magnetized steel pulls", k0 < 1.0f && k1 > 8.0f && pulled > 0.8f &&
                                                                      std::fabs(idle) < 0.05f, buf);
    }

    // ---------------- FL16. 磁気トルク: 磁針は、プレイヤーと異極の端をプレイヤーへ向ける ----------------
    {
        FlowWorld fw;
        fw.reset();
        const Vec3  n0 = normalize(Vec3{-0.5f, 0, 0.866f});   // N 端の始めの向き（プレイヤーの側から 60 度ずれている）
        RigidBody* bar = fw.addFixedMagnetBox({0, 0, 0}, {0.5f, 8.0f, 1.5f}, rotYTo(n0), 0.0f, 0.0f);
        Rotor& r = fw.addRotor(bar, {0, 0, 0}, {0, 1, 0}, -10.0f, 10.0f, 0.24f);
        r.damping = 2.5f;
        fw.attachToRotor(r, fw.addPole(n0 * 8.6f, 1.0f, +1, 0.01f).body);
        fw.attachToRotor(r, fw.addPole(n0 * -8.6f, 1.0f, -1, 0.01f).body);
        fw.params.gravity = 0.0f;
        const Vec3 P{-10.0f, 0, 0};   // プレイヤー（止めておく）
        fw.startPos = P;
        fw.startPolarity = -1;        // S: N 端がこちらを向く
        fw.finishBuild();
        fw.press();
        auto runFor = [&](float secs) {
            for (int i = 0; i < (int)(secs * 120.0f); ++i) {
                fw.player->position = P;
                fw.player->velocity = Vec3{0, 0, 0};
                fw.step(DT);
            }
            const Vec3 nEnd = Quat::fromAxisAngle({0, 1, 0}, fw.rotors[0].angle).rotate(n0);
            return angleDeg(nEnd, P);
        };
        const float errS = runFor(6.0f);
        fw.polarity = +1;             // N に変えると、S 端がこちらを向く（N 端は向こう）
        const float errN = 180.0f - runFor(8.0f);
        std::snprintf(buf, sizeof buf, "S player: N end points at it within %.1f deg; N player: S end within %.1f deg", errS, errN);
        check("FL16 compass needle turns to the opposite pole", errS < 10.0f && errN < 10.0f, buf);
    }

    // ---------------- FL17. チェックポイント: 落ちると戻る（時間は続く）、手で戻る、リプレイで同じ走り ----------------
    {
        FlowWorld fw;
        loadStage(fw, GRAND_STAGE);
        const Vec3 seat0 = fw.pads[0].seat;
        fw.press();
        // 01 と同じ: 2.0 s で打ち上げ、早すぎる 2.38 s で切り替えて落ちる
        std::vector<float> sw{2.0f, 2.38f};
        size_t k = 0;
        float failT = -1.0f;
        while (fw.stats.time < 12.0f && fw.stats.falls == 0) {
            while (k < sw.size() && fw.state == State::Running && fw.stats.time >= sw[k] - 1e-6f) { fw.press(); ++k; }
            fw.step(DT);
            if (fw.state == State::Failed && failT < 0.0f) failT = fw.stats.time;
        }
        const bool backAtSeat = length(fw.player->position - seat0) < 1e-4f && fw.state == State::Running;
        const float tAfter = fw.stats.time;
        const int   breaks = (int)fw.trailBreaks.size();
        // 手で戻る（記録される）
        for (int i = 0; i < 60; ++i) fw.step(DT);
        fw.press();   // 発射
        for (int i = 0; i < 60; ++i) fw.step(DT);
        fw.resetToCheckpoint();
        const bool manualOk = fw.resets.size() == 1 && length(fw.player->position - seat0) < 1e-4f;
        // リプレイ: 同じ入力と手で戻った時刻を同じシミュレーションの時刻に流すと、同じ走りになる
        std::vector<float> in = fw.inputs, rs = fw.resets;
        for (int i = 0; i < 300; ++i) fw.step(DT);
        const Vec3 endA = fw.player->position;
        const float tA = fw.stats.time;
        FlowWorld rp;
        loadStage(rp, GRAND_STAGE);
        size_t ni = 0, nr = 0;
        while (rp.stats.time < tA - 1e-4f) {
            while (nr < rs.size() && rp.state == State::Running && rp.stats.time >= rs[nr] - 1e-4f) { rp.resetToCheckpoint(); ++nr; }
            while (ni < in.size() && (rp.state == State::Ready || (rp.state == State::Running && rp.stats.time >= in[ni] - 1e-4f))) {
                rp.press();
                ++ni;
            }
            rp.step(DT);
        }
        const bool replaySame = std::memcmp(&endA, &rp.player->position, sizeof endA) == 0;
        // 練習: 途中のチェックポイントから（記録しない）
        FlowWorld pr;
        loadStage(pr, GRAND_STAGE);
        pr.warpTo(4);
        const bool warpOk = pr.practice && pr.checkpoint == 4 && pr.state == State::Ready &&
                            length(pr.player->position - pr.pads[4].seat) < 1e-4f && pr.zone == 4;
        std::snprintf(buf, sizeof buf, "fell at %.2f s, back on the start pad %s at %.2f s (falls %d, trail breaks %d); manual "
                      "reset %s; replay %s; practice warp %s", failT, backAtSeat ? "yes" : "NO", tAfter, fw.stats.falls, breaks,
                      manualOk ? "ok" : "WRONG", replaySame ? "same" : "DIFFERENT", warpOk ? "ok" : "WRONG");
        check("FL17 checkpoints: respawn, manual reset, replay, practice", failT > 2.0f && backAtSeat && tAfter >= failT &&
                                                                           fw.stats.falls == 1 && breaks == 1 && manualOk &&
                                                                           replaySame && warpOk, buf);
    }

    // ---------------- FL18. GRAND TOUR を通しで: 9 区間を落ちずに抜ける。区間ごとの切替の窓 ----------------
    {
        FlowWorld fw;
        const GrandRun r = runGrand(fw, grandPlan());
        const StageRecord rec = StageRecord::fromWorld(fw);
        RecordBook book;
        book.offer("GRAND", rec);
        const char* path = "flow_test_grand.tmp";
        bool same = book.save(path);
        RecordBook other;
        same = same && other.load(path);
        std::remove(path);
        const StageRecord* got = other.get("GRAND");
        same = same && got && got->splits.size() == rec.splits.size() && got->inputs.size() == rec.inputs.size() &&
               std::fabs(got->splits.back() - rec.splits.back()) < 1e-4f;
        std::string arrive;
        for (float t : r.arrive) arrive += std::to_string((int)std::lround(t)) + " ";
        std::snprintf(buf, sizeof buf, "%s at %.2f s, falls %d, switches %d, magnets %d/%d, flow %d; checkpoints at %s; "
                      "save/load %s", r.cleared ? "CLEAR" : "NOT CLEARED", r.time, r.falls, fw.stats.switches,
                      fw.stats.magnetsUsed, fw.magnetTotal, fw.stats.bestCombo, arrive.c_str(), same ? "same" : "DIFFERENT");
        check("FL18 GRAND TOUR: the bot clears all 9 zones", r.cleared && r.falls == 0 && r.arrive.size() == 9 &&
                                                             fw.stats.magnetsUsed >= 15 && same, buf);
        // 通しの走りの中で測った窓（振り子の 2 周目、レンズ、磁気嵐、磁気コアの最後）
        struct W { int zone; float from, to; std::vector<float> before; const char* name; };
        const W ws[] = {{2, 3.9f, 4.4f, {}, "pendulum loop 2"},
                        {4, 0.4f, 1.0f, {}, "lens"},
                        {7, 4.5f, 5.0f, {}, "storm"},
                        {9, 11.7f, 12.15f, {2.76f, 4.6f, 9.0f}, "core finale"}};
        std::string widths;
        bool okW = true;
        for (const W& w : ws) {
            const std::string m = grandProbeMap(w.zone - 1, w.from, w.to, 0.02f, 3.0f, w.before);
            const int ms = longestRun(m, '#').len * 20;
            widths += std::string(w.name) + " " + std::to_string(ms) + " ms  ";
            okW = okW && ms >= 80;
        }
        check("FL18 GRAND TOUR windows in the full run >= 80 ms", okW, widths.c_str());
    }

    // ---------------- FL19. サイクロトロン: 一定のリズムで速くなり、出口から出る ----------------
    {
        FlowWorld fw;
        const LabRun r = runLab(fw, 0);
        // 隙間を通る間隔（リズム）と、通るたびに増える速さ
        float iMin = 1e9f, iMax = 0.0f, dMin = 1e9f, dMax = 0.0f;
        for (size_t i = 1; i < r.evT.size(); ++i) {
            iMin = std::min(iMin, r.evT[i] - r.evT[i - 1]);
            iMax = std::max(iMax, r.evT[i] - r.evT[i - 1]);
            dMin = std::min(dMin, r.evV[i] - r.evV[i - 1]);
            dMax = std::max(dMax, r.evV[i] - r.evV[i - 1]);
        }
        const float vFirst = r.evV.empty() ? 0.0f : r.evV.front(), vLast = r.evV.empty() ? 0.0f : r.evV.back();
        FlowWorld f2, f3;
        LabOpts none;
        none.phase = 0;
        none.maxT  = 20.0f;
        const LabRun idle = runLab(f2, 0, none);
        LabOpts skip;
        skip.skipK = 6;
        const LabRun miss = runLab(f3, 0, skip);
        int worst = 100000;
        std::string ws;
        for (int k = 0; k < r.flips; ++k) {
            const std::string m = labWindowMap(0, k, -0.3f, 0.3f, 0.03f, [](const LabRun& x) { return x.cleared; });
            const int ms = longestRun(m, '#').len * 30;
            worst = std::min(worst, ms);
            ws += std::to_string(ms) + " ";
        }
        std::snprintf(buf, sizeof buf, "clear %.2f s, %d flips; speed %.1f -> %.1f (+%.2f..%.2f per gap, in the halves %.2f%%); "
                      "beat %.2f..%.2f s; no input %s, miss flip 6 %s; windows %sms",
                      r.time, r.flips, vFirst, vLast, dMin, dMax, 100.0f * r.dvDee, iMin, iMax,
                      idle.cleared ? "CLEAR" : "no clear", miss.cleared ? "CLEAR" : "no clear", ws.c_str());
        check("FL19 cyclotron: same beat, faster, out the port",
              r.cleared && r.dvDee < 0.01f && vLast >= 2.0f * vFirst && iMax <= 1.25f * iMin && dMin > 1.5f && dMax < 2.1f &&
              !idle.cleared && !miss.cleared && worst >= 200, buf);
    }

    // ---------------- FL20. ガウス加速リング: くぐるたびに速くなり、ビートが詰まる ----------------
    {
        FlowWorld fw;
        const LabRun r = runLab(fw, 1);
        bool faster = r.evV.size() == fw.rings.size(), tighter = true;
        for (size_t i = 1; i < r.evV.size(); ++i) faster = faster && r.evV[i] > r.evV[i - 1] + 1.0f;
        for (size_t i = 2; i < r.evT.size(); ++i) tighter = tighter && r.evT[i] - r.evT[i - 1] < r.evT[i - 1] - r.evT[i - 2];
        FlowWorld f2;
        LabOpts skip;
        skip.skipK = 2;
        const LabRun miss = runLab(f2, 1, skip);
        // すべての切替を同じだけずらしたときにクリアできる範囲
        std::string m;
        for (float d = -0.15f; d <= 0.15f + 1e-6f; d += 0.01f) {
            FlowWorld f3;
            LabOpts o;
            o.delta = d;
            m += runLab(f3, 1, o).cleared ? '#' : '.';
        }
        const int allMs = longestRun(m, '#').len * 10;
        std::snprintf(buf, sizeof buf, "clear %.2f s; speed after each ring %.1f %.1f %.1f %.1f %.1f %.1f; beat %.2f -> %.2f s; "
                      "miss ring 3 %s; all flips shifted -150..150 ms %s (%d ms)",
                      r.time, r.evV.size() > 0 ? r.evV[0] : 0.0f, r.evV.size() > 1 ? r.evV[1] : 0.0f, r.evV.size() > 2 ? r.evV[2] : 0.0f,
                      r.evV.size() > 3 ? r.evV[3] : 0.0f, r.evV.size() > 4 ? r.evV[4] : 0.0f, r.evV.size() > 5 ? r.evV[5] : 0.0f,
                      r.evT.size() > 1 ? r.evT[1] - r.evT[0] : 0.0f, r.evT.size() > 5 ? r.evT[5] - r.evT[4] : 0.0f,
                      miss.cleared ? "CLEAR" : "no clear", m.c_str(), allMs);
        check("FL20 gauss rings: faster each ring, tighter beat", r.cleared && faster && tighter && r.evV.back() >= 20.0f &&
                                                                  !miss.cleared && allMs >= 80, buf);
    }

    // ---------------- FL21. 共振ブランコ: 合った位相で揺れが育って壁を壊し、台が発射台になる ----------------
    {
        FlowWorld fw;
        const LabRun r = runLab(fw, 2);
        bool grows = r.evV.size() >= 3;
        for (size_t i = 1; i < r.evV.size() && r.evT[i] <= r.breakT + 1e-4f; ++i) grows = grows && r.evV[i] > r.evV[i - 1] + 5.0f;
        const RigidBody* bob = fw.rotors[0].attached.empty() ? nullptr : fw.rotors[0].attached[0].body;
        const bool spent = bob && fw.charge[bob->id] == 0.0f;
        FlowWorld f2, f3;
        LabOpts anti;
        anti.phase = -1;
        anti.maxT  = 20.0f;
        const LabRun ra = runLab(f2, 2, anti);
        float antiMax = 0.0f;
        for (float v : ra.evV) antiMax = std::max(antiMax, v);
        LabOpts none;
        none.phase = 0;
        none.maxT  = 20.0f;
        const LabRun rn = runLab(f3, 2, none);
        const bool decays = rn.evV.size() >= 3 && rn.evV.back() < 0.5f * rn.evV.front();
        std::string peaks;
        for (float v : r.evV) peaks += std::to_string((int)std::lround(v)) + " ";
        std::snprintf(buf, sizeof buf, "peaks %sdeg, wall broke at %.2f s, launched before %s, bob %s, clear %.2f s; "
                      "anti-phase max %.0f deg (break %s); no input %.0f -> %.0f deg",
                      peaks.c_str(), r.breakT, r.firedEarly ? "YES" : "no", spent ? "spent" : "STILL MAGNETIC", r.time, antiMax,
                      ra.breakT > 0.0f ? "YES" : "no", rn.evV.empty() ? 0.0f : rn.evV.front(), rn.evV.empty() ? 0.0f : rn.evV.back());
        check("FL21 resonance swing: in phase grows, breaks the wall", r.cleared && grows && r.breakT > 0.0f && r.breakT <= 10.0f &&
                                                                      !r.firedEarly && spent && ra.breakT < 0.0f && antiMax <= 27.0f &&
                                                                      decays, buf);
    }

    // ---------------- FL22. 鉄球のロープ: 鎖は静かに垂れ、つかんで振れ、切り替えて渡る ----------------
    {
        // 何もしないと鎖が静かに垂れている（開始の台の上で時間だけ進める。最初の 1 s で落ち着いた後のずれ）
        FlowWorld fi;
        loadStage(fi, LAB_STAGE + 3);
        fi.player->position = Vec3{-60.0f, 10.0f, 0};   // プレイヤーの磁力が届かない所で待たせる
        fi.addSolid(Vec3{-60.0f, 9.0f, 0}, {1.0f, 0.5f, 1.0f});
        fi.begin();
        std::vector<Vec3> p1;
        float idleMove = 0.0f;
        for (int i = 0; i < 600; ++i) {
            fi.step(DT);
            if (i == 119)
                for (const BallChain& ch : fi.chains)
                    for (const RigidBody* b : ch.balls) p1.push_back(b->position);
            if (i > 119) {
                size_t k = 0;
                for (const BallChain& ch : fi.chains)
                    for (const RigidBody* b : ch.balls) idleMove = std::max(idleMove, length(b->position - p1[k++]));
            }
        }
        FlowWorld fw;
        const LabRun r = runLab(fw, 3);
        const bool order = r.evV.size() == 3 && r.evV[0] == 0.0f && r.evV[1] == 1.0f && r.evV[2] == 2.0f;
        FlowWorld f2;
        LabOpts none;
        none.phase = 0;
        none.maxT  = 12.0f;
        const LabRun rn = runLab(f2, 3, none);
        int worst = 100000;
        std::string ws;
        for (int k = 0; k < (int)LAB_CHAIN_FLIPS.size(); ++k) {
            const std::string m = labWindowMap(3, k, -0.8f, 0.8f, 0.05f, [](const LabRun& x) { return x.cleared; });
            const int ms = longestRun(m, '#').len * 50;
            worst = std::min(worst, ms);
            ws += std::to_string(ms) + " ";
        }
        std::snprintf(buf, sizeof buf, "idle drift %.3f m; clear %.2f s, grabbed chains %s at %.2f %.2f %.2f s; max gap %.3f m; "
                      "no flips %s; windows %sms",
                      idleMove, r.time, order ? "1-2-3" : "OUT OF ORDER", r.evT.size() > 0 ? r.evT[0] : 0.0f,
                      r.evT.size() > 1 ? r.evT[1] : 0.0f, r.evT.size() > 2 ? r.evT[2] : 0.0f, r.maxGap,
                      rn.cleared ? "CLEAR" : "no clear", ws.c_str());
        check("FL22 ball chain: hang, grab, swing, let go, next", idleMove <= 0.05f && r.cleared && order && r.maxGap < 0.05f &&
                                                                  !rn.cleared && worst >= 300, buf);
    }

    std::printf("\n%s (%d failures)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}
