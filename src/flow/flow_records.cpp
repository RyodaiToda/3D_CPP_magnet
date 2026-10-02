// ---------------------------------------------------------------------------
// flow_records.cpp : 最高記録の保存と読み込み（flow_records.h）
//   形式（1 行 1 項目）:
//     stage <key>
//     time <s> / switches <n> / maxspeed <m/s> / combo <n> / magnets <n> / retries <n>
//     inputs <t0> <t1> ...
//     trail <x y z> <x y z> ...
//     falls <n> / splits <t> ... / resets <t> ... / breaks <i> ...（大きなマップ。ないときは書かない）
//     end
// ---------------------------------------------------------------------------
#include "flow/flow_records.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace flow {

StageRecord StageRecord::fromWorld(const FlowWorld& fw) {
    StageRecord r;
    r.time        = fw.stats.time;
    r.switches    = fw.stats.switches;
    r.maxSpeed    = fw.stats.maxSpeed;
    r.bestCombo   = fw.stats.bestCombo;
    r.magnetsUsed = fw.stats.magnetsUsed;
    r.retries     = fw.retries;
    r.inputs      = fw.inputs;
    r.trail       = fw.trail;
    r.falls       = fw.stats.falls;
    r.splits      = fw.splits;
    r.resets      = fw.resets;
    r.breaks      = fw.trailBreaks;
    return r;
}

bool StageRecord::ghostAt(float t, Vec3& out) const {
    if (trail.empty()) return false;
    const float f = std::max(0.0f, t) * 30.0f;   // 1/30 s ごと
    const int   i = (int)f;
    if (i + 1 >= (int)trail.size()) { out = trail.back(); return true; }
    const float a = f - (float)i;
    out = trail[i] + (trail[i + 1] - trail[i]) * a;
    return true;
}

bool RecordBook::offer(const std::string& key, const StageRecord& r) {
    auto it = best_.find(key);
    if (it != best_.end() && it->second.time <= r.time) return false;
    best_[key] = r;
    return true;
}

const StageRecord* RecordBook::get(const std::string& key) const {
    auto it = best_.find(key);
    return it == best_.end() ? nullptr : &it->second;
}

bool RecordBook::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << "# MAGNET FLOW records v1\n";
    for (const auto& [key, r] : best_) {
        // 時刻は float がそのまま戻る桁数で書く（リプレイが同じステップで押す）。軌跡は 6 桁
        f.precision(9);
        f << "stage " << key << "\n";
        f << "time " << r.time << "\nswitches " << r.switches << "\nmaxspeed " << r.maxSpeed << "\ncombo "
          << r.bestCombo << "\nmagnets " << r.magnetsUsed << "\nretries " << r.retries << "\n";
        f << "inputs";
        for (float t : r.inputs) f << ' ' << t;
        f << "\ntrail";
        f.precision(6);
        for (const Vec3& p : r.trail) f << ' ' << p.x << ' ' << p.y << ' ' << p.z;
        f << "\n";
        if (!r.splits.empty()) {
            f.precision(9);
            f << "falls " << r.falls << "\nsplits";
            for (float t : r.splits) f << ' ' << t;
            f << "\nresets";
            for (float t : r.resets) f << ' ' << t;
            f << "\nbreaks";
            for (int i : r.breaks) f << ' ' << i;
            f << "\n";
        }
        f << "end\n";
    }
    return (bool)f;
}

bool RecordBook::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line, key;
    StageRecord r;
    bool in = false;
    while (std::getline(f, line)) {
        std::istringstream s(line);
        std::string tag;
        if (!(s >> tag) || tag[0] == '#') continue;
        if (tag == "stage")         { s >> key; r = StageRecord{}; in = true; }
        else if (!in)               continue;
        else if (tag == "time")     s >> r.time;
        else if (tag == "switches") s >> r.switches;
        else if (tag == "maxspeed") s >> r.maxSpeed;
        else if (tag == "combo")    s >> r.bestCombo;
        else if (tag == "magnets")  s >> r.magnetsUsed;
        else if (tag == "retries")  s >> r.retries;
        else if (tag == "inputs")   { float t; while (s >> t) r.inputs.push_back(t); }
        else if (tag == "trail")    { Vec3 p; while (s >> p.x >> p.y >> p.z) r.trail.push_back(p); }
        else if (tag == "falls")    s >> r.falls;
        else if (tag == "splits")   { float t; while (s >> t) r.splits.push_back(t); }
        else if (tag == "resets")   { float t; while (s >> t) r.resets.push_back(t); }
        else if (tag == "breaks")   { int i; while (s >> i) r.breaks.push_back(i); }
        else if (tag == "end")      { if (r.time > 0.0f && std::isfinite(r.time)) best_[key] = r; in = false; }
    }
    return true;
}

} // namespace flow
