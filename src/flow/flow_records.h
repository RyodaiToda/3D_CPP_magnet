#pragma once
// ---------------------------------------------------------------------------
// flow_records.h : ステージごとの最高記録（raylib に依存しない）
//   仕様 magnetic_flow_gimmick_stage_spec.md §9: クリアタイム、最大速度、連続磁気イベント数、
//   使用磁石数、リトライ回数を記録する。入力の履歴（押した時刻）と軌跡も残し、ゴーストに使う。
//   テキストファイルに保存する（ステージは stageKey で区別する）
// ---------------------------------------------------------------------------
#include "flow/flow_world.h"

#include <map>
#include <string>
#include <vector>

namespace flow {

struct StageRecord {
    float time        = 0.0f;
    int   switches    = 0;
    float maxSpeed    = 0.0f;
    int   bestCombo   = 0;
    int   magnetsUsed = 0;
    int   retries     = 0;
    std::vector<float> inputs;   // 押した時刻（開始 = 0 を含む）
    std::vector<Vec3>  trail;    // 1/30 s ごとの位置（ゴースト）
    // 大きなマップ
    int   falls = 0;             // 落ちてチェックポイントに戻った回数
    std::vector<float> splits;   // チェックポイントに着いた時刻
    std::vector<float> resets;   // 手でチェックポイントに戻った時刻（リプレイで同じ時刻に戻す）
    std::vector<int>   breaks;   // 軌跡を切るところ（チェックポイントに戻った）

    static StageRecord fromWorld(const FlowWorld& fw);
    // 開始から t 秒のゴーストの位置（軌跡の間をつなぐ）。軌跡がなければ false
    bool ghostAt(float t, Vec3& out) const;
};

class RecordBook {
public:
    // 速ければ置き換えて true
    bool offer(const std::string& key, const StageRecord& r);
    const StageRecord* get(const std::string& key) const;
    bool load(const std::string& path);
    bool save(const std::string& path) const;
    void clear() { best_.clear(); }

private:
    std::map<std::string, StageRecord> best_;
};

} // namespace flow
