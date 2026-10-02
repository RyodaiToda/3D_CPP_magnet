#pragma once
// ---------------------------------------------------------------------------
// hud_lines.h : HUD の行（足していき、最後にまとめて背景つきで描く）
//   デモ（main.cpp, fun_scenes）とゲーム（flow_main.cpp）で共有する
// ---------------------------------------------------------------------------
#include "raylib.h"

#include <algorithm>
#include <string>
#include <vector>

struct HudLines {
    struct Line { std::string text; Color color; int size; int gapBefore; };
    std::vector<Line> lines;
    int pendingGap = 0;

    void add(const char* text, Color color = LIGHTGRAY, int size = 16) {
        lines.push_back({text, color, size, pendingGap});
        pendingGap = 0;
    }
    void gap(int px = 8) { pendingGap += px; }

    void draw(int x, int y, int minWidth) const {
        int h = 12, w = minWidth;
        for (const Line& l : lines) {
            h += l.gapBefore + l.size + 4;
            w = std::max(w, MeasureText(l.text.c_str(), l.size) + 24);
        }
        DrawRectangle(x, y, w, h, Fade(BLACK, 0.55f));
        int yy = y + 8;
        for (const Line& l : lines) {
            yy += l.gapBefore;
            DrawText(l.text.c_str(), x + 12, yy, l.size, l.color);
            yy += l.size + 4;
        }
    }
};
