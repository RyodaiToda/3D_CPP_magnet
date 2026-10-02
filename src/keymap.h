// ---------------------------------------------------------------------------
// keymap.h : キーボード配列に合わせた記号キー
//
// raylib（GLFW）はキーを US 配列の物理位置（スキャンコード）で決める。JIS 配列では
// 「[」と刻印されたキーが KEY_RIGHT_BRACKET、「]」が KEY_BACKSLASH になるので、
// IsKeyPressed(KEY_LEFT_BRACKET) は「@」のキーに反応する。ここでは今の配列で
// その文字を打つ物理キーの raylib のキー番号を返す（Windows 以外は US 配列のまま）。
// ---------------------------------------------------------------------------
#pragma once

namespace demo {

// 文字 c（'[' ']' '-' '=' ';' '\'' ',' '.' '/' '`' '\\'）を打つ物理キーの raylib のキー番号。見つからなければ fallback
int keyForChar(char c, int fallback);

// よく使う 2 つ（最初の呼び出しで配列を調べ、あとは覚えておく）
int keyLeftBracket();    // '['（強さを下げる、前へ）
int keyRightBracket();   // ']'（強さを上げる、次へ）

} // namespace demo
