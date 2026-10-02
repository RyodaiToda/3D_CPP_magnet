// ---------------------------------------------------------------------------
// keymap.cpp : キーボード配列に合わせた記号キー（keymap.h）
//   raylib.h と windows.h は名前がぶつかるので、このファイルは raylib を含めない。
// ---------------------------------------------------------------------------
#include "keymap.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace demo {

namespace {

// US 配列のスキャンコード → GLFW（raylib）のキー番号。記号のキーだけ
struct ScanKey { int scancode, key; };
constexpr ScanKey SCAN_KEYS[] = {
    {0x29, 96},   // ` KEY_GRAVE
    {0x0C, 45},   // - KEY_MINUS
    {0x0D, 61},   // = KEY_EQUAL
    {0x1A, 91},   // [ KEY_LEFT_BRACKET
    {0x1B, 93},   // ] KEY_RIGHT_BRACKET
    {0x2B, 92},   // \ KEY_BACKSLASH
    {0x27, 59},   // ; KEY_SEMICOLON
    {0x28, 39},   // ' KEY_APOSTROPHE
    {0x33, 44},   // , KEY_COMMA
    {0x34, 46},   // . KEY_PERIOD
    {0x35, 47},   // / KEY_SLASH
};

} // namespace

int keyForChar(char c, int fallback) {
#ifdef _WIN32
    const HKL layout = GetKeyboardLayout(0);
    for (const ScanKey& sk : SCAN_KEYS) {
        const UINT vk = MapVirtualKeyExW((UINT)sk.scancode, MAPVK_VSC_TO_VK_EX, layout);
        if (vk == 0) continue;
        const UINT ch = MapVirtualKeyExW(vk, MAPVK_VK_TO_CHAR, layout) & 0xFFFFu;   // シフトなしの文字（デッドキーの印は落とす）
        if (ch == (UINT)(unsigned char)c) return sk.key;
    }
    return fallback;
#else
    (void)c;
    return fallback;
#endif
}

int keyLeftBracket() {
    static const int key = keyForChar('[', 91);
    return key;
}

int keyRightBracket() {
    static const int key = keyForChar(']', 93);
    return key;
}

} // namespace demo
