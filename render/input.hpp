#pragma once

#include <bitset>

#include "common.hpp"

namespace nova::rnd
{

/**
 * @brief 统一键码枚举
 * 使用 uint16_t 节省空间，enum class 保证类型安全
 */
enum class KeyCode : uint16_t
{
    None = 0,

    // --- 鼠标 ---
    MouseLeft   = 1,  // 0x01
    MouseRight  = 2,  // 0x02
    MouseMiddle = 4,  // 0x04
    MouseX1     = 5,  // 0x05
    MouseX2     = 6,  // 0x06

    // --- 控制键 ---
    Backspace = 8,   // 0x08
    Tab       = 9,   // 0x09
    Clear     = 12,  // 0x0C
    Return    = 13,  // 0x0D
    Pause     = 19,  // 0x13
    Escape    = 27,  // 0x1B
    Space     = 32,  // 0x20

    // --- 编辑/导航键 ---
    PageUp   = 33,  // 0x21
    PageDown = 34,  // 0x22
    End      = 35,  // 0x23
    Home     = 36,  // 0x24
    Left     = 37,  // 0x25
    Up       = 38,  // 0x26
    Right    = 39,  // 0x27
    Down     = 40,  // 0x28
    Insert   = 45,  // 0x2D
    Delete   = 46,  // 0x2E

    // --- 数字键 (主键盘) ---
    Alpha0 = 48,
    Alpha1,
    Alpha2,
    Alpha3,
    Alpha4,
    Alpha5,
    Alpha6,
    Alpha7,
    Alpha8,
    Alpha9,

    // --- 字母键 ---
    A = 65,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,

    // --- 小键盘 (Numpad) ---
    Numpad0 = 96,  // 0x60
    Numpad1,
    Numpad2,
    Numpad3,
    Numpad4,
    Numpad5,
    Numpad6,
    Numpad7,
    Numpad8,
    Numpad9,

    NumpadMultiply  = 106,  // 0x6A
    NumpadAdd       = 107,  // 0x6B
    NumpadSeparator = 108,  // 0x6C
    NumpadSubtract  = 109,  // 0x6D
    NumpadDecimal   = 110,  // 0x6E
    NumpadDivide    = 111,  // 0x6F

    // --- 功能键 ---
    F1 = 112,  // 0x70
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,

    // --- 修饰键 ---
    LeftShift    = 160,  // 0xA0
    RightShift   = 161,  // 0xA1
    LeftControl  = 162,  // 0xA2
    RightControl = 163,  // 0xA3
    LeftAlt      = 164,  // 0xA4
    RightAlt     = 165,  // 0xA5

    // --- 符号键 ---
    Semicolon    = 186,  // 0xBA: ; :
    Plus         = 187,  // 0xBB: = +
    Comma        = 188,  // 0xBC: , <
    Minus        = 189,  // 0xBD: - _
    Period       = 190,  // 0xBE: . >
    Slash        = 191,  // 0xBF: / ?
    BackQuote    = 192,  // 0xC0: ` ~
    LeftBracket  = 219,  // 0xDB: [ {
    BackSlash    = 220,  // 0xDC: \ |
    RightBracket = 221,  // 0xDD: ] }
    Quote        = 222,  // 0xDE: ' "

    MaxCount = 255
};

class Window;

class Input : public Singleton<Input>
{
    friend class Singleton<Input>;

    Input() noexcept = default;

    std::bitset<static_cast<size_t>(KeyCode::MaxCount) + 1> key_state_,
        key_state_prev_;  // 存储按键状态，true表示按下，false表示松开

    Vector2f mouse_position_{};
    Vector2f mouse_delta_{};
    float mouse_wheel_delta_{};

    uint64_t callback_id_[2]{};
    Window* bound_window_{};

public:
    ~Input() noexcept;

    void RecordPreviousState()
    {
        key_state_prev_    = key_state_;
        mouse_delta_       = {};
        mouse_wheel_delta_ = 0;
    }

    void Unbind();

    void Bind(Window* window = nullptr);

    Vector2f GetMousePosition() const { return mouse_position_; }
    Vector2f GetMouseDelta() const { return mouse_delta_; }
    float GetMouseWheelDelta() const { return mouse_wheel_delta_; }

    bool WasPressed(KeyCode key) const
    {
        return key_state_[static_cast<size_t>(key)] &&
               !key_state_prev_[static_cast<size_t>(key)];  // 当前按下且上次未按下
    }
    bool IsPressed(KeyCode key) const { return key_state_[static_cast<size_t>(key)]; }  // 当前按下
    bool WasReleased(KeyCode key) const
    {
        return !key_state_[static_cast<size_t>(key)] &&
               key_state_prev_[static_cast<size_t>(key)];  // 当前未按下且上次按下
    }
    bool IsReleased(KeyCode key) const { return !key_state_[static_cast<size_t>(key)]; }
};
}  // namespace nova::rnd