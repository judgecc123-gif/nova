#define WIN32_LEAN_AND_MEAN  // 排除绝大部分无用头文件
#define NOMINMAX             // 严禁 windows.h 定义 min/max 宏
#include "render.hpp"

#include <windows.h>

#include "input.hpp"

namespace nova::rnd
{
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    Window *app = nullptr;

    if (msg == WM_NCCREATE) {
        // 1. 从创建参数中提取 this 指针 (lpCreateParams)
        auto cs = reinterpret_cast<CREATESTRUCTW *>(lParam);
        app     = reinterpret_cast<Window *>(cs->lpCreateParams);
        // 2. 将指针存入窗口的 USERDATA 区域
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
    } else {
        // 3. 后续消息直接从 USERDATA 取出指针
        app = reinterpret_cast<Window *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    }

    if (app) app->DispatchMessage(msg, wParam, lParam);

    if (msg == WM_DESTROY) {
        if (app) {
            // 此时窗口由用户关闭

            app->hwnd = nullptr;
            app->callbacks.clear();
            app->next_callback_id_ = 0;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
        }
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

Window::Window(std::wstring_view title, uint32_t width, uint32_t height)
{
    // 注册窗口类
    WNDCLASSEXW wc{sizeof(WNDCLASSEXW)};
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = GetModuleHandleW(nullptr);
    wc.lpszClassName = title.data();

    if (!RegisterClassExW(&wc)) {
        throw std::runtime_error("Failed to register window class.");
    }
    hwnd = CreateWindowExW(0, title.data(), title.data(), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                           CW_USEDEFAULT, width, height, nullptr, nullptr,
                           GetModuleHandleW(nullptr), this);

    if (!hwnd) {
        throw std::runtime_error("Failed to create window.");
    }
}

Window::~Window()
{
    if (hwnd) {
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);

        this->DispatchMessage(WM_DESTROY, 0, 0);
        callbacks.clear();

        DestroyWindow(hwnd);
        hwnd = nullptr;
    }
}

void Window::Show() { ShowWindow(hwnd, SW_SHOW); }

bool Window::IsAlive() const { return hwnd != nullptr; }

HWND Window::GetRawHandle() const { return hwnd; }

Vector2i Window::GetSize() const
{
    if (!hwnd) return {};
    RECT rc;
    GetClientRect(hwnd, &rc);
    return {rc.right - rc.left, rc.bottom - rc.top};
}

void Window::DispatchMessage(uint32_t msg, WPARAM wParam, LPARAM lParam)
{
    if (callbacks.contains(msg)) {
        for (auto &[func, id] : callbacks.at(msg)) {
            if (IsValidCallback(id)) func->OnMessage(*this, wParam, lParam);
        }
    }
}

void Window::ProcessMessage()
{
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) return;
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
}

void Input::Bind(Window *window)
{
    // 解绑之前的窗口和回调
    this->Unbind();

    bound_window_ = window;

    RAWINPUTDEVICE Rid[2];

    Rid[0].usUsagePage = 0x01;  // HID_USAGE_PAGE_GENERIC
    Rid[0].usUsage     = 0x02;  // HID_USAGE_GENERIC_MOUSE
    Rid[0].dwFlags     = 0;     // adds mouse and also ignores legacy mouse messages
    Rid[0].hwndTarget  = window ? window->GetRawHandle() : nullptr;

    Rid[1].usUsagePage = 0x01;  // HID_USAGE_PAGE_GENERIC
    Rid[1].usUsage     = 0x06;  // HID_USAGE_GENERIC_KEYBOARD
    Rid[1].dwFlags     = 0;     // adds keyboard and also ignores legacy keyboard messages
    Rid[1].hwndTarget  = window ? window->GetRawHandle() : nullptr;

    if (!RegisterRawInputDevices(Rid, 2, sizeof(Rid[0]))) {
        throw std::runtime_error("Failed to register raw input devices.");
    }

    if (!window) return;
    callback_id_[0] =
        window->Register(WM_INPUT, [this](Window &window, WPARAM wParam, LPARAM lParam) {
            UINT dwSize;
            // 1. 先获取数据大小
            GetRawInputData((HRAWINPUT)lParam, RID_INPUT, NULL, &dwSize, sizeof(RAWINPUTHEADER));

            auto lpb = std::make_unique<BYTE[]>(dwSize);

            // 2. 获取实际数据
            if (GetRawInputData((HRAWINPUT)lParam, RID_INPUT, lpb.get(), &dwSize,
                                sizeof(RAWINPUTHEADER)) != dwSize) {
                return 0;
            }

            RAWINPUT const *raw = reinterpret_cast<RAWINPUT *>(lpb.get());

            // 3. 根据类型解析
            if (raw->header.dwType == RIM_TYPEMOUSE) {
                auto &mouse = raw->data.mouse;

                if (mouse.usFlags & MOUSE_MOVE_RELATIVE) {
                    mouse_delta_.x += mouse.lLastX;
                    mouse_delta_.y += mouse.lLastY;
                } else if (mouse.usFlags & MOUSE_MOVE_ABSOLUTE) {
                    mouse_position_.x = static_cast<float>(mouse.lLastX);
                    mouse_position_.y = static_cast<float>(mouse.lLastY);
                }

                if (mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_DOWN) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseLeft);
                    key_state_[vkey] = 1;
                } else if (mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_UP) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseLeft);
                    key_state_[vkey] = 0;
                }

                if (mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_DOWN) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseRight);
                    key_state_[vkey] = 1;
                } else if (mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_UP) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseRight);
                    key_state_[vkey] = 0;
                }

                if (mouse.usButtonFlags & RI_MOUSE_MIDDLE_BUTTON_DOWN) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseMiddle);
                    key_state_[vkey] = 1;
                } else if (mouse.usButtonFlags & RI_MOUSE_MIDDLE_BUTTON_UP) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseMiddle);
                    key_state_[vkey] = 0;
                }

                if (mouse.usButtonFlags & RI_MOUSE_BUTTON_4_DOWN) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseX1);
                    key_state_[vkey] = 1;
                } else if (mouse.usButtonFlags & RI_MOUSE_BUTTON_4_UP) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseX1);
                    key_state_[vkey] = 0;
                }

                if (mouse.usButtonFlags & RI_MOUSE_BUTTON_5_DOWN) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseX2);
                    key_state_[vkey] = 1;
                } else if (mouse.usButtonFlags & RI_MOUSE_BUTTON_5_UP) {
                    size_t vkey      = static_cast<size_t>(KeyCode::MouseX2);
                    key_state_[vkey] = 0;
                }

                if (mouse.usButtonFlags & RI_MOUSE_WHEEL) {
                    mouse_wheel_delta_ +=
                        static_cast<short>(mouse.usButtonData) / 120.f;  // 每120单位为一个滚轮刻度
                }

            } else if (raw->header.dwType == RIM_TYPEKEYBOARD) {
                uint32_t msg = raw->data.keyboard.Message;  // WM_KEYDOWN 或 WM_KEYUP
                size_t vkey  = raw->data.keyboard.VKey;
                // std::print("Key {}: {:02b}", (int)vkey, key_state_[vkey]);
                if (msg == WM_KEYDOWN) {
                    key_state_[vkey] = 1;
                } else if (msg == WM_KEYUP) {
                    key_state_[vkey] = 0;
                }

                // std::println(" -> {:02b}", key_state_[vkey]);
            }

            return 0;
        });

    callback_id_[1] = window->Register(
        WM_DESTROY, [this](Window &window, WPARAM wParam, LPARAM lParam) { this->Unbind(); });
}

void Input::Unbind()
{
    if (bound_window_) {
        for (auto &id : callback_id_) {
            bound_window_->Unregister(id);
            id = 0;
        }
        // 注销 Raw Input 设备
        RAWINPUTDEVICE Rid[2];
        Rid[0].usUsagePage = 0x01;          // HID_USAGE_PAGE_GENERIC
        Rid[0].usUsage     = 0x02;          // HID_USAGE_GENERIC_MOUSE
        Rid[0].dwFlags     = RIDEV_REMOVE;  // 移除鼠标
        Rid[0].hwndTarget  = 0;
        Rid[1].usUsagePage = 0x01;          // HID_USAGE_PAGE_GENERIC
        Rid[1].usUsage     = 0x06;          // HID_USAGE_GENERIC_KEYBOARD
        Rid[1].dwFlags     = RIDEV_REMOVE;  // 移除键盘
        Rid[1].hwndTarget  = 0;
        if (!RegisterRawInputDevices(Rid, 2, sizeof(Rid[0]))) {
            throw std::runtime_error("Failed to unregister raw input devices.");
        }
    }

    key_state_.reset();
    key_state_prev_.reset();

    mouse_position_    = {};
    mouse_delta_       = {};
    mouse_wheel_delta_ = 0.0f;
    bound_window_      = nullptr;
}

Input::~Input() noexcept { Unbind(); }

}  // namespace nova::rnd