#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define NODRAWTEXT

#include <d2d1_3.h>
#include <dwrite.h>
#include <winrt/base.h>

#include <memory>
#include <print>
#include <stdexcept>
#include <string_view>

#include "common.hpp"

namespace nova::rnd
{
using namespace winrt;

using Color = D2D1::ColorF;

class Renderer
{
    com_ptr<ID2D1Factory> m_factory;
    com_ptr<ID2D1HwndRenderTarget> m_target;
    com_ptr<ID2D1SolidColorBrush> m_brush;
    com_ptr<IDWriteFactory> m_writeFactory;
    com_ptr<IDWriteTextFormat> m_textFormat;

    float m_strokeWidth{1.0f};

public:
    Renderer() = default;
    Renderer(HWND hwnd) { Initialize(hwnd); }

    void Initialize(HWND hwnd)
    {
        // 1. 创建工厂 (单线程模式)
        check_hresult(D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, m_factory.put()));

        // 4. 获取窗口大小
        RECT rc;
        GetClientRect(hwnd, &rc);

        // 5. 直接创建渲染目标 (绑定到窗口)
        check_hresult(m_factory->CreateHwndRenderTarget(
            D2D1::RenderTargetProperties(),
            D2D1::HwndRenderTargetProperties(hwnd, D2D1::SizeU(rc.right, rc.bottom)),
            m_target.put()));

        // 6. 创建一把默认颜色刷子
        check_hresult(m_target->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::CornflowerBlue),
                                                      m_brush.put()));
    }

    void InitializeText(const wchar_t *font_family = L"Arial", float font_size = 16.0f)
    {
        // 2. 创建DirectWrite工厂
        check_hresult(DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory),
                                          reinterpret_cast<IUnknown **>(m_writeFactory.put())));

        // 3. 创建默认文本格式
        check_hresult(m_writeFactory->CreateTextFormat(
            font_family, nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL, font_size, L"en-us", m_textFormat.put()));
    }

    void SetTransform(const Matrix3x2f &transform)
    {
        m_target->SetTransform(D2D1::Matrix3x2F(transform.m00, transform.m01, transform.m10,
                                                transform.m11, transform.m20, transform.m21));
    }

    // 窗口大小改变时只需调用这个
    void Resize(UINT width, UINT height) { m_target->Resize(D2D1::SizeU(width, height)); }

    bool IsReady() const { return m_target && m_brush; }

    void BeginDraw()
    {
        if (m_target)
            m_target->BeginDraw();
        else
            throw std::runtime_error("Render target is not initialized.");
    }

    void EndDraw()
    {
        if (m_target)
            check_hresult(m_target->EndDraw());
        else
            throw std::runtime_error("Render target is not initialized.");
    }

    Color GetColor() const
    {
        return {m_brush->GetColor().r, m_brush->GetColor().g, m_brush->GetColor().b,
                m_brush->GetColor().a};
    }

    void SetColor(const Color &color) { m_brush->SetColor(color); }

    void SetStrokeWidth(float width) { m_strokeWidth = width; }

    float GetStrokeWidth() const { return m_strokeWidth; }

    void Clear(const Color &color) { m_target->Clear(color); }

    void DrawLine(Vector2f p0, Vector2f p1)
    {
        m_target->DrawLine(p0, p1, m_brush.get(), m_strokeWidth);
    }

    void DrawRectangle(float left, float top, float right, float bottom)
    {
        m_target->DrawRectangle(D2D1::RectF(left, top, right, bottom), m_brush.get(),
                                m_strokeWidth);
    }

    void FillRectangle(float left, float top, float right, float bottom)
    {
        m_target->FillRectangle(D2D1::RectF(left, top, right, bottom), m_brush.get());
    }

    void FillEllipse(Vector2f center, Vector2f radius)
    {
        m_target->FillEllipse(D2D1::Ellipse(center, radius.x, radius.y), m_brush.get());
    }

    void FillCircle(Vector2f center, float radius)
    {
        m_target->FillEllipse(D2D1::Ellipse(center, radius, radius), m_brush.get());
    }

    void DrawCircle(Vector2f center, float radius)
    {
        m_target->DrawEllipse(D2D1::Ellipse(center, radius, radius), m_brush.get(), m_strokeWidth);
    }

    void DrawRoundedRectangle(float left, float top, float right, float bottom, float rx, float ry)
    {
        m_target->DrawRoundedRectangle(
            D2D1::RoundedRect(D2D1::RectF(left, top, right, bottom), rx, ry), m_brush.get(),
            m_strokeWidth);
    }

    void FillRoundedRectangle(float left, float top, float right, float bottom, float rx, float ry)
    {
        m_target->FillRoundedRectangle(
            D2D1::RoundedRect(D2D1::RectF(left, top, right, bottom), rx, ry), m_brush.get());
    }

    void DrawGeometry(winrt::com_ptr<ID2D1Geometry> geometry)
    {
        m_target->DrawGeometry(geometry.get(), m_brush.get(), m_strokeWidth);
    }

    void FillGeometry(winrt::com_ptr<ID2D1Geometry> geometry)
    {
        m_target->FillGeometry(geometry.get(), m_brush.get());
    }

    void DrawText(std::wstring_view text, Vector2f position, Vector2f size = {})
    {
        if (!m_textFormat) return;

        D2D1_RECT_F rect{position.x, position.y, position.x + size.x, position.y + size.y};
        if (size.x == 0 || size.y == 0) {
            // 自动计算文本大小
            com_ptr<IDWriteTextLayout> layout;
            check_hresult(m_writeFactory->CreateTextLayout(
                text.data(), static_cast<uint32_t>(text.size()), m_textFormat.get(),
                std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                layout.put()));
            DWRITE_TEXT_METRICS metrics;
            layout->GetMetrics(&metrics);
            rect = {position.x, position.y, position.x + metrics.width,
                    position.y + metrics.height};
        }

        m_target->DrawText(text.data(), static_cast<uint32_t>(text.size()), m_textFormat.get(),
                           rect, m_brush.get());
    }

    void SetTextSize(float size)
    {
        if (m_textFormat) {
            // 需要重新创建文本格式来改变字体大小
            check_hresult(m_writeFactory->CreateTextFormat(
                L"Arial", nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL,
                DWRITE_FONT_STRETCH_NORMAL, size, L"en-us", m_textFormat.put()));
        }
    }

    float GetTextSize() const
    {
        if (m_textFormat) {
            return m_textFormat->GetFontSize();
        }
        return 16.0f;
    }
};

class Window final
{
    friend LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

public:
    static void ProcessMessage();

    class Callback
    {
    public:
        virtual void OnMessage(Window &window, WPARAM wParam, LPARAM lParam) = 0;
        virtual ~Callback()                                                  = default;
    };

    class LambdaCallback final : public Callback
    {
        class DetailBase
        {
        public:
            virtual void OnMessage(Window &window, WPARAM wParam, LPARAM lParam) = 0;

            virtual ~DetailBase() = default;
        };
        std::unique_ptr<DetailBase> p;

    public:
        template <class Func>
        LambdaCallback(Func &&func)
        {
            class Detail final : public DetailBase
            {
                Func f;

            public:
                Detail(Func &&func) : f(std::forward<Func>(func)) {}

                void OnMessage(Window &window, WPARAM wParam, LPARAM lParam) override
                {
                    f(window, wParam, lParam);
                }
            };
            p = std::make_unique<Detail>(std::forward<Func>(func));
        }

        void OnMessage(Window &window, WPARAM wParam, LPARAM lParam) override
        {
            p->OnMessage(window, wParam, lParam);
        }
    };

    Window(std::wstring_view title, uint32_t width, uint32_t height);
    ~Window();

    void Show();

    HWND GetRawHandle() const;

    using CallbackID = uint64_t;

    // 返回ID
    template <class Func>
    CallbackID Register(uint32_t msg, Func &&callback)
    {
        callbacks[msg].emplace_back(std::make_unique<LambdaCallback>(std::forward<Func>(callback)),
                                    next_callback_id_);
        return (static_cast<uint64_t>(msg) << 32) | (next_callback_id_++);
    }

    CallbackID Register(uint32_t msg, std::unique_ptr<Callback> callback)
    {
        callbacks[msg].emplace_back(std::move(callback), next_callback_id_);
        return (static_cast<uint64_t>(msg) << 32) | (next_callback_id_++);
    }

    void Unregister(CallbackID id)
    {
        uint32_t msg = static_cast<uint32_t>(id >> 32);
        uint32_t uid = static_cast<uint32_t>(id);
        if (callbacks.contains(msg)) {
            for (auto it = callbacks[msg].begin(); it != callbacks[msg].end(); ++it) {
                if (it->id == uid) {
                    *it = std::move(callbacks[msg].back());
                    callbacks[msg].pop_back();
                    return;
                }
            }
        }
        // std::println("Warning: Callback ID {} not found for unregistration.", id);
    }

    bool IsAlive() const;

    Vector2i GetSize() const;

private:
    struct CallbackData
    {
        std::unique_ptr<Callback> callback;
        uint32_t id;
    };

    std::unordered_map<uint32_t, std::vector<CallbackData>> callbacks;  // 消息回调列表
    HWND hwnd{};
    uint32_t next_callback_id_{};
};

}  // namespace nova::rnd
