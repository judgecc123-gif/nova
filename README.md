# NOVA
## Simply universal, yet elegantly practiced

 此示例，创建窗口、渲染器
```C++
#include "render/render.hpp"

int main(){
    using namespace nova::rnd;

    try {
        Window window{L"Nova", 800, 600}; 

        Renderer renderer{window.GetRawHandle()}; 

        window.Show();
        while (window.IsAlive()) {
            Window::ProcessMessage();

            renderer.BeginDraw();
            renderer.Clear(Color::White);

            renderer.DrawCircle({200,200}, 10);

            renderer.EndDraw();
        }
    } catch (const std::exception &e) {
        // std::println("{}", e.what());
    }
}
```

此样例为行星绕日模型
```C++
#include <print>

#include "phyx/celestial.hpp" 
#include "render/input.hpp"
#include "render/render.hpp"
#include "render/view2d.hpp"

int main()
{
    try {
        using namespace nova::phy;
        using namespace nova::rnd;

        // --- 物理常数与环境设置 ---
        auto universe = CelestialWorld();
        // 在 AU, yr, solar_mass 单位制下，G = 4 * pi^2 是最精确的
        constexpr Real PI = 3.14159265358979323846;
        universe.G        = 4 * PI * PI;

        // 辅助 lambda：根据半径 r 计算圆轨道速度 v = sqrt(G * M / r)
        auto get_orbit_vel = [&](Real r) { return std::sqrt(universe.G * 1.0 / r); };

        // --- 创建太阳 ---
        auto sun = universe.CreateBody({.position = {0, 0}, .velocity = {0, 0}, .mass = 1.0});

        // --- 创建八大行星 (数据近似值) ---
        // 名字      | 距离(AU) | 质量(太阳质量)
        auto mercury = universe.CreateBody(
            {.position = {0.387, 0}, .velocity = {0, get_orbit_vel(0.387)}, .mass = 1.66e-7});
        auto venus = universe.CreateBody(
            {.position = {0.723, 0}, .velocity = {0, get_orbit_vel(0.723)}, .mass = 2.45e-6});
        auto mars = universe.CreateBody(
            {.position = {1.524, 0}, .velocity = {0, get_orbit_vel(1.524)}, .mass = 3.23e-7});
        auto jupiter = universe.CreateBody(
            {.position = {5.203, 0}, .velocity = {0, get_orbit_vel(5.203)}, .mass = 9.54e-4});
        auto saturn = universe.CreateBody(
            {.position = {9.537, 0}, .velocity = {0, get_orbit_vel(9.537)}, .mass = 2.86e-4});
        auto uranus = universe.CreateBody(
            {.position = {19.191, 0}, .velocity = {0, get_orbit_vel(19.191)}, .mass = 4.37e-5});
        auto neptune = universe.CreateBody(
            {.position = {30.069, 0}, .velocity = {0, get_orbit_vel(30.069)}, .mass = 5.15e-5});

        // --- 地球参数 ---
        Real earth_r    = 1.000;
        Real earth_mass = 3.00e-6;
        auto earth      = universe.CreateBody({.position = {earth_r, 0},
                                               .velocity = {0, get_orbit_vel(earth_r)},
                                               .mass     = earth_mass});

        // --- 月球参数修正 ---
        Real moon_dist_to_earth = 0.00257;
        // 关键：月球相对于地球的速度 v = sqrt(G * M_earth / r_moon)
        Real v_moon_relative = std::sqrt(universe.G * earth_mass / moon_dist_to_earth);

        auto moon_pos = earth.GetPosition() + Vector2r{moon_dist_to_earth, 0};
        auto moon_vel = earth.GetVelocity() + Vector2r{0, v_moon_relative};

        auto moon = universe.CreateBody({.position = moon_pos, .velocity = moon_vel, .mass = 3e-8});

        Window window{L"Phy-Engine", 3000, 2000};

        Renderer renderer{window.GetRawHandle()};
        window.Register(
            WM_SIZE, [&renderer](nova::rnd::Window &window, WPARAM wParam, LPARAM lParam) {
                if (wParam != SIZE_MINIMIZED) {
                    renderer.Resize(LOWORD(lParam), HIWORD(lParam));
                    std::println("Window resized: {}x{}", LOWORD(lParam), HIWORD(lParam));
                }
            });

        window.Register(WM_CLOSE, [](nova::rnd::Window &window, WPARAM, LPARAM) {
            std::println("Window is closing...");
        });

        window.Show();

        nova::rnd::View2D view;  // 1.5 pixels per AU
        view.zoom = 20.0f;

        Input::instance().Bind(&window);
        while (window.IsAlive()) {
            Input::instance().RecordPreviousState();
            Window::ProcessMessage();
            // --- 物理更新 ---
            for (int i = 0; i < 100; ++i)  // 每帧多次物理更新以提高稳定性
                universe.Step(1.0 / (365.0 * 120));

            // --- 视图控制 ---
            if (Input::instance().IsPressed(KeyCode::W)) {
                view.center.y -= 1.5f / view.zoom;
            }
            if (Input::instance().IsPressed(KeyCode::S)) {
                view.center.y += 1.5f / view.zoom;
            }
            if (Input::instance().IsPressed(KeyCode::A)) {
                view.center.x -= 1.5f / view.zoom;
            }
            if (Input::instance().IsPressed(KeyCode::D)) {
                view.center.x += 1.5f / view.zoom;
            }

            if (Input::instance().GetMouseWheelDelta() != 0.0f) {
                view.zoom *= std::pow(1.1f, Input::instance().GetMouseWheelDelta());
            }

            // --- 渲染部分 ---
            renderer.BeginDraw();
            auto transform = view.GetTransformMatrix(window.GetSize());
            renderer.SetTransform(transform);
            renderer.Clear({0.05f, 0.05f, 0.08f, 1.0f});  // 深空蓝黑

            // 太阳
            renderer.SetColor(0xFFCC00);
            renderer.FillCircle(sun.GetPosition(), 0.15f);

            // 水星 - 灰色
            renderer.SetColor(0xAAAAAA);
            renderer.FillCircle(mercury.GetPosition(), 0.02f);

            // 金星 - 淡黄色
            renderer.SetColor(0xE3BB76);
            renderer.FillCircle(venus.GetPosition(), 0.04f);

            // 地球 - 蓝色
            renderer.SetColor(0x2277FF);
            renderer.FillCircle(earth.GetPosition(), 0.04f);

            // 月球 - 白色
            renderer.SetColor(0xFFFFFF);
            renderer.FillCircle(moon.GetPosition(), 0.01f);

            // 火星 - 红色
            renderer.SetColor(0xFF4422);
            renderer.FillCircle(mars.GetPosition(), 0.03f);

            // 木星 - 橙棕色
            renderer.SetColor(0xD8CA9D);
            renderer.FillCircle(jupiter.GetPosition(), 0.10f);

            // 土星 - 金黄色
            renderer.SetColor(0xEAD6B8);
            renderer.FillCircle(saturn.GetPosition(), 0.08f);

            // 天王星 - 浅蓝色
            renderer.SetColor(0xA5D5D5);
            renderer.FillCircle(uranus.GetPosition(), 0.06f);

            // 海王星 - 深蓝色
            renderer.SetColor(0x4566FF);
            renderer.FillCircle(neptune.GetPosition(), 0.06f);

            renderer.EndDraw();
        }

        // WindowManager::instance().Quit();

    } catch (const std::exception &e) {
        std::println("Error: {}", e.what());
    }

    std::print("Application exited gracefully.\n");

    return 0;
}
```
