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
