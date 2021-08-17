#include "joints.h"
#include "raylib.h"

int main(int argc, char** argv) {
    const unsigned int scrnWidth = 800;
    const unsigned int scrnHeight = 600;

    InitWindow(scrnWidth, scrnHeight, "FABRIK");
    SetTargetFPS(30);

    Body* body = init_body(15, 300, 400, 500);
    bool follow_mode = false;

    while(!WindowShouldClose()){
        if (IsKeyReleased(KEY_F))
            follow_mode = !follow_mode;

        if (follow_mode)
            set_body_target(body, GetMousePosition());
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            set_body_root(body, GetMousePosition());

        update_body(body);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            draw_body(body);
        EndDrawing();
    }
   CloseWindow(); 
   free_body(body);
}
