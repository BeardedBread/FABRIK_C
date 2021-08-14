#include "joints.h"

int main(int argc, char** argv) {
    const unsigned int scrnWidth = 800;
    const unsigned int scrnHeight = 600;

    InitWindow(scrnWidth, scrnHeight, "FABRIK");
    SetTargetFPS(60);

    Body* body = init_body(15, 300, 400, 500);

    while(!WindowShouldClose()){
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)){
            set_body_target(body, GetMousePosition());
        }
        update_body(body);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            draw_body(body);
        EndDrawing();
    }
   CloseWindow(); 
   free_body(body);
}
