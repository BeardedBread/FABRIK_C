#include "raylib.h"

typedef struct _Joint {
    Vector2 pos;
} Joint;

typedef struct _Body {
    Vector2 final_target;
    int N;
    float total_length;
    float angle_limit;
    Vector2 target;
    Vector2 root_pos;
    float link_length;
    float* links_lengths;
    float current_length;
    Joint* joints;
} Body;


Body* init_body(int N, float link_length, float root_x, float root_y);
void set_body_root(Body* body, Vector2 new_pos);
void set_body_target(Body* body, Vector2 new_target);
void set_current_length(Body* body, float new_length);
void update_body(Body* body);
void draw_body(Body* body);
void free_body(Body* body);
