#include "raylib.h"

typedef struct _Joint {
    Vector2 pos;
    struct _Joint* parent;
    struct _Joint* child;
} Joint;

typedef struct _Body {
    Vector2 final_target;
    Vector2 target;
    Vector2 root_pos;
    unsigned int N;
    float* links_lengths;
    float total_length;
    Joint* root;
    Joint* end;
} Body;


Body* init_body(unsigned int N, int total_length, float root_x, float root_y);
void set_body_root(Body* body, Vector2 new_pos);
void set_body_target(Body* body, Vector2 new_target);
void set_link_length(Body* body, unsigned int idx, double length);
void update_body(Body* body);
void draw_body(Body* body);
void free_body(Body* body);
