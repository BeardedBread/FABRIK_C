#include "joints.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

Body* init_body(int N, float link_length, float root_x, float root_y){
    Body *body = (Body *)malloc(sizeof(Body));
    body->N = N;
    body->total_length = 0;

    body->link_length = link_length;
    body->links_lengths = (float *)calloc(N - 1, sizeof(float));
    body->total_length = link_length * (N - 1);
    body->current_length = body->total_length;
    body->target_length = link_length * (N - 1);
    body->target = (Vector2){root_x, body->total_length};
    body->final_target = (Vector2){root_x, body->total_length};
    body->angle_limit = PI;

    body->root_pos = (Vector2){root_x, root_y};

    body->joints = (Joint *)calloc(N, sizeof(Joint));
    body->joints[0].pos = body->root_pos;
    for (int i = 1; i<N;++i){
        body->links_lengths[i-1] = link_length;
        body->joints[i].pos = (Vector2){root_x, root_y - link_length * i};
    }

#ifdef DEBUG
    for (int i = 0; i<N;++i){
        printf("x %.2f, y %.2f\n", body->joints[i].pos.x, body->joints[i].pos.y);
    }
#endif
    return body;
}

void set_body_root(Body* body, Vector2 new_pos){
    Vector2 offset = Vector2Subtract(new_pos, body->root_pos);
    body->root_pos = new_pos;
    set_body_target(body, Vector2Add(body->final_target, offset));
}

void set_body_target(Body* body, Vector2 new_target){
    body->final_target = new_target;
}

void set_length_target(Body* body, float new_length){
    body->target_length = Clamp(new_length, 0, body->total_length);
}

void _update_body_length(Body* body){
    body->current_length = Lerp(body->current_length, body->target_length, 0.2);
    body->N = (int)(body->current_length / body->link_length);

    for (int i=0; i< body->N; ++i)
        body->links_lengths[i] = body->link_length;

    float remainder = body->current_length - body->N * body->link_length;
    if (remainder >= 1) {
        body->links_lengths[body->N] = remainder;
        body->N += 1;
    }
    body->N += 1;
}

Vector2 _get_new_pos(Vector2 p1, Vector2 p2, float length){
    float dist = Vector2Distance(p1, p2);
    // Ensure it's not zero. Due to integer math, this is possible
    // Just set it to 1 and let it settle
    dist = (dist == 0) ? 1 : dist; 

    float lambda = length / dist;
    return Vector2Add(
        Vector2Scale(p2,1-lambda),
        Vector2Scale(p1, lambda)
    );
}

Vector2 _limit_new_pos(Vector2 a, Vector2 b, Vector2 c, float length, float angle){
    Vector2 d_hat = Vector2Normalize(Vector2Subtract(c, b));
    Vector2 e_hat = Vector2Normalize(Vector2Subtract(b, a));
    Vector2 f_hat = (Vector2){-e_hat.y, e_hat.x};
   
    if (Vector2DotProduct(d_hat, e_hat) > cos(angle)){
        return _get_new_pos(c, b, length);
    }

    float lambda = length * cos(angle); 
    float mu = length * sin(angle); 
    if (Vector2DotProduct(d_hat, f_hat) < 0)
        mu *= -1;

    Vector2 v = Vector2Add(Vector2Scale(e_hat, lambda), Vector2Scale(f_hat, mu));
    return Vector2Add(b, v);
}

void update_body(Body *body){
    _update_body_length(body);
    body->target = Vector2Lerp(body->target, body->final_target, 0.2);

    // Check distance
    float dist = Vector2Distance(body->joints[0].pos, body->target);
    if (dist >= body->current_length){
        for(int i = 0; i< body->N-1; ++i){
            body->joints[i+1].pos = _get_new_pos(
                body->target,
                body->joints[i].pos,
                body->links_lengths[i]
            );
        }
       return;
    }

    // Perform FABRIK
    double error = 1;
    int iterations = 0;
    Vector2 prev_pos; 
    while (error > 1e-2 && iterations < 100){
        ++iterations;
        prev_pos = body->joints[body->N-1].pos;

        // Backward Pass
        body->joints[body->N-1].pos = body->target;
        body->joints[body->N-2].pos = _get_new_pos(
            body->joints[body->N-2].pos,
            body->joints[body->N-1].pos,
            body->links_lengths[body->N-2]
        );
        for(int i = body->N - 3; i >= 0; --i){
            // Do regular calc for the one after the end
            body->joints[i].pos = _limit_new_pos(
                body->joints[i+2].pos,
                body->joints[i+1].pos,
                body->joints[i].pos,
                body->links_lengths[i],
                body->angle_limit
            );
        }
    
        // Forward Pass
        body->joints[0].pos = body->root_pos;
        body->joints[1].pos = _get_new_pos(
            body->joints[1].pos,
            body->joints[0].pos,
            body->links_lengths[1]
        );
        for(int i = 2; i < body->N; ++i){
            body->joints[i].pos = _limit_new_pos(
                body->joints[i-2].pos,
                body->joints[i-1].pos,
                body->joints[i].pos,
                body->links_lengths[i-1],
                body->angle_limit
            );
        }
        error = Vector2Distance(prev_pos, body->joints[body->N-1].pos);
    }
}

void draw_body(Body* body){
    DrawCircleV(body->joints[0].pos, 5, BLACK);
    DrawLineEx(body->joints[0].pos, body->joints[1].pos, body->N, BLACK);
    for(int i = 1; i < body->N-1; ++i){
        DrawLineEx(body->joints[i].pos, body->joints[i+1].pos, body->N - i, BLACK);
#ifdef DEBUG
        DrawCircleV(body->joints[i].pos, 5, BLUE);
#else
        DrawCircleV(body->joints[i].pos,(int)((body->N - i) / 2), BLACK);
#endif
    }
#ifdef DEBUG
    DrawCircleV(body->target, 3, RED); 
#endif
}

void free_body(Body *body){
    free(body->joints);
    free(body->links_lengths);
}
