#include "joints.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

Body* init_body(unsigned int N, int total_length, float root_x, float root_y){
    Body *body = (Body *)malloc(sizeof(Body));
    body->N = N;
    body->total_length = 0;

    float link_len = total_length * 1.0f / ((N - 1)*1.0f);
    body->links_lengths = (float *)calloc(N - 1, sizeof(float));
    body->total_length = link_len * (N - 1);
    body->current_length = link_len * (N - 1);
    body->target = (Vector2){root_x, body->total_length};
    body->final_target = (Vector2){root_x, body->total_length};
    body->angle_limit = PI/6;

    body->root_pos = (Vector2){root_x, root_y};

    body->joints = (Joint *)calloc(N, sizeof(Joint));
    body->joints[0].pos = body->root_pos;
    for (int i = 1; i<N;++i){
        body->links_lengths[i-1] = link_len;
        body->joints[i].pos = (Vector2){root_x, root_y - link_len * i};
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

void set_current_length(Body* body, float new_length){

}

Vector2 _get_new_pos(Vector2 p1, Vector2 p2, float length){
    float dist = Vector2Distance(p1, p2);
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
    body->target = Vector2Lerp(body->target, body->final_target, 0.2);

    // Check distance
    float dist = Vector2Distance(body->joints[0].pos, body->target);
    if (dist >= body->total_length){
        for(unsigned int i = 1; i< body->N; ++i){
            body->joints[i].pos = _get_new_pos(body->target, body->joints[i-1].pos, body->links_lengths[i]);
        }
       return;
    }

    double error = 1;
    Vector2 prev_pos; 
    while (error > 1e-2){
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
    DrawLineV(body->joints[0].pos, body->joints[1].pos, BLACK);
    for(int i = 1; i < body->N-1; ++i){
        DrawLineV(body->joints[i].pos, body->joints[i+1].pos, BLACK);
#ifdef DEBUG
        DrawCircleV(body->joints[i].pos, 5, BLUE);
#endif
    }
    DrawCircleV(body->target, 3, RED); 
}

void free_body(Body *body){
    free(body->joints);
    free(body->links_lengths);
}
