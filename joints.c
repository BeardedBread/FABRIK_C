#include "joints.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>

Body* init_body(unsigned int N, int total_length, float root_x, float root_y){
    Body *body = (Body *)malloc(sizeof(Body));
    body->N = N;
    body->total_length = 0;

    body->links_lengths = (float *)calloc(N - 1, sizeof(float));
    float link_len = total_length * 1.0f / ((N - 1)*1.0f);
    body->total_length = link_len * (N - 1);
    body->target = (Vector2){root_x, body->total_length};
    body->final_target = (Vector2){root_x, body->total_length};

    body->root_pos = (Vector2){root_x, root_y};
    body->root = (Joint *)malloc(sizeof(Joint));
    body->root->pos = (Vector2){root_x, root_y};
    body->root->parent = NULL;

    Joint* prev_joint = body->root;
    for (int i = 1; i<N;++i){
        body->links_lengths[i-1] = link_len;
        Joint* new_joint = (Joint *)malloc(sizeof(Joint));
        new_joint->pos = (Vector2){root_x, root_y - link_len * i};
        new_joint->parent = prev_joint;
        new_joint->child = NULL;
        prev_joint->child = new_joint; 
        prev_joint = new_joint;
        if (i == N-1){
            body->end = new_joint;
        }
    }

#ifdef DEBUG
    Joint* current_joint = body->root;
    for (Joint* jt = body->root;jt != NULL; jt = jt->child){
        printf("x %.2f, y %.2f\n", jt->pos.x, jt->pos.y);
    }
#endif
    return body;
}

void set_body_target(Body* body, Vector2 new_target){
    body->final_target = new_target;
}

Vector2 _get_new_pos(Vector2 p1, Vector2 p2, float length){
    float dist = Vector2Distance(p1, p2);
    float lambda = length / dist;
    return Vector2Add(
        Vector2Scale(p2,1-lambda),
        Vector2Scale(p1, lambda)
    );
}

void update_body(Body *body){
    body->target = Vector2Lerp(body->target, body->final_target, 0.2);

    // Check distance
    float dist = Vector2Distance(body->root->pos, body->target);
    unsigned int i = 0;
    if (dist > body->total_length){
        for (Joint *jt=body->root->child;jt != NULL; jt= jt->child){
            jt->pos = _get_new_pos(body->target, jt->parent->pos, body->links_lengths[i]);
            ++i;
        }
       return;
    }

    double error = 1;
    Vector2 prev_pos; 
    while (error > 1e-2){
        prev_pos = body->end->pos;

        // Backward Pass
        body->end->pos = body->target;
        i = body->N - 2;
        for (Joint *jt = body->end->parent; jt != NULL; jt = jt->parent){
            jt->pos = _get_new_pos(jt->pos, jt->child->pos, body->links_lengths[i]);
            --i;
        }
    
        // Forward Pass
        body->root->pos.x = body->root_pos.x;
        body->root->pos.y = body->root_pos.y;
        i = 0;
        for (Joint *jt = body->root->child; jt != NULL; jt = jt->child){
            jt->pos = _get_new_pos(jt->pos, jt->parent->pos, body->links_lengths[i]);
            ++i;
        }
        error = Vector2Distance(prev_pos, body->end->pos);
    }
}

void draw_body(Body* body){
    for (Joint* jt = body->root; jt != NULL; jt = jt->child){
        if (jt->child != NULL)
            DrawLineV(jt->pos, jt->child->pos, BLACK);
        if (jt != body->root)
            DrawCircleV(jt->pos, 5, BLUE);
        else
            DrawCircleV(jt->pos, 5, BLACK);
    }
    DrawCircleV(body->target, 3, RED); 
}

void free_body(Body *body){
    Joint* current_joint  = body->root;
    Joint* next_joint = body->root;
    while (current_joint != NULL){
        current_joint->parent = NULL;
        next_joint = current_joint->child;
        current_joint->child = NULL;
        free(current_joint);
        current_joint = next_joint;
    }
}
