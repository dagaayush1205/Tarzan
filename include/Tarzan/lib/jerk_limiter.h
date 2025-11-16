#ifndef JERK_LIMITER_H
#define JERK_LIMITER_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    float v_last;   // last output velocity (m/s or rad/s)
    float a_last;   // last output acceleration (m/s^2 or rad/s^2)
    float v_max;    // max velocity
    float a_max;    // max accel magnitude
    float j_max;    // max jerk magnitude
} jerk_limiter_t;

static inline void jerk_limiter_init(jerk_limiter_t *lim,float v0,float a0,float v_max,float a_max,float j_max)
{
    lim->v_last = v0;
    lim->a_last = a0;
    lim->v_max  = v_max;
    lim->a_max  = a_max;
    lim->j_max  = j_max;
}

static inline float jerk_limiter_step(jerk_limiter_t *lim, float v_target, float dt_sec)
{
 
    if (v_target == 0.0f) {
        lim->a_last = 0.0f; 
        lim->v_last = 0.0f; // Reset velocity state
        return 0.0f;      // Return neutral immediately
    }
    if (dt_sec <= 0.0f) return lim->v_last;

    // desired accel to reach target in one tick (unclamped)
    float a_des = (v_target - lim->v_last) / dt_sec;

    // limit jerk: Δa = a_des - a_last  => clamp to ± j_max * dt
    float max_da = lim->j_max * dt_sec;
    float da = a_des - lim->a_last;
    if (da >  max_da) da =  max_da;
    if (da < -max_da) da = -max_da;

    float a_smooth = lim->a_last + da;

    //clamp accel magnitude
    if (a_smooth >  lim->a_max) a_smooth =  lim->a_max;
    if (a_smooth < -lim->a_max) a_smooth = -lim->a_max;

    //integrate acceleration->velocity
    float v_new = lim->v_last + a_smooth * dt_sec;

    //clamp velocity magnitude
    if (v_new >  lim->v_max) v_new =  lim->v_max;
    if (v_new < -lim->v_max) v_new = -lim->v_max;

    //update state
    lim->a_last = a_smooth;
    lim->v_last = v_new;

    return (v_new);
}

#endif // JERK_LIMITER_H
