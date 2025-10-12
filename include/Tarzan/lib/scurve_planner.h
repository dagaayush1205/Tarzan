#ifndef SCURVE_PLANNER_H
#define SCURVE_PLANNER_H

#include <math.h>
#include <stdbool.h>

//physical limit
typedef struct {
    float v_max; 
    float a_max; 
    float j_max; 
} scurve_constraints_t;

//calculated parameter for scurve profile
typedef struct {
    float v_start; //start vel
    float v_end;   //end vel
    float dist;    //total dist

    float v_m;

    // Time durations for each phase
    float t_j1;    //jerk time for acceleration 
    float t_a;     //total time for acceleration 
    float t_j2;    //jerk time for deceleration 
    float t_d;     //total time for deceleration 
    float t_const; //time at constant velocity v_m
    float T;       //total time for the entire move

    float dist_a;  //distance during acceleration
    float dist_d;  //distance during deceleration
    float dist_const; //distance covered at const velocity

    scurve_constraints_t limits;

} scurve_profile_t;

bool scurve_plan_profile(scurve_profile_t *p,
                         const scurve_constraints_t *limits,
                         float dist,
                         float v_start,
                         float v_end);


float scurve_evaluate_velocity(const scurve_profile_t *p, float t);

#endif // SCURVE_PLANNER_H
