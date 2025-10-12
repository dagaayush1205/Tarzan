#include <Tarzan/lib/scurve_planner.h>

// Helper function to calculate acceleration/deceleration phase durations and distance
static void calculate_phase(float v0, float v1, float a_max, float j_max, float* t_j, float* t_a, float* dist_a) {
    float dv = fabsf(v1 - v0);

    //case1- Acceleration limit is not reached so the profile is a simple jerk-limited ramp.
    //based on the PDF equation (23)/(42) referred
    if (sqrtf(dv*j_max)<=a_max) {
        *t_j= sqrtf(dv / j_max);
        *t_a= 2.0f * (*t_j);
        *dist_a= (*t_a) * (v0 + v1) / 2.0f; //based on PDF equation (32)/(51)
    }
    //Case2-Acceleration limit is reached. A const accel phase is needed.
    //based on PDF equation (24)/(43).
    else {
        *t_j= a_max / j_max;
        *t_a= (dv / a_max) + (*t_j); //PDF equation (31)/(50)
        *dist_a = (*t_a) * (v0 + v1) / 2.0f; //PDF equation (32)/(51)
    }
}


bool scurve_plan_profile(scurve_profile_t *p, const scurve_constraints_t *limits, float dist, float v_start, float v_end)
{
    p->limits= *limits;
    p->dist= dist;
    p->v_start= v_start;
    p->v_end= v_end;

    //binary search for the optimal v_m(max vel)
    //follows the logic from Section 4 of the PDF.
    float v_low= fmaxf(v_start, v_end);
    float v_high= limits->v_max;
    int iterations= 10;

    for (int i = 0; i < iterations; ++i)
    {
        float v_m_trial = (v_low + v_high) / 2.0f;
        calculate_phase(v_start, v_m_trial, limits->a_max, limits->j_max, &p->t_j1, &p->t_a, &p->dist_a);
        calculate_phase(v_end, v_m_trial, limits->a_max, limits->j_max, &p->t_j2, &p->t_d, &p->dist_d);

        // Check if the move is possible within the given distance
        // PDF equation (56)
        if ((p->dist_a + p->dist_d) > dist) 
        {
            v_high = v_m_trial; //too fast
        } 
        else 
        {
            v_low = v_m_trial;  // Possible, try for a higher v_m
        }
    }
    p->v_m = v_low;//as we use highest possible v_m
  
    //final calculation with the optimized v_m
    calculate_phase(v_start, p->v_m, limits->a_max, limits->j_max, &p->t_j1, &p->t_a, &p->dist_a);
    calculate_phase(v_end, p->v_m, limits->a_max, limits->j_max, &p->t_j2, &p->t_d, &p->dist_d);

    //calculate the constant velocity phase duration and distance
    //PDF equation (54)&(55)
    p->dist_const = dist - p->dist_a - p->dist_d;
    if (p->dist_const < 0) p->dist_const = 0; //should not happen with binary search
    
    p->t_const = p->dist_const / p->v_m;
    p->T = p->t_a + p->t_const + p->t_d;

    return true;
}


float scurve_evaluate_velocity(const scurve_profile_t *p, float t) {
    if (t < 0) return p->v_start;
    if (t >= p->T) return p->v_end;

    float j_max= p->limits.j_max;
    float a_max= p->limits.a_max;

    //1.Acceleration Phase (0 <= t < t_a)
    if (t < p->t_a) {
        // a. Positive Jerk region [0, t_j1)
        if (t < p->t_j1) {
            // v(t) = v_start + 0.5 * j_max * t^2
            // Derived from integrating jerk -> accel, then accel -> velocity. See PDF eq (19).
            return p->v_start + 0.5f * j_max * t * t;
        }
        // b. Constant Accel region [t_j1, t_a - t_j1]
        else if (t < p->t_a - p->t_j1) {
            // v(t) = v(t_j1) + a_max * (t - t_j1)
            float v_at_tj1 = p->v_start + 0.5f * j_max * p->t_j1 * p->t_j1;
            return v_at_tj1 + a_max * (t - p->t_j1);
        }
        //-ve jerk region (t_a - t_j1, t_a]
        else {
            //v(t) = v_m-0.5*j_max*(t_a-t)^2
            float dt = p->t_a - t;
            return p->v_m - 0.5f * j_max * dt * dt;
        }
    }
    //const velocity phase (t_a <= t < t_a + t_const)
    else if (t < p->t_a + p->t_const) {
        return p->v_m;
    }
    //deceleration Phase (t_a+t_const<=t<T)
    else {
        float t_rel = t - (p->t_a + p->t_const); // time relative to start of decel
        // a. Negative Jerk region
        if (t_rel < p->t_j2) {
            // v(t) = v_m - 0.5 * j_max * t_rel^2
            return p->v_m - 0.5f * j_max * t_rel * t_rel;
        }
        //const decel
        else if (t_rel < p->t_d - p->t_j2) {
            float v_at_tj2 = p->v_m - 0.5f * j_max * p->t_j2 * p->t_j2;
            return v_at_tj2 - a_max * (t_rel - p->t_j2);
        }
        //+ve jerk region
        else {
            // v(t) = v_end + 0.5 * j_max * (T - t)^2
            float dt = p->T - t;
            return p->v_end + 0.5f * j_max * dt * dt;
        }
    }
}
