#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/scurve_planner.h>
#include <Tarzan/lib/jerk_limiter.h>

#define LINEAR_V_MAX  1.5f  
#define LINEAR_A_MAX  0.8f   
#define LINEAR_J_MAX  1.5f  
#define ANGULAR_V_MAX 2.0f   
#define ANGULAR_A_MAX 5.0f  
#define ANGULAR_J_MAX 8.0f   

/* interpolates sbus channel value to velocity
 *param :
 *channel - sbus channel
 *velocity_range - velocity range for interpolation
 *channel_range - sbus channel range */
float sbus_velocity_interpolation(uint16_t channel, float *velocity_range,
                                  uint16_t *channel_range) {

  if (channel > channel_range[1])
    return velocity_range[1];

  if (channel < channel_range[0])
    return velocity_range[0];

  if (channel < 1050 && channel > 950)                  // deadzone
    return (velocity_range[0] + velocity_range[1]) / 2; // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dvel = velocity_range[1] - velocity_range[0];

  float vel_interp =
      velocity_range[0] + (dvel / dchannel) * (channel - channel_range[0]);

  return vel_interp;
}

/* interpolates sbus channel value to pwm
 * param :
 * channel - sbus channel value
 * pwm_range - pwm range for interpolation
 * channel_range - sbus channel range */
uint32_t sbus_pwm_interpolation(uint16_t channel, uint32_t *pwm_range,
                                uint16_t *channel_range) {

  if (channel > channel_range[1])
    return pwm_range[2];

  if (channel < channel_range[0])
    return pwm_range[0];

  if (channel < 1050 && channel > 950) // deadzone
    return 1500000;                    // neutral

  float dchannel = channel_range[1] - channel_range[0];
  float dpwm = pwm_range[1] - pwm_range[0];

  uint32_t pwm_interp =
      pwm_range[0] + (dpwm / dchannel) * (channel - channel_range[0]);

  return pwm_interp;
}

/* struct DiffDriveCtx* drive_init(struct DiffDriveConfig *config, int (*velocity_callback)(const float *velocity_buffer, int buffer_len, int wheels_per_side)) { */
/**/
/*     struct DiffDriveCtx ctx; */
/**/
/*     ctx.velocity_callback = velocity_callback; */
/*     ctx.previous_update_timestamp = 0; */
/*     ctx.drive_control.move_timer = 0.0f; */
/*     ctx.drive_control.mode=MANUAL; */
/*     ctx.drive_control.move_timer= 0.0f; */
/**/
/*     // Initialize the teleop jerk limiters with your rover's physical limits */
/*     struct DiffDriveCtrl drive_control; */
/*     /* jerk_limiter_init(&(drive_control.linear_limiter),0.0f,0.0f,LINEAR_V_MAX,LINEAR_A_MAX,LINEAR_J_MAX); */ 
/*     /**/ 
/*     /* jerk_limiter_init(&(drive_control.angular_limiter),0.0f,0.0f,ANGULAR_V_MAX,ANGULAR_A_MAX,ANGULAR_J_MAX); */ 
/*     /**/ 
/**/
/*     // scurve_constraints_t linear_limits = {LINEAR_V_MAX, LINEAR_A_MAX, LINEAR_J_MAX}; */
/*     // scurve_constraints_t angular_limits = {ANGULAR_V_MAX, ANGULAR_A_MAX, ANGULAR_J_MAX}; */
/*     // */
/*     // scurve_plan_profile(&drive_control.linear_profile, &linear_limits, linear_distance, 0.0f, 0.0f); */
/*     // scurve_plan_profile(&drive_control.angular_profile, &angular_limits, angular_distance, 0.0f, 0.0f); */
/**/
/**/
/* 	struct DiffDriveCtx *heap_ctx = (struct DiffDriveCtx *)malloc(sizeof(ctx)); */
/* 	memcpy(heap_ctx, &ctx, sizeof(*heap_ctx)); */
/* 	memcpy((void *)&heap_ctx->drive_config, config, sizeof(heap_ctx->drive_config)); */
/* 	return (void *)heap_ctx; */
/* } */
/**/

struct DiffDriveCtx* drive_init(struct DiffDriveConfig *config, int (*velocity_callback)(const float *velocity_buffer, int buffer_len, int wheels_per_side)) 
{
    struct DiffDriveCtx *ctx = (struct DiffDriveCtx *)malloc(sizeof(struct DiffDriveCtx));
    if (!ctx) {
        return NULL; // Return NULL if memory allocation fails
    }

    memcpy(&ctx->drive_config, config, sizeof(ctx->drive_config));
    ctx->velocity_callback = velocity_callback;
    ctx->previous_update_timestamp = k_uptime_get(); 
    //ctx->drive_control.mode = MANUAL;
    ctx->drive_control.is_auto_active = false; 
    ctx->drive_control.move_timer = 0.0f;

    //Initialize the limiters inside ctx->drive_control
    jerk_limiter_init(&ctx->drive_control.linear_limiter, 0.0f, 0.0f, LINEAR_V_MAX, LINEAR_A_MAX, LINEAR_J_MAX);
    jerk_limiter_init(&ctx->drive_control.angular_limiter, 0.0f, 0.0f, ANGULAR_V_MAX, ANGULAR_A_MAX, ANGULAR_J_MAX);
    
    return ctx; // Return the correctly initialized struct
}

int diffdrive_kine(struct DiffDriveCtx* ctx, struct DiffDriveTwist command, float dt_sec)
{
    int ret = 0;
    float linear_command;
    float angular_command;
	
    // Get time since last update
    if (k_uptime_delta(&ctx->previous_update_timestamp) / 1000 >
	    ctx->drive_config.command_timeout_seconds) {
		command.linear_x = 0.0;
		command.angular_z = 0.0;
	}
  printk("DT_SEC: %.2f\n",dt_sec);
  //const double dt_sec = k_uptime_delta(&ctx->previous_update_timestamp) / 1000.0;
  // printf("%.2f\n",dt_sec);

   //MANUAL MODE
   linear_command = jerk_limiter_step(&ctx->drive_control.linear_limiter, command.linear_x, dt_sec);
   angular_command = jerk_limiter_step(&ctx->drive_control.angular_limiter, command.angular_z, dt_sec);
   printk("DEBUG: Using ANGULAR_A_MAX: %f\n", ANGULAR_A_MAX); 
   printk("Raw CMD: %f,%f | ", command.linear_x, command.angular_z);
   printk("Smooth CMD:%f,%f | ", linear_command, angular_command);
   // printf("Actual : %f | %f", command.linear_x, command.angular_z);
   // printf("Liminted : %f | %f\n",linear_command,angular_command);
    
 //    if(mode==AUTONOMOUS){
 //            //AUTO MODE- we are using scurve
 //                ctx->move_timer += dt_sec;
	//
 //                linear_command = scurve_evaluate_velocity(&ctx->linear_profile, ctx->move_timer);
 //                angular_command = scurve_evaluate_velocity(&ctx->angular_profile, ctx->move_timer);
	//
 //                // Stop the move when the longer profile is complete
 //                float total_move_time = fmaxf(ctx->linear_profile.T, ctx->angular_profile.T);
 //                if (ctx->move_timer >= total_move_time) {
 //                    ctx->is_auto_move_active = false;
 //                }
 //            }
	// }

	const float wheel_separation =
		ctx->drive_config.wheel_separation_multiplier * ctx->drive_config.wheel_separation;
	const float left_wheel_radius =
		ctx->drive_config.left_wheel_radius_multiplier * ctx->drive_config.wheel_radius;
	const float right_wheel_radius =
		ctx->drive_config.right_wheel_radius_multiplier * ctx->drive_config.wheel_radius;
	const int feedback_buffer_size = ctx->drive_config.wheels_per_side * 2;

	const float velocity_left =
		(linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
	const float velocity_right =
		(linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;
  printk("Wheels: Left=%.2f, Right=%.2f\n", velocity_left, velocity_right);

	float *velocity_buffer = (float *)malloc(sizeof(float) * feedback_buffer_size);

	for (int i = 0; i < ctx->drive_config.wheels_per_side; i++) {
		velocity_buffer[i] = velocity_left;
		velocity_buffer[ctx->drive_config.wheels_per_side + i] = velocity_right;
	}
	if (ctx->velocity_callback(velocity_buffer, feedback_buffer_size,
				     ctx->drive_config.wheels_per_side)) {
		// ERROR: Something went wrong writing the velocities
		ret = 3;
	}
	free(velocity_buffer);
	return ret;
}

