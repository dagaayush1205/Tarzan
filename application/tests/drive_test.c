#include <stdio.h>
/* Velocity and PWM ranges */
float vel_range[] = {-10, 10};
//uint32_t pwm_range[] = {1120000, 1880000};
float la_speed_range[] = {-127.0, 127.0};
//uint32_t pid_pwm_range[] = {1300000, 1700000};
float angle_range[] = {-270, 270};

float *velocity_interpolation(float val[],int len)
{
	for(int i=0;i<len;i++)
	{
		val[i]-=1024;
		val[i]=(val[i]/1024)*vel_range[1];
	}
	return val;
}
float *SBUS(uint16_t channels[]){
	//sbus read using val[] also declare and initialize val
	static float val[14]={1024,0,452,4,122,5,6,7,8,9,345,643,234,100};      
       	float *ptr = velocity_interpolation(val,14);
	return val;
}
int main(){
	float *val=SBUS();
	for (int i = 0; i < 14; i++) {
        printf("%f ", *val);
	val++;
	}
}
