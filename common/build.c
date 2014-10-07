#include "build.h"

int LOG_LEVEL = 0xff;



#if QUADCOPTER == 1




#else

float pid_limit[3][3] = 				// pid_limit[roll,pitch,yaw][p max offset, I limit, d dummy]
{
	{PI/4, PI/2, 1},
	{PI/4, PI/2, 1},
	{PI/6, PI/3, 1},
};


float pid_factor[3][3] = 			// pid_factor[roll,pitch,yaw][p,i,d]
{
	{1.50, 0.05, 0.36,},
	{1.50, 0.05, 0.36,},
	//{0, 0.0, 20,},
	//{0, 0.0, 20,},
	{0, 0, 0,},
};

float pid_factor2[3][3] = 				// pid_limit[roll,pitch,yaw][p max offset, I limit, d dummy]
{
	0, 			// we dont need it in fixed wing!
};

#endif
