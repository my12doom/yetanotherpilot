#include "ads1115_worker.h"

static int ads1115_work_pos = 0;
static int ads1115_work_count = 0;
static ads1115_work ads1115_works[MAX_ADS1115_WORKS];

int ads1115_new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out)
{
	ads1115_work *p = &ads1115_works[ads1115_work_count++];
	if (ads1115_work_count >= MAX_ADS1115_WORKS)
		return -1;
	
	p->speed = speed;
	p->channel = channel;
	p->gain = gain;
	p->out = out;

	if (ads1115_work_count == 1)
	{
		ads1115_config(speed, channel, gain, ads1115_mode_singleshot);
		ads1115_startconvert();
	}
	
	return 0;
}

int ads1115_go_on()
{
	int res;
	if (ads1115_work_count == 0)
		return 1;

	res = ads1115_getresult(ads1115_works[ads1115_work_pos].out);
	if (res == 0)
	{
		ads1115_work *p;
		ads1115_work_pos = (ads1115_work_pos+1)%ads1115_work_count;
		p = &ads1115_works[ads1115_work_pos];
		ads1115_config(p->speed, p->channel, p->gain, ads1115_mode_singleshot);
		ads1115_startconvert();
		return 0;
	}

	return 1;
}
